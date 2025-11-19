#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEClient.h>
#include <esp_gap_ble_api.h>

#include "racebox_ble.h"

// ===============================
// Config
// ===============================
static const char *RACEBOX_MAC_STR = "D1:B7:54:6E:4E:71";  // your RaceBox MAC
static const uint32_t CONNECT_TIMEOUT_MS = 10000;          // 10s
static const uint32_t DATA_STALE_MS      = 500;            // how "fresh" data must be

// ===============================
// Internal state
// ===============================
static BLEClient *g_client       = nullptr;
static bool       g_connected    = false;

static RaceboxSnapshot g_snap = {
    false,  // connected
    false,  // hasFix
    0.0,    // latDeg
    0.0,    // lonDeg
    0.0f,   // speedKPH
    0.0f, 0.0f, 0.0f,  // gX,gY,gZ
    0.0f,              // leanDeg
    0.0f, 0.0f,        // yaw,pitch
    0                  // lastUpdateMs
};

// Spinlock to guard g_snap
static portMUX_TYPE g_snapMux = portMUX_INITIALIZER_UNLOCKED;

// UBX reassembly buffer
static const size_t RBX_BUF_SIZE = 512;
static uint8_t rbxBuf[RBX_BUF_SIZE];
static size_t  rbxLen = 0;

// Lean calibration offset (if you ever want an "upright" zero)
static float rbxLeanOffset = 0.0f;

// ===============================
// Little-endian helpers
// ===============================
static inline uint16_t leU16(const uint8_t *p) {
    return (uint16_t)p[0] | ((uint16_t)p[1] << 8);
}
static inline int16_t leI16(const uint8_t *p) {
    return (int16_t)leU16(p);
}
static inline uint32_t leU32(const uint8_t *p) {
    return (uint32_t)p[0] |
           ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) |
           ((uint32_t)p[3] << 24);
}
static inline int32_t leI32(const uint8_t *p) {
    return (int32_t)leU32(p);
}

// ===============================
// Parse RaceBox FF/01 (len 80) payload
// ===============================
// NOTE: lat/lon offsets are left as TODO because they depend
// on the exact vendor doc. You can wire them up once you have
// the correct byte indices.
//
static void parseRaceboxDataMessage(const uint8_t *payload /* len = 80 */) {
    // Offsets you already confirmed from your test sketch
    uint8_t  fixStatus   = payload[20];
    uint8_t  fixFlags    = payload[21];
    int32_t  speed_mm_s  = leI32(payload + 48);
    int16_t  gX_mg       = leI16(payload + 68);
    int16_t  gY_mg       = leI16(payload + 70);
    int16_t  gZ_mg       = leI16(payload + 72);

    float speed_kph = (speed_mm_s / 1000.0f) * 3.6f;
    float gX        = gX_mg / 1000.0f;
    float gY        = gY_mg / 1000.0f;
    float gZ        = gZ_mg / 1000.0f;

    bool fix_ok = (fixStatus == 3) && (fixFlags & 0x01);

    // Lean angle from Gy/Gz
    float leanDeg = g_snap.leanDeg;  // default: keep old
    float mag     = sqrtf(gX*gX + gY*gY + gZ*gZ);
    if (mag > 0.5f && mag < 1.5f) {
        float rawLeanRad = atan2f(gY, gZ);
        float rawLeanDeg = rawLeanRad * 180.0f / PI;
        leanDeg = rawLeanDeg - rbxLeanOffset;
    }

    // ---- TODO: wire up GNSS lat/lon from vendor doc ----
    // RaceBox protocol reportedly stores lat/lon as signed int32
    // in 1e-7 degrees, like standard UBX.
    //
    // Example (you MUST fix these offsets to real ones):
    // int32_t lat_e7 = leI32(payload + LAT_OFFSET);
    // int32_t lon_e7 = leI32(payload + LON_OFFSET);
    // double  latDeg = lat_e7 * 1e-7;
    // double  lonDeg = lon_e7 * 1e-7;
    //
    double latDeg = g_snap.latDeg; // placeholder until you fill offsets
    double lonDeg = g_snap.lonDeg; // placeholder until you fill offsets

    uint32_t nowMs = millis();

    // Update atomic snapshot
    portENTER_CRITICAL(&g_snapMux);
    g_snap.connected    = g_connected;
    g_snap.hasFix       = fix_ok;
    g_snap.latDeg       = latDeg;
    g_snap.lonDeg       = lonDeg;
    g_snap.speedKPH     = speed_kph;
    g_snap.gX           = gX;
    g_snap.gY           = gY;
    g_snap.gZ           = gZ;
    g_snap.leanDeg      = leanDeg;
    // yawDeg / pitchDeg left untouched for now
    g_snap.lastUpdateMs = nowMs;
    portEXIT_CRITICAL(&g_snapMux);

    Serial.printf(
        "RBX DATA: fix=%u flags=0x%02X ok=%d speed=%.2f kph  "
        "Gx=%.3f Gy=%.3f Gz=%.3f  LEAN=%.1f deg\n",
        fixStatus, fixFlags, fix_ok, speed_kph, gX, gY, gZ, leanDeg
    );
}

// ===============================
// UBX reassembly from BLE notifications
// ===============================
static void raceboxFeed(const uint8_t *data, size_t len) {
    if (!len) return;

    if (len > RBX_BUF_SIZE - rbxLen) {
        rbxLen = 0;  // overflow protection
    }

    memcpy(rbxBuf + rbxLen, data, len);
    rbxLen += len;

    size_t idx = 0;
    while (rbxLen - idx >= 6) {
        // UBX header
        if (!(rbxBuf[idx] == 0xB5 && rbxBuf[idx + 1] == 0x62)) {
            idx++;
            continue;
        }

        uint8_t  cls      = rbxBuf[idx + 2];
        uint8_t  id       = rbxBuf[idx + 3];
        uint16_t plen     = leU16(&rbxBuf[idx + 4]);
        size_t   frameLen = 6 + plen + 2; // header + payload + checksum

        if (rbxLen - idx < frameLen) {
            // not enough data yet
            break;
        }

        // Checksum
        uint8_t ckA = 0, ckB = 0;
        for (size_t i = idx + 2; i < idx + 6 + plen; ++i) {
            ckA = ckA + rbxBuf[i];
            ckB = ckB + ckA;
        }
        uint8_t expA = rbxBuf[idx + 6 + plen];
        uint8_t expB = rbxBuf[idx + 6 + plen + 1];

        if (ckA != expA || ckB != expB) {
            Serial.println("RaceBox: checksum FAIL, skipping one byte");
            idx++;
            continue;
        }

        const uint8_t *payload = &rbxBuf[idx + 6];

        if (cls == 0xFF && id == 0x01 && plen == 80) {
            parseRaceboxDataMessage(payload);
        } else {
            Serial.printf("RaceBox: UBX cls=0x%02X id=0x%02X len=%u\n",
                          cls, id, plen);
        }

        idx += frameLen;
    }

    // Compact buffer
    if (idx > 0) {
        memmove(rbxBuf, rbxBuf + idx, rbxLen - idx);
        rbxLen -= idx;
    }
}

// ===============================
// BLE notification callback
// ===============================
static void notifyCallback(
    BLERemoteCharacteristic *pChar,
    uint8_t                 *pData,
    size_t                   length,
    bool                     isNotify
) {
    // Optionally print raw to debug:
    // Serial.printf("RX %u bytes\n", (unsigned)length);
    raceboxFeed(pData, length);
}

// ===============================
// BLE client callbacks
// ===============================
class MyClientCallbacks : public BLEClientCallbacks {
    void onConnect(BLEClient *pClient) override {
        Serial.println("[BLE] Connected to RaceBox");
        g_connected = true;
        portENTER_CRITICAL(&g_snapMux);
        g_snap.connected = true;
        portEXIT_CRITICAL(&g_snapMux);
    }

    void onDisconnect(BLEClient *pClient) override {
        Serial.println("[BLE] Disconnected from RaceBox");
        g_connected = false;
        portENTER_CRITICAL(&g_snapMux);
        g_snap.connected = false;
        g_snap.hasFix    = false;
        portEXIT_CRITICAL(&g_snapMux);
    }
};

// ===============================
// Connect helpers
// ===============================
static bool tryConnectWithType(BLEAddress addr, esp_ble_addr_type_t addrType) {
    const char *typeStr =
        (addrType == BLE_ADDR_TYPE_RANDOM) ? "RANDOM" :
        (addrType == BLE_ADDR_TYPE_PUBLIC) ? "PUBLIC" :
        "OTHER";

    Serial.printf("[BLE] Trying connect to %s as %s address...\n",
                  addr.toString().c_str(), typeStr);

    unsigned long start = millis();
    while (!g_client->connect(addr, addrType)) {
        if (millis() - start > CONNECT_TIMEOUT_MS) {
            Serial.printf("[BLE] Connect(%s) timed out.\n", typeStr);
            return false;
        }
        Serial.printf("[BLE] Connect(%s) failed, retrying...\n", typeStr);
        delay(500);
    }

    Serial.printf("[BLE] Connect(%s) succeeded.\n", typeStr);
    return true;
}

static bool connectToRaceBox() {
    Serial.println("[BLE] Initialising client...");

    BLEAddress raceboxAddr(RACEBOX_MAC_STR);

    if (!g_client) {
        g_client = BLEDevice::createClient();
        g_client->setClientCallbacks(new MyClientCallbacks());
    }

    if (g_client->isConnected()) {
        Serial.println("[BLE] Client already connected, disconnecting first...");
        g_client->disconnect();
        delay(200);
    }

    Serial.print("[BLE] Target MAC: ");
    Serial.println(RACEBOX_MAC_STR);

    // 1) Try RANDOM
    if (!tryConnectWithType(raceboxAddr, BLE_ADDR_TYPE_RANDOM)) {
        // 2) Then PUBLIC
        if (!tryConnectWithType(raceboxAddr, BLE_ADDR_TYPE_PUBLIC)) {
            Serial.println("[BLE] Could not connect with RANDOM or PUBLIC address types.");
            g_connected = false;
            return false;
        }
    }

    g_connected = true;
    Serial.println("[BLE] Connected, discovering services...");

    auto services = g_client->getServices();
    if (!services || services->empty()) {
        Serial.println("[BLE] No services found.");
        return true;  // still connected
    }

    for (auto it = services->begin(); it != services->end(); ++it) {
        BLERemoteService *svc = it->second;
        Serial.print("\n[BLE] Service: ");
        Serial.println(svc->getUUID().toString().c_str());

        auto chars = svc->getCharacteristics();
        for (auto itc = chars->begin(); itc != chars->end(); ++itc) {
            BLERemoteCharacteristic *ch = itc->second;
            auto uuidStr = ch->getUUID().toString();

            Serial.print("  Char: ");
            Serial.print(uuidStr.c_str());
            Serial.print("  props: ");
            if (ch->canRead())            Serial.print("READ ");
            if (ch->canWrite())           Serial.print("WRITE ");
            if (ch->canWriteNoResponse()) Serial.print("WRITE_NR ");
            if (ch->canNotify())          Serial.print("NOTIFY ");
            if (ch->canIndicate())        Serial.print("INDICATE ");
            Serial.println();

            if (ch->canNotify()) {
                Serial.print("    -> Subscribing to notifications on ");
                Serial.println(uuidStr.c_str());
                ch->registerForNotify(notifyCallback);
            }
        }
    }

    Serial.println("\n[BLE] Setup complete. Waiting for notifications...");
    return true;
}

// ===============================
// FreeRTOS task on Core 0
// ===============================
static TaskHandle_t g_rbxTaskHandle = nullptr;

static void racebox_task(void *param) {
    Serial.println("[BLE] RaceBox task started on Core 0");

    for (;;) {
        if (!g_connected) {
            Serial.println("[BLE] Not connected, attempting to connect...");
            connectToRaceBox();
        }

        // Even if connected, loop gently; notifications do the heavy lifting.
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

// ===============================
// Public API
// ===============================
void racebox_init() {
    Serial.println("[BLE] racebox_init()");

    BLEDevice::init("ESP32-RaceBox-Display");

    // Create BLE task pinned to Core 0
    if (!g_rbxTaskHandle) {
        xTaskCreatePinnedToCore(
            racebox_task,
            "RaceBoxTask",
            8192,       // stack size
            nullptr,
            1,          // priority
            &g_rbxTaskHandle,
            0           // Core 0
        );
    }
}

void racebox_tick() {
    // currently nothing; kept for future hook if needed
}

bool racebox_is_connected() {
    return g_connected;
}

bool racebox_has_valid_fix() {
    RaceboxSnapshot snap;
    racebox_get_snapshot(snap);

    uint32_t now = millis();
    bool fresh = (now - snap.lastUpdateMs) <= DATA_STALE_MS;

    return snap.connected && snap.hasFix && fresh;
}

void racebox_get_snapshot(RaceboxSnapshot &out) {
    portENTER_CRITICAL(&g_snapMux);
    out = g_snap;
    portEXIT_CRITICAL(&g_snapMux);
}
