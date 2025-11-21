#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <lvgl.h>
#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include "bsp_cst816.h"
#include "racebox_ble.h"   // *** NEW ***

// -------------------------
// Physical panel pinout
// -------------------------
#define TFT_BL     1
#define TFT_RST    0
#define TFT_DC     42
#define TFT_CS     45
#define TFT_MOSI   38
#define TFT_MISO   40
#define TFT_SCLK   39
#define TP_SDA     48
#define TP_SCL     47

// define physical buttons
#define BTN_PREV   6
#define BTN_NEXT   7
#define BTN_STOP   8

// -------------------------
// SD card
// -------------------------
#define SD_CS      41

// -------------------------
// Display
// -------------------------
#define PANEL_WIDTH   240
#define PANEL_HEIGHT  320
#define LCD_WIDTH     320
#define LCD_HEIGHT    240

// -------------------------
// Arduino_GFX
// -------------------------
Arduino_DataBus *bus = new Arduino_ESP32SPI(
    TFT_DC, TFT_CS, TFT_SCLK, TFT_MOSI, TFT_MISO
);

Arduino_GFX *gfx = new Arduino_ST7789(
    bus, TFT_RST, 1, true, PANEL_WIDTH, PANEL_HEIGHT
);

// =======================
// TRACK DEFINITIONS
// =======================
struct TrackDef {
  const char *name;
  double left_lat, left_lon;
  double center_lat, center_lon;
  double right_lat, right_lon;
};

const TrackDef TRACKS[] = {
  { "Andalucia",       37.088263, -2.272684, 37.088249, -2.272745, 37.088219, -2.272799 },
  { "Anglesey GP",     53.188465, -4.496437, 53.188415, -4.496418, 53.188359, -4.496396 },
  { "Brands GP",       51.360300,  0.260057, 51.360252,  0.260093, 51.360207,  0.260127 },
  { "Brands Indy",     51.360300,  0.260057, 51.360252,  0.260093, 51.360207,  0.260127 },
  { "Cadwell",         53.310304, -0.059409, 53.310321, -0.059471, 53.310339, -0.059527 },
  { "Croft",           54.455398, -1.555609, 54.455384, -1.555522, 54.455371, -1.555430 },
  { "Donington",       52.829805, -1.379605, 52.829859, -1.379585, 52.829910, -1.379571 },
  { "Donington Short", 52.829805, -1.379605, 52.829859, -1.379585, 52.829910, -1.379571 },
  { "Knockhill",       56.130967, -3.506586, 56.130918, -3.506594, 56.130867, -3.506605 },
  { "Mallory",         52.598672, -1.336914, 52.598669, -1.336980, 52.598669, -1.337047 },
  { "Oulton",          53.180028, -2.612714, 53.180018, -2.612808, 53.180010, -2.612897 },
  { "Portimao",        37.232101, -8.630921, 37.232123, -8.630825, 37.232144, -8.630731 },
  { "Snetterton 300",  52.463366,  0.944775, 52.463412,  0.944767, 52.463452,  0.944767 }
};

const int TRACK_COUNT = sizeof(TRACKS) / sizeof(TRACKS[0]);
int currentTrackIndex = 10; // Oulton default

// =======================
// LAP STORAGE
// =======================
#define MAX_STORED_LAPS 100
#define NUM_DISPLAY_PREV 5

unsigned long lapTimes[MAX_STORED_LAPS] = {0};
uint16_t storedLapCount = 0;
uint32_t totalLapCount  = 0;
unsigned long best_lap_ms = 0xFFFFFFFF;

bool sdOk = false;

unsigned long currentLapStartMs = 0;
bool haveCurrentLap = false;
bool recordingEnabled = true;

// =======================
// HARDWARE BUTTON STATE
// =======================
static int lastPrevState = HIGH;
static int lastNextState = HIGH;
static int lastStopState = HIGH;
static unsigned long lastHwBtnMs = 0;
const unsigned long HW_BTN_DEBOUNCE_MS = 200;

// =======================
// SESSION STATE
// =======================
bool sessionActive = false;
bool sessionWasManuallyStopped = false;
bool autoStartEnabled = true;
unsigned long lowSpeedStartMs = 0;

// These remain, but are now fed from RaceBox when available.
float currentSpeedKPH = 0.0f;
bool  rbHasSpeed      = false;
float fakeSpeedKPH    = 0.0f;

bool fakeOverrideEnabled = false;
float fakeOverrideSpeedKPH = 0.0f;

// =======================
// LVGL LAP objects
// =======================
static lv_obj_t *label_current_lap;
static lv_obj_t *label_lap_prefix;
static lv_obj_t *label_best_lap;
static lv_obj_t *labels_prev[NUM_DISPLAY_PREV];
static lv_obj_t *label_lap_count;
static lv_obj_t *label_track;
static lv_obj_t *btn_prev_track;
static lv_obj_t *btn_next_track;
static lv_obj_t *btn_stop;
static lv_obj_t *status_dot;
static lv_obj_t *status_GPS;

// =======================
// TELEMETRY PAGE OBJECTS
// =======================

static bool onTelemetryPage = false;

// Telemetry UI objects
static lv_obj_t *label_speed = nullptr;
static lv_obj_t *label_lean = nullptr;
static lv_obj_t *label_max_left = nullptr;
static lv_obj_t *label_max_right = nullptr;
static lv_obj_t *label_yawpitch = nullptr;

// Lean gauge
static lv_obj_t *gauge_arc = nullptr;
static lv_obj_t *needle_line = nullptr;
static lv_obj_t *max_left_line = nullptr;
static lv_obj_t *max_right_line = nullptr;

static lv_obj_t *screen_lap = nullptr;
static lv_obj_t *screen_tele = nullptr;


// Line point buffers (must live as long as objects)
static lv_point_t needle_points[2];
static lv_point_t max_left_points[2];
static lv_point_t max_right_points[2];

// Telemetry data (to be driven from RaceBox later)
static float tele_lean_deg       = 0.0f;
static float tele_max_lean_left  = 0.0f;   // usually negative (left)
static float tele_max_lean_right = 0.0f;   // positive (right)
static float tele_yaw_deg        = 0.0f;
static float tele_pitch_deg      = 0.0f;
// For now, we just use currentSpeedKPH for speed display

// Long-press timing (shared)
const unsigned long LONG_PRESS_MS = 800;

// REAL button press timing
static unsigned long uiRealPressStart = 0;
static bool uiRealPressed = false;

// Hardware STOP timing
static unsigned long hwStopPressStart = 0;
static bool hwStopPressed = false;
static bool hwStopLongHandled = false;

// Gauge long-press timing
static unsigned long gaugePressStart = 0;
static bool gaugePressed = false;


// =======================
// GEO + SF LINE
// =======================
const double DEG2RAD = 3.14159265358979323846 / 180.0;
const double METERS_PER_DEG = 111320.0;

static double g_lat0     = TRACKS[currentTrackIndex].center_lat;
static double g_lon0     = TRACKS[currentTrackIndex].center_lon;
static double g_lat0_rad = g_lat0 * DEG2RAD;

static void latLonToXY(double lat, double lon, float &x, float &y) {
  double dLat = lat - g_lat0;
  double dLon = lon - g_lon0;
  x = (float)(dLon * cos(g_lat0_rad) * METERS_PER_DEG);
  y = (float)(dLat * METERS_PER_DEG);
}

static void xyToLatLon(float x, float y, double &lat, double &lon) {
  lat = g_lat0 + (double)y / METERS_PER_DEG;
  lon = g_lon0 + (double)x / (cos(g_lat0_rad) * METERS_PER_DEG);
}

struct StartFinishLine {
  float xL, yL;
  float xR, yR;
  float dx, dy;
  float nx, ny;
  float xC, yC;
  float len2;
  bool  ready;
};

StartFinishLine sfLine = {0};

// =======================
// TOUCHSCREEN
// =======================
static lv_indev_t *touch_indev = nullptr;

static void touchpad_read(lv_indev_drv_t *indev_drv, lv_indev_data_t *data) {
  uint16_t x, y;
  bsp_touch_read();

  if (bsp_touch_get_coordinates(&x, &y)) {
    data->state   = LV_INDEV_STATE_PRESSED;
    data->point.x = x;
    data->point.y = y;
  } else {
    data->state = LV_INDEV_STATE_RELEASED;
  }
}

// =======================
// SD SUPPORT
// =======================
void formatLapTime(char *buf, size_t sz, unsigned long ms) {
  unsigned int min = ms / 60000;
  unsigned int sec = (ms / 1000) % 60;
  unsigned int millis = ms % 1000;
  snprintf(buf, sz, "%u:%02u.%03u", min, sec, millis);
}

void initSD() {
  Serial.println("Init SD...");

  static SPIClass sdSPI(FSPI);
  sdSPI.begin(TFT_SCLK, TFT_MISO, TFT_MOSI, SD_CS);

  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);

  if (!SD.begin(SD_CS, sdSPI, 10000000)) {
    Serial.println("SD.begin() FAILED");
    sdOk = false;
    return;
  }

  sdOk = true;
  Serial.println("SD OK");
}

// per-track filename like /laps_cadwell.csv
String getTrackFileName() {
  String name = TRACKS[currentTrackIndex].name;
  name.toLowerCase();
  name.replace(" ", "_");
  return "/laps_" + name + ".csv";
}

// Save selected track
void saveTrackToSD() {
  if (!sdOk) return;
  File f = SD.open("/track.cfg", FILE_WRITE);
  if (!f) return;
  f.printf("%d\n", currentTrackIndex);
  f.close();
}

void loadTrackFromSD();

// write a lap to the per-track file
void logLapToSD(unsigned long lapTimeMs) {
  if (!sdOk) return;

  String fileName = getTrackFileName();
  File f = SD.open(fileName, FILE_APPEND);
  if (!f) {
    Serial.printf("SD write fail: %s\n", fileName.c_str());
    sdOk = false;
    return;
  }

  char buf[16];
  formatLapTime(buf, sizeof(buf), lapTimeMs);
  uint8_t isBest = (lapTimeMs == best_lap_ms) ? 1 : 0;

  f.printf("%lu,%s,%u\n", lapTimeMs, buf, isBest);
  f.close();
}

void applyLapInMemory(unsigned long lapTimeMs) {
  totalLapCount++;
  if (lapTimeMs < best_lap_ms) best_lap_ms = lapTimeMs;

  if (storedLapCount < MAX_STORED_LAPS) {
    lapTimes[storedLapCount++] = lapTimeMs;
  } else {
    memmove(lapTimes, lapTimes + 1, (MAX_STORED_LAPS - 1) * sizeof(unsigned long));
    lapTimes[MAX_STORED_LAPS - 1] = lapTimeMs;
  }
}

void storeLap(unsigned long lapTimeMs) {
  applyLapInMemory(lapTimeMs);
  logLapToSD(lapTimeMs);
}

void loadLapsFromSD() {
  if (!sdOk) return;

  String file = getTrackFileName();
  File f = SD.open(file, FILE_READ);
  if (!f) {
    Serial.printf("No laps file: %s\n", file.c_str());
    best_lap_ms = 0xFFFFFFFF;
    storedLapCount = 0;
    totalLapCount  = 0;
    return;
  }

  best_lap_ms = 0xFFFFFFFF;
  storedLapCount = 0;
  totalLapCount  = 0;

  while (f.available()) {
    String line = f.readStringUntil('\n');
    line.trim();
    int c = line.indexOf(',');
    if (c < 0) continue;
    unsigned long lapMs = line.substring(0, c).toInt();
    if (lapMs > 0) applyLapInMemory(lapMs);
  }
  f.close();
}

// =======================
// Init the telemetry page
// =======================

void telemetry_ui_init();
void update_telemetry_ui();
void togglePage();

// =======================
// SF LINE INIT
// =======================
void initStartFinishLine(double lat_left, double lon_left,
                         double lat_right, double lon_right)
{
  float xL, yL, xR, yR;
  latLonToXY(lat_left,  lon_left,  xL, yL);
  latLonToXY(lat_right, lon_right, xR, yR);

  sfLine.xL = xL;
  sfLine.yL = yL;
  sfLine.xR = xR;
  sfLine.yR = yR;
  sfLine.dx = xR - xL;
  sfLine.dy = yR - yL;
  sfLine.len2 = sfLine.dx * sfLine.dx + sfLine.dy * sfLine.dy;

  float len = sqrtf(sfLine.len2);
  if (len < 0.01f) {
    sfLine.ready = false;
    return;
  }

  sfLine.nx = -sfLine.dy / len;
  sfLine.ny =  sfLine.dx / len;
  sfLine.xC = 0.5f * (xL + xR);
  sfLine.yC = 0.5f * (yL + yR);
  sfLine.ready = true;
}

void applyTrack(int idx) {
  currentTrackIndex = idx;

  const TrackDef &t = TRACKS[idx];
  g_lat0     = t.center_lat;
  g_lon0     = t.center_lon;
  g_lat0_rad = g_lat0 * DEG2RAD;

  initStartFinishLine(t.left_lat, t.left_lon,
                      t.right_lat, t.right_lon);

  if (label_track)
    lv_label_set_text(label_track, t.name);
}

void loadTrackFromSD() {
  if (!sdOk) {
    applyTrack(currentTrackIndex);
    return;
  }

  File f = SD.open("/track.cfg", FILE_READ);
  if (!f) {
    applyTrack(currentTrackIndex);
    return;
  }

  String s = f.readStringUntil('\n');
  s.trim();
  f.close();

  int idx = s.toInt();
  if (idx >= 0 && idx < TRACK_COUNT)
    applyTrack(idx);
  else
    applyTrack(currentTrackIndex);
}

void update_ui(); // forward

void setCurrentTrack(int idx) {
  if (idx < 0 || idx >= TRACK_COUNT) return;

  applyTrack(idx);

  sessionActive = false;
  sessionWasManuallyStopped = false;
  autoStartEnabled = true;
  haveCurrentLap = false;
  lowSpeedStartMs = 0;
  currentLapStartMs = 0;

  loadLapsFromSD();

  update_ui();
}

// ======================
// page switcher
// ======================

void togglePage() {
  if (onTelemetryPage) {
    // Back to lap page
    onTelemetryPage = false;
    lvgl_ui_init();
    update_ui();
  } else {
    // Go to telemetry page
    telemetry_ui_init();
    update_telemetry_ui();
  }
}

// =======================
// LAP DETECTOR
// =======================
static int lastSide = 0;
static bool haveLastSide = false;

const float MIN_LAP_TIME_SEC = 10.0f;

static float signedDistanceToLine(float x, float y) {
  float vx = x - sfLine.xL;
  float vy = y - sfLine.yL;
  return vx * sfLine.dy - vy * sfLine.dx;
}

static int signf_simple(float v) {
  if (v > 0) return 1;
  if (v < 0) return -1;
  return 0;
}

void updateLapFromLatLon(double lat, double lon) {
  if (!sfLine.ready) return;
  if (!sessionActive) return;

  float x, y;
  latLonToXY(lat, lon, x, y);

  float d = signedDistanceToLine(x, y);
  int side = signf_simple(d);
  if (side == 0) return;

  if (!haveLastSide) {
    lastSide = side;
    haveLastSide = true;
    return;
  }

  if (side == lastSide) {
    lastSide = side;
    return;
  }

  unsigned long now = millis();
  lastSide = side;

  if (!haveCurrentLap) {
    currentLapStartMs = now;
    haveCurrentLap = true;
    return;
  }

  float dt = (now - currentLapStartMs) / 1000.0f;
  if (dt < MIN_LAP_TIME_SEC) return;

  unsigned long lapTime = now - currentLapStartMs;
  currentLapStartMs = now;
  storeLap(lapTime);
}

void resetStoredLapsForCurrentTrack() {
    // Preserve best_lap_ms
    unsigned long preservedBest = best_lap_ms;

    // Clear in-memory laps
    storedLapCount = 0;
    totalLapCount  = 0;

    for (int i = 0; i < MAX_STORED_LAPS; i++) {
        lapTimes[i] = 0;
    }

    // Remove old CSV
    String fileName = getTrackFileName();
    if (SD.exists(fileName)) {
        SD.remove(fileName);
        Serial.printf("Deleted SD file: %s\n", fileName.c_str());
    }

    // Recreate CSV with ONLY the best lap (if valid)
    if (preservedBest < 0xFFFFFFFF) {
        File f = SD.open(fileName, FILE_WRITE);
        if (f) {
            char buf[16];
            formatLapTime(buf, sizeof(buf), preservedBest);
            f.printf("%lu,%s,1\n", preservedBest, buf); // mark as best
            f.close();
            Serial.printf("Recreated CSV with preserved best lap: %lu ms\n", preservedBest);
        }
        // Re-add the best lap to memory
        lapTimes[0] = preservedBest;
        storedLapCount = 1;
        totalLapCount  = 1;
    }

    // Push changes to UI
    update_ui();

    Serial.printf("Stored laps reset (best lap preserved) for track %s\n",
                  TRACKS[currentTrackIndex].name);
}



// =======================
// FAKE GPS + SPEED MODEL
// =======================
static float simPhase = 0.0f;
static bool uiHasDrawnOnce = false;

void feedFakeGps() {
  if (!sfLine.ready) return;

  if (!uiHasDrawnOnce) {
    fakeSpeedKPH = 0;
    return;
  }

  const float AMP_METERS = 10.0f;
  const float PHASE_STEP = 0.05f;

  simPhase += PHASE_STEP;
  if (simPhase > 2.0f * 3.14159f)
    simPhase -= 2.0f * 3.14159f;

  float offset = AMP_METERS * sinf(simPhase);

  float x = sfLine.xC + offset * sfLine.nx;
  float y = sfLine.yC + offset * sfLine.ny;

  double lat, lon;
  xyToLatLon(x, y, lat, lon);

  fakeSpeedKPH = fabsf(sinf(simPhase)) * 120.0f;

  updateLapFromLatLon(lat, lon);
}

// =============================
// SESSION AUTO-START / STOP
// =============================
void sessionAutoController() {
  float speed;

  RaceboxSnapshot rbx;
  racebox_get_snapshot(rbx);

  // Block auto-start unless:
  //   • Fake mode is ON, OR
  //   • RaceBox is connected AND has a valid fix
  bool canAutoStart =
    fakeOverrideEnabled ||
    (rbx.connected && racebox_has_valid_fix());

  if (fakeOverrideEnabled) {
    speed = fakeOverrideSpeedKPH;
  } else if (rbHasSpeed) {
    speed = currentSpeedKPH;
  } else {
    speed = fakeSpeedKPH;
  }

  // AUTO START
  if (!sessionActive &&
    autoStartEnabled &&
    canAutoStart &&      // << NEW GUARD
    speed > 30.0f)
{
    sessionActive = true;
    sessionWasManuallyStopped = false;
    haveCurrentLap = false;
    lowSpeedStartMs = 0;
    Serial.println("SESSION AUTO-STARTED");
}

  // AUTO STOP
  if (sessionActive) {
    if (speed < 10.0f) {
      if (lowSpeedStartMs == 0) {
        lowSpeedStartMs = millis();
      } else if (millis() - lowSpeedStartMs > 5000) {
        sessionActive = false;
        sessionWasManuallyStopped = false;
        autoStartEnabled = true;
        haveCurrentLap = false;
        lowSpeedStartMs = 0;
        Serial.println("SESSION AUTO-STOPPED");
      }
    } else {
      lowSpeedStartMs = 0;
    }
  }
}

void stopSessionManual() {
  sessionActive = false;
  sessionWasManuallyStopped = true;
  autoStartEnabled = true;
  haveCurrentLap = false;
  recordingEnabled = true;
  lowSpeedStartMs = 0;

  update_ui();
  Serial.println("SESSION MANUALLY STOPPED");
}
// =============================
// Current track reset popup
// =============================

static lv_obj_t *reset_mbox = NULL;

static void reset_mbox_event_cb(lv_event_t *e) {
    lv_obj_t *mbox = lv_event_get_current_target(e);   // <-- this is the msgbox
    const char *btn_txt = lv_msgbox_get_active_btn_text(mbox);

    if (btn_txt && strcmp(btn_txt, "OK") == 0) {
        resetStoredLapsForCurrentTrack();
    }

    lv_msgbox_close(mbox);
    reset_mbox = NULL;
}

static void show_reset_popup() {
    if (reset_mbox != NULL) return;   // Already open

    static const char *btns[] = { "OK", "Cancel", "" };

    reset_mbox = lv_msgbox_create(NULL,
                                  "Reset Lap Times?",
                                  "Clear stored laps for this track?\n"
                                  "(Best lap will be kept)",
                                  btns, false);

    lv_obj_center(reset_mbox);

    // Attach the callback to the MSGBOX — NOT the button matrix
    lv_obj_add_event_cb(reset_mbox,
                        reset_mbox_event_cb,
                        LV_EVENT_VALUE_CHANGED,
                        NULL);
}

//==============================
// Hardware button stuff for later
//==============================

void handleHardwareButtons() {
  unsigned long now = millis();

  int prevState = digitalRead(BTN_PREV);
  int nextState = digitalRead(BTN_NEXT);
  int stopState = digitalRead(BTN_STOP);

  // ---------- Previous track (falling edge: HIGH -> LOW) ----------
  if (prevState == LOW && lastPrevState == HIGH) {
    int idx = (currentTrackIndex - 1 + TRACK_COUNT) % TRACK_COUNT;

    // Use setCurrentTrack so laps & session state are handled consistently
    setCurrentTrack(idx);
    saveTrackToSD();

    lastHwBtnMs = now;
  }

  // ---------- Next track (falling edge) ----------
  if (nextState == LOW && lastNextState == HIGH) {
    int idx = (currentTrackIndex + 1) % TRACK_COUNT;

    setCurrentTrack(idx);
    saveTrackToSD();

    lastHwBtnMs = now;
  }

  // ---------- Stop button (short/long) ----------
  if (stopState == LOW && !hwStopPressed) {
    // Just pressed
    hwStopPressed = true;
    hwStopLongHandled = false;
    hwStopPressStart = now;
  }

  if (stopState == LOW && hwStopPressed && !hwStopLongHandled) {
    if (now - hwStopPressStart >= LONG_PRESS_MS) {
    // LONG PRESS → stop session
    stopSessionManual();
    Serial.println("SESSION STOPPED (HW LONG PRESS)");
    hwStopLongHandled = true;
    lastHwBtnMs = now;
    }
  }

  if (stopState == HIGH && hwStopPressed) {
  // Released
  if (!hwStopLongHandled) {
    // SHORT PRESS → page toggle
    togglePage();
    Serial.println("PAGE SWITCH (HW SHORT PRESS)");
    lastHwBtnMs = now;
  }
  hwStopPressed = false;
  hwStopLongHandled = false;
  }

  // Remember previous states for edge detection
  lastPrevState = prevState;
  lastNextState = nextState;
  lastStopState = stopState;
}

// =======================
// LVGL EVENT HANDLERS
// =======================
static void track_btn_event_cb(lv_event_t *e) {
  lv_obj_t *btn = lv_event_get_target(e);
  int idx = currentTrackIndex;

  if (btn == btn_prev_track) {
    idx = (currentTrackIndex - 1 + TRACK_COUNT) % TRACK_COUNT;
  } else if (btn == btn_next_track) {
    idx = (currentTrackIndex + 1) % TRACK_COUNT;
  } else {
    return;
  }

  setCurrentTrack(idx);
  saveTrackToSD();
  Serial.printf("Track changed to %d: %s\n", idx, TRACKS[idx].name);
}

static void stop_btn_event_cb(lv_event_t *e) {
  if (lv_event_get_code(e) != LV_EVENT_CLICKED) return;
  stopSessionManual();
}

// =======================
// LVGL Lap UI
// =======================

void lvgl_ui_init() {
  
  // Create a **dedicated screen** for the lap page
    screen_lap = lv_obj_create(NULL);
    lv_scr_load(screen_lap);

  // -------- Status dot (top-left) --------
  status_dot = lv_obj_create(screen_lap);
  lv_obj_set_size(status_dot, 12, 12);
  lv_obj_set_style_radius(status_dot, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_width(status_dot, 0, 0);
  lv_obj_clear_flag(status_dot, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_align(status_dot, LV_ALIGN_TOP_LEFT, 10, 10);

  // -------- GPS Status Dot --------
  status_GPS = lv_obj_create(screen_lap);
  lv_obj_set_size(status_GPS, 12, 12);
  lv_obj_set_style_radius(status_GPS, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_width(status_GPS, 0, 0);
  lv_obj_clear_flag(status_GPS, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_align(status_GPS, LV_ALIGN_TOP_RIGHT, -10, 10);

  // Initial colour — safe default
  lv_obj_set_style_bg_color(status_GPS, lv_palette_main(LV_PALETTE_RED), 0);



  // -------- "Lap:" prefix --------
  label_lap_prefix = lv_label_create(screen_lap);
  lv_label_set_text(label_lap_prefix, "Lap:");
  lv_obj_set_style_text_font(label_lap_prefix, &lv_font_montserrat_28, 0);
  lv_obj_align(label_lap_prefix, LV_ALIGN_TOP_LEFT, 30, 8);

  // -------- Current lap time --------
  label_current_lap = lv_label_create(screen_lap);
  lv_label_set_text(label_current_lap, "--:--.---");
  lv_obj_set_style_text_font(label_current_lap, &lv_font_montserrat_28, 0);
  lv_obj_align_to(label_current_lap, label_lap_prefix,
                  LV_ALIGN_OUT_RIGHT_MID, 10, 0);

  // -------- Best lap --------
  label_best_lap = lv_label_create(screen_lap);
  lv_obj_align(label_best_lap, LV_ALIGN_TOP_LEFT, 10, 48);
  lv_obj_set_style_text_font(label_best_lap, &lv_font_montserrat_16, 0);
  lv_label_set_text(label_best_lap, "Best: --:--.---");

  // -------- Lap count --------
  label_lap_count = lv_label_create(screen_lap);
  lv_obj_align(label_lap_count, LV_ALIGN_TOP_RIGHT, -10, 48);
  lv_obj_set_style_text_font(label_lap_count, &lv_font_montserrat_16, 0);
  lv_label_set_text(label_lap_count, "Laps: 0");

  // -------- Previous laps --------
  for (uint8_t i = 0; i < NUM_DISPLAY_PREV; i++) {
    labels_prev[i] = lv_label_create(screen_lap);
    lv_obj_align(labels_prev[i], LV_ALIGN_TOP_LEFT, 10, 75 + i * 22);
    lv_obj_set_style_text_font(labels_prev[i], &lv_font_montserrat_14, 0);
    lv_label_set_text(labels_prev[i], "");
  }


  // -------- Track prev "<" --------
  btn_prev_track = lv_btn_create(screen_lap);
  lv_obj_set_size(btn_prev_track, 40, 30);
  lv_obj_align(btn_prev_track, LV_ALIGN_BOTTOM_LEFT, 10, -10);
  lv_obj_add_event_cb(btn_prev_track, track_btn_event_cb, LV_EVENT_CLICKED, NULL);
  {
    lv_obj_t *lbl = lv_label_create(btn_prev_track);
    lv_label_set_text(lbl, "<");
    lv_obj_center(lbl);
  }

  // -------- Track next ">" --------
  btn_next_track = lv_btn_create(screen_lap);
  lv_obj_set_size(btn_next_track, 40, 30);
  lv_obj_align(btn_next_track, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
  lv_obj_add_event_cb(btn_next_track, track_btn_event_cb, LV_EVENT_CLICKED, NULL);
  {
    lv_obj_t *lbl = lv_label_create(btn_next_track);
    lv_label_set_text(lbl, ">");
    lv_obj_center(lbl);
  }

  // -------- Track name --------
  label_track = lv_label_create(screen_lap);
  lv_label_set_text(label_track, TRACKS[currentTrackIndex].name);
  lv_obj_align(label_track, LV_ALIGN_BOTTOM_MID, 0, -10);

  // ----- REAL GPS button -----
  lv_obj_t *btn_real = lv_btn_create(screen_lap);
  lv_obj_set_size(btn_real, 60, 30);
  lv_obj_align(btn_real, LV_ALIGN_BOTTOM_RIGHT, -10, -120);

  lv_obj_add_event_cb(btn_real, [](lv_event_t *e){
  lv_event_code_t code = lv_event_get_code(e);

    if (code == LV_EVENT_PRESSED) {
      uiRealPressStart = millis();
      uiRealPressed = true;
    }
    else if (code == LV_EVENT_RELEASED) {
      if (!uiRealPressed) return;
        uiRealPressed = false;
        unsigned long elapsed = millis() - uiRealPressStart;

      if (elapsed >= LONG_PRESS_MS) {
        // LONG PRESS → REAL GPS mode
        fakeOverrideEnabled = false;
        Serial.println("REAL GPS MODE ACTIVATED (UI LONG PRESS)");
      } else {
        // SHORT PRESS → page toggle
        togglePage();
        Serial.println("PAGE SWITCH (UI SHORT PRESS: REAL)");
      }
    }
  }, LV_EVENT_ALL, NULL);

  {
    lv_obj_t *lbl = lv_label_create(btn_real);
    lv_label_set_text(lbl, "REAL");
    lv_obj_center(lbl);
  }
}

// ===================================
// LVGL Telemetry UI
// ===================================

void telemetry_ui_init() {
  onTelemetryPage = true;

  // Create a **dedicated screen** for the telemetry page
    screen_tele = lv_obj_create(NULL);
    lv_scr_load(screen_tele);

  // -------- Status dot (top-left) --------
  status_dot = lv_obj_create(screen_tele);
  lv_obj_set_size(status_dot, 12, 12);
  lv_obj_set_style_radius(status_dot, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_width(status_dot, 0, 0);
  lv_obj_clear_flag(status_dot, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_align(status_dot, LV_ALIGN_TOP_LEFT, 10, 10);

  // -------- GPS Status Dot (top-right) --------
  status_GPS = lv_obj_create(screen_tele);
  lv_obj_set_size(status_GPS, 12, 12);
  lv_obj_set_style_radius(status_GPS, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_width(status_GPS, 0, 0);
  lv_obj_clear_flag(status_GPS, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_align(status_GPS, LV_ALIGN_TOP_RIGHT, -10, 10);
  lv_obj_set_style_bg_color(status_GPS, lv_palette_main(LV_PALETTE_RED), 0);

  // --- Tap status dot to return to Lap page ---
  lv_obj_add_event_cb(status_dot, [](lv_event_t *e){
    if (lv_event_get_code(e) == LV_EVENT_CLICKED) {
        togglePage();
        Serial.println("PAGE SWITCH (tap status dot)");
    }
  }, LV_EVENT_CLICKED, NULL);

  // -------- Speed (top, medium font) --------
  label_speed = lv_label_create(screen_tele);
  lv_obj_set_style_text_font(label_speed, &lv_font_montserrat_28, 0);
  lv_label_set_text(label_speed, "0.0 km/h");
  lv_obj_align(label_speed, LV_ALIGN_TOP_MID, 0, 30);

  // -------- Lean Gauge (semicircle) --------
  const int GAUGE_SIZE = 160;

  gauge_arc = lv_arc_create(screen_tele);
  lv_obj_set_size(gauge_arc, GAUGE_SIZE, GAUGE_SIZE);
  // top semicircle from left (180) to right (360)
  lv_arc_set_bg_angles(gauge_arc, 180, 360);
  lv_arc_set_angles(gauge_arc, 180, 360);
  lv_arc_set_rotation(gauge_arc, 0);
  lv_arc_set_range(gauge_arc, 0, 100);
  lv_obj_center(gauge_arc);
  lv_obj_set_y(gauge_arc, lv_obj_get_y(gauge_arc) + 40); // drop slightly down

  // Needle line
  needle_line = lv_line_create(gauge_arc);
  lv_obj_set_style_line_width(needle_line, 3, 0);
  lv_obj_set_style_line_color(needle_line, lv_palette_main(LV_PALETTE_RED), 0);
  lv_line_set_points(needle_line, needle_points, 2);

  // Max left tick
  max_left_line = lv_line_create(gauge_arc);
  lv_obj_set_style_line_width(max_left_line, 3, 0);
  lv_obj_set_style_line_color(max_left_line, lv_color_hex(0xFF5500), 0);  // bright orange-red
  lv_line_set_points(max_left_line, max_left_points, 2);

  // Max right tick
  max_right_line = lv_line_create(gauge_arc);
  lv_obj_set_style_line_width(max_right_line, 3, 0);
  lv_obj_set_style_line_color(max_right_line, lv_color_hex(0xFF5500), 0);  // bright orange-red
  lv_line_set_points(max_right_line, max_right_points, 2);

  // Gauge long-press to reset max lean
  lv_obj_add_event_cb(gauge_arc, [](lv_event_t *e) {
      lv_event_code_t code = lv_event_get_code(e);
      if (code == LV_EVENT_PRESSED) {
        gaugePressStart = millis();
        gaugePressed = true;
      } else if (code == LV_EVENT_RELEASED) {
        if (!gaugePressed) return;
        unsigned long elapsed = millis() - gaugePressStart;
        gaugePressed = false;
        if (elapsed >= LONG_PRESS_MS) {
          tele_max_lean_left  = 0.0f;
          tele_max_lean_right = 0.0f;
          Serial.println("Telemetry: max lean reset via gauge long-press");
        }
      }
  }, LV_EVENT_ALL, NULL);

  // -------- Lean / Max / Yaw/Pitch labels --------

  // Current lean
  label_lean = lv_label_create(screen_tele);
  lv_obj_set_style_text_font(label_lean, &lv_font_montserrat_16, 0);
  lv_label_set_text(label_lean, "Lean: 0.0°");
  lv_obj_align(label_lean, LV_ALIGN_CENTER, 0, 70);

  // Max left/right
  label_max_left = lv_label_create(screen_tele);
  lv_obj_set_style_text_font(label_max_left, &lv_font_montserrat_14, 0);
  lv_label_set_text(label_max_left, "Max L: 0.0°");
  lv_obj_align(label_max_left, LV_ALIGN_BOTTOM_LEFT, 10, -20);

  label_max_right = lv_label_create(screen_tele);
  lv_obj_set_style_text_font(label_max_right, &lv_font_montserrat_14, 0);
  lv_label_set_text(label_max_right, "Max R: 0.0°");
  lv_obj_align(label_max_right, LV_ALIGN_BOTTOM_RIGHT, -10, -20);

  // Yaw/Pitch (bottom centre)
  label_yawpitch = lv_label_create(screen_tele);
  lv_obj_set_style_text_font(label_yawpitch, &lv_font_montserrat_14, 0);
  lv_label_set_text(label_yawpitch, "Yaw: 0.0°  Pitch: 0.0°");
  lv_obj_align(label_yawpitch, LV_ALIGN_BOTTOM_MID, 0, -5);


  // ----- FAKE SPEED 120 -----
  lv_obj_t *btn_fake120 = lv_btn_create(screen_tele);
  lv_obj_set_size(btn_fake120, 60, 30);
  lv_obj_align(btn_fake120, LV_ALIGN_BOTTOM_RIGHT, -10, -50);
  lv_obj_add_event_cb(btn_fake120, [](lv_event_t *e){
    fakeOverrideEnabled = true;
    fakeOverrideSpeedKPH = 120.0f;
    Serial.println("FAKE SPEED SET TO 120");
  }, LV_EVENT_CLICKED, NULL);
  {
    lv_obj_t *lbl = lv_label_create(btn_fake120);
    lv_label_set_text(lbl, "120");
    lv_obj_center(lbl);
  }

  // ----- FAKE SPEED 0 -----
  lv_obj_t *btn_fake0 = lv_btn_create(screen_tele);
  lv_obj_set_size(btn_fake0, 60, 30);
  lv_obj_align(btn_fake0, LV_ALIGN_BOTTOM_RIGHT, -10, -85);
  lv_obj_add_event_cb(btn_fake0, [](lv_event_t *e){
    fakeOverrideEnabled = true;
    fakeOverrideSpeedKPH = 0.0f;
    Serial.println("FAKE SPEED SET TO 0");
  }, LV_EVENT_CLICKED, NULL);
  {
    lv_obj_t *lbl = lv_label_create(btn_fake0);
    lv_label_set_text(lbl, "0");
    lv_obj_center(lbl);
  }

  // ----- Long press anywhere to reset laps -----
  lv_obj_add_event_cb(screen_tele, [](lv_event_t *e){
    if (lv_event_get_code(e) == LV_EVENT_LONG_PRESSED) {
        show_reset_popup();
    }
  }, LV_EVENT_LONG_PRESSED, NULL);
}

// =======================
// SERIAL COMMAND PARSER
// =======================
void handleSerialCommands() {
  if (!Serial.available()) return;

  String cmd = Serial.readStringUntil('\n');
  cmd.trim();

  if (cmd == "tracks") {
    Serial.println("Tracks:");
    for (int i = 0; i < TRACK_COUNT; i++)
      Serial.printf("%2d: %s%s\n", i, TRACKS[i].name,
                    i == currentTrackIndex ? "  <current>" : "");
    return;
  }

  if (!cmd.startsWith("track ")) {
    Serial.printf("Unknown: '%s'\n", cmd.c_str());
    return;
  }

  String arg = cmd.substring(6);
  arg.trim();

  int idx = arg.toInt();
  bool ok = false;

  if (String(idx) == arg && idx >= 0 && idx < TRACK_COUNT) {
    ok = true;
  } else {
    // Name match
    String needle = arg;
    needle.toLowerCase();
    for (int i = 0; i < TRACK_COUNT; i++) {
      String n = TRACKS[i].name;
      n.toLowerCase();
      if (n.indexOf(needle) >= 0) {
        idx = i;
        ok = true;
        break;
      }
    }
  }

  if (!ok) {
    Serial.printf("Track not found: '%s'\n", arg.c_str());
    return;
  }

  setCurrentTrack(idx);
  saveTrackToSD();
  Serial.printf("Track set to %d: %s\n", idx, TRACKS[idx].name);
}

// =======================
// Update the telemetry
// =======================

static void set_label_float(lv_obj_t *lbl, const char *prefix, float value, const char *suffix = "")
{
  char buf[32];
  dtostrf(value, 0, 1, buf);   // 1 decimal place
  char out[48];
  snprintf(out, sizeof(out), "%s%s%s", prefix, buf, suffix);
  lv_label_set_text(lbl, out);
}

void update_telemetry_ui() {
  // For now, speed display is driven from currentSpeedKPH
  float speed = currentSpeedKPH;
  float lean = tele_lean_deg;

  // Speed
  {
    char buf[32];
    dtostrf(speed, 0, 1, buf);
    char out[40];
    snprintf(out, sizeof(out), "%s km/h", buf);
    lv_label_set_text(label_speed, out);
  }

  // Lean
  set_label_float(label_lean, "Lean: ", lean, "°");

  // Max left / right
  set_label_float(label_max_left,  "Max L: ", fabsf(tele_max_lean_left), "°");
  set_label_float(label_max_right, "Max R: ", fabsf(tele_max_lean_right), "°");

  // Yaw + pitch
  {
    char yawbuf[16], pitchbuf[16], out[48];
    dtostrf(tele_yaw_deg, 0, 1, yawbuf);
    dtostrf(tele_pitch_deg, 0, 1, pitchbuf);
    snprintf(out, sizeof(out), "Yaw: %s°  Pitch: %s°", yawbuf, pitchbuf);
    lv_label_set_text(label_yawpitch, out);
  }
  
  // --- Lean live + maxima ---
  const float LEAN_MAX = 60.0f;
  if (lean > LEAN_MAX) lean = LEAN_MAX;
  if (lean < -LEAN_MAX) lean = -LEAN_MAX;

  // Update max left/right
  if (lean < tele_max_lean_left)
    tele_max_lean_left = lean;
  if (lean > tele_max_lean_right)
    tele_max_lean_right = lean;

  // --- Needle + tick geometry ---
  // Map lean [-60..+60] -> LVGL angle [180..360] (left..top..right)
  float angle_lean = 270.0f + (lean * 90.0f / LEAN_MAX);

  // Use same mapping for max left/right (if non-zero)
  float angle_maxL = 270.0f + (tele_max_lean_left * 90.0f / LEAN_MAX);
  float angle_maxR = 270.0f + (tele_max_lean_right * 90.0f / LEAN_MAX);

  int size = lv_obj_get_width(gauge_arc);
  int cx = size / 2;
  int cy = size / 2;
  float radius = (float)size / 2.0f - 6.0f;  // margin from edge

  auto set_line_from_angle = [&](lv_point_t *pts, float angleDeg, float rInner, float rOuter) {
    float rad = angleDeg * DEG2RAD;
    pts[0].x = cx + (int16_t)(rInner * cosf(rad));
    pts[0].y = cy + (int16_t)(rInner * sinf(rad));
    pts[1].x = cx + (int16_t)(rOuter * cosf(rad));
    pts[1].y = cy + (int16_t)(rOuter * sinf(rad));
  };

  // Needle: full radius from centre to arc
  set_line_from_angle(needle_points, angle_lean, 0.0f, radius);
  lv_line_set_points(needle_line, needle_points, 2);

  // Max L/R ticks: small segments near the arc
  float rInnerTick = radius - 6.0f;
  float rOuterTick = radius;

  set_line_from_angle(max_left_points, angle_maxL, rInnerTick, rOuterTick);
  lv_line_set_points(max_left_line, max_left_points, 2);

  set_line_from_angle(max_right_points, angle_maxR, rInnerTick, rOuterTick);
  lv_line_set_points(max_right_line, max_right_points, 2);

  // -------- Status dot (session) --------
  lv_color_t col;
  if (sessionActive)
    col = lv_palette_main(LV_PALETTE_GREEN);
  else if (sessionWasManuallyStopped)
    col = lv_palette_main(LV_PALETTE_RED);
  else
    col = lv_palette_main(LV_PALETTE_BLUE);

  if (status_dot) {
    lv_obj_set_style_bg_color(status_dot, col, 0);
  }

  // -------- GPS Status Dot --------
  
  RaceboxSnapshot rbx;
  racebox_get_snapshot(rbx);

  lv_color_t gpsCol;

  if (fakeOverrideEnabled) {
    gpsCol = lv_palette_main(LV_PALETTE_YELLOW);       // Fake mode
  }
  else if (!rbx.connected) {
    gpsCol = lv_palette_main(LV_PALETTE_RED);          // No BLE
  }
  else if (racebox_has_valid_fix()) {
    gpsCol = lv_palette_main(LV_PALETTE_GREEN);        // BLE + Fix
  }
  else {
    gpsCol = lv_palette_main(LV_PALETTE_BLUE);         // BLE, no fix
  }

  lv_obj_set_style_bg_color(status_GPS, gpsCol, 0);


  // We can treat this as "UI has drawn once" too
  uiHasDrawnOnce = true;
}

// =======================
// UI UPDATE
// =======================
void update_ui() {
  char buf[32];

  // Current lap
  if (haveCurrentLap && sessionActive) {
    unsigned long now = millis();
    unsigned long elapsed = now - currentLapStartMs;
    formatLapTime(buf, sizeof(buf), elapsed);
    lv_label_set_text(label_current_lap, buf);
  } else {
    lv_label_set_text(label_current_lap, "--:--.---");
  }

  // Best lap
  if (best_lap_ms < 0xFFFFFFFF) {
    formatLapTime(buf, sizeof(buf), best_lap_ms);
    lv_label_set_text_fmt(label_best_lap, "Best: %s", buf);
  } else {
    lv_label_set_text(label_best_lap, "Best: --:--.---");
  }

  // Lap count
  lv_label_set_text_fmt(label_lap_count, "Laps: %lu",
                        (unsigned long) totalLapCount);

  // Previous laps
  for (int i = 0; i < NUM_DISPLAY_PREV; i++) {
    if (i >= storedLapCount) {
      lv_label_set_text(labels_prev[i], "");
      continue;
    }

    int idx = storedLapCount - 1 - i;
    unsigned long lapMs = lapTimes[idx];

    formatLapTime(buf, sizeof(buf), lapMs);
    lv_label_set_text_fmt(labels_prev[i],
                          "#%d: %s",
                          (totalLapCount - storedLapCount) + idx + 1,
                          buf);
  }

  // -------- Status dot --------
  lv_color_t col;
  if (sessionActive)
    col = lv_palette_main(LV_PALETTE_GREEN); // Active session
  else if (sessionWasManuallyStopped)
    col = lv_palette_main(LV_PALETTE_RED); // Manual stop
  else
    col = lv_palette_main(LV_PALETTE_BLUE); // Standby

  lv_obj_set_style_bg_color(status_dot, col, 0);

  // -------- GPS Status Dot --------
  RaceboxSnapshot rbx;
  racebox_get_snapshot(rbx);

  lv_color_t gpsCol;

  if (fakeOverrideEnabled) {
    gpsCol = lv_palette_main(LV_PALETTE_YELLOW);       // Fake mode
  }
  else if (!rbx.connected) {
    gpsCol = lv_palette_main(LV_PALETTE_RED);          // No BLE
  }
  else if (racebox_has_valid_fix()) {
    gpsCol = lv_palette_main(LV_PALETTE_GREEN);        // BLE + Fix
  }
  else {
    gpsCol = lv_palette_main(LV_PALETTE_BLUE);         // BLE, no fix
  }

  lv_obj_set_style_bg_color(status_GPS, gpsCol, 0);

}

/// =======================
// ARDUINO SETUP
// =======================
void setup() {
  Serial.begin(115200);
  Serial.println("Lap Timer + FakeGPS + Per-Track Storage + Session Logic");

  // Hardware buttons
  pinMode(BTN_PREV, INPUT_PULLUP);
  pinMode(BTN_NEXT, INPUT_PULLUP);
  pinMode(BTN_STOP, INPUT_PULLUP);

  // Display Init
  if (!gfx->begin()) {
    Serial.println("gfx->begin() failed!");
    while (1);
  }
  gfx->fillScreen(BLACK);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  // LVGL Init
  lv_init();

  static lv_disp_draw_buf_t draw_buf;
  static lv_color_t buf1[LCD_WIDTH * 10];
  lv_disp_draw_buf_init(&draw_buf, buf1, NULL, LCD_WIDTH * 10);

  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.draw_buf = &draw_buf;
  disp_drv.hor_res = LCD_WIDTH;
  disp_drv.ver_res = LCD_HEIGHT;

  disp_drv.flush_cb = [](lv_disp_drv_t *disp,
                         const lv_area_t *area,
                         lv_color_t *color_p)
  {
    int32_t w = area->x2 - area->x1 + 1;
    int32_t h = area->y2 - area->y1 + 1;

    gfx->draw16bitRGBBitmap(
        area->x1, area->y1,
        (uint16_t *)color_p,
        w, h
    );

    lv_disp_flush_ready(disp);
  };

  lv_disp_drv_register(&disp_drv);

  // Touch Init
  Wire.begin(TP_SDA, TP_SCL);
  bsp_touch_init(&Wire, gfx->getRotation(), LCD_WIDTH, LCD_HEIGHT);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = touchpad_read;
  touch_indev = lv_indev_drv_register(&indev_drv);

  // SD + Track/Laps
  initSD();
  loadTrackFromSD();
  loadLapsFromSD();

  // Build UI
  lvgl_ui_init();
  update_ui();

  // *** RaceBox BLE init (Core 0 task) ***
  racebox_init();

  Serial.println("Setup complete.");
}

// =======================
// ARDUINO LOOP
// =======================
void loop() {

  // ===========================================
// 1) RaceBox vs Fake GPS + GPS Outage Grace
// ===========================================
    // How long we tolerate missing GPS before stopping session
    const unsigned long GPS_OUTAGE_GRACE_MS = 5000;
    static unsigned long gpsLossStart = 0;

    RaceboxSnapshot rbx;
    racebox_get_snapshot(rbx);

    // IMU ALWAYS UPDATES whenever BLE is connected
    if (rbx.connected) {
    tele_lean_deg   = -rbx.leanDeg;
    tele_yaw_deg    = rbx.yawDeg;
    tele_pitch_deg = -atan2(rbx.gX, rbx.gZ) * 57.2957795f;
    }

    if (recordingEnabled) {
    // -------------------------
    // FAKE OVERRIDE MODE
    // -------------------------
    }

    if (fakeOverrideEnabled)
    {
        // Fake GNSS path only affects speed + lap simulation
        feedFakeGps();
        currentSpeedKPH = fakeOverrideSpeedKPH;
        rbHasSpeed = false;
        gpsLossStart = 0;
    }
    else {
        // REAL MODE
        if (racebox_has_valid_fix()) {
            rbHasSpeed      = true;
            currentSpeedKPH = rbx.speedKPH;

            gpsLossStart = 0;

            if (rbx.latDeg != 0.0 || rbx.lonDeg != 0.0)
            updateLapFromLatLon(rbx.latDeg, rbx.lonDeg);
            }
        else {
            // outage logic stays identical
            rbHasSpeed = false;

            if (gpsLossStart == 0)
                gpsLossStart = millis();

            unsigned long outage = millis() - gpsLossStart;

            if (outage >= GPS_OUTAGE_GRACE_MS) {
                    currentSpeedKPH = 0.0f;

                if (sessionActive) {
                        sessionActive = false;
                        sessionWasManuallyStopped = false;
                        autoStartEnabled = true;
                        haveCurrentLap = false;
                        lowSpeedStartMs = 0;
                        Serial.println("SESSION STOPPED (GPS LOST)");
                }
            }
            else {
                // hold last speed during grace
                currentSpeedKPH = currentSpeedKPH;
            }
        }
    }


  if (!rbx.connected) {
      // Recording disabled = hard stop on everything
      rbHasSpeed      = false;
      currentSpeedKPH = 0.0f;
      gpsLossStart    = 0;
  }


  // *** 1b) BLE housekeeping (cheap, non-blocking) ***
  racebox_tick();

  // 2) Session auto-start / stop
  sessionAutoController();

  // 3) Serial commands
  handleSerialCommands();

  // 4) Hardware input
  handleHardwareButtons();

  // 5) UI update
    // 5) UI update
  if (onTelemetryPage) {
    update_telemetry_ui();
  } else {
    update_ui();
  }

  // 6) LVGL
  lv_tick_inc(20);
  lv_timer_handler();

  delay(20);
}