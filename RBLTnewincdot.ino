#include <Arduino.h>
#include <SPI.h>
#include <SD.h>
#include <lvgl.h>
#include <Arduino_GFX_Library.h>
#include <Wire.h>
#include "bsp_cst816.h"

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

// =======================
// SESSION STATE
// =======================
bool sessionActive = false;
bool sessionWasManuallyStopped = false;
bool autoStartEnabled = true;   // << NEW
unsigned long lowSpeedStartMs = 0;

float currentSpeedKPH = 0.0f;
bool  rbHasSpeed      = false;
float fakeSpeedKPH    = 0.0f;

// =======================
// LVGL UI objects
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

// =======================
// LAP DETECTOR
// =======================
static int lastSide = 0;
static bool haveLastSide = false;

const float MIN_LAP_TIME_SEC = 1.0f;

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

// =======================
// FAKE GPS + SPEED MODEL
// =======================
static float simPhase = 0.0f;
static bool uiHasDrawnOnce = false;   // Prevent instant green on boot

void feedFakeGps() {
  if (!sfLine.ready) return;

  // Delay fake-speed activation until FIRST UI frame has been painted
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

  // Derive fake speed from simPhase
  fakeSpeedKPH = fabsf(sinf(simPhase)) * 120.0f;

  updateLapFromLatLon(lat, lon);
}

// =============================
// SESSION AUTO-START / STOP
// =============================
void sessionAutoController() {

  // Choose real RaceBox speed (if available) else fake speed
  float speed = rbHasSpeed ? currentSpeedKPH : fakeSpeedKPH;

  // ---------- AUTO START ----------
  // Only if:
  //  • Session is inactive
  //  • autoStartEnabled is true
  //  • speed > 20 km/h
  if (!sessionActive &&
      autoStartEnabled &&
      speed > 20.0f)
  {
    sessionActive = true;
    sessionWasManuallyStopped = false;
    haveCurrentLap = false;
    lowSpeedStartMs = 0;
    Serial.println("SESSION AUTO-STARTED");
  }

  // ---------- AUTO STOP ----------
  if (sessionActive) {
    if (speed < 10.0f) {
      if (lowSpeedStartMs == 0) {
        lowSpeedStartMs = millis();
      } else if (millis() - lowSpeedStartMs > 5000) {

        // Auto-stop event
        sessionActive = false;
        sessionWasManuallyStopped = false;

        // Crucial: allow new sessions to start automatically
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

  sessionActive = false;
  sessionWasManuallyStopped = true;
  autoStartEnabled = false;    // <<< BLOCK AUTO-START
  haveCurrentLap = false;
  lowSpeedStartMs = 0;

  update_ui();
  Serial.println("SESSION MANUALLY STOPPED");
}

// =======================
// LVGL UI BUILD
// =======================

void lvgl_ui_init() {
  lv_obj_clean(lv_scr_act());

  // -------- Status dot (top-left) --------
  status_dot = lv_obj_create(lv_scr_act());
  lv_obj_set_size(status_dot, 12, 12);
  lv_obj_set_style_radius(status_dot, LV_RADIUS_CIRCLE, 0);
  lv_obj_set_style_border_width(status_dot, 0, 0);
  lv_obj_clear_flag(status_dot, LV_OBJ_FLAG_SCROLLABLE);
  lv_obj_align(status_dot, LV_ALIGN_TOP_LEFT, 10, 10);

  // -------- "Lap:" prefix --------
  label_lap_prefix = lv_label_create(lv_scr_act());
  lv_label_set_text(label_lap_prefix, "Lap:");
  lv_obj_set_style_text_font(label_lap_prefix, &lv_font_montserrat_28, 0);
  lv_obj_align(label_lap_prefix, LV_ALIGN_TOP_LEFT, 30, 8);

  // -------- Current lap time --------
  label_current_lap = lv_label_create(lv_scr_act());
  lv_label_set_text(label_current_lap, "--:--.---");
  lv_obj_set_style_text_font(label_current_lap, &lv_font_montserrat_28, 0);
  lv_obj_align_to(label_current_lap, label_lap_prefix,
                  LV_ALIGN_OUT_RIGHT_MID, 10, 0);

  // -------- Best lap --------
  label_best_lap = lv_label_create(lv_scr_act());
  lv_obj_align(label_best_lap, LV_ALIGN_TOP_LEFT, 10, 48);
  lv_obj_set_style_text_font(label_best_lap, &lv_font_montserrat_16, 0);
  lv_label_set_text(label_best_lap, "Best: --:--.---");

  // -------- Lap count --------
  label_lap_count = lv_label_create(lv_scr_act());
  lv_obj_align(label_lap_count, LV_ALIGN_TOP_RIGHT, -10, 48);
  lv_obj_set_style_text_font(label_lap_count, &lv_font_montserrat_16, 0);
  lv_label_set_text(label_lap_count, "Laps: 0");

  // -------- Previous laps --------
  for (uint8_t i = 0; i < NUM_DISPLAY_PREV; i++) {
    labels_prev[i] = lv_label_create(lv_scr_act());
    lv_obj_align(labels_prev[i], LV_ALIGN_TOP_LEFT, 10, 75 + i * 22);
    lv_obj_set_style_text_font(labels_prev[i], &lv_font_montserrat_14, 0);
    lv_label_set_text(labels_prev[i], "");
  }

  // -------- STOP button (bottom-right) --------
  btn_stop = lv_btn_create(lv_scr_act());
  lv_obj_set_size(btn_stop, 60, 30);
  lv_obj_align(btn_stop, LV_ALIGN_BOTTOM_RIGHT, -10, -10);
  lv_obj_add_event_cb(btn_stop, stop_btn_event_cb, LV_EVENT_CLICKED, NULL);
  {
    lv_obj_t *lbl = lv_label_create(btn_stop);
    lv_label_set_text(lbl, "STOP");
    lv_obj_center(lbl);
  }

  // -------- Track prev "<" --------
  btn_prev_track = lv_btn_create(lv_scr_act());
  lv_obj_set_size(btn_prev_track, 40, 30);
  lv_obj_align(btn_prev_track, LV_ALIGN_BOTTOM_LEFT, 10, -10);
  lv_obj_add_event_cb(btn_prev_track, track_btn_event_cb, LV_EVENT_CLICKED, NULL);
  {
    lv_obj_t *lbl = lv_label_create(btn_prev_track);
    lv_label_set_text(lbl, "<");
    lv_obj_center(lbl);
  }

  // -------- Track next ">" --------
  btn_next_track = lv_btn_create(lv_scr_act());
  lv_obj_set_size(btn_next_track, 40, 30);
  lv_obj_align(btn_next_track, LV_ALIGN_BOTTOM_RIGHT, -10, -50);
  lv_obj_add_event_cb(btn_next_track, track_btn_event_cb, LV_EVENT_CLICKED, NULL);
  {
    lv_obj_t *lbl = lv_label_create(btn_next_track);
    lv_label_set_text(lbl, ">");
    lv_obj_center(lbl);
  }

  // -------- Track name --------
  label_track = lv_label_create(lv_scr_act());
  lv_label_set_text(label_track, TRACKS[currentTrackIndex].name);
  lv_obj_align(label_track, LV_ALIGN_BOTTOM_MID, 0, -10);
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
    col = lv_palette_main(LV_PALETTE_GREEN);
  else if (sessionWasManuallyStopped)
    col = lv_palette_main(LV_PALETTE_RED);
  else
    col = lv_palette_main(LV_PALETTE_BLUE);

  lv_obj_set_style_bg_color(status_dot, col, 0);

  uiHasDrawnOnce = true;   // First valid frame happened
}

// =======================
// ARDUINO SETUP
// =======================
void setup() {
  Serial.begin(115200);
  Serial.println("Lap Timer + FakeGPS + Per-Track Storage + Session Logic");

  // -------- Display Init --------
  if (!gfx->begin()) {
    Serial.println("gfx->begin() failed!");
    while (1);
  }
  gfx->fillScreen(BLACK);

  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);

  // -------- LVGL Init --------
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

  // -------- Touch Init --------
  Wire.begin(TP_SDA, TP_SCL);
  bsp_touch_init(&Wire, gfx->getRotation(), LCD_WIDTH, LCD_HEIGHT);

  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  indev_drv.read_cb = touchpad_read;
  touch_indev = lv_indev_drv_register(&indev_drv);

  // -------- SD + Track Load --------
  initSD();
  loadTrackFromSD();
  loadLapsFromSD();

  // -------- Build UI --------
  lvgl_ui_init();
  update_ui();

  Serial.println("Setup complete.");
}

// =======================
// ARDUINO LOOP
// =======================
void loop() {

  // 1) GPS / Fake GPS feed (updates fakeSpeedKPH)
  feedFakeGps();

  // 2) Session auto-start / auto-stop
  sessionAutoController();

  // 3) Serial commands: track switching
  handleSerialCommands();

  // 4) UI update
  update_ui();

  // 5) LVGL internal housekeeping
  lv_tick_inc(20);
  lv_timer_handler();

  delay(20);
}
