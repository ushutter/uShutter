#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <math.h>

// =================================================
// Version
// =================================================
#define VERSION_STR "ver 0.9.2"
#define NAME  "uShutter-5"
#define SLOGAN ""

// =================================================
// Parameters
// =================================================
#define MIN_VALID_EXPOSURE 1000       // Minimum valid exposure time (us)
#define MAX_VALID_EXPOSURE 1500000    // Maximum valid exposure time (us)

#define BTN_RESET 6                   // Reset button pin (active-low)
#define BTN_DEBOUNCE_MS 50            // Button debounce time (ms)
#define BTN_LONGPRESS_MS 1200         // Long press threshold (ms)

#define STABLE_COUNT 5
#define STABLE_TOLERANCE_PERCENT 10.0 // Stability threshold (%), compared against stability_percent (CV%)

#define MATCH_THRESHOLD_PERMILLE 150  // 15% threshold for standard shutter match

#define LEFT_MARGIN   3
#define RIGHT_MARGIN  3

// Dual-capture timeout: if a capture starts but doesn't complete -> ERROR
#define CAPTURE_TIMEOUT_US 300000UL   // 300ms (tune if needed)

// =================================================
// Measurement Mode
// =================================================
enum MeasureMode {
  MODE_SINGLE_EXPOSURE,   // Single sensor: exposure time
  MODE_DUAL_CURTAIN       // Dual sensors: curtain travel & speeds (v1/v2)
};

// Default mode:
MeasureMode measure_mode = MODE_DUAL_CURTAIN;

// =================================================
// Sensors (UNO/Nano interrupts: D2=INT0, D3=INT1)
// NOTE: Renaming:
//   Sensor A -> s2 (on D2)
//   Sensor B -> s1 (on D3)
// =================================================
#define SENSOR_A 2  // s2 sensor on D2 (INT0)
#define SENSOR_B 3  // s1 sensor on D3 (INT1)
#define SENSOR_DISTANCE_MM 20.0f

// =================================================
// Frontend auto-detect via A0
// Hardware requirement:
//   - A0 has fixed pulldown to GND (recommend 100k)
//   - Sensor module provides pullup to VCC:
//       LM393  -> 100k
//       74HC14 -> 1M
// =================================================
#define FRONTEND_DETECT_PIN A0

// ADC thresholds (for Rdown=100k):
//   100k/100k -> ~512
//   1M /100k  -> ~93
#define DETECT_TH_LM393_MIN 300   // above this -> LM393
#define DETECT_TH_74HC14_MAX 200  // below this -> 74HC14

// =================================================
// Sensor frontend selection
// =================================================
enum SensorFrontend {
  FRONTEND_LM393,
  FRONTEND_74HC14
};

// If you want to force a type, set to 1.
// Otherwise, auto-detect from A0 at boot.
#define FRONTEND_FORCE_MANUAL 0
SensorFrontend sensor_frontend = FRONTEND_LM393;

// Polarity: active_low=true means LOW = shutter open/light detected
const bool active_low = true;

// Edge glitch filter (us). LM393 may chatter around threshold.
// 74HC14 is usually clean, can be 0.
volatile uint16_t MIN_EDGE_SPACING_US = 200;

// =================================================
// OLED (page buffer)
// =================================================
U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2(U8G2_R0);

// =================================================
// UI State machine
// =================================================
enum UIState { UI_READY, UI_MEASURING, UI_HOLD, UI_UNSTABLE, UI_ERROR };
UIState ui_state = UI_READY;

uint8_t error_code = 0; // E01..E05

// =================================================
// Direction (auto-detect)
// =================================================
enum TravelDirection {
  DIR_UNKNOWN = 0,
  DIR_S2_TO_S1,
  DIR_S1_TO_S2
};

volatile TravelDirection last_dir = DIR_UNKNOWN;

// =================================================
// Standard shutter speeds (PROGMEM)
// =================================================
const uint16_t shutter_denoms[] PROGMEM = {
  1000,500,400,250,200,125,100,60,50,30,
  25,15,10,8,5,4,2,1
};

const char shutter_labels[][8] PROGMEM = {
  "1/1000","1/500","1/400","1/250","1/200",
  "1/125","1/100","1/60","1/50","1/30",
  "1/25","1/15","1/10","1/8","1/5",
  "1/4","1/2","1\""
};

#define SHUTTER_COUNT (sizeof(shutter_denoms) / sizeof(uint16_t))

// =================================================
// Global display buffers
// =================================================
char g_label[16];
char g_buf[32];
char g_tmp[20];

// =================================================
// Edge capture (per sensor)
// =================================================
struct EdgeTimes {
  volatile unsigned long t_open;
  volatile unsigned long t_close;
  volatile bool opened;
};

// Rename: sA -> s2, sB -> s1
volatile EdgeTimes s2 = {0, 0, false};
volatile EdgeTimes s1 = {0, 0, false};

volatile bool new_result = false;
volatile bool capture_done = false;

// Capture start tracking (for timeout)
volatile bool capture_started = false;
volatile unsigned long capture_start_us = 0;

// Edge spacing filter state
volatile unsigned long last_edge_s2_us = 0;
volatile unsigned long last_edge_s1_us = 0;

// =================================================
// History buffers (per shot)
// NOTE: Keeping expA_hist/expB_hist naming to reduce churn,
// but conceptually expA=exp_s2, expB=exp_s1
// =================================================
unsigned long expA_hist[STABLE_COUNT]; // s2 exposure
unsigned long expB_hist[STABLE_COUNT]; // s1 exposure

unsigned long travel1_hist[STABLE_COUNT]; // abs(delta open)
unsigned long travel2_hist[STABLE_COUNT]; // abs(delta close)

int16_t errA_hist[STABLE_COUNT]; // s2 error
int16_t errB_hist[STABLE_COUNT]; // s1 error

uint8_t hist_index = 0;
uint8_t hist_filled = 0;

// Stability metrics (all are CV%)
// - Single mode: stability_percent == cv_exp_s2_percent
// - Dual mode  : stability_percent == max(cv_v1_percent, cv_v2_percent)
float stability_percent = 0.0f;
float cv_exp_s2_percent = 0.0f;
float cv_v1_percent = 0.0f;
float cv_v2_percent = 0.0f;

// =================================================
// Frontend detect (A0)
// =================================================
static uint16_t readA0Avg(uint8_t n = 16) {
  unsigned long sum = 0;
  for (uint8_t i = 0; i < n; i++) {
    sum += analogRead(FRONTEND_DETECT_PIN);
    delayMicroseconds(200);
  }
  return (uint16_t)(sum / n);
}

static SensorFrontend detectFrontendFromA0() {
  pinMode(FRONTEND_DETECT_PIN, INPUT);
  delay(5); // settle
  uint16_t adc = readA0Avg(16);

  if (adc >= DETECT_TH_LM393_MIN) return FRONTEND_LM393;
  if (adc <= DETECT_TH_74HC14_MAX) return FRONTEND_74HC14;

  // In-between: default to LM393 (safer: pullups + filtering)
  return FRONTEND_LM393;
}

// =================================================
// Configure sensor pins & glitch filter based on frontend type
// =================================================
void configureSensorFrontend() {
  if (sensor_frontend == FRONTEND_LM393) {
    pinMode(SENSOR_A, INPUT_PULLUP);
    pinMode(SENSOR_B, INPUT_PULLUP);
    MIN_EDGE_SPACING_US = 200;   // tune 100~500us if needed
  } else {
    pinMode(SENSOR_A, INPUT);
    pinMode(SENSOR_B, INPUT);
    MIN_EDGE_SPACING_US = 0;     // usually clean
  }
}

// =================================================
// Shutter speed mapping
// Returns label, writes error_permille; if not matched, error_permille=0x7FFF
// =================================================
const char* shutterLabelFromUs(unsigned long us, char* buf, int16_t* out_err_permille) {
  unsigned long best_err = 0xFFFFFFFF;
  uint8_t best = 0;

  for (uint8_t i = 0; i < SHUTTER_COUNT; i++) {
    uint16_t d = pgm_read_word(&shutter_denoms[i]);
    unsigned long ideal = 1000000UL / d;
    unsigned long err = (us > ideal) ? (us - ideal) : (ideal - us);
    if (err < best_err) {
      best_err = err;
      best = i;
    }
  }

  uint16_t denom = pgm_read_word(&shutter_denoms[best]);
  unsigned long ideal = 1000000UL / denom;

  if (best_err * 1000UL / ideal <= MATCH_THRESHOLD_PERMILLE) {
    long diff = (long)us - (long)ideal;
    long permille = diff * 1000L / (long)ideal;
    if (permille > 32767) permille = 32767;
    if (permille < -32768) permille = -32768;
    *out_err_permille = (int16_t)permille;
    strcpy_P(buf, shutter_labels[best]);
  } else {
    *out_err_permille = 0x7FFF;
    if (us < 1000000UL) sprintf(buf, "~1/%lu", 1000000UL / us);
    else                sprintf(buf, "~%lu\"", us / 1000000UL);
  }

  return buf;
}

// =================================================
// UI helpers
// =================================================
void displayCentered(const char* msg) {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_7x13B_tr);
    int w = u8g2.getStrWidth(msg);
    u8g2.drawStr((128 - w) / 2, 32, msg);
  } while (u8g2.nextPage());
}

void displayModeSplash() {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_7x13B_tr);
    u8g2.drawStr(LEFT_MARGIN, 18, "MODE");

    u8g2.setFont(u8g2_font_6x12_tr);
    const char* m = (measure_mode == MODE_DUAL_CURTAIN) ? "DUAL (Curtain)" : "SINGLE (Expo)";
    int w = u8g2.getStrWidth(m);
    u8g2.drawStr((128 - w) / 2, 36, m);

    u8g2.setFont(u8g2_font_6x10_tr);
    const char* f = (sensor_frontend == FRONTEND_LM393) ? "LM393" : "74HC14";
    sprintf(g_buf, "FE:%s", f);
    u8g2.drawStr(LEFT_MARGIN, 62, g_buf);
  } while (u8g2.nextPage());
}

void displayErrorCode(uint8_t code) {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_7x13B_tr);
    u8g2.drawStr(LEFT_MARGIN, 22, "ERROR");

    u8g2.setFont(u8g2_font_6x12_tr);
    sprintf(g_buf, "E%02u", code);
    int w = u8g2.getStrWidth(g_buf);
    u8g2.drawStr((128 - w) / 2, 44, g_buf);

    u8g2.setFont(u8g2_font_6x10_tr);
    if (code == 1) u8g2.drawStr(LEFT_MARGIN, 62, "s2 missing OPEN");
    else if (code == 2) u8g2.drawStr(LEFT_MARGIN, 62, "s2 missing CLOSE");
    else if (code == 3) u8g2.drawStr(LEFT_MARGIN, 62, "s1 missing OPEN");
    else if (code == 4) u8g2.drawStr(LEFT_MARGIN, 62, "s1 missing CLOSE");
    else if (code == 5) u8g2.drawStr(LEFT_MARGIN, 62, "Timing order invalid");
  } while (u8g2.nextPage());
}

// =================================================
// Interrupt control
// =================================================
void disableLightISR() {
  detachInterrupt(digitalPinToInterrupt(SENSOR_A));
  detachInterrupt(digitalPinToInterrupt(SENSOR_B));
}

static inline void markCaptureStarted(unsigned long now_us) {
  if (!capture_started) {
    capture_started = true;
    capture_start_us = now_us;
  }
}

static inline bool isOpenLevel(bool level_high) {
  // level_high == digitalRead(pin)
  // active_low: OPEN when LOW, else OPEN when HIGH
  return active_low ? (!level_high) : (level_high);
}

// ISR s2/s1: CHANGE + read level (OPEN/CLOSE based on active_low)
// SENSOR_A -> s2
void isr_s2() {
  if (capture_done) return;

  unsigned long now = micros();
  if (MIN_EDGE_SPACING_US > 0) {
    unsigned long dt = now - last_edge_s2_us;
    if (dt < MIN_EDGE_SPACING_US) return;
    last_edge_s2_us = now;
  }

  bool level_high = digitalRead(SENSOR_A);

  if (isOpenLevel(level_high)) { // OPEN
    markCaptureStarted(now);
    s2.t_open = now;
    s2.opened = true;
  } else if (!isOpenLevel(level_high) && s2.opened) { // CLOSE
    markCaptureStarted(now);
    s2.t_close = now;
    s2.opened = false;
  }

  if (measure_mode == MODE_SINGLE_EXPOSURE) {
    if (s2.t_open && s2.t_close) {
      capture_done = true;
      new_result = true;
    }
  } else {
    if (s2.t_open && s2.t_close && s1.t_open && s1.t_close) {
      capture_done = true;
      new_result = true;
    }
  }
}

// SENSOR_B -> s1
void isr_s1() {
  if (capture_done) return;
  if (measure_mode != MODE_DUAL_CURTAIN) return;

  unsigned long now = micros();
  if (MIN_EDGE_SPACING_US > 0) {
    unsigned long dt = now - last_edge_s1_us;
    if (dt < MIN_EDGE_SPACING_US) return;
    last_edge_s1_us = now;
  }

  bool level_high = digitalRead(SENSOR_B);

  if (isOpenLevel(level_high)) { // OPEN
    markCaptureStarted(now);
    s1.t_open = now;
    s1.opened = true;
  } else if (!isOpenLevel(level_high) && s1.opened) { // CLOSE
    markCaptureStarted(now);
    s1.t_close = now;
    s1.opened = false;
  }

  if (s2.t_open && s2.t_close && s1.t_open && s1.t_close) {
    capture_done = true;
    new_result = true;
  }
}

void enableLightISR() {
  attachInterrupt(digitalPinToInterrupt(SENSOR_A), isr_s2, CHANGE);
  if (measure_mode == MODE_DUAL_CURTAIN) {
    attachInterrupt(digitalPinToInterrupt(SENSOR_B), isr_s1, CHANGE);
  }
}

// =================================================
// State transitions
// =================================================
void enterError(uint8_t code) {
  ui_state = UI_ERROR;
  error_code = code;
  disableLightISR();     // freeze until RESET
  displayErrorCode(error_code);
}

void resetMeasurement() {
  hist_index = 0;
  hist_filled = 0;

  stability_percent = 0.0f;
  cv_exp_s2_percent = 0.0f;
  cv_v1_percent = 0.0f;
  cv_v2_percent = 0.0f;

  ui_state = UI_READY;
  error_code = 0;
  last_dir = DIR_UNKNOWN;

  noInterrupts();
  s2.t_open = s2.t_close = 0; s2.opened = false;
  s1.t_open = s1.t_close = 0; s1.opened = false;
  new_result = false;
  capture_done = false;
  capture_started = false;
  capture_start_us = 0;
  last_edge_s2_us = 0;
  last_edge_s1_us = 0;
  interrupts();

  displayModeSplash();
  delay(600);
  displayCentered("READY");
  enableLightISR();
}

// =================================================
// Speed helpers
// =================================================
static float speedFromTravelUs(unsigned long travel_us) {
  if (travel_us == 0) return 0.0f;
  float D_m = SENSOR_DISTANCE_MM / 1000.0f;
  float t_s = (float)travel_us / 1000000.0f;
  return D_m / t_s;
}

// =================================================
// Compute CV% with SAMPLE variance (divide by N-1)
// Returns CV%
// =================================================
static float cv_percent_sample_from_array(const float* x, uint8_t n) {
  if (n < 2) return 100.0f;

  float sum = 0.0f;
  for (uint8_t i = 0; i < n; i++) sum += x[i];
  float mean = sum / (float)n;
  if (mean <= 0.0f) return 100.0f;

  float sse = 0.0f; // sum of squared errors
  for (uint8_t i = 0; i < n; i++) {
    float d = x[i] - mean;
    sse += d * d;
  }

  float variance = sse / (float)(n - 1); // SAMPLE variance
  if (variance < 0.0f) variance = 0.0f;

  float stddev = sqrt(variance);
  return (stddev / mean) * 100.0f;
}

// =================================================
// Stability calculation
// - SINGLE: CV of exp_s2 (sample variance)
// - DUAL  : CV of v1 and v2 (sample variance), take MAX as overall
// Uses CV% (stability_percent) for HOLD/UNSTABLE decision.
// =================================================
void calculateStability() {
  if (hist_filled < STABLE_COUNT) return;

  // Clear
  cv_exp_s2_percent = 0.0f;
  cv_v1_percent = 0.0f;
  cv_v2_percent = 0.0f;
  stability_percent = 0.0f;

  if (measure_mode == MODE_SINGLE_EXPOSURE) {
    float x[STABLE_COUNT];
    for (uint8_t i = 0; i < STABLE_COUNT; i++) x[i] = (float)expA_hist[i];

    cv_exp_s2_percent = cv_percent_sample_from_array(x, STABLE_COUNT);
    stability_percent = cv_exp_s2_percent;
  } else {
    float v1[STABLE_COUNT];
    float v2[STABLE_COUNT];
    for (uint8_t i = 0; i < STABLE_COUNT; i++) {
      v1[i] = speedFromTravelUs(travel1_hist[i]);
      v2[i] = speedFromTravelUs(travel2_hist[i]);
    }

    cv_v1_percent = cv_percent_sample_from_array(v1, STABLE_COUNT);
    cv_v2_percent = cv_percent_sample_from_array(v2, STABLE_COUNT);

    stability_percent = (cv_v1_percent > cv_v2_percent) ? cv_v1_percent : cv_v2_percent;
  }

  ui_state = (stability_percent <= STABLE_TOLERANCE_PERCENT) ? UI_HOLD : UI_UNSTABLE;
  disableLightISR(); // freeze until RESET
}

// =================================================
// Convert CV% -> "Stability score"
// score = 100 - 3*CV%  (clamped 0..100)
// =================================================
static float stabilityScoreFromCV(float cv_percent) {
  float score = 100.0f - 3.0f * cv_percent;
  if (score < 0.0f) score = 0.0f;
  if (score > 100.0f) score = 100.0f;
  return score;
}

// =================================================
// Display: 5-shot table
//   - DUAL  : each row "1/60 v1:xxx v2:xxx", last row "Stability v1:.. v2:.."
//   - SINGLE: each row "1/60 12345us +x.x%", last row "Stability: xx.x%"
// =================================================
void updateDisplayTable() {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tr);

    for (uint8_t i = 0; i < hist_filled; i++) {
      unsigned long exp_s2 = expA_hist[i];   // s2 exposure

      char label[16];
      int16_t tmp;
      shutterLabelFromUs(exp_s2, label, &tmp);

      char line[32];
      if (measure_mode == MODE_DUAL_CURTAIN) {
        float v1 = speedFromTravelUs(travel1_hist[i]);
        float v2 = speedFromTravelUs(travel2_hist[i]);

        char v1buf[8], v2buf[8];
        dtostrf(v1, 0, 2, v1buf);
        dtostrf(v2, 0, 2, v2buf);

        // Example: "1/60 v1:2.34 v2:2.31"
        snprintf(line, sizeof(line), "%s v1:%s v2:%s", label, v1buf, v2buf);
      } else {
        char err_str[10];
        if (errA_hist[i] != 0x7FFF)
          snprintf(err_str, sizeof(err_str), "%+d.%d%%", errA_hist[i] / 10, abs(errA_hist[i] % 10));
        else
          strcpy(err_str, "--");

        // Example: "1/60 16667us -0.0%"
        snprintf(line, sizeof(line), "%s %luus %s", label, exp_s2, err_str);
      }

      u8g2.drawStr(LEFT_MARGIN, 10 + i * 10, line);
    }

    // ---------- last summary line ----------
    if (hist_filled == STABLE_COUNT) {
      if (measure_mode == MODE_DUAL_CURTAIN) {
        // Dual: show per-curtain stability (score) for v1 and v2
        float score_v1 = stabilityScoreFromCV(cv_v1_percent);
        float score_v2 = stabilityScoreFromCV(cv_v2_percent);

        char s1buf[8], s2buf[8];
        dtostrf(score_v1, 0, 1, s1buf);
        dtostrf(score_v2, 0, 1, s2buf);

        // Keep it short to fit 128px
        // Example: "Stability v1:95.2 v2:93.1"
        snprintf(g_buf, sizeof(g_buf), "Stability %s%% %s%%", s1buf, s2buf);
        u8g2.drawStr(LEFT_MARGIN, 10 + STABLE_COUNT * 10, g_buf);
      } else {
        // Single: Stability score based on CV_exp_s2
        float score = stabilityScoreFromCV(cv_exp_s2_percent);
        dtostrf(score, 0, 1, g_tmp);
        snprintf(g_buf, sizeof(g_buf), "Stability: %s%%", g_tmp);
        u8g2.drawStr(LEFT_MARGIN, 10 + STABLE_COUNT * 10, g_buf);
      }
    }
  } while (u8g2.nextPage());
}

// =================================================
// Display: dual realtime (before table is full)
// =================================================
void updateDisplayDual(unsigned long exp_s2_us, int16_t err_s2,
                      unsigned long exp_s1_us, int16_t err_s1,
                      unsigned long travel1_us, unsigned long travel2_us,
                      TravelDirection dir) {
  float v1 = speedFromTravelUs(travel1_us);
  float v2 = speedFromTravelUs(travel2_us);

  u8g2.firstPage();
  do {
    for (uint8_t i = 0; i < STABLE_COUNT; i++) {
      int x = LEFT_MARGIN + i * 10;
      if (i < hist_filled) u8g2.drawBox(x, 4, 8, 8);
      else u8g2.drawFrame(x, 4, 8, 8);
    }

    // Direction tag (top-right)
    u8g2.setFont(u8g2_font_6x10_tr);
    const char* dstr = (dir == DIR_S2_TO_S1) ? "s2->s1" : (dir == DIR_S1_TO_S2) ? "s1->s2" : "----";
    int wd = u8g2.getStrWidth(dstr);
    u8g2.drawStr(128 - wd - RIGHT_MARGIN, 12, dstr);

    // Shutter label (from s2 exposure)
    u8g2.setFont(u8g2_font_7x13B_tr);
    int w = u8g2.getStrWidth(g_label);
    u8g2.drawStr((128 - w) / 2, 26, g_label);

    // s2 line
    u8g2.setFont(u8g2_font_6x12_tr);
    char e2[10];
    if (err_s2 != 0x7FFF) snprintf(e2, sizeof(e2), "%+d.%d%%", err_s2 / 10, abs(err_s2 % 10));
    else strcpy(e2, "--");
    snprintf(g_buf, sizeof(g_buf), "s2:%luus %s", exp_s2_us, e2);
    if (dir == DIR_S1_TO_S2) u8g2.drawStr(LEFT_MARGIN, 54, g_buf);
    else                    u8g2.drawStr(LEFT_MARGIN, 42, g_buf);

    // s1 line
    char e1[10];
    if (err_s1 != 0x7FFF) snprintf(e1, sizeof(e1), "%+d.%d%%", err_s1 / 10, abs(err_s1 % 10));
    else strcpy(e1, "--");
    snprintf(g_buf, sizeof(g_buf), "s1:%luus %s", exp_s1_us, e1);
    if (dir == DIR_S1_TO_S2) u8g2.drawStr(LEFT_MARGIN, 42, g_buf);
    else                     u8g2.drawStr(LEFT_MARGIN, 54, g_buf);

    // v1/v2 line
    char v1buf[10], v2buf[10];
    dtostrf(v1, 0, 2, v1buf);
    dtostrf(v2, 0, 2, v2buf);
    snprintf(g_buf, sizeof(g_buf), "v1:%s v2:%s m/s", v1buf, v2buf);
    u8g2.drawStr(LEFT_MARGIN, 64, g_buf);

  } while (u8g2.nextPage());
}

// =================================================
// Reset button handler (short press = reset, long press = toggle mode)
// =================================================
bool handleResetButton() {
  static bool last_level = true;           // true = released (pull-up)
  static unsigned long t_last_change = 0;
  static unsigned long t_pressed = 0;
  static bool long_handled = false;

  bool level = digitalRead(BTN_RESET);     // HIGH released, LOW pressed
  unsigned long now_ms = millis();

  if (level != last_level) {
    if (now_ms - t_last_change < BTN_DEBOUNCE_MS) {
      return true;
    }
    t_last_change = now_ms;
    last_level = level;

    if (!level) {
      t_pressed = now_ms;
      long_handled = false;
    } else {
      if (!long_handled) {
        resetMeasurement();
      }
    }
    return true;
  }

  if (!level && !long_handled) {
    if (now_ms - t_pressed >= BTN_LONGPRESS_MS) {
      long_handled = true;

      measure_mode = (measure_mode == MODE_DUAL_CURTAIN) ? MODE_SINGLE_EXPOSURE : MODE_DUAL_CURTAIN;

      disableLightISR();
      displayModeSplash();
      delay(600);

      resetMeasurement();
      return true;
    }
  }

  return false;
}

// =================================================
// SETUP
// =================================================
void setup() {
#if FRONTEND_FORCE_MANUAL
  // keep sensor_frontend as configured above
#else
  sensor_frontend = detectFrontendFromA0();
#endif

  configureSensorFrontend();
  pinMode(BTN_RESET, INPUT_PULLUP);

  u8g2.begin();

  // Splash screen
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_7x13B_tr);
    int len = u8g2.getStrWidth(NAME);
    u8g2.drawStr((128 - len) / 2 + LEFT_MARGIN, 24, NAME);

    u8g2.setFont(u8g2_font_6x12_tr);
    int w = u8g2.getStrWidth(SLOGAN);
    u8g2.drawStr((128 - w) / 2, 40, SLOGAN);

    w = u8g2.getStrWidth(VERSION_STR);
    u8g2.drawStr(128 - w - RIGHT_MARGIN - 2, 62, VERSION_STR);
  } while (u8g2.nextPage());

  delay(2000);
  displayModeSplash();
  delay(1500);
  displayCentered("READY");
  enableLightISR();
}

// =================================================
// LOOP
// =================================================
void loop() {
  handleResetButton();

  if (ui_state == UI_ERROR) {
    displayErrorCode(error_code);
    return;
  }
  if (ui_state == UI_HOLD || ui_state == UI_UNSTABLE) {
    updateDisplayTable();
    return;
  }

  // Dual mode timeout: capture started but not completed
  if (measure_mode == MODE_DUAL_CURTAIN && capture_started && !new_result) {
    unsigned long now_us = micros();
    if ((now_us - capture_start_us) > CAPTURE_TIMEOUT_US) {
      uint8_t code = 0;
      noInterrupts();
      bool s2_open_ok  = (s2.t_open  != 0);
      bool s2_close_ok = (s2.t_close != 0);
      bool s1_open_ok  = (s1.t_open  != 0);
      bool s1_close_ok = (s1.t_close != 0);
      interrupts();

      if (!s2_open_ok) code = 1;
      else if (!s2_close_ok) code = 2;
      else if (!s1_open_ok) code = 3;
      else if (!s1_close_ok) code = 4;
      else code = 5;

      enterError(code);
      return;
    }
  }

  if (!new_result) return;

  // Copy timestamps atomically
  unsigned long s2_open, s2_close, s1_open, s1_close;
  noInterrupts();
  s2_open  = s2.t_open;   s2_close  = s2.t_close;
  s1_open  = s1.t_open;   s1_close  = s1.t_close;

  new_result = false;
  capture_done = false;
  capture_started = false;
  capture_start_us = 0;

  // Clear for next shot
  s2.t_open = s2.t_close = 0; s2.opened = false;
  s1.t_open = s1.t_close = 0; s1.opened = false;
  interrupts();

  // Validate s2
  if (s2_close <= s2_open) { enterError(5); return; }
  unsigned long exp_s2 = s2_close - s2_open;
  if (exp_s2 < MIN_VALID_EXPOSURE || exp_s2 > MAX_VALID_EXPOSURE) return;

  unsigned long exp_s1 = 0;
  unsigned long travel1 = 0, travel2 = 0;
  int16_t err_s2 = 0x7FFF, err_s1 = 0x7FFF;

  shutterLabelFromUs(exp_s2, g_label, &err_s2);

  TravelDirection dir = DIR_UNKNOWN;

  if (measure_mode == MODE_DUAL_CURTAIN) {
    // Validate s1
    if (s1_close <= s1_open) { enterError(5); return; }

    exp_s1 = s1_close - s1_open;
    if (exp_s1 < MIN_VALID_EXPOSURE || exp_s1 > MAX_VALID_EXPOSURE) return;

    // --------- AUTO direction detect ----------
    long dOpen  = (long)s1_open  - (long)s2_open;
    long dClose = (long)s1_close - (long)s2_close;

    if (dOpen == 0) { enterError(5); return; }

    // Determine direction by OPEN edge order
    dir = (dOpen > 0) ? DIR_S2_TO_S1 : DIR_S1_TO_S2;

    // Travel times are absolute (direction-insensitive)
    travel1 = (unsigned long)labs(dOpen);
    travel2 = (unsigned long)labs(dClose);

    // Consistency check: CLOSE edges must keep same order as OPEN edges
    if ((dClose == 0) || ((dOpen > 0) != (dClose > 0))) {
      enterError(5);
      return;
    }

    // Error of s1 exposure relative to standard shutter speeds (optional)
    shutterLabelFromUs(exp_s1, g_tmp, &err_s1);
  }

  // Save direction (for display)
  last_dir = dir;

  // Store history (s2->A arrays, s1->B arrays)
  expA_hist[hist_index]  = exp_s2;
  errA_hist[hist_index]  = err_s2;

  expB_hist[hist_index]  = exp_s1;
  errB_hist[hist_index]  = err_s1;

  travel1_hist[hist_index] = travel1;
  travel2_hist[hist_index] = travel2;

  hist_index++;
  if (hist_index >= STABLE_COUNT) hist_index = 0;
  if (hist_filled < STABLE_COUNT) hist_filled++;

  // After 5 samples -> HOLD or UNSTABLE (both freeze until RESET)
  if (hist_filled == STABLE_COUNT) {
    calculateStability();
    updateDisplayTable();
    return;
  }

  // Real-time display
  if (measure_mode == MODE_DUAL_CURTAIN) {
    updateDisplayDual(exp_s2, err_s2, exp_s1, err_s1, travel1, travel2, dir);
  } else {
    updateDisplayTable();
  }
}
