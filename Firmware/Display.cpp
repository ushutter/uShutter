// Display.cpp
// OLED/UI rendering functions.

#include <Arduino.h>
#include <U8g2lib.h>
#include "Display.h"
#include "AppShared.h"


extern U8G2_SH1106_128X64_NONAME_1_HW_I2C u8g2;
extern char g_buf[32];

extern MeasureMode measure_mode;
extern SensorFrontend sensor_frontend;
extern bool singleUsesS1();

void displayCentered(const char* msg) {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_7x13B_tr);
    int w = u8g2.getStrWidth(msg);
    u8g2.drawStr((128 - w) / 2, 32, msg);
  } while (u8g2.nextPage());
}

void displayFrontendSplash() {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(LEFT_MARGIN, 22, "Front End");

    u8g2.setFont(u8g2_font_7x13B_tr);
    const char* fe = "Reserved";
    if (sensor_frontend == FRONTEND_FE1_DUAL) fe = "Dual Sensors";
    else if (sensor_frontend == FRONTEND_FE2_SINGLE) fe = "Single Sensor";

    snprintf(g_buf, sizeof(g_buf), "%s", fe);
    int w = u8g2.getStrWidth(g_buf);
    u8g2.drawStr((128 - w) / 2, 44, g_buf);
  } while (u8g2.nextPage());
}

void displayModeSplash() {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x12_tr);
    u8g2.drawStr(LEFT_MARGIN, 18, "MODE");

    u8g2.setFont(u8g2_font_7x13B_tr);
    const char* m = (measure_mode == MODE_DUAL_CURTAIN) ? "Curtain Mode" : "Leaf Mode";
    int w = u8g2.getStrWidth(m);
    u8g2.drawStr((128 - w) / 2, 36, m);

    // In Leaf mode with dual-sensor frontend, show selected sensor at bottom-right
    if (measure_mode == MODE_SINGLE_EXPOSURE && sensor_frontend == FRONTEND_FE1_DUAL) {
      u8g2.setFont(u8g2_font_6x10_tr);
      snprintf(g_buf, sizeof(g_buf), "Sensor:%s", singleUsesS1() ? "R" : "L");
      int ws = u8g2.getStrWidth(g_buf);
      u8g2.drawStr(128 - ws - RIGHT_MARGIN, 62, g_buf);
    }

  } while (u8g2.nextPage());
}

void displayErrorCode(unsigned char code) {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_7x13B_tr);
    u8g2.drawStr(LEFT_MARGIN, 22, "ERROR");

    u8g2.setFont(u8g2_font_6x12_tr);
    sprintf(g_buf, "E%02u", code);
    int w = u8g2.getStrWidth(g_buf);
    u8g2.drawStr((128 - w) / 2, 44, g_buf);

    u8g2.setFont(u8g2_font_6x10_tr);
    if (code == 1) u8g2.drawStr(LEFT_MARGIN, 62, "R no OPEN");
    else if (code == 2) u8g2.drawStr(LEFT_MARGIN, 62, "R no CLOSE");
    else if (code == 3) u8g2.drawStr(LEFT_MARGIN, 62, "L no OPEN");
    else if (code == 4) u8g2.drawStr(LEFT_MARGIN, 62, "L no CLOSE");
    else if (code == 5) u8g2.drawStr(LEFT_MARGIN, 62, "order invalid");
  } while (u8g2.nextPage());
}

// externs for table/single views
extern unsigned long expA_hist[];
extern short errA_hist[];
extern unsigned long travel1_hist[];
extern unsigned long travel2_hist[];
extern unsigned char hist_filled;
extern float cv_v1_percent, cv_v2_percent, cv_exp_s2_percent;
extern char g_label[16];
extern char g_tmp[20];

extern const char* shutterLabelFromUs(unsigned long us, char* buf, int16_t* out_err_permille);
extern float speedFromTravelUs(unsigned long travel_us);
extern float stabilityScoreFromCV(float cv_percent);


void updateDisplayTable() {
  u8g2.firstPage();
  do {
    u8g2.setFont(u8g2_font_6x10_tr);

    for (uint8_t i = 0; i < hist_filled; i++) {
      unsigned long exp_s2 = expA_hist[i];  // s2 exposure

      char label[16];
      short tmp;
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



void updateDisplaySingle(unsigned long exp_us, int16_t err_permille, bool use_s1) {
  u8g2.firstPage();
  do {
    // progress bar
    for (uint8_t i = 0; i < STABLE_COUNT; i++) {
      int x = LEFT_MARGIN + i * 10;
      if (i < hist_filled) u8g2.drawBox(x, 4, 8, 8);
      else u8g2.drawFrame(x, 4, 8, 8);
    }

    // sensor tag (top-right)
    u8g2.setFont(u8g2_font_6x10_tr);
    const char* tag = use_s1 ? "R" : "L";
    int wt = u8g2.getStrWidth(tag);
    u8g2.drawStr(128 - wt - RIGHT_MARGIN, 12, tag);

    // shutter label
    u8g2.setFont(u8g2_font_7x13B_tr);
    int w = u8g2.getStrWidth(g_label);
    u8g2.drawStr((128 - w) / 2, 26, g_label);

    // measured time + deviation
    u8g2.setFont(u8g2_font_6x12_tr);
    char ebuf[10];
    if (err_permille != 0x7FFF) snprintf(ebuf, sizeof(ebuf), "%+d.%d%%", err_permille / 10, abs(err_permille % 10));
    else strcpy(ebuf, "--");
    snprintf(g_buf, sizeof(g_buf), "%s:%luus %s", tag, exp_us, ebuf);
    u8g2.drawStr(LEFT_MARGIN, 54, g_buf);

  } while (u8g2.nextPage());
}



extern int last_dir;

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
    const char* dstr = (dir == DIR_S2_TO_S1) ? "L->R" : (dir == DIR_S1_TO_S2) ? "R->L"
                                                                                : "----";
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
    snprintf(g_buf, sizeof(g_buf), "L:%luus %s", exp_s2_us, e2);
    if (dir == DIR_S1_TO_S2) u8g2.drawStr(LEFT_MARGIN, 54, g_buf);
    else u8g2.drawStr(LEFT_MARGIN, 42, g_buf);

    // s1 line
    char e1[10];
    if (err_s1 != 0x7FFF) snprintf(e1, sizeof(e1), "%+d.%d%%", err_s1 / 10, abs(err_s1 % 10));
    else strcpy(e1, "--");
    snprintf(g_buf, sizeof(g_buf), "R:%luus %s", exp_s1_us, e1);
    if (dir == DIR_S1_TO_S2) u8g2.drawStr(LEFT_MARGIN, 42, g_buf);
    else u8g2.drawStr(LEFT_MARGIN, 54, g_buf);

    // v1/v2 line
    char v1buf[10], v2buf[10];
    dtostrf(v1, 0, 2, v1buf);
    dtostrf(v2, 0, 2, v2buf);
    snprintf(g_buf, sizeof(g_buf), "v1:%s v2:%s m/s", v1buf, v2buf);
    u8g2.drawStr(LEFT_MARGIN, 64, g_buf);

  } while (u8g2.nextPage());
}


