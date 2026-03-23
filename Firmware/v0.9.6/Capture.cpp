// Capture.cpp
// Interrupt/capture edge handling functions.

#include <Arduino.h>
#include "Capture.h"
#include "AppShared.h"

extern MeasureMode measure_mode;
extern volatile uint16_t MIN_EDGE_SPACING_US;
extern volatile bool capture_done;
extern volatile bool new_result;
extern volatile bool capture_started;
extern volatile unsigned long capture_start_us;
extern volatile unsigned long last_edge_s2_us;
extern volatile unsigned long last_edge_s1_us;
extern bool idle_high_s2;
extern bool idle_high_s1;
extern volatile EdgeTimes s2;
extern volatile EdgeTimes s1;

extern bool singleUsesS1();

static inline void markCaptureStarted(unsigned long now_us) {
  if (!capture_started) {
    capture_started = true;
    capture_start_us = now_us;
  }
}


void disableLightISR() {
  detachInterrupt(digitalPinToInterrupt(SENSOR_A));
  detachInterrupt(digitalPinToInterrupt(SENSOR_B));
}



void isr_s2() {
  if (capture_done) return;

  // In SINGLE mode, ignore s2 if SINGLE_MODE_SENSOR selects s1
  if (measure_mode == MODE_SINGLE_EXPOSURE && singleUsesS1()) return;

  unsigned long now = micros();

  // Edge spacing filter (mainly for LM393 chatter)
  if (MIN_EDGE_SPACING_US > 0) {
    unsigned long dt = now - last_edge_s2_us;
    if (dt < MIN_EDGE_SPACING_US) return;
    last_edge_s2_us = now;
  }

  bool level_high = digitalRead(SENSOR_A);

  // v1.0.3 polarity fix:
  // We treat "OPEN" as the first transition away from idle,
  // and "CLOSE" as the transition back to idle.
  if (!s2.opened && s2.t_open == 0) {
    if (level_high == idle_high_s2) return; // still idle -> ignore
    markCaptureStarted(now);
    s2.t_open = now;
    s2.opened = true;
  } else if (s2.opened && s2.t_close == 0) {
    if (level_high != idle_high_s2) return; // not back to idle yet
    markCaptureStarted(now);
    s2.t_close = now;
    s2.opened = false;
  } else {
    return; // ignore extra edges
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





void isr_s1() {
  if (capture_done) return;
  // In SINGLE mode, only handle s1 if user selected s1
  if (measure_mode == MODE_SINGLE_EXPOSURE && !singleUsesS1()) return;
  // In DUAL mode, s1 is always active
  if (measure_mode != MODE_DUAL_CURTAIN && measure_mode != MODE_SINGLE_EXPOSURE) return;

  unsigned long now = micros();
  if (MIN_EDGE_SPACING_US > 0) {
    unsigned long dt = now - last_edge_s1_us;
    if (dt < MIN_EDGE_SPACING_US) return;
    last_edge_s1_us = now;
  }

  bool level_high = digitalRead(SENSOR_B);

  // Polarity-agnostic capture:
  // - First transition away from idle is treated as OPEN.
  // - First transition back to idle is treated as CLOSE.
  // This avoids missing CLOSE when the signal polarity differs across PCB revisions.
  if (s1.t_open == 0) {
    if (level_high == idle_high_s1) return; // still idle -> ignore
    markCaptureStarted(now);
    s1.t_open = now;
    s1.opened = true;
  } else if (s1.opened && s1.t_close == 0) {
    if (level_high != idle_high_s1) return; // not back to idle yet
    markCaptureStarted(now);
    s1.t_close = now;
    s1.opened = false;
  } else {
    return; // ignore extra edges
  }

  if (measure_mode == MODE_SINGLE_EXPOSURE) {
    // SINGLE: selected sensor only (s1 here)
    if (s1.t_open && s1.t_close) {
      capture_done = true;
      new_result = true;
    }
  } else {
    // DUAL: need both sensors complete
    if (s2.t_open && s2.t_close && s1.t_open && s1.t_close) {
      capture_done = true;
      new_result = true;
    }
  }
}




void enableLightISR() {
  // Always start clean
  disableLightISR();

  if (measure_mode == MODE_SINGLE_EXPOSURE) {
    // Attach only the selected sensor
    if (singleUsesS1()) {
      attachInterrupt(digitalPinToInterrupt(SENSOR_B), isr_s1, CHANGE);
    } else {
      attachInterrupt(digitalPinToInterrupt(SENSOR_A), isr_s2, CHANGE);
    }
  } else {
    // Dual mode: both sensors
    attachInterrupt(digitalPinToInterrupt(SENSOR_A), isr_s2, CHANGE);
    attachInterrupt(digitalPinToInterrupt(SENSOR_B), isr_s1, CHANGE);
  }
}


