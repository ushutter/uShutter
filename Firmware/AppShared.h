#pragma once
#include <Arduino.h>

enum MeasureMode {
  MODE_SINGLE_EXPOSURE,
  MODE_DUAL_CURTAIN
};

enum SensorFrontend {
  FRONTEND_FE1_DUAL,      // FE1: 1k pulldown (Dual)
  FRONTEND_FE2_SINGLE,    // FE2: 2.2k pulldown (Single)
  FRONTEND_FE3_RESERVED,  // FE3: 4.7k pulldown (reserved)
  FRONTEND_FE4_RESERVED   // FE4: 10k pulldown (reserved)
};

enum TravelDirection {
  DIR_UNKNOWN = 0,
  DIR_S2_TO_S1,
  DIR_S1_TO_S2
};

#define SENSOR_A 2
#define SENSOR_B 3
#define SENSOR_DISTANCE_MM 20.0f
#define STABLE_COUNT 5
#define LEFT_MARGIN 3
#define RIGHT_MARGIN 3

struct EdgeTimes {
  volatile unsigned long t_open;
  volatile unsigned long t_close;
  volatile bool opened;
};
