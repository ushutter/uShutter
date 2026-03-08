#include <Arduino.h>
#include "AppShared.h"
#pragma once

void displayCentered(const char* msg);
void displayFrontendSplash();
void displayModeSplash();
void displayErrorCode(unsigned char code);
void updateDisplayTable();
void updateDisplaySingle(unsigned long exp_us, int16_t err_permille, bool use_s1);
void updateDisplayDual(unsigned long exp_s2_us, int16_t err_s2,
                       unsigned long exp_s1_us, int16_t err_s1,
                       unsigned long travel1_us, unsigned long travel2_us,
                       TravelDirection dir);
