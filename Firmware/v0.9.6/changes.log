uShutter change summary: uShutter_v0_9_5_refactor_wip -> uShutter_v0_9_6
Generated: Fri Feb 27 22:52:08 PST 2026

== Files only in each folder ==
Only in v0_9_5_refactor_wip:
uShutter_v0_9_5_refactor_wip.ino

Only in v0_9_6:
uShutter_v0_9_6.ino

== High-level functional differences ==
- BTN_RESET pin changed to D4 in v0_9_6 (from D6 in refactor_wip at latest baseline).
- BTN_MODE pin remains D5, but behavior changed in v0_9_6:
  * short click: toggle sensor s1/s2 (only when FE=Dual and mode=SINGLE)
  * long hold: toggle mode DUAL<->SINGLE
- BTN_RESET behavior in v0_9_6 is single-click reset only.
- Startup splash flow remains 3-stage with 2.0s each splash.

== Key code-level diff snippets (app.cpp / Display.cpp / headers) ==
--- /Users/leonyao/Documents/uShutter/uShutter_v0_9_5_refactor_wip/app.cpp	2026-02-27 22:16:06
+++ /Users/leonyao/Documents/uShutter/uShutter_v0_9_6/app.cpp	2026-02-27 22:44:57
@@ -11,7 +11,7 @@
 // =================================================
 // Version
 // =================================================
-#define VERSION_STR "ver 0.9.5"
+#define VERSION_STR "ver 0.9.6"
 #define NAME "uShutter"
 #define SLOGAN ""
 
@@ -21,7 +21,8 @@
 #define MIN_VALID_EXPOSURE 1000     // Minimum valid exposure time (us)
 #define MAX_VALID_EXPOSURE 1500000  // Maximum valid exposure time (us)
 
-#define BTN_RESET 5            // Reset button pin (active-low)
+#define BTN_RESET 4            // Reset button pin (active-low)
+#define BTN_MODE 5
 #define BTN_DEBOUNCE_MS 50     // Button debounce time (ms)
 #define BTN_LONGPRESS_MS 1200  // Long press threshold (ms)
 
@@ -133,8 +134,9 @@
 volatile EdgeTimes s1 = { 0, 0, false };
 
 // Helpers for SINGLE mode sensor routing
+uint8_t single_mode_sensor = SINGLE_MODE_SENSOR;  // runtime: 1->s1, 2->s2
 bool singleUsesS1() {
-  return (SINGLE_MODE_SENSOR == 1);
+  return (single_mode_sensor == 1);
 }
 
 volatile bool new_result = false;
@@ -483,40 +485,71 @@
 bool handleResetButton() {
   static bool last_level = true;  // true = released (pull-up)
   static unsigned long t_last_change = 0;
-  static unsigned long t_pressed = 0;
-  static bool long_handled = false;
 
   bool level = digitalRead(BTN_RESET);  // HIGH released, LOW pressed
   unsigned long now_ms = millis();
 
   if (level != last_level) {
-    if (now_ms - t_last_change < BTN_DEBOUNCE_MS) {
-      return true;
+    if (now_ms - t_last_change < BTN_DEBOUNCE_MS) return true;
+    t_last_change = now_ms;
+    last_level = level;
+
+    // single click on release => reset
+    if (level) {
+      resetMeasurement();
     }
+    return true;
+  }
+  return false;
+}
+
+// BTN_MODE:
+// - Single click: toggle mode (DUAL<->SINGLE)
+// - Double click (only FE Dual + SINGLE mode): toggle single sensor s1/s2
+bool handleModeButton() {
+  static bool last_level = true;
+  static unsigned long t_last_change = 0;
+  static unsigned long t_pressed = 0;
+  static bool long_handled = false;
+
+  bool level = digitalRead(BTN_MODE);
+  unsigned long now_ms = millis();
+
+  bool sensor_toggle_enabled = (sensor_frontend == FRONTEND_FE1_DUAL && measure_mode == MODE_SINGLE_EXPOSURE);
+
+  if (level != last_level) {
+    if (now_ms - t_last_change < BTN_DEBOUNCE_MS) return true;
     t_last_change = now_ms;
     last_level = level;
 
     if (!level) {
+      // pressed
       t_pressed = now_ms;
       long_handled = false;
     } else {
+      // released
       if (!long_handled) {
-        resetMeasurement();
+        // short click: toggle sensor s1/s2 (only when FE=Dual and mode=SINGLE)
+        if (sensor_toggle_enabled) {
+          single_mode_sensor = (single_mode_sensor == 1) ? 2 : 1;
+          disableLightISR();
+          displayModeSplash();
+          delay(600);
+          resetMeasurement();
+        }
       }
     }
     return true;
   }
 
+  // long press: toggle mode
   if (!level && !long_handled) {
     if (now_ms - t_pressed >= BTN_LONGPRESS_MS) {
       long_handled = true;
-
       measure_mode = (measure_mode == MODE_DUAL_CURTAIN) ? MODE_SINGLE_EXPOSURE : MODE_DUAL_CURTAIN;
-
       disableLightISR();
       displayModeSplash();
       delay(600);
-
       resetMeasurement();
       return true;
     }
@@ -525,6 +558,7 @@
   return false;
 }
 
+
 // =================================================
 // SETUP
 // =================================================
@@ -540,6 +574,7 @@
 
   configureSensorFrontend();
   pinMode(BTN_RESET, INPUT_PULLUP);
+  pinMode(BTN_MODE, INPUT_PULLUP);
 
   u8g2.begin();
 
@@ -570,6 +605,7 @@
 // =================================================
 void loop() {
   handleResetButton();
+  handleModeButton();
 
   if (ui_state == UI_ERROR) {
     displayErrorCode(error_code);


