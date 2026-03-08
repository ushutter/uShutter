# uShutter refactor notes (WIP)

This project has been split from a monolithic `.ino` into multiple translation units to improve maintainability and avoid Arduino preprocessor issues.

## File responsibilities

- `uShutter.ino`
  - Minimal Arduino entry placeholder.
  - Keep this tiny; do not put logic here.

- `app.cpp`
  - Main orchestration/state machine.
  - Setup/loop flow.
  - Measurement logic, stability calculation, button handling.
  - Calls into display and capture modules.

- `Display.h` / `Display.cpp`
  - OLED/UI rendering only.
  - Splash, error screen, realtime views, table view.
  - Should avoid business logic changes where possible.

- `Capture.h` / `Capture.cpp`
  - Interrupt attach/detach.
  - ISR edge capture and capture-complete signaling.
  - No UI rendering.

- `AppShared.h`
  - Shared enums/types/constants used across modules.
  - Single source for common declarations (`MeasureMode`, `SensorFrontend`, `TravelDirection`, `EdgeTimes`, sensor pin constants, etc.).

## Why this structure

Arduino `.ino` files are auto-preprocessed (prototype generation), which can break during aggressive function splitting. Keeping logic in `.cpp/.h` avoids those fragile transforms.

## Safe edit rules

1. **Prefer editing `.cpp/.h`, not `.ino`.**
2. **After each micro-change, compile immediately.**
3. **Keep cross-module contracts explicit:**
   - Put shared enums/structs in `AppShared.h`.
   - Keep module APIs in `Display.h` / `Capture.h`.
4. **Avoid duplicate type definitions** across modules.
5. **If introducing new shared state**, declare in `app.cpp`, reference via `extern` in module `.cpp` files.

## Compile check

Reference compile command:

```bash
arduino-cli compile --fqbn arduino:avr:nano /Users/leonyao/Documents/uShutter_v0_9_5_refactor_wip
```

Latest known good result (at time of writing):
- Flash: ~26,346 bytes (85%)
- RAM: ~1,396 bytes (68%)

## Next recommended refactor steps

1. Replace remaining broad `extern` usage with a thinner shared context struct (optional).
2. Move frontend detection helpers into a dedicated `Frontend.cpp/.h` (optional).
3. Add a small host-side smoke checklist (compile + key behavior sanity list).
