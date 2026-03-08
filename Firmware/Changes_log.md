# High-level change log
## Compare: `uShutter_v0_9_4` → `uShutter_v0_9_5`

Date: 2026-03-08

## Executive summary
- The project moved from a **single-file sketch** (`uShutter.ino`) to a **modular multi-file layout**.
- This is primarily a **refactor for maintainability/structure**, not a feature-heavy rewrite.
- Core responsibilities are now separated into app flow, capture/ISR, display/UI, and shared types.

## Top-level structural changes


### Added (new layout)
- `app.cpp` / `app.h` (main orchestration/state machine)
- `Capture.cpp` / `Capture.h` (interrupt + edge capture responsibilities)
- `Display.cpp` / `Display.h` (OLED/UI rendering responsibilities)
- `AppShared.h` (shared enums/types/constants)
- `README_refactor.md` (refactor notes and guardrails)

## Architecture-level differences
1. **Monolith → Modules**
   - Old: one large `.ino` contained logic, UI, capture, and shared declarations.
   - New: domain responsibilities split into dedicated translation units.

2. **Cleaner separation of concerns**
   - ISR/capture handling is isolated from display rendering.
   - App/state flow lives in `app.cpp`.
   - Shared declarations centralized in `AppShared.h`.

3. **Arduino preprocessing risk reduced**
   - More logic moved from `.ino` into `.cpp/.h`, reducing prototype-generation quirks from Arduino’s auto-preprocessor.

## Likely impact
- **Positive:** easier future edits, lower merge friction, clearer ownership by file.
- **Positive:** easier debugging since UI/capture/app are separated.
- **Watchouts:** cross-module `extern`/shared-state coupling should be monitored and gradually reduced.

## Notes
- `README_refactor.md` in the new folder documents intent, file responsibilities, and safe edit rules.


## Recommendation
- Treat this as a **refactor milestone** version.
- For next steps, consider:
  - replacing broad shared `extern` usage with a narrow shared context,
  - optional frontend module extraction,
  - a small compile + behavior smoke checklist per edit cycle.
