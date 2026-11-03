# uShutter Nano User Guide

| Item | Value |
| --- | --- |
| Product | uShutter |
| Edition | Nano |
| Guide version | 1.0.0 |
| Firmware shown at startup | `ver 1.0.0` |
| Controller | Arduino Nano / ATmega328P |
| Display | 128 × 64 OLED |
| Status | Current firmware draft, suitable for website and PDF export |

## Disclaimer

uShutter is a diagnostic and reference tool for camera repair, shutter testing, and calibration support. Results depend on sensor alignment, light-source stability, shutter condition, the attached front end, firmware version, and how the test is set up.

Readings are intended to reveal trends, compare repeated tests, and support practical adjustment decisions. They are not laboratory-certified measurements.

Camera repair can involve delicate mechanisms. You are responsible for safe handling and for any adjustment made from these results. The developer and distributor are not responsible for damage to cameras, shutters, test equipment, or other property resulting from use, misuse, incorrect interpretation, or improper repair.

## Contents

1. [What this edition does](#1-what-this-edition-does)
2. [Before you start](#2-before-you-start)
3. [Quick start](#3-quick-start)
4. [Controls and screens](#4-controls-and-screens)
5. [Front ends](#5-front-ends)
6. [Reading a result](#6-reading-a-result)
7. [Saved records](#7-saved-records)
8. [Errors](#8-errors)
9. [Using the numbers for adjustment](#9-using-the-numbers-for-adjustment)
10. [Firmware update](#10-firmware-update)
11. [Specifications](#11-specifications)
12. [What this edition does not include](#12-what-this-edition-does-not-include)

## 1. What this edition does

The Nano edition is the compact shutter tester. It measures exposure time with optical sensors and shows the result on the OLED.

It supports:

- Leaf mode, for a single sensor path
- Curtain mode, for a two-point focal-plane comparison
- Opening and closing curtain-edge speed
- A five-shot repeatability summary
- Alignment readings
- Up to eight saved summaries

The front end plugged into the tester selects the available measurement. The Nano edition recognizes FE1 and FE2. It does not run the Pico meter front ends.

## 2. Before you start

### 2.1 Power and display

Connect power and wait for the splash screen. It shows the product name and `ver 1.0.0`, then the tester enters `READY`.

### 2.2 Buttons

Four buttons sit under the OLED. Their labels change with the screen. From left to right they are normally:

| Button | Ready label | Typical long-press use |
| --- | --- | --- |
| 1 | `RST` | Not used for alignment on this edition |
| 2 | `MOD` | Switch Leaf and Curtain, when the front end allows it |
| 3 | `ALN` | Enter alignment |
| 4 | `REC` | Open saved records |

A short press is a normal click. A long press is about 1.2 seconds.

### 2.3 Sensor convention

- `L` is the left sensor path, Nano `D2` / `SIG1`.
- `R` is the right sensor path, Nano `D3` / `SIG2`.

Keep this convention consistent between the front end, the camera, and the direction shown on screen. Swapping the two cables reverses `L` and `R`.

## 3. Quick start

1. Attach FE1 for a focal-plane test, or FE2 for a single-path leaf test.
2. Place the sensors in the optical path and turn on a stable light source.
3. From `READY`, long-press `ALN` and confirm that both channels respond when the light path changes.
4. Short-press `RST` to return to `READY`.
5. Cock and fire the shutter five times at the same setting.
6. Read the live result, then the summary pages.
7. If the set is stable and you want to keep it, long-press `SAV`.

## 4. Controls and screens

### 4.1 Ready

The top-left label is `CURTAIN` or `LEAF`. The bottom row is `RST`, `MOD`, `ALN`, `REC`.

| Action | Result |
| --- | --- |
| Short-press `RST` | Clear the current measurement and return to ready |
| Short-press `MOD` | In Leaf mode on a dual front end, switch the selected sensor between `L` and `R` |
| Long-press `MOD` | On FE1, switch between Curtain and Leaf |
| Long-press `ALN` | Enter alignment |
| Long-press `REC` | Open saved records |

FE2 is a single-sensor front end. It stays in Leaf mode and does not offer a left/right sensor choice.

### 4.2 Live measurement

After a valid shutter event the screen shows:

- Progress toward the five-shot set
- Nearest shutter-speed label
- Measured exposure in microseconds
- Exposure error

Curtain mode also shows the travel direction, separate left and right readings, and the calculated edge speeds.

### 4.3 Five-shot summary

Leaf mode has two pages: exposure, then stability.

Curtain mode has four pages:

1. Left exposure
2. Right exposure
3. Curtain speed
4. Stability

The bottom labels become `EXT`, `SAV`, `PGU`, and `PGD`.

| Action | Result |
| --- | --- |
| Short-press `EXT` | Leave the summary and arm a new test |
| Long-press `SAV` | Save a stable summary |
| Short-press `PGU` / `PGD` | Move between summary pages |

`HOLD` means the five-shot set passed the stability check. `UNSTABLE` means it did not. An unstable set can be reviewed, but it is not saved.

### 4.4 Alignment

Long-press `ALN` from `READY`. The screen shows live analog readings for the left and right paths. Use it to position the front end and confirm that the light level changes. Short-press `EXT` to leave.

Normal capture is paused while alignment is active.

### 4.5 Saved records

Long-press `REC` from `READY`. If nothing has been saved, the screen shows `NO RECORD`.

| Action | Result |
| --- | --- |
| Short-press `EXT` | Leave record view |
| Short-press the record-number button | Go to the next saved record |
| Long-press the record-number button | Ask `CLEAR ALL?` |
| Short-press `PGU` / `PGD` | Move through that record's pages |
| `YES` on the confirmation screen | Erase all saved summaries |
| `NO` | Cancel |

## 5. Front ends

The tester identifies the front end from a resistor on the detect pin. Attach it before power-on.

| Front end | Nano behavior |
| --- | --- |
| FE1, dual | Curtain and Leaf. Leaf can use `L` or `R`. |
| FE2, single | Leaf only, on its single sensor path |
| No front end | No shutter measurement. Alignment-style idle screen only. |
| FE3, FE4, FE5, FE6 | Not supported on this edition. Treated as no front end. |

FE6, T-stop, light meter, and distance meter require the Pico edition.

## 6. Reading a result

### 6.1 Exposure

Exposure is the time between the detected opening edge and the confirmed closing edge. It is shown in microseconds.

Examples:

- `1/500` is `2000 us`
- `1/250` is `4000 us`
- `1/125` is `8000 us`
- `1"` is `1000000 us`

### 6.2 Speed label and error

The tester compares the measurement with a standard shutter-speed table and shows the nearest label. A large mismatch is marked as approximate, for example `~1/8`.

Error is relative to that nominal speed:

- Positive means longer, or slower, than nominal.
- Negative means shorter, or faster, than nominal.

### 6.3 Curtain direction and speed

Direction comes from which sensor sees the opening edge first:

- `L->R`: left sensor first
- `R->L`: right sensor first
- `----`: direction could not be determined

`Vo` is the opening-edge speed. `Vc` is the closing-edge speed. Both use the configured `19.0 mm` sensor spacing and are shown in cm/s.

### 6.4 Stability

A five-shot set is stable when the largest sample deviation from the average is within `10%`. The result then freezes as `HOLD`. Otherwise it freezes as `UNSTABLE`.

## 7. Saved records

Only an explicit long-press of `SAV` writes a summary. The store holds eight summaries. If the store cannot accept another one, the screen reports `REC FULL`.

A storage failure may instead show an `EEP ...` message. Retry the save. If it persists, power-cycle with the front end attached and check the board connection before clearing records.

## 8. Errors

| Screen | Meaning | Check |
| --- | --- | --- |
| `E01` | Right sensor saw no opening edge | Alignment, light, cable, front end |
| `E02` | Right sensor saw open but no close | Shutter close, signal level, noise |
| `E03` | Left sensor saw no opening edge | Same checks as `E01`, on the left path |
| `E04` | Left sensor saw open but no close | Same checks as `E02`, on the left path |
| `E05` | Edge order is not physically valid | Incomplete capture, noise, or impossible timing |
| `E06` | Direction changed between samples | Setup moved, camera orientation changed, or channels swapped |
| `TOO FAST` | Shorter than `250 us`, faster than `1/4000` | Speed is outside the valid range, or the edge is a glitch |
| `TOO SLOW` | Longer than `1.5 s` | Speed is outside the valid range, or close was missed |

`TOO FAST` does not mean the shutter is merely a little fast. An exposure of exactly `250 us` is still valid; anything shorter is rejected.

If results wander:

- Recheck alignment with the light source on.
- Keep ambient light steady.
- Confirm `L` and `R` are not swapped.
- Reset before starting a new five-shot series.
- Do not move the front end between shots in one series.

## 9. Using the numbers for adjustment

Compare left and right exposure in Curtain mode. A repeatable difference means the exposure changes across the travel path.

Compare `Vo` and `Vc` on the speed page. A large, repeatable difference means the slit width is changing as the curtains travel.

As a practical bench reference, not a formal tolerance:

- Under `0.1 EV` left/right difference is usually very good.
- Around `0.2 EV` may be acceptable for the camera and its use.
- Over `0.3 EV` is often visible and worth investigating.

Spring adjustment is the usual first place to look when curtain speed is consistently wrong or the two edges do not match. Mechanical curtain alignment is more invasive and is worth considering only after speed is reasonable but a consistent exposure gradient remains.

## 10. Firmware update

The Nano edition is updated through its USB serial port. Use the supplied offline installer when you have a release package. Arduino IDE is not required.

1. Connect the Nano with a USB data cable.
2. Close Serial Monitor and any other program using the port.
3. Run `Install uShutterUp.command` on macOS.
4. If several ports appear, choose the Nano port.
5. Wait for the written-and-verified message.

The installer tries the current bootloader at `115200` baud, then the old bootloader at `57600` baud. If automatic reset does not start the bootloader, it asks you to press Return and immediately press and release the Nano `RESET` button.

If the board is inside a case and the reset button cannot be reached, the reliable alternatives are a working DTR auto-reset circuit, a reset access hole, or a power-cycle procedure that actually removes power from the Nano. Pulling USB does not reset the board when another supply is still powering it.

This installer is for an ATmega328P Nano. It is not for Nano Every, Nano 33, or a Raspberry Pi Pico.

## 11. Specifications

| Item | Value |
| --- | --- |
| Valid exposure | `250 us` to `1,500,000 us` (`1/4000` to `1.5 s`) |
| Capture window | Long enough for the slowest valid exposure, plus margin |
| Close confirmation | `8 ms` at the idle level; the original edge time is kept |
| Sensor spacing used for speed | `19.0 mm` |
| Stability tolerance | `10%` |
| Samples per summary | `5` |
| Saved summaries | `8` |
| Speed match threshold | `15%` |
| Long press | about `1.2 s` |

Timing resolution comes from the microcontroller clock. Practical accuracy still depends on the sensor, slit, light, alignment, and signal quality.

## 12. What this edition does not include

The following belong to the Pico edition, firmware `ver 2.0.0`, and are absent here:

- FE6 dual analog/comparator waveform display
- FE3 T-stop test
- FE4 light meter
- FE5 distance meter
- 240 × 240 color LCD
- Six camera profiles with per-speed median history

Do not expect those screens, buttons, or front ends to work on a Nano running `ver 1.0.0`.
