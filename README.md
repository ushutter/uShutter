# uShutter
uShutter is an open, reproducible shutter timing specification and implementation, focused on time-domain shutter speed and curtain travel measurement.

uShutter provides a firmware and multiple reference hardware interface implementation for measuring:
* shutter exposure time and stability
* curtain travel time and stability

It uses pure time-domain signals from different optical sensors. Measurement accuracy highly depends on the sensors selected and PCB board manufacturing quality.

## What uShutter Is
* An open measurement core for shutter timing
* A reference implementation for single- and dual-sensor setups
* A stable timing interface others can implement or extend
* Suitable for DIY, education, analysis, and experimentation

## Core Capabilities (Open Scope)
uShutter includes the following open-source capabilities:
### Shutter Timing
* Optical trigger–based event capture
* Microsecond-resolution timestamps
* Direction-independent logic
### Single-Sensor Measurement
* Exposure duration measurement
* Multi-shot repeatability analysis
### Dual-Sensor Curtain Analysis
* Curtain travel time
* Relative curtain speed

## Hardware Philosophy
uShutter is designed to work with:
* Arduino / ATmega328P MCUs
* Simple optical sensors (e.g. LM393-based comparators)
* No specific sensor type, threshold method, or light source is assumed by the interface.

## Project Structure
/firmware    Code implementation 

/hardware    Reference schematics and layouts

/docs        Theory, usage, and build notes

## License
Firmware: Apache License 2.0

Hardware designs: CERN-OHL-S

This project is open by design and intent.

## Status
uShutter is under active development.

Interface stability and clarity take priority over feature expansion.

