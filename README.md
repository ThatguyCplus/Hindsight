# Hindsight

**Senior Design Project**: Build a safe, paraplegic-focused electronic throttle and brake control system for a 2002 Honda Insight. We’re designing the driver interface, selecting and validating sensors and actuators, and integrating everything on the vehicle for a live demo that aligns with SAE safety principles and ADA usability goals.usability goals.

## Project Overview

Hindsight retrofits a 2002 Honda Insight with an electronic throttle-and-brake assist system so paraplegic drivers can control the car using alternative hand inputs—without taking away the original pedals for other family members. It converts pedal commands into a safe, intuitive electronic interface while prioritizing fail-safes, manual override, and everyday usability for all drivers.


## Repository Structure

### 📂 Brakes/
Contains the brake control system implementation using a Tesla Gen2 iBooster unit.

<!-- TODO: Add image of iBooster installation in vehicle brake system -->
<!-- <img width="800" alt="iBooster Installation" src="path/to/ibooster-installation.jpg" /> -->

- **`Iboostercontrolnotworkingjustthrottlepostion/`**
  - `Iboostercontrolnotworkingjustthrottlepostion.ino` - Tesla Gen2 iBooster CAN bus controller
    - Controls brake system via CAN bus (500 kbps)
    - Uses MCP2515 CAN transceiver with Arduino Mega
    - Implements serial commands for brake control (apply/release/hold)
    - Monitors brake rod position and status via 0x39D CAN messages

<!-- TODO: Add image of brake control CAN bus setup or wiring diagram -->
<!-- <img width="800" alt="CAN Bus Wiring" src="path/to/can-bus-wiring.jpg" /> -->

### 📂 Throttle/
Complete throttle control system with Arduino firmware and mechanical CAD designs.

<!-- TODO: Add image of throttle mechanism with motor actuator and sensor -->
<!-- <img width="800" alt="Throttle Actuator System" src="path/to/throttle-actuator.jpg" /> -->

#### **Throttle Arduino Sketches/**
Arduino firmware for throttle control and sensor testing:

- **`as5048atestingscketches/livereadout/`**
  - `livereadout.ino` - AS5048A magnetic angle sensor testing with motor control
    - Logs angle measurements at 100 Hz during motor cycles
    - Tests IBT-2/BTS7960 motor driver with forward/reverse/coast sequences
    - CSV output format for data analysis

- **`initialjoystick/`**
  - `initialjoystick.ino` - Joystick-based throttle control interface
    - Maps joystick input to motor control with exponential response curve
    - Implements hard-stop (electronic brake) with current monitoring
    - Safety features: dead zone, current limits, relay control

- **`Intialtestingcruisecontrolhindsight/`**
  - `Intialtestingcruisecontrolhindsight.ino` - Cruise control functionality testing
    - Initial implementation for throttle position holding

#### **Throttle CAD work/**
Mechanical design files for sensor mounting and bracket systems:

- Sensor support bracket designs (v2, v3, v5)
- Cruise control bracket designs
- Files in multiple formats: `.SLDPRT` (SolidWorks), `.STL`, `.3MF`
- Key components:
  - `SensorSupportbracketv5covered.STL/.3MF` - Latest sensor mount design
  - `NoarmCC.SLDPRT` - Cruise control bracket without arm
  - `IntialCCbracket.SLDPRT` - Initial cruise control bracket concept

<!-- TODO: Add rendered image of CAD bracket designs or 3D printed parts -->
<!-- <img width="800" alt="Sensor Support Bracket v5" src="path/to/bracket-v5.jpg" /> -->
<!-- <img width="800" alt="3D Printed Bracket Installed" src="path/to/bracket-installed.jpg" /> -->

### 📂 Honda Insight 2002 3D scans/
3D scanned models of vehicle interior components used for design and integration.

<!-- TODO: Add example 3D scan visualization showing brake system context -->
<!-- <img width="800" alt="3D Scan - Brake System Context" src="path/to/brake-scan-view.jpg" /> -->

- **Brakes/**
  - iBooster context scans with corrected axis
  - Full detail scans of brake system mounting area
  - Formats: `.obj`, `.mtl`, `.SLDPRT`

- **Dashboard/**
  - Dashboard assembly with left interior pillar trim
  - Used for HMI (Human-Machine Interface) placement and integration

- **Footwell/**
  - Footwell area scans with trims installed
  - Critical for understanding pedal replacement constraints and mounting locations

- **Insight behind trunk empty space/**
  - Trunk empty space scans for component placement
  - Potential location for control electronics and actuators

- **UI (Between two seats parking brake area)/**
  - Area between seats where parking brake is located
  - Candidate location for alternative control interface placement

<!-- TODO: Add image of footwell area or dashboard showing integration points -->
<!-- <img width="800" alt="Footwell Integration" src="path/to/footwell-integration.jpg" /> -->
<!-- <img width="800" alt="UI Control Location" src="path/to/ui-location.jpg" /> -->

## Hardware Components

<!-- TODO: Add image of complete hardware setup or component layout diagram -->
<!-- <img width="800" alt="Hardware Component Layout" src="path/to/hardware-layout.jpg" /> -->

### Control Electronics
- **Arduino Mega 2560** - Primary microcontroller
- **MCP2515** - CAN bus controller (for brake system communication)
- **AS5048A** - Magnetic angle position sensor (for throttle position feedback)

### Actuators & Drivers
- **IBT-2 / BTS7960** - High-current H-bridge motor driver for throttle actuation
- **Tesla Gen2 iBooster** - Electronic brake booster (controlled via CAN bus)

### Sensors & Input
- **Analog Joystick** - Primary user input interface
- **Current Sense Resistors** - Motor current monitoring for safety

<!-- TODO: Add image of joystick interface or user control panel -->
<!-- <img width="800" alt="Joystick Control Interface" src="path/to/joystick-interface.jpg" /> -->

## Safety Features

- Current monitoring with automatic shutdown on overcurrent
- Hard-stop (electronic brake) functionality for rapid motor stopping
- CAN bus status monitoring for brake system feedback
- Exponential response curves for smooth control input
- Dead zone implementation to prevent unintended operation

## Software Requirements

- Arduino IDE (for firmware development)
- SolidWorks (for CAD file viewing/editing - `.SLDPRT` files)
- 3D viewer/printing software (for `.STL`, `.3MF` files)
- CAN bus analysis tools (optional, for debugging)

## Getting Started

1. **Hardware Setup**: Review the wiring diagrams in the Arduino sketch comments
2. **Firmware**: Upload the appropriate sketch to your Arduino Mega
3. **Calibration**: Adjust tuning parameters in the firmware (dead zones, current limits, etc.)
4. **Testing**: Use serial monitor for debugging and status monitoring

<!-- TODO: Add wiring diagram or connection schematic -->
<!-- <img width="800" alt="System Wiring Diagram" src="path/to/wiring-diagram.jpg" /> -->

## Serial Commands (Brake System)

When using the iBooster controller (`Iboostercontrolnotworkingjustthrottlepostion.ino`):

- `r` - Toggle raw CAN log
- `e` - Enable external request (take control)
- `d` - Disable external request (driver only, flow to zero-point)
- `b` - Apply brakes (flow_rate > zero point)
- `s` - Release brakes (flow_rate < zero point)
- `z` - Hold pressure (flow_rate = zero point)

## License

This project is licensed under the GNU General Public License v3.0 - see the [LICENSE](LICENSE) file for details.

## Contributing

This is a senior design project. For questions or contributions, please open an issue or contact the project maintainers.

---

## ⚠️ Important Disclaimer

**RESEARCH PROJECT ONLY — NOT FOR PRODUCTION USE**

This project is an academic research and development effort conducted as part of a senior design course. It is intended **solely for educational, research, and demonstration purposes**.

**NO WARRANTIES**: This software, hardware designs, and documentation are provided "AS IS" without warranty of any kind, express or implied, including but not limited to warranties of merchantability, fitness for a particular purpose, or safety for road use.

**NOT INTENDED FOR PUBLIC ROAD USE**: 
- This system modifies critical vehicle safety systems (throttle and brake controls)
- **DO NOT install, test, or operate this system on public roads**
- This system has NOT been certified, validated, or approved for real-world vehicle operation
- Any use of this system is at your own risk

**RESPONSIBILITY**: Anyone using, modifying, or building upon this project assumes full responsibility and liability for any damages, injuries, or legal consequences that may result. The authors and contributors are not responsible for any outcomes resulting from the use of this research project.

**FOR RESEARCH AND EDUCATIONAL PURPOSES ONLY.**

