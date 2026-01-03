# RobStride Motor Control Library for ESP32

Arduino library for controlling RobStride EDULITE05 motors via CAN bus on ESP32 platforms.

## Features

- Support for all RobStride motor control modes:
  - **MIT Motion Control**: Direct position/velocity control with adjustable stiffness (Kp) and damping (Kd)
  - **Position Control (PP)**: Point-to-point position mode with velocity/acceleration limits
  - **Position Control (CSP)**: Continuous position streaming mode
  - **Speed Control**: Velocity control with current limiting
  - **Current/Torque Control**: Direct current/torque command
- Clean API with atomic operations (no internal delays)
- Proper mode switching with safety procedures
- Built on ESP32-TWAI-CAN library

## Hardware Requirements

- ESP32-S3 or ESP32 board (tested on M5Stack AtomS3)
- RobStride EDULITE05 motor
- CAN transceiver (SN65HVD230 or compatible)

## Wiring

```
ESP32        CAN Transceiver
GPIO1   -->  CRX (RX)
GPIO2   -->  CTX (TX)
3.3V    -->  VCC
GND     -->  GND
```

Connect CAN transceiver to RobStride motor CAN bus (CAN-H and CAN-L).

## Installation

### PlatformIO

Add to your `platformio.ini`:

```ini
lib_deps =
    handmade0octopus/ESP32-TWAI-CAN@^1.0.1
    https://github.com/necobit/RobStride_Library.git
```

### Arduino IDE

1. Download this library as ZIP
2. In Arduino IDE: Sketch → Include Library → Add .ZIP Library
3. Install ESP32-TWAI-CAN library from Library Manager

## Quick Start

```cpp
#include <Arduino.h>
#include <ESP32-TWAI-CAN.hpp>
#include <RobStride.h>

RobStrideMotor motor(1);  // Motor ID = 1

void setup() {
  Serial.begin(115200);

  // Initialize CAN bus: 1Mbps, TX=GPIO2, RX=GPIO1
  ESP32Can.begin(TWAI_SPEED_1000KBPS, 2, 1);

  delay(100);

  // Initialize motor
  motor.stop();
  delay(50);
  motor.clear_fault();
  delay(50);

  // Set motion control mode
  motor.set_run_mode(RS_MODE_MOTION);
  delay(20);

  motor.enable();
}

void loop() {
  // MIT motion control: position, velocity, Kp, Kd, torque
  motor.send_motion_command(1.0, 0, 10.0, 0.5, 0);
  delay(10);
}
```

## Control Modes

### MIT Motion Control (RS_MODE_MOTION)

Direct control with position, velocity, and impedance parameters:

```cpp
motor.set_run_mode(RS_MODE_MOTION);
delay(20);
motor.enable();

// position, velocity, Kp, Kd, feedforward_torque
motor.send_motion_command(1.5, 0, 10.0, 0.5, 0);
```

### Position Control - Point to Point (RS_MODE_POS_PP)

```cpp
motor.set_limit_current(3.0);
delay(10);
motor.set_pp_limits(5.0, 5.0);  // velocity, acceleration
delay(10);
motor.set_run_mode(RS_MODE_POS_PP);
delay(20);
motor.enable();

motor.set_position_reference(2.0);  // Target position in radians
```

### Position Control - Continuous (RS_MODE_POS_CSP)

```cpp
motor.set_limit_speed(5.0);
delay(10);
motor.set_limit_current(3.0);
delay(10);
motor.set_run_mode(RS_MODE_POS_CSP);
delay(20);
motor.enable();

motor.set_position_reference(1.0);  // Update position continuously
```

### Speed Control (RS_MODE_SPEED)

```cpp
motor.set_limit_current(3.0);
delay(10);
motor.set_limit_accel_rad(10.0);
delay(10);
motor.set_run_mode(RS_MODE_SPEED);
delay(20);
motor.enable();

motor.set_speed_reference(5.0);  // rad/s
```

### Current/Torque Control (RS_MODE_CURRENT)

```cpp
motor.set_run_mode(RS_MODE_CURRENT);
delay(20);
motor.enable();

motor.set_current_reference(0.5);  // Amperes
```

## API Reference

### Basic Commands

```cpp
motor.enable();              // Enable motor
motor.stop();                // Stop motor (disable)
motor.clear_fault();         // Clear fault status
motor.set_zero_position();   // Set current position as zero
```

### Mode Configuration

```cpp
motor.set_run_mode(mode);    // Set control mode
// Modes: RS_MODE_MOTION, RS_MODE_POS_PP, RS_MODE_POS_CSP,
//        RS_MODE_SPEED, RS_MODE_CURRENT
```

### Control Commands

```cpp
// MIT Motion Control
motor.send_motion_command(position, velocity, kp, kd, torque);

// Position/Speed/Current Reference
motor.set_position_reference(position);
motor.set_speed_reference(speed);
motor.set_current_reference(current);
```

### Limits Configuration

```cpp
motor.set_limit_current(current);      // Set current limit (A)
motor.set_limit_speed(speed);          // Set speed limit (rad/s)
motor.set_limit_accel_rad(accel);      // Set acceleration limit for speed mode
motor.set_pp_limits(velocity, accel);  // Set limits for PP position mode
```

### Parameter Ranges

| Parameter | Min | Max | Unit |
|-----------|-----|-----|------|
| Position | -12.57 | 12.57 | rad |
| Velocity | -50.0 | 50.0 | rad/s |
| Kp | 0.0 | 500.0 | - |
| Kd | 0.0 | 5.0 | - |
| Torque | -6.0 | 6.0 | Nm |

## Important Notes

### Safe Mode Switching

Always follow this sequence when switching control modes:

1. Stop motor
2. Clear faults (recommended twice)
3. Set parameter limits BEFORE setting new mode
4. Set run mode
5. Enable motor

```cpp
void switch_mode_safely(uint8_t new_mode) {
  motor.stop();
  delay(50);
  motor.clear_fault();
  delay(10);
  motor.clear_fault();
  delay(50);

  // Set limits based on mode
  if (new_mode == RS_MODE_SPEED) {
    motor.set_limit_current(3.0);
    delay(10);
    motor.set_limit_accel_rad(10.0);
    delay(10);
  }

  motor.set_run_mode(new_mode);
  delay(20);
  motor.enable();
}
```

### Timing Requirements

- 10ms delay between parameter writes
- 20ms delay after mode changes before enabling
- 50ms delay after stop/fault clear operations

### Library Design

All library methods are **atomic** (no internal delays). Application code is responsible for proper timing between commands.

## Examples

See the `examples/` folder for complete examples:

- **BasicControl**: Simple position control example
- **DemoLoop**: Comprehensive demo cycling through all control modes

## License

MIT License

## Credits

Developed for RobStride EDULITE05 motors using the ESP32-TWAI-CAN library.
