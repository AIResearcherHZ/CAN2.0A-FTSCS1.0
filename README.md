# CAN2.0A to FTSCS1.0 Controller User Guide

## Overview

This project implements a servo controller based on the CAN2.0A to FTSCS1.0 protocol, supporting all control instructions for FEETECH servos.

## File Description

- `FTSCS_Controller.py` - FTSCS controller main class
- `test_ftscs.py` - Test example file
- `USB2CAN.py` - Low-level CAN communication class

## Quick Start

### 1. Basic Usage

```python
import serial
from USB2CAN import MotorControl
from FTSCS_Controller import FTSCSController

# Create serial connection
serial_device = serial.Serial('COM5', 1000000, timeout=0.5)
motor_control = MotorControl(serial_device)

# Create FTSCS controller
ftscs = FTSCSController(motor_control, can_station=1)

# Control servo position
ftscs.position_control(servo_id=1, target_pos=2000, acceleration=32, speed=1000, current=300)
```

### 2. Supported Control Commands

#### 2.1 Memory Table Read Command (0x01)
```python
ftscs.memory_read(servo_id=1, mem_addr=56, mem_length=2)
```

#### 2.2 Memory Table Write Command (0x02)
```python
ftscs.memory_write(servo_id=1, mem_addr=42, mem_data=[100, 200])
```

#### 2.3 Position Control Command (0x03)
```python
ftscs.position_control(servo_id=1, target_pos=2000, acceleration=32, speed=1000, current=300)
```

#### 2.4 Sync Read Command (0x04)
```python
ftscs.sync_read(mem_addr=56, mem_length=2, servo_ids=[1, 2, 3])
```

#### 2.5 Async Control Command (0x05)
```python
ftscs.async_control(servo_id=1, target_pos=1500, acceleration=16, speed=800, current=250)
ftscs.async_execute()  # Execute async command
```

#### 2.6 Sync Control Command (0x07)
```python
ftscs.sync_control(servo_id=1, target_pos=2500, acceleration=20, speed=1200, current=400)
ftscs.sync_control(servo_id=2, target_pos=1000, acceleration=25, speed=900, current=350)
ftscs.sync_execute(command_type=0)  # Execute sync command
```

#### 2.7 CAN Configuration Command (0x0A)
```python
# Query configuration
ftscs.can_config()

# Set configuration
ftscs.can_config(can_baudrate=0, can_station_new=2, servo_baudrate=1)
```

#### 2.8 Servo Calibration Command (0x0B)
```python
# Calibrate to specified position
ftscs.servo_calibrate(servo_id=1, calibrate_pos=2048)

# Calibrate to center position
ftscs.servo_calibrate(servo_id=1)
```

## Parameter Description

### CAN Station ID
- Range: 1-63
- 0 is broadcast address

### Baud Rate Configuration
- CAN Baud Rate: 0=1M, 1=500k, 2=250k, 3=100k
- Servo Baud Rate: 0=1M, 1=500k, 2=250k, 3=115200

### Position Parameters
- Position Range: 0-4095 (corresponds to 0-360 degrees)
- Center Position: 2048 (corresponds to 180 degrees)

### Speed and Acceleration
- Speed: 0-1023
- Acceleration: 0-254

### Current Limit
- Range: 0-1023
- Unit: mA

## Notes

1. **Communication Rules**: Follow the question-answer communication rule to avoid 485/TTL bus data conflicts
2. **Data Length**: Memory read/write operations support up to 6 bytes of data
3. **Sync Operations**: Sync control commands need to be used with sync execute commands
4. **Configuration Effect**: CAN configuration changes require device restart to take effect
5. **Serial Settings**: Ensure serial port number and baud rate settings are correct

## Running Tests

```bash
python test_ftscs.py
```

Modify the serial port number (COM5) in `test_ftscs.py` to the actually used serial port.

## Protocol Reference

This implementation strictly follows the CAN2.0A to FTSCS1.0 protocol specification, supporting all standard command codes:

- 0x01: Memory table read command
- 0x02: Memory table write command
- 0x03: Position control command
- 0x04: Sync read command
- 0x05: Async control command
- 0x06: Async execute command
- 0x07: Sync control command
- 0x08: Sync execute command
- 0x0A: CAN configuration command
- 0x0B: Servo calibration command

---

[中文版本](README.md)
