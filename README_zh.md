# CAN2.0A转FTSCS1.0控制器使用说明

[English Version](README.md)

## 概述

本项目实现了基于CAN2.0A转FTSCS1.0协议的舵机控制器，支持FEETECH舵机的所有控制指令。

## 文件说明

- `FTSCS_Controller.py` - FTSCS控制器主类
- `test_ftscs.py` - 测试示例文件
- `USB2CAN.py` - 底层CAN通信类

## 快速开始

### 1. 基本使用

```python
import serial
from USB2CAN import MotorControl
from FTSCS_Controller import FTSCSController

# 创建串口连接
serial_device = serial.Serial('COM5', 1000000, timeout=0.5)
motor_control = MotorControl(serial_device)

# 创建FTSCS控制器
ftscs = FTSCSController(motor_control, can_station=1)

# 控制舵机位置
ftscs.position_control(servo_id=1, target_pos=2000, acceleration=32, speed=1000, current=300)
```

### 2. 支持的控制指令

#### 2.1 内存表读指令 (0x01)
```python
ftscs.memory_read(servo_id=1, mem_addr=56, mem_length=2)
```

#### 2.2 内存表写指令 (0x02)
```python
ftscs.memory_write(servo_id=1, mem_addr=42, mem_data=[100, 200])
```

#### 2.3 位置控制指令 (0x03)
```python
ftscs.position_control(servo_id=1, target_pos=2000, acceleration=32, speed=1000, current=300)
```

#### 2.4 同步读指令 (0x04)
```python
ftscs.sync_read(mem_addr=56, mem_length=2, servo_ids=[1, 2, 3])
```

#### 2.5 异步控制指令 (0x05)
```python
ftscs.async_control(servo_id=1, target_pos=1500, acceleration=16, speed=800, current=250)
ftscs.async_execute()  # 执行异步指令
```

#### 2.6 同步控制指令 (0x07)
```python
ftscs.sync_control(servo_id=1, target_pos=2500, acceleration=20, speed=1200, current=400)
ftscs.sync_control(servo_id=2, target_pos=1000, acceleration=25, speed=900, current=350)
ftscs.sync_execute(command_type=0)  # 执行同步指令
```

#### 2.7 CAN配置指令 (0x0A)
```python
# 查询配置
ftscs.can_config()

# 设置配置
ftscs.can_config(can_baudrate=0, can_station_new=2, servo_baudrate=1)
```

#### 2.8 舵机校准指令 (0x0B)
```python
# 校准到指定位置
ftscs.servo_calibrate(servo_id=1, calibrate_pos=2048)

# 校准到中位
ftscs.servo_calibrate(servo_id=1)
```

## 参数说明

### CAN站号
- 范围：1-63
- 0为广播地址

### 波特率配置
- CAN波特率：0=1M, 1=500k, 2=250k, 3=100k
- 舵机波特率：0=1M, 1=500k, 2=250k, 3=115200

### 位置参数
- 位置范围：0-4095（对应0-360度）
- 中位：2048（对应180度）

### 速度和加速度
- 速度：0-1023
- 加速度：0-254

### 电流限制
- 范围：0-1023
- 单位：mA

## 注意事项

1. **通信规则**：遵守一问一答的通信规则，避免485/TTL总线数据冲突
2. **数据长度**：内存读写操作最多支持6字节数据
3. **同步操作**：同步控制指令需要配合同步执行指令使用
4. **配置生效**：CAN配置修改后需要重启设备才能生效
5. **串口设置**：确保串口号和波特率设置正确

## 运行测试

```bash
python test_ftscs.py
```

修改`test_ftscs.py`中的串口号（COM5）为实际使用的串口。

## 协议参考

本实现严格按照CAN2.0A转FTSCS1.0协议规范，支持所有标准指令码：

- 0x01: 内存表读指令
- 0x02: 内存表写指令  
- 0x03: 位置控制指令
- 0x04: 同步读指令
- 0x05: 异步控制指令
- 0x06: 异步执行指令
- 0x07: 同步控制指令
- 0x08: 同步执行指令
- 0x0A: CAN配置指令
- 0x0B: 舵机校准指令
