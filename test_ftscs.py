#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
FTSCS控制器测试文件
演示如何使用FTSCS_Controller类发送各种CAN2.0A转FTSCS1.0协议指令
"""

import serial
import time
from USB2CAN import MotorControl
from FTSCS_Controller import FTSCSController

def main():
    # 串口配置
    port = 'COM3'  # 根据实际情况修改串口号
    baudrate = 1000000
    
    try:
        # 创建串口连接
        serial_device = serial.Serial(port, baudrate, timeout=0.5)
        print(f"串口 {port} 连接成功，波特率: {baudrate}")
        
        # 创建电机控制对象
        motor_control = MotorControl(serial_device)
        
        # 创建FTSCS控制器，CAN站号为1
        ftscs = FTSCSController(motor_control, can_station=1)
        
        print("\n=== FTSCS控制器测试开始 ===")
        
        # 1. 内存表读指令测试
        print("\n1. 发送内存表读指令...")
        ftscs.memory_read(servo_id=1, mem_addr=56, mem_length=2)  # 读取位置信息
        time.sleep(0.1)
        
        # 2. 位置控制指令测试
        print("2. 发送位置控制指令...")
        ftscs.position_control(servo_id=1, target_pos=2000, acceleration=32, speed=1000, current=300)
        time.sleep(0.5)
        
        # 3. 异步控制指令测试
        print("3. 发送异步控制指令...")
        ftscs.async_control(servo_id=1, target_pos=1500, acceleration=16, speed=800, current=250)
        time.sleep(0.1)
        
        # 4. 异步执行指令
        print("4. 发送异步执行指令...")
        ftscs.async_execute()
        time.sleep(0.5)
        
        # 5. 同步控制指令测试
        print("5. 发送同步控制指令...")
        ftscs.sync_control(servo_id=1, target_pos=2500, acceleration=20, speed=1200, current=400)
        ftscs.sync_control(servo_id=2, target_pos=1000, acceleration=25, speed=900, current=350)
        time.sleep(0.1)
        
        # 6. 同步执行指令
        print("6. 发送同步执行指令...")
        ftscs.sync_execute(command_type=0)  # 执行
        time.sleep(0.5)
        
        # 7. 同步读指令测试
        print("7. 发送同步读指令...")
        ftscs.sync_read(mem_addr=56, mem_length=2, servo_ids=[1, 2, 3])  # 读取多个舵机位置
        time.sleep(0.1)
        
        # 8. 内存表写指令测试
        print("8. 发送内存表写指令...")
        ftscs.memory_write(servo_id=1, mem_addr=42, mem_data=[100, 200])  # 写入速度限制
        time.sleep(0.1)
        
        # 9. CAN配置查询
        print("9. 查询CAN配置...")
        ftscs.can_config()  # 查询当前配置
        time.sleep(0.1)
        
        # 10. 舵机校准测试
        print("10. 发送舵机校准指令...")
        ftscs.servo_calibrate(servo_id=1, calibrate_pos=2048)  # 校准到中位
        time.sleep(0.1)
        
        # 11. 查询同步缓存数量
        print("11. 查询同步缓存数量...")
        ftscs.sync_execute(command_type=1)  # 查询
        time.sleep(0.1)
        
        print("\n=== 所有测试指令发送完成 ===")
        print("注意：此测试只发送指令，不处理返回数据")
        
    except serial.SerialException as e:
        print(f"串口错误: {e}")
        print("请检查串口号是否正确，设备是否连接")
    except Exception as e:
        print(f"程序错误: {e}")
    finally:
        if 'serial_device' in locals() and serial_device.is_open:
            serial_device.close()
            print("串口已关闭")

if __name__ == "__main__":
    main()