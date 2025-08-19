from USB2CAN import MotorControl
import numpy as np
import time

class FTSCSController:
    """
    CAN2.0A转FTSCS1.0协议控制器
    基于现有的MotorControl类实现FEETECH舵机控制指令
    """
    
    def __init__(self, motor_control: MotorControl, can_station=1):
        """
        初始化FTSCS控制器
        :param motor_control: MotorControl对象
        :param can_station: CAN站号 (1-63)
        """
        self.motor_control = motor_control
        self.can_station = can_station
    
    def memory_read(self, servo_id, mem_addr, mem_length):
        """
        内存表读指令 (0x01)
        :param servo_id: 舵机ID (1字节)
        :param mem_addr: 内存表地址 (1字节)
        :param mem_length: 内存表长度 (1字节, N<=6)
        """
        can_id = 0x080 + self.can_station
        data = np.array([servo_id, mem_addr, mem_length, 0, 0, 0, 0, 0], dtype=np.uint8)
        self.motor_control._MotorControl__send_data(can_id, data)
    
    def memory_write(self, servo_id, mem_addr, mem_data):
        """
        内存表写指令 (0x02)
        :param servo_id: 舵机ID (1字节)
        :param mem_addr: 内存表地址 (1字节)
        :param mem_data: 内存表数据 (最多6字节的列表或数组)
        """
        can_id = 0x100 + self.can_station
        data = np.array([0, 0, 0, 0, 0, 0, 0, 0], dtype=np.uint8)
        data[0] = servo_id
        data[1] = mem_addr
        
        # 填充内存数据，最多6字节
        mem_data = np.array(mem_data, dtype=np.uint8)
        data_len = min(len(mem_data), 6)
        data[2:2+data_len] = mem_data[:data_len]
        
        self.motor_control._MotorControl__send_data(can_id, data)
    
    def position_control(self, servo_id, target_pos, acceleration, speed, current):
        """
        位置控制指令 (0x03)
        :param servo_id: 舵机ID (1字节)
        :param target_pos: 目标位置 (2字节)
        :param acceleration: 运行加速度 (1字节)
        :param speed: 运行速度 (2字节)
        :param current: 运行电流 (2字节)
        """
        can_id = 0x180 + self.can_station
        data = np.array([
            servo_id,
            target_pos & 0xFF,          # 位置低字节
            (target_pos >> 8) & 0xFF,   # 位置高字节
            acceleration,
            speed & 0xFF,               # 速度低字节
            (speed >> 8) & 0xFF,        # 速度高字节
            current & 0xFF,             # 电流低字节
            (current >> 8) & 0xFF       # 电流高字节
        ], dtype=np.uint8)
        self.motor_control._MotorControl__send_data(can_id, data)
    
    def sync_read(self, mem_addr, mem_length, servo_ids):
        """
        同步读指令 (0x04)
        :param mem_addr: 内存表地址 (1字节)
        :param mem_length: 内存表长度 (1字节, N<=6)
        :param servo_ids: 舵机ID列表 (最多6个)
        """
        can_id = 0x200 + self.can_station
        data = np.array([0, 0, 0, 0, 0, 0, 0, 0], dtype=np.uint8)
        data[0] = mem_addr
        data[1] = mem_length
        
        # 填充舵机ID，最多6个
        servo_ids = np.array(servo_ids, dtype=np.uint8)
        id_count = min(len(servo_ids), 6)
        data[2:2+id_count] = servo_ids[:id_count]
        
        self.motor_control._MotorControl__send_data(can_id, data)
    
    def async_control(self, servo_id, target_pos, acceleration, speed, current):
        """
        异步控制指令 (0x05)
        :param servo_id: 舵机ID (1字节)
        :param target_pos: 目标位置 (2字节)
        :param acceleration: 运行加速度 (1字节)
        :param speed: 运行速度 (2字节)
        :param current: 运行电流 (2字节)
        """
        can_id = 0x280 + self.can_station
        data = np.array([
            servo_id,
            target_pos & 0xFF,          # 位置低字节
            (target_pos >> 8) & 0xFF,   # 位置高字节
            acceleration,
            speed & 0xFF,               # 速度低字节
            (speed >> 8) & 0xFF,        # 速度高字节
            current & 0xFF,             # 电流低字节
            (current >> 8) & 0xFF       # 电流高字节
        ], dtype=np.uint8)
        self.motor_control._MotorControl__send_data(can_id, data)
    
    def async_execute(self):
        """
        异步执行指令 (0x06)
        执行缓存的异步控制指令
        """
        can_id = 0x300 + self.can_station
        data = np.array([0, 0, 0, 0, 0, 0, 0, 0], dtype=np.uint8)
        self.motor_control._MotorControl__send_data(can_id, data)
    
    def sync_control(self, servo_id, target_pos, acceleration, speed, current):
        """
        同步控制指令 (0x07)
        :param servo_id: 舵机ID (1字节)
        :param target_pos: 目标位置 (2字节)
        :param acceleration: 运行加速度 (1字节)
        :param speed: 运行速度 (2字节)
        :param current: 运行电流 (2字节)
        """
        can_id = 0x380 + self.can_station
        data = np.array([
            servo_id,
            target_pos & 0xFF,          # 位置低字节
            (target_pos >> 8) & 0xFF,   # 位置高字节
            acceleration,
            speed & 0xFF,               # 速度低字节
            (speed >> 8) & 0xFF,        # 速度高字节
            current & 0xFF,             # 电流低字节
            (current >> 8) & 0xFF       # 电流高字节
        ], dtype=np.uint8)
        self.motor_control._MotorControl__send_data(can_id, data)
    
    def sync_execute(self, command_type=0):
        """
        同步执行指令 (0x08)
        :param command_type: 0=执行, 1=查询缓存数量, 其他=清除缓存
        """
        can_id = 0x400 + self.can_station
        if command_type == 0:  # 执行指令
            data = np.array([0, 0, 0, 0, 0, 0, 0, 0], dtype=np.uint8)
        elif command_type == 1:  # 查询指令
            data = np.array([1, 0, 0, 0, 0, 0, 0, 0], dtype=np.uint8)
        else:  # 清除指令
            data = np.array([0, 0, 0, 0, 0, 0, 0, 0], dtype=np.uint8)
        
        self.motor_control._MotorControl__send_data(can_id, data)
    
    def can_config(self, can_baudrate=None, can_station_new=None, servo_baudrate=None):
        """
        CAN配置指令 (0x0A)
        :param can_baudrate: CAN波特率 (0=1M, 1=500k, 2=250k, 3=100k)
        :param can_station_new: 新CAN站号 (1-63)
        :param servo_baudrate: 舵机波特率 (0=1M, 1=500k, 2=250k, 3=115200)
        如果参数为None，则查询当前配置
        """
        can_id = 0x500 + self.can_station
        
        if can_baudrate is None and can_station_new is None and servo_baudrate is None:
            # 查询配置
            data = np.array([0, 0, 0, 0, 0, 0, 0, 0], dtype=np.uint8)
        else:
            # 设置配置
            data = np.array([
                can_baudrate if can_baudrate is not None else 0,
                can_station_new if can_station_new is not None else self.can_station,
                servo_baudrate if servo_baudrate is not None else 0,
                0, 0, 0, 0, 0
            ], dtype=np.uint8)
        
        self.motor_control._MotorControl__send_data(can_id, data)
    
    def servo_calibrate(self, servo_id, calibrate_pos=None):
        """
        舵机校准指令 (0x0B)
        :param servo_id: 舵机ID (1字节)
        :param calibrate_pos: 校准位置 (2字节)，如果为None则校准为中位
        """
        can_id = 0x580 + self.can_station
        
        if calibrate_pos is None:
            # 校准为中位
            data = np.array([servo_id, 0, 0, 0, 0, 0, 0, 0], dtype=np.uint8)
        else:
            # 校准为指定位置
            data = np.array([
                servo_id,
                calibrate_pos & 0xFF,        # 位置低字节
                (calibrate_pos >> 8) & 0xFF, # 位置高字节
                0, 0, 0, 0, 0
            ], dtype=np.uint8)
        
        self.motor_control._MotorControl__send_data(can_id, data)

# 使用示例
if __name__ == "__main__":
    import serial
    
    # 创建串口对象和电机控制对象
    serial_device = serial.Serial('COM3', 1000000, timeout=0.5)
    motor_control = MotorControl(serial_device)
    
    # 创建FTSCS控制器
    ftscs = FTSCSController(motor_control, can_station=1)
    
    print("FTSCS控制器示例...")
    
    # 示例：位置控制
    ftscs.position_control(servo_id=2, target_pos=2000, acceleration=32, speed=1000, current=300)
    time.sleep(0.1)
    
    # 示例：内存读取
    ftscs.memory_read(servo_id=1, mem_addr=56, mem_length=2)
    time.sleep(0.1)
    
    # 示例：CAN配置查询
    ftscs.can_config()
    time.sleep(0.1)
    
    print("示例完成")