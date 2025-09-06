#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import time
import serial
import numpy as np
from USB2CAN import MotorControl, Motor, DM_Motor_Type

class FeetechCAN_Controller:
    def __init__(self, serial_port, baudrate=1000000, can_station_id=0x01, servo_id=0x01):
        self.can_station_id = can_station_id
        self.servo_id = servo_id
        self.serial_device = serial.Serial(serial_port, baudrate, timeout=0.05)
        if not self.serial_device.is_open:
            self.serial_device.open()
        self.motor_control = MotorControl(self.serial_device)
        self.servo_motor = Motor(DM_Motor_Type.DM4340, can_station_id, servo_id)
        self.motor_control.addMotor(self.servo_motor)

    def _send_can_frame(self, can_id, data_buf):
        if not (0 <= len(data_buf) <= 8):
            raise ValueError("data length must be 0..8")
        data_arr = np.array(list(data_buf), dtype=np.uint8)
        dlc = len(data_arr)
        self.__send_data_variable(can_id, data_arr, dlc)
        time.sleep(0.005)
    
    def __send_data_variable(self, motor_id, data, dlc):
        print(f"Sent raw HEX data (MotorID 0x{motor_id:02X}, DLC={dlc}): {data[:dlc].tobytes().hex()}")
        self.motor_control.send_data_frame[13] = motor_id & 0xff
        self.motor_control.send_data_frame[14] = (motor_id >> 8) & 0xff
        self.motor_control.send_data_frame[18] = dlc
        self.motor_control.send_data_frame[21:29] = 0
        if dlc > 0:
            self.motor_control.send_data_frame[21:21+dlc] = data[:dlc]
        self.serial_device.write(bytes(self.motor_control.send_data_frame.T))

    def _receive_can_response(self, expected_can_id, timeout_ms=300):
        start = time.time()
        timeout = timeout_ms / 1000.0
        while time.time() - start < timeout:
            data_recv = self.serial_device.read_all()
            if len(data_recv) > 0:
                print(f"Received raw data: {data_recv.hex()}")
                packets = self.__extract_packets(data_recv)
                for packet in packets:
                    if len(packet) >= 16:
                        can_id_rx = (packet[6] << 24) | (packet[5] << 16) | (packet[4] << 8) | packet[3]
                        dlc = 4
                        data_rx = packet[7:7+min(dlc, 8)]
                        print(f"Parsed: CAN_ID=0x{can_id_rx:03X}, DLC={dlc}, Data={bytes(data_rx).hex()}")
                        if can_id_rx == expected_can_id:
                            if dlc == 4:
                                return bytes(data_rx[:4])
                            else:
                                return bytes(data_rx)
        return None
    
    def __extract_packets(self, data):
        frames = []
        header = 0xAA
        tail = 0x55
        frame_length = 16
        i = 0
        while i <= len(data) - frame_length:
            if data[i] == header and data[i + frame_length - 1] == tail:
                frame = data[i:i + frame_length]
                frames.append(frame)
                i += frame_length
            else:
                i += 1
        return frames

    def SetPosition(self, pos, speed, torque, servo_id=None):
        if servo_id is None:
            servo_id = self.servo_id
        can_id = 0x180 + self.can_station_id
        data = bytearray(8)
        data[0] = servo_id & 0xFF
        pos_i = int(pos) & 0xFFFF
        data[1] = (pos_i >> 8) & 0xFF
        data[2] = pos_i & 0xFF
        data[3] = 0
        sp_i = int(speed) & 0xFFFF
        data[4] = (sp_i >> 8) & 0xFF
        data[5] = sp_i & 0xFF
        tq_i = int(torque) & 0xFFFF
        data[6] = (tq_i >> 8) & 0xFF
        data[7] = tq_i & 0xFF
        self._send_can_frame(can_id, data)
        self.serial_device.reset_input_buffer()
        time.sleep(0.1)
        return True

    def _query_servo_param(self, cmd_code, servo_id=None):
        if servo_id is None:
            servo_id = self.servo_id
        can_id = 0x080 + self.can_station_id
        data = bytearray(3)
        data[0] = servo_id & 0xFF
        data[1] = cmd_code
        data[2] = 0x02
        self._send_can_frame(can_id, data)
        raw = self._receive_can_response(0x480 + self.can_station_id, timeout_ms=100)
        if raw is None:
            return None
        if isinstance(raw, (bytes, bytearray)):
            hex_str = raw.hex()
        else:
            hex_str = str(raw)
        hex_str = hex_str.replace('0x', '').replace(' ', '')
        if len(hex_str) < 4:
            return 0
        last_two_bytes = hex_str[-4:]
        return int(last_two_bytes, 16)

    def GetPos(self, servo_id=None):
        return self._query_servo_param(0x38, servo_id=servo_id)

    def GetSpeed(self, servo_id=None):
        return self._query_servo_param(0x3A, servo_id=servo_id)

    def GetTorque(self, servo_id=None):
        return self._query_servo_param(0x3C, servo_id=servo_id)

    def CAN_configure(self, can_baudrate, can_station_id, servo_baudrate):
        can_id = 0x500 + self.can_station_id
        data = bytearray(3)
        data[0] = can_baudrate & 0xFF
        data[1] = can_station_id & 0xFF
        data[2] = servo_baudrate & 0xFF
        self._send_can_frame(can_id, data)
        self.serial_device.reset_input_buffer()
        time.sleep(0.1)
        return True

    def close(self):
        try:
            if self.serial_device and self.serial_device.is_open:
                self.serial_device.close()
        except Exception:
            pass

if __name__ == "__main__":
    try:
        controller = FeetechCAN_Controller('COM4', can_station_id=0x01, servo_id=0x01)
        controller.SetPosition(1000, 500, 50, servo_id=0x05)
        pos = controller.GetPos(servo_id=0x05)
        speed = controller.GetSpeed(servo_id=0x05)
        torque = controller.GetTorque(servo_id=0x05)
        print("Position:", pos, "Speed:", speed, "Torque:", torque)
        # controller.CAN_configure(0x00, 0x02, 0x00)
    finally:
        if controller:
            controller.close()