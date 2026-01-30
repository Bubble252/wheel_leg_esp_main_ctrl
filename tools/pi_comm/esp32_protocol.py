#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ESP32 串口通信协议库

用于 Raspberry Pi 与 ESP32 之间的二进制协议通信。
协议特性:
- 帧格式: [0xAA 0x55] [LEN] [SEQ] [CMD] [DATA...] [CRC16]
- 字节序: 大端序 (Big-Endian)
- CRC: CRC16-CCITT (0x1021, 初值 0xFFFF)

作者: Bubble
日期: 2026-01-27
"""

import serial
import struct
import threading
import time
import logging
from enum import IntEnum
from dataclasses import dataclass, field
from typing import Callable, Optional, Dict, Any
from collections import deque

# ============================================================================
# 日志配置
# ============================================================================

logging.basicConfig(level=logging.INFO, format='%(asctime)s [%(levelname)s] %(message)s')
logger = logging.getLogger('ESP32Protocol')

# ============================================================================
# 命令码定义
# ============================================================================

class ProtocolConstants:
    """协议常量 - 兼容 GUI 使用"""
    # 帧头
    FRAME_HEADER_1 = 0xAA
    FRAME_HEADER_2 = 0x55
    
    # 系统命令 (0x00-0x1F)
    CMD_HEARTBEAT       = 0x01
    CMD_HEARTBEAT_ACK   = 0x02
    CMD_HANDSHAKE       = 0x03
    CMD_HANDSHAKE_ACK   = 0x04
    CMD_GET_VERSION     = 0x05
    CMD_VERSION_REPORT  = 0x06
    CMD_RESET           = 0x07
    
    # 控制命令 (0x20-0x3F)
    CMD_SET_VELOCITY    = 0x20
    CMD_SET_MODE        = 0x21
    CMD_SET_HEIGHT      = 0x22
    CMD_EMERGENCY_STOP  = 0x23
    CMD_MOTOR_ENABLE    = 0x24
    CMD_SET_PITCH       = 0x25
    CMD_SET_ROLL        = 0x26
    CMD_SET_POSE        = 0x27
    CMD_JUMP            = 0x28
    
    # 查询命令 (0x40-0x5F)
    CMD_GET_STATUS      = 0x40
    CMD_GET_IMU         = 0x41
    CMD_GET_MOTOR       = 0x42
    CMD_GET_BATTERY     = 0x43
    
    # 上报命令 (0x60-0x7F)
    CMD_STATUS_REPORT   = 0x60
    CMD_IMU_REPORT      = 0x61
    CMD_MOTOR_REPORT    = 0x62
    CMD_ERROR_REPORT    = 0x63
    
    # 参数配置 (0x80-0x9F)
    CMD_SET_PID         = 0x80
    CMD_GET_PID         = 0x81
    
    # 通用响应
    CMD_ACK             = 0xF0
    CMD_NACK            = 0xF1


class CMD(IntEnum):
    """命令码枚举"""
    # 系统命令 (0x00-0x1F)
    HEARTBEAT           = 0x01
    HEARTBEAT_ACK       = 0x02
    HANDSHAKE           = 0x03
    HANDSHAKE_ACK       = 0x04
    GET_VERSION         = 0x05
    VERSION_REPORT      = 0x06
    RESET               = 0x07
    ENTER_DEBUG         = 0x08
    EXIT_DEBUG          = 0x09
    SET_LOG_LEVEL       = 0x0A
    
    # 控制命令 (0x20-0x3F)
    SET_VELOCITY        = 0x20
    SET_MODE            = 0x21
    SET_HEIGHT          = 0x22
    EMERGENCY_STOP      = 0x23
    MOTOR_ENABLE        = 0x24
    SET_PITCH           = 0x25
    SET_ROLL            = 0x26
    SET_POSE            = 0x27
    JUMP                = 0x28
    
    # 查询命令 (0x40-0x5F)
    GET_STATUS          = 0x40
    GET_IMU             = 0x41
    GET_MOTOR           = 0x42
    GET_BATTERY         = 0x43
    GET_ERROR           = 0x44
    
    # 上报命令 (0x60-0x7F)
    STATUS_REPORT       = 0x60
    IMU_REPORT          = 0x61
    MOTOR_REPORT        = 0x62
    ERROR_REPORT        = 0x63
    EVENT_REPORT        = 0x64
    
    # 参数配置 (0x80-0x9F)
    SET_PID             = 0x80
    GET_PID             = 0x81
    SET_LQR             = 0x82
    GET_LQR             = 0x83
    SAVE_CONFIG         = 0x84
    LOAD_CONFIG         = 0x85
    RESET_CONFIG        = 0x86
    
    # 通用响应 (0xF0-0xFF)
    ACK                 = 0xF0
    NACK                = 0xF1


class MotionMode(IntEnum):
    """运动模式"""
    IDLE        = 0
    STAND       = 1
    WALK        = 2
    CROUCH      = 3
    JUMP        = 4
    RECOVERY    = 5


class SystemStatus(IntEnum):
    """系统状态"""
    BOOT        = 0
    IDLE        = 1
    RUNNING     = 2
    ERROR       = 3
    ESTOP       = 4
    RECOVERING  = 5


# 状态标志位
FLAG_MOTOR_ENABLED  = 0x01
FLAG_BALANCE_ACTIVE = 0x02
FLAG_YAW_HOLDING    = 0x04
FLAG_PI_CONNECTED   = 0x08
FLAG_LOW_BATTERY    = 0x10
FLAG_OVER_TILT      = 0x20

# ============================================================================
# 数据结构
# ============================================================================

@dataclass
class StatusReport:
    """状态上报数据"""
    timestamp: int = 0
    mode: int = 0
    status: int = 0
    error_code: int = 0
    flags: int = 0
    pitch: float = 0.0
    roll: float = 0.0
    yaw: float = 0.0
    vx_actual: float = 0.0
    yaw_rate_actual: float = 0.0
    battery_voltage: float = 0.0
    height_actual: float = 0.0
    
    @classmethod
    def from_bytes(cls, data: bytes) -> 'StatusReport':
        """从字节数据解析 (大端序)"""
        if len(data) < 36:
            raise ValueError(f"Data too short: {len(data)} < 36")
        
        timestamp, = struct.unpack('>I', data[0:4])
        mode = data[4]
        status = data[5]
        error_code = data[6]
        flags = data[7]
        pitch, roll, yaw = struct.unpack('>fff', data[8:20])
        vx_actual, yaw_rate_actual = struct.unpack('>ff', data[20:28])
        battery_voltage, height_actual = struct.unpack('>ff', data[28:36])
        
        return cls(
            timestamp=timestamp,
            mode=mode,
            status=status,
            error_code=error_code,
            flags=flags,
            pitch=pitch,
            roll=roll,
            yaw=yaw,
            vx_actual=vx_actual,
            yaw_rate_actual=yaw_rate_actual,
            battery_voltage=battery_voltage,
            height_actual=height_actual
        )
    
    def __str__(self):
        mode_str = MotionMode(self.mode).name if self.mode < 6 else str(self.mode)
        status_str = SystemStatus(self.status).name if self.status < 6 else str(self.status)
        return (f"Status[t={self.timestamp}ms mode={mode_str} status={status_str} "
                f"pitch={self.pitch:.1f}° roll={self.roll:.1f}° yaw={self.yaw:.1f}° "
                f"vx={self.vx_actual:.3f}m/s batt={self.battery_voltage:.2f}V]")


@dataclass
class IMUReport:
    """IMU 上报数据"""
    timestamp: int = 0
    pitch: float = 0.0
    roll: float = 0.0
    yaw: float = 0.0
    pitch_rate: float = 0.0
    roll_rate: float = 0.0
    yaw_rate: float = 0.0
    accel_x: float = 0.0
    accel_y: float = 0.0
    accel_z: float = 0.0
    
    @classmethod
    def from_bytes(cls, data: bytes) -> 'IMUReport':
        """从字节数据解析 (大端序)"""
        if len(data) < 40:
            raise ValueError(f"Data too short: {len(data)} < 40")
        
        values = struct.unpack('>Ifffffffff', data[0:40])
        return cls(
            timestamp=values[0],
            pitch=values[1],
            roll=values[2],
            yaw=values[3],
            pitch_rate=values[4],
            roll_rate=values[5],
            yaw_rate=values[6],
            accel_x=values[7],
            accel_y=values[8],
            accel_z=values[9]
        )


@dataclass
class HeartbeatAck:
    """心跳响应"""
    timestamp: int = 0
    esp_time: int = 0
    cpu_load: int = 0
    status: int = 0
    rtt_ms: float = 0.0
    
    @classmethod
    def from_bytes(cls, data: bytes, send_time: int) -> 'HeartbeatAck':
        if len(data) < 10:
            raise ValueError(f"Data too short: {len(data)} < 10")
        
        timestamp, esp_time = struct.unpack('>II', data[0:8])
        cpu_load = data[8]
        status = data[9]
        
        # 计算 RTT (都是截断后的 32 位值)
        now_ms = int(time.time() * 1000) & 0xFFFFFFFF
        rtt_ms = (now_ms - send_time) & 0xFFFFFFFF  # 处理回绕
        if rtt_ms > 0x7FFFFFFF:  # 如果差值为负，修正
            rtt_ms = 0
        
        return cls(
            timestamp=timestamp,
            esp_time=esp_time,
            cpu_load=cpu_load,
            status=status,
            rtt_ms=rtt_ms
        )


# ============================================================================
# CRC16-CCITT 计算
# ============================================================================

def crc16_ccitt(data: bytes) -> int:
    """计算 CRC16-CCITT (多项式 0x1021, 初值 0xFFFF)"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = (crc << 1) ^ 0x1021
            else:
                crc <<= 1
            crc &= 0xFFFF
    return crc


# ============================================================================
# 帧解析器
# ============================================================================

class FrameParser:
    """帧解析器"""
    
    HEADER = bytes([0xAA, 0x55])
    
    def __init__(self):
        self.buffer = bytearray()
        self.state = 'WAIT_HEADER_0'
        self.expected_len = 0
    
    def reset(self):
        self.buffer.clear()
        self.state = 'WAIT_HEADER_0'
        self.expected_len = 0
    
    def feed(self, data: bytes) -> list:
        """输入数据，返回解析出的帧列表 [(seq, cmd, data), ...]"""
        frames = []
        
        for byte in data:
            frame = self._feed_byte(byte)
            if frame:
                frames.append(frame)
        
        return frames
    
    def _feed_byte(self, byte: int) -> Optional[tuple]:
        if self.state == 'WAIT_HEADER_0':
            if byte == 0xAA:
                self.buffer = bytearray([byte])
                self.state = 'WAIT_HEADER_1'
                
        elif self.state == 'WAIT_HEADER_1':
            if byte == 0x55:
                self.buffer.append(byte)
                self.state = 'WAIT_LEN'
            elif byte == 0xAA:
                self.buffer = bytearray([byte])
            else:
                self.reset()
                
        elif self.state == 'WAIT_LEN':
            if 2 <= byte <= 252:
                self.buffer.append(byte)
                self.expected_len = byte
                self.state = 'WAIT_DATA'
            else:
                self.reset()
                
        elif self.state == 'WAIT_DATA':
            self.buffer.append(byte)
            # HEAD(2) + LEN(1) + DATA(expected_len) + CRC(2)
            if len(self.buffer) >= 3 + self.expected_len + 2:
                return self._parse_frame()
        
        return None
    
    def _parse_frame(self) -> Optional[tuple]:
        """解析完整帧"""
        try:
            length = self.buffer[2]
            payload = self.buffer[3:3+length]
            crc_recv = struct.unpack('>H', self.buffer[3+length:5+length])[0]
            
            # 验证 CRC
            crc_calc = crc16_ccitt(payload)
            if crc_recv != crc_calc:
                logger.warning(f"CRC error: recv={crc_recv:04X} calc={crc_calc:04X}")
                return None
            
            seq = payload[0]
            cmd = payload[1]
            data = bytes(payload[2:]) if len(payload) > 2 else b''
            
            return (seq, cmd, data)
            
        finally:
            self.reset()


# ============================================================================
# ESP32 协议类
# ============================================================================

class ESP32Protocol:
    """ESP32 通信协议主类"""
    
    def __init__(self, port: str = '/dev/ttyUSB0', baudrate: int = 115200):
        """
        初始化
        
        Args:
            port: 串口设备路径
            baudrate: 波特率
        """
        self.port = port
        self.baudrate = baudrate
        self.ser: Optional[serial.Serial] = None
        self.parser = FrameParser()
        self.seq = 0
        self.connected = False
        self.running = False
        
        # 回调
        self.callbacks: Dict[int, Callable] = {}
        
        # 状态
        self.last_status: Optional[StatusReport] = None
        self.last_heartbeat_time = 0
        self.heartbeat_send_time = 0
        
        # 线程
        self.rx_thread: Optional[threading.Thread] = None
        self.hb_thread: Optional[threading.Thread] = None
        
        # 同步
        self.lock = threading.Lock()
        self.ack_event = threading.Event()
        self.ack_data: Optional[tuple] = None
        
        # 统计
        self.stats = {
            'tx_frames': 0,
            'rx_frames': 0,
            'crc_errors': 0,
            'timeouts': 0,
        }
    
    def connect(self) -> bool:
        """连接串口"""
        try:
            self.ser = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=0.1
            )
            self.running = True
            
            # 启动接收线程
            self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
            self.rx_thread.start()
            
            # 启动心跳线程
            self.hb_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
            self.hb_thread.start()
            
            logger.info(f"Connected to {self.port} @ {self.baudrate} bps")
            return True
            
        except Exception as e:
            logger.error(f"Failed to connect: {e}")
            return False
    
    def disconnect(self):
        """断开连接"""
        self.running = False
        self.connected = False
        
        if self.rx_thread:
            self.rx_thread.join(timeout=1.0)
        if self.hb_thread:
            self.hb_thread.join(timeout=1.0)
        
        if self.ser:
            self.ser.close()
            self.ser = None
        
        logger.info("Disconnected")
    
    # ========== 注册回调 ==========
    
    def on_status(self, callback: Callable[[StatusReport], None]):
        """注册状态回调"""
        self.callbacks[CMD.STATUS_REPORT] = callback
    
    def on_imu(self, callback: Callable[[IMUReport], None]):
        """注册 IMU 回调"""
        self.callbacks[CMD.IMU_REPORT] = callback
    
    def on_error(self, callback: Callable[[int, int, int, str], None]):
        """注册错误回调 (error_code, severity, source, message)"""
        self.callbacks[CMD.ERROR_REPORT] = callback
    
    def on_heartbeat_ack(self, callback: Callable[[HeartbeatAck], None]):
        """注册心跳响应回调"""
        self.callbacks[CMD.HEARTBEAT_ACK] = callback
    
    # ========== 控制命令 ==========
    
    def send_velocity(self, vx: float, yaw_rate: float, priority: int = 0, flags: int = 0):
        """
        发送速度指令
        
        Args:
            vx: 前进速度 (m/s), 正=前进
            yaw_rate: 偏航角速度 (rad/s), 正=左转
            priority: 优先级 (0-255)
            flags: 标志位
        """
        data = struct.pack('>ffBB', vx, yaw_rate, priority, flags)
        self._send_frame(CMD.SET_VELOCITY, data)
    
    def send_mode(self, mode: int, sub_mode: int = 0) -> bool:
        """设置运动模式"""
        data = struct.pack('>BB', mode, sub_mode)
        return self._send_with_ack(CMD.SET_MODE, data)
    
    def send_height(self, height: float, duration: float = 0.0) -> bool:
        """设置腿部高度"""
        data = struct.pack('>ff', height, duration)
        return self._send_with_ack(CMD.SET_HEIGHT, data)
    
    def send_pitch(self, pitch: float, duration: float = 0.0) -> bool:
        """设置目标 Pitch 角度"""
        data = struct.pack('>ff', pitch, duration)
        return self._send_with_ack(CMD.SET_PITCH, data)
    
    def send_roll(self, roll: float, duration: float = 0.0) -> bool:
        """设置目标 Roll 角度"""
        data = struct.pack('>ff', roll, duration)
        return self._send_with_ack(CMD.SET_ROLL, data)
    
    def send_pose(self, pitch: float, roll: float, height: float, duration: float = 0.0) -> bool:
        """设置完整姿态"""
        data = struct.pack('>ffff', pitch, roll, height, duration)
        return self._send_with_ack(CMD.SET_POSE, data)
    
    def send_motor_enable(self, enable: bool) -> bool:
        """电机使能"""
        data = struct.pack('>B', 1 if enable else 0)
        return self._send_with_ack(CMD.MOTOR_ENABLE, data)
    
    def send_emergency_stop(self, reason: int = 0) -> bool:
        """紧急停止"""
        data = struct.pack('>B', reason)
        return self._send_with_ack(CMD.EMERGENCY_STOP, data)
    
    def send_handshake(self, protocol_ver: int = 1, capabilities: int = 0) -> bool:
        """发送握手请求"""
        data = struct.pack('>BI', protocol_ver, capabilities)
        return self._send_with_ack(CMD.HANDSHAKE, data, ack_cmd=CMD.HANDSHAKE_ACK)
    
    def request_status(self):
        """请求状态"""
        self._send_frame(CMD.GET_STATUS, b'')
    
    # ========== 内部方法 ==========
    
    def _send_frame(self, cmd: int, data: bytes):
        """发送帧"""
        if not self.ser:
            return
        
        with self.lock:
            self.seq = (self.seq + 1) & 0xFF
            seq = self.seq
        
        # 构建 payload: SEQ + CMD + DATA
        payload = bytes([seq, cmd]) + data
        
        # 计算 CRC
        crc = crc16_ccitt(payload)
        
        # 构建帧: HEAD + LEN + PAYLOAD + CRC
        frame = bytes([0xAA, 0x55, len(payload)]) + payload + struct.pack('>H', crc)
        
        try:
            self.ser.write(frame)
            self.stats['tx_frames'] += 1
            logger.debug(f"TX: SEQ={seq:02X} CMD={cmd:02X} LEN={len(data)}")
        except Exception as e:
            logger.error(f"Send error: {e}")
    
    def _send_with_ack(self, cmd: int, data: bytes, ack_cmd: int = CMD.ACK, 
                       timeout: float = 0.5) -> bool:
        """发送并等待 ACK"""
        self.ack_event.clear()
        self.ack_data = None
        
        with self.lock:
            self.seq = (self.seq + 1) & 0xFF
            expected_seq = self.seq
        
        # 发送
        payload = bytes([expected_seq, cmd]) + data
        crc = crc16_ccitt(payload)
        frame = bytes([0xAA, 0x55, len(payload)]) + payload + struct.pack('>H', crc)
        
        try:
            self.ser.write(frame)
            self.stats['tx_frames'] += 1
        except Exception as e:
            logger.error(f"Send error: {e}")
            return False
        
        # 等待 ACK
        if self.ack_event.wait(timeout):
            if self.ack_data and self.ack_data[0] == expected_seq:
                return True
        
        self.stats['timeouts'] += 1
        logger.warning(f"ACK timeout for CMD={cmd:02X}")
        return False
    
    def _rx_loop(self):
        """接收线程"""
        while self.running:
            try:
                data = self.ser.read(256)
                if data:
                    frames = self.parser.feed(data)
                    for seq, cmd, payload in frames:
                        self.stats['rx_frames'] += 1
                        self._handle_frame(seq, cmd, payload)
            except Exception as e:
                if self.running:
                    logger.error(f"RX error: {e}")
    
    def _handle_frame(self, seq: int, cmd: int, data: bytes):
        """处理接收到的帧"""
        logger.debug(f"RX: SEQ={seq:02X} CMD={cmd:02X} LEN={len(data)}")
        
        # ACK/NACK
        if cmd == CMD.ACK:
            self.ack_data = (data[0], data[1]) if len(data) >= 2 else None
            self.ack_event.set()
            return
        
        if cmd == CMD.NACK:
            logger.warning(f"NACK received: seq={data[0]:02X} cmd={data[1]:02X} err={data[2]:02X}")
            self.ack_data = None
            self.ack_event.set()
            return
        
        # 心跳响应
        if cmd == CMD.HEARTBEAT_ACK:
            self.connected = True
            self.last_heartbeat_time = time.time()
            try:
                ack = HeartbeatAck.from_bytes(data, self.heartbeat_send_time)
                if CMD.HEARTBEAT_ACK in self.callbacks:
                    self.callbacks[CMD.HEARTBEAT_ACK](ack)
            except Exception as e:
                logger.error(f"Parse heartbeat ack error: {e}")
            
            # 握手响应也算连接成功
            self.ack_data = (seq, cmd)
            self.ack_event.set()
            return
        
        # 握手响应
        if cmd == CMD.HANDSHAKE_ACK:
            self.connected = True
            self.last_heartbeat_time = time.time()
            logger.info("Handshake successful")
            self.ack_data = (seq, cmd)
            self.ack_event.set()
            return
        
        # 状态上报
        if cmd == CMD.STATUS_REPORT:
            try:
                status = StatusReport.from_bytes(data)
                self.last_status = status
                if CMD.STATUS_REPORT in self.callbacks:
                    self.callbacks[CMD.STATUS_REPORT](status)
            except Exception as e:
                logger.error(f"Parse status error: {e}")
            return
        
        # IMU 上报
        if cmd == CMD.IMU_REPORT:
            try:
                imu = IMUReport.from_bytes(data)
                if CMD.IMU_REPORT in self.callbacks:
                    self.callbacks[CMD.IMU_REPORT](imu)
            except Exception as e:
                logger.error(f"Parse IMU error: {e}")
            return
        
        # 错误上报
        if cmd == CMD.ERROR_REPORT:
            try:
                if len(data) >= 7:
                    timestamp, = struct.unpack('>I', data[0:4])
                    error_code = data[4]
                    severity = data[5]
                    source = data[6]
                    message = data[7:39].decode('utf-8', errors='ignore').rstrip('\x00')
                    logger.warning(f"Error from ESP32: code={error_code} severity={severity} "
                                   f"source={source} msg={message}")
                    if CMD.ERROR_REPORT in self.callbacks:
                        self.callbacks[CMD.ERROR_REPORT](error_code, severity, source, message)
            except Exception as e:
                logger.error(f"Parse error report: {e}")
            return
    
    def _heartbeat_loop(self):
        """心跳线程"""
        while self.running:
            try:
                # 发送心跳 (时间戳截断为 32 位)
                self.heartbeat_send_time = int(time.time() * 1000) & 0xFFFFFFFF
                data = struct.pack('>I', self.heartbeat_send_time)
                self._send_frame(CMD.HEARTBEAT, data)
                
                time.sleep(0.1)  # 100ms 间隔
                
                # 检查连接超时
                if self.connected and (time.time() - self.last_heartbeat_time) > 0.5:
                    logger.warning("Connection lost (heartbeat timeout)")
                    self.connected = False
                    
            except Exception as e:
                if self.running:
                    logger.error(f"Heartbeat error: {e}")


# ============================================================================
# 测试/示例
# ============================================================================

def main():
    """测试主函数"""
    import argparse
    
    parser = argparse.ArgumentParser(description='ESP32 Protocol Test')
    parser.add_argument('-p', '--port', default='/dev/ttyUSB0', help='Serial port')
    parser.add_argument('-b', '--baud', type=int, default=115200, help='Baudrate')
    args = parser.parse_args()
    
    # 创建协议实例
    esp = ESP32Protocol(port=args.port, baudrate=args.baud)
    
    # 注册回调
    def on_status(status: StatusReport):
        print(f"[STATUS] {status}")
    
    def on_heartbeat(ack: HeartbeatAck):
        print(f"[HB] RTT={ack.rtt_ms:.1f}ms ESP_time={ack.esp_time}ms")
    
    esp.on_status(on_status)
    esp.on_heartbeat_ack(on_heartbeat)
    
    # 连接
    if not esp.connect():
        print("Failed to connect!")
        return
    
    print(f"Connected to {args.port}")
    print("Commands: v=velocity, m=mode, h=height, p=pitch, r=roll, e=estop, q=quit")
    
    try:
        while True:
            cmd = input("> ").strip().lower()
            
            if cmd == 'q':
                break
            elif cmd == 'v':
                vx = float(input("  vx (m/s): "))
                yr = float(input("  yaw_rate (rad/s): "))
                esp.send_velocity(vx, yr)
                print("  Sent velocity")
            elif cmd == 'm':
                mode = int(input("  mode (0=idle, 1=stand, 2=walk): "))
                if esp.send_mode(mode):
                    print("  Mode set OK")
                else:
                    print("  Mode set FAILED")
            elif cmd == 'h':
                height = float(input("  height (m): "))
                if esp.send_height(height):
                    print("  Height set OK")
                else:
                    print("  Height set FAILED")
            elif cmd == 'p':
                pitch = float(input("  pitch (deg): "))
                if esp.send_pitch(pitch):
                    print("  Pitch set OK")
                else:
                    print("  Pitch set FAILED")
            elif cmd == 'r':
                roll = float(input("  roll (deg): "))
                if esp.send_roll(roll):
                    print("  Roll set OK")
                else:
                    print("  Roll set FAILED")
            elif cmd == 'e':
                if esp.send_emergency_stop():
                    print("  EMERGENCY STOP sent")
                else:
                    print("  ESTOP FAILED")
            elif cmd == 's':
                if esp.last_status:
                    print(f"  {esp.last_status}")
                else:
                    print("  No status received yet")
            elif cmd == '?':
                print("Commands: v=velocity, m=mode, h=height, p=pitch, r=roll, e=estop, s=status, q=quit")
            else:
                print("Unknown command. Type '?' for help")
                
    except KeyboardInterrupt:
        print("\nInterrupted")
    finally:
        esp.disconnect()
        print("Disconnected")


if __name__ == '__main__':
    main()
