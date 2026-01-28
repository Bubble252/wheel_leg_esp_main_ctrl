#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pi-ESP32 通信调试工具 - Qt GUI
==============================
用于树莓派与ESP32轮腿机器人之间的二进制协议通信调试

功能:
- 速度控制 (vx, yaw_rate)
- 高度控制 (leg_length)
- 姿态控制 (pitch, roll)
- 预设姿态 (pose)
- 实时状态显示 (IMU, 电池, 温度等)
- 实时波形显示

协议: 二进制帧格式, 大端序, CRC16-CCITT

作者: Bubble
日期: 2025-01-23
"""

import sys
import os
import struct
import time
from datetime import datetime
from collections import deque
from threading import Lock

# 添加当前目录到路径
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QLabel, QPushButton, QComboBox, QDoubleSpinBox,
    QSpinBox, QTextEdit, QTabWidget, QGridLayout, QMessageBox,
    QSplitter, QFrame, QSlider, QCheckBox, QSizePolicy
)
from PyQt5.QtCore import Qt, QThread, pyqtSignal, QTimer
from PyQt5.QtGui import QFont, QColor, QPalette

import pyqtgraph as pg
import serial
import serial.tools.list_ports

# 导入协议库
from esp32_protocol import ESP32Protocol, ProtocolConstants


# ============================================================================
# 串口通信线程
# ============================================================================
class SerialThread(QThread):
    """串口数据接收线程"""
    
    # 信号定义
    data_received = pyqtSignal(bytes)      # 原始数据
    frame_received = pyqtSignal(dict)       # 解析后的帧
    connection_status = pyqtSignal(bool)    # 连接状态
    error_occurred = pyqtSignal(str)        # 错误信息
    
    def __init__(self):
        super().__init__()
        self.protocol = None
        self.running = False
        self.serial = None
        self.send_lock = Lock()
    
    def connect_serial(self, port, baudrate=115200):
        """连接串口"""
        try:
            self.serial = serial.Serial(
                port=port,
                baudrate=baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=0.1
            )
            self.protocol = ESP32Protocol(port, baudrate, connect=False)
            self.protocol.serial = self.serial
            
            # 注册回调
            self.protocol.register_callback('state', self._on_state)
            self.protocol.register_callback('heartbeat_ack', self._on_heartbeat_ack)
            self.protocol.register_callback('param_response', self._on_param_response)
            
            self.running = True
            self.connection_status.emit(True)
            return True
        except Exception as e:
            self.error_occurred.emit(f"连接失败: {str(e)}")
            return False
    
    def disconnect_serial(self):
        """断开串口"""
        self.running = False
        if self.serial and self.serial.is_open:
            self.serial.close()
        self.serial = None
        self.protocol = None
        self.connection_status.emit(False)
    
    def run(self):
        """线程主循环"""
        while self.running:
            if self.serial and self.serial.is_open:
                try:
                    if self.serial.in_waiting > 0:
                        data = self.serial.read(self.serial.in_waiting)
                        self.data_received.emit(data)
                        if self.protocol:
                            self.protocol._process_rx_data(data)
                except Exception as e:
                    self.error_occurred.emit(f"读取错误: {str(e)}")
            self.msleep(10)
    
    def send_frame(self, cmd, data=b''):
        """发送数据帧"""
        if not self.serial or not self.serial.is_open:
            return False
        
        try:
            with self.send_lock:
                if self.protocol:
                    frame = self.protocol._build_frame(cmd, data)
                    self.serial.write(frame)
                    return True
        except Exception as e:
            self.error_occurred.emit(f"发送错误: {str(e)}")
        return False
    
    def _on_state(self, state):
        """收到状态数据回调"""
        self.frame_received.emit({'type': 'state', 'data': state})
    
    def _on_heartbeat_ack(self, timestamp):
        """收到心跳响应回调"""
        self.frame_received.emit({'type': 'heartbeat_ack', 'data': timestamp})
    
    def _on_param_response(self, param_id, success, value):
        """收到参数响应回调"""
        self.frame_received.emit({
            'type': 'param_response',
            'data': {'param_id': param_id, 'success': success, 'value': value}
        })


# ============================================================================
# 控制面板 - 速度控制
# ============================================================================
class VelocityControlPanel(QWidget):
    """速度控制面板"""
    
    command_requested = pyqtSignal(int, bytes)  # cmd, data
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # 速度参数
        param_group = QGroupBox("🚀 速度控制")
        param_layout = QGridLayout()
        param_layout.setSpacing(15)
        
        # Vx (前进速度)
        param_layout.addWidget(QLabel("Vx (m/s):"), 0, 0)
        self.vx_input = QDoubleSpinBox()
        self.vx_input.setRange(-2.0, 2.0)
        self.vx_input.setDecimals(2)
        self.vx_input.setSingleStep(0.1)
        self.vx_input.setValue(0.0)
        self.vx_input.setStyleSheet("font-size: 14px; padding: 5px;")
        param_layout.addWidget(self.vx_input, 0, 1)
        
        # Vx 滑块
        self.vx_slider = QSlider(Qt.Horizontal)
        self.vx_slider.setRange(-200, 200)
        self.vx_slider.setValue(0)
        self.vx_slider.valueChanged.connect(lambda v: self.vx_input.setValue(v / 100.0))
        self.vx_input.valueChanged.connect(lambda v: self.vx_slider.setValue(int(v * 100)))
        param_layout.addWidget(self.vx_slider, 0, 2)
        
        # Yaw Rate (转向角速度)
        param_layout.addWidget(QLabel("Yaw Rate (rad/s):"), 1, 0)
        self.yaw_rate_input = QDoubleSpinBox()
        self.yaw_rate_input.setRange(-3.14, 3.14)
        self.yaw_rate_input.setDecimals(2)
        self.yaw_rate_input.setSingleStep(0.1)
        self.yaw_rate_input.setValue(0.0)
        self.yaw_rate_input.setStyleSheet("font-size: 14px; padding: 5px;")
        param_layout.addWidget(self.yaw_rate_input, 1, 1)
        
        # Yaw Rate 滑块
        self.yaw_slider = QSlider(Qt.Horizontal)
        self.yaw_slider.setRange(-314, 314)
        self.yaw_slider.setValue(0)
        self.yaw_slider.valueChanged.connect(lambda v: self.yaw_rate_input.setValue(v / 100.0))
        self.yaw_rate_input.valueChanged.connect(lambda v: self.yaw_slider.setValue(int(v * 100)))
        param_layout.addWidget(self.yaw_slider, 1, 2)
        
        param_group.setLayout(param_layout)
        layout.addWidget(param_group)
        
        # 操作按钮
        btn_layout = QHBoxLayout()
        
        self.send_btn = QPushButton("📤 发送速度")
        self.send_btn.setStyleSheet("font-size: 14px; font-weight: bold; padding: 10px; background-color: #4CAF50; color: white;")
        self.send_btn.clicked.connect(self.send_velocity)
        btn_layout.addWidget(self.send_btn)
        
        self.stop_btn = QPushButton("🛑 急停")
        self.stop_btn.setStyleSheet("font-size: 14px; font-weight: bold; padding: 10px; background-color: #f44336; color: white;")
        self.stop_btn.clicked.connect(self.emergency_stop)
        btn_layout.addWidget(self.stop_btn)
        
        self.reset_btn = QPushButton("🔄 归零")
        self.reset_btn.setStyleSheet("font-size: 14px; padding: 10px;")
        self.reset_btn.clicked.connect(self.reset_values)
        btn_layout.addWidget(self.reset_btn)
        
        layout.addLayout(btn_layout)
        layout.addStretch()
    
    def send_velocity(self):
        """发送速度命令"""
        vx = self.vx_input.value()
        yaw_rate = self.yaw_rate_input.value()
        # 大端序打包: 两个float32
        data = struct.pack('>ff', vx, yaw_rate)
        self.command_requested.emit(ProtocolConstants.CMD_SET_VELOCITY, data)
    
    def emergency_stop(self):
        """紧急停止"""
        self.vx_input.setValue(0)
        self.yaw_rate_input.setValue(0)
        data = struct.pack('>ff', 0.0, 0.0)
        self.command_requested.emit(ProtocolConstants.CMD_SET_VELOCITY, data)
        # 同时发送紧急停止命令
        self.command_requested.emit(ProtocolConstants.CMD_EMERGENCY_STOP, b'')
    
    def reset_values(self):
        """重置数值"""
        self.vx_input.setValue(0)
        self.yaw_rate_input.setValue(0)


# ============================================================================
# 控制面板 - 高度控制
# ============================================================================
class HeightControlPanel(QWidget):
    """高度控制面板"""
    
    command_requested = pyqtSignal(int, bytes)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        param_group = QGroupBox("📏 高度控制")
        param_layout = QGridLayout()
        param_layout.setSpacing(15)
        
        # 腿长度
        param_layout.addWidget(QLabel("腿长度 (m):"), 0, 0)
        self.height_input = QDoubleSpinBox()
        self.height_input.setRange(0.10, 0.20)
        self.height_input.setDecimals(3)
        self.height_input.setSingleStep(0.005)
        self.height_input.setValue(0.14)
        self.height_input.setStyleSheet("font-size: 14px; padding: 5px;")
        param_layout.addWidget(self.height_input, 0, 1)
        
        # 高度滑块
        self.height_slider = QSlider(Qt.Horizontal)
        self.height_slider.setRange(100, 200)  # 0.10 ~ 0.20m
        self.height_slider.setValue(140)
        self.height_slider.valueChanged.connect(lambda v: self.height_input.setValue(v / 1000.0))
        self.height_input.valueChanged.connect(lambda v: self.height_slider.setValue(int(v * 1000)))
        param_layout.addWidget(self.height_slider, 0, 2)
        
        param_group.setLayout(param_layout)
        layout.addWidget(param_group)
        
        # 预设高度按钮
        preset_group = QGroupBox("预设高度")
        preset_layout = QHBoxLayout()
        
        presets = [
            ("低姿态", 0.11),
            ("中姿态", 0.14),
            ("高姿态", 0.17),
        ]
        
        for name, value in presets:
            btn = QPushButton(name)
            btn.clicked.connect(lambda checked, v=value: self.set_height(v))
            preset_layout.addWidget(btn)
        
        preset_group.setLayout(preset_layout)
        layout.addWidget(preset_group)
        
        # 发送按钮
        btn_layout = QHBoxLayout()
        
        self.send_btn = QPushButton("📤 发送高度")
        self.send_btn.setStyleSheet("font-size: 14px; font-weight: bold; padding: 10px; background-color: #2196F3; color: white;")
        self.send_btn.clicked.connect(self.send_height)
        btn_layout.addWidget(self.send_btn)
        
        layout.addLayout(btn_layout)
        layout.addStretch()
    
    def set_height(self, value):
        """设置高度值"""
        self.height_input.setValue(value)
    
    def send_height(self):
        """发送高度命令"""
        height = self.height_input.value()
        data = struct.pack('>f', height)
        self.command_requested.emit(ProtocolConstants.CMD_SET_HEIGHT, data)


# ============================================================================
# 控制面板 - 姿态控制 (Pitch/Roll)
# ============================================================================
class AttitudeControlPanel(QWidget):
    """姿态控制面板 (Pitch/Roll)"""
    
    command_requested = pyqtSignal(int, bytes)
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # Pitch 控制
        pitch_group = QGroupBox("📐 Pitch (俯仰) 控制")
        pitch_layout = QGridLayout()
        pitch_layout.setSpacing(10)
        
        pitch_layout.addWidget(QLabel("Pitch (度):"), 0, 0)
        self.pitch_input = QDoubleSpinBox()
        self.pitch_input.setRange(-30.0, 30.0)
        self.pitch_input.setDecimals(1)
        self.pitch_input.setSingleStep(1.0)
        self.pitch_input.setValue(0.0)
        self.pitch_input.setStyleSheet("font-size: 14px; padding: 5px;")
        pitch_layout.addWidget(self.pitch_input, 0, 1)
        
        self.pitch_slider = QSlider(Qt.Horizontal)
        self.pitch_slider.setRange(-300, 300)
        self.pitch_slider.setValue(0)
        self.pitch_slider.valueChanged.connect(lambda v: self.pitch_input.setValue(v / 10.0))
        self.pitch_input.valueChanged.connect(lambda v: self.pitch_slider.setValue(int(v * 10)))
        pitch_layout.addWidget(self.pitch_slider, 0, 2)
        
        self.send_pitch_btn = QPushButton("发送 Pitch")
        self.send_pitch_btn.clicked.connect(self.send_pitch)
        pitch_layout.addWidget(self.send_pitch_btn, 0, 3)
        
        pitch_group.setLayout(pitch_layout)
        layout.addWidget(pitch_group)
        
        # Roll 控制
        roll_group = QGroupBox("📐 Roll (横滚) 控制")
        roll_layout = QGridLayout()
        roll_layout.setSpacing(10)
        
        roll_layout.addWidget(QLabel("Roll (度):"), 0, 0)
        self.roll_input = QDoubleSpinBox()
        self.roll_input.setRange(-30.0, 30.0)
        self.roll_input.setDecimals(1)
        self.roll_input.setSingleStep(1.0)
        self.roll_input.setValue(0.0)
        self.roll_input.setStyleSheet("font-size: 14px; padding: 5px;")
        roll_layout.addWidget(self.roll_input, 0, 1)
        
        self.roll_slider = QSlider(Qt.Horizontal)
        self.roll_slider.setRange(-300, 300)
        self.roll_slider.setValue(0)
        self.roll_slider.valueChanged.connect(lambda v: self.roll_input.setValue(v / 10.0))
        self.roll_input.valueChanged.connect(lambda v: self.roll_slider.setValue(int(v * 10)))
        roll_layout.addWidget(self.roll_slider, 0, 2)
        
        self.send_roll_btn = QPushButton("发送 Roll")
        self.send_roll_btn.clicked.connect(self.send_roll)
        roll_layout.addWidget(self.send_roll_btn, 0, 3)
        
        roll_group.setLayout(roll_layout)
        layout.addWidget(roll_group)
        
        # 快捷操作
        action_layout = QHBoxLayout()
        
        self.reset_btn = QPushButton("🔄 姿态归零")
        self.reset_btn.setStyleSheet("font-size: 14px; padding: 10px;")
        self.reset_btn.clicked.connect(self.reset_attitude)
        action_layout.addWidget(self.reset_btn)
        
        self.send_both_btn = QPushButton("📤 发送 Pitch+Roll")
        self.send_both_btn.setStyleSheet("font-size: 14px; font-weight: bold; padding: 10px; background-color: #9C27B0; color: white;")
        self.send_both_btn.clicked.connect(self.send_both)
        action_layout.addWidget(self.send_both_btn)
        
        layout.addLayout(action_layout)
        layout.addStretch()
    
    def send_pitch(self):
        """发送Pitch命令"""
        pitch_deg = self.pitch_input.value()
        pitch_rad = pitch_deg * 3.14159265 / 180.0  # 转换为弧度
        data = struct.pack('>f', pitch_rad)
        self.command_requested.emit(ProtocolConstants.CMD_SET_PITCH, data)
    
    def send_roll(self):
        """发送Roll命令"""
        roll_deg = self.roll_input.value()
        roll_rad = roll_deg * 3.14159265 / 180.0
        data = struct.pack('>f', roll_rad)
        self.command_requested.emit(ProtocolConstants.CMD_SET_ROLL, data)
    
    def send_both(self):
        """同时发送Pitch和Roll"""
        self.send_pitch()
        self.send_roll()
    
    def reset_attitude(self):
        """重置姿态"""
        self.pitch_input.setValue(0)
        self.roll_input.setValue(0)
        self.send_both()


# ============================================================================
# 控制面板 - 预设姿态
# ============================================================================
class PoseControlPanel(QWidget):
    """预设姿态控制面板"""
    
    command_requested = pyqtSignal(int, bytes)
    
    # 预设姿态定义
    POSES = {
        0: ("站立", "🧍", "#4CAF50"),
        1: ("蹲下", "🧎", "#2196F3"),
        2: ("前倾", "↗️", "#FF9800"),
        3: ("后仰", "↙️", "#FF5722"),
        4: ("左倾", "⬅️", "#9C27B0"),
        5: ("右倾", "➡️", "#E91E63"),
        6: ("准备跳跃", "🦘", "#00BCD4"),
    }
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        pose_group = QGroupBox("🎭 预设姿态")
        pose_layout = QGridLayout()
        pose_layout.setSpacing(10)
        
        row, col = 0, 0
        for pose_id, (name, icon, color) in self.POSES.items():
            btn = QPushButton(f"{icon} {name}")
            btn.setStyleSheet(f"""
                font-size: 14px; font-weight: bold; padding: 15px;
                background-color: {color}; color: white;
                border-radius: 5px;
            """)
            btn.clicked.connect(lambda checked, pid=pose_id: self.send_pose(pid))
            pose_layout.addWidget(btn, row, col)
            col += 1
            if col > 2:
                col = 0
                row += 1
        
        pose_group.setLayout(pose_layout)
        layout.addWidget(pose_group)
        
        # 当前姿态显示
        status_group = QGroupBox("当前姿态")
        status_layout = QHBoxLayout()
        
        self.current_pose_label = QLabel("未知")
        self.current_pose_label.setStyleSheet(
            "font-size: 18px; font-weight: bold; color: #00ff00; "
            "background-color: #2b2b2b; padding: 10px; border-radius: 5px;"
        )
        self.current_pose_label.setAlignment(Qt.AlignCenter)
        status_layout.addWidget(self.current_pose_label)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        layout.addStretch()
    
    def send_pose(self, pose_id):
        """发送姿态命令"""
        data = struct.pack('>B', pose_id)
        self.command_requested.emit(ProtocolConstants.CMD_SET_POSE, data)
        
        # 更新显示
        if pose_id in self.POSES:
            name, icon, _ = self.POSES[pose_id]
            self.current_pose_label.setText(f"{icon} {name}")


# ============================================================================
# 状态显示面板
# ============================================================================
class StatusDisplayPanel(QWidget):
    """机器人状态显示面板"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # IMU 数据
        imu_group = QGroupBox("🧭 IMU 数据")
        imu_layout = QGridLayout()
        imu_layout.setSpacing(10)
        
        # Pitch
        imu_layout.addWidget(QLabel("Pitch:"), 0, 0)
        self.pitch_label = self.create_value_label()
        imu_layout.addWidget(self.pitch_label, 0, 1)
        imu_layout.addWidget(QLabel("°"), 0, 2)
        
        # Roll
        imu_layout.addWidget(QLabel("Roll:"), 0, 3)
        self.roll_label = self.create_value_label()
        imu_layout.addWidget(self.roll_label, 0, 4)
        imu_layout.addWidget(QLabel("°"), 0, 5)
        
        # Yaw
        imu_layout.addWidget(QLabel("Yaw:"), 1, 0)
        self.yaw_label = self.create_value_label()
        imu_layout.addWidget(self.yaw_label, 1, 1)
        imu_layout.addWidget(QLabel("°"), 1, 2)
        
        # Yaw Rate
        imu_layout.addWidget(QLabel("Yaw Rate:"), 1, 3)
        self.yaw_rate_label = self.create_value_label()
        imu_layout.addWidget(self.yaw_rate_label, 1, 4)
        imu_layout.addWidget(QLabel("rad/s"), 1, 5)
        
        imu_group.setLayout(imu_layout)
        layout.addWidget(imu_group)
        
        # 运动状态
        motion_group = QGroupBox("🏃 运动状态")
        motion_layout = QGridLayout()
        
        motion_layout.addWidget(QLabel("速度 Vx:"), 0, 0)
        self.vx_label = self.create_value_label()
        motion_layout.addWidget(self.vx_label, 0, 1)
        motion_layout.addWidget(QLabel("m/s"), 0, 2)
        
        motion_layout.addWidget(QLabel("腿长度:"), 0, 3)
        self.leg_length_label = self.create_value_label()
        motion_layout.addWidget(self.leg_length_label, 0, 4)
        motion_layout.addWidget(QLabel("m"), 0, 5)
        
        motion_group.setLayout(motion_layout)
        layout.addWidget(motion_group)
        
        # 系统状态
        sys_group = QGroupBox("🔋 系统状态")
        sys_layout = QGridLayout()
        
        sys_layout.addWidget(QLabel("电池电压:"), 0, 0)
        self.battery_label = self.create_value_label()
        sys_layout.addWidget(self.battery_label, 0, 1)
        sys_layout.addWidget(QLabel("V"), 0, 2)
        
        sys_layout.addWidget(QLabel("温度:"), 0, 3)
        self.temp_label = self.create_value_label()
        sys_layout.addWidget(self.temp_label, 0, 4)
        sys_layout.addWidget(QLabel("℃"), 0, 5)
        
        sys_layout.addWidget(QLabel("控制状态:"), 1, 0)
        self.control_state_label = QLabel("--")
        self.control_state_label.setStyleSheet(
            "font-size: 14px; font-weight: bold; color: #ffaa00; padding: 5px;"
        )
        sys_layout.addWidget(self.control_state_label, 1, 1, 1, 2)
        
        sys_layout.addWidget(QLabel("错误码:"), 1, 3)
        self.error_label = self.create_value_label()
        sys_layout.addWidget(self.error_label, 1, 4, 1, 2)
        
        sys_group.setLayout(sys_layout)
        layout.addWidget(sys_group)
        
        layout.addStretch()
    
    def create_value_label(self):
        """创建数值显示标签"""
        label = QLabel("--")
        label.setStyleSheet(
            "font-size: 14px; font-weight: bold; color: #00ff00; "
            "background-color: #1e1e1e; padding: 5px; min-width: 60px;"
        )
        label.setAlignment(Qt.AlignCenter)
        return label
    
    def update_state(self, state):
        """更新状态显示"""
        if 'pitch' in state:
            self.pitch_label.setText(f"{state['pitch'] * 180 / 3.14159:.1f}")
        if 'roll' in state:
            self.roll_label.setText(f"{state['roll'] * 180 / 3.14159:.1f}")
        if 'yaw' in state:
            self.yaw_label.setText(f"{state['yaw'] * 180 / 3.14159:.1f}")
        if 'yaw_rate' in state:
            self.yaw_rate_label.setText(f"{state['yaw_rate']:.2f}")
        if 'velocity' in state:
            self.vx_label.setText(f"{state['velocity']:.2f}")
        if 'leg_length' in state:
            self.leg_length_label.setText(f"{state['leg_length']:.3f}")
        if 'battery_voltage' in state:
            self.battery_label.setText(f"{state['battery_voltage']:.1f}")
        if 'temperature' in state:
            self.temp_label.setText(f"{state['temperature']:.1f}")
        if 'control_state' in state:
            states = {0: '停止', 1: '运行', 2: '错误', 3: '初始化'}
            self.control_state_label.setText(states.get(state['control_state'], '未知'))
        if 'error_code' in state:
            self.error_label.setText(f"0x{state['error_code']:04X}")


# ============================================================================
# 波形显示面板
# ============================================================================
class WaveformPanel(QWidget):
    """波形显示面板"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.max_points = 500
        self.data_counter = 0
        
        # 数据缓冲
        self.time_data = deque(maxlen=self.max_points)
        self.pitch_data = deque(maxlen=self.max_points)
        self.roll_data = deque(maxlen=self.max_points)
        self.vx_data = deque(maxlen=self.max_points)
        self.yaw_rate_data = deque(maxlen=self.max_points)
        
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # 姿态波形
        attitude_group = QGroupBox("姿态波形 (Pitch/Roll)")
        attitude_layout = QVBoxLayout()
        
        self.attitude_plot = pg.PlotWidget()
        self.attitude_plot.setBackground('k')
        self.attitude_plot.showGrid(x=True, y=True, alpha=0.3)
        self.attitude_plot.setLabel('left', '角度 (°)')
        self.attitude_plot.addLegend()
        
        self.pitch_curve = self.attitude_plot.plot(pen=pg.mkPen(color='b', width=2), name='Pitch')
        self.roll_curve = self.attitude_plot.plot(pen=pg.mkPen(color='r', width=2), name='Roll')
        
        attitude_layout.addWidget(self.attitude_plot)
        attitude_group.setLayout(attitude_layout)
        layout.addWidget(attitude_group)
        
        # 运动波形
        motion_group = QGroupBox("运动波形 (Vx/Yaw Rate)")
        motion_layout = QVBoxLayout()
        
        self.motion_plot = pg.PlotWidget()
        self.motion_plot.setBackground('k')
        self.motion_plot.showGrid(x=True, y=True, alpha=0.3)
        self.motion_plot.setLabel('left', '值')
        self.motion_plot.addLegend()
        
        self.vx_curve = self.motion_plot.plot(pen=pg.mkPen(color='g', width=2), name='Vx (m/s)')
        self.yaw_rate_curve = self.motion_plot.plot(pen=pg.mkPen(color='y', width=2), name='Yaw Rate (rad/s)')
        
        motion_layout.addWidget(self.motion_plot)
        motion_group.setLayout(motion_layout)
        layout.addWidget(motion_group)
        
        # 控制按钮
        btn_layout = QHBoxLayout()
        
        self.clear_btn = QPushButton("清空波形")
        self.clear_btn.clicked.connect(self.clear_plots)
        btn_layout.addWidget(self.clear_btn)
        
        self.pause_btn = QPushButton("暂停")
        self.pause_btn.setCheckable(True)
        btn_layout.addWidget(self.pause_btn)
        
        btn_layout.addStretch()
        layout.addLayout(btn_layout)
    
    def update_data(self, state):
        """更新波形数据"""
        if self.pause_btn.isChecked():
            return
        
        self.data_counter += 1
        self.time_data.append(self.data_counter)
        
        pitch_deg = state.get('pitch', 0) * 180 / 3.14159
        roll_deg = state.get('roll', 0) * 180 / 3.14159
        vx = state.get('velocity', 0)
        yaw_rate = state.get('yaw_rate', 0)
        
        self.pitch_data.append(pitch_deg)
        self.roll_data.append(roll_deg)
        self.vx_data.append(vx)
        self.yaw_rate_data.append(yaw_rate)
        
        self.pitch_curve.setData(list(self.time_data), list(self.pitch_data))
        self.roll_curve.setData(list(self.time_data), list(self.roll_data))
        self.vx_curve.setData(list(self.time_data), list(self.vx_data))
        self.yaw_rate_curve.setData(list(self.time_data), list(self.yaw_rate_data))
    
    def clear_plots(self):
        """清空波形"""
        self.time_data.clear()
        self.pitch_data.clear()
        self.roll_data.clear()
        self.vx_data.clear()
        self.yaw_rate_data.clear()
        self.data_counter = 0
        
        self.pitch_curve.setData([], [])
        self.roll_curve.setData([], [])
        self.vx_curve.setData([], [])
        self.yaw_rate_curve.setData([], [])


# ============================================================================
# 主窗口
# ============================================================================
class PiCommMainWindow(QMainWindow):
    """主窗口"""
    
    def __init__(self):
        super().__init__()
        self.serial_thread = SerialThread()
        self.heartbeat_timer = QTimer()
        self.init_ui()
        self.connect_signals()
    
    def init_ui(self):
        self.setWindowTitle("Pi-ESP32 通信调试工具 v1.0")
        self.setMinimumSize(1200, 800)
        
        # 中央部件
        central = QWidget()
        self.setCentralWidget(central)
        main_layout = QVBoxLayout(central)
        
        # 顶部 - 连接控制
        self.create_connection_panel(main_layout)
        
        # 主分割器
        splitter = QSplitter(Qt.Horizontal)
        
        # 左侧 - 控制面板
        left_widget = QWidget()
        left_layout = QVBoxLayout(left_widget)
        left_layout.setContentsMargins(0, 0, 0, 0)
        
        self.control_tabs = QTabWidget()
        
        # 速度控制
        self.velocity_panel = VelocityControlPanel()
        self.control_tabs.addTab(self.velocity_panel, "🚀 速度")
        
        # 高度控制
        self.height_panel = HeightControlPanel()
        self.control_tabs.addTab(self.height_panel, "📏 高度")
        
        # 姿态控制
        self.attitude_panel = AttitudeControlPanel()
        self.control_tabs.addTab(self.attitude_panel, "📐 姿态")
        
        # 预设姿态
        self.pose_panel = PoseControlPanel()
        self.control_tabs.addTab(self.pose_panel, "🎭 预设")
        
        left_layout.addWidget(self.control_tabs)
        splitter.addWidget(left_widget)
        
        # 右侧 - 状态和波形
        right_widget = QWidget()
        right_layout = QVBoxLayout(right_widget)
        right_layout.setContentsMargins(0, 0, 0, 0)
        
        self.display_tabs = QTabWidget()
        
        # 状态显示
        self.status_panel = StatusDisplayPanel()
        self.display_tabs.addTab(self.status_panel, "📊 状态")
        
        # 波形显示
        self.waveform_panel = WaveformPanel()
        self.display_tabs.addTab(self.waveform_panel, "📈 波形")
        
        right_layout.addWidget(self.display_tabs)
        splitter.addWidget(right_widget)
        
        splitter.setSizes([400, 600])
        main_layout.addWidget(splitter)
        
        # 底部 - 日志
        self.create_log_panel(main_layout)
    
    def create_connection_panel(self, layout):
        """创建连接控制面板"""
        conn_group = QGroupBox("🔌 串口连接")
        conn_layout = QHBoxLayout()
        
        # 端口选择
        conn_layout.addWidget(QLabel("端口:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(150)
        conn_layout.addWidget(self.port_combo)
        
        self.refresh_btn = QPushButton("刷新")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        conn_layout.addWidget(self.refresh_btn)
        
        # 波特率
        conn_layout.addWidget(QLabel("波特率:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(['9600', '19200', '38400', '57600', '115200', '230400', '460800', '921600'])
        self.baud_combo.setCurrentText('115200')
        conn_layout.addWidget(self.baud_combo)
        
        conn_layout.addStretch()
        
        # 连接按钮
        self.connect_btn = QPushButton("🔗 连接")
        self.connect_btn.setStyleSheet("font-weight: bold; padding: 8px 20px;")
        self.connect_btn.clicked.connect(self.toggle_connection)
        conn_layout.addWidget(self.connect_btn)
        
        # 心跳
        self.heartbeat_cb = QCheckBox("心跳")
        self.heartbeat_cb.setChecked(True)
        conn_layout.addWidget(self.heartbeat_cb)
        
        # 状态指示
        self.status_indicator = QLabel("● 未连接")
        self.status_indicator.setStyleSheet("color: #ff4444; font-weight: bold;")
        conn_layout.addWidget(self.status_indicator)
        
        conn_group.setLayout(conn_layout)
        layout.addWidget(conn_group)
        
        # 初始刷新端口
        self.refresh_ports()
    
    def create_log_panel(self, layout):
        """创建日志面板"""
        log_group = QGroupBox("📝 通信日志")
        log_layout = QVBoxLayout()
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setMaximumHeight(150)
        self.log_text.setStyleSheet(
            "background-color: #1e1e1e; color: #d4d4d4; font-family: Consolas, Monaco, monospace;"
        )
        log_layout.addWidget(self.log_text)
        
        btn_layout = QHBoxLayout()
        clear_btn = QPushButton("清空日志")
        clear_btn.clicked.connect(lambda: self.log_text.clear())
        btn_layout.addWidget(clear_btn)
        btn_layout.addStretch()
        log_layout.addLayout(btn_layout)
        
        log_group.setLayout(log_layout)
        layout.addWidget(log_group)
    
    def connect_signals(self):
        """连接信号和槽"""
        # 串口线程信号
        self.serial_thread.connection_status.connect(self.on_connection_status)
        self.serial_thread.frame_received.connect(self.on_frame_received)
        self.serial_thread.error_occurred.connect(self.on_error)
        self.serial_thread.data_received.connect(self.on_raw_data)
        
        # 控制面板信号
        self.velocity_panel.command_requested.connect(self.send_command)
        self.height_panel.command_requested.connect(self.send_command)
        self.attitude_panel.command_requested.connect(self.send_command)
        self.pose_panel.command_requested.connect(self.send_command)
        
        # 心跳定时器
        self.heartbeat_timer.timeout.connect(self.send_heartbeat)
    
    def refresh_ports(self):
        """刷新串口列表"""
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(f"{port.device} - {port.description}", port.device)
    
    def toggle_connection(self):
        """切换连接状态"""
        if self.serial_thread.serial and self.serial_thread.serial.is_open:
            self.disconnect_serial()
        else:
            self.connect_serial()
    
    def connect_serial(self):
        """连接串口"""
        port_data = self.port_combo.currentData()
        if not port_data:
            QMessageBox.warning(self, "警告", "请选择串口!")
            return
        
        baudrate = int(self.baud_combo.currentText())
        
        if self.serial_thread.connect_serial(port_data, baudrate):
            self.serial_thread.start()
            self.log(f"✓ 已连接: {port_data} @ {baudrate}")
            
            # 启动心跳
            if self.heartbeat_cb.isChecked():
                self.heartbeat_timer.start(1000)
        else:
            self.log("✗ 连接失败", is_error=True)
    
    def disconnect_serial(self):
        """断开串口"""
        self.heartbeat_timer.stop()
        self.serial_thread.disconnect_serial()
        self.serial_thread.wait(1000)
        self.log("已断开连接")
    
    def send_command(self, cmd, data):
        """发送命令"""
        if not self.is_connected():
            self.log("✗ 未连接", is_error=True)
            return
        
        if self.serial_thread.send_frame(cmd, data):
            cmd_names = {
                0x20: 'SET_VELOCITY',
                0x21: 'SET_HEIGHT',
                0x25: 'SET_PITCH',
                0x26: 'SET_ROLL',
                0x27: 'SET_POSE',
                0xFF: 'EMERGENCY_STOP',
            }
            self.log(f"→ {cmd_names.get(cmd, f'0x{cmd:02X}')} [{len(data)}字节]")
    
    def send_heartbeat(self):
        """发送心跳"""
        if self.is_connected():
            timestamp = int(time.time() * 1000) & 0xFFFFFFFF
            data = struct.pack('>I', timestamp)
            self.serial_thread.send_frame(ProtocolConstants.CMD_HEARTBEAT, data)
    
    def is_connected(self):
        """检查是否已连接"""
        return self.serial_thread.serial and self.serial_thread.serial.is_open
    
    def on_connection_status(self, connected):
        """连接状态变化"""
        if connected:
            self.connect_btn.setText("🔌 断开")
            self.status_indicator.setText("● 已连接")
            self.status_indicator.setStyleSheet("color: #00ff00; font-weight: bold;")
        else:
            self.connect_btn.setText("🔗 连接")
            self.status_indicator.setText("● 未连接")
            self.status_indicator.setStyleSheet("color: #ff4444; font-weight: bold;")
    
    def on_frame_received(self, frame_info):
        """收到数据帧"""
        frame_type = frame_info.get('type')
        data = frame_info.get('data')
        
        if frame_type == 'state':
            self.status_panel.update_state(data)
            self.waveform_panel.update_data(data)
        elif frame_type == 'heartbeat_ack':
            pass  # 心跳确认, 不记录
        elif frame_type == 'param_response':
            self.log(f"← 参数响应: ID={data['param_id']}, 成功={data['success']}")
    
    def on_raw_data(self, data):
        """收到原始数据"""
        # 可选: 显示原始数据用于调试
        pass
    
    def on_error(self, error_msg):
        """错误处理"""
        self.log(f"✗ {error_msg}", is_error=True)
    
    def log(self, message, is_error=False):
        """记录日志"""
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        color = "#ff4444" if is_error else "#d4d4d4"
        self.log_text.append(f'<span style="color: #888888;">[{timestamp}]</span> '
                            f'<span style="color: {color};">{message}</span>')
        # 滚动到底部
        scrollbar = self.log_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())
    
    def closeEvent(self, event):
        """关闭事件"""
        self.disconnect_serial()
        event.accept()


# ============================================================================
# 主程序入口
# ============================================================================
def main():
    # 高DPI支持
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)
    
    app = QApplication(sys.argv)
    
    # 深色主题
    app.setStyle('Fusion')
    palette = QPalette()
    palette.setColor(QPalette.Window, QColor(53, 53, 53))
    palette.setColor(QPalette.WindowText, Qt.white)
    palette.setColor(QPalette.Base, QColor(25, 25, 25))
    palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
    palette.setColor(QPalette.ToolTipBase, Qt.white)
    palette.setColor(QPalette.ToolTipText, Qt.white)
    palette.setColor(QPalette.Text, Qt.white)
    palette.setColor(QPalette.Button, QColor(53, 53, 53))
    palette.setColor(QPalette.ButtonText, Qt.white)
    palette.setColor(QPalette.BrightText, Qt.red)
    palette.setColor(QPalette.Link, QColor(42, 130, 218))
    palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
    palette.setColor(QPalette.HighlightedText, Qt.black)
    app.setPalette(palette)
    
    window = PiCommMainWindow()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
