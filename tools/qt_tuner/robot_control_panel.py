"""
轮腿机器人控制面板 - 全功能 GUI 界面
功能:
- 电机控制面板: 速度/位置/力矩控制，状态监控
- IMU 面板: 初始化、数据显示、校准
- 平衡控制面板: balance init/start/stop、参数调节
- 传感器面板: 按键、温湿度、电源监控
- PID调参面板: 集成 Commander 协议

作者: Bubble
日期: 2026-01-22
"""

import sys
import time
import re
from collections import deque
from datetime import datetime

import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QComboBox, QPushButton, QTextEdit, QGroupBox, QGridLayout,
    QTabWidget, QDoubleSpinBox, QSpinBox, QMessageBox, QSplitter,
    QSlider, QCheckBox, QFrame, QProgressBar, QLineEdit
)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QThread
from PyQt5.QtGui import QFont, QPalette, QColor
import pyqtgraph as pg
import numpy as np


# ============================================================================
# 串口通信线程
# ============================================================================
class SerialThread(QThread):
    """串口读取线程"""
    data_received = pyqtSignal(str)
    
    def __init__(self):
        super().__init__()
        self.serial_port = None
        self.running = False
        
    def set_serial(self, port, baudrate=115200):
        try:
            if self.serial_port and self.serial_port.is_open:
                self.serial_port.close()
            self.serial_port = serial.Serial(port, baudrate, timeout=0.1)
            return True
        except Exception as e:
            print(f"串口打开失败: {e}")
            return False
    
    def send_command(self, cmd):
        """发送命令到串口"""
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.reset_input_buffer()
                cmd_with_newline = cmd + '\n'
                self.serial_port.write(cmd_with_newline.encode('utf-8'))
                self.serial_port.flush()
                self.msleep(30)
                return True
            except Exception as e:
                print(f"发送命令失败: {e}")
                return False
        return False
    
    def run(self):
        self.running = True
        buffer = ""
        while self.running:
            if self.serial_port and self.serial_port.is_open:
                try:
                    if self.serial_port.in_waiting:
                        raw_data = self.serial_port.read(self.serial_port.in_waiting)
                        data = raw_data.decode('utf-8', errors='ignore')
                        buffer += data
                        
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            line = line.strip()
                            if line:
                                self.data_received.emit(line)
                except Exception as e:
                    pass
            self.msleep(10)
    
    def stop(self):
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()


# ============================================================================
# 电机控制面板
# ============================================================================
class MotorControlPanel(QWidget):
    """电机控制面板"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.motor_states = {}  # motor_id -> state dict
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # 电机选择和全局控制
        top_group = QGroupBox("全局控制")
        top_layout = QHBoxLayout()
        
        self.scan_btn = QPushButton("🔍 扫描电机")
        self.scan_btn.clicked.connect(lambda: self.send_cmd("scan"))
        top_layout.addWidget(self.scan_btn)
        
        self.enable_all_btn = QPushButton("✅ 使能全部")
        self.enable_all_btn.clicked.connect(lambda: self.send_cmd("enable all"))
        top_layout.addWidget(self.enable_all_btn)
        
        self.stop_all_btn = QPushButton("🛑 停止全部")
        self.stop_all_btn.setStyleSheet("background-color: #ff4444; color: white; font-weight: bold;")
        self.stop_all_btn.clicked.connect(lambda: self.send_cmd("stop all"))
        top_layout.addWidget(self.stop_all_btn)
        
        self.read_all_btn = QPushButton("📊 读取全部")
        self.read_all_btn.clicked.connect(lambda: self.send_cmd("read all"))
        top_layout.addWidget(self.read_all_btn)
        
        top_group.setLayout(top_layout)
        layout.addWidget(top_group)
        
        # 单电机控制
        motor_group = QGroupBox("单电机控制")
        motor_layout = QGridLayout()
        
        # 电机选择
        motor_layout.addWidget(QLabel("电机:"), 0, 0)
        self.motor_combo = QComboBox()
        self.motor_combo.addItems([
            "1 - 左髋 (L_Hip)",
            "2 - 左膝 (L_Knee)", 
            "3 - 左轮 (L_Wheel)",
            "4 - 右髋 (R_Hip)",
            "5 - 右膝 (R_Knee)",
            "6 - 右轮 (R_Wheel)"
        ])
        motor_layout.addWidget(self.motor_combo, 0, 1, 1, 2)
        
        # 控制模式
        motor_layout.addWidget(QLabel("模式:"), 1, 0)
        self.mode_combo = QComboBox()
        self.mode_combo.addItems([
            "0 - 力矩模式",
            "1 - 速度模式",
            "2 - 位置梯形",
            "3 - 位置滤波",
            "4 - 位置直接",
            "5 - 低速模式"
        ])
        motor_layout.addWidget(self.mode_combo, 1, 1)
        
        self.set_mode_btn = QPushButton("设置模式")
        self.set_mode_btn.clicked.connect(self.set_motor_mode)
        motor_layout.addWidget(self.set_mode_btn, 1, 2)
        
        # 速度控制
        motor_layout.addWidget(QLabel("速度 (rpm):"), 2, 0)
        self.speed_input = QDoubleSpinBox()
        self.speed_input.setRange(-500, 500)
        self.speed_input.setSingleStep(10)
        motor_layout.addWidget(self.speed_input, 2, 1)
        
        self.set_speed_btn = QPushButton("设置速度")
        self.set_speed_btn.clicked.connect(self.set_motor_speed)
        motor_layout.addWidget(self.set_speed_btn, 2, 2)
        
        # 速度滑块
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(-200, 200)
        self.speed_slider.setValue(0)
        self.speed_slider.valueChanged.connect(lambda v: self.speed_input.setValue(v))
        motor_layout.addWidget(self.speed_slider, 3, 0, 1, 3)
        
        # 位置控制
        motor_layout.addWidget(QLabel("位置 (°):"), 4, 0)
        self.pos_input = QDoubleSpinBox()
        self.pos_input.setRange(-360, 360)
        self.pos_input.setSingleStep(5)
        motor_layout.addWidget(self.pos_input, 4, 1)
        
        self.set_pos_btn = QPushButton("设置位置")
        self.set_pos_btn.clicked.connect(self.set_motor_position)
        motor_layout.addWidget(self.set_pos_btn, 4, 2)
        
        # 力矩控制
        motor_layout.addWidget(QLabel("力矩:"), 5, 0)
        self.torque_input = QDoubleSpinBox()
        self.torque_input.setRange(-5, 5)
        self.torque_input.setSingleStep(0.1)
        self.torque_input.setDecimals(2)
        motor_layout.addWidget(self.torque_input, 5, 1)
        
        self.set_torque_btn = QPushButton("设置力矩")
        self.set_torque_btn.clicked.connect(self.set_motor_torque)
        motor_layout.addWidget(self.set_torque_btn, 5, 2)
        
        # 单电机操作按钮
        btn_layout = QHBoxLayout()
        self.enable_btn = QPushButton("使能")
        self.enable_btn.clicked.connect(lambda: self.send_motor_cmd("enable"))
        btn_layout.addWidget(self.enable_btn)
        
        self.idle_btn = QPushButton("空闲")
        self.idle_btn.clicked.connect(lambda: self.send_motor_cmd("idle"))
        btn_layout.addWidget(self.idle_btn)
        
        self.stop_btn = QPushButton("停止")
        self.stop_btn.clicked.connect(lambda: self.send_motor_cmd("stop"))
        btn_layout.addWidget(self.stop_btn)
        
        self.read_btn = QPushButton("读取")
        self.read_btn.clicked.connect(lambda: self.send_motor_cmd("read"))
        btn_layout.addWidget(self.read_btn)
        
        motor_layout.addLayout(btn_layout, 6, 0, 1, 3)
        
        # 高级操作
        adv_layout = QHBoxLayout()
        self.zero_btn = QPushButton("设零点")
        self.zero_btn.clicked.connect(lambda: self.send_motor_adv_cmd("origin"))
        adv_layout.addWidget(self.zero_btn)
        
        self.save_btn = QPushButton("保存参数")
        self.save_btn.clicked.connect(lambda: self.send_motor_adv_cmd("save"))
        adv_layout.addWidget(self.save_btn)
        
        self.reboot_btn = QPushButton("重启")
        self.reboot_btn.clicked.connect(lambda: self.send_motor_adv_cmd("reboot"))
        adv_layout.addWidget(self.reboot_btn)
        
        self.error_btn = QPushButton("查错误码")
        self.error_btn.clicked.connect(lambda: self.send_motor_adv_cmd("error"))
        adv_layout.addWidget(self.error_btn)
        
        motor_layout.addLayout(adv_layout, 7, 0, 1, 3)
        
        motor_group.setLayout(motor_layout)
        layout.addWidget(motor_group)
        
        # 电机状态显示
        status_group = QGroupBox("电机状态")
        status_layout = QGridLayout()
        
        headers = ["ID", "位置(°)", "速度(rpm)", "电流(A)", "状态"]
        for col, header in enumerate(headers):
            label = QLabel(header)
            label.setStyleSheet("font-weight: bold;")
            status_layout.addWidget(label, 0, col)
        
        self.status_labels = {}
        motor_names = ["L_Hip", "L_Knee", "L_Wheel", "R_Hip", "R_Knee", "R_Wheel"]
        for row, name in enumerate(motor_names, 1):
            status_layout.addWidget(QLabel(f"{row} {name}"), row, 0)
            self.status_labels[row] = {
                'pos': QLabel("--"),
                'spd': QLabel("--"),
                'cur': QLabel("--"),
                'status': QLabel("--")
            }
            status_layout.addWidget(self.status_labels[row]['pos'], row, 1)
            status_layout.addWidget(self.status_labels[row]['spd'], row, 2)
            status_layout.addWidget(self.status_labels[row]['cur'], row, 3)
            status_layout.addWidget(self.status_labels[row]['status'], row, 4)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        layout.addStretch()
    
    def get_motor_id(self):
        return self.motor_combo.currentIndex() + 1
    
    def send_cmd(self, cmd):
        if self.parent_window:
            self.parent_window.send_command(cmd)
    
    def send_motor_cmd(self, action):
        motor_id = self.get_motor_id()
        self.send_cmd(f"{action} {motor_id}")
    
    def send_motor_adv_cmd(self, action):
        motor_id = self.get_motor_id()
        self.send_cmd(f"motor {motor_id} {action}")
    
    def set_motor_mode(self):
        motor_id = self.get_motor_id()
        mode = self.mode_combo.currentIndex()
        self.send_cmd(f"mode {motor_id} {mode}")
    
    def set_motor_speed(self):
        motor_id = self.get_motor_id()
        speed = self.speed_input.value()
        self.send_cmd(f"speed {motor_id} {speed}")
    
    def set_motor_position(self):
        motor_id = self.get_motor_id()
        pos = self.pos_input.value()
        self.send_cmd(f"pos {motor_id} {pos}")
    
    def set_motor_torque(self):
        motor_id = self.get_motor_id()
        torque = self.torque_input.value()
        self.send_cmd(f"torque {motor_id} {torque}")
    
    def update_motor_status(self, motor_id, pos, spd, cur, online):
        if motor_id in self.status_labels:
            self.status_labels[motor_id]['pos'].setText(f"{pos:.1f}")
            self.status_labels[motor_id]['spd'].setText(f"{spd:.1f}")
            self.status_labels[motor_id]['cur'].setText(f"{cur:.2f}")
            status_text = "在线" if online else "离线"
            color = "green" if online else "red"
            self.status_labels[motor_id]['status'].setText(status_text)
            self.status_labels[motor_id]['status'].setStyleSheet(f"color: {color}; font-weight: bold;")


# ============================================================================
# IMU 控制面板
# ============================================================================
class IMUControlPanel(QWidget):
    """IMU 控制面板"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # 初始化控制
        init_group = QGroupBox("IMU 初始化")
        init_layout = QHBoxLayout()
        
        self.init_btn = QPushButton("📡 初始化 IMU")
        self.init_btn.setStyleSheet("font-size: 14px; padding: 10px;")
        self.init_btn.clicked.connect(lambda: self.send_cmd("imu init"))
        init_layout.addWidget(self.init_btn)
        
        self.deinit_btn = QPushButton("关闭 IMU")
        self.deinit_btn.clicked.connect(lambda: self.send_cmd("imu deinit"))
        init_layout.addWidget(self.deinit_btn)
        
        self.status_btn = QPushButton("查看状态")
        self.status_btn.clicked.connect(lambda: self.send_cmd("imu status"))
        init_layout.addWidget(self.status_btn)
        
        init_group.setLayout(init_layout)
        layout.addWidget(init_group)
        
        # 数据读取
        read_group = QGroupBox("数据读取")
        read_layout = QHBoxLayout()
        
        self.start_btn = QPushButton("▶ 开始连续读取")
        self.start_btn.setStyleSheet("background-color: #44aa44; color: white;")
        self.start_btn.clicked.connect(lambda: self.send_cmd("imu start"))
        read_layout.addWidget(self.start_btn)
        
        self.stop_btn = QPushButton("⏹ 停止读取")
        self.stop_btn.clicked.connect(lambda: self.send_cmd("imu stop"))
        read_layout.addWidget(self.stop_btn)
        
        self.read_once_btn = QPushButton("单次读取")
        self.read_once_btn.clicked.connect(lambda: self.send_cmd("imu read"))
        read_layout.addWidget(self.read_once_btn)
        
        self.print_toggle = QCheckBox("打印数据")
        self.print_toggle.stateChanged.connect(self.toggle_print)
        read_layout.addWidget(self.print_toggle)
        
        read_group.setLayout(read_layout)
        layout.addWidget(read_group)
        
        # 输出速率
        rate_group = QGroupBox("输出速率设置")
        rate_layout = QHBoxLayout()
        
        rate_layout.addWidget(QLabel("速率:"))
        self.rate_combo = QComboBox()
        self.rate_combo.addItems([
            "0 - 0.2Hz", "1 - 0.5Hz", "2 - 1Hz", "3 - 2Hz",
            "4 - 5Hz", "5 - 10Hz", "6 - 20Hz", "7 - 50Hz",
            "8 - 100Hz", "9 - 200Hz", "10 - 无输出", "11 - 单次", "12 - 无输出"
        ])
        self.rate_combo.setCurrentIndex(8)  # 默认 100Hz
        rate_layout.addWidget(self.rate_combo)
        
        self.set_rate_btn = QPushButton("设置")
        self.set_rate_btn.clicked.connect(self.set_rate)
        rate_layout.addWidget(self.set_rate_btn)
        
        rate_group.setLayout(rate_layout)
        layout.addWidget(rate_group)
        
        # 校准
        cali_group = QGroupBox("校准")
        cali_layout = QHBoxLayout()
        
        self.cali_acc_btn = QPushButton("🎯 加速度计校准")
        self.cali_acc_btn.clicked.connect(lambda: self.send_cmd("imu cali acc"))
        cali_layout.addWidget(self.cali_acc_btn)
        
        self.cali_mag_btn = QPushButton("🧭 磁力计校准")
        self.cali_mag_btn.clicked.connect(lambda: self.send_cmd("imu cali mag"))
        cali_layout.addWidget(self.cali_mag_btn)
        
        self.cali_stop_btn = QPushButton("停止校准")
        self.cali_stop_btn.clicked.connect(lambda: self.send_cmd("imu cali stop"))
        cali_layout.addWidget(self.cali_stop_btn)
        
        cali_group.setLayout(cali_layout)
        layout.addWidget(cali_group)
        
        # IMU 数据显示
        data_group = QGroupBox("IMU 数据")
        data_layout = QGridLayout()
        
        # 欧拉角
        data_layout.addWidget(QLabel("Roll (横滚):"), 0, 0)
        self.roll_label = self.create_data_label()
        data_layout.addWidget(self.roll_label, 0, 1)
        
        data_layout.addWidget(QLabel("Pitch (俯仰):"), 1, 0)
        self.pitch_label = self.create_data_label()
        data_layout.addWidget(self.pitch_label, 1, 1)
        
        data_layout.addWidget(QLabel("Yaw (偏航):"), 2, 0)
        self.yaw_label = self.create_data_label()
        data_layout.addWidget(self.yaw_label, 2, 1)
        
        # 角速度
        data_layout.addWidget(QLabel("Gyro X:"), 0, 2)
        self.gx_label = self.create_data_label()
        data_layout.addWidget(self.gx_label, 0, 3)
        
        data_layout.addWidget(QLabel("Gyro Y:"), 1, 2)
        self.gy_label = self.create_data_label()
        data_layout.addWidget(self.gy_label, 1, 3)
        
        data_layout.addWidget(QLabel("Gyro Z:"), 2, 2)
        self.gz_label = self.create_data_label()
        data_layout.addWidget(self.gz_label, 2, 3)
        
        data_group.setLayout(data_layout)
        layout.addWidget(data_group)
        
        layout.addStretch()
    
    def create_data_label(self):
        label = QLabel("--")
        label.setStyleSheet("font-size: 16px; font-weight: bold; color: #00ccff; "
                          "background-color: #1a1a2e; padding: 8px; border-radius: 4px;")
        label.setAlignment(Qt.AlignCenter)
        label.setMinimumWidth(100)
        return label
    
    def send_cmd(self, cmd):
        if self.parent_window:
            self.parent_window.send_command(cmd)
    
    def toggle_print(self, state):
        if state == Qt.Checked:
            self.send_cmd("imu print on")
        else:
            self.send_cmd("imu print off")
    
    def set_rate(self):
        rate = self.rate_combo.currentIndex()
        self.send_cmd(f"imu rate {rate}")
    
    def update_imu_data(self, roll, pitch, yaw, gx, gy, gz):
        self.roll_label.setText(f"{roll:.2f}°")
        self.pitch_label.setText(f"{pitch:.2f}°")
        self.yaw_label.setText(f"{yaw:.2f}°")
        self.gx_label.setText(f"{gx:.2f}°/s")
        self.gy_label.setText(f"{gy:.2f}°/s")
        self.gz_label.setText(f"{gz:.2f}°/s")


# ============================================================================
# 平衡控制面板
# ============================================================================
class BalanceControlPanel(QWidget):
    """平衡控制面板"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.balance_initialized = False
        self.balance_running = False
        self.balance_enabled = False
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # 初始化控制
        init_group = QGroupBox("平衡系统初始化")
        init_layout = QVBoxLayout()
        
        btn_layout = QHBoxLayout()
        self.init_btn = QPushButton("🚀 初始化平衡系统 (balance init)")
        self.init_btn.setStyleSheet("font-size: 16px; padding: 15px; background-color: #4488ff;")
        self.init_btn.clicked.connect(self.do_balance_init)
        btn_layout.addWidget(self.init_btn)
        
        self.init_status = QLabel("未初始化")
        self.init_status.setStyleSheet("font-size: 14px; color: gray;")
        btn_layout.addWidget(self.init_status)
        
        init_layout.addLayout(btn_layout)
        
        note_label = QLabel("⚠️ 注意: 必须先初始化才能使用平衡控制和 PID 调参功能")
        note_label.setStyleSheet("color: orange;")
        init_layout.addWidget(note_label)
        
        init_group.setLayout(init_layout)
        layout.addWidget(init_group)
        
        # 运行控制
        run_group = QGroupBox("运行控制")
        run_layout = QHBoxLayout()
        
        self.start_btn = QPushButton("▶ 启动 (balance start)")
        self.start_btn.setStyleSheet("font-size: 14px; padding: 10px; background-color: #44aa44;")
        self.start_btn.clicked.connect(lambda: self.send_cmd("balance start"))
        run_layout.addWidget(self.start_btn)
        
        self.stop_btn = QPushButton("⏹ 停止 (balance stop)")
        self.stop_btn.setStyleSheet("font-size: 14px; padding: 10px;")
        self.stop_btn.clicked.connect(lambda: self.send_cmd("balance stop"))
        run_layout.addWidget(self.stop_btn)
        
        self.status_btn = QPushButton("📊 状态")
        self.status_btn.clicked.connect(lambda: self.send_cmd("balance status"))
        run_layout.addWidget(self.status_btn)
        
        run_group.setLayout(run_layout)
        layout.addWidget(run_group)
        
        # 平衡使能
        enable_group = QGroupBox("平衡控制")
        enable_layout = QHBoxLayout()
        
        self.enable_btn = QPushButton("✅ 使能平衡")
        self.enable_btn.setStyleSheet("font-size: 14px; padding: 10px; background-color: #44aa44;")
        self.enable_btn.clicked.connect(lambda: self.send_cmd("balance enable"))
        enable_layout.addWidget(self.enable_btn)
        
        self.disable_btn = QPushButton("❌ 禁用平衡")
        self.disable_btn.clicked.connect(lambda: self.send_cmd("balance disable"))
        enable_layout.addWidget(self.disable_btn)
        
        self.estop_btn = QPushButton("🛑 紧急停止")
        self.estop_btn.setStyleSheet("font-size: 14px; padding: 10px; background-color: #ff4444; color: white; font-weight: bold;")
        self.estop_btn.clicked.connect(lambda: self.send_cmd("balance estop"))
        enable_layout.addWidget(self.estop_btn)
        
        self.reset_btn = QPushButton("🔄 重置")
        self.reset_btn.clicked.connect(lambda: self.send_cmd("balance reset"))
        enable_layout.addWidget(self.reset_btn)
        
        enable_group.setLayout(enable_layout)
        layout.addWidget(enable_group)
        
        # 角度零点设置
        zero_group = QGroupBox("角度零点设置")
        zero_layout = QHBoxLayout()
        
        zero_layout.addWidget(QLabel("零点角度 (°):"))
        self.zero_input = QDoubleSpinBox()
        self.zero_input.setRange(-30, 30)
        self.zero_input.setSingleStep(0.1)
        self.zero_input.setDecimals(2)
        self.zero_input.setValue(7.4)
        zero_layout.addWidget(self.zero_input)
        
        self.set_zero_btn = QPushButton("设置零点")
        self.set_zero_btn.clicked.connect(self.set_zero_point)
        zero_layout.addWidget(self.set_zero_btn)
        
        self.get_zero_btn = QPushButton("获取当前零点")
        self.get_zero_btn.clicked.connect(lambda: self.send_cmd("balance zero"))
        zero_layout.addWidget(self.get_zero_btn)
        
        zero_group.setLayout(zero_layout)
        layout.addWidget(zero_group)
        
        # 波形输出控制
        plot_group = QGroupBox("波形输出")
        plot_layout = QHBoxLayout()
        
        self.plot_on_btn = QPushButton("开启波形")
        self.plot_on_btn.clicked.connect(lambda: self.send_cmd("balance plot on"))
        plot_layout.addWidget(self.plot_on_btn)
        
        self.plot_off_btn = QPushButton("关闭波形")
        self.plot_off_btn.clicked.connect(lambda: self.send_cmd("balance plot off"))
        plot_layout.addWidget(self.plot_off_btn)
        
        plot_layout.addWidget(QLabel("分频:"))
        self.plot_div_input = QSpinBox()
        self.plot_div_input.setRange(1, 255)
        self.plot_div_input.setValue(10)
        plot_layout.addWidget(self.plot_div_input)
        
        self.set_div_btn = QPushButton("设置")
        self.set_div_btn.clicked.connect(self.set_plot_divider)
        plot_layout.addWidget(self.set_div_btn)
        
        plot_group.setLayout(plot_layout)
        layout.addWidget(plot_group)
        
        # 状态显示
        status_group = QGroupBox("当前状态")
        status_layout = QGridLayout()
        
        status_layout.addWidget(QLabel("系统状态:"), 0, 0)
        self.sys_status_label = QLabel("未初始化")
        self.sys_status_label.setStyleSheet("font-weight: bold; color: gray;")
        status_layout.addWidget(self.sys_status_label, 0, 1)
        
        status_layout.addWidget(QLabel("控制状态:"), 0, 2)
        self.ctrl_status_label = QLabel("--")
        status_layout.addWidget(self.ctrl_status_label, 0, 3)
        
        status_layout.addWidget(QLabel("Pitch 角度:"), 1, 0)
        self.pitch_label = QLabel("--")
        self.pitch_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #00ff00;")
        status_layout.addWidget(self.pitch_label, 1, 1)
        
        status_layout.addWidget(QLabel("Roll 角度:"), 1, 2)
        self.roll_label = QLabel("--")
        self.roll_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #00ff00;")
        status_layout.addWidget(self.roll_label, 1, 3)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        layout.addStretch()
    
    def send_cmd(self, cmd):
        if self.parent_window:
            self.parent_window.send_command(cmd)
    
    def do_balance_init(self):
        self.send_cmd("balance init")
        self.init_status.setText("初始化中...")
        self.init_status.setStyleSheet("color: orange;")
    
    def set_zero_point(self):
        zero = self.zero_input.value()
        self.send_cmd(f"balance zero {zero}")
    
    def set_plot_divider(self):
        div = self.plot_div_input.value()
        self.send_cmd(f"balance plot div {div}")
    
    def on_init_success(self):
        self.balance_initialized = True
        self.init_status.setText("✓ 已初始化")
        self.init_status.setStyleSheet("color: green; font-weight: bold;")
        self.sys_status_label.setText("就绪")
        self.sys_status_label.setStyleSheet("font-weight: bold; color: green;")


# ============================================================================
# 传感器面板
# ============================================================================
class SensorPanel(QWidget):
    """传感器控制面板"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # 电源监控
        power_group = QGroupBox("⚡ 电源监控")
        power_layout = QVBoxLayout()
        
        btn_layout = QHBoxLayout()
        self.power_btn = QPushButton("读取电源状态")
        self.power_btn.clicked.connect(lambda: self.send_cmd("power"))
        btn_layout.addWidget(self.power_btn)
        
        self.mpower_on_btn = QPushButton("电机供电 ON")
        self.mpower_on_btn.setStyleSheet("background-color: #44aa44;")
        self.mpower_on_btn.clicked.connect(lambda: self.send_cmd("mpower on"))
        btn_layout.addWidget(self.mpower_on_btn)
        
        self.mpower_off_btn = QPushButton("电机供电 OFF")
        self.mpower_off_btn.setStyleSheet("background-color: #aa4444;")
        self.mpower_off_btn.clicked.connect(lambda: self.send_cmd("mpower off"))
        btn_layout.addWidget(self.mpower_off_btn)
        
        power_layout.addLayout(btn_layout)
        
        # 电源状态显示
        status_layout = QHBoxLayout()
        status_layout.addWidget(QLabel("电池:"))
        self.battery_label = QLabel("--")
        self.battery_label.setStyleSheet("font-size: 16px; font-weight: bold;")
        status_layout.addWidget(self.battery_label)
        
        status_layout.addWidget(QLabel("电机供电:"))
        self.mpower_label = QLabel("--")
        status_layout.addWidget(self.mpower_label)
        
        power_layout.addLayout(status_layout)
        power_group.setLayout(power_layout)
        layout.addWidget(power_group)
        
        # 按键
        btn_group = QGroupBox("🔘 按键")
        btn_layout = QHBoxLayout()
        
        self.btn_init_btn = QPushButton("初始化按键")
        self.btn_init_btn.clicked.connect(lambda: self.send_cmd("btn init"))
        btn_layout.addWidget(self.btn_init_btn)
        
        self.btn_read_btn = QPushButton("读取按键")
        self.btn_read_btn.clicked.connect(lambda: self.send_cmd("btn read"))
        btn_layout.addWidget(self.btn_read_btn)
        
        self.btn_start_btn = QPushButton("开始监控")
        self.btn_start_btn.clicked.connect(lambda: self.send_cmd("btn start"))
        btn_layout.addWidget(self.btn_start_btn)
        
        self.btn_stop_btn = QPushButton("停止监控")
        self.btn_stop_btn.clicked.connect(lambda: self.send_cmd("btn stop"))
        btn_layout.addWidget(self.btn_stop_btn)
        
        btn_group.setLayout(btn_layout)
        layout.addWidget(btn_group)
        
        # 温湿度传感器
        sht_group = QGroupBox("🌡️ 温湿度传感器 (SHT30)")
        sht_layout = QVBoxLayout()
        
        ctrl_layout = QHBoxLayout()
        self.sht_init_btn = QPushButton("初始化")
        self.sht_init_btn.clicked.connect(lambda: self.send_cmd("sht init"))
        ctrl_layout.addWidget(self.sht_init_btn)
        
        self.sht_read_btn = QPushButton("读取")
        self.sht_read_btn.clicked.connect(lambda: self.send_cmd("sht read"))
        ctrl_layout.addWidget(self.sht_read_btn)
        
        self.sht_start_btn = QPushButton("开始监控")
        self.sht_start_btn.clicked.connect(lambda: self.send_cmd("sht start"))
        ctrl_layout.addWidget(self.sht_start_btn)
        
        self.sht_stop_btn = QPushButton("停止监控")
        self.sht_stop_btn.clicked.connect(lambda: self.send_cmd("sht stop"))
        ctrl_layout.addWidget(self.sht_stop_btn)
        
        self.sht_scan_btn = QPushButton("扫描 I2C")
        self.sht_scan_btn.clicked.connect(lambda: self.send_cmd("sht scan"))
        ctrl_layout.addWidget(self.sht_scan_btn)
        
        sht_layout.addLayout(ctrl_layout)
        
        # 温湿度显示
        data_layout = QHBoxLayout()
        data_layout.addWidget(QLabel("温度:"))
        self.temp_label = QLabel("--")
        self.temp_label.setStyleSheet("font-size: 20px; font-weight: bold; color: #ff6600;")
        data_layout.addWidget(self.temp_label)
        
        data_layout.addWidget(QLabel("湿度:"))
        self.humi_label = QLabel("--")
        self.humi_label.setStyleSheet("font-size: 20px; font-weight: bold; color: #0066ff;")
        data_layout.addWidget(self.humi_label)
        
        sht_layout.addLayout(data_layout)
        sht_group.setLayout(sht_layout)
        layout.addWidget(sht_group)
        
        # WiFi 遥控
        wifi_group = QGroupBox("📶 WiFi 遥控器")
        wifi_layout = QHBoxLayout()
        
        self.wifi_start_btn = QPushButton("启动 WiFi AP")
        self.wifi_start_btn.setStyleSheet("background-color: #4488ff;")
        self.wifi_start_btn.clicked.connect(lambda: self.send_cmd("wifi start"))
        wifi_layout.addWidget(self.wifi_start_btn)
        
        self.wifi_stop_btn = QPushButton("停止 WiFi")
        self.wifi_stop_btn.clicked.connect(lambda: self.send_cmd("wifi stop"))
        wifi_layout.addWidget(self.wifi_stop_btn)
        
        self.wifi_status_btn = QPushButton("查看状态")
        self.wifi_status_btn.clicked.connect(lambda: self.send_cmd("wifi status"))
        wifi_layout.addWidget(self.wifi_status_btn)
        
        wifi_group.setLayout(wifi_layout)
        layout.addWidget(wifi_group)
        
        # WiFi 信息
        info_group = QGroupBox("WiFi 连接信息")
        info_layout = QVBoxLayout()
        info_layout.addWidget(QLabel("SSID: WL-PRO"))
        info_layout.addWidget(QLabel("密码: 12345678"))
        info_layout.addWidget(QLabel("控制页面: http://192.168.4.1"))
        info_group.setLayout(info_layout)
        layout.addWidget(info_group)
        
        layout.addStretch()
    
    def send_cmd(self, cmd):
        if self.parent_window:
            self.parent_window.send_command(cmd)


# ============================================================================
# 终端面板
# ============================================================================
class TerminalPanel(QWidget):
    """终端/命令行面板"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # 输出区域
        self.output_text = QTextEdit()
        self.output_text.setReadOnly(True)
        self.output_text.setStyleSheet("""
            QTextEdit {
                background-color: #1a1a2e;
                color: #00ff00;
                font-family: 'Consolas', 'Monaco', monospace;
                font-size: 12px;
                padding: 5px;
            }
        """)
        layout.addWidget(self.output_text)
        
        # 输入区域
        input_layout = QHBoxLayout()
        
        input_layout.addWidget(QLabel("命令:"))
        self.cmd_input = QLineEdit()
        self.cmd_input.setStyleSheet("font-size: 14px; padding: 5px;")
        self.cmd_input.returnPressed.connect(self.send_command)
        input_layout.addWidget(self.cmd_input)
        
        self.send_btn = QPushButton("发送")
        self.send_btn.clicked.connect(self.send_command)
        input_layout.addWidget(self.send_btn)
        
        self.clear_btn = QPushButton("清空")
        self.clear_btn.clicked.connect(self.output_text.clear)
        input_layout.addWidget(self.clear_btn)
        
        layout.addLayout(input_layout)
        
        # 快捷命令
        quick_layout = QHBoxLayout()
        quick_cmds = ["help", "scan", "read all", "balance status", "imu status", "power"]
        for cmd in quick_cmds:
            btn = QPushButton(cmd)
            btn.clicked.connect(lambda checked, c=cmd: self.quick_send(c))
            quick_layout.addWidget(btn)
        layout.addLayout(quick_layout)
    
    def send_command(self):
        cmd = self.cmd_input.text().strip()
        if cmd and self.parent_window:
            self.parent_window.send_command(cmd)
            self.cmd_input.clear()
    
    def quick_send(self, cmd):
        if self.parent_window:
            self.parent_window.send_command(cmd)
    
    def append_output(self, text, is_tx=False):
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        if is_tx:
            self.output_text.append(f'<span style="color: #ffff00;">[{timestamp}] [TX] {text}</span>')
        else:
            self.output_text.append(f'<span style="color: #00ff00;">[{timestamp}] {text}</span>')
        
        # 自动滚动到底部
        scrollbar = self.output_text.verticalScrollBar()
        scrollbar.setValue(scrollbar.maximum())


# ============================================================================
# 主窗口
# ============================================================================
class RobotControlPanel(QMainWindow):
    """机器人控制面板主窗口"""
    
    def __init__(self):
        super().__init__()
        self.serial_thread = SerialThread()
        self.serial_thread.data_received.connect(self.on_data_received)
        self.connected = False
        
        self.init_ui()
        self.refresh_ports()
    
    def init_ui(self):
        self.setWindowTitle("轮腿机器人控制面板 v1.0")
        self.setGeometry(100, 100, 1200, 800)
        
        # 中央部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # 串口连接区域
        conn_group = QGroupBox("串口连接")
        conn_layout = QHBoxLayout()
        
        conn_layout.addWidget(QLabel("串口:"))
        self.port_combo = QComboBox()
        self.port_combo.setMinimumWidth(200)
        conn_layout.addWidget(self.port_combo)
        
        self.refresh_btn = QPushButton("🔄 刷新")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        conn_layout.addWidget(self.refresh_btn)
        
        conn_layout.addWidget(QLabel("波特率:"))
        self.baud_combo = QComboBox()
        self.baud_combo.addItems(["115200", "9600", "57600", "230400", "460800"])
        conn_layout.addWidget(self.baud_combo)
        
        self.connect_btn = QPushButton("🔌 连接")
        self.connect_btn.setStyleSheet("font-weight: bold; padding: 8px 20px;")
        self.connect_btn.clicked.connect(self.toggle_connection)
        conn_layout.addWidget(self.connect_btn)
        
        self.status_label = QLabel("未连接")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        conn_layout.addWidget(self.status_label)
        
        conn_layout.addStretch()
        conn_group.setLayout(conn_layout)
        main_layout.addWidget(conn_group)
        
        # 选项卡
        self.tab_widget = QTabWidget()
        self.tab_widget.setStyleSheet("QTabBar::tab { padding: 10px 20px; font-size: 14px; }")
        
        # 各个面板
        self.balance_panel = BalanceControlPanel(self)
        self.tab_widget.addTab(self.balance_panel, "🎮 平衡控制")
        
        self.motor_panel = MotorControlPanel(self)
        self.tab_widget.addTab(self.motor_panel, "⚙️ 电机控制")
        
        self.imu_panel = IMUControlPanel(self)
        self.tab_widget.addTab(self.imu_panel, "📐 IMU")
        
        self.sensor_panel = SensorPanel(self)
        self.tab_widget.addTab(self.sensor_panel, "🔧 传感器")
        
        self.terminal_panel = TerminalPanel(self)
        self.tab_widget.addTab(self.terminal_panel, "💻 终端")
        
        main_layout.addWidget(self.tab_widget)
    
    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(f"{port.device} - {port.description}", port.device)
    
    def toggle_connection(self):
        if self.connected:
            self.disconnect_serial()
        else:
            self.connect_serial()
    
    def connect_serial(self):
        port = self.port_combo.currentData()
        baud = int(self.baud_combo.currentText())
        
        if port and self.serial_thread.set_serial(port, baud):
            self.serial_thread.start()
            self.connected = True
            self.connect_btn.setText("断开")
            self.connect_btn.setStyleSheet("font-weight: bold; padding: 8px 20px; background-color: #ff6666;")
            self.status_label.setText(f"已连接: {port}")
            self.status_label.setStyleSheet("color: green; font-weight: bold;")
            self.log(f"已连接到 {port} @ {baud}")
        else:
            QMessageBox.warning(self, "连接失败", f"无法打开串口 {port}")
    
    def disconnect_serial(self):
        self.serial_thread.stop()
        self.serial_thread.wait()
        self.connected = False
        self.connect_btn.setText("🔌 连接")
        self.connect_btn.setStyleSheet("font-weight: bold; padding: 8px 20px;")
        self.status_label.setText("未连接")
        self.status_label.setStyleSheet("color: red; font-weight: bold;")
        self.log("已断开连接")
    
    def is_connected(self):
        return self.connected
    
    def send_command(self, cmd):
        if self.connected and self.serial_thread.send_command(cmd):
            self.log(f"{cmd}", is_tx=True)
            return True
        return False
    
    def log(self, msg, is_tx=False, is_error=False):
        self.terminal_panel.append_output(msg, is_tx)
    
    def on_data_received(self, line):
        self.log(line)
        
        # 解析特定响应
        self.parse_response(line)
    
    def parse_response(self, line):
        """解析串口响应并更新 UI"""
        
        # 检测 balance init 成功
        if "Balance test init: OK" in line or "Balance test module initialized" in line:
            self.balance_panel.on_init_success()
        
        # 检测 IMU 数据
        # 格式: [IMU] Roll=xxx Pitch=xxx Yaw=xxx
        imu_match = re.search(r'Roll[=:]?\s*([-\d.]+).*Pitch[=:]?\s*([-\d.]+).*Yaw[=:]?\s*([-\d.]+)', line, re.IGNORECASE)
        if imu_match:
            try:
                roll = float(imu_match.group(1))
                pitch = float(imu_match.group(2))
                yaw = float(imu_match.group(3))
                self.imu_panel.update_imu_data(roll, pitch, yaw, 0, 0, 0)
            except:
                pass
        
        # 检测电机状态
        # 格式: M1: pos=xxx spd=xxx cur=xxx ONLINE/OFFLINE
        motor_match = re.search(r'M(\d):\s*pos=([-\d.]+).*spd=([-\d.]+).*cur=([-\d.]+).*?(ONLINE|OFFLINE)', line, re.IGNORECASE)
        if motor_match:
            try:
                motor_id = int(motor_match.group(1))
                pos = float(motor_match.group(2))
                spd = float(motor_match.group(3))
                cur = float(motor_match.group(4))
                online = motor_match.group(5).upper() == "ONLINE"
                self.motor_panel.update_motor_status(motor_id, pos, spd, cur, online)
            except:
                pass
    
    def closeEvent(self, event):
        if self.connected:
            self.disconnect_serial()
        event.accept()


# ============================================================================
# 主程序
# ============================================================================
def main():
    app = QApplication(sys.argv)
    
    # 设置应用样式
    app.setStyle("Fusion")
    
    # 深色主题
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
    
    window = RobotControlPanel()
    window.show()
    
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
