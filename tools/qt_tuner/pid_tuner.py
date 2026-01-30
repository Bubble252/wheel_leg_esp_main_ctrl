"""
轮腿机器人控制面板 - 全功能 GUI 界面
功能:
- 通过串口使用Commander协议读取/设置PID参数
- 图形化界面调节P、I、D、Limit、Ramp参数
- 一键查询当前参数值
- 支持多个PID控制器切换
- 实时波形显示目标值和控制量
- Web遥控器状态监控
- 电机控制面板: 速度/位置/力矩控制，状态监控
- IMU 面板: 初始化、数据显示、校准
- 平衡控制面板: balance init/start/stop、参数调节
- 传感器面板: 按键、温湿度、电源监控
- 终端命令行面板

基于 shibo_wheel_leg 项目的 qt_board/pid_tuner.py
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
                # 清空输入缓冲区,避免旧数据干扰
                self.serial_port.reset_input_buffer()
                # 发送命令 - Commander需要换行符
                cmd_with_newline = cmd + '\n'
                self.serial_port.write(cmd_with_newline.encode('utf-8'))
                # 等待数据发送完成
                self.serial_port.flush()
                # 短暂延迟让ESP32处理命令
                self.msleep(50)
                return True
            except Exception as e:
                print(f"发送命令失败: {e}")
                return False
        return False
    
    def run(self):
        self.running = True
        buffer = ""
        error_count = 0
        max_errors = 5  # 连续错误次数超过此值则停止打印
        
        while self.running:
            if self.serial_port and self.serial_port.is_open:
                try:
                    if self.serial_port.in_waiting:
                        raw_data = self.serial_port.read(self.serial_port.in_waiting)
                        data = raw_data.decode('utf-8', errors='ignore')
                        buffer += data
                        
                        # 按行处理
                        while '\n' in buffer:
                            line, buffer = buffer.split('\n', 1)
                            line = line.strip()
                            if line:
                                self.data_received.emit(line)
                        error_count = 0  # 重置错误计数
                except OSError as e:
                    error_count += 1
                    if error_count <= max_errors:
                        print(f"串口读取错误: {e}")
                    elif error_count == max_errors + 1:
                        print("串口断开，请检查 USB 连接后重新连接串口")
                    # 串口可能已断开，关闭它
                    try:
                        self.serial_port.close()
                    except:
                        pass
                except Exception as e:
                    error_count += 1
                    if error_count <= max_errors:
                        print(f"串口读取错误: {e}")
            self.msleep(10)
    
    def stop(self):
        self.running = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()


# ============================================================================
# PID控制面板
# ============================================================================
class PIDControlPanel(QWidget):
    """单个PID控制器的参数面板"""
    
    def __init__(self, name, commander_id, parent=None):
        super().__init__(parent)
        self.name = name
        self.commander_id = commander_id
        self.parent_window = parent
        
        # 数据缓存 (最多保存500个点)
        self.max_points = 500
        self.time_data = deque(maxlen=self.max_points)
        self.target_data = deque(maxlen=self.max_points)
        self.control_data = deque(maxlen=self.max_points)
        self.data_counter = 0
        
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # 参数输入区域
        param_group = QGroupBox(f"{self.name} 参数设置")
        param_layout = QGridLayout()
        param_layout.setSpacing(15)
        
        # P参数
        param_layout.addWidget(QLabel("P (比例):"), 0, 0)
        self.p_input = QDoubleSpinBox()
        self.p_input.setRange(-1000, 1000)
        self.p_input.setDecimals(4)
        self.p_input.setSingleStep(0.1)
        self.p_input.setStyleSheet("font-size: 14px; padding: 5px;")
        param_layout.addWidget(self.p_input, 0, 1)
        
        self.p_set_btn = QPushButton("设置 P")
        self.p_set_btn.clicked.connect(lambda: self.set_param('P', self.p_input.value()))
        param_layout.addWidget(self.p_set_btn, 0, 2)
        
        # I参数
        param_layout.addWidget(QLabel("I (积分):"), 1, 0)
        self.i_input = QDoubleSpinBox()
        self.i_input.setRange(-1000, 1000)
        self.i_input.setDecimals(4)
        self.i_input.setSingleStep(0.1)
        self.i_input.setStyleSheet("font-size: 14px; padding: 5px;")
        param_layout.addWidget(self.i_input, 1, 1)
        
        self.i_set_btn = QPushButton("设置 I")
        self.i_set_btn.clicked.connect(lambda: self.set_param('I', self.i_input.value()))
        param_layout.addWidget(self.i_set_btn, 1, 2)
        
        # D参数
        param_layout.addWidget(QLabel("D (微分):"), 2, 0)
        self.d_input = QDoubleSpinBox()
        self.d_input.setRange(-1000, 1000)
        self.d_input.setDecimals(4)
        self.d_input.setSingleStep(0.1)
        self.d_input.setStyleSheet("font-size: 14px; padding: 5px;")
        param_layout.addWidget(self.d_input, 2, 1)
        
        self.d_set_btn = QPushButton("设置 D")
        self.d_set_btn.clicked.connect(lambda: self.set_param('D', self.d_input.value()))
        param_layout.addWidget(self.d_set_btn, 2, 2)
        
        # Limit参数
        param_layout.addWidget(QLabel("Limit (限幅):"), 3, 0)
        self.limit_input = QDoubleSpinBox()
        self.limit_input.setRange(0, 1000)
        self.limit_input.setDecimals(4)
        self.limit_input.setSingleStep(0.5)
        self.limit_input.setStyleSheet("font-size: 14px; padding: 5px;")
        param_layout.addWidget(self.limit_input, 3, 1)
        
        self.limit_set_btn = QPushButton("设置 Limit")
        self.limit_set_btn.clicked.connect(lambda: self.set_param('L', self.limit_input.value()))
        param_layout.addWidget(self.limit_set_btn, 3, 2)
        
        # Ramp参数
        param_layout.addWidget(QLabel("Ramp (变化率):"), 4, 0)
        self.ramp_input = QDoubleSpinBox()
        self.ramp_input.setRange(0, 1000000)
        self.ramp_input.setDecimals(0)
        self.ramp_input.setSingleStep(1000)
        self.ramp_input.setStyleSheet("font-size: 14px; padding: 5px;")
        param_layout.addWidget(self.ramp_input, 4, 1)
        
        self.ramp_set_btn = QPushButton("设置 Ramp")
        self.ramp_set_btn.clicked.connect(lambda: self.set_param('R', self.ramp_input.value()))
        param_layout.addWidget(self.ramp_set_btn, 4, 2)
        
        param_group.setLayout(param_layout)
        layout.addWidget(param_group)
        
        # 快捷操作区
        action_group = QGroupBox("快捷操作")
        action_layout = QHBoxLayout()
        
        self.query_btn = QPushButton("🔍 查询当前参数")
        self.query_btn.setStyleSheet("font-size: 14px; font-weight: bold; padding: 10px;")
        self.query_btn.clicked.connect(self.query_params)
        action_layout.addWidget(self.query_btn)
        
        self.set_all_btn = QPushButton("📤 发送全部参数")
        self.set_all_btn.setStyleSheet("font-size: 14px; font-weight: bold; padding: 10px;")
        self.set_all_btn.clicked.connect(self.set_all_params)
        action_layout.addWidget(self.set_all_btn)
        
        self.reset_btn = QPushButton("🔄 重置为0")
        self.reset_btn.setStyleSheet("font-size: 14px; padding: 10px;")
        self.reset_btn.clicked.connect(self.reset_params)
        action_layout.addWidget(self.reset_btn)
        
        action_group.setLayout(action_layout)
        layout.addWidget(action_group)
        
        # 当前参数显示区
        display_group = QGroupBox("当前参数值 (从ESP32读取)")
        display_layout = QGridLayout()
        display_layout.setSpacing(10)
        
        self.p_display = self.create_param_display("--")
        self.i_display = self.create_param_display("--")
        self.d_display = self.create_param_display("--")
        self.limit_display = self.create_param_display("--")
        self.ramp_display = self.create_param_display("--")
        
        display_layout.addWidget(QLabel("P:"), 0, 0)
        display_layout.addWidget(self.p_display, 0, 1)
        display_layout.addWidget(QLabel("I:"), 0, 2)
        display_layout.addWidget(self.i_display, 0, 3)
        display_layout.addWidget(QLabel("D:"), 1, 0)
        display_layout.addWidget(self.d_display, 1, 1)
        display_layout.addWidget(QLabel("Limit:"), 1, 2)
        display_layout.addWidget(self.limit_display, 1, 3)
        display_layout.addWidget(QLabel("Ramp:"), 2, 0)
        display_layout.addWidget(self.ramp_display, 2, 1)
        
        display_group.setLayout(display_layout)
        layout.addWidget(display_group)
        
        # 波形显示区
        plot_group = QGroupBox("实时波形 (蓝色=目标值, 红色=当前值)")
        plot_layout = QVBoxLayout()
        
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('k')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.plot_widget.setLabel('left', '数值')
        self.plot_widget.setLabel('bottom', '时间 (采样点)')
        self.plot_widget.addLegend()
        
        self.target_curve = self.plot_widget.plot(pen=pg.mkPen(color='b', width=2), name='目标值')
        self.control_curve = self.plot_widget.plot(pen=pg.mkPen(color='r', width=2), name='当前值')
        
        plot_layout.addWidget(self.plot_widget)
        
        # 波形控制按钮
        plot_btn_layout = QHBoxLayout()
        self.clear_plot_btn = QPushButton("清空波形")
        self.clear_plot_btn.clicked.connect(self.clear_plot)
        plot_btn_layout.addWidget(self.clear_plot_btn)
        
        self.pause_plot_btn = QPushButton("暂停")
        self.pause_plot_btn.setCheckable(True)
        plot_btn_layout.addWidget(self.pause_plot_btn)
        
        plot_btn_layout.addStretch()
        plot_layout.addLayout(plot_btn_layout)
        
        plot_group.setLayout(plot_layout)
        layout.addWidget(plot_group)
    
    def create_param_display(self, text):
        """创建参数显示标签"""
        label = QLabel(text)
        label.setStyleSheet("font-size: 16px; font-weight: bold; color: #00ff00; "
                          "background-color: #2b2b2b; padding: 8px; border-radius: 3px;")
        label.setAlignment(Qt.AlignCenter)
        return label
    
    def set_param(self, param_type, value):
        """设置单个参数"""
        if not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        
        cmd = f"{self.commander_id}{param_type}{value}"
        
        if self.parent_window.send_command(cmd):
            self.parent_window.log(f"发送: {cmd} -> {self.name} {param_type}={value}")
        else:
            self.parent_window.log(f"✗ 发送失败: {cmd}", is_error=True)
    
    def set_all_params(self):
        """发送所有参数"""
        if not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        
        p = self.p_input.value()
        i = self.i_input.value()
        d = self.d_input.value()
        limit = self.limit_input.value()
        ramp = self.ramp_input.value()
        
        self.parent_window.send_command(f"{self.commander_id}P{p}")
        time.sleep(0.05)
        self.parent_window.send_command(f"{self.commander_id}I{i}")
        time.sleep(0.05)
        self.parent_window.send_command(f"{self.commander_id}D{d}")
        time.sleep(0.05)
        self.parent_window.send_command(f"{self.commander_id}L{limit}")
        time.sleep(0.05)
        self.parent_window.send_command(f"{self.commander_id}R{ramp}")
        
        self.parent_window.log(f"已发送 {self.name} 全部参数: P={p}, I={i}, D={d}, L={limit}, R={ramp}")
    
    def query_params(self):
        """查询当前参数"""
        if not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        
        cmd = f"{self.commander_id}?"
        self.parent_window.send_command(cmd)
        self.parent_window.log(f"查询 {self.name} 参数...")
    
    def reset_params(self):
        """重置所有参数为0"""
        self.p_input.setValue(0)
        self.i_input.setValue(0)
        self.d_input.setValue(0)
        self.limit_input.setValue(0)
        self.ramp_input.setValue(100000)
    
    def update_display(self, p, i, d, limit, ramp):
        """更新显示的参数值"""
        self.p_display.setText(f"{p:.4f}")
        self.i_display.setText(f"{i:.4f}")
        self.d_display.setText(f"{d:.4f}")
        self.limit_display.setText(f"{limit:.4f}")
        self.ramp_display.setText(f"{ramp:.0f}")
    
    def update_plot(self, target, control):
        """更新波形数据"""
        if self.pause_plot_btn.isChecked():
            return
        
        self.data_counter += 1
        self.time_data.append(self.data_counter)
        self.target_data.append(target)
        self.control_data.append(control)
        
        self.target_curve.setData(list(self.time_data), list(self.target_data))
        self.control_curve.setData(list(self.time_data), list(self.control_data))
    
    def clear_plot(self):
        """清除波形数据"""
        self.time_data.clear()
        self.target_data.clear()
        self.control_data.clear()
        self.data_counter = 0
        self.target_curve.setData([], [])
        self.control_curve.setData([], [])


# ============================================================================
# 低通滤波器面板
# ============================================================================
class LPFControlPanel(QWidget):
    """低通滤波器参数面板"""
    
    def __init__(self, name, commander_id, parent=None):
        super().__init__(parent)
        self.name = name
        self.commander_id = commander_id
        self.parent_window = parent
        
        self.max_points = 500
        self.time_data = deque(maxlen=self.max_points)
        self.raw_data = deque(maxlen=self.max_points)       # 滤波前
        self.filtered_data = deque(maxlen=self.max_points)  # 滤波后
        self.data_counter = 0
        
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # Tf参数设置
        param_group = QGroupBox(f"{self.name} 参数设置")
        param_layout = QGridLayout()
        
        param_layout.addWidget(QLabel("Tf (时间常数):"), 0, 0)
        self.tf_input = QDoubleSpinBox()
        self.tf_input.setRange(0, 10)
        self.tf_input.setDecimals(4)
        self.tf_input.setSingleStep(0.01)
        self.tf_input.setValue(0.01)
        self.tf_input.setStyleSheet("font-size: 14px; padding: 5px;")
        param_layout.addWidget(self.tf_input, 0, 1)
        
        self.tf_set_btn = QPushButton("设置 Tf")
        self.tf_set_btn.clicked.connect(self.set_tf)
        param_layout.addWidget(self.tf_set_btn, 0, 2)
        
        param_group.setLayout(param_layout)
        layout.addWidget(param_group)
        
        # 快捷操作
        action_layout = QHBoxLayout()
        
        self.query_btn = QPushButton("🔍 查询Tf")
        self.query_btn.clicked.connect(self.query_tf)
        action_layout.addWidget(self.query_btn)
        
        action_layout.addStretch()
        layout.addLayout(action_layout)
        
        # 当前值显示
        display_group = QGroupBox("当前参数值")
        display_layout = QHBoxLayout()
        display_layout.addWidget(QLabel("Tf:"))
        self.tf_display = QLabel("--")
        self.tf_display.setStyleSheet("font-size: 16px; font-weight: bold; color: #00ff00; "
                                     "background-color: #2b2b2b; padding: 8px;")
        display_layout.addWidget(self.tf_display)
        display_layout.addStretch()
        display_group.setLayout(display_layout)
        layout.addWidget(display_group)
        
        # 波形 (滤波前后对比)
        plot_group = QGroupBox("实时波形 (蓝色=滤波前, 红色=滤波后)")
        plot_layout = QVBoxLayout()
        
        self.plot_widget = pg.PlotWidget()
        self.plot_widget.setBackground('k')
        self.plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.raw_curve = self.plot_widget.plot(pen=pg.mkPen(color='b', width=2), name='滤波前')
        self.filtered_curve = self.plot_widget.plot(pen=pg.mkPen(color='r', width=2), name='滤波后')
        self.plot_widget.addLegend()
        
        plot_layout.addWidget(self.plot_widget)
        
        plot_btn_layout = QHBoxLayout()
        self.clear_btn = QPushButton("清空")
        self.clear_btn.clicked.connect(self.clear_plot)
        plot_btn_layout.addWidget(self.clear_btn)
        plot_btn_layout.addStretch()
        plot_layout.addLayout(plot_btn_layout)
        
        plot_group.setLayout(plot_layout)
        layout.addWidget(plot_group)
    
    def set_tf(self):
        if not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        
        tf = self.tf_input.value()
        cmd = f"{self.commander_id}T{tf}"
        self.parent_window.send_command(cmd)
        self.parent_window.log(f"发送: {cmd} -> {self.name} Tf={tf}")
    
    def query_tf(self):
        if not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        
        cmd = f"{self.commander_id}?"
        self.parent_window.send_command(cmd)
        self.parent_window.log(f"查询 {self.name} Tf...")
    
    def update_display(self, tf):
        self.tf_display.setText(f"{tf:.4f}")
    
    def update_plot(self, raw_value, filtered_value=None):
        """更新波形 (支持滤波前后对比)"""
        self.data_counter += 1
        self.time_data.append(self.data_counter)
        self.raw_data.append(raw_value)
        
        if filtered_value is not None:
            self.filtered_data.append(filtered_value)
        else:
            self.filtered_data.append(raw_value)
        
        self.raw_curve.setData(list(self.time_data), list(self.raw_data))
        self.filtered_curve.setData(list(self.time_data), list(self.filtered_data))
    
    def clear_plot(self):
        self.time_data.clear()
        self.raw_data.clear()
        self.filtered_data.clear()
        self.data_counter = 0
        self.raw_curve.setData([], [])
        self.filtered_curve.setData([], [])


# ============================================================================
# Web监控面板
# ============================================================================
class WebMonitorPanel(QWidget):
    """Web命令监控面板"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # 状态显示
        status_group = QGroupBox("Web遥控器状态")
        status_layout = QGridLayout()
        status_layout.setSpacing(20)
        
        # Go状态
        status_layout.addWidget(QLabel("Go (启动):"), 0, 0)
        self.go_label = QLabel("⭕ 停止")
        self.go_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #ff4444;")
        status_layout.addWidget(self.go_label, 0, 1)
        
        # Dir状态
        status_layout.addWidget(QLabel("Dir (方向):"), 0, 2)
        self.dir_label = QLabel("0")
        self.dir_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #00aaff;")
        status_layout.addWidget(self.dir_label, 0, 3)
        
        # 摇杆X
        status_layout.addWidget(QLabel("JoyX:"), 1, 0)
        self.joyx_label = QLabel("0")
        self.joyx_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #00ff00;")
        status_layout.addWidget(self.joyx_label, 1, 1)
        
        # 摇杆Y
        status_layout.addWidget(QLabel("JoyY:"), 1, 2)
        self.joyy_label = QLabel("0")
        self.joyy_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #00ff00;")
        status_layout.addWidget(self.joyy_label, 1, 3)
        
        # 高度
        status_layout.addWidget(QLabel("Height (高度):"), 2, 0)
        self.height_label = QLabel("0")
        self.height_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #ffaa00;")
        status_layout.addWidget(self.height_label, 2, 1)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        # 历史记录
        history_group = QGroupBox("命令历史")
        history_layout = QVBoxLayout()
        
        self.history_text = QTextEdit()
        self.history_text.setReadOnly(True)
        self.history_text.setMaximumHeight(200)
        self.history_text.setStyleSheet("background-color: #1e1e1e; color: #d4d4d4; font-family: Consolas;")
        history_layout.addWidget(self.history_text)
        
        clear_btn = QPushButton("清空历史")
        clear_btn.clicked.connect(lambda: self.history_text.clear())
        history_layout.addWidget(clear_btn)
        
        history_group.setLayout(history_layout)
        layout.addWidget(history_group)
        
        layout.addStretch()
    
    def update_web_status(self, go, dir_val, joyx, joyy, height):
        """更新Web状态显示"""
        if go == "1":
            self.go_label.setText("🟢 运行")
            self.go_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #00ff00;")
        else:
            self.go_label.setText("⭕ 停止")
            self.go_label.setStyleSheet("font-size: 18px; font-weight: bold; color: #ff4444;")
        
        self.dir_label.setText(str(dir_val))
        self.joyx_label.setText(str(joyx))
        self.joyy_label.setText(str(joyy))
        self.height_label.setText(str(height))
        
        # 添加到历史
        timestamp = datetime.now().strftime("%H:%M:%S")
        status = "运行" if go == "1" else "停止"
        self.history_text.append(f"[{timestamp}] {status} | 摇杆({joyx},{joyy}) | 高度:{height}")


# ============================================================================
# 速度自适应P参数面板
# ============================================================================
class SpeedAdaptivePanel(QWidget):
    """速度环自适应P参数面板 - 根据机身高度自动调整Kp
    
    对应 lqr_balance.c 中的 lqr_adaptive_speed_p() 函数:
    - speed_kp_min: 高姿态时使用的最小P值 (稳定性优先)
    - speed_kp_max: 低姿态时使用的最大P值 (响应速度优先)
    
    算法: kp = kp_max - height_normalized * (kp_max - kp_min)
    """
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.commander_id = "M"
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # ===== Kp范围设置 =====
        range_group = QGroupBox("速度环自适应Kp范围 (对应 lqr_params_t)")
        range_layout = QGridLayout()
        range_layout.setSpacing(15)
        
        # Kp_Max (低姿态)
        range_layout.addWidget(QLabel("Kp_Max (低姿态时):"), 0, 0)
        self.kp_max_input = QDoubleSpinBox()
        self.kp_max_input.setRange(0, 10)
        self.kp_max_input.setDecimals(3)
        self.kp_max_input.setValue(1.0)  # 对应 default_params.speed_kp_max
        self.kp_max_input.setSingleStep(0.1)
        self.kp_max_input.setStyleSheet("font-size: 14px; padding: 5px;")
        range_layout.addWidget(self.kp_max_input, 0, 1)
        
        self.kp_max_btn = QPushButton("设置")
        self.kp_max_btn.clicked.connect(lambda: self.set_param('H', self.kp_max_input.value()))  # H = High/Max
        range_layout.addWidget(self.kp_max_btn, 0, 2)
        
        self.kp_max_display = self.create_display_label("--")
        range_layout.addWidget(self.kp_max_display, 0, 3)
        
        # Kp_Min (高姿态)
        range_layout.addWidget(QLabel("Kp_Min (高姿态时):"), 1, 0)
        self.kp_min_input = QDoubleSpinBox()
        self.kp_min_input.setRange(0, 10)
        self.kp_min_input.setDecimals(3)
        self.kp_min_input.setValue(0.3)  # 对应 default_params.speed_kp_min
        self.kp_min_input.setSingleStep(0.1)
        self.kp_min_input.setStyleSheet("font-size: 14px; padding: 5px;")
        range_layout.addWidget(self.kp_min_input, 1, 1)
        
        self.kp_min_btn = QPushButton("设置")
        self.kp_min_btn.clicked.connect(lambda: self.set_param('L', self.kp_min_input.value()))  # L = Low/Min
        range_layout.addWidget(self.kp_min_btn, 1, 2)
        
        self.kp_min_display = self.create_display_label("--")
        range_layout.addWidget(self.kp_min_display, 1, 3)
        
        range_group.setLayout(range_layout)
        layout.addWidget(range_group)
        
        # ===== 高度范围设置 =====
        height_group = QGroupBox("腿高度范围 (用于归一化计算)")
        height_layout = QGridLayout()
        height_layout.setSpacing(15)
        
        # 最小高度
        height_layout.addWidget(QLabel("最小高度 (m):"), 0, 0)
        self.height_min_input = QDoubleSpinBox()
        self.height_min_input.setRange(0, 1)
        self.height_min_input.setDecimals(3)
        self.height_min_input.setValue(0.1)  # 对应代码中的 0.1m
        self.height_min_input.setSingleStep(0.01)
        self.height_min_input.setStyleSheet("font-size: 14px; padding: 5px;")
        height_layout.addWidget(self.height_min_input, 0, 1)
        
        # 最大高度
        height_layout.addWidget(QLabel("最大高度 (m):"), 0, 2)
        self.height_max_input = QDoubleSpinBox()
        self.height_max_input.setRange(0, 1)
        self.height_max_input.setDecimals(3)
        self.height_max_input.setValue(0.3)  # 对应代码中的 0.3m
        self.height_max_input.setSingleStep(0.01)
        self.height_max_input.setStyleSheet("font-size: 14px; padding: 5px;")
        height_layout.addWidget(self.height_max_input, 0, 3)
        
        height_group.setLayout(height_layout)
        layout.addWidget(height_group)
        
        # ===== 实时计算预览 =====
        preview_group = QGroupBox("📊 实时Kp计算预览")
        preview_layout = QGridLayout()
        
        preview_layout.addWidget(QLabel("当前腿高度 (m):"), 0, 0)
        self.current_height_input = QDoubleSpinBox()
        self.current_height_input.setRange(0, 1)
        self.current_height_input.setDecimals(3)
        self.current_height_input.setValue(0.2)
        self.current_height_input.setSingleStep(0.01)
        self.current_height_input.valueChanged.connect(self.update_kp_preview)
        preview_layout.addWidget(self.current_height_input, 0, 1)
        
        preview_layout.addWidget(QLabel("计算得到的Kp:"), 0, 2)
        self.calculated_kp_display = self.create_display_label("--")
        self.calculated_kp_display.setStyleSheet("font-size: 16px; font-weight: bold; color: #00ff00; "
                                                  "background-color: #1a1a1a; padding: 8px; border-radius: 4px;")
        preview_layout.addWidget(self.calculated_kp_display, 0, 3)
        
        preview_group.setLayout(preview_layout)
        layout.addWidget(preview_group)
        
        # 连接信号以实时更新预览
        self.kp_max_input.valueChanged.connect(self.update_kp_preview)
        self.kp_min_input.valueChanged.connect(self.update_kp_preview)
        self.height_min_input.valueChanged.connect(self.update_kp_preview)
        self.height_max_input.valueChanged.connect(self.update_kp_preview)
        
        # 快捷操作
        action_layout = QHBoxLayout()
        
        self.query_btn = QPushButton("🔍 查询参数")
        self.query_btn.clicked.connect(self.query_params)
        action_layout.addWidget(self.query_btn)
        
        self.set_all_btn = QPushButton("📤 发送Kp范围")
        self.set_all_btn.clicked.connect(self.set_all_params)
        action_layout.addWidget(self.set_all_btn)
        
        action_layout.addStretch()
        layout.addLayout(action_layout)
        
        # 设计原理说明
        principle_group = QGroupBox("📚 算法原理 (对应 lqr_adaptive_speed_p)")
        principle_layout = QVBoxLayout()
        
        principle_text = QTextEdit()
        principle_text.setReadOnly(True)
        principle_text.setMaximumHeight(180)
        principle_text.setStyleSheet("font-size: 12px; background-color: #2b2b2b; color: #d4d4d4; padding: 8px;")
        principle_text.setHtml("""
        <b style="color: #00ff00;">增益调度算法 (Gain Scheduling):</b><br><br>
        <code style="color: #ffaa00;">
        height_normalized = (leg_height - height_min) / (height_max - height_min)<br>
        height_normalized = clamp(height_normalized, 0, 1)<br>
        kp = kp_max - height_normalized * (kp_max - kp_min)
        </code><br><br>
        <b>原理说明:</b><br>
        • <span style="color: #ff6666;">低姿态 (height→min)</span>: 重心低、惯性小 → 使用 <b>Kp_Max</b>，响应更灵敏<br>
        • <span style="color: #66ff66;">高姿态 (height→max)</span>: 重心高、惯性大 → 使用 <b>Kp_Min</b>，避免振荡<br><br>
        <span style="color: #aaaaaa;">参数存储在 lqr_params_t 结构体的 speed_kp_min 和 speed_kp_max 中</span>
        """)
        principle_layout.addWidget(principle_text)
        principle_group.setLayout(principle_layout)
        layout.addWidget(principle_group)
        
        layout.addStretch()
        
        # 初始化预览
        self.update_kp_preview()
    
    def create_display_label(self, text):
        label = QLabel(text)
        label.setStyleSheet("font-size: 14px; font-weight: bold; color: #00ff00; "
                          "background-color: #2b2b2b; padding: 5px;")
        label.setAlignment(Qt.AlignCenter)
        label.setMinimumWidth(80)
        return label
    
    def update_kp_preview(self):
        """实时计算并显示当前高度对应的Kp值"""
        kp_max = self.kp_max_input.value()
        kp_min = self.kp_min_input.value()
        height_min = self.height_min_input.value()
        height_max = self.height_max_input.value()
        current_height = self.current_height_input.value()
        
        # 防止除零
        height_range = height_max - height_min
        if height_range <= 0:
            height_range = 0.2
        
        # 归一化
        height_normalized = (current_height - height_min) / height_range
        height_normalized = max(0.0, min(1.0, height_normalized))
        
        # 计算Kp
        kp = kp_max - height_normalized * (kp_max - kp_min)
        
        self.calculated_kp_display.setText(f"{kp:.3f}")
    
    def set_param(self, param_type, value):
        if not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        
        cmd = f"{self.commander_id}{param_type}{value:.4f}"
        
        if self.parent_window.send_command(cmd):
            param_names = {'L': 'Kp_Min', 'H': 'Kp_Max'}
            self.parent_window.log(f"发送: {cmd} -> 速度环 {param_names.get(param_type, param_type)}={value:.4f}")
        else:
            self.parent_window.log(f"✗ 发送失败: {cmd}", is_error=True)
    
    def set_all_params(self):
        if not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        
        params = [
            ('H', self.kp_max_input.value()),  # Kp_Max
            ('L', self.kp_min_input.value()),  # Kp_Min
        ]
        
        for param_type, value in params:
            self.set_param(param_type, value)
            QApplication.processEvents()
    
    def query_params(self):
        if not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        
        cmd = f"{self.commander_id}?"
        self.parent_window.send_command(cmd)
        self.parent_window.log(f"查询速度环自适应Kp范围参数...")
    
    def update_display(self, kp_min, kp_max):
        """更新显示 (从设备返回的数据)"""
        self.kp_min_display.setText(f"{kp_min:.3f}")
        self.kp_max_display.setText(f"{kp_max:.3f}")
        # 同步更新输入框
        self.kp_min_input.setValue(kp_min)
        self.kp_max_input.setValue(kp_max)


# ============================================================================
# 腿部控制面板 (独立于平衡控制)
# ============================================================================
class LegControlPanel(QWidget):
    """腿部电机控制面板 - 运动学控制 + 直接角度控制
    
    功能:
    - 运动学控制: 通过腿长和身体夹角控制腿部姿态
    - 直接角度控制: 直接设置髋关节和膝关节电机角度
    - 运动学测试: 正运动学(FK)和逆运动学(IK)计算验证
    - 预设姿态: 常用姿态快捷设置
    
    对应串口命令:
    - balance leg on/off: 使能/禁用腿部电机
    - balance leg set <lh> <lk> <rh> <rk>: 直接设置电机角度
    - balance leg target <length> <angle> [left|right|both]: 运动学目标设置
    - balance leg speed <rpm>: 设置腿部移动速度
    - balance leg status: 查询腿部状态
    - balance leg test fk/ik: 运动学测试
    """
    
    # 运动学参数 (与 leg_kinematics.h 保持一致)
    LEG_THIGH_LENGTH = 0.10  # 大腿长度 (m)
    LEG_SHANK_LENGTH = 0.10  # 小腿长度 (m)
    DEFAULT_LEG_LENGTH_MIN = 0.07    # 默认最小腿长 (m) - 与 C 代码一致
    DEFAULT_LEG_LENGTH_MAX = 0.17    # 默认最大腿长 (m) - 与 C 代码一致
    LEG_BODY_ANGLE_MIN = -160.0  # 最小身体夹角 (度), 向前蹬腿
    LEG_BODY_ANGLE_MAX = -20.0   # 最大身体夹角 (度), 向后蹬腿
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        # 可调节的腿长范围
        self.leg_length_min = self.DEFAULT_LEG_LENGTH_MIN
        self.leg_length_max = self.DEFAULT_LEG_LENGTH_MAX
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # ===== 警告提示 =====
        warning_label = QLabel("⚠️ 重要: 必须先在【平衡控制】面板执行【初始化平衡系统】才能使用腿部控制!")
        warning_label.setStyleSheet(
            "background-color: #ff9800; color: black; padding: 8px; "
            "border-radius: 4px; font-weight: bold;"
        )
        warning_label.setWordWrap(True)
        layout.addWidget(warning_label)
        
        # ===== 腿部电机使能 =====
        enable_group = QGroupBox("🦿 腿部电机控制")
        enable_layout = QHBoxLayout()
        
        self.leg_status = QLabel("状态: 未知")
        self.leg_status.setStyleSheet("font-size: 14px; font-weight: bold;")
        enable_layout.addWidget(self.leg_status)
        
        enable_layout.addStretch()
        
        self.leg_on_btn = QPushButton("✅ 使能腿部")
        self.leg_on_btn.setStyleSheet("background-color: #4CAF50; color: white; padding: 8px 20px;")
        self.leg_on_btn.clicked.connect(lambda: self.send_leg_cmd("on"))
        enable_layout.addWidget(self.leg_on_btn)
        
        self.leg_off_btn = QPushButton("❌ 禁用腿部")
        self.leg_off_btn.setStyleSheet("background-color: #f44336; color: white; padding: 8px 20px;")
        self.leg_off_btn.clicked.connect(lambda: self.send_leg_cmd("off"))
        enable_layout.addWidget(self.leg_off_btn)
        
        self.leg_status_btn = QPushButton("📊 状态")
        self.leg_status_btn.clicked.connect(lambda: self.send_leg_cmd("status"))
        enable_layout.addWidget(self.leg_status_btn)
        
        enable_group.setLayout(enable_layout)
        layout.addWidget(enable_group)
        
        # ===== 腿长范围设置 =====
        range_group = QGroupBox("📏 腿长范围设置")
        range_layout = QGridLayout()
        range_layout.setSpacing(8)
        
        # 最小腿长
        range_layout.addWidget(QLabel("最小腿长 (m):"), 0, 0)
        self.leg_min_input = QDoubleSpinBox()
        self.leg_min_input.setRange(0.05, 0.15)  # 硬件限制范围
        self.leg_min_input.setValue(self.leg_length_min)
        self.leg_min_input.setDecimals(3)
        self.leg_min_input.setSingleStep(0.005)
        self.leg_min_input.setStyleSheet("font-size: 12px; padding: 3px;")
        self.leg_min_input.valueChanged.connect(self.on_leg_range_changed)
        range_layout.addWidget(self.leg_min_input, 0, 1)
        
        # 最大腿长
        range_layout.addWidget(QLabel("最大腿长 (m):"), 0, 2)
        self.leg_max_input = QDoubleSpinBox()
        self.leg_max_input.setRange(0.10, 0.20)  # 硬件限制范围
        self.leg_max_input.setValue(self.leg_length_max)
        self.leg_max_input.setDecimals(3)
        self.leg_max_input.setSingleStep(0.005)
        self.leg_max_input.setStyleSheet("font-size: 12px; padding: 3px;")
        self.leg_max_input.valueChanged.connect(self.on_leg_range_changed)
        range_layout.addWidget(self.leg_max_input, 0, 3)
        
        # 发送到ESP32按钮
        self.send_range_btn = QPushButton("📤 发送到ESP32")
        self.send_range_btn.setStyleSheet("background-color: #9C27B0; color: white; padding: 5px 15px;")
        self.send_range_btn.clicked.connect(self.send_leg_range_to_esp)
        range_layout.addWidget(self.send_range_btn, 0, 4)
        
        # 重置按钮
        self.reset_range_btn = QPushButton("🔄 重置")
        self.reset_range_btn.setToolTip("重置为默认值 (0.07 ~ 0.17)")
        self.reset_range_btn.clicked.connect(self.reset_leg_range)
        range_layout.addWidget(self.reset_range_btn, 0, 5)
        
        # 说明标签
        range_hint = QLabel("💡 修改范围后会自动更新下方滑块，发送到ESP32可同步设备端限制")
        range_hint.setStyleSheet("font-size: 10px; color: #888;")
        range_layout.addWidget(range_hint, 1, 0, 1, 6)
        
        range_group.setLayout(range_layout)
        layout.addWidget(range_group)
        
        # ===== 运动学控制 (腿长 + 身体夹角) =====
        kin_group = QGroupBox("📐 运动学控制 (腿长 + 身体夹角)")
        kin_layout = QGridLayout()
        kin_layout.setSpacing(10)
        
        # 腿长设置
        kin_layout.addWidget(QLabel("腿长 (m):"), 0, 0)
        self.leg_length_input = QDoubleSpinBox()
        self.leg_length_input.setRange(self.leg_length_min, self.leg_length_max)
        self.leg_length_input.setValue(0.15)
        self.leg_length_input.setDecimals(3)
        self.leg_length_input.setSingleStep(0.005)
        self.leg_length_input.setStyleSheet("font-size: 14px; padding: 5px;")
        self.leg_length_input.valueChanged.connect(self.update_ik_preview)
        kin_layout.addWidget(self.leg_length_input, 0, 1)
        
        # 腿长滑块
        self.leg_length_slider = QSlider(Qt.Horizontal)
        self.leg_length_slider.setRange(int(self.leg_length_min * 1000), int(self.leg_length_max * 1000))
        self.leg_length_slider.setValue(150)
        self.leg_length_slider.valueChanged.connect(lambda v: self.leg_length_input.setValue(v / 1000.0))
        self.leg_length_input.valueChanged.connect(lambda v: self.leg_length_slider.setValue(int(v * 1000)))
        kin_layout.addWidget(self.leg_length_slider, 0, 2, 1, 2)
        
        # 身体夹角设置
        kin_layout.addWidget(QLabel("身体夹角 (°):"), 1, 0)
        self.body_angle_input = QDoubleSpinBox()
        self.body_angle_input.setRange(self.LEG_BODY_ANGLE_MIN, self.LEG_BODY_ANGLE_MAX)
        self.body_angle_input.setValue(-90)  # 默认垂直向下
        self.body_angle_input.setDecimals(1)
        self.body_angle_input.setSingleStep(1)
        self.body_angle_input.setStyleSheet("font-size: 14px; padding: 5px;")
        self.body_angle_input.valueChanged.connect(self.update_ik_preview)
        kin_layout.addWidget(self.body_angle_input, 1, 1)
        
        # 身体夹角滑块
        self.body_angle_slider = QSlider(Qt.Horizontal)
        self.body_angle_slider.setRange(int(self.LEG_BODY_ANGLE_MIN), int(self.LEG_BODY_ANGLE_MAX))
        self.body_angle_slider.setValue(-90)  # 默认垂直向下
        self.body_angle_slider.valueChanged.connect(lambda v: self.body_angle_input.setValue(v))
        self.body_angle_input.valueChanged.connect(lambda v: self.body_angle_slider.setValue(int(v)))
        kin_layout.addWidget(self.body_angle_slider, 1, 2, 1, 2)
        
        # 目标选择
        kin_layout.addWidget(QLabel("目标:"), 2, 0)
        self.target_combo = QComboBox()
        self.target_combo.addItems(["双腿 (both)", "仅左腿 (left)", "仅右腿 (right)"])
        kin_layout.addWidget(self.target_combo, 2, 1)
        
        # 发送按钮
        self.send_kin_btn = QPushButton("📤 发送运动学目标")
        self.send_kin_btn.setStyleSheet("background-color: #2196F3; color: white; padding: 10px; font-size: 14px;")
        self.send_kin_btn.clicked.connect(self.send_kinematics_target)
        kin_layout.addWidget(self.send_kin_btn, 2, 2, 1, 2)
        
        # IK 预览显示 - 电机角度
        kin_layout.addWidget(QLabel("IK预览 (电机):"), 3, 0)
        self.ik_preview_label = QLabel("Hip: --, Knee: --")
        self.ik_preview_label.setStyleSheet("font-size: 12px; color: #888; padding: 5px;")
        kin_layout.addWidget(self.ik_preview_label, 3, 1, 1, 3)
        
        # IK 预览显示 - 运动学角度 theta1/theta2
        kin_layout.addWidget(QLabel("IK预览 (θ1,θ2):"), 4, 0)
        self.ik_theta_label = QLabel("θ1: --, θ2: --")
        self.ik_theta_label.setStyleSheet("font-size: 12px; color: #00ccff; padding: 5px; font-weight: bold;")
        kin_layout.addWidget(self.ik_theta_label, 4, 1, 1, 3)
        
        kin_group.setLayout(kin_layout)
        layout.addWidget(kin_group)
        
        # ===== 直接角度控制 =====
        angle_group = QGroupBox("🔧 直接角度控制 (电机角度)")
        angle_layout = QGridLayout()
        angle_layout.setSpacing(10)
        
        # 左腿
        angle_layout.addWidget(QLabel("左髋 (L_Hip):"), 0, 0)
        self.left_hip_input = QDoubleSpinBox()
        self.left_hip_input.setRange(-180, 180)
        self.left_hip_input.setValue(-105)  # 默认站立
        self.left_hip_input.setDecimals(1)
        self.left_hip_input.setSingleStep(5)
        self.left_hip_input.setStyleSheet("font-size: 14px; padding: 5px;")
        angle_layout.addWidget(self.left_hip_input, 0, 1)
        
        angle_layout.addWidget(QLabel("左膝 (L_Knee):"), 1, 0)
        self.left_knee_input = QDoubleSpinBox()
        self.left_knee_input.setRange(-180, 180)
        self.left_knee_input.setValue(60)  # 默认站立
        self.left_knee_input.setDecimals(1)
        self.left_knee_input.setSingleStep(5)
        self.left_knee_input.setStyleSheet("font-size: 14px; padding: 5px;")
        angle_layout.addWidget(self.left_knee_input, 1, 1)
        
        # 右腿
        angle_layout.addWidget(QLabel("右髋 (R_Hip):"), 0, 2)
        self.right_hip_input = QDoubleSpinBox()
        self.right_hip_input.setRange(-180, 180)
        self.right_hip_input.setValue(105)  # 右腿镜像
        self.right_hip_input.setDecimals(1)
        self.right_hip_input.setSingleStep(5)
        self.right_hip_input.setStyleSheet("font-size: 14px; padding: 5px;")
        angle_layout.addWidget(self.right_hip_input, 0, 3)
        
        angle_layout.addWidget(QLabel("右膝 (R_Knee):"), 1, 2)
        self.right_knee_input = QDoubleSpinBox()
        self.right_knee_input.setRange(-180, 180)
        self.right_knee_input.setValue(-60)  # 右腿镜像
        self.right_knee_input.setDecimals(1)
        self.right_knee_input.setSingleStep(5)
        self.right_knee_input.setStyleSheet("font-size: 14px; padding: 5px;")
        angle_layout.addWidget(self.right_knee_input, 1, 3)
        
        # 同步选项
        self.sync_checkbox = QPushButton("🔗 左右镜像同步")
        self.sync_checkbox.setCheckable(True)
        self.sync_checkbox.setChecked(True)
        self.sync_checkbox.clicked.connect(self.on_sync_toggled)
        angle_layout.addWidget(self.sync_checkbox, 2, 0, 1, 2)
        
        # 连接左侧输入到右侧 (同步时)
        self.left_hip_input.valueChanged.connect(self.sync_left_to_right)
        self.left_knee_input.valueChanged.connect(self.sync_left_to_right)
        
        # 发送角度按钮
        self.send_angle_btn = QPushButton("📤 发送电机角度")
        self.send_angle_btn.setStyleSheet("background-color: #FF9800; color: white; padding: 10px; font-size: 14px;")
        self.send_angle_btn.clicked.connect(self.send_leg_angles)
        angle_layout.addWidget(self.send_angle_btn, 2, 2, 1, 2)
        
        angle_group.setLayout(angle_layout)
        layout.addWidget(angle_group)
        
        # ===== 速度设置 =====
        speed_group = QGroupBox("⚡ 腿部移动速度")
        speed_layout = QHBoxLayout()
        
        speed_layout.addWidget(QLabel("速度 (rpm):"))
        self.leg_speed_input = QDoubleSpinBox()
        self.leg_speed_input.setRange(1, 200)
        self.leg_speed_input.setValue(50)
        self.leg_speed_input.setDecimals(0)
        self.leg_speed_input.setSingleStep(10)
        self.leg_speed_input.setStyleSheet("font-size: 14px; padding: 5px;")
        speed_layout.addWidget(self.leg_speed_input)
        
        self.set_speed_btn = QPushButton("设置速度")
        self.set_speed_btn.clicked.connect(self.send_leg_speed)
        speed_layout.addWidget(self.set_speed_btn)
        
        speed_layout.addStretch()
        speed_group.setLayout(speed_layout)
        layout.addWidget(speed_group)
        
        # ===== 预设姿态 (运动学) =====
        preset_group = QGroupBox("🎯 预设姿态 (运动学)")
        preset_layout = QGridLayout()
        
        # 运动学预设 (腿长, 身体夹角) - 垂直向下是-90°
        kin_presets = [
            ("站立", 0.14, -90),    # 正常站立，垂直向下
            ("半蹲", 0.12, -90),    # 半蹲
            ("蹲下", 0.07, -90),    # 蹲下 (最小腿长)
            ("伸直", 0.17, -90),    # 腿伸直
            ("前蹬", 0.14, -120),   # 向前蹬腿
            ("后蹬", 0.14, -60),    # 向后蹬腿
        ]
        
        for i, (name, length, angle) in enumerate(kin_presets):
            btn = QPushButton(f"{name}\n({length}m, {angle}°)")
            btn.setStyleSheet("padding: 8px;")
            btn.clicked.connect(lambda checked, l=length, a=angle: self.apply_kin_preset(l, a))
            preset_layout.addWidget(btn, i // 3, i % 3)
        
        preset_group.setLayout(preset_layout)
        layout.addWidget(preset_group)
        
        # ===== 运动学测试 =====
        test_group = QGroupBox("🧪 运动学测试 (FK/IK)")
        test_layout = QVBoxLayout()
        
        # FK 测试
        fk_layout = QHBoxLayout()
        fk_layout.addWidget(QLabel("FK: Hip="))
        self.fk_hip_input = QDoubleSpinBox()
        self.fk_hip_input.setRange(-180, 180)
        self.fk_hip_input.setValue(-105)
        self.fk_hip_input.setDecimals(1)
        fk_layout.addWidget(self.fk_hip_input)
        
        fk_layout.addWidget(QLabel("Knee="))
        self.fk_knee_input = QDoubleSpinBox()
        self.fk_knee_input.setRange(-180, 180)
        self.fk_knee_input.setValue(60)
        self.fk_knee_input.setDecimals(1)
        fk_layout.addWidget(self.fk_knee_input)
        
        self.fk_side_combo = QComboBox()
        self.fk_side_combo.addItems(["left", "right"])
        fk_layout.addWidget(self.fk_side_combo)
        
        self.fk_test_btn = QPushButton("计算 FK")
        self.fk_test_btn.clicked.connect(self.test_fk)
        fk_layout.addWidget(self.fk_test_btn)
        
        test_layout.addLayout(fk_layout)
        
        # IK 测试
        ik_layout = QHBoxLayout()
        ik_layout.addWidget(QLabel("IK: L="))
        self.ik_length_input = QDoubleSpinBox()
        self.ik_length_input.setRange(0.07, 0.17)
        self.ik_length_input.setValue(0.14)
        self.ik_length_input.setDecimals(3)
        ik_layout.addWidget(self.ik_length_input)
        
        ik_layout.addWidget(QLabel("Angle="))
        self.ik_angle_input = QDoubleSpinBox()
        self.ik_angle_input.setRange(-160, -20)
        self.ik_angle_input.setValue(-90)  # 垂直向下
        self.ik_angle_input.setDecimals(1)
        ik_layout.addWidget(self.ik_angle_input)
        
        self.ik_side_combo = QComboBox()
        self.ik_side_combo.addItems(["left", "right"])
        ik_layout.addWidget(self.ik_side_combo)
        
        self.ik_test_btn = QPushButton("计算 IK")
        self.ik_test_btn.clicked.connect(self.test_ik)
        ik_layout.addWidget(self.ik_test_btn)
        
        test_layout.addLayout(ik_layout)
        
        # 测试结果显示
        self.test_result_label = QLabel("测试结果: --")
        self.test_result_label.setStyleSheet("font-size: 12px; color: #00ff00; background-color: #1a1a2e; padding: 8px;")
        test_layout.addWidget(self.test_result_label)
        
        test_group.setLayout(test_layout)
        layout.addWidget(test_group)
        
        # ===== 腿部状态显示 =====
        state_group = QGroupBox("📊 当前腿部状态")
        state_layout = QGridLayout()
        
        state_layout.addWidget(QLabel(""), 0, 0)
        state_layout.addWidget(QLabel("左腿"), 0, 1)
        state_layout.addWidget(QLabel("右腿"), 0, 2)
        
        state_layout.addWidget(QLabel("腿长 (m):"), 1, 0)
        self.left_length_label = self.create_state_label()
        state_layout.addWidget(self.left_length_label, 1, 1)
        self.right_length_label = self.create_state_label()
        state_layout.addWidget(self.right_length_label, 1, 2)
        
        state_layout.addWidget(QLabel("身体夹角 (°):"), 2, 0)
        self.left_angle_label = self.create_state_label()
        state_layout.addWidget(self.left_angle_label, 2, 1)
        self.right_angle_label = self.create_state_label()
        state_layout.addWidget(self.right_angle_label, 2, 2)
        
        state_layout.addWidget(QLabel("髋关节 (°):"), 3, 0)
        self.left_hip_label = self.create_state_label()
        state_layout.addWidget(self.left_hip_label, 3, 1)
        self.right_hip_label = self.create_state_label()
        state_layout.addWidget(self.right_hip_label, 3, 2)
        
        state_layout.addWidget(QLabel("膝关节 (°):"), 4, 0)
        self.left_knee_label = self.create_state_label()
        state_layout.addWidget(self.left_knee_label, 4, 1)
        self.right_knee_label = self.create_state_label()
        state_layout.addWidget(self.right_knee_label, 4, 2)
        
        # 添加 theta1/theta2 显示 (从编码器计算)
        state_layout.addWidget(QLabel("θ1 (大腿角):"), 5, 0)
        self.left_theta1_label = self.create_state_label("#ffcc00")
        state_layout.addWidget(self.left_theta1_label, 5, 1)
        self.right_theta1_label = self.create_state_label("#ffcc00")
        state_layout.addWidget(self.right_theta1_label, 5, 2)
        
        state_layout.addWidget(QLabel("θ2 (膝角):"), 6, 0)
        self.left_theta2_label = self.create_state_label("#ffcc00")
        state_layout.addWidget(self.left_theta2_label, 6, 1)
        self.right_theta2_label = self.create_state_label("#ffcc00")
        state_layout.addWidget(self.right_theta2_label, 6, 2)
        
        state_group.setLayout(state_layout)
        layout.addWidget(state_group)
        
        layout.addStretch()
        
        # 初始化 IK 预览
        self.update_ik_preview()
    
    def create_state_label(self, color="#00ccff"):
        """创建状态显示标签"""
        label = QLabel("--")
        label.setStyleSheet(f"font-size: 14px; font-weight: bold; color: {color}; "
                          "background-color: #1a1a2e; padding: 5px; border-radius: 3px;")
        label.setAlignment(Qt.AlignCenter)
        label.setMinimumWidth(80)
        return label
    
    def on_leg_range_changed(self):
        """腿长范围改变时更新UI控件的范围"""
        new_min = self.leg_min_input.value()
        new_max = self.leg_max_input.value()
        
        # 确保最小值小于最大值
        if new_min >= new_max:
            return
        
        self.leg_length_min = new_min
        self.leg_length_max = new_max
        
        # 更新腿长输入框的范围
        current_value = self.leg_length_input.value()
        self.leg_length_input.setRange(new_min, new_max)
        
        # 如果当前值超出新范围，调整到范围内
        if current_value < new_min:
            self.leg_length_input.setValue(new_min)
        elif current_value > new_max:
            self.leg_length_input.setValue(new_max)
        
        # 更新腿长滑块的范围
        self.leg_length_slider.setRange(int(new_min * 1000), int(new_max * 1000))
        
    def reset_leg_range(self):
        """重置腿长范围为默认值"""
        self.leg_min_input.setValue(self.DEFAULT_LEG_LENGTH_MIN)
        self.leg_max_input.setValue(self.DEFAULT_LEG_LENGTH_MAX)
        
    def send_leg_range_to_esp(self):
        """发送腿长范围到ESP32"""
        min_len = self.leg_min_input.value()
        max_len = self.leg_max_input.value()
        
        if min_len >= max_len:
            self.log_message(f"❌ 错误: 最小腿长({min_len})必须小于最大腿长({max_len})")
            return
        
        # 发送设置命令到ESP32
        cmd = f"balance leg range {min_len:.3f} {max_len:.3f}"
        if self.parent_window and hasattr(self.parent_window, 'send_command'):
            self.parent_window.send_command(cmd)
            self.log_message(f"📤 发送腿长范围: {min_len:.3f}m ~ {max_len:.3f}m")
        else:
            self.log_message(f"❌ 无法发送命令，请先连接串口")
    
    def update_ik_preview(self):
        """更新逆运动学预览 (本地计算) - 与 C 代码 leg_kinematics.c 保持一致"""
        import math
        
        length = self.leg_length_input.value()
        body_angle = self.body_angle_input.value()
        
        L1 = self.LEG_THIGH_LENGTH
        L2 = self.LEG_SHANK_LENGTH
        
        # 检查可达性
        L_min = abs(L1 - L2) + 0.001
        L_max = L1 + L2 - 0.001
        if length < L_min or length > L_max:
            self.ik_preview_label.setText("⚠️ 目标不可达!")
            self.ik_preview_label.setStyleSheet("font-size: 12px; color: #ff4444; padding: 5px;")
            self.ik_theta_label.setText("θ1: --, θ2: --")
            return
        
        # ===== 与 C 代码一致的 IK 算法 =====
        # 左腿 offset (校准: Hip=-60°, Knee=-55° → body_angle=-90°, theta2=-90°)
        hip_offset = -15.0
        knee_offset = 35.0
        
        # Step 1: 余弦定理求膝关节角度 theta2
        cos_theta2 = (length*length - L1*L1 - L2*L2) / (2 * L1 * L2)
        cos_theta2 = max(-1.0, min(1.0, cos_theta2))
        theta2 = -math.acos(cos_theta2)  # 取负号，因为弯曲方向是负的
        theta2_deg = math.degrees(theta2)
        
        # Step 2: 计算 beta 角
        sin_theta2 = math.sin(theta2)
        beta = math.atan2(L2 * sin_theta2, L1 + L2 * cos_theta2)
        
        # Step 3: 身体夹角 -> 大腿角度
        alpha = math.radians(body_angle)
        theta1 = alpha - beta
        theta1_deg = math.degrees(theta1)
        
        # Step 4: 运动学角度 -> 电机角度
        hip_motor = theta1_deg + hip_offset
        knee_motor = theta2_deg + knee_offset
        
        # 右腿镜像
        right_hip_motor = -theta1_deg + (-hip_offset)  # 右腿 offset 符号相反
        right_knee_motor = -theta2_deg + (-knee_offset)
        
        self.ik_preview_label.setText(
            f"左腿: Hip={hip_motor:.1f}°, Knee={knee_motor:.1f}° | "
            f"右腿: Hip={right_hip_motor:.1f}°, Knee={right_knee_motor:.1f}°"
        )
        self.ik_preview_label.setStyleSheet("font-size: 12px; color: #888; padding: 5px;")
        
        # 显示 theta1/theta2 (运动学角度)
        self.ik_theta_label.setText(f"θ1={theta1_deg:.1f}° (大腿), θ2={theta2_deg:.1f}° (膝关节, 0=伸直, 负=弯曲)")
        self.ik_theta_label.setStyleSheet("font-size: 12px; color: #00ccff; padding: 5px; font-weight: bold;")
    
    def on_sync_toggled(self, checked):
        if checked:
            self.sync_checkbox.setText("🔗 左右镜像同步 (开)")
            self.sync_left_to_right()
        else:
            self.sync_checkbox.setText("🔗 左右镜像同步 (关)")
    
    def sync_left_to_right(self):
        """左右腿镜像同步"""
        if self.sync_checkbox.isChecked():
            # 右腿是左腿的镜像
            self.right_hip_input.setValue(-self.left_hip_input.value())
            self.right_knee_input.setValue(-self.left_knee_input.value())
    
    def apply_kin_preset(self, length, angle):
        """应用运动学预设"""
        self.leg_length_input.setValue(length)
        self.body_angle_input.setValue(angle)
    
    def send_leg_cmd(self, state):
        """发送腿部控制命令"""
        if not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        
        # 检查是否已初始化平衡系统
        if hasattr(self.parent_window, 'balance_panel') and \
           not self.parent_window.balance_panel.balance_initialized:
            QMessageBox.warning(self, "警告", 
                "请先在【平衡控制】面板点击【初始化平衡系统】!\n\n"
                "腿部电机需要 balance init 命令创建电机句柄后才能使能。")
            return
        
        cmd = f"balance leg {state}"
        if self.parent_window.send_command(cmd):
            self.parent_window.log(f"发送: {cmd}")
            if state == "on":
                self.leg_status.setText("状态: 已使能")
                self.leg_status.setStyleSheet("font-size: 14px; font-weight: bold; color: #4CAF50;")
            elif state == "off":
                self.leg_status.setText("状态: 已禁用")
                self.leg_status.setStyleSheet("font-size: 14px; font-weight: bold; color: #f44336;")
    
    def send_kinematics_target(self):
        """发送运动学目标"""
        if not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        
        # 检查是否已初始化
        if hasattr(self.parent_window, 'balance_panel') and \
           not self.parent_window.balance_panel.balance_initialized:
            QMessageBox.warning(self, "警告", 
                "请先在【平衡控制】面板点击【初始化平衡系统】!")
            return
        
        length = self.leg_length_input.value()
        angle = self.body_angle_input.value()
        
        target_map = {0: "both", 1: "left", 2: "right"}
        target = target_map[self.target_combo.currentIndex()]
        
        cmd = f"balance leg target {length:.3f} {angle:.1f} {target}"
        if self.parent_window.send_command(cmd):
            self.parent_window.log(f"发送: {cmd}")
    
    def send_leg_angles(self):
        """发送直接电机角度"""
        if not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        
        # 检查是否已初始化
        if hasattr(self.parent_window, 'balance_panel') and \
           not self.parent_window.balance_panel.balance_initialized:
            QMessageBox.warning(self, "警告", 
                "请先在【平衡控制】面板点击【初始化平衡系统】!")
            return
        
        lh = self.left_hip_input.value()
        lk = self.left_knee_input.value()
        rh = self.right_hip_input.value()
        rk = self.right_knee_input.value()
        
        cmd = f"balance leg set {lh:.1f} {lk:.1f} {rh:.1f} {rk:.1f}"
        if self.parent_window.send_command(cmd):
            self.parent_window.log(f"发送: {cmd}")
    
    def send_leg_speed(self):
        """发送腿部速度"""
        if not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        
        spd = self.leg_speed_input.value()
        cmd = f"balance leg speed {spd:.0f}"
        if self.parent_window.send_command(cmd):
            self.parent_window.log(f"发送: {cmd}")
    
    def test_fk(self):
        """正运动学测试 (发送到设备)"""
        if not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        
        hip = self.fk_hip_input.value()
        knee = self.fk_knee_input.value()
        side = self.fk_side_combo.currentText()
        
        cmd = f"balance leg test fk {hip:.1f} {knee:.1f} {side}"
        if self.parent_window.send_command(cmd):
            self.parent_window.log(f"发送: {cmd}")
    
    def test_ik(self):
        """逆运动学测试 (发送到设备)"""
        if not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        
        length = self.ik_length_input.value()
        angle = self.ik_angle_input.value()
        side = self.ik_side_combo.currentText()
        
        cmd = f"balance leg test ik {length:.3f} {angle:.1f} {side}"
        if self.parent_window.send_command(cmd):
            self.parent_window.log(f"发送: {cmd}")
    
    def update_leg_state(self, left_state, right_state):
        """更新腿部状态显示
        
        Args:
            left_state: dict with keys: length, angle, hip, knee
            right_state: dict with keys: length, angle, hip, knee
        """
        # 左腿 offset (校准: Hip=-60°, Knee=-55° → body_angle=-90°)
        left_hip_offset = -15.0
        left_knee_offset = 35.0
        # 右腿 offset (镜像)
        right_hip_offset = 15.0
        right_knee_offset = -35.0
        
        if left_state:
            self.left_length_label.setText(f"{left_state.get('length', 0):.3f}")
            self.left_angle_label.setText(f"{left_state.get('angle', 0):.1f}")
            self.left_hip_label.setText(f"{left_state.get('hip', 0):.1f}")
            self.left_knee_label.setText(f"{left_state.get('knee', 0):.1f}")
            
            # 计算 theta1/theta2 从编码器 (正运动学的逆操作)
            hip_motor = left_state.get('hip', 0)
            knee_motor = left_state.get('knee', 0)
            theta1 = hip_motor - left_hip_offset
            theta2 = knee_motor - left_knee_offset
            self.left_theta1_label.setText(f"{theta1:.1f}")
            self.left_theta2_label.setText(f"{theta2:.1f}")
        
        if right_state:
            self.right_length_label.setText(f"{right_state.get('length', 0):.3f}")
            self.right_angle_label.setText(f"{right_state.get('angle', 0):.1f}")
            self.right_hip_label.setText(f"{right_state.get('hip', 0):.1f}")
            self.right_knee_label.setText(f"{right_state.get('knee', 0):.1f}")
            
            # 计算 theta1/theta2 从编码器 (右腿镜像)
            hip_motor = right_state.get('hip', 0)
            knee_motor = right_state.get('knee', 0)
            # 右腿镜像: theta = -(motor - offset)
            theta1 = -(hip_motor - right_hip_offset)
            theta2 = -(knee_motor - right_knee_offset)
            self.right_theta1_label.setText(f"{theta1:.1f}")
            self.right_theta2_label.setText(f"{theta2:.1f}")
    
    def update_test_result(self, result_text):
        """更新测试结果显示"""
        self.test_result_label.setText(result_text)
        if "failed" in result_text.lower() or "error" in result_text.lower():
            self.test_result_label.setStyleSheet("font-size: 12px; color: #ff4444; background-color: #1a1a2e; padding: 8px;")
        else:
            self.test_result_label.setStyleSheet("font-size: 12px; color: #00ff00; background-color: #1a1a2e; padding: 8px;")


# ============================================================================
# 电机控制面板 (Motor Control)
# ============================================================================
class MotorControlPanel(QWidget):
    """电机控制面板 - 替代命令行的电机控制"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.motor_states = {}
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # 全局控制
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
        
        motor_layout.addWidget(QLabel("电机:"), 0, 0)
        self.motor_combo = QComboBox()
        self.motor_combo.addItems([
            "1 - 左髋 (L_Hip)", "2 - 左膝 (L_Knee)", "3 - 左轮 (L_Wheel)",
            "4 - 右髋 (R_Hip)", "5 - 右膝 (R_Knee)", "6 - 右轮 (R_Wheel)"
        ])
        motor_layout.addWidget(self.motor_combo, 0, 1, 1, 2)
        
        # 控制模式
        motor_layout.addWidget(QLabel("模式:"), 1, 0)
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["0-力矩", "1-速度", "2-位置梯形", "3-位置滤波", "4-位置直接", "5-低速"])
        motor_layout.addWidget(self.mode_combo, 1, 1)
        self.set_mode_btn = QPushButton("设置")
        self.set_mode_btn.clicked.connect(self.set_motor_mode)
        motor_layout.addWidget(self.set_mode_btn, 1, 2)
        
        # 速度控制
        motor_layout.addWidget(QLabel("速度(rpm):"), 2, 0)
        self.speed_input = QDoubleSpinBox()
        self.speed_input.setRange(-500, 500)
        self.speed_input.setSingleStep(10)
        motor_layout.addWidget(self.speed_input, 2, 1)
        self.set_speed_btn = QPushButton("设置")
        self.set_speed_btn.clicked.connect(self.set_motor_speed)
        motor_layout.addWidget(self.set_speed_btn, 2, 2)
        
        # 位置控制
        motor_layout.addWidget(QLabel("位置(°):"), 3, 0)
        self.pos_input = QDoubleSpinBox()
        self.pos_input.setRange(-360, 360)
        self.pos_input.setSingleStep(5)
        motor_layout.addWidget(self.pos_input, 3, 1)
        self.set_pos_btn = QPushButton("设置")
        self.set_pos_btn.clicked.connect(self.set_motor_position)
        motor_layout.addWidget(self.set_pos_btn, 3, 2)
        
        # 力矩控制
        motor_layout.addWidget(QLabel("力矩:"), 4, 0)
        self.torque_input = QDoubleSpinBox()
        self.torque_input.setRange(-5, 5)
        self.torque_input.setSingleStep(0.1)
        self.torque_input.setDecimals(2)
        motor_layout.addWidget(self.torque_input, 4, 1)
        self.set_torque_btn = QPushButton("设置")
        self.set_torque_btn.clicked.connect(self.set_motor_torque)
        motor_layout.addWidget(self.set_torque_btn, 4, 2)
        
        # 单电机操作
        btn_layout = QHBoxLayout()
        for text, action in [("使能", "enable"), ("空闲", "idle"), ("停止", "stop"), ("读取", "read")]:
            btn = QPushButton(text)
            btn.clicked.connect(lambda checked, a=action: self.send_motor_cmd(a))
            btn_layout.addWidget(btn)
        motor_layout.addLayout(btn_layout, 5, 0, 1, 3)
        
        # 高级操作
        adv_layout = QHBoxLayout()
        for text, action in [("设零点", "origin"), ("保存", "save"), ("重启", "reboot"), ("错误码", "error")]:
            btn = QPushButton(text)
            btn.clicked.connect(lambda checked, a=action: self.send_motor_adv_cmd(a))
            adv_layout.addWidget(btn)
        motor_layout.addLayout(adv_layout, 6, 0, 1, 3)
        
        motor_group.setLayout(motor_layout)
        layout.addWidget(motor_group)
        
        # 电机状态表
        status_group = QGroupBox("电机状态")
        status_layout = QGridLayout()
        
        for col, header in enumerate(["ID", "位置(°)", "速度", "电流(A)", "状态"]):
            label = QLabel(header)
            label.setStyleSheet("font-weight: bold;")
            status_layout.addWidget(label, 0, col)
        
        self.status_labels = {}
        motor_names = ["L_Hip", "L_Knee", "L_Wheel", "R_Hip", "R_Knee", "R_Wheel"]
        for row, name in enumerate(motor_names, 1):
            status_layout.addWidget(QLabel(f"{row} {name}"), row, 0)
            self.status_labels[row] = {'pos': QLabel("--"), 'spd': QLabel("--"), 'cur': QLabel("--"), 'status': QLabel("--")}
            for col, key in enumerate(['pos', 'spd', 'cur', 'status'], 1):
                status_layout.addWidget(self.status_labels[row][key], row, col)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        layout.addStretch()
    
    def get_motor_id(self):
        return self.motor_combo.currentIndex() + 1
    
    def send_cmd(self, cmd):
        if self.parent_window and self.parent_window.is_connected():
            self.parent_window.send_command(cmd)
    
    def send_motor_cmd(self, action):
        self.send_cmd(f"{action} {self.get_motor_id()}")
    
    def send_motor_adv_cmd(self, action):
        self.send_cmd(f"motor {self.get_motor_id()} {action}")
    
    def set_motor_mode(self):
        self.send_cmd(f"mode {self.get_motor_id()} {self.mode_combo.currentIndex()}")
    
    def set_motor_speed(self):
        self.send_cmd(f"speed {self.get_motor_id()} {self.speed_input.value()}")
    
    def set_motor_position(self):
        self.send_cmd(f"pos {self.get_motor_id()} {self.pos_input.value()}")
    
    def set_motor_torque(self):
        self.send_cmd(f"torque {self.get_motor_id()} {self.torque_input.value()}")
    
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
        self.rate_combo.addItems(["0-0.2Hz", "1-0.5Hz", "2-1Hz", "3-2Hz", "4-5Hz", "5-10Hz", 
                                  "6-20Hz", "7-50Hz", "8-100Hz", "9-200Hz", "10-无输出"])
        self.rate_combo.setCurrentIndex(8)
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
        
        data_layout.addWidget(QLabel("Roll (横滚):"), 0, 0)
        self.roll_label = self.create_data_label()
        data_layout.addWidget(self.roll_label, 0, 1)
        
        data_layout.addWidget(QLabel("Pitch (俯仰):"), 1, 0)
        self.pitch_label = self.create_data_label()
        data_layout.addWidget(self.pitch_label, 1, 1)
        
        data_layout.addWidget(QLabel("Yaw (偏航):"), 2, 0)
        self.yaw_label = self.create_data_label()
        data_layout.addWidget(self.yaw_label, 2, 1)
        
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
        if self.parent_window and self.parent_window.is_connected():
            self.parent_window.send_command(cmd)
    
    def toggle_print(self, state):
        self.send_cmd("imu print on" if state == Qt.Checked else "imu print off")
    
    def set_rate(self):
        self.send_cmd(f"imu rate {self.rate_combo.currentIndex()}")
    
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
    """平衡控制面板 - 替代 balance 命令行"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.balance_initialized = False
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
        
        # Roll 控制 (从腿部面板移过来，属于平衡控制)
        roll_group = QGroupBox("Roll 平衡控制 (侧倾补偿)")
        roll_layout = QHBoxLayout()
        
        self.roll_status = QLabel("状态: 未知")
        self.roll_status.setStyleSheet("font-size: 14px; font-weight: bold;")
        roll_layout.addWidget(self.roll_status)
        
        roll_layout.addStretch()
        
        self.roll_on_btn = QPushButton("✅ 开启 Roll")
        self.roll_on_btn.setStyleSheet("background-color: #4CAF50; color: white; padding: 8px 20px;")
        self.roll_on_btn.clicked.connect(lambda: self.send_roll_cmd("on"))
        roll_layout.addWidget(self.roll_on_btn)
        
        self.roll_off_btn = QPushButton("❌ 关闭 Roll")
        self.roll_off_btn.setStyleSheet("background-color: #f44336; color: white; padding: 8px 20px;")
        self.roll_off_btn.clicked.connect(lambda: self.send_roll_cmd("off"))
        roll_layout.addWidget(self.roll_off_btn)
        
        self.roll_status_btn = QPushButton("📊 状态")
        self.roll_status_btn.clicked.connect(lambda: self.send_cmd("balance roll"))
        roll_layout.addWidget(self.roll_status_btn)
        
        roll_group.setLayout(roll_layout)
        layout.addWidget(roll_group)
        
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
        
        self.get_zero_btn = QPushButton("获取当前")
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
        self.set_div_btn.clicked.connect(lambda: self.send_cmd(f"balance plot div {self.plot_div_input.value()}"))
        plot_layout.addWidget(self.set_div_btn)
        
        plot_group.setLayout(plot_layout)
        layout.addWidget(plot_group)
        
        # ========== 环路调试控制 (新增) ==========
        loop_group = QGroupBox("🔧 环路调试 (单环/组合调试)")
        loop_layout = QVBoxLayout()
        
        # 说明标签
        loop_desc = QLabel("LQR控制环: A=角度环 B=角速度环 C=位移环 D=速度环 H=总输出 | Y=Yaw转向")
        loop_desc.setStyleSheet("color: #888; font-size: 11px;")
        loop_layout.addWidget(loop_desc)
        
        # 预设组合
        preset_layout = QHBoxLayout()
        preset_layout.addWidget(QLabel("快捷预设:"))
        
        self.loop_full_btn = QPushButton("🟢 全开 (Full)")
        self.loop_full_btn.setToolTip("启用所有环路: ABCDHY")
        self.loop_full_btn.clicked.connect(lambda: self.set_loop_preset("full"))
        preset_layout.addWidget(self.loop_full_btn)
        
        self.loop_simple_btn = QPushButton("🟡 简单 (AB)")
        self.loop_simple_btn.setToolTip("仅角度+角速度环: A+B")
        self.loop_simple_btn.clicked.connect(lambda: self.set_loop_preset("simple"))
        preset_layout.addWidget(self.loop_simple_btn)
        
        self.loop_none_btn = QPushButton("🔴 全关")
        self.loop_none_btn.setToolTip("关闭所有环路 (电机无输出)")
        self.loop_none_btn.clicked.connect(lambda: self.set_loop_preset("none"))
        preset_layout.addWidget(self.loop_none_btn)
        
        self.loop_status_btn = QPushButton("📊 状态")
        self.loop_status_btn.clicked.connect(lambda: self.send_cmd("balance loop status"))
        preset_layout.addWidget(self.loop_status_btn)
        
        loop_layout.addLayout(preset_layout)
        
        # 分隔线
        line1 = QFrame()
        line1.setFrameShape(QFrame.HLine)
        line1.setStyleSheet("color: #444;")
        loop_layout.addWidget(line1)
        
        # 各环路独立开关 - LQR 内环
        lqr_label = QLabel("LQR 平衡环 (Pitch):")
        lqr_label.setStyleSheet("font-weight: bold;")
        loop_layout.addWidget(lqr_label)
        
        lqr_layout = QHBoxLayout()
        
        # 初始化环路复选框字典
        self.loop_checks = {}
        
        loop_items = [
            ("A", "角度环", "控制俯仰角度"),
            ("B", "角速度环", "控制俯仰角速度"),
            ("C", "位移环", "控制位置/距离"),
            ("D", "速度环", "控制车轮速度"),
            ("H", "总输出", "LQR控制器输出"),
        ]
        
        for code, name, tip in loop_items:
            cb = QCheckBox(f"{code}:{name}")
            cb.setToolTip(tip)
            cb.setChecked(True)  # 默认全开
            cb.stateChanged.connect(lambda state, c=code: self.on_loop_toggle(c, state))
            self.loop_checks[code] = cb
            lqr_layout.addWidget(cb)
        
        loop_layout.addLayout(lqr_layout)
        
        # Yaw 转向控制 (独立于 LQR)
        yaw_layout = QHBoxLayout()
        yaw_label = QLabel("转向控制:")
        yaw_label.setStyleSheet("font-weight: bold;")
        yaw_layout.addWidget(yaw_label)
        
        cb_yaw = QCheckBox("Y:Yaw转向")
        cb_yaw.setToolTip("控制原地转向/航向保持")
        cb_yaw.setChecked(True)
        cb_yaw.stateChanged.connect(lambda state: self.on_loop_toggle("Y", state))
        self.loop_checks["Y"] = cb_yaw
        yaw_layout.addWidget(cb_yaw)
        
        yaw_layout.addStretch()
        loop_layout.addLayout(yaw_layout)
        
        # 分隔线
        line2 = QFrame()
        line2.setFrameShape(QFrame.HLine)
        line2.setStyleSheet("color: #444;")
        loop_layout.addWidget(line2)
        
        # 增益精调 (高级)
        gain_label = QLabel("增益精调 (0.0-1.0):")
        gain_label.setStyleSheet("font-size: 11px; color: #888;")
        loop_layout.addWidget(gain_label)
        
        gain_layout = QHBoxLayout()
        self.gain_loop_combo = QComboBox()
        self.gain_loop_combo.addItems(["A:角度", "B:角速度", "C:位移", "D:速度", "H:总输出", "Y:Yaw"])
        gain_layout.addWidget(self.gain_loop_combo)
        
        self.gain_spin = QDoubleSpinBox()
        self.gain_spin.setRange(0.0, 1.0)
        self.gain_spin.setSingleStep(0.1)
        self.gain_spin.setValue(1.0)
        self.gain_spin.setDecimals(2)
        gain_layout.addWidget(self.gain_spin)
        
        self.gain_set_btn = QPushButton("设置增益")
        self.gain_set_btn.clicked.connect(self.set_loop_gain)
        gain_layout.addWidget(self.gain_set_btn)
        
        gain_layout.addStretch()
        loop_layout.addLayout(gain_layout)
        
        loop_group.setLayout(loop_layout)
        layout.addWidget(loop_group)
        
        # 状态显示
        status_group = QGroupBox("当前状态")
        status_layout = QGridLayout()
        
        status_layout.addWidget(QLabel("系统状态:"), 0, 0)
        self.sys_status_label = QLabel("未初始化")
        self.sys_status_label.setStyleSheet("font-weight: bold; color: gray;")
        status_layout.addWidget(self.sys_status_label, 0, 1)
        
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
        
        # ========== 任务频率监控 (新增) ==========
        freq_group = QGroupBox("📈 任务频率监控")
        freq_layout = QGridLayout()
        
        # 表头
        freq_layout.addWidget(QLabel("任务"), 0, 0)
        freq_layout.addWidget(QLabel("目标频率"), 0, 1)
        freq_layout.addWidget(QLabel("实际频率"), 0, 2)
        freq_layout.addWidget(QLabel("状态"), 0, 3)
        
        # IMU 读取
        freq_layout.addWidget(QLabel("IMU 读取:"), 1, 0)
        freq_layout.addWidget(QLabel("200 Hz"), 1, 1)
        self.freq_imu_label = QLabel("-- Hz")
        self.freq_imu_label.setStyleSheet("font-weight: bold; color: #00ccff;")
        freq_layout.addWidget(self.freq_imu_label, 1, 2)
        self.freq_imu_status = QLabel("--")
        freq_layout.addWidget(self.freq_imu_status, 1, 3)
        
        # 平衡控制
        freq_layout.addWidget(QLabel("平衡控制:"), 2, 0)
        freq_layout.addWidget(QLabel("200 Hz"), 2, 1)
        self.freq_ctrl_label = QLabel("-- Hz")
        self.freq_ctrl_label.setStyleSheet("font-weight: bold; color: #00ccff;")
        freq_layout.addWidget(self.freq_ctrl_label, 2, 2)
        self.freq_ctrl_status = QLabel("--")
        freq_layout.addWidget(self.freq_ctrl_status, 2, 3)
        
        # 电机通信 (轮)
        freq_layout.addWidget(QLabel("轮电机通信:"), 3, 0)
        freq_layout.addWidget(QLabel("500 Hz"), 3, 1)
        self.freq_motor_label = QLabel("-- Hz")
        self.freq_motor_label.setStyleSheet("font-weight: bold; color: #00ccff;")
        freq_layout.addWidget(self.freq_motor_label, 3, 2)
        self.freq_motor_status = QLabel("--")
        freq_layout.addWidget(self.freq_motor_status, 3, 3)
        
        # 腿电机
        freq_layout.addWidget(QLabel("腿电机控制:"), 4, 0)
        freq_layout.addWidget(QLabel("50 Hz"), 4, 1)
        self.freq_leg_label = QLabel("-- Hz")
        self.freq_leg_label.setStyleSheet("font-weight: bold; color: #00ccff;")
        freq_layout.addWidget(self.freq_leg_label, 4, 2)
        self.freq_leg_status = QLabel("--")
        freq_layout.addWidget(self.freq_leg_status, 4, 3)
        
        # 刷新按钮
        freq_btn_layout = QHBoxLayout()
        self.freq_refresh_btn = QPushButton("🔄 刷新频率")
        self.freq_refresh_btn.clicked.connect(self.refresh_task_freq)
        freq_btn_layout.addWidget(self.freq_refresh_btn)
        
        self.freq_auto_check = QCheckBox("自动刷新 (1秒)")
        self.freq_auto_check.setChecked(False)
        self.freq_auto_check.stateChanged.connect(self.toggle_auto_freq_refresh)
        freq_btn_layout.addWidget(self.freq_auto_check)
        
        freq_btn_layout.addStretch()
        
        freq_v_layout = QVBoxLayout()
        freq_v_layout.addLayout(freq_layout)
        freq_v_layout.addLayout(freq_btn_layout)
        freq_group.setLayout(freq_v_layout)
        layout.addWidget(freq_group)
        
        # 创建自动刷新定时器
        self.freq_timer = QTimer()
        self.freq_timer.timeout.connect(self.refresh_task_freq)
        
        # ========== 延迟诊断面板 (紧凑版) ==========
        latency_group = QGroupBox("⏱️ 延迟诊断")
        latency_group.setStyleSheet("QGroupBox { font-size: 11px; }")
        latency_layout = QGridLayout()
        latency_layout.setSpacing(2)  # 减小间距
        
        # 小字号样式
        small_label_style = "font-size: 10px;"
        small_value_style = "font-size: 10px; font-weight: bold; color: #00ccff;"
        
        # 表头
        for col, text in enumerate(["延迟项", "us", "ms", "状态"]):
            lbl = QLabel(text)
            lbl.setStyleSheet(small_label_style + "color: #888;")
            latency_layout.addWidget(lbl, 0, col)
        
        # IMU -> 控制
        lbl = QLabel("IMU→Ctrl:")
        lbl.setStyleSheet(small_label_style)
        latency_layout.addWidget(lbl, 1, 0)
        self.latency_imu_ctrl_us = QLabel("--")
        self.latency_imu_ctrl_us.setStyleSheet(small_value_style)
        latency_layout.addWidget(self.latency_imu_ctrl_us, 1, 1)
        self.latency_imu_ctrl_ms = QLabel("--")
        self.latency_imu_ctrl_ms.setStyleSheet(small_label_style)
        latency_layout.addWidget(self.latency_imu_ctrl_ms, 1, 2)
        self.latency_imu_ctrl_status = QLabel("--")
        self.latency_imu_ctrl_status.setStyleSheet(small_label_style)
        latency_layout.addWidget(self.latency_imu_ctrl_status, 1, 3)
        
        # 控制计算
        lbl = QLabel("计算:")
        lbl.setStyleSheet(small_label_style)
        latency_layout.addWidget(lbl, 2, 0)
        self.latency_ctrl_calc_us = QLabel("--")
        self.latency_ctrl_calc_us.setStyleSheet(small_value_style)
        latency_layout.addWidget(self.latency_ctrl_calc_us, 2, 1)
        self.latency_ctrl_calc_ms = QLabel("--")
        self.latency_ctrl_calc_ms.setStyleSheet(small_label_style)
        latency_layout.addWidget(self.latency_ctrl_calc_ms, 2, 2)
        self.latency_ctrl_calc_status = QLabel("--")
        self.latency_ctrl_calc_status.setStyleSheet(small_label_style)
        latency_layout.addWidget(self.latency_ctrl_calc_status, 2, 3)
        
        # 控制 -> 电机
        lbl = QLabel("Ctrl→Motor:")
        lbl.setStyleSheet(small_label_style)
        latency_layout.addWidget(lbl, 3, 0)
        self.latency_ctrl_motor_us = QLabel("--")
        self.latency_ctrl_motor_us.setStyleSheet(small_value_style)
        latency_layout.addWidget(self.latency_ctrl_motor_us, 3, 1)
        self.latency_ctrl_motor_ms = QLabel("--")
        self.latency_ctrl_motor_ms.setStyleSheet(small_label_style)
        latency_layout.addWidget(self.latency_ctrl_motor_ms, 3, 2)
        self.latency_ctrl_motor_status = QLabel("--")
        self.latency_ctrl_motor_status.setStyleSheet(small_label_style)
        latency_layout.addWidget(self.latency_ctrl_motor_status, 3, 3)
        
        # 总延迟 (稍大字号)
        lbl = QLabel("📊 IMU总延迟:")
        lbl.setStyleSheet("font-size: 11px; font-weight: bold;")
        latency_layout.addWidget(lbl, 4, 0)
        self.latency_total_us = QLabel("--")
        self.latency_total_us.setStyleSheet("font-size: 12px; font-weight: bold; color: #ffcc00;")
        latency_layout.addWidget(self.latency_total_us, 4, 1)
        self.latency_total_ms = QLabel("--")
        self.latency_total_ms.setStyleSheet("font-size: 12px; font-weight: bold; color: #ffcc00;")
        latency_layout.addWidget(self.latency_total_ms, 4, 2)
        self.latency_total_status = QLabel("--")
        self.latency_total_status.setStyleSheet("font-size: 10px; font-weight: bold;")
        latency_layout.addWidget(self.latency_total_status, 4, 3)
        
        # 统计信息
        lbl = QLabel("Avg/Min/Max:")
        lbl.setStyleSheet(small_label_style + "color: #888;")
        latency_layout.addWidget(lbl, 5, 0)
        self.latency_stats_label = QLabel("-- / -- / --")
        self.latency_stats_label.setStyleSheet(small_value_style)
        latency_layout.addWidget(self.latency_stats_label, 5, 1, 1, 3)
        
        # WiFi 延迟 (合并显示)
        lbl = QLabel("📶 WiFi延迟:")
        lbl.setStyleSheet("font-size: 11px; font-weight: bold; color: #00aaff;")
        latency_layout.addWidget(lbl, 6, 0)
        self.wifi_latency_total_us = QLabel("--")
        self.wifi_latency_total_us.setStyleSheet("font-size: 12px; font-weight: bold; color: #00ff88;")
        latency_layout.addWidget(self.wifi_latency_total_us, 6, 1)
        self.wifi_latency_total_ms = QLabel("--")
        self.wifi_latency_total_ms.setStyleSheet("font-size: 12px; color: #00ff88;")
        latency_layout.addWidget(self.wifi_latency_total_ms, 6, 2)
        self.wifi_latency_status = QLabel("--")
        self.wifi_latency_status.setStyleSheet("font-size: 10px;")
        latency_layout.addWidget(self.wifi_latency_status, 6, 3)
        
        lbl = QLabel("WiFi统计:")
        lbl.setStyleSheet(small_label_style + "color: #888;")
        latency_layout.addWidget(lbl, 7, 0)
        self.wifi_latency_stats_label = QLabel("-- / -- / --")
        self.wifi_latency_stats_label.setStyleSheet(small_value_style)
        latency_layout.addWidget(self.wifi_latency_stats_label, 7, 1, 1, 3)
        
        # 按钮行 (紧凑)
        latency_btn_layout = QHBoxLayout()
        self.latency_refresh_btn = QPushButton("刷新")
        self.latency_refresh_btn.setStyleSheet("font-size: 10px; padding: 2px 8px;")
        self.latency_refresh_btn.clicked.connect(self.refresh_latency)
        latency_btn_layout.addWidget(self.latency_refresh_btn)
        
        self.latency_auto_check = QCheckBox("自动")
        self.latency_auto_check.setStyleSheet("font-size: 10px;")
        self.latency_auto_check.setChecked(False)
        self.latency_auto_check.stateChanged.connect(self.toggle_auto_latency_refresh)
        latency_btn_layout.addWidget(self.latency_auto_check)
        
        latency_btn_layout.addStretch()
        
        latency_v_layout = QVBoxLayout()
        latency_v_layout.setSpacing(2)
        latency_v_layout.addLayout(latency_layout)
        latency_v_layout.addLayout(latency_btn_layout)
        latency_group.setLayout(latency_v_layout)
        layout.addWidget(latency_group)
        
        # 创建延迟自动刷新定时器
        self.latency_timer = QTimer()
        self.latency_timer.timeout.connect(self.refresh_latency)
        
        layout.addStretch()
    
    def send_cmd(self, cmd):
        if self.parent_window and self.parent_window.is_connected():
            self.parent_window.send_command(cmd)
    
    def send_roll_cmd(self, state):
        """发送 Roll 控制命令"""
        if not self.parent_window or not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        cmd = f"balance roll {state}"
        if self.parent_window.send_command(cmd):
            self.parent_window.log(f"发送: {cmd}")
            if state == "on":
                self.roll_status.setText("状态: 已开启")
                self.roll_status.setStyleSheet("font-size: 14px; font-weight: bold; color: #4CAF50;")
            else:
                self.roll_status.setText("状态: 已关闭")
                self.roll_status.setStyleSheet("font-size: 14px; font-weight: bold; color: #f44336;")
    
    def do_balance_init(self):
        self.send_cmd("balance init")
        self.init_status.setText("初始化中...")
        self.init_status.setStyleSheet("color: orange;")
    
    def set_zero_point(self):
        self.send_cmd(f"balance zero {self.zero_input.value()}")
    
    def on_init_success(self):
        self.balance_initialized = True
        self.init_status.setText("✓ 已初始化")
        self.init_status.setStyleSheet("color: green; font-weight: bold;")
        self.sys_status_label.setText("就绪")
        self.sys_status_label.setStyleSheet("font-weight: bold; color: green;")
    
    def set_loop_preset(self, preset):
        """设置环路预设"""
        self.send_cmd(f"balance loop {preset}")
        # 更新复选框状态
        if preset == "full":
            for cb in self.loop_checks.values():
                cb.blockSignals(True)
                cb.setChecked(True)
                cb.blockSignals(False)
        elif preset == "none":
            for cb in self.loop_checks.values():
                cb.blockSignals(True)
                cb.setChecked(False)
                cb.blockSignals(False)
        elif preset == "simple":
            # simple = A + B only
            simple_on = {"A", "B"}
            for code, cb in self.loop_checks.items():
                cb.blockSignals(True)
                cb.setChecked(code in simple_on)
                cb.blockSignals(False)
    
    def on_loop_toggle(self, loop_code, state):
        """单个环路开关切换"""
        if state == Qt.Checked:
            self.send_cmd(f"balance loop {loop_code} on")
        else:
            self.send_cmd(f"balance loop {loop_code} off")
    
    def set_loop_gain(self):
        """设置环路增益"""
        loop_text = self.gain_loop_combo.currentText()
        loop_code = loop_text.split(":")[0]  # "A:角度" -> "A"
        gain = self.gain_spin.value()
        self.send_cmd(f"balance loop {loop_code} {gain}")
    
    def refresh_task_freq(self):
        """刷新任务频率"""
        self.send_cmd("balance freq")
    
    def refresh_latency(self):
        """刷新延迟诊断"""
        self.send_cmd("balance latency")
    
    def toggle_auto_freq_refresh(self, state):
        """切换自动刷新"""
        if state == Qt.Checked:
            self.freq_timer.start(1000)  # 1秒刷新一次
        else:
            self.freq_timer.stop()
    
    def toggle_auto_latency_refresh(self, state):
        """切换延迟自动刷新"""
        if state == Qt.Checked:
            self.latency_timer.start(1000)  # 1秒刷新一次
        else:
            self.latency_timer.stop()
    
    def update_task_freq(self, imu_hz, ctrl_hz, motor_hz, leg_hz):
        """更新任务频率显示 (由主窗口解析串口数据后调用)"""
        # IMU
        self.freq_imu_label.setText(f"{imu_hz:.1f} Hz")
        self._update_freq_status(self.freq_imu_status, imu_hz, 200, 10)
        
        # Control
        self.freq_ctrl_label.setText(f"{ctrl_hz:.1f} Hz")
        self._update_freq_status(self.freq_ctrl_status, ctrl_hz, 200, 10)
        
        # Motor
        self.freq_motor_label.setText(f"{motor_hz:.1f} Hz")
        self._update_freq_status(self.freq_motor_status, motor_hz, 500, 25)
        
        # Leg
        self.freq_leg_label.setText(f"{leg_hz:.1f} Hz")
        self._update_freq_status(self.freq_leg_status, leg_hz, 50, 5)
    
    def _update_freq_status(self, label, actual, target, tolerance):
        """更新频率状态指示"""
        if actual == 0:
            label.setText("⚫ 停止")
            label.setStyleSheet("color: gray;")
        elif abs(actual - target) <= tolerance:
            label.setText("🟢 正常")
            label.setStyleSheet("color: #00ff00;")
        elif actual < target - tolerance:
            label.setText("🟡 偏低")
            label.setStyleSheet("color: #ffaa00;")
        else:
            label.setText("🔴 偏高")
            label.setStyleSheet("color: #ff4444;")
    
    def update_latency(self, imu_to_ctrl_us, ctrl_calc_us, ctrl_to_motor_us, total_us, avg_us=None, min_us=None, max_us=None):
        """更新延迟显示 (由主窗口解析串口数据后调用)"""
        # IMU -> 控制
        self.latency_imu_ctrl_us.setText(f"{imu_to_ctrl_us:.0f} us")
        self.latency_imu_ctrl_ms.setText(f"{imu_to_ctrl_us/1000:.2f} ms")
        self._update_latency_status(self.latency_imu_ctrl_status, imu_to_ctrl_us, 3000, 5000)
        
        # 控制计算
        self.latency_ctrl_calc_us.setText(f"{ctrl_calc_us:.0f} us")
        self.latency_ctrl_calc_ms.setText(f"{ctrl_calc_us/1000:.2f} ms")
        self._update_latency_status(self.latency_ctrl_calc_status, ctrl_calc_us, 500, 1000)
        
        # 控制 -> 电机
        self.latency_ctrl_motor_us.setText(f"{ctrl_to_motor_us:.0f} us")
        self.latency_ctrl_motor_ms.setText(f"{ctrl_to_motor_us/1000:.2f} ms")
        self._update_latency_status(self.latency_ctrl_motor_status, ctrl_to_motor_us, 3000, 5000)
        
        # 总延迟
        self.latency_total_us.setText(f"{total_us:.0f} us")
        self.latency_total_ms.setText(f"{total_us/1000:.2f} ms")
        self._update_latency_status(self.latency_total_status, total_us, 5000, 10000)
        
        # 统计信息
        if avg_us is not None and min_us is not None and max_us is not None:
            self.latency_stats_label.setText(f"{avg_us:.0f} / {min_us:.0f} / {max_us:.0f} us")
            # 根据最大值设置颜色
            if max_us <= 8000:
                self.latency_stats_label.setStyleSheet("font-weight: bold; color: #00ff00;")
            elif max_us <= 15000:
                self.latency_stats_label.setStyleSheet("font-weight: bold; color: #ffaa00;")
            else:
                self.latency_stats_label.setStyleSheet("font-weight: bold; color: #ff4444;")
    
    def update_wifi_latency(self, wifi_ctrl_us, total_us, avg_us, min_us, max_us):
        """更新 WiFi 遥控延迟显示"""
        # WiFi 总延迟
        self.wifi_latency_total_us.setText(f"{total_us:.0f} us")
        self.wifi_latency_total_ms.setText(f"{total_us/1000:.2f} ms")
        # WiFi 延迟阈值: < 20ms 优秀, < 50ms 正常
        self._update_latency_status(self.wifi_latency_status, total_us, 20000, 50000)
        
        # 统计信息
        self.wifi_latency_stats_label.setText(f"{avg_us:.0f} / {min_us:.0f} / {max_us:.0f} us")
        if max_us <= 30000:
            self.wifi_latency_stats_label.setStyleSheet("font-weight: bold; color: #00ff00;")
        elif max_us <= 80000:
            self.wifi_latency_stats_label.setStyleSheet("font-weight: bold; color: #ffaa00;")
        else:
            self.wifi_latency_stats_label.setStyleSheet("font-weight: bold; color: #ff4444;")
    
    def _update_latency_status(self, label, us_value, good_threshold, warn_threshold):
        """更新延迟状态指示"""
        if us_value <= good_threshold:
            label.setText("🟢 优秀")
            label.setStyleSheet("color: #00ff00; font-weight: bold;")
        elif us_value <= warn_threshold:
            label.setText("🟡 正常")
            label.setStyleSheet("color: #ffaa00; font-weight: bold;")
        else:
            label.setText("🔴 需优化")
            label.setStyleSheet("color: #ff4444; font-weight: bold;")


# ============================================================================
# YAW 调试面板 (独立标签页)
# ============================================================================
class YawDebugPanel(QWidget):
    """YAW 转向调试面板 - 独立标签页"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # ========== 状态监控区 ==========
        status_group = QGroupBox("🧭 YAW 状态监控")
        status_layout = QGridLayout()
        status_layout.setSpacing(10)
        
        # 样式定义
        title_style = "font-size: 14px; font-weight: bold; color: #888;"
        value_style = "font-size: 24px; font-weight: bold; color: #00ccff;"
        unit_style = "font-size: 12px; color: #888;"
        
        # 第一行: 角度信息
        row = 0
        # 目标角度
        lbl = QLabel("🎯 目标角度")
        lbl.setStyleSheet(title_style)
        status_layout.addWidget(lbl, row, 0, Qt.AlignCenter)
        
        # 当前角度
        lbl = QLabel("📍 当前角度")
        lbl.setStyleSheet(title_style)
        status_layout.addWidget(lbl, row, 1, Qt.AlignCenter)
        
        # 角度误差
        lbl = QLabel("⚠️ 角度误差")
        lbl.setStyleSheet(title_style)
        status_layout.addWidget(lbl, row, 2, Qt.AlignCenter)
        
        # 第二行: 角度数值
        row = 1
        self.yaw_target_angle = QLabel("--")
        self.yaw_target_angle.setStyleSheet(value_style)
        self.yaw_target_angle.setAlignment(Qt.AlignCenter)
        status_layout.addWidget(self.yaw_target_angle, row, 0)
        
        self.yaw_current_angle = QLabel("--")
        self.yaw_current_angle.setStyleSheet(value_style)
        self.yaw_current_angle.setAlignment(Qt.AlignCenter)
        status_layout.addWidget(self.yaw_current_angle, row, 1)
        
        self.yaw_angle_error = QLabel("--")
        self.yaw_angle_error.setStyleSheet("font-size: 24px; font-weight: bold; color: #ffcc00;")
        self.yaw_angle_error.setAlignment(Qt.AlignCenter)
        status_layout.addWidget(self.yaw_angle_error, row, 2)
        
        # 第三行: 控制信息标题
        row = 2
        lbl = QLabel("🔧 YAW 输出")
        lbl.setStyleSheet(title_style)
        status_layout.addWidget(lbl, row, 0, Qt.AlignCenter)
        
        lbl = QLabel("🔄 角速度")
        lbl.setStyleSheet(title_style)
        status_layout.addWidget(lbl, row, 1, Qt.AlignCenter)
        
        lbl = QLabel("📌 控制模式")
        lbl.setStyleSheet(title_style)
        status_layout.addWidget(lbl, row, 2, Qt.AlignCenter)
        
        # 第四行: 控制信息数值
        row = 3
        self.yaw_output = QLabel("--")
        self.yaw_output.setStyleSheet("font-size: 24px; font-weight: bold; color: #ff8800;")
        self.yaw_output.setAlignment(Qt.AlignCenter)
        status_layout.addWidget(self.yaw_output, row, 0)
        
        self.yaw_rate = QLabel("--")
        self.yaw_rate.setStyleSheet(value_style)
        self.yaw_rate.setAlignment(Qt.AlignCenter)
        status_layout.addWidget(self.yaw_rate, row, 1)
        
        self.yaw_holding_status = QLabel("--")
        self.yaw_holding_status.setStyleSheet("font-size: 18px; font-weight: bold; color: #00aaff;")
        self.yaw_holding_status.setAlignment(Qt.AlignCenter)
        status_layout.addWidget(self.yaw_holding_status, row, 2)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        # ========== 诊断区 ==========
        diag_group = QGroupBox("💡 实时诊断")
        diag_layout = QVBoxLayout()
        
        self.yaw_diagnosis = QLabel("等待数据...")
        self.yaw_diagnosis.setStyleSheet("font-size: 16px; color: #aaa; padding: 10px;")
        self.yaw_diagnosis.setWordWrap(True)
        self.yaw_diagnosis.setAlignment(Qt.AlignCenter)
        diag_layout.addWidget(self.yaw_diagnosis)
        
        diag_group.setLayout(diag_layout)
        layout.addWidget(diag_group)
        
        # ========== 控制区 ==========
        ctrl_group = QGroupBox("🎮 YAW 控制")
        ctrl_layout = QHBoxLayout()
        
        # YAW 环开关
        self.yaw_enable_btn = QPushButton("开启 YAW 环")
        self.yaw_enable_btn.setStyleSheet("font-size: 14px; padding: 10px 20px; background-color: #44aa44;")
        self.yaw_enable_btn.clicked.connect(lambda: self.send_cmd("balance loop Y on"))
        ctrl_layout.addWidget(self.yaw_enable_btn)
        
        self.yaw_disable_btn = QPushButton("关闭 YAW 环")
        self.yaw_disable_btn.setStyleSheet("font-size: 14px; padding: 10px 20px; background-color: #aa4444;")
        self.yaw_disable_btn.clicked.connect(lambda: self.send_cmd("balance loop Y off"))
        ctrl_layout.addWidget(self.yaw_disable_btn)
        
        # 重置 YAW 角度
        self.yaw_reset_btn = QPushButton("重置目标角度")
        self.yaw_reset_btn.setStyleSheet("font-size: 14px; padding: 10px 20px; background-color: #4488aa;")
        self.yaw_reset_btn.clicked.connect(lambda: self.send_cmd("balance yaw reset"))
        ctrl_layout.addWidget(self.yaw_reset_btn)
        
        ctrl_layout.addStretch()
        ctrl_group.setLayout(ctrl_layout)
        layout.addWidget(ctrl_group)
        
        # ========== PID 参数快捷调节 ==========
        pid_group = QGroupBox("🔧 YAW PID 参数快捷调节")
        pid_layout = QGridLayout()
        
        # YAW 角度 PID (E)
        row = 0
        lbl = QLabel("角度 PID (E):")
        lbl.setStyleSheet("font-weight: bold;")
        pid_layout.addWidget(lbl, row, 0)
        
        pid_layout.addWidget(QLabel("P:"), row, 1)
        self.yaw_angle_p = QDoubleSpinBox()
        self.yaw_angle_p.setRange(0, 100)
        self.yaw_angle_p.setDecimals(3)
        self.yaw_angle_p.setSingleStep(0.1)
        pid_layout.addWidget(self.yaw_angle_p, row, 2)
        
        pid_layout.addWidget(QLabel("I:"), row, 3)
        self.yaw_angle_i = QDoubleSpinBox()
        self.yaw_angle_i.setRange(0, 100)
        self.yaw_angle_i.setDecimals(3)
        self.yaw_angle_i.setSingleStep(0.01)
        pid_layout.addWidget(self.yaw_angle_i, row, 4)
        
        pid_layout.addWidget(QLabel("D:"), row, 5)
        self.yaw_angle_d = QDoubleSpinBox()
        self.yaw_angle_d.setRange(0, 100)
        self.yaw_angle_d.setDecimals(3)
        self.yaw_angle_d.setSingleStep(0.001)
        pid_layout.addWidget(self.yaw_angle_d, row, 6)
        
        self.yaw_angle_set_btn = QPushButton("设置")
        self.yaw_angle_set_btn.clicked.connect(self.set_yaw_angle_pid)
        pid_layout.addWidget(self.yaw_angle_set_btn, row, 7)
        
        self.yaw_angle_read_btn = QPushButton("读取")
        self.yaw_angle_read_btn.clicked.connect(lambda: self.send_cmd("E?"))
        pid_layout.addWidget(self.yaw_angle_read_btn, row, 8)
        
        # YAW 角速度 PID (F)
        row = 1
        lbl = QLabel("角速度 PID (F):")
        lbl.setStyleSheet("font-weight: bold;")
        pid_layout.addWidget(lbl, row, 0)
        
        pid_layout.addWidget(QLabel("P:"), row, 1)
        self.yaw_gyro_p = QDoubleSpinBox()
        self.yaw_gyro_p.setRange(0, 100)
        self.yaw_gyro_p.setDecimals(3)
        self.yaw_gyro_p.setSingleStep(0.1)
        pid_layout.addWidget(self.yaw_gyro_p, row, 2)
        
        pid_layout.addWidget(QLabel("I:"), row, 3)
        self.yaw_gyro_i = QDoubleSpinBox()
        self.yaw_gyro_i.setRange(0, 100)
        self.yaw_gyro_i.setDecimals(3)
        self.yaw_gyro_i.setSingleStep(0.01)
        pid_layout.addWidget(self.yaw_gyro_i, row, 4)
        
        pid_layout.addWidget(QLabel("D:"), row, 5)
        self.yaw_gyro_d = QDoubleSpinBox()
        self.yaw_gyro_d.setRange(0, 100)
        self.yaw_gyro_d.setDecimals(3)
        self.yaw_gyro_d.setSingleStep(0.001)
        pid_layout.addWidget(self.yaw_gyro_d, row, 6)
        
        self.yaw_gyro_set_btn = QPushButton("设置")
        self.yaw_gyro_set_btn.clicked.connect(self.set_yaw_gyro_pid)
        pid_layout.addWidget(self.yaw_gyro_set_btn, row, 7)
        
        self.yaw_gyro_read_btn = QPushButton("读取")
        self.yaw_gyro_read_btn.clicked.connect(lambda: self.send_cmd("F?"))
        pid_layout.addWidget(self.yaw_gyro_read_btn, row, 8)
        
        pid_group.setLayout(pid_layout)
        layout.addWidget(pid_group)
        
        # ========== 说明区 ==========
        help_group = QGroupBox("📚 YAW 控制说明")
        help_layout = QVBoxLayout()
        
        help_text = QLabel("""
<b>控制模式说明:</b>
• <b>🔒 方向保持</b>: 遥控器松手时，自动锁定当前朝向，通过角度PID+角速度阻尼保持方向
• <b>🔄 角速度跟踪</b>: 遥控器有输入时，跟踪目标角速度实现转弯

<b>调参建议:</b>
• 误差大但输出小 → 增大角度 P
• 误差小但振荡 → 减小角度 P 或增大角速度 P (阻尼)
• 转弯响应慢 → 增大角速度 P
• 转弯时抖动 → 减小角速度 P

<b>控制输出:</b> 左轮 = LQR_u + YAW输出, 右轮 = LQR_u - YAW输出
        """)
        help_text.setStyleSheet("font-size: 12px; color: #aaa;")
        help_text.setWordWrap(True)
        help_layout.addWidget(help_text)
        
        help_group.setLayout(help_layout)
        layout.addWidget(help_group)
        
        layout.addStretch()
    
    def send_cmd(self, cmd):
        if self.parent_window and self.parent_window.is_connected():
            self.parent_window.send_command(cmd)
            self.parent_window.log(f"发送: {cmd}")
    
    def set_yaw_angle_pid(self):
        p = self.yaw_angle_p.value()
        i = self.yaw_angle_i.value()
        d = self.yaw_angle_d.value()
        self.send_cmd(f"EP{p}")
        self.send_cmd(f"EI{i}")
        self.send_cmd(f"ED{d}")
    
    def set_yaw_gyro_pid(self):
        p = self.yaw_gyro_p.value()
        i = self.yaw_gyro_i.value()
        d = self.yaw_gyro_d.value()
        self.send_cmd(f"FP{p}")
        self.send_cmd(f"FI{i}")
        self.send_cmd(f"FD{d}")
    
    def update_yaw_debug(self, target_angle, current_angle, error, output, holding, yaw_rate):
        """更新 YAW 调试数据"""
        # 目标角度
        self.yaw_target_angle.setText(f"{target_angle:.1f}°")
        
        # 当前角度
        self.yaw_current_angle.setText(f"{current_angle:.1f}°")
        
        # 角度误差 (根据大小设置颜色)
        self.yaw_angle_error.setText(f"{error:.2f}°")
        if abs(error) < 1.0:
            self.yaw_angle_error.setStyleSheet("font-size: 24px; font-weight: bold; color: #00ff00;")
        elif abs(error) < 5.0:
            self.yaw_angle_error.setStyleSheet("font-size: 24px; font-weight: bold; color: #ffcc00;")
        else:
            self.yaw_angle_error.setStyleSheet("font-size: 24px; font-weight: bold; color: #ff4444;")
        
        # YAW 输出 (根据大小设置颜色)
        self.yaw_output.setText(f"{output:.3f}")
        if abs(output) < 0.1:
            self.yaw_output.setStyleSheet("font-size: 24px; font-weight: bold; color: #00ff00;")
        elif abs(output) < 0.5:
            self.yaw_output.setStyleSheet("font-size: 24px; font-weight: bold; color: #ff8800;")
        else:
            self.yaw_output.setStyleSheet("font-size: 24px; font-weight: bold; color: #ff4444;")
        
        # 保持模式状态
        if holding:
            self.yaw_holding_status.setText("🔒 方向保持")
            self.yaw_holding_status.setStyleSheet("font-size: 18px; font-weight: bold; color: #00aaff;")
        else:
            self.yaw_holding_status.setText("🔄 角速度跟踪")
            self.yaw_holding_status.setStyleSheet("font-size: 18px; font-weight: bold; color: #ffaa00;")
        
        # 角速度
        self.yaw_rate.setText(f"{yaw_rate:.2f} rad/s")
        
        # 诊断建议
        diagnosis = self._diagnose_yaw(error, output, holding, yaw_rate)
        self.yaw_diagnosis.setText(diagnosis)
    
    def _diagnose_yaw(self, error, output, holding, yaw_rate):
        """根据 YAW 状态给出诊断建议"""
        issues = []
        
        if holding:
            # 方向保持模式下的诊断
            if abs(error) > 5.0:
                issues.append(f"❌ 误差大 ({error:.1f}°)，角度P可能太小")
            if abs(error) > 1.0 and abs(output) < 0.05:
                issues.append("❌ 有误差但输出小，检查角度P是否为0")
            if abs(yaw_rate) > 0.5 and abs(output) < 0.1:
                issues.append("⚠️ 角速度大但阻尼小，检查角速度P")
            if abs(output) > 0.3 and abs(error) < 1.0:
                issues.append("⚠️ 误差小但输出大，可能振荡，减小P")
        else:
            # 角速度跟踪模式
            if abs(output) > 0.5:
                issues.append("ℹ️ 转向输出较大")
        
        if not issues:
            if holding:
                return "✅ 方向保持正常 - 误差小，输出稳定"
            else:
                return "✅ 转向跟踪中 - 正在响应遥控器输入"
        
        return "\n".join(issues)


# ============================================================================
# 传感器面板
# ============================================================================
class SensorPanel(QWidget):
    """传感器控制面板 - 替代命令行传感器控制"""
    
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
        for text, cmd in [("初始化", "btn init"), ("读取", "btn read"), ("开始监控", "btn start"), ("停止", "btn stop")]:
            btn = QPushButton(text)
            btn.clicked.connect(lambda checked, c=cmd: self.send_cmd(c))
            btn_layout.addWidget(btn)
        btn_group.setLayout(btn_layout)
        layout.addWidget(btn_group)
        
        # 温湿度传感器
        sht_group = QGroupBox("🌡️ 温湿度传感器 (SHT30)")
        sht_layout = QVBoxLayout()
        
        ctrl_layout = QHBoxLayout()
        for text, cmd in [("初始化", "sht init"), ("读取", "sht read"), ("开始监控", "sht start"), ("停止", "sht stop"), ("扫描I2C", "sht scan")]:
            btn = QPushButton(text)
            btn.clicked.connect(lambda checked, c=cmd: self.send_cmd(c))
            ctrl_layout.addWidget(btn)
        sht_layout.addLayout(ctrl_layout)
        
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
        info_layout.addWidget(QLabel("SSID: WL-PRO   密码: 12345678   控制页面: http://192.168.4.1"))
        info_group.setLayout(info_layout)
        layout.addWidget(info_group)
        
        layout.addStretch()
    
    def send_cmd(self, cmd):
        if self.parent_window and self.parent_window.is_connected():
            self.parent_window.send_command(cmd)


# ============================================================================
# 终端面板
# ============================================================================
class TerminalPanel(QWidget):
    """终端/命令行面板 - 直接发送任意命令"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
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
        
        layout.addLayout(input_layout)
        
        # 快捷命令
        quick_layout = QHBoxLayout()
        quick_cmds = ["help", "scan", "read all", "balance status", "imu status", "power", "balance init"]
        for cmd in quick_cmds:
            btn = QPushButton(cmd)
            btn.clicked.connect(lambda checked, c=cmd: self.quick_send(c))
            quick_layout.addWidget(btn)
        layout.addLayout(quick_layout)
        
        # 说明
        help_label = QLabel("💡 提示: 在此面板可以直接输入任意命令发送到设备，输出显示在右侧日志窗口")
        help_label.setStyleSheet("color: gray; font-size: 12px;")
        layout.addWidget(help_label)
        
        layout.addStretch()
    
    def send_command(self):
        cmd = self.cmd_input.text().strip()
        if cmd and self.parent_window and self.parent_window.is_connected():
            self.parent_window.send_command(cmd)
            self.cmd_input.clear()
    
    def quick_send(self, cmd):
        if self.parent_window and self.parent_window.is_connected():
            self.parent_window.send_command(cmd)


# ============================================================================
# 主窗口
# ============================================================================
class PIDTunerUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("轮腿机器人控制面板 - PID调参 + 设备控制")
        self.setGeometry(100, 100, 1200, 800)
        
        self.serial_thread = SerialThread()
        self.serial_thread.data_received.connect(self.process_serial_data)
        
        self.pid_panels = {}
        self.lpf_panels = {}
        self.speed_adaptive_panel = None
        self.debug_mode = False
        
        # YAW 调试数据缓存
        self._yaw_target_angle = 0.0
        self._yaw_current_angle = 0.0
        
        self.init_ui()
    
    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        
        # ===== 串口控制区 =====
        serial_group = QGroupBox("串口连接")
        serial_layout = QHBoxLayout()
        
        self.port_combo = QComboBox()
        self.refresh_ports()
        serial_layout.addWidget(QLabel("串口:"))
        serial_layout.addWidget(self.port_combo)
        
        self.refresh_btn = QPushButton("刷新")
        self.refresh_btn.clicked.connect(self.refresh_ports)
        serial_layout.addWidget(self.refresh_btn)
        
        self.connect_btn = QPushButton("连接")
        self.connect_btn.clicked.connect(self.toggle_connection)
        serial_layout.addWidget(self.connect_btn)
        
        self.test_btn = QPushButton("🔧 测试通信")
        self.test_btn.clicked.connect(self.test_communication)
        self.test_btn.setEnabled(False)
        serial_layout.addWidget(self.test_btn)
        
        self.status_label = QLabel("未连接")
        self.status_label.setStyleSheet("color: red; font-weight: bold; font-size: 14px;")
        serial_layout.addWidget(self.status_label)
        
        serial_layout.addStretch()
        serial_group.setLayout(serial_layout)
        main_layout.addWidget(serial_group)
        
        # ===== 主分割区域 =====
        splitter = QSplitter(Qt.Horizontal)
        
        # 左侧: Tab切换区
        self.tab_widget = QTabWidget()
        
        # 根据Commander映射创建标签页
        # PID控制器 - 平衡控制相关
        self.pid_panels['angle'] = PIDControlPanel("角度控制 (Angle)", "A", self)
        self.tab_widget.addTab(self.pid_panels['angle'], "A - 角度PID")
        
        self.pid_panels['gyro'] = PIDControlPanel("角速度控制 (Gyro)", "B", self)
        self.tab_widget.addTab(self.pid_panels['gyro'], "B - 角速度PID")
        
        self.pid_panels['distance'] = PIDControlPanel("位移控制 (Distance)", "C", self)
        self.tab_widget.addTab(self.pid_panels['distance'], "C - 位移PID")
        
        self.pid_panels['speed'] = PIDControlPanel("速度控制 (Speed)", "D", self)
        self.tab_widget.addTab(self.pid_panels['speed'], "D - 速度PID")
        
        self.pid_panels['yaw_angle'] = PIDControlPanel("YAW角度控制", "E", self)
        self.tab_widget.addTab(self.pid_panels['yaw_angle'], "E - YAW角度PID")
        
        self.pid_panels['yaw_gyro'] = PIDControlPanel("YAW角速度控制", "F", self)
        self.tab_widget.addTab(self.pid_panels['yaw_gyro'], "F - YAW角速度PID")
        
        self.pid_panels['lqr_u'] = PIDControlPanel("LQR输出补偿", "H", self)
        self.tab_widget.addTab(self.pid_panels['lqr_u'], "H - LQR输出PID")
        
        self.pid_panels['zeropoint'] = PIDControlPanel("零点自适应", "I", self)
        self.tab_widget.addTab(self.pid_panels['zeropoint'], "I - 零点PID")
        
        self.pid_panels['roll_angle'] = PIDControlPanel("Roll轴平衡", "K", self)
        self.tab_widget.addTab(self.pid_panels['roll_angle'], "K - Roll角度PID")
        
        # 低通滤波器
        self.lpf_panels['joyy'] = LPFControlPanel("摇杆Y轴滤波", "G", self)
        self.tab_widget.addTab(self.lpf_panels['joyy'], "G - 摇杆滤波")
        
        self.lpf_panels['zeropoint'] = LPFControlPanel("零点滤波", "J", self)
        self.tab_widget.addTab(self.lpf_panels['zeropoint'], "J - 零点滤波")
        
        self.lpf_panels['roll'] = LPFControlPanel("Roll角度滤波", "L", self)
        self.tab_widget.addTab(self.lpf_panels['roll'], "L - Roll滤波")
        
        # 速度自适应面板
        self.speed_adaptive_panel = SpeedAdaptivePanel(self)
        self.tab_widget.addTab(self.speed_adaptive_panel, "M - 速度自适应P")
        
        # 轮速调试面板 (用于离地检测阈值调试)
        # O - 左轮: 蓝线=速度(rad/s), 红线=加速度(rad/s²)
        # P - 右轮: 蓝线=速度(rad/s), 红线=加速度(rad/s²)
        self.wheel_panels = {}
        self.wheel_panels['left'] = PIDControlPanel("左轮调试 (蓝=速度rad/s, 红=加速度rad/s²)", "O", self)
        self.tab_widget.addTab(self.wheel_panels['left'], "O - 左轮调试")
        
        self.wheel_panels['right'] = PIDControlPanel("右轮调试 (蓝=速度rad/s, 红=加速度rad/s²)", "P", self)
        self.tab_widget.addTab(self.wheel_panels['right'], "P - 右轮调试")
        
        # 腿部控制面板
        self.leg_panel = LegControlPanel(self)
        self.tab_widget.addTab(self.leg_panel, "🦿 腿部控制")
        
        # Web监控面板
        self.web_monitor = WebMonitorPanel(self)
        self.tab_widget.addTab(self.web_monitor, "📱 Web监控")
        
        # ===== 新增设备控制面板 =====
        # 平衡控制面板
        self.balance_panel = BalanceControlPanel(self)
        self.tab_widget.addTab(self.balance_panel, "🎮 平衡控制")
        
        # YAW 调试面板 (独立标签页)
        self.yaw_panel = YawDebugPanel(self)
        self.tab_widget.addTab(self.yaw_panel, "🧭 YAW调试")
        
        # 电机控制面板
        self.motor_panel = MotorControlPanel(self)
        self.tab_widget.addTab(self.motor_panel, "⚙️ 电机控制")
        
        # IMU控制面板
        self.imu_panel = IMUControlPanel(self)
        self.tab_widget.addTab(self.imu_panel, "📐 IMU")
        
        # 传感器面板
        self.sensor_panel = SensorPanel(self)
        self.tab_widget.addTab(self.sensor_panel, "🔧 传感器")
        
        # 终端面板
        self.terminal_panel = TerminalPanel(self)
        self.tab_widget.addTab(self.terminal_panel, "💻 终端")
        
        splitter.addWidget(self.tab_widget)
        
        # 右侧: 日志区
        log_widget = QWidget()
        log_layout = QVBoxLayout(log_widget)
        
        log_group = QGroupBox("通信日志")
        log_inner_layout = QVBoxLayout()
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet("background-color: #1e1e1e; color: #d4d4d4; font-family: Consolas; font-size: 12px;")
        log_inner_layout.addWidget(self.log_text)
        
        log_btn_layout = QHBoxLayout()
        clear_log_btn = QPushButton("清空日志")
        clear_log_btn.clicked.connect(lambda: self.log_text.clear())
        log_btn_layout.addWidget(clear_log_btn)
        
        help_btn = QPushButton("❓ 帮助")
        help_btn.clicked.connect(self.show_help)
        log_btn_layout.addWidget(help_btn)
        
        debug_btn = QPushButton("🐛 调试模式")
        debug_btn.setCheckable(True)
        debug_btn.toggled.connect(self.toggle_debug_mode)
        log_btn_layout.addWidget(debug_btn)
        
        log_btn_layout.addStretch()
        log_inner_layout.addLayout(log_btn_layout)
        
        log_group.setLayout(log_inner_layout)
        log_layout.addWidget(log_group)
        
        splitter.addWidget(log_widget)
        splitter.setSizes([800, 400])
        
        main_layout.addWidget(splitter)
    
    def refresh_ports(self):
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(f"{port.device} - {port.description}")
    
    def toggle_connection(self):
        if not self.serial_thread.running:
            port = self.port_combo.currentText().split(' ')[0]
            if self.serial_thread.set_serial(port):
                self.serial_thread.start()
                self.connect_btn.setText("断开")
                self.test_btn.setEnabled(True)
                self.status_label.setText("✓ 已连接")
                self.status_label.setStyleSheet("color: green; font-weight: bold; font-size: 14px;")
                self.log(f"✓ 已连接到 {port}")
            else:
                QMessageBox.critical(self, "错误", "串口连接失败!")
                self.log("✗ 串口连接失败", is_error=True)
        else:
            self.serial_thread.stop()
            self.serial_thread.wait()
            self.connect_btn.setText("连接")
            self.test_btn.setEnabled(False)
            self.status_label.setText("未连接")
            self.status_label.setStyleSheet("color: red; font-weight: bold; font-size: 14px;")
            self.log("串口已断开")
    
    def test_communication(self):
        if not self.is_connected():
            return
        
        self.log("="*50)
        self.log("🔧 开始测试Commander通信...")
        self.log("发送测试命令: A?")
        self.send_command("A?")
    
    def is_connected(self):
        return self.serial_thread.running
    
    def send_command(self, cmd):
        return self.serial_thread.send_command(cmd)
    
    def process_serial_data(self, line):
        """处理串口接收的数据"""
        # 调试模式显示所有数据
        if self.debug_mode:
            self.log(f"← {line}", is_receive=True)
        
        # 解析数据流格式: #DATA,ID,Target,Control
        if line.startswith("#DATA,"):
            parts = line.split(',')
            if len(parts) == 4:
                try:
                    panel_id = parts[1].strip()
                    target = float(parts[2].strip())
                    control = float(parts[3].strip())
                    
                    # PID 面板映射
                    panel_map = {
                        'A': 'angle', 'B': 'gyro', 'C': 'distance',
                        'D': 'speed', 'E': 'yaw_angle', 'F': 'yaw_gyro',
                        'H': 'lqr_u', 'I': 'zeropoint', 'K': 'roll_angle'
                    }
                    
                    # LPF 面板映射
                    lpf_map = {
                        'G': 'joyy',      # 摇杆滤波
                        'J': 'zeropoint', # 零点滤波
                        'L': 'roll'       # Roll滤波
                    }
                    
                    # 轮速调试面板映射
                    wheel_map = {
                        'O': 'left',      # 左轮调试
                        'P': 'right'      # 右轮调试
                    }
                    
                    if panel_id in panel_map:
                        panel_key = panel_map[panel_id]
                        if panel_key in self.pid_panels:
                            self.pid_panels[panel_key].update_plot(target, control)
                    elif panel_id in lpf_map:
                        panel_key = lpf_map[panel_id]
                        if panel_key in self.lpf_panels:
                            self.lpf_panels[panel_key].update_plot(target, control)
                    elif panel_id in wheel_map:
                        panel_key = wheel_map[panel_id]
                        if panel_key in self.wheel_panels:
                            self.wheel_panels[panel_key].update_plot(target, control)
                    return
                except (ValueError, IndexError) as e:
                    self.log(f"解析数据失败: {line} ({e})", is_error=True)
                    return
        
        # 解析Web命令格式: #WEB,go,dir,joyx,joyy,height
        if line.startswith("#WEB,"):
            parts = line.split(',')
            if len(parts) == 6:
                try:
                    go = parts[1].strip()
                    dir_val = parts[2].strip()
                    joyx = parts[3].strip()
                    joyy = parts[4].strip()
                    height = parts[5].strip()
                    
                    self.web_monitor.update_web_status(go, dir_val, joyx, joyy, height)
                    return
                except (ValueError, IndexError):
                    pass
        
        # 非调试模式只显示重要信息
        if not self.debug_mode:
            if "err" in line.lower():
                self.log(f"⚠ {line}", is_error=True)
            elif any(kw in line for kw in ['PID:', 'LPF:', 'Speed']):
                self.log(f"← {line}", is_receive=True)
        
        # 解析PID参数返回
        match = re.search(r'PID:\s*P:\s*([-\d.]+)\s*I:\s*([-\d.]+)\s*D:\s*([-\d.]+)\s*R:\s*([-\d.]+)\s*L:\s*([-\d.]+)', line)
        if match:
            p = float(match.group(1))
            i = float(match.group(2))
            d = float(match.group(3))
            ramp = float(match.group(4))
            limit = float(match.group(5))
            
            current_widget = self.tab_widget.currentWidget()
            if isinstance(current_widget, PIDControlPanel):
                current_widget.update_display(p, i, d, limit, ramp)
        
        # 解析LPF返回
        match = re.search(r'LPF:\s*Tf:\s*([-\d.]+)', line)
        if match:
            tf = float(match.group(1))
            current_widget = self.tab_widget.currentWidget()
            if isinstance(current_widget, LPFControlPanel):
                current_widget.update_display(tf)
        
        # 解析速度自适应P参数 (只有Low和High两个参数)
        match = re.search(r'Speed Adaptive P:\s*Low=([-\d.]+)\s*High=([-\d.]+)', line)
        if match:
            low = float(match.group(1))
            high = float(match.group(2))
            if self.speed_adaptive_panel:
                self.speed_adaptive_panel.update_display(low, high)
        
        # ===== 新增: 设备控制面板数据解析 =====
        
        # 检测 balance init 成功
        if "Balance test init: OK" in line or "Balance test module initialized" in line:
            if hasattr(self, 'balance_panel'):
                self.balance_panel.on_init_success()
            self.log(f"✓ {line}", is_receive=True)
        
        # 检测 IMU 数据: Roll=xxx Pitch=xxx Yaw=xxx
        # 格式: "Angle: Roll= +0.05 Pitch= -0.63 Yaw= +67.12 (°)"
        imu_match = re.search(r'Roll[=:]\s*([-+]?\d+\.?\d*)\s+Pitch[=:]\s*([-+]?\d+\.?\d*)\s+Yaw[=:]\s*([-+]?\d+\.?\d*)', line, re.IGNORECASE)
        if imu_match:
            try:
                roll = float(imu_match.group(1))
                pitch = float(imu_match.group(2))
                yaw = float(imu_match.group(3))
                if hasattr(self, 'imu_panel') and self.imu_panel is not None:
                    self.imu_panel.update_imu_data(roll, pitch, yaw, 0, 0, 0)
                    # 在状态栏显示 IMU 数据
                    self.status_label.setText(f"R:{roll:.1f}° P:{pitch:.1f}° Y:{yaw:.1f}°")
                # 同时更新腿部面板中的 Roll 显示 (如果有)
                if hasattr(self, 'leg_panel') and self.leg_panel is not None:
                    if hasattr(self.leg_panel, 'update_roll_display'):
                        self.leg_panel.update_roll_display(roll)
            except Exception as e:
                if self.debug_mode:
                    self.log(f"IMU 解析错误: {e}", is_error=True)
        
        # 检测 Gyro 数据: Gyro: X= +0.18 Y= -0.18 Z= -0.49 (°/s)
        gyro_match = re.search(r'Gyro:\s*X[=:]\s*([-+]?\d+\.?\d*)\s+Y[=:]\s*([-+]?\d+\.?\d*)\s+Z[=:]\s*([-+]?\d+\.?\d*)', line, re.IGNORECASE)
        if gyro_match:
            try:
                gx = float(gyro_match.group(1))
                gy = float(gyro_match.group(2))
                gz = float(gyro_match.group(3))
                if hasattr(self, 'imu_panel') and self.imu_panel is not None:
                    # 只更新角速度部分
                    self.imu_panel.gx_label.setText(f"{gx:.2f}°/s")
                    self.imu_panel.gy_label.setText(f"{gy:.2f}°/s")
                    self.imu_panel.gz_label.setText(f"{gz:.2f}°/s")
            except Exception as e:
                if self.debug_mode:
                    self.log(f"Gyro 解析错误: {e}", is_error=True)
        
        # 检测电机状态: M1: pos=xxx spd=xxx cur=xxx ONLINE/OFFLINE
        motor_match = re.search(r'M(\d):\s*pos=([-\d.]+).*spd=([-\d.]+).*cur=([-\d.]+).*?(ONLINE|OFFLINE)', line, re.IGNORECASE)
        if motor_match:
            try:
                motor_id = int(motor_match.group(1))
                pos = float(motor_match.group(2))
                spd = float(motor_match.group(3))
                cur = float(motor_match.group(4))
                online = motor_match.group(5).upper() == "ONLINE"
                if hasattr(self, 'motor_panel'):
                    self.motor_panel.update_motor_status(motor_id, pos, spd, cur, online)
            except:
                pass
        
        # ===== 腿部运动学测试结果解析 =====
        
        # FK 测试结果: FK (left): Hip=xxx, Knee=xxx -> Length=xxxm, Angle=xxxdeg
        fk_match = re.search(r'FK\s*\((\w+)\):\s*Hip=([-\d.]+),\s*Knee=([-\d.]+)\s*->\s*Length=([-\d.]+)m,\s*Angle=([-\d.]+)deg', line)
        if fk_match:
            side = fk_match.group(1)
            hip = float(fk_match.group(2))
            knee = float(fk_match.group(3))
            length = float(fk_match.group(4))
            angle = float(fk_match.group(5))
            result = f"FK ({side}): Hip={hip:.1f}°, Knee={knee:.1f}° → L={length:.3f}m, θ={angle:.1f}°"
            if hasattr(self, 'leg_panel'):
                self.leg_panel.update_test_result(result)
            self.log(f"✓ {result}", is_receive=True)
        
        # IK 测试结果: IK (left): Length=xxxm, Angle=xxxdeg -> Hip=xxx, Knee=xxx
        ik_match = re.search(r'IK\s*\((\w+)\):\s*Length=([-\d.]+)m,\s*Angle=([-\d.]+)deg\s*->\s*Hip=([-\d.]+),\s*Knee=([-\d.]+)', line)
        if ik_match:
            side = ik_match.group(1)
            length = float(ik_match.group(2))
            angle = float(ik_match.group(3))
            hip = float(ik_match.group(4))
            knee = float(ik_match.group(5))
            result = f"IK ({side}): L={length:.3f}m, θ={angle:.1f}° → Hip={hip:.1f}°, Knee={knee:.1f}°"
            if hasattr(self, 'leg_panel'):
                self.leg_panel.update_test_result(result)
            self.log(f"✓ {result}", is_receive=True)
        
        # IK 失败: IK failed (target unreachable)
        if "IK failed" in line or "target unreachable" in line:
            if hasattr(self, 'leg_panel'):
                self.leg_panel.update_test_result("❌ IK 失败: 目标不可达")
            self.log(f"⚠ {line}", is_error=True)
        
        # FK 失败
        if "FK failed" in line:
            if hasattr(self, 'leg_panel'):
                self.leg_panel.update_test_result("❌ FK 失败")
            self.log(f"⚠ {line}", is_error=True)
        
        # 腿部状态解析: LEG_STATE: L_Len=xxx L_Ang=xxx L_Hip=xxx L_Knee=xxx R_Len=xxx ...
        leg_state_match = re.search(
            r'LEG_STATE:\s*L_Len=([-\d.]+)\s*L_Ang=([-\d.]+)\s*L_Hip=([-\d.]+)\s*L_Knee=([-\d.]+)\s*'
            r'R_Len=([-\d.]+)\s*R_Ang=([-\d.]+)\s*R_Hip=([-\d.]+)\s*R_Knee=([-\d.]+)', line)
        if leg_state_match:
            try:
                left_state = {
                    'length': float(leg_state_match.group(1)),
                    'angle': float(leg_state_match.group(2)),
                    'hip': float(leg_state_match.group(3)),
                    'knee': float(leg_state_match.group(4))
                }
                right_state = {
                    'length': float(leg_state_match.group(5)),
                    'angle': float(leg_state_match.group(6)),
                    'hip': float(leg_state_match.group(7)),
                    'knee': float(leg_state_match.group(8))
                }
                if hasattr(self, 'leg_panel'):
                    self.leg_panel.update_leg_state(left_state, right_state)
            except:
                pass
        
        # 目标设置成功: Target set: Length=xxxm, Angle=xxxdeg
        target_match = re.search(r'Target set:\s*Length=([-\d.]+)m,\s*Angle=([-\d.]+)deg', line)
        if target_match:
            self.log(f"✓ {line}", is_receive=True)
        
        # ===== 任务频率解析 =====
        # 格式: FREQ:IMU=xxx,CTRL=xxx,MOTOR=xxx,LEG=xxx
        freq_match = re.search(r'FREQ:IMU=([-\d.]+),CTRL=([-\d.]+),MOTOR=([-\d.]+),LEG=([-\d.]+)', line)
        if freq_match:
            try:
                imu_hz = float(freq_match.group(1))
                ctrl_hz = float(freq_match.group(2))
                motor_hz = float(freq_match.group(3))
                leg_hz = float(freq_match.group(4))
                if hasattr(self, 'balance_panel'):
                    self.balance_panel.update_task_freq(imu_hz, ctrl_hz, motor_hz, leg_hz)
                self.log(f"📈 频率: IMU={imu_hz:.0f}Hz, Ctrl={ctrl_hz:.0f}Hz, Motor={motor_hz:.0f}Hz, Leg={leg_hz:.0f}Hz", is_receive=True)
            except:
                pass
        
        # ===== 延迟诊断解析 =====
        # 格式: LATENCY:IMU_CTRL=xxx,CALC=xxx,CTRL_MOTOR=xxx,TOTAL=xxx,AVG=xxx,MIN=xxx,MAX=xxx
        latency_match = re.search(r'LATENCY:IMU_CTRL=([-\d.]+),CALC=([-\d.]+),CTRL_MOTOR=([-\d.]+),TOTAL=([-\d.]+)(?:,AVG=([-\d.]+),MIN=([-\d.]+),MAX=([-\d.]+))?', line)
        if latency_match:
            try:
                imu_ctrl_us = float(latency_match.group(1))
                calc_us = float(latency_match.group(2))
                ctrl_motor_us = float(latency_match.group(3))
                total_us = float(latency_match.group(4))
                # 可选的统计字段
                avg_us = float(latency_match.group(5)) if latency_match.group(5) else total_us
                min_us = float(latency_match.group(6)) if latency_match.group(6) else total_us
                max_us = float(latency_match.group(7)) if latency_match.group(7) else total_us
                if hasattr(self, 'balance_panel'):
                    self.balance_panel.update_latency(imu_ctrl_us, calc_us, ctrl_motor_us, total_us, avg_us, min_us, max_us)
                self.log(f"⏱️ IMU延迟: Total={total_us:.0f}us (Avg={avg_us:.0f}, Min={min_us:.0f}, Max={max_us:.0f}us)", is_receive=True)
            except:
                pass
        
        # ===== WiFi 遥控延迟解析 =====
        # 格式: WIFI_LATENCY:WIFI_CTRL=xxx,TOTAL=xxx,AVG=xxx,MIN=xxx,MAX=xxx
        wifi_latency_match = re.search(r'WIFI_LATENCY:WIFI_CTRL=([-\d.]+),TOTAL=([-\d.]+),AVG=([-\d.]+),MIN=([-\d.]+),MAX=([-\d.]+)', line)
        if wifi_latency_match:
            try:
                wifi_ctrl_us = float(wifi_latency_match.group(1))
                wifi_total_us = float(wifi_latency_match.group(2))
                wifi_avg_us = float(wifi_latency_match.group(3))
                wifi_min_us = float(wifi_latency_match.group(4))
                wifi_max_us = float(wifi_latency_match.group(5))
                if hasattr(self, 'balance_panel'):
                    self.balance_panel.update_wifi_latency(wifi_ctrl_us, wifi_total_us, wifi_avg_us, wifi_min_us, wifi_max_us)
                self.log(f"📶 WiFi延迟: Total={wifi_total_us:.0f}us (Avg={wifi_avg_us:.0f}, Min={wifi_min_us:.0f}, Max={wifi_max_us:.0f}us)", is_receive=True)
            except:
                pass
        
        # ===== YAW 调试数据解析 =====
        # 格式: #YAW_DBG,out=xxx,err=xxx,hold=x,rate=xxx
        yaw_dbg_match = re.search(r'#YAW_DBG,out=([-\d.]+),err=([-\d.]+),hold=(\d),rate=([-\d.]+)', line)
        if yaw_dbg_match:
            try:
                yaw_output = float(yaw_dbg_match.group(1))
                yaw_error = float(yaw_dbg_match.group(2))
                yaw_holding = int(yaw_dbg_match.group(3)) == 1
                yaw_rate = float(yaw_dbg_match.group(4))
                # 更新 YAW 调试面板
                if hasattr(self, 'yaw_panel'):
                    self.yaw_panel.update_yaw_debug(
                        self._yaw_target_angle, self._yaw_current_angle,
                        yaw_error, yaw_output, yaw_holding, yaw_rate
                    )
            except:
                pass
            return  # 不在日志显示
        
        # 格式: #DATA,Y,target_angle,current_angle (YAW 角度数据)
        if line.startswith("#DATA,Y,"):
            parts = line.split(',')
            if len(parts) == 4:
                try:
                    self._yaw_target_angle = float(parts[2].strip())
                    self._yaw_current_angle = float(parts[3].strip())
                except:
                    pass
            return  # 不在日志显示
    
    def log(self, msg, is_receive=False, is_error=False):
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        
        if is_error:
            color = "#ff4444"
            prefix = "ERR"
        elif is_receive:
            color = "#00aaff"
            prefix = "RX"
        else:
            color = "#00ff00"
            prefix = "TX"
        
        self.log_text.append(f'<span style="color: gray;">[{timestamp}]</span> '
                            f'<span style="color: {color}; font-weight: bold;">[{prefix}]</span> '
                            f'<span style="color: white;">{msg}</span>')
    
    def toggle_debug_mode(self, enabled):
        self.debug_mode = enabled
        if enabled:
            self.log("🐛 调试模式已开启 - 将显示所有串口数据", is_error=True)
        else:
            self.log("🐛 调试模式已关闭", is_error=True)
    
    def show_help(self):
        help_text = """
<h3>SimpleFOC Commander PID调参工具使用说明</h3>

<h4>📌 快速开始</h4>
<ol>
<li><b>连接串口</b>: 选择ESP32的COM口,点击"连接"</li>
<li><b>测试通信</b>: 点击"测试通信"按钮,检查Commander是否正常</li>
<li><b>选择PID</b>: 切换到要调节的PID标签页</li>
<li><b>修改参数</b>: 在输入框输入新值,点击对应的"设置"按钮</li>
</ol>

<h4>🎯 Commander命令格式</h4>
<pre>
&lt;ID&gt;&lt;param&gt;&lt;value&gt;  设置参数
&lt;ID&gt;?               查询参数

例如: AP1.5 = 设置A控制器的P值为1.5
</pre>

<h4>💡 调参技巧</h4>
<ul>
<li>从P开始,逐步增加,每次小幅度调整(±0.1)</li>
<li>观察波形响应,避免震荡</li>
<li>测试稳定性,轻推机器人观察恢复情况</li>
</ul>
"""
        QMessageBox.information(self, "使用帮助", help_text)
    
    def closeEvent(self, event):
        if self.serial_thread.running:
            self.serial_thread.stop()
            self.serial_thread.wait()
        event.accept()


# ============================================================================
# 深色主题
# ============================================================================
def apply_dark_theme(app):
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


if __name__ == '__main__':
    app = QApplication(sys.argv)
    app.setStyle('Fusion')
    apply_dark_theme(app)
    
    window = PIDTunerUI()
    window.show()
    sys.exit(app.exec_())
