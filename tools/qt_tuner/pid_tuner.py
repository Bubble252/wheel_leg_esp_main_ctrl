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
    QSlider, QCheckBox, QFrame, QProgressBar, QLineEdit, QScrollArea,
    QFileDialog, QTableWidget, QTableWidgetItem, QHeaderView
)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QThread
from PyQt5.QtGui import QFont, QPalette, QColor
import pyqtgraph as pg
import numpy as np


# ============================================================================
# DPI 自适应缩放
# ============================================================================
# 全局缩放因子, 根据屏幕分辨率自动计算
# 基准: 1920x1080, scale=1.0
_SCALE_FACTOR = 1.0

def get_scale():
    """获取当前缩放因子"""
    return _SCALE_FACTOR

def sp(px):
    """缩放像素值 (Scaled Pixels): 将设计时的像素值按屏幕 DPI 缩放
    用于 padding, margin, border-radius 等"""
    return max(1, int(px * _SCALE_FACTOR))

def sf(pt):
    """缩放字体大小 (Scaled Font): 将设计时的字号按屏幕 DPI 缩放"""
    return max(8, int(pt * _SCALE_FACTOR))

def _compute_scale_factor(app):
    """根据屏幕物理 DPI 和分辨率计算缩放因子"""
    global _SCALE_FACTOR
    try:
        screen = app.primaryScreen()
        if screen is None:
            _SCALE_FACTOR = 1.0
            return
        dpi = screen.logicalDotsPerInch()
        geo = screen.availableGeometry()
        # 基准: 96 DPI, 1920x1080
        dpi_scale = dpi / 96.0
        # 取宽高中较小的维度来计算分辨率缩放
        res_scale = min(geo.width() / 1920.0, geo.height() / 1080.0)
        # 综合: DPI 权重 0.6, 分辨率权重 0.4
        _SCALE_FACTOR = max(0.6, min(3.0, dpi_scale * 0.6 + res_scale * 0.4))
    except Exception:
        _SCALE_FACTOR = 1.0


def _build_global_stylesheet():
    """构建全局自适应 stylesheet, 基于当前 _SCALE_FACTOR"""
    s = _SCALE_FACTOR
    return f"""
        /* ---- 全局字体基线 ---- */
        QWidget {{
            font-size: {sf(13)}px;
        }}
        QGroupBox {{
            font-size: {sf(13)}px;
            font-weight: bold;
            padding-top: {sp(14)}px;
            margin-top: {sp(8)}px;
        }}
        QGroupBox::title {{
            subcontrol-origin: margin;
            left: {sp(10)}px;
            padding: 0 {sp(4)}px;
        }}
        QPushButton {{
            font-size: {sf(12)}px;
            padding: {sp(5)}px {sp(12)}px;
            min-height: {sp(22)}px;
        }}
        QLabel {{
            font-size: {sf(12)}px;
        }}
        QComboBox, QSpinBox, QDoubleSpinBox, QLineEdit {{
            font-size: {sf(12)}px;
            padding: {sp(3)}px;
            min-height: {sp(20)}px;
        }}
        QTextEdit {{
            font-size: {sf(11)}px;
        }}
        QTabBar::tab {{
            font-size: {sf(11)}px;
            padding: {sp(5)}px {sp(10)}px;
            min-width: {sp(60)}px;
        }}
        QCheckBox, QRadioButton {{
            font-size: {sf(12)}px;
            spacing: {sp(4)}px;
        }}
        QSlider::groove:horizontal {{
            height: {sp(6)}px;
        }}
        QSlider::handle:horizontal {{
            width: {sp(14)}px;
            height: {sp(14)}px;
            margin: -{sp(4)}px 0;
        }}
        QProgressBar {{
            font-size: {sf(11)}px;
            min-height: {sp(16)}px;
        }}
        QScrollArea {{
            border: none;
            background: transparent;
        }}
        QScrollArea > QWidget > QWidget {{
            background: transparent;
        }}
    """


# 正则: 匹配 font-size: XXpx 并缩放
_RE_FONT_SIZE = re.compile(r'font-size:\s*(\d+)px')
_RE_PADDING = re.compile(r'padding:\s*(\d+)px')
_RE_MIN_WIDTH = re.compile(r'min-width:\s*(\d+)px')

def SS(style_str):
    """Scaled StyleSheet — 自动将 inline stylesheet 中的 px 值按 DPI 缩放.
    用法: widget.setStyleSheet(SS("font-size: 14px; padding: 5px;"))
    """
    if _SCALE_FACTOR == 1.0:
        return style_str
    def _scale_font(m):
        return f'font-size: {sf(int(m.group(1)))}px'
    def _scale_padding(m):
        return f'padding: {sp(int(m.group(1)))}px'
    def _scale_min_width(m):
        return f'min-width: {sp(int(m.group(1)))}px'
    result = _RE_FONT_SIZE.sub(_scale_font, style_str)
    result = _RE_PADDING.sub(_scale_padding, result)
    result = _RE_MIN_WIDTH.sub(_scale_min_width, result)
    return result


def _make_scrollable(widget):
    """将 widget 包裹在 QScrollArea 中, 使其可滚动.
    用于 tab 页内容过多时允许垂直滚动."""
    scroll = QScrollArea()
    scroll.setWidget(widget)
    scroll.setWidgetResizable(True)
    scroll.setFrameShape(QFrame.NoFrame)  # 无边框, 视觉一致
    scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
    return scroll


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
    
    def __init__(self, name, commander_id, parent=None, output_channel_id=None, output_dual=False,
                 extra_channel_id=None, extra_labels=None):
        super().__init__(parent)
        self.name = name
        self.commander_id = commander_id
        self.parent_window = parent
        self.output_channel_id = output_channel_id  # 环路输出通道 ID (Q/R/S/T/U/V)
        self.output_dual = output_dual  # 是否双线模式 (target + control)
        self.extra_channel_id = extra_channel_id  # 额外观测通道 ID (如 'Z')
        self.extra_labels = extra_labels or ('值1', '值2')  # 额外通道的两条线标签
        
        # 数据缓存 (最多保存500个点)
        self.max_points = 500
        self.time_data = deque(maxlen=self.max_points)
        self.target_data = deque(maxlen=self.max_points)
        self.control_data = deque(maxlen=self.max_points)
        self.data_counter = 0
        
        # 环路输出波形数据缓存
        self.output_time_data = deque(maxlen=self.max_points)
        self.output_data = deque(maxlen=self.max_points)
        self.output_data2 = deque(maxlen=self.max_points)  # 双线模式第二条线
        self.output_counter = 0
        
        # 额外通道波形数据缓存
        self.extra_time_data = deque(maxlen=self.max_points)
        self.extra_data1 = deque(maxlen=self.max_points)
        self.extra_data2 = deque(maxlen=self.max_points)
        self.extra_counter = 0
        
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
        self.p_input.setDecimals(6)
        self.p_input.setSingleStep(0.0001)
        self.p_input.setStyleSheet(SS("font-size: 14px; padding: 5px;"))
        param_layout.addWidget(self.p_input, 0, 1)
        
        self.p_set_btn = QPushButton("设置 P")
        self.p_set_btn.clicked.connect(lambda: self.set_param('P', self.p_input.value()))
        param_layout.addWidget(self.p_set_btn, 0, 2)
        
        # I参数
        param_layout.addWidget(QLabel("I (积分):"), 1, 0)
        self.i_input = QDoubleSpinBox()
        self.i_input.setRange(-1000, 1000)
        self.i_input.setDecimals(6)
        self.i_input.setSingleStep(0.0001)
        self.i_input.setStyleSheet(SS("font-size: 14px; padding: 5px;"))
        param_layout.addWidget(self.i_input, 1, 1)
        
        self.i_set_btn = QPushButton("设置 I")
        self.i_set_btn.clicked.connect(lambda: self.set_param('I', self.i_input.value()))
        param_layout.addWidget(self.i_set_btn, 1, 2)
        
        # D参数
        param_layout.addWidget(QLabel("D (微分):"), 2, 0)
        self.d_input = QDoubleSpinBox()
        self.d_input.setRange(-1000, 1000)
        self.d_input.setDecimals(6)
        self.d_input.setSingleStep(0.0001)
        self.d_input.setStyleSheet(SS("font-size: 14px; padding: 5px;"))
        param_layout.addWidget(self.d_input, 2, 1)
        
        self.d_set_btn = QPushButton("设置 D")
        self.d_set_btn.clicked.connect(lambda: self.set_param('D', self.d_input.value()))
        param_layout.addWidget(self.d_set_btn, 2, 2)
        
        # Limit参数
        param_layout.addWidget(QLabel("Limit (限幅):"), 3, 0)
        self.limit_input = QDoubleSpinBox()
        self.limit_input.setRange(0, 1000)
        self.limit_input.setDecimals(4)
        self.limit_input.setSingleStep(0.1)
        self.limit_input.setStyleSheet(SS("font-size: 14px; padding: 5px;"))
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
        self.ramp_input.setStyleSheet(SS("font-size: 14px; padding: 5px;"))
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
        self.query_btn.setStyleSheet(SS("font-size: 14px; font-weight: bold; padding: 10px;"))
        self.query_btn.clicked.connect(self.query_params)
        action_layout.addWidget(self.query_btn)
        
        self.set_all_btn = QPushButton("📤 发送全部参数")
        self.set_all_btn.setStyleSheet(SS("font-size: 14px; font-weight: bold; padding: 10px;"))
        self.set_all_btn.clicked.connect(self.set_all_params)
        action_layout.addWidget(self.set_all_btn)
        
        self.reset_btn = QPushButton("🔄 重置为0")
        self.reset_btn.setStyleSheet(SS("font-size: 14px; padding: 10px;"))
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
        
        # 环路输出波形显示区 (仅当指定了输出通道时显示)
        if self.output_channel_id:
            if self.output_dual:
                output_group = QGroupBox(f"📊 环路输出对比 (通道 {self.output_channel_id}: 蓝=处理前, 绿=处理后)")
            else:
                output_group = QGroupBox(f"📊 环路输出大小 (通道 {self.output_channel_id})")
            output_plot_layout = QVBoxLayout()
            
            # 当前输出值显示
            self.output_value_label = QLabel("当前输出: --")
            self.output_value_label.setStyleSheet(SS(
                "font-size: 16px; font-weight: bold; color: #00ff88; "
                "background-color: #1a2a1a; padding: 8px; border-radius: 5px; "
                "border: 1px solid #00ff88;"
            ))
            self.output_value_label.setAlignment(Qt.AlignCenter)
            output_plot_layout.addWidget(self.output_value_label)
            
            self.output_plot_widget = pg.PlotWidget()
            self.output_plot_widget.setBackground('#1a1a2e')
            self.output_plot_widget.showGrid(x=True, y=True, alpha=0.3)
            self.output_plot_widget.setLabel('left', '输出大小')
            self.output_plot_widget.setLabel('bottom', '时间 (采样点)')
            self.output_plot_widget.setMinimumHeight(150)
            self.output_plot_widget.setMaximumHeight(200)
            
            # 零线
            self.output_plot_widget.addLine(y=0, pen=pg.mkPen(color='w', width=1, style=Qt.DashLine))
            
            if self.output_dual:
                self.output_plot_widget.addLegend()
                # 蓝色 = 处理前 (raw), 绿色 = 处理后 (final)
                self.output_curve2 = self.output_plot_widget.plot(
                    pen=pg.mkPen(color='#4488ff', width=2), name='处理前(raw)'
                )
                self.output_curve = self.output_plot_widget.plot(
                    pen=pg.mkPen(color='#00ff88', width=2), name='处理后(final)'
                )
            else:
                self.output_curve = self.output_plot_widget.plot(
                    pen=pg.mkPen(color='#00ff88', width=2), name='环路输出'
                )
            
            output_plot_layout.addWidget(self.output_plot_widget)
            
            # 输出波形控制按钮
            out_btn_layout = QHBoxLayout()
            self.clear_output_btn = QPushButton("清空输出波形")
            self.clear_output_btn.clicked.connect(self.clear_output_plot)
            out_btn_layout.addWidget(self.clear_output_btn)
            
            self.pause_output_btn = QPushButton("暂停")
            self.pause_output_btn.setCheckable(True)
            out_btn_layout.addWidget(self.pause_output_btn)
            
            out_btn_layout.addStretch()
            output_plot_layout.addLayout(out_btn_layout)
            
            output_group.setLayout(output_plot_layout)
            layout.addWidget(output_group)
        
        # 额外观测通道波形显示区 (仅当指定了 extra_channel_id 时显示)
        if self.extra_channel_id:
            self._init_extra_channel_ui(layout)
    
    def _init_extra_channel_ui(self, layout):
        """初始化额外观测通道的波形UI"""
        label1, label2 = self.extra_labels
        extra_group = QGroupBox(
            f"📊 观测通道 {self.extra_channel_id} (🟢绿={label1}, 🟡黄={label2})"
        )
        extra_layout = QVBoxLayout()
        
        # 当前值显示
        self.extra_value_label = QLabel(f"{label1}: --  |  {label2}: --")
        self.extra_value_label.setStyleSheet(SS(
            "font-size: 14px; font-weight: bold; color: #cccc00; "
            "background-color: #1a1a0a; padding: 8px; border-radius: 5px; "
            "border: 1px solid #666600;"
        ))
        self.extra_value_label.setAlignment(Qt.AlignCenter)
        extra_layout.addWidget(self.extra_value_label)
        
        # 波形图
        self.extra_plot_widget = pg.PlotWidget()
        self.extra_plot_widget.setBackground('#0a1a0a')
        self.extra_plot_widget.showGrid(x=True, y=True, alpha=0.3)
        self.extra_plot_widget.setLabel('left', '数值')
        self.extra_plot_widget.setLabel('bottom', '时间 (采样点)')
        self.extra_plot_widget.setMinimumHeight(150)
        self.extra_plot_widget.setMaximumHeight(200)
        self.extra_plot_widget.addLegend()
        
        # 零线
        self.extra_plot_widget.addLine(y=0, pen=pg.mkPen(color='w', width=1, style=Qt.DashLine))
        
        # 绿色=值1, 黄色=值2
        self.extra_curve1 = self.extra_plot_widget.plot(
            pen=pg.mkPen(color='#00cc66', width=2), name=label1
        )
        self.extra_curve2 = self.extra_plot_widget.plot(
            pen=pg.mkPen(color='#cccc00', width=2), name=label2
        )
        
        extra_layout.addWidget(self.extra_plot_widget)
        
        # 控制按钮
        extra_btn_layout = QHBoxLayout()
        self.clear_extra_btn = QPushButton("清空")
        self.clear_extra_btn.clicked.connect(self.clear_extra_plot)
        extra_btn_layout.addWidget(self.clear_extra_btn)
        
        self.pause_extra_btn = QPushButton("暂停")
        self.pause_extra_btn.setCheckable(True)
        extra_btn_layout.addWidget(self.pause_extra_btn)
        
        extra_btn_layout.addStretch()
        extra_layout.addLayout(extra_btn_layout)
        
        extra_group.setLayout(extra_layout)
        layout.addWidget(extra_group)
    
    def create_param_display(self, text):
        """创建参数显示标签"""
        label = QLabel(text)
        label.setStyleSheet(SS("font-size: 16px; font-weight: bold; color: #00ff00; "
                          "background-color: #2b2b2b; padding: 8px; border-radius: 3px;"))
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
    
    def update_output_plot(self, value, value2=None):
        """更新环路输出波形数据
        
        Args:
            value: 主输出值 (单线模式) 或处理后值 (双线模式)
            value2: 双线模式的处理前值 (raw)
        """
        if not self.output_channel_id:
            return
        if hasattr(self, 'pause_output_btn') and self.pause_output_btn.isChecked():
            return
        
        self.output_counter += 1
        self.output_time_data.append(self.output_counter)
        self.output_data.append(value)
        
        self.output_curve.setData(list(self.output_time_data), list(self.output_data))
        
        # 双线模式
        if self.output_dual and value2 is not None:
            self.output_data2.append(value2)
            self.output_curve2.setData(list(self.output_time_data), list(self.output_data2))
            self.output_value_label.setText(f"raw: {value2:.4f}  →  final: {value:.4f}")
        else:
            # 更新数值显示
            self.output_value_label.setText(f"当前输出: {value:.4f}")
    
    def clear_output_plot(self):
        """清除环路输出波形数据"""
        self.output_time_data.clear()
        self.output_data.clear()
        self.output_data2.clear()
        self.output_counter = 0
        if hasattr(self, 'output_curve'):
            self.output_curve.setData([], [])
        if hasattr(self, 'output_curve2'):
            self.output_curve2.setData([], [])
        if hasattr(self, 'output_value_label'):
            self.output_value_label.setText("当前输出: --")
    
    def update_extra_plot(self, value1, value2):
        """更新额外观测通道波形数据"""
        if not self.extra_channel_id:
            return
        if hasattr(self, 'pause_extra_btn') and self.pause_extra_btn.isChecked():
            return
        
        self.extra_counter += 1
        self.extra_time_data.append(self.extra_counter)
        self.extra_data1.append(value1)
        self.extra_data2.append(value2)
        
        self.extra_curve1.setData(list(self.extra_time_data), list(self.extra_data1))
        self.extra_curve2.setData(list(self.extra_time_data), list(self.extra_data2))
        
        # 更新数值显示
        label1, label2 = self.extra_labels
        self.extra_value_label.setText(f"{label1}: {value1:.3f}  |  {label2}: {value2:.1f}")
    
    def clear_extra_plot(self):
        """清除额外通道波形数据"""
        self.extra_time_data.clear()
        self.extra_data1.clear()
        self.extra_data2.clear()
        self.extra_counter = 0
        if hasattr(self, 'extra_curve1'):
            self.extra_curve1.setData([], [])
        if hasattr(self, 'extra_curve2'):
            self.extra_curve2.setData([], [])
        if hasattr(self, 'extra_value_label'):
            label1, label2 = self.extra_labels
            self.extra_value_label.setText(f"{label1}: --  |  {label2}: --")


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
        self.tf_input.setSingleStep(0.001)
        self.tf_input.setValue(0.01)
        self.tf_input.setStyleSheet(SS("font-size: 14px; padding: 5px;"))
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
        self.tf_display.setStyleSheet(SS("font-size: 16px; font-weight: bold; color: #00ff00; "
                                     "background-color: #2b2b2b; padding: 8px;"))
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
# 轮速滤波面板 (双模式: LPF / 限幅滤波)
# ============================================================================
class SpeedFilterPanel(QWidget):
    """轮速滤波面板 - 支持 LPF 和 限幅滤波(Slew-Rate) 两种模式切换"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.commander_id = "W"
        self.parent_window = parent
        
        self.max_points = 500
        self.time_data = deque(maxlen=self.max_points)
        self.raw_data = deque(maxlen=self.max_points)
        self.filtered_data = deque(maxlen=self.max_points)
        self.data_counter = 0
        
        self.current_mode = 0  # 0=LPF, 1=SlewRate
        
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # ========== 模式选择 ==========
        mode_group = QGroupBox("滤波模式选择")
        mode_layout = QHBoxLayout()
        
        self.mode_lpf_btn = QPushButton("📉 低通滤波 (LPF)")
        self.mode_lpf_btn.setCheckable(True)
        self.mode_lpf_btn.setChecked(True)
        self.mode_lpf_btn.setStyleSheet(SS(
            "QPushButton { font-size: 14px; padding: 8px 16px; border: 2px solid #555; border-radius: 6px; }"
            "QPushButton:checked { background-color: #2196F3; color: white; border-color: #1976D2; }"
        ))
        self.mode_lpf_btn.clicked.connect(lambda: self._set_mode(0))
        mode_layout.addWidget(self.mode_lpf_btn)
        
        self.mode_sr_btn = QPushButton("📊 限幅滤波 (Slew-Rate)")
        self.mode_sr_btn.setCheckable(True)
        self.mode_sr_btn.setChecked(False)
        self.mode_sr_btn.setStyleSheet(SS(
            "QPushButton { font-size: 14px; padding: 8px 16px; border: 2px solid #555; border-radius: 6px; }"
            "QPushButton:checked { background-color: #FF9800; color: white; border-color: #F57C00; }"
        ))
        self.mode_sr_btn.clicked.connect(lambda: self._set_mode(1))
        mode_layout.addWidget(self.mode_sr_btn)
        
        mode_group.setLayout(mode_layout)
        layout.addWidget(mode_group)
        
        # ========== LPF 参数 ==========
        self.lpf_group = QGroupBox("低通滤波参数 (LPF)")
        lpf_layout = QGridLayout()
        
        lpf_layout.addWidget(QLabel("Tf (时间常数):"), 0, 0)
        self.tf_input = QDoubleSpinBox()
        self.tf_input.setRange(0, 10)
        self.tf_input.setDecimals(4)
        self.tf_input.setSingleStep(0.001)
        self.tf_input.setValue(0.01)
        self.tf_input.setStyleSheet(SS("font-size: 14px; padding: 5px;"))
        lpf_layout.addWidget(self.tf_input, 0, 1)
        
        self.tf_set_btn = QPushButton("设置 Tf")
        self.tf_set_btn.clicked.connect(self._set_tf)
        lpf_layout.addWidget(self.tf_set_btn, 0, 2)
        
        lpf_layout.addWidget(QLabel("💡 Tf 越大滤波越强，但延迟越大"), 1, 0, 1, 3)
        
        self.lpf_group.setLayout(lpf_layout)
        layout.addWidget(self.lpf_group)
        
        # ========== 限幅滤波参数 ==========
        self.sr_group = QGroupBox("限幅滤波参数 (Slew-Rate Limiter)")
        sr_layout = QGridLayout()
        
        sr_layout.addWidget(QLabel("最大变化率 (单位/秒):"), 0, 0)
        self.rate_input = QDoubleSpinBox()
        self.rate_input.setRange(0.1, 10000)
        self.rate_input.setDecimals(2)
        self.rate_input.setSingleStep(1.0)
        self.rate_input.setValue(50.0)
        self.rate_input.setStyleSheet(SS("font-size: 14px; padding: 5px;"))
        sr_layout.addWidget(self.rate_input, 0, 1)
        
        self.rate_set_btn = QPushButton("设置 Rate")
        self.rate_set_btn.clicked.connect(self._set_rate)
        sr_layout.addWidget(self.rate_set_btn, 0, 2)
        
        sr_layout.addWidget(QLabel(
            "💡 正常变化直通(零延迟)，超过阈值时钳位\n"
            "   适合过滤编码器突跳毛刺，同时保持正常信号无延迟"
        ), 1, 0, 1, 3)
        
        self.sr_group.setLayout(sr_layout)
        self.sr_group.setVisible(False)  # 默认隐藏
        layout.addWidget(self.sr_group)
        
        # ========== 快捷操作 ==========
        action_layout = QHBoxLayout()
        
        self.query_btn = QPushButton("🔍 查询参数")
        self.query_btn.clicked.connect(self._query)
        action_layout.addWidget(self.query_btn)
        
        action_layout.addStretch()
        layout.addLayout(action_layout)
        
        # ========== 当前值显示 ==========
        display_group = QGroupBox("当前参数值")
        display_layout = QGridLayout()
        
        display_layout.addWidget(QLabel("模式:"), 0, 0)
        self.mode_display = QLabel("LPF")
        self.mode_display.setStyleSheet(SS("font-size: 16px; font-weight: bold; color: #2196F3; "
                                           "background-color: #2b2b2b; padding: 8px;"))
        display_layout.addWidget(self.mode_display, 0, 1)
        
        display_layout.addWidget(QLabel("Tf:"), 1, 0)
        self.tf_display = QLabel("--")
        self.tf_display.setStyleSheet(SS("font-size: 16px; font-weight: bold; color: #00ff00; "
                                         "background-color: #2b2b2b; padding: 8px;"))
        display_layout.addWidget(self.tf_display, 1, 1)
        
        display_layout.addWidget(QLabel("Rate:"), 2, 0)
        self.rate_display = QLabel("--")
        self.rate_display.setStyleSheet(SS("font-size: 16px; font-weight: bold; color: #FF9800; "
                                           "background-color: #2b2b2b; padding: 8px;"))
        display_layout.addWidget(self.rate_display, 2, 1)
        
        display_layout.setColumnStretch(1, 1)
        display_group.setLayout(display_layout)
        layout.addWidget(display_group)
        
        # ========== 波形 (滤波前后对比) ==========
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
    
    def _set_mode(self, mode):
        """切换滤波模式并发送到设备"""
        self.current_mode = mode
        self.mode_lpf_btn.setChecked(mode == 0)
        self.mode_sr_btn.setChecked(mode == 1)
        self.lpf_group.setVisible(mode == 0)
        self.sr_group.setVisible(mode == 1)
        
        if self.parent_window and self.parent_window.is_connected():
            cmd = f"WM{mode}"
            self.parent_window.send_command(cmd)
            mode_name = "LPF" if mode == 0 else "SlewRate"
            self.parent_window.log(f"发送: {cmd} -> 轮速滤波模式={mode_name}")
        
        self._update_mode_display()
    
    def _set_tf(self):
        if not self.parent_window or not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        tf = self.tf_input.value()
        cmd = f"WT{tf}"
        self.parent_window.send_command(cmd)
        self.parent_window.log(f"发送: {cmd} -> 轮速LPF Tf={tf}")
    
    def _set_rate(self):
        if not self.parent_window or not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        rate = self.rate_input.value()
        cmd = f"WR{rate}"
        self.parent_window.send_command(cmd)
        self.parent_window.log(f"发送: {cmd} -> 限幅滤波 Rate={rate}")
    
    def _query(self):
        if not self.parent_window or not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        cmd = "W?"
        self.parent_window.send_command(cmd)
        self.parent_window.log("查询轮速滤波参数...")
    
    def _update_mode_display(self):
        if self.current_mode == 0:
            self.mode_display.setText("LPF (低通滤波)")
            self.mode_display.setStyleSheet(SS("font-size: 16px; font-weight: bold; color: #2196F3; "
                                               "background-color: #2b2b2b; padding: 8px;"))
        else:
            self.mode_display.setText("SlewRate (限幅滤波)")
            self.mode_display.setStyleSheet(SS("font-size: 16px; font-weight: bold; color: #FF9800; "
                                               "background-color: #2b2b2b; padding: 8px;"))
    
    def update_display(self, mode, tf, rate):
        """更新显示 (从查询响应调用)"""
        self.current_mode = int(mode)
        self.mode_lpf_btn.setChecked(self.current_mode == 0)
        self.mode_sr_btn.setChecked(self.current_mode == 1)
        self.lpf_group.setVisible(self.current_mode == 0)
        self.sr_group.setVisible(self.current_mode == 1)
        self._update_mode_display()
        
        self.tf_display.setText(f"{tf:.4f}")
        self.rate_display.setText(f"{rate:.2f}")
        self.tf_input.setValue(tf)
        self.rate_input.setValue(rate)
    
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


class GyroFilterPanel(QWidget):
    """角速度滤波面板 - 支持 LPF 和 限幅滤波(Slew-Rate) 两种模式切换"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.commander_id = "N"
        self.parent_window = parent
        
        self.max_points = 500
        self.time_data = deque(maxlen=self.max_points)
        self.raw_data = deque(maxlen=self.max_points)
        self.filtered_data = deque(maxlen=self.max_points)
        self.data_counter = 0
        
        self.current_mode = 1  # 默认限幅滤波
        
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # ========== 模式选择 ==========
        mode_group = QGroupBox("滤波模式选择")
        mode_layout = QHBoxLayout()
        
        self.mode_lpf_btn = QPushButton("📉 低通滤波 (LPF)")
        self.mode_lpf_btn.setCheckable(True)
        self.mode_lpf_btn.setChecked(False)
        self.mode_lpf_btn.setStyleSheet(SS(
            "QPushButton { font-size: 14px; padding: 8px 16px; border: 2px solid #555; border-radius: 6px; }"
            "QPushButton:checked { background-color: #2196F3; color: white; border-color: #1976D2; }"
        ))
        self.mode_lpf_btn.clicked.connect(lambda: self._set_mode(0))
        mode_layout.addWidget(self.mode_lpf_btn)
        
        self.mode_sr_btn = QPushButton("📊 限幅滤波 (Slew-Rate)")
        self.mode_sr_btn.setCheckable(True)
        self.mode_sr_btn.setChecked(True)
        self.mode_sr_btn.setStyleSheet(SS(
            "QPushButton { font-size: 14px; padding: 8px 16px; border: 2px solid #555; border-radius: 6px; }"
            "QPushButton:checked { background-color: #FF9800; color: white; border-color: #F57C00; }"
        ))
        self.mode_sr_btn.clicked.connect(lambda: self._set_mode(1))
        mode_layout.addWidget(self.mode_sr_btn)
        
        mode_group.setLayout(mode_layout)
        layout.addWidget(mode_group)
        
        # ========== LPF 参数 ==========
        self.lpf_group = QGroupBox("低通滤波参数 (LPF)")
        lpf_layout = QGridLayout()
        
        lpf_layout.addWidget(QLabel("Tf (时间常数):"), 0, 0)
        self.tf_input = QDoubleSpinBox()
        self.tf_input.setRange(0, 10)
        self.tf_input.setDecimals(4)
        self.tf_input.setSingleStep(0.001)
        self.tf_input.setValue(0.005)
        self.tf_input.setStyleSheet(SS("font-size: 14px; padding: 5px;"))
        lpf_layout.addWidget(self.tf_input, 0, 1)
        
        self.tf_set_btn = QPushButton("设置 Tf")
        self.tf_set_btn.clicked.connect(self._set_tf)
        lpf_layout.addWidget(self.tf_set_btn, 0, 2)
        
        lpf_layout.addWidget(QLabel("💡 Tf 越大滤波越强，但延迟越大"), 1, 0, 1, 3)
        
        self.lpf_group.setLayout(lpf_layout)
        self.lpf_group.setVisible(False)  # 默认隐藏 (默认限幅模式)
        layout.addWidget(self.lpf_group)
        
        # ========== 限幅滤波参数 ==========
        self.sr_group = QGroupBox("限幅滤波参数 (Slew-Rate Limiter)")
        sr_layout = QGridLayout()
        
        sr_layout.addWidget(QLabel("最大变化率 (°/s²):"), 0, 0)
        self.rate_input = QDoubleSpinBox()
        self.rate_input.setRange(0.1, 100000)
        self.rate_input.setDecimals(1)
        self.rate_input.setSingleStep(10.0)
        self.rate_input.setValue(500.0)
        self.rate_input.setStyleSheet(SS("font-size: 14px; padding: 5px;"))
        sr_layout.addWidget(self.rate_input, 0, 1)
        
        self.rate_set_btn = QPushButton("设置 Rate")
        self.rate_set_btn.clicked.connect(self._set_rate)
        sr_layout.addWidget(self.rate_set_btn, 0, 2)
        
        sr_layout.addWidget(QLabel(
            "💡 正常角速度变化直通(零延迟)，超过阈值时钳位\n"
            "   适合过滤IMU角速度阶梯跳变，同时保持正常信号无延迟"
        ), 1, 0, 1, 3)
        
        self.sr_group.setLayout(sr_layout)
        layout.addWidget(self.sr_group)
        
        # ========== 快捷操作 ==========
        action_layout = QHBoxLayout()
        
        self.query_btn = QPushButton("🔍 查询参数")
        self.query_btn.clicked.connect(self._query)
        action_layout.addWidget(self.query_btn)
        
        action_layout.addStretch()
        layout.addLayout(action_layout)
        
        # ========== 当前值显示 ==========
        display_group = QGroupBox("当前参数值")
        display_layout = QGridLayout()
        
        display_layout.addWidget(QLabel("模式:"), 0, 0)
        self.mode_display = QLabel("SlewRate (限幅滤波)")
        self.mode_display.setStyleSheet(SS("font-size: 16px; font-weight: bold; color: #FF9800; "
                                           "background-color: #2b2b2b; padding: 8px;"))
        display_layout.addWidget(self.mode_display, 0, 1)
        
        display_layout.addWidget(QLabel("Tf:"), 1, 0)
        self.tf_display = QLabel("--")
        self.tf_display.setStyleSheet(SS("font-size: 16px; font-weight: bold; color: #00ff00; "
                                         "background-color: #2b2b2b; padding: 8px;"))
        display_layout.addWidget(self.tf_display, 1, 1)
        
        display_layout.addWidget(QLabel("Rate:"), 2, 0)
        self.rate_display = QLabel("--")
        self.rate_display.setStyleSheet(SS("font-size: 16px; font-weight: bold; color: #FF9800; "
                                           "background-color: #2b2b2b; padding: 8px;"))
        display_layout.addWidget(self.rate_display, 2, 1)
        
        display_layout.setColumnStretch(1, 1)
        display_group.setLayout(display_layout)
        layout.addWidget(display_group)
        
        # ========== 波形 (滤波前后对比) ==========
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
    
    def _set_mode(self, mode):
        """切换滤波模式并发送到设备"""
        self.current_mode = mode
        self.mode_lpf_btn.setChecked(mode == 0)
        self.mode_sr_btn.setChecked(mode == 1)
        self.lpf_group.setVisible(mode == 0)
        self.sr_group.setVisible(mode == 1)
        
        if self.parent_window and self.parent_window.is_connected():
            cmd = f"NM{mode}"
            self.parent_window.send_command(cmd)
            mode_name = "LPF" if mode == 0 else "SlewRate"
            self.parent_window.log(f"发送: {cmd} -> 角速度滤波模式={mode_name}")
        
        self._update_mode_display()
    
    def _set_tf(self):
        if not self.parent_window or not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        tf = self.tf_input.value()
        cmd = f"NT{tf}"
        self.parent_window.send_command(cmd)
        self.parent_window.log(f"发送: {cmd} -> 角速度LPF Tf={tf}")
    
    def _set_rate(self):
        if not self.parent_window or not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        rate = self.rate_input.value()
        cmd = f"NR{rate}"
        self.parent_window.send_command(cmd)
        self.parent_window.log(f"发送: {cmd} -> 角速度限幅滤波 Rate={rate}")
    
    def _query(self):
        if not self.parent_window or not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        cmd = "N?"
        self.parent_window.send_command(cmd)
        self.parent_window.log("查询角速度滤波参数...")
    
    def _update_mode_display(self):
        if self.current_mode == 0:
            self.mode_display.setText("LPF (低通滤波)")
            self.mode_display.setStyleSheet(SS("font-size: 16px; font-weight: bold; color: #2196F3; "
                                               "background-color: #2b2b2b; padding: 8px;"))
        else:
            self.mode_display.setText("SlewRate (限幅滤波)")
            self.mode_display.setStyleSheet(SS("font-size: 16px; font-weight: bold; color: #FF9800; "
                                               "background-color: #2b2b2b; padding: 8px;"))
    
    def update_display(self, mode, tf, rate):
        """更新显示 (从查询响应调用)"""
        self.current_mode = int(mode)
        self.mode_lpf_btn.setChecked(self.current_mode == 0)
        self.mode_sr_btn.setChecked(self.current_mode == 1)
        self.lpf_group.setVisible(self.current_mode == 0)
        self.sr_group.setVisible(self.current_mode == 1)
        self._update_mode_display()
        
        self.tf_display.setText(f"{tf:.4f}")
        self.rate_display.setText(f"{rate:.1f}")
        self.tf_input.setValue(tf)
        self.rate_input.setValue(rate)
    
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
        self.go_label.setStyleSheet(SS("font-size: 18px; font-weight: bold; color: #ff4444;"))
        status_layout.addWidget(self.go_label, 0, 1)
        
        # Dir状态
        status_layout.addWidget(QLabel("Dir (方向):"), 0, 2)
        self.dir_label = QLabel("0")
        self.dir_label.setStyleSheet(SS("font-size: 18px; font-weight: bold; color: #00aaff;"))
        status_layout.addWidget(self.dir_label, 0, 3)
        
        # 摇杆X
        status_layout.addWidget(QLabel("JoyX:"), 1, 0)
        self.joyx_label = QLabel("0")
        self.joyx_label.setStyleSheet(SS("font-size: 18px; font-weight: bold; color: #00ff00;"))
        status_layout.addWidget(self.joyx_label, 1, 1)
        
        # 摇杆Y
        status_layout.addWidget(QLabel("JoyY:"), 1, 2)
        self.joyy_label = QLabel("0")
        self.joyy_label.setStyleSheet(SS("font-size: 18px; font-weight: bold; color: #00ff00;"))
        status_layout.addWidget(self.joyy_label, 1, 3)
        
        # 高度
        status_layout.addWidget(QLabel("Height (高度):"), 2, 0)
        self.height_label = QLabel("0")
        self.height_label.setStyleSheet(SS("font-size: 18px; font-weight: bold; color: #ffaa00;"))
        status_layout.addWidget(self.height_label, 2, 1)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        # 历史记录
        history_group = QGroupBox("命令历史")
        history_layout = QVBoxLayout()
        
        self.history_text = QTextEdit()
        self.history_text.setReadOnly(True)
        self.history_text.setMaximumHeight(sp(200))
        self.history_text.setStyleSheet(SS("background-color: #1e1e1e; color: #d4d4d4; font-family: Consolas;"))
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
            self.go_label.setStyleSheet(SS("font-size: 18px; font-weight: bold; color: #00ff00;"))
        else:
            self.go_label.setText("⭕ 停止")
            self.go_label.setStyleSheet(SS("font-size: 18px; font-weight: bold; color: #ff4444;"))
        
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
        self.kp_max_input.setDecimals(4)
        self.kp_max_input.setValue(1.0)  # 对应 default_params.speed_kp_max
        self.kp_max_input.setSingleStep(0.001)
        self.kp_max_input.setStyleSheet(SS("font-size: 14px; padding: 5px;"))
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
        self.kp_min_input.setDecimals(4)
        self.kp_min_input.setValue(0.3)  # 对应 default_params.speed_kp_min
        self.kp_min_input.setSingleStep(0.001)
        self.kp_min_input.setStyleSheet(SS("font-size: 14px; padding: 5px;"))
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
        self.height_min_input.setStyleSheet(SS("font-size: 14px; padding: 5px;"))
        height_layout.addWidget(self.height_min_input, 0, 1)
        
        # 最大高度
        height_layout.addWidget(QLabel("最大高度 (m):"), 0, 2)
        self.height_max_input = QDoubleSpinBox()
        self.height_max_input.setRange(0, 1)
        self.height_max_input.setDecimals(3)
        self.height_max_input.setValue(0.3)  # 对应代码中的 0.3m
        self.height_max_input.setSingleStep(0.01)
        self.height_max_input.setStyleSheet(SS("font-size: 14px; padding: 5px;"))
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
        self.calculated_kp_display.setStyleSheet(SS("font-size: 16px; font-weight: bold; color: #00ff00; "
                                                  "background-color: #1a1a1a; padding: 8px; border-radius: 4px;"))
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
        principle_text.setMaximumHeight(sp(180))
        principle_text.setStyleSheet(SS("font-size: 12px; background-color: #2b2b2b; color: #d4d4d4; padding: 8px;"))
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
        label.setStyleSheet(SS("font-size: 14px; font-weight: bold; color: #00ff00; "
                          "background-color: #2b2b2b; padding: 5px;"))
        label.setAlignment(Qt.AlignCenter)
        label.setMinimumWidth(sp(80))
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
    LEG_THIGH_LENGTH = 0.065  # 大腿长度 (m)
    LEG_SHANK_LENGTH = 0.065  # 小腿长度 (m)
    DEFAULT_LEG_LENGTH_MIN = 0.045   # 默认最小腿长 (m) - 与 C 代码一致
    DEFAULT_LEG_LENGTH_MAX = 0.11    # 默认最大腿长 (m) - 与 C 代码一致
    LEG_BODY_ANGLE_MIN = -160.0  # 最小身体夹角 (度), 向前蹬腿
    LEG_BODY_ANGLE_MAX = 10.0    # 最大身体夹角 (度), 向后蹬腿
    
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
        warning_label.setStyleSheet(SS(
            "background-color: #ff9800; color: black; padding: 8px; "
            "border-radius: 4px; font-weight: bold;"
        ))
        warning_label.setWordWrap(True)
        layout.addWidget(warning_label)
        
        # ===== 腿部电机使能 =====
        enable_group = QGroupBox("🦿 腿部电机控制")
        enable_layout = QHBoxLayout()
        
        self.leg_status = QLabel("状态: 未知")
        self.leg_status.setStyleSheet(SS("font-size: 14px; font-weight: bold;"))
        enable_layout.addWidget(self.leg_status)
        
        enable_layout.addStretch()
        
        self.leg_on_btn = QPushButton("✅ 使能腿部")
        self.leg_on_btn.setStyleSheet(SS("background-color: #4CAF50; color: white; padding: 8px 20px;"))
        self.leg_on_btn.clicked.connect(lambda: self.send_leg_cmd("on"))
        enable_layout.addWidget(self.leg_on_btn)
        
        self.leg_off_btn = QPushButton("❌ 禁用腿部")
        self.leg_off_btn.setStyleSheet(SS("background-color: #f44336; color: white; padding: 8px 20px;"))
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
        self.leg_min_input.setStyleSheet(SS("font-size: 12px; padding: 3px;"))
        self.leg_min_input.valueChanged.connect(self.on_leg_range_changed)
        range_layout.addWidget(self.leg_min_input, 0, 1)
        
        # 最大腿长
        range_layout.addWidget(QLabel("最大腿长 (m):"), 0, 2)
        self.leg_max_input = QDoubleSpinBox()
        self.leg_max_input.setRange(0.10, 0.20)  # 硬件限制范围
        self.leg_max_input.setValue(self.leg_length_max)
        self.leg_max_input.setDecimals(3)
        self.leg_max_input.setSingleStep(0.005)
        self.leg_max_input.setStyleSheet(SS("font-size: 12px; padding: 3px;"))
        self.leg_max_input.valueChanged.connect(self.on_leg_range_changed)
        range_layout.addWidget(self.leg_max_input, 0, 3)
        
        # 发送到ESP32按钮
        self.send_range_btn = QPushButton("📤 发送到ESP32")
        self.send_range_btn.setStyleSheet(SS("background-color: #9C27B0; color: white; padding: 5px 15px;"))
        self.send_range_btn.clicked.connect(self.send_leg_range_to_esp)
        range_layout.addWidget(self.send_range_btn, 0, 4)
        
        # 重置按钮
        self.reset_range_btn = QPushButton("🔄 重置")
        self.reset_range_btn.setToolTip("重置为默认值 (0.07 ~ 0.17)")
        self.reset_range_btn.clicked.connect(self.reset_leg_range)
        range_layout.addWidget(self.reset_range_btn, 0, 5)
        
        # 说明标签
        range_hint = QLabel("💡 修改范围后会自动更新下方滑块，发送到ESP32可同步设备端限制")
        range_hint.setStyleSheet(SS("font-size: 10px; color: #888;"))
        range_layout.addWidget(range_hint, 1, 0, 1, 6)
        
        range_group.setLayout(range_layout)
        layout.addWidget(range_group)
        
        # ===== 运动学控制 (腿长 + 身体夹角 / 笛卡尔) =====
        kin_group = QGroupBox("📐 运动学控制")
        kin_layout = QGridLayout()
        kin_layout.setSpacing(10)
        
        # 内部标志: 防止循环更新 (在创建控件前初始化)
        self._updating_coord = False
        
        # 坐标系选择
        coord_label = QLabel("坐标系:")
        coord_label.setStyleSheet(SS("font-weight: bold;"))
        kin_layout.addWidget(coord_label, 0, 0)
        
        from PyQt5.QtWidgets import QButtonGroup, QRadioButton
        self.coord_polar_radio = QRadioButton("极坐标 (L, α)")
        self.coord_cart_radio = QRadioButton("笛卡尔 (x, y)")
        self.coord_polar_radio.setChecked(True)
        self.coord_polar_radio.toggled.connect(self.on_coord_mode_changed)
        coord_btn_group = QButtonGroup(self)
        coord_btn_group.addButton(self.coord_polar_radio)
        coord_btn_group.addButton(self.coord_cart_radio)
        kin_layout.addWidget(self.coord_polar_radio, 0, 1)
        kin_layout.addWidget(self.coord_cart_radio, 0, 2, 1, 2)
        
        # --- 极坐标控件 ---
        # 腿长设置
        self.polar_label_L = QLabel("腿长 (m):")
        kin_layout.addWidget(self.polar_label_L, 1, 0)
        self.leg_length_input = QDoubleSpinBox()
        self.leg_length_input.setRange(self.leg_length_min, self.leg_length_max)
        self.leg_length_input.setValue(0.15)
        self.leg_length_input.setDecimals(3)
        self.leg_length_input.setSingleStep(0.005)
        self.leg_length_input.setStyleSheet(SS("font-size: 14px; padding: 5px;"))
        self.leg_length_input.valueChanged.connect(self.on_polar_changed)
        kin_layout.addWidget(self.leg_length_input, 1, 1)
        
        self.leg_length_slider = QSlider(Qt.Horizontal)
        self.leg_length_slider.setRange(int(self.leg_length_min * 1000), int(self.leg_length_max * 1000))
        self.leg_length_slider.setValue(150)
        self.leg_length_slider.valueChanged.connect(lambda v: self.leg_length_input.setValue(v / 1000.0))
        self.leg_length_input.valueChanged.connect(lambda v: self.leg_length_slider.setValue(int(v * 1000)))
        kin_layout.addWidget(self.leg_length_slider, 1, 2, 1, 2)
        
        # 身体夹角设置
        self.polar_label_alpha = QLabel("身体夹角 (°):")
        kin_layout.addWidget(self.polar_label_alpha, 2, 0)
        self.body_angle_input = QDoubleSpinBox()
        self.body_angle_input.setRange(self.LEG_BODY_ANGLE_MIN, self.LEG_BODY_ANGLE_MAX)
        self.body_angle_input.setValue(-90)
        self.body_angle_input.setDecimals(1)
        self.body_angle_input.setSingleStep(1)
        self.body_angle_input.setStyleSheet(SS("font-size: 14px; padding: 5px;"))
        self.body_angle_input.valueChanged.connect(self.on_polar_changed)
        kin_layout.addWidget(self.body_angle_input, 2, 1)
        
        self.body_angle_slider = QSlider(Qt.Horizontal)
        self.body_angle_slider.setRange(int(self.LEG_BODY_ANGLE_MIN), int(self.LEG_BODY_ANGLE_MAX))
        self.body_angle_slider.setValue(-90)
        self.body_angle_slider.valueChanged.connect(lambda v: self.body_angle_input.setValue(v))
        self.body_angle_input.valueChanged.connect(lambda v: self.body_angle_slider.setValue(int(v)))
        kin_layout.addWidget(self.body_angle_slider, 2, 2, 1, 2)
        
        # --- 笛卡尔控件 (初始隐藏) ---
        self.cart_label_x = QLabel("x 水平 (m):")
        kin_layout.addWidget(self.cart_label_x, 3, 0)
        self.cart_x_input = QDoubleSpinBox()
        self.cart_x_input.setRange(-0.17, 0.17)
        self.cart_x_input.setValue(0.0)
        self.cart_x_input.setDecimals(4)
        self.cart_x_input.setSingleStep(0.005)
        self.cart_x_input.setStyleSheet(SS("font-size: 14px; padding: 5px;"))
        self.cart_x_input.valueChanged.connect(self.on_cart_changed)
        kin_layout.addWidget(self.cart_x_input, 3, 1)
        
        self.cart_x_slider = QSlider(Qt.Horizontal)
        self.cart_x_slider.setRange(-170, 170)
        self.cart_x_slider.setValue(0)
        self.cart_x_slider.valueChanged.connect(lambda v: self.cart_x_input.setValue(v / 1000.0))
        self.cart_x_input.valueChanged.connect(lambda v: self.cart_x_slider.setValue(int(v * 1000)))
        kin_layout.addWidget(self.cart_x_slider, 3, 2, 1, 2)
        
        self.cart_label_y = QLabel("y 垂直 (m):")
        kin_layout.addWidget(self.cart_label_y, 4, 0)
        self.cart_y_input = QDoubleSpinBox()
        self.cart_y_input.setRange(-0.17, 0.0)
        self.cart_y_input.setValue(-0.15)
        self.cart_y_input.setDecimals(4)
        self.cart_y_input.setSingleStep(0.005)
        self.cart_y_input.setStyleSheet(SS("font-size: 14px; padding: 5px;"))
        self.cart_y_input.valueChanged.connect(self.on_cart_changed)
        kin_layout.addWidget(self.cart_y_input, 4, 1)
        
        self.cart_y_slider = QSlider(Qt.Horizontal)
        self.cart_y_slider.setRange(-170, 0)
        self.cart_y_slider.setValue(-150)
        self.cart_y_slider.valueChanged.connect(lambda v: self.cart_y_input.setValue(v / 1000.0))
        self.cart_y_input.valueChanged.connect(lambda v: self.cart_y_slider.setValue(int(v * 1000)))
        kin_layout.addWidget(self.cart_y_slider, 4, 2, 1, 2)
        
        # 笛卡尔控件初始隐藏
        for w in [self.cart_label_x, self.cart_x_input, self.cart_x_slider,
                  self.cart_label_y, self.cart_y_input, self.cart_y_slider]:
            w.setVisible(False)
        
        # 目标选择
        kin_layout.addWidget(QLabel("目标:"), 5, 0)
        self.target_combo = QComboBox()
        self.target_combo.addItems(["双腿 (both)", "仅左腿 (left)", "仅右腿 (right)"])
        kin_layout.addWidget(self.target_combo, 5, 1)
        
        # 发送按钮
        self.send_kin_btn = QPushButton("📤 发送运动学目标")
        self.send_kin_btn.setStyleSheet(SS("background-color: #2196F3; color: white; padding: 10px; font-size: 14px;"))
        self.send_kin_btn.clicked.connect(self.send_kinematics_target)
        kin_layout.addWidget(self.send_kin_btn, 5, 2, 1, 2)
        
        # IK 预览显示 - 电机角度
        kin_layout.addWidget(QLabel("IK预览 (电机):"), 6, 0)
        self.ik_preview_label = QLabel("Hip: --, Knee: --")
        self.ik_preview_label.setStyleSheet(SS("font-size: 12px; color: #888; padding: 5px;"))
        kin_layout.addWidget(self.ik_preview_label, 6, 1, 1, 3)
        
        # IK 预览显示 - 运动学角度 theta1/theta2
        kin_layout.addWidget(QLabel("IK预览 (θ1,θ2):"), 7, 0)
        self.ik_theta_label = QLabel("θ1: --, θ2: --")
        self.ik_theta_label.setStyleSheet(SS("font-size: 12px; color: #00ccff; padding: 5px; font-weight: bold;"))
        kin_layout.addWidget(self.ik_theta_label, 7, 1, 1, 3)
        
        # 坐标换算显示
        kin_layout.addWidget(QLabel("坐标换算:"), 8, 0)
        self.coord_convert_label = QLabel("L=0.150m, α=-90.0° ↔ x=0.0000m, y=-0.1500m")
        self.coord_convert_label.setStyleSheet(SS("font-size: 12px; color: #aaffaa; padding: 5px;"))
        kin_layout.addWidget(self.coord_convert_label, 8, 1, 1, 3)
        
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
        self.left_hip_input.setStyleSheet(SS("font-size: 14px; padding: 5px;"))
        angle_layout.addWidget(self.left_hip_input, 0, 1)
        
        angle_layout.addWidget(QLabel("左膝 (L_Knee):"), 1, 0)
        self.left_knee_input = QDoubleSpinBox()
        self.left_knee_input.setRange(-180, 180)
        self.left_knee_input.setValue(60)  # 默认站立
        self.left_knee_input.setDecimals(1)
        self.left_knee_input.setSingleStep(5)
        self.left_knee_input.setStyleSheet(SS("font-size: 14px; padding: 5px;"))
        angle_layout.addWidget(self.left_knee_input, 1, 1)
        
        # 右腿
        angle_layout.addWidget(QLabel("右髋 (R_Hip):"), 0, 2)
        self.right_hip_input = QDoubleSpinBox()
        self.right_hip_input.setRange(-180, 180)
        self.right_hip_input.setValue(105)  # 右腿镜像
        self.right_hip_input.setDecimals(1)
        self.right_hip_input.setSingleStep(5)
        self.right_hip_input.setStyleSheet(SS("font-size: 14px; padding: 5px;"))
        angle_layout.addWidget(self.right_hip_input, 0, 3)
        
        angle_layout.addWidget(QLabel("右膝 (R_Knee):"), 1, 2)
        self.right_knee_input = QDoubleSpinBox()
        self.right_knee_input.setRange(-180, 180)
        self.right_knee_input.setValue(-60)  # 右腿镜像
        self.right_knee_input.setDecimals(1)
        self.right_knee_input.setSingleStep(5)
        self.right_knee_input.setStyleSheet(SS("font-size: 14px; padding: 5px;"))
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
        self.send_angle_btn.setStyleSheet(SS("background-color: #FF9800; color: white; padding: 10px; font-size: 14px;"))
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
        self.leg_speed_input.setStyleSheet(SS("font-size: 14px; padding: 5px;"))
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
            btn.setStyleSheet(SS("padding: 8px;"))
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
        self.test_result_label.setStyleSheet(SS("font-size: 12px; color: #00ff00; background-color: #1a1a2e; padding: 8px;"))
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
        
        state_layout.addWidget(QLabel("x 水平 (m):"), 7, 0)
        self.left_x_label = self.create_state_label("#aaffaa")
        state_layout.addWidget(self.left_x_label, 7, 1)
        self.right_x_label = self.create_state_label("#aaffaa")
        state_layout.addWidget(self.right_x_label, 7, 2)
        
        state_layout.addWidget(QLabel("y 垂直 (m):"), 8, 0)
        self.left_y_label = self.create_state_label("#aaffaa")
        state_layout.addWidget(self.left_y_label, 8, 1)
        self.right_y_label = self.create_state_label("#aaffaa")
        state_layout.addWidget(self.right_y_label, 8, 2)
        
        state_group.setLayout(state_layout)
        layout.addWidget(state_group)
        
        layout.addStretch()
        
        # 初始化 IK 预览
        self.update_ik_preview()
    
    def create_state_label(self, color="#00ccff"):
        """创建状态显示标签"""
        label = QLabel("--")
        label.setStyleSheet(SS(f"font-size: 14px; font-weight: bold; color: {color}; "
                          "background-color: #1a1a2e; padding: 5px; border-radius: 3px;"))
        label.setAlignment(Qt.AlignCenter)
        label.setMinimumWidth(sp(80))
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
    
    def on_coord_mode_changed(self, checked):
        """坐标系模式切换"""
        polar_mode = self.coord_polar_radio.isChecked()
        # 切换极坐标控件显示
        for w in [self.polar_label_L, self.leg_length_input, self.leg_length_slider,
                  self.polar_label_alpha, self.body_angle_input, self.body_angle_slider]:
            w.setVisible(polar_mode)
        # 切换笛卡尔控件显示
        for w in [self.cart_label_x, self.cart_x_input, self.cart_x_slider,
                  self.cart_label_y, self.cart_y_input, self.cart_y_slider]:
            w.setVisible(not polar_mode)
        # 切换时同步一次
        if polar_mode:
            self.on_cart_changed()  # cart → polar 一次性同步
        else:
            self.on_polar_changed()  # polar → cart 一次性同步

    def on_polar_changed(self):
        """极坐标值改变 → 同步笛卡尔 + 更新预览"""
        import math
        if self._updating_coord:
            return
        self._updating_coord = True
        try:
            L = self.leg_length_input.value()
            alpha_deg = self.body_angle_input.value()
            alpha_rad = math.radians(alpha_deg)
            x = L * math.cos(alpha_rad)
            y = L * math.sin(alpha_rad)
            self.cart_x_input.setValue(x)
            self.cart_y_input.setValue(y)
            self.coord_convert_label.setText(
                f"L={L:.3f}m, α={alpha_deg:.1f}° → x={x:.4f}m, y={y:.4f}m")
        finally:
            self._updating_coord = False
        self.update_ik_preview()

    def on_cart_changed(self):
        """笛卡尔值改变 → 同步极坐标 + 更新预览"""
        import math
        if self._updating_coord:
            return
        self._updating_coord = True
        try:
            x = self.cart_x_input.value()
            y = self.cart_y_input.value()
            L = math.sqrt(x * x + y * y)
            if L < 1e-6:
                L = 0.001
            alpha_rad = math.atan2(y, x)
            alpha_deg = math.degrees(alpha_rad)
            # 限制到极坐标范围
            L = max(self.leg_length_min, min(self.leg_length_max, L))
            alpha_deg = max(self.LEG_BODY_ANGLE_MIN, min(self.LEG_BODY_ANGLE_MAX, alpha_deg))
            self.leg_length_input.setValue(L)
            self.body_angle_input.setValue(alpha_deg)
            self.coord_convert_label.setText(
                f"x={x:.4f}m, y={y:.4f}m → L={L:.3f}m, α={alpha_deg:.1f}°")
        finally:
            self._updating_coord = False
        self.update_ik_preview()

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
            self.ik_preview_label.setStyleSheet(SS("font-size: 12px; color: #ff4444; padding: 5px;"))
            self.ik_theta_label.setText("θ1: --, θ2: --")
            return
        
        # ===== 与 C 代码一致的 IK 算法 =====
        # 左腿 offset (校准: Hip=-60°, Knee=-55° → body_angle=-90°, theta2=-90°)
        hip_offset = -20.0
        knee_offset = 55.0
        
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
        self.ik_preview_label.setStyleSheet(SS("font-size: 12px; color: #888; padding: 5px;"))
        
        # 显示 theta1/theta2 (运动学角度)
        self.ik_theta_label.setText(f"θ1={theta1_deg:.1f}° (大腿), θ2={theta2_deg:.1f}° (膝关节, 0=伸直, 负=弯曲)")
        self.ik_theta_label.setStyleSheet(SS("font-size: 12px; color: #00ccff; padding: 5px; font-weight: bold;"))
    
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
                self.leg_status.setStyleSheet(SS("font-size: 14px; font-weight: bold; color: #4CAF50;"))
            elif state == "off":
                self.leg_status.setText("状态: 已禁用")
                self.leg_status.setStyleSheet(SS("font-size: 14px; font-weight: bold; color: #f44336;"))
    
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
        # 左腿 offset (校准: Hip=-65°, Knee=-35° → body_angle=-90°)
        left_hip_offset = -20.0
        left_knee_offset = 55.0
        # 右腿 offset (镜像)
        right_hip_offset = 20.0
        right_knee_offset = -55.0
        
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
            
            # 计算笛卡尔坐标 x = L*cos(α), y = L*sin(α)
            import math
            L = left_state.get('length', 0)
            alpha_deg = left_state.get('angle', 0)
            alpha_rad = math.radians(alpha_deg)
            self.left_x_label.setText(f"{L * math.cos(alpha_rad):.4f}")
            self.left_y_label.setText(f"{L * math.sin(alpha_rad):.4f}")
        
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
            
            # 计算笛卡尔坐标 x = L*cos(α), y = L*sin(α)
            import math
            L = right_state.get('length', 0)
            alpha_deg = right_state.get('angle', 0)
            alpha_rad = math.radians(alpha_deg)
            self.right_x_label.setText(f"{L * math.cos(alpha_rad):.4f}")
            self.right_y_label.setText(f"{L * math.sin(alpha_rad):.4f}")
    
    def update_test_result(self, result_text):
        """更新测试结果显示"""
        self.test_result_label.setText(result_text)
        if "failed" in result_text.lower() or "error" in result_text.lower():
            self.test_result_label.setStyleSheet(SS("font-size: 12px; color: #ff4444; background-color: #1a1a2e; padding: 8px;"))
        else:
            self.test_result_label.setStyleSheet(SS("font-size: 12px; color: #00ff00; background-color: #1a1a2e; padding: 8px;"))


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
        self.stop_all_btn.setStyleSheet(SS("background-color: #ff4444; color: white; font-weight: bold;"))
        self.stop_all_btn.clicked.connect(self._stop_all_motors)
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
        self.torque_input.setSingleStep(0.01)
        self.torque_input.setDecimals(3)
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
            label.setStyleSheet(SS("font-weight: bold;"))
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
    
    def _stop_all_motors(self):
        """先禁用平衡控制，再停止所有电机，确保控制循环不会覆盖停止命令"""
        self.send_cmd("balance disable")
        # 短延时后发送停止命令，确保 disable 先被处理
        from PyQt5.QtCore import QTimer
        QTimer.singleShot(50, lambda: self.send_cmd("stop all"))
    
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
            self.status_labels[motor_id]['status'].setStyleSheet(SS(f"color: {color}; font-weight: bold;"))


# ============================================================================
# STW 电机配置面板 (仅 STW 品牌电机适用)
# ============================================================================
class STWConfigPanel(QWidget):
    """STW 电机驱动器内部参数配置面板"""

    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)

        # ---- 电机选择 ----
        sel_layout = QHBoxLayout()
        sel_layout.addWidget(QLabel("电机:"))
        self.motor_combo = QComboBox()
        self.motor_combo.addItems([
            "1 - 左髋", "2 - 左膝", "3 - 左轮",
            "4 - 右髋", "5 - 右膝", "6 - 右轮"
        ])
        sel_layout.addWidget(self.motor_combo)
        self.info_btn = QPushButton("📋 读取电机信息")
        self.info_btn.clicked.connect(self._read_info)
        sel_layout.addWidget(self.info_btn)
        layout.addLayout(sel_layout)

        # 电机信息显示
        self.info_label = QLabel("极对数: --  Kt: --  减速比: --")
        self.info_label.setStyleSheet(SS("font-weight: bold; padding: 4px;"))
        layout.addWidget(self.info_label)

        # ---- 参数限制 (0xB2-0xB5) ----
        limit_group = QGroupBox("参数限制")
        limit_layout = QGridLayout()

        self.limit_inputs = {}
        limits = [
            ("maxspeed", "最大速度 (RPM)", -10000, 10000, 100, 1),
            ("maxcur",   "最大电流 (A)",   0, 50, 0.5, 2),
            ("slope",    "电流斜率 (A/s)", 0, 10000, 10, 1),
            ("accel",    "加速度 (RPM/s)", 0, 100000, 100, 1),
        ]
        for row, (key, label, lo, hi, step, dec) in enumerate(limits):
            limit_layout.addWidget(QLabel(label), row, 0)
            spin = QDoubleSpinBox()
            spin.setRange(lo, hi)
            spin.setSingleStep(step)
            spin.setDecimals(dec)
            limit_layout.addWidget(spin, row, 1)
            btn = QPushButton("设置")
            btn.clicked.connect(lambda checked, k=key: self._set_limit(k))
            limit_layout.addWidget(btn, row, 2)
            self.limit_inputs[key] = spin

        limit_group.setLayout(limit_layout)
        layout.addWidget(limit_group)

        # ---- 驱动器 PID (0xB6-0xB9) ----
        pid_group = QGroupBox("驱动器内部 PID")
        pid_layout = QGridLayout()

        self.pid_inputs = {}
        pid_params = [
            ("pos_kp", "位置 Kp"),
            ("pos_ki", "位置 Ki"),
            ("spd_kp", "速度 Kp"),
            ("spd_ki", "速度 Ki"),
        ]
        for row, (key, label) in enumerate(pid_params):
            pid_layout.addWidget(QLabel(label), row, 0)
            spin = QDoubleSpinBox()
            spin.setRange(0, 99999)
            spin.setSingleStep(0.1)
            spin.setDecimals(6)
            pid_layout.addWidget(spin, row, 1)
            w_btn = QPushButton("写入")
            w_btn.clicked.connect(lambda checked, k=key: self._write_pid(k))
            pid_layout.addWidget(w_btn, row, 2)
            self.pid_inputs[key] = spin

        read_all_btn = QPushButton("📖 读取全部 PID")
        read_all_btn.clicked.connect(self._read_all_pid)
        pid_layout.addWidget(read_all_btn, len(pid_params), 0, 1, 3)

        pid_group.setLayout(pid_layout)
        layout.addWidget(pid_group)

        # ---- MIT 运控模式 ----
        mit_group = QGroupBox("MIT 阻抗控制")
        mit_layout = QGridLayout()

        # MIT 配置
        mit_layout.addWidget(QLabel("pos_max (rad):"), 0, 0)
        self.mit_pmax = QDoubleSpinBox(); self.mit_pmax.setRange(0, 100); self.mit_pmax.setDecimals(2); self.mit_pmax.setValue(12.57)
        mit_layout.addWidget(self.mit_pmax, 0, 1)
        mit_layout.addWidget(QLabel("vel_max (rad/s):"), 0, 2)
        self.mit_vmax = QDoubleSpinBox(); self.mit_vmax.setRange(0, 500); self.mit_vmax.setDecimals(2); self.mit_vmax.setValue(45.0)
        mit_layout.addWidget(self.mit_vmax, 0, 3)
        mit_layout.addWidget(QLabel("t_max (Nm):"), 0, 4)
        self.mit_tmax = QDoubleSpinBox(); self.mit_tmax.setRange(0, 100); self.mit_tmax.setDecimals(2); self.mit_tmax.setValue(18.0)
        mit_layout.addWidget(self.mit_tmax, 0, 5)
        self.mit_cfg_btn = QPushButton("设置 MIT 配置")
        self.mit_cfg_btn.clicked.connect(self._set_mit_config)
        mit_layout.addWidget(self.mit_cfg_btn, 0, 6)

        # MIT 控制
        mit_ctrl_labels = [("目标位置 (rad):", "mit_pos", -50, 50, 0.1, 3),
                           ("目标速度 (rad/s):", "mit_vel", -100, 100, 1, 2),
                           ("Kp (0~500):", "mit_kp", 0, 500, 1, 1),
                           ("Kd (0~5):", "mit_kd", 0, 5, 0.01, 3),
                           ("前馈力矩 (Nm):", "mit_torque", -20, 20, 0.01, 3)]
        self.mit_ctrl_inputs = {}
        for col, (label, key, lo, hi, step, dec) in enumerate(mit_ctrl_labels):
            mit_layout.addWidget(QLabel(label), 1, col)
            spin = QDoubleSpinBox(); spin.setRange(lo, hi); spin.setSingleStep(step); spin.setDecimals(dec)
            mit_layout.addWidget(spin, 2, col)
            self.mit_ctrl_inputs[key] = spin

        self.mit_ctrl_btn = QPushButton("▶ 发送 MIT 控制")
        self.mit_ctrl_btn.setStyleSheet(SS("background-color: #ff8800; color: white; font-weight: bold;"))
        self.mit_ctrl_btn.clicked.connect(self._send_mit_ctrl)
        mit_layout.addWidget(self.mit_ctrl_btn, 2, 5, 1, 2)

        self.mit_state_btn = QPushButton("📊 读取 MIT 状态")
        self.mit_state_btn.clicked.connect(self._read_mit_state)
        mit_layout.addWidget(self.mit_state_btn, 3, 0, 1, 2)

        self.mit_state_label = QLabel("MIT 状态: --")
        mit_layout.addWidget(self.mit_state_label, 3, 2, 1, 5)

        mit_group.setLayout(mit_layout)
        layout.addWidget(mit_group)
        layout.addStretch()

    # ------- helpers -------
    def _mid(self):
        return self.motor_combo.currentIndex() + 1

    def _send(self, cmd):
        if self.parent_window and self.parent_window.is_connected():
            self.parent_window.send_command(cmd)

    def _read_info(self):
        self._send(f"stw info {self._mid()}")

    def _set_limit(self, key):
        val = self.limit_inputs[key].value()
        self._send(f"stw {key} {self._mid()} {val}")

    def _write_pid(self, key):
        val = self.pid_inputs[key].value()
        self._send(f"stw pid {self._mid()} {key} {val}")

    def _read_all_pid(self):
        self._send(f"stw pid {self._mid()} read")

    def _set_mit_config(self):
        self._send(f"stw mit config {self._mid()} {self.mit_pmax.value()} {self.mit_vmax.value()} {self.mit_tmax.value()}")

    def _send_mit_ctrl(self):
        m = self.mit_ctrl_inputs
        self._send(f"stw mit ctrl {self._mid()} {m['mit_pos'].value()} {m['mit_vel'].value()} "
                    f"{m['mit_kp'].value()} {m['mit_kd'].value()} {m['mit_torque'].value()}")

    def _read_mit_state(self):
        self._send(f"stw mit state {self._mid()}")


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
        self.init_btn.setStyleSheet(SS("font-size: 14px; padding: 10px;"))
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
        self.start_btn.setStyleSheet(SS("background-color: #44aa44; color: white;"))
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
        label.setStyleSheet(SS("font-size: 16px; font-weight: bold; color: #00ccff; "
                          "background-color: #1a1a2e; padding: 8px; border-radius: 4px;"))
        label.setAlignment(Qt.AlignCenter)
        label.setMinimumWidth(sp(100))
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
        self.init_btn.setStyleSheet(SS("font-size: 16px; padding: 15px; background-color: #4488ff;"))
        self.init_btn.clicked.connect(self.do_balance_init)
        btn_layout.addWidget(self.init_btn)
        
        self.init_status = QLabel("未初始化")
        self.init_status.setStyleSheet(SS("font-size: 14px; color: gray;"))
        btn_layout.addWidget(self.init_status)
        
        init_layout.addLayout(btn_layout)
        
        note_label = QLabel("⚠️ 注意: 必须先初始化才能使用平衡控制和 PID 调参功能")
        note_label.setStyleSheet(SS("color: orange;"))
        init_layout.addWidget(note_label)
        
        init_group.setLayout(init_layout)
        layout.addWidget(init_group)
        
        # 运行控制
        run_group = QGroupBox("运行控制")
        run_layout = QHBoxLayout()
        
        self.start_btn = QPushButton("▶ 启动 (balance start)")
        self.start_btn.setStyleSheet(SS("font-size: 14px; padding: 10px; background-color: #44aa44;"))
        self.start_btn.clicked.connect(lambda: self.send_cmd("balance start"))
        run_layout.addWidget(self.start_btn)
        
        self.stop_btn = QPushButton("⏹ 停止 (balance stop)")
        self.stop_btn.setStyleSheet(SS("font-size: 14px; padding: 10px;"))
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
        self.enable_btn.setStyleSheet(SS("font-size: 14px; padding: 10px; background-color: #44aa44;"))
        self.enable_btn.clicked.connect(lambda: self.send_cmd("balance enable"))
        enable_layout.addWidget(self.enable_btn)
        
        self.disable_btn = QPushButton("❌ 禁用平衡")
        self.disable_btn.clicked.connect(lambda: self.send_cmd("balance disable"))
        enable_layout.addWidget(self.disable_btn)
        
        self.estop_btn = QPushButton("🛑 紧急停止")
        self.estop_btn.setStyleSheet(SS("font-size: 14px; padding: 10px; background-color: #ff4444; color: white; font-weight: bold;"))
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
        self.roll_status.setStyleSheet(SS("font-size: 14px; font-weight: bold;"))
        roll_layout.addWidget(self.roll_status)
        
        roll_layout.addStretch()
        
        self.roll_on_btn = QPushButton("✅ 开启 Roll")
        self.roll_on_btn.setStyleSheet(SS("background-color: #4CAF50; color: white; padding: 8px 20px;"))
        self.roll_on_btn.clicked.connect(lambda: self.send_roll_cmd("on"))
        roll_layout.addWidget(self.roll_on_btn)
        
        self.roll_off_btn = QPushButton("❌ 关闭 Roll")
        self.roll_off_btn.setStyleSheet(SS("background-color: #f44336; color: white; padding: 8px 20px;"))
        self.roll_off_btn.clicked.connect(lambda: self.send_roll_cmd("off"))
        roll_layout.addWidget(self.roll_off_btn)
        
        self.roll_status_btn = QPushButton("📊 状态")
        self.roll_status_btn.clicked.connect(lambda: self.send_cmd("balance roll"))
        roll_layout.addWidget(self.roll_status_btn)
        
        roll_group.setLayout(roll_layout)
        layout.addWidget(roll_group)
        
        # Pitch 腿部角度补偿
        pitch_comp_group = QGroupBox("Pitch 腿部角度补偿 (俯仰补偿)")
        pitch_comp_layout = QHBoxLayout()
        
        self.pitch_comp_status = QLabel("状态: 未知")
        self.pitch_comp_status.setStyleSheet(SS("font-size: 14px; font-weight: bold;"))
        pitch_comp_layout.addWidget(self.pitch_comp_status)
        
        pitch_comp_layout.addStretch()
        
        self.pitch_comp_on_btn = QPushButton("✅ 开启 PitchComp")
        self.pitch_comp_on_btn.setStyleSheet(SS("background-color: #4CAF50; color: white; padding: 8px 20px;"))
        self.pitch_comp_on_btn.clicked.connect(lambda: self.send_pitch_comp_cmd("on"))
        pitch_comp_layout.addWidget(self.pitch_comp_on_btn)
        
        self.pitch_comp_off_btn = QPushButton("❌ 关闭 PitchComp")
        self.pitch_comp_off_btn.setStyleSheet(SS("background-color: #f44336; color: white; padding: 8px 20px;"))
        self.pitch_comp_off_btn.clicked.connect(lambda: self.send_pitch_comp_cmd("off"))
        pitch_comp_layout.addWidget(self.pitch_comp_off_btn)
        
        pitch_comp_group.setLayout(pitch_comp_layout)
        layout.addWidget(pitch_comp_group)
        
        # X-Offset 腿部速度自适应偏移
        xoffset_group = QGroupBox("🦶 X-Offset 腿部速度自适应偏移")
        xoffset_layout = QVBoxLayout()
        
        # 第一行: 状态 + 开关
        xoff_ctrl_layout = QHBoxLayout()
        
        self.xoffset_status = QLabel("状态: 未知")
        self.xoffset_status.setStyleSheet(SS("font-size: 14px; font-weight: bold;"))
        xoff_ctrl_layout.addWidget(self.xoffset_status)
        
        xoff_ctrl_layout.addStretch()
        
        self.xoffset_on_btn = QPushButton("✅ 开启 XOffset")
        self.xoffset_on_btn.setStyleSheet(SS("background-color: #4CAF50; color: white; padding: 8px 20px;"))
        self.xoffset_on_btn.clicked.connect(lambda: self.send_xoffset_cmd("on"))
        xoff_ctrl_layout.addWidget(self.xoffset_on_btn)
        
        self.xoffset_off_btn = QPushButton("❌ 关闭 XOffset")
        self.xoffset_off_btn.setStyleSheet(SS("background-color: #f44336; color: white; padding: 8px 20px;"))
        self.xoffset_off_btn.clicked.connect(lambda: self.send_xoffset_cmd("off"))
        xoff_ctrl_layout.addWidget(self.xoffset_off_btn)
        
        self.xoffset_query_btn = QPushButton("📊 状态")
        self.xoffset_query_btn.clicked.connect(lambda: self.send_cmd("balance xoffset"))
        xoff_ctrl_layout.addWidget(self.xoffset_query_btn)
        
        xoffset_layout.addLayout(xoff_ctrl_layout)
        
        # 第二行: PID 参数
        xoff_pid_layout = QGridLayout()
        xoff_pid_layout.setSpacing(6)
        
        xoff_pid_layout.addWidget(QLabel("Kp:"), 0, 0)
        self.xoffset_kp = QDoubleSpinBox()
        self.xoffset_kp.setRange(0, 1.0)
        self.xoffset_kp.setSingleStep(0.001)
        self.xoffset_kp.setDecimals(4)
        self.xoffset_kp.setValue(0.01)
        xoff_pid_layout.addWidget(self.xoffset_kp, 0, 1)
        
        xoff_pid_layout.addWidget(QLabel("Ki:"), 0, 2)
        self.xoffset_ki = QDoubleSpinBox()
        self.xoffset_ki.setRange(0, 1.0)
        self.xoffset_ki.setSingleStep(0.001)
        self.xoffset_ki.setDecimals(4)
        self.xoffset_ki.setValue(0.0)
        xoff_pid_layout.addWidget(self.xoffset_ki, 0, 3)
        
        xoff_pid_layout.addWidget(QLabel("Kd:"), 0, 4)
        self.xoffset_kd = QDoubleSpinBox()
        self.xoffset_kd.setRange(0, 1.0)
        self.xoffset_kd.setSingleStep(0.001)
        self.xoffset_kd.setDecimals(4)
        self.xoffset_kd.setValue(0.0)
        xoff_pid_layout.addWidget(self.xoffset_kd, 0, 5)
        
        xoff_pid_layout.addWidget(QLabel("Limit(m):"), 0, 6)
        self.xoffset_limit = QDoubleSpinBox()
        self.xoffset_limit.setRange(0.001, 0.08)
        self.xoffset_limit.setSingleStep(0.005)
        self.xoffset_limit.setDecimals(3)
        self.xoffset_limit.setValue(0.03)
        xoff_pid_layout.addWidget(self.xoffset_limit, 0, 7)
        
        self.xoffset_apply_btn = QPushButton("📤 应用参数")
        self.xoffset_apply_btn.setStyleSheet(SS("background-color: #2196F3; color: white; padding: 6px 16px;"))
        self.xoffset_apply_btn.clicked.connect(self.apply_xoffset_params)
        xoff_pid_layout.addWidget(self.xoffset_apply_btn, 0, 8)
        
        xoffset_layout.addLayout(xoff_pid_layout)
        
        # 第三行: 实时值显示
        xoff_val_layout = QHBoxLayout()
        self.xoffset_val_label = QLabel("x_offset: --- m | speed: --- m/s")
        self.xoffset_val_label.setStyleSheet(SS("font-size: 12px; color: #aaa; background: #1a1a2e; padding: 4px; border-radius: 3px;"))
        xoff_val_layout.addWidget(self.xoffset_val_label)
        xoffset_layout.addLayout(xoff_val_layout)
        
        xoffset_group.setLayout(xoffset_layout)
        layout.addWidget(xoffset_group)
        
        # Leg Sync 防劈叉 (左右腿同步补偿)
        sync_group = QGroupBox("🔗 Leg Sync 防劈叉 (左右腿同步)")
        sync_layout = QVBoxLayout()
        
        # 第一行: 状态 + 开关
        sync_ctrl_layout = QHBoxLayout()
        
        self.leg_sync_status = QLabel("状态: 未知")
        self.leg_sync_status.setStyleSheet(SS("font-size: 14px; font-weight: bold;"))
        sync_ctrl_layout.addWidget(self.leg_sync_status)
        
        sync_ctrl_layout.addStretch()
        
        self.leg_sync_on_btn = QPushButton("✅ 开启 Sync")
        self.leg_sync_on_btn.setStyleSheet(SS("background-color: #4CAF50; color: white; padding: 8px 20px;"))
        self.leg_sync_on_btn.clicked.connect(lambda: self.send_leg_sync_cmd("on"))
        sync_ctrl_layout.addWidget(self.leg_sync_on_btn)
        
        self.leg_sync_off_btn = QPushButton("❌ 关闭 Sync")
        self.leg_sync_off_btn.setStyleSheet(SS("background-color: #f44336; color: white; padding: 8px 20px;"))
        self.leg_sync_off_btn.clicked.connect(lambda: self.send_leg_sync_cmd("off"))
        sync_ctrl_layout.addWidget(self.leg_sync_off_btn)
        
        self.leg_sync_query_btn = QPushButton("📊 状态")
        self.leg_sync_query_btn.clicked.connect(lambda: self.send_cmd("balance sync"))
        sync_ctrl_layout.addWidget(self.leg_sync_query_btn)
        
        sync_layout.addLayout(sync_ctrl_layout)
        
        # 第二行: 参数设置
        sync_param_layout = QGridLayout()
        sync_param_layout.setSpacing(6)
        
        sync_param_layout.addWidget(QLabel("增益 (0~1):"), 0, 0)
        self.leg_sync_gain = QDoubleSpinBox()
        self.leg_sync_gain.setRange(0.0, 1.0)
        self.leg_sync_gain.setSingleStep(0.05)
        self.leg_sync_gain.setDecimals(2)
        self.leg_sync_gain.setValue(0.30)
        sync_param_layout.addWidget(self.leg_sync_gain, 0, 1)
        
        sync_param_layout.addWidget(QLabel("最大修正 (°):"), 0, 2)
        self.leg_sync_max = QDoubleSpinBox()
        self.leg_sync_max.setRange(1.0, 45.0)
        self.leg_sync_max.setSingleStep(1.0)
        self.leg_sync_max.setDecimals(1)
        self.leg_sync_max.setValue(15.0)
        sync_param_layout.addWidget(self.leg_sync_max, 0, 3)
        
        self.leg_sync_apply_btn = QPushButton("📤 应用参数")
        self.leg_sync_apply_btn.setStyleSheet(SS("background-color: #2196F3; color: white; padding: 6px 16px;"))
        self.leg_sync_apply_btn.clicked.connect(self.apply_leg_sync_params)
        sync_param_layout.addWidget(self.leg_sync_apply_btn, 0, 4)
        
        sync_layout.addLayout(sync_param_layout)
        
        # 第三行: 实时值显示
        sync_val_layout = QHBoxLayout()
        self.leg_sync_val_label = QLabel("diff: --- ° | correction: --- °")
        self.leg_sync_val_label.setStyleSheet(SS("font-size: 12px; color: #aaa; background: #1a1a2e; padding: 4px; border-radius: 3px;"))
        sync_val_layout.addWidget(self.leg_sync_val_label)
        sync_layout.addLayout(sync_val_layout)
        
        sync_group.setLayout(sync_layout)
        layout.addWidget(sync_group)
        
        # 角度零点设置 & 自适应调试
        zero_group = QGroupBox("🎯 角度零点设置 & 自适应")
        zero_main_layout = QVBoxLayout()
        
        # 第一行: 滑条调节零点 + 微调箭头
        zero_slider_row = QHBoxLayout()
        zero_slider_row.addWidget(QLabel("零点 (°):"))
        
        self.zero_dec_btn = QPushButton("◀")
        self.zero_dec_btn.setFixedWidth(30)
        self.zero_dec_btn.setToolTip("减少 0.1°")
        self.zero_dec_btn.clicked.connect(lambda: self._nudge_zero(-0.1))
        zero_slider_row.addWidget(self.zero_dec_btn)
        
        self.zero_slider = QSlider(Qt.Horizontal)
        self.zero_slider.setRange(-3000, 3000)  # -30.00° ~ +30.00°, 单位 0.01°
        self.zero_slider.setValue(740)           # 默认 7.40°
        self.zero_slider.setTickPosition(QSlider.TicksBelow)
        self.zero_slider.setTickInterval(500)    # 每 5° 一个刻度
        self.zero_slider.valueChanged.connect(self._on_zero_slider_changed)
        zero_slider_row.addWidget(self.zero_slider, 1)
        
        self.zero_inc_btn = QPushButton("▶")
        self.zero_inc_btn.setFixedWidth(30)
        self.zero_inc_btn.setToolTip("增加 0.1°")
        self.zero_inc_btn.clicked.connect(lambda: self._nudge_zero(+0.1))
        zero_slider_row.addWidget(self.zero_inc_btn)
        
        self.zero_value_label = QLabel("7.400°")
        self.zero_value_label.setFixedWidth(60)
        self.zero_value_label.setStyleSheet(SS("font-weight: bold; font-size: 12px;"))
        zero_slider_row.addWidget(self.zero_value_label)
        zero_main_layout.addLayout(zero_slider_row)
        
        # 第二行: 精确输入 + 设置/查询按钮
        zero_row1 = QHBoxLayout()
        zero_row1.addWidget(QLabel("精确值:"))
        self.zero_input = QDoubleSpinBox()
        self.zero_input.setRange(-30, 30)
        self.zero_input.setSingleStep(0.01)
        self.zero_input.setDecimals(3)
        self.zero_input.setValue(7.4)
        self.zero_input.valueChanged.connect(self._on_zero_spinbox_changed)
        zero_row1.addWidget(self.zero_input)
        
        self.set_zero_btn = QPushButton("设置零点")
        self.set_zero_btn.clicked.connect(self.set_zero_point)
        zero_row1.addWidget(self.set_zero_btn)
        
        self.get_zero_btn = QPushButton("查询状态")
        self.get_zero_btn.clicked.connect(lambda: self.send_cmd("balance zero"))
        zero_row1.addWidget(self.get_zero_btn)
        zero_main_layout.addLayout(zero_row1)
        
        # 第二行: 轮速阈值设置
        zero_row2 = QHBoxLayout()
        zero_row2.addWidget(QLabel("轮速阈值 (m/s):"))
        self.zp_threshold_input = QDoubleSpinBox()
        self.zp_threshold_input.setRange(0.01, 5.0)
        self.zp_threshold_input.setSingleStep(0.01)
        self.zp_threshold_input.setDecimals(3)
        self.zp_threshold_input.setValue(0.1)
        self.zp_threshold_input.setToolTip("轮速低于此值时零点自适应PID才工作")
        zero_row2.addWidget(self.zp_threshold_input)
        
        self.set_zp_thr_btn = QPushButton("设置阈值")
        self.set_zp_thr_btn.clicked.connect(self.set_zp_threshold)
        zero_row2.addWidget(self.set_zp_thr_btn)
        zero_main_layout.addLayout(zero_row2)
        
        # 第三行: 自适应状态显示
        self.zp_status_label = QLabel("零点自适应: 等待查询...")
        self.zp_status_label.setStyleSheet(SS("color: #aaa; font-size: 10px;"))
        self.zp_status_label.setWordWrap(True)
        zero_main_layout.addWidget(self.zp_status_label)
        
        # 波形提示
        zp_plot_hint = QLabel("📊 波形通道: I=pitch vs 零点 | J=PID输出(raw/filtered) | Z=角度误差+激活状态")
        zp_plot_hint.setStyleSheet(SS("color: #888; font-size: 9px;"))
        zero_main_layout.addWidget(zp_plot_hint)
        
        zero_group.setLayout(zero_main_layout)
        layout.addWidget(zero_group)
        
        # ========== 遥杆映射比例调节 ==========
        joy_group = QGroupBox("🕹️ 遥杆映射比例 (Joystick Scale)")
        joy_layout = QVBoxLayout()
        
        joy_desc = QLabel("控制遥杆值到目标速度/转向速率的映射比例\n"
                          "target_speed = joy_y × speed_scale | target_yaw = joy_x × yaw_scale\n"
                          "joy 范围 -100~100, 最大值 = 100 × scale")
        joy_desc.setStyleSheet(SS("color: #888; font-size: 10px;"))
        joy_layout.addWidget(joy_desc)
        
        joy_param_layout = QGridLayout()
        joy_param_layout.setSpacing(6)
        
        # Speed scale
        joy_param_layout.addWidget(QLabel("Speed Scale:"), 0, 0)
        self.joy_speed_scale = QDoubleSpinBox()
        self.joy_speed_scale.setRange(0.0, 1.0)
        self.joy_speed_scale.setSingleStep(0.0001)
        self.joy_speed_scale.setDecimals(6)
        self.joy_speed_scale.setValue(0.003)
        joy_param_layout.addWidget(self.joy_speed_scale, 0, 1)
        
        self.joy_speed_max_label = QLabel("max ±0.300")
        self.joy_speed_max_label.setStyleSheet(SS("color: #aaa; font-size: 10px;"))
        joy_param_layout.addWidget(self.joy_speed_max_label, 0, 2)
        self.joy_speed_scale.valueChanged.connect(
            lambda v: self.joy_speed_max_label.setText(f"max ±{100*v:.3f}"))
        
        # Yaw scale
        joy_param_layout.addWidget(QLabel("Yaw Scale:"), 1, 0)
        self.joy_yaw_scale = QDoubleSpinBox()
        self.joy_yaw_scale.setRange(0.0, 1.0)
        self.joy_yaw_scale.setSingleStep(0.001)
        self.joy_yaw_scale.setDecimals(6)
        self.joy_yaw_scale.setValue(0.03)
        joy_param_layout.addWidget(self.joy_yaw_scale, 1, 1)
        
        self.joy_yaw_max_label = QLabel("max ±3.000")
        self.joy_yaw_max_label.setStyleSheet(SS("color: #aaa; font-size: 10px;"))
        joy_param_layout.addWidget(self.joy_yaw_max_label, 1, 2)
        self.joy_yaw_scale.valueChanged.connect(
            lambda v: self.joy_yaw_max_label.setText(f"max ±{100*v:.3f}"))
        
        # 应用按钮
        self.joy_apply_btn = QPushButton("📤 应用")
        self.joy_apply_btn.setStyleSheet(SS("background-color: #2196F3; color: white; padding: 6px 16px;"))
        self.joy_apply_btn.clicked.connect(self.apply_joy_scale)
        joy_param_layout.addWidget(self.joy_apply_btn, 0, 3, 2, 1)
        
        # 查询按钮
        self.joy_query_btn = QPushButton("📊 查询")
        self.joy_query_btn.clicked.connect(lambda: self.send_cmd("balance joy"))
        joy_param_layout.addWidget(self.joy_query_btn, 0, 4)
        
        joy_layout.addLayout(joy_param_layout)
        
        joy_group.setLayout(joy_layout)
        layout.addWidget(joy_group)
        
        # 波形输出控制
        plot_group = QGroupBox("📊 波形输出控制")
        plot_main_layout = QVBoxLayout()
        
        # 第一行: 开关 + 分频
        plot_ctrl_layout = QHBoxLayout()
        
        self.plot_on_btn = QPushButton("▶ 开启波形")
        self.plot_on_btn.setStyleSheet(SS("background-color: #4CAF50; color: white; padding: 5px 12px;"))
        self.plot_on_btn.clicked.connect(self.plot_start)
        plot_ctrl_layout.addWidget(self.plot_on_btn)
        
        self.plot_off_btn = QPushButton("⏹ 关闭波形")
        self.plot_off_btn.setStyleSheet(SS("background-color: #f44336; color: white; padding: 5px 12px;"))
        self.plot_off_btn.clicked.connect(lambda: self.send_cmd("balance plot off"))
        plot_ctrl_layout.addWidget(self.plot_off_btn)
        
        plot_ctrl_layout.addWidget(QLabel("分频:"))
        self.plot_div_input = QSpinBox()
        self.plot_div_input.setRange(1, 255)
        self.plot_div_input.setValue(10)
        plot_ctrl_layout.addWidget(self.plot_div_input)
        
        self.set_div_btn = QPushButton("设置")
        self.set_div_btn.clicked.connect(lambda: self.send_cmd(f"balance plot div {self.plot_div_input.value()}"))
        plot_ctrl_layout.addWidget(self.set_div_btn)
        
        plot_ctrl_layout.addStretch()
        plot_main_layout.addLayout(plot_ctrl_layout)
        
        # 第二行: 通道选择 (分组显示)
        ch_group_label = QLabel("通道选择 (勾选需要的通道，减少数据量):")
        ch_group_label.setStyleSheet(SS("color: #aaa; font-size: 11px; margin-top: 5px;"))
        plot_main_layout.addWidget(ch_group_label)
        
        # 通道定义: (ID, 名称, 默认勾选)
        self.plot_channels = {
            # 基础状态量
            'A': ('角度 Pitch', True),
            'B': ('角速度', True),
            'C': ('位移', False),
            'D': ('速度', True),
            # YAW
            'E': ('YAW角度', False),
            'F': ('YAW角速度', False),
            # 滤波器
            'G': ('摇杆LPF', False),
            'N': ('角速度滤波', False),
            'W': ('轮速滤波', False),
            # 输出
            'H': ('LQR输出', True),
            'V': ('输出PID前后', False),
            # 零点/Roll
            'I': ('零点偏移', False),
            'J': ('零点LPF', False),
            'Z': ('零点误差/激活', False),
            'K': ('Roll角度', False),
            'L': ('Roll LPF', False),
            # 速度自适应
            'M': ('速度自适应Kp', False),
            # 轮子
            'O': ('左轮速/加速', False),
            'P': ('右轮速/加速', False),
            'X': ('电机电流', False),
            # 各环输出
            'Q': ('角度环输出', False),
            'R': ('角速度环输出', False),
            'S': ('位移环输出', False),
            'T': ('速度环输出', False),
            'U': ('YAW输出', False),
            # YAW调试
            'Y': ('YAW调试', False),
        }
        
        self.plot_ch_checkboxes = {}
        
        # 按组排列
        ch_groups = [
            ("基础", ['A', 'B', 'C', 'D']),
            ("YAW", ['E', 'F', 'Y']),
            ("滤波", ['G', 'N', 'W']),
            ("输出", ['H', 'V']),
            ("零点/Roll", ['I', 'J', 'Z', 'K', 'L']),
            ("轮子", ['O', 'P', 'X']),
            ("环路分量", ['Q', 'R', 'S', 'T', 'U']),
            ("自适应", ['M']),
        ]
        
        ch_grid = QGridLayout()
        ch_grid.setSpacing(2)
        col = 0
        for group_name, ch_ids in ch_groups:
            # 组标签
            glabel = QLabel(f"[{group_name}]")
            glabel.setStyleSheet(SS("color: #888; font-size: 10px; font-weight: bold;"))
            ch_grid.addWidget(glabel, 0, col, 1, len(ch_ids))
            for i, ch_id in enumerate(ch_ids):
                name, default = self.plot_channels[ch_id]
                cb = QCheckBox(f"{ch_id}:{name}")
                cb.setChecked(default)
                cb.setStyleSheet(SS("font-size: 10px;"))
                self.plot_ch_checkboxes[ch_id] = cb
                ch_grid.addWidget(cb, 1, col + i)
            col += len(ch_ids)
        
        plot_main_layout.addLayout(ch_grid)
        
        # 第三行: 快捷按钮
        ch_quick_layout = QHBoxLayout()
        
        ch_all_btn = QPushButton("全选")
        ch_all_btn.setFixedWidth(50)
        ch_all_btn.clicked.connect(lambda: self.set_all_plot_channels(True))
        ch_quick_layout.addWidget(ch_all_btn)
        
        ch_none_btn = QPushButton("全不选")
        ch_none_btn.setFixedWidth(60)
        ch_none_btn.clicked.connect(lambda: self.set_all_plot_channels(False))
        ch_quick_layout.addWidget(ch_none_btn)
        
        ch_basic_btn = QPushButton("基础 (ABDH)")
        ch_basic_btn.clicked.connect(lambda: self.set_plot_channel_preset("ABDH"))
        ch_quick_layout.addWidget(ch_basic_btn)
        
        ch_filter_btn = QPushButton("滤波 (ABNW)")
        ch_filter_btn.clicked.connect(lambda: self.set_plot_channel_preset("ABNW"))
        ch_quick_layout.addWidget(ch_filter_btn)
        
        ch_loop_btn = QPushButton("环路分析 (QRSTUV)")
        ch_loop_btn.clicked.connect(lambda: self.set_plot_channel_preset("QRSTUV"))
        ch_quick_layout.addWidget(ch_loop_btn)
        
        ch_quick_layout.addStretch()
        
        self.ch_apply_btn = QPushButton("⬆ 应用通道设置")
        self.ch_apply_btn.setStyleSheet(SS("background-color: #2196F3; color: white; padding: 5px 15px;"))
        self.ch_apply_btn.clicked.connect(self.apply_plot_channels)
        ch_quick_layout.addWidget(self.ch_apply_btn)
        
        plot_main_layout.addLayout(ch_quick_layout)
        
        plot_group.setLayout(plot_main_layout)
        layout.addWidget(plot_group)
        
        # ========== 环路调试控制 (新增) ==========
        loop_group = QGroupBox("🔧 环路调试 (单环/组合调试)")
        loop_layout = QVBoxLayout()
        
        # 说明标签
        loop_desc = QLabel("LQR控制环: A=角度环 B=角速度环 C=位移环 D=速度环 H=总输出 | Y=Yaw转向")
        loop_desc.setStyleSheet(SS("color: #888; font-size: 11px;"))
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
        line1.setStyleSheet(SS("color: #444;"))
        loop_layout.addWidget(line1)
        
        # 各环路独立开关 - LQR 内环
        lqr_label = QLabel("LQR 平衡环 (Pitch):")
        lqr_label.setStyleSheet(SS("font-weight: bold;"))
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
        yaw_label.setStyleSheet(SS("font-weight: bold;"))
        yaw_layout.addWidget(yaw_label)
        
        cb_yaw = QCheckBox("Y:Yaw转向")
        cb_yaw.setToolTip("控制原地转向/航向保持")
        cb_yaw.setChecked(True)
        cb_yaw.stateChanged.connect(lambda state: self.on_loop_toggle("Y", state))
        self.loop_checks["Y"] = cb_yaw
        yaw_layout.addWidget(cb_yaw)
        
        self.cb_yaw_force = QCheckBox("强制使能(无遥控)")
        self.cb_yaw_force.setToolTip("无需遥控器remote.go即可启用YAW控制\n对应CLI: balance yaw on/off")
        self.cb_yaw_force.setChecked(False)
        self.cb_yaw_force.stateChanged.connect(self.on_yaw_force_toggle)
        yaw_layout.addWidget(self.cb_yaw_force)
        
        yaw_layout.addStretch()
        loop_layout.addLayout(yaw_layout)
        
        # 分隔线
        line2 = QFrame()
        line2.setFrameShape(QFrame.HLine)
        line2.setStyleSheet(SS("color: #444;"))
        loop_layout.addWidget(line2)
        
        # 增益精调 (高级)
        gain_label = QLabel("增益精调 (0.0-5.0):")
        gain_label.setStyleSheet(SS("font-size: 11px; color: #888;"))
        loop_layout.addWidget(gain_label)
        
        gain_layout = QHBoxLayout()
        self.gain_loop_combo = QComboBox()
        self.gain_loop_combo.addItems(["A:角度", "B:角速度", "C:位移", "D:速度", "H:总输出", "Y:Yaw"])
        gain_layout.addWidget(self.gain_loop_combo)
        
        self.gain_spin = QDoubleSpinBox()
        self.gain_spin.setRange(0.0, 5.0)
        self.gain_spin.setSingleStep(0.01)
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
        self.sys_status_label.setStyleSheet(SS("font-weight: bold; color: gray;"))
        status_layout.addWidget(self.sys_status_label, 0, 1)
        
        status_layout.addWidget(QLabel("Pitch 角度:"), 1, 0)
        self.pitch_label = QLabel("--")
        self.pitch_label.setStyleSheet(SS("font-size: 18px; font-weight: bold; color: #00ff00;"))
        status_layout.addWidget(self.pitch_label, 1, 1)
        
        status_layout.addWidget(QLabel("Roll 角度:"), 1, 2)
        self.roll_label = QLabel("--")
        self.roll_label.setStyleSheet(SS("font-size: 18px; font-weight: bold; color: #00ff00;"))
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
        self.freq_imu_label.setStyleSheet(SS("font-weight: bold; color: #00ccff;"))
        freq_layout.addWidget(self.freq_imu_label, 1, 2)
        self.freq_imu_status = QLabel("--")
        freq_layout.addWidget(self.freq_imu_status, 1, 3)
        
        # 平衡控制
        freq_layout.addWidget(QLabel("平衡控制:"), 2, 0)
        freq_layout.addWidget(QLabel("200 Hz"), 2, 1)
        self.freq_ctrl_label = QLabel("-- Hz")
        self.freq_ctrl_label.setStyleSheet(SS("font-weight: bold; color: #00ccff;"))
        freq_layout.addWidget(self.freq_ctrl_label, 2, 2)
        self.freq_ctrl_status = QLabel("--")
        freq_layout.addWidget(self.freq_ctrl_status, 2, 3)
        
        # 电机通信 (轮)
        freq_layout.addWidget(QLabel("轮电机通信:"), 3, 0)
        freq_layout.addWidget(QLabel("200 Hz"), 3, 1)
        self.freq_motor_label = QLabel("-- Hz")
        self.freq_motor_label.setStyleSheet(SS("font-weight: bold; color: #00ccff;"))
        freq_layout.addWidget(self.freq_motor_label, 3, 2)
        self.freq_motor_status = QLabel("--")
        freq_layout.addWidget(self.freq_motor_status, 3, 3)
        
        # 腿电机
        freq_layout.addWidget(QLabel("腿电机控制:"), 4, 0)
        freq_layout.addWidget(QLabel("20 Hz"), 4, 1)
        self.freq_leg_label = QLabel("-- Hz")
        self.freq_leg_label.setStyleSheet(SS("font-weight: bold; color: #00ccff;"))
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
        latency_group.setStyleSheet(SS("QGroupBox { font-size: 11px; }"))
        latency_layout = QGridLayout()
        latency_layout.setSpacing(2)  # 减小间距
        
        # 小字号样式
        small_label_style = SS("font-size: 10px;")
        small_value_style = SS("font-size: 10px; font-weight: bold; color: #00ccff;")
        
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
        lbl.setStyleSheet(SS("font-size: 11px; font-weight: bold;"))
        latency_layout.addWidget(lbl, 4, 0)
        self.latency_total_us = QLabel("--")
        self.latency_total_us.setStyleSheet(SS("font-size: 12px; font-weight: bold; color: #ffcc00;"))
        latency_layout.addWidget(self.latency_total_us, 4, 1)
        self.latency_total_ms = QLabel("--")
        self.latency_total_ms.setStyleSheet(SS("font-size: 12px; font-weight: bold; color: #ffcc00;"))
        latency_layout.addWidget(self.latency_total_ms, 4, 2)
        self.latency_total_status = QLabel("--")
        self.latency_total_status.setStyleSheet(SS("font-size: 10px; font-weight: bold;"))
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
        lbl.setStyleSheet(SS("font-size: 11px; font-weight: bold; color: #00aaff;"))
        latency_layout.addWidget(lbl, 6, 0)
        self.wifi_latency_total_us = QLabel("--")
        self.wifi_latency_total_us.setStyleSheet(SS("font-size: 12px; font-weight: bold; color: #00ff88;"))
        latency_layout.addWidget(self.wifi_latency_total_us, 6, 1)
        self.wifi_latency_total_ms = QLabel("--")
        self.wifi_latency_total_ms.setStyleSheet(SS("font-size: 12px; color: #00ff88;"))
        latency_layout.addWidget(self.wifi_latency_total_ms, 6, 2)
        self.wifi_latency_status = QLabel("--")
        self.wifi_latency_status.setStyleSheet(SS("font-size: 10px;"))
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
        self.latency_refresh_btn.setStyleSheet(SS("font-size: 10px; padding: 2px 8px;"))
        self.latency_refresh_btn.clicked.connect(self.refresh_latency)
        latency_btn_layout.addWidget(self.latency_refresh_btn)
        
        self.latency_auto_check = QCheckBox("自动")
        self.latency_auto_check.setStyleSheet(SS("font-size: 10px;"))
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
                self.roll_status.setStyleSheet(SS("font-size: 14px; font-weight: bold; color: #4CAF50;"))
            else:
                self.roll_status.setText("状态: 已关闭")
                self.roll_status.setStyleSheet(SS("font-size: 14px; font-weight: bold; color: #f44336;"))
    
    def send_pitch_comp_cmd(self, state):
        """发送 Pitch 腿部角度补偿命令"""
        if not self.parent_window or not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        cmd = f"balance pitchcomp {state}"
        if self.parent_window.send_command(cmd):
            self.parent_window.log(f"发送: {cmd}")
            if state == "on":
                self.pitch_comp_status.setText("状态: 已开启")
                self.pitch_comp_status.setStyleSheet(SS("font-size: 14px; font-weight: bold; color: #4CAF50;"))
            else:
                self.pitch_comp_status.setText("状态: 已关闭")
                self.pitch_comp_status.setStyleSheet(SS("font-size: 14px; font-weight: bold; color: #f44336;"))
    
    def send_xoffset_cmd(self, state):
        """发送 X-Offset 控制命令"""
        if not self.parent_window or not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        cmd = f"balance xoffset {state}"
        if self.parent_window.send_command(cmd):
            self.parent_window.log(f"发送: {cmd}")
            if state == "on":
                self.xoffset_status.setText("状态: 已开启")
                self.xoffset_status.setStyleSheet(SS("font-size: 14px; font-weight: bold; color: #4CAF50;"))
            else:
                self.xoffset_status.setText("状态: 已关闭")
                self.xoffset_status.setStyleSheet(SS("font-size: 14px; font-weight: bold; color: #f44336;"))
    
    def apply_xoffset_params(self):
        """应用 X-Offset PID 参数和限幅"""
        if not self.parent_window or not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        kp = self.xoffset_kp.value()
        ki = self.xoffset_ki.value()
        kd = self.xoffset_kd.value()
        limit = self.xoffset_limit.value()
        # 分别发送各参数
        self.parent_window.send_command(f"balance xoffset kp {kp:.4f}")
        self.parent_window.send_command(f"balance xoffset ki {ki:.4f}")
        self.parent_window.send_command(f"balance xoffset kd {kd:.4f}")
        self.parent_window.send_command(f"balance xoffset limit {limit:.3f}")
        self.parent_window.log(f"X-Offset: Kp={kp:.4f} Ki={ki:.4f} Kd={kd:.4f} Limit={limit:.3f}m")
    
    def apply_joy_scale(self):
        """应用遥杆映射比例"""
        if not self.parent_window or not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        speed_scale = self.joy_speed_scale.value()
        yaw_scale = self.joy_yaw_scale.value()
        self.parent_window.send_command(f"balance joy speed {speed_scale:.6f}")
        self.parent_window.send_command(f"balance joy yaw {yaw_scale:.6f}")
        self.parent_window.log(f"Joy Scale: speed={speed_scale:.6f} (max ±{100*speed_scale:.3f}), "
                               f"yaw={yaw_scale:.6f} (max ±{100*yaw_scale:.3f})")
    
    def send_leg_sync_cmd(self, state):
        """发送 Leg Sync 控制命令"""
        if not self.parent_window or not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        cmd = f"balance sync {state}"
        if self.parent_window.send_command(cmd):
            self.parent_window.log(f"发送: {cmd}")
            if state == "on":
                self.leg_sync_status.setText("状态: 已开启")
                self.leg_sync_status.setStyleSheet(SS("font-size: 14px; font-weight: bold; color: #4CAF50;"))
            else:
                self.leg_sync_status.setText("状态: 已关闭")
                self.leg_sync_status.setStyleSheet(SS("font-size: 14px; font-weight: bold; color: #f44336;"))
    
    def apply_leg_sync_params(self):
        """应用 Leg Sync 参数"""
        if not self.parent_window or not self.parent_window.is_connected():
            QMessageBox.warning(self, "警告", "请先连接串口!")
            return
        gain = self.leg_sync_gain.value()
        max_corr = self.leg_sync_max.value()
        self.parent_window.send_command(f"balance sync gain {gain:.2f}")
        self.parent_window.send_command(f"balance sync max {max_corr:.1f}")
        self.parent_window.log(f"Leg Sync: gain={gain:.2f} max={max_corr:.1f}°")
    
    def do_balance_init(self):
        self.send_cmd("balance init")
        self.init_status.setText("初始化中...")
        self.init_status.setStyleSheet(SS("color: orange;"))
    
    def set_zero_point(self):
        self.send_cmd(f"balance zero {self.zero_input.value()}")
    
    def _on_zero_slider_changed(self, int_val):
        """滑条值变化 → 同步 spinbox 和 label"""
        val = int_val / 100.0
        self.zero_input.blockSignals(True)
        self.zero_input.setValue(val)
        self.zero_input.blockSignals(False)
        self.zero_value_label.setText(f"{val:.2f}°")
    
    def _on_zero_spinbox_changed(self, val):
        """spinbox 值变化 → 同步滑条"""
        self.zero_slider.blockSignals(True)
        self.zero_slider.setValue(int(val * 100))
        self.zero_slider.blockSignals(False)
        self.zero_value_label.setText(f"{val:.2f}°")
    
    def _nudge_zero(self, delta):
        """微调箭头: 调整零点并立即发送"""
        new_val = self.zero_input.value() + delta
        new_val = max(-30.0, min(30.0, new_val))
        self.zero_input.setValue(new_val)
        self.set_zero_point()
    
    def set_zp_threshold(self):
        self.send_cmd(f"balance zero threshold {self.zp_threshold_input.value()}")
    
    def on_init_success(self):
        self.balance_initialized = True
        self.init_status.setText("✓ 已初始化")
        self.init_status.setStyleSheet(SS("color: green; font-weight: bold;"))
        self.sys_status_label.setText("就绪")
        self.sys_status_label.setStyleSheet(SS("font-weight: bold; color: green;"))
    
    def set_all_plot_channels(self, checked):
        """全选/全不选所有波形通道"""
        for cb in self.plot_ch_checkboxes.values():
            cb.setChecked(checked)
    
    def set_plot_channel_preset(self, channels):
        """设置通道预设 (先全不选, 再选指定通道)"""
        for cb in self.plot_ch_checkboxes.values():
            cb.setChecked(False)
        for ch in channels.upper():
            if ch in self.plot_ch_checkboxes:
                self.plot_ch_checkboxes[ch].setChecked(True)
    
    def apply_plot_channels(self):
        """将当前勾选的通道发送给 ESP32"""
        selected = ""
        for ch_id, cb in self.plot_ch_checkboxes.items():
            if cb.isChecked():
                selected += ch_id
        if selected:
            self.send_cmd(f"balance plot ch {selected}")
        else:
            self.send_cmd("balance plot ch none")
    
    def plot_start(self):
        """开启波形: 先应用通道选择, 再开启"""
        self.apply_plot_channels()
        self.send_cmd("balance plot on")
    
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
    
    def on_yaw_force_toggle(self, state):
        """YAW强制使能开关 (无需遥控器)"""
        if state == Qt.Checked:
            self.send_cmd("balance yaw on")
        else:
            self.send_cmd("balance yaw off")
    
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
        self._update_freq_status(self.freq_motor_status, motor_hz, 200, 10)
        
        # Leg
        self.freq_leg_label.setText(f"{leg_hz:.1f} Hz")
        self._update_freq_status(self.freq_leg_status, leg_hz, 20, 3)
    
    def _update_freq_status(self, label, actual, target, tolerance):
        """更新频率状态指示"""
        if actual == 0:
            label.setText("⚫ 停止")
            label.setStyleSheet(SS("color: gray;"))
        elif abs(actual - target) <= tolerance:
            label.setText("🟢 正常")
            label.setStyleSheet(SS("color: #00ff00;"))
        elif actual < target - tolerance:
            label.setText("🟡 偏低")
            label.setStyleSheet(SS("color: #ffaa00;"))
        else:
            label.setText("🔴 偏高")
            label.setStyleSheet(SS("color: #ff4444;"))
    
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
                self.latency_stats_label.setStyleSheet(SS("font-weight: bold; color: #00ff00;"))
            elif max_us <= 15000:
                self.latency_stats_label.setStyleSheet(SS("font-weight: bold; color: #ffaa00;"))
            else:
                self.latency_stats_label.setStyleSheet(SS("font-weight: bold; color: #ff4444;"))
    
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
            self.wifi_latency_stats_label.setStyleSheet(SS("font-weight: bold; color: #00ff00;"))
        elif max_us <= 80000:
            self.wifi_latency_stats_label.setStyleSheet(SS("font-weight: bold; color: #ffaa00;"))
        else:
            self.wifi_latency_stats_label.setStyleSheet(SS("font-weight: bold; color: #ff4444;"))
    
    def _update_latency_status(self, label, us_value, good_threshold, warn_threshold):
        """更新延迟状态指示"""
        if us_value <= good_threshold:
            label.setText("🟢 优秀")
            label.setStyleSheet(SS("color: #00ff00; font-weight: bold;"))
        elif us_value <= warn_threshold:
            label.setText("🟡 正常")
            label.setStyleSheet(SS("color: #ffaa00; font-weight: bold;"))
        else:
            label.setText("🔴 需优化")
            label.setStyleSheet(SS("color: #ff4444; font-weight: bold;"))


# ============================================================================
# 双环 PID 调参面板
# ============================================================================
class DualPIDPanel(QWidget):
    """双环 PID 调参面板 - 直立环 + 速度环 (扭矩输出)"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # 标题和说明
        title_label = QLabel("🎯 双环 PID 平衡控制 (扭矩输出)")
        title_label.setStyleSheet(SS("font-size: 18px; font-weight: bold; color: #00aaff;"))
        layout.addWidget(title_label)
        
        self.desc_label = QLabel(
            "控制架构: 角度环(外环) → 速度环(内环) → 扭矩输出\n"
            "角度环: pitch → 目标速度 | 速度环: 速度误差 → 扭矩"
        )
        self.desc_label.setStyleSheet(SS("color: #888; font-size: 11px;"))
        layout.addWidget(self.desc_label)
        
        # 当前环序状态
        self.loop_order = 0  # 0=ANGLE_FIRST, 1=SPEED_FIRST
        
        # 模式切换
        mode_group = QGroupBox("控制模式切换")
        mode_layout = QHBoxLayout()
        
        self.mode_label = QLabel("当前模式: 未知")
        self.mode_label.setStyleSheet(SS("font-size: 14px; font-weight: bold;"))
        mode_layout.addWidget(self.mode_label)
        
        mode_layout.addStretch()
        
        self.lqr_btn = QPushButton("🔵 LQR")
        self.lqr_btn.setToolTip("切换到 LQR 多环控制 (默认)")
        self.lqr_btn.clicked.connect(lambda: self.set_mode("lqr"))
        mode_layout.addWidget(self.lqr_btn)
        
        self.pid_btn = QPushButton("🟢 双环PID")
        self.pid_btn.setToolTip("双环 PID 控制 (扭矩模式)")
        self.pid_btn.setStyleSheet(SS("background-color: #4CAF50; color: white;"))
        self.pid_btn.clicked.connect(lambda: self.set_mode("pid"))
        mode_layout.addWidget(self.pid_btn)
        
        self.spid_btn = QPushButton("🟡 单环PID")
        self.spid_btn.setToolTip("单环 PID 控制 (速度模式)")
        self.spid_btn.setStyleSheet(SS("background-color: #FF9800; color: white;"))
        self.spid_btn.clicked.connect(lambda: self.set_mode("spid"))
        mode_layout.addWidget(self.spid_btn)
        
        self.tpid_btn = QPushButton("🟣 四环PID")
        self.tpid_btn.setToolTip("四环 PID 控制 (位移→速度→角度→轮速)")
        self.tpid_btn.setStyleSheet(SS("background-color: #9C27B0; color: white;"))
        self.tpid_btn.clicked.connect(lambda: self.set_mode("tpid"))
        mode_layout.addWidget(self.tpid_btn)
        
        self.mode_status_btn = QPushButton("📊 状态")
        self.mode_status_btn.clicked.connect(lambda: self.send_cmd("balance mode"))
        mode_layout.addWidget(self.mode_status_btn)
        
        mode_group.setLayout(mode_layout)
        layout.addWidget(mode_group)
        
        # 环路顺序切换
        order_group = QGroupBox("🔀 环路顺序 (双环 PID)")
        order_layout = QHBoxLayout()
        
        self.order_label = QLabel("当前: 角度优先")
        self.order_label.setStyleSheet(SS("font-size: 13px; font-weight: bold; color: #4CAF50;"))
        order_layout.addWidget(self.order_label)
        
        order_layout.addStretch()
        
        self.order_af_btn = QPushButton("角度优先 (AF)")
        self.order_af_btn.setToolTip("角度环(外) → 速度环(内)\npitch → target_speed → torque")
        self.order_af_btn.setStyleSheet(SS("background-color: #4CAF50; color: white;"))
        self.order_af_btn.clicked.connect(lambda: self.set_loop_order(0))
        order_layout.addWidget(self.order_af_btn)
        
        self.order_sf_btn = QPushButton("速度优先 (SF)")
        self.order_sf_btn.setToolTip("速度环(外) → 角度环(内)\n0 - wheel_speed → pitch_target → torque")
        self.order_sf_btn.clicked.connect(lambda: self.set_loop_order(1))
        order_layout.addWidget(self.order_sf_btn)
        
        self.order_query_btn = QPushButton("📊 查询")
        self.order_query_btn.clicked.connect(lambda: self.send_cmd("balance dpid order"))
        order_layout.addWidget(self.order_query_btn)
        
        order_group.setLayout(order_layout)
        layout.addWidget(order_group)
        
        # 角度环 PID
        self.angle_group = QGroupBox("🎯 角度环 PID (外环: Pitch → 目标速度)")
        angle_layout = QGridLayout()
        
        angle_layout.addWidget(QLabel("Kp:"), 0, 0)
        self.angle_kp = QDoubleSpinBox()
        self.angle_kp.setRange(0, 100)
        self.angle_kp.setSingleStep(0.1)
        self.angle_kp.setDecimals(6)
        self.angle_kp.setValue(0.025)
        angle_layout.addWidget(self.angle_kp, 0, 1)
        
        angle_layout.addWidget(QLabel("Ki:"), 0, 2)
        self.angle_ki = QDoubleSpinBox()
        self.angle_ki.setRange(0, 10)
        self.angle_ki.setSingleStep(0.001)
        self.angle_ki.setDecimals(6)
        self.angle_ki.setValue(0.000003)
        angle_layout.addWidget(self.angle_ki, 0, 3)
        
        angle_layout.addWidget(QLabel("Kd:"), 0, 4)
        self.angle_kd = QDoubleSpinBox()
        self.angle_kd.setRange(0, 10)
        self.angle_kd.setSingleStep(0.001)
        self.angle_kd.setDecimals(6)
        self.angle_kd.setValue(0.003)
        angle_layout.addWidget(self.angle_kd, 0, 5)
        
        self.angle_send_btn = QPushButton("发送")
        self.angle_send_btn.clicked.connect(self.send_angle_pid)
        angle_layout.addWidget(self.angle_send_btn, 0, 6)
        
        # 直立环说明
        angle_note = QLabel("tip: 前倾(pitch>0)→输出正速度, P越大响应越快但易震荡")
        angle_note.setStyleSheet(SS("color: #666; font-size: 10px;"))
        angle_layout.addWidget(angle_note, 1, 0, 1, 7)
        
        self.angle_group.setLayout(angle_layout)
        layout.addWidget(self.angle_group)
        
        # 速度环 PID
        self.speed_group = QGroupBox("🔄 速度环 PID (内环: 速度误差 → 扭矩)")
        speed_layout = QGridLayout()
        
        speed_layout.addWidget(QLabel("Kp:"), 0, 0)
        self.speed_kp = QDoubleSpinBox()
        self.speed_kp.setRange(0, 10)
        self.speed_kp.setSingleStep(0.001)
        self.speed_kp.setDecimals(6)
        self.speed_kp.setValue(0.0005)
        speed_layout.addWidget(self.speed_kp, 0, 1)
        
        speed_layout.addWidget(QLabel("Ki:"), 0, 2)
        self.speed_ki = QDoubleSpinBox()
        self.speed_ki.setRange(0, 1)
        self.speed_ki.setSingleStep(0.001)
        self.speed_ki.setDecimals(6)
        self.speed_ki.setValue(0.000001)
        speed_layout.addWidget(self.speed_ki, 0, 3)
        
        speed_layout.addWidget(QLabel("Kd:"), 0, 4)
        self.speed_kd = QDoubleSpinBox()
        self.speed_kd.setRange(0, 1)
        self.speed_kd.setSingleStep(0.001)
        self.speed_kd.setDecimals(6)
        self.speed_kd.setValue(0.000015)
        speed_layout.addWidget(self.speed_kd, 0, 5)
        
        self.speed_send_btn = QPushButton("发送")
        self.speed_send_btn.clicked.connect(self.send_speed_pid)
        speed_layout.addWidget(self.speed_send_btn, 0, 6)
        
        # 速度环说明
        speed_note = QLabel("tip: I项消除稳态误差, 过大会导致震荡")
        speed_note.setStyleSheet(SS("color: #666; font-size: 10px;"))
        speed_layout.addWidget(speed_note, 1, 0, 1, 4)
        
        # 速度指令增益 (SPEED_FIRST 外环)
        speed_layout.addWidget(QLabel("指令增益:"), 1, 4)
        self.speed_cmd_gain = QDoubleSpinBox()
        self.speed_cmd_gain.setRange(0, 999999)
        self.speed_cmd_gain.setSingleStep(1000)
        self.speed_cmd_gain.setDecimals(1)
        self.speed_cmd_gain.setValue(33333.0)
        self.speed_cmd_gain.setToolTip("SPEED_FIRST 模式: target_speed × 此增益 → 速度外环输入")
        speed_layout.addWidget(self.speed_cmd_gain, 1, 5)
        
        self.gain_send_btn = QPushButton("发送")
        self.gain_send_btn.clicked.connect(self.send_speed_cmd_gain)
        speed_layout.addWidget(self.gain_send_btn, 1, 6)
        
        self.speed_group.setLayout(speed_layout)
        layout.addWidget(self.speed_group)
        
        # 角速度阻尼 PID (Gyro damping)
        gyro_group = QGroupBox("🌀 角速度阻尼 PID (pitch_rate → 扭矩补偿)")
        gyro_layout = QGridLayout()
        
        gyro_layout.addWidget(QLabel("Kp:"), 0, 0)
        self.dpid_gyro_kp = QDoubleSpinBox()
        self.dpid_gyro_kp.setRange(0, 10)
        self.dpid_gyro_kp.setSingleStep(0.001)
        self.dpid_gyro_kp.setDecimals(4)
        self.dpid_gyro_kp.setValue(0.0)
        gyro_layout.addWidget(self.dpid_gyro_kp, 0, 1)
        
        gyro_layout.addWidget(QLabel("Ki:"), 0, 2)
        self.dpid_gyro_ki = QDoubleSpinBox()
        self.dpid_gyro_ki.setRange(0, 1)
        self.dpid_gyro_ki.setSingleStep(0.001)
        self.dpid_gyro_ki.setDecimals(4)
        self.dpid_gyro_ki.setValue(0.0)
        gyro_layout.addWidget(self.dpid_gyro_ki, 0, 3)
        
        gyro_layout.addWidget(QLabel("Kd:"), 0, 4)
        self.dpid_gyro_kd = QDoubleSpinBox()
        self.dpid_gyro_kd.setRange(0, 1)
        self.dpid_gyro_kd.setSingleStep(0.0001)
        self.dpid_gyro_kd.setDecimals(6)
        self.dpid_gyro_kd.setValue(0.0)
        gyro_layout.addWidget(self.dpid_gyro_kd, 0, 5)
        
        self.dpid_gyro_send_btn = QPushButton("发送")
        self.dpid_gyro_send_btn.clicked.connect(self.send_dpid_gyro)
        gyro_layout.addWidget(self.dpid_gyro_send_btn, 0, 6)
        
        gyro_note = QLabel("tip: 参考LQR角速度阻尼, setpoint=0, 默认全0(关闭)")
        gyro_note.setStyleSheet(SS("color: #666; font-size: 10px;"))
        gyro_layout.addWidget(gyro_note, 1, 0, 1, 7)
        
        gyro_group.setLayout(gyro_layout)
        layout.addWidget(gyro_group)
        
        # 角度零点
        zero_group = QGroupBox("角度零点")
        zero_layout = QHBoxLayout()
        
        zero_layout.addWidget(QLabel("零点 (°):"))
        self.zero_input = QDoubleSpinBox()
        self.zero_input.setRange(-30, 30)
        self.zero_input.setSingleStep(0.01)
        self.zero_input.setDecimals(3)
        self.zero_input.setValue(0.0)
        zero_layout.addWidget(self.zero_input)
        
        self.zero_send_btn = QPushButton("设置")
        self.zero_send_btn.clicked.connect(self.send_zero)
        zero_layout.addWidget(self.zero_send_btn)
        
        zero_layout.addStretch()
        
        self.reset_btn = QPushButton("🔄 重置 PID")
        self.reset_btn.clicked.connect(lambda: self.send_cmd("balance dpid reset"))
        zero_layout.addWidget(self.reset_btn)
        
        zero_group.setLayout(zero_layout)
        layout.addWidget(zero_group)
        
        # 实时状态显示 (扩展版)
        status_group = QGroupBox("📊 实时 PID 调试数据")
        status_main_layout = QVBoxLayout()
        
        # === 双环 PID 状态 ===
        dpid_frame = QFrame()
        dpid_frame.setStyleSheet(SS("QFrame { background-color: #1a2a1a; border-radius: 5px; padding: 5px; }"))
        dpid_layout = QGridLayout(dpid_frame)
        dpid_layout.setSpacing(8)
        
        dpid_title = QLabel("🟢 双环 PID")
        dpid_title.setStyleSheet(SS("font-size: 12px; font-weight: bold; color: #4CAF50;"))
        dpid_layout.addWidget(dpid_title, 0, 0, 1, 8)
        
        # 第一行: 输入信息
        dpid_layout.addWidget(QLabel("Pitch:"), 1, 0)
        self.dpid_pitch_label = QLabel("--°")
        self.dpid_pitch_label.setStyleSheet(SS("font-weight: bold; color: #ffcc00;"))
        dpid_layout.addWidget(self.dpid_pitch_label, 1, 1)
        
        dpid_layout.addWidget(QLabel("误差:"), 1, 2)
        self.angle_err_label = QLabel("--°")
        self.angle_err_label.setStyleSheet(SS("font-weight: bold; color: #ff8800;"))
        dpid_layout.addWidget(self.angle_err_label, 1, 3)
        
        dpid_layout.addWidget(QLabel("角速度:"), 1, 4)
        self.dpid_rate_label = QLabel("--°/s")
        self.dpid_rate_label.setStyleSheet(SS("font-weight: bold; color: #88ccff;"))
        dpid_layout.addWidget(self.dpid_rate_label, 1, 5)
        
        # 第二行: 直立环 PID 分量
        dpid_layout.addWidget(QLabel("直立环:"), 2, 0)
        dpid_layout.addWidget(QLabel("P="), 2, 1)
        self.dpid_angle_p = QLabel("--")
        self.dpid_angle_p.setStyleSheet(SS("color: #ff6666;"))
        dpid_layout.addWidget(self.dpid_angle_p, 2, 2)
        dpid_layout.addWidget(QLabel("I="), 2, 3)
        self.dpid_angle_i = QLabel("--")
        self.dpid_angle_i.setStyleSheet(SS("color: #66ff66;"))
        dpid_layout.addWidget(self.dpid_angle_i, 2, 4)
        dpid_layout.addWidget(QLabel("D="), 2, 5)
        self.dpid_angle_d = QLabel("--")
        self.dpid_angle_d.setStyleSheet(SS("color: #6666ff;"))
        dpid_layout.addWidget(self.dpid_angle_d, 2, 6)
        dpid_layout.addWidget(QLabel("→"), 2, 7)
        self.target_speed_label = QLabel("-- rad/s")
        self.target_speed_label.setStyleSheet(SS("font-weight: bold; color: #00aaff;"))
        dpid_layout.addWidget(self.target_speed_label, 2, 8)
        
        # 第三行: 速度环 PID 分量
        dpid_layout.addWidget(QLabel("速度环:"), 3, 0)
        dpid_layout.addWidget(QLabel("err="), 3, 1)
        self.speed_err_label = QLabel("--")
        self.speed_err_label.setStyleSheet(SS("color: #ff8800;"))
        dpid_layout.addWidget(self.speed_err_label, 3, 2)
        dpid_layout.addWidget(QLabel("P="), 3, 3)
        self.dpid_speed_p = QLabel("--")
        self.dpid_speed_p.setStyleSheet(SS("color: #ff6666;"))
        dpid_layout.addWidget(self.dpid_speed_p, 3, 4)
        dpid_layout.addWidget(QLabel("I="), 3, 5)
        self.dpid_speed_i = QLabel("--")
        self.dpid_speed_i.setStyleSheet(SS("color: #66ff66;"))
        dpid_layout.addWidget(self.dpid_speed_i, 3, 6)
        dpid_layout.addWidget(QLabel("D="), 3, 7)
        self.dpid_speed_d = QLabel("--")
        self.dpid_speed_d.setStyleSheet(SS("color: #6666ff;"))
        dpid_layout.addWidget(self.dpid_speed_d, 3, 8)
        
        # 第四行: 输出扭矩
        dpid_layout.addWidget(QLabel("输出扭矩:"), 4, 0, 1, 2)
        self.torque_label = QLabel("-- Nm")
        self.torque_label.setStyleSheet(SS("font-size: 16px; font-weight: bold; color: #00ff88;"))
        dpid_layout.addWidget(self.torque_label, 4, 2, 1, 3)
        
        status_main_layout.addWidget(dpid_frame)
        
        # === 单环 PID 状态 ===
        spid_frame = QFrame()
        spid_frame.setStyleSheet(SS("QFrame { background-color: #2a2a1a; border-radius: 5px; padding: 5px; }"))
        spid_layout = QGridLayout(spid_frame)
        spid_layout.setSpacing(8)
        
        spid_title = QLabel("🟡 单环 PID")
        spid_title.setStyleSheet(SS("font-size: 12px; font-weight: bold; color: #FF9800;"))
        spid_layout.addWidget(spid_title, 0, 0, 1, 8)
        
        # 第一行: 输入信息
        spid_layout.addWidget(QLabel("Pitch:"), 1, 0)
        self.spid_pitch_label = QLabel("--°")
        self.spid_pitch_label.setStyleSheet(SS("font-weight: bold; color: #ffcc00;"))
        spid_layout.addWidget(self.spid_pitch_label, 1, 1)
        
        spid_layout.addWidget(QLabel("误差:"), 1, 2)
        self.spid_err_label = QLabel("--°")
        self.spid_err_label.setStyleSheet(SS("font-weight: bold; color: #ff8800;"))
        spid_layout.addWidget(self.spid_err_label, 1, 3)
        
        spid_layout.addWidget(QLabel("角速度:"), 1, 4)
        self.spid_rate_label = QLabel("--°/s")
        self.spid_rate_label.setStyleSheet(SS("font-weight: bold; color: #88ccff;"))
        spid_layout.addWidget(self.spid_rate_label, 1, 5)
        
        # 第二行: PID 分量
        spid_layout.addWidget(QLabel("直立环:"), 2, 0)
        spid_layout.addWidget(QLabel("P="), 2, 1)
        self.spid_p = QLabel("--")
        self.spid_p.setStyleSheet(SS("color: #ff6666;"))
        spid_layout.addWidget(self.spid_p, 2, 2)
        spid_layout.addWidget(QLabel("I="), 2, 3)
        self.spid_i = QLabel("--")
        self.spid_i.setStyleSheet(SS("color: #66ff66;"))
        spid_layout.addWidget(self.spid_i, 2, 4)
        spid_layout.addWidget(QLabel("D="), 2, 5)
        self.spid_d = QLabel("--")
        self.spid_d.setStyleSheet(SS("color: #6666ff;"))
        spid_layout.addWidget(self.spid_d, 2, 6)
        
        # 第三行: 输出速度
        spid_layout.addWidget(QLabel("输出速度:"), 3, 0, 1, 2)
        self.spid_speed_label = QLabel("-- rad/s")
        self.spid_speed_label.setStyleSheet(SS("font-size: 14px; font-weight: bold; color: #00aaff;"))
        spid_layout.addWidget(self.spid_speed_label, 3, 2, 1, 2)
        self.spid_rpm_label = QLabel("(-- rpm)")
        self.spid_rpm_label.setStyleSheet(SS("color: #888;"))
        spid_layout.addWidget(self.spid_rpm_label, 3, 4, 1, 2)
        
        status_main_layout.addWidget(spid_frame)
        
        # === 四环 PID 状态 ===
        tpid_frame = QFrame()
        tpid_frame.setStyleSheet(SS("QFrame { background-color: #1a1a2a; border-radius: 5px; padding: 5px; }"))
        tpid_status_layout = QGridLayout(tpid_frame)
        tpid_status_layout.setSpacing(8)
        
        tpid_title = QLabel("🟣 四环 PID")
        tpid_title.setStyleSheet(SS("font-size: 12px; font-weight: bold; color: #9C27B0;"))
        tpid_status_layout.addWidget(tpid_title, 0, 0, 1, 2)
        
        self.tpid_wmode_label = QLabel("模式: --")
        self.tpid_wmode_label.setStyleSheet(SS("font-weight: bold; color: #bb88ff;"))
        tpid_status_layout.addWidget(self.tpid_wmode_label, 0, 2, 1, 2)
        
        self.tpid_pitch_label = QLabel("pitch: --°")
        self.tpid_pitch_label.setStyleSheet(SS("font-weight: bold; color: #ffcc00;"))
        tpid_status_layout.addWidget(self.tpid_pitch_label, 0, 4, 1, 2)
        
        self.tpid_wspd_label = QLabel("spd: --")
        self.tpid_wspd_label.setStyleSheet(SS("font-weight: bold; color: #88ccff;"))
        tpid_status_layout.addWidget(self.tpid_wspd_label, 0, 6, 1, 2)
        
        # 第1.5行: 位移环(最外, 可选)
        self.tpid_dist_row_label = QLabel("位移环(最外):")
        self.tpid_dist_row_label.setStyleSheet(SS("color: #666;"))
        tpid_status_layout.addWidget(self.tpid_dist_row_label, 1, 0)
        tpid_status_layout.addWidget(QLabel("err="), 1, 1)
        self.tpid_dist_err = QLabel("--")
        self.tpid_dist_err.setStyleSheet(SS("color: #ff8800;"))
        tpid_status_layout.addWidget(self.tpid_dist_err, 1, 2)
        tpid_status_layout.addWidget(QLabel("→ spd_corr="), 1, 3)
        self.tpid_dist_corr = QLabel("--")
        self.tpid_dist_corr.setStyleSheet(SS("color: #00aaff;"))
        tpid_status_layout.addWidget(self.tpid_dist_corr, 1, 4)
        
        # 第二行: 速度环(外)
        tpid_status_layout.addWidget(QLabel("速度环(外):"), 2, 0)
        tpid_status_layout.addWidget(QLabel("err="), 2, 1)
        self.tpid_spd_err = QLabel("--")
        self.tpid_spd_err.setStyleSheet(SS("color: #ff8800;"))
        tpid_status_layout.addWidget(self.tpid_spd_err, 2, 2)
        tpid_status_layout.addWidget(QLabel("→ pitch_tgt="), 2, 3)
        self.tpid_pitch_tgt = QLabel("--°")
        self.tpid_pitch_tgt.setStyleSheet(SS("color: #00aaff;"))
        tpid_status_layout.addWidget(self.tpid_pitch_tgt, 2, 4)
        
        # 第三行: 角度环(中)
        tpid_status_layout.addWidget(QLabel("角度环(中):"), 3, 0)
        tpid_status_layout.addWidget(QLabel("err="), 3, 1)
        self.tpid_ang_err = QLabel("--")
        self.tpid_ang_err.setStyleSheet(SS("color: #ff8800;"))
        tpid_status_layout.addWidget(self.tpid_ang_err, 3, 2)
        tpid_status_layout.addWidget(QLabel("→ whl_tgt="), 3, 3)
        self.tpid_whl_tgt = QLabel("--")
        self.tpid_whl_tgt.setStyleSheet(SS("color: #00aaff;"))
        tpid_status_layout.addWidget(self.tpid_whl_tgt, 3, 4)
        
        # 第四行: 轮速环(内)
        tpid_status_layout.addWidget(QLabel("轮速环(内):"), 4, 0)
        tpid_status_layout.addWidget(QLabel("err="), 4, 1)
        self.tpid_whl_err = QLabel("--")
        self.tpid_whl_err.setStyleSheet(SS("color: #ff8800;"))
        tpid_status_layout.addWidget(self.tpid_whl_err, 4, 2)
        tpid_status_layout.addWidget(QLabel("→ out="), 4, 3)
        self.tpid_out = QLabel("--")
        self.tpid_out.setStyleSheet(SS("font-size: 14px; font-weight: bold; color: #00ff88;"))
        tpid_status_layout.addWidget(self.tpid_out, 4, 4)
        
        status_main_layout.addWidget(tpid_frame)
        
        # 刷新按钮
        status_btn_layout = QHBoxLayout()
        self.refresh_btn = QPushButton("🔄 刷新状态")
        self.refresh_btn.clicked.connect(lambda: self.send_cmd("balance dpid status"))
        status_btn_layout.addWidget(self.refresh_btn)
        
        self.auto_refresh_check = QCheckBox("自动刷新 (500ms)")
        self.auto_refresh_check.stateChanged.connect(self.toggle_auto_refresh)
        status_btn_layout.addWidget(self.auto_refresh_check)
        
        status_btn_layout.addStretch()
        
        # 调试数据更新指示
        self.debug_update_label = QLabel("调试数据: 等待...")
        self.debug_update_label.setStyleSheet(SS("color: #666; font-size: 10px;"))
        status_btn_layout.addWidget(self.debug_update_label)
        
        status_main_layout.addLayout(status_btn_layout)
        status_group.setLayout(status_main_layout)
        layout.addWidget(status_group)
        
        # 自动刷新定时器
        self.refresh_timer = QTimer()
        self.refresh_timer.timeout.connect(lambda: self.send_cmd("balance dpid status"))
        
        # 快速调参按钮
        quick_group = QGroupBox("⚡ 快速调参")
        quick_layout = QHBoxLayout()
        
        self.preset1_btn = QPushButton("预设1: 保守")
        self.preset1_btn.setToolTip("angle: 10,0,0.3 | speed: 0.3,0.05,0")
        self.preset1_btn.clicked.connect(lambda: self.apply_preset(10, 0, 0.3, 0.3, 0.05, 0))
        quick_layout.addWidget(self.preset1_btn)
        
        self.preset2_btn = QPushButton("预设2: 平衡")
        self.preset2_btn.setToolTip("angle: 15,0,0.5 | speed: 0.5,0.1,0.01")
        self.preset2_btn.clicked.connect(lambda: self.apply_preset(15, 0, 0.5, 0.5, 0.1, 0.01))
        quick_layout.addWidget(self.preset2_btn)
        
        self.preset3_btn = QPushButton("预设3: 激进")
        self.preset3_btn.setToolTip("angle: 20,0,0.8 | speed: 0.8,0.15,0.02")
        self.preset3_btn.clicked.connect(lambda: self.apply_preset(20, 0, 0.8, 0.8, 0.15, 0.02))
        quick_layout.addWidget(self.preset3_btn)
        
        quick_group.setLayout(quick_layout)
        layout.addWidget(quick_group)
        
        # ========== 单环 PID 参数区域 (速度输出模式) ==========
        spid_group = QGroupBox("🟡 单环 PID 参数 (速度输出模式)")
        spid_group.setStyleSheet(SS("QGroupBox { color: #FF9800; }"))
        spid_layout = QGridLayout()
        
        spid_desc = QLabel("适合电机速度模式，输出目标速度 (rad/s) 送给电机内部速度环")
        spid_desc.setStyleSheet(SS("color: #888; font-size: 10px;"))
        spid_layout.addWidget(spid_desc, 0, 0, 1, 7)
        
        spid_layout.addWidget(QLabel("Kp:"), 1, 0)
        self.spid_kp = QDoubleSpinBox()
        self.spid_kp.setRange(0, 100)
        self.spid_kp.setSingleStep(0.5)
        self.spid_kp.setDecimals(2)
        self.spid_kp.setValue(15.0)
        spid_layout.addWidget(self.spid_kp, 1, 1)
        
        spid_layout.addWidget(QLabel("Ki:"), 1, 2)
        self.spid_ki = QDoubleSpinBox()
        self.spid_ki.setRange(0, 10)
        self.spid_ki.setSingleStep(0.01)
        self.spid_ki.setDecimals(3)
        self.spid_ki.setValue(0.0)
        spid_layout.addWidget(self.spid_ki, 1, 3)
        
        spid_layout.addWidget(QLabel("Kd:"), 1, 4)
        self.spid_kd = QDoubleSpinBox()
        self.spid_kd.setRange(0, 10)
        self.spid_kd.setSingleStep(0.01)
        self.spid_kd.setDecimals(4)
        self.spid_kd.setValue(0.5)
        spid_layout.addWidget(self.spid_kd, 1, 5)
        
        self.spid_send_btn = QPushButton("发送")
        self.spid_send_btn.clicked.connect(self.send_spid_params)
        spid_layout.addWidget(self.spid_send_btn, 1, 6)
        
        # 输出限幅
        spid_layout.addWidget(QLabel("输出限幅:"), 2, 0)
        self.spid_limit = QDoubleSpinBox()
        self.spid_limit.setRange(1, 200)
        self.spid_limit.setSingleStep(5)
        self.spid_limit.setDecimals(1)
        self.spid_limit.setValue(100.0)
        self.spid_limit.setSuffix(" rad/s")
        spid_layout.addWidget(self.spid_limit, 2, 1)
        
        self.spid_limit_btn = QPushButton("设置限幅")
        self.spid_limit_btn.clicked.connect(self.send_spid_limit)
        spid_layout.addWidget(self.spid_limit_btn, 2, 2)
        
        self.spid_status_btn = QPushButton("📊 单环状态")
        self.spid_status_btn.clicked.connect(lambda: self.send_cmd("balance spid status"))
        spid_layout.addWidget(self.spid_status_btn, 2, 4, 1, 2)
        
        self.spid_reset_btn = QPushButton("🔄 重置")
        self.spid_reset_btn.clicked.connect(lambda: self.send_cmd("balance spid reset"))
        spid_layout.addWidget(self.spid_reset_btn, 2, 6)
        
        spid_group.setLayout(spid_layout)
        layout.addWidget(spid_group)
        
        # ========== 四环 PID 参数区域 (位移→速度→角度→轮速) ==========
        tpid_group = QGroupBox("🟣 四环 PID 参数 (位移→速度→角度→轮速)")
        tpid_group.setStyleSheet(SS("QGroupBox { color: #9C27B0; }"))
        tpid_main_layout = QVBoxLayout()
        
        tpid_desc = QLabel("四环串级: 位移环(最外,可选)→速度环(外)→角度环(中)→轮速环(内)，轮速环可选速度/扭矩输出")
        tpid_desc.setStyleSheet(SS("color: #888; font-size: 10px;"))
        tpid_main_layout.addWidget(tpid_desc)
        
        # 速度环 PID (外环)
        tpid_speed_layout = QGridLayout()
        tpid_speed_layout.addWidget(QLabel("速度环(外):"), 0, 0)
        tpid_speed_layout.addWidget(QLabel("Kp:"), 0, 1)
        self.tpid_speed_kp = QDoubleSpinBox()
        self.tpid_speed_kp.setRange(0, 10)
        self.tpid_speed_kp.setSingleStep(0.0001)
        self.tpid_speed_kp.setDecimals(6)
        self.tpid_speed_kp.setValue(0.85)
        tpid_speed_layout.addWidget(self.tpid_speed_kp, 0, 2)
        tpid_speed_layout.addWidget(QLabel("Ki:"), 0, 3)
        self.tpid_speed_ki = QDoubleSpinBox()
        self.tpid_speed_ki.setRange(0, 1)
        self.tpid_speed_ki.setSingleStep(0.000001)
        self.tpid_speed_ki.setDecimals(6)
        self.tpid_speed_ki.setValue(0.00006)
        tpid_speed_layout.addWidget(self.tpid_speed_ki, 0, 4)
        tpid_speed_layout.addWidget(QLabel("Kd:"), 0, 5)
        self.tpid_speed_kd = QDoubleSpinBox()
        self.tpid_speed_kd.setRange(0, 1)
        self.tpid_speed_kd.setSingleStep(0.000001)
        self.tpid_speed_kd.setDecimals(6)
        self.tpid_speed_kd.setValue(0.0)
        tpid_speed_layout.addWidget(self.tpid_speed_kd, 0, 6)
        tpid_speed_layout.addWidget(QLabel("Limit:"), 0, 7)
        self.tpid_speed_limit = QDoubleSpinBox()
        self.tpid_speed_limit.setRange(0.1, 500)
        self.tpid_speed_limit.setSingleStep(1.0)
        self.tpid_speed_limit.setDecimals(1)
        self.tpid_speed_limit.setValue(40.0)
        self.tpid_speed_limit.setToolTip("速度环输出限幅 (最大目标倾角, deg)")
        tpid_speed_layout.addWidget(self.tpid_speed_limit, 0, 8)
        self.tpid_speed_send = QPushButton("发送")
        self.tpid_speed_send.clicked.connect(self.send_tpid_speed)
        tpid_speed_layout.addWidget(self.tpid_speed_send, 0, 9)
        tpid_main_layout.addLayout(tpid_speed_layout)
        
        # 角度环 PID (中环)
        tpid_angle_layout = QGridLayout()
        tpid_angle_layout.addWidget(QLabel("角度环(中):"), 0, 0)
        tpid_angle_layout.addWidget(QLabel("Kp:"), 0, 1)
        self.tpid_angle_kp = QDoubleSpinBox()
        self.tpid_angle_kp.setRange(0, 100)
        self.tpid_angle_kp.setSingleStep(0.001)
        self.tpid_angle_kp.setDecimals(6)
        self.tpid_angle_kp.setValue(1.2)
        tpid_angle_layout.addWidget(self.tpid_angle_kp, 0, 2)
        tpid_angle_layout.addWidget(QLabel("Ki:"), 0, 3)
        self.tpid_angle_ki = QDoubleSpinBox()
        self.tpid_angle_ki.setRange(0, 10)
        self.tpid_angle_ki.setSingleStep(0.000001)
        self.tpid_angle_ki.setDecimals(6)
        self.tpid_angle_ki.setValue(0.0)
        tpid_angle_layout.addWidget(self.tpid_angle_ki, 0, 4)
        tpid_angle_layout.addWidget(QLabel("Kd:"), 0, 5)
        self.tpid_angle_kd = QDoubleSpinBox()
        self.tpid_angle_kd.setRange(0, 10)
        self.tpid_angle_kd.setSingleStep(0.001)
        self.tpid_angle_kd.setDecimals(6)
        self.tpid_angle_kd.setValue(0.0)
        tpid_angle_layout.addWidget(self.tpid_angle_kd, 0, 6)
        tpid_angle_layout.addWidget(QLabel("Limit:"), 0, 7)
        self.tpid_angle_limit = QDoubleSpinBox()
        self.tpid_angle_limit.setRange(0.1, 1000)
        self.tpid_angle_limit.setSingleStep(5.0)
        self.tpid_angle_limit.setDecimals(1)
        self.tpid_angle_limit.setValue(20.0)
        self.tpid_angle_limit.setToolTip("角度环输出限幅 (最大目标轮速, rad/s)")
        tpid_angle_layout.addWidget(self.tpid_angle_limit, 0, 8)
        self.tpid_angle_send = QPushButton("发送")
        self.tpid_angle_send.clicked.connect(self.send_tpid_angle)
        tpid_angle_layout.addWidget(self.tpid_angle_send, 0, 9)
        tpid_main_layout.addLayout(tpid_angle_layout)
        
        # 轮速环 PID (内环)
        tpid_wheel_layout = QGridLayout()
        tpid_wheel_layout.addWidget(QLabel("轮速环(内):"), 0, 0)
        tpid_wheel_layout.addWidget(QLabel("Kp:"), 0, 1)
        self.tpid_wheel_kp = QDoubleSpinBox()
        self.tpid_wheel_kp.setRange(0, 100)
        self.tpid_wheel_kp.setSingleStep(0.1)
        self.tpid_wheel_kp.setDecimals(4)
        self.tpid_wheel_kp.setValue(0.5)
        tpid_wheel_layout.addWidget(self.tpid_wheel_kp, 0, 2)
        tpid_wheel_layout.addWidget(QLabel("Ki:"), 0, 3)
        self.tpid_wheel_ki = QDoubleSpinBox()
        self.tpid_wheel_ki.setRange(0, 10)
        self.tpid_wheel_ki.setSingleStep(0.01)
        self.tpid_wheel_ki.setDecimals(4)
        self.tpid_wheel_ki.setValue(0.01)
        tpid_wheel_layout.addWidget(self.tpid_wheel_ki, 0, 4)
        tpid_wheel_layout.addWidget(QLabel("Kd:"), 0, 5)
        self.tpid_wheel_kd = QDoubleSpinBox()
        self.tpid_wheel_kd.setRange(0, 10)
        self.tpid_wheel_kd.setSingleStep(0.001)
        self.tpid_wheel_kd.setDecimals(4)
        self.tpid_wheel_kd.setValue(0.0)
        tpid_wheel_layout.addWidget(self.tpid_wheel_kd, 0, 6)
        tpid_wheel_layout.addWidget(QLabel("Limit:"), 0, 7)
        self.tpid_wheel_limit = QDoubleSpinBox()
        self.tpid_wheel_limit.setRange(0.1, 100)
        self.tpid_wheel_limit.setSingleStep(1.0)
        self.tpid_wheel_limit.setDecimals(1)
        self.tpid_wheel_limit.setValue(15.0)
        self.tpid_wheel_limit.setToolTip("轮速环输出限幅 (最大扭矩, Nm)")
        tpid_wheel_layout.addWidget(self.tpid_wheel_limit, 0, 8)
        self.tpid_wheel_send = QPushButton("发送")
        self.tpid_wheel_send.clicked.connect(self.send_tpid_wheel)
        tpid_wheel_layout.addWidget(self.tpid_wheel_send, 0, 9)
        tpid_main_layout.addLayout(tpid_wheel_layout)
        
        # 角速度阻尼 PID (Gyro damping)
        tpid_gyro_layout = QGridLayout()
        tpid_gyro_layout.addWidget(QLabel("角速度阻尼:"), 0, 0)
        tpid_gyro_layout.addWidget(QLabel("Kp:"), 0, 1)
        self.tpid_gyro_kp = QDoubleSpinBox()
        self.tpid_gyro_kp.setRange(0, 10)
        self.tpid_gyro_kp.setSingleStep(0.001)
        self.tpid_gyro_kp.setDecimals(4)
        self.tpid_gyro_kp.setValue(0.1)
        tpid_gyro_layout.addWidget(self.tpid_gyro_kp, 0, 2)
        tpid_gyro_layout.addWidget(QLabel("Ki:"), 0, 3)
        self.tpid_gyro_ki = QDoubleSpinBox()
        self.tpid_gyro_ki.setRange(0, 1)
        self.tpid_gyro_ki.setSingleStep(0.001)
        self.tpid_gyro_ki.setDecimals(4)
        self.tpid_gyro_ki.setValue(0.0)
        tpid_gyro_layout.addWidget(self.tpid_gyro_ki, 0, 4)
        tpid_gyro_layout.addWidget(QLabel("Kd:"), 0, 5)
        self.tpid_gyro_kd = QDoubleSpinBox()
        self.tpid_gyro_kd.setRange(0, 1)
        self.tpid_gyro_kd.setSingleStep(0.0001)
        self.tpid_gyro_kd.setDecimals(6)
        self.tpid_gyro_kd.setValue(0.0)
        tpid_gyro_layout.addWidget(self.tpid_gyro_kd, 0, 6)
        tpid_gyro_layout.addWidget(QLabel("Limit:"), 0, 7)
        self.tpid_gyro_limit = QDoubleSpinBox()
        self.tpid_gyro_limit.setRange(0.1, 100)
        self.tpid_gyro_limit.setSingleStep(1.0)
        self.tpid_gyro_limit.setDecimals(1)
        self.tpid_gyro_limit.setValue(10.0)
        self.tpid_gyro_limit.setToolTip("角速度阻尼环输出限幅")
        tpid_gyro_layout.addWidget(self.tpid_gyro_limit, 0, 8)
        self.tpid_gyro_send = QPushButton("发送")
        self.tpid_gyro_send.clicked.connect(self.send_tpid_gyro)
        tpid_gyro_layout.addWidget(self.tpid_gyro_send, 0, 9)
        tpid_main_layout.addLayout(tpid_gyro_layout)
        
        # 位移环 PID (最外环) + 使能开关
        tpid_dist_layout = QGridLayout()
        tpid_dist_layout.addWidget(QLabel("位移环(最外):"), 0, 0)
        tpid_dist_layout.addWidget(QLabel("Kp:"), 0, 1)
        self.tpid_dist_kp = QDoubleSpinBox()
        self.tpid_dist_kp.setRange(0, 100)
        self.tpid_dist_kp.setSingleStep(0.1)
        self.tpid_dist_kp.setDecimals(4)
        self.tpid_dist_kp.setValue(2.0)
        tpid_dist_layout.addWidget(self.tpid_dist_kp, 0, 2)
        tpid_dist_layout.addWidget(QLabel("Ki:"), 0, 3)
        self.tpid_dist_ki = QDoubleSpinBox()
        self.tpid_dist_ki.setRange(0, 10)
        self.tpid_dist_ki.setSingleStep(0.001)
        self.tpid_dist_ki.setDecimals(4)
        self.tpid_dist_ki.setValue(0.001)
        tpid_dist_layout.addWidget(self.tpid_dist_ki, 0, 4)
        tpid_dist_layout.addWidget(QLabel("Kd:"), 0, 5)
        self.tpid_dist_kd = QDoubleSpinBox()
        self.tpid_dist_kd.setRange(0, 10)
        self.tpid_dist_kd.setSingleStep(0.001)
        self.tpid_dist_kd.setDecimals(4)
        self.tpid_dist_kd.setValue(0.1)
        tpid_dist_layout.addWidget(self.tpid_dist_kd, 0, 6)
        tpid_dist_layout.addWidget(QLabel("Limit:"), 0, 7)
        self.tpid_dist_limit = QDoubleSpinBox()
        self.tpid_dist_limit.setRange(0.001, 100)
        self.tpid_dist_limit.setSingleStep(0.1)
        self.tpid_dist_limit.setDecimals(3)
        self.tpid_dist_limit.setValue(10.0)
        self.tpid_dist_limit.setToolTip("位移环输出限幅 (最大速度修正)")
        tpid_dist_layout.addWidget(self.tpid_dist_limit, 0, 8)
        self.tpid_dist_send = QPushButton("发送")
        self.tpid_dist_send.clicked.connect(self.send_tpid_distance)
        tpid_dist_layout.addWidget(self.tpid_dist_send, 0, 9)
        tpid_main_layout.addLayout(tpid_dist_layout)
        
        # 位移环使能开关
        dist_en_layout = QHBoxLayout()
        dist_en_layout.addWidget(QLabel("位移环:"))
        self.tpid_dist_enable_btn = QPushButton("位移环: ON")
        self.tpid_dist_enable_btn.setCheckable(True)
        self.tpid_dist_enable_btn.setChecked(True)
        self.tpid_dist_enable_btn.setToolTip("开启后位移环生效，松开遥杆时机器人会回到停车位置\n"
                                              "关闭时位移环不参与控制，等同三环PID")
        self.tpid_dist_enable_btn.setStyleSheet("""
            QPushButton { padding: 4px 12px; border-radius: 4px; 
                          background: #555; color: #ccc; font-weight: bold; }
            QPushButton:checked { background: #2196F3; color: white; }
        """)
        self.tpid_dist_enable_btn.clicked.connect(self.send_tpid_disten)
        dist_en_layout.addWidget(self.tpid_dist_enable_btn)
        dist_en_layout.addStretch()
        tpid_main_layout.addLayout(dist_en_layout)
        
        # 轮速环模式 + 增益 + 操作
        tpid_ctrl_layout = QHBoxLayout()
        tpid_ctrl_layout.addWidget(QLabel("轮速环模式:"))
        self.tpid_wmode_combo = QComboBox()
        self.tpid_wmode_combo.addItem("速度直驱 (0)", 0)
        self.tpid_wmode_combo.addItem("扭矩PID (1)", 1)
        tpid_ctrl_layout.addWidget(self.tpid_wmode_combo)
        self.tpid_wmode_btn = QPushButton("切换")
        self.tpid_wmode_btn.clicked.connect(self.send_tpid_wmode)
        tpid_ctrl_layout.addWidget(self.tpid_wmode_btn)
        
        tpid_ctrl_layout.addWidget(QLabel("指令增益:"))
        self.tpid_gain = QDoubleSpinBox()
        self.tpid_gain.setRange(0, 999999)
        self.tpid_gain.setSingleStep(1000)
        self.tpid_gain.setDecimals(1)
        self.tpid_gain.setValue(8000.0)
        self.tpid_gain.setToolTip("speed_cmd_gain: target_speed × 此增益 → 速度外环输入\n三环模式建议 5000~10000 (角度环输出是速度，量级比双环大)")
        tpid_ctrl_layout.addWidget(self.tpid_gain)
        self.tpid_gain_btn = QPushButton("发送")
        self.tpid_gain_btn.clicked.connect(self.send_tpid_gain)
        tpid_ctrl_layout.addWidget(self.tpid_gain_btn)
        
        tpid_ctrl_layout.addStretch()
        
        self.tpid_status_btn = QPushButton("📊 状态")
        self.tpid_status_btn.clicked.connect(lambda: self.send_cmd("balance tpid status"))
        tpid_ctrl_layout.addWidget(self.tpid_status_btn)
        
        self.tpid_reset_btn = QPushButton("🔄 重置")
        self.tpid_reset_btn.clicked.connect(lambda: self.send_cmd("balance tpid reset"))
        tpid_ctrl_layout.addWidget(self.tpid_reset_btn)
        
        tpid_main_layout.addLayout(tpid_ctrl_layout)
        
        # 轮速 WMA 滤波器开关 (12点加权滑动平均)
        wma_layout = QHBoxLayout()
        wma_layout.addWidget(QLabel("轮速滤波:"))
        self.wma_toggle_btn = QPushButton("WMA 滤波: OFF")
        self.wma_toggle_btn.setCheckable(True)
        self.wma_toggle_btn.setChecked(False)
        self.wma_toggle_btn.setToolTip("12点加权滑动平均滤波器 (用于双环/三环PID的轮速输入)\n"
                                        "权重线性递增 w=[1,2,...,12]，群延迟~6ms @500Hz\n"
                                        "对编码器噪声有良好抑制效果")
        self.wma_toggle_btn.setStyleSheet("""
            QPushButton { padding: 4px 12px; border-radius: 4px; 
                          background: #555; color: #ccc; font-weight: bold; }
            QPushButton:checked { background: #4CAF50; color: white; }
        """)
        self.wma_toggle_btn.clicked.connect(self.toggle_wma_filter)
        wma_layout.addWidget(self.wma_toggle_btn)
        wma_layout.addStretch()
        tpid_main_layout.addLayout(wma_layout)
        
        tpid_group.setLayout(tpid_main_layout)
        layout.addWidget(tpid_group)
        
        # ========== 调试输出控制 ==========
        debug_group = QGroupBox("🔧 实时调试输出")
        debug_group.setStyleSheet(SS("QGroupBox { color: #9C27B0; }"))
        debug_layout = QHBoxLayout()
        
        debug_desc = QLabel("开启后实时打印 PID 内部状态 (P/I/D分量、误差、输出)")
        debug_desc.setStyleSheet(SS("color: #888; font-size: 10px;"))
        debug_layout.addWidget(debug_desc)
        
        debug_layout.addStretch()
        
        self.debug_on_btn = QPushButton("🟢 开启调试")
        self.debug_on_btn.setStyleSheet(SS("background-color: #4CAF50; color: white;"))
        self.debug_on_btn.clicked.connect(lambda: self.send_cmd("balance debug on"))
        debug_layout.addWidget(self.debug_on_btn)
        
        self.debug_off_btn = QPushButton("🔴 关闭调试")
        self.debug_off_btn.setStyleSheet(SS("background-color: #f44336; color: white;"))
        self.debug_off_btn.clicked.connect(lambda: self.send_cmd("balance debug off"))
        debug_layout.addWidget(self.debug_off_btn)
        
        debug_layout.addWidget(QLabel("频率:"))
        self.debug_div = QSpinBox()
        self.debug_div.setRange(1, 255)
        self.debug_div.setValue(50)
        self.debug_div.setToolTip("分频系数 (1~255)，输出频率 = 200Hz / 分频")
        debug_layout.addWidget(self.debug_div)
        
        self.debug_div_btn = QPushButton("设置")
        self.debug_div_btn.clicked.connect(self.send_debug_div)
        debug_layout.addWidget(self.debug_div_btn)
        
        self.debug_status_btn = QPushButton("📊 状态")
        self.debug_status_btn.clicked.connect(lambda: self.send_cmd("balance debug"))
        debug_layout.addWidget(self.debug_status_btn)
        
        debug_group.setLayout(debug_layout)
        layout.addWidget(debug_group)
        
        layout.addStretch()
    
    def send_cmd(self, cmd):
        if self.parent_window and self.parent_window.is_connected():
            self.parent_window.send_command(cmd)
    
    def set_mode(self, mode):
        self.send_cmd(f"balance mode {mode}")
        if mode == "pid":
            self.mode_label.setText("当前模式: 双环 PID (扭矩)")
            self.mode_label.setStyleSheet(SS("font-size: 14px; font-weight: bold; color: #4CAF50;"))
        elif mode == "spid":
            self.mode_label.setText("当前模式: 单环 PID (速度)")
            self.mode_label.setStyleSheet(SS("font-size: 14px; font-weight: bold; color: #FF9800;"))
        elif mode == "tpid":
            self.mode_label.setText("当前模式: 四环 PID")
            self.mode_label.setStyleSheet(SS("font-size: 14px; font-weight: bold; color: #9C27B0;"))
        else:
            self.mode_label.setText("当前模式: LQR")
            self.mode_label.setStyleSheet(SS("font-size: 14px; font-weight: bold; color: #2196F3;"))
    
    def set_loop_order(self, order):
        """设置环路顺序并更新UI"""
        self.send_cmd(f"balance dpid order {order}")
        self.update_loop_order_ui(order)
    
    def update_loop_order_ui(self, order):
        """根据环路顺序更新所有相关UI标签"""
        self.loop_order = order
        if order == 1:  # SPEED_FIRST
            self.order_label.setText("当前: 速度优先 (SF)")
            self.order_label.setStyleSheet(SS("font-size: 13px; font-weight: bold; color: #FF9800;"))
            self.order_sf_btn.setStyleSheet(SS("background-color: #FF9800; color: white;"))
            self.order_af_btn.setStyleSheet(SS(""))
            self.desc_label.setText(
                "控制架构: 速度环(外环) → 角度环(内环) → 扭矩输出\n"
                "速度环: 0-轮速 → 目标倾角 | 角度环: 倾角误差 → 扭矩"
            )
            self.angle_group.setTitle("🎯 角度环 PID (内环: 倾角误差 → 扭矩)")
            self.speed_group.setTitle("🔄 速度环 PID (外环: 0 - 轮速 → 目标倾角)")
        else:  # ANGLE_FIRST
            self.order_label.setText("当前: 角度优先 (AF)")
            self.order_label.setStyleSheet(SS("font-size: 13px; font-weight: bold; color: #4CAF50;"))
            self.order_af_btn.setStyleSheet(SS("background-color: #4CAF50; color: white;"))
            self.order_sf_btn.setStyleSheet(SS(""))
            self.desc_label.setText(
                "控制架构: 角度环(外环) → 速度环(内环) → 扭矩输出\n"
                "角度环: pitch → 目标速度 | 速度环: 速度误差 → 扭矩"
            )
            self.angle_group.setTitle("🎯 角度环 PID (外环: Pitch → 目标速度)")
            self.speed_group.setTitle("🔄 速度环 PID (内环: 速度误差 → 扭矩)")
    
    def send_angle_pid(self):
        kp = self.angle_kp.value()
        ki = self.angle_ki.value()
        kd = self.angle_kd.value()
        self.send_cmd(f"balance dpid angle {kp} {ki} {kd}")
    
    def send_speed_pid(self):
        kp = self.speed_kp.value()
        ki = self.speed_ki.value()
        kd = self.speed_kd.value()
        self.send_cmd(f"balance dpid speed {kp} {ki} {kd}")
    
    def send_dpid_gyro(self):
        """发送双环 PID 角速度阻尼参数"""
        kp = self.dpid_gyro_kp.value()
        ki = self.dpid_gyro_ki.value()
        kd = self.dpid_gyro_kd.value()
        self.send_cmd(f"balance dpid gyro {kp} {ki} {kd}")
    
    def send_speed_cmd_gain(self):
        gain = self.speed_cmd_gain.value()
        self.send_cmd(f"balance dpid gain {gain}")
    
    def send_spid_params(self):
        """发送单环 PID 参数"""
        kp = self.spid_kp.value()
        ki = self.spid_ki.value()
        kd = self.spid_kd.value()
        self.send_cmd(f"balance spid angle {kp} {ki} {kd}")
    
    def send_spid_limit(self):
        """发送单环 PID 输出限幅"""
        limit = self.spid_limit.value()
        self.send_cmd(f"balance spid limit {limit}")
    
    def send_tpid_speed(self):
        """发送三环 PID 速度环参数"""
        kp = self.tpid_speed_kp.value()
        ki = self.tpid_speed_ki.value()
        kd = self.tpid_speed_kd.value()
        limit = self.tpid_speed_limit.value()
        self.send_cmd(f"balance tpid speed {kp} {ki} {kd}")
        self.send_cmd(f"balance tpid limit speed {limit}")
    
    def send_tpid_angle(self):
        """发送三环 PID 角度环参数"""
        kp = self.tpid_angle_kp.value()
        ki = self.tpid_angle_ki.value()
        kd = self.tpid_angle_kd.value()
        limit = self.tpid_angle_limit.value()
        self.send_cmd(f"balance tpid angle {kp} {ki} {kd}")
        self.send_cmd(f"balance tpid limit angle {limit}")
    
    def send_tpid_wheel(self):
        """发送三环 PID 轮速环参数"""
        kp = self.tpid_wheel_kp.value()
        ki = self.tpid_wheel_ki.value()
        kd = self.tpid_wheel_kd.value()
        limit = self.tpid_wheel_limit.value()
        self.send_cmd(f"balance tpid wheel {kp} {ki} {kd}")
        self.send_cmd(f"balance tpid limit wheel {limit}")
    
    def send_tpid_gyro(self):
        """发送三环 PID 角速度阻尼参数"""
        kp = self.tpid_gyro_kp.value()
        ki = self.tpid_gyro_ki.value()
        kd = self.tpid_gyro_kd.value()
        limit = self.tpid_gyro_limit.value()
        self.send_cmd(f"balance tpid gyro {kp} {ki} {kd}")
        self.send_cmd(f"balance tpid limit gyro {limit}")
    
    def send_tpid_distance(self):
        """发送四环 PID 位移环参数"""
        kp = self.tpid_dist_kp.value()
        ki = self.tpid_dist_ki.value()
        kd = self.tpid_dist_kd.value()
        limit = self.tpid_dist_limit.value()
        self.send_cmd(f"balance tpid distance {kp} {ki} {kd}")
        self.send_cmd(f"balance tpid limit distance {limit}")
    
    def send_tpid_disten(self):
        """切换位移环使能"""
        enabled = self.tpid_dist_enable_btn.isChecked()
        self.send_cmd(f"balance tpid disten {1 if enabled else 0}")
        self.tpid_dist_enable_btn.setText(f"位移环: {'ON' if enabled else 'OFF'}")
    
    def send_tpid_wmode(self):
        """发送三环 PID 轮速环模式切换"""
        wmode = self.tpid_wmode_combo.currentData()
        self.send_cmd(f"balance tpid wmode {wmode}")
    
    def send_tpid_gain(self):
        """发送三环 PID 速度指令增益"""
        gain = self.tpid_gain.value()
        self.send_cmd(f"balance tpid gain {gain}")
    
    def toggle_wma_filter(self):
        """切换轮速 WMA 滤波器开关"""
        enabled = self.wma_toggle_btn.isChecked()
        self.send_cmd(f"balance wma {'on' if enabled else 'off'}")
        self.wma_toggle_btn.setText(f"WMA 滤波: {'ON' if enabled else 'OFF'}")
    
    def send_debug_div(self):
        """发送调试输出分频系数"""
        div = self.debug_div.value()
        self.send_cmd(f"balance debug div {div}")
    
    def send_zero(self):
        zero = self.zero_input.value()
        self.send_cmd(f"balance dpid zero {zero}")
    
    def apply_preset(self, a_kp, a_ki, a_kd, s_kp, s_ki, s_kd):
        """应用预设参数"""
        self.angle_kp.setValue(a_kp)
        self.angle_ki.setValue(a_ki)
        self.angle_kd.setValue(a_kd)
        self.speed_kp.setValue(s_kp)
        self.speed_ki.setValue(s_ki)
        self.speed_kd.setValue(s_kd)
        self.send_angle_pid()
        self.send_speed_pid()
    
    def toggle_auto_refresh(self, state):
        if state == Qt.Checked:
            self.refresh_timer.start(500)
        else:
            self.refresh_timer.stop()
    
    def update_status(self, angle_err, target_speed, speed_err, torque):
        """更新状态显示 (由主窗口解析数据后调用)"""
        self.angle_err_label.setText(f"{angle_err:.2f}°")
        self.target_speed_label.setText(f"{target_speed:.2f} rad/s")
        self.speed_err_label.setText(f"{speed_err:.2f}")
        self.torque_label.setText(f"{torque:.2f} Nm")
    
    def update_dpid_debug(self, pitch, err, rate, angle_p, angle_i, angle_d, tgt_spd, 
                          spd_err, speed_p, speed_i, speed_d, torque):
        """更新双环 PID 调试数据 - 角度优先模式 (解析 [DPID-AF] 输出)"""
        self.dpid_pitch_label.setText(f"{pitch:.2f}°")
        self.angle_err_label.setText(f"{err:.2f}°")
        self.dpid_rate_label.setText(f"{rate:.1f}°/s")
        self.dpid_angle_p.setText(f"{angle_p:.2f}")
        self.dpid_angle_i.setText(f"{angle_i:.3f}")
        self.dpid_angle_d.setText(f"{angle_d:.3f}")
        self.target_speed_label.setText(f"{tgt_spd:.2f} rad/s")
        self.speed_err_label.setText(f"{spd_err:.2f}")
        self.dpid_speed_p.setText(f"{speed_p:.2f}")
        self.dpid_speed_i.setText(f"{speed_i:.3f}")
        self.dpid_speed_d.setText(f"{speed_d:.3f}")
        self.torque_label.setText(f"{torque:.3f} Nm")
        self.debug_update_label.setText(f"调试数据: 双环 PID (角度优先) ✓")
        self.debug_update_label.setStyleSheet(SS("color: #4CAF50; font-size: 10px;"))
    
    def update_dpid_debug_sf(self, pitch, spd, spd_err, speed_p, speed_i, speed_d, tgt_pitch,
                              angle_err, angle_p, angle_i, angle_d, torque):
        """更新双环 PID 调试数据 - 速度优先模式 (解析 [DPID-SF] 输出)"""
        self.dpid_pitch_label.setText(f"{pitch:.2f}°")
        self.dpid_rate_label.setText(f"spd={spd:.2f}")
        # 速度环(外环)数据 → 显示在"直立环"行
        self.angle_err_label.setText(f"{spd_err:.2f}")
        self.dpid_angle_p.setText(f"{speed_p:.2f}")
        self.dpid_angle_i.setText(f"{speed_i:.3f}")
        self.dpid_angle_d.setText(f"{speed_d:.3f}")
        self.target_speed_label.setText(f"{tgt_pitch:.2f}° (目标倾角)")
        # 角度环(内环)数据 → 显示在"速度环"行
        self.speed_err_label.setText(f"{angle_err:.2f}")
        self.dpid_speed_p.setText(f"{angle_p:.2f}")
        self.dpid_speed_i.setText(f"{angle_i:.3f}")
        self.dpid_speed_d.setText(f"{angle_d:.3f}")
        self.torque_label.setText(f"{torque:.3f} Nm")
        self.debug_update_label.setText(f"调试数据: 双环 PID (速度优先) ✓")
        self.debug_update_label.setStyleSheet(SS("color: #FF9800; font-size: 10px;"))
    
    def update_spid_debug(self, pitch, err, rate, p, i, d, speed, rpm):
        """更新单环 PID 调试数据 (解析 [SPID] 输出)"""
        self.spid_pitch_label.setText(f"{pitch:.2f}°")
        self.spid_err_label.setText(f"{err:.2f}°")
        self.spid_rate_label.setText(f"{rate:.1f}°/s")
        self.spid_p.setText(f"{p:.2f}")
        self.spid_i.setText(f"{i:.3f}")
        self.spid_d.setText(f"{d:.3f}")
        self.spid_speed_label.setText(f"{speed:.2f} rad/s")
        self.spid_rpm_label.setText(f"({rpm:.0f} rpm)")
        self.debug_update_label.setText(f"调试数据: 单环 PID ✓")
        self.debug_update_label.setStyleSheet(SS("color: #FF9800; font-size: 10px;"))
    
    def update_tpid_debug(self, pitch, spd, spd_err, pitch_tgt, ang_err, whl_tgt, whl_err, out, wmode,
                          dist_err=None, dist_corr=None):
        """更新四环 PID 调试数据 (解析 [TPID] 输出)"""
        self.tpid_pitch_label.setText(f"pitch: {pitch:.2f}°")
        self.tpid_wspd_label.setText(f"spd: {spd:.2f}")
        self.tpid_wmode_label.setText(f"模式: {'速度' if wmode == 'spd' else '扭矩'}")
        # 位移环
        if dist_err is not None and dist_corr is not None:
            self.tpid_dist_err.setText(f"{dist_err:.3f}")
            self.tpid_dist_corr.setText(f"{dist_corr:.3f}")
            self.tpid_dist_row_label.setStyleSheet(SS("color: #00ff88; font-weight: bold;"))
        else:
            self.tpid_dist_err.setText("--")
            self.tpid_dist_corr.setText("--")
            self.tpid_dist_row_label.setStyleSheet(SS("color: #666;"))
        self.tpid_spd_err.setText(f"{spd_err:.2f}")
        self.tpid_pitch_tgt.setText(f"{pitch_tgt:.2f}°")
        self.tpid_ang_err.setText(f"{ang_err:.2f}")
        self.tpid_whl_tgt.setText(f"{whl_tgt:.2f}")
        self.tpid_whl_err.setText(f"{whl_err:.2f}")
        self.tpid_out.setText(f"{out:.3f}")
        dist_tag = " +位移" if dist_err is not None else ""
        self.debug_update_label.setText(f"调试数据: 四环 PID [{wmode}]{dist_tag} ✓")
        self.debug_update_label.setStyleSheet(SS("color: #9C27B0; font-size: 10px;"))
    
    def update_mode(self, mode):
        """更新模式显示 (由主窗口解析数据后调用)"""
        if mode == "DUAL_PID":
            self.mode_label.setText("当前模式: 双环 PID (扭矩)")
            self.mode_label.setStyleSheet(SS("font-size: 14px; font-weight: bold; color: #4CAF50;"))
        elif mode == "SINGLE_PID":
            self.mode_label.setText("当前模式: 单环 PID (速度)")
            self.mode_label.setStyleSheet(SS("font-size: 14px; font-weight: bold; color: #FF9800;"))
        elif mode == "TRIPLE_PID":
            self.mode_label.setText("当前模式: 四环 PID")
            self.mode_label.setStyleSheet(SS("font-size: 14px; font-weight: bold; color: #9C27B0;"))
        elif mode == "CAR":
            self.mode_label.setText("当前模式: 小车模式")
            self.mode_label.setStyleSheet(SS("font-size: 14px; font-weight: bold; color: #FF5722;"))
        else:
            self.mode_label.setText("当前模式: LQR")
            self.mode_label.setStyleSheet(SS("font-size: 14px; font-weight: bold; color: #2196F3;"))


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
        title_style = SS("font-size: 14px; font-weight: bold; color: #888;")
        value_style = SS("font-size: 24px; font-weight: bold; color: #00ccff;")
        unit_style = SS("font-size: 12px; color: #888;")
        
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
        self.yaw_angle_error.setStyleSheet(SS("font-size: 24px; font-weight: bold; color: #ffcc00;"))
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
        self.yaw_output.setStyleSheet(SS("font-size: 24px; font-weight: bold; color: #ff8800;"))
        self.yaw_output.setAlignment(Qt.AlignCenter)
        status_layout.addWidget(self.yaw_output, row, 0)
        
        self.yaw_rate = QLabel("--")
        self.yaw_rate.setStyleSheet(value_style)
        self.yaw_rate.setAlignment(Qt.AlignCenter)
        status_layout.addWidget(self.yaw_rate, row, 1)
        
        self.yaw_holding_status = QLabel("--")
        self.yaw_holding_status.setStyleSheet(SS("font-size: 18px; font-weight: bold; color: #00aaff;"))
        self.yaw_holding_status.setAlignment(Qt.AlignCenter)
        status_layout.addWidget(self.yaw_holding_status, row, 2)
        
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)
        
        # ========== 诊断区 ==========
        diag_group = QGroupBox("💡 实时诊断")
        diag_layout = QVBoxLayout()
        
        self.yaw_diagnosis = QLabel("等待数据...")
        self.yaw_diagnosis.setStyleSheet(SS("font-size: 16px; color: #aaa; padding: 10px;"))
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
        self.yaw_enable_btn.setStyleSheet(SS("font-size: 14px; padding: 10px 20px; background-color: #44aa44;"))
        self.yaw_enable_btn.clicked.connect(lambda: self.send_cmd("balance loop Y on"))
        ctrl_layout.addWidget(self.yaw_enable_btn)
        
        self.yaw_disable_btn = QPushButton("关闭 YAW 环")
        self.yaw_disable_btn.setStyleSheet(SS("font-size: 14px; padding: 10px 20px; background-color: #aa4444;"))
        self.yaw_disable_btn.clicked.connect(lambda: self.send_cmd("balance loop Y off"))
        ctrl_layout.addWidget(self.yaw_disable_btn)
        
        # 重置 YAW 角度
        self.yaw_reset_btn = QPushButton("重置目标角度")
        self.yaw_reset_btn.setStyleSheet(SS("font-size: 14px; padding: 10px 20px; background-color: #4488aa;"))
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
        lbl.setStyleSheet(SS("font-weight: bold;"))
        pid_layout.addWidget(lbl, row, 0)
        
        pid_layout.addWidget(QLabel("P:"), row, 1)
        self.yaw_angle_p = QDoubleSpinBox()
        self.yaw_angle_p.setRange(0, 100)
        self.yaw_angle_p.setDecimals(4)
        self.yaw_angle_p.setSingleStep(0.001)
        pid_layout.addWidget(self.yaw_angle_p, row, 2)
        
        pid_layout.addWidget(QLabel("I:"), row, 3)
        self.yaw_angle_i = QDoubleSpinBox()
        self.yaw_angle_i.setRange(0, 100)
        self.yaw_angle_i.setDecimals(4)
        self.yaw_angle_i.setSingleStep(0.001)
        pid_layout.addWidget(self.yaw_angle_i, row, 4)
        
        pid_layout.addWidget(QLabel("D:"), row, 5)
        self.yaw_angle_d = QDoubleSpinBox()
        self.yaw_angle_d.setRange(0, 100)
        self.yaw_angle_d.setDecimals(5)
        self.yaw_angle_d.setSingleStep(0.0001)
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
        lbl.setStyleSheet(SS("font-weight: bold;"))
        pid_layout.addWidget(lbl, row, 0)
        
        pid_layout.addWidget(QLabel("P:"), row, 1)
        self.yaw_gyro_p = QDoubleSpinBox()
        self.yaw_gyro_p.setRange(0, 100)
        self.yaw_gyro_p.setDecimals(4)
        self.yaw_gyro_p.setSingleStep(0.0001)
        pid_layout.addWidget(self.yaw_gyro_p, row, 2)
        
        pid_layout.addWidget(QLabel("I:"), row, 3)
        self.yaw_gyro_i = QDoubleSpinBox()
        self.yaw_gyro_i.setRange(0, 100)
        self.yaw_gyro_i.setDecimals(4)
        self.yaw_gyro_i.setSingleStep(0.001)
        pid_layout.addWidget(self.yaw_gyro_i, row, 4)
        
        pid_layout.addWidget(QLabel("D:"), row, 5)
        self.yaw_gyro_d = QDoubleSpinBox()
        self.yaw_gyro_d.setRange(0, 100)
        self.yaw_gyro_d.setDecimals(5)
        self.yaw_gyro_d.setSingleStep(0.0001)
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
        help_text.setStyleSheet(SS("font-size: 12px; color: #aaa;"))
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
            self.yaw_angle_error.setStyleSheet(SS("font-size: 24px; font-weight: bold; color: #00ff00;"))
        elif abs(error) < 5.0:
            self.yaw_angle_error.setStyleSheet(SS("font-size: 24px; font-weight: bold; color: #ffcc00;"))
        else:
            self.yaw_angle_error.setStyleSheet(SS("font-size: 24px; font-weight: bold; color: #ff4444;"))
        
        # YAW 输出 (根据大小设置颜色)
        self.yaw_output.setText(f"{output:.3f}")
        if abs(output) < 0.1:
            self.yaw_output.setStyleSheet(SS("font-size: 24px; font-weight: bold; color: #00ff00;"))
        elif abs(output) < 0.5:
            self.yaw_output.setStyleSheet(SS("font-size: 24px; font-weight: bold; color: #ff8800;"))
        else:
            self.yaw_output.setStyleSheet(SS("font-size: 24px; font-weight: bold; color: #ff4444;"))
        
        # 保持模式状态
        if holding:
            self.yaw_holding_status.setText("🔒 方向保持")
            self.yaw_holding_status.setStyleSheet(SS("font-size: 18px; font-weight: bold; color: #00aaff;"))
        else:
            self.yaw_holding_status.setText("🔄 角速度跟踪")
            self.yaw_holding_status.setStyleSheet(SS("font-size: 18px; font-weight: bold; color: #ffaa00;"))
        
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
# 关节电机监控面板
# ============================================================================
class JointMonitorPanel(QWidget):
    """关节电机实时监控面板 - 显示两条腿的角度/速度/电流波形
    
    6个波形区域 (2行3列):
    - 髋关节角度 (左蓝/右红)  | 膝关节角度 (左蓝/右红)  | 数值显示
    - 髋关节速度 (左蓝/右红, 虚线=原始/实线=滤波后)  | 膝关节速度 | 数值显示
    - 髋关节电流 (左蓝/右红)  | 膝关节电流 (左蓝/右红)  | 数值显示
    """
    
    PLOT_BUFFER = 500  # 波形缓存长度
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.plots = {}      # 波形区域
        self.curves = {}     # 曲线
        self.data = {}       # 数据缓存
        self.checkboxes = {} # 显示开关
        self.value_labels = {} # 数值显示
        self.init_ui()
    
    def send_cmd(self, cmd):
        if self.parent_window and self.parent_window.is_connected():
            self.parent_window.send_command(cmd)
            self.parent_window.log(f"发送: {cmd}")
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # ========== 控制栏 ==========
        ctrl_layout = QHBoxLayout()
        
        self.stream_on_btn = QPushButton("📈 开启关节数据流")
        self.stream_on_btn.setStyleSheet(SS("background-color: #4CAF50; color: white; padding: 8px 15px; font-size: 13px;"))
        self.stream_on_btn.clicked.connect(lambda: self.send_cmd("balance joint on"))
        ctrl_layout.addWidget(self.stream_on_btn)
        
        self.stream_off_btn = QPushButton("⏹️ 关闭")
        self.stream_off_btn.setStyleSheet(SS("background-color: #f44336; color: white; padding: 8px 15px; font-size: 13px;"))
        self.stream_off_btn.clicked.connect(lambda: self.send_cmd("balance joint off"))
        ctrl_layout.addWidget(self.stream_off_btn)
        
        ctrl_layout.addWidget(QLabel("  "))
        
        # 显示开关
        ctrl_layout.addWidget(QLabel("显示: "))
        
        plot_defs = [
            ('hip_pos',  '髋关节角度'),
            ('knee_pos', '膝关节角度'),
            ('hip_spd',  '髋关节速度'),
            ('knee_spd', '膝关节速度'),
            ('hip_cur',  '髋关节电流'),
            ('knee_cur', '膝关节电流'),
        ]
        for key, label in plot_defs:
            cb = QCheckBox(label)
            cb.setChecked(True)
            cb.setStyleSheet(SS("font-size: 11px;"))
            cb.toggled.connect(lambda checked, k=key: self._toggle_plot(k, checked))
            self.checkboxes[key] = cb
            ctrl_layout.addWidget(cb)
        
        # 全选/全不选
        all_btn = QPushButton("全选")
        all_btn.setFixedWidth(45)
        all_btn.clicked.connect(lambda: self._set_all_plots(True))
        ctrl_layout.addWidget(all_btn)
        
        none_btn = QPushButton("全不选")
        none_btn.setFixedWidth(55)
        none_btn.clicked.connect(lambda: self._set_all_plots(False))
        ctrl_layout.addWidget(none_btn)
        
        clear_btn = QPushButton("🗑️ 清空")
        clear_btn.setFixedWidth(55)
        clear_btn.clicked.connect(self._clear_all)
        ctrl_layout.addWidget(clear_btn)
        
        ctrl_layout.addStretch()
        layout.addLayout(ctrl_layout)
        
        # ========== 关节速度滤波控制 (双模式: Median / SlewRate) ==========
        filter_layout = QHBoxLayout()
        
        filter_layout.addWidget(QLabel("⚡ 关节速度滤波:"))
        
        self.filter_on_btn = QPushButton("✅ 开启")
        self.filter_on_btn.setStyleSheet(SS("background-color: #FF9800; color: white; padding: 5px 10px; font-size: 12px;"))
        self.filter_on_btn.clicked.connect(lambda: self.send_cmd("balance joint filter on"))
        filter_layout.addWidget(self.filter_on_btn)
        
        self.filter_off_btn = QPushButton("❌ 关闭")
        self.filter_off_btn.setStyleSheet(SS("background-color: #757575; color: white; padding: 5px 10px; font-size: 12px;"))
        self.filter_off_btn.clicked.connect(lambda: self.send_cmd("balance joint filter off"))
        filter_layout.addWidget(self.filter_off_btn)
        
        filter_layout.addWidget(QLabel("  │ "))
        
        # 模式切换按钮 (Median / SlewRate)
        self.mode_median_btn = QPushButton("📊 中值滤波")
        self.mode_median_btn.setCheckable(True)
        self.mode_median_btn.setChecked(False)
        self.mode_median_btn.setStyleSheet(SS(
            "QPushButton { font-size: 12px; padding: 5px 12px; border: 2px solid #555; border-radius: 4px; }"
            "QPushButton:checked { background-color: #2196F3; color: white; border-color: #1976D2; }"
        ))
        self.mode_median_btn.clicked.connect(lambda: self._set_filter_mode(0))
        filter_layout.addWidget(self.mode_median_btn)
        
        self.mode_slew_btn = QPushButton("📈 限幅滤波")
        self.mode_slew_btn.setCheckable(True)
        self.mode_slew_btn.setChecked(True)
        self.mode_slew_btn.setStyleSheet(SS(
            "QPushButton { font-size: 12px; padding: 5px 12px; border: 2px solid #555; border-radius: 4px; }"
            "QPushButton:checked { background-color: #FF9800; color: white; border-color: #F57C00; }"
        ))
        self.mode_slew_btn.clicked.connect(lambda: self._set_filter_mode(1))
        filter_layout.addWidget(self.mode_slew_btn)
        
        filter_layout.addWidget(QLabel("  │ "))
        
        # 中值滤波窗口大小 (默认隐藏, 因为默认是限幅模式)
        self.median_window_label = QLabel("窗口:")
        self.median_window_label.setVisible(False)
        filter_layout.addWidget(self.median_window_label)
        self.median_window_input = QSpinBox()
        self.median_window_input.setRange(3, 9)
        self.median_window_input.setSingleStep(2)
        self.median_window_input.setValue(3)
        self.median_window_input.setStyleSheet(SS("font-size: 13px; padding: 4px; min-width: 50px;"))
        self.median_window_input.setVisible(False)
        filter_layout.addWidget(self.median_window_input)
        
        self.median_window_set_btn = QPushButton("📤 设置窗口")
        self.median_window_set_btn.setStyleSheet(SS("background-color: #2196F3; color: white; padding: 5px 10px; font-size: 12px;"))
        self.median_window_set_btn.clicked.connect(self._set_median_window)
        self.median_window_set_btn.setVisible(False)
        filter_layout.addWidget(self.median_window_set_btn)
        
        # 限幅滤波 Rate 参数 (默认可见)
        self.rate_label = QLabel("Rate(°/s²):")
        self.rate_label.setVisible(True)
        filter_layout.addWidget(self.rate_label)
        self.rate_input = QDoubleSpinBox()
        self.rate_input.setRange(10, 100000)
        self.rate_input.setDecimals(0)
        self.rate_input.setSingleStep(100)
        self.rate_input.setValue(3000)
        self.rate_input.setStyleSheet(SS("font-size: 13px; padding: 4px; min-width: 80px;"))
        self.rate_input.setVisible(True)
        filter_layout.addWidget(self.rate_input)
        
        self.rate_set_btn = QPushButton("📤 设置Rate")
        self.rate_set_btn.setStyleSheet(SS("background-color: #FF9800; color: white; padding: 5px 10px; font-size: 12px;"))
        self.rate_set_btn.setVisible(True)
        self.rate_set_btn.clicked.connect(self._set_rate)
        filter_layout.addWidget(self.rate_set_btn)
        
        self.filter_query_btn = QPushButton("🔍 查询")
        self.filter_query_btn.clicked.connect(lambda: self.send_cmd("balance joint"))
        filter_layout.addWidget(self.filter_query_btn)
        
        filter_layout.addStretch()
        layout.addLayout(filter_layout)
        
        # ========== 波形区域 (3行2列) ==========
        plot_grid = QGridLayout()
        
        # 波形配置: (key, title, y_label, row, col)
        plot_configs = [
            ('hip_pos',  '髋关节角度 (°)',  '角度 (°)',  0, 0),
            ('knee_pos', '膝关节角度 (°)',  '角度 (°)',  0, 1),
            ('hip_spd',  '髋关节速度 (°/s)', '速度 (°/s)', 1, 0),
            ('knee_spd', '膝关节速度 (°/s)', '速度 (°/s)', 1, 1),
            ('hip_cur',  '髋关节电流 (A)',   '电流 (A)',   2, 0),
            ('knee_cur', '膝关节电流 (A)',   '电流 (A)',   2, 1),
        ]
        
        for key, title, y_label, row, col in plot_configs:
            # 创建波形区域
            pw = pg.PlotWidget()
            pw.setBackground('#1a1a2e')
            pw.showGrid(x=True, y=True, alpha=0.3)
            pw.setTitle(title, color='w', size='10pt')
            pw.setLabel('left', y_label)
            pw.setLabel('bottom', '采样点')
            pw.setMinimumHeight(160)
            pw.addLegend(offset=(60, 5))
            
            # 速度类型: 增加滤波后曲线 (实线=滤波后, 虚线=原始)
            is_spd = key in ('hip_spd', 'knee_spd')
            
            if is_spd:
                # 原始速度: 虚线
                left_curve = pw.plot(pen=pg.mkPen(color='#4fc3f7', width=1, style=Qt.DashLine), name='左原始')
                right_curve = pw.plot(pen=pg.mkPen(color='#ef5350', width=1, style=Qt.DashLine), name='右原始')
                # 滤波后速度: 实线
                left_filtered_curve = pw.plot(pen=pg.mkPen(color='#4fc3f7', width=2), name='左滤波')
                right_filtered_curve = pw.plot(pen=pg.mkPen(color='#ef5350', width=2), name='右滤波')
            else:
                # 非速度: 只有两条实线
                left_curve = pw.plot(pen=pg.mkPen(color='#4fc3f7', width=2), name='左腿')
                right_curve = pw.plot(pen=pg.mkPen(color='#ef5350', width=2), name='右腿')
            
            self.plots[key] = pw
            self.curves[key] = {'left': left_curve, 'right': right_curve}
            self.data[key] = {'left': [], 'right': []}
            
            if is_spd:
                self.curves[key]['left_filtered'] = left_filtered_curve
                self.curves[key]['right_filtered'] = right_filtered_curve
                self.data[key]['left_filtered'] = []
                self.data[key]['right_filtered'] = []
            
            # 包装: 波形 + 数值显示
            container = QVBoxLayout()
            container.addWidget(pw)
            
            # 数值行
            val_layout = QHBoxLayout()
            left_lbl = QLabel("左: --")
            left_lbl.setStyleSheet(SS("color: #4fc3f7; font-size: 12px; font-weight: bold;"))
            right_lbl = QLabel("右: --")
            right_lbl.setStyleSheet(SS("color: #ef5350; font-size: 12px; font-weight: bold;"))
            val_layout.addWidget(left_lbl)
            val_layout.addWidget(right_lbl)
            
            if is_spd:
                left_f_lbl = QLabel("左滤波: --")
                left_f_lbl.setStyleSheet(SS("color: #81d4fa; font-size: 11px;"))
                right_f_lbl = QLabel("右滤波: --")
                right_f_lbl.setStyleSheet(SS("color: #ef9a9a; font-size: 11px;"))
                val_layout.addWidget(left_f_lbl)
                val_layout.addWidget(right_f_lbl)
                self.value_labels[key] = {'left': left_lbl, 'right': right_lbl,
                                          'left_filtered': left_f_lbl, 'right_filtered': right_f_lbl}
            else:
                self.value_labels[key] = {'left': left_lbl, 'right': right_lbl}
            
            val_layout.addStretch()
            container.addLayout(val_layout)
            
            plot_grid.addLayout(container, row, col)
        
        layout.addLayout(plot_grid)
    
    def _set_rate(self):
        """设置关节速度限幅滤波 Rate"""
        rate = int(self.rate_input.value())
        self.send_cmd(f"balance joint rate {rate}")
    
    def _set_filter_mode(self, mode):
        """切换关节速度滤波模式: 0=Median, 1=SlewRate"""
        self.mode_median_btn.setChecked(mode == 0)
        self.mode_slew_btn.setChecked(mode == 1)
        # 显示/隐藏对应参数控件
        self.median_window_label.setVisible(mode == 0)
        self.median_window_input.setVisible(mode == 0)
        self.median_window_set_btn.setVisible(mode == 0)
        self.rate_label.setVisible(mode == 1)
        self.rate_input.setVisible(mode == 1)
        self.rate_set_btn.setVisible(mode == 1)
        # 发送命令
        self.send_cmd(f"balance joint mode {mode}")
    
    def _set_median_window(self):
        """设置中值滤波窗口大小"""
        w = self.median_window_input.value()
        # 强制奇数
        if w % 2 == 0:
            w += 1
            self.median_window_input.setValue(w)
        self.send_cmd(f"balance joint window {w}")
    
    def _toggle_plot(self, key, visible):
        """切换波形显示/隐藏"""
        if key in self.plots:
            self.plots[key].setVisible(visible)
    
    def _set_all_plots(self, checked):
        """全选/全不选"""
        for cb in self.checkboxes.values():
            cb.setChecked(checked)
    
    def _clear_all(self):
        """清空所有波形数据"""
        for key in self.data:
            for sub_key in self.data[key]:
                self.data[key][sub_key].clear()
            for curve_key in self.curves[key]:
                self.curves[key][curve_key].setData([])
            self.value_labels[key]['left'].setText("左: --")
            self.value_labels[key]['right'].setText("右: --")
            if 'left_filtered' in self.value_labels[key]:
                self.value_labels[key]['left_filtered'].setText("左滤波: --")
                self.value_labels[key]['right_filtered'].setText("右滤波: --")
    
    def update_joint_data(self, lh_pos, lh_spd, lh_cur, lh_spd_f,
                          lk_pos, lk_spd, lk_cur, lk_spd_f,
                          rh_pos, rh_spd, rh_cur, rh_spd_f,
                          rk_pos, rk_spd, rk_cur, rk_spd_f):
        """更新关节电机数据 (由 process_serial_data 调用)
        
        每个关节4个值: pos, spd(raw), cur, spd_filtered
        """
        # 数据映射: (key, left_val, right_val, fmt, left_f, right_f)
        updates = [
            ('hip_pos',  lh_pos, rh_pos, '%.1f°',  None, None),
            ('knee_pos', lk_pos, rk_pos, '%.1f°',  None, None),
            ('hip_spd',  lh_spd, rh_spd, '%.0f',   lh_spd_f, rh_spd_f),
            ('knee_spd', lk_spd, rk_spd, '%.0f',   lk_spd_f, rk_spd_f),
            ('hip_cur',  lh_cur, rh_cur, '%.3fA',  None, None),
            ('knee_cur', lk_cur, rk_cur, '%.3fA',  None, None),
        ]
        
        for key, lval, rval, fmt, lval_f, rval_f in updates:
            buf = self.data[key]
            buf['left'].append(lval)
            buf['right'].append(rval)
            
            has_filtered = lval_f is not None
            if has_filtered:
                buf['left_filtered'].append(lval_f)
                buf['right_filtered'].append(rval_f)
            
            # 限制缓存长度
            if len(buf['left']) > self.PLOT_BUFFER:
                buf['left'] = buf['left'][-self.PLOT_BUFFER:]
                buf['right'] = buf['right'][-self.PLOT_BUFFER:]
                if has_filtered:
                    buf['left_filtered'] = buf['left_filtered'][-self.PLOT_BUFFER:]
                    buf['right_filtered'] = buf['right_filtered'][-self.PLOT_BUFFER:]
            
            # 更新波形 (仅在可见时)
            if self.checkboxes[key].isChecked():
                self.curves[key]['left'].setData(buf['left'])
                self.curves[key]['right'].setData(buf['right'])
                if has_filtered:
                    self.curves[key]['left_filtered'].setData(buf['left_filtered'])
                    self.curves[key]['right_filtered'].setData(buf['right_filtered'])
            
            # 更新数值
            self.value_labels[key]['left'].setText(f"左: {fmt % lval}")
            self.value_labels[key]['right'].setText(f"右: {fmt % rval}")
            if has_filtered:
                self.value_labels[key]['left_filtered'].setText(f"左滤波: {fmt % lval_f}")
                self.value_labels[key]['right_filtered'].setText(f"右滤波: {fmt % rval_f}")


# ============================================================================
# VMC 调试面板
# ============================================================================
class VMCDebugPanel(QWidget):
    """VMC (虚拟模型控制) 调试面板 - 腿部力控制调参"""
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # ========== VMC 开关控制 ==========
        ctrl_group = QGroupBox("🦿 VMC 控制")
        ctrl_layout = QHBoxLayout()
        
        self.vmc_on_btn = QPushButton("✅ 开启 VMC")
        self.vmc_on_btn.setStyleSheet(SS("background-color: #4CAF50; color: white; padding: 10px 20px; font-size: 14px;"))
        self.vmc_on_btn.clicked.connect(lambda: self.send_cmd("balance vmc on"))
        ctrl_layout.addWidget(self.vmc_on_btn)
        
        self.vmc_off_btn = QPushButton("❌ 关闭 VMC")
        self.vmc_off_btn.setStyleSheet(SS("background-color: #f44336; color: white; padding: 10px 20px; font-size: 14px;"))
        self.vmc_off_btn.clicked.connect(lambda: self.send_cmd("balance vmc off"))
        ctrl_layout.addWidget(self.vmc_off_btn)
        
        self.vmc_status_btn = QPushButton("📊 查看状态")
        self.vmc_status_btn.setStyleSheet(SS("padding: 10px 20px; font-size: 14px;"))
        self.vmc_status_btn.clicked.connect(lambda: self.send_cmd("balance vmc status"))
        ctrl_layout.addWidget(self.vmc_status_btn)
        
        self.stream_on_btn = QPushButton("📈 开启数据流")
        self.stream_on_btn.setStyleSheet(SS("background-color: #2196F3; color: white; padding: 10px 15px;"))
        self.stream_on_btn.setToolTip("开启 VMC 实时数据流，用于 UI 监控")
        self.stream_on_btn.clicked.connect(lambda: self.send_cmd("balance vmc stream on"))
        ctrl_layout.addWidget(self.stream_on_btn)
        
        self.stream_off_btn = QPushButton("⏹️ 关闭数据流")
        self.stream_off_btn.setStyleSheet(SS("padding: 10px 15px;"))
        self.stream_off_btn.clicked.connect(lambda: self.send_cmd("balance vmc stream off"))
        ctrl_layout.addWidget(self.stream_off_btn)
        
        ctrl_layout.addStretch()
        ctrl_group.setLayout(ctrl_layout)
        layout.addWidget(ctrl_group)
        
        # ========== 坐标系选择 ==========
        coord_group = QGroupBox("📐 坐标系选择")
        coord_layout = QHBoxLayout()
        
        coord_layout.addWidget(QLabel("VMC 坐标系:"))
        
        self.coord_world_btn = QPushButton("🌍 世界坐标系 (World)")
        self.coord_world_btn.setToolTip("世界坐标系: 控制 F_x (水平力) 和 F_y (垂直力)")
        self.coord_world_btn.clicked.connect(lambda: self.send_cmd("balance vmc coord world"))
        coord_layout.addWidget(self.coord_world_btn)
        
        self.coord_body_btn = QPushButton("🤖 机身坐标系 (Body)")
        self.coord_body_btn.setToolTip("机身坐标系: 控制 F_L (腿长方向力) 和 F_α (摆角方向力矩)")
        self.coord_body_btn.clicked.connect(lambda: self.send_cmd("balance vmc coord body"))
        coord_layout.addWidget(self.coord_body_btn)
        
        self.coord_status = QLabel("当前: --")
        self.coord_status.setStyleSheet(SS("font-weight: bold; color: #00ccff;"))
        coord_layout.addWidget(self.coord_status)
        
        coord_layout.addStretch()
        coord_group.setLayout(coord_layout)
        layout.addWidget(coord_group)
        
        # ========== 速度估计方法选择 ==========
        diff_group = QGroupBox("🔬 速度估计方法 (VMC Kd 项)")
        diff_layout = QHBoxLayout()
        
        diff_layout.addWidget(QLabel("dL/dα 速度来源:"))
        
        self.diff_jacobian_btn = QPushButton("📊 雅可比 × 关节速度 (Jacobian)")
        self.diff_jacobian_btn.setCheckable(True)
        self.diff_jacobian_btn.setChecked(True)
        self.diff_jacobian_btn.setStyleSheet(SS(
            "QPushButton { font-size: 13px; padding: 8px 14px; border: 2px solid #555; border-radius: 6px; }"
            "QPushButton:checked { background-color: #2196F3; color: white; border-color: #1976D2; }"
        ))
        self.diff_jacobian_btn.setToolTip("通过雅可比矩阵将关节角速度映射到工作空间速度\n零延迟、解析精确，但受关节速度噪声影响")
        self.diff_jacobian_btn.clicked.connect(lambda: self._set_diff_method('jacobian'))
        diff_layout.addWidget(self.diff_jacobian_btn)
        
        self.diff_numeric_btn = QPushButton("📈 位置数值微分 (Numeric)")
        self.diff_numeric_btn.setCheckable(True)
        self.diff_numeric_btn.setChecked(False)
        self.diff_numeric_btn.setStyleSheet(SS(
            "QPushButton { font-size: 13px; padding: 8px 14px; border: 2px solid #555; border-radius: 6px; }"
            "QPushButton:checked { background-color: #FF9800; color: white; border-color: #F57C00; }"
        ))
        self.diff_numeric_btn.setToolTip("通过 FK 位置差分计算速度\n一帧延迟，但不受关节速度噪声影响，信号更干净")
        self.diff_numeric_btn.clicked.connect(lambda: self._set_diff_method('numeric'))
        diff_layout.addWidget(self.diff_numeric_btn)
        
        self.diff_status_label = QLabel("当前: Jacobian")
        self.diff_status_label.setStyleSheet(SS("font-weight: bold; color: #2196F3;"))
        diff_layout.addWidget(self.diff_status_label)
        
        diff_layout.addStretch()
        diff_group.setLayout(diff_layout)
        layout.addWidget(diff_group)
        
        # ========== 世界坐标系参数 (K_vx, K_y, D_y) ==========
        world_group = QGroupBox("🌍 世界坐标系参数 (World Coord)")
        world_layout = QGridLayout()
        
        # K_vx - 速度反馈增益
        world_layout.addWidget(QLabel("K_vx (速度反馈):"), 0, 0)
        self.kvx_input = QDoubleSpinBox()
        self.kvx_input.setRange(-100, 100)
        self.kvx_input.setDecimals(4)
        self.kvx_input.setSingleStep(0.01)
        self.kvx_input.setValue(0.0)
        world_layout.addWidget(self.kvx_input, 0, 1)
        self.kvx_set_btn = QPushButton("设置")
        self.kvx_set_btn.clicked.connect(lambda: self.send_cmd(f"balance vmc kvx {self.kvx_input.value()}"))
        world_layout.addWidget(self.kvx_set_btn, 0, 2)
        
        # K_y - 高度刚度
        world_layout.addWidget(QLabel("K_y (高度刚度 N/m):"), 1, 0)
        self.ky_input = QDoubleSpinBox()
        self.ky_input.setRange(0, 5000)
        self.ky_input.setDecimals(3)
        self.ky_input.setSingleStep(10)
        self.ky_input.setValue(0.0)  # 默认 0
        world_layout.addWidget(self.ky_input, 1, 1)
        self.ky_set_btn = QPushButton("设置")
        self.ky_set_btn.clicked.connect(lambda: self.send_cmd(f"balance vmc ky {self.ky_input.value()}"))
        world_layout.addWidget(self.ky_set_btn, 1, 2)
        
        # D_y - 高度阻尼
        world_layout.addWidget(QLabel("D_y (高度阻尼 Ns/m):"), 2, 0)
        self.dy_input = QDoubleSpinBox()
        self.dy_input.setRange(0, 500)
        self.dy_input.setDecimals(3)
        self.dy_input.setSingleStep(1)
        self.dy_input.setValue(0.0)  # 默认 0
        world_layout.addWidget(self.dy_input, 2, 1)
        self.dy_set_btn = QPushButton("设置")
        self.dy_set_btn.clicked.connect(lambda: self.send_cmd(f"balance vmc dy {self.dy_input.value()}"))
        world_layout.addWidget(self.dy_set_btn, 2, 2)
        
        world_group.setLayout(world_layout)
        layout.addWidget(world_group)
        
        # ========== 机身坐标系参数 (K_L, D_L, K_α, D_α) ==========
        body_group = QGroupBox("🤖 机身坐标系参数 (Body Coord)")
        body_layout = QGridLayout()
        
        # K_L - 腿长刚度
        body_layout.addWidget(QLabel("K_L (腿长刚度 N/m):"), 0, 0)
        self.kl_input = QDoubleSpinBox()
        self.kl_input.setRange(0, 5000)
        self.kl_input.setDecimals(3)
        self.kl_input.setSingleStep(10)
        self.kl_input.setValue(0.0)  # 默认 0
        body_layout.addWidget(self.kl_input, 0, 1)
        self.kl_set_btn = QPushButton("设置")
        self.kl_set_btn.clicked.connect(lambda: self.send_cmd(f"balance vmc kl {self.kl_input.value()}"))
        body_layout.addWidget(self.kl_set_btn, 0, 2)
        
        # D_L - 腿长阻尼
        body_layout.addWidget(QLabel("D_L (腿长阻尼 Ns/m):"), 1, 0)
        self.dl_input = QDoubleSpinBox()
        self.dl_input.setRange(0, 500)
        self.dl_input.setDecimals(3)
        self.dl_input.setSingleStep(1)
        self.dl_input.setValue(0.0)  # 默认 0
        body_layout.addWidget(self.dl_input, 1, 1)
        self.dl_set_btn = QPushButton("设置")
        self.dl_set_btn.clicked.connect(lambda: self.send_cmd(f"balance vmc dl {self.dl_input.value()}"))
        body_layout.addWidget(self.dl_set_btn, 1, 2)
        
        # K_alpha - 摆角刚度
        body_layout.addWidget(QLabel("K_α (摆角刚度 Nm/rad):"), 2, 0)
        self.ka_input = QDoubleSpinBox()
        self.ka_input.setRange(0, 500)
        self.ka_input.setDecimals(4)
        self.ka_input.setSingleStep(0.1)
        self.ka_input.setValue(0.0)  # 默认 0
        body_layout.addWidget(self.ka_input, 2, 1)
        self.ka_set_btn = QPushButton("设置")
        self.ka_set_btn.clicked.connect(lambda: self.send_cmd(f"balance vmc ka {self.ka_input.value()}"))
        body_layout.addWidget(self.ka_set_btn, 2, 2)
        
        # D_alpha - 摆角阻尼
        body_layout.addWidget(QLabel("D_α (摆角阻尼 Nm·s/rad):"), 3, 0)
        self.da_input = QDoubleSpinBox()
        self.da_input.setRange(0, 50)
        self.da_input.setDecimals(4)
        self.da_input.setSingleStep(0.05)
        self.da_input.setValue(0.0)  # 默认 0
        body_layout.addWidget(self.da_input, 3, 1)
        self.da_set_btn = QPushButton("设置")
        self.da_set_btn.clicked.connect(lambda: self.send_cmd(f"balance vmc da {self.da_input.value()}"))
        body_layout.addWidget(self.da_set_btn, 3, 2)
        
        body_group.setLayout(body_layout)
        layout.addWidget(body_group)
        
        # ========== 通用参数 ==========
        common_group = QGroupBox("⚙️ 通用参数")
        common_layout = QGridLayout()
        
        # 重力补偿系数
        common_layout.addWidget(QLabel("重力补偿系数 (0~1.5):"), 0, 0)
        self.gc_input = QDoubleSpinBox()
        self.gc_input.setRange(0, 1.5)
        self.gc_input.setDecimals(2)
        self.gc_input.setSingleStep(0.05)
        self.gc_input.setValue(0.0)  # 默认 0
        common_layout.addWidget(self.gc_input, 0, 1)
        self.gc_set_btn = QPushButton("设置")
        self.gc_set_btn.clicked.connect(lambda: self.send_cmd(f"balance vmc gc {self.gc_input.value()}"))
        common_layout.addWidget(self.gc_set_btn, 0, 2)
        
        # 机器人质量
        common_layout.addWidget(QLabel("机器人质量 (kg):"), 1, 0)
        self.mass_input = QDoubleSpinBox()
        self.mass_input.setRange(0.1, 50)
        self.mass_input.setDecimals(3)
        self.mass_input.setSingleStep(0.01)
        self.mass_input.setValue(1.0)  # 默认 1.0 kg
        common_layout.addWidget(self.mass_input, 1, 1)
        self.mass_set_btn = QPushButton("设置")
        self.mass_set_btn.clicked.connect(lambda: self.send_cmd(f"balance vmc mass {self.mass_input.value()}"))
        common_layout.addWidget(self.mass_set_btn, 1, 2)
        
        # 目标腿高
        common_layout.addWidget(QLabel("目标腿高 (m):"), 2, 0)
        self.height_input = QDoubleSpinBox()
        self.height_input.setRange(0.05, 0.3)
        self.height_input.setDecimals(3)
        self.height_input.setSingleStep(0.005)
        self.height_input.setValue(0.14)  # 默认 0.14m (与 g_vmc_target_y 一致)
        common_layout.addWidget(self.height_input, 2, 1)
        self.height_set_btn = QPushButton("设置")
        self.height_set_btn.clicked.connect(lambda: self.send_cmd(f"balance vmc height {self.height_input.value()}"))
        common_layout.addWidget(self.height_set_btn, 2, 2)
        
        # 目标速度
        common_layout.addWidget(QLabel("目标速度 (m/s):"), 3, 0)
        self.vx_input = QDoubleSpinBox()
        self.vx_input.setRange(-2, 2)
        self.vx_input.setDecimals(3)
        self.vx_input.setSingleStep(0.01)
        self.vx_input.setValue(0.0)
        common_layout.addWidget(self.vx_input, 3, 1)
        self.vx_set_btn = QPushButton("设置")
        self.vx_set_btn.clicked.connect(lambda: self.send_cmd(f"balance vmc vx {self.vx_input.value()}"))
        common_layout.addWidget(self.vx_set_btn, 3, 2)
        
        common_group.setLayout(common_layout)
        layout.addWidget(common_group)
        
        # ========== Pitch 控制参数 ==========
        pitch_group = QGroupBox("📐 Pitch 俯仰控制 (补偿)")
        pitch_layout = QVBoxLayout()
        
        # Pitch 开关
        pitch_ctrl_layout = QHBoxLayout()
        
        self.pitch_on_btn = QPushButton("✅ 开启 Pitch 补偿")
        self.pitch_on_btn.setStyleSheet(SS("background-color: #4CAF50; color: white; padding: 8px 15px;"))
        self.pitch_on_btn.clicked.connect(lambda: self.send_cmd("balance vmc pitch on"))
        pitch_ctrl_layout.addWidget(self.pitch_on_btn)
        
        self.pitch_off_btn = QPushButton("❌ 关闭 Pitch 补偿")
        self.pitch_off_btn.setStyleSheet(SS("background-color: #f44336; color: white; padding: 8px 15px;"))
        self.pitch_off_btn.clicked.connect(lambda: self.send_cmd("balance vmc pitch off"))
        pitch_ctrl_layout.addWidget(self.pitch_off_btn)
        
        pitch_ctrl_layout.addStretch()
        pitch_layout.addLayout(pitch_ctrl_layout)
        
        # Pitch 参数
        pitch_param_layout = QGridLayout()
        
        # Kp
        pitch_param_layout.addWidget(QLabel("Pitch Kp:"), 0, 0)
        self.pitch_kp_input = QDoubleSpinBox()
        self.pitch_kp_input.setRange(0, 100)
        self.pitch_kp_input.setDecimals(4)
        self.pitch_kp_input.setSingleStep(0.01)
        self.pitch_kp_input.setValue(0.0)  # 默认 0
        pitch_param_layout.addWidget(self.pitch_kp_input, 0, 1)
        self.pitch_kp_set_btn = QPushButton("设置")
        self.pitch_kp_set_btn.clicked.connect(lambda: self.send_cmd(f"balance vmc pitch kp {self.pitch_kp_input.value()}"))
        pitch_param_layout.addWidget(self.pitch_kp_set_btn, 0, 2)
        
        # Kd
        pitch_param_layout.addWidget(QLabel("Pitch Kd:"), 1, 0)
        self.pitch_kd_input = QDoubleSpinBox()
        self.pitch_kd_input.setRange(0, 50)
        self.pitch_kd_input.setDecimals(4)
        self.pitch_kd_input.setSingleStep(0.005)
        self.pitch_kd_input.setValue(0.0)  # 默认 0
        pitch_param_layout.addWidget(self.pitch_kd_input, 1, 1)
        self.pitch_kd_set_btn = QPushButton("设置")
        self.pitch_kd_set_btn.clicked.connect(lambda: self.send_cmd(f"balance vmc pitch kd {self.pitch_kd_input.value()}"))
        pitch_param_layout.addWidget(self.pitch_kd_set_btn, 1, 2)
        
        # Target
        pitch_param_layout.addWidget(QLabel("目标角度 (°):"), 2, 0)
        self.pitch_target_input = QDoubleSpinBox()
        self.pitch_target_input.setRange(-30, 30)
        self.pitch_target_input.setDecimals(1)
        self.pitch_target_input.setSingleStep(0.5)
        self.pitch_target_input.setValue(0.0)
        pitch_param_layout.addWidget(self.pitch_target_input, 2, 1)
        self.pitch_target_set_btn = QPushButton("设置")
        self.pitch_target_set_btn.clicked.connect(lambda: self.send_cmd(f"balance vmc pitch target {self.pitch_target_input.value()}"))
        pitch_param_layout.addWidget(self.pitch_target_set_btn, 2, 2)
        
        pitch_layout.addLayout(pitch_param_layout)
        pitch_group.setLayout(pitch_layout)
        layout.addWidget(pitch_group)
        
        # ========== 双腿协调控制 (Leg Sync) ==========
        sync_group = QGroupBox("🔗 双腿协调控制 (Leg Sync)")
        sync_layout = QVBoxLayout()
        
        # 协调控制开关
        sync_ctrl_layout = QHBoxLayout()
        
        self.sync_on_btn = QPushButton("✅ 开启协调控制")
        self.sync_on_btn.setStyleSheet(SS("background-color: #4CAF50; color: white; padding: 8px 15px;"))
        self.sync_on_btn.setToolTip("消除左右腿 body_angle 差异，保持机体姿态一致")
        self.sync_on_btn.clicked.connect(lambda: self.send_cmd("balance vmc sync on"))
        sync_ctrl_layout.addWidget(self.sync_on_btn)
        
        self.sync_off_btn = QPushButton("❌ 关闭协调控制")
        self.sync_off_btn.setStyleSheet(SS("background-color: #f44336; color: white; padding: 8px 15px;"))
        self.sync_off_btn.clicked.connect(lambda: self.send_cmd("balance vmc sync off"))
        sync_ctrl_layout.addWidget(self.sync_off_btn)
        
        self.sync_status_btn = QPushButton("📊 查看状态")
        self.sync_status_btn.clicked.connect(lambda: self.send_cmd("balance vmc sync"))
        sync_ctrl_layout.addWidget(self.sync_status_btn)
        
        sync_ctrl_layout.addStretch()
        sync_layout.addLayout(sync_ctrl_layout)
        
        # 协调控制参数
        sync_param_layout = QGridLayout()
        
        # K_sync - 协调 P 增益
        sync_param_layout.addWidget(QLabel("K_sync (P增益 Nm/rad):"), 0, 0)
        self.sync_kp_input = QDoubleSpinBox()
        self.sync_kp_input.setRange(0, 50)
        self.sync_kp_input.setDecimals(4)
        self.sync_kp_input.setSingleStep(0.005)
        self.sync_kp_input.setValue(0.0)  # 默认 0
        self.sync_kp_input.setToolTip("角度差比例增益：越大响应越快，但可能震荡")
        sync_param_layout.addWidget(self.sync_kp_input, 0, 1)
        self.sync_kp_set_btn = QPushButton("设置")
        self.sync_kp_set_btn.clicked.connect(lambda: self.send_cmd(f"balance vmc sync kp {self.sync_kp_input.value()}"))
        sync_param_layout.addWidget(self.sync_kp_set_btn, 0, 2)
        
        # D_sync - 协调 D 增益
        sync_param_layout.addWidget(QLabel("D_sync (D增益 Nm·s/rad):"), 1, 0)
        self.sync_kd_input = QDoubleSpinBox()
        self.sync_kd_input.setRange(0, 5)
        self.sync_kd_input.setDecimals(4)
        self.sync_kd_input.setSingleStep(0.005)
        self.sync_kd_input.setValue(0.0)  # 默认 0
        self.sync_kd_input.setToolTip("角速度差阻尼增益：抑制振荡")
        sync_param_layout.addWidget(self.sync_kd_input, 1, 1)
        self.sync_kd_set_btn = QPushButton("设置")
        self.sync_kd_set_btn.clicked.connect(lambda: self.send_cmd(f"balance vmc sync kd {self.sync_kd_input.value()}"))
        sync_param_layout.addWidget(self.sync_kd_set_btn, 1, 2)
        
        sync_layout.addLayout(sync_param_layout)
        
        # 实时状态显示
        sync_status_layout = QGridLayout()
        sync_status_layout.addWidget(QLabel("左腿角度:"), 0, 0)
        self.left_angle_label = QLabel("-- °")
        self.left_angle_label.setStyleSheet(SS("font-weight: bold; color: #00ccff;"))
        sync_status_layout.addWidget(self.left_angle_label, 0, 1)
        
        sync_status_layout.addWidget(QLabel("右腿角度:"), 0, 2)
        self.right_angle_label = QLabel("-- °")
        self.right_angle_label.setStyleSheet(SS("font-weight: bold; color: #00ccff;"))
        sync_status_layout.addWidget(self.right_angle_label, 0, 3)
        
        sync_status_layout.addWidget(QLabel("角度差:"), 1, 0)
        self.angle_diff_label = QLabel("-- °")
        self.angle_diff_label.setStyleSheet(SS("font-weight: bold; color: #ffaa00;"))
        sync_status_layout.addWidget(self.angle_diff_label, 1, 1)
        
        sync_status_layout.addWidget(QLabel("F_sync:"), 1, 2)
        self.f_sync_label = QLabel("-- Nm")
        self.f_sync_label.setStyleSheet(SS("font-weight: bold; color: #ff6600;"))
        sync_status_layout.addWidget(self.f_sync_label, 1, 3)
        
        sync_layout.addLayout(sync_status_layout)
        
        sync_group.setLayout(sync_layout)
        layout.addWidget(sync_group)
        
        # ========== 快捷预设 ==========
        preset_group = QGroupBox("🎯 快捷预设")
        preset_layout = QHBoxLayout()
        
        self.soft_btn = QPushButton("🧸 软腿 (Soft)")
        self.soft_btn.setToolTip("低刚度低阻尼: 适合缓冲/柔软触地")
        self.soft_btn.clicked.connect(lambda: self.send_cmd("balance vmc soft"))
        preset_layout.addWidget(self.soft_btn)
        
        self.stiff_btn = QPushButton("💪 硬腿 (Stiff)")
        self.stiff_btn.setToolTip("高刚度高阻尼: 适合稳定站立/负载")
        self.stiff_btn.clicked.connect(lambda: self.send_cmd("balance vmc stiff"))
        preset_layout.addWidget(self.stiff_btn)
        
        preset_layout.addStretch()
        preset_group.setLayout(preset_layout)
        layout.addWidget(preset_group)
        
        # ========== VMC 实时状态监控 ==========
        monitor_group = QGroupBox("📈 VMC 实时状态")
        monitor_layout = QGridLayout()
        
        # 左腿状态
        monitor_layout.addWidget(QLabel("【左腿】"), 0, 0)
        monitor_layout.addWidget(QLabel("腿长:"), 0, 1)
        self.left_leg_length_label = QLabel("-- m")
        self.left_leg_length_label.setStyleSheet(SS("color: #00ff00;"))
        monitor_layout.addWidget(self.left_leg_length_label, 0, 2)
        monitor_layout.addWidget(QLabel("F_L:"), 0, 3)
        self.left_fl_label = QLabel("-- N")
        self.left_fl_label.setStyleSheet(SS("color: #00ccff;"))
        monitor_layout.addWidget(self.left_fl_label, 0, 4)
        monitor_layout.addWidget(QLabel("F_α:"), 0, 5)
        self.left_fa_label = QLabel("-- Nm")
        self.left_fa_label.setStyleSheet(SS("color: #ffaa00;"))
        monitor_layout.addWidget(self.left_fa_label, 0, 6)
        
        # 右腿状态
        monitor_layout.addWidget(QLabel("【右腿】"), 1, 0)
        monitor_layout.addWidget(QLabel("腿长:"), 1, 1)
        self.right_leg_length_label = QLabel("-- m")
        self.right_leg_length_label.setStyleSheet(SS("color: #00ff00;"))
        monitor_layout.addWidget(self.right_leg_length_label, 1, 2)
        monitor_layout.addWidget(QLabel("F_L:"), 1, 3)
        self.right_fl_label = QLabel("-- N")
        self.right_fl_label.setStyleSheet(SS("color: #00ccff;"))
        monitor_layout.addWidget(self.right_fl_label, 1, 4)
        monitor_layout.addWidget(QLabel("F_α:"), 1, 5)
        self.right_fa_label = QLabel("-- Nm")
        self.right_fa_label.setStyleSheet(SS("color: #ffaa00;"))
        monitor_layout.addWidget(self.right_fa_label, 1, 6)
        
        # 扭矩输出
        monitor_layout.addWidget(QLabel("【扭矩】"), 2, 0)
        monitor_layout.addWidget(QLabel("L髋:"), 2, 1)
        self.left_hip_torque_label = QLabel("-- Nm")
        monitor_layout.addWidget(self.left_hip_torque_label, 2, 2)
        monitor_layout.addWidget(QLabel("L膝:"), 2, 3)
        self.left_knee_torque_label = QLabel("-- Nm")
        monitor_layout.addWidget(self.left_knee_torque_label, 2, 4)
        monitor_layout.addWidget(QLabel("R髋:"), 2, 5)
        self.right_hip_torque_label = QLabel("-- Nm")
        monitor_layout.addWidget(self.right_hip_torque_label, 2, 6)
        monitor_layout.addWidget(QLabel("R膝:"), 2, 7)
        self.right_knee_torque_label = QLabel("-- Nm")
        monitor_layout.addWidget(self.right_knee_torque_label, 2, 8)
        
        monitor_group.setLayout(monitor_layout)
        layout.addWidget(monitor_group)
        
        # ========== VMC 波形显示 (4个波形: 左右 F_α, 左右 F_L) ==========
        waveform_group = QGroupBox("📊 VMC 虚拟力波形 (期望 vs 实际)")
        waveform_layout = QVBoxLayout()
        
        # 波形数据缓冲区
        self.vmc_wave_max_points = 500
        self.vmc_wave_data = {
            'left_F_alpha': [], 'right_F_alpha': [],
            'left_F_L': [], 'right_F_L': [],
            'left_actual_F_alpha': [], 'right_actual_F_alpha': [],
            'left_actual_F_L': [], 'right_actual_F_L': [],
            'time': []
        }
        self.vmc_wave_counter = 0
        
        # 波形网格: 2行2列
        wave_grid = QGridLayout()
        
        # --- 左腿 F_α 波形 ---
        self.pw_left_fa = pg.PlotWidget()
        self.pw_left_fa.setBackground('#1a1a2e')
        self.pw_left_fa.showGrid(x=True, y=True, alpha=0.3)
        self.pw_left_fa.setTitle('左腿 F_α (角度力矩)', color='w', size='10pt')
        self.pw_left_fa.setLabel('left', 'Nm')
        self.pw_left_fa.setLabel('bottom', '采样点')
        self.pw_left_fa.setMinimumHeight(150)
        self.pw_left_fa.addLegend(offset=(60, 5))
        self.curve_left_fa = self.pw_left_fa.plot(pen=pg.mkPen(color='#4fc3f7', width=2), name='期望')
        self.curve_left_fa_actual = self.pw_left_fa.plot(pen=pg.mkPen(color='#ff9800', width=2, style=Qt.DashLine), name='实际')
        wave_grid.addWidget(self.pw_left_fa, 0, 0)
        
        # --- 右腿 F_α 波形 ---
        self.pw_right_fa = pg.PlotWidget()
        self.pw_right_fa.setBackground('#1a1a2e')
        self.pw_right_fa.showGrid(x=True, y=True, alpha=0.3)
        self.pw_right_fa.setTitle('右腿 F_α (角度力矩)', color='w', size='10pt')
        self.pw_right_fa.setLabel('left', 'Nm')
        self.pw_right_fa.setLabel('bottom', '采样点')
        self.pw_right_fa.setMinimumHeight(150)
        self.pw_right_fa.addLegend(offset=(60, 5))
        self.curve_right_fa = self.pw_right_fa.plot(pen=pg.mkPen(color='#ef5350', width=2), name='期望')
        self.curve_right_fa_actual = self.pw_right_fa.plot(pen=pg.mkPen(color='#ff9800', width=2, style=Qt.DashLine), name='实际')
        wave_grid.addWidget(self.pw_right_fa, 0, 1)
        
        # --- 左腿 F_L 波形 (足端支持力) ---
        self.pw_left_fl = pg.PlotWidget()
        self.pw_left_fl.setBackground('#1a1a2e')
        self.pw_left_fl.showGrid(x=True, y=True, alpha=0.3)
        self.pw_left_fl.setTitle('左腿 F_L (足端支持力)', color='w', size='10pt')
        self.pw_left_fl.setLabel('left', 'N')
        self.pw_left_fl.setLabel('bottom', '采样点')
        self.pw_left_fl.setMinimumHeight(150)
        self.pw_left_fl.addLegend(offset=(60, 5))
        self.curve_left_fl = self.pw_left_fl.plot(pen=pg.mkPen(color='#4fc3f7', width=2), name='期望')
        self.curve_left_fl_actual = self.pw_left_fl.plot(pen=pg.mkPen(color='#ff9800', width=2, style=Qt.DashLine), name='实际')
        wave_grid.addWidget(self.pw_left_fl, 1, 0)
        
        # --- 右腿 F_L 波形 (足端支持力) ---
        self.pw_right_fl = pg.PlotWidget()
        self.pw_right_fl.setBackground('#1a1a2e')
        self.pw_right_fl.showGrid(x=True, y=True, alpha=0.3)
        self.pw_right_fl.setTitle('右腿 F_L (足端支持力)', color='w', size='10pt')
        self.pw_right_fl.setLabel('left', 'N')
        self.pw_right_fl.setLabel('bottom', '采样点')
        self.pw_right_fl.setMinimumHeight(150)
        self.pw_right_fl.addLegend(offset=(60, 5))
        self.curve_right_fl = self.pw_right_fl.plot(pen=pg.mkPen(color='#ef5350', width=2), name='期望')
        self.curve_right_fl_actual = self.pw_right_fl.plot(pen=pg.mkPen(color='#ff9800', width=2, style=Qt.DashLine), name='实际')
        wave_grid.addWidget(self.pw_right_fl, 1, 1)
        
        waveform_layout.addLayout(wave_grid)
        
        # 波形控制栏
        wave_ctrl_layout = QHBoxLayout()
        
        self.wave_clear_btn = QPushButton("🗑️ 清空波形")
        self.wave_clear_btn.clicked.connect(self._clear_vmc_waveforms)
        wave_ctrl_layout.addWidget(self.wave_clear_btn)
        
        wave_ctrl_layout.addWidget(QLabel("缓冲点数:"))
        self.wave_points_input = QSpinBox()
        self.wave_points_input.setRange(100, 5000)
        self.wave_points_input.setSingleStep(100)
        self.wave_points_input.setValue(500)
        self.wave_points_input.valueChanged.connect(lambda v: setattr(self, 'vmc_wave_max_points', v))
        wave_ctrl_layout.addWidget(self.wave_points_input)
        
        wave_ctrl_layout.addStretch()
        waveform_layout.addLayout(wave_ctrl_layout)
        
        waveform_group.setLayout(waveform_layout)
        layout.addWidget(waveform_group)
        
        layout.addStretch()
    
    def send_cmd(self, cmd):
        """发送命令到串口"""
        if self.parent_window:
            self.parent_window.send_command(cmd)
    
    def _set_diff_method(self, method):
        """切换 VMC 速度估计方法: jacobian 或 numeric"""
        is_jacobian = (method == 'jacobian')
        self.diff_jacobian_btn.setChecked(is_jacobian)
        self.diff_numeric_btn.setChecked(not is_jacobian)
        if is_jacobian:
            self.diff_status_label.setText("当前: Jacobian")
            self.diff_status_label.setStyleSheet(SS("font-weight: bold; color: #2196F3;"))
        else:
            self.diff_status_label.setText("当前: Numeric")
            self.diff_status_label.setStyleSheet(SS("font-weight: bold; color: #FF9800;"))
        self.send_cmd(f"balance vmc diff {method}")
    
    def update_coord_status(self, coord_type):
        """更新坐标系状态显示"""
        if coord_type == "world":
            self.coord_status.setText("当前: 世界坐标系 (World)")
            self.coord_status.setStyleSheet(SS("font-weight: bold; color: #00ff00;"))
        elif coord_type == "body":
            self.coord_status.setText("当前: 机身坐标系 (Body)")
            self.coord_status.setStyleSheet(SS("font-weight: bold; color: #00ccff;"))
    
    def update_sync_status(self, left_angle, right_angle, angle_diff, f_sync):
        """更新双腿协调控制状态显示"""
        self.left_angle_label.setText(f"{left_angle:.2f} °")
        self.right_angle_label.setText(f"{right_angle:.2f} °")
        self.angle_diff_label.setText(f"{angle_diff:.2f} °")
        self.f_sync_label.setText(f"{f_sync:.3f} Nm")
        
        # 根据角度差大小改变颜色
        if abs(angle_diff) < 1.0:
            self.angle_diff_label.setStyleSheet(SS("font-weight: bold; color: #00ff00;"))  # 绿色 - 良好
        elif abs(angle_diff) < 3.0:
            self.angle_diff_label.setStyleSheet(SS("font-weight: bold; color: #ffaa00;"))  # 黄色 - 警告
        else:
            self.angle_diff_label.setStyleSheet(SS("font-weight: bold; color: #ff4444;"))  # 红色 - 需调整
    
    def update_vmc_monitor(self, data):
        """更新 VMC 实时监控数据
        
        Args:
            data: dict 包含以下键:
                - left_leg_length, right_leg_length
                - left_body_angle, right_body_angle
                - left_F_L, right_F_L
                - left_F_alpha, right_F_alpha
                - left_hip_torque, left_knee_torque
                - right_hip_torque, right_knee_torque
                - angle_diff, f_sync
        """
        if 'left_leg_length' in data:
            self.left_leg_length_label.setText(f"{data['left_leg_length']:.3f} m")
        if 'right_leg_length' in data:
            self.right_leg_length_label.setText(f"{data['right_leg_length']:.3f} m")
        if 'left_F_L' in data:
            self.left_fl_label.setText(f"{data['left_F_L']:.2f} N")
        if 'right_F_L' in data:
            self.right_fl_label.setText(f"{data['right_F_L']:.2f} N")
        if 'left_F_alpha' in data:
            self.left_fa_label.setText(f"{data['left_F_alpha']:.3f} Nm")
        if 'right_F_alpha' in data:
            self.right_fa_label.setText(f"{data['right_F_alpha']:.3f} Nm")
        if 'left_hip_torque' in data:
            self.left_hip_torque_label.setText(f"{data['left_hip_torque']:.3f} Nm")
        if 'left_knee_torque' in data:
            self.left_knee_torque_label.setText(f"{data['left_knee_torque']:.3f} Nm")
        if 'right_hip_torque' in data:
            self.right_hip_torque_label.setText(f"{data['right_hip_torque']:.3f} Nm")
        if 'right_knee_torque' in data:
            self.right_knee_torque_label.setText(f"{data['right_knee_torque']:.3f} Nm")
        
        # 更新协调控制状态
        if 'left_body_angle' in data and 'right_body_angle' in data:
            self.update_sync_status(
                data.get('left_body_angle', 0),
                data.get('right_body_angle', 0),
                data.get('angle_diff', 0),
                data.get('f_sync', 0)
            )
        
        # 更新波形数据
        self.vmc_wave_counter += 1
        self.vmc_wave_data['time'].append(self.vmc_wave_counter)
        self.vmc_wave_data['left_F_alpha'].append(data.get('left_F_alpha', 0.0))
        self.vmc_wave_data['right_F_alpha'].append(data.get('right_F_alpha', 0.0))
        self.vmc_wave_data['left_F_L'].append(data.get('left_F_L', 0.0))
        self.vmc_wave_data['right_F_L'].append(data.get('right_F_L', 0.0))
        self.vmc_wave_data['left_actual_F_alpha'].append(data.get('left_actual_F_alpha', 0.0))
        self.vmc_wave_data['right_actual_F_alpha'].append(data.get('right_actual_F_alpha', 0.0))
        self.vmc_wave_data['left_actual_F_L'].append(data.get('left_actual_F_L', 0.0))
        self.vmc_wave_data['right_actual_F_L'].append(data.get('right_actual_F_L', 0.0))
        
        # 限制缓冲区大小
        max_pts = self.vmc_wave_max_points
        for key in self.vmc_wave_data:
            if len(self.vmc_wave_data[key]) > max_pts:
                self.vmc_wave_data[key] = self.vmc_wave_data[key][-max_pts:]
        
        # 刷新波形曲线 (期望 + 实际)
        t = self.vmc_wave_data['time']
        self.curve_left_fa.setData(t, self.vmc_wave_data['left_F_alpha'])
        self.curve_left_fa_actual.setData(t, self.vmc_wave_data['left_actual_F_alpha'])
        self.curve_right_fa.setData(t, self.vmc_wave_data['right_F_alpha'])
        self.curve_right_fa_actual.setData(t, self.vmc_wave_data['right_actual_F_alpha'])
        self.curve_left_fl.setData(t, self.vmc_wave_data['left_F_L'])
        self.curve_left_fl_actual.setData(t, self.vmc_wave_data['left_actual_F_L'])
        self.curve_right_fl.setData(t, self.vmc_wave_data['right_F_L'])
        self.curve_right_fl_actual.setData(t, self.vmc_wave_data['right_actual_F_L'])
    
    def _clear_vmc_waveforms(self):
        """清空 VMC 波形数据"""
        self.vmc_wave_counter = 0
        for key in self.vmc_wave_data:
            self.vmc_wave_data[key] = []
        self.curve_left_fa.setData([], [])
        self.curve_left_fa_actual.setData([], [])
        self.curve_right_fa.setData([], [])
        self.curve_right_fa_actual.setData([], [])
        self.curve_left_fl.setData([], [])
        self.curve_left_fl_actual.setData([], [])
        self.curve_right_fl.setData([], [])
        self.curve_right_fl_actual.setData([], [])


# ============================================================================
# 电机功率监控面板
# ============================================================================
class MotorPowerPanel(QWidget):
    """电机功率监控面板 - 显示6个电机的电压/电流, 总电流波形和平均电压"""
    
    MOTOR_NAMES = ['左髋', '左膝', '左轮', '右髋', '右膝', '右轮']
    MOTOR_KEYS = ['lh', 'lk', 'lw', 'rh', 'rk', 'rw']
    MOTOR_COLORS = ['#4fc3f7', '#81c784', '#ffb74d', '#ef5350', '#ce93d8', '#fff176']
    
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.mpow_wave_max_points = 500
        self.init_ui()
    
    def init_ui(self):
        layout = QVBoxLayout(self)
        
        # ========== 数据流开关 ==========
        ctrl_group = QGroupBox("⚡ 电机功率监控")
        ctrl_layout = QHBoxLayout()
        
        self.stream_on_btn = QPushButton("📈 开启数据流")
        self.stream_on_btn.setStyleSheet(SS("background-color: #2196F3; color: white; padding: 10px 15px; font-size: 14px;"))
        self.stream_on_btn.clicked.connect(lambda: self.send_cmd("balance mpow on"))
        ctrl_layout.addWidget(self.stream_on_btn)
        
        self.stream_off_btn = QPushButton("⏹️ 关闭数据流")
        self.stream_off_btn.setStyleSheet(SS("padding: 10px 15px; font-size: 14px;"))
        self.stream_off_btn.clicked.connect(lambda: self.send_cmd("balance mpow off"))
        ctrl_layout.addWidget(self.stream_off_btn)
        
        ctrl_layout.addStretch()
        ctrl_group.setLayout(ctrl_layout)
        layout.addWidget(ctrl_group)
        
        # ========== 电压数字显示 (7个: 平均 + 6个电机) ==========
        volt_group = QGroupBox("🔋 电压 (V)")
        volt_layout = QGridLayout()
        
        # 平均电压 (大字体, 高亮)
        volt_layout.addWidget(QLabel("平均电压:"), 0, 0)
        self.avg_volt_label = QLabel("-- V")
        self.avg_volt_label.setStyleSheet(SS("font-size: 22px; font-weight: bold; color: #ffd740; padding: 4px;"))
        volt_layout.addWidget(self.avg_volt_label, 0, 1)
        
        # 6个电机电压
        self.volt_labels = {}
        for i, (name, key) in enumerate(zip(self.MOTOR_NAMES, self.MOTOR_KEYS)):
            col_base = (i % 3) * 2 + 2
            row = i // 3
            lbl = QLabel(f"{name}:")
            lbl.setStyleSheet(SS(f"color: {self.MOTOR_COLORS[i]};"))
            volt_layout.addWidget(lbl, row, col_base)
            val = QLabel("-- V")
            val.setStyleSheet(SS(f"font-size: 14px; font-weight: bold; color: {self.MOTOR_COLORS[i]};"))
            volt_layout.addWidget(val, row, col_base + 1)
            self.volt_labels[key] = val
        
        volt_group.setLayout(volt_layout)
        layout.addWidget(volt_group)
        
        # ========== 总电流数字显示 ==========
        total_group = QGroupBox("⚡ 总电流")
        total_layout = QHBoxLayout()
        total_layout.addWidget(QLabel("电流之和:"))
        self.total_cur_label = QLabel("-- A")
        self.total_cur_label.setStyleSheet(SS("font-size: 22px; font-weight: bold; color: #ff5252; padding: 4px;"))
        total_layout.addWidget(self.total_cur_label)
        total_layout.addStretch()
        total_group.setLayout(total_layout)
        layout.addWidget(total_group)
        
        # ========== 电流波形区域 (7个: 总电流 + 6个电机) ==========
        wave_group = QGroupBox("📊 电流波形 (A)")
        wave_layout = QVBoxLayout()
        
        # 初始化波形数据
        self.mpow_wave_data = {k: [] for k in self.MOTOR_KEYS}
        self.mpow_wave_data['total'] = []
        self.mpow_wave_data['time'] = []
        self.mpow_wave_counter = 0
        
        # 总电流波形 (单独一行, 突出显示)
        self.pw_total_cur = pg.PlotWidget()
        self.pw_total_cur.setBackground('#1a1a2e')
        self.pw_total_cur.showGrid(x=True, y=True, alpha=0.3)
        self.pw_total_cur.setTitle('总电流 (6电机之和)', color='w', size='10pt')
        self.pw_total_cur.setLabel('left', 'A')
        self.pw_total_cur.setLabel('bottom', '采样点')
        self.pw_total_cur.setMinimumHeight(130)
        self.curve_total = self.pw_total_cur.plot(pen=pg.mkPen(color='#ff5252', width=2))
        wave_layout.addWidget(self.pw_total_cur)
        
        # 6个电机电流波形 (2行3列网格)
        motor_grid = QGridLayout()
        self.pw_motors = {}
        self.curve_motors = {}
        for i, (name, key, color) in enumerate(zip(self.MOTOR_NAMES, self.MOTOR_KEYS, self.MOTOR_COLORS)):
            pw = pg.PlotWidget()
            pw.setBackground('#1a1a2e')
            pw.showGrid(x=True, y=True, alpha=0.3)
            pw.setTitle(f'{name} 电流', color='w', size='9pt')
            pw.setLabel('left', 'A')
            pw.setMinimumHeight(110)
            curve = pw.plot(pen=pg.mkPen(color=color, width=2))
            motor_grid.addWidget(pw, i // 3, i % 3)
            self.pw_motors[key] = pw
            self.curve_motors[key] = curve
        
        wave_layout.addLayout(motor_grid)
        
        # 波形控制栏
        wave_ctrl = QHBoxLayout()
        self.wave_clear_btn = QPushButton("🗑️ 清空波形")
        self.wave_clear_btn.clicked.connect(self._clear_waveforms)
        wave_ctrl.addWidget(self.wave_clear_btn)
        
        wave_ctrl.addWidget(QLabel("缓冲点数:"))
        self.wave_points_input = QSpinBox()
        self.wave_points_input.setRange(100, 5000)
        self.wave_points_input.setSingleStep(100)
        self.wave_points_input.setValue(500)
        self.wave_points_input.valueChanged.connect(lambda v: setattr(self, 'mpow_wave_max_points', v))
        wave_ctrl.addWidget(self.wave_points_input)
        
        wave_ctrl.addStretch()
        wave_layout.addLayout(wave_ctrl)
        
        wave_group.setLayout(wave_layout)
        layout.addWidget(wave_group)
        
        layout.addStretch()
    
    def send_cmd(self, cmd):
        if self.parent_window:
            self.parent_window.send_command(cmd)
    
    def update_mpow_data(self, currents, voltages):
        """更新电机功率数据
        
        Args:
            currents: list of 6 floats [LH, LK, LW, RH, RK, RW] in A
            voltages: list of 6 floats [LH, LK, LW, RH, RK, RW] in V
        """
        # 更新电压数字
        valid_volts = [v for v in voltages if v > 0.1]
        avg_volt = sum(valid_volts) / len(valid_volts) if valid_volts else 0.0
        self.avg_volt_label.setText(f"{avg_volt:.2f} V")
        
        for i, key in enumerate(self.MOTOR_KEYS):
            self.volt_labels[key].setText(f"{voltages[i]:.2f} V")
        
        # 更新总电流数字
        total_cur = sum(abs(c) for c in currents)
        self.total_cur_label.setText(f"{total_cur:.3f} A")
        
        # 更新波形数据
        self.mpow_wave_counter += 1
        self.mpow_wave_data['time'].append(self.mpow_wave_counter)
        self.mpow_wave_data['total'].append(total_cur)
        for i, key in enumerate(self.MOTOR_KEYS):
            self.mpow_wave_data[key].append(currents[i])
        
        # 限制缓冲区
        max_pts = self.mpow_wave_max_points
        for k in self.mpow_wave_data:
            if len(self.mpow_wave_data[k]) > max_pts:
                self.mpow_wave_data[k] = self.mpow_wave_data[k][-max_pts:]
        
        # 刷新波形
        t = self.mpow_wave_data['time']
        self.curve_total.setData(t, self.mpow_wave_data['total'])
        for key in self.MOTOR_KEYS:
            self.curve_motors[key].setData(t, self.mpow_wave_data[key])
    
    def _clear_waveforms(self):
        self.mpow_wave_counter = 0
        for k in self.mpow_wave_data:
            self.mpow_wave_data[k] = []
        self.curve_total.setData([], [])
        for key in self.MOTOR_KEYS:
            self.curve_motors[key].setData([], [])


# ============================================================================
# 速度/位移观测器面板
# ============================================================================
class ObserverPanel(QWidget):
    """速度/位移观测器面板 - 卡尔曼滤波融合速度观测"""

    WAVE_KEYS = ['v_raw', 'v_enc', 'v_filt', 'x_filt', 'a_imu']
    WAVE_NAMES = ['原始轮速 (m/s)', '补偿轮速 (m/s)', 'KF速度 (m/s)', 'KF位移 (m)', 'IMU加速度 (m/s²)']
    WAVE_COLORS = ['#ffb74d', '#4fc3f7', '#66bb6a', '#ce93d8', '#ef5350']

    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.obsv_wave_max_points = 500
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)

        # ========== 控制区 ==========
        ctrl_group = QGroupBox("📊 速度/位移观测器 (KF)")
        ctrl_layout = QHBoxLayout()

        self.stream_on_btn = QPushButton("📈 开启数据流")
        self.stream_on_btn.setStyleSheet(SS("background-color: #2196F3; color: white; padding: 10px 15px; font-size: 14px;"))
        self.stream_on_btn.clicked.connect(lambda: self.send_cmd("balance obsv stream on"))
        ctrl_layout.addWidget(self.stream_on_btn)

        self.stream_off_btn = QPushButton("⏹️ 关闭数据流")
        self.stream_off_btn.setStyleSheet(SS("padding: 10px 15px; font-size: 14px;"))
        self.stream_off_btn.clicked.connect(lambda: self.send_cmd("balance obsv stream off"))
        ctrl_layout.addWidget(self.stream_off_btn)

        ctrl_layout.addSpacing(20)

        self.obsv_on_btn = QPushButton("✅ 启用观测器")
        self.obsv_on_btn.setStyleSheet(SS("background-color: #4CAF50; color: white; padding: 10px 15px; font-size: 14px;"))
        self.obsv_on_btn.clicked.connect(lambda: self.send_cmd("balance obsv on"))
        ctrl_layout.addWidget(self.obsv_on_btn)

        self.obsv_off_btn = QPushButton("❌ 禁用观测器")
        self.obsv_off_btn.setStyleSheet(SS("background-color: #f44336; color: white; padding: 10px 15px; font-size: 14px;"))
        self.obsv_off_btn.clicked.connect(lambda: self.send_cmd("balance obsv off"))
        ctrl_layout.addWidget(self.obsv_off_btn)

        self.obsv_reset_btn = QPushButton("🔄 复位")
        self.obsv_reset_btn.setStyleSheet(SS("padding: 10px 15px; font-size: 14px;"))
        self.obsv_reset_btn.clicked.connect(lambda: self.send_cmd("balance obsv reset"))
        ctrl_layout.addWidget(self.obsv_reset_btn)

        ctrl_layout.addStretch()
        ctrl_group.setLayout(ctrl_layout)
        layout.addWidget(ctrl_group)

        # ========== KF参数调节 ==========
        param_group = QGroupBox("🎛️ 卡尔曼参数")
        param_layout = QGridLayout()

        param_layout.addWidget(QLabel("过程噪声 Q_v:"), 0, 0)
        self.qv_input = QDoubleSpinBox()
        self.qv_input.setRange(0.0001, 100.0)
        self.qv_input.setDecimals(4)
        self.qv_input.setSingleStep(0.01)
        self.qv_input.setValue(0.1)
        param_layout.addWidget(self.qv_input, 0, 1)
        self.qv_btn = QPushButton("设置")
        self.qv_btn.clicked.connect(lambda: self.send_cmd(f"balance obsv qv {self.qv_input.value():.4f}"))
        param_layout.addWidget(self.qv_btn, 0, 2)

        param_layout.addWidget(QLabel("过程噪声 Q_a:"), 0, 3)
        self.qa_input = QDoubleSpinBox()
        self.qa_input.setRange(0.0001, 100.0)
        self.qa_input.setDecimals(4)
        self.qa_input.setSingleStep(0.01)
        self.qa_input.setValue(0.1)
        param_layout.addWidget(self.qa_input, 0, 4)
        self.qa_btn = QPushButton("设置")
        self.qa_btn.clicked.connect(lambda: self.send_cmd(f"balance obsv qa {self.qa_input.value():.4f}"))
        param_layout.addWidget(self.qa_btn, 0, 5)

        param_layout.addWidget(QLabel("观测噪声 R_v:"), 1, 0)
        self.rv_input = QDoubleSpinBox()
        self.rv_input.setRange(0.01, 1000.0)
        self.rv_input.setDecimals(2)
        self.rv_input.setSingleStep(1.0)
        self.rv_input.setValue(50.0)
        param_layout.addWidget(self.rv_input, 1, 1)
        self.rv_btn = QPushButton("设置")
        self.rv_btn.clicked.connect(lambda: self.send_cmd(f"balance obsv rv {self.rv_input.value():.2f}"))
        param_layout.addWidget(self.rv_btn, 1, 2)

        param_layout.addWidget(QLabel("观测噪声 R_a:"), 1, 3)
        self.ra_input = QDoubleSpinBox()
        self.ra_input.setRange(0.01, 1000.0)
        self.ra_input.setDecimals(2)
        self.ra_input.setSingleStep(1.0)
        self.ra_input.setValue(100.0)
        param_layout.addWidget(self.ra_input, 1, 4)
        self.ra_btn = QPushButton("设置")
        self.ra_btn.clicked.connect(lambda: self.send_cmd(f"balance obsv ra {self.ra_input.value():.2f}"))
        param_layout.addWidget(self.ra_btn, 1, 5)

        param_group.setLayout(param_layout)
        layout.addWidget(param_group)

        # ========== 数值显示 ==========
        val_group = QGroupBox("🔢 当前值")
        val_layout = QHBoxLayout()

        self.val_labels = {}
        for key, name, color in zip(self.WAVE_KEYS, self.WAVE_NAMES, self.WAVE_COLORS):
            frame = QVBoxLayout()
            lbl_name = QLabel(name.split(' ')[0])
            lbl_name.setStyleSheet(SS(f"color: {color}; font-size: 11px;"))
            lbl_name.setAlignment(Qt.AlignCenter)
            frame.addWidget(lbl_name)
            lbl_val = QLabel("--")
            lbl_val.setStyleSheet(SS(f"font-size: 18px; font-weight: bold; color: {color}; padding: 2px;"))
            lbl_val.setAlignment(Qt.AlignCenter)
            frame.addWidget(lbl_val)
            val_layout.addLayout(frame)
            self.val_labels[key] = lbl_val

        val_group.setLayout(val_layout)
        layout.addWidget(val_group)

        # ========== 波形区域 (5个小波形) ==========
        wave_group = QGroupBox("📈 波形")
        wave_layout = QVBoxLayout()

        # 初始化波形数据
        self.obsv_wave_data = {k: [] for k in self.WAVE_KEYS}
        self.obsv_wave_data['time'] = []
        self.obsv_wave_counter = 0

        # 5个波形: 上面3个(速度相关), 下面2个(位移+加速度)
        self.pw_plots = {}
        self.curves = {}

        # 上排: 3个速度波形
        top_grid = QGridLayout()
        for i, (key, name, color) in enumerate(zip(
                self.WAVE_KEYS[:3], self.WAVE_NAMES[:3], self.WAVE_COLORS[:3])):
            pw = pg.PlotWidget()
            pw.setBackground('#1a1a2e')
            pw.showGrid(x=True, y=True, alpha=0.3)
            pw.setTitle(name, color='w', size='9pt')
            pw.setLabel('left', name.split('(')[1].rstrip(')') if '(' in name else '')
            pw.setMinimumHeight(120)
            curve = pw.plot(pen=pg.mkPen(color=color, width=2))
            top_grid.addWidget(pw, 0, i)
            self.pw_plots[key] = pw
            self.curves[key] = curve

        wave_layout.addLayout(top_grid)

        # 下排: 2个波形 (位移 + 加速度)
        bot_grid = QGridLayout()
        for i, (key, name, color) in enumerate(zip(
                self.WAVE_KEYS[3:], self.WAVE_NAMES[3:], self.WAVE_COLORS[3:])):
            pw = pg.PlotWidget()
            pw.setBackground('#1a1a2e')
            pw.showGrid(x=True, y=True, alpha=0.3)
            pw.setTitle(name, color='w', size='9pt')
            pw.setLabel('left', name.split('(')[1].rstrip(')') if '(' in name else '')
            pw.setMinimumHeight(120)
            curve = pw.plot(pen=pg.mkPen(color=color, width=2))
            bot_grid.addWidget(pw, 0, i)
            self.pw_plots[key] = pw
            self.curves[key] = curve

        wave_layout.addLayout(bot_grid)

        # 波形控制栏
        wave_ctrl = QHBoxLayout()
        self.wave_clear_btn = QPushButton("🗑️ 清空波形")
        self.wave_clear_btn.clicked.connect(self._clear_waveforms)
        wave_ctrl.addWidget(self.wave_clear_btn)

        wave_ctrl.addWidget(QLabel("缓冲点数:"))
        self.wave_points_input = QSpinBox()
        self.wave_points_input.setRange(100, 5000)
        self.wave_points_input.setSingleStep(100)
        self.wave_points_input.setValue(500)
        self.wave_points_input.valueChanged.connect(lambda v: setattr(self, 'obsv_wave_max_points', v))
        wave_ctrl.addWidget(self.wave_points_input)

        wave_ctrl.addStretch()
        wave_layout.addLayout(wave_ctrl)

        wave_group.setLayout(wave_layout)
        layout.addWidget(wave_group)

        layout.addStretch()

    def send_cmd(self, cmd):
        if self.parent_window:
            self.parent_window.send_command(cmd)

    def update_obsv_data(self, v_raw, v_enc, v_filt, x_filt, a_imu):
        """更新观测器数据 (5个浮点值)"""
        vals = {'v_raw': v_raw, 'v_enc': v_enc, 'v_filt': v_filt, 'x_filt': x_filt, 'a_imu': a_imu}

        # 数值标签
        self.val_labels['v_raw'].setText(f"{v_raw:.4f}")
        self.val_labels['v_enc'].setText(f"{v_enc:.4f}")
        self.val_labels['v_filt'].setText(f"{v_filt:.4f}")
        self.val_labels['x_filt'].setText(f"{x_filt:.4f}")
        self.val_labels['a_imu'].setText(f"{a_imu:.3f}")

        # 波形
        self.obsv_wave_counter += 1
        self.obsv_wave_data['time'].append(self.obsv_wave_counter)
        for key in self.WAVE_KEYS:
            self.obsv_wave_data[key].append(vals[key])

        max_pts = self.obsv_wave_max_points
        for k in self.obsv_wave_data:
            if len(self.obsv_wave_data[k]) > max_pts:
                self.obsv_wave_data[k] = self.obsv_wave_data[k][-max_pts:]

        t = self.obsv_wave_data['time']
        for key in self.WAVE_KEYS:
            self.curves[key].setData(t, self.obsv_wave_data[key])

    def _clear_waveforms(self):
        self.obsv_wave_counter = 0
        for k in self.obsv_wave_data:
            self.obsv_wave_data[k] = []
        for key in self.WAVE_KEYS:
            self.curves[key].setData([], [])


# ============================================================================
# Full LQR 调试面板
# ============================================================================
class FullLqrPanel(QWidget):
    """Full LQR 控制器调试面板 - 参数调节、MATLAB系数导入、实时波形"""

    # MATLAB变量名到 K 索引的映射
    # a1x -> T (轮子扭矩), a2x -> Tp (腿部扭矩)
    # x = 1..6 -> theta, d_theta, x, v, pitch, pitch_rate
    MATLAB_VAR_MAP = {
        'a11': 0,  'a12': 1,  'a13': 2,  'a14': 3,  'a15': 4,  'a16': 5,
        'a21': 6,  'a22': 7,  'a23': 8,  'a24': 9,  'a25': 10, 'a26': 11,
    }

    K_LABELS = [
        'K[0]  θ→T',   'K[1]  dθ→T',  'K[2]  x→T',
        'K[3]  v→T',   'K[4]  φ→T',   'K[5]  dφ→T',
        'K[6]  θ→Tp',  'K[7]  dθ→Tp', 'K[8]  x→Tp',
        'K[9]  v→Tp',  'K[10] φ→Tp',  'K[11] dφ→Tp',
    ]

    WAVE_KEYS = ['T_left', 'T_right', 'Tp_left', 'Tp_right', 'L0', 'theta', 'v', 'pitch']
    WAVE_NAMES = ['左轮T (Nm)', '右轮T (Nm)', '左腿Tp (Nm)', '右腿Tp (Nm)',
                  '腿长L0 (m)', 'theta (°)', '速度v (m/s)', 'pitch (°)']
    WAVE_COLORS = ['#4fc3f7', '#ffb74d', '#66bb6a', '#ce93d8',
                   '#ffffff', '#ef5350', '#26c6da', '#ffa726']

    # 默认仿真文件路径
    DEFAULT_SIM_DIR = '/home/bubble/wheel-legged/打印件版/仿真/simulation'

    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent_window = parent
        self.wave_max_points = 500
        self.wave_counter = 0
        self.wave_data = {k: [] for k in self.WAVE_KEYS}
        self.wave_data['time'] = []
        # 当前系数表 (本地缓存, 12×4)
        self.local_coeff = [[0.0]*4 for _ in range(12)]
        self.init_ui()

    def init_ui(self):
        layout = QVBoxLayout(self)

        # ==================== 顶部: 模式/数据流控制 ====================
        ctrl_group = QGroupBox("🎛️ Full LQR 控制")
        ctrl_layout = QHBoxLayout()

        self.mode_btn = QPushButton("🔄 切换Full LQR模式")
        self.mode_btn.setStyleSheet(SS("background-color: #9C27B0; color: white; padding: 8px 12px; font-size: 13px;"))
        self.mode_btn.clicked.connect(lambda: self.send_cmd("balance mode flqr"))
        ctrl_layout.addWidget(self.mode_btn)

        self.stream_on_btn = QPushButton("📈 开启数据流")
        self.stream_on_btn.setStyleSheet(SS("background-color: #2196F3; color: white; padding: 8px 12px;"))
        self.stream_on_btn.clicked.connect(lambda: self.send_cmd("balance flqr stream on"))
        ctrl_layout.addWidget(self.stream_on_btn)

        self.stream_off_btn = QPushButton("⏹️ 关闭数据流")
        self.stream_off_btn.setStyleSheet(SS("padding: 8px 12px;"))
        self.stream_off_btn.clicked.connect(lambda: self.send_cmd("balance flqr stream off"))
        ctrl_layout.addWidget(self.stream_off_btn)

        self.query_btn = QPushButton("🔍 查询参数")
        self.query_btn.setStyleSheet(SS("padding: 8px 12px;"))
        self.query_btn.clicked.connect(lambda: self.send_cmd("balance flqr"))
        ctrl_layout.addWidget(self.query_btn)

        ctrl_layout.addStretch()
        ctrl_group.setLayout(ctrl_layout)
        layout.addWidget(ctrl_group)

        # ==================== 参数调节区 ====================
        param_group = QGroupBox("⚙️ 参数调节")
        param_layout = QGridLayout()

        # 行0: pitch_offset, v_scale
        param_layout.addWidget(QLabel("Pitch偏移 (rad):"), 0, 0)
        self.pitch_offset_input = QDoubleSpinBox()
        self.pitch_offset_input.setRange(-0.5, 0.5)
        self.pitch_offset_input.setDecimals(4)
        self.pitch_offset_input.setSingleStep(0.005)
        self.pitch_offset_input.setValue(0.04)
        param_layout.addWidget(self.pitch_offset_input, 0, 1)
        po_btn = QPushButton("设置")
        po_btn.clicked.connect(lambda: self.send_cmd(f"balance flqr pitch_offset {self.pitch_offset_input.value():.4f}"))
        param_layout.addWidget(po_btn, 0, 2)

        param_layout.addWidget(QLabel("速度缩放:"), 0, 3)
        self.v_scale_input = QDoubleSpinBox()
        self.v_scale_input.setRange(0.01, 5.0)
        self.v_scale_input.setDecimals(2)
        self.v_scale_input.setSingleStep(0.1)
        self.v_scale_input.setValue(0.4)
        param_layout.addWidget(self.v_scale_input, 0, 4)
        vs_btn = QPushButton("设置")
        vs_btn.clicked.connect(lambda: self.send_cmd(f"balance flqr v_scale {self.v_scale_input.value():.2f}"))
        param_layout.addWidget(vs_btn, 0, 5)

        # 行1: max_t, max_tp
        param_layout.addWidget(QLabel("轮扭矩限幅 (Nm):"), 1, 0)
        self.max_t_input = QDoubleSpinBox()
        self.max_t_input.setRange(0.1, 10.0)
        self.max_t_input.setDecimals(2)
        self.max_t_input.setSingleStep(0.5)
        self.max_t_input.setValue(2.0)
        param_layout.addWidget(self.max_t_input, 1, 1)
        mt_btn = QPushButton("设置")
        mt_btn.clicked.connect(lambda: self.send_cmd(f"balance flqr max_t {self.max_t_input.value():.2f}"))
        param_layout.addWidget(mt_btn, 1, 2)

        param_layout.addWidget(QLabel("Tp限幅 (Nm):"), 1, 3)
        self.max_tp_input = QDoubleSpinBox()
        self.max_tp_input.setRange(0.1, 20.0)
        self.max_tp_input.setDecimals(2)
        self.max_tp_input.setSingleStep(1.0)
        self.max_tp_input.setValue(8.0)
        param_layout.addWidget(self.max_tp_input, 1, 4)
        mtp_btn = QPushButton("设置")
        mtp_btn.clicked.connect(lambda: self.send_cmd(f"balance flqr max_tp {self.max_tp_input.value():.2f}"))
        param_layout.addWidget(mtp_btn, 1, 5)

        # 行2: 防劈叉 PD
        param_layout.addWidget(QLabel("防劈叉 Kp:"), 2, 0)
        self.split_kp_input = QDoubleSpinBox()
        self.split_kp_input.setRange(0.0, 50.0)
        self.split_kp_input.setDecimals(2)
        self.split_kp_input.setSingleStep(1.0)
        self.split_kp_input.setValue(5.0)
        param_layout.addWidget(self.split_kp_input, 2, 1)

        param_layout.addWidget(QLabel("Kd:"), 2, 2)
        self.split_kd_input = QDoubleSpinBox()
        self.split_kd_input.setRange(0.0, 10.0)
        self.split_kd_input.setDecimals(3)
        self.split_kd_input.setSingleStep(0.1)
        self.split_kd_input.setValue(0.2)
        param_layout.addWidget(self.split_kd_input, 2, 3)

        param_layout.addWidget(QLabel("limit:"), 2, 4)
        self.split_limit_input = QDoubleSpinBox()
        self.split_limit_input.setRange(0.0, 20.0)
        self.split_limit_input.setDecimals(2)
        self.split_limit_input.setSingleStep(0.5)
        self.split_limit_input.setValue(2.0)
        param_layout.addWidget(self.split_limit_input, 2, 5)

        split_btn = QPushButton("设置防劈叉")
        split_btn.clicked.connect(lambda: self.send_cmd(
            f"balance flqr split {self.split_kp_input.value():.2f} "
            f"{self.split_kd_input.value():.3f} {self.split_limit_input.value():.2f}"))
        param_layout.addWidget(split_btn, 2, 6)

        # 行3: 转向 PD
        param_layout.addWidget(QLabel("转向 Kp:"), 3, 0)
        self.turn_kp_input = QDoubleSpinBox()
        self.turn_kp_input.setRange(0.0, 50.0)
        self.turn_kp_input.setDecimals(2)
        self.turn_kp_input.setSingleStep(1.0)
        self.turn_kp_input.setValue(5.0)
        param_layout.addWidget(self.turn_kp_input, 3, 1)

        param_layout.addWidget(QLabel("Kd:"), 3, 2)
        self.turn_kd_input = QDoubleSpinBox()
        self.turn_kd_input.setRange(0.0, 10.0)
        self.turn_kd_input.setDecimals(4)
        self.turn_kd_input.setSingleStep(0.01)
        self.turn_kd_input.setValue(0.1)
        param_layout.addWidget(self.turn_kd_input, 3, 3)

        param_layout.addWidget(QLabel("limit:"), 3, 4)
        self.turn_limit_input = QDoubleSpinBox()
        self.turn_limit_input.setRange(0.0, 20.0)
        self.turn_limit_input.setDecimals(2)
        self.turn_limit_input.setSingleStep(0.5)
        self.turn_limit_input.setValue(2.0)
        param_layout.addWidget(self.turn_limit_input, 3, 5)

        turn_btn = QPushButton("设置转向")
        turn_btn.clicked.connect(lambda: self.send_cmd(
            f"balance flqr turn {self.turn_kp_input.value():.2f} "
            f"{self.turn_kd_input.value():.4f} {self.turn_limit_input.value():.2f}"))
        param_layout.addWidget(turn_btn, 3, 6)

        param_group.setLayout(param_layout)
        layout.addWidget(param_group)

        # ==================== MATLAB 系数导入区 ====================
        import_group = QGroupBox("📥 MATLAB 仿真系数导入")
        import_layout = QVBoxLayout()

        # 说明
        desc_label = QLabel(
            "粘贴 MATLAB get_k.m 输出的 fp32 格式文本, 或从文件加载。\n"
            "格式: fp32 a11[6] = {0,-85.9446,51.8245,-16.0279,0.0171};\n"
            "映射: a1x → T(轮扭矩) K[0-5],  a2x → Tp(腿扭矩) K[6-11]\n"
            "系数: {0, c0, c1, c2, c3} → poly_coeff[i] = [c0, c1, c2, c3]"
        )
        desc_label.setStyleSheet(SS("color: #aaa; font-size: 11px; padding: 4px;"))
        desc_label.setWordWrap(True)
        import_layout.addWidget(desc_label)

        # 文本输入区
        self.matlab_text = QTextEdit()
        self.matlab_text.setPlaceholderText(
            "在此粘贴 MATLAB 输出，例如:\n"
            "fp32 a11[6] = {0,-85.9446,51.8245,-16.0279,0.0171};\n"
            "fp32 a12[6] = {0,-0.7067,0.2248,-1.2956,0.0074};\n"
            "...\n"
            "fp32 a26[6] = {0,112.8738,-59.0739,11.0640,0.0073};"
        )
        self.matlab_text.setMaximumHeight(sf(150))
        self.matlab_text.setStyleSheet(SS(
            "background-color: #1a1a2e; color: #e0e0e0; "
            "font-family: Consolas, monospace; font-size: 12px;"
        ))
        import_layout.addWidget(self.matlab_text)

        # 按钮行
        btn_layout = QHBoxLayout()

        self.load_file_btn = QPushButton("📂 从文件加载")
        self.load_file_btn.setStyleSheet(SS("padding: 8px 15px;"))
        self.load_file_btn.clicked.connect(self._load_from_file)
        btn_layout.addWidget(self.load_file_btn)

        self.parse_btn = QPushButton("🔍 解析预览")
        self.parse_btn.setStyleSheet(SS("background-color: #FF9800; color: white; padding: 8px 15px; font-weight: bold;"))
        self.parse_btn.clicked.connect(self._parse_matlab_text)
        btn_layout.addWidget(self.parse_btn)

        self.apply_btn = QPushButton("📤 应用到ESP32")
        self.apply_btn.setStyleSheet(SS("background-color: #4CAF50; color: white; padding: 8px 15px; font-weight: bold;"))
        self.apply_btn.clicked.connect(self._apply_coefficients)
        self.apply_btn.setEnabled(False)
        btn_layout.addWidget(self.apply_btn)

        self.query_coeff_btn = QPushButton("🔍 查询当前系数")
        self.query_coeff_btn.setStyleSheet(SS("padding: 8px 15px;"))
        self.query_coeff_btn.clicked.connect(lambda: self.send_cmd("balance flqr coeff"))
        btn_layout.addWidget(self.query_coeff_btn)

        btn_layout.addStretch()

        self.parse_status = QLabel("")
        self.parse_status.setStyleSheet(SS("font-size: 12px; padding: 4px;"))
        btn_layout.addWidget(self.parse_status)

        import_layout.addLayout(btn_layout)

        # 系数预览表格 (12行 × 4列)
        self.coeff_table = QTableWidget(12, 4)
        self.coeff_table.setHorizontalHeaderLabels(['c0 (L0³)', 'c1 (L0²)', 'c2 (L0)', 'c3 (常数)'])
        self.coeff_table.setVerticalHeaderLabels(self.K_LABELS)
        self.coeff_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
        self.coeff_table.setMaximumHeight(sf(320))
        self.coeff_table.setStyleSheet(SS(
            "QTableWidget { background-color: #1a1a2e; color: #e0e0e0; "
            "font-family: Consolas; font-size: 12px; gridline-color: #333; }"
            "QHeaderView::section { background-color: #2d2d44; color: #ccc; padding: 4px; }"
        ))
        # 初始填充空值
        for r in range(12):
            for c in range(4):
                item = QTableWidgetItem("--")
                item.setTextAlignment(Qt.AlignCenter)
                self.coeff_table.setItem(r, c, item)
        import_layout.addWidget(self.coeff_table)

        import_group.setLayout(import_layout)
        layout.addWidget(import_group)

        # ==================== 数值显示 ====================
        val_group = QGroupBox("🔢 实时数据")
        val_layout = QHBoxLayout()
        self.val_labels = {}
        for key, name, color in zip(self.WAVE_KEYS, self.WAVE_NAMES, self.WAVE_COLORS):
            frame = QVBoxLayout()
            lbl_name = QLabel(name.split(' ')[0])
            lbl_name.setStyleSheet(SS(f"color: {color}; font-size: 10px;"))
            lbl_name.setAlignment(Qt.AlignCenter)
            frame.addWidget(lbl_name)
            lbl_val = QLabel("--")
            lbl_val.setStyleSheet(SS(f"font-size: 16px; font-weight: bold; color: {color}; padding: 2px;"))
            lbl_val.setAlignment(Qt.AlignCenter)
            frame.addWidget(lbl_val)
            val_layout.addLayout(frame)
            self.val_labels[key] = lbl_val
        val_group.setLayout(val_layout)
        layout.addWidget(val_group)

        # ==================== 波形区域 ====================
        wave_group = QGroupBox("📈 Full LQR 波形")
        wave_layout = QVBoxLayout()

        self.pw_plots = {}
        self.curves = {}

        # 上排: T_left, T_right, Tp_left, Tp_right (4个扭矩)
        top_grid = QGridLayout()
        for i, (key, name, color) in enumerate(zip(
                self.WAVE_KEYS[:4], self.WAVE_NAMES[:4], self.WAVE_COLORS[:4])):
            pw = pg.PlotWidget()
            pw.setBackground('#1a1a2e')
            pw.showGrid(x=True, y=True, alpha=0.3)
            pw.setTitle(name, color='w', size='9pt')
            pw.setMinimumHeight(sf(100))
            curve = pw.plot(pen=pg.mkPen(color=color, width=2))
            top_grid.addWidget(pw, 0, i)
            self.pw_plots[key] = pw
            self.curves[key] = curve
        wave_layout.addLayout(top_grid)

        # 下排: L0, theta, v, pitch (4个状态)
        bot_grid = QGridLayout()
        for i, (key, name, color) in enumerate(zip(
                self.WAVE_KEYS[4:], self.WAVE_NAMES[4:], self.WAVE_COLORS[4:])):
            pw = pg.PlotWidget()
            pw.setBackground('#1a1a2e')
            pw.showGrid(x=True, y=True, alpha=0.3)
            pw.setTitle(name, color='w', size='9pt')
            pw.setMinimumHeight(sf(100))
            curve = pw.plot(pen=pg.mkPen(color=color, width=2))
            bot_grid.addWidget(pw, 0, i)
            self.pw_plots[key] = pw
            self.curves[key] = curve
        wave_layout.addLayout(bot_grid)

        # 波形控制
        wave_ctrl = QHBoxLayout()
        clear_btn = QPushButton("🗑️ 清空波形")
        clear_btn.clicked.connect(self._clear_waveforms)
        wave_ctrl.addWidget(clear_btn)

        wave_ctrl.addWidget(QLabel("缓冲点数:"))
        pts_input = QSpinBox()
        pts_input.setRange(100, 5000)
        pts_input.setSingleStep(100)
        pts_input.setValue(500)
        pts_input.valueChanged.connect(lambda v: setattr(self, 'wave_max_points', v))
        wave_ctrl.addWidget(pts_input)
        wave_ctrl.addStretch()
        wave_layout.addLayout(wave_ctrl)

        wave_group.setLayout(wave_layout)
        layout.addWidget(wave_group)

        layout.addStretch()

    # ----------------------------------------------------------------
    # MATLAB 系数解析
    # ----------------------------------------------------------------
    def _parse_matlab_line(self, line):
        """解析一行 MATLAB fp32 输出, 返回 (var_name, [c0,c1,c2,c3]) 或 None"""
        # 格式: fp32 a11[6] = {0,-85.9446,51.8245,-16.0279,0.0171};
        # 也支持不带 fp32 前缀的格式
        m = re.match(
            r'(?:fp32\s+)?(a[12][1-6])\s*\[\d+\]\s*=\s*\{([^}]+)\}',
            line.strip()
        )
        if not m:
            return None
        var_name = m.group(1)
        values_str = m.group(2)
        try:
            values = [float(v.strip()) for v in values_str.split(',')]
        except ValueError:
            return None

        # MATLAB 输出格式: {0, c0, c1, c2, c3} (共5个值, 首个0是占位)
        if len(values) == 5:
            coeffs = values[1:]  # 跳过第一个 0
        elif len(values) == 4:
            coeffs = values      # 直接是 c0,c1,c2,c3
        else:
            return None

        if var_name not in self.MATLAB_VAR_MAP:
            return None

        return (var_name, coeffs)

    def _parse_matlab_text(self):
        """解析文本框中的 MATLAB 输出"""
        text = self.matlab_text.toPlainText().strip()
        if not text:
            self.parse_status.setText("⚠️ 请粘贴 MATLAB 输出文本")
            self.parse_status.setStyleSheet(SS("color: #FF9800; font-size: 12px;"))
            return

        parsed = {}
        errors = []
        for i, line in enumerate(text.strip().split('\n'), 1):
            line = line.strip()
            if not line or line.startswith('%') or line.startswith('//'):
                continue
            result = self._parse_matlab_line(line)
            if result:
                var_name, coeffs = result
                k_idx = self.MATLAB_VAR_MAP[var_name]
                parsed[k_idx] = coeffs
            else:
                if 'a1' in line or 'a2' in line or 'fp32' in line:
                    errors.append(f"行{i}: 无法解析")

        if not parsed:
            self.parse_status.setText("❌ 未找到有效的系数行")
            self.parse_status.setStyleSheet(SS("color: #f44336; font-size: 12px;"))
            self.apply_btn.setEnabled(False)
            return

        # 更新本地缓存和表格
        for k_idx, coeffs in parsed.items():
            self.local_coeff[k_idx] = coeffs[:]
            for c in range(4):
                item = self.coeff_table.item(k_idx, c)
                if item:
                    item.setText(f"{coeffs[c]:+.4f}")
                    item.setBackground(QColor('#1b3a1b'))  # 绿色高亮已更新

        missing = [i for i in range(12) if i not in parsed]
        status_parts = [f"✅ 已解析 {len(parsed)}/12 行"]
        if missing:
            status_parts.append(f"缺少: {', '.join(self.K_LABELS[i].split()[0] for i in missing)}")
        if errors:
            status_parts.append(f"⚠️ {len(errors)}行解析失败")
        self.parse_status.setText(' | '.join(status_parts))
        self.parse_status.setStyleSheet(SS(
            f"color: {'#4CAF50' if len(parsed) == 12 else '#FF9800'}; font-size: 12px;"
        ))

        self.apply_btn.setEnabled(len(parsed) > 0)

    def _load_from_file(self):
        """从文件加载 MATLAB 输出"""
        file_path, _ = QFileDialog.getOpenFileName(
            self, "加载 MATLAB 系数文件",
            self.DEFAULT_SIM_DIR,
            "MATLAB 文件 (*.m *.txt);;所有文件 (*)"
        )
        if not file_path:
            return

        try:
            with open(file_path, 'r', encoding='utf-8') as f:
                content = f.read()
        except Exception as e:
            QMessageBox.warning(self, "读取失败", f"无法读取文件:\n{e}")
            return

        # 提取含 fp32 a[12][1-6] 的行
        lines = []
        for line in content.split('\n'):
            line_stripped = line.strip()
            # 跳过注释行
            if line_stripped.startswith('%') or line_stripped.startswith('//'):
                # 但检查注释中是否有 fprintf 输出格式
                continue
            # 检查 fprintf 语句 → 提取格式字符串中的变量名
            if 'fprintf' in line_stripped:
                # 从 fprintf 的格式字符串中提取: fprintf('fp32 a11[6] = {0,%.4f,...};\n', ...)
                m_fprintf = re.search(r"fprintf\s*\(\s*'([^']+)'", line_stripped)
                if m_fprintf:
                    fmt_str = m_fprintf.group(1)
                    # 这只是模板，不是实际数据，跳过
                    continue
            # 直接含 fp32 aXX 的行
            if re.search(r'(?:fp32\s+)?a[12][1-6]\s*\[', line_stripped):
                lines.append(line_stripped)

        if lines:
            self.matlab_text.setPlainText('\n'.join(lines))
            self.parse_status.setText(f"📂 从文件提取了 {len(lines)} 行，请点击'解析预览'")
            self.parse_status.setStyleSheet(SS("color: #2196F3; font-size: 12px;"))
        else:
            # 文件可能是 .m 脚本，不含实际系数行
            # 尝试将整个内容放入文本框
            self.matlab_text.setPlainText(content)
            self.parse_status.setText("📂 文件已加载 (未自动识别系数行，请手动粘贴 MATLAB 命令行输出)")
            self.parse_status.setStyleSheet(SS("color: #FF9800; font-size: 12px;"))

    def _apply_coefficients(self):
        """将解析的系数通过串口发送到 ESP32"""
        # 先重新解析确保最新
        text = self.matlab_text.toPlainText().strip()
        if not text:
            return

        parsed = {}
        for line in text.strip().split('\n'):
            result = self._parse_matlab_line(line.strip())
            if result:
                var_name, coeffs = result
                k_idx = self.MATLAB_VAR_MAP[var_name]
                parsed[k_idx] = coeffs

        if not parsed:
            QMessageBox.warning(self, "应用失败", "没有已解析的系数可以应用")
            return

        # 确认对话框
        reply = QMessageBox.question(
            self, "确认应用",
            f"即将向 ESP32 发送 {len(parsed)} 行系数更新。\n"
            f"这将修改 Full LQR 的多项式拟合系数。\n\n继续？",
            QMessageBox.Yes | QMessageBox.No, QMessageBox.No
        )
        if reply != QMessageBox.Yes:
            return

        # 逐行发送 balance flqr coeff <row> <c0> <c1> <c2> <c3>
        sent = 0
        for k_idx in sorted(parsed.keys()):
            coeffs = parsed[k_idx]
            cmd = f"balance flqr coeff {k_idx} {coeffs[0]:.4f} {coeffs[1]:.4f} {coeffs[2]:.4f} {coeffs[3]:.4f}"
            self.send_cmd(cmd)
            sent += 1
            # 在表格中标记为已发送
            for c in range(4):
                item = self.coeff_table.item(k_idx, c)
                if item:
                    item.setBackground(QColor('#0d3b66'))  # 蓝色表示已发送

        self.parse_status.setText(f"📤 已发送 {sent} 行系数到 ESP32")
        self.parse_status.setStyleSheet(SS("color: #4CAF50; font-size: 12px;"))

    # ----------------------------------------------------------------
    # 波形更新
    # ----------------------------------------------------------------
    def update_flqr_data(self, T_left, T_right, Tp_left, Tp_right,
                         L0, theta, d_theta, x, v, pitch, pitch_rate, split_comp):
        """更新 Full LQR 数据 (12个浮点值, 来自 #FLQR 数据流)"""
        vals = {
            'T_left': T_left, 'T_right': T_right,
            'Tp_left': Tp_left, 'Tp_right': Tp_right,
            'L0': L0, 'theta': theta, 'v': v, 'pitch': pitch,
        }

        # 数值标签
        self.val_labels['T_left'].setText(f"{T_left:.3f}")
        self.val_labels['T_right'].setText(f"{T_right:.3f}")
        self.val_labels['Tp_left'].setText(f"{Tp_left:.4f}")
        self.val_labels['Tp_right'].setText(f"{Tp_right:.4f}")
        self.val_labels['L0'].setText(f"{L0:.3f}")
        self.val_labels['theta'].setText(f"{theta:.1f}")
        self.val_labels['v'].setText(f"{v:.3f}")
        self.val_labels['pitch'].setText(f"{pitch:.1f}")

        # 波形
        self.wave_counter += 1
        self.wave_data['time'].append(self.wave_counter)
        for key in self.WAVE_KEYS:
            self.wave_data[key].append(vals[key])

        max_pts = self.wave_max_points
        for k in self.wave_data:
            if len(self.wave_data[k]) > max_pts:
                self.wave_data[k] = self.wave_data[k][-max_pts:]

        t = self.wave_data['time']
        for key in self.WAVE_KEYS:
            self.curves[key].setData(t, self.wave_data[key])

    def _clear_waveforms(self):
        self.wave_counter = 0
        for k in self.wave_data:
            self.wave_data[k] = []
        for key in self.WAVE_KEYS:
            self.curves[key].setData([], [])

    def send_cmd(self, cmd):
        if self.parent_window:
            self.parent_window.send_command(cmd)


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
        self.mpower_on_btn.setStyleSheet(SS("background-color: #44aa44;"))
        self.mpower_on_btn.clicked.connect(lambda: self.send_cmd("mpower on"))
        btn_layout.addWidget(self.mpower_on_btn)
        
        self.mpower_off_btn = QPushButton("电机供电 OFF")
        self.mpower_off_btn.setStyleSheet(SS("background-color: #aa4444;"))
        self.mpower_off_btn.clicked.connect(lambda: self.send_cmd("mpower off"))
        btn_layout.addWidget(self.mpower_off_btn)
        
        power_layout.addLayout(btn_layout)
        
        status_layout = QHBoxLayout()
        status_layout.addWidget(QLabel("电池:"))
        self.battery_label = QLabel("--")
        self.battery_label.setStyleSheet(SS("font-size: 16px; font-weight: bold;"))
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
        self.temp_label.setStyleSheet(SS("font-size: 20px; font-weight: bold; color: #ff6600;"))
        data_layout.addWidget(self.temp_label)
        data_layout.addWidget(QLabel("湿度:"))
        self.humi_label = QLabel("--")
        self.humi_label.setStyleSheet(SS("font-size: 20px; font-weight: bold; color: #0066ff;"))
        data_layout.addWidget(self.humi_label)
        sht_layout.addLayout(data_layout)
        sht_group.setLayout(sht_layout)
        layout.addWidget(sht_group)
        
        # WiFi 遥控
        wifi_group = QGroupBox("📶 WiFi 遥控器")
        wifi_layout = QHBoxLayout()
        
        self.wifi_start_btn = QPushButton("启动 WiFi AP")
        self.wifi_start_btn.setStyleSheet(SS("background-color: #4488ff;"))
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
        self.cmd_input.setStyleSheet(SS("font-size: 14px; padding: 5px;"))
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
        help_label.setStyleSheet(SS("color: gray; font-size: 12px;"))
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
        # 自适应窗口大小: 占屏幕 75%
        screen = QApplication.primaryScreen()
        if screen:
            geo = screen.availableGeometry()
            w = int(geo.width() * 0.85)
            h = int(geo.height() * 0.75)
            self.setGeometry(
                geo.x() + (geo.width() - w) // 2,
                geo.y() + (geo.height() - h) // 2,
                w, h
            )
        else:
            self.setGeometry(100, 100, 1200, 800)
        
        self.serial_thread = SerialThread()
        self.serial_thread.data_received.connect(self.process_serial_data)
        
        self.pid_panels = {}
        self.lpf_panels = {}
        self.speed_adaptive_panel = None
        self.debug_mode = False
        self.show_high_freq_data = False  # 是否在日志中显示高频数据
        
        # YAW 调试数据缓存
        self._yaw_target_angle = 0.0
        self._yaw_current_angle = 0.0
        
        self.init_ui()
    
    def init_ui(self):
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setContentsMargins(4, 4, 4, 4)
        main_layout.setSpacing(4)
        
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
        self.status_label.setStyleSheet(SS("color: red; font-weight: bold; font-size: 14px;"))
        serial_layout.addWidget(self.status_label)
        
        serial_layout.addStretch()
        serial_group.setLayout(serial_layout)
        # 串口区域不拉伸, 只占最小高度
        from PyQt5.QtWidgets import QSizePolicy
        serial_group.setSizePolicy(QSizePolicy.Preferred, QSizePolicy.Fixed)
        main_layout.addWidget(serial_group, 0)  # stretch=0
        
        # ===== 主分割区域 =====
        splitter = QSplitter(Qt.Horizontal)
        
        # 左侧: Tab切换区
        self.tab_widget = QTabWidget()
        
        # 根据Commander映射创建标签页
        # PID控制器 - 平衡控制相关
        self.pid_panels['angle'] = PIDControlPanel("角度控制 (Angle)", "A", self, output_channel_id='Q')
        self.tab_widget.addTab(_make_scrollable(self.pid_panels['angle']), "A - 角度PID")
        
        self.pid_panels['gyro'] = PIDControlPanel("角速度控制 (Gyro)", "B", self, output_channel_id='R')
        self.tab_widget.addTab(_make_scrollable(self.pid_panels['gyro']), "B - 角速度PID")
        
        self.pid_panels['distance'] = PIDControlPanel("位移控制 (Distance)", "C", self, output_channel_id='S')
        self.tab_widget.addTab(_make_scrollable(self.pid_panels['distance']), "C - 位移PID")
        
        self.pid_panels['speed'] = PIDControlPanel("速度控制 (Speed)", "D", self, output_channel_id='T')
        self.tab_widget.addTab(_make_scrollable(self.pid_panels['speed']), "D - 速度PID")
        
        self.pid_panels['yaw_angle'] = PIDControlPanel("YAW角度控制", "E", self, output_channel_id='U')
        self.tab_widget.addTab(_make_scrollable(self.pid_panels['yaw_angle']), "E - YAW角度PID")
        
        self.pid_panels['yaw_gyro'] = PIDControlPanel("YAW角速度控制", "F", self, output_channel_id='U')
        self.tab_widget.addTab(_make_scrollable(self.pid_panels['yaw_gyro']), "F - YAW角速度PID")
        
        self.pid_panels['lqr_u'] = PIDControlPanel("LQR输出补偿", "H", self, output_channel_id='V', output_dual=True)
        self.tab_widget.addTab(_make_scrollable(self.pid_panels['lqr_u']), "H - LQR输出PID")
        
        self.pid_panels['zeropoint'] = PIDControlPanel(
            "零点自适应", "I", self,
            extra_channel_id='Z',
            extra_labels=('角度误差', '激活状态')
        )
        self.tab_widget.addTab(_make_scrollable(self.pid_panels['zeropoint']), "I - 零点PID")
        
        self.pid_panels['roll_angle'] = PIDControlPanel("Roll轴平衡", "K", self)
        self.tab_widget.addTab(_make_scrollable(self.pid_panels['roll_angle']), "K - Roll角度PID")
        
        # 低通滤波器
        self.lpf_panels['joyy'] = LPFControlPanel("摇杆Y轴滤波", "G", self)
        self.tab_widget.addTab(_make_scrollable(self.lpf_panels['joyy']), "G - 摇杆滤波")
        
        self.lpf_panels['zeropoint'] = LPFControlPanel("零点滤波", "J", self)
        self.tab_widget.addTab(_make_scrollable(self.lpf_panels['zeropoint']), "J - 零点滤波")
        
        self.lpf_panels['roll'] = LPFControlPanel("Roll角度滤波", "L", self)
        self.tab_widget.addTab(_make_scrollable(self.lpf_panels['roll']), "L - Roll滤波")
        
        self.lpf_panels['gyro'] = GyroFilterPanel(self)
        self.tab_widget.addTab(_make_scrollable(self.lpf_panels['gyro']), "N - 角速度滤波")
        
        self.lpf_panels['speed'] = SpeedFilterPanel(self)
        self.tab_widget.addTab(_make_scrollable(self.lpf_panels['speed']), "W - 轮速滤波")
        
        # 速度自适应面板
        self.speed_adaptive_panel = SpeedAdaptivePanel(self)
        self.tab_widget.addTab(_make_scrollable(self.speed_adaptive_panel), "M - 速度自适应P")
        
        # 轮速调试面板 (用于离地检测阈值调试)
        # O - 左轮: 蓝线=速度(rad/s), 红线=加速度(rad/s²)
        # P - 右轮: 蓝线=速度(rad/s), 红线=加速度(rad/s²)
        self.wheel_panels = {}
        self.wheel_panels['left'] = PIDControlPanel("左轮调试 (蓝=速度rad/s, 红=加速度rad/s²)", "O", self)
        self.tab_widget.addTab(_make_scrollable(self.wheel_panels['left']), "O - 左轮调试")
        
        self.wheel_panels['right'] = PIDControlPanel("右轮调试 (蓝=速度rad/s, 红=加速度rad/s²)", "P", self)
        self.tab_widget.addTab(_make_scrollable(self.wheel_panels['right']), "P - 右轮调试")
        
        self.wheel_panels['current'] = PIDControlPanel("电机电流 (蓝=左轮A, 红=右轮A)", "X", self)
        self.tab_widget.addTab(_make_scrollable(self.wheel_panels['current']), "X - 电机电流")
        
        # 腿部控制面板
        self.leg_panel = LegControlPanel(self)
        self.tab_widget.addTab(_make_scrollable(self.leg_panel), "🦿 腿部控制")
        
        # Web监控面板
        self.web_monitor = WebMonitorPanel(self)
        self.tab_widget.addTab(_make_scrollable(self.web_monitor), "📱 Web监控")
        
        # ===== 新增设备控制面板 =====
        # 平衡控制面板
        self.balance_panel = BalanceControlPanel(self)
        self.tab_widget.addTab(_make_scrollable(self.balance_panel), "🎮 平衡控制")
        
        # 双环 PID 调参面板
        self.dual_pid_panel = DualPIDPanel(self)
        self.tab_widget.addTab(_make_scrollable(self.dual_pid_panel), "🎯 双环PID")
        
        # YAW 调试面板 (独立标签页)
        self.yaw_panel = YawDebugPanel(self)
        self.tab_widget.addTab(_make_scrollable(self.yaw_panel), "🧭 YAW调试")
        
        # VMC 调试面板
        self.vmc_panel = VMCDebugPanel(self)
        self.tab_widget.addTab(_make_scrollable(self.vmc_panel), "🦿 VMC调试")
        
        # 关节电机监控面板
        self.joint_panel = JointMonitorPanel(self)
        self.tab_widget.addTab(_make_scrollable(self.joint_panel), "🔩 关节监控")
        
        # 电机功率监控面板
        self.mpow_panel = MotorPowerPanel(self)
        self.tab_widget.addTab(_make_scrollable(self.mpow_panel), "⚡ 电机功率")
        
        # 速度/位移观测器面板
        self.obsv_panel = ObserverPanel(self)
        self.tab_widget.addTab(_make_scrollable(self.obsv_panel), "📊 速度观测")
        
        # Full LQR 调试面板
        self.flqr_panel = FullLqrPanel(self)
        self.tab_widget.addTab(_make_scrollable(self.flqr_panel), "🧮 Full LQR")
        
        # 电机控制面板
        self.motor_panel = MotorControlPanel(self)
        self.tab_widget.addTab(_make_scrollable(self.motor_panel), "⚙️ 电机控制")
        
        # STW 配置面板
        self.stw_panel = STWConfigPanel(self)
        self.tab_widget.addTab(_make_scrollable(self.stw_panel), "🔧 STW配置")
        
        # IMU控制面板
        self.imu_panel = IMUControlPanel(self)
        self.tab_widget.addTab(_make_scrollable(self.imu_panel), "📐 IMU")
        
        # 传感器面板
        self.sensor_panel = SensorPanel(self)
        self.tab_widget.addTab(_make_scrollable(self.sensor_panel), "🔧 传感器")
        
        # 终端面板
        self.terminal_panel = TerminalPanel(self)
        self.tab_widget.addTab(_make_scrollable(self.terminal_panel), "💻 终端")
        
        splitter.addWidget(self.tab_widget)
        
        # 右侧: 日志区
        log_widget = QWidget()
        log_layout = QVBoxLayout(log_widget)
        
        log_group = QGroupBox("通信日志")
        log_inner_layout = QVBoxLayout()
        
        self.log_text = QTextEdit()
        self.log_text.setReadOnly(True)
        self.log_text.setStyleSheet(SS("background-color: #1e1e1e; color: #d4d4d4; font-family: Consolas; font-size: 12px;"))
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
        
        high_freq_btn = QPushButton("📊 高频数据")
        high_freq_btn.setCheckable(True)
        high_freq_btn.setToolTip("开启后在日志中显示高频数据(IMU/腿部状态/频率等)")
        high_freq_btn.toggled.connect(self.toggle_high_freq_data)
        log_btn_layout.addWidget(high_freq_btn)
        
        log_btn_layout.addStretch()
        log_inner_layout.addLayout(log_btn_layout)
        
        log_group.setLayout(log_inner_layout)
        log_layout.addWidget(log_group)
        
        splitter.addWidget(log_widget)
        splitter.setSizes([900, 300])
        splitter.setStretchFactor(0, 3)   # tab 区域占更多
        splitter.setStretchFactor(1, 1)   # 日志区域占较少
        
        main_layout.addWidget(splitter, 1)  # stretch=1, 占满剩余空间
    
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
                self.status_label.setStyleSheet(SS("color: green; font-weight: bold; font-size: 14px;"))
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
            self.status_label.setStyleSheet(SS("color: red; font-weight: bold; font-size: 14px;"))
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
        
        # ===== 高频波形数据 - 只更新图表，不打印日志 =====
        
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
                        'L': 'roll',      # Roll滤波
                        'N': 'gyro',      # 角速度滤波
                        'W': 'speed',     # 轮速滤波
                    }
                    
                    # 轮速调试面板映射
                    wheel_map = {
                        'O': 'left',      # 左轮调试
                        'P': 'right',     # 右轮调试
                        'X': 'current',   # 电机电流
                    }
                    
                    # 环路输出映射 (通道ID → PID面板key)
                    output_map = {
                        'Q': 'angle',      # 角度环输出
                        'R': 'gyro',       # 角速度环输出
                        'S': 'distance',   # 位移环输出
                        'T': 'speed',      # 速度环输出
                        'U': 'yaw_angle',  # YAW控制输出 (同时更新yaw_gyro面板)
                        'V': 'lqr_u',      # LQR输出PID处理前后对比
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
                    elif panel_id in output_map:
                        panel_key = output_map[panel_id]
                        if panel_key in self.pid_panels:
                            panel = self.pid_panels[panel_key]
                            if panel.output_dual:
                                # 双线模式: target=处理前(raw), control=处理后(final)
                                panel.update_output_plot(control, value2=target)
                            else:
                                panel.update_output_plot(control)
                        # YAW输出同时更新 yaw_gyro 面板
                        if panel_id == 'U' and 'yaw_gyro' in self.pid_panels:
                            self.pid_panels['yaw_gyro'].update_output_plot(control)
                    
                    # 额外观测通道: Z → 零点面板的 extra 波形
                    elif panel_id == 'Z':
                        if 'zeropoint' in self.pid_panels:
                            self.pid_panels['zeropoint'].update_extra_plot(target, control)
                    
                except (ValueError, IndexError) as e:
                    if self.debug_mode:
                        self.log(f"解析数据失败: {line} ({e})", is_error=True)
            if not self.debug_mode and not self.show_high_freq_data:
                return  # 波形数据不打印日志(非debug模式)
        
        # 解析Web命令格式: #WEB,go,dir,joyx,joyy,height (高频数据)
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
                except (ValueError, IndexError):
                    pass
            if not self.debug_mode and not self.show_high_freq_data:
                return  # Web数据不打印日志(非debug模式)
        
        # YAW 调试数据 (高频)
        if line.startswith("#YAW_DBG,"):
            yaw_dbg_match = re.search(r'#YAW_DBG,out=([-\d.]+),err=([-\d.]+),hold=(\d),rate=([-\d.]+)', line)
            if yaw_dbg_match:
                try:
                    yaw_output = float(yaw_dbg_match.group(1))
                    yaw_error = float(yaw_dbg_match.group(2))
                    yaw_holding = int(yaw_dbg_match.group(3)) == 1
                    yaw_rate = float(yaw_dbg_match.group(4))
                    if hasattr(self, 'yaw_panel'):
                        self.yaw_panel.update_yaw_debug(
                            self._yaw_target_angle, self._yaw_current_angle,
                            yaw_error, yaw_output, yaw_holding, yaw_rate
                        )
                except:
                    pass
            if not self.debug_mode and not self.show_high_freq_data:
                return  # 不打印日志(非debug模式)
        
        # 双环 PID 状态 (高频)
        if line.startswith("DPID_STATUS:"):
            dpid_match = re.search(r'DPID_STATUS:PITCH_ERR=([-\d.]+),TGT_SPD=([-\d.]+),SPD_ERR=([-\d.]+),TORQUE=([-\d.]+)(?:,ORDER=(\d+))?', line)
            if dpid_match:
                try:
                    pitch_err = float(dpid_match.group(1))
                    target_speed = float(dpid_match.group(2))
                    speed_err = float(dpid_match.group(3))
                    torque = float(dpid_match.group(4))
                    if hasattr(self, 'dual_pid_panel'):
                        self.dual_pid_panel.update_status(pitch_err, target_speed, speed_err, torque)
                        # 更新环序UI
                        if dpid_match.group(5) is not None:
                            order = int(dpid_match.group(5))
                            self.dual_pid_panel.update_loop_order_ui(order)
                except:
                    pass
            if not self.debug_mode and not self.show_high_freq_data:
                return  # 不打印日志(非debug模式)
        
        # 双环 PID 调试输出 (实时)
        # 角度优先: [DPID-AF] pitch=X° err=X° rate=X°/s | Angle(外): P=X I=X D=X → tgt_spd=X | Speed(内): err=X P=X I=X D=X → torque=X
        # 速度优先: [DPID-SF] pitch=X° spd=X | Speed(外): err=X P=X I=X D=X → tgt_pitch=X° | Angle(内): err=X P=X I=X D=X → torque=X
        if line.startswith("[DPID-AF]"):
            dpid_debug_match = re.search(
                r'\[DPID-AF\] pitch=([-\d.]+)° err=([-\d.]+)° rate=([-\d.]+)°/s \| '
                r'Angle\(外\): P=([-\d.]+) I=([-\d.]+) D=([-\d.]+) → tgt_spd=([-\d.]+) \| '
                r'Speed\(内\): err=([-\d.]+) P=([-\d.]+) I=([-\d.]+) D=([-\d.]+) → torque=([-\d.]+)', line)
            if dpid_debug_match:
                try:
                    pitch = float(dpid_debug_match.group(1))
                    err = float(dpid_debug_match.group(2))
                    rate = float(dpid_debug_match.group(3))
                    angle_p = float(dpid_debug_match.group(4))
                    angle_i = float(dpid_debug_match.group(5))
                    angle_d = float(dpid_debug_match.group(6))
                    tgt_spd = float(dpid_debug_match.group(7))
                    spd_err = float(dpid_debug_match.group(8))
                    speed_p = float(dpid_debug_match.group(9))
                    speed_i = float(dpid_debug_match.group(10))
                    speed_d = float(dpid_debug_match.group(11))
                    torque = float(dpid_debug_match.group(12))
                    if hasattr(self, 'dual_pid_panel'):
                        self.dual_pid_panel.update_loop_order_ui(0)
                        self.dual_pid_panel.update_dpid_debug(
                            pitch, err, rate, angle_p, angle_i, angle_d, tgt_spd,
                            spd_err, speed_p, speed_i, speed_d, torque)
                except:
                    pass
            if not self.debug_mode and not self.show_high_freq_data:
                return
        
        if line.startswith("[DPID-SF]"):
            dpid_sf_match = re.search(
                r'\[DPID-SF\] pitch=([-\d.]+)° spd=([-\d.]+) \| '
                r'Speed\(外\): err=([-\d.]+) P=([-\d.]+) I=([-\d.]+) D=([-\d.]+) → tgt_pitch=([-\d.]+)° \| '
                r'Angle\(内\): err=([-\d.]+) P=([-\d.]+) I=([-\d.]+) D=([-\d.]+) → torque=([-\d.]+)', line)
            if dpid_sf_match:
                try:
                    pitch = float(dpid_sf_match.group(1))
                    spd = float(dpid_sf_match.group(2))
                    spd_err = float(dpid_sf_match.group(3))
                    speed_p = float(dpid_sf_match.group(4))
                    speed_i = float(dpid_sf_match.group(5))
                    speed_d = float(dpid_sf_match.group(6))
                    tgt_pitch = float(dpid_sf_match.group(7))
                    angle_err = float(dpid_sf_match.group(8))
                    angle_p = float(dpid_sf_match.group(9))
                    angle_i = float(dpid_sf_match.group(10))
                    angle_d = float(dpid_sf_match.group(11))
                    torque = float(dpid_sf_match.group(12))
                    if hasattr(self, 'dual_pid_panel'):
                        self.dual_pid_panel.update_loop_order_ui(1)
                        self.dual_pid_panel.update_dpid_debug_sf(
                            pitch, spd, spd_err, speed_p, speed_i, speed_d, tgt_pitch,
                            angle_err, angle_p, angle_i, angle_d, torque)
                except:
                    pass
            if not self.debug_mode and not self.show_high_freq_data:
                return
            if not self.debug_mode and not self.show_high_freq_data:
                return  # 不打印日志(非debug模式)
        
        # 单环 PID 调试输出 (实时)
        # 格式: [SPID] pitch=X° err=X° rate=X°/s | Angle: P=X I=X D=X → speed=X rad/s (X rpm)
        if line.startswith("[SPID]"):
            spid_debug_match = re.search(
                r'\[SPID\] pitch=([-\d.]+)° err=([-\d.]+)° rate=([-\d.]+)°/s \| '
                r'Angle: P=([-\d.]+) I=([-\d.]+) D=([-\d.]+) → speed=([-\d.]+) rad/s \(([-\d.]+) rpm\)', line)
            if spid_debug_match:
                try:
                    pitch = float(spid_debug_match.group(1))
                    err = float(spid_debug_match.group(2))
                    rate = float(spid_debug_match.group(3))
                    p = float(spid_debug_match.group(4))
                    i = float(spid_debug_match.group(5))
                    d = float(spid_debug_match.group(6))
                    speed = float(spid_debug_match.group(7))
                    rpm = float(spid_debug_match.group(8))
                    if hasattr(self, 'dual_pid_panel'):
                        self.dual_pid_panel.update_spid_debug(pitch, err, rate, p, i, d, speed, rpm)
                except:
                    pass
            if not self.debug_mode and not self.show_high_freq_data:
                return  # 不打印日志(非debug模式)
        
        # 四环 PID 调试输出 (实时)
        # 格式(无位移环): [TPID] pitch=X° spd=X | Speed(外): err=X → pitch_tgt=X° | Angle(中): err=X → whl_tgt=X | Wheel(内): err=X → out=X [spd|trq]
        # 格式(有位移环): [TPID] pitch=X° spd=X | Dist(最外): err=X → spd_corr=X | Speed(外): err=X → pitch_tgt=X° | Angle(中): err=X → whl_tgt=X | Wheel(内): err=X → out=X [spd|trq]
        if line.startswith("[TPID]"):
            # 先尝试带位移环的格式
            tpid_dist_match = re.search(
                r'\[TPID\] pitch=([-\d.]+)° spd=([-\d.]+) \| '
                r'Dist\(最外\): err=([-\d.]+) → spd_corr=([-\d.]+) \| '
                r'Speed\(外\): err=([-\d.]+) → pitch_tgt=([-\d.]+)° \| '
                r'Angle\(中\): err=([-\d.]+) → whl_tgt=([-\d.]+) \| '
                r'Wheel\(内\): err=([-\d.]+) → out=([-\d.]+) \[(spd|trq)\]', line)
            if tpid_dist_match:
                try:
                    pitch = float(tpid_dist_match.group(1))
                    spd = float(tpid_dist_match.group(2))
                    dist_err = float(tpid_dist_match.group(3))
                    dist_corr = float(tpid_dist_match.group(4))
                    spd_err = float(tpid_dist_match.group(5))
                    pitch_tgt = float(tpid_dist_match.group(6))
                    ang_err = float(tpid_dist_match.group(7))
                    whl_tgt = float(tpid_dist_match.group(8))
                    whl_err = float(tpid_dist_match.group(9))
                    out = float(tpid_dist_match.group(10))
                    wmode = tpid_dist_match.group(11)
                    if hasattr(self, 'dual_pid_panel'):
                        self.dual_pid_panel.update_tpid_debug(
                            pitch, spd, spd_err, pitch_tgt, ang_err, whl_tgt, whl_err, out, wmode,
                            dist_err=dist_err, dist_corr=dist_corr)
                except:
                    pass
            else:
                # 不带位移环的格式
                tpid_debug_match = re.search(
                    r'\[TPID\] pitch=([-\d.]+)° spd=([-\d.]+) \| '
                    r'Speed\(外\): err=([-\d.]+) → pitch_tgt=([-\d.]+)° \| '
                    r'Angle\(中\): err=([-\d.]+) → whl_tgt=([-\d.]+) \| '
                    r'Wheel\(内\): err=([-\d.]+) → out=([-\d.]+) \[(spd|trq)\]', line)
                if tpid_debug_match:
                    try:
                        pitch = float(tpid_debug_match.group(1))
                        spd = float(tpid_debug_match.group(2))
                        spd_err = float(tpid_debug_match.group(3))
                        pitch_tgt = float(tpid_debug_match.group(4))
                        ang_err = float(tpid_debug_match.group(5))
                        whl_tgt = float(tpid_debug_match.group(6))
                        whl_err = float(tpid_debug_match.group(7))
                        out = float(tpid_debug_match.group(8))
                        wmode = tpid_debug_match.group(9)
                        if hasattr(self, 'dual_pid_panel'):
                            self.dual_pid_panel.update_tpid_debug(
                                pitch, spd, spd_err, pitch_tgt, ang_err, whl_tgt, whl_err, out, wmode)
                    except:
                        pass
            if not self.debug_mode and not self.show_high_freq_data:
                return  # 不打印日志(非debug模式)
        
        # X-Offset 调试输出
        # 格式: [XOFF] spd=X → x_off=Xm (Kp=X Ki=X Kd=X lim=X)
        if line.startswith("[XOFF]"):
            xoff_match = re.search(
                r'\[XOFF\] spd=([-\d.]+) → x_off=([-\d.]+)m', line)
            if xoff_match:
                try:
                    spd = float(xoff_match.group(1))
                    x_off = float(xoff_match.group(2))
                    if hasattr(self, 'balance_panel'):
                        self.balance_panel.xoffset_val_label.setText(
                            f"x_offset: {x_off:.4f} m ({x_off*100:.2f} cm) | speed: {spd:.3f} m/s")
                        self.balance_panel.xoffset_status.setText("状态: 已开启")
                        self.balance_panel.xoffset_status.setStyleSheet(SS(
                            "font-size: 14px; font-weight: bold; color: #4CAF50;"))
                except:
                    pass
            if not self.debug_mode and not self.show_high_freq_data:
                return
        
        # Leg Sync 调试输出
        # 格式: [SYNC] diff=X.XX° → corr=X.XX° (gain=X.XX max=X.X°)
        if line.startswith("[SYNC]"):
            sync_match = re.search(
                r'\[SYNC\] diff=([-\d.]+)° → corr=([-\d.]+)° \(gain=([-\d.]+) max=([-\d.]+)°\)', line)
            if sync_match:
                try:
                    diff = float(sync_match.group(1))
                    corr = float(sync_match.group(2))
                    gain = float(sync_match.group(3))
                    max_c = float(sync_match.group(4))
                    if hasattr(self, 'balance_panel'):
                        self.balance_panel.leg_sync_val_label.setText(
                            f"diff: {diff:.2f}° | correction: {corr:.2f}° | gain: {gain:.2f} | max: {max_c:.1f}°")
                        self.balance_panel.leg_sync_status.setText("状态: 已开启")
                        self.balance_panel.leg_sync_status.setStyleSheet(SS(
                            "font-size: 14px; font-weight: bold; color: #4CAF50;"))
                except:
                    pass
            if not self.debug_mode and not self.show_high_freq_data:
                return
        
        # 控制模式切换
        if line.startswith("CTRL_MODE:"):
            ctrl_mode_match = re.search(r'CTRL_MODE:(LQR|DUAL_PID|SINGLE_PID|CAR|TRIPLE_PID)', line)
            if ctrl_mode_match:
                try:
                    mode = ctrl_mode_match.group(1)
                    if hasattr(self, 'dual_pid_panel'):
                        self.dual_pid_panel.update_mode(mode)
                except:
                    pass
            if not self.debug_mode and not self.show_high_freq_data:
                return  # 不打印日志(非debug模式)
        
        # WMA 滤波器状态反馈
        if line.startswith("WMA_STATUS:"):
            wma_match = re.search(r'WMA_STATUS:(\d)', line)
            if wma_match:
                try:
                    enabled = int(wma_match.group(1)) == 1
                    if hasattr(self, 'dual_pid_panel') and hasattr(self.dual_pid_panel, 'wma_toggle_btn'):
                        self.dual_pid_panel.wma_toggle_btn.setChecked(enabled)
                        self.dual_pid_panel.wma_toggle_btn.setText(f"WMA 滤波: {'ON' if enabled else 'OFF'}")
                except:
                    pass
        
        # 腿部状态 (高频): LEG_STATE: L_Len=xxx ...
        if line.startswith("LEG_STATE:"):
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
            if not self.debug_mode and not self.show_high_freq_data:
                return  # 不打印日志(非debug模式)
        
        # VMC 数据流 (高频): #VMC,L_len,L_ang,L_FL,L_Fa,L_hip,L_knee,R_len,R_ang,R_FL,R_Fa,R_hip,R_knee,diff,Fsync,L_aFL,L_aFa,R_aFL,R_aFa
        if line.startswith("#VMC,"):
            parts = line.split(',')
            if len(parts) >= 15:  # #VMC + 至少 14 个数据 (兼容旧格式)
                try:
                    vmc_data = {
                        'left_leg_length': float(parts[1]),
                        'left_body_angle': float(parts[2]),
                        'left_F_L': float(parts[3]),
                        'left_F_alpha': float(parts[4]),
                        'left_hip_torque': float(parts[5]),
                        'left_knee_torque': float(parts[6]),
                        'right_leg_length': float(parts[7]),
                        'right_body_angle': float(parts[8]),
                        'right_F_L': float(parts[9]),
                        'right_F_alpha': float(parts[10]),
                        'right_hip_torque': float(parts[11]),
                        'right_knee_torque': float(parts[12]),
                        'angle_diff': float(parts[13]),
                        'f_sync': float(parts[14])
                    }
                    # 新增: 从电机电流反解的实际 F_L 和 F_alpha (后4个字段)
                    if len(parts) >= 19:
                        vmc_data['left_actual_F_L'] = float(parts[15])
                        vmc_data['left_actual_F_alpha'] = float(parts[16])
                        vmc_data['right_actual_F_L'] = float(parts[17])
                        vmc_data['right_actual_F_alpha'] = float(parts[18])
                    if hasattr(self, 'vmc_panel'):
                        self.vmc_panel.update_vmc_monitor(vmc_data)
                except (ValueError, IndexError) as e:
                    if self.debug_mode:
                        self.log(f"VMC data parse error: {e}", is_error=True)
            if not self.debug_mode and not self.show_high_freq_data:
                return  # 不打印日志(非debug模式)
        
        # 关节电机数据流 (高频): #JOINT,LH_pos,LH_spd,LH_cur,LH_spd_f,LK_pos,LK_spd,LK_cur,LK_spd_f,RH_pos,RH_spd,RH_cur,RH_spd_f,RK_pos,RK_spd,RK_cur,RK_spd_f
        if line.startswith("#JOINT,"):
            parts = line.split(',')
            if len(parts) == 17:  # #JOINT + 16 个数据 (每关节4个: pos,spd,cur,spd_filtered)
                try:
                    if hasattr(self, 'joint_panel'):
                        self.joint_panel.update_joint_data(
                            float(parts[1]),  float(parts[2]),  float(parts[3]),  float(parts[4]),   # LH pos,spd,cur,spd_f
                            float(parts[5]),  float(parts[6]),  float(parts[7]),  float(parts[8]),   # LK pos,spd,cur,spd_f
                            float(parts[9]),  float(parts[10]), float(parts[11]), float(parts[12]),  # RH pos,spd,cur,spd_f
                            float(parts[13]), float(parts[14]), float(parts[15]), float(parts[16]))  # RK pos,spd,cur,spd_f
                except (ValueError, IndexError) as e:
                    if self.debug_mode:
                        self.log(f"Joint data parse error: {e}", is_error=True)
            if not self.debug_mode and not self.show_high_freq_data:
                return  # 不打印日志(非debug模式)
        
        # 电机功率数据流 (高频): #MPOW,LH_cur,LK_cur,LW_cur,RH_cur,RK_cur,RW_cur,LH_vol,LK_vol,LW_vol,RH_vol,RK_vol,RW_vol
        if line.startswith("#MPOW,"):
            parts = line.split(',')
            if len(parts) == 13:  # #MPOW + 12 values (6 currents + 6 voltages)
                try:
                    currents = [float(parts[i]) for i in range(1, 7)]
                    voltages = [float(parts[i]) for i in range(7, 13)]
                    if hasattr(self, 'mpow_panel'):
                        self.mpow_panel.update_mpow_data(currents, voltages)
                except (ValueError, IndexError) as e:
                    if self.debug_mode:
                        self.log(f"MPOW data parse error: {e}", is_error=True)
            if not self.debug_mode and not self.show_high_freq_data:
                return  # 不打印日志(非debug模式)
        
        # 速度/位移观测器数据流 (高频): #OBSV,v_raw,v_encoder,v_filter,x_filter,a_imu
        if line.startswith("#OBSV,"):
            parts = line.split(',')
            if len(parts) == 6:  # #OBSV + 5 values
                try:
                    vals = [float(parts[i]) for i in range(1, 6)]
                    if hasattr(self, 'obsv_panel'):
                        self.obsv_panel.update_obsv_data(*vals)
                except (ValueError, IndexError) as e:
                    if self.debug_mode:
                        self.log(f"OBSV data parse error: {e}", is_error=True)
            if not self.debug_mode and not self.show_high_freq_data:
                return  # 不打印日志(非debug模式)
        
        # Full LQR 数据流 (高频): #FLQR,T_L,T_R,Tp_L,Tp_R,L0,theta,d_theta,x,v,pitch,pitch_rate,split
        if line.startswith("#FLQR,"):
            parts = line.split(',')
            if len(parts) == 13:  # #FLQR + 12 values
                try:
                    vals = [float(parts[i]) for i in range(1, 13)]
                    if hasattr(self, 'flqr_panel'):
                        self.flqr_panel.update_flqr_data(*vals)
                except (ValueError, IndexError) as e:
                    if self.debug_mode:
                        self.log(f"FLQR data parse error: {e}", is_error=True)
            if not self.debug_mode and not self.show_high_freq_data:
                return  # 不打印日志(非debug模式)
        
        # IMU 高频数据 (Angle: Roll= ... Pitch= ... Yaw= ...)
        if line.startswith("Angle:") or "Roll=" in line and "Pitch=" in line:
            imu_match = re.search(r'Roll[=:]\s*([-+]?\d+\.?\d*)\s+Pitch[=:]\s*([-+]?\d+\.?\d*)\s+Yaw[=:]\s*([-+]?\d+\.?\d*)', line, re.IGNORECASE)
            if imu_match:
                try:
                    roll = float(imu_match.group(1))
                    pitch = float(imu_match.group(2))
                    yaw = float(imu_match.group(3))
                    if hasattr(self, 'imu_panel') and self.imu_panel is not None:
                        self.imu_panel.update_imu_data(roll, pitch, yaw, 0, 0, 0)
                        self.status_label.setText(f"R:{roll:.1f}° P:{pitch:.1f}° Y:{yaw:.1f}°")
                    if hasattr(self, 'leg_panel') and self.leg_panel is not None:
                        if hasattr(self.leg_panel, 'update_roll_display'):
                            self.leg_panel.update_roll_display(roll)
                except:
                    pass
            if not self.debug_mode and not self.show_high_freq_data:
                return  # 不打印日志 (高频,非debug模式)
        
        # Gyro 高频数据
        if line.startswith("Gyro:"):
            gyro_match = re.search(r'Gyro:\s*X[=:]\s*([-+]?\d+\.?\d*)\s+Y[=:]\s*([-+]?\d+\.?\d*)\s+Z[=:]\s*([-+]?\d+\.?\d*)', line, re.IGNORECASE)
            if gyro_match:
                try:
                    gx = float(gyro_match.group(1))
                    gy = float(gyro_match.group(2))
                    gz = float(gyro_match.group(3))
                    if hasattr(self, 'imu_panel') and self.imu_panel is not None:
                        self.imu_panel.gx_label.setText(f"{gx:.2f}°/s")
                        self.imu_panel.gy_label.setText(f"{gy:.2f}°/s")
                        self.imu_panel.gz_label.setText(f"{gz:.2f}°/s")
                except:
                    pass
            if not self.debug_mode and not self.show_high_freq_data:
                return  # 不打印日志 (高频,非debug模式)
        
        # 频率数据 (定期输出，但不需要刷屏)
        if line.startswith("FREQ:"):
            freq_match = re.search(r'FREQ:IMU=([-\d.]+),CTRL=([-\d.]+),MOTOR=([-\d.]+),LEG=([-\d.]+)', line)
            if freq_match:
                try:
                    imu_hz = float(freq_match.group(1))
                    ctrl_hz = float(freq_match.group(2))
                    motor_hz = float(freq_match.group(3))
                    leg_hz = float(freq_match.group(4))
                    if hasattr(self, 'balance_panel'):
                        self.balance_panel.update_task_freq(imu_hz, ctrl_hz, motor_hz, leg_hz)
                except:
                    pass
            if not self.debug_mode and not self.show_high_freq_data:
                return  # 不打印日志(非debug模式)
        
        # 延迟数据 (定期输出)
        if line.startswith("LATENCY:"):
            latency_match = re.search(r'LATENCY:IMU_CTRL=([-\d.]+),CALC=([-\d.]+),CTRL_MOTOR=([-\d.]+),TOTAL=([-\d.]+)(?:,AVG=([-\d.]+),MIN=([-\d.]+),MAX=([-\d.]+))?', line)
            if latency_match:
                try:
                    imu_ctrl_us = float(latency_match.group(1))
                    calc_us = float(latency_match.group(2))
                    ctrl_motor_us = float(latency_match.group(3))
                    total_us = float(latency_match.group(4))
                    avg_us = float(latency_match.group(5)) if latency_match.group(5) else total_us
                    min_us = float(latency_match.group(6)) if latency_match.group(6) else total_us
                    max_us = float(latency_match.group(7)) if latency_match.group(7) else total_us
                    if hasattr(self, 'balance_panel'):
                        self.balance_panel.update_latency(imu_ctrl_us, calc_us, ctrl_motor_us, total_us, avg_us, min_us, max_us)
                except:
                    pass
            if not self.debug_mode and not self.show_high_freq_data:
                return  # 不打印日志(非debug模式)
        
        # WiFi 延迟数据
        if line.startswith("WIFI_LATENCY:"):
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
                except:
                    pass
            if not self.debug_mode and not self.show_high_freq_data:
                return  # 不打印日志(非debug模式)
        
        # ===== 以下是低频/重要数据，会打印到日志 =====
        
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
            # QScrollArea 包裹时，需要取出内部的实际 widget
            if isinstance(current_widget, QScrollArea):
                current_widget = current_widget.widget()
            if isinstance(current_widget, PIDControlPanel):
                current_widget.update_display(p, i, d, limit, ramp)
        
        # 解析LPF返回
        match = re.search(r'LPF:\s*Tf:\s*([-\d.]+)', line)
        if match:
            tf = float(match.group(1))
            current_widget = self.tab_widget.currentWidget()
            # QScrollArea 包裹时，需要取出内部的实际 widget
            if isinstance(current_widget, QScrollArea):
                current_widget = current_widget.widget()
            if isinstance(current_widget, LPFControlPanel):
                current_widget.update_display(tf)
        
        # 解析速度自适应P参数 (只有Low和High两个参数)
        match = re.search(r'Speed Adaptive P:\s*Low=([-\d.]+)\s*High=([-\d.]+)', line)
        if match:
            low = float(match.group(1))
            high = float(match.group(2))
            if self.speed_adaptive_panel:
                self.speed_adaptive_panel.update_display(low, high)
        
        # 解析轮速滤波参数 (双模式)
        # 格式: SpeedFilter: Mode=0 Tf=0.0100 Rate=50.0000
        match = re.search(r'SpeedFilter:\s*Mode=(\d+)\s*Tf=([-\d.]+)\s*Rate=([-\d.]+)', line)
        if match:
            mode = int(match.group(1))
            tf = float(match.group(2))
            rate = float(match.group(3))
            if 'speed' in self.lpf_panels and isinstance(self.lpf_panels['speed'], SpeedFilterPanel):
                self.lpf_panels['speed'].update_display(mode, tf, rate)
        
        # 解析角速度滤波参数 (双模式)
        # 格式: GyroFilter: Mode=1 Tf=0.0050 Rate=500.0000
        match = re.search(r'GyroFilter:\s*Mode=(\d+)\s*Tf=([-\d.]+)\s*Rate=([-\d.]+)', line)
        if match:
            mode = int(match.group(1))
            tf = float(match.group(2))
            rate = float(match.group(3))
            if 'gyro' in self.lpf_panels and isinstance(self.lpf_panels['gyro'], GyroFilterPanel):
                self.lpf_panels['gyro'].update_display(mode, tf, rate)
        
        # 解析角度零点查询响应 (新格式: 完整自适应状态)
        # 格式: "  当前零点: %.3f°"
        match = re.search(r'当前零点:\s*([-\d.]+)', line)
        if match:
            zeropoint = float(match.group(1))
            if hasattr(self, 'balance_panel') and hasattr(self.balance_panel, 'zero_input'):
                self.balance_panel.zero_input.setValue(zeropoint)
            if hasattr(self, 'dual_pid_panel') and hasattr(self.dual_pid_panel, 'zero_input'):
                self.dual_pid_panel.zero_input.setValue(zeropoint)
        
        # 解析自适应状态多行信息, 拼成状态文本
        # "  当前pitch: %.3f°"
        match = re.search(r'当前pitch:\s*([-\d.]+)', line)
        if match and hasattr(self, 'balance_panel') and hasattr(self.balance_panel, 'zp_status_label'):
            self._zp_status_pitch = float(match.group(1))
        # "  角度误差: %.3f°"
        match = re.search(r'角度误差:\s*([-\d.]+)', line)
        if match:
            self._zp_status_err = float(match.group(1))
        # "  PID输出: raw=%.6f, filtered=%.6f"
        match = re.search(r'PID输出:\s*raw=([-\d.]+),\s*filtered=([-\d.]+)', line)
        if match:
            self._zp_status_raw = float(match.group(1))
            self._zp_status_filt = float(match.group(2))
        # "  轮速阈值: %.3f m/s (当前轮速: %.3f)"
        match = re.search(r'轮速阈值:\s*([-\d.]+)\s*m/s.*当前轮速:\s*([-\d.]+)', line)
        if match:
            thr = float(match.group(1))
            spd = float(match.group(2))
            if hasattr(self, 'balance_panel') and hasattr(self.balance_panel, 'zp_threshold_input'):
                self.balance_panel.zp_threshold_input.setValue(thr)
            self._zp_status_thr = thr
            self._zp_status_spd = spd
        # "  PID激活: YES/NO"
        match = re.search(r'PID激活:\s*(YES|NO)', line)
        if match:
            active = match.group(1)
            self._zp_status_active = active
        # "  PID参数: kp=%.6f, ki=%.6f, kd=%.6f"
        match = re.search(r'PID参数:\s*kp=([-\d.]+),\s*ki=([-\d.]+),\s*kd=([-\d.]+)', line)
        if match:
            kp = match.group(1)
            ki = match.group(2)
            kd = match.group(3)
            # 最后一行, 拼接完整状态
            if hasattr(self, 'balance_panel') and hasattr(self.balance_panel, 'zp_status_label'):
                zp_val = getattr(self, '_zp_status_pitch', 0)
                err_val = getattr(self, '_zp_status_err', 0)
                raw_val = getattr(self, '_zp_status_raw', 0)
                filt_val = getattr(self, '_zp_status_filt', 0)
                thr_val = getattr(self, '_zp_status_thr', 0.1)
                spd_val = getattr(self, '_zp_status_spd', 0)
                act_val = getattr(self, '_zp_status_active', '?')
                status_text = (f"pitch={zp_val:.2f}° | 误差={err_val:.3f}° | "
                               f"PID: raw={raw_val:.6f} filt={filt_val:.6f} | "
                               f"轮速={spd_val:.3f}/{thr_val:.3f} | "
                               f"激活={act_val} | kp={kp} ki={ki} kd={kd}")
                self.balance_panel.zp_status_label.setText(status_text)
                color = "#4CAF50" if act_val == "YES" else "#FF9800"
                self.balance_panel.zp_status_label.setStyleSheet(SS(f"color: {color}; font-size: 10px;"))
            self.log(f"✓ 零点自适应状态已更新", is_receive=True)
        
        # 兼容旧格式: "Current angle zeropoint: %.2f"
        match = re.search(r'Current angle zeropoint:\s*([-\d.]+)', line)
        if match:
            zeropoint = float(match.group(1))
            if hasattr(self, 'balance_panel') and hasattr(self.balance_panel, 'zero_input'):
                self.balance_panel.zero_input.setValue(zeropoint)
            if hasattr(self, 'dual_pid_panel') and hasattr(self.dual_pid_panel, 'zero_input'):
                self.dual_pid_panel.zero_input.setValue(zeropoint)
            self.log(f"✓ 零点已同步: {zeropoint:.2f}°", is_receive=True)
        
        # 解析零点设置确认响应
        # 格式: Angle zeropoint set to %.2f (LQR)
        # 格式: Dual PID angle zeropoint set to %.2f
        # 格式: Single PID angle zeropoint set to %.2f
        match = re.search(r'(?:Dual PID |Single PID )?[Aa]ngle zeropoint set to\s*([-\d.]+)', line)
        if match:
            zeropoint = float(match.group(1))
            # 设置后同步到所有面板 (固件已同步所有控制器)
            if hasattr(self, 'balance_panel') and hasattr(self.balance_panel, 'zero_input'):
                self.balance_panel.zero_input.setValue(zeropoint)
            if hasattr(self, 'dual_pid_panel') and hasattr(self.dual_pid_panel, 'zero_input'):
                self.dual_pid_panel.zero_input.setValue(zeropoint)
            self.log(f"✓ 零点已设置: {zeropoint:.2f}°", is_receive=True)
        
        # 解析零点轮速阈值设置确认
        match = re.search(r'Zeropoint speed threshold set to\s*([-\d.]+)', line)
        if match:
            thr = float(match.group(1))
            if hasattr(self, 'balance_panel') and hasattr(self.balance_panel, 'zp_threshold_input'):
                self.balance_panel.zp_threshold_input.setValue(thr)
            self.log(f"✓ 零点轮速阈值: {thr:.3f} m/s", is_receive=True)
        
        # ===== 新增: 设备控制面板数据解析 =====
        
        # 检测 balance init 成功
        if "Balance test init: OK" in line or "Balance test module initialized" in line:
            if hasattr(self, 'balance_panel'):
                self.balance_panel.on_init_success()
            self.log(f"✓ {line}", is_receive=True)
        
        # 检测 IMU 数据: Roll=xxx Pitch=xxx Yaw=xxx (已在前面处理高频数据)
        # 这里处理一次性查询的IMU数据 (不是以 "Angle:" 开头的)
        
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
        
        # 目标设置成功: Target set: Length=xxxm, Angle=xxxdeg
        target_match = re.search(r'Target set:\s*Length=([-\d.]+)m,\s*Angle=([-\d.]+)deg', line)
        if target_match:
            self.log(f"✓ {line}", is_receive=True)
    
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
    
    def toggle_high_freq_data(self, enabled):
        self.show_high_freq_data = enabled
        if enabled:
            self.log("📊 高频数据显示已开启 - IMU/腿部/频率等数据将显示在日志中", is_error=True)
        else:
            self.log("📊 高频数据显示已关闭", is_error=True)
    
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
    
    # 计算 DPI 缩放因子并应用全局自适应样式
    _compute_scale_factor(app)
    app.setStyleSheet(_build_global_stylesheet())
    print(f"[UI] Screen scale factor: {_SCALE_FACTOR:.2f}")
    
    window = PIDTunerUI()
    window.show()
    sys.exit(app.exec_())
