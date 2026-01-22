# PID调参面板

Qt5 + PyQt5 实现的PID参数调节工具，用于实时调节轮腿机器人的控制参数。

基于 [shibo_wheel_leg](https://github.com/pxxxxxx/shibo_wheel_leg) 项目的调参工具。

## 文件说明

| 文件 | 说明 |
|:-----|:-----|
| `pid_tuner.py` | PID参数调节面板，支持多个控制器的P/I/D/Limit/Ramp参数调节 |
| `robot_monitor.py` | 机器人状态监控面板，显示IMU、编码器、伺服等实时数据 |
| `requirements.txt` | Python依赖包列表 |

## 功能特性

### pid_tuner.py
- **多控制器支持**：角度控制、角速度控制、位移控制、速度控制、YAW控制、Roll控制等
- **LPF滤波器调节**：摇杆滤波、零点滤波、Roll滤波
- **速度自适应P值**：根据机身高度调整速度环P值
- **实时数据绘图**：显示目标值和控制输出波形
- **Web遥控器状态监控**：显示遥控器的按键和摇杆状态
- **串口通信**：支持多种波特率

### robot_monitor.py
- **系统状态**：运行频率、电池电压
- **IMU数据**：姿态角度、角速度
- **编码器反馈**：电机位置和速度
- **伺服位置**：各轴角度显示
- **实时波形**：频率和电机速度曲线

## 依赖安装

```bash
cd tools/qt_tuner
pip install -r requirements.txt
```

或手动安装：
```bash
pip install pyqt5 pyserial pyqtgraph
```

## 运行方式

```bash
cd tools/qt_tuner

# PID调参面板
python pid_tuner.py

# 机器人状态监控
python robot_monitor.py
```

## 通信协议

### 发送格式（下位机接收）

```
<控制器ID><参数字符><数值>
```

**控制器ID列表：**
| ID | 类型 | 说明 |
|:---|:-----|:-----|
| A | PID | 角度控制 (Angle) |
| B | PID | 角速度控制 (Gyro) |
| C | PID | 位移控制 (Distance) |
| D | PID | 速度控制 (Speed) |
| E | PID | YAW角度控制 |
| F | PID | YAW角速度控制 |
| G | LPF | 摇杆Y轴滤波 |
| H | PID | LQR输出补偿 |
| I | PID | 零点自适应 |
| J | LPF | 零点滤波 |
| K | PID | Roll轴平衡 |
| L | LPF | Roll角度滤波 |
| M | 自适应 | 速度自适应参数 |

**PID参数字符（用于A,B,C,D,E,F,H,I,K控制器）：**
| 字符 | 说明 |
|:-----|:-----|
| P | P值 |
| I | I值 |
| D | D值 |
| L | 输出限幅 |
| R | 斜坡限制 |
| T | 低通滤波时间常数 |
| ? | 查询当前参数 |

**LPF参数字符（用于G,J,L控制器）：**
| 字符 | 说明 |
|:-----|:-----|
| T | 滤波时间常数 |

**速度自适应参数字符（用于M控制器）：**
| 字符 | 说明 |
|:-----|:-----|
| L | Kp_Min (最小P值) |
| H | Kp_Max (最大P值) |

**示例：**
- `AP1.5` - 设置角度控制器的P值为1.5
- `BD0.01` - 设置角速度控制器的D值为0.01
- `GT0.05` - 设置摇杆Y轴LPF的时间常数为0.05
- `ML0.3` - 设置速度自适应 Kp_Min 为 0.3
- `MH1.0` - 设置速度自适应 Kp_Max 为 1.0
- `A?` - 查询角度控制器的所有参数

### 接收格式（下位机发送）

**数据流格式（用于绘图）：**
```
#DATA,<ID>,<Target>,<Control>
```

| ID | 说明 |
|:---|:-----|
| A | 角度 (target=0平衡点, control=当前角度) |
| B | 角速度 (target=0, control=当前角速度) |
| C | 位移 (target=目标位移, control=当前位移) |
| D | 速度 (target=目标速度, control=当前速度) |
| H | LQR输出 (target=0, control=LQR_u) |
| K | Roll角度 (target=0, control=当前Roll) |

例如：`#DATA,A,0.0,5.2` 表示角度目标为0度，当前角度5.2度

**Web遥控器状态格式：**
```
#WEB,<go>,<dir>,<joyx>,<joyy>,<height>
```

例如：`#WEB,1,0,0.5,0.3,50`

## ESP32端对接

设备端已实现 `commander_parser` 组件，并与 `balance_test` 模块集成。

**相关文件：**
- `components/device/include/commander_parser.h` - 头文件
- `components/device/src/commander_parser.c` - 解析器实现
- `components/app/src/balance_test.c` - 平衡测试（含参数同步回调）
- `components/app/src/motor_test.c` - 命令行入口

**使用步骤：**

1. **烧录固件并连接串口**
```bash
idf.py flash monitor
```

2. **初始化平衡测试模块**
```
motor> balance init
```

3. **通过串口发送调参命令**
```
motor> AP1.5      # 设置角度控制器 P=1.5
motor> BD0.01     # 设置角速度控制器 D=0.01
motor> A?         # 查询角度控制器参数
motor> tune status # 打印所有参数
```

4. **开启波形输出 (用于面板绘图)**
```
motor> balance plot on    # 开启波形输出
motor> balance plot div 5 # 设置分频系数为5 (40Hz输出)
motor> balance plot off   # 关闭波形输出
```

5. **或使用 Python 调参面板**
```bash
cd tools/qt_tuner
python pid_tuner.py
```
选择 ESP32 的串口，连接后即可图形化调参。

**参数自动同步：**
- Commander 解析器收到命令后，会自动调用回调函数
- 回调函数将参数同步到 LQR 控制器 (`lqr_set_params`)
- 参数修改实时生效，无需重启

## 注意事项

1. 确保ESP32和电脑的串口波特率一致（默认115200）
2. 调参时建议先小幅度调整，观察机器人响应
3. 建议先在静止状态下测试参数，再进行动态测试
4. 可使用查询命令（如 `A?`）获取当前参数值

