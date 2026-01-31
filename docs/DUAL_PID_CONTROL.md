# 双环 PID 平衡控制器

## 概述

双环 PID 是一种简单直观的平衡控制方案，相比 LQR 多环控制更容易理解和调参。

### 控制架构

```
                    ┌─────────────────┐
   目标角度 (0°) ───►│   直立环 PID    │───► 目标速度
                    │  (外环/角度环)   │
                    └────────┬────────┘
                             │
   实际 Pitch ───────────────┘
   
                    ┌─────────────────┐
     目标速度 ─────►│   速度环 PID    │───► 输出扭矩
                    │  (内环/速度环)   │
                    └────────┬────────┘
                             │
   实际轮速 ─────────────────┘
```

### 控制原理

1. **直立环（外环）**: 检测机器人倾斜角度，输出期望的轮子速度
   - 当机器人**前倾** (pitch > 0) → 输出**正速度**（向前加速追赶重心）
   - 当机器人**后倾** (pitch < 0) → 输出**负速度**（向后加速追赶重心）

2. **速度环（内环）**: 跟踪直立环给出的目标速度，输出电机扭矩
   - 速度误差 = 目标速度 - 实际轮速
   - 扭矩 = PID(速度误差)

## CLI 命令

### 切换控制模式

```bash
# 查看当前模式
balance mode

# 切换到 LQR 模式 (默认)
balance mode lqr

# 切换到双环 PID 模式
balance mode pid
```

### 双环 PID 调参

```bash
# 查看当前参数
balance dpid

# 设置直立环 PID (kp ki kd)
balance dpid angle 15.0 0.0 0.5

# 设置速度环 PID (kp ki kd)
balance dpid speed 0.5 0.1 0.01

# 设置角度零点
balance dpid zero -0.5

# 重置 PID 控制器
balance dpid reset

# 查看实时状态
balance dpid status
```

## 默认参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| **直立环** | | |
| angle_kp | 15.0 | 角度P增益 |
| angle_ki | 0.0 | 角度I增益 |
| angle_kd | 0.5 | 角度D增益 |
| angle_limit | 10.0 | 最大目标速度 (rad/s) |
| **速度环** | | |
| speed_kp | 0.5 | 速度P增益 |
| speed_ki | 0.1 | 速度I增益 |
| speed_kd | 0.01 | 速度D增益 |
| speed_limit | 8.0 | 最大输出扭矩 (Nm) |
| **其他** | | |
| angle_zeropoint | 0.0 | 机械零点偏移 (度) |
| emergency_angle | 45.0 | 紧急停止角度 (度) |
| max_torque | 8.0 | 最大输出扭矩 (Nm) |

## 调参指南

### 1. 先调直立环（外环）

直立环决定了机器人对倾斜的响应速度。

```bash
# 从小 P 开始
balance dpid angle 5.0 0 0

# 逐步增大 P，直到机器人能稳定站立但有轻微震荡
balance dpid angle 10.0 0 0
balance dpid angle 15.0 0 0

# 加入 D 消除震荡
balance dpid angle 15.0 0 0.3
balance dpid angle 15.0 0 0.5

# 一般不需要 I，除非有稳态误差
```

**症状诊断**：
- 前倾后不回正 → 增大 Kp
- 来回震荡 → 减小 Kp 或增大 Kd
- 高频抖动 → 减小 Kd

### 2. 再调速度环（内环）

速度环决定了扭矩输出的响应。

```bash
# 从小 P 开始
balance dpid speed 0.3 0 0

# 加入 I 消除稳态误差
balance dpid speed 0.3 0.05 0

# 逐步调整
balance dpid speed 0.5 0.1 0.01
```

**症状诊断**：
- 响应慢 → 增大 Kp
- 过冲/震荡 → 减小 Kp，增大 Kd
- 无法保持目标速度 → 增大 Ki

### 3. 调整角度零点

如果机器人静止时不垂直：

```bash
# 查看当前 pitch
balance status

# 如果 pitch 显示 -0.5 时机器人平衡，设置零点
balance dpid zero -0.5
```

## 与 LQR 模式对比

| 特性 | 双环 PID | LQR 多环 |
|------|----------|----------|
| 复杂度 | 简单 (2 个 PID) | 复杂 (4+ 个 PID) |
| 调参难度 | 较易 | 较难 |
| 位置保持 | ❌ 无 | ✅ 有位移环 |
| 抗干扰 | 一般 | 较强 |
| 适用场景 | 测试/简单平衡 | 完整功能 |

## 代码结构

```
components/algorithm/
├── include/lqr_balance.h    # 双环 PID 结构和接口定义
└── src/lqr_balance.c        # 双环 PID 实现

components/app/
└── src/balance_test.c       # 控制模式切换和 CLI 命令
```

### 关键函数

```c
// 初始化
dual_pid_init(&ctrl, NULL);

// 设置参数
dual_pid_set_angle_gains(&ctrl, kp, ki, kd);
dual_pid_set_speed_gains(&ctrl, kp, ki, kd);

// 控制循环
dual_pid_balance_loop(&ctrl, pitch, pitch_rate, wheel_speed, dt, &output);

// 输出
output.torque;          // 电机扭矩
output.target_speed;    // 中间变量：目标速度
output.angle_error;     // 调试：角度误差
output.speed_error;     // 调试：速度误差
```

## 波形数据

双环 PID 模式下，波形数据通道含义：

| 通道 | 名称 | 说明 |
|------|------|------|
| A | Angle | 角度误差 (目标0 - 实际pitch) |
| B | Gyro | 角度D项输出 |
| D | Speed | 速度P项输出 |
| G | Filter | 滤波后目标速度 |
| H | LQR_u | 输出扭矩 |

## 注意事项

1. **先在 LQR 模式调好角度零点**，再切换到双环 PID
2. 双环 PID 没有位移环，机器人可能会缓慢漂移
3. 切换模式后会自动重置 PID 积分，避免跳变
4. YAW 控制在两种模式下都可用（需要 go=true）
5. 紧急停止逻辑两种模式共用

## Qt 调参工具支持

在 Qt 调参工具中可以通过串口发送以下命令：

```
balance mode pid       # 切换到双环 PID 模式
balance dpid angle 15 0 0.5  # 设置直立环参数
balance dpid speed 0.5 0.1 0  # 设置速度环参数
balance dpid status    # 查询状态
```

响应格式（可解析）：
```
DPID:ANGLE,15.0000,0.0000,0.5000
DPID:SPEED,0.5000,0.1000,0.0000
DPID_STATUS:0.50,-1.23,0.77,0.38
```
