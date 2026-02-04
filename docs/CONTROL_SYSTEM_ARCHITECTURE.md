# 轮腿机器人控制系统架构文档

## 1. 系统概述

本文档描述了轮腿机器人控制系统的完整架构，包括电机方向约定、坐标系定义、控制算法和数据流。

### 1.1 电机方向约定（关键！）

**轮电机方向：**
- 两轮力矩/速度**为负**时，机器人**前进**
- 两轮力矩/速度**为正**时，机器人**后退**
- 左轮正、右轮负时，机器人**左转**
- 左轮负、右轮正时，机器人**右转**

**腿电机方向：**
- 左腿：hip_offset = -15°, knee_offset = +35°
- 右腿：hip_offset = +15°, knee_offset = -35°（镜像安装）
- 右腿运动学角度 = -电机角度（镜像处理）
- 右腿扭矩 = -计算扭矩（镜像输出）

### 1.2 坐标系定义

**机身坐标系 (Body Frame, L-α)：**
```
    ← 前 (Forward)
    
        机身
         │
         │ α = body_angle
         │   α = -90°: 垂直向下（正常站立）
         │   α > -90°: 腿向后摆
         │   α < -90°: 腿向前摆
         ↓
        轮子
        
    L = 腿长（髋关节到轮子距离）
```

**世界坐标系 (World Frame, x-y)：**
```
    y ↑ (向上为正)
      │
      │  原点 = 髋关节
      │
      └──────→ x (向后为正)
      
    y < 0: 轮子在髋关节下方
```

**IMU 姿态约定：**
- pitch > 0: 机身前倾
- pitch < 0: 机身后倾
- roll > 0: 机身右倾
- roll < 0: 机身左倾

---

## 2. 控制模式

### 2.1 LQR 多环控制（默认）

```
输入: pitch, pitch_rate, distance, speed, joy_y
输出: wheel_torque

LQR_u = angle_control + gyro_control + distance_control + speed_control

各环符号约定:
- pitch > 0 (前倾) → angle_control < 0 → 负扭矩 → 轮子加速前进 → 追上倾倒
- speed > target (实际比期望快) → speed_control > 0 → 正扭矩 → 减速
```

### 2.2 双环 PID 控制

```
外环 (直立环): pitch → target_speed
  - pitch > 0 (前倾) → target_speed < 0 (希望前进)
  
内环 (速度环): speed_error → torque
  - target_speed < wheel_speed → speed_error < 0 → torque > 0 → 减速
  - target_speed > wheel_speed → speed_error > 0 → torque < 0 → 加速
```

### 2.3 VMC 力控模式

```
机身坐标系输入: F_L, F_alpha
  - F_L > 0: 腿伸展力（支撑重力）
  - F_alpha > 0: 身体角度向后力矩（从左侧看逆时针，与 body_angle 正方向一致）

扭矩计算: τ = J^T × F
  - 左腿: 直接使用
  - 右腿: 取反（镜像安装补偿）
```

---

## 3. 数据流架构

### 3.1 当前架构问题

```
问题1: 重复计算
  - deg→rad 转换在多处重复
  - rpm→rad/s 转换在多处重复
  - FK/雅可比在 VMC 和 leg_ctrl 中重复计算

问题2: 职责混乱
  - VMC 函数内部计算 PD 力（应该由调用者提供）
  - LQR 的 yaw/roll 环与 PID 模式不共享
  
问题3: 单位不统一
  - 有的用度，有的用弧度
  - 转换分散在各处
```

### 3.2 重构后架构

```
┌─────────────────────────────────────────────────────────────┐
│                    robot_state_t (弧度制)                    │
│  统一存储所有传感器数据，只计算一次，供所有算法使用            │
├─────────────────────────────────────────────────────────────┤
│ IMU (imu_state_t):                                          │
│   pitch, pitch_rate, pitch_acc (rad, rad/s, rad/s²)         │
│   roll, roll_rate, roll_acc (rad, rad/s, rad/s²)            │
│   yaw, yaw_rate, yaw_total (rad, rad/s)                     │
│                                                             │
│ 轮电机 (wheel_state_t):                                      │
│   left_pos, left_vel (rad, rad/s)                           │
│   right_pos, right_vel (rad, rad/s)                         │
│                                                             │
│ 腿电机 (leg_extended_state_t × 2):                          │
│   关节空间:                                                  │
│     hip_pos_deg, knee_pos_deg (度, 用于VMC接口)              │
│     hip_pos, knee_pos (rad)                                  │
│     hip_vel, knee_vel (rad/s)                               │
│     hip_acc, knee_acc (rad/s²)                              │
│   工作空间 (机身坐标系):                                      │
│     leg_length (m), body_angle (rad)                        │
│     dL (m/s), dalpha (rad/s)                                │
│     ddL (m/s²), ddalpha (rad/s²)                            │
│   世界坐标系:                                                │
│     world_x, world_y (m)                                    │
│     world_vx, world_vy (m/s)                                │
│     world_ay (m/s²) - 用于跳跃/着地检测                      │
│   腿相对世界竖直方向:                                        │
│     theta_world = pitch + body_angle + π/2 (rad)            │
│     dtheta_world = pitch_rate + dalpha (rad/s)              │
│     ddtheta_world = pitch_acc + ddalpha (rad/s²)            │
│   雅可比矩阵:                                                │
│     J[4] - 机身坐标系                                        │
│     J_world[4] - 世界坐标系                                  │
│                                                             │
│ 运动学 (robot_kinematics_t):                                │
│   robot_vx (m/s, 向后为正)                                  │
│   lqr_speed (m/s, 前进为正)                                 │
│   lqr_distance (m, 前进为正)                                │
│   wheel_vel_avg (rad/s, 前进为正)                           │
│   body_height (m)                                           │
│                                                             │
│ Pitch 补偿:                                                  │
│   pitch_compensated (rad) - 双腿平均                        │
│   pitch_comp_left, pitch_comp_right (rad) - 单腿            │
│   body_ay_world (m/s²) - 机身竖直加速度                      │
│                                                             │
│ 双腿平均值:                                                  │
│   avg_leg_length, avg_body_angle                            │
│   avg_dL, avg_dalpha                                        │
└─────────────────────────────────────────────────────────────┘
                            │
                            ↓
┌─────────────────────────────────────────────────────────────┐
│                    wheel_balance_t                           │
│  轮电机平衡控制，输出 wheel_torque                           │
├─────────────────────────────────────────────────────────────┤
│ 输入: robot_state_t                                         │
│ 输出: left_wheel_torque, right_wheel_torque                 │
│                                                             │
│ 内部环路 (可独立开关):                                       │
│   - angle_loop: pitch → torque                              │
│   - gyro_loop: pitch_rate → torque                          │
│   - distance_loop: distance → torque                        │
│   - speed_loop: speed → torque                              │
│   - yaw_loop: yaw → differential_torque                     │
└─────────────────────────────────────────────────────────────┘
                            │
                            ↓
┌─────────────────────────────────────────────────────────────┐
│                    leg_force_ctrl_t                          │
│  腿电机力控制，输入虚拟力，输出关节扭矩                       │
├─────────────────────────────────────────────────────────────┤
│ 输入:                                                        │
│   - F_L_left, F_L_right (腿长方向力, N)                      │
│   - F_alpha_left, F_alpha_right (身体角度力矩, Nm)           │
│   - robot_state_t (用于雅可比计算)                           │
│                                                             │
│ 输出:                                                        │
│   - left_hip_torque, left_knee_torque                       │
│   - right_hip_torque, right_knee_torque                     │
│                                                             │
│ 计算:                                                        │
│   τ = J^T × [F_L; F_alpha]                                  │
│   右腿: τ = -τ (镜像补偿)                                    │
└─────────────────────────────────────────────────────────────┘
                            │
                            ↓
┌─────────────────────────────────────────────────────────────┐
│                    高层控制策略                               │
├─────────────────────────────────────────────────────────────┤
│ leg_length_ctrl: 腿长弹簧控制                                │
│   F_L = K_L * (target_L - current_L) + D_L * dL_error       │
│                                                             │
│ body_angle_ctrl: 身体角度弹簧控制                            │
│   F_alpha = K_alpha * (target_alpha - current_alpha) + ...  │
│                                                             │
│ roll_balance_ctrl: Roll 平衡控制                             │
│   left_L_delta = -K_roll * roll                             │
│   right_L_delta = +K_roll * roll                            │
│                                                             │
│ leg_sync_ctrl: 双腿同步控制                                  │
│   angle_diff = left_alpha - right_alpha                     │
│   F_sync = K_sync * angle_diff                              │
│   left: F_alpha -= F_sync                                   │
│   right: F_alpha += F_sync                                  │
│                                                             │
│ gravity_comp: 重力补偿                                       │
│   F_gravity = m * g * cos(pitch + alpha + 90°)              │
└─────────────────────────────────────────────────────────────┘
```

---

## 4. UI 调参对接

### 4.1 Commander 协议 (A-M)

| ID | 控制器 | 参数 |
|----|--------|------|
| A | 角度环 | P, I, D, Limit |
| B | 角速度环 | P, I, D, Limit |
| C | 位移环 | P, I, D, Limit |
| D | 速度环 | P, I, D, Limit |
| E | YAW角度 | P, I, D, Limit |
| F | YAW角速度 | P, I, D, Limit |
| G | 摇杆滤波 | Tf |
| H | LQR输出 | P, I, D, Limit |
| I | 零点自适应 | P, I, D, Limit |
| J | 零点滤波 | Tf |
| K | Roll控制 | P, I, D, Limit |
| L | Roll滤波 | Tf |
| M | 速度自适应 | Min, Max |

### 4.2 波形输出格式

```
#P,pitch,speed,lqr_u,yaw,left_L,right_L
```

---

## 5. 文件结构

```
components/
├── algorithm/
│   ├── include/
│   │   ├── robot_state.h       # 机器人状态结构体定义
│   │   ├── wheel_balance.h     # 轮平衡控制器
│   │   ├── leg_force_ctrl.h    # 腿力控制器 (纯雅可比转换)
│   │   ├── leg_kinematics.h    # 腿运动学 (FK/IK/雅可比)
│   │   ├── pid_controller.h    # PID 控制器
│   │   └── lowpass_filter.h    # 低通滤波器
│   └── src/
│       ├── robot_state.c       # 状态计算 (单位转换、FK)
│       ├── wheel_balance.c     # 轮平衡控制
│       ├── leg_force_ctrl.c    # 腿力控制
│       └── leg_kinematics.c    # 运动学计算
└── app/
    └── src/
        └── balance_test.c      # 主控制任务
```

---

## 6. 符号速查表

| 变量 | 单位 | 正方向 | 说明 |
|------|------|--------|------|
| pitch | rad | 前倾 | IMU 俯仰角 |
| roll | rad | 右倾 | IMU 横滚角 |
| yaw | rad | 逆时针 | IMU 偏航角 |
| wheel_vel | rad/s | 见下 | 正转时机器人后退 |
| robot_vx | m/s | 向后 | 机器人水平速度 |
| lqr_speed | m/s | 前进 | 机器人线速度 (= -robot_vx) |
| body_angle | rad | 向后 | -90°=垂直, >-90°向后 |
| F_L | N | 伸展 | 腿长方向力 |
| F_alpha | Nm | 向后 | 身体角度力矩（从左侧看逆时针）|
| wheel_torque | Nm | 见下 | 负=前进, 正=后退 |
| leg_torque | Nm | 见实现 | 右腿需取反 |

---

## 7. 重构计划

### 第一阶段：创建通用状态结构 ✅ 已完成
1. ✅ 创建 `robot_state.h` 定义统一状态结构
   - `sensor_raw_data_t`: 原始传感器数据（度/rpm）
   - `robot_state_t`: 转换后状态（弧度/rad/s）
   - `leg_extended_state_t`: 腿部完整状态
2. ✅ 创建 `robot_state.c` 实现一次性计算
   - `robot_state_update()`: 单位转换 + FK + 雅可比 + 加速度
   - YAW 过零处理
   - 位移累积
3. ✅ 添加完整的腿部状态计算:
   - 关节空间: `hip_pos`, `knee_pos` (rad), `hip_vel`, `knee_vel` (rad/s)
   - 关节加速度: `hip_acc`, `knee_acc` (rad/s²)
   - 工作空间: `leg_length`, `body_angle`, `dL`, `dalpha`
   - 二阶导: `ddL` (m/s²), `ddalpha` (rad/s²)
   - 世界坐标系: `world_x`, `world_y`, `world_vx`, `world_vy`, `world_ay`
   - 腿相对世界竖直: `theta_world = pitch + body_angle + π/2`
4. ✅ Pitch 补偿 (考虑腿部角度):
   - `pitch_compensated`: 双腿平均补偿
   - `pitch_comp_left`, `pitch_comp_right`: 单腿补偿
5. ⏳ 在 `balance_test.c` 中使用新结构（待实施）

### 第二阶段：重构 VMC ✅ 已完成
1. ✅ 添加 `vmc_force_to_torque()` - 纯雅可比转换接口
   - 直接接受 F_L, F_alpha 输入
   - 自动处理右腿扭矩取反
2. ✅ 添加 `vmc_dual_force_to_torque()` - 双腿版本
3. 原有 VMC 接口保持兼容

### 第三阶段：统一控制环路（待实施）
1. ⏳ yaw/roll 环路独立于控制模式
2. ⏳ LQR 和 PID 模式共享这些环路

### 第四阶段：清理重复代码（待实施）
1. ⏳ 删除重复的单位转换
2. ⏳ 删除重复的 FK 计算
3. ⏳ 统一使用 `robot_state_t`

---

## 8. 新增 API 参考

### 8.1 robot_state.h

```c
// 原始传感器数据 -> 统一状态
esp_err_t robot_state_update(const sensor_raw_data_t *raw, 
                              robot_state_t *state,
                              bool enable_leg_comp);

// 重置累积量
void robot_state_reset(robot_state_t *state);
void robot_state_reset_distance(robot_state_t *state);
void robot_state_reset_yaw(robot_state_t *state);
```

### 8.2 leg_kinematics.h (新增)

```c
// 虚拟力直接转换为关节扭矩（不做 PD 计算）
esp_err_t vmc_force_to_torque(const vmc_force_input_t *input,
                               bool is_left,
                               vmc_force_output_t *output);

// 双腿版本
esp_err_t vmc_dual_force_to_torque(const vmc_force_input_t *left_input,
                                    const vmc_force_input_t *right_input,
                                    vmc_force_output_t *left_output,
                                    vmc_force_output_t *right_output);
```

使用示例:
```c
// 外部计算虚拟力
float F_L = K_L * (target_L - current_L) + D_L * (0 - dL);
float F_alpha = roll_ctrl_output + sync_output;

// 转换为关节扭矩
vmc_force_input_t input = {
    .F_L = F_L,
    .F_alpha = F_alpha,
    .hip_angle_deg = hip_pos,
    .knee_angle_deg = knee_pos
};
vmc_force_output_t output;
vmc_force_to_torque(&input, is_left, &output);

// 输出已自动处理右腿取反
motor_set_torque(hip_id, output.hip_torque);
motor_set_torque(knee_id, output.knee_torque);
```
