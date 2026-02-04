# VMC 右腿控制问题全链条分析

## 问题现象
- F_L 单独使能时：左右腿都正常
- F_alpha 使能后：左腿正常，右腿异常（腿伸直 + 向前摆动）

## 数据流全链条分析

### 1. 传感器数据采集 (balance_test.c)

```c
// 左腿传感器
.left.sensor = {
    .hip_angle = can_motor_read_position(g_motor_left_hip),   // 电机角度
    .knee_angle = can_motor_read_position(g_motor_left_knee),
    .hip_velocity = can_motor_read_speed(g_motor_left_hip) * 6.0f,  // rpm
    .knee_velocity = can_motor_read_speed(g_motor_left_knee) * 6.0f
}

// 右腿传感器 (同样是直接读取电机角度)
.right.sensor = {
    .hip_angle = can_motor_read_position(g_motor_right_hip),
    .knee_angle = can_motor_read_position(g_motor_right_knee),
    ...
}
```

### 2. 电机偏置定义 (leg_kinematics.h)

```c
// 左腿
LEG_LEFT_HIP_OFFSET  = -15.0f
LEG_LEFT_KNEE_OFFSET = 35.0f

// 右腿 (镜像)
LEG_RIGHT_HIP_OFFSET  = 15.0f
LEG_RIGHT_KNEE_OFFSET = -35.0f
```

### 3. 正运动学 (leg_kin_forward)

```c
// 电机角度 → 运动学角度
theta1 = DEG2RAD(hip_angle - hip_offset)
theta2 = DEG2RAD(knee_angle - knee_offset)

// 右腿镜像处理
if (!is_left) {
    theta1 = -theta1;  // 取反
    theta2 = -theta2;  // 取反
}

// 计算腿长和 body_angle
L = sqrt(L1² + L2² + 2*L1*L2*cos(θ2))
β = atan2(L2*sin(θ2), L1 + L2*cos(θ2))
α = θ1 + β

workspace->leg_length = L
workspace->body_angle = RAD2DEG(α)  // 输出的 α 是统一的坐标系
```

### 4. 雅可比矩阵计算 (leg_kin_jacobian) - **问题点**

```c
// 电机角度 → 运动学角度
theta2 = DEG2RAD(knee_angle - knee_offset)

if (!is_left) {
    theta2 = -theta2;  // 右腿镜像
}

// 使用镜像后的 theta2 计算雅可比
c2 = cos(theta2)
s2 = sin(theta2)

J[0] = 0
J[1] = -L1 * L2 * s2 / L       // ∂L/∂θ2
J[2] = 1
J[3] = L2 * (L1 * c2 + L2) / L²  // ∂α/∂θ2

// 当前代码：没有对 J 做额外处理（刚删除了取反）
```

### 5. VMC 扭矩计算 (vmc_compute_torque_body)

```c
// 计算虚拟力
F_L = K_L * (target_L - current_L) + D_L * dL_error + F_gravity
F_alpha = K_alpha * (target_alpha - current_alpha) + D_alpha * dalpha_error

// 扭矩 = J^T × F
tau_hip  = J[0] * F_L + J[2] * F_alpha
tau_knee = J[1] * F_L + J[3] * F_alpha
```

### 6. 扭矩发送 (apply_leg_motor_commands)

```c
// 直接发送，没有任何镜像处理
can_motor_set_torque(g_motor_left_hip, left.hip_torque);
can_motor_set_torque(g_motor_left_knee, left.knee_torque);
can_motor_set_torque(g_motor_right_hip, right.hip_torque);
can_motor_set_torque(g_motor_right_knee, right.knee_torque);
```

---

## 问题分析

### 核心问题：雅可比输出的扭矩是针对哪个坐标系的？

#### 情况分析

设右腿当前姿态：膝关节电机读数 `knee_motor = +55°`

**步骤 1: 计算 θ2**
```
θ2_kin = knee_motor - offset = 55 - (-35) = 90°
θ2_mirror = -90° (取反后)
```

**步骤 2: 计算雅可比 (用 θ2_mirror)**
```
s2 = sin(-90°) = -1
c2 = cos(-90°) = 0
L = L1 (因为 cos(-90°)=0，简化计算)

J[1] = -L1 * L2 * (-1) / L = +L1*L2/L > 0
J[3] = L2 * (L1 * 0 + L2) / L² = L2²/L² > 0
```

**步骤 3: 场景 - F_alpha 控制**

假设腿向前偏了（body_angle = -100°，比目标 -90° 小）：
```
alpha_error = target(-90°) - current(-100°) = +10° > 0
F_alpha = K_alpha * 10° > 0  （需要向后摆的力矩）
```

**步骤 4: 计算扭矩**
```
tau_knee = J[3] * F_alpha = 正 × 正 = 正
```

**步骤 5: 物理效果**

问题来了：**右腿膝电机正扭矩的物理效果是什么？**

---

## 🔑 关键发现

### 雅可比的数学意义

雅可比 `J = ∂(L,α)/∂(θ1,θ2)` 建立的是：
- **运动学角度 (θ1, θ2)** 到 **工作空间 (L, α)** 的映射

当我们把右腿的 θ2 取反后，雅可比描述的是：
- **镜像后的运动学角度** 到 **工作空间** 的映射

所以 `τ = J^T × F` 计算出的扭矩是针对 **镜像后的运动学角度** 的！

### 但电机接收的是什么？

电机接收的扭矩是针对 **电机实际角度** 的！

### 转换关系

```
θ2_mirror = -θ2_kin = -(knee_motor - offset)

对 θ2_mirror 的正扭矩 = 对 θ2_kin 的负扭矩 = 对电机的负扭矩
```

**所以：计算出的 tau_knee 需要取反才能发送给右腿电机！**

---

## 解决方案

### 方案 A: 在扭矩输出时取反（推荐）

在 `vmc_compute_torque_body` 或 `vmc_ctrl_compute` 最后：

```c
// 右腿：扭矩取反（因为雅可比是基于镜像角度计算的）
if (!is_left) {
    output->hip_torque = -output->hip_torque;
    output->knee_torque = -output->knee_torque;
}
```

### 方案 B: 在雅可比中处理（恢复原来的取反，但只取反 J[1]）

```c
// 不对 J[3] 取反（因为 α 的定义一致）
// 但需要对 J[1] 取反（因为 L 的变化方向...）
```

**方案 B 更复杂，不推荐**

### 方案 C: 不在雅可比中镜像 θ2，而是在最后统一处理

这需要更大的重构。

---

## 验证方案 A 的正确性

### 场景：右腿需要向后摆（F_alpha > 0）

1. `theta2_mirror = -90°`（等效于左腿弯曲）
2. `J[3] > 0`
3. `tau_knee_raw = J[3] * F_alpha > 0`（正扭矩）
4. `tau_knee_final = -tau_knee_raw < 0`（取反后为负）
5. 右腿膝电机负扭矩 → **物理效果是？**

### 需要确认的物理问题

**右腿膝电机负扭矩的物理效果**：
- 如果负扭矩 = 伸腿，那向后摆是正确的 ✅
- 如果负扭矩 = 弯腿，那就是错的 ❌

**这取决于电机的安装方向和正方向定义！**

---

## 电机方向分析

从偏置定义推断：
```c
// 左腿垂直时：knee_motor = -55°, theta2_kin = -90°
// 右腿垂直时：knee_motor = +55°, theta2_kin = +90°
```

左右腿电机在相同 body_angle 下读数是相反的，说明 **电机安装是镜像的**。

如果左腿膝电机：正扭矩 → 伸腿
那右腿膝电机：正扭矩 → **也是伸腿**（因为镜像安装，正方向的物理效果相同）

**等等！这说明右腿不需要额外取反！**

---

## 重新分析：实际问题在哪里

让我重新从头分析整个流程...

### 正运动学验证

左腿垂直时（body_angle = -90°）：
```
knee_motor = -55°
theta2_kin = -55 - 35 = -90°
// 不取反
theta2 = -90°
cos(-90°) = 0, sin(-90°) = -1
β = atan2(L2*(-1), L1+L2*0) = atan2(-L2, L1) ≈ -45°
α = θ1 + β
// 如果 body_angle = -90°，则 θ1 = -90° - (-45°) = -45°
```

右腿垂直时（body_angle = -90°）：
```
knee_motor = +55°
theta2_kin = 55 - (-35) = 90°
// 取反
theta2 = -90°  // 与左腿相同！
// 后续计算相同
```

**正运动学是正确的** ✅

### 雅可比分析

左腿：
```
theta2 = -90°
s2 = sin(-90°) = -1
J[1] = -L1*L2*(-1)/L = +L1*L2/L > 0
J[3] = L2*(L1*0+L2)/L² = L2²/L² > 0
```

右腿：
```
theta2 = -90° (取反后)
s2 = sin(-90°) = -1  // 相同！
J[1] = +L1*L2/L > 0  // 相同！
J[3] = L2²/L² > 0    // 相同！
```

**左右腿在相同 body_angle 下，雅可比相同！**

### 扭矩分析

假设两腿都需要伸长（F_L > 0, F_alpha = 0）：
```
tau_knee = J[1] * F_L = 正 × 正 = 正
```

两腿膝电机都收到正扭矩。

**但是！**

左腿膝电机正扭矩：电机往某个方向转 → 伸腿
右腿膝电机正扭矩：电机往某个方向转 → **也是伸腿吗？**

**如果右腿电机安装方向相反，正扭矩可能是弯腿！**

---

## 🔥 最终结论

### 问题根因

雅可比计算是正确的（对于统一的工作空间定义）。

**问题在于：扭矩发送时没有考虑右腿电机的安装方向！**

右腿电机是镜像安装的，所以：
- 相同的"正扭矩"在左腿是伸腿，在右腿是弯腿

### 解决方案

**在发送扭矩时对右腿取反**：

```c
// balance_test.c 中 apply_leg_motor_commands
if (g_motor_right_hip) {
    can_motor_set_torque(g_motor_right_hip, -g_vmc_dual_output.right.hip_torque);
}
if (g_motor_right_knee) {
    can_motor_set_torque(g_motor_right_knee, -g_vmc_dual_output.right.knee_torque);
}
```

或者在 `vmc_ctrl_compute` 输出时处理：

```c
// leg_kinematics.c 中 vmc_ctrl_compute 的最后
if (!is_left) {
    output->hip_torque = -output->hip_torque;
    output->knee_torque = -output->knee_torque;
}
```

---

## 为什么 F_L 单独工作时正常？

这可能是巧合，或者实际上也有问题只是不明显：
1. F_L 的控制是弹簧模型，误差双向都能收敛
2. 即使方向反了，只要增益足够小，系统还是稳定的
3. 右腿可能在"反向工作"但表现出"类似正确"的行为

而 F_alpha 使能后：
1. 角度控制更敏感
2. 错误的方向导致正反馈
3. 系统迅速发散（腿一直向一个方向摆）

---

## 实施建议

**推荐在 leg_kinematics.c 中处理**，这样封装更好：

```c
// vmc_ctrl_compute 函数末尾
// === 8. 填充输出 ===
output->hip_torque = vmc_output.hip_torque;
output->knee_torque = vmc_output.knee_torque;

// 右腿扭矩方向修正（因为电机镜像安装）
if (!is_left) {
    output->hip_torque = -output->hip_torque;
    output->knee_torque = -output->knee_torque;
}
```
