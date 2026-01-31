# 二连杆 VMC 腿部控制方案

## 概述

VMC (Virtual Model Control，虚拟模型控制) 是一种将复杂的多关节机构映射到简单的虚拟模型上进行控制的方法。对于轮腿机器人的二连杆结构，VMC 可以实现更柔顺、更直观的腿部控制。

### 当前项目架构

```
         机身 (Body)
            │
    ┌───────┴───────┐
    │  Hip Motor    │ ← θ1 大腿电机
    └───────┬───────┘
            │ L1 = 0.10m
            │
    ┌───────┴───────┐
    │  Knee Motor   │ ← θ2 小腿电机
    └───────┬───────┘
            │ L2 = 0.10m
            │
    ┌───────┴───────┐
    │  Wheel Motor  │ ← 轮子
    └───────────────┘
```

## 当前控制方式 vs VMC

### 当前方式：位置控制

```
目标 (腿长, 身体角) → 逆运动学 IK → 电机位置命令 (θ1, θ2)
```

**存在的问题：**
1. **刚性**：电机位置闭环，遇到外力冲击时无法柔顺响应
2. **无力控制**：无法主动控制地面接触力
3. **着地冲击大**：腿部落地时冲击直接传递到机身
4. **无法实现弹簧阻尼效果**：跳跃、缓冲能力受限

### VMC 方式：力控制 + 虚拟弹簧阻尼

```
                    ┌─────────────────────────────┐
目标 (L₀, α₀) ─────▶│  虚拟弹簧阻尼模型            │
                    │  F = K(L₀-L) + D(L̇₀-L̇)      │
当前 (L, α, L̇) ────▶│                             │──▶ 虚拟力 (F_L, τ_α)
                    └─────────────────────────────┘
                                │
                                ▼
                    ┌─────────────────────────────┐
                    │  雅可比转置映射               │
                    │  τ = J^T · F                 │
                    └─────────────────────────────┘
                                │
                                ▼
                      电机扭矩 (τ_hip, τ_knee)
```

## VMC 核心公式

### 坐标系定义

本项目采用**世界坐标系**作为 VMC 的输入，更直观易懂：

```
        机身 (Body)
         │
         ●──────────────────→ +X (水平向前)
        ╱│
       ╱ │
      ╱  │
     ●   │
    ╱ ╲  │
   ╱   ╲ │
  ●     ↓ +Y (垂直向下)
  轮子

世界坐标系:
  F_x: 水平力 (向前为正)
  F_y: 垂直力 (向下为正，支撑力为负)
```

### 1. 世界坐标系虚拟力 (推荐使用)

在世界坐标系 (x, y) 上定义控制策略：

```
┌──────────────────────────────────────────────────────────────┐
│  轮子位置:                                                    │
│    x = L × sin(α)      // 水平位置                           │
│    y = L × cos(α)      // 垂直位置 (向下为正)                 │
│                                                              │
│  虚拟力:                                                      │
│    F_x = K_vx × (ẋ₀ - ẋ)                    // 水平: 速度控制 │
│    F_y = K_y × (y₀ - y) + D_y × (ẏ₀ - ẏ) + F_gravity        │
│                                             // 垂直: 位置控制 │
└──────────────────────────────────────────────────────────────┘

**设计理念**:
- **F_x (水平力)**: 纯速度控制，用于调节机身前后运动
  - 与平衡控制配合：LQR/PID 输出期望速度，VMC 产生对应力
  - 不需要位置项，因为轮子会滚动
- **F_y (垂直力)**: 弹簧-阻尼控制，用于控制腿长
  - 位置项：维持目标腿长
  - 阻尼项：抑制垂直震荡
  - 重力补偿：支撑机身重量

| 参数 | 含义 | 典型值 |
|------|------|--------|
| K_vx | 水平速度增益 (Ns/m) | 10~100 |
| K_y | 垂直方向刚度 (N/m) | 500~2000 |
| D_y | 垂直方向阻尼 (Ns/m) | 20~100 |
| F_gravity | 重力补偿力 (N) | m×g (向上为负) |
```

### 2. 笛卡尔雅可比矩阵 (世界坐标系)

需要新增笛卡尔坐标系的雅可比矩阵：

```c
// J_cart 将关节速度映射到笛卡尔速度: [dx; dy] = J_cart × [dθ1; dθ2]
//
// 轮子位置:
//   x = L1×sin(θ1) + L2×sin(θ1+θ2)
//   y = L1×cos(θ1) + L2×cos(θ1+θ2)
//
//            ┌ ∂x/∂θ1   ∂x/∂θ2 ┐
// J_cart =  │                  │
//            └ ∂y/∂θ1   ∂y/∂θ2 ┘
//
// 展开:
//   J_cart[0] = L1×cos(θ1) + L2×cos(θ1+θ2)   // ∂x/∂θ1
//   J_cart[1] = L2×cos(θ1+θ2)                 // ∂x/∂θ2
//   J_cart[2] = -L1×sin(θ1) - L2×sin(θ1+θ2)  // ∂y/∂θ1
//   J_cart[3] = -L2×sin(θ1+θ2)                // ∂y/∂θ2
```

### 3. 关节扭矩计算 (世界坐标系)

通过笛卡尔雅可比矩阵转置将世界坐标系力映射到关节扭矩：

```
┌────────────────────────────────────────────────────────────────┐
│  ┌ τ_hip  ┐            ┌ J[0]  J[2] ┐   ┌ F_x ┐               │
│  │        │ = J_cart^T │            │ × │     │                │
│  └ τ_knee ┘            └ J[1]  J[3] ┘   └ F_y ┘               │
│                                                                │
│  展开:                                                         │
│    τ_hip  = J[0] × F_x + J[2] × F_y                           │
│    τ_knee = J[1] × F_x + J[3] × F_y                           │
└────────────────────────────────────────────────────────────────┘

注意: 这里 F_x, F_y 都直接影响两个关节的扭矩！
```

### 4. 旧的腿坐标系方法 (作为参考)

原来的 `leg_kin_jacobian` 是腿坐标系 (L, α) 的雅可比：

```c
// J_leg 将关节速度映射到腿坐标系速度: [dL; dα] = J_leg × [dθ1; dθ2]
//
//          ┌ 0            -L1×L2×sin(θ2)/L  ┐
// J_leg = │                                │
//          └ 1    L2×(L1×cos(θ2)+L2)/L²    ┘
```

**两种方法的对比**：

| 特性 | 世界坐标系 (F_x, F_y) | 腿坐标系 (F_L, τ_α) |
|------|----------------------|---------------------|
| 直观性 | ✅ 更直观 | 需要理解 |
| 与重力对齐 | ✅ 自然对齐 | 需要分解 |
| 髋关节参与 | ✅ 自然耦合 | J11=0 易误解 |
| 解耦性 | 耦合 | 腿长/角度解耦 |
| 雅可比矩阵 | 需要新增 | 已有实现 |
│  └ τ_knee ┘       └ J12  J22 ┘   └ τ_α ┘              │
│                                                        │
│  展开:                                                 │
│    τ_hip  = J11 × F_L + J21 × τ_α = 0 × F_L + 1 × τ_α │
│    τ_knee = J12 × F_L + J22 × τ_α                      │
└────────────────────────────────────────────────────────┘
```

## ⚠️ 重要修正：关于 J11=0 的正确理解

### 常见误解

看到 J11 = 0，容易得出错误结论："髋关节扭矩不影响腿长方向的支撑力"。

### 这是错的！

**J11 = 0 只说明**：在运动学上，转动髋关节（θ1）不改变腿长（L）。

**但不意味着**：当腿需要承受轴向支撑力时，髋关节不需要出力！

### 物理直觉

想象一下：你的腿在承受体重时，只靠膝盖能站住吗？

```
        机身
         │
         ● ← 髋关节 (必须锁住！否则会折叠)
        ╱
       ╱  30°
      ╱
     ● ← 膝关节
    ╱ ╲
   ╱   ╲
  ●─────→ 地面反作用力 F
  轮子   ↑
```

当腿斜着的时候，地面反力 F 对髋关节产生力矩，髋关节必须产生反向扭矩才能保持姿态！

### 雅可比公式为什么给出 J11=0？

因为雅可比映射的是**工作空间力 (F_L, τ_α)** 到关节扭矩。

关键是：**F_L 是沿腿长方向的力，不是垂直力！**

```
        机身
         │
         ●──────→ τ_α (角度方向力矩)
        ╱│
       ╱ │
      ╱  │ F_L (沿腿长方向的力，斜向!)
     ↙   │
    ╱    │
   ●     ↓
   轮子  地面
```

当 F_L 沿着腿的方向时：
- 这个力通过两个关节（髋和膝）的连线
- 髋关节处的力臂为 0！
- 所以 F_L **确实**不产生髋关节扭矩

**但是！** 实际地面反力是**垂直向上**的，不是沿腿方向的！

### 从垂直力到工作空间力的转换

如果地面给轮子一个垂直向上的力 F_ground：

```
F_ground 分解为:
  F_L = F_ground × cos(α)    // 沿腿方向分量
  F_perp = F_ground × sin(α)  // 垂直于腿方向分量

F_perp 会产生:
  τ_α = F_perp × L = F_ground × sin(α) × L
```

所以：

```
τ_hip = τ_α = F_ground × L × sin(α)   ← 髋关节确实需要扭矩！
```

### 正确的完整公式

对于垂直地面反力 F_ground，需要：

```
┌──────────────────────────────────────────────────────────────┐
│  F_L = F_ground × cos(α)        // 腿长方向力                │
│  τ_α = F_ground × L × sin(α)    // 角度方向力矩              │
│                                                              │
│  τ_hip  = J21 × τ_α = τ_α = F_ground × L × sin(α)           │
│  τ_knee = J12 × F_L + J22 × τ_α                              │
└──────────────────────────────────────────────────────────────┘
```

### 结论

| 关节 | 作用 |
|------|------|
| 髋关节 | 1. 控制角度 α<br>2. 平衡地面力的水平分量产生的力矩 |
| 膝关节 | 1. 控制腿长 L<br>2. 主要承受腿长方向的力<br>3. 部分参与角度控制 |

**两个关节协同工作，缺一不可！**

**因此，推荐使用世界坐标系 (F_x, F_y) 作为 VMC 输入，更直观且自然处理了髋关节的参与！**

## 针对本项目的实现建议

### 方案一：世界坐标系 VMC (推荐)

使用世界坐标系 (水平力 F_x, 垂直力 F_y) 作为输入。

#### 步骤 1：添加 VMC 数据结构

```c
// leg_kinematics.h 中添加

typedef struct {
    // 水平方向 (X) - 速度控制
    float K_vx;             // 水平速度增益 (Ns/m)
    
    // 垂直方向 (Y) - 位置控制
    float K_y;              // 垂直刚度 (N/m)
    float D_y;              // 垂直阻尼 (Ns/m)
    
    // 重力补偿
    float gravity_comp;     // 重力补偿系数 (0~1)
    float robot_mass;       // 机器人质量 (kg)
    
    // 输出限幅
    float max_hip_torque;   // 髋关节最大扭矩 (Nm)
    float max_knee_torque;  // 膝关节最大扭矩 (Nm)
} vmc_params_t;

typedef struct {
    // 目标 (世界坐标系)
    float target_vx;        // 目标水平速度 (m/s)，由平衡控制给出
    float target_y;         // 目标垂直位置 (m)，即目标腿长
    float target_vy;        // 目标垂直速度 (m/s)，一般为 0
    
    // 当前状态 (世界坐标系)
    float current_y;        // 当前垂直位置 (m)
    float current_vx;       // 当前水平速度 (m/s)
    float current_vy;       // 当前垂直速度 (m/s)
} vmc_input_t;

typedef struct {
    float hip_torque;       // 髋关节扭矩 (Nm)
    float knee_torque;      // 膝关节扭矩 (Nm)
    
    // 调试信息
    float F_x;              // 水平虚拟力 (N)
    float F_y;              // 垂直虚拟力 (N)
    float F_gravity;        // 重力补偿力 (N)
} vmc_output_t;
```

#### 步骤 2：添加笛卡尔雅可比函数

```c
// leg_kinematics.c 中添加

/**
 * @brief 计算笛卡尔坐标系雅可比矩阵 (世界坐标系)
 * @param joint 当前关节状态
 * @param is_left 是否为左腿
 * @param params 运动学参数 (NULL 使用默认参数)
 * @param J 输出 2x2 雅可比矩阵 [J00, J01; J10, J11]
 * @return ESP_OK 成功
 * @note J 将关节速度映射到笛卡尔速度: [dx; dy] = J × [dθ1; dθ2]
 */
esp_err_t leg_kin_jacobian_cartesian(const leg_joint_state_t *joint, 
                                      bool is_left,
                                      const leg_kin_params_t *params,
                                      float J[4]) {
    if (joint == NULL || J == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    leg_kin_params_t p;
    if (params == NULL) {
        leg_kin_get_default_params(is_left, &p);
    } else {
        memcpy(&p, params, sizeof(leg_kin_params_t));
    }
    
    float L1 = p.thigh_length;
    float L2 = p.shank_length;
    
    // 电机角度 -> 运动学角度 (弧度)
    float theta1 = DEG2RAD(joint->hip_angle - p.hip_offset);
    float theta2 = DEG2RAD(joint->knee_angle - p.knee_offset);
    
    // 右腿镜像处理
    if (!is_left) {
        theta1 = -theta1;
        theta2 = -theta2;
    }
    
    float c1 = cosf(theta1);
    float s1 = sinf(theta1);
    float c12 = cosf(theta1 + theta2);
    float s12 = sinf(theta1 + theta2);
    
    // 笛卡尔雅可比: [dx/dθ1, dx/dθ2; dy/dθ1, dy/dθ2]
    // x = L1*sin(θ1) + L2*sin(θ1+θ2)  (向前为正)
    // y = L1*cos(θ1) + L2*cos(θ1+θ2)  (向下为正)
    
    J[0] = L1 * c1 + L2 * c12;    // ∂x/∂θ1
    J[1] = L2 * c12;              // ∂x/∂θ2
    J[2] = -L1 * s1 - L2 * s12;   // ∂y/∂θ1
    J[3] = -L2 * s12;             // ∂y/∂θ2
    
    // 右腿镜像处理 (x 方向反向)
    if (!is_left) {
        J[0] = -J[0];
        J[1] = -J[1];
    }
    
    return ESP_OK;
}

/**
 * @brief 正运动学: 关节角度 -> 笛卡尔位置
 */
esp_err_t leg_kin_forward_cartesian(const leg_joint_state_t *joint,
                                     bool is_left,
                                     const leg_kin_params_t *params,
                                     float *x, float *y) {
    if (joint == NULL || x == NULL || y == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    leg_kin_params_t p;
    if (params == NULL) {
        leg_kin_get_default_params(is_left, &p);
    } else {
        memcpy(&p, params, sizeof(leg_kin_params_t));
    }
    
    float L1 = p.thigh_length;
    float L2 = p.shank_length;
    
    float theta1 = DEG2RAD(joint->hip_angle - p.hip_offset);
    float theta2 = DEG2RAD(joint->knee_angle - p.knee_offset);
    
    if (!is_left) {
        theta1 = -theta1;
        theta2 = -theta2;
    }
    
    *x = L1 * sinf(theta1) + L2 * sinf(theta1 + theta2);
    *y = L1 * cosf(theta1) + L2 * cosf(theta1 + theta2);
    
    if (!is_left) {
        *x = -(*x);
    }
    
    return ESP_OK;
}
```

#### 步骤 3：实现世界坐标系 VMC 核心函数

```c
/**
 * @brief VMC 计算关节扭矩 (世界坐标系输入)
 * @param params VMC 参数
 * @param input VMC 输入 (世界坐标系速度和位置)
 * @param joint 当前关节状态
 * @param is_left 是否为左腿
 * @param output VMC 输出 (关节扭矩)
 */
esp_err_t vmc_compute_torque(const vmc_params_t *params,
                              const vmc_input_t *input,
                              const leg_joint_state_t *joint,
                              bool is_left,
                              vmc_output_t *output) {
    if (params == NULL || input == NULL || joint == NULL || output == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 1. 计算水平虚拟力 (纯速度控制)
    float vx_error = input->target_vx - input->current_vx;
    float F_x = params->K_vx * vx_error;
    
    // 2. 计算垂直虚拟力 (弹簧-阻尼模型)
    float y_error = input->target_y - input->current_y;
    float vy_error = input->target_vy - input->current_vy;
    float F_y = params->K_y * y_error + params->D_y * vy_error;
    
    // 3. 重力补偿 (垂直向上，对抗重力)
    // 注意: y 向下为正，所以重力补偿是负的 F_y
    float F_gravity = params->gravity_comp * params->robot_mass * 9.81f;
    F_y -= F_gravity;  // 向上的力 (负方向)
    
    // 4. 获取笛卡尔雅可比矩阵
    float J[4];
    leg_kin_jacobian_cartesian(joint, is_left, NULL, J);
    
    // 5. J^T × F = τ
    // ┌ τ_hip  ┐   ┌ J[0]  J[2] ┐   ┌ F_x ┐
    // │        │ = │            │ × │     │
    // └ τ_knee ┘   └ J[1]  J[3] ┘   └ F_y ┘
    float tau_hip  = J[0] * F_x + J[2] * F_y;
    float tau_knee = J[1] * F_x + J[3] * F_y;
    
    // 6. 扭矩限幅
    tau_hip  = clamp_f(tau_hip, -params->max_hip_torque, params->max_hip_torque);
    tau_knee = clamp_f(tau_knee, -params->max_knee_torque, params->max_knee_torque);
    
    // 7. 输出
    output->hip_torque = tau_hip;
    output->knee_torque = tau_knee;
    output->F_x = F_x;
    output->F_y = F_y;
    output->F_gravity = F_gravity;
    
    return ESP_OK;
}
```

#### 步骤 4：在 balance_test.c 中集成

```c
// 添加 VMC 开关和参数
static bool g_vmc_enabled = false;
static vmc_params_t g_vmc_params = {
    .K_vx = 50.0f,          // 水平速度增益 (Ns/m)
    .K_y = 1000.0f,         // 垂直刚度 (N/m)
    .D_y = 50.0f,           // 垂直阻尼 (Ns/m)
    .gravity_comp = 0.5f,   // 50% 重力补偿
    .robot_mass = 3.0f,     // 3kg
    .max_hip_torque = 5.0f,
    .max_knee_torque = 8.0f,
};

// VMC 目标 (可由平衡控制给出)
static float g_vmc_target_vx = 0.0f;     // 目标水平速度 (m/s)
static float g_vmc_target_y = 0.15f;     // 目标垂直位置 (m)，即腿长

// 在 apply_leg_motor_commands 中修改
static void apply_leg_motor_commands(void) {
    if (!g_leg_control_enabled) return;
    
    if (g_vmc_enabled) {
        // ===== VMC 力控模式 =====
        vmc_input_t vmc_in;
        vmc_output_t vmc_out;
        
        // 获取当前关节状态
        leg_joint_state_t left_joint = {
            .hip_angle = g_motor_left_hip->feedback.position,
            .knee_angle = g_motor_left_knee->feedback.position
        };
        
        // 计算当前笛卡尔位置
        float current_x, current_y;
        leg_kin_forward_cartesian(&left_joint, true, NULL, &current_x, &current_y);
        
        // 计算当前笛卡尔速度 (通过雅可比)
        float J[4];
        leg_kin_jacobian_cartesian(&left_joint, true, NULL, J);
        float hip_vel = DEG2RAD(g_motor_left_hip->feedback.velocity);
        float knee_vel = DEG2RAD(g_motor_left_knee->feedback.velocity);
        float current_vx = J[0] * hip_vel + J[1] * knee_vel;
        float current_vy = J[2] * hip_vel + J[3] * knee_vel;
        
        // 填充 VMC 输入
        vmc_in.target_vx = g_vmc_target_vx;   // 目标水平速度 (由平衡控制给出)
        vmc_in.target_y = g_vmc_target_y;     // 目标腿长
        vmc_in.target_vy = 0;
        vmc_in.current_y = current_y;
        vmc_in.current_vx = current_vx;
        vmc_in.current_vy = current_vy;
        
        // 计算扭矩
        vmc_compute_torque(&g_vmc_params, &vmc_in, &left_joint, true, &vmc_out);
        
        // 发送扭矩命令
        can_motor_set_torque(g_motor_left_hip, vmc_out.hip_torque);
        can_motor_set_torque(g_motor_left_knee, vmc_out.knee_torque);
        
        // 右腿类似...
        
    } else {
        // ===== 原有位置控制模式 =====
        can_motor_set_position(g_motor_left_hip, g_leg_left_hip_angle, g_leg_move_speed);
        can_motor_set_position(g_motor_left_knee, g_leg_left_knee_angle, g_leg_move_speed);
        // ...
    }
}
```

### 方案二：混合控制 (进阶)

垂直方向用 VMC 力控，水平方向用位置控制。

```
              垂直方向 (Y)                 水平方向 (X)
         ┌──────────────┐            ┌──────────────┐
目标 ───▶│ 虚拟弹簧阻尼  │───▶ F_y    │  位置 PID    │───▶ F_x
         └──────────────┘            └──────────────┘
                │                           │
                └───────────┬───────────────┘
                            ▼
                    ┌─────────────┐
                    │ J_cart^T    │───▶ (τ_hip, τ_knee)
                    └─────────────┘
```

**优点**：
- 垂直方向柔顺，可以吸收冲击
- 水平方向可以精确定位
- 实现复杂度适中

### 方案三：导纳控制 (高级)

```
                    ┌─────────────────────┐
外力检测 ─────────▶│   导纳模型           │───▶ 目标位置修正
(通过电流估算)       │  M·ẍ + D·ẋ + K·x = F │
                    └─────────────────────┘
```

**适用场景**：地面接触力闭环、跳跃着陆缓冲

## 速度估算问题

VMC 需要工作空间的速度 (dL/dt, dα/dt)，但当前项目可能没有直接测量。

### 解决方案

#### 方案 A：关节速度 + 雅可比正运动学

```c
// 从关节速度计算工作空间速度
// [dL/dt; dα/dt] = J × [dθ1/dt; dθ2/dt]

float joint_vel[2] = {
    motor_hip->feedback.velocity,    // θ1 速度 (deg/s → rad/s)
    motor_knee->feedback.velocity    // θ2 速度 (deg/s → rad/s)
};

float J[4];
leg_kin_jacobian(&joint, is_left, NULL, J);

float length_vel = J[0] * joint_vel[0] + J[1] * joint_vel[1];
float angle_vel  = J[2] * joint_vel[0] + J[3] * joint_vel[1];
```

#### 方案 B：数值微分 + 低通滤波

```c
// 简单差分 (需要滤波消除噪声)
static float prev_length = 0;
float length_vel = (current_length - prev_length) / dt;
length_vel = lpf_compute(&lpf_length_vel, length_vel);  // 低通滤波
prev_length = current_length;
```

## 推荐实施路线

```
阶段 1: 基础 VMC (1-2 天)
├── 添加 vmc_params_t, vmc_input_t, vmc_output_t
├── 实现 vmc_compute_torque()
├── 添加 g_vmc_enabled 开关和 CLI 命令
└── 测试单腿静止状态下的柔顺性

阶段 2: 速度估算 (1 天)
├── 实现关节速度 → 工作空间速度转换
├── 添加速度滤波
└── 测试动态响应

阶段 3: 参数调优 (2-3 天)
├── 调整 K_L, D_L 实现合适的腿部刚度
├── 调整重力补偿系数
├── 添加 Qt 调参界面
└── 测试跳跃/着陆缓冲效果

阶段 4: 与平衡控制集成 (2 天)
├── VMC 输出与 Roll 控制协调
├── 考虑腿部力对 pitch 的影响
└── 完整系统测试
```

## CLI 命令设计

```bash
# VMC 开关
balance vmc on       # 启用 VMC 力控
balance vmc off      # 禁用 VMC (回到位置控制)

# 参数调整
balance vmc kvx 50   # 水平速度增益 (Ns/m)
balance vmc ky 1000  # 垂直刚度 (N/m)
balance vmc dy 50    # 垂直阻尼 (Ns/m)
balance vmc gc 0.5   # 重力补偿系数 (0~1)

# 目标设置
balance vmc vx 0.5   # 设置目标水平速度 (m/s)
balance vmc y 0.15   # 设置目标腿长 (m)

# 状态显示
balance vmc status   # 显示 VMC 状态和输出

# 预设
balance vmc soft     # 柔软模式 (K_y=500, D_y=80)
balance vmc stiff    # 刚硬模式 (K_y=1500, D_y=30)
balance vmc jump     # 跳跃模式 (准备着陆)
```

## 安全注意事项

1. **扭矩限幅**：必须限制输出扭矩，防止电机过载
2. **重力补偿**：初始设置偏低 (0.3~0.5)，逐步调高
3. **阻尼系数**：不能太低，否则会震荡
4. **切换保护**：位置控制 ↔ 力控制切换时，需要平滑过渡
5. **急停逻辑**：VMC 模式下也要保留紧急停止

## 参考资料

1. MIT Cheetah Mini VMC 实现
2. Pratt, G.A. "Virtual Model Control" - MIT Leg Lab
3. 电机力矩模式使用指南 (达妙电机文档)

## 附录 A：笛卡尔坐标系雅可比推导

对于本项目的二连杆结构（世界坐标系）：

```
末端位置:
x = L1·sin(θ1) + L2·sin(θ1 + θ2)    (水平方向，向前为正)
y = L1·cos(θ1) + L2·cos(θ1 + θ2)    (垂直方向，向下为正)

笛卡尔雅可比矩阵 J_cart:
     [∂x/∂θ1  ∂x/∂θ2]
J =  [∂y/∂θ1  ∂y/∂θ2]

J11 = L1·cos(θ1) + L2·cos(θ1 + θ2)
J12 = L2·cos(θ1 + θ2)
J21 = -L1·sin(θ1) - L2·sin(θ1 + θ2)
J22 = -L2·sin(θ1 + θ2)
```

**力-扭矩映射**:
```
[τ1]      T   [F_x]
[τ2] = J_cart [F_y]

展开:
τ1 = J11·F_x + J21·F_y
τ2 = J12·F_x + J22·F_y
```

## 附录 B：两种坐标系对比

| 特性 | 腿长坐标 (L, α) | 世界坐标 (x, y) |
|------|-----------------|-----------------|
| 水平输入 | τ_α (角度力矩) | F_x (水平力) |
| 垂直输入 | F_L (沿腿方向) | F_y (垂直力) |
| 物理直觉 | 腿部视角 | 世界视角 |
| 重力补偿 | 需要分解到 (L, α) | 直接: F_y = m·g |
| 与平衡控制结合 | 需要转换 | 直接对接 |
| 推荐场景 | 纯腿长控制 | **平衡机器人 (本项目)** |

**本项目 VMC 控制策略**：
| 方向 | 控制类型 | 公式 | 作用 |
|------|---------|------|------|
| 水平 (F_x) | **速度控制** | F_x = K_vx × (ẋ₀ - ẋ) | 配合平衡控制调节速度 |
| 垂直 (F_y) | **位置控制** | F_y = K_y × (y₀ - y) + D_y × (ẏ₀ - ẏ) | 维持腿长，柔顺缓冲 |

## 附录 C：雅可比奇异性

**奇异位置**（det(J) = 0）：
- 腿完全伸直：θ2 = 0
- 腿完全折叠：θ2 = π

**处理方法**:
```c
// 奇异性检测
float det = J11 * J22 - J12 * J21;
if (fabsf(det) < SINGULARITY_THRESHOLD) {
    // 方案1: 限制关节角度，永不进入奇异区
    // 方案2: 阻尼最小二乘法
    det = (det > 0) ? SINGULARITY_THRESHOLD : -SINGULARITY_THRESHOLD;
}
```

## 附录 D：与腿长坐标系的关系

旧版文档使用腿长坐标系 (L, α)，其雅可比特点：
- J11 = 0 (髋关节不改变腿长——运动学上正确)
- 但这**不意味着**髋关节不参与支撑力！

**静力学分析**:
当腿与垂直方向成角度 α 时，地面反作用力 F 在髋关节产生力矩：
```
M_hip = F · L · sin(α)
```
髋关节电机必须提供 τ_hip = -M_hip 来平衡。

**结论**：两种坐标系最终会得到相同的扭矩输出，但世界坐标系对于平衡机器人更直观。
