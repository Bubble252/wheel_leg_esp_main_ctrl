/**
 * @file REFACTOR_GUIDE.md
 * @brief balance_test.c 重构指南
 * 
 * 本文档说明如何将 balance_test.c 迁移到新的架构
 */

# balance_test.c 重构指南

## 1. 当前问题

### 1.1 重复的单位转换
```c
// 当前代码中多处重复：
float pitch_rad = DEG2RAD(imu_data.pitch);
float roll_rad = DEG2RAD(imu_data.roll);
// ... 在多个函数中都有类似代码
```

### 1.2 重复的 FK 计算
```c
// VMC 模式和其他地方都在计算：
leg_kin_forward(...);  // 在 VMC 中调用
leg_kin_forward(...);  // 在 Roll 控制中也调用
```

### 1.3 VMC 做了太多事情
```c
// 当前 vmc_ctrl_compute 内部计算 PD：
F_L = K_L * (target_L - current_L) + D_L * (0 - dL);
F_alpha = K_alpha * (target_alpha - current_alpha) + ...
```

---

## 2. 重构目标

### 2.1 集中状态计算
```c
// 每个控制周期只调用一次：
sensor_raw_data_t raw = {...};  // 收集原始数据
robot_state_t state;
robot_state_update(&raw, &state, true);

// 之后所有控制器都使用 state 中的数据
float pitch = state.imu.pitch;           // 已经是 rad
float wheel_speed = state.kinematics.lqr_speed;  // 已经是 m/s，前进为正
float left_L = state.left_leg.leg_length;        // 已经计算好
```

### 2.2 简化 VMC 调用
```c
// 外部计算虚拟力：
// Roll 控制 -> F_alpha
// 腿长控制 -> F_L
// 双腿同步 -> delta_F_alpha

float F_L = compute_leg_length_force(target_L, state.left_leg);
float F_alpha = roll_output + sync_output;

// VMC 只做雅可比转换：
vmc_force_input_t force_input = {
    .F_L = F_L,
    .F_alpha = F_alpha,
    .hip_angle_deg = state.left_leg.hip_pos_deg,
    .knee_angle_deg = state.left_leg.knee_pos_deg
};
vmc_force_output_t force_output;
vmc_force_to_torque(&force_input, true, &force_output);
```

---

## 3. 分步迁移计划

### 第一步：添加状态更新（不改变现有逻辑）
```c
void balance_control_task(void *arg) {
    robot_state_t state;
    
    while (1) {
        // === 新增：收集原始数据并更新状态 ===
        sensor_raw_data_t raw = {
            .imu = {
                .pitch_deg = imu_data.pitch,
                .pitch_rate_dps = imu_data.pitch_rate,
                // ...
            },
            .wheel = {
                .left_pos_deg = wheel_left_pos,
                .left_vel_rpm = wheel_left_vel,
                // ...
            },
            // ...
        };
        robot_state_update(&raw, &state, false);
        
        // === 现有代码保持不变 ===
        // 暂时不使用 state，只是并行运行验证
        
        // 现有代码继续使用原来的变量...
    }
}
```

### 第二步：逐步替换变量引用
```c
// 原来：
float pitch_rad = DEG2RAD(imu_data.pitch);

// 替换为：
float pitch_rad = state.imu.pitch;  // 已经是 rad
```

### 第三步：替换 VMC 调用
```c
// 原来：
vmc_ctrl_input_t vmc_input = {...};
vmc_ctrl_compute(&vmc_params, &vmc_input, true, &vmc_output);

// 替换为：
// 1. 分离 PD 计算
float F_L = K_L * (target_L - state.left_leg.leg_length);
float F_alpha = roll_output;

// 2. 使用简化接口
vmc_force_input_t force_input = {
    .F_L = F_L,
    .F_alpha = F_alpha,
    .hip_angle_deg = state.left_leg.hip_pos_deg,
    .knee_angle_deg = state.left_leg.knee_pos_deg
};
vmc_force_to_torque(&force_input, true, &output);
```

---

## 4. 共享控制回路

### 4.1 Yaw 回路（所有模式共享）
```c
// 目前只在 LQR 模式使用，应该独立出来
float compute_yaw_control(const robot_state_t *state, float target_yaw_rate) {
    static float yaw_integral = 0;
    
    float yaw_error = target_yaw_rate - state->imu.yaw_rate;
    yaw_integral += yaw_error * state->dt;
    
    return g_yaw_angle_pid.P * yaw_error + 
           g_yaw_angle_pid.I * yaw_integral;
}
```

### 4.2 Roll 回路（VMC/LQR 共享）
```c
// 目前在 VMC 模式使用，应该对 LQR 也可用
float compute_roll_control(const robot_state_t *state) {
    float roll_error = 0 - state->imu.roll;
    return g_roll_pid.P * roll_error + 
           g_roll_pid.D * (0 - state->imu.roll_rate);
}

// Roll -> 左右腿长差异
void apply_roll_to_legs(float roll_output, 
                        float *left_target_L, 
                        float *right_target_L) {
    float delta_L = roll_output * ROLL_TO_LEG_LENGTH_FACTOR;
    *left_target_L = base_L + delta_L;
    *right_target_L = base_L - delta_L;
}
```

---

## 5. 关键注意事项

### 5.1 保持符号约定
```c
// 轮速: 前进为正
state.kinematics.lqr_speed  // 前进 > 0

// 轮扭矩: 前进为负
wheel_torque                // 前进 < 0

// 右腿扭矩: vmc_force_to_torque 已自动取反
```

### 5.2 保持 UI 对接
```c
// Commander 回调保持不变
// g_lqr_angle, g_lqr_speed 等波形输出变量保持不变
// 只是内部计算使用 state 结构
```

### 5.3 渐进式迁移
```c
// 每次只改一小部分，确保编译通过+功能正常
// 可以保留旧代码作为对比验证
#ifdef USE_NEW_STATE
    float pitch = state.imu.pitch;
#else
    float pitch = DEG2RAD(imu_data.pitch);
#endif
```

---

## 6. 测试清单

- [ ] 编译通过
- [ ] LQR 模式正常平衡
- [ ] 双环 PID 模式正常平衡  
- [ ] VMC 模式正常
- [ ] 遥控器前进/后退正确
- [ ] 遥控器左右转向正确
- [ ] Roll 补偿正常
- [ ] 双腿同步正常
- [ ] 波形输出数据正确
- [ ] 参数调节功能正常
