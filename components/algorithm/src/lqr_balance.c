/**
 * @file lqr_balance.c
 * @brief LQR 平衡控制器实现
 * @author Bubble
 * @date 2026-01-15
 * @note 参考 shibo_wheel_leg 项目的 LQR 平衡算法
 */

#include "lqr_balance.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "LQR_BALANCE";

// ============== 默认参数 ==============
static const lqr_params_t default_params = {
    // 角度环 PID
    .angle_kp = 0.7f,
    .angle_ki = 0.1f,
    .angle_kd = 0.00001f,
    .angle_limit = 10.0f,
    
    // 角速度环 PID
    .gyro_kp = 0.05f,
    .gyro_ki = 0.0f,
    .gyro_kd = 0.0f,
    .gyro_limit = 8.0f,
    
    // 位移环 PID
    .distance_kp = 0.5f,
    .distance_ki = 0.0f,
    .distance_kd = 0.0f,
    .distance_limit = 8.0f,
    
    // 速度环 PID
    .speed_kp = 0.5f,
    .speed_ki = 0.0f,
    .speed_kd = 0.0f,
    .speed_limit = 8.0f,
    .speed_kp_min = 0.3f,
    .speed_kp_max = 1.0f,
    
    // LQR 输出 PID
    .lqr_u_kp = 1.0f,
    .lqr_u_ki = 1.0f,
    .lqr_u_kd = 0.0f,
    .lqr_u_limit = 8.0f,
    
    // 偏航控制 PID
    .yaw_angle_kp = 0.4f,
    .yaw_angle_ki = 0.0f,
    .yaw_angle_kd = 0.0f,
    .yaw_angle_limit = 5.0f,
    
    .yaw_gyro_kp = 0.05f,
    .yaw_gyro_ki = 0.0f,
    .yaw_gyro_kd = 0.0f,
    .yaw_gyro_limit = 3.0f,
    
    // 横滚控制 PID
    .roll_kp = 0.0f,
    .roll_ki = 0.0f,
    .roll_kd = 0.0f,
    .roll_limit = 5.0f,
    
    // 零点调整 PID
    .zeropoint_kp = 0.0f,
    .zeropoint_ki = 0.0f,
    .zeropoint_kd = 0.0f,
    .zeropoint_limit = 5.0f,
    
    // 角度零点
    .angle_zeropoint = -0.1f,
    
    // 低通滤波器
    .lpf_joyy_tf = 0.2f,
    .lpf_zeropoint_tf = 0.08f,
    .lpf_roll_tf = 0.1f,
    
    // 安全阈值
    .emergency_angle = 45.0f,
    
    // 轮子离地检测
    .wheel_off_ground_speed_threshold = 50.0f,
    .wheel_off_ground_accel_threshold = 100.0f,
    
    // 输出限幅
    .max_wheel_torque = 8.0f,
    .max_leg_torque = 10.0f,
    
    // 环路使能系数 (默认全部启用)
    .angle_enable = 1.0f,
    .gyro_enable = 1.0f,
    .distance_enable = 1.0f,
    .speed_enable = 1.0f,
    .lqr_u_enable = 1.0f,
};

// 限幅函数
static inline float clamp_f(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void lqr_get_default_params(lqr_params_t *params) {
    if (params != NULL) {
        memcpy(params, &default_params, sizeof(lqr_params_t));
    }
}

// 初始化单个 PID 控制器
static void init_pid(pid_controller_t *pid, float kp, float ki, float kd, float limit) {
    pid_params_t params = {
        .kp = kp,
        .ki = ki,
        .kd = kd,
        .output_min = -limit,
        .output_max = limit,
        .integral_max = limit * 2.0f,
        .d_filter_coef = 0.1f,
    };
    pid_init(pid, &params);
}

esp_err_t lqr_init(lqr_controller_t *ctrl, const lqr_params_t *params) {
    if (ctrl == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memset(ctrl, 0, sizeof(lqr_controller_t));
    
    // 使用提供的参数或默认参数
    if (params != NULL) {
        memcpy(&ctrl->params, params, sizeof(lqr_params_t));
    } else {
        memcpy(&ctrl->params, &default_params, sizeof(lqr_params_t));
    }
    
    const lqr_params_t *p = &ctrl->params;
    
    // 初始化 PID 控制器
    init_pid(&ctrl->pid_angle, p->angle_kp, p->angle_ki, p->angle_kd, p->angle_limit);
    init_pid(&ctrl->pid_gyro, p->gyro_kp, p->gyro_ki, p->gyro_kd, p->gyro_limit);
    init_pid(&ctrl->pid_distance, p->distance_kp, p->distance_ki, p->distance_kd, p->distance_limit);
    init_pid(&ctrl->pid_speed, p->speed_kp, p->speed_ki, p->speed_kd, p->speed_limit);
    init_pid(&ctrl->pid_lqr_u, p->lqr_u_kp, p->lqr_u_ki, p->lqr_u_kd, p->lqr_u_limit);
    init_pid(&ctrl->pid_yaw_angle, p->yaw_angle_kp, p->yaw_angle_ki, p->yaw_angle_kd, p->yaw_angle_limit);
    init_pid(&ctrl->pid_yaw_gyro, p->yaw_gyro_kp, p->yaw_gyro_ki, p->yaw_gyro_kd, p->yaw_gyro_limit);
    init_pid(&ctrl->pid_roll, p->roll_kp, p->roll_ki, p->roll_kd, p->roll_limit);
    init_pid(&ctrl->pid_zeropoint, p->zeropoint_kp, p->zeropoint_ki, p->zeropoint_kd, p->zeropoint_limit);
    
    // 初始化低通滤波器
    lpf_init(&ctrl->lpf_joyy, p->lpf_joyy_tf);
    lpf_init(&ctrl->lpf_zeropoint, p->lpf_zeropoint_tf);
    lpf_init(&ctrl->lpf_roll, p->lpf_roll_tf);
    
    // 初始化状态
    ctrl->distance_zeropoint = 0.0f;
    ctrl->current_angle_zeropoint = p->angle_zeropoint;
    ctrl->yaw_angle_target = 0.0f;
    ctrl->yaw_holding = false;
    ctrl->lqr_distance = 0.0f;
    ctrl->prev_left_wheel_pos = 0.0f;
    ctrl->prev_right_wheel_pos = 0.0f;
    ctrl->first_run = true;
    ctrl->state = LQR_STATE_IDLE;
    ctrl->initialized = true;
    
    ESP_LOGI(TAG, "LQR controller initialized, angle_zeropoint=%.2f", p->angle_zeropoint);
    
    return ESP_OK;
}

void lqr_reset(lqr_controller_t *ctrl) {
    if (ctrl == NULL || !ctrl->initialized) return;
    
    // 重置所有 PID
    pid_reset(&ctrl->pid_angle);
    pid_reset(&ctrl->pid_gyro);
    pid_reset(&ctrl->pid_distance);
    pid_reset(&ctrl->pid_speed);
    pid_reset(&ctrl->pid_lqr_u);
    pid_reset(&ctrl->pid_yaw_angle);
    pid_reset(&ctrl->pid_yaw_gyro);
    pid_reset(&ctrl->pid_roll);
    pid_reset(&ctrl->pid_zeropoint);
    
    // 重置滤波器
    lpf_reset(&ctrl->lpf_joyy);
    lpf_reset(&ctrl->lpf_zeropoint);
    lpf_reset(&ctrl->lpf_roll);
    
    // 重置状态
    ctrl->distance_zeropoint = 0.0f;
    ctrl->yaw_angle_target = 0.0f;
    ctrl->yaw_holding = false;
    ctrl->lqr_distance = 0.0f;
    ctrl->first_run = true;
    
    ESP_LOGI(TAG, "LQR controller reset");
}

void lqr_set_params(lqr_controller_t *ctrl, const lqr_params_t *params) {
    if (ctrl == NULL || params == NULL) return;
    
    memcpy(&ctrl->params, params, sizeof(lqr_params_t));
    
    const lqr_params_t *p = &ctrl->params;
    
    // 更新 PID 参数
    pid_set_gains(&ctrl->pid_angle, p->angle_kp, p->angle_ki, p->angle_kd);
    pid_set_output_limits(&ctrl->pid_angle, -p->angle_limit, p->angle_limit);
    
    pid_set_gains(&ctrl->pid_gyro, p->gyro_kp, p->gyro_ki, p->gyro_kd);
    pid_set_output_limits(&ctrl->pid_gyro, -p->gyro_limit, p->gyro_limit);
    
    pid_set_gains(&ctrl->pid_distance, p->distance_kp, p->distance_ki, p->distance_kd);
    pid_set_output_limits(&ctrl->pid_distance, -p->distance_limit, p->distance_limit);
    
    pid_set_gains(&ctrl->pid_speed, p->speed_kp, p->speed_ki, p->speed_kd);
    pid_set_output_limits(&ctrl->pid_speed, -p->speed_limit, p->speed_limit);
    
    pid_set_gains(&ctrl->pid_lqr_u, p->lqr_u_kp, p->lqr_u_ki, p->lqr_u_kd);
    pid_set_output_limits(&ctrl->pid_lqr_u, -p->lqr_u_limit, p->lqr_u_limit);
    
    pid_set_gains(&ctrl->pid_yaw_angle, p->yaw_angle_kp, p->yaw_angle_ki, p->yaw_angle_kd);
    pid_set_output_limits(&ctrl->pid_yaw_angle, -p->yaw_angle_limit, p->yaw_angle_limit);
    
    pid_set_gains(&ctrl->pid_yaw_gyro, p->yaw_gyro_kp, p->yaw_gyro_ki, p->yaw_gyro_kd);
    pid_set_output_limits(&ctrl->pid_yaw_gyro, -p->yaw_gyro_limit, p->yaw_gyro_limit);
    
    pid_set_gains(&ctrl->pid_roll, p->roll_kp, p->roll_ki, p->roll_kd);
    pid_set_output_limits(&ctrl->pid_roll, -p->roll_limit, p->roll_limit);
    
    pid_set_gains(&ctrl->pid_zeropoint, p->zeropoint_kp, p->zeropoint_ki, p->zeropoint_kd);
    pid_set_output_limits(&ctrl->pid_zeropoint, -p->zeropoint_limit, p->zeropoint_limit);
    
    // 更新滤波器
    lpf_set_tf(&ctrl->lpf_joyy, p->lpf_joyy_tf);
    lpf_set_tf(&ctrl->lpf_zeropoint, p->lpf_zeropoint_tf);
    lpf_set_tf(&ctrl->lpf_roll, p->lpf_roll_tf);
}

void lqr_set_angle_zeropoint(lqr_controller_t *ctrl, float zeropoint) {
    if (ctrl == NULL) return;
    ctrl->params.angle_zeropoint = zeropoint;
    ctrl->current_angle_zeropoint = zeropoint;
}

void lqr_set_speed_kp(lqr_controller_t *ctrl, float kp) {
    if (ctrl == NULL) return;
    ctrl->params.speed_kp = kp;
    pid_set_gains(&ctrl->pid_speed, kp, ctrl->params.speed_ki, ctrl->params.speed_kd);
}

esp_err_t lqr_balance_loop(lqr_controller_t *ctrl, const lqr_input_t *input, lqr_output_t *output) {
    if (ctrl == NULL || input == NULL || output == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!ctrl->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // 清零输出
    memset(output, 0, sizeof(lqr_output_t));
    
    float dt = input->dt;
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.01f;  // 默认 100Hz
    }
    
    // ===== 检查紧急停止 =====
    // 使用 raw_pitch (IMU 原始值) 来判断，不受腿部补偿影响
    // 如果 raw_pitch 为 0 (未设置)，则回退使用 pitch
    float pitch_for_emergency = (input->raw_pitch != 0.0f) ? input->raw_pitch : input->pitch;
    if (lqr_check_emergency(ctrl, pitch_for_emergency)) {
        ctrl->state = LQR_STATE_EMERGENCY;
        output->state = LQR_STATE_EMERGENCY;
        // 限流打印：每秒最多打印一次
        static uint32_t last_emergency_log = 0;
        uint32_t now = (uint32_t)(esp_log_timestamp());
        if (now - last_emergency_log > 1000) {
            ESP_LOGW(TAG, "Emergency stop! raw_pitch=%.2f (pitch=%.2f)", pitch_for_emergency, input->pitch);
            last_emergency_log = now;
        }
        return ESP_ERR_INVALID_STATE;
    }
    
    // ===== 使用外部传入的 LQR 状态量 =====
    // 位移和速度由外部 (balance_test.c) 计算，已经考虑了电机方向
    float lqr_angle = input->pitch;           // 俯仰角
    float lqr_gyro = input->pitch_rate;       // 俯仰角速度
    float lqr_distance = input->lqr_distance; // 累积位移 (由外部计算)
    float lqr_speed = input->lqr_speed;       // 平均速度 (由外部计算)
    
    // ===== 速度输入滤波 =====
    float filtered_target_speed = lpf_compute_dt(&ctrl->lpf_joyy, input->target_speed, dt);
    
    // ===== 获取环路使能系数 =====
    const float angle_en = ctrl->params.angle_enable;
    const float gyro_en = ctrl->params.gyro_enable;
    const float distance_en = ctrl->params.distance_enable;
    const float speed_en = ctrl->params.speed_enable;
    const float lqr_u_en = ctrl->params.lqr_u_enable;
    
    // ===== 角度控制 =====
    // angle_control = pid_angle(LQR_angle - angle_zeropoint) * enable
    float angle_error = lqr_angle - ctrl->current_angle_zeropoint;
    float angle_control = pid_compute(&ctrl->pid_angle, 0.0f, angle_error, dt) * angle_en;
    
    // ===== 角速度控制 =====
    // gyro_control = pid_gyro(LQR_gyro) * enable
    float gyro_control = pid_compute(&ctrl->pid_gyro, 0.0f, lqr_gyro, dt) * gyro_en;
    
    // ===== 位移控制 =====
    // distance_control = pid_distance(LQR_distance - distance_zeropoint) * enable
    float distance_error = -(lqr_distance - ctrl->distance_zeropoint);
    float distance_control = pid_compute(&ctrl->pid_distance, 0.0f, distance_error, dt) * distance_en;
    
    // ===== 速度控制 =====
    // speed_control = pid_speed(LQR_speed - target_speed) * enable
    float speed_error = -(lqr_speed - filtered_target_speed);
    float speed_control = pid_compute(&ctrl->pid_speed, 0.0f, speed_error, dt) * speed_en;
    
    // ===== LQR 综合输出 =====
    // LQR_u = angle_control + gyro_control + distance_control + speed_control
    float lqr_u_raw = angle_control + gyro_control + distance_control + speed_control;
    
    // LQR 输出经过 PID 处理 (主要是积分环节)
    // 当 lqr_u_enable = 0 时，直接使用 lqr_u_raw (跳过积分)
    float lqr_u;
    if (lqr_u_en > 0.0f) {
        lqr_u = pid_compute(&ctrl->pid_lqr_u, 0.0f, -lqr_u_raw, dt) * lqr_u_en 
              + lqr_u_raw * (1.0f - lqr_u_en);
    } else {
        lqr_u = lqr_u_raw;
    }
    
    // ===== 输出限幅 =====
    lqr_u = clamp_f(lqr_u, -ctrl->params.max_wheel_torque, ctrl->params.max_wheel_torque);
    
    // ===== 更新位移零点 (重心自动调整) =====
    // 当速度接近目标时，调整位移零点以补偿重心偏移
    float zeropoint_adjust_raw = 0.0f;
    float zeropoint_adjust_filtered = 0.0f;
    if (fabsf(speed_error) < 0.5f && fabsf(lqr_speed) < 1.0f) {
        zeropoint_adjust_raw = pid_compute(&ctrl->pid_zeropoint, 0.0f, angle_error, dt);
        zeropoint_adjust_filtered = lpf_compute_dt(&ctrl->lpf_zeropoint, zeropoint_adjust_raw, dt);
        ctrl->distance_zeropoint += zeropoint_adjust_filtered;
    }
    
    // ===== 保存输出 =====
    // output->left_wheel_torque = lqr_u;
    // output->right_wheel_torque = lqr_u;
    
    output->angle_control = angle_control;
    output->gyro_control = gyro_control;
    output->distance_control = distance_control;
    output->speed_control = speed_control;
    output->lqr_u = lqr_u;
    output->filtered_target_speed = filtered_target_speed;  // 滤波后的目标速度
    output->zeropoint_adjust_raw = zeropoint_adjust_raw;    // 零点调整原始值
    output->zeropoint_adjust_filtered = zeropoint_adjust_filtered; // 零点调整滤波后
    
    output->state = LQR_STATE_BALANCING;
    output->wheel_on_ground = true;
    
    ctrl->state = LQR_STATE_BALANCING;
    
    return ESP_OK;
}

esp_err_t lqr_yaw_loop(lqr_controller_t *ctrl, const lqr_input_t *input, lqr_output_t *output) {
    if (ctrl == NULL || input == NULL || output == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!ctrl->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    float dt = input->dt;
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.01f;
    }
    
    // ===== 方向保持控制逻辑 =====
    // 参考 shibo_wheel_leg: 松手时锁定当前方向，有输入时跟随角速度
    
    float yaw_control = 0.0f;
    
    if (fabsf(input->target_yaw_rate) > 0.1f) {
        // ===== 有转向输入: 角速度控制模式 =====
        float yaw_rate_error = input->yaw_rate - input->target_yaw_rate;
        yaw_control = pid_compute(&ctrl->pid_yaw_gyro, 0.0f, yaw_rate_error, dt);
        
        // 更新目标角度为当前角度 (为松手时的方向保持做准备)
        ctrl->yaw_angle_target = input->yaw_total;
        ctrl->yaw_holding = false;
    } else {
        // ===== 无转向输入: 方向保持模式 =====
        if (!ctrl->yaw_holding) {
            // 刚松手，锁定当前方向
            ctrl->yaw_angle_target = input->yaw_total;
            ctrl->yaw_holding = true;
            pid_reset(&ctrl->pid_yaw_angle);  // 重置积分，避免跳变
        }
        
        // 角度位置控制: 保持在目标方向
        float yaw_angle_error = input->yaw_total - ctrl->yaw_angle_target;
        float yaw_angle_control = pid_compute(&ctrl->pid_yaw_angle, 0.0f, yaw_angle_error, dt);
        
        // 角速度阻尼: 抑制转动
        float yaw_gyro_control = pid_compute(&ctrl->pid_yaw_gyro, 0.0f, input->yaw_rate, dt);
        
        yaw_control = yaw_angle_control + yaw_gyro_control;
    }
    
    output->yaw_control = yaw_control;
    
    return ESP_OK;
}

bool lqr_check_wheel_off_ground(lqr_controller_t *ctrl, float wheel_speed, float wheel_accel) {
    if (ctrl == NULL) return false;
    
    // 根据速度和加速度判断轮子是否离地
    // 离地时轮子容易快速加速
    if (fabsf(wheel_speed) > ctrl->params.wheel_off_ground_speed_threshold &&
        fabsf(wheel_accel) > ctrl->params.wheel_off_ground_accel_threshold) {
        return true;
    }
    
    return false;
}

bool lqr_check_emergency(lqr_controller_t *ctrl, float pitch) {
    if (ctrl == NULL) return true;
    return fabsf(pitch) > ctrl->params.emergency_angle;
}

void lqr_adaptive_speed_p(lqr_controller_t *ctrl, float leg_height) {
    if (ctrl == NULL) return;
    
    // 腿越高，速度P越小 (稳定性优先)
    // 腿越低，速度P越大 (响应速度优先)
    // 假设腿高范围 0.1m ~ 0.3m
    float height_normalized = (leg_height - 0.1f) / 0.2f;  // 归一化到 0~1
    height_normalized = clamp_f(height_normalized, 0.0f, 1.0f);
    
    // P 值从 speed_kp_max 线性降低到 speed_kp_min
    float kp = ctrl->params.speed_kp_max - height_normalized * 
               (ctrl->params.speed_kp_max - ctrl->params.speed_kp_min);
    
    lqr_set_speed_kp(ctrl, kp);
}

lqr_state_t lqr_get_state(lqr_controller_t *ctrl) {
    if (ctrl == NULL) return LQR_STATE_IDLE;
    return ctrl->state;
}

void lqr_start(lqr_controller_t *ctrl) {
    if (ctrl == NULL) return;
    
    lqr_reset(ctrl);
    ctrl->state = LQR_STATE_BALANCING;
    ESP_LOGI(TAG, "LQR balance started");
}

void lqr_stop(lqr_controller_t *ctrl) {
    if (ctrl == NULL) return;
    
    ctrl->state = LQR_STATE_IDLE;
    ESP_LOGI(TAG, "LQR balance stopped");
}

void lqr_set_loop_enable(lqr_controller_t *ctrl, 
                         float angle_en, float gyro_en, 
                         float distance_en, float speed_en, 
                         float lqr_u_en) {
    if (ctrl == NULL) return;
    
    ctrl->params.angle_enable = clamp_f(angle_en, 0.0f, 1.0f);
    ctrl->params.gyro_enable = clamp_f(gyro_en, 0.0f, 1.0f);
    ctrl->params.distance_enable = clamp_f(distance_en, 0.0f, 1.0f);
    ctrl->params.speed_enable = clamp_f(speed_en, 0.0f, 1.0f);
    ctrl->params.lqr_u_enable = clamp_f(lqr_u_en, 0.0f, 1.0f);
    // Note: Removed ESP_LOGI to avoid blocking in real-time control loop
}

void lqr_set_simple_balance_mode(lqr_controller_t *ctrl) {
    if (ctrl == NULL) return;
    
    // 仅角度+角速度环，无位移/速度/积分
    lqr_set_loop_enable(ctrl, 1.0f, 1.0f, 0.0f, 0.0f, 0.0f);
    // Note: Removed ESP_LOGI to avoid blocking in real-time control loop
}

void lqr_set_full_balance_mode(lqr_controller_t *ctrl) {
    if (ctrl == NULL) return;
    
    // 所有环路启用
    lqr_set_loop_enable(ctrl, 1.0f, 1.0f, 1.0f, 1.0f, 1.0f);
    // Note: Removed ESP_LOGI to avoid blocking in real-time control loop
}

// ============================================================================
// Roll 控制 (用于腿长控制)
// ============================================================================

esp_err_t lqr_roll_loop(lqr_controller_t *ctrl, const lqr_input_t *input, lqr_roll_output_t *roll_output) {
    if (ctrl == NULL || input == NULL || roll_output == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!ctrl->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // 清零输出
    memset(roll_output, 0, sizeof(lqr_roll_output_t));
    
    float dt = input->dt;
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.01f;
    }
    
    // ===== Roll 角度滤波 =====
    float filtered_roll = lpf_compute_dt(&ctrl->lpf_roll, input->roll, dt);
    
    // ===== Roll PID 控制 =====
    // 目标是让 roll = 0 (机身水平)
    float roll_control = pid_compute(&ctrl->pid_roll, 0.0f, filtered_roll, dt);
    
    // ===== 转换为腿长增量 =====
    // Roll 平衡原理:
    //   - roll > 0 (向右倾): 左腿缩短, 右腿伸长 -> 机身回正
    //   - roll < 0 (向左倾): 左腿伸长, 右腿缩短 -> 机身回正
    //
    // roll_control 的符号设计:
    //   - filtered_roll > 0 -> roll_control < 0 -> 需要左腿缩短
    float left_leg_delta = roll_control;
    float right_leg_delta = -roll_control;
    
    // ===== 输出 =====
    roll_output->filtered_roll = filtered_roll;
    roll_output->roll_control = roll_control;
    roll_output->left_leg_delta = left_leg_delta;
    roll_output->right_leg_delta = right_leg_delta;
    
    return ESP_OK;
}

void lqr_roll_reset(lqr_controller_t *ctrl) {
    if (ctrl == NULL || !ctrl->initialized) return;
    
    pid_reset(&ctrl->pid_roll);
    lpf_reset(&ctrl->lpf_roll);
    
    ESP_LOGI(TAG, "Roll controller reset");
}

void lqr_set_distance_zeropoint(lqr_controller_t *ctrl, float zeropoint) {
    if (ctrl == NULL) return;
    ctrl->distance_zeropoint = zeropoint;
}

float lqr_get_distance_zeropoint(lqr_controller_t *ctrl) {
    if (ctrl == NULL) return 0.0f;
    return ctrl->distance_zeropoint;
}

float lqr_yaw_angle_addup(float current_yaw, float last_yaw, float *yaw_total) {
    if (yaw_total == NULL) return 0.0f;
    
    // 计算两种可能的角度差
    // 1. 直接相减
    // 2. 考虑过零 (±360°)
    float delta1, delta2;
    
    if (current_yaw > last_yaw) {
        delta1 = current_yaw - last_yaw;           // 正向变化
        delta2 = current_yaw - last_yaw - 360.0f;  // 考虑从 +180 跳到 -180 (实际减少)
    } else {
        delta1 = current_yaw - last_yaw;           // 负向变化
        delta2 = current_yaw - last_yaw + 360.0f;  // 考虑从 -180 跳到 +180 (实际增加)
    }
    
    // 选择绝对值较小的增量 (避免过零跳变)
    float delta;
    if (fabsf(delta1) > fabsf(delta2)) {
        delta = delta2;
    } else {
        delta = delta1;
    }
    
    // 累加到总角度
    *yaw_total += delta;
    
    return delta;
}
