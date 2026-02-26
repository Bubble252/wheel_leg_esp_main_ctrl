/**
 * @file lqr_balance.c
 * @brief LQR 平衡控制器实现
 * @author Bubble
 * @date 2026-01-15
 * @note 参考 shibo_wheel_leg 项目的 LQR 平衡算法
 */

#include "lqr_balance.h"
#include "leg_kinematics.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "LQR_BALANCE";

// ============== 默认参数 ==============
static const lqr_params_t default_params = {
    // 角度环 PID
    .angle_kp = 0.036f,
    .angle_ki = 0.000f,
    .angle_kd = 0.000001f,
    .angle_limit = 30.0f,
    
    // 角速度环 PID
    .gyro_kp = 0.0005f,
    .gyro_ki = 0.0f,
    .gyro_kd = 0.0f,
    .gyro_limit = 16.0f,
    
    // 位移环 PID
    .distance_kp = 4.0f,
    .distance_ki = 0.0f,
    .distance_kd = 0.0f,
    .distance_limit = 0.05f,
    
    // 速度环 PID
    .speed_kp = 0.5f,
    .speed_ki = 0.0f,
    .speed_kd = 0.0f,
    .speed_limit = 3.0f,
    .speed_kp_min = 0.951f,
    .speed_kp_max = 0.951f,
    
    // LQR 输出 PID
    .lqr_u_kp = 1.0f,
    .lqr_u_ki = 0.5f,
    .lqr_u_kd = 0.0f,
    .lqr_u_limit = 20.0f,
    
    // 偏航控制 PID
    .yaw_angle_kp = 0.0025f,
    .yaw_angle_ki = 0.0f,
    .yaw_angle_kd = 0.0001f,
    .yaw_angle_limit = 10.0f,
    
    .yaw_gyro_kp = 0.0001f,
    .yaw_gyro_ki = 0.0f,
    .yaw_gyro_kd = 0.0f,
    .yaw_gyro_limit = 7.0f,
    
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
    .angle_zeropoint = -10.5f,
    
    // 低通滤波器
    .lpf_joyy_tf = 0.2f,
    .lpf_zeropoint_tf = 0.08f,
    .lpf_roll_tf = 0.1f,
    .lpf_gyro_tf = 0.005f,
    .lpf_speed_tf = 0.01f,
    
    // 角速度滤波模式
    .gyro_filter_mode = 1,       // 默认使用限幅滤波
    .gyro_slew_rate = 1000.0f,    // 限幅滤波最大变化率 (度/秒 per second)
    
    // 轮速滤波模式
    .speed_filter_mode = 1,      // 默认使用限幅滤波
    .speed_slew_rate = 50.0f,    // 限幅滤波最大变化率 (m/s per second)
    
    // 安全阈值
    .emergency_angle = 45.0f,
    
    // 轮子离地检测
    .wheel_off_ground_speed_threshold = 25.0f,
    .wheel_off_ground_accel_threshold = 450.0f,
    
    // 输出限幅
    .max_wheel_torque = 8.0f,
    .max_leg_torque = 10.0f,
    
    // 环路使能系数 (默认全部启用)
    .angle_enable = 1.4f,
    .gyro_enable = 1.0f,
    .distance_enable = 1.0f,
    .speed_enable = 1.4f,
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
        .d_filter_coef = 0.8f,
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
    lpf_init(&ctrl->lpf_gyro, p->lpf_gyro_tf);
    lpf_init(&ctrl->lpf_speed, p->lpf_speed_tf);
    
    // 初始化限幅滤波器
    slewrate_init(&ctrl->sr_gyro, p->gyro_slew_rate);
    slewrate_init(&ctrl->sr_speed, p->speed_slew_rate);
    
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
    lpf_reset(&ctrl->lpf_gyro);
    lpf_reset(&ctrl->lpf_speed);
    
    // 重置限幅滤波器
    slewrate_reset(&ctrl->sr_gyro);
    slewrate_reset(&ctrl->sr_speed);
    
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
    lpf_set_tf(&ctrl->lpf_gyro, p->lpf_gyro_tf);
    lpf_set_tf(&ctrl->lpf_speed, p->lpf_speed_tf);
    
    // 更新限幅滤波器
    slewrate_set_max_rate(&ctrl->sr_gyro, p->gyro_slew_rate);
    slewrate_set_max_rate(&ctrl->sr_speed, p->speed_slew_rate);
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
    
    // 角速度滤波: 根据模式选择 LPF 或 限幅滤波
    float lqr_gyro;
    if (ctrl->params.gyro_filter_mode == 1) {
        // 限幅滤波 (slew-rate limiter): 小变化直通, 大变化钳位
        lqr_gyro = slewrate_compute_dt(&ctrl->sr_gyro, input->pitch_rate, dt);
    } else {
        // 低通滤波 (LPF)
        lqr_gyro = lpf_compute_dt(&ctrl->lpf_gyro, input->pitch_rate, dt);
    }
    
    float lqr_distance = input->lqr_distance; // 累积位移 (由外部计算)
    
    // 轮速滤波: 根据模式选择 LPF 或 限幅滤波
    float lqr_speed;
    if (ctrl->params.speed_filter_mode == 1) {
        // 限幅滤波 (slew-rate limiter): 小变化直通, 大变化钳位
        lqr_speed = slewrate_compute_dt(&ctrl->sr_speed, input->lqr_speed, dt);
    } else {
        // 低通滤波 (默认)
        lqr_speed = lpf_compute_dt(&ctrl->lpf_speed, input->lqr_speed, dt);
    }
    
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
    
    // 注: 角度零点自动调整 (重心补偿) 已移至 balance_test.c 统一处理,
    // 通过 lqr_zeropoint_auto_adjust() 函数实现, 所有模式 (LQR/DualPID/SinglePID) 共用.
    
    // ===== 保存输出 =====
    // output->left_wheel_torque = lqr_u;
    // output->right_wheel_torque = lqr_u;
    
    output->angle_control = angle_control;
    output->gyro_control = gyro_control;
    output->distance_control = distance_control;
    output->speed_control = speed_control;
    output->lqr_u = lqr_u;
    output->lqr_u_raw = lqr_u_raw;
    output->filtered_target_speed = filtered_target_speed;  // 滤波后的目标速度
    output->zeropoint_adjust_raw = 0.0f;    // 由 balance_test.c 统一填充
    output->zeropoint_adjust_filtered = 0.0f; // 由 balance_test.c 统一填充
    output->filtered_gyro = lqr_gyro;                       // 滤波后的角速度
    output->filtered_speed = lqr_speed;                     // 滤波后的轮速
    
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
    // 腿高范围由 LEG_LENGTH_MIN ~ LEG_LENGTH_MAX 决定
    float height_normalized = (leg_height - LEG_LENGTH_MIN) / (LEG_LENGTH_MAX - LEG_LENGTH_MIN);  // 归一化到 0~1
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
    
    ctrl->params.angle_enable = clamp_f(angle_en, 0.0f, 5.0f);
    ctrl->params.gyro_enable = clamp_f(gyro_en, 0.0f, 5.0f);
    ctrl->params.distance_enable = clamp_f(distance_en, 0.0f, 5.0f);
    ctrl->params.speed_enable = clamp_f(speed_en, 0.0f, 5.0f);
    ctrl->params.lqr_u_enable = clamp_f(lqr_u_en, 0.0f, 5.0f);
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

// ============================================================================
// 角度零点自动调整 (重心补偿, 所有模式通用)
// ============================================================================

float lqr_zeropoint_auto_adjust(lqr_controller_t *ctrl, float angle_error,
                                 float wheel_speed, float dt,
                                 float *out_raw, float *out_filtered) {
    if (ctrl == NULL || !ctrl->initialized) {
        if (out_raw) *out_raw = 0.0f;
        if (out_filtered) *out_filtered = 0.0f;
        return 0.0f;
    }
    
    float adjust_raw = 0.0f;
    float adjust_filtered = 0.0f;
    float delta = 0.0f;
    
    // 仅在近乎静止时启用自动调整, 避免运动中干扰
    if (fabsf(wheel_speed) < 0.1f) {
        // 使用 zeropoint PID: 角度误差 → 零点调整量
        // pid_compute(0, angle_error): error = 0 - angle_error = -angle_error
        //   → output ∝ -angle_error → delta 与 angle_error 反号
        adjust_raw = pid_compute(&ctrl->pid_zeropoint, 0.0f, angle_error, dt);
        // 低通滤波: 确保调整非常平缓
        adjust_filtered = lpf_compute_dt(&ctrl->lpf_zeropoint, adjust_raw, dt);
        delta = adjust_filtered;
    }
    
    if (out_raw) *out_raw = adjust_raw;
    if (out_filtered) *out_filtered = adjust_filtered;
    
    return delta;
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

// ============================================================================
// 双环 PID 控制器实现 (直立环 + 速度环)
// ============================================================================

// 双环PID默认参数
static const dual_pid_params_t dual_pid_default_params = {
    // 角度环 (内环): 倾角误差 → 扭矩
    .angle_kp = 0.025f,
    .angle_ki = 0.000003f,
    .angle_kd = 0.003f,
    .angle_limit = 50.0f,       // 最大目标速度 50 rad/s

    // 速度环 (外环): 0 - 轮速 → 目标倾角
    .speed_kp = 0.0005f,
    .speed_ki = 0.000001f,
    .speed_kd = 0.000015f,
    .speed_limit = 20.0f,        // 最大扭矩 20 Nm

    // 角度零点
    .angle_zeropoint = -7.0f,
    
    // 安全阈值
    .emergency_angle = 45.0f,
    
    // 输出限幅
    .max_torque = 15.0f,
    
    // 速度指令增益 (SPEED_FIRST 模式外环)
    .speed_cmd_gain = 33333.0f,
    
    // 遥杆目标速度低通滤波 (与 LQR 的 lpf_joyy 相同)
    .lpf_joyy_tf = 0.2f,
    
    // 角速度阻尼环 (默认关闭, kp=0)
    .gyro_kp = 0.0f,
    .gyro_ki = 0.0f,
    .gyro_kd = 0.0f,
    .gyro_limit = 10.0f,
    
    // 环路顺序: 速度优先 (默认)
    .loop_order = DUAL_PID_SPEED_FIRST,
};

void dual_pid_get_default_params(dual_pid_params_t *params) {
    if (params != NULL) {
        memcpy(params, &dual_pid_default_params, sizeof(dual_pid_params_t));
    }
}

esp_err_t dual_pid_init(dual_pid_controller_t *ctrl, const dual_pid_params_t *params) {
    if (ctrl == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memset(ctrl, 0, sizeof(dual_pid_controller_t));
    
    // 使用提供的参数或默认参数
    if (params != NULL) {
        memcpy(&ctrl->params, params, sizeof(dual_pid_params_t));
    } else {
        memcpy(&ctrl->params, &dual_pid_default_params, sizeof(dual_pid_params_t));
    }
    
    const dual_pid_params_t *p = &ctrl->params;
    
    // 初始化直立环 PID (外环)
    pid_params_t angle_pid_params = {
        .kp = p->angle_kp,
        .ki = p->angle_ki,
        .kd = p->angle_kd,
        .output_min = -p->angle_limit,
        .output_max = p->angle_limit,
        .integral_max = p->angle_limit * 2.0f,
        .d_filter_coef = 0.8f,
    };
    pid_init(&ctrl->pid_angle, &angle_pid_params);
    
    // 初始化速度环 PID (内环)
    pid_params_t speed_pid_params = {
        .kp = p->speed_kp,
        .ki = p->speed_ki,
        .kd = p->speed_kd,
        .output_min = -p->speed_limit,
        .output_max = p->speed_limit,
        .integral_max = p->speed_limit * 2.0f,
        .d_filter_coef = 0.8f,
    };
    pid_init(&ctrl->pid_speed, &speed_pid_params);
    
    // 初始化遥杆目标速度低通滤波器
    lpf_init(&ctrl->lpf_joyy, p->lpf_joyy_tf);
    
    // 初始化角速度阻尼环 PID
    pid_params_t gyro_pid_params = {
        .kp = p->gyro_kp,
        .ki = p->gyro_ki,
        .kd = p->gyro_kd,
        .output_min = -p->gyro_limit,
        .output_max = p->gyro_limit,
        .integral_max = p->gyro_limit * 2.0f,
        .d_filter_coef = 0.8f,
    };
    pid_init(&ctrl->pid_gyro, &gyro_pid_params);
    
    ctrl->initialized = true;
    
    ESP_LOGI(TAG, "Dual PID controller initialized (loop_order=%s)",
             p->loop_order == DUAL_PID_SPEED_FIRST ? "SPEED_FIRST" : "ANGLE_FIRST");
    ESP_LOGI(TAG, "  Angle PID: kp=%.2f, ki=%.2f, kd=%.3f, limit=%.1f",
             p->angle_kp, p->angle_ki, p->angle_kd, p->angle_limit);
    ESP_LOGI(TAG, "  Speed PID: kp=%.2f, ki=%.2f, kd=%.3f, limit=%.1f",
             p->speed_kp, p->speed_ki, p->speed_kd, p->speed_limit);
    ESP_LOGI(TAG, "  Gyro PID:  kp=%.4f, ki=%.4f, kd=%.4f, limit=%.1f",
             p->gyro_kp, p->gyro_ki, p->gyro_kd, p->gyro_limit);
    
    return ESP_OK;
}

void dual_pid_reset(dual_pid_controller_t *ctrl) {
    if (ctrl == NULL || !ctrl->initialized) return;
    
    pid_reset(&ctrl->pid_angle);
    pid_reset(&ctrl->pid_speed);
    pid_reset(&ctrl->pid_gyro);
    lpf_reset(&ctrl->lpf_joyy);
    
    ESP_LOGI(TAG, "Dual PID controller reset");
}

void dual_pid_set_params(dual_pid_controller_t *ctrl, const dual_pid_params_t *params) {
    if (ctrl == NULL || params == NULL) return;
    
    memcpy(&ctrl->params, params, sizeof(dual_pid_params_t));
    
    const dual_pid_params_t *p = &ctrl->params;
    
    // 更新直立环 PID
    pid_set_gains(&ctrl->pid_angle, p->angle_kp, p->angle_ki, p->angle_kd);
    pid_set_output_limits(&ctrl->pid_angle, -p->angle_limit, p->angle_limit);
    
    // 更新速度环 PID
    pid_set_gains(&ctrl->pid_speed, p->speed_kp, p->speed_ki, p->speed_kd);
    pid_set_output_limits(&ctrl->pid_speed, -p->speed_limit, p->speed_limit);
    
    // 更新角速度阻尼环 PID
    pid_set_gains(&ctrl->pid_gyro, p->gyro_kp, p->gyro_ki, p->gyro_kd);
    pid_set_output_limits(&ctrl->pid_gyro, -p->gyro_limit, p->gyro_limit);
    
    // 更新遥杆低通滤波器
    lpf_set_tf(&ctrl->lpf_joyy, p->lpf_joyy_tf);
}

void dual_pid_set_angle_gains(dual_pid_controller_t *ctrl, float kp, float ki, float kd) {
    if (ctrl == NULL) return;
    
    ctrl->params.angle_kp = kp;
    ctrl->params.angle_ki = ki;
    ctrl->params.angle_kd = kd;
    pid_set_gains(&ctrl->pid_angle, kp, ki, kd);
}

void dual_pid_set_speed_gains(dual_pid_controller_t *ctrl, float kp, float ki, float kd) {
    if (ctrl == NULL) return;
    
    ctrl->params.speed_kp = kp;
    ctrl->params.speed_ki = ki;
    ctrl->params.speed_kd = kd;
    pid_set_gains(&ctrl->pid_speed, kp, ki, kd);
}

void dual_pid_set_gyro_gains(dual_pid_controller_t *ctrl, float kp, float ki, float kd) {
    if (ctrl == NULL) return;
    
    ctrl->params.gyro_kp = kp;
    ctrl->params.gyro_ki = ki;
    ctrl->params.gyro_kd = kd;
    pid_set_gains(&ctrl->pid_gyro, kp, ki, kd);
}

void dual_pid_set_angle_zeropoint(dual_pid_controller_t *ctrl, float zeropoint) {
    if (ctrl == NULL) return;
    ctrl->params.angle_zeropoint = zeropoint;
}

void dual_pid_set_loop_order(dual_pid_controller_t *ctrl, uint8_t loop_order) {
    if (ctrl == NULL) return;
    ctrl->params.loop_order = loop_order;
    // 切换模式时重置PID积分，避免遗留积分导致跳变
    pid_reset(&ctrl->pid_angle);
    pid_reset(&ctrl->pid_speed);
    ESP_LOGI(TAG, "Dual PID loop order set to %s", 
             loop_order == DUAL_PID_SPEED_FIRST ? "SPEED_FIRST" : "ANGLE_FIRST");
}

bool dual_pid_check_emergency(dual_pid_controller_t *ctrl, float pitch) {
    if (ctrl == NULL) return true;
    return fabsf(pitch) > ctrl->params.emergency_angle;
}

esp_err_t dual_pid_balance_loop(dual_pid_controller_t *ctrl, 
                                 float pitch, float pitch_rate,
                                 float wheel_speed, float target_speed,
                                 float dt,
                                 dual_pid_output_t *output) {
    if (ctrl == NULL || output == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!ctrl->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // 清零输出
    memset(output, 0, sizeof(dual_pid_output_t));
    
    // 时间步长检查
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.005f;  // 默认 200Hz
    }
    
    // ===== 紧急停止检查 =====
    if (dual_pid_check_emergency(ctrl, pitch)) {
        output->emergency = true;
        output->torque = 0.0f;
        return ESP_ERR_INVALID_STATE;
    }
    
    // ===== 遥杆目标速度低通滤波 (与 LQR 的 lpf_joyy 相同) =====
    float filtered_target_speed = lpf_compute_dt(&ctrl->lpf_joyy, target_speed, dt);
    
    if (ctrl->params.loop_order == DUAL_PID_SPEED_FIRST) {
        // =============================================================
        // 速度优先模式 (经典串级PID):
        //   外环: 速度环  target_speed - wheel_speed → pitch_target (目标倾角)
        //   内环: 角度环  pitch_target - pitch → torque (输出扭矩)
        //
        // 物理直觉: 速度偏了 → 倾斜去纠正 → 扭矩维持倾斜
        // =============================================================
        
        // ===== 速度环 (外环): target_speed - wheel_speed → pitch_target =====
        // 当 wheel_speed > target_speed → 需要后倾 (pitch_target < 0) 来减速
        // 当 wheel_speed < target_speed → 需要前倾 (pitch_target > 0) 来加速
        // 
        // target_speed 经过 speed_cmd_gain 放大后再送入速度环:
        //   原始 target_speed 量级很小 (如 0.3 rad/s), speed_kp 也很小 (如 0.00015),
        //   直接相乘得到的 pitch_target 几乎为零.
        //   speed_cmd_gain 将 target_speed 放大到速度环能有效响应的量级.
        //
        // pid_compute(0, wheel_speed - amplified_target): error = 0 - (wheel_speed - amplified_target)
        //   = amplified_target - wheel_speed
        //   wheel_speed > amplified_target → error < 0 → output < 0 (负倾角=后倾) ✓
        float amplified_target = filtered_target_speed * ctrl->params.speed_cmd_gain;
        float speed_measurement = wheel_speed - amplified_target;
        float pitch_target = pid_compute(&ctrl->pid_speed, 0.0f, speed_measurement, dt);
        
        // 保存速度环调试信息 (复用 output 字段)
        output->speed_error = amplified_target - wheel_speed;
        output->target_speed = pitch_target;  // 在此模式下含义 = pitch_target
        output->speed_p_out = ctrl->params.speed_kp * (amplified_target - wheel_speed);
        output->speed_i_out = ctrl->pid_speed.integral;
        output->speed_d_out = ctrl->pid_speed.prev_d_term;
        
        // ===== 角度环 (内环): pitch_target - pitch → torque =====
        // 目标: 让 pitch 趋近 pitch_target (而非0)
        // pitch_target 已含 angle_zeropoint 的补偿意义，但为了与 angle-first 模式
        // 保持一致的 zeropoint 行为，将 zeropoint 也加到 target 上
        float adjusted_target = pitch_target + ctrl->params.angle_zeropoint;
        float angle_error = pitch - adjusted_target;
        
        // pid_compute(0, -angle_error): error = 0 - (-angle_error) = angle_error
        //   pitch > adjusted_target → angle_error > 0 → output > 0
        //   前倾超过目标 → 需要正扭矩(向前加速追上去) ✓
        float torque = pid_compute(&ctrl->pid_angle, 0.0f, -angle_error, dt);
        
        // ===== 角速度阻尼 (参考 LQR gyro_control) =====
        // pid_gyro(0, pitch_rate): 抑制角速度，提供阻尼
        // pitch_rate > 0 (正在前倾) → gyro_control < 0 (产生后倾扭矩来阻尼)
        float gyro_control = pid_compute(&ctrl->pid_gyro, 0.0f, pitch_rate, dt);
        torque += gyro_control;
        
        // 输出限幅
        torque = clamp_f(torque, -ctrl->params.max_torque, ctrl->params.max_torque);
        
        // 保存角度环调试信息
        output->angle_error = angle_error;
        output->torque = -torque;  // 与 angle-first 模式保持一致的符号约定
        output->angle_p_out = ctrl->params.angle_kp * (-angle_error);
        output->angle_i_out = ctrl->pid_angle.integral;
        output->angle_d_out = ctrl->pid_angle.prev_d_term;
        output->gyro_p_out = ctrl->params.gyro_kp * (0.0f - pitch_rate);
        output->gyro_i_out = ctrl->pid_gyro.integral;
        output->gyro_d_out = ctrl->pid_gyro.prev_d_term;
        output->gyro_control = gyro_control;
        
    } else {
        // =============================================================
        // 角度优先模式 (默认，原有逻辑):
        //   外环: 角度环  pitch → target_speed (目标轮速)
        //   内环: 速度环  speed_error → torque (输出扭矩)
        //
        // 物理直觉: 倒了 → 给速度追 → 扭矩实现该速度
        // =============================================================
        
        // ===== 角度环 (外环): pitch → target_speed =====
        // 目标: 让 pitch 趋近于 angle_zeropoint (通常是 0)
        // 当机器人前倾 (pitch > 0) 时，需要向前加速 (正速度)
        // 当机器人后倾 (pitch < 0) 时，需要向后加速 (负速度)
        // 
        // 遥杆前进后退: target_speed 产生角度偏移
        // 倒立摆物理: 要前进必须先前倾, 前倾=pitch更正(更不负)
        // 但 ANGLE_FIRST 的信号链经过两次取反(pid_compute的measurement取负 + output取负),
        // 最终: zeropoint 减小 → angle_error 增大(正) → torque 变负 → 轮子后转 → 机体前倾 → 前进
        // 所以: target_speed > 0(前进) → zeropoint 减小(负偏移)
        //        target_speed < 0(后退) → zeropoint 增大(正偏移)
        // 系数含义: target_speed (rad/s) → 角度偏移 (度), 需要合适的增益
        float speed_to_angle_gain = 2.0f;  // 1 rad/s 目标速度 → 2度倾角偏移
        float adjusted_zeropoint = ctrl->params.angle_zeropoint - filtered_target_speed * speed_to_angle_gain;
        float angle_error = pitch - adjusted_zeropoint;
        
        // 直立环 PID 计算
        // 注意: 误差取负号，因为我们希望 pitch 减小时输出正速度
        float target_speed = pid_compute(&ctrl->pid_angle, 0.0f, -angle_error, dt);
        
        // 保存直立环调试信息
        output->angle_error = angle_error;
        output->target_speed = target_speed;
        output->angle_p_out = ctrl->params.angle_kp * (-angle_error);
        output->angle_i_out = ctrl->pid_angle.integral;
        output->angle_d_out = ctrl->pid_angle.prev_d_term;
        
        // ===== 速度环 (内环): speed_error → torque =====
        // 目标: 让 wheel_speed 趋近于 target_speed
        float speed_error = target_speed - wheel_speed;
        
        // 速度环 PID 计算
        // 注意: 使用 -speed_error 作为 measurement，使得 error = 0 - (-speed_error) = speed_error
        // 这样当 target_speed > wheel_speed 时，输出正扭矩来加速
        float torque = pid_compute(&ctrl->pid_speed, 0.0f, -speed_error, dt);
        
        // ===== 角速度阻尼 (参考 LQR gyro_control) =====
        float gyro_control = pid_compute(&ctrl->pid_gyro, 0.0f, pitch_rate, dt);
        torque += gyro_control;
        
        // 输出限幅
        torque = clamp_f(torque, -ctrl->params.max_torque, ctrl->params.max_torque);
        
        // 保存速度环调试信息
        output->speed_error = speed_error;
        output->torque = -torque;
        output->speed_p_out = ctrl->params.speed_kp * speed_error;
        output->speed_i_out = ctrl->pid_speed.integral;
        output->speed_d_out = ctrl->pid_speed.prev_d_term;
        output->gyro_p_out = ctrl->params.gyro_kp * (0.0f - pitch_rate);
        output->gyro_i_out = ctrl->pid_gyro.integral;
        output->gyro_d_out = ctrl->pid_gyro.prev_d_term;
        output->gyro_control = gyro_control;
    }
    
    output->emergency = false;
    
    return ESP_OK;
}

// ============================================================================
// 单环 PID 控制器实现 (直立环 → 目标速度，配合轮电机速度模式)
// ============================================================================

// 单环PID默认参数
static const single_pid_params_t single_pid_default_params = {
    .angle_kp = 15.0f,
    .angle_ki = 0.0f,
    .angle_kd = 0.5f,
    .angle_limit = 100.0f,      // 最大目标速度 100 rad/s
    .angle_zeropoint = 0.0f,
    .emergency_angle = 45.0f,
};

void single_pid_get_default_params(single_pid_params_t *params) {
    if (params != NULL) {
        memcpy(params, &single_pid_default_params, sizeof(single_pid_params_t));
    }
}

esp_err_t single_pid_init(single_pid_controller_t *ctrl, const single_pid_params_t *params) {
    if (ctrl == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memset(ctrl, 0, sizeof(single_pid_controller_t));
    
    if (params != NULL) {
        memcpy(&ctrl->params, params, sizeof(single_pid_params_t));
    } else {
        memcpy(&ctrl->params, &single_pid_default_params, sizeof(single_pid_params_t));
    }
    
    const single_pid_params_t *p = &ctrl->params;
    
    // 初始化直立环 PID
    pid_params_t angle_pid_params = {
        .kp = p->angle_kp,
        .ki = p->angle_ki,
        .kd = p->angle_kd,
        .output_min = -p->angle_limit,
        .output_max = p->angle_limit,
        .integral_max = p->angle_limit * 2.0f,
        .d_filter_coef = 0.8f,
    };
    pid_init(&ctrl->pid_angle, &angle_pid_params);
    
    ctrl->initialized = true;
    
    ESP_LOGI(TAG, "Single PID controller initialized");
    ESP_LOGI(TAG, "  Angle PID: kp=%.2f, ki=%.2f, kd=%.3f, limit=%.1f rad/s",
             p->angle_kp, p->angle_ki, p->angle_kd, p->angle_limit);
    
    return ESP_OK;
}

void single_pid_reset(single_pid_controller_t *ctrl) {
    if (ctrl == NULL || !ctrl->initialized) return;
    pid_reset(&ctrl->pid_angle);
    ESP_LOGI(TAG, "Single PID controller reset");
}

void single_pid_set_params(single_pid_controller_t *ctrl, const single_pid_params_t *params) {
    if (ctrl == NULL || params == NULL) return;
    
    memcpy(&ctrl->params, params, sizeof(single_pid_params_t));
    
    const single_pid_params_t *p = &ctrl->params;
    pid_set_gains(&ctrl->pid_angle, p->angle_kp, p->angle_ki, p->angle_kd);
    pid_set_output_limits(&ctrl->pid_angle, -p->angle_limit, p->angle_limit);
}

void single_pid_set_angle_gains(single_pid_controller_t *ctrl, float kp, float ki, float kd) {
    if (ctrl == NULL) return;
    ctrl->params.angle_kp = kp;
    ctrl->params.angle_ki = ki;
    ctrl->params.angle_kd = kd;
    pid_set_gains(&ctrl->pid_angle, kp, ki, kd);
}

void single_pid_set_angle_zeropoint(single_pid_controller_t *ctrl, float zeropoint) {
    if (ctrl == NULL) return;
    ctrl->params.angle_zeropoint = zeropoint;
}

bool single_pid_check_emergency(single_pid_controller_t *ctrl, float pitch) {
    if (ctrl == NULL) return true;
    return fabsf(pitch) > ctrl->params.emergency_angle;
}

esp_err_t single_pid_balance_loop(single_pid_controller_t *ctrl, 
                                   float pitch, float pitch_rate,
                                   float dt,
                                   single_pid_output_t *output) {
    if (ctrl == NULL || output == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!ctrl->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    memset(output, 0, sizeof(single_pid_output_t));
    
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.005f;
    }
    
    // 紧急停止检查
    if (single_pid_check_emergency(ctrl, pitch)) {
        output->emergency = true;
        output->target_speed = 0.0f;
        return ESP_ERR_INVALID_STATE;
    }
    
    // 角度误差: pitch 相对于零点的偏差
    float angle_error = pitch - ctrl->params.angle_zeropoint;
    
    // 直立环 PID: angle_error → target_speed
    // 前倾 (pitch > 0) → 需要正速度(向前追) → error > 0 → output > 0
    float target_speed = pid_compute(&ctrl->pid_angle, 0.0f, -angle_error, dt);
    
    // 保存调试信息
    output->angle_error = angle_error;
    output->target_speed = -target_speed;  // 取负使符号直觉正确: 前倾→前进
    output->angle_p_out = ctrl->params.angle_kp * (-angle_error);
    output->angle_i_out = ctrl->pid_angle.integral;
    output->angle_d_out = ctrl->pid_angle.prev_d_term;
    output->emergency = false;
    
    return ESP_OK;
}

// ============================================================================
// 三环 PID 控制器实现 (速度环→角度环→轮速环)
// ============================================================================

// 三环PID默认参数 (前两环复用双环PID SPEED_FIRST 的参数)
static const triple_pid_params_t triple_pid_default_params = {
    // 速度环 (外环): target_speed - wheel_speed → pitch_target
    .speed_kp = 0.85f,
    .speed_ki = 0.00006f,
    .speed_kd = 0.0f,
    .speed_limit = 40.0f,       // 最大目标倾角 20 deg

    // 角度环 (中环): pitch_target - pitch → wheel_speed_target
    .angle_kp = 1.2f,
    .angle_ki = 0.0f,
    .angle_kd = 0.0f,
    .angle_limit = 100.0f,       // 最大目标轮速 50 rad/s

    // 轮速环 (内环): wheel_speed_target - wheel_speed → torque
    .wheel_kp = 0.5f,
    .wheel_ki = 0.01f,
    .wheel_kd = 0.0f,
    .wheel_limit = 15.0f,       // 最大扭矩 15 Nm

    // 通用参数
    .angle_zeropoint = -7.0f,
    .emergency_angle = 45.0f,
    .max_torque = 15.0f,
    .speed_cmd_gain = 15.0f,
    .lpf_joyy_tf = 0.2f,
    
    // 角速度阻尼环
    .gyro_kp = 0.1f,
    .gyro_ki = 0.0f,
    .gyro_kd = 0.0f,
    .gyro_limit = 10.0f,
    
    // 默认使用电机速度模式 (不经过软件轮速PID)
    .wheel_mode = TRIPLE_PID_WHEEL_SPEED,
};

void triple_pid_get_default_params(triple_pid_params_t *params) {
    if (params != NULL) {
        memcpy(params, &triple_pid_default_params, sizeof(triple_pid_params_t));
    }
}

esp_err_t triple_pid_init(triple_pid_controller_t *ctrl, const triple_pid_params_t *params) {
    if (ctrl == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    memset(ctrl, 0, sizeof(triple_pid_controller_t));
    
    if (params != NULL) {
        memcpy(&ctrl->params, params, sizeof(triple_pid_params_t));
    } else {
        memcpy(&ctrl->params, &triple_pid_default_params, sizeof(triple_pid_params_t));
    }
    
    const triple_pid_params_t *p = &ctrl->params;
    
    // 初始化角度环 PID (中环)
    pid_params_t angle_pid_params = {
        .kp = p->angle_kp,
        .ki = p->angle_ki,
        .kd = p->angle_kd,
        .output_min = -p->angle_limit,
        .output_max = p->angle_limit,
        .integral_max = p->angle_limit * 2.0f,
        .d_filter_coef = 0.8f,
    };
    pid_init(&ctrl->pid_angle, &angle_pid_params);
    
    // 初始化速度环 PID (外环)
    pid_params_t speed_pid_params = {
        .kp = p->speed_kp,
        .ki = p->speed_ki,
        .kd = p->speed_kd,
        .output_min = -p->speed_limit,
        .output_max = p->speed_limit,
        .integral_max = p->speed_limit * 2.0f,
        .d_filter_coef = 0.8f,
    };
    pid_init(&ctrl->pid_speed, &speed_pid_params);
    
    // 初始化轮速环 PID (内环, 仅 WHEEL_TORQUE 模式使用)
    pid_params_t wheel_pid_params = {
        .kp = p->wheel_kp,
        .ki = p->wheel_ki,
        .kd = p->wheel_kd,
        .output_min = -p->wheel_limit,
        .output_max = p->wheel_limit,
        .integral_max = p->wheel_limit * 2.0f,
        .d_filter_coef = 0.8f,
    };
    pid_init(&ctrl->pid_wheel, &wheel_pid_params);
    
    // 初始化角速度阻尼环 PID
    pid_params_t gyro_pid_params = {
        .kp = p->gyro_kp,
        .ki = p->gyro_ki,
        .kd = p->gyro_kd,
        .output_min = -p->gyro_limit,
        .output_max = p->gyro_limit,
        .integral_max = p->gyro_limit * 2.0f,
        .d_filter_coef = 0.8f,
    };
    pid_init(&ctrl->pid_gyro, &gyro_pid_params);
    
    // 初始化遥杆目标速度低通滤波器
    lpf_init(&ctrl->lpf_joyy, p->lpf_joyy_tf);
    
    ctrl->initialized = true;
    
    ESP_LOGI(TAG, "Triple PID controller initialized (wheel_mode=%s)",
             p->wheel_mode == TRIPLE_PID_WHEEL_SPEED ? "SPEED_CMD" : "TORQUE_PID");
    ESP_LOGI(TAG, "  Speed PID (outer): kp=%.4f, ki=%.6f, kd=%.6f",
             p->speed_kp, p->speed_ki, p->speed_kd);
    ESP_LOGI(TAG, "  Angle PID (mid):   kp=%.4f, ki=%.6f, kd=%.4f",
             p->angle_kp, p->angle_ki, p->angle_kd);
    ESP_LOGI(TAG, "  Gyro PID (damp):   kp=%.4f, ki=%.4f, kd=%.4f",
             p->gyro_kp, p->gyro_ki, p->gyro_kd);
    ESP_LOGI(TAG, "  Wheel PID (inner): kp=%.4f, ki=%.4f, kd=%.4f",
             p->wheel_kp, p->wheel_ki, p->wheel_kd);
    
    return ESP_OK;
}

void triple_pid_reset(triple_pid_controller_t *ctrl) {
    if (ctrl == NULL || !ctrl->initialized) return;
    
    pid_reset(&ctrl->pid_angle);
    pid_reset(&ctrl->pid_speed);
    pid_reset(&ctrl->pid_wheel);
    pid_reset(&ctrl->pid_gyro);
    lpf_reset(&ctrl->lpf_joyy);
    
    ESP_LOGI(TAG, "Triple PID controller reset");
}

void triple_pid_set_angle_gains(triple_pid_controller_t *ctrl, float kp, float ki, float kd) {
    if (ctrl == NULL) return;
    ctrl->params.angle_kp = kp;
    ctrl->params.angle_ki = ki;
    ctrl->params.angle_kd = kd;
    pid_set_gains(&ctrl->pid_angle, kp, ki, kd);
}

void triple_pid_set_speed_gains(triple_pid_controller_t *ctrl, float kp, float ki, float kd) {
    if (ctrl == NULL) return;
    ctrl->params.speed_kp = kp;
    ctrl->params.speed_ki = ki;
    ctrl->params.speed_kd = kd;
    pid_set_gains(&ctrl->pid_speed, kp, ki, kd);
}

void triple_pid_set_wheel_gains(triple_pid_controller_t *ctrl, float kp, float ki, float kd) {
    if (ctrl == NULL) return;
    ctrl->params.wheel_kp = kp;
    ctrl->params.wheel_ki = ki;
    ctrl->params.wheel_kd = kd;
    pid_set_gains(&ctrl->pid_wheel, kp, ki, kd);
}

void triple_pid_set_gyro_gains(triple_pid_controller_t *ctrl, float kp, float ki, float kd) {
    if (ctrl == NULL) return;
    ctrl->params.gyro_kp = kp;
    ctrl->params.gyro_ki = ki;
    ctrl->params.gyro_kd = kd;
    pid_set_gains(&ctrl->pid_gyro, kp, ki, kd);
}

void triple_pid_set_angle_zeropoint(triple_pid_controller_t *ctrl, float zeropoint) {
    if (ctrl == NULL) return;
    ctrl->params.angle_zeropoint = zeropoint;
}

void triple_pid_set_wheel_mode(triple_pid_controller_t *ctrl, uint8_t wheel_mode) {
    if (ctrl == NULL) return;
    ctrl->params.wheel_mode = wheel_mode;
    // 切换模式时重置轮速环PID积分
    pid_reset(&ctrl->pid_wheel);
    ESP_LOGI(TAG, "Triple PID wheel mode set to %s",
             wheel_mode == TRIPLE_PID_WHEEL_SPEED ? "SPEED_CMD" : "TORQUE_PID");
}

bool triple_pid_check_emergency(triple_pid_controller_t *ctrl, float pitch) {
    if (ctrl == NULL) return true;
    return fabsf(pitch) > ctrl->params.emergency_angle;
}

esp_err_t triple_pid_balance_loop(triple_pid_controller_t *ctrl,
                                   float pitch, float pitch_rate,
                                   float wheel_speed, float target_speed,
                                   float dt,
                                   triple_pid_output_t *output) {
    if (ctrl == NULL || output == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!ctrl->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    memset(output, 0, sizeof(triple_pid_output_t));
    
    if (dt <= 0.0f || dt > 0.1f) {
        dt = 0.005f;
    }
    
    // ===== 紧急停止检查 =====
    if (triple_pid_check_emergency(ctrl, pitch)) {
        output->emergency = true;
        output->torque = 0.0f;
        return ESP_ERR_INVALID_STATE;
    }
    
    // ===== 遥杆目标速度低通滤波 =====
    float filtered_target_speed = lpf_compute_dt(&ctrl->lpf_joyy, target_speed, dt);
    
    // =============================================================
    // 第一环: 速度环 (外环)
    //   target_speed - wheel_speed → pitch_target
    //   (与双环PID SPEED_FIRST 的速度环完全相同)
    // =============================================================
    float amplified_target = filtered_target_speed * ctrl->params.speed_cmd_gain;
    float speed_measurement = wheel_speed - amplified_target;
    float pitch_target = pid_compute(&ctrl->pid_speed, 0.0f, speed_measurement, dt);
    
    output->speed_error = amplified_target - wheel_speed;
    output->pitch_target = pitch_target;
    output->speed_p_out = ctrl->params.speed_kp * (amplified_target - wheel_speed);
    output->speed_i_out = ctrl->pid_speed.integral;
    output->speed_d_out = ctrl->pid_speed.prev_d_term;
    
    // =============================================================
    // 第二环: 角度环 (中环)
    //   pitch_target - pitch → wheel_speed_target
    //   (与双环PID SPEED_FIRST 的角度环类似，但输出含义变了:
    //    双环输出 torque, 三环输出 wheel_speed_target)
    // =============================================================
    float adjusted_target = pitch_target + ctrl->params.angle_zeropoint;
    float angle_error = pitch - adjusted_target;
    
    float wheel_speed_target = pid_compute(&ctrl->pid_angle, 0.0f, -angle_error, dt);
    // 取反: 与双环PID保持一致的符号约定
    wheel_speed_target = -wheel_speed_target;
    
    // ===== 角速度阻尼 (参考 LQR gyro_control) =====
    // pid_gyro(0, pitch_rate): 抑制角速度，提供阻尼
    float gyro_control = pid_compute(&ctrl->pid_gyro, 0.0f, pitch_rate, dt);
    wheel_speed_target += gyro_control;  // 叠加到角度环输出 (wheel_speed_target)
    
    output->angle_error = angle_error;
    output->wheel_speed_target = wheel_speed_target;
    output->angle_p_out = ctrl->params.angle_kp * (-angle_error);
    output->angle_i_out = ctrl->pid_angle.integral;
    output->angle_d_out = ctrl->pid_angle.prev_d_term;
    output->gyro_p_out = ctrl->params.gyro_kp * (0.0f - pitch_rate);
    output->gyro_i_out = ctrl->pid_gyro.integral;
    output->gyro_d_out = ctrl->pid_gyro.prev_d_term;
    output->gyro_control = gyro_control;
    
    // =============================================================
    // 第三环: 轮速环 (内环)
    //   根据 wheel_mode 选择:
    //   WHEEL_SPEED:  直接输出 wheel_speed_target (送电机速度模式)
    //   WHEEL_TORQUE: wheel_speed_target - wheel_speed → torque (送电机扭矩模式)
    // =============================================================
    if (ctrl->params.wheel_mode == TRIPLE_PID_WHEEL_TORQUE) {
        // 软件PID轮速环
        float wheel_error = wheel_speed_target - wheel_speed;
        float torque = pid_compute(&ctrl->pid_wheel, 0.0f, -wheel_error, dt);
        torque = clamp_f(-torque, -ctrl->params.max_torque, ctrl->params.max_torque);
        
        output->wheel_speed_error = wheel_error;
        output->torque = torque;
        output->wheel_p_out = ctrl->params.wheel_kp * wheel_error;
        output->wheel_i_out = ctrl->pid_wheel.integral;
        output->wheel_d_out = ctrl->pid_wheel.prev_d_term;
    } else {
        // 电机速度模式: 直接输出 wheel_speed_target
        // torque 字段存储的是速度 (rad/s)，后续由 apply_motor_commands 处理
        output->wheel_speed_error = 0.0f;
        output->torque = wheel_speed_target;
        output->wheel_p_out = 0.0f;
        output->wheel_i_out = 0.0f;
        output->wheel_d_out = 0.0f;
    }
    
    output->emergency = false;
    
    return ESP_OK;
}
