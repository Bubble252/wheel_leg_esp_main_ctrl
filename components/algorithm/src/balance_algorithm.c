/**
 * @file balance_algorithm.c
 * @brief 平衡算法实现
 * @author Bubble
 * @date 2026-01-15
 */

#include "balance_algorithm.h"
#include "pid_controller.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "BALANCE";

// 紧急停止角度阈值
#define EMERGENCY_ANGLE_THRESHOLD 45.0f

// PID 控制器实例
static pid_controller_t g_balance_pid;  // 平衡环 (俯仰角 -> 速度)
static pid_controller_t g_speed_pid;    // 速度环 (速度差 -> 角度补偿)
static pid_controller_t g_turn_pid;     // 转向环 (偏航 -> 差速)

// 当前参数
static balance_params_t g_params;
static bool g_initialized = false;

// 默认参数
static const balance_params_t default_params = {
    .balance_kp = 50.0f,
    .balance_ki = 0.5f,
    .balance_kd = 2.0f,
    .speed_kp = 0.5f,
    .speed_ki = 0.01f,
    .turn_kp = 10.0f,
    .turn_kd = 0.5f,
    .target_angle = 0.0f,
    .max_wheel_speed = 500.0f,
};

// 限幅函数
static inline float clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

esp_err_t balance_init(const balance_params_t *params) {
    if (g_initialized) {
        return ESP_OK;
    }
    
    // 使用提供的参数或默认参数
    if (params != NULL) {
        memcpy(&g_params, params, sizeof(balance_params_t));
    } else {
        memcpy(&g_params, &default_params, sizeof(balance_params_t));
    }
    
    // 初始化平衡环 PID
    pid_params_t balance_pid_params = {
        .kp = g_params.balance_kp,
        .ki = g_params.balance_ki,
        .kd = g_params.balance_kd,
        .output_min = -g_params.max_wheel_speed,
        .output_max = g_params.max_wheel_speed,
        .integral_max = 50.0f,
        .d_filter_coef = 0.2f,
    };
    pid_init(&g_balance_pid, &balance_pid_params);
    
    // 初始化速度环 PID
    pid_params_t speed_pid_params = {
        .kp = g_params.speed_kp,
        .ki = g_params.speed_ki,
        .kd = 0.0f,
        .output_min = -10.0f,  // 角度补偿范围
        .output_max = 10.0f,
        .integral_max = 20.0f,
        .d_filter_coef = 0.1f,
    };
    pid_init(&g_speed_pid, &speed_pid_params);
    
    // 初始化转向环 PID
    pid_params_t turn_pid_params = {
        .kp = g_params.turn_kp,
        .ki = 0.0f,
        .kd = g_params.turn_kd,
        .output_min = -200.0f,  // 差速范围
        .output_max = 200.0f,
        .integral_max = 0.0f,
        .d_filter_coef = 0.1f,
    };
    pid_init(&g_turn_pid, &turn_pid_params);
    
    g_initialized = true;
    ESP_LOGI(TAG, "Balance algorithm initialized");
    
    return ESP_OK;
}

void balance_reset(void) {
    pid_reset(&g_balance_pid);
    pid_reset(&g_speed_pid);
    pid_reset(&g_turn_pid);
    ESP_LOGI(TAG, "Balance algorithm reset");
}

void balance_set_params(const balance_params_t *params) {
    if (params == NULL) return;
    
    memcpy(&g_params, params, sizeof(balance_params_t));
    
    // 更新 PID 参数
    pid_set_gains(&g_balance_pid, g_params.balance_kp, g_params.balance_ki, g_params.balance_kd);
    pid_set_output_limits(&g_balance_pid, -g_params.max_wheel_speed, g_params.max_wheel_speed);
    
    pid_set_gains(&g_speed_pid, g_params.speed_kp, g_params.speed_ki, 0.0f);
    
    pid_set_gains(&g_turn_pid, g_params.turn_kp, 0.0f, g_params.turn_kd);
}

void balance_get_params(balance_params_t *params) {
    if (params != NULL) {
        memcpy(params, &g_params, sizeof(balance_params_t));
    }
}

esp_err_t balance_compute(const balance_input_t *input, balance_output_t *output, float dt) {
    if (!g_initialized || input == NULL || output == NULL || dt <= 0.0f) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 清零输出
    memset(output, 0, sizeof(balance_output_t));
    
    // 检查紧急停止
    if (balance_check_emergency(input->pitch)) {
        // 限流打印：每秒最多打印一次
        static uint32_t last_emergency_log = 0;
        uint32_t now = (uint32_t)(esp_log_timestamp());
        if (now - last_emergency_log > 1000) {
            ESP_LOGW(TAG, "Emergency stop! Pitch: %.2f", input->pitch);
            last_emergency_log = now;
        }
        return ESP_ERR_INVALID_STATE;
    }
    
    // ===== 速度环 =====
    // 根据目标速度和当前速度计算角度补偿
    float current_speed = (input->left_wheel_speed + input->right_wheel_speed) / 2.0f;
    float angle_compensation = pid_compute(&g_speed_pid, input->target_speed, current_speed, dt);
    
    // ===== 平衡环 =====
    // 目标角度 = 基础角度 + 速度环补偿
    float target_angle = g_params.target_angle + angle_compensation;
    
    // 计算平衡控制输出 (轮子速度)
    float balance_output = pid_compute(&g_balance_pid, target_angle, input->pitch, dt);
    
    // ===== 转向环 =====
    // 根据目标偏航角速度计算差速
    float turn_output = pid_compute(&g_turn_pid, input->target_yaw_rate, input->yaw_rate, dt);
    
    // ===== 合成输出 =====
    output->left_wheel_speed = clamp(balance_output + turn_output, 
                                     -g_params.max_wheel_speed, g_params.max_wheel_speed);
    output->right_wheel_speed = clamp(balance_output - turn_output, 
                                      -g_params.max_wheel_speed, g_params.max_wheel_speed);
    
    // 腿部扭矩暂时为0 (后续实现 VMC)
    output->left_hip_torque = 0.0f;
    output->right_hip_torque = 0.0f;
    output->left_knee_torque = 0.0f;
    output->right_knee_torque = 0.0f;
    
    return ESP_OK;
}

bool balance_check_emergency(float pitch) {
    return fabsf(pitch) > EMERGENCY_ANGLE_THRESHOLD;
}
