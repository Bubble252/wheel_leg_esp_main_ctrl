/**
 * @file pid_controller.c
 * @brief PID 控制器实现
 * @author Bubble
 * @date 2026-01-15
 */

#include "pid_controller.h"
#include <string.h>
#include <math.h>

// 限幅函数
static inline float clamp(float value, float min, float max) {
    if (value < min) return min;
    if (value > max) return max;
    return value;
}

void pid_init(pid_controller_t *pid, const pid_params_t *params) {
    if (pid == NULL) return;
    
    memset(pid, 0, sizeof(pid_controller_t));
    
    if (params != NULL) {
        pid->kp = params->kp;
        pid->ki = params->ki;
        pid->kd = params->kd;
        pid->output_min = params->output_min;
        pid->output_max = params->output_max;
        pid->integral_max = params->integral_max;
        pid->d_filter_coef = params->d_filter_coef;
    } else {
        // 默认值
        pid->kp = 1.0f;
        pid->ki = 0.0f;
        pid->kd = 0.0f;
        pid->output_min = -1000.0f;
        pid->output_max = 1000.0f;
        pid->integral_max = 100.0f;
        pid->d_filter_coef = 0.1f;
    }
    
    pid->first_run = true;
}

void pid_reset(pid_controller_t *pid) {
    if (pid == NULL) return;
    
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_output = 0.0f;
    pid->prev_d_term = 0.0f;
    pid->first_run = true;
}

void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd) {
    if (pid == NULL) return;
    
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
}

void pid_set_output_limits(pid_controller_t *pid, float min, float max) {
    if (pid == NULL) return;
    
    pid->output_min = min;
    pid->output_max = max;
}

float pid_compute(pid_controller_t *pid, float setpoint, float measurement, float dt) {
    if (pid == NULL || dt <= 0.0f) return 0.0f;
    
    // 计算误差
    float error = setpoint - measurement;
    
    // 比例项
    float p_term = pid->kp * error;
    
    // 积分项
    pid->integral += error * dt;
    pid->integral = clamp(pid->integral, -pid->integral_max, pid->integral_max);
    float i_term = pid->ki * pid->integral;
    
    // 微分项 (带滤波)
    float d_term = 0.0f;
    if (!pid->first_run) {
        float raw_d = (error - pid->prev_error) / dt;
        // 低通滤波
        d_term = pid->prev_d_term + pid->d_filter_coef * (pid->kd * raw_d - pid->prev_d_term);
        pid->prev_d_term = d_term;
    }
    
    // 总输出
    float output = p_term + i_term + d_term;
    
    // 输出限幅
    output = clamp(output, pid->output_min, pid->output_max);
    
    // 抗积分饱和
    if (output >= pid->output_max || output <= pid->output_min) {
        // 如果输出饱和，停止积分累积
        pid->integral -= error * dt;
    }
    
    // 更新状态
    pid->prev_error = error;
    pid->prev_output = output;
    pid->first_run = false;
    
    return output;
}

float pid_compute_incremental(pid_controller_t *pid, float setpoint, float measurement, float dt) {
    if (pid == NULL || dt <= 0.0f) return 0.0f;
    
    float error = setpoint - measurement;
    
    float delta_output = 0.0f;
    
    if (!pid->first_run) {
        // 增量式 PID
        float d_error = error - pid->prev_error;
        delta_output = pid->kp * d_error + 
                       pid->ki * error * dt + 
                       pid->kd * d_error / dt;
    }
    
    // 更新输出
    float output = pid->prev_output + delta_output;
    output = clamp(output, pid->output_min, pid->output_max);
    
    // 更新状态
    pid->prev_error = error;
    pid->prev_output = output;
    pid->first_run = false;
    
    return output;
}
