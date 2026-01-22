/**
 * @file pid_controller.h
 * @brief PID 控制器
 * @author Bubble
 * @date 2026-01-15
 */

#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief PID 控制器结构体
 */
typedef struct {
    // 参数
    float kp;           // 比例增益
    float ki;           // 积分增益
    float kd;           // 微分增益
    
    // 限幅
    float output_min;   // 输出下限
    float output_max;   // 输出上限
    float integral_max; // 积分限幅
    
    // 内部状态
    float integral;     // 积分累积
    float prev_error;   // 上次误差
    float prev_output;  // 上次输出
    bool first_run;     // 首次运行标志
    
    // 滤波
    float d_filter_coef; // 微分项滤波系数 (0-1)
    float prev_d_term;   // 上次微分项
} pid_controller_t;

/**
 * @brief PID 参数结构
 */
typedef struct {
    float kp;
    float ki;
    float kd;
    float output_min;
    float output_max;
    float integral_max;
    float d_filter_coef;
} pid_params_t;

/**
 * @brief 初始化 PID 控制器
 * @param pid 控制器实例
 * @param params 参数
 */
void pid_init(pid_controller_t *pid, const pid_params_t *params);

/**
 * @brief 重置 PID 控制器状态
 * @param pid 控制器实例
 */
void pid_reset(pid_controller_t *pid);

/**
 * @brief 设置 PID 参数
 * @param pid 控制器实例
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 */
void pid_set_gains(pid_controller_t *pid, float kp, float ki, float kd);

/**
 * @brief 设置输出限幅
 * @param pid 控制器实例
 * @param min 最小值
 * @param max 最大值
 */
void pid_set_output_limits(pid_controller_t *pid, float min, float max);

/**
 * @brief 计算 PID 输出
 * @param pid 控制器实例
 * @param setpoint 目标值
 * @param measurement 测量值
 * @param dt 时间步长 (秒)
 * @return PID 输出
 */
float pid_compute(pid_controller_t *pid, float setpoint, float measurement, float dt);

/**
 * @brief 增量式 PID 计算
 * @param pid 控制器实例
 * @param setpoint 目标值
 * @param measurement 测量值
 * @param dt 时间步长 (秒)
 * @return PID 输出增量
 */
float pid_compute_incremental(pid_controller_t *pid, float setpoint, float measurement, float dt);

#ifdef __cplusplus
}
#endif

#endif // PID_CONTROLLER_H
