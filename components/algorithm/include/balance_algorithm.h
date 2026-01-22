/**
 * @file balance_algorithm.h
 * @brief 平衡算法接口
 * @author Bubble
 * @date 2026-01-15
 */

#ifndef BALANCE_ALGORITHM_H
#define BALANCE_ALGORITHM_H

#include <stdint.h>
#include <stdbool.h>
#include "types.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 平衡控制输出
 */
typedef struct {
    float left_wheel_speed;   // 左轮速度 (rpm)
    float right_wheel_speed;  // 右轮速度 (rpm)
    float left_hip_torque;    // 左髋扭矩
    float right_hip_torque;   // 右髋扭矩
    float left_knee_torque;   // 左膝扭矩
    float right_knee_torque;  // 右膝扭矩
} balance_output_t;

/**
 * @brief 平衡控制输入
 */
typedef struct {
    // IMU 数据
    float pitch;          // 俯仰角 (度)
    float pitch_rate;     // 俯仰角速度 (度/秒)
    float roll;           // 滚转角 (度)
    float roll_rate;      // 滚转角速度 (度/秒)
    float yaw_rate;       // 偏航角速度 (度/秒)
    
    // 遥控输入
    float target_speed;   // 目标速度 (m/s)
    float target_yaw_rate; // 目标偏航角速度 (度/秒)
    float target_height;  // 目标高度 (比例 0-1)
    
    // 电机反馈
    float left_wheel_speed;  // 左轮当前速度
    float right_wheel_speed; // 右轮当前速度
} balance_input_t;

/**
 * @brief 平衡参数结构
 */
typedef struct {
    // 平衡环参数
    float balance_kp;
    float balance_ki;
    float balance_kd;
    
    // 速度环参数
    float speed_kp;
    float speed_ki;
    
    // 转向环参数
    float turn_kp;
    float turn_kd;
    
    // 目标平衡角度
    float target_angle;
    
    // 输出限制
    float max_wheel_speed;
} balance_params_t;

/**
 * @brief 初始化平衡算法
 * @param params 参数 (NULL 使用默认值)
 * @return ESP_OK 成功
 */
esp_err_t balance_init(const balance_params_t *params);

/**
 * @brief 重置平衡算法状态
 */
void balance_reset(void);

/**
 * @brief 设置平衡参数
 * @param params 参数
 */
void balance_set_params(const balance_params_t *params);

/**
 * @brief 获取当前参数
 * @param params 输出参数
 */
void balance_get_params(balance_params_t *params);

/**
 * @brief 计算平衡控制输出
 * @param input 输入数据
 * @param output 输出数据
 * @param dt 时间步长 (秒)
 * @return ESP_OK 成功
 */
esp_err_t balance_compute(const balance_input_t *input, balance_output_t *output, float dt);

/**
 * @brief 检查是否需要紧急停止
 * @param pitch 当前俯仰角
 * @return true 需要紧急停止
 */
bool balance_check_emergency(float pitch);

#ifdef __cplusplus
}
#endif

#endif // BALANCE_ALGORITHM_H
