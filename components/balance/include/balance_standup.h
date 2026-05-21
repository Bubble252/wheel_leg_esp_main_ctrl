#pragma once

/**
 * @brief 起身相关状态机
 *
 * 包含三个状态机:
 * 1. standup: 从趴着站起来 (IDLE → RETRACT → LEFT_ROLL → LEFT_EXTEND → WAIT → RIGHT_ROLL → DONE)
 * 2. car_standup: 从小车模式站起来进入平衡 (IDLE → SWING → DONE)
 * 3. bal_to_car: 从平衡模式过渡到小车 (IDLE → RETRACT → TILT → SETTLE)
 */

#include <stdbool.h>
#include <stdint.h>
#include "balance_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 起身状态枚举
// ============================================================================

typedef enum {
    STANDUP_IDLE = 0,           // 空闲 (等待指令)
    STANDUP_RETRACT,            // 收腿 (双腿缩到最短, 保持向后)
    STANDUP_LEFT_ROLL,          // 左腿翻转 (左hip MIT + 左轮慢转)
    STANDUP_LEFT_EXTEND,        // 左腿伸长 (hip位置闭环, 伸长腿长)
    STANDUP_WAIT,               // 等待 (左腿伸长后稳定一段时间)
    STANDUP_RIGHT_ROLL,         // 右腿翻转 (右hip MIT + 右轮慢转)
    STANDUP_DONE,               // 完成, 进入小车模式
} standup_state_t;

typedef enum {
    CAR_STANDUP_IDLE = 0,       // 空闲
    CAR_STANDUP_SWING,          // 腿从 -130° 摆向 -90°
    CAR_STANDUP_DONE,           // 到达目标, 等待切入平衡
} car_standup_state_t;

typedef enum {
    BAL_TO_CAR_IDLE = 0,        // 空闲 (直接切换)
    BAL_TO_CAR_RETRACT,         // 步骤1: 收腿 (缩短腿长, 平衡控制仍运行)
    BAL_TO_CAR_TILT,            // 步骤2: 退出平衡, 两轮低速前转 → 机身后倾
    BAL_TO_CAR_SETTLE,          // 步骤3: 等待机身后倾稳定后进入 car 模式
} bal_to_car_state_t;

// ============================================================================
// 外部变量声明 (用于 apply_leg_motor_commands)
// ============================================================================

extern bool g_standup_mit_active;            // MIT 模式激活标志
extern bool g_standup_mit_left_hip;          // 左 hip MIT 激活
extern bool g_standup_mit_right_hip;         // 右 hip MIT 激活
extern float g_standup_ff_sign_left;         // 左 hip 前馈力矩方向
extern float g_standup_ff_sign_right;        // 右 hip 前馈力矩方向
extern float g_standup_mit_kp;               // MIT 位置刚度
extern float g_standup_mit_kd;               // MIT 速度阻尼
extern float g_standup_mit_target_left_hip;  // 左 hip MIT 目标位置 (rad)
extern float g_standup_mit_target_right_hip; // 右 hip MIT 目标位置 (rad)
extern float g_standup_mit_ff_torque;        // MIT 前馈力矩 (Nm)

// ============================================================================
// 状态变量声明 (供 WiFi handler 触发使用)
// ============================================================================

extern standup_state_t g_standup_state;
extern uint32_t g_standup_state_enter_ms;
extern bool g_standup_last_btn;

extern car_standup_state_t g_car_standup_state;
extern uint32_t g_car_standup_enter_ms;
extern bool g_car_standup_last_btn;

extern bal_to_car_state_t g_bal_to_car_state;
extern uint32_t g_bal_to_car_enter_ms;

// 起身参数 (可调)
extern float g_standup_retract_length;
extern float g_standup_retract_angle;
extern float g_standup_zero_threshold;

// ============================================================================
// API
// ============================================================================

/**
 * @brief 起身状态机是否正在执行 (非 IDLE)
 */
bool standup_is_active(void);

/**
 * @brief 小车起身状态机是否正在执行 (非 IDLE)
 */
bool car_standup_is_active(void);

/**
 * @brief 平衡→小车过渡状态机是否正在执行 (非 IDLE)
 */
bool bal_to_car_is_active(void);

/**
 * @brief 起身状态机更新 (每帧调用)
 */
void standup_state_machine_update(void);

/**
 * @brief 小车起身状态机更新 (每帧调用)
 */
void car_standup_state_machine_update(void);

/**
 * @brief 平衡→小车过渡状态机更新 (每帧调用)
 */
void bal_to_car_state_machine_update(void);

#ifdef __cplusplus
}
#endif