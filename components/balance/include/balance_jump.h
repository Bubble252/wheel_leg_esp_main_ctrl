#pragma once

/**
 * @brief 跳跃状态机
 *
 * 状态流: IDLE → CROUCH → EXTEND → AIR_RETRACT → IDLE
 *
 * IDLE:     等待指令
 * CROUCH:   蹲下蓄力 (腿长→68mm)
 * EXTEND:   蹬伸起跳 (腿长→110mm, MIT 阻抗控制)
 * AIR_RETRACT: 空中收腿 (腿长→68mm, 轮速=0)
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 跳跃状态枚举
// ============================================================================

typedef enum {
    JUMP_IDLE = 0,          // 空闲 (等待指令)
    JUMP_CROUCH,            // 蹲下蓄力 (腿长→68mm)
    JUMP_EXTEND,            // 蹬伸起跳 (腿长→110mm)
    JUMP_AIR_RETRACT,       // 空中收腿 (腿长→68mm, 轮速=0)
    JUMP_RECOVER,           // 着地恢复 (不再使用)
} jump_state_t;

// ============================================================================
// API
// ============================================================================

/**
 * @brief 跳跃状态机更新 (每帧调用)
 */
void jump_state_machine_update(void);

/**
 * @brief 跳跃状态机是否正在执行 (非 IDLE)
 */
bool jump_is_active(void);

/**
 * @brief 跳跃状态机是否要求轮速为零
 * @return true: 空中阶段, 轮速应强制为0
 */
bool jump_wants_zero_wheel(void);

/**
 * @brief MIT 蹬伸模式是否激活 (供电机控制任务使用)
 */
bool jump_mit_is_active(void);

/**
 * @brief 触发跳跃序列 (从 CLI/遥控调用)
 * @param leg_length 当前腿长 (将保存用于恢复)
 * @param base_angle 当前身体夹角 (将保存用于恢复)
 * @return true 成功触发, false 跳跃已在进行中
 */
bool jump_trigger(float leg_length, float base_angle);

/**
 * @brief 设置跳跃按钮状态 (上升沿检测用)
 * @param pressed 当前按钮状态
 */
void jump_set_btn(bool pressed);

#ifdef __cplusplus
}
#endif