#pragma once

/**
 * @brief 核心控制算法模块
 *
 * 包含:
 * - compute_balance_output(): 平衡控制主算法
 * - apply_motor_commands(): 轮电机命令发送
 */

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 平衡控制主算法 - 根据控制模式计算输出
 * @param dt 控制周期 (秒)
 */
void compute_balance_output(float dt);

/**
 * @brief 发送轮电机命令
 */
void apply_motor_commands(void);

#ifdef __cplusplus
}
#endif
