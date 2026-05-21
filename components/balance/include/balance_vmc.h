#pragma once

/**
 * @brief VMC (Virtual Model Control) 力控模块
 *
 * 包含:
 * - vmc_compute_leg_state(): VMC 腿部状态计算
 * - compute_support_force(): 支持力估计
 * - 离地检测
 */

#include <stdbool.h>
#include "balance_types.h"
#include "leg_kinematics.h"
#include "lqr_balance.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// VMC 状态变量 (extern 供 balance_test.c 访问)
// ============================================================================

extern bool g_vmc_enabled;
extern vmc_params_t g_vmc_params;
extern float g_vmc_target_vx;
extern float g_vmc_target_y;
extern vmc_dual_output_t g_vmc_dual_output;
extern bool g_vmc_input_valid;
extern bool g_vmc_stream_enable;

// 支持力估计
extern float g_support_force_left_FL;
extern float g_support_force_right_FL;
extern float g_support_force_left_Fa;
extern float g_support_force_right_Fa;
extern bool g_sforce_stream_enable;

// 离地检测
extern bool g_sforce_left_off;
extern bool g_sforce_right_off;
extern int g_sforce_left_off_cnt;
extern int g_sforce_right_off_cnt;

// 离地检测参数
#define SFORCE_FL_THRESHOLD   1.0f
#define SFORCE_OFF_ENTER_CNT  5
#define SFORCE_OFF_EXIT_CNT   10

// ============================================================================
// API
// ============================================================================

/**
 * @brief 计算 VMC 输入状态和输出扭矩
 * @param lqr_input LQR 输入数据 (用于 pitch/gyro)
 */
void vmc_compute_leg_state(const lqr_input_t *lqr_input);

/**
 * @brief 支持力估计 (任意模式下均可调用)
 */
void compute_support_force(void);

#ifdef __cplusplus
}
#endif
