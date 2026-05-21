#pragma once

/**
 * @brief 速度/位移观测器 (2x2 卡尔曼滤波)
 *
 * 融合编码器速度和 IMU 加速度, 输出滤波速度和积分位移。
 * 独立任务运行 (默认 333Hz)。
 */

#include <stdbool.h>
#include <stdint.h>
#include "balance_types.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// API
// ============================================================================

/**
 * @brief 观测器任务入口 (由 balance_test_start 创建任务)
 */
void task_observer(void *arg);

/**
 * @brief 速度观测器主函数 (在 observer task 中每帧调用)
 */
void velocity_observer_update(float dt, const shared_imu_data_t *imu,
                              float left_vel_rad, float right_vel_rad);

/**
 * @brief 重置观测器状态
 */
void kf_observer_reset(void);

// ============================================================================
// 观测器输出 (extern 供 balance_test.c 读取)
// ============================================================================

extern float g_obsv_v_encoder;     // 运动学补偿后的编码器速度 (m/s)
extern float g_obsv_v_filter;      // 卡尔曼滤波后的速度 (m/s)
extern float g_obsv_x_filter;      // 滤波速度积分位移 (m)
extern float g_obsv_a_imu;         // IMU 前进方向加速度 (m/s²)
extern float g_obsv_wheel_v_raw;   // 原始轮速 (无补偿) (m/s)

// 观测器参数 (可 CLI 调参)
extern bool g_observer_enabled;
extern bool g_tpid_use_observer_speed;
extern bool g_obsv_stream_enable;
extern int g_obsv_period_ms;

// KF 参数 (可在线调参)
extern float g_kf_Q_v;
extern float g_kf_Q_a;
extern float g_kf_R_v;
extern float g_kf_R_a;

#ifdef __cplusplus
}
#endif
