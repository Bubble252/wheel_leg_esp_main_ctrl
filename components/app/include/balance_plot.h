#pragma once

/**
 * @brief 波形输出 + PID 调试模块
 *
 * 包含:
 * - output_plot_data(): 波形数据输出 (#DATA 格式)
 * - output_pid_debug(): PID 调试输出
 * - balance_test_set_plot/get: 波形开关控制
 */

#include <stdbool.h>
#include <stdint.h>
#include "lqr_balance.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// Plot 通道掩码操作
// ============================================================================

static inline uint32_t plot_ch_bit(char ch) {
    if (ch >= 'A' && ch <= 'Z') return 1U << (ch - 'A');
    return 0;
}

// g_plot_channel_mask 在 balance_test.c 中定义, 需要 extern 声明
extern uint32_t g_plot_channel_mask;

#define PLOT_CH_ENABLED(ch) (g_plot_channel_mask & plot_ch_bit(ch))

// ============================================================================
// Plot 变量 (extern 供 CLI 和控制循环访问)
// ============================================================================

extern bool g_plot_enabled;
extern uint8_t g_plot_divider;
extern uint8_t g_plot_counter;
extern float g_last_lqr_u;

extern bool g_pid_debug_enabled;
extern uint8_t g_pid_debug_divider;
extern uint8_t g_pid_debug_counter;

// ============================================================================
// API
// ============================================================================

void balance_test_set_plot(bool enable);
void balance_test_set_plot_divider(uint8_t divider);
bool balance_test_get_plot_enabled(void);

void output_plot_data(const lqr_input_t *input, const lqr_output_t *output);
void output_pid_debug(const lqr_input_t *input);

#ifdef __cplusplus
}
#endif
