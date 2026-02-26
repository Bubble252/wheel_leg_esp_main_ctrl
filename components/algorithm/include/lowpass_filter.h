/**
 * @file lowpass_filter.h
 * @brief 低通滤波器
 * @author Bubble
 * @date 2026-01-15
 * @note 参考 SimpleFOC 的低通滤波器实现
 */

#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 低通滤波器结构体
 */
typedef struct {
    float tf;           // 滤波时间常数 (秒)
    float y_prev;       // 上一次输出值
    unsigned long timestamp_prev;  // 上一次时间戳 (微秒)
} lowpass_filter_t;

/**
 * @brief 限幅滤波器结构体 (slew-rate limiter)
 * @note 限制信号每秒最大变化量，超过阈值则钳位，否则直通 (零延迟)
 */
typedef struct {
    float max_rate;     // 最大变化率 (单位/秒)
    float y_prev;       // 上一次输出值
    bool first_run;     // 首次运行标志
} slewrate_filter_t;

/**
 * @brief 加权滑动平均滤波器 (Weighted Moving Average)
 * @note 12 点窗口，线性递增权重 (1,2,3,...,12)，最新采样权重最大
 *       群延迟约 3.15 个采样 (~6.3 ms @500Hz)，-3dB 截止约 55Hz
 */
#define WMA_WINDOW_SIZE 12
typedef struct {
    float buf[WMA_WINDOW_SIZE];   // 环形缓冲区
    int   head;                    // 下一个写入位置
    int   count;                   // 已填充的采样数 (0~WMA_WINDOW_SIZE)
} weighted_ma_filter_t;

/**
 * @brief 初始化低通滤波器
 * @param lpf 滤波器实例
 * @param tf 滤波时间常数 (秒)，越大滤波越强
 */
void lpf_init(lowpass_filter_t *lpf, float tf);

/**
 * @brief 重置低通滤波器
 * @param lpf 滤波器实例
 */
void lpf_reset(lowpass_filter_t *lpf);

/**
 * @brief 设置滤波时间常数
 * @param lpf 滤波器实例
 * @param tf 新的时间常数
 */
void lpf_set_tf(lowpass_filter_t *lpf, float tf);

/**
 * @brief 滤波计算 (使用系统时间)
 * @param lpf 滤波器实例
 * @param x 输入值
 * @return 滤波后的输出值
 */
float lpf_compute(lowpass_filter_t *lpf, float x);

/**
 * @brief 滤波计算 (使用指定时间步长)
 * @param lpf 滤波器实例
 * @param x 输入值
 * @param dt 时间步长 (秒)
 * @return 滤波后的输出值
 */
float lpf_compute_dt(lowpass_filter_t *lpf, float x, float dt);

/**
 * @brief 初始化限幅滤波器
 * @param sf 滤波器实例
 * @param max_rate 最大变化率 (单位/秒)，0 表示不限制 (直通)
 */
void slewrate_init(slewrate_filter_t *sf, float max_rate);

/**
 * @brief 重置限幅滤波器
 * @param sf 滤波器实例
 */
void slewrate_reset(slewrate_filter_t *sf);

/**
 * @brief 设置最大变化率
 * @param sf 滤波器实例
 * @param max_rate 新的最大变化率 (单位/秒)
 */
void slewrate_set_max_rate(slewrate_filter_t *sf, float max_rate);

/**
 * @brief 限幅滤波计算
 * @param sf 滤波器实例
 * @param x 输入值
 * @param dt 时间步长 (秒)
 * @return 滤波后的输出值
 * @note 若 |x - y_prev| <= max_rate * dt，则直通 (零延迟)
 *       否则钳位到 y_prev ± max_rate * dt
 */
float slewrate_compute_dt(slewrate_filter_t *sf, float x, float dt);

// ============================================================================
// 加权滑动平均滤波器 (Weighted Moving Average, 12-point)
// ============================================================================

/**
 * @brief 初始化加权滑动平均滤波器
 * @param wma 滤波器实例
 */
void wma_init(weighted_ma_filter_t *wma);

/**
 * @brief 重置加权滑动平均滤波器
 * @param wma 滤波器实例
 */
void wma_reset(weighted_ma_filter_t *wma);

/**
 * @brief 加权滑动平均滤波计算
 * @param wma 滤波器实例
 * @param x 输入值
 * @return 滤波后的输出值
 * @note 权重线性递增: w[0]=1, w[1]=2, ..., w[11]=12
 *       output = Σ(w[i]*x[i]) / Σ(w[i])，Σw = 78
 */
float wma_compute(weighted_ma_filter_t *wma, float x);

#ifdef __cplusplus
}
#endif

#endif // LOWPASS_FILTER_H
