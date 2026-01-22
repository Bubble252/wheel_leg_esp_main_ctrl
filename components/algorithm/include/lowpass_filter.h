/**
 * @file lowpass_filter.h
 * @brief 低通滤波器
 * @author Bubble
 * @date 2026-01-15
 * @note 参考 SimpleFOC 的低通滤波器实现
 */

#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

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

#ifdef __cplusplus
}
#endif

#endif // LOWPASS_FILTER_H
