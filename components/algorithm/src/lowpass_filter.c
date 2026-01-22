/**
 * @file lowpass_filter.c
 * @brief 低通滤波器实现
 * @author Bubble
 * @date 2026-01-15
 */

#include "lowpass_filter.h"
#include "esp_timer.h"
#include <stddef.h>

void lpf_init(lowpass_filter_t *lpf, float tf) {
    if (lpf == NULL) return;
    
    lpf->tf = tf;
    lpf->y_prev = 0.0f;
    lpf->timestamp_prev = 0;
}

void lpf_reset(lowpass_filter_t *lpf) {
    if (lpf == NULL) return;
    
    lpf->y_prev = 0.0f;
    lpf->timestamp_prev = 0;
}

void lpf_set_tf(lowpass_filter_t *lpf, float tf) {
    if (lpf == NULL) return;
    lpf->tf = tf;
}

float lpf_compute(lowpass_filter_t *lpf, float x) {
    if (lpf == NULL) return x;
    
    unsigned long timestamp = (unsigned long)esp_timer_get_time();
    float dt = (timestamp - lpf->timestamp_prev) * 1e-6f;  // 微秒转秒
    
    // 首次运行或时间间隔异常
    if (lpf->timestamp_prev == 0 || dt <= 0.0f || dt > 0.5f) {
        lpf->y_prev = x;
        lpf->timestamp_prev = timestamp;
        return x;
    }
    
    lpf->timestamp_prev = timestamp;
    return lpf_compute_dt(lpf, x, dt);
}

float lpf_compute_dt(lowpass_filter_t *lpf, float x, float dt) {
    if (lpf == NULL) return x;
    
    // 时间步长异常检查
    if (dt <= 0.0f || dt > 0.5f) {
        lpf->y_prev = x;
        return x;
    }
    
    // 时间常数为0时不滤波
    if (lpf->tf <= 0.0f) {
        lpf->y_prev = x;
        return x;
    }
    
    // 一阶低通滤波公式:
    // alpha = dt / (tf + dt)
    // y = alpha * x + (1 - alpha) * y_prev
    float alpha = dt / (lpf->tf + dt);
    float y = alpha * x + (1.0f - alpha) * lpf->y_prev;
    
    lpf->y_prev = y;
    return y;
}
