/**
 * @file lowpass_filter.c
 * @brief 低通滤波器实现
 * @author Bubble
 * @date 2026-01-15
 */

#include "lowpass_filter.h"
#include "esp_timer.h"
#include <stddef.h>
#include <math.h>

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

// ============================================================================
// 限幅滤波器 (Slew-Rate Limiter)
// ============================================================================

void slewrate_init(slewrate_filter_t *sf, float max_rate) {
    if (sf == NULL) return;
    sf->max_rate = max_rate;
    sf->y_prev = 0.0f;
    sf->first_run = true;
}

void slewrate_reset(slewrate_filter_t *sf) {
    if (sf == NULL) return;
    sf->y_prev = 0.0f;
    sf->first_run = true;
}

void slewrate_set_max_rate(slewrate_filter_t *sf, float max_rate) {
    if (sf == NULL) return;
    sf->max_rate = max_rate;
}

float slewrate_compute_dt(slewrate_filter_t *sf, float x, float dt) {
    if (sf == NULL) return x;

    // 首次运行直接输出
    if (sf->first_run) {
        sf->y_prev = x;
        sf->first_run = false;
        return x;
    }

    // max_rate <= 0 表示不限制 (直通)
    if (sf->max_rate <= 0.0f || dt <= 0.0f) {
        sf->y_prev = x;
        return x;
    }

    float delta = x - sf->y_prev;
    float max_delta = sf->max_rate * dt;

    if (fabsf(delta) <= max_delta) {
        // 变化量在阈值内，直通 (零延迟)
        sf->y_prev = x;
        return x;
    } else {
        // 超过阈值，钳位
        float y = sf->y_prev + ((delta > 0.0f) ? max_delta : -max_delta);
        sf->y_prev = y;
        return y;
    }
}

// ============================================================================
// 加权滑动平均滤波器 (Weighted Moving Average, 12-point)
// 权重: w[oldest]=1, w[oldest+1]=2, ..., w[newest]=12
// Σw = 12*13/2 = 78
// ============================================================================

void wma_init(weighted_ma_filter_t *wma) {
    if (wma == NULL) return;
    for (int i = 0; i < WMA_WINDOW_SIZE; i++) {
        wma->buf[i] = 0.0f;
    }
    wma->head = 0;
    wma->count = 0;
}

void wma_reset(weighted_ma_filter_t *wma) {
    wma_init(wma);
}

float wma_compute(weighted_ma_filter_t *wma, float x) {
    if (wma == NULL) return x;
    
    // 写入环形缓冲区
    wma->buf[wma->head] = x;
    wma->head = (wma->head + 1) % WMA_WINDOW_SIZE;
    if (wma->count < WMA_WINDOW_SIZE) {
        wma->count++;
    }
    
    // 窗口未填满时直通 (避免启动阶段失真)
    if (wma->count < WMA_WINDOW_SIZE) {
        return x;
    }
    
    // 计算加权平均
    // buf[head] 是最老的采样 (即将被覆盖的位置), 权重=1
    // buf[head-1] 是最新的采样, 权重=12
    float weighted_sum = 0.0f;
    int idx = wma->head;  // 最老的采样
    for (int w = 1; w <= WMA_WINDOW_SIZE; w++) {
        weighted_sum += (float)w * wma->buf[idx];
        idx = (idx + 1) % WMA_WINDOW_SIZE;
    }
    
    // Σw = WMA_WINDOW_SIZE * (WMA_WINDOW_SIZE + 1) / 2 = 78
    const float weight_sum = (float)(WMA_WINDOW_SIZE * (WMA_WINDOW_SIZE + 1)) / 2.0f;
    return weighted_sum / weight_sum;
}

// ============================================================================
// 中值滤波器 (Median Filter)
// 取窗口内中间值，对脉冲噪声 (编码器突跳) 有极好的抑制效果
// ============================================================================

void median_init(median_filter_t *mf, int window_size) {
    if (mf == NULL) return;
    // 限制窗口大小: 3~9, 必须奇数
    if (window_size < 3) window_size = 3;
    if (window_size > MEDIAN_FILTER_MAX_WINDOW) window_size = MEDIAN_FILTER_MAX_WINDOW;
    if (window_size % 2 == 0) window_size++;  // 偶数→奇数
    mf->window_size = window_size;
    mf->head = 0;
    mf->count = 0;
    for (int i = 0; i < MEDIAN_FILTER_MAX_WINDOW; i++) {
        mf->buf[i] = 0.0f;
    }
}

void median_reset(median_filter_t *mf) {
    if (mf == NULL) return;
    mf->head = 0;
    mf->count = 0;
    for (int i = 0; i < MEDIAN_FILTER_MAX_WINDOW; i++) {
        mf->buf[i] = 0.0f;
    }
}

void median_set_window(median_filter_t *mf, int window_size) {
    if (mf == NULL) return;
    if (window_size < 3) window_size = 3;
    if (window_size > MEDIAN_FILTER_MAX_WINDOW) window_size = MEDIAN_FILTER_MAX_WINDOW;
    if (window_size % 2 == 0) window_size++;
    mf->window_size = window_size;
    // 重置缓冲区 (窗口大小变化后旧数据无意义)
    mf->head = 0;
    mf->count = 0;
}

float median_compute(median_filter_t *mf, float x) {
    if (mf == NULL) return x;
    
    // 写入环形缓冲区
    mf->buf[mf->head] = x;
    mf->head = (mf->head + 1) % mf->window_size;
    if (mf->count < mf->window_size) {
        mf->count++;
    }
    
    // 窗口未填满时直通
    if (mf->count < mf->window_size) {
        return x;
    }
    
    // 复制窗口数据到临时数组进行排序
    float tmp[MEDIAN_FILTER_MAX_WINDOW];
    for (int i = 0; i < mf->window_size; i++) {
        tmp[i] = mf->buf[i];
    }
    
    // 简单插入排序 (窗口最大9，效率足够)
    for (int i = 1; i < mf->window_size; i++) {
        float key = tmp[i];
        int j = i - 1;
        while (j >= 0 && tmp[j] > key) {
            tmp[j + 1] = tmp[j];
            j--;
        }
        tmp[j + 1] = key;
    }
    
    // 返回中间值
    return tmp[mf->window_size / 2];
}
