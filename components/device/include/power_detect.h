/**
 * @file power_detect.h
 * @brief 电源检测驱动
 * @author Bubble
 * @date 2026-01-15
 */

#ifndef POWER_DETECT_H
#define POWER_DETECT_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化电源检测和电机使能控制
 * @return ESP_OK 成功
 */
esp_err_t power_init(void);

/**
 * @brief 反初始化
 */
void power_deinit(void);

/**
 * @brief 检测是否有外部供电
 * @return true 有电
 */
bool power_is_available(void);

/**
 * @brief 检测是否为电池供电
 * @return true 电池供电, false USB 供电
 */
bool power_is_battery(void);

/**
 * @brief 使能电机电源
 * @param enable true 打开, false 关闭
 * @return ESP_OK 成功
 */
esp_err_t power_motor_enable(bool enable);

/**
 * @brief 获取电机电源状态
 * @return true 已使能
 */
bool power_motor_is_enabled(void);

#ifdef __cplusplus
}
#endif

#endif // POWER_DETECT_H
