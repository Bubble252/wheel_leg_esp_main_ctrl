/**
 * @file button_driver.h
 * @brief 按钮驱动
 * @author Bubble
 * @date 2026-01-15
 */

#ifndef BUTTON_DRIVER_H
#define BUTTON_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 按钮ID定义
 */
typedef enum {
    BUTTON_1 = 0,   // IO13
    BUTTON_2 = 1,   // IO9
    BUTTON_COUNT
} button_id_t;

/**
 * @brief 按钮事件类型
 */
typedef enum {
    BUTTON_EVENT_PRESS,
    BUTTON_EVENT_RELEASE,
    BUTTON_EVENT_LONG_PRESS,
    BUTTON_EVENT_DOUBLE_CLICK,
} button_event_t;

/**
 * @brief 按钮事件回调函数
 */
typedef void (*button_callback_t)(button_id_t button, button_event_t event);

/**
 * @brief 初始化按钮驱动
 * @return ESP_OK 成功
 */
esp_err_t button_init(void);

/**
 * @brief 反初始化按钮
 */
void button_deinit(void);

/**
 * @brief 注册按钮回调
 * @param callback 回调函数
 */
void button_register_callback(button_callback_t callback);

/**
 * @brief 读取按钮状态
 * @param button 按钮ID
 * @return true 按下
 */
bool button_is_pressed(button_id_t button);

/**
 * @brief 按钮轮询处理 (如果不使用中断)
 */
void button_poll(void);

#ifdef __cplusplus
}
#endif

#endif // BUTTON_DRIVER_H
