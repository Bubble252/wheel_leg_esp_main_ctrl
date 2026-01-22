/**
 * @file motor_test.h
 * @brief 电机测试模块
 * @author Bubble
 * @date 2026-01-15
 */

#ifndef MOTOR_TEST_H
#define MOTOR_TEST_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 启动电机测试模式
 * @return ESP_OK 成功
 */
esp_err_t motor_test_start(void);

/**
 * @brief 停止电机测试模式
 */
void motor_test_stop(void);

#ifdef __cplusplus
}
#endif

#endif // MOTOR_TEST_H
