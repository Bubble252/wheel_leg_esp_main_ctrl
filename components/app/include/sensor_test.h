/**
 * @file sensor_test.h
 * @brief 传感器测试模块 (按键、温湿度等)
 * @author Bubble
 * @date 2026-01-16
 */

#ifndef SENSOR_TEST_H
#define SENSOR_TEST_H

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化传感器测试模块
 * @return ESP_OK 成功
 */
esp_err_t sensor_test_init(void);

/**
 * @brief 反初始化传感器测试模块
 */
void sensor_test_deinit(void);

/**
 * @brief 处理传感器测试命令
 * @param cmd_line 命令行字符串 (不含 "sensor" 前缀)
 */
void sensor_test_process_cmd(const char *cmd_line);

/**
 * @brief 打印传感器测试帮助
 */
void sensor_test_print_help(void);

/**
 * @brief 处理按键命令
 * @param cmd_line 命令行字符串 (不含 "btn" 前缀)
 */
void btn_process_cmd(const char *cmd_line);

/**
 * @brief 处理 SHT30 命令
 * @param cmd_line 命令行字符串 (不含 "sht" 前缀)
 */
void sht_process_cmd(const char *cmd_line);

/**
 * @brief 处理电源检测命令
 */
void power_process_cmd(void);

/**
 * @brief 处理电机供电命令
 * @param cmd_line 命令行字符串 (不含 "motor" 前缀，应为 "on" 或 "off")
 */
void motor_power_process_cmd(const char *cmd_line);

#ifdef __cplusplus
}
#endif

#endif // SENSOR_TEST_H
