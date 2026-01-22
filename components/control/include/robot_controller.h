/**
 * @file robot_controller.h
 * @brief 机器人主控制器
 * @author Bubble
 * @date 2026-01-15
 */

#ifndef ROBOT_CONTROLLER_H
#define ROBOT_CONTROLLER_H

#include <stdint.h>
#include <stdbool.h>
#include "types.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化机器人控制器
 * @return ESP_OK 成功
 */
esp_err_t robot_controller_init(void);

/**
 * @brief 启动机器人控制器
 * @return ESP_OK 成功
 */
esp_err_t robot_controller_start(void);

/**
 * @brief 停止机器人控制器
 */
void robot_controller_stop(void);

/**
 * @brief 设置遥控命令
 * @param cmd 遥控命令
 */
void robot_controller_set_command(const remote_cmd_t *cmd);

/**
 * @brief 获取机器人状态数据
 * @param data 输出数据
 * @return ESP_OK 成功
 */
esp_err_t robot_controller_get_data(robot_data_t *data);

/**
 * @brief 请求站立
 * @return ESP_OK 成功
 */
esp_err_t robot_controller_stand_up(void);

/**
 * @brief 请求坐下
 * @return ESP_OK 成功
 */
esp_err_t robot_controller_sit_down(void);

/**
 * @brief 触发紧急停止
 */
void robot_controller_emergency_stop(void);

#ifdef __cplusplus
}
#endif

#endif // ROBOT_CONTROLLER_H
