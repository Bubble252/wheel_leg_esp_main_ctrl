/**
 * @file state_machine.h
 * @brief 机器人状态机
 * @author Bubble
 * @date 2026-01-15
 */

#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>
#include <stdbool.h>
#include "types.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 状态切换回调函数
 */
typedef void (*state_change_callback_t)(robot_state_t old_state, robot_state_t new_state);

/**
 * @brief 初始化状态机
 * @return ESP_OK 成功
 */
esp_err_t state_machine_init(void);

/**
 * @brief 获取当前状态
 * @return 当前状态
 */
robot_state_t state_machine_get_state(void);

/**
 * @brief 请求状态切换
 * @param new_state 目标状态
 * @return ESP_OK 成功, ESP_ERR_INVALID_STATE 不允许的切换
 */
esp_err_t state_machine_request(robot_state_t new_state);

/**
 * @brief 触发紧急停止
 * @param reason 停止原因
 */
void state_machine_emergency_stop(const char *reason);

/**
 * @brief 清除紧急停止状态
 * @return ESP_OK 成功
 */
esp_err_t state_machine_clear_emergency(void);

/**
 * @brief 注册状态切换回调
 * @param callback 回调函数
 */
void state_machine_register_callback(state_change_callback_t callback);

/**
 * @brief 检查是否允许从当前状态切换到目标状态
 * @param target 目标状态
 * @return true 允许
 */
bool state_machine_can_transition(robot_state_t target);

/**
 * @brief 获取状态名称字符串
 * @param state 状态
 * @return 状态名称
 */
const char *state_machine_get_state_name(robot_state_t state);

#ifdef __cplusplus
}
#endif

#endif // STATE_MACHINE_H
