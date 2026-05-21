#pragma once

/**
 * @brief 任务管理、初始化、WiFi遥控
 */

#include "balance_types.h"

#ifdef __cplusplus
extern "C" {
#endif

void task_imu_read(void *arg);
void task_balance_ctrl(void *arg);
void task_motor_comm(void *arg);
void task_unified_control(void *arg);
void task_remote_watchdog(void *arg);
void update_remote_from_wifi(void);

#ifdef __cplusplus
}
#endif
