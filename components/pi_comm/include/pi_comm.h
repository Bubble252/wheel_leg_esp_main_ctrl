/**
 * @file pi_comm.h
 * @brief 树莓派通信模块公共接口
 * @author Bubble
 * @date 2026-01-27
 */

#ifndef PI_COMM_H
#define PI_COMM_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "pi_protocol.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 配置
// ============================================================================

#define PI_COMM_UART_NUM            UART_NUM_1
#define PI_COMM_UART_TX_PIN         4
#define PI_COMM_UART_RX_PIN         5
#define PI_COMM_UART_BAUD           115200
#define PI_COMM_TASK_STACK_SIZE     4096
#define PI_COMM_TASK_PRIORITY       5
#define PI_COMM_TASK_CORE           0

#define PI_COMM_HEARTBEAT_TIMEOUT_MS    2000    // 心跳超时 (ms), 需大于 Python 心跳间隔
#define PI_COMM_STATUS_REPORT_INTERVAL  50      // 状态上报间隔 (ms)

// ============================================================================
// 连接状态
// ============================================================================

typedef enum {
    PI_CONN_DISCONNECTED = 0,
    PI_CONN_HANDSHAKING,
    PI_CONN_CONNECTED,
    PI_CONN_RECONNECTING,
} pi_conn_state_t;

// ============================================================================
// 回调函数类型
// ============================================================================

// 速度指令回调
typedef void (*pi_velocity_callback_t)(float vx, float yaw_rate);

// 模式切换回调
typedef void (*pi_mode_callback_t)(uint8_t mode);

// 高度指令回调
typedef void (*pi_height_callback_t)(float height, float duration);

// Pitch 指令回调
typedef void (*pi_pitch_callback_t)(float pitch, float duration);

// Roll 指令回调
typedef void (*pi_roll_callback_t)(float roll, float duration);

// 姿态指令回调
typedef void (*pi_pose_callback_t)(float pitch, float roll, float height, float duration);

// 电机使能回调
typedef void (*pi_motor_enable_callback_t)(bool enable);

// 紧急停止回调
typedef void (*pi_estop_callback_t)(uint8_t reason);

// 断连回调
typedef void (*pi_disconnect_callback_t)(void);

// ============================================================================
// 回调注册结构
// ============================================================================

typedef struct {
    pi_velocity_callback_t      on_velocity;
    pi_mode_callback_t          on_mode;
    pi_height_callback_t        on_height;
    pi_pitch_callback_t         on_pitch;
    pi_roll_callback_t          on_roll;
    pi_pose_callback_t          on_pose;
    pi_motor_enable_callback_t  on_motor_enable;
    pi_estop_callback_t         on_estop;
    pi_disconnect_callback_t    on_disconnect;
} pi_comm_callbacks_t;

// ============================================================================
// 状态数据结构 (用于上报)
// ============================================================================

typedef struct {
    uint8_t  mode;
    uint8_t  status;
    uint8_t  error_code;
    uint8_t  flags;
    float    pitch;
    float    roll;
    float    yaw;
    float    vx_actual;
    float    yaw_rate_actual;
    float    battery_voltage;
    float    height_actual;
} pi_robot_state_t;

// ============================================================================
// 公共函数
// ============================================================================

/**
 * @brief 初始化 Pi 通信模块
 * @return ESP_OK 成功
 */
esp_err_t pi_comm_init(void);

/**
 * @brief 反初始化
 */
void pi_comm_deinit(void);

/**
 * @brief 注册回调函数
 * @param callbacks 回调函数结构
 */
void pi_comm_register_callbacks(const pi_comm_callbacks_t *callbacks);

/**
 * @brief 获取连接状态
 * @return 连接状态
 */
pi_conn_state_t pi_comm_get_state(void);

/**
 * @brief 检查是否已连接
 * @return true 已连接
 */
bool pi_comm_is_connected(void);

/**
 * @brief 更新机器人状态 (供其他模块调用)
 * @param state 状态数据
 */
void pi_comm_update_state(const pi_robot_state_t *state);

/**
 * @brief 发送错误上报
 * @param error_code 错误码
 * @param severity 严重程度
 * @param source 错误来源
 * @param message 错误描述
 */
void pi_comm_send_error(uint8_t error_code, uint8_t severity, 
                        uint8_t source, const char *message);

/**
 * @brief 发送事件上报
 * @param event_type 事件类型
 * @param data 事件数据
 * @param data_len 数据长度
 */
void pi_comm_send_event(uint8_t event_type, const uint8_t *data, size_t data_len);

/**
 * @brief 获取统计信息
 */
typedef struct {
    uint32_t tx_frames;
    uint32_t rx_frames;
    uint32_t rx_errors;
    uint32_t crc_errors;
    uint32_t heartbeat_count;
    uint32_t last_rx_time;
} pi_comm_stats_t;

void pi_comm_get_stats(pi_comm_stats_t *stats);

#ifdef __cplusplus
}
#endif

#endif // PI_COMM_H
