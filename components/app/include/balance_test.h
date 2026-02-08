/**
 * @file balance_test.h
 * @brief 平衡控制测试模块 - 仅轮电机 (不含腿部)
 * @author Bubble
 * @date 2026-01-17
 * @note 参考 shibo_wheel_leg 项目的控制逻辑
 * 
 * 架构说明:
 * - Core 0: WiFi遥控任务 (低优先级)
 * - Core 1: IMU读取任务, 平衡控制任务, 电机通信任务 (高优先级)
 * 
 * 轮电机:
 * - 左轮: ID=3
 * - 右轮: ID=6
 */

#ifndef BALANCE_TEST_H
#define BALANCE_TEST_H

#include <stdbool.h>
#include "esp_err.h"
#include "leg_kinematics.h"  // 腿部运动学类型和接口

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 平衡测试状态
 */
typedef enum {
    BALANCE_TEST_IDLE = 0,      // 空闲 (电机断开)
    BALANCE_TEST_READY,         // 就绪 (等待使能)
    BALANCE_TEST_RUNNING,       // 平衡运行中
    BALANCE_TEST_EMERGENCY,     // 紧急停止
    BALANCE_TEST_ERROR,         // 错误状态
} balance_test_state_t;

/**
 * @brief 平衡测试统计信息
 */
typedef struct {
    uint32_t imu_read_count;        // IMU 读取次数
    uint32_t control_loop_count;    // 控制循环次数
    uint32_t motor_cmd_count;       // 电机命令次数
    uint32_t wifi_msg_count;        // WiFi 消息次数
    float control_freq_hz;          // 实际控制频率
    float imu_freq_hz;              // 实际IMU频率
    float motor_freq_hz;            // 实际电机通信频率
    float leg_freq_hz;              // 实际腿部电机频率
} balance_test_stats_t;

/**
 * @brief 初始化平衡测试模块
 * @return ESP_OK 成功
 */
esp_err_t balance_test_init(void);

/**
 * @brief 启动平衡测试 (创建所有任务)
 * @return ESP_OK 成功
 */
esp_err_t balance_test_start(void);

/**
 * @brief 停止平衡测试 (删除所有任务)
 */
void balance_test_stop(void);

/**
 * @brief 使能平衡控制
 */
void balance_test_enable(void);

/**
 * @brief 禁用平衡控制 (电机输出为0)
 */
void balance_test_disable(void);

/**
 * @brief 紧急停止
 */
void balance_test_emergency_stop(void);

/**
 * @brief 复位紧急停止
 */
void balance_test_reset_emergency(void);

/**
 * @brief 获取当前状态
 * @return 当前状态
 */
balance_test_state_t balance_test_get_state(void);

/**
 * @brief 获取统计信息
 * @param stats 输出统计信息
 */
void balance_test_get_stats(balance_test_stats_t *stats);

/**
 * @brief 设置角度零点
 * @param zeropoint 角度零点 (度)
 */
void balance_test_set_angle_zeropoint(float zeropoint);

/**
 * @brief 获取角度零点
 * @return 角度零点 (度)
 */
float balance_test_get_angle_zeropoint(void);

/**
 * @brief 使能/禁用波形数据输出
 * @param enable 是否使能
 * @note 波形输出格式: #DATA,<ID>,<Target>,<Control>
 */
void balance_test_set_plot(bool enable);

/**
 * @brief 设置波形输出分频系数
 * @param divider 分频系数 (1-255), 每 N 次控制循环输出一次
 * @note 控制循环 200Hz, divider=10 时输出 20Hz
 */
void balance_test_set_plot_divider(uint8_t divider);

/**
 * @brief 获取波形输出状态
 * @return true 已使能, false 已禁用
 */
bool balance_test_get_plot_enabled(void);

// ============================================================================
// 环路使能控制 (用于单环调试)
// ============================================================================

/**
 * @brief LQR 环路使能掩码
 */
typedef enum {
    LOOP_ANGLE    = (1 << 0),   // 角度环 (pitch -> torque)
    LOOP_GYRO     = (1 << 1),   // 角速度环 (pitch_rate -> torque)
    LOOP_DISTANCE = (1 << 2),   // 位移环 (displacement -> torque)
    LOOP_SPEED    = (1 << 3),   // 速度环 (velocity -> torque)
    LOOP_LQR_U    = (1 << 4),   // LQR输出PID (积分环节)
    LOOP_YAW      = (1 << 5),   // 偏航控制环
    
    // 预设组合
    LOOP_NONE     = 0,                                          // 全部禁用
    LOOP_SIMPLE   = (LOOP_ANGLE | LOOP_GYRO),                   // 简单平衡 (角度+角速度)
    LOOP_FULL     = (LOOP_ANGLE | LOOP_GYRO | LOOP_DISTANCE |   // 完整平衡
                     LOOP_SPEED | LOOP_LQR_U | LOOP_YAW),
} loop_enable_mask_t;

/**
 * @brief 设置环路使能 (使用掩码)
 * @param mask 环路使能掩码 (LOOP_xxx 的组合)
 * @note 例: balance_test_set_loop_enable(LOOP_ANGLE | LOOP_GYRO)
 */
void balance_test_set_loop_enable(uint8_t mask);

/**
 * @brief 获取当前环路使能掩码
 * @return 环路使能掩码
 */
uint8_t balance_test_get_loop_enable(void);

/**
 * @brief 设置单个环路的使能系数
 * @param loop 环路ID (LOOP_ANGLE, LOOP_GYRO, 等)
 * @param enable 使能系数 (0.0~1.0, 支持渐变)
 */
void balance_test_set_loop_gain(loop_enable_mask_t loop, float enable);

/**
 * @brief 获取单个环路的使能系数
 * @param loop 环路ID
 * @return 使能系数 (0.0~1.0)
 */
float balance_test_get_loop_gain(loop_enable_mask_t loop);

/**
 * @brief 打印环路使能状态
 */
void balance_test_print_loop_status(void);

// ============================================================================
// 腿部控制接口 (应用层封装 leg_kinematics 模块)
// ============================================================================

/**
 * @brief 初始化腿部控制
 * @note 需要在 balance_test_init() 之后调用
 */
void leg_ctrl_init(void);

/**
 * @brief 读取当前腿部状态 (从编码器)
 * @param is_left 是否为左腿
 * @param state 输出腿部状态
 * @return ESP_OK 成功
 */
esp_err_t leg_ctrl_get_state(bool is_left, leg_state_t *state);

/**
 * @brief 设置目标腿部状态 (腿长 + 身体夹角)
 * @param is_left 是否为左腿
 * @param leg_length 目标腿长 (米)
 * @param body_angle 目标身体夹角 (度)
 * @return ESP_OK 成功, ESP_ERR_INVALID_ARG 目标不可达
 */
esp_err_t leg_ctrl_set_target(bool is_left, float leg_length, float body_angle);

/**
 * @brief 设置双腿目标状态
 * @param left_length 左腿腿长 (米)
 * @param left_angle 左腿身体夹角 (度)
 * @param right_length 右腿腿长 (米)  
 * @param right_angle 右腿身体夹角 (度)
 * @return ESP_OK 成功
 */
esp_err_t leg_ctrl_set_both(float left_length, float left_angle,
                             float right_length, float right_angle);

/**
 * @brief 打印当前腿部状态
 */
void leg_ctrl_print_status(void);

/**
 * @brief 打印当前状态
 */
void balance_test_print_status(void);

// ============================================================================
// Leg Sync (防劈叉) API
// ============================================================================

/**
 * @brief 使能/禁用 Leg Sync (左右腿同步补偿)
 */
void balance_test_set_leg_sync(bool enable);

/**
 * @brief 获取 Leg Sync 状态
 */
bool balance_test_get_leg_sync(void);

/**
 * @brief 设置 Leg Sync 增益 (0~1)
 */
void balance_test_set_leg_sync_gain(float gain);

/**
 * @brief 设置 Leg Sync 最大修正量 (度)
 */
void balance_test_set_leg_sync_max(float max_deg);

/**
 * @brief 处理测试命令
 * @param cmd_str 命令字符串
 */
void balance_test_process_cmd(const char *cmd_str);

#ifdef __cplusplus
}
#endif

#endif // BALANCE_TEST_H
