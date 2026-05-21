#pragma once

/**
 * @brief 跨子模块共享类型定义
 *
 * 控制 mode、balance state、wheel command 等类型
 * 供 balance 组件和 app 组件共用，避免循环依赖。
 */

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 控制模式
// ============================================================================

typedef enum {
    CTRL_MODE_LQR = 0,      // LQR 多环控制 (默认)
    CTRL_MODE_DUAL_PID,     // 双环 PID 控制 (直立环+速度环) - 扭矩模式
    CTRL_MODE_SINGLE_PID,   // 单环 PID 控制 (直立环→速度) - 速度模式
    CTRL_MODE_CAR,          // 普通小车模式 (无直立环, 趴下跑)
    CTRL_MODE_TRIPLE_PID,   // 三环 PID 控制 (速度环→角度环→轮速环)
    CTRL_MODE_FULL_LQR,     // 完整 LQR 控制 (同时输出 T 和 Tp, K 随腿长插值)
} control_mode_t;

// ============================================================================
// 平衡测试状态
// ============================================================================

typedef enum {
    BALANCE_TEST_IDLE = 0,      // 空闲 (电机断开)
    BALANCE_TEST_READY,         // 就绪 (等待使能)
    BALANCE_TEST_RUNNING,       // 平衡运行中
    BALANCE_TEST_EMERGENCY,     // 紧急停止
    BALANCE_TEST_ERROR,         // 错误状态
} balance_test_state_t;

// ============================================================================
// 轮电机命令
// ============================================================================

typedef struct {
    float left_torque;      // 左轮力矩 (扭矩模式) / 左轮速度 (速度模式)
    float right_torque;     // 右轮力矩 (扭矩模式) / 右轮速度 (速度模式)
    bool enabled;           // 使能标志
    bool use_speed_mode;    // true=速度模式, false=扭矩模式
} shared_wheel_cmd_t;

// ============================================================================
// IMU 数据 (由 task_imu_read 写入, 多任务读取)
// ============================================================================

typedef struct {
    float pitch;            // 俯仰角 (度)
    float pitch_rate;       // 俯仰角速度 (度/秒)
    float roll;             // 横滚角 (度)
    float roll_rate;        // 横滚角速度 (度/秒)
    float yaw;              // 偏航角 (度)
    float yaw_rate;         // 偏航角速度 (度/秒)
    float accel_x;          // 加速度 X (g)
    float accel_y;          // 加速度 Y (g)
    float accel_z;          // 加速度 Z (g)
    uint32_t timestamp;     // 时间戳 (ms)
    uint64_t read_time_us;  // 精确读取时间 (us) - 用于延迟测量
    bool valid;             // 数据有效
} shared_imu_data_t;

// ============================================================================
// 轮电机状态 (由 task_motor_comm 写入, 多任务读取)
// ============================================================================

typedef struct {
    float left_position;    // 左轮位置 (度)
    float right_position;   // 右轮位置 (度)
    float left_speed;       // 左轮速度 (rpm)
    float right_speed;      // 右轮速度 (rpm)
    bool left_online;       // 左轮在线
    bool right_online;      // 右轮在线
    uint32_t timestamp;     // 时间戳 (ms)
} shared_wheel_state_t;

// ============================================================================
// 遥控数据 (由 WiFi 回调写入, 控制任务读取)
// ============================================================================

typedef struct {
    int16_t joy_x;          // 摇杆 X (-100~100) 转向
    int16_t joy_y;          // 摇杆 Y (-100~100) 速度
    int16_t joy_x_last;     // 上一次摇杆 X
    int16_t joy_y_last;     // 上一次摇杆 Y
    bool go;                // 使能开关
    bool car_mode;          // 小车模式开关 (来自 Web UI)
    uint32_t last_update;   // 最后更新时间 (ms)
    uint64_t receive_time_us; // 精确接收时间 (us) - 用于延迟测量
} shared_remote_data_t;

// ============================================================================
// 共享常量 (Bal-to-Car 过渡参数)
// ============================================================================

#define CAR_MODE_BODY_ANGLE         (-130.0f)   // 小车模式身体夹角 (度), 趴下
#define CAR_MODE_LEG_LENGTH         (0.067f)    // 小车模式腿长 (米)
#define CAR_TO_BAL_LEG_LENGTH       (0.068f)    // car→balance 恢复时腿长 (m)
#define BAL_TO_CAR_RETRACT_LENGTH   (0.068f)    // 收腿目标腿长 (m)
#define BAL_TO_CAR_RETRACT_TIME_MS  (800)       // 收腿最长等待时间 (ms)
#define BAL_TO_CAR_TILT_SPEED_RPM   (50.0f)     // 后倾阶段轮速 (rpm, 向前转)
#define BAL_TO_CAR_TILT_TIME_MS     (500)       // 后倾持续时间 (ms)

#ifdef __cplusplus
}
#endif
