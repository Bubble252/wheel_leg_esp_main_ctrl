/**
 * @file types.h
 * @brief 自定义类型定义
 * @author Bubble
 * @date 2026-01-15
 */

#ifndef __TYPES_H__
#define __TYPES_H__

#include <stdint.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 枚举类型
// ============================================================================

// 腿部标识
typedef enum {
    LEG_LEFT = 0,
    LEG_RIGHT,
    LEG_COUNT
} leg_id_t;

// 电机类型
typedef enum {
    MOTOR_TYPE_HIP = 0,     // 大腿/髋关节
    MOTOR_TYPE_KNEE,        // 小腿/膝关节
    MOTOR_TYPE_WHEEL,       // 轮电机
    MOTOR_TYPE_COUNT
} motor_type_t;

// 控制模式
typedef enum {
    MODE_TORQUE = 0,        // 力矩模式
    MODE_SPEED,             // 速度模式
    MODE_POS_TRAP,          // 位置梯形轨迹
    MODE_POS_FILTER,        // 位置滤波模式
    MODE_POS_DIRECT,        // 位置直通模式
    MODE_LOW_SPEED,         // 低速大扭模式
} motor_mode_t;

// 机器人状态
typedef enum {
    STATE_INIT = 0,         // 初始化中
    STATE_IDLE,             // 空闲 (电机断电)
    STATE_STANDING_UP,      // 站立过程
    STATE_BALANCING,        // 站立平衡中
    STATE_SITTING_DOWN,     // 蹲下过程
    STATE_ERROR,            // 故障状态
    STATE_EMERGENCY_STOP,   // 紧急停止
} robot_state_t;

// 遥控方向命令
typedef enum {
    DIR_FORWARD = 0,
    DIR_BACK,
    DIR_RIGHT,
    DIR_LEFT,
    DIR_STOP,
    DIR_JUMP,
} direction_t;

// ============================================================================
// 数据结构
// ============================================================================

// IMU 数据
typedef struct {
    float roll;             // 横滚角 (°)
    float pitch;            // 俯仰角 (°)
    float yaw;              // 偏航角 (°)
    float gyro_x;           // 角速度 X (°/s)
    float gyro_y;           // 角速度 Y (°/s)
    float gyro_z;           // 角速度 Z (°/s)
    float accel_x;          // 加速度 X (g)
    float accel_y;          // 加速度 Y (g)
    float accel_z;          // 加速度 Z (g)
    uint32_t timestamp;     // 时间戳 (ms)
    bool is_valid;          // 数据有效标志
} imu_data_t;

// 单个电机状态
typedef struct {
    uint8_t motor_id;       // 电机 ID
    float position;         // 位置 (°)
    float speed;            // 速度 (rpm)
    float current;          // 电流 (A)
    float voltage;          // 电压 (V)
    float driver_temp;      // 驱动器温度 (°C)
    float motor_temp;       // 电机温度 (°C)
    uint32_t error_code;    // 错误码
    bool is_online;         // 在线状态
    uint32_t last_update;   // 最后更新时间 (ms)
} motor_state_t;

// 电机指令
typedef struct {
    uint8_t motor_id;       // 电机 ID
    motor_mode_t mode;      // 控制模式
    float target;           // 目标值 (根据模式: 力矩/速度/位置)
    float speed_limit;      // 速度限制 (位置模式)
} motor_cmd_t;

// 遥控器数据 (来自 Web/手机)
typedef struct {
    direction_t dir;        // 方向命令
    direction_t dir_last;   // 上一次方向
    int16_t height;         // 腿高度 (mm)
    int16_t roll;           // Roll 目标角度 (°)
    float speed;            // 速度目标 (m/s)
    float turn;             // 转向目标 (°/s)
    int16_t linear;         // 线速度
    int16_t angular;        // 角速度
    int16_t joy_x;          // 摇杆 X (-100~100)
    int16_t joy_y;          // 摇杆 Y (-100~100)
    int16_t joy_x_last;     // 上一次摇杆 X
    int16_t joy_y_last;     // 上一次摇杆 Y
    bool go;                // 使能开关
    uint32_t last_update;   // 最后更新时间 (ms)
} remote_cmd_t;

// 机器人全局状态
typedef struct {
    // 传感器数据
    imu_data_t imu;
    motor_state_t motors[6];        // 6个电机
    
    // 系统状态
    robot_state_t state;
    float battery_voltage;
    bool is_battery_powered;
    uint32_t timestamp;             // 时间戳 (ms)
    
    // 遥控数据
    remote_cmd_t remote;
    
    // 互斥锁
    SemaphoreHandle_t mutex;
} robot_data_t;

// ============================================================================
// 全局变量声明
// ============================================================================

extern robot_data_t g_robot;

// ============================================================================
// 线程安全访问宏
// ============================================================================

#define ROBOT_LOCK()    do { if (g_robot.mutex) xSemaphoreTake(g_robot.mutex, portMAX_DELAY); } while(0)
#define ROBOT_UNLOCK()  do { if (g_robot.mutex) xSemaphoreGive(g_robot.mutex); } while(0)

// ============================================================================
// 工具宏
// ============================================================================

#define CLAMP(x, min, max)  ((x) < (min) ? (min) : ((x) > (max) ? (max) : (x)))
#define DEG_TO_RAD(deg)     ((deg) * 0.017453292519943295f)
#define RAD_TO_DEG(rad)     ((rad) * 57.29577951308232f)
#define ABS(x)              ((x) < 0 ? -(x) : (x))

#ifdef __cplusplus
}
#endif

#endif // __TYPES_H__
