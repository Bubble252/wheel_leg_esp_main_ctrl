/**
 * @file robot_state.h
 * @brief 机器人状态计算模块 - 统一的物理量计算和单位转换
 * @author Bubble
 * @date 2026-02-04
 * 
 * 设计目标:
 *   1. 所有传感器数据统一转换为 SI 单位 (m, rad, s)
 *   2. 避免在多处重复计算相同的物理量
 *   3. 为各控制模块提供一致的输入数据
 * 
 * 单位约定:
 *   - 角度: 弧度 (rad)
 *   - 长度: 米 (m)
 *   - 速度: m/s 或 rad/s
 *   - 力: N
 *   - 力矩: Nm
 *   - 电机输入角度: 度 (deg) - 仅用于与电机通信
 *   - 电机输入速度: rpm - 仅用于与电机通信
 */

#ifndef ROBOT_STATE_H
#define ROBOT_STATE_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 物理常量
// ============================================================================

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 机械参数
#define ROBOT_WHEEL_RADIUS_M        0.03f       // 轮子半径 (m)
#define ROBOT_THIGH_LENGTH_M        0.10f       // 大腿长度 (m)
#define ROBOT_SHANK_LENGTH_M        0.10f       // 小腿长度 (m)
#define ROBOT_MASS_KG               1.0f        // 机器人质量 (kg)
#define ROBOT_GRAVITY_M_S2          9.81f       // 重力加速度 (m/s²)

// 腿部工作空间限制
#define ROBOT_LEG_LENGTH_MIN_M      0.07f       // 最小腿长 (m)
#define ROBOT_LEG_LENGTH_MAX_M      0.17f       // 最大腿长 (m)
#define ROBOT_LEG_ANGLE_MIN_RAD     (-160.0f * M_PI / 180.0f)  // -160°
#define ROBOT_LEG_ANGLE_MAX_RAD     (-20.0f * M_PI / 180.0f)   // -20°
#define ROBOT_LEG_DEFAULT_LENGTH_M  0.14f       // 默认腿长 (m)
#define ROBOT_LEG_DEFAULT_ANGLE_RAD (-90.0f * M_PI / 180.0f)   // -90° (垂直)

// 单位转换宏
#define DEG2RAD(deg)            ((deg) * M_PI / 180.0f)
#define RAD2DEG(rad)            ((rad) * 180.0f / M_PI)
#define RPM2RAD_S(rpm)          ((rpm) * M_PI / 30.0f)
#define RAD_S2RPM(rad_s)        ((rad_s) * 30.0f / M_PI)

// 电机方向: 轮电机正转时机器人后退，需要取负
#define WHEEL_MOTOR_DIR         (-1.0f)

// ============================================================================
// 数据结构
// ============================================================================

/**
 * @brief 单腿状态 (SI 单位)
 */
typedef struct {
    // 工作空间状态 (SI 单位)
    float length_m;             // 腿长 (m)
    float body_angle_rad;       // 身体角度 (rad), -π/2 = 垂直向下
    float d_length_m_s;         // 腿长变化率 (m/s)
    float d_angle_rad_s;        // 身体角速度 (rad/s)
    
    // 关节空间状态 (用于电机通信, 保留原始单位)
    float hip_angle_deg;        // 髋电机角度 (deg)
    float knee_angle_deg;       // 膝电机角度 (deg)
    float hip_vel_rpm;          // 髋电机速度 (rpm)
    float knee_vel_rpm;         // 膝电机速度 (rpm)
    
    // 状态标志
    bool valid;                 // 数据是否有效 (电机在线)
} leg_state_si_t;

/**
 * @brief IMU 状态 (SI 单位, 全部弧度)
 */
typedef struct {
    float pitch_rad;            // 俯仰角 (rad), 前倾为正
    float roll_rad;             // 横滚角 (rad), 右倾为正
    float yaw_rad;              // 偏航角 (rad), 左转为正
    float pitch_rate_rad_s;     // 俯仰角速度 (rad/s)
    float roll_rate_rad_s;      // 横滚角速度 (rad/s)
    float yaw_rate_rad_s;       // 偏航角速度 (rad/s)
    float yaw_total_rad;        // 累积偏航角 (rad), 经过过零处理
    bool valid;                 // 数据是否有效
} imu_state_si_t;

/**
 * @brief 轮子状态 (SI 单位)
 */
typedef struct {
    // 左轮
    float left_pos_rad;         // 左轮位置 (rad)
    float left_vel_rad_s;       // 左轮速度 (rad/s)
    bool left_online;           // 左轮在线
    
    // 右轮
    float right_pos_rad;        // 右轮位置 (rad)
    float right_vel_rad_s;      // 右轮速度 (rad/s)
    bool right_online;          // 右轮在线
} wheel_state_si_t;

/**
 * @brief 机器人整体状态 (SI 单位)
 */
typedef struct {
    // IMU 状态
    imu_state_si_t imu;
    
    // 轮子状态
    wheel_state_si_t wheel;
    
    // 机器人运动状态 (由轮子位置/速度计算得出)
    float distance_m;           // 累积位移 (m), 向前为正
    float velocity_m_s;         // 线速度 (m/s), 向前为正
    
    // 腿部状态
    leg_state_si_t left_leg;
    leg_state_si_t right_leg;
    
    // 平均腿长和角度 (用于某些控制算法)
    float avg_leg_length_m;
    float avg_body_angle_rad;
    
    // 时间信息
    uint64_t timestamp_us;      // 时间戳 (us)
    float dt_s;                 // 距上次更新的时间 (s)
} robot_state_t;

/**
 * @brief 机器人状态原始输入 (传感器原始数据)
 * @note 此结构用于从传感器收集数据，然后由 robot_state_update() 转换
 */
typedef struct {
    // IMU 原始数据 (度)
    float pitch_deg;
    float pitch_rate_deg_s;
    float roll_deg;
    float roll_rate_deg_s;
    float yaw_deg;
    float yaw_rate_deg_s;
    bool imu_valid;
    
    // 轮电机原始数据
    float left_wheel_pos_deg;
    float left_wheel_vel_rpm;
    bool left_wheel_online;
    float right_wheel_pos_deg;
    float right_wheel_vel_rpm;
    bool right_wheel_online;
    
    // 腿电机原始数据 (度和 rpm)
    float left_hip_pos_deg;
    float left_hip_vel_rpm;
    float left_knee_pos_deg;
    float left_knee_vel_rpm;
    bool left_leg_online;
    
    float right_hip_pos_deg;
    float right_hip_vel_rpm;
    float right_knee_pos_deg;
    float right_knee_vel_rpm;
    bool right_leg_online;
    
    // 时间戳
    uint64_t timestamp_us;
} robot_state_raw_t;

// ============================================================================
// API 函数
// ============================================================================

/**
 * @brief 初始化机器人状态模块
 */
void robot_state_init(void);

/**
 * @brief 重置机器人状态 (清零累积量)
 * @note 在开始平衡控制前调用
 */
void robot_state_reset(void);

/**
 * @brief 从原始传感器数据更新机器人状态
 * @param raw 原始传感器数据
 * @param state 输出的 SI 单位状态
 * @return ESP_OK 成功
 * 
 * 此函数完成:
 *   1. 角度单位转换 (deg → rad)
 *   2. 速度单位转换 (rpm → rad/s)
 *   3. 机器人位移/速度计算
 *   4. 腿部 FK 计算 (如果腿电机在线)
 *   5. YAW 过零处理
 *   6. dt 计算
 */
esp_err_t robot_state_update(const robot_state_raw_t *raw, robot_state_t *state);

/**
 * @brief 获取当前机器人状态 (线程安全)
 * @param state 输出状态
 * @return ESP_OK 成功
 */
esp_err_t robot_state_get(robot_state_t *state);

/**
 * @brief 获取 YAW 累积角度 (用于方向保持)
 * @return 累积 YAW (rad)
 */
float robot_state_get_yaw_total(void);

/**
 * @brief 重置 YAW 累积角度
 */
void robot_state_reset_yaw_total(void);

/**
 * @brief 重置位移累积量
 */
void robot_state_reset_distance(void);

/**
 * @brief 获取距离零点
 * @return 距离零点 (m)
 */
float robot_state_get_distance_zeropoint(void);

/**
 * @brief 设置距离零点
 * @param zeropoint 新的零点 (m)
 */
void robot_state_set_distance_zeropoint(float zeropoint);

// ============================================================================
// 辅助函数 (供外部使用)
// ============================================================================

/**
 * @brief 轮速转机器人速度
 * @param left_vel_rad_s 左轮角速度 (rad/s)
 * @param right_vel_rad_s 右轮角速度 (rad/s)
 * @return 机器人线速度 (m/s), 向前为正
 */
static inline float robot_wheel_to_velocity(float left_vel_rad_s, float right_vel_rad_s) {
    // 轮电机正转时机器人后退，所以取负
    return WHEEL_MOTOR_DIR * 0.5f * (left_vel_rad_s + right_vel_rad_s) * ROBOT_WHEEL_RADIUS_M;
}

/**
 * @brief 轮位置转机器人位移
 * @param left_pos_rad 左轮位置 (rad)
 * @param right_pos_rad 右轮位置 (rad)
 * @return 机器人位移 (m), 向前为正
 */
static inline float robot_wheel_to_distance(float left_pos_rad, float right_pos_rad) {
    return WHEEL_MOTOR_DIR * 0.5f * (left_pos_rad + right_pos_rad) * ROBOT_WHEEL_RADIUS_M;
}

/**
 * @brief 限幅函数
 */
static inline float robot_clamp_f(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

#ifdef __cplusplus
}
#endif

#endif // ROBOT_STATE_H
