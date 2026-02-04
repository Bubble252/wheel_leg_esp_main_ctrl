/**
 * @file robot_state.h
 * @brief 机器人状态结构体定义 - 统一的传感器数据和计算结果存储
 * @author Bubble
 * @date 2026-02-04
 * 
 * 设计目标:
 *   1. 所有单位转换只在一处进行
 *   2. 所有 FK 计算只在一处进行
 *   3. 所有算法共享同一份数据，避免重复计算
 *   4. 内部统一使用弧度制
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
// 常量定义
// ============================================================================

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 单位转换常量 (编译时计算，避免运行时开销)
#define DEG_TO_RAD      (M_PI / 180.0f)         // 度 → 弧度
#define RAD_TO_DEG      (180.0f / M_PI)         // 弧度 → 度
#define RPM_TO_RAD_S    (M_PI / 30.0f)          // rpm → rad/s
#define RAD_S_TO_RPM    (30.0f / M_PI)          // rad/s → rpm

// 机器人物理参数
#ifndef WHEEL_RADIUS_M
#define WHEEL_RADIUS_M  0.03f                   // 轮子半径 (m)
#endif

// ============================================================================
// 原始传感器数据结构 (应用层填充)
// ============================================================================

/**
 * @brief IMU 原始数据 (度制)
 */
typedef struct {
    float pitch_deg;            // 俯仰角 (度), 前倾为正
    float pitch_rate_dps;       // 俯仰角速度 (度/秒)
    float roll_deg;             // 横滚角 (度), 右倾为正
    float roll_rate_dps;        // 横滚角速度 (度/秒)
    float yaw_deg;              // 偏航角 (度), 逆时针为正
    float yaw_rate_dps;         // 偏航角速度 (度/秒)
    bool valid;                 // 数据有效标志
} imu_raw_data_t;

/**
 * @brief 轮电机原始数据
 */
typedef struct {
    float left_pos_deg;         // 左轮位置 (度)
    float left_vel_rpm;         // 左轮速度 (rpm)
    float right_pos_deg;        // 右轮位置 (度)
    float right_vel_rpm;        // 右轮速度 (rpm)
    bool left_online;           // 左轮在线
    bool right_online;          // 右轮在线
} wheel_raw_data_t;

/**
 * @brief 单腿电机原始数据
 */
typedef struct {
    float hip_pos_deg;          // 髋关节位置 (度)
    float hip_vel_rpm;          // 髋关节速度 (rpm)
    float knee_pos_deg;         // 膝关节位置 (度)
    float knee_vel_rpm;         // 膝关节速度 (rpm)
    bool hip_online;            // 髋关节在线
    bool knee_online;           // 膝关节在线
} leg_raw_data_t;

/**
 * @brief 遥控器原始数据
 */
typedef struct {
    int16_t joy_x;              // 摇杆 X (-100~100), 转向
    int16_t joy_y;              // 摇杆 Y (-100~100), 速度
    bool go;                    // 使能开关
    uint32_t last_update_ms;    // 最后更新时间
} remote_raw_data_t;

/**
 * @brief 所有原始传感器数据
 */
typedef struct {
    imu_raw_data_t imu;
    wheel_raw_data_t wheel;
    leg_raw_data_t left_leg;
    leg_raw_data_t right_leg;
    remote_raw_data_t remote;
    float dt;                   // 控制周期 (秒)
    uint32_t timestamp_ms;      // 时间戳 (毫秒)
} sensor_raw_data_t;

// ============================================================================
// 计算后的状态数据 (弧度制)
// ============================================================================

/**
 * @brief IMU 状态 (弧度制)
 */
typedef struct {
    float pitch;                // 俯仰角 (rad), 前倾为正
    float pitch_rate;           // 俯仰角速度 (rad/s)
    float pitch_acc;            // 俯仰角加速度 (rad/s²) - 数值微分
    float roll;                 // 横滚角 (rad), 右倾为正
    float roll_rate;            // 横滚角速度 (rad/s)
    float roll_acc;             // 横滚角加速度 (rad/s²) - 数值微分
    float yaw;                  // 偏航角 (rad)
    float yaw_rate;             // 偏航角速度 (rad/s)
    float yaw_total;            // 累积偏航角 (rad), 经过过零处理
} imu_state_t;

/**
 * @brief 轮电机状态 (弧度制)
 */
typedef struct {
    float left_pos;             // 左轮位置 (rad)
    float left_vel;             // 左轮速度 (rad/s)
    float right_pos;            // 右轮位置 (rad)
    float right_vel;            // 右轮速度 (rad/s)
} wheel_state_t;

/**
 * @brief 单腿状态 (运动学计算结果)
 * 
 * 使用 leg_extended_state_t 避免与 leg_kinematics.h 中的 leg_state_t 冲突
 */
typedef struct {
    // === 关节空间 (电机角度) ===
    float hip_pos_deg;          // 髋关节位置 (度) - 保留原始值用于 VMC 接口
    float knee_pos_deg;         // 膝关节位置 (度)
    float hip_pos;              // 髋关节位置 (rad)
    float knee_pos;             // 膝关节位置 (rad)
    float hip_vel;              // 髋关节速度 (rad/s)
    float knee_vel;             // 膝关节速度 (rad/s)
    float hip_acc;              // 髋关节加速度 (rad/s²) - 数值微分
    float knee_acc;             // 膝关节加速度 (rad/s²)
    
    // === 工作空间 (FK 计算结果) ===
    float leg_length;           // 腿长 (m)
    float body_angle;           // 身体夹角 (rad), -π/2 = 垂直向下
    float dL;                   // 腿长变化率 (m/s), 一阶导
    float dalpha;               // 身体角速度 (rad/s), 一阶导
    float ddL;                  // 腿长加速度 (m/s²), 二阶导
    float ddalpha;              // 身体角加速度 (rad/s²), 二阶导
    
    // === 世界坐标系位置 (需要 IMU pitch) ===
    float world_x;              // 轮子相对髋关节的水平位置 (m), 向后为正
    float world_y;              // 轮子相对髋关节的垂直位置 (m), 向上为正 (通常为负)
    float world_vx;             // 世界坐标系水平速度 (m/s)
    float world_vy;             // 世界坐标系垂直速度 (m/s)
    float world_ay;             // 世界坐标系垂直加速度 (m/s²) - 用于跳跃/着地检测
    
    // === 雅可比矩阵 (缓存) ===
    float J[4];                 // 机身坐标系雅可比 [dL/dhip, dL/dknee; dalpha/dhip, dalpha/dknee]
    float J_world[4];           // 世界坐标系雅可比 [dx/dhip, dx/dknee; dy/dhip, dy/dknee]
    
    // === 腿相对世界竖直方向的角度及其导数 ===
    float theta_world;          // 腿相对世界竖直向下的角度 (rad)
                                // theta_world = pitch + body_angle + π/2
                                // = 0 时腿垂直向下
    float dtheta_world;         // theta_world 的一阶导 (rad/s)
                                // = pitch_rate + dalpha
    float ddtheta_world;        // theta_world 的二阶导 (rad/s²)
                                // = pitch_acc + ddalpha (pitch_acc 由 IMU 提供或数值微分)
    
    bool valid;                 // 数据有效
} leg_extended_state_t;

/**
 * @brief 机器人运动学状态
 */
typedef struct {
    // === 线速度估计 (基于轮速) ===
    float robot_vx;             // 机器人水平速度 (m/s), 向后为正
    float lqr_speed;            // LQR 速度 (m/s), 前进为正 (= -robot_vx)
    
    // === 位移累积 (所有控制模式共用) ===
    float lqr_distance;         // LQR 累积位移 (m), 前进为正
    float left_wheel_distance;  // 左轮累积位移 (m)
    float right_wheel_distance; // 右轮累积位移 (m)
    
    // === 轮子平均值 ===
    float wheel_vel_avg;        // 平均轮速 (rad/s), 前进为正，用于速度环
    float wheel_pos_avg;        // 平均轮位置 (rad)
    
    // === 差速 (用于 YAW 控制) ===
    float wheel_vel_diff;       // 轮速差 (rad/s), left - right
    float wheel_pos_diff;       // 轮位置差 (rad), left - right
    
    // === 机身高度估计 ===
    float body_height;          // 机身离地高度 (m), 基于世界坐标系
} robot_kinematics_t;

/**
 * @brief 遥控器状态 (归一化)
 */
typedef struct {
    float target_speed;         // 目标速度 (m/s 或 rad/s，取决于控制模式)
    float target_yaw_rate;      // 目标偏航角速度 (rad/s)
    bool enabled;               // 控制使能
} remote_state_t;

/**
 * @brief 完整机器人状态 (所有算法的输入)
 */
typedef struct {
    // === 转换后的传感器状态 (弧度制) ===
    imu_state_t imu;
    wheel_state_t wheel;
    leg_extended_state_t left_leg;
    leg_extended_state_t right_leg;
    robot_kinematics_t kinematics;
    remote_state_t remote;
    
    // === 控制周期 ===
    float dt;
    uint32_t timestamp_ms;
    
    // === Pitch 补偿 (考虑腿部角度) ===
    // theta3 = pitch + body_angle + π/2
    // 当 body_angle = -π/2（垂直向下）时，theta3 = pitch
    // 当腿向前摆时，theta3 增大
    float pitch_compensated;    // 左右腿平均补偿后的 pitch (rad)
    float pitch_comp_left;      // 左腿补偿后的 pitch (rad)
    float pitch_comp_right;     // 右腿补偿后的 pitch (rad)
    
    // === 机身加速度 (世界坐标系) ===
    float body_ay_world;        // 机身竖直方向加速度 (m/s²), 向上为正
                                // 用于跳跃检测、着地检测、WBC等
    
    // === 双腿平均值 (便于使用) ===
    float avg_leg_length;       // 平均腿长 (m)
    float avg_body_angle;       // 平均身体角度 (rad)
    float avg_dL;               // 平均腿长变化率 (m/s)
    float avg_dalpha;           // 平均身体角速度 (rad/s)
    
    // === 数据有效性 ===
    bool imu_valid;
    bool wheel_valid;
    bool leg_valid;
} robot_state_t;

// ============================================================================
// 控制输出结构
// ============================================================================

/**
 * @brief 轮电机控制输出
 */
typedef struct {
    float left_torque;          // 左轮扭矩 (Nm), 负=前进
    float right_torque;         // 右轮扭矩 (Nm), 负=前进
    bool use_speed_mode;        // 使用速度模式 (此时 torque 实为 rad/s)
} wheel_ctrl_output_t;

/**
 * @brief 腿电机控制输出
 */
typedef struct {
    float left_hip_torque;      // 左髋扭矩 (Nm)
    float left_knee_torque;     // 左膝扭矩 (Nm)
    float right_hip_torque;     // 右髋扭矩 (Nm)
    float right_knee_torque;    // 右膝扭矩 (Nm)
} leg_ctrl_output_t;

/**
 * @brief 虚拟力输入 (用于腿力控制)
 */
typedef struct {
    float F_L;                  // 腿长方向力 (N), 正=伸展
    float F_alpha;              // 身体角度力矩 (Nm), 正=向前
} leg_virtual_force_t;

// ============================================================================
// API 函数
// ============================================================================

/**
 * @brief 初始化机器人状态计算模块
 * @return ESP_OK 成功
 */
esp_err_t robot_state_init(void);

/**
 * @brief 从原始传感器数据更新机器人状态
 * @param raw 原始传感器数据
 * @param state 输出的机器人状态
 * @param enable_leg_comp 是否启用腿部 pitch 补偿
 * @return ESP_OK 成功
 * 
 * @note 此函数执行:
 *   1. 所有单位转换 (deg→rad, rpm→rad/s)
 *   2. 腿部 FK 计算 (关节角→腿长/身体角度)
 *   3. 雅可比计算和缓存
 *   4. 速度估计
 *   5. YAW 过零处理
 */
esp_err_t robot_state_update(const sensor_raw_data_t *raw, 
                              robot_state_t *state,
                              bool enable_leg_comp);

/**
 * @brief 重置机器人状态 (清除累积量)
 * @param state 机器人状态
 */
void robot_state_reset(robot_state_t *state);

/**
 * @brief 重置位移累积
 * @param state 机器人状态
 */
void robot_state_reset_distance(robot_state_t *state);

/**
 * @brief 重置 YAW 累积
 * @param state 机器人状态
 */
void robot_state_reset_yaw(robot_state_t *state);

/**
 * @brief 打印机器人状态 (调试用)
 * @param state 机器人状态
 */
void robot_state_print(const robot_state_t *state);

// ============================================================================
// 辅助宏 (内联转换)
// ============================================================================

// 角度 → 弧度
static inline float deg2rad(float deg) {
    return deg * DEG_TO_RAD;
}

// 弧度 → 角度
static inline float rad2deg(float rad) {
    return rad * RAD_TO_DEG;
}

// rpm → rad/s
static inline float rpm2rads(float rpm) {
    return rpm * RPM_TO_RAD_S;
}

// rad/s → rpm
static inline float rads2rpm(float rads) {
    return rads * RAD_S_TO_RPM;
}

// 限幅
static inline float clampf(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

#ifdef __cplusplus
}
#endif

#endif // ROBOT_STATE_H
