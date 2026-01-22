/**
 * @file lqr_balance.h
 * @brief LQR 平衡控制器
 * @author Bubble
 * @date 2026-01-15
 * @note 参考 shibo_wheel_leg 项目的 LQR 平衡算法
 * 
 * LQR 控制公式:
 * LQR_u = angle_control + gyro_control + distance_control + speed_control
 * 
 * 其中:
 * - angle_control: 俯仰角控制 (pitch -> torque)
 * - gyro_control: 角速度控制 (pitch_rate -> torque)
 * - distance_control: 轮子位移控制 (displacement -> torque)
 * - speed_control: 速度控制 (velocity -> torque)
 */

#ifndef LQR_BALANCE_H
#define LQR_BALANCE_H

#include "pid_controller.h"
#include "lowpass_filter.h"
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief LQR 控制器状态枚举
 */
typedef enum {
    LQR_STATE_IDLE = 0,         // 空闲状态
    LQR_STATE_BALANCING,        // 平衡中
    LQR_STATE_WHEEL_OFF_GROUND, // 轮子离地
    LQR_STATE_EMERGENCY,        // 紧急停止
} lqr_state_t;

/**
 * @brief LQR 参数结构体
 */
typedef struct {
    // 角度环 PID 参数 (pitch angle -> control)
    float angle_kp;         // 默认 0.7
    float angle_ki;         // 默认 0.55
    float angle_kd;         // 默认 0.001
    float angle_limit;      // 默认 10.0
    
    // 角速度环 PID 参数 (pitch rate -> control)
    float gyro_kp;          // 默认 0.05
    float gyro_ki;          // 默认 0.0
    float gyro_kd;          // 默认 0.0
    float gyro_limit;       // 默认 8.0
    
    // 位移环 PID 参数 (displacement -> control)
    float distance_kp;      // 默认 0.5
    float distance_ki;      // 默认 0.0
    float distance_kd;      // 默认 0.0
    float distance_limit;   // 默认 8.0
    
    // 速度环 PID 参数 (velocity -> control)
    float speed_kp;         // 默认 0.7 (自适应)
    float speed_ki;         // 默认 0.0
    float speed_kd;         // 默认 0.0
    float speed_limit;      // 默认 8.0
    float speed_kp_min;     // 速度P最小值
    float speed_kp_max;     // 速度P最大值
    
    // LQR 输出 PID 参数 (final output processing)
    float lqr_u_kp;         // 默认 1.0
    float lqr_u_ki;         // 默认 8.5
    float lqr_u_kd;         // 默认 0.0
    float lqr_u_limit;      // 默认 8.0
    
    // 偏航控制 PID 参数
    float yaw_angle_kp;     // 偏航角P
    float yaw_angle_ki;     // 偏航角I
    float yaw_angle_kd;     // 偏航角D
    float yaw_angle_limit;  // 偏航角限幅
    
    float yaw_gyro_kp;      // 偏航角速度P
    float yaw_gyro_ki;      // 偏航角速度I
    float yaw_gyro_kd;      // 偏航角速度D
    float yaw_gyro_limit;   // 偏航角速度限幅
    
    // 横滚控制 PID 参数
    float roll_kp;          // 横滚角P
    float roll_ki;          // 横滚角I
    float roll_kd;          // 横滚角D
    float roll_limit;       // 横滚角限幅
    
    // 零点调整 PID 参数
    float zeropoint_kp;     // 重心调整P
    float zeropoint_ki;     // 重心调整I
    float zeropoint_kd;     // 重心调整D
    float zeropoint_limit;  // 重心调整限幅
    
    // 角度零点
    float angle_zeropoint;  // 俯仰角零点偏移 (默认 7.4 度)
    
    // 低通滤波器时间常数
    float lpf_joyy_tf;      // 摇杆输入滤波 (默认 0.2)
    float lpf_zeropoint_tf; // 零点滤波 (默认 0.08)
    float lpf_roll_tf;      // 横滚滤波 (默认 0.1)
    
    // 安全阈值
    float emergency_angle;  // 紧急停止角度 (默认 45 度)
    
    // 轮子离地检测参数
    float wheel_off_ground_speed_threshold;    // 速度阈值
    float wheel_off_ground_accel_threshold;    // 加速度阈值
    
    // 输出限幅
    float max_wheel_torque; // 最大轮子扭矩
    float max_leg_torque;   // 最大腿部扭矩
    
    // 环路使能系数 (0.0 = 禁用, 1.0 = 启用, 可用于平滑过渡)
    float angle_enable;     // 角度环使能 (默认 1.0)
    float gyro_enable;      // 角速度环使能 (默认 1.0)
    float distance_enable;  // 位移环使能 (默认 1.0)
    float speed_enable;     // 速度环使能 (默认 1.0)
    float lqr_u_enable;     // LQR输出PID使能 (默认 1.0, 设为0则跳过积分)
} lqr_params_t;

/**
 * @brief LQR 传感器输入结构体
 */
typedef struct {
    // 姿态数据 (来自 IMU)
    float pitch;            // 俯仰角 (度)
    float pitch_rate;       // 俯仰角速度 (度/秒)
    float roll;             // 横滚角 (度)
    float roll_rate;        // 横滚角速度 (度/秒)
    float yaw;              // 偏航角 (度)
    float yaw_rate;         // 偏航角速度 (度/秒)
    
    // 轮子数据 (来自电机编码器)
    float left_wheel_pos;   // 左轮位置 (弧度)
    float right_wheel_pos;  // 右轮位置 (弧度)
    float left_wheel_vel;   // 左轮速度 (弧度/秒)
    float right_wheel_vel;  // 右轮速度 (弧度/秒)
    
    // LQR 状态量 (由外部计算并传入，考虑电机方向)
    float lqr_distance;     // 累积位移 (弧度)，正值=机器人前进方向
    float lqr_speed;        // 平均速度 (弧度/秒)，正值=机器人前进方向
    
    // YAW 累积角度 (经过过零处理，用于方向保持)
    float yaw_total;        // 累积 YAW 角度 (度)
    
    // 腿部数据 (来自电机编码器)
    float left_leg_length;  // 左腿长度 (米)
    float right_leg_length; // 右腿长度 (米)
    float left_leg_angle;   // 左腿角度 (弧度)
    float right_leg_angle;  // 右腿角度 (弧度)
    
    // 控制目标
    float target_speed;     // 目标速度 (弧度/秒)
    float target_yaw_rate;  // 目标偏航角速度 (度/秒)
    float target_height;    // 目标腿部高度 (米)
    
    // 时间步长
    float dt;               // 控制周期 (秒)
} lqr_input_t;

/**
 * @brief LQR 控制输出结构体
 */
typedef struct {
    // 轮子扭矩输出
    float left_wheel_torque;    // 左轮扭矩 (N·m)
    float right_wheel_torque;   // 右轮扭矩 (N·m)
    
    // 腿部扭矩输出 (预留接口)
    float left_hip_torque;      // 左髋关节扭矩 (N·m)
    float right_hip_torque;     // 右髋关节扭矩 (N·m)
    float left_knee_torque;     // 左膝关节扭矩 (N·m)
    float right_knee_torque;    // 右膝关节扭矩 (N·m)
    
    // 控制分量 (调试用)
    float angle_control;        // 角度控制量
    float gyro_control;         // 角速度控制量
    float distance_control;     // 位移控制量
    float speed_control;        // 速度控制量
    float lqr_u;                // LQR 综合输出
    float yaw_control;          // 偏航控制量
    float roll_control;         // 横滚控制量 (用于腿长控制)
    
    // 状态
    lqr_state_t state;          // 当前状态
    bool wheel_on_ground;       // 轮子是否在地面上
} lqr_output_t;

/**
 * @brief Roll 控制输出结构体 (用于腿长控制)
 */
typedef struct {
    float roll_control;         // Roll PID 输出
    float left_leg_delta;       // 左腿长度增量 (正=伸长)
    float right_leg_delta;      // 右腿长度增量 (正=伸长)
    float filtered_roll;        // 滤波后的 Roll 角度
} lqr_roll_output_t;

/**
 * @brief LQR 控制器结构体
 */
typedef struct {
    // PID 控制器
    pid_controller_t pid_angle;     // 角度环
    pid_controller_t pid_gyro;      // 角速度环
    pid_controller_t pid_distance;  // 位移环
    pid_controller_t pid_speed;     // 速度环
    pid_controller_t pid_lqr_u;     // LQR输出处理
    pid_controller_t pid_yaw_angle; // 偏航角环
    pid_controller_t pid_yaw_gyro;  // 偏航角速度环
    pid_controller_t pid_roll;      // 横滚环
    pid_controller_t pid_zeropoint; // 零点调整环
    
    // 低通滤波器
    lowpass_filter_t lpf_joyy;      // 速度输入滤波
    lowpass_filter_t lpf_zeropoint; // 零点滤波
    lowpass_filter_t lpf_roll;      // 横滚滤波
    
    // 参数
    lqr_params_t params;
    
    // 内部状态
    float distance_zeropoint;   // 位移零点
    float current_angle_zeropoint;  // 当前角度零点
    float yaw_angle_target;     // YAW 目标角度 (用于方向保持)
    bool yaw_holding;           // 是否正在保持方向
    float lqr_distance;         // 累积位移
    float prev_left_wheel_pos;  // 上次左轮位置
    float prev_right_wheel_pos; // 上次右轮位置
    bool first_run;             // 首次运行标志
    
    // 状态
    lqr_state_t state;
    bool initialized;
} lqr_controller_t;

/**
 * @brief 获取默认 LQR 参数
 * @param params 参数结构体指针
 */
void lqr_get_default_params(lqr_params_t *params);

/**
 * @brief 初始化 LQR 控制器
 * @param ctrl 控制器实例
 * @param params 参数 (NULL 使用默认参数)
 * @return ESP_OK 成功
 */
esp_err_t lqr_init(lqr_controller_t *ctrl, const lqr_params_t *params);

/**
 * @brief 重置 LQR 控制器
 * @param ctrl 控制器实例
 */
void lqr_reset(lqr_controller_t *ctrl);

/**
 * @brief 设置 LQR 参数
 * @param ctrl 控制器实例
 * @param params 新参数
 */
void lqr_set_params(lqr_controller_t *ctrl, const lqr_params_t *params);

/**
 * @brief 设置角度零点
 * @param ctrl 控制器实例
 * @param zeropoint 角度零点 (度)
 */
void lqr_set_angle_zeropoint(lqr_controller_t *ctrl, float zeropoint);

/**
 * @brief 设置速度环 P 增益 (用于自适应控制)
 * @param ctrl 控制器实例
 * @param kp P增益
 */
void lqr_set_speed_kp(lqr_controller_t *ctrl, float kp);

/**
 * @brief 主平衡控制循环
 * @param ctrl 控制器实例
 * @param input 传感器输入
 * @param output 控制输出
 * @return ESP_OK 成功
 */
esp_err_t lqr_balance_loop(lqr_controller_t *ctrl, const lqr_input_t *input, lqr_output_t *output);

/**
 * @brief 偏航控制循环
 * @param ctrl 控制器实例
 * @param input 传感器输入
 * @param output 控制输出
 * @return ESP_OK 成功
 */
esp_err_t lqr_yaw_loop(lqr_controller_t *ctrl, const lqr_input_t *input, lqr_output_t *output);

/**
 * @brief 检查轮子是否离地
 * @param ctrl 控制器实例
 * @param wheel_speed 轮子速度
 * @param wheel_accel 轮子加速度
 * @return true 离地, false 在地面上
 */
bool lqr_check_wheel_off_ground(lqr_controller_t *ctrl, float wheel_speed, float wheel_accel);

/**
 * @brief 检查是否需要紧急停止
 * @param ctrl 控制器实例
 * @param pitch 俯仰角 (度)
 * @return true 需要紧急停止
 */
bool lqr_check_emergency(lqr_controller_t *ctrl, float pitch);

/**
 * @brief 根据腿部高度自适应调整速度P
 * @param ctrl 控制器实例
 * @param leg_height 腿部高度 (米)
 */
void lqr_adaptive_speed_p(lqr_controller_t *ctrl, float leg_height);

/**
 * @brief 获取当前状态
 * @param ctrl 控制器实例
 * @return 当前状态
 */
lqr_state_t lqr_get_state(lqr_controller_t *ctrl);

/**
 * @brief 启动平衡控制
 * @param ctrl 控制器实例
 */
void lqr_start(lqr_controller_t *ctrl);

/**
 * @brief 停止平衡控制
 * @param ctrl 控制器实例
 */
void lqr_stop(lqr_controller_t *ctrl);

/**
 * @brief 设置环路使能系数
 * @param ctrl 控制器实例
 * @param angle_en 角度环使能 (0.0~1.0)
 * @param gyro_en 角速度环使能 (0.0~1.0)
 * @param distance_en 位移环使能 (0.0~1.0)
 * @param speed_en 速度环使能 (0.0~1.0)
 * @param lqr_u_en LQR输出PID使能 (0.0~1.0)
 * @note 使能系数可用于：
 *       - 0.0: 完全禁用该环路
 *       - 1.0: 完全启用该环路
 *       - 0.0~1.0: 平滑过渡 (可用于渐变切换控制模式)
 */
void lqr_set_loop_enable(lqr_controller_t *ctrl, 
                         float angle_en, float gyro_en, 
                         float distance_en, float speed_en, 
                         float lqr_u_en);

/**
 * @brief 设置简单平衡模式 (仅角度+角速度环)
 * @param ctrl 控制器实例
 * @note 等效于: angle=1, gyro=1, distance=0, speed=0, lqr_u=0
 */
void lqr_set_simple_balance_mode(lqr_controller_t *ctrl);

/**
 * @brief 设置完整平衡模式 (所有环路启用)
 * @param ctrl 控制器实例
 * @note 等效于: angle=1, gyro=1, distance=1, speed=1, lqr_u=1
 */
void lqr_set_full_balance_mode(lqr_controller_t *ctrl);

/**
 * @brief Roll 控制循环 (用于腿长控制)
 * @param ctrl 控制器实例
 * @param input 传感器输入 (主要使用 roll, roll_rate)
 * @param roll_output Roll 控制输出
 * @return ESP_OK 成功
 * 
 * @note Roll 平衡是通过调整左右腿长实现的:
 *       - 机器人向右倾斜 (roll > 0) -> 左腿伸长, 右腿缩短
 *       - 机器人向左倾斜 (roll < 0) -> 左腿缩短, 右腿伸长
 * 
 * @example
 *   lqr_roll_output_t roll_out;
 *   lqr_roll_loop(&ctrl, &input, &roll_out);
 *   
 *   // 使用输出控制腿长
 *   float base_leg_length = 0.2f;
 *   float left_leg_target = base_leg_length + roll_out.left_leg_delta;
 *   float right_leg_target = base_leg_length + roll_out.right_leg_delta;
 */
esp_err_t lqr_roll_loop(lqr_controller_t *ctrl, const lqr_input_t *input, lqr_roll_output_t *roll_output);

/**
 * @brief 重置 Roll 控制器
 * @param ctrl 控制器实例
 * @note 重置 Roll PID 和滤波器，在腿长控制启用/禁用切换时调用
 */
void lqr_roll_reset(lqr_controller_t *ctrl);

/**
 * @brief 设置位移零点
 * @param ctrl 控制器实例
 * @param zeropoint 位移零点 (弧度)
 */
void lqr_set_distance_zeropoint(lqr_controller_t *ctrl, float zeropoint);

/**
 * @brief 获取位移零点
 * @param ctrl 控制器实例
 * @return 位移零点 (弧度)
 */
float lqr_get_distance_zeropoint(lqr_controller_t *ctrl);

/**
 * @brief YAW 角度过零处理累加
 * @param current_yaw 当前 YAW 角度 (度, 范围通常 -180~180 或 0~360)
 * @param last_yaw 上一次 YAW 角度 (度)
 * @param yaw_total 累积 YAW 角度 (会被更新)
 * @return 本次 YAW 增量 (度)
 * 
 * @note 原理: 当 YAW 从 +180° 跳到 -180°（或反向）时，直接相减会产生 360° 的跳变。
 *       此函数选择绝对值较小的增量来避免过零跳变。
 * 
 * @example
 *   static float g_yaw_last = 0;
 *   static float g_yaw_total = 0;
 *   
 *   // 每次 IMU 更新时调用
 *   float delta = lqr_yaw_angle_addup(imu.yaw, g_yaw_last, &g_yaw_total);
 *   g_yaw_last = imu.yaw;
 */
float lqr_yaw_angle_addup(float current_yaw, float last_yaw, float *yaw_total);

#ifdef __cplusplus
}
#endif

#endif // LQR_BALANCE_H
