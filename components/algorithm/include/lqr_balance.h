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
    float lpf_gyro_tf;      // 角速度滤波 (默认 0.005, 消除IMU阶梯跳变)
    float lpf_speed_tf;     // 轮速滤波 (默认 0.01, 消除编码器噪声)
    
    // 角速度滤波模式
    uint8_t gyro_filter_mode;   // 0=LPF, 1=限幅滤波 (slew-rate)
    float gyro_slew_rate;       // 限幅滤波最大变化率 (度/秒²，默认 500.0)
    
    // 轮速滤波模式
    uint8_t speed_filter_mode;  // 0=LPF, 1=限幅滤波 (slew-rate)
    float speed_slew_rate;      // 限幅滤波最大变化率 (单位/秒, 默认 50.0)
    
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
    float pitch;            // 俯仰角 (度) - 可能经过腿部补偿，用于平衡控制
    float pitch_rate;       // 俯仰角速度 (度/秒)
    float roll;             // 横滚角 (度)
    float roll_rate;        // 横滚角速度 (度/秒)
    float yaw;              // 偏航角 (度)
    float yaw_rate;         // 偏航角速度 (度/秒)
    float raw_pitch;        // IMU 原始俯仰角 (度) - 用于紧急停止判断，不受腿部补偿影响
    
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
    float lqr_u;                // LQR 综合输出 (经 pid_lqr_u 处理后)
    float lqr_u_raw;            // LQR 原始输出 (pid_lqr_u 处理前)
    float yaw_control;          // 偏航控制量
    float roll_control;         // 横滚控制量 (用于腿长控制)
    
    // 滤波后的值 (调试用)
    float filtered_target_speed; // 滤波后的目标速度
    float zeropoint_adjust_raw;  // 零点调整原始值 (滤波前)
    float zeropoint_adjust_filtered; // 零点调整滤波后值
    float filtered_gyro;         // 滤波后的角速度
    float filtered_speed;        // 滤波后的轮速
    
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
    lowpass_filter_t lpf_gyro;      // 角速度滤波
    lowpass_filter_t lpf_speed;     // 轮速滤波
    
    // 限幅滤波器 (slew-rate limiter)
    slewrate_filter_t sr_gyro;      // 角速度限幅滤波
    slewrate_filter_t sr_speed;     // 轮速限幅滤波
    
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
 * @brief 角度零点自动调整 (重心补偿, 所有模式通用)
 * @param ctrl 控制器实例 (使用其 pid_zeropoint 和 lpf_zeropoint)
 * @param angle_error 当前角度误差 = pitch - angle_zeropoint (度)
 * @param wheel_speed 当前轮速 (m/s 或 rad/s, 用于判断是否静止)
 * @param speed_threshold 轮速阈值, 低于此值才启用自动调整 (默认 0.1)
 * @param dt 时间步长 (秒)
 * @param out_raw [out] 零点调整原始值 (可为NULL)
 * @param out_filtered [out] 零点调整滤波后值 (可为NULL)
 * @return 本次角度零点增量 (度), 调用者应累加到 angle_zeropoint
 * 
 * @note 原理: 当机器人静止时, 如果角度持续偏离零点, 说明重心不在正上方.
 *       通过缓慢调整 angle_zeropoint 使得平均角度误差趋向零.
 *       使用 zeropoint_kp/ki/kd PID + lpf_zeropoint 低通滤波, 
 *       通过 Commander ID='I' 或 CLI 可调参.
 *       仅在轮速 < speed_threshold 时启用 (静止或低速), 避免运动中干扰.
 */
float lqr_zeropoint_auto_adjust(lqr_controller_t *ctrl, float angle_error,
                                 float wheel_speed, float speed_threshold,
                                 float dt,
                                 float *out_raw, float *out_filtered);

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

// ============================================================================
// 双环 PID 控制器 (直立环 + 速度环)
// ============================================================================

/**
 * @brief 双环PID环路顺序
 */
#define DUAL_PID_ANGLE_FIRST  0   // 角度优先: 角度环(外)→速度环(内) → 输出扭矩
#define DUAL_PID_SPEED_FIRST  1   // 速度优先: 速度环(外)→角度环(内) → 输出扭矩

/**
 * @brief 双环PID参数结构体
 * 
 * 控制架构 (取决于 loop_order):
 * 
 *   ANGLE_FIRST (默认):
 *     目标角度(0°) → [角度环PID] → 目标速度 → [速度环PID] → 输出扭矩
 *         ↑                              ↑
 *     实际Pitch                      实际轮速
 * 
 *   SPEED_FIRST (经典串级):
 *     目标速度(0) → [速度环PID] → 目标倾角 → [角度环PID] → 输出扭矩
 *         ↑                              ↑
 *     实际轮速                       实际Pitch
 */
typedef struct {
    // 角度环 PID
    //   ANGLE_FIRST 时为外环: pitch → target_speed
    //   SPEED_FIRST 时为内环: pitch_target - pitch → torque
    float angle_kp;         // 角度P (默认 1.5)
    float angle_ki;         // 角度I (默认 0.0)
    float angle_kd;         // 角度D (默认 0.3)
    float angle_limit;      // 输出限幅 (默认 20.0)
    
    // 速度环 PID
    //   ANGLE_FIRST 时为内环: speed_error → torque
    //   SPEED_FIRST 时为外环: 0 - wheel_speed → pitch_target
    float speed_kp;         // 速度P (默认 0.4)
    float speed_ki;         // 速度I (默认 0.05)
    float speed_kd;         // 速度D (默认 0.0)
    float speed_limit;      // 输出限幅 (默认 12.0)
    
    // 角度零点
    float angle_zeropoint;  // 机械零点偏移 (默认 0.0)
    
    // 安全阈值
    float emergency_angle;  // 紧急停止角度 (默认 45.0)
    
    // 输出限幅
    float max_torque;       // 最大输出扭矩 (默认 8.0)
    
    // 速度指令增益 (仅 SPEED_FIRST 模式外环生效)
    // target_speed_amplified = target_speed * speed_cmd_gain
    // 用于将小量级的遥杆映射值放大到速度环可感知的量级
    float speed_cmd_gain;   // 默认 33333.0 (补偿 speed_kp 极小值)
    
    // 遥杆目标速度低通滤波 (与 LQR 的 lpf_joyy 相同)
    float lpf_joyy_tf;     // 滤波时间常数 (默认 0.2)
    
    // 角速度阻尼环 PID (叠加到角度环输出，参考 LQR 的 gyro_control)
    // gyro_control = pid_gyro(0, pitch_rate) → 与角度环输出相加
    float gyro_kp;          // 角速度P (默认 0.0, 即关闭)
    float gyro_ki;          // 角速度I (默认 0.0)
    float gyro_kd;          // 角速度D (默认 0.0)
    float gyro_limit;       // 角速度环输出限幅 (默认 10.0)
    
    // 环路顺序
    uint8_t loop_order;     // DUAL_PID_ANGLE_FIRST(0) 或 DUAL_PID_SPEED_FIRST(1)
} dual_pid_params_t;

/**
 * @brief 双环PID控制器输出结构体
 */
typedef struct {
    float angle_error;      // 角度误差 (目标-实际)
    float target_speed;     // 直立环输出的目标速度
    float speed_error;      // 速度误差 (目标-实际)
    float torque;           // 最终输出扭矩
    
    // 用于调试的中间量
    float angle_p_out;      // 角度环P输出
    float angle_i_out;      // 角度环I输出
    float angle_d_out;      // 角度环D输出
    float speed_p_out;      // 速度环P输出
    float speed_i_out;      // 速度环I输出
    float speed_d_out;      // 速度环D输出
    float gyro_p_out;       // 角速度阻尼P输出
    float gyro_i_out;       // 角速度阻尼I输出
    float gyro_d_out;       // 角速度阻尼D输出
    float gyro_control;     // 角速度阻尼总输出
    
    bool emergency;         // 是否紧急停止
} dual_pid_output_t;

/**
 * @brief 双环PID控制器结构体
 */
typedef struct {
    pid_controller_t pid_angle;     // 直立环 (外环)
    pid_controller_t pid_speed;     // 速度环 (内环)
    pid_controller_t pid_gyro;      // 角速度阻尼环 (叠加到角度环输出)
    lowpass_filter_t lpf_joyy;      // 遥杆目标速度低通滤波
    
    dual_pid_params_t params;
    
    bool initialized;
} dual_pid_controller_t;

/**
 * @brief 获取双环PID默认参数
 * @param params 参数结构体指针
 */
void dual_pid_get_default_params(dual_pid_params_t *params);

/**
 * @brief 初始化双环PID控制器
 * @param ctrl 控制器实例
 * @param params 参数 (NULL 使用默认参数)
 * @return ESP_OK 成功
 */
esp_err_t dual_pid_init(dual_pid_controller_t *ctrl, const dual_pid_params_t *params);

/**
 * @brief 重置双环PID控制器
 * @param ctrl 控制器实例
 */
void dual_pid_reset(dual_pid_controller_t *ctrl);

/**
 * @brief 设置双环PID参数
 * @param ctrl 控制器实例
 * @param params 新参数
 */
void dual_pid_set_params(dual_pid_controller_t *ctrl, const dual_pid_params_t *params);

/**
 * @brief 设置直立环PID增益
 * @param ctrl 控制器实例
 * @param kp P增益
 * @param ki I增益
 * @param kd D增益
 */
void dual_pid_set_angle_gains(dual_pid_controller_t *ctrl, float kp, float ki, float kd);

/**
 * @brief 设置速度环PID增益
 * @param ctrl 控制器实例
 * @param kp P增益
 * @param ki I增益
 * @param kd D增益
 */
void dual_pid_set_speed_gains(dual_pid_controller_t *ctrl, float kp, float ki, float kd);

/**
 * @brief 设置角速度阻尼环PID增益
 * @param ctrl 控制器实例
 * @param kp P增益
 * @param ki I增益
 * @param kd D增益
 */
void dual_pid_set_gyro_gains(dual_pid_controller_t *ctrl, float kp, float ki, float kd);

/**
 * @brief 设置角度零点
 * @param ctrl 控制器实例
 * @param zeropoint 角度零点 (度)
 */
void dual_pid_set_angle_zeropoint(dual_pid_controller_t *ctrl, float zeropoint);

/**
 * @brief 设置环路顺序
 * @param ctrl 控制器实例
 * @param loop_order DUAL_PID_ANGLE_FIRST(0) 或 DUAL_PID_SPEED_FIRST(1)
 */
void dual_pid_set_loop_order(dual_pid_controller_t *ctrl, uint8_t loop_order);

/**
 * @brief 双环PID平衡控制循环
 * @param ctrl 控制器实例
 * @param pitch 当前俯仰角 (度)
 * @param pitch_rate 当前俯仰角速度 (度/秒) - 可用于D项
 * @param wheel_speed 当前轮子速度 (rad/s)
 * @param target_speed 目标速度 (rad/s), 来自遥杆 joy_y, 0 = 原地平衡
 * @param dt 时间步长 (秒)
 * @param output 控制输出
 * @return ESP_OK 成功
 * 
 * @note 控制流程取决于 loop_order:
 *   ANGLE_FIRST:
 *     1. 角度环(外): pitch_error → target_speed
 *     2. 速度环(内): speed_error → torque
 *   SPEED_FIRST:
 *     1. 速度环(外): target_speed - wheel_speed → pitch_target
 *     2. 角度环(内): pitch_target - pitch → torque
 */
esp_err_t dual_pid_balance_loop(dual_pid_controller_t *ctrl, 
                                 float pitch, float pitch_rate,
                                 float wheel_speed, float target_speed,
                                 float dt,
                                 dual_pid_output_t *output);

/**
 * @brief 检查是否触发紧急停止
 * @param ctrl 控制器实例
 * @param pitch 当前俯仰角 (度)
 * @return true 需要紧急停止
 */
bool dual_pid_check_emergency(dual_pid_controller_t *ctrl, float pitch);

// ============================================================================
// 单环 PID 控制器 (直立环 → 目标速度，配合轮电机速度模式)
// ============================================================================

/**
 * @brief 单环PID参数结构体
 * 
 * 控制架构:
 *   目标角度(0°) → [直立环PID] → 目标速度 (送给轮电机速度模式)
 *       ↑
 *   实际Pitch
 * 
 * @note 与双环 PID 区别:
 *   - 双环: 直立环→速度环→扭矩 (适合扭矩模式)
 *   - 单环: 直立环→速度 (适合速度模式，由电机内部速度环闭环)
 */
typedef struct {
    // 直立环 PID: pitch → target_speed
    float angle_kp;         // 角度P (默认 15.0)
    float angle_ki;         // 角度I (默认 0.0)
    float angle_kd;         // 角度D (默认 0.5)
    float angle_limit;      // 输出限幅 - 最大目标速度 (默认 100.0 rad/s)
    
    // 角度零点
    float angle_zeropoint;  // 机械零点偏移 (默认 0.0 度)
    
    // 安全阈值
    float emergency_angle;  // 紧急停止角度 (默认 45.0 度)
} single_pid_params_t;

/**
 * @brief 单环PID控制器输出结构体
 */
typedef struct {
    float angle_error;      // 角度误差 (目标-实际)
    float target_speed;     // 输出目标速度 (rad/s)
    
    // 用于调试的中间量
    float angle_p_out;      // 角度环P输出
    float angle_i_out;      // 角度环I输出
    float angle_d_out;      // 角度环D输出
    
    bool emergency;         // 是否紧急停止
} single_pid_output_t;

/**
 * @brief 单环PID控制器结构体
 */
typedef struct {
    pid_controller_t pid_angle;     // 直立环
    
    single_pid_params_t params;
    
    bool initialized;
} single_pid_controller_t;

/**
 * @brief 获取单环PID默认参数
 * @param params 参数结构体指针
 */
void single_pid_get_default_params(single_pid_params_t *params);

/**
 * @brief 初始化单环PID控制器
 * @param ctrl 控制器实例
 * @param params 参数 (NULL 使用默认)
 * @return ESP_OK 成功
 */
esp_err_t single_pid_init(single_pid_controller_t *ctrl, const single_pid_params_t *params);

/**
 * @brief 重置单环PID控制器
 * @param ctrl 控制器实例
 */
void single_pid_reset(single_pid_controller_t *ctrl);

/**
 * @brief 设置单环PID参数
 * @param ctrl 控制器实例
 * @param params 新参数
 */
void single_pid_set_params(single_pid_controller_t *ctrl, const single_pid_params_t *params);

/**
 * @brief 设置直立环PID增益
 * @param ctrl 控制器实例
 * @param kp P增益
 * @param ki I增益
 * @param kd D增益
 */
void single_pid_set_angle_gains(single_pid_controller_t *ctrl, float kp, float ki, float kd);

/**
 * @brief 设置角度零点
 * @param ctrl 控制器实例
 * @param zeropoint 角度零点 (度)
 */
void single_pid_set_angle_zeropoint(single_pid_controller_t *ctrl, float zeropoint);

/**
 * @brief 单环PID平衡控制循环 (输出速度，适合电机速度模式)
 * @param ctrl 控制器实例
 * @param pitch 当前俯仰角 (度)
 * @param pitch_rate 当前俯仰角速度 (度/秒) - 可用于D项优化
 * @param dt 时间步长 (秒)
 * @param output 控制输出
 * @return ESP_OK 成功
 * 
 * @note 输出 target_speed 直接送给轮电机的速度模式
 */
esp_err_t single_pid_balance_loop(single_pid_controller_t *ctrl, 
                                   float pitch, float pitch_rate,
                                   float dt,
                                   single_pid_output_t *output);

/**
 * @brief 检查是否触发紧急停止
 * @param ctrl 控制器实例
 * @param pitch 当前俯仰角 (度)
 * @return true 需要紧急停止
 */
bool single_pid_check_emergency(single_pid_controller_t *ctrl, float pitch);

// ============================================================================
// 三环 PID 控制器 (速度环→角度环→轮速环)
// ============================================================================

/**
 * @brief 三环PID轮速环输出方式
 */
#define TRIPLE_PID_WHEEL_SPEED  0   // 输出速度命令给电机 (MODE_SPEED, 默认)
#define TRIPLE_PID_WHEEL_TORQUE 1   // 软件PID输出扭矩 (MODE_TORQUE)

/**
 * @brief 三环PID参数结构体
 * 
 * 控制架构 (固定 SPEED_FIRST):
 *   速度环(外): target_speed - wheel_speed → pitch_target
 *   角度环(中): pitch_target - pitch → wheel_speed_target
 *   轮速环(内): wheel_speed_target → torque (软件PID) 或 speed_cmd (电机速度模式)
 * 
 * 前两环参数完全复用 dual_pid_params_t (SPEED_FIRST 模式)
 * 第三环 (轮速环) 有两种工作方式:
 *   TRIPLE_PID_WHEEL_SPEED:  角度环输出+yaw → 直接作为速度命令发给电机 (MODE_SPEED)
 *   TRIPLE_PID_WHEEL_TORQUE: 角度环输出+yaw → 轮速PID → 扭矩命令 (MODE_TORQUE)
 */
typedef struct {
    // ---- 前两环: 复用双环PID参数 (固定 SPEED_FIRST) ----
    // 角度环 PID (中环): pitch_target - pitch → wheel_speed_target
    float angle_kp;
    float angle_ki;
    float angle_kd;
    float angle_limit;      // 角度环输出限幅 (max wheel_speed_target, rad/s)
    
    // 速度环 PID (外环): target_speed - wheel_speed → pitch_target
    float speed_kp;
    float speed_ki;
    float speed_kd;
    float speed_limit;      // 速度环输出限幅 (max pitch_target, deg)
    
    // ---- 第三环: 轮速环 (仅 WHEEL_TORQUE 模式使用) ----
    float wheel_kp;         // 轮速P (默认 0.5)
    float wheel_ki;         // 轮速I (默认 0.01)
    float wheel_kd;         // 轮速D (默认 0.0)
    float wheel_limit;      // 轮速环输出限幅 (max torque, Nm)
    
    // ---- 通用参数 ----
    float angle_zeropoint;  // 角度零点偏移 (度)
    float emergency_angle;  // 紧急停止角度 (默认 45.0)
    float max_torque;       // 最大输出扭矩 (Nm, WHEEL_TORQUE 模式)
    float speed_cmd_gain;   // 速度指令增益 (同 dual_pid)
    float lpf_joyy_tf;     // 遥杆目标速度低通滤波时间常数
    
    // ---- 角速度阻尼环 PID (叠加到角度环输出) ----
    float gyro_kp;          // 角速度P (默认 0.0, 即关闭)
    float gyro_ki;          // 角速度I (默认 0.0)
    float gyro_kd;          // 角速度D (默认 0.0)
    float gyro_limit;       // 角速度环输出限幅 (默认 10.0)
    
    // ---- 轮速环工作模式 ----
    uint8_t wheel_mode;     // TRIPLE_PID_WHEEL_SPEED(0) 或 TRIPLE_PID_WHEEL_TORQUE(1)
} triple_pid_params_t;

/**
 * @brief 三环PID控制器输出结构体
 */
typedef struct {
    // 速度环 (外环)
    float speed_error;      // 速度误差
    float pitch_target;     // 速度环输出: 目标倾角
    float speed_p_out;
    float speed_i_out;
    float speed_d_out;
    
    // 角度环 (中环)
    float angle_error;      // 角度误差
    float wheel_speed_target; // 角度环输出: 目标轮速 (rad/s)
    float angle_p_out;
    float angle_i_out;
    float angle_d_out;
    
    // 角速度阻尼环 (叠加到角度环输出)
    float gyro_p_out;
    float gyro_i_out;
    float gyro_d_out;
    float gyro_control;     // 角速度阻尼总输出
    
    // 轮速环 (内环, 仅 WHEEL_TORQUE 模式有意义)
    float wheel_speed_error; // 轮速误差
    float torque;           // 最终输出扭矩 (WHEEL_TORQUE) 或速度 (WHEEL_SPEED)
    float wheel_p_out;
    float wheel_i_out;
    float wheel_d_out;
    
    bool emergency;
} triple_pid_output_t;

/**
 * @brief 三环PID控制器结构体
 */
typedef struct {
    pid_controller_t pid_angle;     // 角度环 (中环)
    pid_controller_t pid_speed;     // 速度环 (外环)
    pid_controller_t pid_wheel;     // 轮速环 (内环, 仅 WHEEL_TORQUE 模式)
    pid_controller_t pid_gyro;      // 角速度阻尼环 (叠加到角度环输出)
    lowpass_filter_t lpf_joyy;      // 遥杆目标速度低通滤波
    
    triple_pid_params_t params;
    
    bool initialized;
} triple_pid_controller_t;

/** @brief 获取三环PID默认参数 */
void triple_pid_get_default_params(triple_pid_params_t *params);

/** @brief 初始化三环PID控制器 */
esp_err_t triple_pid_init(triple_pid_controller_t *ctrl, const triple_pid_params_t *params);

/** @brief 重置三环PID控制器 */
void triple_pid_reset(triple_pid_controller_t *ctrl);

/** @brief 设置角度环PID增益 */
void triple_pid_set_angle_gains(triple_pid_controller_t *ctrl, float kp, float ki, float kd);

/** @brief 设置速度环PID增益 */
void triple_pid_set_speed_gains(triple_pid_controller_t *ctrl, float kp, float ki, float kd);

/** @brief 设置轮速环PID增益 */
void triple_pid_set_wheel_gains(triple_pid_controller_t *ctrl, float kp, float ki, float kd);

/** @brief 设置角速度阻尼环PID增益 */
void triple_pid_set_gyro_gains(triple_pid_controller_t *ctrl, float kp, float ki, float kd);

/** @brief 设置角度零点 */
void triple_pid_set_angle_zeropoint(triple_pid_controller_t *ctrl, float zeropoint);

/** @brief 设置轮速环工作模式 */
void triple_pid_set_wheel_mode(triple_pid_controller_t *ctrl, uint8_t wheel_mode);

/** @brief 检查是否触发紧急停止 */
bool triple_pid_check_emergency(triple_pid_controller_t *ctrl, float pitch);

/**
 * @brief 三环PID平衡控制循环
 * @param ctrl 控制器实例
 * @param pitch 当前俯仰角 (度)
 * @param pitch_rate 当前俯仰角速度 (度/秒)
 * @param wheel_speed 当前轮子速度 (rad/s)
 * @param target_speed 目标速度 (rad/s), 来自遥杆
 * @param dt 时间步长 (秒)
 * @param output 控制输出
 * @return ESP_OK 成功
 * 
 * @note 控制流程 (固定 SPEED_FIRST):
 *   1. 速度环(外): target_speed - wheel_speed → pitch_target
 *   2. 角度环(中): pitch_target - pitch → wheel_speed_target
 *   3. 轮速环(内): 
 *      WHEEL_SPEED 模式: 直接输出 wheel_speed_target (送电机速度模式)
 *      WHEEL_TORQUE 模式: wheel_speed_target - wheel_speed → torque (送电机扭矩模式)
 */
esp_err_t triple_pid_balance_loop(triple_pid_controller_t *ctrl,
                                   float pitch, float pitch_rate,
                                   float wheel_speed, float target_speed,
                                   float dt,
                                   triple_pid_output_t *output);

#ifdef __cplusplus
}
#endif

#endif // LQR_BALANCE_H
