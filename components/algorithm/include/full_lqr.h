/**
 * @file full_lqr.h
 * @brief 完整 LQR 平衡控制器 (同时输出轮子扭矩 T 和腿部摆动扭矩 Tp)
 * @author Bubble
 * @date 2026-03-14
 * 
 * @note 参考 DM-balance 五连杆轮足项目的 LQR 控制方法
 * 
 * 状态向量 x = [theta, d_theta, x, v, pitch, pitch_rate]
 *   - theta:     腿部摆角 (rad), theta = pitch + body_angle + 90°
 *   - d_theta:   腿部摆角速度 (rad/s), d_theta = pitch_rate + d_alpha
 *   - x:         机器人位移 (m), 前进为正
 *   - v:         机器人速度 (m/s), 前进为正  
 *   - pitch:     机身俯仰角 (rad), 前倾为正
 *   - pitch_rate:机身俯仰角速度 (rad/s)
 * 
 * 输出:
 *   T  = K[0..5]  × (x - x_ref)   轮子扭矩 (Nm)
 *   Tp = K[6..11] × (x - x_ref)   腿部摆动扭矩 (Nm), 通过 VMC J^T 转换到关节
 * 
 * K 增益随腿长 L0 变化, 用三次多项式拟合:
 *   K_i(L0) = c0*L0^3 + c1*L0^2 + c2*L0 + c3
 */

#ifndef FULL_LQR_H
#define FULL_LQR_H

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/** @brief 状态数量 */
#define FULL_LQR_STATE_DIM   6

/** @brief K 增益数量 (2 输出 × 6 状态) */
#define FULL_LQR_K_DIM       12

/** @brief 多项式系数阶数 (三次多项式, 4个系数) */
#define FULL_LQR_POLY_ORDER  4

/**
 * @brief Full LQR 参数结构体
 */
typedef struct {
    // 三次多项式拟合系数: K_i(L0) = c[0]*L0^3 + c[1]*L0^2 + c[2]*L0 + c[3]
    // poly_coeff[i][j]: 第 i 个增益的第 j 个系数
    float poly_coeff[FULL_LQR_K_DIM][FULL_LQR_POLY_ORDER];
    
    // Pitch 零点偏移 (rad), 用于补偿 IMU 安装角度
    float pitch_offset;
    
    // 目标速度缩放 (参考代码用 0.4)
    float v_set_scale;
    
    // 输出限幅
    float max_wheel_torque;     // 轮子扭矩限幅 (Nm)
    float max_leg_torque;       // 腿部摆动扭矩 Tp 限幅 (Nm)
    
    // 防劈叉 PD 参数
    float split_kp;             // 防劈叉P
    float split_kd;             // 防劈叉D
    float split_limit;          // 防劈叉限幅
    
    // 转向 PD 参数
    float turn_kp;              // 转向P
    float turn_kd;              // 转向D
    float turn_limit;           // 转向限幅
    
    // 安全阈值
    float emergency_angle;      // 紧急停止角度 (度)
    
    // 低通滤波器时间常数
    float lpf_speed_tf;         // 速度滤波 (秒)
    
    // 离地时仅保留 theta + d_theta
    bool enable_off_ground_protect;
    
} full_lqr_params_t;

/**
 * @brief Full LQR 输入结构体
 */
typedef struct {
    // 腿部状态 (来自 VMC/FK)
    float theta;                // 腿部摆角 (rad): pitch + body_angle + 90°
    float d_theta;              // 腿部摆角速度 (rad/s): pitch_rate + d_alpha
    float L0;                   // 当前腿长 (m), 用于插值 K 增益
    
    // 机器人状态 (来自观测器或编码器)
    float x;                    // 位移 (m), 前进为正
    float v;                    // 速度 (m/s), 前进为正
    
    // IMU 数据
    float pitch;                // 俯仰角 (rad)
    float pitch_rate;           // 俯仰角速度 (rad/s)
    float yaw_total;            // 累积偏航角 (rad)
    float yaw_rate;             // 偏航角速度 (rad/s)
    
    // 左右腿角度差 (用于防劈叉)
    float theta_left;           // 左腿 theta (rad)
    float theta_right;          // 右腿 theta (rad)
    
    // 控制目标
    float v_set;                // 目标速度 (m/s)
    float x_set;                // 目标位移 (m)
    float turn_set;             // 目标偏航角 (rad), 用于方向保持
    float theta_set;            // theta 偏移 (rad), 用于腿部姿态调整
    
    // 离地标志
    bool off_ground;            // 轮子离地
    
    // 时间步长
    float dt;                   // 控制周期 (秒)
    
    // 选择用于哪条腿 (左/右有不同符号)
    bool is_left;               // true=左腿, false=右腿
} full_lqr_input_t;

/**
 * @brief Full LQR 输出结构体
 */
typedef struct {
    float wheel_torque;         // 轮子扭矩 T (Nm)
    float leg_torque;           // 腿部摆动扭矩 Tp (Nm), 需通过 J^T 转换到关节
    float turn_torque;          // 转向差速扭矩 (Nm)
    float split_comp;           // 防劈叉补偿 (Nm)
    
    // 调试: 实时 K 增益 (插值后)
    float K[FULL_LQR_K_DIM];   // 12 个增益值
    
    // 调试: 各状态分量贡献
    float state_contrib[FULL_LQR_STATE_DIM]; // 各状态对 T 的贡献
} full_lqr_output_t;

/**
 * @brief Full LQR 控制器结构体
 */
typedef struct {
    full_lqr_params_t params;
    
    // 内部状态
    float x_filter;             // 位移滤波值
    float x_set;                // 当前位移目标
    float turn_set;             // 当前转向目标 (rad)
    float K[FULL_LQR_K_DIM];   // 当前 K 增益 (上一次计算)
    
    bool initialized;
} full_lqr_controller_t;

/**
 * @brief 获取默认 Full LQR 参数
 * @param params 参数结构体指针
 */
void full_lqr_get_default_params(full_lqr_params_t *params);

/**
 * @brief 初始化 Full LQR 控制器
 * @param ctrl 控制器实例
 * @param params 参数 (NULL 使用默认参数)
 * @return ESP_OK 成功
 */
esp_err_t full_lqr_init(full_lqr_controller_t *ctrl, const full_lqr_params_t *params);

/**
 * @brief 重置 Full LQR 控制器
 * @param ctrl 控制器实例
 */
void full_lqr_reset(full_lqr_controller_t *ctrl);

/**
 * @brief 设置参数
 * @param ctrl 控制器实例
 * @param params 新参数
 */
void full_lqr_set_params(full_lqr_controller_t *ctrl, const full_lqr_params_t *params);

/**
 * @brief 根据腿长 L0 计算多项式插值的 K 增益
 * @param coeff 多项式系数 [c0, c1, c2, c3]
 * @param L0 腿长 (m)
 * @return K 增益值
 */
float full_lqr_poly_eval(const float *coeff, float L0);

/**
 * @brief 主控制循环 - 计算单腿的 T 和 Tp
 * @param ctrl 控制器实例
 * @param input 输入
 * @param output 输出 (T, Tp, 转向, 防劈叉)
 * @return ESP_OK 成功
 * 
 * @note 对于左右两条腿, 分别调用此函数:
 *       - 左腿: input.is_left = true
 *       - 右腿: input.is_left = false
 *       各腿的 theta, d_theta 来自 VMC FK,
 *       x, v 是共享的 (两腿使用同一个观测器/编码器值)
 * 
 * 输出解释:
 *   wheel_torque: 轮子扭矩, 直接发给轮毂电机
 *   leg_torque:   Tp, 需与 F0(腿长PD+重力补偿) 一起通过 VMC J^T 转换:
 *                 τ_hip  = J[0]*F0 + J[2]*Tp
 *                 τ_knee = J[1]*F0 + J[3]*Tp
 */
esp_err_t full_lqr_compute(full_lqr_controller_t *ctrl,
                            const full_lqr_input_t *input,
                            full_lqr_output_t *output);

#ifdef __cplusplus
}
#endif

#endif // FULL_LQR_H
