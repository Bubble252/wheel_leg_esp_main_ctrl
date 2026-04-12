/**
 * @file full_lqr.c
 * @brief 完整 LQR 平衡控制器实现 (同时输出轮子扭矩 T 和腿部摆动扭矩 Tp)
 * @author Bubble
 * @date 2026-03-14
 * 
 * @note 参考 DM-balance 五连杆轮足项目的 LQR 控制方法
 * 
 * 与简易 LQR 的主要区别:
 *   1. 状态向量 6 维: [theta, d_theta, x, v, phi, phi_rate]
 *   2. 同时输出轮子扭矩 T 和腿部摆动扭矩 Tp (不再仅输出 T)
 *   3. K 增益根据腿长 L0 实时插值 (三次多项式拟合)
 *   4. Tp 通过 VMC 的 J^T 转换为 hip/knee 关节扭矩
 * 
 * 统一约定 (左右腿完全相同, 不再区分左右):
 *   theta   = 90° + alpha + pitch
 *   d_theta = pitch_rate + d_alpha
 *   phi     = -pitch
 *   phi_rate= -pitch_rate
 *   x, v:   前进为正
 * 
 * 控制律: u = -K * x  (标准 LQR)
 */

#include "full_lqr.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "FULL_LQR";

// ============================================================================
// 默认多项式拟合系数 (来自 get_k.m 的新系数)
// K_i(L0) = c[0]*L0^3 + c[1]*L0^2 + c[2]*L0 + c[3]
// K[0..5]  -> 轮子扭矩 T 的 6 个增益
// K[6..11] -> 腿部摆动扭矩 Tp 的 6 个增益
// ============================================================================
static const float DEFAULT_POLY_COEFF[FULL_LQR_K_DIM][FULL_LQR_POLY_ORDER] = {
    /* K[0] */ { -39.7608f,  32.9933f, -11.4248f, -0.0230f },
    /* K[1] */ {   8.3684f,  -2.3621f,  -0.6878f, -0.0016f },
    /* K[2] */ { -85.6693f,  40.7763f,  -7.0582f, -0.0254f },
    /* K[3] */ { -65.2460f,  32.8343f,  -6.2390f, -0.0410f },
    /* K[4] */ { 312.1315f, -46.9570f,  -9.3711f,  2.3798f },
    /* K[5] */ {  15.5784f,  -1.9285f,  -0.6498f,  0.1500f },

    /* K[6] */ {1005.1398f,-303.6799f,  24.4459f,  1.7229f },
    /* K[7] */ {  67.1922f, -22.0253f,   2.3310f,  0.1789f },
    /* K[8] */ { 312.1315f, -46.9570f,  -9.3711f,  2.3798f },
    /* K[9] */ { 209.5264f, -20.1930f, -10.8093f,  2.3252f },
    /* K[10]*/ {1713.3859f,-815.5264f, 141.1633f,  0.5080f },
    /* K[11]*/ { 113.5664f, -52.6891f,   8.9861f, -0.0344f },
};

// ============================================================================
// 默认参数
// ============================================================================
void full_lqr_get_default_params(full_lqr_params_t *params) {
    if (params == NULL) return;
    
    memset(params, 0, sizeof(full_lqr_params_t));
    
    // 多项式系数
    memcpy(params->poly_coeff, DEFAULT_POLY_COEFF, sizeof(DEFAULT_POLY_COEFF));
    
    // Pitch 零点偏移 (参考代码 0.04 rad ≈ 2.3°)
    params->pitch_offset = 0.04f;
    
    // 目标速度缩放 (参考代码: v_set * 0.4)
    params->v_set_scale = 0.4f;
    
    // 输出限幅
    params->max_wheel_torque = 0.3f;    // 轮子扭矩限幅 (轮电机实际极限 0.3 Nm)
    params->max_leg_torque = 1.6f;      // 腿部摆动扭矩 Tp 限幅 (hip 极限, J[2]=1)
    
    // 防劈叉 PD 参数 (对应参考代码 Tp_Pid)
    params->split_kp = 0.0f;
    params->split_kd = 0.0f;
    params->split_limit = 2.0f;
    
    // 转向 PD 参数 (对应参考代码 Turn_Pid)
    params->turn_kp = 0.0f;
    params->turn_kd = 0.0f;
    params->turn_limit = 1.0f;
    
    // 安全阈值
    params->emergency_angle = 45.0f;
    
    // 低通滤波器
    params->lpf_speed_tf = 0.01f;
    
    // 离地保护
    params->enable_off_ground_protect = true;
}

// ============================================================================
// 初始化/重置
// ============================================================================
esp_err_t full_lqr_init(full_lqr_controller_t *ctrl, const full_lqr_params_t *params) {
    if (ctrl == NULL) return ESP_ERR_INVALID_ARG;
    
    memset(ctrl, 0, sizeof(full_lqr_controller_t));
    
    if (params != NULL) {
        memcpy(&ctrl->params, params, sizeof(full_lqr_params_t));
    } else {
        full_lqr_get_default_params(&ctrl->params);
    }
    
    ctrl->initialized = true;
    
    ESP_LOGI(TAG, "Full LQR initialized: max_T=%.1f max_Tp=%.1f pitch_offset=%.3f v_scale=%.2f",
             ctrl->params.max_wheel_torque, ctrl->params.max_leg_torque,
             ctrl->params.pitch_offset, ctrl->params.v_set_scale);
    
    return ESP_OK;
}

void full_lqr_reset(full_lqr_controller_t *ctrl) {
    if (ctrl == NULL) return;
    
    ctrl->x_filter = 0.0f;
    ctrl->x_set = 0.0f;
    ctrl->turn_set = 0.0f;
    memset(ctrl->K, 0, sizeof(ctrl->K));
}

void full_lqr_set_params(full_lqr_controller_t *ctrl, const full_lqr_params_t *params) {
    if (ctrl == NULL || params == NULL) return;
    memcpy(&ctrl->params, params, sizeof(full_lqr_params_t));
}

// ============================================================================
// 三次多项式求值: K = c[0]*L^3 + c[1]*L^2 + c[2]*L + c[3]
// ============================================================================
float full_lqr_poly_eval(const float *coeff, float L0) {
    // Horner's method: ((c[0]*L + c[1])*L + c[2])*L + c[3]
    float result = coeff[0];
    result = result * L0 + coeff[1];
    result = result * L0 + coeff[2];
    result = result * L0 + coeff[3];
    return result;
}

// ============================================================================
// 辅助: 限幅
// ============================================================================
static inline float clamp(float val, float min_val, float max_val) {
    if (val < min_val) return min_val;
    if (val > max_val) return max_val;
    return val;
}

// ============================================================================
// 主控制循环
// ============================================================================
esp_err_t full_lqr_compute(full_lqr_controller_t *ctrl,
                            const full_lqr_input_t *input,
                            full_lqr_output_t *output) {
    if (ctrl == NULL || input == NULL || output == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!ctrl->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    memset(output, 0, sizeof(full_lqr_output_t));
    
    // === 1. 根据腿长 L0 插值 K 增益 ===
    float L0 = input->L0;
    // 安全限幅 (避免极端值导致 K 增益爆炸)
    L0 = clamp(L0, 0.07f, 0.20f);
    
    for (int i = 0; i < FULL_LQR_K_DIM; i++) {
        ctrl->K[i] = full_lqr_poly_eval(ctrl->params.poly_coeff[i], L0);
        output->K[i] = ctrl->K[i];
    }
    
    // === 2. 构建状态向量 ===
    // 统一约定 (左右腿完全相同, 不再区分 is_left):
    //   theta   = 90° + alpha + pitch  (调用前已计算好, 传入 input->theta)
    //   d_theta = pitch_rate + d_alpha  (调用前已计算好, 传入 input->d_theta)
    //   phi     = -pitch               (调用前已计算好, 传入 input->pitch = -IMU_pitch)
    //   phi_rate= -pitch_rate          (调用前已计算好, 传入 input->pitch_rate = -IMU_pitch_rate)
    //   x, v:   前进为正 (统一)
    //
    // 控制律: u = -K * x  (标准 LQR)
    //   T  = -(K[0]*theta + K[1]*d_theta + K[2]*(x-x_set) + K[3]*(v-v_scale*v_set) + K[4]*phi + K[5]*phi_rate)
    //   Tp = -(K[6]*theta + K[7]*d_theta + K[8]*(x-x_set) + K[9]*(v-v_scale*v_set) + K[10]*phi + K[11]*phi_rate)
    
    float theta = input->theta;             // 腿部摆角 (rad): 90° + alpha + pitch
    float d_theta = input->d_theta;         // 腿部摆角速度 (rad/s): pitch_rate + d_alpha
    float phi = input->pitch;               // 机身俯仰 (rad): -IMU_pitch (已由调用者取反)
    float phi_rate = input->pitch_rate;     // 机身俯仰角速度 (rad/s): -IMU_pitch_rate (已由调用者取反)
    float pitch_offset = ctrl->params.pitch_offset;
    float v_scale = ctrl->params.v_set_scale;
    
    // 状态误差
    float x_err = input->x - input->x_set;
    float v_err = input->v - v_scale * input->v_set;
    float phi_err = phi - pitch_offset;
    float phi_rate_err = phi_rate;
    
    // === 3. 计算轮子扭矩 T = -(K[0..5] × state) ===
    float T = -(ctrl->K[0] * theta
              + ctrl->K[1] * d_theta
              + ctrl->K[2] * x_err
              + ctrl->K[3] * v_err
              + ctrl->K[4] * phi_err
              + ctrl->K[5] * phi_rate_err);
    
    // 保存各分量用于调试 (带负号)
    output->state_contrib[0] = -(ctrl->K[0] * theta);
    output->state_contrib[1] = -(ctrl->K[1] * d_theta);
    output->state_contrib[2] = -(ctrl->K[2] * x_err);
    output->state_contrib[3] = -(ctrl->K[3] * v_err);
    output->state_contrib[4] = -(ctrl->K[4] * phi_err);
    output->state_contrib[5] = -(ctrl->K[5] * phi_rate_err);
    
    // === 4. 计算腿部摆动扭矩 Tp = -(K[6..11] × state) ===
    float Tp = -(ctrl->K[6] * (theta + input->theta_set)
               + ctrl->K[7] * d_theta
               + ctrl->K[8] * x_err
               + ctrl->K[9] * v_err
               + ctrl->K[10] * phi_err
               + ctrl->K[11] * phi_rate_err);
    
    // === 5. 防劈叉补偿 (PD 控制) ===
    // theta_err = left.theta - right.theta
    // 当 theta_err > 0 时 (左腿偏后), 两腿 Tp 都加上补偿
    float theta_err = input->theta_left - input->theta_right;
    float split_comp = ctrl->params.split_kp * theta_err;
    split_comp = clamp(split_comp, -ctrl->params.split_limit, ctrl->params.split_limit);
    
    // 防劈叉补偿加到 Tp 上
    Tp += split_comp;
    
    // === 6. 转向差速 ===
    // 参考: turn_T = Kp * (turn_set - yaw) - Kd * yaw_rate
    float turn_err = input->turn_set - input->yaw_total;
    float turn_torque = ctrl->params.turn_kp * turn_err - ctrl->params.turn_kd * input->yaw_rate;
    turn_torque = clamp(turn_torque, -ctrl->params.turn_limit, ctrl->params.turn_limit);
    
    // 转向叠加到轮子扭矩 (差速):
    // 参考: wheel_T = wheel_T - turn_T (右轮)
    //        wheel_T = wheel_T - turn_T (左轮)  ← 参考代码左右都减
    // 在我们的项目中, 正值 turn_torque 应使机器人右转:
    //   left_torque  += turn_torque
    //   right_torque -= turn_torque
    // 这里先只输出 turn_torque, 在上层做差速
    
    // === 7. 离地保护 ===
    if (input->off_ground && ctrl->params.enable_off_ground_protect) {
        // 离地时: 清零 x/v 分量, 仅保留 theta + d_theta
        T = -(ctrl->K[0] * theta + ctrl->K[1] * d_theta);
        Tp = -(ctrl->K[6] * theta + ctrl->K[7] * d_theta) + split_comp;
        // 清零轮子扭矩 (参考代码离地时 wheel_T = 0)
        T = 0.0f;
    }
    
    // === 8. 限幅 ===
    T = clamp(T, -ctrl->params.max_wheel_torque, ctrl->params.max_wheel_torque);
    Tp = clamp(Tp, -ctrl->params.max_leg_torque, ctrl->params.max_leg_torque);
    
    // === 9. 输出 ===
    output->wheel_torque = T;
    output->leg_torque = Tp;
    output->turn_torque = turn_torque;
    output->split_comp = split_comp;
    
    return ESP_OK;
}
