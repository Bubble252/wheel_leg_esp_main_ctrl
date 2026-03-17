/**
 * @file full_lqr.c
 * @brief 完整 LQR 平衡控制器实现 (同时输出轮子扭矩 T 和腿部摆动扭矩 Tp)
 * @author Bubble
 * @date 2026-03-14
 * 
 * @note 参考 DM-balance 五连杆轮足项目的 LQR 控制方法
 * 
 * 与简易 LQR 的主要区别:
 *   1. 状态向量 6 维: [theta, d_theta, x, v, pitch, pitch_rate]
 *   2. 同时输出轮子扭矩 T 和腿部摆动扭矩 Tp (不再仅输出 T)
 *   3. K 增益根据腿长 L0 实时插值 (三次多项式拟合)
 *   4. Tp 通过 VMC 的 J^T 转换为 hip/knee 关节扭矩
 */

#include "full_lqr.h"
#include "esp_log.h"
#include <string.h>
#include <math.h>

static const char *TAG = "FULL_LQR";

// ============================================================================
// 默认多项式拟合系数 (来自 DM-balance 项目)
// K_i(L0) = c[0]*L0^3 + c[1]*L0^2 + c[2]*L0 + c[3]
// K[0..5]  -> 轮子扭矩 T 的 6 个增益
// K[6..11] -> 腿部摆动扭矩 Tp 的 6 个增益
// ============================================================================
static const float DEFAULT_POLY_COEFF[FULL_LQR_K_DIM][FULL_LQR_POLY_ORDER] = {
    // K[0]: theta 对 T 的增益
    {-213.6885f, 153.3306f, -50.978f, -0.13318f},
    // K[1]: d_theta 对 T 的增益
    {-1.1412f, 1.2471f, -3.633f, 0.056666f},
    // K[2]: x 对 T 的增益
    {-82.3054f, 49.8361f, -10.6676f, -0.73082f},
    // K[3]: v 对 T 的增益
    {-70.3514f, 43.3124f, -10.1995f, -0.64679f},
    // K[4]: pitch 对 T 的增益
    {-246.3632f, 173.9108f, -47.6573f, 6.1294f},
    // K[5]: pitch_rate 对 T 的增益
    {-13.1949f, 10.2265f, -3.1718f, 0.52012f},
    // K[6]: theta 对 Tp 的增益
    {114.4332f, -51.7589f, 2.8343f, 2.599f},
    // K[7]: d_theta 对 Tp 的增益
    {14.4172f, -8.5621f, 1.6232f, 0.13359f},
    // K[8]: x 对 Tp 的增益
    {-154.0047f, 107.3901f, -28.8305f, 3.5029f},
    // K[9]: v 对 Tp 的增益
    {-128.9122f, 90.0203f, -24.2995f, 3.035f},
    // K[10]: pitch 对 Tp 的增益
    {577.6103f, -351.5575f, 75.9638f, 4.2419f},
    // K[11]: pitch_rate 对 Tp 的增益
    {46.4618f, -29.0229f, 6.5446f, 0.061617f},
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
    params->max_wheel_torque = 2.0f;    // 轮子扭矩限幅 (参考代码 2.0 Nm)
    params->max_leg_torque = 8.0f;      // 腿部摆动扭矩 Tp 限幅
    
    // 防劈叉 PD 参数 (对应参考代码 Tp_Pid)
    params->split_kp = 5.0f;
    params->split_kd = 0.2f;
    params->split_limit = 2.0f;
    
    // 转向 PD 参数 (对应参考代码 Turn_Pid)
    params->turn_kp = 0.5f;
    params->turn_kd = 0.01f;
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
    
    // === 2. 构建状态误差向量 ===
    // 参考代码的状态向量: [theta, d_theta, x_err, v_err, pitch_err, pitch_rate]
    //
    // 符号约定 (参考 DM-balance):
    //   - 右腿: x_err = (x - x_set),     v_err = (v - v_scale * v_set)
    //   - 左腿: x_err = (x_set - x),     v_err = (v_scale * v_set - v)
    //   - 右腿: pitch_err = (pitch - offset), pitch_rate 正常
    //   - 左腿: pitch = -IMU_Pitch,           pitch_rate = -IMU_PitchRate
    //
    // 在我们的项目中:
    //   theta/d_theta 已经在调用前按左右腿分别计算:
    //     左腿: theta_L = +Pitch + alpha + 90°,  d_theta_L = +pitch_rate + d_alpha
    //     右腿: theta_R = -Pitch + alpha + 90°,  d_theta_R = -pitch_rate + d_alpha
    //   (对应参考代码 VMC_calc_1_left/right 中的 theta/d_theta)
    //   x, v: 统一的位移/速度 (前进为正)
    //   pitch, pitch_rate: IMU 原始值 （但是需要取反）
    //
    // 参考代码中左右腿的 x/v 和 pitch/pitch_rate 符号相反
    // 这是因为 K 增益是基于右腿推导的,
    // 左腿通过取反实现对称控制
    
    float theta = input->theta;             // 腿部摆角 (rad)
    float d_theta = input->d_theta;         // 腿部摆角速度 (rad/s)
    float pitch_offset = ctrl->params.pitch_offset;
    float v_scale = ctrl->params.v_set_scale;
    
    float x_err, v_err, pitch_err, pitch_rate_err;
    
    if (input->is_left) {
        // 左腿: 反转 x/v 和 pitch/pitch_rate 的符号
        // 参考: chassis->x_set - chassis->x_filter
        // 参考: myPithL = -ins->Pitch, 所以 pitch_err = (-pitch) - (-offset) = -pitch + offset
        x_err = input->x_set - input->x;
        v_err = v_scale * input->v_set - input->v;
        pitch_err = input->pitch - pitch_offset;  // = -pitch + offset
        pitch_rate_err = input->pitch_rate;
    } else {
        // 右腿: 原始符号
        // 参考: chassis->x_filter - chassis->x_set
        x_err = input->x - input->x_set;
        v_err = input->v - v_scale * input->v_set;
        pitch_err =  -input->pitch - (-pitch_offset);//因为原参考代码时从右边看 逆时针为正
        pitch_rate_err = -input->pitch_rate;
    }
    
    // === 3. 计算轮子扭矩 T (K[0..5] × state) ===
    float T = ctrl->K[0] * theta
            + ctrl->K[1] * d_theta
            + ctrl->K[2] * x_err
            + ctrl->K[3] * v_err
            + ctrl->K[4] * pitch_err
            + ctrl->K[5] * pitch_rate_err;
    
    // 保存各分量用于调试
    output->state_contrib[0] = ctrl->K[0] * theta;
    output->state_contrib[1] = ctrl->K[1] * d_theta;
    output->state_contrib[2] = ctrl->K[2] * x_err;
    output->state_contrib[3] = ctrl->K[3] * v_err;
    output->state_contrib[4] = ctrl->K[4] * pitch_err;
    output->state_contrib[5] = ctrl->K[5] * pitch_rate_err;
    
    // === 4. 计算腿部摆动扭矩 Tp (K[6..11] × state) ===
    float Tp = ctrl->K[6] * (theta + input->theta_set)
             + ctrl->K[7] * d_theta
             + ctrl->K[8] * x_err
             + ctrl->K[9] * v_err
             + ctrl->K[10] * pitch_err
             + ctrl->K[11] * pitch_rate_err;
    
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
        T = ctrl->K[0] * theta + ctrl->K[1] * d_theta;
        Tp = ctrl->K[6] * theta + ctrl->K[7] * d_theta + split_comp;
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
