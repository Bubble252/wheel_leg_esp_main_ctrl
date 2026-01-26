/**
 * @file leg_kinematics.c
 * @brief 腿部运动学实现 - 串联二连杆正逆运动学
 * @author Bubble
 * @date 2026-01-24
 */

#include "leg_kinematics.h"
#include <math.h>
#include <string.h>
#include "esp_log.h"

static const char *TAG = "LEG_KIN";

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

// 角度弧度转换
#define DEG2RAD(d) ((d) * M_PI / 180.0f)
#define RAD2DEG(r) ((r) * 180.0f / M_PI)

/**
 * @brief 获取默认腿部参数
 */
void leg_kin_get_default_params(bool is_left, leg_kin_params_t *params) {
    if (params == NULL) return;
    
    params->thigh_length = LEG_THIGH_LENGTH;
    params->shank_length = LEG_SHANK_LENGTH;
    
    if (is_left) {
        params->hip_offset = LEG_LEFT_HIP_OFFSET;
        params->knee_offset = LEG_LEFT_KNEE_OFFSET;
    } else {
        params->hip_offset = LEG_RIGHT_HIP_OFFSET;
        params->knee_offset = LEG_RIGHT_KNEE_OFFSET;
    }
}

/**
 * @brief 正运动学: 关节空间 -> 工作空间
 * 
 * 几何计算:
 *   θ1 = hip_motor - hip_offset  (大腿相对机身角度)
 *   θ2 = knee_motor - knee_offset (小腿相对大腿角度, 0=伸直)
 *   
 *   腿长 L = sqrt(L1² + L2² + 2*L1*L2*cos(θ2))     (余弦定理)
 *   β = atan2(L2*sin(θ2), L1 + L2*cos(θ2))         (小腿引起的角度偏移)
 *   身体夹角 α = θ1 + β
 */
esp_err_t leg_kin_forward(const leg_joint_state_t *joint, bool is_left,
                          const leg_kin_params_t *params,
                          leg_workspace_state_t *workspace) {
    if (joint == NULL || workspace == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 使用默认参数或传入参数
    leg_kin_params_t p;
    if (params == NULL) {
        leg_kin_get_default_params(is_left, &p);
    } else {
        memcpy(&p, params, sizeof(leg_kin_params_t));
    }
    
    // 电机角度 -> 运动学角度 (弧度)
    float theta1 = DEG2RAD(joint->hip_angle - p.hip_offset);   // 大腿相对机身
    float theta2 = DEG2RAD(joint->knee_angle - p.knee_offset); // 小腿相对大腿 (0=伸直)
    
    // 右腿镜像处理
    if (!is_left) {
        theta1 = -theta1;
        theta2 = -theta2;
    }
    
    float L1 = p.thigh_length;
    float L2 = p.shank_length;
    
    // 腿长计算 (余弦定理)
    float cos_theta2 = cosf(theta2);
    float sin_theta2 = sinf(theta2);
    float L_sq = L1*L1 + L2*L2 + 2*L1*L2*cos_theta2;
    float L = sqrtf(L_sq);
    
    // 身体夹角计算
    // β = atan2(L2*sin(θ2), L1 + L2*cos(θ2))
    float beta = atan2f(L2 * sin_theta2, L1 + L2 * cos_theta2);
    float alpha = theta1 + beta;
    
    workspace->leg_length = L;
    workspace->body_angle = RAD2DEG(alpha);
    
    return ESP_OK;
}

/**
 * @brief 逆运动学: 工作空间 -> 关节空间
 * 
 * 给定: 腿长 L, 身体夹角 α
 * 求解: 大腿电机角度, 小腿电机角度
 * 
 * 几何关系:
 *   θ2 = acos((L² - L1² - L2²) / (2*L1*L2))  (余弦定理求膝关节角)
 *   β = atan2(L2*sin(θ2), L1 + L2*cos(θ2))
 *   θ1 = α - β
 */
esp_err_t leg_kin_inverse(const leg_workspace_state_t *workspace, bool is_left,
                          const leg_kin_params_t *params,
                          leg_joint_state_t *joint) {
    if (workspace == NULL || joint == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 使用默认参数或传入参数
    leg_kin_params_t p;
    if (params == NULL) {
        leg_kin_get_default_params(is_left, &p);
    } else {
        memcpy(&p, params, sizeof(leg_kin_params_t));
    }
    
    float L = workspace->leg_length;
    float L1 = p.thigh_length;
    float L2 = p.shank_length;
    
    // 检查腿长是否在可达范围内
    float L_min = fabsf(L1 - L2) + 0.001f;  // 最小腿长 (几乎折叠)
    float L_max = L1 + L2 - 0.001f;          // 最大腿长 (几乎伸直)
    
    if (L < L_min || L > L_max) {
        ESP_LOGD(TAG, "Leg length %.3f out of range [%.3f, %.3f]", L, L_min, L_max);
        return ESP_ERR_INVALID_ARG;
    }
    
    // 使用余弦定理求膝关节角度 θ2
    float cos_theta2 = (L*L - L1*L1 - L2*L2) / (2*L1*L2);
    
    // 数值稳定性检查
    if (cos_theta2 < -1.0f) cos_theta2 = -1.0f;
    if (cos_theta2 > 1.0f) cos_theta2 = 1.0f;
    
    // 膝关节角度: acos 返回正值，但我们的定义是弯曲为负
    float theta2 = -acosf(cos_theta2);  // 取负号，因为弯曲方向是负的
    
    // 计算 β 角
    float sin_theta2 = sinf(theta2);
    float beta = atan2f(L2 * sin_theta2, L1 + L2 * cos_theta2);
    
    // 身体夹角 -> 大腿角度
    float alpha = DEG2RAD(workspace->body_angle);
    float theta1 = alpha - beta;
    
    // 右腿镜像处理
    if (!is_left) {
        theta1 = -theta1;
        theta2 = -theta2;
    }
    
    // 运动学角度 -> 电机角度
    joint->hip_angle = RAD2DEG(theta1) + p.hip_offset;
    joint->knee_angle = RAD2DEG(theta2) + p.knee_offset;
    
    return ESP_OK;
}

/**
 * @brief 检查工作空间目标是否可达
 */
bool leg_kin_is_reachable(float leg_length, float body_angle, 
                          const leg_kin_params_t *params) {
    // 使用默认参数
    leg_kin_params_t p;
    if (params == NULL) {
        leg_kin_get_default_params(true, &p);  // 左右腿长度参数相同
    } else {
        memcpy(&p, params, sizeof(leg_kin_params_t));
    }
    
    float L1 = p.thigh_length;
    float L2 = p.shank_length;
    
    // 几何可达性检查
    float L_min = fabsf(L1 - L2) + 0.001f;
    float L_max = L1 + L2 - 0.001f;
    
    if (leg_length < L_min || leg_length > L_max) {
        return false;
    }
    
    // 工作空间限位检查
    if (leg_length < LEG_LENGTH_MIN || leg_length > LEG_LENGTH_MAX) {
        return false;
    }
    if (body_angle < LEG_BODY_ANGLE_MIN || body_angle > LEG_BODY_ANGLE_MAX) {
        return false;
    }
    
    return true;
}

/**
 * @brief 限制工作空间目标到可达范围
 */
void leg_kin_clamp_workspace(float *leg_length, float *body_angle,
                             const leg_kin_params_t *params) {
    if (leg_length == NULL || body_angle == NULL) return;
    
    // 使用默认参数
    leg_kin_params_t p;
    if (params == NULL) {
        leg_kin_get_default_params(true, &p);
    } else {
        memcpy(&p, params, sizeof(leg_kin_params_t));
    }
    
    float L1 = p.thigh_length;
    float L2 = p.shank_length;
    
    // 几何限制
    float L_min = fabsf(L1 - L2) + 0.005f;  // 留一点余量
    float L_max = L1 + L2 - 0.005f;
    
    // 工作空间限制 (取交集)
    if (L_min < LEG_LENGTH_MIN) L_min = LEG_LENGTH_MIN;
    if (L_max > LEG_LENGTH_MAX) L_max = LEG_LENGTH_MAX;
    
    // 限幅
    if (*leg_length < L_min) *leg_length = L_min;
    if (*leg_length > L_max) *leg_length = L_max;
    if (*body_angle < LEG_BODY_ANGLE_MIN) *body_angle = LEG_BODY_ANGLE_MIN;
    if (*body_angle > LEG_BODY_ANGLE_MAX) *body_angle = LEG_BODY_ANGLE_MAX;
}

/**
 * @brief 计算腿部雅可比矩阵
 * 
 * 雅可比矩阵 J 将关节速度映射到工作空间速度:
 *   [dL/dt  ]   [J11  J12] [dθ1/dt]
 *   [dα/dt ] = [J21  J22] [dθ2/dt]
 * 
 * 其中:
 *   J11 = ∂L/∂θ1 = 0 (腿长与大腿角度无关)
 *   J12 = ∂L/∂θ2 = -L1*L2*sin(θ2) / L
 *   J21 = ∂α/∂θ1 = 1
 *   J22 = ∂α/∂θ2 = ∂β/∂θ2
 */
esp_err_t leg_kin_jacobian(const leg_joint_state_t *joint, bool is_left,
                           const leg_kin_params_t *params,
                           float J[4]) {
    if (joint == NULL || J == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 使用默认参数
    leg_kin_params_t p;
    if (params == NULL) {
        leg_kin_get_default_params(is_left, &p);
    } else {
        memcpy(&p, params, sizeof(leg_kin_params_t));
    }
    
    // 电机角度 -> 运动学角度
    float theta2 = DEG2RAD(joint->knee_angle - p.knee_offset);
    if (!is_left) theta2 = -theta2;
    
    float L1 = p.thigh_length;
    float L2 = p.shank_length;
    
    float cos_theta2 = cosf(theta2);
    float sin_theta2 = sinf(theta2);
    
    // 当前腿长
    float L_sq = L1*L1 + L2*L2 + 2*L1*L2*cos_theta2;
    float L = sqrtf(L_sq);
    
    // 雅可比矩阵元素
    // J11 = ∂L/∂θ1 = 0
    J[0] = 0.0f;
    
    // J12 = ∂L/∂θ2 = -L1*L2*sin(θ2) / L
    J[1] = -L1 * L2 * sin_theta2 / L;
    
    // J21 = ∂α/∂θ1 = 1
    J[2] = 1.0f;
    
    // J22 = ∂α/∂θ2 = ∂β/∂θ2 = L1*L2*cos(θ2) / (L1² + L2² + 2*L1*L2*cos(θ2))
    //              = L1*L2*cos(θ2) / L²
    // 但更精确的推导:
    // β = atan2(L2*sin(θ2), L1 + L2*cos(θ2))
    // ∂β/∂θ2 = (L2*cos(θ2)*(L1+L2*cos(θ2)) + L2²*sin²(θ2)) / ((L1+L2*cos(θ2))² + (L2*sin(θ2))²)
    //        = L2*(L1*cos(θ2) + L2) / L²
    J[3] = L2 * (L1 * cos_theta2 + L2) / L_sq;
    
    // 右腿镜像处理
    if (!is_left) {
        J[1] = -J[1];
        J[3] = -J[3];
    }
    
    return ESP_OK;
}

/**
 * @brief 打印腿部参数信息
 */
void leg_kin_print_params(const leg_kin_params_t *params, bool is_left) {
    leg_kin_params_t p;
    if (params == NULL) {
        leg_kin_get_default_params(is_left, &p);
    } else {
        memcpy(&p, params, sizeof(leg_kin_params_t));
    }
    
    ESP_LOGI(TAG, "%s Leg Kinematics Parameters:", is_left ? "Left" : "Right");
    ESP_LOGI(TAG, "  Thigh length: %.4f m", p.thigh_length);
    ESP_LOGI(TAG, "  Shank length: %.4f m", p.shank_length);
    ESP_LOGI(TAG, "  Hip offset:   %.2f deg", p.hip_offset);
    ESP_LOGI(TAG, "  Knee offset:  %.2f deg", p.knee_offset);
    ESP_LOGI(TAG, "  Length range: [%.3f, %.3f] m", LEG_LENGTH_MIN, LEG_LENGTH_MAX);
    ESP_LOGI(TAG, "  Angle range:  [%.1f, %.1f] deg", LEG_BODY_ANGLE_MIN, LEG_BODY_ANGLE_MAX);
}
