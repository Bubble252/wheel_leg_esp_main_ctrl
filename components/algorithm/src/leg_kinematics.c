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
#include "esp_timer.h"

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
 * 坐标系定义 (body_angle / α):
 *   α = 0°:   腿部水平向后
 *   α = -180°: 腿部水平向前
 *   α = -90°: 腿部垂直向下 (正常站立姿态)
 *   -90° < α < 0°:   腿部向后摆
 *  -180° < α < -90°:   腿部向前摆
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
 * @brief 计算腿部雅可比矩阵 (机身坐标系 L-α)
 * 
 * 雅可比矩阵 J 将关节速度映射到工作空间速度:
 *   [dL/dt ]   [J[0]  J[1]] [dθ1/dt]
 *   [dα/dt] = [J[2]  J[3]] [dθ2/dt]
 * 
 * 运动学关系:
 *   L = sqrt(L1² + L2² + 2*L1*L2*cos(θ2))   (余弦定理)
 *   β = atan2(L2*sin(θ2), L1 + L2*cos(θ2))  (小腿引起的角度偏移)
 *   α = θ1 + β                               (身体角度)
 * 
 * 雅可比矩阵元素:
 *   J[0] = ∂L/∂θ1 = 0                       (腿长与髋关节角度无关)
 *   J[1] = ∂L/∂θ2 = -L1*L2*sin(θ2) / L     (膝关节影响腿长)
 *   J[2] = ∂α/∂θ1 = 1                       (髋关节直接影响身体角度)
 *   J[3] = ∂α/∂θ2 = L2*(L1*cos(θ2) + L2) / L²  (膝关节通过β影响身体角度)
 * 
 * @note θ2 定义: 膝关节角度，伸直为0，弯曲为负
 * @note 右腿处理: θ2 在计算前取反，使左右腿使用相同的数学公式
 *       输出的 J 可直接用于扭矩计算，无需额外镜像处理
 */
esp_err_t leg_kin_jacobian(const leg_joint_state_t *joint, bool is_left,
                           const leg_kin_params_t *params,
                           float J[4]) {
    if (joint == NULL || J == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 获取参数
    leg_kin_params_t p;
    if (params == NULL) {
        leg_kin_get_default_params(is_left, &p);
    } else {
        memcpy(&p, params, sizeof(leg_kin_params_t));
    }
    
    const float L1 = p.thigh_length;
    const float L2 = p.shank_length;
    
    // 电机角度 -> 运动学角度 (弧度)
    float theta2 = DEG2RAD(joint->knee_angle - p.knee_offset);
    if (!is_left) {
        theta2 = -theta2;  // 右腿镜像
    }
    
    // 三角函数预计算
    const float c2 = cosf(theta2);
    const float s2 = sinf(theta2);
    
    // 当前腿长 L = sqrt(L1² + L2² + 2*L1*L2*cos(θ2))
    const float L_sq = L1*L1 + L2*L2 + 2.0f*L1*L2*c2;
    const float L = sqrtf(L_sq);
    
    // 雅可比矩阵元素
    // 注意：θ2 已经在前面做了镜像处理（右腿取反）
    // 计算出的 J 是相对于"等效左腿姿态"的雅可比
    // 由于 L 和 α 的定义对左右腿是相同的（不需要镜像），
    // 所以 J 不需要再做额外的符号处理
    J[0] = 0.0f;                              // ∂L/∂θ1 = 0
    J[1] = -L1 * L2 * s2 / L;                 // ∂L/∂θ2
    J[2] = 1.0f;                              // ∂α/∂θ1 = 1
    J[3] = L2 * (L1 * c2 + L2) / L_sq;        // ∂α/∂θ2 = ∂β/∂θ2
    
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

// ============================================================================
// Body 坐标系笛卡尔接口实现 (极坐标 ↔ 直角坐标)
// ============================================================================
// 坐标关系 (body 坐标系, 原点在髋关节):
//   x = L * cos(α)    向后为正
//   y = L * sin(α)    向上为正 (站立时 α=-90°, y=-L, 即向下)
//   L = sqrt(x² + y²)
//   α = atan2(y, x)

void leg_kin_polar_to_cartesian(float leg_length, float body_angle_deg,
                                float *x, float *y) {
    if (x == NULL || y == NULL) return;
    float alpha_rad = DEG2RAD(body_angle_deg);
    *x = leg_length * cosf(alpha_rad);
    *y = leg_length * sinf(alpha_rad);
}

void leg_kin_cartesian_to_polar(float x, float y,
                                float *leg_length, float *body_angle_deg) {
    if (leg_length == NULL || body_angle_deg == NULL) return;
    *leg_length = sqrtf(x * x + y * y);
    *body_angle_deg = RAD2DEG(atan2f(y, x));
}

esp_err_t leg_kin_inverse_cartesian_body(float x, float y, bool is_left,
                                         const leg_kin_params_t *params,
                                         leg_joint_state_t *joint) {
    if (joint == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 笛卡尔 → 极坐标
    float L = sqrtf(x * x + y * y);
    float alpha_deg = RAD2DEG(atan2f(y, x));
    
    // 复用极坐标 IK
    leg_workspace_state_t ws = {
        .leg_length = L,
        .body_angle = alpha_deg
    };
    return leg_kin_inverse(&ws, is_left, params, joint);
}

esp_err_t leg_kin_forward_cartesian_body(const leg_joint_state_t *joint,
                                         bool is_left,
                                         const leg_kin_params_t *params,
                                         float *x, float *y) {
    if (joint == NULL || x == NULL || y == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 先做极坐标 FK
    leg_workspace_state_t ws;
    esp_err_t ret = leg_kin_forward(joint, is_left, params, &ws);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 极坐标 → 笛卡尔
    leg_kin_polar_to_cartesian(ws.leg_length, ws.body_angle, x, y);
    return ESP_OK;
}

void leg_kin_clamp_cartesian_body(float *x, float *y,
                                  const leg_kin_params_t *params) {
    if (x == NULL || y == NULL) return;
    
    // 笛卡尔 → 极坐标
    float L, alpha_deg;
    leg_kin_cartesian_to_polar(*x, *y, &L, &alpha_deg);
    
    // 复用极坐标 clamp
    leg_kin_clamp_workspace(&L, &alpha_deg, params);
    
    // 极坐标 → 笛卡尔
    leg_kin_polar_to_cartesian(L, alpha_deg, x, y);
}

bool leg_kin_is_reachable_cartesian_body(float x, float y,
                                         const leg_kin_params_t *params) {
    float L = sqrtf(x * x + y * y);
    float alpha_deg = RAD2DEG(atan2f(y, x));
    return leg_kin_is_reachable(L, alpha_deg, params);
}

// ============================================================================
// VMC (Virtual Model Control) 实现
// ============================================================================

// 辅助函数: 浮点数限幅
static inline float clamp_f(float value, float min_val, float max_val) {
    if (value < min_val) return min_val;
    if (value > max_val) return max_val;
    return value;
}

// ============================================================================
// 电机扭矩补偿 (非线性正解)
// ============================================================================

/**
 * @brief 扭矩补偿: 将期望实际扭矩(Nm)反解为需要发送的电机命令扭矩(Nm)
 * 
 * 使用拟合的三次多项式直接从期望扭矩(y)计算命令扭矩(x):
 *   x = 6.4068 * y^3 - 5.8745 * y^2 + 2.3684 * y + 0.0529
 * 
 * 比之前的二次方程求根法更快 (无 sqrtf，仅乘加运算)。
 * 
 * @param desired_torque_Nm 期望的实际输出扭矩 (Nm), 可正可负
 * @return 需要发送给电机的命令扭矩 (Nm)
 */
float vmc_torque_compensate(float desired_torque_Nm) {
    // 处理接近零的情况
    if (fabsf(desired_torque_Nm) < 0.001f) {
        return 0.0f;
    }
    
    // 取绝对值计算，保持关于原点中心对称
    float y = fabsf(desired_torque_Nm);
    
    // 三次多项式: x = a3*y^3 + a2*y^2 + a1*y + a0
    // Horner 形式: x = ((a3*y + a2)*y + a1)*y + a0
    float cmd_Nm = ((6.4068f * y - 5.8745f) * y + 2.3684f) * y + 0.0529f;
    if (cmd_Nm < 0.0f) cmd_Nm = 0.0f;
    
    return (desired_torque_Nm >= 0) ? cmd_Nm : -cmd_Nm;
}

/**
 * @brief 扭矩正解: 命令扭矩 → 实际扭矩 (用于显示/调试)
 * 
 * 逆多项式的正解: 给定命令扭矩 x, 计算实际扭矩 y
 * y = 1.7485 * x^2 + 0.0085 * x - 0.0002
 * 
 * @param cmd_torque_Nm 命令扭矩 (Nm)
 * @return 预测的实际输出扭矩 (Nm)
 */
float vmc_torque_forward(float cmd_torque_Nm) {
    float x = fabsf(cmd_torque_Nm);
    float y = 1.7485f * x * x + 0.0085f * x - 0.0002f;
    if (y < 0.0f) y = 0.0f;
    return (cmd_torque_Nm >= 0) ? y : -y;
}

/**
 * @brief 从电机电流反解实际扭矩 (Nm)
 * 
 * 1A 电流 ≈ 0.25 Nm 扭矩
 * 
 * @param current_A 电机反馈电流 (A)
 * @return 实际扭矩 (Nm)
 */
float vmc_current_to_torque(float current_A) {
    return current_A * 0.25f;
}

/**
 * @brief 获取默认 VMC 参数
 */
void vmc_get_default_params(vmc_params_t *params) {
    if (params == NULL) return;
    
    // 默认使用机身坐标系 (更适合轮腿机器人)
    params->coord_type = VMC_COORD_BODY;
    
    // === 世界坐标系参数 ===
    params->K_vx = 0.0f;           // 水平速度增益 (Ns/m)
    params->K_y = 0.0f;          // 垂直刚度 (N/m)
    params->D_y = 0.0f;            // 垂直阻尼 (Ns/m)
    
    // === 机身坐标系参数 ===
    params->K_L = 0.0f;          // 腿长刚度 (N/m)
    params->D_L = 0.0f;            // 腿长阻尼 (Ns/m)
    params->K_alpha = 0.0f;         // 身体角度刚度 (Nm/rad)
    params->D_alpha = 0.0f;         // 身体角度阻尼 (Nm·s/rad)
    
    // === 通用参数 ===
    // 机身 Pitch 控制 (默认关闭)
    params->K_pitch = 0.0f;         // Pitch 刚度 (Nm/rad), 保守值
    params->D_pitch = 0.0f;         // Pitch 阻尼 (Nm·s/rad)
    params->target_pitch = 0.0f;    // 目标 pitch = 0 (机身水平)
    params->pitch_ctrl_enable = false; // 默认关闭，需要手动开启
    
    // 双腿协调控制 (默认关闭)
    params->K_sync = 0.0f;          // 协调控制 P 增益 (Nm/rad)
    params->D_sync = 0.0f;         // 协调控制 D 增益 (Nm·s/rad)
    params->sync_enable = false;    // 默认关闭
    params->sync_diff_method = VMC_DIFF_JACOBIAN; // 默认雅可比
    
    // 单腿 VMC 速度估计方法
    params->vmc_diff_method = VMC_DIFF_JACOBIAN; // 默认雅可比 (解析精确、无延迟)
    
    // 重力补偿
    params->gravity_comp = 0.0f;    // 50% 重力补偿 (保守起见)
    params->robot_mass = 1.0f;      // 机器人质量 (kg)
    
    // 输出限幅
    params->max_hip_torque = 8.0f;  // 髋关节最大扭矩 (Nm)
    params->max_knee_torque = 8.0f; // 膝关节最大扭矩 (Nm)
}

/**
 * @brief 计算笛卡尔坐标系雅可比矩阵 (世界坐标系)
 * 
 * 坐标系定义 (世界坐标系，考虑机身倾斜):
 *   x: 向后为正 (水平方向)
 *   y: 向上为正 (垂直方向，轮子在髋关节下方所以 y < 0)
 *   原点: 髋关节轴心
 *   θ1: 大腿相对于 x 轴（水平向后）的夹角，向下（顺时针）为负
 *       θ1 = 0° 时大腿水平向后
 *       θ1 = -90° 时大腿垂直向下
 *   
 *   θ1_world = θ1_body + pitch  (世界坐标系下的大腿角度)
 *   x =  L1*cos(θ1_world) + L2*cos(θ1_world+θ2)  (向后为正)
 *   y =  L1*sin(θ1_world) + L2*sin(θ1_world+θ2)  (向下为负，θ1=-90°时 y<0)
 * 
 * 雅可比矩阵 (对 θ1_body 和 θ2 求偏导):
 *   J = [∂x/∂θ1, ∂x/∂θ2]   [-L1*s1w - L2*s12w,   -L2*s12w]
 *       [∂y/∂θ1, ∂y/∂θ2] = [ L1*c1w + L2*c12w,    L2*c12w]
 *   其中 c1w = cos(θ1_world), s1w = sin(θ1_world), etc.
 */
esp_err_t leg_kin_jacobian_cartesian(const leg_joint_state_t *joint, 
                                      bool is_left,
                                      const leg_kin_params_t *params,
                                      float pitch_rad,
                                      float J[4]) {
    if (joint == NULL || J == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    leg_kin_params_t p;
    if (params == NULL) {
        leg_kin_get_default_params(is_left, &p);
    } else {
        memcpy(&p, params, sizeof(leg_kin_params_t));
    }
    
    float L1 = p.thigh_length;
    float L2 = p.shank_length;
    
    // 电机角度 -> 运动学角度 (弧度)
    float theta1_body = DEG2RAD(joint->hip_angle - p.hip_offset);
    float theta2 = DEG2RAD(joint->knee_angle - p.knee_offset);
    
    // 右腿镜像处理: 运动学角度取反
    if (!is_left) {
        theta1_body = -theta1_body;
        theta2 = -theta2;
    }
    
    // 世界坐标系下的大腿角度 = 机身坐标系角度 + IMU pitch
    // pitch > 0 表示机身前倾
    float theta1_world = theta1_body + pitch_rad;
    
    float c1w = cosf(theta1_world);
    float s1w = sinf(theta1_world);
    float c12w = cosf(theta1_world + theta2);
    float s12w = sinf(theta1_world + theta2);
    
    // 笛卡尔雅可比: [dx/dθ1, dx/dθ2; dy/dθ1, dy/dθ2]
    // x = L1*cos(θ1_world) + L2*cos(θ1_world+θ2)
    // y = L1*sin(θ1_world) + L2*sin(θ1_world+θ2)
    J[0] = -L1 * s1w - L2 * s12w;   // ∂x/∂θ1
    J[1] = -L2 * s12w;              // ∂x/∂θ2
    J[2] =  L1 * c1w + L2 * c12w;   // ∂y/∂θ1
    J[3] =  L2 * c12w;              // ∂y/∂θ2
    
    // 右腿镜像处理: x 方向反向
    if (!is_left) {
        J[0] = -J[0];
        J[1] = -J[1];
    }
    
    return ESP_OK;
}

/**
 * @brief 正运动学: 关节角度 -> 笛卡尔位置 (世界坐标系)
 * 
 * 坐标系定义 (世界坐标系，考虑机身倾斜):
 *   x: 向后为正 (水平方向)
 *   y: 向上为正 (垂直方向，轮子在髋关节下方所以 y < 0)
 *   θ1: 大腿相对于 x 轴的夹角，向下（顺时针）为负
 *       θ1 = 0° 时大腿水平向后
 *       θ1 = -90° 时大腿垂直向下
 */
esp_err_t leg_kin_forward_cartesian(const leg_joint_state_t *joint,
                                     bool is_left,
                                     const leg_kin_params_t *params,
                                     float pitch_rad,
                                     float *x, float *y) {
    if (joint == NULL || x == NULL || y == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    leg_kin_params_t p;
    if (params == NULL) {
        leg_kin_get_default_params(is_left, &p);
    } else {
        memcpy(&p, params, sizeof(leg_kin_params_t));
    }
    
    float L1 = p.thigh_length;
    float L2 = p.shank_length;
    
    // 电机角度 -> 运动学角度 (弧度)
    float theta1_body = DEG2RAD(joint->hip_angle - p.hip_offset);
    float theta2 = DEG2RAD(joint->knee_angle - p.knee_offset);
    
    // 右腿镜像处理
    if (!is_left) {
        theta1_body = -theta1_body;
        theta2 = -theta2;
    }
    
    // 世界坐标系下的大腿角度 = 机身坐标系角度 + IMU pitch
    float theta1_world = theta1_body + pitch_rad;
    
    // 计算笛卡尔位置 (世界坐标系)
    // x = L1*cos(θ1) + L2*cos(θ1+θ2)  向后为正
    // y = L1*sin(θ1) + L2*sin(θ1+θ2)  向上为正 (θ1=-90°时 y<0)
    *x = L1 * cosf(theta1_world) + L2 * cosf(theta1_world + theta2);
    *y = L1 * sinf(theta1_world) + L2 * sinf(theta1_world + theta2);
    
    // 右腿镜像处理: x 反向
    if (!is_left) {
        *x = -(*x);
    }
    
    return ESP_OK;
}

/**
 * @brief VMC 世界坐标系版本 (内部函数)
 * 
 * 坐标系定义 (世界坐标系):
 *   x: 向后为正 (水平方向)
 *   y: 向上为正 (垂直方向，相对地面高度)
 * 
 * 控制公式:
 *   F_x = K_vx × (target_vx - current_vx)           // 水平: 纯速度控制
 *   F_y = K_y × (target_y - current_y) + D_y × Δvy + F_gravity  // 垂直: 弹簧-阻尼 + 重力
 *   [τ_hip_vmc; τ_knee] = J_cart^T × [F_x; F_y]
 */
static esp_err_t vmc_compute_torque_world(const vmc_params_t *params,
                                           const vmc_input_t *input,
                                           const leg_joint_state_t *joint,
                                           bool is_left,
                                           float pitch_rad,
                                           vmc_output_t *output) {
    // 1. 计算水平虚拟力 (纯速度控制)
    float vx_error = input->target_vx - input->current_vx;
    float F_x = params->K_vx * vx_error;
    
    // 2. 计算垂直虚拟力 (弹簧-阻尼 + 重力补偿)向下给力
    float y_error = input->target_y - input->current_y;//变高 向下给力 erro<0
    float vy_error = input->target_vy - input->current_vy;
    float F_spring_damper = (params->K_y * y_error + params->D_y * vy_error);
    
    // 重力补偿 (向下的力，抵消重力)
    float F_gravity = -params->gravity_comp * params->robot_mass * 9.81f;
    float F_y = F_spring_damper + F_gravity;

    // 3. 获取笛卡尔雅可比矩阵 (世界坐标系，考虑 pitch)
    float J[4];
    leg_kin_jacobian_cartesian(joint, is_left, NULL, pitch_rad, J);
    
    // 4. τ_vmc = J^T × F
    float tau_hip_vmc  = (J[0] * F_x + J[2] * F_y);
    float tau_knee = (J[1] * F_x + J[3] * F_y);
    
    // 5. 输出虚拟力信息
    output->F_x = F_x;
    output->F_y = F_y;
    output->F_L = 0.0f;
    output->F_alpha = 0.0f;
    output->F_gravity = F_gravity;
    output->tau_hip_vmc = tau_hip_vmc;
    output->knee_torque = tau_knee;
    
    return ESP_OK;
}

/**
 * @brief VMC 机身坐标系版本 (内部函数)
 * 
 * 坐标系定义 (机身坐标系):
 *   L: 腿长 (髋关节到轮子的直线距离)
 *   α: 身体角度 (body_angle)
 *      α = 0°:   腿部水平向后
 *      α = -90°: 腿部垂直向下 (正常站立姿态)
 *      α > 0°:   腿部向后摆
 *      α < 0°:   腿部向前摆
 * 
 * 控制公式:
 *   F_L = K_L × (target_L - current_L) + D_L × ΔdL + F_gravity_L  // 腿长: 弹簧-阻尼 + 重力
 *   F_α = K_α × (target_α - current_α) + D_α × Δdα               // 角度: 弹簧-阻尼
 *   [τ_hip_vmc; τ_knee] = J_workspace^T × [F_L; F_α]
 * 
 * 优点:
 *   - 与现有位置控制一致 (都是腿长-角度)
 *   - 不需要 pitch 补偿
 *   - 与 LQR 职责分离清晰 (LQR管水平，VMC管腿部)
 *   - 重力补偿更简单 (沿腿方向)
 */
static esp_err_t vmc_compute_torque_body(const vmc_params_t *params,
                                          const vmc_input_t *input,
                                          const leg_joint_state_t *joint,
                                          bool is_left,
                                          float pitch_rad,
                                          vmc_output_t *output) {
    // 1. 计算腿长方向虚拟力 (弹簧-阻尼 + 重力补偿)
    float L_error = input->target_L - input->current_L;
    float dL_error = input->target_dL - input->current_dL;
    float F_spring_damper = params->K_L * L_error + params->D_L * dL_error;
    
    // 重力补偿 (沿腿方向的分量)
    // 重力 mg 垂直向下，沿腿方向的分量 = -mg × cos(pitch + α + 90°)
    // 当机身水平 (pitch=0) 且腿垂直 (α=-π/2) 时，全部重力沿腿方向
    // 注: input->current_alpha 已经是 rad (由上层 DEG2RAD(workspace.body_angle) 转换)
    float total_angle = input->current_alpha + (M_PI / 2.0f);
    float F_gravity = params->gravity_comp * params->robot_mass * 9.81f * cosf(total_angle + pitch_rad);
    float F_L = F_spring_damper + F_gravity;  // 伸展力 (支撑重力)
    
    // 2. 计算身体角度方向虚拟力矩 (弹簧-阻尼)
    float alpha_error = input->target_alpha - input->current_alpha;
    float dalpha_error = input->target_dalpha - input->current_dalpha;
    float F_alpha = params->K_alpha * alpha_error + params->D_alpha * dalpha_error;
    
    // 3. 获取工作空间雅可比矩阵 (机身坐标系)
    // J = [∂L/∂θ1, ∂L/∂θ2; ∂α/∂θ1, ∂α/∂θ2]
    float J[4];
    leg_kin_jacobian(joint, is_left, NULL, J);
    
    // 4. τ = J^T × F
    //  F_L 定义为"末端沿腿长方向的伸展力"
    //       腿伸展 (F_L > 0) → 需要正扭矩使腿伸长
    // ┌ τ_hip  ┐   ┌ J[0]  J[2] ┐   ┌ F_L   ┐
    // │        │ = │            │ × │       │
    // └ τ_knee ┘   └ J[1]  J[3] ┘   └ F_α   ┘
    float tau_hip_vmc  = J[0] * F_L + J[2] * F_alpha;
    float tau_knee = J[1] * F_L + J[3] * F_alpha;
    
    // 5. 输出虚拟力信息
    output->F_x = 0.0f;
    output->F_y = 0.0f;
    output->F_L = F_L;
    output->F_alpha = F_alpha;
    output->F_gravity = F_gravity;
    output->tau_hip_vmc = tau_hip_vmc;
    output->knee_torque = tau_knee;
    
    return ESP_OK;
}

/**
 * @brief VMC 计算关节扭矩 (统一接口，支持两种坐标系)
 * 
 * 根据 params->coord_type 选择计算方式:
 *   VMC_COORD_WORLD: 世界坐标系 (x-y)，需要 pitch 补偿
 *   VMC_COORD_BODY:  机身坐标系 (L-α)，不需要 pitch 补偿
 * 
 * 两种方式都支持额外的 pitch 控制 (通过 params->pitch_ctrl_enable)
 */
esp_err_t vmc_compute_torque(const vmc_params_t *params,
                              const vmc_input_t *input,
                              const leg_joint_state_t *joint,
                              bool is_left,
                              float pitch_rad,
                              vmc_output_t *output) {
    if (params == NULL || input == NULL || joint == NULL || output == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 根据坐标系类型选择计算方式
    esp_err_t ret;
    if (params->coord_type == VMC_COORD_WORLD) {
        ret = vmc_compute_torque_world(params, input, joint, is_left, pitch_rad, output);
    } else {
        ret = vmc_compute_torque_body(params, input, joint, is_left, pitch_rad, output);
    }
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 机身 Pitch 控制 (可选，两种坐标系都支持)
    // τ_hip_pitch 直接作用于髋关节，其反作用力矩使机身向 target_pitch 恢复
    float tau_hip_pitch = 0.0f;
    if (params->pitch_ctrl_enable) {
        float pitch_error = params->target_pitch - input->current_pitch;
        tau_hip_pitch = params->K_pitch * pitch_error - params->D_pitch * input->current_pitch_rate;
    }
    
    // 髋关节总扭矩 = VMC 扭矩 + Pitch 控制扭矩
    float tau_hip = output->tau_hip_vmc + tau_hip_pitch;
    float tau_knee = output->knee_torque;
    
    // 扭矩限幅
    tau_hip  = clamp_f(tau_hip, -params->max_hip_torque, params->max_hip_torque);
    tau_knee = clamp_f(tau_knee, -params->max_knee_torque, params->max_knee_torque);
    
    // 最终输出
    output->hip_torque = tau_hip;
    output->knee_torque = tau_knee;
    output->tau_hip_pitch = tau_hip_pitch;
    
    return ESP_OK;
}

// ============================================================================
// VMC 控制器高层接口实现
// ============================================================================

/**
 * @brief VMC 单腿控制器计算 (高层接口)
 * 
 * 封装完整的 VMC 计算流程:
 *   1. 传感器数据 → 关节状态
 *   2. FK: 关节状态 → 工作空间状态 (腿长、身体角度)
 *   3. FK_world: 关节状态 → 世界坐标 (高度)
 *   4. 雅可比 + 关节速度 → 工作空间速度
 *   5. 填充 VMC 输入
 *   6. 调用 vmc_compute_torque()
 *   7. 输出扭矩和状态
 */
esp_err_t vmc_ctrl_compute(const vmc_params_t *params,
                            const vmc_ctrl_input_t *input,
                            bool is_left,
                            vmc_ctrl_output_t *output) {
    if (params == NULL || input == NULL || output == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // === 1. 传感器数据 → 关节状态 ===
    leg_joint_state_t joint = {
        .hip_angle = input->sensor.hip_angle,
        .knee_angle = input->sensor.knee_angle
    };
    
    // IMU: deg → rad
    float pitch_rad = DEG2RAD(input->pitch_deg);
    float pitch_rate_rad = DEG2RAD(input->pitch_rate_deg);
    
    // === 2. 机身坐标系 FK: 计算腿长和身体角度 ===
    leg_workspace_state_t workspace;
    leg_kin_forward(&joint, is_left, NULL, &workspace);
    float current_L = workspace.leg_length;
    float current_alpha_rad = DEG2RAD(workspace.body_angle);
    
    // === 3. 速度估计: 根据配置选择雅可比或数值微分 ===
    float current_dL, current_dalpha;
    float current_vy = 0.0f;
    
    if (params->vmc_diff_method == VMC_DIFF_NUMERIC) {
        // === 数值微分方法: 不需要电机速度反馈 ===
        if (output->_diff_initialized) {
            // 使用实际时间间隔计算微分
            int64_t now_us = esp_timer_get_time();
            float dt_diff = (now_us - output->_last_timestamp_us) * 1e-6f;
            if (dt_diff > 0.0001f && dt_diff < 0.1f) {
                float inv_dt = 1.0f / dt_diff;
                current_dL = (current_L - output->_last_L) * inv_dt;
                current_dalpha = (current_alpha_rad - output->_last_alpha_rad) * inv_dt;
            } else {
                // 时间间隔异常，不做微分
                current_dL = 0.0f;
                current_dalpha = 0.0f;
            }
        } else {
            // 首次调用，无历史数据
            current_dL = 0.0f;
            current_dalpha = 0.0f;
        }
        // 世界坐标系垂直速度也用数值微分 (通过 current_y 差分)
        // 注: current_vy 留 0, 世界坐标系模式下精度会降低
    } else {
        // === 雅可比方法 (默认): 需要电机速度反馈 ===
        // 关节速度单位: °/s → rad/s
        const float deg_to_rad = M_PI / 180.0f;
        float hip_vel_rad = input->sensor.hip_velocity * deg_to_rad;
        float knee_vel_rad = input->sensor.knee_velocity * deg_to_rad;
        
        // 右腿镜像: FK 内部对 θ1,θ2 取反 (θ_kin = -θ_motor),
        // 所以 dθ_kin/dt = -dθ_motor/dt, 关节速度也需要取反
        // 才能与镜像坐标系下的雅可比矩阵匹配
        if (!is_left) {
            hip_vel_rad = -hip_vel_rad;
            knee_vel_rad = -knee_vel_rad;
        }
        
        float J_body[4];
        leg_kin_jacobian(&joint, is_left, NULL, J_body);
        current_dL = J_body[0] * hip_vel_rad + J_body[1] * knee_vel_rad;
        current_dalpha = J_body[2] * hip_vel_rad + J_body[3] * knee_vel_rad;
        
        // 世界坐标系雅可比: 计算垂直速度
        float J_world[4];
        leg_kin_jacobian_cartesian(&joint, is_left, NULL, pitch_rad, J_world);
        current_vy = J_world[2] * hip_vel_rad + J_world[3] * knee_vel_rad;
    }
    
    // === 4. 世界坐标系 FK: 计算离地高度 ===
    float current_x, current_y;
    leg_kin_forward_cartesian(&joint, is_left, NULL, pitch_rad, &current_x, &current_y);
    
    // === 5. 填充 VMC 输入 ===
    vmc_input_t vmc_input = {0};
    
    // 世界坐标系输入
    // 注意: current_y 是负值 (轮子在髋关节下方)
    //       target_y 也要是负值才能正确比较，所以取反
    vmc_input.target_vx = input->target_vx;
    vmc_input.target_y = -input->target_leg_length;  // 取反: 腿长 → 世界坐标 y (负值)
    vmc_input.target_vy = 0.0f;
    vmc_input.current_y = current_y;
    vmc_input.current_vx = input->robot_vx;
    vmc_input.current_vy = current_vy;
    
    // 机身坐标系输入 (不需要处理，L 本身就是正值)
    vmc_input.target_L = input->target_leg_length;
    vmc_input.target_dL = 0.0f;
    vmc_input.target_alpha = DEG2RAD(input->target_body_angle_deg);
    vmc_input.target_dalpha = 0.0f;
    vmc_input.current_L = current_L;
    vmc_input.current_dL = current_dL;
    vmc_input.current_alpha = current_alpha_rad;
    vmc_input.current_dalpha = current_dalpha;
    
    // IMU 姿态
    vmc_input.current_pitch = pitch_rad;
    vmc_input.current_pitch_rate = pitch_rate_rad;
    
    // === 6. 调用底层 VMC 计算 ===
    vmc_output_t vmc_output = {0};
    esp_err_t ret = vmc_compute_torque(params, &vmc_input, &joint, is_left, pitch_rad, &vmc_output);
    
    if (ret != ESP_OK) {
        return ret;
    }
    
    // === 7. 填充输出 ===
    output->hip_torque = vmc_output.hip_torque;
    output->knee_torque = vmc_output.knee_torque;
    output->current_leg_length = current_L;
    output->current_body_angle = workspace.body_angle;
    output->current_body_angle_rate = current_dalpha * (180.0f / M_PI);  // rad/s → deg/s
    output->current_leg_length_rate = current_dL;  // m/s
    output->current_height = current_y;
    output->debug = vmc_output;
    
    // 保存当前值供下一次数值微分使用
    output->_last_L = current_L;
    output->_last_alpha_rad = current_alpha_rad;
    output->_last_timestamp_us = esp_timer_get_time();
    output->_diff_initialized = true;
    
    // === 8. 右腿扭矩方向修正 ===
    // 雅可比是基于镜像后的运动学角度计算的（统一的工作空间定义）
    // 但右腿电机是镜像安装的，所以实际扭矩需要取反
    if (!is_left) {
        output->hip_torque = -output->hip_torque;
        output->knee_torque = -output->knee_torque;
    }
    
    return ESP_OK;
}

// ============================================================================
// 双腿 VMC 控制器 (包含双腿协调控制)
// ============================================================================

/**
 * @brief 双腿 VMC 控制器计算
 * 
 * 包含双腿协调控制：当左右腿身体角度不一致时，通过 PD 产生补偿力矩
 */
esp_err_t vmc_dual_compute(const vmc_params_t *params,
                           const vmc_dual_input_t *input,
                           vmc_dual_output_t *output) {
    if (params == NULL || input == NULL || output == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // === 1. 构造单腿输入并分别计算 VMC ===
    vmc_ctrl_input_t left_input = {
        .pitch_deg = input->pitch_deg,
        .pitch_rate_deg = input->pitch_rate_deg,
        .robot_vx = input->robot_vx,
        .target_leg_length = input->left.target_leg_length,
        .target_body_angle_deg = input->left.target_body_angle_deg,
        .target_vx = input->target_vx,
        .sensor = input->left.sensor
    };
    
    vmc_ctrl_input_t right_input = {
        .pitch_deg = input->pitch_deg,
        .pitch_rate_deg = input->pitch_rate_deg,
        .robot_vx = input->robot_vx,
        .target_leg_length = input->right.target_leg_length,
        .target_body_angle_deg = input->right.target_body_angle_deg,
        .target_vx = input->target_vx,
        .sensor = input->right.sensor
    };
    
    esp_err_t ret_left = vmc_ctrl_compute(params, &left_input, true, &output->left);
    esp_err_t ret_right = vmc_ctrl_compute(params, &right_input, false, &output->right);
    
    if (ret_left != ESP_OK || ret_right != ESP_OK) {
        return (ret_left != ESP_OK) ? ret_left : ret_right;
    }
    
    // 初始化协调控制输出
    output->angle_diff_deg = 0.0f;
    output->angle_diff_rate_deg = 0.0f;
    output->F_sync = 0.0f;
    
    // === 2. 双腿协调控制 (仅在机身坐标系下生效) ===
    if (params->sync_enable && params->coord_type == VMC_COORD_BODY) {
        // 角度差 = 左腿 - 右腿 (度)
        float angle_diff_deg = output->left.current_body_angle - output->right.current_body_angle;
        float angle_diff_rad = angle_diff_deg * (M_PI / 180.0f);
        
        // 角速度差估计 (根据配置选择方法)
        float angle_diff_rate_rad = 0.0f;
        
        if (params->sync_diff_method == VMC_DIFF_JACOBIAN) {
            // === 雅可比方法: 直接使用 vmc_ctrl_compute 输出的身体角速度 ===
            // angle_diff_rate = left.body_angle_rate - right.body_angle_rate
            // 无延迟，解析精确
            float left_rate_rad = output->left.current_body_angle_rate * (M_PI / 180.0f);
            float right_rate_rad = output->right.current_body_angle_rate * (M_PI / 180.0f);
            angle_diff_rate_rad = left_rate_rad - right_rate_rad;
        } else {
            // === 数值微分方法: 使用静态变量保存上一次角度差 ===
            static float last_angle_diff_rad = 0.0f;
            static int64_t last_sync_timestamp_us = 0;
            
            if (last_sync_timestamp_us != 0) {
                int64_t now_us = esp_timer_get_time();
                float dt_sync = (now_us - last_sync_timestamp_us) * 1e-6f;
                if (dt_sync > 0.0001f && dt_sync < 0.1f) {
                    angle_diff_rate_rad = (angle_diff_rad - last_angle_diff_rad) / dt_sync;
                }
                last_sync_timestamp_us = now_us;
            } else {
                last_sync_timestamp_us = esp_timer_get_time();
            }
            last_angle_diff_rad = angle_diff_rad;
        }
        
        // PD 控制计算 F_sync
        // 当 diff > 0 (左腿更靠后) 时，F_sync > 0
        // 左腿需要 F_alpha 减小 (向前推)，右腿需要 F_alpha 增大 (向后推)
        float F_sync = params->K_sync * angle_diff_rad + params->D_sync * angle_diff_rate_rad;
        
        // 保存调试信息
        output->angle_diff_deg = angle_diff_deg;
        output->angle_diff_rate_deg = angle_diff_rate_rad * (180.0f / M_PI);
        output->F_sync = F_sync;
        
        // 获取左右腿的雅可比矩阵
        leg_joint_state_t left_joint = {
            .hip_angle = input->left.sensor.hip_angle,
            .knee_angle = input->left.sensor.knee_angle
        };
        leg_joint_state_t right_joint = {
            .hip_angle = input->right.sensor.hip_angle,
            .knee_angle = input->right.sensor.knee_angle
        };
        
        float J_left[4], J_right[4];
        leg_kin_jacobian(&left_joint, true, NULL, J_left);
        leg_kin_jacobian(&right_joint, false, NULL, J_right);
        
        // 通过雅可比将 ΔF_alpha 转换为关节扭矩
        // τ = J^T × [0; ΔF_alpha]
        // τ_hip  = J[2] × ΔF_alpha
        // τ_knee = J[3] × ΔF_alpha
        
        // 左腿: ΔF_alpha = -F_sync (角度大时减小)
        float delta_tau_left_hip   = J_left[2]  * (-F_sync);
        float delta_tau_left_knee  = J_left[3]  * (-F_sync);
        
        // 右腿: ΔF_alpha = +F_sync (角度小时增大)
        // 注意: 右腿雅可比是基于镜像运动学计算的，输出扭矩需要取反
        // （与 vmc_ctrl_compute 中的处理一致）
        float delta_tau_right_hip  = -(J_right[2] * (+F_sync));
        float delta_tau_right_knee = -(J_right[3] * (+F_sync));
        
        // 叠加到输出扭矩
        output->left.hip_torque  += delta_tau_left_hip;
        output->left.knee_torque += delta_tau_left_knee;
        output->right.hip_torque  += delta_tau_right_hip;
        output->right.knee_torque += delta_tau_right_knee;
    }
    
    return ESP_OK;
}
