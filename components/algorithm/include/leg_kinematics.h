/**
 * @file leg_kinematics.h
 * @brief 腿部运动学模块 - 串联二连杆正逆运动学
 * @author Bubble
 * @date 2026-01-24
 * 
 * 机构说明:
 *   串联二连杆轮腿结构，每条腿由大腿电机(Hip)和小腿电机(Knee)驱动
 *   
 *         机身 (Body)
 *            │
 *     ┌──────┴──────┐
 *     │  Hip Motor  │  ← 大腿电机 (θ1)
 *     └──────┬──────┘
 *            │ L1 (大腿长度)
 *            │
 *     ┌──────┴──────┐
 *     │ Knee Motor  │  ← 小腿电机 (θ2)
 *     └──────┬──────┘
 *            │ L2 (小腿长度)
 *            │
 *     ┌──────┴──────┐
 *     │ Wheel Motor │  ← 轮电机
 *     └─────────────┘
 *   
 * 运动学空间:
 *   - 腿长 (leg_length): 大腿电机轴心到轮电机轴心的直线距离
 *   - 身体夹角 (body_angle): 该直线与机身垂直向下方向的夹角，向前为正
 */

#ifndef LEG_KINEMATICS_H
#define LEG_KINEMATICS_H

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 配置参数 (可在 menuconfig 或代码中覆盖)
// ============================================================================

#ifndef LEG_THIGH_LENGTH
#define LEG_THIGH_LENGTH            0.10f   // 大腿长度 (米)
#endif

#ifndef LEG_SHANK_LENGTH
#define LEG_SHANK_LENGTH            0.10f   // 小腿长度 (米)
#endif

// 电机零点偏移 (将电机编码器角度转换为运动学角度)
// 运动学定义: theta2=0 表示小腿伸直，顺时针弯曲为负
// 左腿校准: 电机读数 Hip=-60°, Knee=-55° 时，腿部垂直向下(body_angle=-90°)，大小腿夹角90°
// 此时: theta2 = -90°, beta = -45°, theta1 = body_angle - beta = -90 - (-45) = -45°
// theta_knee = motor_knee - knee_offset  =>  -90 = -55 - offset  =>  offset = 35
// theta_hip = motor_hip - hip_offset  =>  -45 = -60 - offset  =>  offset = -15
#ifndef LEG_LEFT_HIP_OFFSET
#define LEG_LEFT_HIP_OFFSET         (-15.0f)
#endif

#ifndef LEG_LEFT_KNEE_OFFSET
#define LEG_LEFT_KNEE_OFFSET        (35.0f)
#endif

// 右腿: 镜像对称
#ifndef LEG_RIGHT_HIP_OFFSET
#define LEG_RIGHT_HIP_OFFSET        (15.0f)
#endif

#ifndef LEG_RIGHT_KNEE_OFFSET
#define LEG_RIGHT_KNEE_OFFSET       (-35.0f)
#endif

// 腿部工作空间限位
#ifndef LEG_LENGTH_MIN
#define LEG_LENGTH_MIN              0.07f   // 最小腿长 (米)
#endif

#ifndef LEG_LENGTH_MAX
#define LEG_LENGTH_MAX              0.17f   // 最大腿长 (米)
#endif

#ifndef LEG_BODY_ANGLE_MIN
#define LEG_BODY_ANGLE_MIN          (-160.0f) // 身体夹角最小值 (度), 向前蹬腿
#endif

#ifndef LEG_BODY_ANGLE_MAX
#define LEG_BODY_ANGLE_MAX          (-20.0f)  // 身体夹角最大值 (度), 向后蹬腿
#endif

// ============================================================================
// 数据结构
// ============================================================================

/**
 * @brief 腿部运动学参数
 */
typedef struct {
    float thigh_length;     // 大腿长度 (米)
    float shank_length;     // 小腿长度 (米)
    float hip_offset;       // 大腿电机零点偏移 (度)
    float knee_offset;      // 小腿电机零点偏移 (度)
} leg_kin_params_t;

/**
 * @brief 腿部关节空间状态 (电机角度)
 */
typedef struct {
    float hip_angle;        // 大腿电机角度 (度)
    float knee_angle;       // 小腿电机角度 (度)
} leg_joint_state_t;

/**
 * @brief 腿部工作空间状态 (腿长+身体夹角)
 */
typedef struct {
    float leg_length;       // 腿长 (米)
    float body_angle;       // 身体夹角 (度), 向前为正
} leg_workspace_state_t;

/**
 * @brief 完整腿部状态
 */
typedef struct {
    leg_joint_state_t joint;        // 关节空间
    leg_workspace_state_t workspace; // 工作空间
    bool is_left;                    // 是否为左腿
    bool valid;                      // 数据是否有效
} leg_state_t;

// ============================================================================
// API 函数
// ============================================================================

/**
 * @brief 获取默认腿部参数
 * @param is_left 是否为左腿
 * @param params 输出参数
 */
void leg_kin_get_default_params(bool is_left, leg_kin_params_t *params);

/**
 * @brief 正运动学: 关节空间 -> 工作空间
 * @param joint 关节空间状态 (电机角度)
 * @param is_left 是否为左腿
 * @param params 运动学参数 (NULL 使用默认参数)
 * @param workspace 输出工作空间状态
 * @return ESP_OK 成功
 */
esp_err_t leg_kin_forward(const leg_joint_state_t *joint, bool is_left,
                          const leg_kin_params_t *params,
                          leg_workspace_state_t *workspace);

/**
 * @brief 逆运动学: 工作空间 -> 关节空间
 * @param workspace 工作空间状态 (腿长+身体夹角)
 * @param is_left 是否为左腿
 * @param params 运动学参数 (NULL 使用默认参数)
 * @param joint 输出关节空间状态
 * @return ESP_OK 成功, ESP_ERR_INVALID_ARG 目标不可达
 */
esp_err_t leg_kin_inverse(const leg_workspace_state_t *workspace, bool is_left,
                          const leg_kin_params_t *params,
                          leg_joint_state_t *joint);

/**
 * @brief 检查工作空间目标是否可达
 * @param leg_length 目标腿长 (米)
 * @param body_angle 目标身体夹角 (度)
 * @param params 运动学参数 (NULL 使用默认参数)
 * @return true 可达, false 不可达
 */
bool leg_kin_is_reachable(float leg_length, float body_angle, 
                          const leg_kin_params_t *params);

/**
 * @brief 限制工作空间目标到可达范围
 * @param leg_length 输入/输出腿长 (米)
 * @param body_angle 输入/输出身体夹角 (度)
 * @param params 运动学参数 (NULL 使用默认参数)
 */
void leg_kin_clamp_workspace(float *leg_length, float *body_angle,
                             const leg_kin_params_t *params);

/**
 * @brief 计算腿部雅可比矩阵 (用于速度/力映射)
 * @param joint 当前关节状态
 * @param is_left 是否为左腿
 * @param params 运动学参数 (NULL 使用默认参数)
 * @param J 输出 2x2 雅可比矩阵 [J11, J12; J21, J22]
 * @return ESP_OK 成功
 * @note J 将关节速度映射到工作空间速度: [dL; dα] = J * [dθ1; dθ2]
 */
esp_err_t leg_kin_jacobian(const leg_joint_state_t *joint, bool is_left,
                           const leg_kin_params_t *params,
                           float J[4]);

/**
 * @brief 打印腿部参数信息
 * @param params 参数 (NULL 打印默认参数)
 * @param is_left 是否为左腿
 */
void leg_kin_print_params(const leg_kin_params_t *params, bool is_left);

#ifdef __cplusplus
}
#endif

#endif // LEG_KINEMATICS_H
