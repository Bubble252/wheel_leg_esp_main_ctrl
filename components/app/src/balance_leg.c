/**
 * @file balance_leg.c
 * @brief 腿部控制上层接口 (调用 leg_kinematics 模块)
 *
 * 从 balance_test.c 拆分出来。包含:
 * - leg_ctrl_init(): 初始化腿部控制
 * - leg_ctrl_get_state(): 读取当前腿部状态 (CAN 请求)
 * - leg_ctrl_get_state_cached(): 读取缓存状态 (无阻塞)
 * - leg_ctrl_set_target(): 设置目标腿长+角度
 * - leg_ctrl_set_both(): 设置双腿目标
 * - leg_ctrl_print_status(): 打印腿部状态
 */

#include "balance_test.h"
#include "can_motor.h"
#include "leg_kinematics.h"

#include "esp_log.h"
#include <stdio.h>
#include <math.h>

static const char *TAG = "LEG_CTRL";

// ============================================================================
// extern 变量 (定义在 balance_test.c)
// ============================================================================

extern bool g_leg_control_enabled;
extern float g_leg_base_length;
extern float g_leg_base_angle;
extern float g_leg_left_target_length;
extern float g_leg_left_target_angle;
extern float g_leg_right_target_length;
extern float g_leg_right_target_angle;
extern float g_leg_left_hip_angle;
extern float g_leg_left_knee_angle;
extern float g_leg_right_hip_angle;
extern float g_leg_right_knee_angle;
extern float g_leg_move_speed;
extern float g_leg_length_min;
extern float g_leg_length_max;

extern can_motor_handle_t g_motor_left_hip;
extern can_motor_handle_t g_motor_left_knee;
extern can_motor_handle_t g_motor_right_hip;
extern can_motor_handle_t g_motor_right_knee;


// ============================================================================
// 腿部控制上层接口 (调用 leg_kinematics 模块)
// ============================================================================

/**
 * @brief 初始化腿部控制
 */
void leg_ctrl_init(void) {
    // 设置初始目标为站立姿态 (垂直向下)
    g_leg_left_target_length = 0.09f;
    g_leg_left_target_angle = -90.0f;  // 垂直向下
    g_leg_right_target_length = 0.09f;
    g_leg_right_target_angle = -90.0f; // 垂直向下
    
    // 用逆运动学计算初始电机角度
    leg_workspace_state_t workspace;
    leg_joint_state_t joint;
    
    // 左腿
    workspace.leg_length = g_leg_left_target_length;
    workspace.body_angle = g_leg_left_target_angle;
    if (leg_kin_inverse(&workspace, true, NULL, &joint) == ESP_OK) {
        g_leg_left_hip_angle = joint.hip_angle;
        g_leg_left_knee_angle = joint.knee_angle;
        ESP_LOGI(TAG, "Left leg init: L=%.3f, A=%.1f -> Hip=%.1f, Knee=%.1f",
                 workspace.leg_length, workspace.body_angle,
                 g_leg_left_hip_angle, g_leg_left_knee_angle);
    }
    
    // 右腿
    workspace.leg_length = g_leg_right_target_length;
    workspace.body_angle = g_leg_right_target_angle;
    if (leg_kin_inverse(&workspace, false, NULL, &joint) == ESP_OK) {
        g_leg_right_hip_angle = joint.hip_angle;
        g_leg_right_knee_angle = joint.knee_angle;
        ESP_LOGI(TAG, "Right leg init: L=%.3f, A=%.1f -> Hip=%.1f, Knee=%.1f",
                 workspace.leg_length, workspace.body_angle,
                 g_leg_right_hip_angle, g_leg_right_knee_angle);
    }
    
    // 打印运动学参数
    leg_kin_print_params(NULL, true);
    leg_kin_print_params(NULL, false);
    
    ESP_LOGI(TAG, "Leg control initialized");
}

/**
 * @brief 主动请求腿部电机位置更新
 * @param is_left 是否为左腿
 * @return ESP_OK 成功
 */
static esp_err_t leg_ctrl_request_position(bool is_left) {
    can_motor_handle_t hip, knee;
    
    if (is_left) {
        hip = g_motor_left_hip;
        knee = g_motor_left_knee;
    } else {
        hip = g_motor_right_hip;
        knee = g_motor_right_knee;
    }
    
    if (hip == NULL || knee == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // 请求读取位置寄存器
    extern esp_err_t can_motor_request_status(can_motor_handle_t motor);
    can_motor_request_status(hip);
    can_motor_request_status(knee);
    
    // 等待 CAN 回复并处理
    vTaskDelay(pdMS_TO_TICKS(20));  // 给 CAN 接收任务时间处理回复
    
    return ESP_OK;
}

/**
 * @brief 读取当前腿部状态 (从编码器)
 * @note 会自动请求电机位置更新，确保读取到最新数据
 * @warning 此函数包含 CAN 请求和阻塞延时 (~48ms/次)，不要在控制循环中使用！
 *          控制循环中请使用 leg_ctrl_get_state_cached()
 */
esp_err_t leg_ctrl_get_state(bool is_left, leg_state_t *state) {
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 先请求位置更新
    leg_ctrl_request_position(is_left);
    
    leg_joint_state_t joint;
    
    if (is_left) {
        if (g_motor_left_hip == NULL || g_motor_left_knee == NULL) {
            state->valid = false;
            return ESP_ERR_INVALID_STATE;
        }
        joint.hip_angle = can_motor_read_position(g_motor_left_hip);
        joint.knee_angle = can_motor_read_position(g_motor_left_knee);
    } else {
        if (g_motor_right_hip == NULL || g_motor_right_knee == NULL) {
            state->valid = false;
            return ESP_ERR_INVALID_STATE;
        }
        joint.hip_angle = can_motor_read_position(g_motor_right_hip);
        joint.knee_angle = can_motor_read_position(g_motor_right_knee);
    }
    
    state->joint = joint;
    state->is_left = is_left;
    
    // 调用运动学正解
    esp_err_t ret = leg_kin_forward(&joint, is_left, NULL, &state->workspace);
    state->valid = (ret == ESP_OK);
    
    return ret;
}

/**
 * @brief 读取当前腿部状态 (从缓存，无阻塞)
 * @note 直接从电机驱动缓存中读取位置，不发送 CAN 请求。
 *       适合在 200Hz 控制循环中使用，延迟 < 1us。
 *       数据由 can_motor_process_rx() 在电机通信中自动更新。
 */
esp_err_t leg_ctrl_get_state_cached(bool is_left, leg_state_t *state) {
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    leg_joint_state_t joint;
    
    if (is_left) {
        if (g_motor_left_hip == NULL || g_motor_left_knee == NULL) {
            state->valid = false;
            return ESP_ERR_INVALID_STATE;
        }
        joint.hip_angle = can_motor_read_position(g_motor_left_hip);
        joint.knee_angle = can_motor_read_position(g_motor_left_knee);
    } else {
        if (g_motor_right_hip == NULL || g_motor_right_knee == NULL) {
            state->valid = false;
            return ESP_ERR_INVALID_STATE;
        }
        joint.hip_angle = can_motor_read_position(g_motor_right_hip);
        joint.knee_angle = can_motor_read_position(g_motor_right_knee);
    }
    
    state->joint = joint;
    state->is_left = is_left;
    
    // 调用运动学正解 (纯数学计算，无阻塞)
    esp_err_t ret = leg_kin_forward(&joint, is_left, NULL, &state->workspace);
    state->valid = (ret == ESP_OK);
    
    return ret;
}

/**
 * @brief 设置目标腿部状态 (腿长 + 身体夹角)
 * @note 此函数会同时更新基础腿长，用于用户手动设置高度
 *       Roll 控制内部不应调用此函数，以避免修改基础值
 */
esp_err_t leg_ctrl_set_target(bool is_left, float leg_length, float body_angle) {
    // 使用动态可调的腿长范围做限幅
    if (leg_length < g_leg_length_min) leg_length = g_leg_length_min;
    if (leg_length > g_leg_length_max) leg_length = g_leg_length_max;
    
    // 运动学库的 clamp (包含几何限制)
    leg_kin_clamp_workspace(&leg_length, &body_angle, NULL);
    
    // 检查可达性
    if (!leg_kin_is_reachable(leg_length, body_angle, NULL)) {
        ESP_LOGW(TAG, "Target unreachable: L=%.3f, A=%.1f", leg_length, body_angle);
        return ESP_ERR_INVALID_ARG;
    }
    
    // 调用运动学逆解
    leg_workspace_state_t workspace = { .leg_length = leg_length, .body_angle = body_angle };
    leg_joint_state_t joint;
    
    esp_err_t ret = leg_kin_inverse(&workspace, is_left, NULL, &joint);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 更新基础腿长 (用户设定的高度)
    // 当用户手动设置时，左右腿应该设置相同的基础值
    g_leg_base_length = leg_length;
    g_leg_base_angle = body_angle;
    
    // 更新实际目标
    if (is_left) {
        g_leg_left_target_length = leg_length;
        g_leg_left_target_angle = body_angle;
        g_leg_left_hip_angle = joint.hip_angle;
        g_leg_left_knee_angle = joint.knee_angle;
    } else {
        g_leg_right_target_length = leg_length;
        g_leg_right_target_angle = body_angle;
        g_leg_right_hip_angle = joint.hip_angle;
        g_leg_right_knee_angle = joint.knee_angle;
    }
    
    // 限流日志：仅在腿长变化超过阈值时打印
    static float s_last_log_left_len = 0, s_last_log_right_len = 0;
    float *p_last_len = is_left ? &s_last_log_left_len : &s_last_log_right_len;
    if (fabsf(leg_length - *p_last_len) > 0.005f) {  // 变化超过 5mm 才打印
        ESP_LOGI(TAG, "%s leg target: L=%.3fm, A=%.1fdeg -> Hip=%.1f, Knee=%.1f",
                 is_left ? "Left" : "Right", leg_length, body_angle, 
                 joint.hip_angle, joint.knee_angle);
        *p_last_len = leg_length;
    }
    
    return ESP_OK;
}

/**
 * @brief 设置双腿目标状态
 */
esp_err_t leg_ctrl_set_both(float left_length, float left_angle,
                             float right_length, float right_angle) {
    esp_err_t ret1 = leg_ctrl_set_target(true, left_length, left_angle);
    esp_err_t ret2 = leg_ctrl_set_target(false, right_length, right_angle);
    
    if (ret1 != ESP_OK) return ret1;
    if (ret2 != ESP_OK) return ret2;
    
    return ESP_OK;
}

/**
 * @brief 打印当前腿部状态
 */
void leg_ctrl_print_status(void) {
    leg_state_t left, right;
    
    // 获取当前状态
    bool left_valid = (leg_ctrl_get_state(true, &left) == ESP_OK && left.valid);
    bool right_valid = (leg_ctrl_get_state(false, &right) == ESP_OK && right.valid);
    
    // 输出机器可解析格式 (供 Qt 面板使用)
    // 格式: LEG_STATE: L_Len=xxx L_Ang=xxx L_Hip=xxx L_Knee=xxx R_Len=xxx R_Ang=xxx R_Hip=xxx R_Knee=xxx
    printf("LEG_STATE: L_Len=%.3f L_Ang=%.1f L_Hip=%.1f L_Knee=%.1f R_Len=%.3f R_Ang=%.1f R_Hip=%.1f R_Knee=%.1f\n",
           left_valid ? left.workspace.leg_length : 0.0f,
           left_valid ? left.workspace.body_angle : 0.0f,
           left_valid ? left.joint.hip_angle : 0.0f,
           left_valid ? left.joint.knee_angle : 0.0f,
           right_valid ? right.workspace.leg_length : 0.0f,
           right_valid ? right.workspace.body_angle : 0.0f,
           right_valid ? right.joint.hip_angle : 0.0f,
           right_valid ? right.joint.knee_angle : 0.0f);
    
    // 输出人类可读格式
    printf("=== Leg Control Status ===\n");
    printf("Control: %s\n", g_leg_control_enabled ? "ENABLED" : "DISABLED");
    printf("Speed: %.1f rpm\n\n", g_leg_move_speed);
    
    // 左腿
    if (left_valid) {
        printf("Left Leg (current):\n");
        printf("  Motor: Hip=%.1f deg, Knee=%.1f deg\n", 
               left.joint.hip_angle, left.joint.knee_angle);
        printf("  State: Length=%.3f m, BodyAngle=%.1f deg\n", 
               left.workspace.leg_length, left.workspace.body_angle);
    } else {
        printf("Left Leg: NOT AVAILABLE\n");
    }
    printf("  Target: Length=%.3f m, BodyAngle=%.1f deg\n", 
           g_leg_left_target_length, g_leg_left_target_angle);
    printf("  Target Motor: Hip=%.1f, Knee=%.1f\n\n", 
           g_leg_left_hip_angle, g_leg_left_knee_angle);
    
    // 右腿
    if (right_valid) {
        printf("Right Leg (current):\n");
        printf("  Motor: Hip=%.1f deg, Knee=%.1f deg\n", 
               right.joint.hip_angle, right.joint.knee_angle);
        printf("  State: Length=%.3f m, BodyAngle=%.1f deg\n", 
               right.workspace.leg_length, right.workspace.body_angle);
    } else {
        printf("Right Leg: NOT AVAILABLE\n");
    }
    printf("  Target: Length=%.3f m, BodyAngle=%.1f deg\n", 
           g_leg_right_target_length, g_leg_right_target_angle);
    printf("  Target Motor: Hip=%.1f, Knee=%.1f\n", 
           g_leg_right_hip_angle, g_leg_right_knee_angle);
}

