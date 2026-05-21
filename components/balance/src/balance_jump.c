#include "balance_jump.h"
#include "can_motor.h"
#include "can_motor_stw_regs.h"
#include "leg_kinematics.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <math.h>

static const char *TAG = "BAL_JUMP";

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define DEG2RAD(d) ((d) * M_PI / 180.0f)

// ============================================================================
// 外部函数声明 (定义在 balance_test.c)
// ============================================================================

extern esp_err_t leg_ctrl_set_target(bool is_left, float leg_length, float body_angle);
extern esp_err_t leg_ctrl_get_state_cached(bool is_left, leg_state_t *state);

// ============================================================================
// 外部变量 (定义在 balance_test.c)
// ============================================================================

extern can_motor_handle_t g_motor_left_hip;
extern can_motor_handle_t g_motor_left_knee;
extern can_motor_handle_t g_motor_right_hip;
extern can_motor_handle_t g_motor_right_knee;

extern float g_leg_base_length;
extern float g_leg_base_angle;

extern float g_leg_left_hip_angle;
extern float g_leg_left_knee_angle;
extern float g_leg_right_hip_angle;
extern float g_leg_right_knee_angle;

extern float g_joint_normal_max_speed_rpm;
extern float g_joint_normal_pos_kp;

// ============================================================================
// 跳跃状态变量 (内部)
// ============================================================================

static jump_state_t g_jump_state = JUMP_IDLE;
static uint32_t g_jump_state_enter_ms = 0;
static float g_jump_saved_leg_length = 0.09f;
static float g_jump_saved_base_angle = -90.0f;
static bool g_jump_last_btn = false;

// 跳跃参数 (可调)
static float g_jump_crouch_length = 0.068f;
static float g_jump_extend_length = 0.110f;
static float g_jump_retract_length = 0.068f;
static float g_jump_pos_threshold = 0.008f;
static uint32_t g_jump_timeout_ms = 2000;
static uint32_t g_jump_extend_hold_ms = 150;
static uint32_t g_jump_air_hold_ms = 300;
static float g_jump_max_speed_rpm = 400.0f;
static float g_jump_retract_pos_kp = 2.0f;

// MIT 模式蹬伸参数 (extern 给 apply_leg_motor_commands 使用)
bool g_jump_mit_active = false;
float g_jump_mit_kp = 3.0f;
float g_jump_mit_kd = 0.1f;
float g_jump_mit_ff_torque = 1.5f;
static float g_jump_mit_vel_max = 41.9f;
float g_jump_mit_target_rad[4] = {0};
float g_jump_mit_ff_sign[4] = {1, 1, 1, 1};

// ============================================================================
// 状态机更新 (原样从 balance_test.c 移动)
// ============================================================================

void jump_state_machine_update(void) {
    if (g_jump_state == JUMP_IDLE) return;

    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t elapsed = now_ms - g_jump_state_enter_ms;

    // 读取当前实际腿长 (编码器 FK)
    leg_state_t left_state, right_state;
    float avg_leg_length = 0.0f;
    bool fk_valid = false;
    if (leg_ctrl_get_state_cached(true, &left_state) == ESP_OK &&
        leg_ctrl_get_state_cached(false, &right_state) == ESP_OK &&
        left_state.valid && right_state.valid) {
        avg_leg_length = (left_state.workspace.leg_length + right_state.workspace.leg_length) / 2.0f;
        fk_valid = true;
    }

    // 调试: 每 200ms 打印一次
    static uint32_t last_jump_debug_ms = 0;
    if (now_ms - last_jump_debug_ms >= 200) {
        ESP_LOGW(TAG, "JUMP: s=%d t=%lums leg=%.0fmm fk=%d",
                 (int)g_jump_state, (unsigned long)elapsed,
                 avg_leg_length * 1000.0f, (int)fk_valid);
        last_jump_debug_ms = now_ms;
    }

    switch (g_jump_state) {
        case JUMP_IDLE:
            break;

        case JUMP_CROUCH: {
            // 蹲下蓄力: 等实际腿长到达蹲下位置附近, 或超时
            bool reached = fk_valid && (fabsf(avg_leg_length - g_jump_crouch_length) < g_jump_pos_threshold);
            bool timeout = (elapsed >= g_jump_timeout_ms);
            if (reached || timeout) {
                g_jump_state = JUMP_EXTEND;
                g_jump_state_enter_ms = now_ms;
                // 保存蹲下时的关节角度 (算法空间, rad)
                float crouch_rad[4] = {
                    DEG2RAD(g_leg_left_hip_angle),
                    DEG2RAD(g_leg_left_knee_angle),
                    DEG2RAD(g_leg_right_hip_angle),
                    DEG2RAD(g_leg_right_knee_angle)
                };
                // 计算蹬伸目标关节角度 (IK)
                leg_ctrl_set_target(true, g_jump_extend_length, g_jump_saved_base_angle+10.0f);
                leg_ctrl_set_target(false, g_jump_extend_length, g_jump_saved_base_angle+10.0f);
                // 保存目标角度为 rad (MIT 使用弧度)
                g_jump_mit_target_rad[0] = DEG2RAD(g_leg_left_hip_angle);
                g_jump_mit_target_rad[1] = DEG2RAD(g_leg_left_knee_angle);
                g_jump_mit_target_rad[2] = DEG2RAD(g_leg_right_hip_angle);
                g_jump_mit_target_rad[3] = DEG2RAD(g_leg_right_knee_angle);
                // 根据运动方向确定每个电机的前馈力矩符号
                for (int i = 0; i < 4; i++) {
                    g_jump_mit_ff_sign[i] = (g_jump_mit_target_rad[i] >= crouch_rad[i]) ? 1.0f : -1.0f;
                }
                // 激活 MIT 模式蹬伸
                g_jump_mit_active = true;
                ESP_LOGW(TAG, "JUMP: EXTEND(MIT)! leg=%.0fmm->%.0fmm kp=%.1f kd=%.2f ff=%.2f (%s)",
                         avg_leg_length * 1000.0f, g_jump_extend_length * 1000.0f,
                         g_jump_mit_kp, g_jump_mit_kd, g_jump_mit_ff_torque,
                         reached ? "reached" : "timeout");
            }
            break;
        }

        case JUMP_EXTEND: {
            // 蹬伸起跳: MIT 阻抗控制不精确收敛, 纯时间退出
            // 蹬伸 extend_hold_ms 后立即收腿, 不等位置到达
            if (elapsed >= g_jump_extend_hold_ms) {
                // 退出 MIT 模式, 恢复位置控制
                g_jump_mit_active = false;
                g_jump_state = JUMP_AIR_RETRACT;
                g_jump_state_enter_ms = now_ms;
                // 收腿前提高位置环 Kp, 加快收腿响应
                can_motor_stw_write_pid(g_motor_left_hip, STW_CMD_POS_KP, g_jump_retract_pos_kp);
                can_motor_stw_write_pid(g_motor_left_knee, STW_CMD_POS_KP, g_jump_retract_pos_kp);
                can_motor_stw_write_pid(g_motor_right_hip, STW_CMD_POS_KP, g_jump_retract_pos_kp);
                can_motor_stw_write_pid(g_motor_right_knee, STW_CMD_POS_KP, g_jump_retract_pos_kp);
                leg_ctrl_set_target(true, g_jump_retract_length, g_jump_saved_base_angle);
                leg_ctrl_set_target(false, g_jump_retract_length, g_jump_saved_base_angle);
                ESP_LOGW(TAG, "JUMP: AIR_RETRACT! leg=%.0fmm->%.0fmm pos_kp=%.1f (hold %lums)",
                         avg_leg_length * 1000.0f, g_jump_retract_length * 1000.0f,
                         g_jump_retract_pos_kp, (unsigned long)elapsed);
            }
            break;
        }

        case JUMP_AIR_RETRACT: {
            // 空中收腿: 等实际腿长到达收腿位置, 或超时
            // 收腿完成后直接回 IDLE, 不恢复腿长, 等遥控下一条腿长指令接管
            bool reached = fk_valid && (fabsf(avg_leg_length - g_jump_retract_length) < g_jump_pos_threshold);
            bool timeout = (elapsed >= g_jump_timeout_ms);
            bool min_hold = (elapsed >= g_jump_air_hold_ms);
            if ((reached && min_hold) || timeout) {
                // 恢复正常关节最大速度和位置 Kp
                can_motor_stw_set_max_speed(g_motor_left_hip, g_joint_normal_max_speed_rpm);
                can_motor_stw_set_max_speed(g_motor_left_knee, g_joint_normal_max_speed_rpm);
                can_motor_stw_set_max_speed(g_motor_right_hip, g_joint_normal_max_speed_rpm);
                can_motor_stw_set_max_speed(g_motor_right_knee, g_joint_normal_max_speed_rpm);
                if (g_joint_normal_pos_kp > 0.0f) {
                    can_motor_stw_write_pid(g_motor_left_hip, STW_CMD_POS_KP, g_joint_normal_pos_kp);
                    can_motor_stw_write_pid(g_motor_left_knee, STW_CMD_POS_KP, g_joint_normal_pos_kp);
                    can_motor_stw_write_pid(g_motor_right_hip, STW_CMD_POS_KP, g_joint_normal_pos_kp);
                    can_motor_stw_write_pid(g_motor_right_knee, STW_CMD_POS_KP, g_joint_normal_pos_kp);
                }
                // 恢复跳跃前的腿长和夹角 (防止停在收腿位置)
                leg_ctrl_set_target(true,  g_jump_saved_leg_length, g_jump_saved_base_angle);
                leg_ctrl_set_target(false, g_jump_saved_leg_length, g_jump_saved_base_angle);
                g_jump_state = JUMP_IDLE;
                ESP_LOGW(TAG, "JUMP: IDLE (complete, leg=%.0fmm->%.0fmm, angle=%.1fdeg, %s)",
                         avg_leg_length * 1000.0f, g_jump_saved_leg_length * 1000.0f,
                         g_jump_saved_base_angle, reached ? "reached" : "timeout");
            }
            break;
        }

        case JUMP_RECOVER:
            // 不再使用, 直接回 IDLE
            g_jump_state = JUMP_IDLE;
            break;
    }
}

bool jump_is_active(void) {
    return g_jump_state != JUMP_IDLE;
}

bool jump_wants_zero_wheel(void) {
    return g_jump_state == JUMP_AIR_RETRACT;
}

bool jump_mit_is_active(void) {
    return g_jump_mit_active;
}

bool jump_trigger(float leg_length, float base_angle) {
    if (g_jump_state != JUMP_IDLE) return false;
    g_jump_saved_leg_length = leg_length;
    g_jump_saved_base_angle = base_angle;
    g_jump_state = JUMP_CROUCH;
    g_jump_state_enter_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    // 跳跃前提升关节最大速度
    can_motor_stw_set_max_speed(g_motor_left_hip, g_jump_max_speed_rpm);
    can_motor_stw_set_max_speed(g_motor_left_knee, g_jump_max_speed_rpm);
    can_motor_stw_set_max_speed(g_motor_right_hip, g_jump_max_speed_rpm);
    can_motor_stw_set_max_speed(g_motor_right_knee, g_jump_max_speed_rpm);
    // 蹲下蓄力
    leg_ctrl_set_target(true, g_jump_crouch_length, base_angle);
    leg_ctrl_set_target(false, g_jump_crouch_length, base_angle);
    ESP_LOGW(TAG, "JUMP: START! saved_leg=%.0fmm, crouch=%.0fmm, spd=%.0f",
             leg_length * 1000.0f, g_jump_crouch_length * 1000.0f, g_jump_max_speed_rpm);
    return true;
}

void jump_set_btn(bool pressed) {
    g_jump_last_btn = pressed;
}