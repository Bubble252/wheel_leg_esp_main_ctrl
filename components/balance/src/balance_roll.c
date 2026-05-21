#include "balance_roll.h"
#include "leg_kinematics.h"
#include <math.h>

// ============================================================================
// 外部变量 (定义在 balance_test.c)
// ============================================================================

extern float g_leg_length_min;
extern float g_leg_length_max;

extern float g_leg_left_target_length;
extern float g_leg_left_target_angle;
extern float g_leg_left_hip_angle;
extern float g_leg_left_knee_angle;

extern float g_leg_right_target_length;
extern float g_leg_right_target_angle;
extern float g_leg_right_hip_angle;
extern float g_leg_right_knee_angle;

// ============================================================================
// 实现
// ============================================================================

void apply_xoffset_and_ik(float *left_length, float *left_angle,
                           float *right_length, float *right_angle,
                           float xoffset)
{
    // X-Offset: 笛卡尔空间偏移
    if (fabsf(xoffset) > 0.0001f) {
        float lx, ly;
        leg_kin_polar_to_cartesian(*left_length, *left_angle, &lx, &ly);
        lx += xoffset;
        leg_kin_clamp_cartesian_body(&lx, &ly, NULL);
        leg_kin_cartesian_to_polar(lx, ly, left_length, left_angle);

        float rx, ry;
        leg_kin_polar_to_cartesian(*right_length, *right_angle, &rx, &ry);
        rx += xoffset;
        leg_kin_clamp_cartesian_body(&rx, &ry, NULL);
        leg_kin_cartesian_to_polar(rx, ry, right_length, right_angle);
    }

    // 腿长限幅
    if (*left_length < g_leg_length_min) *left_length = g_leg_length_min;
    if (*left_length > g_leg_length_max) *left_length = g_leg_length_max;
    if (*right_length < g_leg_length_min) *right_length = g_leg_length_min;
    if (*right_length > g_leg_length_max) *right_length = g_leg_length_max;

    // 工作空间限幅
    leg_kin_clamp_workspace(left_length, left_angle, NULL);
    leg_kin_clamp_workspace(right_length, right_angle, NULL);

    // 逆运动学 → 更新全局腿目标
    leg_workspace_state_t left_ws = { .leg_length = *left_length, .body_angle = *left_angle };
    leg_workspace_state_t right_ws = { .leg_length = *right_length, .body_angle = *right_angle };
    leg_joint_state_t left_joint, right_joint;

    if (leg_kin_inverse(&left_ws, true, NULL, &left_joint) == ESP_OK) {
        g_leg_left_target_length = *left_length;
        g_leg_left_target_angle = *left_angle;
        g_leg_left_hip_angle = left_joint.hip_angle;
        g_leg_left_knee_angle = left_joint.knee_angle;
    }
    if (leg_kin_inverse(&right_ws, false, NULL, &right_joint) == ESP_OK) {
        g_leg_right_target_length = *right_length;
        g_leg_right_target_angle = *right_angle;
        g_leg_right_hip_angle = right_joint.hip_angle;
        g_leg_right_knee_angle = right_joint.knee_angle;
    }
}

void apply_xoffset_single(float base_length, float base_angle, float xoffset)
{
    float new_length = base_length;
    float new_angle = base_angle;

    if (fabsf(xoffset) > 0.0001f) {
        float x, y;
        leg_kin_polar_to_cartesian(new_length, new_angle, &x, &y);
        x += xoffset;
        leg_kin_clamp_cartesian_body(&x, &y, NULL);
        leg_kin_cartesian_to_polar(x, y, &new_length, &new_angle);
    }

    if (new_length < g_leg_length_min) new_length = g_leg_length_min;
    if (new_length > g_leg_length_max) new_length = g_leg_length_max;

    leg_kin_clamp_workspace(&new_length, &new_angle, NULL);

    leg_workspace_state_t ws = { .leg_length = new_length, .body_angle = new_angle };
    leg_joint_state_t left_joint, right_joint;

    if (leg_kin_inverse(&ws, true, NULL, &left_joint) == ESP_OK) {
        g_leg_left_target_length = new_length;
        g_leg_left_target_angle = new_angle;
        g_leg_left_hip_angle = left_joint.hip_angle;
        g_leg_left_knee_angle = left_joint.knee_angle;
    }
    if (leg_kin_inverse(&ws, false, NULL, &right_joint) == ESP_OK) {
        g_leg_right_target_length = new_length;
        g_leg_right_target_angle = new_angle;
        g_leg_right_hip_angle = right_joint.hip_angle;
        g_leg_right_knee_angle = right_joint.knee_angle;
    }
}
