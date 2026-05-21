#include "balance_vmc.h"
#include "balance_types.h"
#include "can_motor.h"
#include "leg_kinematics.h"
#include "lqr_balance.h"
#include "full_lqr.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <math.h>

#define WHEEL_RADIUS_M 0.05f

// ============================================================================
// 外部变量 (定义在 balance_test.c)
// ============================================================================

extern can_motor_handle_t g_motor_left_hip;
extern can_motor_handle_t g_motor_left_knee;
extern can_motor_handle_t g_motor_right_hip;
extern can_motor_handle_t g_motor_right_knee;

extern float g_leg_left_target_length;
extern float g_leg_left_target_angle;
extern float g_leg_right_target_length;
extern float g_leg_right_target_angle;

extern bool g_leg_control_enabled;
extern control_mode_t g_control_mode;

extern shared_wheel_state_t g_wheel_state;
extern SemaphoreHandle_t g_wheel_state_mutex;

extern float g_joint_lh_spd_filtered;
extern float g_joint_lk_spd_filtered;
extern float g_joint_rh_spd_filtered;
extern float g_joint_rk_spd_filtered;

extern bool g_full_lqr_initialized;
extern float g_full_lqr_Tp_left;
extern float g_full_lqr_Tp_right;

// ============================================================================
// VMC 状态变量 (non-static, 供 balance_test.c 和 CLI 访问)
// ============================================================================

bool g_vmc_enabled = false;
vmc_params_t g_vmc_params;
float g_vmc_target_vx = 0.0f;
float g_vmc_target_y = 0.09f;
vmc_dual_output_t g_vmc_dual_output = {0};
bool g_vmc_input_valid = false;
bool g_vmc_stream_enable = false;

// 支持力估计
float g_support_force_left_FL = 0.0f;
float g_support_force_right_FL = 0.0f;
float g_support_force_left_Fa = 0.0f;
float g_support_force_right_Fa = 0.0f;
bool g_sforce_stream_enable = false;

// 离地检测
int g_sforce_left_off_cnt = 0;
int g_sforce_right_off_cnt = 0;
bool g_sforce_left_off = false;
bool g_sforce_right_off = false;

// SFORCE_FL_THRESHOLD 等宏在 balance_vmc.h 中定义

// ============================================================================
// 支持力估计
// ============================================================================

void compute_support_force(void) {
    bool left_valid  = (g_motor_left_hip  && g_motor_left_knee);
    bool right_valid = (g_motor_right_hip && g_motor_right_knee);

    if (!left_valid && !right_valid) return;

    if (!g_vmc_enabled || !g_leg_control_enabled) {
        can_motor_process_rx();
    }

    float l_hip_actual_Nm  = left_valid  ? vmc_current_to_torque(can_motor_read_current(g_motor_left_hip))  : 0;
    float l_knee_actual_Nm = left_valid  ? vmc_current_to_torque(can_motor_read_current(g_motor_left_knee)) : 0;
    float r_hip_actual_Nm  = right_valid ? vmc_current_to_torque(can_motor_read_current(g_motor_right_hip))  : 0;
    float r_knee_actual_Nm = right_valid ? vmc_current_to_torque(can_motor_read_current(g_motor_right_knee)) : 0;

    r_hip_actual_Nm  = -r_hip_actual_Nm;
    r_knee_actual_Nm = -r_knee_actual_Nm;

    float l_FL = 0, l_Fa = 0, r_FL = 0, r_Fa = 0;

    if (left_valid) {
        leg_joint_state_t lj = { .hip_angle = can_motor_read_position(g_motor_left_hip),
                                 .knee_angle = can_motor_read_position(g_motor_left_knee) };
        float J[4];
        leg_kin_jacobian(&lj, true, NULL, J);
        float det = J[0] * J[3] - J[1] * J[2];
        if (fabsf(det) > 1e-6f) {
            float inv_det = 1.0f / det;
            l_FL = inv_det * ( J[3] * l_hip_actual_Nm - J[2] * l_knee_actual_Nm);
            l_Fa = inv_det * (-J[1] * l_hip_actual_Nm + J[0] * l_knee_actual_Nm);
        }
    }
    if (right_valid) {
        leg_joint_state_t rj = { .hip_angle = can_motor_read_position(g_motor_right_hip),
                                 .knee_angle = can_motor_read_position(g_motor_right_knee) };
        float J[4];
        leg_kin_jacobian(&rj, false, NULL, J);
        float det = J[0] * J[3] - J[1] * J[2];
        if (fabsf(det) > 1e-6f) {
            float inv_det = 1.0f / det;
            r_FL = inv_det * ( J[3] * r_hip_actual_Nm - J[2] * r_knee_actual_Nm);
            r_Fa = inv_det * (-J[1] * r_hip_actual_Nm + J[0] * r_knee_actual_Nm);
        }
    }

    g_support_force_left_FL  = l_FL;
    g_support_force_right_FL = r_FL;
    g_support_force_left_Fa  = l_Fa;
    g_support_force_right_Fa = r_Fa;
}

// ============================================================================
// VMC 腿部状态计算
// ============================================================================

void vmc_compute_leg_state(const lqr_input_t *lqr_input) {
    if (!g_vmc_enabled || !g_leg_control_enabled) {
        g_vmc_input_valid = false;
        return;
    }

    can_motor_process_rx();

    float pitch_deg = 0.0f, pitch_rate_deg = 0.0f;
    if (lqr_input != NULL) {
        pitch_deg = lqr_input->raw_pitch;
        pitch_rate_deg = lqr_input->pitch_rate;
    }

    const float rpm_to_rad_s = 3.14159f / 30.0f;
    xSemaphoreTake(g_wheel_state_mutex, portMAX_DELAY);
    float avg_wheel_rpm = (g_wheel_state.left_speed + g_wheel_state.right_speed) / 2.0f;
    xSemaphoreGive(g_wheel_state_mutex);
    float robot_vx = -avg_wheel_rpm * rpm_to_rad_s * WHEEL_RADIUS_M;

    bool left_valid = (g_motor_left_hip && g_motor_left_knee);
    bool right_valid = (g_motor_right_hip && g_motor_right_knee);

    if (left_valid || right_valid) {
        bool need_velocity = (g_vmc_params.vmc_diff_method == VMC_DIFF_JACOBIAN);

        float lh_vel = need_velocity ? g_joint_lh_spd_filtered : 0.0f;
        float lk_vel = need_velocity ? g_joint_lk_spd_filtered : 0.0f;
        float rh_vel = need_velocity ? g_joint_rh_spd_filtered : 0.0f;
        float rk_vel = need_velocity ? g_joint_rk_spd_filtered : 0.0f;

        vmc_dual_input_t dual_input = {
            .pitch_deg = pitch_deg,
            .pitch_rate_deg = pitch_rate_deg,
            .robot_vx = robot_vx,
            .target_vx = g_vmc_target_vx,
            .left = {
                .target_leg_length = g_leg_left_target_length,
                .target_body_angle_deg = g_leg_left_target_angle,
                .sensor = {
                    .hip_angle = left_valid ? can_motor_read_position(g_motor_left_hip) : 0,
                    .knee_angle = left_valid ? can_motor_read_position(g_motor_left_knee) : 0,
                    .hip_velocity = lh_vel,
                    .knee_velocity = lk_vel
                }
            },
            .right = {
                .target_leg_length = g_leg_right_target_length,
                .target_body_angle_deg = g_leg_right_target_angle,
                .sensor = {
                    .hip_angle = right_valid ? can_motor_read_position(g_motor_right_hip) : 0,
                    .knee_angle = right_valid ? can_motor_read_position(g_motor_right_knee) : 0,
                    .hip_velocity = rh_vel,
                    .knee_velocity = rk_vel
                }
            }
        };

        if (vmc_dual_compute(&g_vmc_params, &dual_input, &g_vmc_dual_output) == ESP_OK) {
            g_vmc_input_valid = true;

            // Full LQR Tp 注入 (替代 F_alpha)
            if (g_control_mode == CTRL_MODE_FULL_LQR && g_full_lqr_initialized) {
                leg_joint_state_t left_joint_for_tp = {
                    .hip_angle = left_valid ? can_motor_read_position(g_motor_left_hip) : 0,
                    .knee_angle = left_valid ? can_motor_read_position(g_motor_left_knee) : 0,
                };
                leg_joint_state_t right_joint_for_tp = {
                    .hip_angle = right_valid ? can_motor_read_position(g_motor_right_hip) : 0,
                    .knee_angle = right_valid ? can_motor_read_position(g_motor_right_knee) : 0,
                };

                float J_left[4], J_right[4];
                leg_kin_jacobian(&left_joint_for_tp, true, NULL, J_left);
                leg_kin_jacobian(&right_joint_for_tp, false, NULL, J_right);

                float F_alpha_left  = g_vmc_dual_output.left.debug.F_alpha;
                float F_alpha_right = g_vmc_dual_output.right.debug.F_alpha;

                float Tp_left = g_full_lqr_Tp_left;
                float delta_left = Tp_left - F_alpha_left;
                g_vmc_dual_output.left.hip_torque  += J_left[2] * delta_left;
                g_vmc_dual_output.left.knee_torque += J_left[3] * delta_left;

                float Tp_right = g_full_lqr_Tp_right;
                float delta_right = Tp_right - F_alpha_right;
                g_vmc_dual_output.right.hip_torque  += -(J_right[2] * delta_right);
                g_vmc_dual_output.right.knee_torque += -(J_right[3] * delta_right);
            }

            if (g_vmc_stream_enable) {
                printf("#VMC,%.3f,%.1f,%.2f,%.3f,%.2f,%.2f,%.3f,%.1f,%.2f,%.3f,%.2f,%.2f,%.2f,%.3f,%.2f,%.3f,%.2f,%.3f\n",
                       g_vmc_dual_output.left.current_leg_length,
                       g_vmc_dual_output.left.current_body_angle,
                       g_vmc_dual_output.left.debug.F_L,
                       g_vmc_dual_output.left.debug.F_alpha,
                       g_vmc_dual_output.left.hip_torque,
                       g_vmc_dual_output.left.knee_torque,
                       g_vmc_dual_output.right.current_leg_length,
                       g_vmc_dual_output.right.current_body_angle,
                       g_vmc_dual_output.right.debug.F_L,
                       g_vmc_dual_output.right.debug.F_alpha,
                       g_vmc_dual_output.right.hip_torque,
                       g_vmc_dual_output.right.knee_torque,
                       g_vmc_dual_output.angle_diff_deg,
                       g_vmc_dual_output.F_sync,
                       g_support_force_left_FL, g_support_force_left_Fa,
                       g_support_force_right_FL, g_support_force_right_Fa);
            }
        }
    }
}
