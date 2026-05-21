#include "balance_standup.h"
#include "balance_types.h"
#include "can_motor.h"
#include "can_motor_stw_regs.h"
#include "leg_kinematics.h"
#include "lqr_balance.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <math.h>

static const char *TAG = "BAL_STANDUP";

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define DEG2RAD(d) ((d) * M_PI / 180.0f)

// 共享常量在 balance_types.h 中 (通过 balance_standup.h 包含)
// CAR_MODE_BODY_ANGLE, CAR_MODE_LEG_LENGTH, CAR_TO_BAL_LEG_LENGTH, etc.

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

extern can_motor_handle_t g_motor_left;
extern can_motor_handle_t g_motor_right;

extern float g_leg_base_length;
extern float g_leg_base_angle;
extern float g_leg_length_min;

extern float g_leg_left_hip_angle;
extern float g_leg_left_knee_angle;
extern float g_leg_right_hip_angle;
extern float g_leg_right_knee_angle;

extern float g_leg_move_speed;

extern control_mode_t g_control_mode;
extern balance_test_state_t g_state;
extern bool g_leg_control_enabled;

extern control_mode_t g_car_mode_prev_mode;
extern float g_car_mode_prev_base_angle;
extern float g_car_mode_prev_base_length;

extern int g_uncontrolable;
extern float g_lqr_distance;
extern float g_distance_zeropoint;

extern lqr_controller_t g_lqr_ctrl;
extern triple_pid_controller_t g_triple_pid_ctrl;

extern shared_wheel_cmd_t g_wheel_cmd;
extern SemaphoreHandle_t g_wheel_cmd_mutex;

// ============================================================================
// 起身状态变量 (non-static, 供 balance_test.c 的 WiFi handler 访问)
// ============================================================================

standup_state_t g_standup_state = STANDUP_IDLE;
uint32_t g_standup_state_enter_ms = 0;
bool g_standup_last_btn = false;

car_standup_state_t g_car_standup_state = CAR_STANDUP_IDLE;
uint32_t g_car_standup_enter_ms = 0;
bool g_car_standup_last_btn = false;

bal_to_car_state_t g_bal_to_car_state = BAL_TO_CAR_IDLE;
uint32_t g_bal_to_car_enter_ms = 0;

// 起身参数 (可调, extern 给 WiFi handler 和 CLI 使用)
float g_standup_retract_length = 0.065f;
float g_standup_retract_angle = 0.0f;
float g_standup_zero_threshold = 45.0f;

static float g_standup_mit_ff_torque_val = 0.08f;
static float g_standup_wheel_speed = -200.0f;
static float g_standup_angle_threshold = 30.0f;
static uint32_t g_standup_retract_timeout_ms = 1000;
static uint32_t g_standup_roll_timeout_ms = 2000;
static float g_standup_extend_length = 0.105f;
static uint32_t g_standup_extend_timeout_ms = 500;
static uint32_t g_standup_wait_ms = 1000;

// 起身 MIT 控制状态 (extern 给 apply_leg_motor_commands 使用)
bool g_standup_mit_active = false;
bool g_standup_mit_left_hip = false;
bool g_standup_mit_right_hip = false;
float g_standup_ff_sign_left = 1.0f;
float g_standup_ff_sign_right = 1.0f;
float g_standup_mit_kp = 0.3f;
float g_standup_mit_kd = 0.03f;
float g_standup_mit_target_left_hip = 0.0f;
float g_standup_mit_target_right_hip = 0.0f;
float g_standup_mit_ff_torque = 0.08f;

// ============================================================================
// Query functions
// ============================================================================

bool standup_is_active(void) {
    return g_standup_state != STANDUP_IDLE;
}

bool car_standup_is_active(void) {
    return g_car_standup_state != CAR_STANDUP_IDLE;
}

bool bal_to_car_is_active(void) {
    return g_bal_to_car_state != BAL_TO_CAR_IDLE;
}

// ============================================================================
// 起身状态机
// ============================================================================

void standup_state_machine_update(void) {
    if (g_standup_state == STANDUP_IDLE) return;

    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t elapsed = now_ms - g_standup_state_enter_ms;

    // 读取当前腿部状态 (FK)
    leg_state_t left_state, right_state;
    bool fk_valid = false;
    if (leg_ctrl_get_state_cached(true, &left_state) == ESP_OK &&
        leg_ctrl_get_state_cached(false, &right_state) == ESP_OK &&
        left_state.valid && right_state.valid) {
        fk_valid = true;
    }

    // 调试: 每 500ms 打印一次
    static uint32_t last_standup_debug_ms = 0;
    if (now_ms - last_standup_debug_ms >= 500) {
        float l_angle = fk_valid ? left_state.workspace.body_angle : 0;
        float r_angle = fk_valid ? right_state.workspace.body_angle : 0;
        ESP_LOGW(TAG, "STANDUP: s=%d t=%lums L_angle=%.1f R_angle=%.1f fk=%d",
                 (int)g_standup_state, (unsigned long)elapsed,
                 l_angle, r_angle, (int)fk_valid);
        last_standup_debug_ms = now_ms;
    }

    switch (g_standup_state) {
        case STANDUP_IDLE:
            break;

        case STANDUP_RETRACT: {
            bool reached = fk_valid &&
                (fabsf(left_state.workspace.leg_length - g_standup_retract_length) < 0.008f) &&
                (fabsf(right_state.workspace.leg_length - g_standup_retract_length) < 0.008f);
            bool timeout = (elapsed >= g_standup_retract_timeout_ms);
            if (reached || timeout) {
                g_standup_state = STANDUP_LEFT_ROLL;
                g_standup_state_enter_ms = now_ms;

                g_standup_mit_active = true;
                g_standup_mit_left_hip = true;
                g_standup_mit_right_hip = false;

                float current_hip = can_motor_read_position(g_motor_left_hip);
                leg_workspace_state_t target_ws = { .leg_length = g_standup_retract_length, .body_angle = CAR_MODE_BODY_ANGLE };
                leg_joint_state_t target_joint;
                if (leg_kin_inverse(&target_ws, true, NULL, &target_joint) == ESP_OK) {
                    g_standup_ff_sign_left = (target_joint.hip_angle >= current_hip) ? 1.0f : -1.0f;
                    g_standup_mit_target_left_hip = DEG2RAD(target_joint.hip_angle);
                }

                can_motor_set_mode(g_motor_left, MODE_SPEED);
                can_motor_set_speed(g_motor_left, g_standup_wheel_speed);

                ESP_LOGW(TAG, "STANDUP: LEFT_ROLL! hip=%.1f ff_sign=%.0f wheel=%.0f (%s)",
                         current_hip, g_standup_ff_sign_left, g_standup_wheel_speed,
                         reached ? "reached" : "timeout");
            }
            break;
        }

        case STANDUP_LEFT_ROLL: {
            bool reached = fk_valid &&
                (fabsf(left_state.workspace.body_angle - CAR_MODE_BODY_ANGLE) < g_standup_angle_threshold);
            bool timeout = (elapsed >= g_standup_roll_timeout_ms);
            if (reached || timeout) {
                can_motor_set_speed(g_motor_left, 0);
                g_standup_mit_left_hip = false;
                g_standup_mit_active = false;
                leg_ctrl_set_target(true, g_standup_retract_length, CAR_MODE_BODY_ANGLE);

                g_standup_state = STANDUP_LEFT_EXTEND;
                g_standup_state_enter_ms = now_ms;

                ESP_LOGW(TAG, "STANDUP: LEFT_EXTEND! target_len=%.3f (%s)",
                         g_standup_extend_length, reached ? "reached" : "timeout");
            }
            break;
        }

        case STANDUP_LEFT_EXTEND: {
            leg_ctrl_set_target(true, g_standup_extend_length, CAR_MODE_BODY_ANGLE);

            bool reached = fk_valid &&
                (fabsf(left_state.workspace.leg_length - g_standup_extend_length) < 0.008f);
            bool timeout = (elapsed >= g_standup_extend_timeout_ms);
            if (reached || timeout) {
                g_standup_state = STANDUP_WAIT;
                g_standup_state_enter_ms = now_ms;

                ESP_LOGW(TAG, "STANDUP: WAIT %lums before RIGHT_ROLL (%s)",
                         (unsigned long)g_standup_wait_ms, reached ? "reached" : "timeout");
            }
            break;
        }

        case STANDUP_WAIT: {
            if (elapsed >= g_standup_wait_ms) {
                g_standup_state = STANDUP_RIGHT_ROLL;
                g_standup_state_enter_ms = now_ms;

                g_standup_mit_active = true;
                g_standup_mit_right_hip = true;

                float current_hip = can_motor_read_position(g_motor_right_hip);
                leg_workspace_state_t target_ws = { .leg_length = g_standup_retract_length, .body_angle = CAR_MODE_BODY_ANGLE };
                leg_joint_state_t target_joint;
                if (leg_kin_inverse(&target_ws, false, NULL, &target_joint) == ESP_OK) {
                    g_standup_ff_sign_right = (target_joint.hip_angle >= current_hip) ? 1.0f : -1.0f;
                    g_standup_mit_target_right_hip = DEG2RAD(target_joint.hip_angle);
                }

                can_motor_set_mode(g_motor_right, MODE_SPEED);
                can_motor_set_speed(g_motor_right, g_standup_wheel_speed);

                ESP_LOGW(TAG, "STANDUP: RIGHT_ROLL! hip=%.1f ff_sign=%.0f wheel=%.0f",
                         current_hip, g_standup_ff_sign_right, g_standup_wheel_speed);
            }
            break;
        }

        case STANDUP_RIGHT_ROLL: {
            bool reached = fk_valid &&
                (fabsf(right_state.workspace.body_angle - CAR_MODE_BODY_ANGLE) < g_standup_angle_threshold);
            bool timeout = (elapsed >= g_standup_roll_timeout_ms);
            if (reached || timeout) {
                can_motor_set_speed(g_motor_right, 0);
                g_standup_mit_active = false;
                g_standup_mit_right_hip = false;
                leg_ctrl_set_target(false, g_standup_retract_length, CAR_MODE_BODY_ANGLE);

                g_standup_state = STANDUP_DONE;
                g_standup_state_enter_ms = now_ms;

                ESP_LOGW(TAG, "STANDUP: DONE! right leg rolled (%s)",
                         reached ? "reached" : "timeout");
            }
            break;
        }

        case STANDUP_DONE: {
            if (elapsed >= 100) {
                g_car_mode_prev_mode = g_control_mode;
                g_car_mode_prev_base_angle = g_leg_base_angle;
                g_car_mode_prev_base_length = g_leg_base_length;
                g_control_mode = CTRL_MODE_CAR;

                leg_ctrl_set_target(true, g_standup_retract_length, CAR_MODE_BODY_ANGLE);
                leg_ctrl_set_target(false, g_standup_retract_length, CAR_MODE_BODY_ANGLE);

                if (g_state == BALANCE_TEST_RUNNING) {
                    can_motor_set_mode(g_motor_left, MODE_SPEED);
                    can_motor_set_mode(g_motor_right, MODE_SPEED);
                }

                ESP_LOGW(TAG, "STANDUP: Car mode activated!");
                g_standup_state = STANDUP_IDLE;
            }
            break;
        }
    }
}

// ============================================================================
// 小车模式起身状态机
// ============================================================================

void car_standup_state_machine_update(void) {
    if (g_car_standup_state == CAR_STANDUP_IDLE) return;

    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t elapsed = now_ms - g_car_standup_enter_ms;

    switch (g_car_standup_state) {
        case CAR_STANDUP_SWING: {
            if (elapsed > 3000) {
                ESP_LOGW(TAG, "CAR_STANDUP: timeout, abort");
                g_car_standup_state = CAR_STANDUP_IDLE;
                break;
            }
            leg_ctrl_set_target(true,  g_leg_length_min, -90.0f);
            leg_ctrl_set_target(false, g_leg_length_min, -90.0f);

            leg_state_t left_st;
            if (leg_ctrl_get_state_cached(true, &left_st) == ESP_OK && left_st.valid) {
                if (fabsf(left_st.workspace.body_angle - (-90.0f)) < 20.0f) {
                    g_car_standup_state = CAR_STANDUP_DONE;
                    g_car_standup_enter_ms = now_ms;
                    ESP_LOGW(TAG, "CAR_STANDUP: angle reached (%.1f deg), enabling balance",
                             left_st.workspace.body_angle);
                }
            }
            break;
        }
        case CAR_STANDUP_DONE: {
            if (elapsed >= 200) {
                can_motor_set_speed(g_motor_left, 0);
                can_motor_set_speed(g_motor_right, 0);

                control_mode_t prev = g_car_mode_prev_mode;
                if (prev == CTRL_MODE_SINGLE_PID ||
                    (prev == CTRL_MODE_TRIPLE_PID &&
                     g_triple_pid_ctrl.params.wheel_mode == TRIPLE_PID_WHEEL_SPEED)) {
                    can_motor_set_mode(g_motor_left, MODE_SPEED);
                    can_motor_set_mode(g_motor_right, MODE_SPEED);
                } else {
                    can_motor_set_mode(g_motor_left, MODE_TORQUE);
                    can_motor_set_mode(g_motor_right, MODE_TORQUE);
                }

                g_control_mode = prev;

                leg_ctrl_set_target(true,  CAR_TO_BAL_LEG_LENGTH, -90.0f);
                leg_ctrl_set_target(false, CAR_TO_BAL_LEG_LENGTH, -90.0f);

                g_distance_zeropoint = g_lqr_distance;
                lqr_set_distance_zeropoint(&g_lqr_ctrl, g_distance_zeropoint);
                triple_pid_set_distance_zeropoint(&g_triple_pid_ctrl, g_distance_zeropoint);

                g_uncontrolable = 0;

                const char *mode_str = (prev == CTRL_MODE_LQR) ? "LQR" :
                                       (prev == CTRL_MODE_DUAL_PID) ? "DUAL_PID" :
                                       (prev == CTRL_MODE_TRIPLE_PID) ? "TRIPLE_PID" :
                                       (prev == CTRL_MODE_FULL_LQR) ? "FULL_LQR" : "SINGLE_PID";
                ESP_LOGW(TAG, "CAR_STANDUP: done! Switched to %s, balance active", mode_str);
                printf("CTRL_MODE:%s\n", mode_str);

                g_car_standup_state = CAR_STANDUP_IDLE;
            }
            break;
        }
        default:
            g_car_standup_state = CAR_STANDUP_IDLE;
            break;
    }
}

// ============================================================================
// 平衡→小车 过渡状态机
// ============================================================================

void bal_to_car_state_machine_update(void) {
    if (g_bal_to_car_state == BAL_TO_CAR_IDLE) return;

    uint32_t now_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    uint32_t elapsed = now_ms - g_bal_to_car_enter_ms;

    switch (g_bal_to_car_state) {
        case BAL_TO_CAR_RETRACT: {
            leg_ctrl_set_target(true,  BAL_TO_CAR_RETRACT_LENGTH, g_leg_base_angle);
            leg_ctrl_set_target(false, BAL_TO_CAR_RETRACT_LENGTH, g_leg_base_angle);

            leg_state_t left_st, right_st;
            bool left_ok  = (leg_ctrl_get_state_cached(true,  &left_st)  == ESP_OK && left_st.valid &&
                             fabsf(left_st.workspace.leg_length  - BAL_TO_CAR_RETRACT_LENGTH) < 0.008f);
            bool right_ok = (leg_ctrl_get_state_cached(false, &right_st) == ESP_OK && right_st.valid &&
                             fabsf(right_st.workspace.leg_length - BAL_TO_CAR_RETRACT_LENGTH) < 0.008f);
            bool reached  = left_ok && right_ok;
            bool timeout  = (elapsed >= BAL_TO_CAR_RETRACT_TIME_MS);

            if (reached || timeout) {
                g_bal_to_car_state = BAL_TO_CAR_TILT;
                g_bal_to_car_enter_ms = now_ms;

                g_control_mode = CTRL_MODE_CAR;

                if (g_state == BALANCE_TEST_RUNNING) {
                    can_motor_set_mode(g_motor_left,  MODE_SPEED);
                    can_motor_set_mode(g_motor_right, MODE_SPEED);
                    float tilt_rads = -BAL_TO_CAR_TILT_SPEED_RPM * 0.10472f;
                    xSemaphoreTake(g_wheel_cmd_mutex, portMAX_DELAY);
                    g_wheel_cmd.left_torque    = tilt_rads;
                    g_wheel_cmd.right_torque   = tilt_rads;
                    g_wheel_cmd.use_speed_mode = true;
                    xSemaphoreGive(g_wheel_cmd_mutex);
                }
                ESP_LOGW(TAG, "BAL→CAR: TILT phase (%.0frpm forward, %.0fms) [%s]",
                         BAL_TO_CAR_TILT_SPEED_RPM, (float)BAL_TO_CAR_TILT_TIME_MS,
                         reached ? "reached" : "timeout");
            }
            break;
        }
        case BAL_TO_CAR_TILT: {
            if (g_state == BALANCE_TEST_RUNNING) {
                float tilt_rads = -BAL_TO_CAR_TILT_SPEED_RPM * 0.10472f;
                xSemaphoreTake(g_wheel_cmd_mutex, portMAX_DELAY);
                g_wheel_cmd.left_torque    = tilt_rads;
                g_wheel_cmd.right_torque   = tilt_rads;
                g_wheel_cmd.use_speed_mode = true;
                xSemaphoreGive(g_wheel_cmd_mutex);
            }

            if (elapsed >= BAL_TO_CAR_TILT_TIME_MS) {
                if (g_state == BALANCE_TEST_RUNNING) {
                    xSemaphoreTake(g_wheel_cmd_mutex, portMAX_DELAY);
                    g_wheel_cmd.left_torque    = 0.0f;
                    g_wheel_cmd.right_torque   = 0.0f;
                    g_wheel_cmd.use_speed_mode = true;
                    xSemaphoreGive(g_wheel_cmd_mutex);
                }
                if (g_leg_control_enabled) {
                    leg_ctrl_set_target(true,  CAR_MODE_LEG_LENGTH, CAR_MODE_BODY_ANGLE);
                    leg_ctrl_set_target(false, CAR_MODE_LEG_LENGTH, CAR_MODE_BODY_ANGLE);
                }
                ESP_LOGW(TAG, "BAL→CAR: DONE! car mode active (body_angle=%.0f° leg=%.0fmm)",
                         CAR_MODE_BODY_ANGLE, CAR_MODE_LEG_LENGTH * 1000.0f);
                printf("CTRL_MODE:CAR\n");
                g_bal_to_car_state = BAL_TO_CAR_IDLE;
            }
            break;
        }
        default:
            g_bal_to_car_state = BAL_TO_CAR_IDLE;
            break;
    }
}
