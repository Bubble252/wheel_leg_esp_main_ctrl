#include "balance_observer.h"
#include "balance_types.h"
#include "leg_kinematics.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <math.h>

static const char *TAG = "BAL_OBSV";

#define WHEEL_RADIUS_M 0.05f
#define OBSERVER_PERIOD_MS 3

// ============================================================================
// 外部变量 (定义在 balance_test.c)
// ============================================================================

extern shared_imu_data_t g_imu_data;
extern SemaphoreHandle_t g_imu_mutex;

extern shared_wheel_state_t g_wheel_state;
extern SemaphoreHandle_t g_wheel_state_mutex;

extern bool g_leg_control_enabled;
extern bool g_vmc_input_valid;
extern vmc_dual_output_t g_vmc_dual_output;

extern volatile bool g_tasks_running;
extern TaskHandle_t g_task_observer;

// ============================================================================
// 观测器变量
// ============================================================================

bool g_observer_enabled = true;
bool g_tpid_use_observer_speed = false;
bool g_obsv_stream_enable = false;
int  g_obsv_period_ms = OBSERVER_PERIOD_MS;

// 2x2 卡尔曼滤波器状态: x = [v, a]^T
static float g_kf_x[2] = {0};
static float g_kf_P[4] = {1, 0, 0, 1};
float g_kf_Q_v = 1.0f;
float g_kf_Q_a = 1.0f;
float g_kf_R_v = 200.0f;
float g_kf_R_a = 200.0f;

float g_obsv_v_encoder = 0.0f;
float g_obsv_v_filter = 0.0f;
float g_obsv_x_filter = 0.0f;
float g_obsv_a_imu = 0.0f;
float g_obsv_wheel_v_raw = 0.0f;

static float g_accel_bias = 0.0f;
static bool  g_accel_bias_calibrated = false;
static float g_accel_lpf = 0.0f;
static float g_accel_lpf_tau = 0.009f;
static float g_accel_deadzone = 0.01f;

// ============================================================================
// 2x2 卡尔曼滤波器
// ============================================================================

static void kf_observer_update(float dt, float z_v, float z_a) {
    // Step 1: 状态预测 x' = F * x
    float v_pred = g_kf_x[0] + g_kf_x[1] * dt;
    float a_pred = g_kf_x[1];

    // Step 2: 协方差预测 P' = F * P * F^T + Q
    float p00 = g_kf_P[0], p01 = g_kf_P[1], p10 = g_kf_P[2], p11 = g_kf_P[3];

    float fp00 = p00 + dt * p10;
    float fp01 = p01 + dt * p11;
    float fp10 = p10;
    float fp11 = p11;

    float pp00 = fp00 + fp01 * dt + g_kf_Q_v;
    float pp01 = fp01;
    float pp10 = fp10 + fp11 * dt;
    float pp11 = fp11 + g_kf_Q_a;

    // Step 3: K = P' * (P' + R)^{-1}
    float s00 = pp00 + g_kf_R_v;
    float s01 = pp01;
    float s10 = pp10;
    float s11 = pp11 + g_kf_R_a;

    float det = s00 * s11 - s01 * s10;
    if (fabsf(det) < 1e-10f) det = 1e-10f;
    float inv_det = 1.0f / det;
    float si00 =  s11 * inv_det;
    float si01 = -s01 * inv_det;
    float si10 = -s10 * inv_det;
    float si11 =  s00 * inv_det;

    float k00 = pp00 * si00 + pp01 * si10;
    float k01 = pp00 * si01 + pp01 * si11;
    float k10 = pp10 * si00 + pp11 * si10;
    float k11 = pp10 * si01 + pp11 * si11;

    // Step 4: 状态更新
    float innov_v = z_v - v_pred;
    float innov_a = z_a - a_pred;

    g_kf_x[0] = v_pred + k00 * innov_v + k01 * innov_a;
    g_kf_x[1] = a_pred + k10 * innov_v + k11 * innov_a;

    // Step 5: 协方差更新
    float ikh00 = 1.0f - k00, ikh01 = -k01;
    float ikh10 = -k10,       ikh11 = 1.0f - k11;

    g_kf_P[0] = ikh00 * pp00 + ikh01 * pp10;
    g_kf_P[1] = ikh00 * pp01 + ikh01 * pp11;
    g_kf_P[2] = ikh10 * pp00 + ikh11 * pp10;
    g_kf_P[3] = ikh10 * pp01 + ikh11 * pp11;

    if (g_kf_P[0] < 0.001f) g_kf_P[0] = 0.001f;
    if (g_kf_P[3] < 0.001f) g_kf_P[3] = 0.001f;
}

void kf_observer_reset(void) {
    g_kf_x[0] = 0.0f;
    g_kf_x[1] = 0.0f;
    g_kf_P[0] = 1.0f; g_kf_P[1] = 0.0f;
    g_kf_P[2] = 0.0f; g_kf_P[3] = 1.0f;
    g_obsv_v_encoder = 0.0f;
    g_obsv_v_filter = 0.0f;
    g_obsv_x_filter = 0.0f;
    g_obsv_a_imu = 0.0f;
    g_obsv_wheel_v_raw = 0.0f;
}

// ============================================================================
// 速度观测器主函数
// ============================================================================

void velocity_observer_update(float dt, const shared_imu_data_t *imu,
                              float left_vel_rad, float right_vel_rad) {
    if (!g_observer_enabled) return;

    // 1) 原始轮速
    g_obsv_wheel_v_raw = (-0.5f) * (left_vel_rad + right_vel_rad) * WHEEL_RADIUS_M;

    // 2) 运动学补偿: 减去机体 pitch 旋转
    float pitch_rate_rad = imu->pitch_rate * 0.0174533f;
    float left_ground_rad  = left_vel_rad  - pitch_rate_rad;
    float right_ground_rad = right_vel_rad - pitch_rate_rad;

    // 3) 腿部动力学补偿 (仅在 VMC 使能时)
    float left_vbody = left_ground_rad * WHEEL_RADIUS_M;
    float right_vbody = right_ground_rad * WHEEL_RADIUS_M;

    if (g_leg_control_enabled && g_vmc_input_valid) {
        float l_d_alpha = g_vmc_dual_output.left.current_body_angle_rate * 0.0174533f;
        float l_L0 = g_vmc_dual_output.left.current_leg_length;
        float l_d_L0 = g_vmc_dual_output.left.current_leg_length_rate;
        float l_theta_deg = imu->pitch + g_vmc_dual_output.left.current_body_angle + 90.0f;
        float l_theta = l_theta_deg * 0.0174533f;
        float l_d_theta = pitch_rate_rad + l_d_alpha;

        left_ground_rad -= l_d_alpha;
        left_vbody = left_ground_rad * WHEEL_RADIUS_M
                   + l_L0 * l_d_theta * cosf(l_theta)
                   + l_d_L0 * sinf(l_theta);

        float r_d_alpha = g_vmc_dual_output.right.current_body_angle_rate * 0.0174533f;
        float r_L0 = g_vmc_dual_output.right.current_leg_length;
        float r_d_L0 = g_vmc_dual_output.right.current_leg_length_rate;
        float r_theta_deg = imu->pitch + g_vmc_dual_output.right.current_body_angle + 90.0f;
        float r_theta = r_theta_deg * 0.0174533f;
        float r_d_theta = pitch_rate_rad + r_d_alpha;

        right_ground_rad -= r_d_alpha;
        right_vbody = right_ground_rad * WHEEL_RADIUS_M
                    + r_L0 * r_d_theta * cosf(r_theta)
                    + r_d_L0 * sinf(r_theta);
    }

    // 4) 双轮取平均
    g_obsv_v_encoder = (-0.5f) * (left_vbody + right_vbody);

    // 5) IMU 加速度 (前进方向, 去除重力)
    float pitch_rad = imu->pitch * 0.0174533f;
    float a_raw = ((imu->accel_x + sinf(pitch_rad)) * cosf(pitch_rad)
                 - (imu->accel_z - cosf(pitch_rad)) * sinf(pitch_rad)) * 9.81f;

    if (g_accel_bias_calibrated) {
        a_raw -= g_accel_bias;
    }

    float alpha = dt / (g_accel_lpf_tau + dt);
    g_accel_lpf = g_accel_lpf * (1.0f - alpha) + a_raw * alpha;

    if (fabsf(g_accel_lpf) < g_accel_deadzone) {
        g_obsv_a_imu = 0.0f;
    } else {
        g_obsv_a_imu = g_accel_lpf;
    }

    // 6) 卡尔曼滤波融合
    kf_observer_update(dt, g_obsv_v_encoder, g_obsv_a_imu);

    g_obsv_v_filter = g_kf_x[0];

    // 7) 位移积分
    g_obsv_x_filter += g_obsv_v_filter * dt;
}

// ============================================================================
// 观测器任务
// ============================================================================

void task_observer(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();

    ESP_LOGI(TAG, "[task_observer] Started on Core %d, period %d ms (%.0f Hz)",
             xPortGetCoreID(), g_obsv_period_ms, 1000.0f / g_obsv_period_ms);

    // 启动时加速度计零偏校准 (采集 200 次取平均, ~0.6s)
    if (!g_accel_bias_calibrated) {
        const int CAL_SAMPLES = 200;
        float bias_sum = 0.0f;
        int valid_count = 0;
        ESP_LOGI(TAG, "[observer] Accel bias calibration: collecting %d samples...", CAL_SAMPLES);

        for (int i = 0; i < CAL_SAMPLES && g_tasks_running; i++) {
            shared_imu_data_t imu;
            xSemaphoreTake(g_imu_mutex, portMAX_DELAY);
            memcpy(&imu, &g_imu_data, sizeof(imu));
            xSemaphoreGive(g_imu_mutex);

            if (imu.valid) {
                float pitch_rad = imu.pitch * 0.0174533f;
                float a_raw = ((imu.accel_x + sinf(pitch_rad)) * cosf(pitch_rad)
                             - (imu.accel_z - cosf(pitch_rad)) * sinf(pitch_rad)) * 9.81f;
                bias_sum += a_raw;
                valid_count++;
            }
            vTaskDelay(pdMS_TO_TICKS(3));
        }

        if (valid_count > 0) {
            g_accel_bias = bias_sum / valid_count;
            g_accel_bias_calibrated = true;
            ESP_LOGI(TAG, "[observer] Accel bias calibrated: %.4f m/s² (%d samples)",
                     g_accel_bias, valid_count);
        }
    }

    while (g_tasks_running) {
        const TickType_t period = pdMS_TO_TICKS(g_obsv_period_ms);
        const float dt = g_obsv_period_ms / 1000.0f;

        if (g_observer_enabled) {
            shared_imu_data_t imu;
            xSemaphoreTake(g_imu_mutex, portMAX_DELAY);
            memcpy(&imu, &g_imu_data, sizeof(imu));
            xSemaphoreGive(g_imu_mutex);

            float left_vel_rad, right_vel_rad;
            xSemaphoreTake(g_wheel_state_mutex, portMAX_DELAY);
            left_vel_rad  = g_wheel_state.left_speed  * 0.10472f;
            right_vel_rad = g_wheel_state.right_speed * 0.10472f;
            xSemaphoreGive(g_wheel_state_mutex);

            velocity_observer_update(dt, &imu, left_vel_rad, right_vel_rad);
        }

        vTaskDelayUntil(&last_wake, period);
    }

    ESP_LOGI(TAG, "[task_observer] Stopped");
    g_task_observer = NULL;
    vTaskDelete(NULL);
}
