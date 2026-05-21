/**
 * @file balance_manager.c
 * @brief 任务管理、初始化、WiFi遥控
 *
 * 从 balance_test.c 拆出:
 * - balance_test_init/start/stop/enable/disable/emergency_stop/reset
 * - task_imu_read, task_balance_ctrl, task_motor_comm, task_unified_control
 * - task_remote_watchdog, update_remote_from_wifi
 */

#include "balance_test.h"
#include "balance_types.h"
#include "balance_control.h"
#include "balance_vmc.h"
#include "balance_observer.h"
#include "balance_standup.h"
#include "balance_roll.h"
#include "balance_jump.h"
#include "balance_plot.h"
#include "config.h"
#include "can_motor.h"
#include "can_motor_stw_regs.h"
#include "imu_driver.h"
#include "wit_reg.h"
#include "wifi_remote.h"
#include "lqr_balance.h"
#include "full_lqr.h"
#include "leg_kinematics.h"
#include "lowpass_filter.h"
#include "commander_parser.h"
#include "power_detect.h"
#include "pi_comm.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <math.h>

static const char *TAG = "BAL_MGR";

// forward declarations (定义在本文档下方)
void task_imu_read(void *arg);
void task_balance_ctrl(void *arg);
void task_motor_comm(void *arg);
void task_unified_control(void *arg);
void task_remote_watchdog(void *arg);
void update_remote_from_wifi(void);

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define DEG2RAD(d) ((d) * M_PI / 180.0f)
#define RAD2DEG(r) ((r) * 180.0f / M_PI)

// 任务周期
#undef IMU_READ_PERIOD_MS
#undef BALANCE_CTRL_PERIOD_MS
#undef MOTOR_COMM_PERIOD_MS
#define IMU_READ_PERIOD_MS          3
#define BALANCE_CTRL_PERIOD_MS      2
#define MOTOR_COMM_PERIOD_MS        2
#define LEG_MOTOR_DIVIDER           2
#define WATCHDOG_PERIOD_MS          100

// 任务栈和优先级 (与 balance_test.c 一致)
#define UNIFIED_TASK_STACK          12288
#define UNIFIED_TASK_PRIO           24
#define TASK_STACK_IMU              4096
#define TASK_STACK_BALANCE          8192
#define TASK_STACK_MOTOR            4096
#define TASK_STACK_WATCHDOG         4096
#define TASK_STACK_OBSERVER         4096
#define TASK_PRIO_IMU               22
#define TASK_PRIO_BALANCE           24
#define TASK_PRIO_MOTOR             20
#define TASK_PRIO_WATCHDOG          8
#define TASK_PRIO_OBSERVER          21
#define UNIFIED_TASK_PERIOD_MS      2
#define REMOTE_TIMEOUT_MS           1000

// ============================================================================
// extern 变量 (定义在 balance_test.c)
// ============================================================================

// 状态
extern balance_test_state_t g_state;
extern bool g_initialized;
extern bool g_auto_enable_inhibited;
extern bool g_use_unified_task;
extern bool g_uncontrolable_check_enabled;
extern volatile bool g_tasks_running;
extern balance_test_stats_t g_stats;
extern uint32_t g_imu_count_per_sec, g_ctrl_count_per_sec;
extern uint32_t g_motor_count_per_sec, g_leg_count_per_sec;
extern uint32_t g_last_stat_time;

// 任务句柄
extern TaskHandle_t g_task_imu, g_task_balance, g_task_motor;
extern TaskHandle_t g_task_watchdog, g_task_unified;

// 控制器
extern lqr_controller_t g_lqr_ctrl;
extern float g_lqr_speed;
extern float g_lqr_distance;
extern float g_distance_zeropoint;
extern float g_angle_zeropoint;

// PID 控制器
extern dual_pid_controller_t g_dual_pid_ctrl;
extern bool g_dual_pid_initialized;
extern triple_pid_controller_t g_triple_pid_ctrl;
extern bool g_triple_pid_initialized;
extern single_pid_controller_t g_single_pid_ctrl;
extern bool g_single_pid_initialized;

// Full LQR
extern full_lqr_controller_t g_full_lqr_ctrl;
extern bool g_full_lqr_initialized;

// 轮速
extern weighted_ma_filter_t g_wheel_speed_wma;
extern bool g_wma_enabled;
extern float g_left_wheel_speed_rad, g_right_wheel_speed_rad;
extern float g_left_wheel_accel, g_right_wheel_accel;
extern float g_prev_left_wheel_speed, g_prev_right_wheel_speed;

// 控制模式
extern control_mode_t g_control_mode;
extern control_mode_t g_car_mode_prev_mode;
extern float g_car_mode_prev_base_angle, g_car_mode_prev_base_length;

// Yaw
extern bool g_yaw_control_enabled;
extern bool g_yaw_force_enable;
extern float g_yaw_angle_last, g_yaw_angle_total, g_yaw_output;
extern bool g_yaw_first_run;

// Roll
extern bool g_roll_control_enabled;
extern float g_roll_output, g_roll_left_delta, g_roll_right_delta, g_roll_filtered;

// Leg
extern bool g_leg_control_enabled;
extern float g_leg_base_length, g_leg_base_angle;
extern float g_leg_left_target_length, g_leg_left_target_angle;
extern float g_leg_right_target_length, g_leg_right_target_angle;
extern float g_leg_left_hip_angle, g_leg_left_knee_angle;
extern float g_leg_right_hip_angle, g_leg_right_knee_angle;
extern float g_leg_move_speed;
extern float g_joint_normal_max_speed_rpm;
extern float g_joint_normal_pos_kp;

// Joint filter
extern float g_joint_lh_spd_filtered, g_joint_lk_spd_filtered;
extern float g_joint_rh_spd_filtered, g_joint_rk_spd_filtered;
extern bool g_joint_speed_filter_enable;
extern int g_joint_speed_filter_mode;
extern float g_joint_speed_slew_rate;
extern slewrate_filter_t g_sr_joint_lh, g_sr_joint_lk, g_sr_joint_rh, g_sr_joint_rk;
extern int g_joint_median_window;
extern median_filter_t g_mf_joint_lh, g_mf_joint_lk, g_mf_joint_rh, g_mf_joint_rk;

// 环路
extern uint8_t g_loop_enable_mask;

// X-Offset
extern bool g_xoffset_enabled;
extern pid_controller_t g_xoffset_pid;
extern float g_xoffset_value, g_xoffset_limit, g_xoffset_debug_speed;

// Leg Sync
extern bool g_leg_sync_enabled;
extern float g_leg_sync_gain, g_leg_sync_max_correction;
extern float g_leg_sync_debug_diff, g_leg_sync_debug_correction;

// 腿部补偿
extern bool g_pitch_leg_comp_enabled;

// 延迟
extern volatile uint64_t g_used_imu_time_us, g_ctrl_start_time_us;
extern volatile uint64_t g_ctrl_end_time_us, g_motor_send_time_us;
extern volatile uint64_t g_used_wifi_time_us;
extern float g_latency_imu_to_ctrl_us, g_latency_ctrl_calc_us;
extern float g_latency_ctrl_to_motor_us, g_latency_total_us;
extern float g_latency_total_avg_us, g_latency_total_max_us, g_latency_total_min_us;
extern uint32_t g_latency_sample_count;
extern float g_latency_wifi_to_ctrl_us, g_latency_wifi_total_us;
extern float g_latency_wifi_avg_us, g_latency_wifi_max_us, g_latency_wifi_min_us;
extern uint32_t g_latency_wifi_sample_count;

// 零点自适应
extern float g_zp_speed_threshold, g_zp_pitch_for_ctrl;
extern float g_zp_angle_error, g_zp_pid_raw, g_zp_pid_filtered;
extern bool g_zp_active;

// 遥控器
extern shared_remote_data_t g_remote_data;
extern SemaphoreHandle_t g_remote_mutex;
extern float g_joy_speed_scale, g_joy_yaw_scale;

// 轮电机命令
extern shared_wheel_cmd_t g_wheel_cmd;
extern SemaphoreHandle_t g_wheel_cmd_mutex;
extern shared_wheel_state_t g_wheel_state;
extern SemaphoreHandle_t g_wheel_state_mutex;

// IMU
extern shared_imu_data_t g_imu_data;
extern SemaphoreHandle_t g_imu_mutex;

// 电机
extern can_motor_handle_t g_motor_left, g_motor_right;
extern can_motor_handle_t g_motor_left_hip, g_motor_left_knee;
extern can_motor_handle_t g_motor_right_hip, g_motor_right_knee;

// VMC
extern bool g_vmc_enabled;
extern vmc_params_t g_vmc_params;
extern float g_vmc_target_vx, g_vmc_target_y;
extern vmc_dual_output_t g_vmc_dual_output;
extern bool g_vmc_input_valid;

// Observer
extern bool g_observer_enabled;

// Full LQR
extern float g_full_lqr_Tp_left, g_full_lqr_Tp_right;
extern bool g_full_lqr_stream_enable;
extern full_lqr_output_t g_full_lqr_output_left, g_full_lqr_output_right;

// Standup

// 失控
extern int g_uncontrolable;
extern int g_move_stop_flag;

// PID debug
extern bool g_pid_debug_enabled;
extern uint8_t g_pid_debug_divider, g_pid_debug_counter;

extern bool g_wheel_off_ground;
extern bool g_diff_speed_enabled;
extern float g_leg_length_min;
extern TaskHandle_t g_task_observer;

// 回调函数 (定义在 balance_test.c)
extern void commander_param_callback(char controller_id, char param_char, float value);
extern bool commander_query_callback(char controller_id, commander_pid_params_t *params);
extern void update_pi_comm_state(void);
extern void apply_leg_motor_commands(void);

// PI Comm
extern bool g_pi_comm_enabled;

// ============================================================================

esp_err_t balance_test_init(void) {
    if (g_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing balance test module...");
    
    // 创建互斥锁
    g_imu_mutex = xSemaphoreCreateMutex();
    g_remote_mutex = xSemaphoreCreateMutex();
    g_wheel_cmd_mutex = xSemaphoreCreateMutex();
    g_wheel_state_mutex = xSemaphoreCreateMutex();
    
    if (!g_imu_mutex || !g_remote_mutex || !g_wheel_cmd_mutex || !g_wheel_state_mutex) {
        ESP_LOGE(TAG, "Failed to create mutexes");
        return ESP_ERR_NO_MEM;
    }
    
    // 初始化电源检测和电机供电
    esp_err_t ret = power_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Power init failed, continuing anyway");
    }
    
    // 使能电机供电
    power_motor_enable(true);
    vTaskDelay(pdMS_TO_TICKS(100));  // 等待电机供电稳定
    
    // 初始化 CAN 总线
    ret = can_bus_init(CAN_TX_PIN, CAN_RX_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CAN bus init failed");
        return ret;
    }
    
    // 创建轮电机实例
    g_motor_left = can_motor_create(MOTOR_ID_LEFT_WHEEL);   // ID=3
    g_motor_right = can_motor_create(MOTOR_ID_RIGHT_WHEEL); // ID=6
    
    if (!g_motor_left || !g_motor_right) {
        ESP_LOGE(TAG, "Failed to create wheel motor instances");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Wheel motors created: Left=ID%d, Right=ID%d", 
             MOTOR_ID_LEFT_WHEEL, MOTOR_ID_RIGHT_WHEEL);
    
    // 创建腿部电机实例 (用于固定角度控制)
    g_motor_left_hip = can_motor_create(MOTOR_ID_LEFT_HIP);     // ID=1
    g_motor_left_knee = can_motor_create(MOTOR_ID_LEFT_KNEE);   // ID=2
    g_motor_right_hip = can_motor_create(MOTOR_ID_RIGHT_HIP);   // ID=4
    g_motor_right_knee = can_motor_create(MOTOR_ID_RIGHT_KNEE); // ID=5
    
    if (!g_motor_left_hip || !g_motor_left_knee || 
        !g_motor_right_hip || !g_motor_right_knee) {
        ESP_LOGW(TAG, "Failed to create some leg motor instances (continuing anyway)");
    } else {
        ESP_LOGI(TAG, "Leg motors created: L_Hip=ID%d, L_Knee=ID%d, R_Hip=ID%d, R_Knee=ID%d",
                 MOTOR_ID_LEFT_HIP, MOTOR_ID_LEFT_KNEE, 
                 MOTOR_ID_RIGHT_HIP, MOTOR_ID_RIGHT_KNEE);
        
        // 腿电机开机校零：将当前位置设为 0°，并验证回读
        // 注意：开机前需确保腿部在标定位置！
        ESP_LOGI(TAG, "Setting leg motors origin (current position -> 0)...");

        // 校零验证参数
        const float ORIGIN_VERIFY_THRESHOLD = 2.0f;  // 允许误差 ±2°
        const int   ORIGIN_MAX_RETRY        = 5;      // 最大重试次数
        bool origin_ok = true;

        // --- 对单个关节电机执行 "发校零→回读验证→重试" 的辅助宏 ---
        // 展开为内联逻辑，避免引入函数
        can_motor_handle_t _verify_motors[4] = {
            g_motor_left_hip, g_motor_left_knee,
            g_motor_right_hip, g_motor_right_knee,
        };
        const char *_verify_names[4] = { "L_Hip", "L_Knee", "R_Hip", "R_Knee" };

        for (int _mi = 0; _mi < 4; _mi++) {
            can_motor_handle_t _m = _verify_motors[_mi];
            const char *_name    = _verify_names[_mi];
            bool _motor_ok = false;

            for (int _retry = 0; _retry < ORIGIN_MAX_RETRY; _retry++) {
                // 发送校零指令
                can_motor_set_origin(_m);
                vTaskDelay(pdMS_TO_TICKS(30));  // 等待电机处理（比原来长，给 Flash 写入留时间）

                // 发 0xA3 读角度
                can_motor_request_angle(_m);
                vTaskDelay(pdMS_TO_TICKS(20));
                can_motor_process_rx();

                float _pos = can_motor_read_position(_m);
                ESP_LOGI(TAG, "  [%s] try %d: pos_readback=%.2f deg", _name, _retry + 1, _pos);

                if (fabsf(_pos) <= ORIGIN_VERIFY_THRESHOLD) {
                    ESP_LOGI(TAG, "  [%s] origin OK (%.2f deg)", _name, _pos);
                    _motor_ok = true;
                    break;
                }
                ESP_LOGW(TAG, "  [%s] origin MISMATCH (%.2f deg), retrying...", _name, _pos);
            }

            if (!_motor_ok) {
                ESP_LOGE(TAG, "  [%s] origin verify FAILED after %d retries! Inhibiting enable.",
                         _name, ORIGIN_MAX_RETRY);
                origin_ok = false;
            }
        }

        if (!origin_ok) {
            // 禁止自动使能，防止在错误零位下运动损坏机械结构
            g_auto_enable_inhibited = true;
            ESP_LOGE(TAG, "Leg motor origin verification FAILED. Auto-enable is INHIBITED.");
            ESP_LOGE(TAG, "Please place legs in calibration position and reboot.");
        } else {
            ESP_LOGI(TAG, "Leg motors origin set and verified OK.");
        }
        
        // 设置关节电机位置模式最大速度 (0xB2)
        const float joint_max_speed_rpm = 300.0f;
        can_motor_stw_set_max_speed(g_motor_left_hip, joint_max_speed_rpm);
        vTaskDelay(pdMS_TO_TICKS(10));
        can_motor_stw_set_max_speed(g_motor_left_knee, joint_max_speed_rpm);
        vTaskDelay(pdMS_TO_TICKS(10));
        can_motor_stw_set_max_speed(g_motor_right_hip, joint_max_speed_rpm);
        vTaskDelay(pdMS_TO_TICKS(10));
        can_motor_stw_set_max_speed(g_motor_right_knee, joint_max_speed_rpm);
        vTaskDelay(pdMS_TO_TICKS(50));
        g_joint_normal_max_speed_rpm = joint_max_speed_rpm;
        ESP_LOGI(TAG, "Leg motors max speed set to %.0f RPM", joint_max_speed_rpm);
        
        // 读取位置环 Kp 原始值 (用于跳跃后恢复)
        can_motor_stw_request_pid(g_motor_left_hip, STW_CMD_POS_KP);
        vTaskDelay(pdMS_TO_TICKS(20));
        can_motor_process_rx();
        float pos_kp_val = 0.0f;
        if (can_motor_stw_get_pid(g_motor_left_hip, STW_CMD_POS_KP, &pos_kp_val) == ESP_OK) {
            g_joint_normal_pos_kp = pos_kp_val;
            ESP_LOGI(TAG, "Joint position Kp read: %.2f", g_joint_normal_pos_kp);
        } else {
            g_joint_normal_pos_kp = 1.0f;  // 默认值
            ESP_LOGW(TAG, "Joint position Kp read failed, using default %.2f", g_joint_normal_pos_kp);
        }
    }
    
    // 初始化 IMU
    ret = imu_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU init failed");
        return ret;
    }
    
    // 设置 IMU 输出频率为 200Hz (匹配 ESP32 读取频率)
    ret = imu_set_output_rate(RRATE_200HZ);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "IMU set output rate failed, using default");
    } else {
        ESP_LOGI(TAG, "IMU output rate set to 200Hz");
    }
    
    // 初始化 LQR 控制器 (使用默认参数)
    ret = lqr_init(&g_lqr_ctrl, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LQR init failed");
        return ret;
    }
    
    // 设置角度零点 (可根据实际机器人调整)
    lqr_set_angle_zeropoint(&g_lqr_ctrl, g_angle_zeropoint);
    
    // 初始化双环 PID 控制器 (备用控制模式)
    ret = dual_pid_init(&g_dual_pid_ctrl, NULL);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Dual PID init failed, LQR mode only");
    } else {
        g_dual_pid_initialized = true;
        dual_pid_set_angle_zeropoint(&g_dual_pid_ctrl, g_angle_zeropoint);
        ESP_LOGI(TAG, "Dual PID controller initialized (backup mode)");
    }
    
    // 初始化单环 PID 控制器 (输出速度，适合电机速度模式)
    ret = single_pid_init(&g_single_pid_ctrl, NULL);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Single PID init failed");
    } else {
        g_single_pid_initialized = true;
        single_pid_set_angle_zeropoint(&g_single_pid_ctrl, g_angle_zeropoint);
        ESP_LOGI(TAG, "Single PID controller initialized (speed output mode)");
    }
    
    // 初始化三环 PID 控制器 (速度环→角度环→轮速环)
    ret = triple_pid_init(&g_triple_pid_ctrl, NULL);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Triple PID init failed");
    } else {
        g_triple_pid_initialized = true;
        triple_pid_set_angle_zeropoint(&g_triple_pid_ctrl, g_angle_zeropoint);
        ESP_LOGI(TAG, "Triple PID controller initialized");
    }
    
    // 初始化完整 LQR 控制器 (同时输出 T 和 Tp, K 随腿长插值)
    ret = full_lqr_init(&g_full_lqr_ctrl, NULL);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Full LQR init failed");
    } else {
        g_full_lqr_initialized = true;
        ESP_LOGI(TAG, "Full LQR controller initialized");
    }
    
    // 初始化轮速加权滑动平均滤波器
    wma_init(&g_wheel_speed_wma);
    ESP_LOGI(TAG, "Wheel speed WMA filter initialized (12-point, %s)",
             g_wma_enabled ? "ENABLED" : "DISABLED");
    
    // 初始化 Commander 解析器
    // - set_callback: 收到设置命令时更新 LQR 参数
    // - query_callback: 收到查询命令时返回 LQR 实际参数
    commander_parser_init(commander_param_callback, commander_query_callback);
    ESP_LOGI(TAG, "Commander parser initialized with LQR callbacks");
    
    // 初始化 X-Offset PID (腿部速度自适应偏移)
    {
        pid_params_t xoffset_params = {
            .kp = 0.1f,            // 默认比例增益 (m/s → m)
            .ki = 0.0f,            // 默认无积分
            .kd = 0.0f,            // 默认无微分
            .output_min = -g_xoffset_limit,
            .output_max = g_xoffset_limit,
            .integral_max = g_xoffset_limit * 0.5f,
            .d_filter_coef = 0.1f,
        };
        pid_init(&g_xoffset_pid, &xoffset_params);
        ESP_LOGI(TAG, "X-Offset PID initialized (Kp=%.4f, limit=%.3fm)", 
                 xoffset_params.kp, g_xoffset_limit);
    }
    
    // 初始化 WiFi 遥控 (但不启动)
    ret = wifi_remote_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi remote init failed, continuing anyway");
    }
    
    // 初始化腿部控制 (计算初始电机角度)
    leg_ctrl_init();
    
    // 初始化关节电机速度滤波器 (两种模式)
    slewrate_init(&g_sr_joint_lh, g_joint_speed_slew_rate);
    slewrate_init(&g_sr_joint_lk, g_joint_speed_slew_rate);
    slewrate_init(&g_sr_joint_rh, g_joint_speed_slew_rate);
    slewrate_init(&g_sr_joint_rk, g_joint_speed_slew_rate);
    median_init(&g_mf_joint_lh, g_joint_median_window);
    median_init(&g_mf_joint_lk, g_joint_median_window);
    median_init(&g_mf_joint_rh, g_joint_median_window);
    median_init(&g_mf_joint_rk, g_joint_median_window);
    ESP_LOGI(TAG, "Joint speed filter: %s, mode=%s, slew_rate=%.0f, median_win=%d",
             g_joint_speed_filter_enable ? "ON" : "OFF",
             g_joint_speed_filter_mode == 0 ? "Median" : "SlewRate",
             g_joint_speed_slew_rate, g_joint_median_window);

    // 初始化 VMC 参数
    vmc_get_default_params(&g_vmc_params);
    ESP_LOGI(TAG, "VMC params: K_vx=%.1f, K_y=%.1f, D_y=%.1f, gc=%.2f, mass=%.1fkg",
             g_vmc_params.K_vx, g_vmc_params.K_y, g_vmc_params.D_y,
             g_vmc_params.gravity_comp, g_vmc_params.robot_mass);
    
    // 初始化树莓派串口通信 (Core 0, 仅通信，不影响控制)
    if (g_pi_comm_enabled) {
        ret = pi_comm_init();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Pi comm init failed: %s (continuing anyway)", esp_err_to_name(ret));
        } else {
            ESP_LOGI(TAG, "Pi comm initialized (UART1: TX=GPIO4, RX=GPIO5, 115200bps)");
        }
    } else {
        ESP_LOGI(TAG, "Pi comm disabled (use 'balance picomm 1' to enable)");
    }
    
    g_state = BALANCE_TEST_READY;
    g_initialized = true;
    
    ESP_LOGI(TAG, "Balance test module initialized");
    ESP_LOGI(TAG, "  Angle zeropoint: %.2f deg", g_angle_zeropoint);
    ESP_LOGI(TAG, "  IMU period: %d ms", IMU_READ_PERIOD_MS);
    ESP_LOGI(TAG, "  Balance period: %d ms", BALANCE_CTRL_PERIOD_MS);
    ESP_LOGI(TAG, "  Motor period: %d ms", MOTOR_COMM_PERIOD_MS);
    
    return ESP_OK;
}

// ============================================================================
// 任务启动/停止
// ============================================================================

esp_err_t balance_test_start(void) {
    if (!g_initialized) {
        ESP_LOGE(TAG, "Not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    if (g_tasks_running) {
        ESP_LOGW(TAG, "Tasks already running");
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Starting balance test tasks...");
    
    // 启动 WiFi AP
    esp_err_t ret = wifi_remote_start();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi remote start failed");
    }
    
    // 等待 WiFi PA 电流稳定，避免与电机通信同时启动导致欠压
    ESP_LOGI(TAG, "Waiting 2s for WiFi power stabilization...");
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // 重置 LQR 控制器
    lqr_reset(&g_lqr_ctrl);
    g_lqr_distance = 0.0f;
    g_lqr_speed = 0.0f;
    g_distance_zeropoint = 0.0f;
    g_yaw_angle_last = 0.0f;
    g_yaw_angle_total = 0.0f;
    g_yaw_output = 0.0f;
    g_yaw_first_run = true;
    g_move_stop_flag = 0;
    g_uncontrolable = 0;
    
    // 重置统计
    memset(&g_stats, 0, sizeof(g_stats));
    g_imu_count_per_sec = 0;
    g_ctrl_count_per_sec = 0;
    g_last_stat_time = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // 根据架构选择创建任务
    if (g_use_unified_task) {
        // ===== 合并任务架构 =====
        // IMU读取 + 控制算法 + 电机通信 在同一个任务中执行
        ESP_LOGI(TAG, "Using UNIFIED task architecture");
        xTaskCreatePinnedToCore(task_unified_control, "unified_ctrl", UNIFIED_TASK_STACK,
                                NULL, UNIFIED_TASK_PRIO, &g_task_unified, 1);
    } else {
        // ===== 分离任务架构 (默认) =====
        ESP_LOGI(TAG, "Using SEPARATE task architecture");
        // 创建任务 (Core 1 - 实时控制)
        xTaskCreatePinnedToCore(task_imu_read, "imu_read", TASK_STACK_IMU,
                                NULL, TASK_PRIO_IMU, &g_task_imu, 1);
        xTaskCreatePinnedToCore(task_balance_ctrl, "balance_ctrl", TASK_STACK_BALANCE,
                                NULL, TASK_PRIO_BALANCE, &g_task_balance, 1);
        xTaskCreatePinnedToCore(task_motor_comm, "motor_comm", TASK_STACK_MOTOR,
                                NULL, TASK_PRIO_MOTOR, &g_task_motor, 1);
    }
    
    // 创建任务 (Core 0 - 非实时) - 两种架构都需要
    xTaskCreatePinnedToCore(task_remote_watchdog, "watchdog", TASK_STACK_WATCHDOG,
                            NULL, TASK_PRIO_WATCHDOG, &g_task_watchdog, 0);
    
    // 创建速度观测器独立任务 (Core 1 - 实时, 独立于控制循环)
    xTaskCreatePinnedToCore(task_observer, "observer", TASK_STACK_OBSERVER,
                            NULL, TASK_PRIO_OBSERVER, &g_task_observer, 1);
    
    g_tasks_running = true;
    g_state = BALANCE_TEST_READY;
    
    ESP_LOGI(TAG, "Balance test tasks started (%s mode)", 
             g_use_unified_task ? "unified" : "separate");
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "WiFi AP: WL-PRO (password: 12345678)");
    ESP_LOGI(TAG, "Open http://192.168.4.1 in browser");
    ESP_LOGI(TAG, "Auto-enable after IMU stable & pitch < 30 deg");
    ESP_LOGI(TAG, "=================================");
    
    return ESP_OK;
}

void balance_test_stop(void) {
    if (!g_tasks_running) {
        return;
    }
    
    ESP_LOGI(TAG, "Stopping balance test tasks...");
    
    // 先禁用输出
    xSemaphoreTake(g_wheel_cmd_mutex, portMAX_DELAY);
    g_wheel_cmd.enabled = false;
    g_wheel_cmd.left_torque = 0;
    g_wheel_cmd.right_torque = 0;
    xSemaphoreGive(g_wheel_cmd_mutex);
    
    // 停止电机
    can_motor_set_torque(g_motor_left, 0);
    can_motor_set_torque(g_motor_right, 0);
    can_motor_set_idle(g_motor_left);
    can_motor_set_idle(g_motor_right);
    
    // 设置退出标志，等待任务自行退出
    g_tasks_running = false;
    
    // 等待任务自行退出 (最长等待 500ms)
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // 任务会自行删除并清空句柄，这里只是确保清空
    g_task_imu = NULL;
    g_task_balance = NULL;
    g_task_motor = NULL;
    g_task_watchdog = NULL;
    g_task_observer = NULL;
    
    // 停止 WiFi
    wifi_remote_stop();
    
    g_state = BALANCE_TEST_IDLE;
    
    ESP_LOGI(TAG, "Balance test tasks stopped");
}

// ============================================================================
// 使能控制
// ============================================================================

void balance_test_enable(void) {
    if (g_state == BALANCE_TEST_EMERGENCY) {
        ESP_LOGW(TAG, "Cannot enable in emergency state, call reset first");
        return;
    }
    
    ESP_LOGI(TAG, "Balance control ENABLED");
    
    // 清除自动使能抑制 (手动 enable 后允许后续自动使能)
    g_auto_enable_inhibited = false;
    
    // 重置控制器
    lqr_reset(&g_lqr_ctrl);
    g_distance_zeropoint = g_lqr_distance;
    lqr_set_distance_zeropoint(&g_lqr_ctrl, g_distance_zeropoint);  // 同步到 LQR 控制器
    triple_pid_set_distance_zeropoint(&g_triple_pid_ctrl, g_distance_zeropoint);  // 同步到四环PID
    
    // 重置 YAW 累积角度，并初始化为当前方向 (避免启动跳变)
    g_yaw_angle_total = 0.0f;
    g_yaw_first_run = true;  // 让下一帧重新初始化 yaw_angle_last
    g_uncontrolable = 0;
    
    // 电机进入闭环 (根据控制模式选择电机模式)
    if (g_control_mode == CTRL_MODE_SINGLE_PID || g_control_mode == CTRL_MODE_CAR ||
        (g_control_mode == CTRL_MODE_TRIPLE_PID && g_triple_pid_ctrl.params.wheel_mode == TRIPLE_PID_WHEEL_SPEED)) {
        // 单环 PID / 小车 / 三环SPEED模式 使用速度模式
        can_motor_set_mode(g_motor_left, MODE_SPEED);
        can_motor_set_mode(g_motor_right, MODE_SPEED);
        ESP_LOGI(TAG, "Motor mode: SPEED");
    } else {
        // LQR / 双环 PID / 三环TORQUE模式 使用扭矩模式
        can_motor_set_mode(g_motor_left, MODE_TORQUE);
        can_motor_set_mode(g_motor_right, MODE_TORQUE);
        ESP_LOGI(TAG, "Motor mode: TORQUE");
    }
    can_motor_enter_closed_loop(g_motor_left);
    can_motor_enter_closed_loop(g_motor_right);
    
    xSemaphoreTake(g_wheel_cmd_mutex, portMAX_DELAY);
    g_wheel_cmd.enabled = true;
    xSemaphoreGive(g_wheel_cmd_mutex);
    
    g_state = BALANCE_TEST_RUNNING;
}

void balance_test_disable(void) {
    ESP_LOGI(TAG, "Balance control DISABLED");
    
    // 禁止自动重新使能 (手动 disable 后需要手动 enable)
    g_auto_enable_inhibited = true;
    
    xSemaphoreTake(g_wheel_cmd_mutex, portMAX_DELAY);
    g_wheel_cmd.enabled = false;
    g_wheel_cmd.left_torque = 0;
    g_wheel_cmd.right_torque = 0;
    xSemaphoreGive(g_wheel_cmd_mutex);
    
    // 电机输出清零
    can_motor_set_torque(g_motor_left, 0);
    can_motor_set_torque(g_motor_right, 0);
    
    if (g_state != BALANCE_TEST_EMERGENCY) {
        g_state = BALANCE_TEST_READY;
    }
}

void balance_test_emergency_stop(void) {
    ESP_LOGW(TAG, "EMERGENCY STOP!");
    
    g_state = BALANCE_TEST_EMERGENCY;
    g_uncontrolable = 1;
    
    xSemaphoreTake(g_wheel_cmd_mutex, portMAX_DELAY);
    g_wheel_cmd.enabled = false;
    g_wheel_cmd.left_torque = 0;
    g_wheel_cmd.right_torque = 0;
    xSemaphoreGive(g_wheel_cmd_mutex);
    
    // 立即停止电机
    can_motor_set_torque(g_motor_left, 0);
    can_motor_set_torque(g_motor_right, 0);
    can_motor_set_idle(g_motor_left);
    can_motor_set_idle(g_motor_right);
}

void balance_test_reset_emergency(void) {
    if (g_state != BALANCE_TEST_EMERGENCY) {
        return;
    }
    
    ESP_LOGI(TAG, "Emergency reset");
    g_uncontrolable = 0;
    g_state = BALANCE_TEST_READY;
}

// ============================================================================
// 状态获取
// ============================================================================

balance_test_state_t balance_test_get_state(void) {
    return g_state;
}

void balance_test_get_stats(balance_test_stats_t *stats) {
    if (stats) {
        memcpy(stats, &g_stats, sizeof(balance_test_stats_t));
    }
}

void balance_test_set_angle_zeropoint(float zeropoint) {
    g_angle_zeropoint = zeropoint;
    lqr_set_angle_zeropoint(&g_lqr_ctrl, zeropoint);
    if (g_dual_pid_initialized) {
        dual_pid_set_angle_zeropoint(&g_dual_pid_ctrl, zeropoint);
    }
    if (g_single_pid_initialized) {
        single_pid_set_angle_zeropoint(&g_single_pid_ctrl, zeropoint);
    }
    if (g_triple_pid_initialized) {
        triple_pid_set_angle_zeropoint(&g_triple_pid_ctrl, zeropoint);
    }
    ESP_LOGI(TAG, "Angle zeropoint set to %.2f (synced to all controllers)", zeropoint);
}

float balance_test_get_angle_zeropoint(void) {
    return g_angle_zeropoint;
}

void balance_test_print_status(void) {
    const char *state_names[] = {"IDLE", "READY", "RUNNING", "EMERGENCY", "ERROR"};
    const char *mode_names[] = {"LQR", "DUAL_PID", "SINGLE_PID", "CAR", "TRIPLE_PID"};
    
    ESP_LOGI(TAG, "=== Balance Test Status ===");
    ESP_LOGI(TAG, "State: %s", state_names[g_state]);
    ESP_LOGI(TAG, "Task mode: %s", g_use_unified_task ? "UNIFIED" : "SEPARATE");
    ESP_LOGI(TAG, "Control mode: %s (%s)", 
             mode_names[g_control_mode],
             g_control_mode == CTRL_MODE_SINGLE_PID ? "speed output" : 
             g_control_mode == CTRL_MODE_CAR ? "car speed mode" :
             (g_control_mode == CTRL_MODE_TRIPLE_PID && g_triple_pid_ctrl.params.wheel_mode == TRIPLE_PID_WHEEL_SPEED) ? "triple pid speed" : "torque output");
    ESP_LOGI(TAG, "Safety check: %s (uncontrolable=%d)", 
             g_uncontrolable_check_enabled ? "ENABLED" : "DISABLED", g_uncontrolable);
    ESP_LOGI(TAG, "Wheel off-ground: %s (spd_th=%.1f acc_th=%.1f)",
             g_wheel_off_ground ? "OFF GROUND" : "on ground",
             g_lqr_ctrl.params.wheel_off_ground_speed_threshold,
             g_lqr_ctrl.params.wheel_off_ground_accel_threshold);
    ESP_LOGI(TAG, "Angle zeropoint: %.2f deg", g_angle_zeropoint);
    ESP_LOGI(TAG, "Roll control: %s", g_roll_control_enabled ? "ENABLED" : "DISABLED");
    ESP_LOGI(TAG, "Pitch leg comp: %s", g_pitch_leg_comp_enabled ? "ENABLED" : "DISABLED");
    ESP_LOGI(TAG, "X-Offset: %s (val=%.4fm, speed=%.3fm/s, Kp=%.4f Ki=%.4f Kd=%.4f lim=%.3f)",
             g_xoffset_enabled ? "ENABLED" : "DISABLED",
             g_xoffset_value, g_xoffset_debug_speed,
             g_xoffset_pid.kp, g_xoffset_pid.ki, g_xoffset_pid.kd, g_xoffset_limit);
    ESP_LOGI(TAG, "Leg Sync: %s (gain=%.2f max=%.1f° diff=%.2f° corr=%.2f°)",
             g_leg_sync_enabled ? "ENABLED" : "DISABLED",
             g_leg_sync_gain, g_leg_sync_max_correction,
             g_leg_sync_debug_diff, g_leg_sync_debug_correction);
    ESP_LOGI(TAG, "Leg control: %s", g_leg_control_enabled ? "ENABLED" : "DISABLED");
    if (g_leg_control_enabled) {
        ESP_LOGI(TAG, "  Leg targets: L=%.3fm/%.1fdeg, R=%.3fm/%.1fdeg",
                 g_leg_left_target_length, g_leg_left_target_angle,
                 g_leg_right_target_length, g_leg_right_target_angle);
        ESP_LOGI(TAG, "  Leg angles: L_Hip=%.1f, L_Knee=%.1f, R_Hip=%.1f, R_Knee=%.1f",
                 g_leg_left_hip_angle, g_leg_left_knee_angle,
                 g_leg_right_hip_angle, g_leg_right_knee_angle);
        if (g_roll_control_enabled) {
            ESP_LOGI(TAG, "  Roll ctrl: out=%.3f, L_delta=%.4f, R_delta=%.4f, roll_filt=%.2f",
                     g_roll_output, g_roll_left_delta, g_roll_right_delta, g_roll_filtered);
        }
    }
    ESP_LOGI(TAG, "Control freq: %.1f Hz", g_stats.control_freq_hz);
    ESP_LOGI(TAG, "IMU freq: %.1f Hz", g_stats.imu_freq_hz);
    ESP_LOGI(TAG, "Motor comm freq: %.1f Hz", g_stats.motor_freq_hz);
    ESP_LOGI(TAG, "Leg motor freq: %.1f Hz", g_stats.leg_freq_hz);
    
    // IMU 数据
    xSemaphoreTake(g_imu_mutex, portMAX_DELAY);
    ESP_LOGI(TAG, "IMU: pitch=%.2f, roll=%.2f, yaw=%.2f", 
             g_imu_data.pitch, g_imu_data.roll, g_imu_data.yaw);
    xSemaphoreGive(g_imu_mutex);
    
    // 遥控数据
    xSemaphoreTake(g_remote_mutex, portMAX_DELAY);
    ESP_LOGI(TAG, "Remote: joy=(%d,%d) go=%d", 
             g_remote_data.joy_x, g_remote_data.joy_y, g_remote_data.go);
    xSemaphoreGive(g_remote_mutex);
    
    // 电机状态
    xSemaphoreTake(g_wheel_state_mutex, portMAX_DELAY);
    ESP_LOGI(TAG, "Left wheel: pos=%.1f, spd=%.1f, online=%d",
             g_wheel_state.left_position, g_wheel_state.left_speed, 
             g_wheel_state.left_online);
    ESP_LOGI(TAG, "Right wheel: pos=%.1f, spd=%.1f, online=%d",
             g_wheel_state.right_position, g_wheel_state.right_speed, 
             g_wheel_state.right_online);
    xSemaphoreGive(g_wheel_state_mutex);
    
    ESP_LOGI(TAG, "LQR distance: %.2f", g_lqr_distance);
    ESP_LOGI(TAG, "Distance zeropoint: %.2f", g_distance_zeropoint);
    
    // CAN 总线错误统计
    uint32_t busoff_cnt, tx_err_cnt, recovery_cnt;
    can_bus_get_error_stats(&busoff_cnt, &tx_err_cnt, &recovery_cnt);
    if (busoff_cnt > 0 || tx_err_cnt > 0) {
        ESP_LOGW(TAG, "CAN errors: Bus-Off=%lu, TX_fail=%lu, Recovered=%lu",
                 busoff_cnt, tx_err_cnt, recovery_cnt);
    } else {
        ESP_LOGI(TAG, "CAN bus: OK (no errors)");
    }
    
    ESP_LOGI(TAG, "===========================");
}

// ============================================================================
// 任务实现
// ============================================================================

/**
 * @brief IMU 读取任务 (Core 1, 2ms 周期)
 */


void task_imu_read(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(IMU_READ_PERIOD_MS);
    imu_data_t imu;
    
    ESP_LOGI(TAG, "[task_imu_read] Started on Core %d", xPortGetCoreID());
    
    while (g_tasks_running) {
        // 读取 IMU 数据
        if (imu_read_data(&imu) == ESP_OK && imu.is_valid) {
            // 在获取锁之前记录精确时间，减少互斥锁带来的时间偏差
            uint64_t read_time = esp_timer_get_time();
            
            xSemaphoreTake(g_imu_mutex, portMAX_DELAY);
            g_imu_data.pitch = imu.pitch;
            g_imu_data.pitch_rate = imu.gyro_y;
            g_imu_data.roll = imu.roll;
            g_imu_data.roll_rate = imu.gyro_x;
            g_imu_data.yaw = imu.yaw;
            g_imu_data.yaw_rate = imu.gyro_z;
            g_imu_data.accel_x = imu.accel_x;
            g_imu_data.accel_y = imu.accel_y;
            g_imu_data.accel_z = imu.accel_z;
            g_imu_data.timestamp = imu.timestamp;
            g_imu_data.read_time_us = read_time;  // 记录精确读取时间到数据结构中
            g_imu_data.valid = true;
            xSemaphoreGive(g_imu_mutex);
            
            g_stats.imu_read_count++;
            g_imu_count_per_sec++;
        }
        
        vTaskDelayUntil(&last_wake, period);
    }
    
    ESP_LOGI(TAG, "[task_imu_read] Stopped");
    g_task_imu = NULL;
    vTaskDelete(NULL);
}

/**
 * @brief 平衡控制任务 (Core 1, 5ms 周期)
 * @note 这是核心控制循环，参考 shibo_wheel_leg 的 lqr_balance_loop
 */
void task_balance_ctrl(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(BALANCE_CTRL_PERIOD_MS);
    const float dt = BALANCE_CTRL_PERIOD_MS / 1000.0f;
    
    ESP_LOGI(TAG, "[task_balance_ctrl] Started on Core %d", xPortGetCoreID());
    
    while (g_tasks_running) {
        // 更新遥控数据 (在计时之前)
        update_remote_from_wifi();
        
        // 记录控制开始时间 (在读取IMU数据之前)
        uint64_t ctrl_start = esp_timer_get_time();
        
        // 计算平衡控制输出 (内部会保存实际使用的 IMU 时间戳到 g_used_imu_time_us)
        compute_balance_output(dt);
        
        // 跳跃状态机更新 (在平衡输出之后，可覆盖轮命令)
        jump_state_machine_update();
        
        // 起身状态机更新
        standup_state_machine_update();
        
        // 小车模式起身状态机更新
        car_standup_state_machine_update();
        
        // 记录控制结束时间
        uint64_t ctrl_end = esp_timer_get_time();
        
        // 计算延迟 (更精确的方式)
        // IMU→控制: 从IMU数据读取到开始使用的等待时间
        if (g_used_imu_time_us > 0) {
            g_latency_imu_to_ctrl_us = (float)(ctrl_start - g_used_imu_time_us);
        }
        // 控制计算: 纯粹的计算耗时
        g_latency_ctrl_calc_us = (float)(ctrl_end - ctrl_start);
        
        // 保存时间点供电机任务使用
        g_ctrl_start_time_us = ctrl_start;
        g_ctrl_end_time_us = ctrl_end;
        
        // 更新统计
        g_stats.control_loop_count++;
        g_ctrl_count_per_sec++;
        
        // 每秒更新一次频率统计
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - g_last_stat_time >= 1000) {
            g_stats.imu_freq_hz = (float)g_imu_count_per_sec;
            g_stats.control_freq_hz = (float)g_ctrl_count_per_sec;
            g_stats.motor_freq_hz = (float)g_motor_count_per_sec;
            g_stats.leg_freq_hz = (float)g_leg_count_per_sec;
            g_imu_count_per_sec = 0;
            g_ctrl_count_per_sec = 0;
            g_motor_count_per_sec = 0;
            g_leg_count_per_sec = 0;
            g_last_stat_time = now;
        }
        
        // 更新 pi_comm 状态 (仅在使能时)
        if (g_pi_comm_enabled) {
            update_pi_comm_state();
        }
        
        vTaskDelayUntil(&last_wake, period);
    }
    
    ESP_LOGI(TAG, "[task_balance_ctrl] Stopped");
    g_task_balance = NULL;
    vTaskDelete(NULL);
}

/**
 * @brief 电机通信任务 (Core 1, 2ms 周期)
 */
void task_motor_comm(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(MOTOR_COMM_PERIOD_MS);
    static uint8_t leg_divider_count = 0;  // 腿电机分频计数器
    
    ESP_LOGI(TAG, "[task_motor_comm] Started on Core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "  Wheel motor: %dHz, Leg motor: %dHz", 
             1000 / MOTOR_COMM_PERIOD_MS, 
             1000 / MOTOR_COMM_PERIOD_MS / LEG_MOTOR_DIVIDER);
    
    while (g_tasks_running) {
        // ======== 先发送上一轮计算好的电机命令 ========
        // 尽早发送，让电机有更多时间回复
        apply_motor_commands();
        g_motor_count_per_sec++;
        
        // 腿电机 (分频执行)
        leg_divider_count++;
        if (leg_divider_count >= LEG_MOTOR_DIVIDER) {
            leg_divider_count = 0;
            apply_leg_motor_commands();
            g_leg_count_per_sec++;
        }
        
        // ======== 处理 CAN 接收 ========
        // 发送命令后电机回复 + 上一轮的回复都在此处理
        can_motor_process_rx();
        
        // ======== 读取轮电机状态 (用最新缓存) ========
        xSemaphoreTake(g_wheel_state_mutex, portMAX_DELAY);
        g_wheel_state.left_position = can_motor_read_position(g_motor_left);
        g_wheel_state.left_speed = can_motor_read_speed(g_motor_left);
        g_wheel_state.left_online = can_motor_is_online(g_motor_left, 100);
        g_wheel_state.right_position = can_motor_read_position(g_motor_right);
        g_wheel_state.right_speed = can_motor_read_speed(g_motor_right);
        g_wheel_state.right_online = can_motor_is_online(g_motor_right, 100);
        g_wheel_state.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
        xSemaphoreGive(g_wheel_state_mutex);
        
        g_stats.motor_cmd_count++;
        
        vTaskDelayUntil(&last_wake, period);
    }
    
    ESP_LOGI(TAG, "[task_motor_comm] Stopped");
    g_task_motor = NULL;
    vTaskDelete(NULL);
}

/**
 * @brief 合并任务: IMU读取 + 控制算法 + 电机通信 (Core 1, 2ms 周期)
 * 
 * 将三个任务合并到一个任务中执行，减少任务切换开销和同步延迟。
 * 执行顺序: 1.发送电机命令 → 2.读IMU → 3.处理CAN接收 → 4.计算控制
 * 先发命令让电机有时间回复，IMU读取期间(~0.15-0.3ms@400kHz)电机回复到达，
 * 然后 process_rx 获取最新数据，控制计算用的是最新电机状态。
 * 
 * @note 500Hz 时序预算 (2ms):
 *   - CAN TX 轮电机 2帧: ~0.26ms
 *   - CAN TX 腿电机 4帧 (分频): 平均 ~0.13ms/cycle
 *   - I2C IMU 读取 (400kHz): ~0.6ms
 *   - CAN RX 处理: ~0.1ms
 *   - 控制计算: ~0.3ms
 *   - 总计: ~1.4ms < 2ms ✓
 */
void task_unified_control(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(UNIFIED_TASK_PERIOD_MS);
    const float dt = UNIFIED_TASK_PERIOD_MS / 1000.0f;
    static uint8_t leg_divider_count = 0;
    imu_data_t imu_raw;
    
    ESP_LOGI(TAG, "[task_unified_control] Started on Core %d", xPortGetCoreID());
    ESP_LOGI(TAG, "  Period: %d ms (%.0f Hz)", UNIFIED_TASK_PERIOD_MS, 1000.0f / UNIFIED_TASK_PERIOD_MS);
    ESP_LOGI(TAG, "  Mode: IMU + Control + Motor in ONE task");
    
    while (g_tasks_running) {
        uint64_t cycle_start = esp_timer_get_time();
        
        // ======== Step 1: 先发送上一轮计算好的电机命令 ========
        // 尽早发送命令，让电机有更多时间回复
        // (首次循环发送的是初始值 0，无影响)
        apply_motor_commands();
        g_motor_count_per_sec++;
        
        // 腿电机 (分频执行)
        leg_divider_count++;
        if (leg_divider_count >= LEG_MOTOR_DIVIDER) {
            leg_divider_count = 0;
            apply_leg_motor_commands();
            g_leg_count_per_sec++;
        }
        
        // ======== Step 2: 读取 IMU 数据 ========
        if (imu_read_data(&imu_raw) == ESP_OK && imu_raw.is_valid) {
            g_imu_data.pitch = imu_raw.pitch;
            g_imu_data.pitch_rate = imu_raw.gyro_y;
            g_imu_data.roll = imu_raw.roll;
            g_imu_data.roll_rate = imu_raw.gyro_x;
            g_imu_data.yaw = imu_raw.yaw;
            g_imu_data.yaw_rate = imu_raw.gyro_z;
            g_imu_data.accel_x = imu_raw.accel_x;
            g_imu_data.accel_y = imu_raw.accel_y;
            g_imu_data.accel_z = imu_raw.accel_z;
            g_imu_data.timestamp = imu_raw.timestamp;
            g_imu_data.read_time_us = cycle_start;
            g_imu_data.valid = true;
            
            g_stats.imu_read_count++;
            g_imu_count_per_sec++;
        }
        
        // ======== Step 2.5: 自动使能 (等待 IMU 稳定 + 姿态安全) ========
        // 在 READY 状态下, 连续 500 帧 (1 秒) IMU 有效且 pitch < 30° 后自动使能
        {
            static int auto_enable_count = 0;
            const int AUTO_ENABLE_THRESHOLD = 500;  // 500 frames @ 500Hz = 1s
            const float AUTO_ENABLE_MAX_PITCH = 30.0f;
            
            if (g_state == BALANCE_TEST_READY && g_imu_data.valid && !g_auto_enable_inhibited) {
                if (fabsf(g_imu_data.pitch) < AUTO_ENABLE_MAX_PITCH) {
                    auto_enable_count++;
                    if (auto_enable_count >= AUTO_ENABLE_THRESHOLD) {
                        ESP_LOGI(TAG, "Auto-enable: IMU stable, pitch=%.1f deg", g_imu_data.pitch);
                        balance_test_enable();
                        auto_enable_count = 0;
                    }
                } else {
                    auto_enable_count = 0;  // 姿态不安全, 重新计数
                }
            } else if (g_state != BALANCE_TEST_READY) {
                auto_enable_count = 0;  // 非 READY 状态, 重置计数
            }
        }
        
        // ======== Step 3: 处理 CAN 接收 ========
        // 此时上一轮命令的电机回复 + 刚发命令后的快速回复都可能已到达
        // IMU 读取耗时 ~0.3-1ms，给了电机足够的回复时间
        can_motor_process_rx();
        
        // ======== Step 4: 读取轮电机状态 ========
        g_wheel_state.left_position = can_motor_read_position(g_motor_left);
        g_wheel_state.left_speed = can_motor_read_speed(g_motor_left);
        g_wheel_state.left_online = can_motor_is_online(g_motor_left, 100);
        g_wheel_state.right_position = can_motor_read_position(g_motor_right);
        g_wheel_state.right_speed = can_motor_read_speed(g_motor_right);
        g_wheel_state.right_online = can_motor_is_online(g_motor_right, 100);
        g_wheel_state.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
        
        // ======== Step 5: 更新遥控数据 ========
        update_remote_from_wifi();
        
        // ======== Step 6: 计算平衡控制输出 ========
        // 此时电机状态是最新的 (刚 process_rx 过)
        uint64_t ctrl_start = esp_timer_get_time();
        compute_balance_output(dt);
        uint64_t ctrl_end = esp_timer_get_time();
        
        // ======== Step 7: 跳跃状态机更新 (在平衡输出之后) ========
        jump_state_machine_update();
        
        // ======== Step 8: 起身状态机更新 ========
        standup_state_machine_update();
        
        // ======== Step 9: 平衡→小车过渡状态机更新 ========
        bal_to_car_state_machine_update();
        
        // 延迟统计
        g_latency_imu_to_ctrl_us = (float)(ctrl_start - cycle_start);
        g_latency_ctrl_calc_us = (float)(ctrl_end - ctrl_start);
        
        g_stats.control_loop_count++;
        g_ctrl_count_per_sec++;
        
        g_stats.motor_cmd_count++;
        
        // 记录总延迟
        uint64_t cycle_end = esp_timer_get_time();
        g_latency_ctrl_to_motor_us = (float)(cycle_end - ctrl_end);
        g_latency_total_us = (float)(cycle_end - cycle_start);
        
        // 更新最大/最小值
        if (g_latency_total_us > g_latency_total_max_us) {
            g_latency_total_max_us = g_latency_total_us;
        }
        if (g_latency_total_us < g_latency_total_min_us) {
            g_latency_total_min_us = g_latency_total_us;
        }
        
        // 每秒更新一次频率统计
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - g_last_stat_time >= 1000) {
            g_stats.imu_freq_hz = (float)g_imu_count_per_sec;
            g_stats.control_freq_hz = (float)g_ctrl_count_per_sec;
            g_stats.motor_freq_hz = (float)g_motor_count_per_sec;
            g_stats.leg_freq_hz = (float)g_leg_count_per_sec;
            g_imu_count_per_sec = 0;
            g_ctrl_count_per_sec = 0;
            g_motor_count_per_sec = 0;
            g_leg_count_per_sec = 0;
            g_last_stat_time = now;
        }
        
        // 更新 pi_comm 状态 (仅在使能时)
        if (g_pi_comm_enabled) {
            update_pi_comm_state();
        }
        
        vTaskDelayUntil(&last_wake, period);
    }
    
    ESP_LOGI(TAG, "[task_unified_control] Stopped");
    g_task_unified = NULL;
    vTaskDelete(NULL);
}

/**
 * @brief 遥控看门狗任务 (Core 0, 100ms 周期)
 */
void task_remote_watchdog(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(WATCHDOG_PERIOD_MS);
    
    ESP_LOGI(TAG, "[task_remote_watchdog] Started on Core %d", xPortGetCoreID());
    
    while (g_tasks_running) {
        // 检查遥控超时
        if (wifi_remote_check_timeout(REMOTE_TIMEOUT_MS)) {
            // 超时，紧急停止 (仅在非 EMERGENCY 状态时触发一次)
            if (g_state != BALANCE_TEST_EMERGENCY) {
                ESP_LOGW(TAG, "Remote timeout, emergency stop");
                balance_test_emergency_stop();
            }
        } else {
            // WiFi 恢复连接，自动清除超时导致的 EMERGENCY 状态
            if (g_state == BALANCE_TEST_EMERGENCY) {
                ESP_LOGI(TAG, "Remote reconnected, clearing emergency");
                balance_test_reset_emergency();
            }
        }
        
        // 统计 WiFi 消息数
        remote_data_t *wifi_data = wifi_remote_get_data();
        g_stats.wifi_msg_count = wifi_data->msg_count;
        
        vTaskDelayUntil(&last_wake, period);
    }
    
    ESP_LOGI(TAG, "[task_remote_watchdog] Stopped");
    g_task_watchdog = NULL;
    vTaskDelete(NULL);
}

// ============================================================================
// 内部函数实现
// ============================================================================



/**
 * @brief 从 WiFi 模块更新遥控数据
 */
void update_remote_from_wifi(void) {
    remote_data_t *wifi_data = wifi_remote_get_data();
    
    xSemaphoreTake(g_remote_mutex, portMAX_DELAY);
    
    // 保存上一次数据
    g_remote_data.joy_x_last = g_remote_data.joy_x;
    g_remote_data.joy_y_last = g_remote_data.joy_y;
    
    // 更新新数据
    g_remote_data.joy_x = wifi_data->joy_x;
    g_remote_data.joy_y = wifi_data->joy_y;
    g_remote_data.go = wifi_data->go;
    g_remote_data.car_mode = wifi_data->car_mode;
    g_remote_data.last_update = wifi_data->last_update_ms;
    g_remote_data.receive_time_us = wifi_data->receive_time_us;  // 复制精确时间戳
    
    xSemaphoreGive(g_remote_mutex);
    
    // ======== 处理紧急停止 ========
    static bool last_estop = false;
    if (wifi_data->estop && !last_estop) {
        ESP_LOGW(TAG, "WiFi: E-STOP activated!");
        balance_test_emergency_stop();
    } else if (!wifi_data->estop && last_estop) {
        ESP_LOGI(TAG, "WiFi: E-STOP released");
        balance_test_reset_emergency();
    }
    last_estop = wifi_data->estop;
    
    // ======== 处理平衡使能 ========
    static bool last_balance_enable = false;
    if (wifi_data->balance_enable && !last_balance_enable) {
        if (!wifi_data->estop) {
            balance_test_enable();
            ESP_LOGI(TAG, "WiFi: Balance ENABLED");
        }
    } else if (!wifi_data->balance_enable && last_balance_enable) {
        balance_test_disable();
        ESP_LOGI(TAG, "WiFi: Balance DISABLED");
    }
    last_balance_enable = wifi_data->balance_enable;
    
    // ======== 处理控制模式切换 ========
    static int8_t last_ctrl_mode = 0;
    if (wifi_data->control_mode != last_ctrl_mode && !wifi_data->estop) {
        control_mode_t new_mode;
        const char *mode_str = "Unknown";
        switch (wifi_data->control_mode) {
            case 0: new_mode = CTRL_MODE_LQR; mode_str = "LQR"; break;
            case 1: new_mode = CTRL_MODE_DUAL_PID; mode_str = "DUAL_PID"; break;
            case 2: new_mode = CTRL_MODE_SINGLE_PID; mode_str = "SINGLE_PID"; break;
            case 4: new_mode = CTRL_MODE_TRIPLE_PID; mode_str = "TRIPLE_PID"; break;
            default: new_mode = g_control_mode; break;  // 无效值忽略
        }
        if (new_mode != g_control_mode && g_control_mode != CTRL_MODE_CAR) {
            g_control_mode = new_mode;
            // 根据模式设置电机模式
            if (g_state == BALANCE_TEST_RUNNING) {
                if (g_control_mode == CTRL_MODE_SINGLE_PID ||
                    (g_control_mode == CTRL_MODE_TRIPLE_PID && g_triple_pid_ctrl.params.wheel_mode == TRIPLE_PID_WHEEL_SPEED)) {
                    can_motor_set_mode(g_motor_left, MODE_SPEED);
                    can_motor_set_mode(g_motor_right, MODE_SPEED);
                } else {
                    can_motor_set_mode(g_motor_left, MODE_TORQUE);
                    can_motor_set_mode(g_motor_right, MODE_TORQUE);
                }
            }
            ESP_LOGI(TAG, "WiFi: Control mode → %s", mode_str);
            printf("CTRL_MODE:%s\n", mode_str);
        }
    }
    last_ctrl_mode = wifi_data->control_mode;
    
    // ======== 处理小车模式开关 ========
    static bool last_car_mode = false;
    
    // 检测 standup 等内部流程设置了 CAR 模式但 WiFi UI 不知情的情况
    // 此时 wifi_data->car_mode == false 且 last_car_mode == false, 边沿检测失效
    // 需要主动退出 CAR 模式, 恢复到之前的控制模式
    if (g_control_mode == CTRL_MODE_CAR && !wifi_data->car_mode && !last_car_mode) {
        if (g_state == BALANCE_TEST_RUNNING) {
            can_motor_set_speed(g_motor_left, 0);
            can_motor_set_speed(g_motor_right, 0);
        }
        g_control_mode = g_car_mode_prev_mode;
        if (g_state == BALANCE_TEST_RUNNING) {
            if (g_control_mode == CTRL_MODE_SINGLE_PID ||
                (g_control_mode == CTRL_MODE_TRIPLE_PID && g_triple_pid_ctrl.params.wheel_mode == TRIPLE_PID_WHEEL_SPEED)) {
                can_motor_set_mode(g_motor_left, MODE_SPEED);
                can_motor_set_mode(g_motor_right, MODE_SPEED);
            } else {
                can_motor_set_mode(g_motor_left, MODE_TORQUE);
                can_motor_set_mode(g_motor_right, MODE_TORQUE);
            }
        }
        const char *restored_str = (g_control_mode == CTRL_MODE_LQR) ? "LQR" :
                                   (g_control_mode == CTRL_MODE_DUAL_PID) ? "DUAL_PID" :
                                   (g_control_mode == CTRL_MODE_TRIPLE_PID) ? "TRIPLE_PID" :
                                   (g_control_mode == CTRL_MODE_FULL_LQR) ? "FULL_LQR" : "SINGLE_PID";
        if (g_leg_control_enabled) {
            leg_ctrl_set_target(true,  CAR_TO_BAL_LEG_LENGTH, -90.0f);
            leg_ctrl_set_target(false, CAR_TO_BAL_LEG_LENGTH, -90.0f);
        }
        ESP_LOGW(TAG, "WiFi: Auto-exit CAR mode (set by standup), restored to %s", restored_str);
        printf("CTRL_MODE:%s\n", restored_str);
        g_car_standup_state = CAR_STANDUP_IDLE;  // 中止任何进行中的 car standup
    }
    
    if (wifi_data->car_mode && !last_car_mode && !wifi_data->estop) {
        // 进入小车模式
        g_car_standup_state = CAR_STANDUP_IDLE;  // 进入 car mode 时重置起身状态机
        g_car_mode_prev_mode = g_control_mode;
        g_car_mode_prev_base_angle = g_leg_base_angle;
        g_car_mode_prev_base_length = g_leg_base_length;

        // 判断当前是否处于平衡模式: 若是，触发过渡状态机
        bool is_balance_mode = (g_control_mode == CTRL_MODE_LQR ||
                                g_control_mode == CTRL_MODE_DUAL_PID ||
                                g_control_mode == CTRL_MODE_SINGLE_PID ||
                                g_control_mode == CTRL_MODE_TRIPLE_PID ||
                                g_control_mode == CTRL_MODE_FULL_LQR);

        if (is_balance_mode && g_state == BALANCE_TEST_RUNNING && g_leg_control_enabled
            && g_bal_to_car_state == BAL_TO_CAR_IDLE) {
            // 触发平衡→小车过渡状态机 (不立即切换 CTRL_MODE_CAR)
            g_bal_to_car_state = BAL_TO_CAR_RETRACT;
            g_bal_to_car_enter_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
            // 开始收腿 (平衡控制仍继续运行)
            leg_ctrl_set_target(true,  BAL_TO_CAR_RETRACT_LENGTH, g_leg_base_angle);
            leg_ctrl_set_target(false, BAL_TO_CAR_RETRACT_LENGTH, g_leg_base_angle);
            ESP_LOGW(TAG, "WiFi: BAL→CAR transition START (retract leg to %.0fmm)",
                     BAL_TO_CAR_RETRACT_LENGTH * 1000.0f);
        } else {
            // 非平衡模式或未运行: 直接切换
            g_control_mode = CTRL_MODE_CAR;
            if (g_state == BALANCE_TEST_RUNNING) {
                can_motor_set_mode(g_motor_left, MODE_SPEED);
                can_motor_set_mode(g_motor_right, MODE_SPEED);
            }
            if (g_leg_control_enabled) {
                leg_ctrl_set_target(true,  CAR_MODE_LEG_LENGTH, CAR_MODE_BODY_ANGLE);
                leg_ctrl_set_target(false, CAR_MODE_LEG_LENGTH, CAR_MODE_BODY_ANGLE);
            }
            ESP_LOGI(TAG, "WiFi: CAR mode ON (direct, body_angle=%.0f°, leg=%.0fmm)",
                     CAR_MODE_BODY_ANGLE, CAR_MODE_LEG_LENGTH * 1000.0f);
            printf("CTRL_MODE:CAR\n");
        }
    } else if (!wifi_data->car_mode && last_car_mode) {
        // 退出小车模式: 恢复之前的模式和腿部姿态
        if (g_control_mode == CTRL_MODE_CAR) {
            if (g_state == BALANCE_TEST_RUNNING) {
                can_motor_set_speed(g_motor_left, 0);
                can_motor_set_speed(g_motor_right, 0);
            }
            
            if (g_leg_control_enabled) {
                leg_ctrl_set_target(true,  CAR_TO_BAL_LEG_LENGTH, -90.0f);
                leg_ctrl_set_target(false, CAR_TO_BAL_LEG_LENGTH, -90.0f);
            }
            
            g_control_mode = g_car_mode_prev_mode;
            
            if (g_state == BALANCE_TEST_RUNNING) {
                if (g_control_mode == CTRL_MODE_SINGLE_PID ||
                    (g_control_mode == CTRL_MODE_TRIPLE_PID && g_triple_pid_ctrl.params.wheel_mode == TRIPLE_PID_WHEEL_SPEED)) {
                    can_motor_set_mode(g_motor_left, MODE_SPEED);
                    can_motor_set_mode(g_motor_right, MODE_SPEED);
                } else {
                    can_motor_set_mode(g_motor_left, MODE_TORQUE);
                    can_motor_set_mode(g_motor_right, MODE_TORQUE);
                }
            }
            
            const char *restored_str = (g_control_mode == CTRL_MODE_LQR) ? "LQR" :
                                       (g_control_mode == CTRL_MODE_DUAL_PID) ? "DUAL_PID" :
                                       (g_control_mode == CTRL_MODE_TRIPLE_PID) ? "TRIPLE_PID" :
                                       (g_control_mode == CTRL_MODE_FULL_LQR) ? "FULL_LQR" : "SINGLE_PID";
            ESP_LOGI(TAG, "WiFi: CAR mode OFF, restored to %s", restored_str);
            printf("CTRL_MODE:%s\n", restored_str);
            g_car_standup_state = CAR_STANDUP_IDLE;  // 中止任何进行中的 car standup
            g_bal_to_car_state = BAL_TO_CAR_IDLE;    // 中止任何进行中的过渡状态机
        }
    } else if (!wifi_data->car_mode && last_car_mode && g_bal_to_car_state != BAL_TO_CAR_IDLE) {
        // car_mode 在过渡状态机运行中被关闭 → 中止
        g_bal_to_car_state = BAL_TO_CAR_IDLE;
        ESP_LOGW(TAG, "WiFi: CAR mode OFF during BAL→CAR transition, aborted");
    }
    last_car_mode = wifi_data->car_mode;
    
    // ======== 处理 Pitch 补偿开关 ========
    static bool last_pitch_comp = false;
    if (wifi_data->pitch_comp != last_pitch_comp) {
        balance_test_set_pitch_comp(wifi_data->pitch_comp);
        ESP_LOGI(TAG, "WiFi: Pitch comp %s", wifi_data->pitch_comp ? "ON" : "OFF");
    }
    last_pitch_comp = wifi_data->pitch_comp;
    
    // ======== 处理遥杆增益 ========
    static float last_joy_speed_gain = 0.003f;
    static float last_joy_yaw_gain = 0.03f;
    if (fabsf(wifi_data->joy_speed_gain - last_joy_speed_gain) > 0.0001f) {
        g_joy_speed_scale = wifi_data->joy_speed_gain;
        ESP_LOGI(TAG, "WiFi: Joy speed scale = %.4f", g_joy_speed_scale);
        last_joy_speed_gain = wifi_data->joy_speed_gain;
    }
    if (fabsf(wifi_data->joy_yaw_gain - last_joy_yaw_gain) > 0.0001f) {
        g_joy_yaw_scale = wifi_data->joy_yaw_gain;
        ESP_LOGI(TAG, "WiFi: Joy yaw scale = %.4f", g_joy_yaw_scale);
        last_joy_yaw_gain = wifi_data->joy_yaw_gain;
    }
    
    // ======== 处理位移环开关 ========
    static bool last_dist_enable = true;  // 默认开启
    if (wifi_data->dist_enable != last_dist_enable) {
        g_triple_pid_ctrl.params.distance_enable = wifi_data->dist_enable ? 1 : 0;
        if (wifi_data->dist_enable) {
            // 启用时重置位移零点为当前位置，避免跳变
            triple_pid_set_distance_zeropoint(&g_triple_pid_ctrl, g_lqr_distance);
            g_distance_zeropoint = g_lqr_distance;
            pid_reset(&g_triple_pid_ctrl.pid_distance);
        }
        ESP_LOGI(TAG, "WiFi: Distance loop %s", wifi_data->dist_enable ? "ENABLED" : "DISABLED");
        printf("Triple PID distance loop %s\n", wifi_data->dist_enable ? "ENABLED" : "DISABLED");
        printf("TPID:DISTEN,%d\n", wifi_data->dist_enable ? 1 : 0);
        last_dist_enable = wifi_data->dist_enable;
    }
    
    // ======== 处理 Yaw 闭环开关 ========
    static bool last_yaw_enable = false;  // 默认关闭
    if (wifi_data->yaw_enable != last_yaw_enable) {
        g_yaw_control_enabled = wifi_data->yaw_enable;
        if (!wifi_data->yaw_enable) {
            g_yaw_output = 0.0f;
        } else {
            // 开启时同步方向锁定点到当前航向角，避免累积误差冲击
            g_lqr_ctrl.yaw_angle_target = g_yaw_angle_total;
            g_lqr_ctrl.yaw_holding = false;
            pid_reset(&g_lqr_ctrl.pid_yaw_angle);
            pid_reset(&g_lqr_ctrl.pid_yaw_gyro);
        }
        ESP_LOGW(TAG, "WiFi: Yaw %s, target=%.1f total=%.1f",
                 wifi_data->yaw_enable ? "ON" : "OFF",
                 g_lqr_ctrl.yaw_angle_target, g_yaw_angle_total);
        last_yaw_enable = wifi_data->yaw_enable;
    }
    
    // ======== 处理差速转向开关 ========
    static bool last_diff_speed = false;
    if (wifi_data->diff_speed_enable != last_diff_speed) {
        g_diff_speed_enabled = wifi_data->diff_speed_enable;
        ESP_LOGI(TAG, "WiFi: Diff steer %s", g_diff_speed_enabled ? "ENABLED" : "DISABLED");
        last_diff_speed = wifi_data->diff_speed_enable;
    }
    
    // ======== 处理 Roll 闭环开关 ========
    static bool last_roll_enable = false;  // 默认关闭
    if (wifi_data->roll_enable != last_roll_enable) {
        balance_test_set_roll_control(wifi_data->roll_enable);
        ESP_LOGI(TAG, "WiFi: Roll control %s", wifi_data->roll_enable ? "ENABLED" : "DISABLED");
        last_roll_enable = wifi_data->roll_enable;
    }
    
    // ======== 处理三环PID速度源切换 ========
    static bool last_obsv_speed = false;
    if (wifi_data->obsv_speed != last_obsv_speed) {
        g_tpid_use_observer_speed = wifi_data->obsv_speed;
        ESP_LOGI(TAG, "WiFi: TPID speed source = %s", g_tpid_use_observer_speed ? "OBSERVER" : "WHEEL");
        last_obsv_speed = wifi_data->obsv_speed;
    }
    
    // ======== 处理X-Offset开关 ========
    static bool last_xoffset_enable = false;
    if (wifi_data->xoffset_enable != last_xoffset_enable) {
        balance_test_set_xoffset(wifi_data->xoffset_enable);
        ESP_LOGI(TAG, "WiFi: X-Offset %s", wifi_data->xoffset_enable ? "ENABLED" : "DISABLED");
        last_xoffset_enable = wifi_data->xoffset_enable;
    }

    // ======== 处理X-Offset Kp调节 ========
    static float last_xoffset_kp = 0.1f;
    if (fabsf(wifi_data->xoffset_kp - last_xoffset_kp) > 0.001f) {
        balance_test_set_xoffset_pid(wifi_data->xoffset_kp, g_xoffset_pid.ki, g_xoffset_pid.kd);
        ESP_LOGI(TAG, "WiFi: X-Offset Kp=%.4f", wifi_data->xoffset_kp);
        last_xoffset_kp = wifi_data->xoffset_kp;
    }

    // ======== 处理角度零点调节 ========
    static float last_angle_zero = -1.5f;
    if (fabsf(wifi_data->angle_zero - last_angle_zero) > 0.01f) {
        balance_test_set_angle_zeropoint(wifi_data->angle_zero);
        last_angle_zero = wifi_data->angle_zero;
    }
    
    // ======== 处理跳跃按钮 (上升沿触发) ========
    {
        bool jump_btn = wifi_data->jump || (wifi_data->dir == DIR_JUMP);
        if (false && jump_btn && !jump_is_active()) { // DEBUG: disabled
            // 上升沿 + 当前空闲 → 启动跳跃序列
            bool both_on_ground = (!g_sforce_left_off && !g_sforce_right_off);
            if (g_leg_control_enabled && !wifi_data->estop && both_on_ground) {
                jump_trigger(g_leg_base_length, g_leg_base_angle);
            } else if (!both_on_ground) {
                ESP_LOGW(TAG, "JUMP: IGNORED - not both on ground (L=%d R=%d)",
                         g_sforce_left_off, g_sforce_right_off);
            }
        }
        jump_set_btn(jump_btn);
    }
    
    // ======== 处理起身按钮 (上升沿触发) ========
    {
        bool standup_btn = wifi_data->standup;
        if (false && standup_btn && !g_standup_last_btn && g_standup_state == STANDUP_IDLE) { // DEBUG: disabled
            if (g_leg_control_enabled && !wifi_data->estop) {
                // 安全检查: 4个关节电机必须在零位附近 (双腿伸直向后)
                float lh = fabsf(can_motor_read_position(g_motor_left_hip));
                float lk = fabsf(can_motor_read_position(g_motor_left_knee));
                float rh = fabsf(can_motor_read_position(g_motor_right_hip));
                float rk = fabsf(can_motor_read_position(g_motor_right_knee));
                
                if (lh < g_standup_zero_threshold && lk < g_standup_zero_threshold &&
                    rh < g_standup_zero_threshold && rk < g_standup_zero_threshold) {
                    // 读取当前 FK 身体夹角作为收腿角度
                    leg_state_t left_st;
                    if (leg_ctrl_get_state_cached(true, &left_st) == ESP_OK && left_st.valid) {
                        g_standup_retract_angle = left_st.workspace.body_angle;
                    }
                    
                    g_standup_state = STANDUP_RETRACT;
                    g_standup_state_enter_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                    
                    // 收腿到最短, 保持当前身体夹角 (向后)
                    leg_ctrl_set_target(true, g_standup_retract_length, g_standup_retract_angle);
                    leg_ctrl_set_target(false, g_standup_retract_length, g_standup_retract_angle);
                    
                    ESP_LOGW(TAG, "STANDUP: START! retract=%.0fmm angle=%.0f°",
                             g_standup_retract_length * 1000.0f, g_standup_retract_angle);
                    ESP_LOGW(TAG, "  joint pos: LH=%.1f LK=%.1f RH=%.1f RK=%.1f",
                             can_motor_read_position(g_motor_left_hip),
                             can_motor_read_position(g_motor_left_knee),
                             can_motor_read_position(g_motor_right_hip),
                             can_motor_read_position(g_motor_right_knee));
                } else {
                    ESP_LOGW(TAG, "STANDUP: REJECTED! Not at zero (LH=%.1f LK=%.1f RH=%.1f RK=%.1f thresh=%.0f)",
                             lh, lk, rh, rk, g_standup_zero_threshold);
                }
            }
        }
        g_standup_last_btn = standup_btn;
    }
    
    // ======== 处理小车模式起身按钮 (上升沿触发) ========
    {
        bool car_standup_btn = wifi_data->car_standup;
        if (car_standup_btn && !g_car_standup_last_btn) {
            if (g_control_mode == CTRL_MODE_CAR && g_leg_control_enabled &&
                !wifi_data->estop && g_car_standup_state == CAR_STANDUP_IDLE) {
                g_car_standup_state = CAR_STANDUP_SWING;
                g_car_standup_enter_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                leg_ctrl_set_target(true,  g_leg_length_min, -90.0f);
                leg_ctrl_set_target(false, g_leg_length_min, -90.0f);
                ESP_LOGW(TAG, "CAR_STANDUP: triggered, swinging to -90deg");
            } else if (g_car_standup_state == CAR_STANDUP_IDLE) {
                ESP_LOGW(TAG, "CAR_STANDUP: ignored (mode=%d leg=%d estop=%d)",
                         g_control_mode, g_leg_control_enabled, wifi_data->estop);
            }
        }
        g_car_standup_last_btn = car_standup_btn;
    }
    
    // ======== 处理腿部使能 ========
    static bool last_leg_enable = false;
    if (wifi_data->leg_enable != last_leg_enable && !wifi_data->estop) {
        balance_test_set_leg_control(wifi_data->leg_enable);
        ESP_LOGI(TAG, "WiFi: Leg control %s", wifi_data->leg_enable ? "ENABLED" : "DISABLED");
    }
    last_leg_enable = wifi_data->leg_enable;
    
    // ======== 处理腿部角度和长度 ========
    // 跳跃/起身过程中不允许遥控覆盖腿长 (由状态机接管)
    if (g_leg_control_enabled && !wifi_data->estop && !jump_is_active() && !standup_is_active() && !car_standup_is_active() && !bal_to_car_is_active()) {
        static float last_leg_angle = -90.0f;
        static float last_leg_length = 0.09f;
        if (fabsf(wifi_data->leg_angle - last_leg_angle) > 0.5f ||
            fabsf(wifi_data->leg_length - last_leg_length) > 0.001f) {
            g_leg_base_angle = wifi_data->leg_angle;
            g_leg_base_length = wifi_data->leg_length;
            leg_ctrl_set_target(true, g_leg_base_length, g_leg_base_angle);
            leg_ctrl_set_target(false, g_leg_base_length, g_leg_base_angle);
            last_leg_angle = wifi_data->leg_angle;
            last_leg_length = wifi_data->leg_length;
        }
    }
    
    // ======== 详细调控模式 ========
    static bool last_detail_mode = false;
    static control_mode_t detail_prev_mode = CTRL_MODE_TRIPLE_PID;
    static bool detail_prev_go = false;
    if (wifi_data->detail_mode && !last_detail_mode) {
        // 进入详细调控模式: 保存当前状态, 停止平衡控制, 切换轮电机到速度模式
        detail_prev_mode = g_control_mode;
        detail_prev_go = wifi_data->go;
        balance_test_disable();
        if (g_motor_left != NULL) {
            can_motor_set_mode(g_motor_left, MODE_SPEED);
            can_motor_set_mode(g_motor_right, MODE_SPEED);
            can_motor_set_speed(g_motor_left, 0);
            can_motor_set_speed(g_motor_right, 0);
        }
        ESP_LOGI(TAG, "WiFi: Detail control mode ENTERED (prev_mode=%d)", detail_prev_mode);
    } else if (!wifi_data->detail_mode && last_detail_mode) {
        // 退出详细调控模式: 停止轮电机, 恢复之前的控制模式和电机模式
        if (g_motor_left != NULL) {
            can_motor_set_speed(g_motor_left, 0);
            can_motor_set_speed(g_motor_right, 0);
        }
        // 恢复控制模式
        g_control_mode = detail_prev_mode;
        // 恢复电机模式 (根据控制模式选择扭矩/速度)
        if (g_motor_left != NULL) {
            if (g_control_mode == CTRL_MODE_SINGLE_PID || g_control_mode == CTRL_MODE_CAR ||
                (g_control_mode == CTRL_MODE_TRIPLE_PID && g_triple_pid_ctrl.params.wheel_mode == TRIPLE_PID_WHEEL_SPEED)) {
                can_motor_set_mode(g_motor_left, MODE_SPEED);
                can_motor_set_mode(g_motor_right, MODE_SPEED);
            } else {
                can_motor_set_mode(g_motor_left, MODE_TORQUE);
                can_motor_set_mode(g_motor_right, MODE_TORQUE);
            }
        }
        const char *restored_str = (detail_prev_mode == CTRL_MODE_LQR) ? "LQR" :
                                   (detail_prev_mode == CTRL_MODE_DUAL_PID) ? "DUAL_PID" :
                                   (detail_prev_mode == CTRL_MODE_TRIPLE_PID) ? "TRIPLE_PID" :
                                   (detail_prev_mode == CTRL_MODE_SINGLE_PID) ? "SINGLE_PID" : "CAR";
        ESP_LOGI(TAG, "WiFi: Detail control mode EXITED → restored %s", restored_str);
        printf("CTRL_MODE:%s\n", restored_str);
    }
    last_detail_mode = wifi_data->detail_mode;
    
    // 详细调控模式下的实时控制
    if (wifi_data->detail_mode && !wifi_data->estop) {
        // --- 左腿控制 ---
        static float last_dl_len = 0.09f, last_dl_ang = -90.0f;
        static float last_dr_len = 0.09f, last_dr_ang = -90.0f;
        
        if (wifi_data->detail_sync) {
            // 协同模式: 左侧滑条同时控制双腿
            if (fabsf(wifi_data->detail_left_length - last_dl_len) > 0.001f ||
                fabsf(wifi_data->detail_left_angle - last_dl_ang) > 0.5f) {
                if (g_leg_control_enabled) {
                    leg_ctrl_set_target(true, wifi_data->detail_left_length, wifi_data->detail_left_angle);
                    leg_ctrl_set_target(false, wifi_data->detail_left_length, wifi_data->detail_left_angle);
                }
                last_dl_len = wifi_data->detail_left_length;
                last_dl_ang = wifi_data->detail_left_angle;
                last_dr_len = wifi_data->detail_left_length;
                last_dr_ang = wifi_data->detail_left_angle;
            }
        } else {
            // 独立模式: 左右分别控制
            if (fabsf(wifi_data->detail_left_length - last_dl_len) > 0.001f ||
                fabsf(wifi_data->detail_left_angle - last_dl_ang) > 0.5f) {
                if (g_leg_control_enabled) {
                    leg_ctrl_set_target(true, wifi_data->detail_left_length, wifi_data->detail_left_angle);
                }
                last_dl_len = wifi_data->detail_left_length;
                last_dl_ang = wifi_data->detail_left_angle;
            }
            if (fabsf(wifi_data->detail_right_length - last_dr_len) > 0.001f ||
                fabsf(wifi_data->detail_right_angle - last_dr_ang) > 0.5f) {
                if (g_leg_control_enabled) {
                    leg_ctrl_set_target(false, wifi_data->detail_right_length, wifi_data->detail_right_angle);
                }
                last_dr_len = wifi_data->detail_right_length;
                last_dr_ang = wifi_data->detail_right_angle;
            }
        }
        
        // --- 轮速控制 ---
        static float last_dl_spd = 0.0f, last_dr_spd = 0.0f;
        if (wifi_data->detail_sync) {
            // 协同模式: 左侧速度同时控制双轮
            if (fabsf(wifi_data->detail_left_speed - last_dl_spd) > 0.5f) {
                if (g_motor_left != NULL) {
                    can_motor_set_speed(g_motor_left, wifi_data->detail_left_speed);
                    can_motor_set_speed(g_motor_right, wifi_data->detail_left_speed);
                }
                last_dl_spd = wifi_data->detail_left_speed;
                last_dr_spd = wifi_data->detail_left_speed;
            }
        } else {
            // 独立模式: 左右分别控制
            if (fabsf(wifi_data->detail_left_speed - last_dl_spd) > 0.5f) {
                if (g_motor_left != NULL) {
                    can_motor_set_speed(g_motor_left, wifi_data->detail_left_speed);
                }
                last_dl_spd = wifi_data->detail_left_speed;
            }
            if (fabsf(wifi_data->detail_right_speed - last_dr_spd) > 0.5f) {
                if (g_motor_right != NULL) {
                    can_motor_set_speed(g_motor_right, wifi_data->detail_right_speed);
                }
                last_dr_spd = wifi_data->detail_right_speed;
            }
        }
    }
}
