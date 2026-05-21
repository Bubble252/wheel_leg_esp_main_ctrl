/**
 * @file balance_control.c
 * @brief 核心控制算法实现
 *
 * 从 balance_test.c 拆分出来。包含:
 * - compute_balance_output(): 平衡控制主算法 (按控制模式计算输出)
 * - apply_motor_commands(): 轮电机命令发送
 *
 * 控制模式:
 * - CTRL_MODE_LQR: 简单 LQR 平衡
 * - CTRL_MODE_DUAL_PID: 双环 PID
 * - CTRL_MODE_SINGLE_PID: 单环 PID
 * - CTRL_MODE_CAR: 小车模式
 * - CTRL_MODE_TRIPLE_PID: 三环 PID
 * - CTRL_MODE_FULL_LQR: Full LQR (含腿力控制)
 */

#include "balance_test.h"
#include "balance_types.h"
#include "balance_vmc.h"
#include "balance_observer.h"
#include "balance_plot.h"
#include "balance_standup.h"
#include "balance_roll.h"
#include "balance_jump.h"
#include "config.h"
#include "lqr_balance.h"
#include "full_lqr.h"
#include "leg_kinematics.h"
#include "can_motor.h"
#include "can_motor_stw_regs.h"
#include "imu_driver.h"
#include "lowpass_filter.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <math.h>

static const char *TAG = "BAL_CTRL";

// 角度-弧度转换
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define DEG2RAD(d) ((d) * M_PI / 180.0f)
#define RAD2DEG(r) ((r) * 180.0f / M_PI)

// 安全限幅 (与 balance_test.c 保持一致)
#define WHEEL_TORQUE_LIMIT          0.3f
#define REMOTE_TIMEOUT_MS           500
#define EMERGENCY_ANGLE_DEG         45.0f
#define CAR_MODE_MAX_SPEED          (200.0f)
#define CAR_MODE_YAW_GAIN           (80.0f)
#define OFF_GROUND_ENTER_COUNT      8
#define OFF_GROUND_EXIT_COUNT       13

// ============================================================================
// extern 变量 (定义在 balance_test.c)
// ============================================================================

// 状态
extern balance_test_state_t g_state;
extern bool g_initialized;
extern volatile bool g_tasks_running;

// 控制器
extern lqr_controller_t g_lqr_ctrl;
extern float g_lqr_speed;
extern float g_lqr_distance;
extern float g_distance_zeropoint;

// Dual PID
extern dual_pid_controller_t g_dual_pid_ctrl;
extern bool g_dual_pid_initialized;
extern dual_pid_output_t g_dual_pid_output;

// Single PID
extern single_pid_controller_t g_single_pid_ctrl;
extern bool g_single_pid_initialized;
extern single_pid_output_t g_single_pid_output;

// Triple PID
extern triple_pid_controller_t g_triple_pid_ctrl;
extern bool g_triple_pid_initialized;
extern triple_pid_output_t g_triple_pid_output;
extern float g_tpid_yaw_scale;

// Full LQR
extern full_lqr_controller_t g_full_lqr_ctrl;
extern bool g_full_lqr_initialized;
extern full_lqr_output_t g_full_lqr_output_left;
extern full_lqr_output_t g_full_lqr_output_right;
extern bool g_full_lqr_stream_enable;
extern float g_full_lqr_wheel_T_left;
extern float g_full_lqr_wheel_T_right;
extern float g_full_lqr_Tp_left;
extern float g_full_lqr_Tp_right;

// 控制模式
extern control_mode_t g_control_mode;

// 环路使能
extern uint8_t g_loop_enable_mask;
extern bool g_loop_manual_mode;

// Yaw 控制
extern bool g_yaw_control_enabled;
extern bool g_yaw_force_enable;
extern float g_yaw_angle_last;
extern float g_yaw_angle_total;
extern float g_yaw_output;
extern bool g_yaw_first_run;

// Roll 控制
extern bool g_roll_control_enabled;
extern float g_roll_output;
extern float g_roll_left_delta;
extern float g_roll_right_delta;
extern float g_roll_filtered;

// 腿部控制
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
extern float g_leg_length_min;
extern float g_leg_length_max;
extern float g_leg_move_speed;

// Leg Sync
extern bool g_leg_sync_enabled;
extern float g_leg_sync_gain;
extern float g_leg_sync_max_correction;
extern float g_leg_sync_debug_diff;
extern float g_leg_sync_debug_correction;

// X-Offset
extern bool g_xoffset_enabled;
extern pid_controller_t g_xoffset_pid;
extern float g_xoffset_value;
extern float g_xoffset_limit;
extern float g_xoffset_debug_speed;

// 腿部补偿
extern bool g_pitch_leg_comp_enabled;

// 遥控器/手柄
extern float g_joy_speed_scale;
extern float g_joy_yaw_scale;
extern shared_remote_data_t g_remote_data;
extern SemaphoreHandle_t g_remote_mutex;

// 轮速/WMA
extern weighted_ma_filter_t g_wheel_speed_wma;
extern bool g_wma_enabled;
extern float g_left_wheel_speed_rad;
extern float g_right_wheel_speed_rad;
extern float g_left_wheel_accel;
extern float g_right_wheel_accel;
extern float g_prev_left_wheel_speed;
extern float g_prev_right_wheel_speed;
extern bool g_wheel_off_ground;
extern int g_off_ground_counter;

// 轮电机命令
extern shared_wheel_cmd_t g_wheel_cmd;
extern SemaphoreHandle_t g_wheel_cmd_mutex;
extern shared_wheel_state_t g_wheel_state;
extern SemaphoreHandle_t g_wheel_state_mutex;

// 差速转向
extern bool g_diff_speed_enabled;
extern float g_diff_speed_scale;

// 轮电机命令
extern shared_wheel_cmd_t g_wheel_cmd;
extern SemaphoreHandle_t g_wheel_cmd_mutex;

// IMU 数据
extern shared_imu_data_t g_imu_data;
extern SemaphoreHandle_t g_imu_mutex;

// 延迟测量
extern volatile uint64_t g_used_imu_time_us;
extern volatile uint64_t g_ctrl_start_time_us;
extern volatile uint64_t g_ctrl_end_time_us;
extern volatile uint64_t g_motor_send_time_us;
extern volatile uint64_t g_used_wifi_time_us;
extern float g_latency_ctrl_calc_us;
extern float g_latency_ctrl_to_motor_us;
extern float g_latency_total_us;
extern float g_latency_total_avg_us;
extern float g_latency_total_max_us;
extern float g_latency_total_min_us;
extern uint32_t g_latency_sample_count;
extern float g_latency_wifi_to_ctrl_us;
extern float g_latency_wifi_total_us;
extern float g_latency_wifi_avg_us;
extern float g_latency_wifi_max_us;
extern float g_latency_wifi_min_us;
extern uint32_t g_latency_wifi_sample_count;

// 电机句柄
extern can_motor_handle_t g_motor_left;
extern can_motor_handle_t g_motor_right;
extern can_motor_handle_t g_motor_left_hip;
extern can_motor_handle_t g_motor_left_knee;
extern can_motor_handle_t g_motor_right_hip;
extern can_motor_handle_t g_motor_right_knee;

// 失控标志
extern int g_uncontrolable;
extern bool g_uncontrolable_check_enabled;

// 停止标志
extern int g_move_stop_flag;

// 零点自适应
extern float g_angle_zeropoint;
extern float g_zp_speed_threshold;
extern float g_zp_pitch_for_ctrl;
extern float g_zp_angle_error;
extern float g_zp_pid_raw;
extern float g_zp_pid_filtered;
extern bool g_zp_active;

// 关节速度滤波
extern bool g_joint_speed_filter_enable;
extern int g_joint_speed_filter_mode;
extern float g_joint_speed_slew_rate;
extern slewrate_filter_t g_sr_joint_lh;
extern slewrate_filter_t g_sr_joint_lk;
extern slewrate_filter_t g_sr_joint_rh;
extern slewrate_filter_t g_sr_joint_rk;
extern int g_joint_median_window;
extern median_filter_t g_mf_joint_lh;
extern median_filter_t g_mf_joint_lk;
extern median_filter_t g_mf_joint_rh;
extern median_filter_t g_mf_joint_rk;
extern float g_joint_lh_spd_filtered;
extern float g_joint_lk_spd_filtered;
extern float g_joint_rh_spd_filtered;
extern float g_joint_rk_spd_filtered;

// VMC
extern bool g_vmc_enabled;
extern vmc_params_t g_vmc_params;
extern float g_vmc_target_vx;
extern float g_vmc_target_y;
extern vmc_dual_output_t g_vmc_dual_output;
extern bool g_vmc_input_valid;

// 观测器
extern bool g_observer_enabled;
extern float g_obsv_v_encoder;
extern float g_obsv_v_filter;
extern float g_obsv_x_filter;


void compute_balance_output(float dt) {
    // 读取共享数据
    shared_imu_data_t imu;
    shared_remote_data_t remote;
    shared_wheel_state_t wheel;
    
    xSemaphoreTake(g_imu_mutex, portMAX_DELAY);
    memcpy(&imu, &g_imu_data, sizeof(imu));
    xSemaphoreGive(g_imu_mutex);
    
    // 保存实际使用的 IMU 数据时间戳 (用于精确延迟测量)
    g_used_imu_time_us = imu.read_time_us;
    
    xSemaphoreTake(g_remote_mutex, portMAX_DELAY);
    memcpy(&remote, &g_remote_data, sizeof(remote));
    xSemaphoreGive(g_remote_mutex);
    
    // 保存实际使用的 WiFi 数据时间戳 (用于精确延迟测量)
    g_used_wifi_time_us = remote.receive_time_us;
    
    xSemaphoreTake(g_wheel_state_mutex, portMAX_DELAY);
    memcpy(&wheel, &g_wheel_state, sizeof(wheel));
    xSemaphoreGive(g_wheel_state_mutex);
    
    // ======== 检查紧急停止 (可通过开关禁用) ========
    if (g_uncontrolable_check_enabled) {
        if (fabsf(imu.pitch) > EMERGENCY_ANGLE_DEG) {
            if (g_uncontrolable == 0) {
                ESP_LOGW(TAG, "Pitch angle too large: %.1f deg", imu.pitch);
            }
            g_uncontrolable = 1;
        }
        
        // 恢复检测 (参考 shibo_wheel_leg)
        if (g_uncontrolable != 0) {
            if (fabsf(imu.pitch) < 10.0f) {
                g_uncontrolable++;
            } else {
                // 角度仍然过大，重置计数器
                g_uncontrolable = 1;
            }
            if (g_uncontrolable > 100) {  // 约 0.5 秒延时
                g_uncontrolable = 0;
                ESP_LOGI(TAG, "Recovered from uncontrolable state");
            }
        }
    } else {
        // 失控检测禁用时，始终保持可控状态
        g_uncontrolable = 0;
    }
    
    // ======== 计算 LQR 状态量 ========
    // 位置: 使用电机位置 (度 -> 弧度)
    // 注: 电机返回的是累积角度，不会有过零问题
    float left_pos_rad = wheel.left_position * 0.0174533f;   // deg to rad
    float right_pos_rad = wheel.right_position * 0.0174533f;
    
    // 速度: rpm -> rad/s
    float left_vel_rad = wheel.left_speed * 0.10472f;   // rpm to rad/s
    float right_vel_rad = wheel.right_speed * 0.10472f;
    
    // ======== 计算轮子加速度 (用于离地检测) ========
    // 加速度 = (当前速度 - 上次速度) / dt
    g_left_wheel_speed_rad = left_vel_rad;
    g_right_wheel_speed_rad = right_vel_rad;
    g_left_wheel_accel = (left_vel_rad - g_prev_left_wheel_speed) / dt;
    g_right_wheel_accel = (right_vel_rad - g_prev_right_wheel_speed) / dt;
    g_prev_left_wheel_speed = left_vel_rad;
    g_prev_right_wheel_speed = right_vel_rad;
    
    // ======== 轮子离地检测 (参考 shibo_wheel_leg) ========
    // 任一轮: 速度 > 阈值 且 加速度 > 阈值 → 判定离地
    // 使用计数器去抖: 连续 N 帧满足才判定，避免瞬间误触发
    {
        float speed_th = g_lqr_ctrl.params.wheel_off_ground_speed_threshold;
        float accel_th = g_lqr_ctrl.params.wheel_off_ground_accel_threshold;
        // 速度 AND 加速度都超阈值才算单轮离地 (避免正常行驶误触发)
        bool left_off = (fabsf(left_vel_rad) > speed_th) && (fabsf(g_left_wheel_accel) > accel_th);
        bool right_off = (fabsf(right_vel_rad) > speed_th) && (fabsf(g_right_wheel_accel) > accel_th);
        bool off_ground_raw = left_off || right_off;
        
        // 计数器去抖: 进入离地需连续 N 帧，退出离地需连续 M 帧
        if (off_ground_raw) {
            if (g_off_ground_counter < OFF_GROUND_ENTER_COUNT) {
                g_off_ground_counter++;
            }
        } else {
            if (g_off_ground_counter > -OFF_GROUND_EXIT_COUNT) {
                g_off_ground_counter--;
            }
        }
        
        // 状态切换
        bool off_ground_now;
        if (!g_wheel_off_ground) {
            // 当前在地面 → 需要连续 ENTER_COUNT 帧才进入离地
            off_ground_now = (g_off_ground_counter >= OFF_GROUND_ENTER_COUNT);
        } else {
            // 当前离地 → 需要连续 EXIT_COUNT 帧才恢复着地
            off_ground_now = (g_off_ground_counter > -OFF_GROUND_EXIT_COUNT);
        }
        
        if (off_ground_now && !g_wheel_off_ground) {
            ESP_LOGW(TAG, "WHEEL OFF GROUND! L: spd=%.1f acc=%.1f  R: spd=%.1f acc=%.1f",
                     left_vel_rad, g_left_wheel_accel, right_vel_rad, g_right_wheel_accel);
        } else if (!off_ground_now && g_wheel_off_ground) {
            ESP_LOGI(TAG, "Wheel back on ground (counter=%d)", g_off_ground_counter);
        }
        g_wheel_off_ground = off_ground_now;
    }
    
    // 累积位移和速度 (考虑轮子半径，转换为实际距离/速度)
    // 位移公式: s = r * θ (θ为弧度)
    // 速度公式: v = r * ω (ω为角速度 rad/s)
    // 轮子半径: WHEEL_RADIUS_M = 0.03m (直径60mm)
    // 电机方向: 右轮顺时针为负机器人前进，左轮逆时针为负机器人前进
    // 所以两轮都为负时，机器人前进；取负后位移为正表示前进
    g_lqr_distance = (-0.5f) * (left_pos_rad + right_pos_rad) * WHEEL_RADIUS_M;  // 单位: m
    g_lqr_speed = (-0.5f) * (left_vel_rad + right_vel_rad) * WHEEL_RADIUS_M;     // 单位: m/s
    
    // ======== 速度/位移观测器 ========
    // 观测器在独立任务 task_observer 中运行, 这里只读取结果
    // 可选: 用滤波后的速度/位移替代原始值
    if (g_observer_enabled) {
        g_lqr_speed = g_obsv_v_filter;
        g_lqr_distance = g_obsv_x_filter;
    }
    
    // ======== YAW 过零处理 ========
    // 处理 IMU YAW 从 +180° 跳到 -180° 的情况
    if (g_yaw_first_run) {
        g_yaw_angle_last = imu.yaw;
        g_yaw_first_run = false;
    }
    lqr_yaw_angle_addup(imu.yaw, g_yaw_angle_last, &g_yaw_angle_total);
    g_yaw_angle_last = imu.yaw;
    
    // ======== 准备 LQR 输入 ========
    // 计算真实倾斜角 theta3: 当腿部使能且补偿开启时，考虑腿部角度补偿
    // theta3 = pitch(IMU) + body_angle(腿相对机身) + 90
    // body_angle 定义：腿与机身垂直向下方向的夹角，-90°=垂直向下
    // 当 body_angle = -90° 时，theta3 = pitch + 0 = pitch (无补偿)
    // 当腿向前倾斜 body_angle > -90° 时，theta3 > pitch
    float pitch_for_control = imu.pitch;
    if (g_leg_control_enabled && g_pitch_leg_comp_enabled) {
        // 使用 FK 从电机编码器缓存获取实际的 body_angle (无阻塞)
        leg_state_t left_leg_state, right_leg_state;
        float avg_body_angle = -90.0f;  // 默认垂直
        
        if (leg_ctrl_get_state_cached(true, &left_leg_state) == ESP_OK && 
            leg_ctrl_get_state_cached(false, &right_leg_state) == ESP_OK &&
            left_leg_state.valid && right_leg_state.valid) {
            // 使用左右腿实际角度的平均值
            float left_body_angle = left_leg_state.workspace.body_angle;
            float right_body_angle = right_leg_state.workspace.body_angle;
            
            // 安全检查: body_angle 应该在合理范围内 (-160° ~ -20°)
            // 如果不在范围内，说明电机数据异常，使用默认值
            if (left_body_angle >= LEG_BODY_ANGLE_MIN && left_body_angle <= LEG_BODY_ANGLE_MAX &&
                right_body_angle >= LEG_BODY_ANGLE_MIN && right_body_angle <= LEG_BODY_ANGLE_MAX) {
                avg_body_angle = (left_body_angle + right_body_angle) / 2.0f;
            } else {
                // 数据异常，使用默认值并打印警告
                static uint32_t last_warn_time = 0;
                uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
                if (now - last_warn_time > 2000) {  // 每 2 秒打印一次
                    ESP_LOGW(TAG, "Invalid body_angle from FK: L=%.1f, R=%.1f, using default -90",
                             left_body_angle, right_body_angle);
                    last_warn_time = now;
                }
            }
        }
        
        pitch_for_control = imu.pitch + avg_body_angle + 90.0f;
    }
    
    lqr_input_t input = {
        .pitch = pitch_for_control,       // 经过腿部补偿的 pitch，用于平衡控制
        .raw_pitch = imu.pitch,           // IMU 原始 pitch，用于紧急停止判断
        .pitch_rate = imu.pitch_rate,
        .roll = imu.roll,
        .roll_rate = imu.roll_rate,
        .yaw = imu.yaw,
        .yaw_rate = imu.yaw_rate,
        .left_wheel_pos = left_pos_rad,
        .right_wheel_pos = right_pos_rad,
        .left_wheel_vel = left_vel_rad,
        .right_wheel_vel = right_vel_rad,
        .lqr_distance = g_lqr_distance,         // 累积位移 (已考虑电机方向)
        .lqr_speed = g_lqr_speed,               // 当前速度 (已考虑电机方向)
        .yaw_total = g_yaw_angle_total,         // YAW 累积角度 (用于方向保持)
        .target_speed = remote.joy_y * g_joy_speed_scale,   // joy_y (-100~100) -> target speed
        .target_yaw_rate = -remote.joy_x * g_joy_yaw_scale,  // joy_x -> target yaw rate
        .dt = dt,
    };
    
    // Roll 控制附加条件: pitch 小于 15° 且不在跳跃中
    bool roll_active = g_roll_control_enabled
                    && (fabsf(pitch_for_control) < 15.0f)
                    && !jump_is_active()
                    && !standup_is_active();
    
    // ======== 运动细节优化 (参考 shibo_wheel_leg) ========
    
    // 有前后方向运动指令时，重置位移零点 (仅在刚开始移动时)
    static bool was_moving = false;
    bool is_moving = (remote.joy_y != 0);
    bool is_turning = (remote.joy_x != 0);
    if (is_moving && !was_moving) {
        // joy_y 从 0 变为非 0，开始移动
        g_distance_zeropoint = g_lqr_distance;
        lqr_set_distance_zeropoint(&g_lqr_ctrl, g_distance_zeropoint);
        triple_pid_set_distance_zeropoint(&g_triple_pid_ctrl, g_distance_zeropoint);
        // 保存 yaw 状态 (lqr_reset 会清零)
        float saved_yaw_target = g_lqr_ctrl.yaw_angle_target;
        bool saved_yaw_holding = g_lqr_ctrl.yaw_holding;
        lqr_reset(&g_lqr_ctrl);  // 仅重置一次
        // 恢复 yaw 状态 (移动时不应该丢失方向保持)
        g_lqr_ctrl.yaw_angle_target = saved_yaw_target;
        g_lqr_ctrl.yaw_holding = saved_yaw_holding;
    }
    if (is_moving || is_turning) {
        // 移动或转向过程中持续更新位移零点 (防止位移环干扰)
        g_distance_zeropoint = g_lqr_distance;
        lqr_set_distance_zeropoint(&g_lqr_ctrl, g_distance_zeropoint);
        triple_pid_set_distance_zeropoint(&g_triple_pid_ctrl, g_distance_zeropoint);
    }
    was_moving = is_moving;
    
    // 运动指令复零时的原地停车处理
    if ((remote.joy_x_last != 0 && remote.joy_x == 0) ||
        (remote.joy_y_last != 0 && remote.joy_y == 0)) {
        g_move_stop_flag = 1;
        // 立即清零速度指令滤波器: 避免 LPF 历史值让速度环误以为还在行进, 导致制动延迟
        lpf_reset(&g_triple_pid_ctrl.lpf_joyy);
        lpf_reset(&g_lqr_ctrl.lpf_joyy);
        lpf_reset(&g_dual_pid_ctrl.lpf_joyy);
        // 清零速度环积分: 行进中积累的正积分会持续推动前倾, 松杆后必须清除以允许完整 P 项制动
        pid_reset(&g_triple_pid_ctrl.pid_speed);
    }

    //停车逻辑: 当检测到停止指令且速度较低时，重置位移零点，确保后续移动从当前位移开始计算
    if ((g_move_stop_flag == 1) && (fabsf(g_lqr_speed) < 10000.0f)) {
        g_distance_zeropoint = g_lqr_distance;
        lqr_set_distance_zeropoint(&g_lqr_ctrl, g_distance_zeropoint);
        triple_pid_set_distance_zeropoint(&g_triple_pid_ctrl, g_distance_zeropoint);
        g_move_stop_flag = 0;
    }
    
    // 被快速推动时的原地停车处理 (仅在没有停车指令时, 避免松杆后高速无法制动)
    if (g_move_stop_flag == 0 && fabsf(g_lqr_speed) > 1000.8f) {
        g_distance_zeropoint = g_lqr_distance;
        lqr_set_distance_zeropoint(&g_lqr_ctrl, g_distance_zeropoint);
        triple_pid_set_distance_zeropoint(&g_triple_pid_ctrl, g_distance_zeropoint);
    }
    
    // ======== YAW 使能边沿检测 (所有控制模式通用) ========
    // 检测 OFF→ON 瞬间: 重置目标角到当前角, 清 PID, 使 error 从 0 起步
    {
        static bool prev_yaw_enabled = false;  // 匹配 g_yaw_control_enabled 的启动默认值
        bool yaw_now = g_yaw_control_enabled && (g_uncontrolable == 0);

        if (yaw_now && !prev_yaw_enabled) {
            // OFF→ON: 同步目标到当前角度, 清零 PID, error 立刻为 0
            g_lqr_ctrl.yaw_angle_target = g_yaw_angle_total;
            g_lqr_ctrl.yaw_holding = false;
            pid_reset(&g_lqr_ctrl.pid_yaw_angle);
            pid_reset(&g_lqr_ctrl.pid_yaw_gyro);
            g_yaw_output = 0.0f;
            ESP_LOGW(TAG, "YAW ON edge: target=%.1f total=%.1f rate=%.1f",
                     g_lqr_ctrl.yaw_angle_target, g_yaw_angle_total, imu.yaw_rate);
        }
        if (!yaw_now && prev_yaw_enabled) {
            g_yaw_output = 0.0f;
            ESP_LOGW(TAG, "YAW OFF edge: total=%.1f", g_yaw_angle_total);
        }
        prev_yaw_enabled = yaw_now;
    }

    // ======== 根据控制模式选择算法 ========
    lqr_output_t output = {0};
    
    if (g_control_mode == CTRL_MODE_DUAL_PID && g_dual_pid_initialized) {
        // ======== 双环 PID 控制模式 ========
        // 直立环 (外环): pitch → target_speed
        // 速度环 (内环): target_speed - actual_speed → torque
        
        // 注意: 电机安装方向导致正转时机器人向后移动，需要取负号
        // 与 LQR 模式的 g_lqr_speed 计算保持一致
        float wheel_speed_avg = -(left_vel_rad + right_vel_rad) / 2.0f;  // 平均轮速 (rad/s)
        if (g_wma_enabled) {
            wheel_speed_avg = wma_compute(&g_wheel_speed_wma, wheel_speed_avg);
        }
        
        esp_err_t ret = dual_pid_balance_loop(&g_dual_pid_ctrl, 
                                               pitch_for_control, imu.pitch_rate,
                                               wheel_speed_avg, input.target_speed,
                                               dt,
                                               &g_dual_pid_output);
        
        if (ret != ESP_OK || g_dual_pid_output.emergency) {
            // 失败或紧急停止
            output.left_wheel_torque = 0;
            output.right_wheel_torque = 0;
            g_last_lqr_u = 0;
        } else {
            // 双环 PID 输出 (左右轮相同)
            float torque = g_dual_pid_output.torque;
            output.lqr_u = torque;  // 用于波形显示兼容
            
            // YAW 控制
            if (g_yaw_control_enabled) {
                lqr_yaw_loop(&g_lqr_ctrl, &input, &output);
                g_yaw_output = output.yaw_control;
                output.left_wheel_torque = torque - g_yaw_output;
                output.right_wheel_torque = torque + g_yaw_output;
            } else if (g_diff_speed_enabled && remote.joy_x != 0) {
                // 差速转向: 开环差速, joy_x > 0 → 右转
                float diff = remote.joy_x / 100.0f * g_diff_speed_scale;
                output.left_wheel_torque = torque + diff;
                output.right_wheel_torque = torque - diff;
                g_yaw_output = diff;
            } else {
                output.left_wheel_torque = torque;
                output.right_wheel_torque = torque;
                g_yaw_output = 0.0f;
            }
            g_last_lqr_u = torque;
        }
        
        // 轮子离地保护: 重置位移零点 (Dual PID 无位移环，但保持零点同步)
        if (g_wheel_off_ground) {
            g_distance_zeropoint = g_lqr_distance;
            lqr_set_distance_zeropoint(&g_lqr_ctrl, g_distance_zeropoint);
            triple_pid_set_distance_zeropoint(&g_triple_pid_ctrl, g_distance_zeropoint);
        }
        
        // 填充兼容字段用于波形显示
        output.angle_control = g_dual_pid_output.angle_p_out;
        output.gyro_control = g_dual_pid_output.angle_d_out;
        output.speed_control = g_dual_pid_output.speed_p_out;
        output.filtered_target_speed = g_dual_pid_output.target_speed;
        
    } else if (g_control_mode == CTRL_MODE_SINGLE_PID && g_single_pid_initialized) {
        // ======== 单环 PID 控制模式 (输出速度，适合电机速度模式) ========
        // 直立环: pitch → target_speed (直接输出给电机速度模式)
        
        esp_err_t ret = single_pid_balance_loop(&g_single_pid_ctrl, 
                                                 pitch_for_control, imu.pitch_rate,
                                                 dt,
                                                 &g_single_pid_output);
        
        if (ret != ESP_OK || g_single_pid_output.emergency) {
            // 失败或紧急停止
            output.left_wheel_torque = 0;
            output.right_wheel_torque = 0;
            g_last_lqr_u = 0;
        } else {
            // 单环 PID 输出是目标速度 (rad/s)
            float target_speed = g_single_pid_output.target_speed;
            
            // 存入 output 结构 (注意这里是速度不是扭矩，需要后续特殊处理)
            output.lqr_u = target_speed;  // 用于波形显示兼容
            
            // YAW 控制
            // 单环模式下 YAW 也是速度差速
            if (g_yaw_control_enabled) {
                lqr_yaw_loop(&g_lqr_ctrl, &input, &output);
                g_yaw_output = output.yaw_control;
                output.left_wheel_torque = target_speed + g_yaw_output;
                output.right_wheel_torque = target_speed - g_yaw_output;
            } else {
                output.left_wheel_torque = target_speed;
                output.right_wheel_torque = target_speed;
                g_yaw_output = 0.0f;
            }
            g_last_lqr_u = target_speed;
        }
        
        // 轮子离地保护: 重置位移零点 (Single PID 无位移环，但保持零点同步)
        if (g_wheel_off_ground) {
            g_distance_zeropoint = g_lqr_distance;
            lqr_set_distance_zeropoint(&g_lqr_ctrl, g_distance_zeropoint);
            triple_pid_set_distance_zeropoint(&g_triple_pid_ctrl, g_distance_zeropoint);
        }
        
        // 填充兼容字段用于波形显示
        output.angle_control = g_single_pid_output.angle_p_out;
        output.gyro_control = g_single_pid_output.angle_d_out;
        output.speed_control = 0;  // 单环无速度环
        output.filtered_target_speed = g_single_pid_output.target_speed;
        
    } else if (g_control_mode == CTRL_MODE_CAR) {
        // ======== 普通小车模式 (无直立环, 趴下跑) ========
        // 直接把遥杆映射为左右轮速度, 差速转向
        // joy_y: 前进/后退速度  joy_x: 左右转向
        
        float speed_cmd = -remote.joy_y *0.4 / 100.0f * CAR_MODE_MAX_SPEED;  // -MAX ~ +MAX rpm
        float yaw_cmd = -remote.joy_x / 100.0f * CAR_MODE_YAW_GAIN;   // 差速转向 rpm
        
        float left_speed_rpm = speed_cmd + yaw_cmd;
        float right_speed_rpm = speed_cmd - yaw_cmd;
        
        // 限幅
        if (left_speed_rpm > CAR_MODE_MAX_SPEED) left_speed_rpm = CAR_MODE_MAX_SPEED;
        if (left_speed_rpm < -CAR_MODE_MAX_SPEED) left_speed_rpm = -CAR_MODE_MAX_SPEED;
        if (right_speed_rpm > CAR_MODE_MAX_SPEED) right_speed_rpm = CAR_MODE_MAX_SPEED;
        if (right_speed_rpm < -CAR_MODE_MAX_SPEED) right_speed_rpm = -CAR_MODE_MAX_SPEED;
        
        // 紧急停止检查: 失控状态下不输出速度
        if (g_uncontrolable != 0) {
            left_speed_rpm = 0;
            right_speed_rpm = 0;
        }
        
        // 存入 output (此模式是速度模式, 类似 SINGLE_PID)
        // left/right_wheel_torque 复用存储速度值 (rpm → rad/s, 后续 apply_motor_commands 会转换)
        float left_speed_rads = left_speed_rpm * 0.10472f;   // rpm → rad/s
        float right_speed_rads = right_speed_rpm * 0.10472f;
        output.left_wheel_torque = left_speed_rads;
        output.right_wheel_torque = right_speed_rads;
        output.lqr_u = (left_speed_rads + right_speed_rads) / 2.0f;
        g_last_lqr_u = output.lqr_u;
        
        // 填充兼容字段用于波形显示
        output.angle_control = 0;
        output.gyro_control = 0;
        output.speed_control = speed_cmd;
        output.filtered_target_speed = speed_cmd;
        
    } else if (g_control_mode == CTRL_MODE_TRIPLE_PID && g_triple_pid_initialized) {
        // ======== 三环 PID 控制模式 ========
        // 速度环(外): target_speed - wheel_speed → pitch_target
        // 角度环(中): pitch_target - pitch → wheel_speed_target
        // 轮速环(内): wheel_speed_target → torque(软件PID) 或 speed_cmd(电机速度模式)
        
        float wheel_speed_avg;
        if (g_tpid_use_observer_speed) {
            // 观测器速度 (m/s) → 轮角速度 (rad/s)
            wheel_speed_avg = g_obsv_v_filter / WHEEL_RADIUS_M;
        } else {
            wheel_speed_avg = -(left_vel_rad + right_vel_rad) / 2.0f;
            wheel_speed_avg = wma_compute(&g_wheel_speed_wma, wheel_speed_avg);
        }
        
        // // 根据腿长线性插值角度环Kp: L0=0.06→Kp=1.3, L0=0.11→Kp=0.9
        // {
        //     float avg_L0 = (g_leg_left_target_length + g_leg_right_target_length) * 0.5f;
        //     if (avg_L0 < 0.06f) avg_L0 = 0.06f;
        //     if (avg_L0 > 0.11f) avg_L0 = 0.11f;
        //     float interp_angle_kp = 1.25f + (1.15f - 1.25f) * (avg_L0 - 0.06f) / (0.11f - 0.06f);
        //     triple_pid_set_angle_gains(&g_triple_pid_ctrl, interp_angle_kp,
        //                                g_triple_pid_ctrl.params.angle_ki,
        //                                g_triple_pid_ctrl.params.angle_kd);
        // }
        
        // // 根据腿长线性插值角速度环Kp: L0=0.06→Kp=0.11, L0=0.11→Kp=0.09
        // {
        //     float avg_L0 = (g_leg_left_target_length + g_leg_right_target_length) * 0.5f;
        //     if (avg_L0 < 0.06f) avg_L0 = 0.06f;
        //     if (avg_L0 > 0.11f) avg_L0 = 0.11f;
        //     float interp_gyro_kp = 0.105f + (0.095f - 0.105f) * (avg_L0 - 0.06f) / (0.11f - 0.06f);
        //     triple_pid_set_gyro_gains(&g_triple_pid_ctrl, interp_gyro_kp,
        //                               g_triple_pid_ctrl.params.gyro_ki,
        //                               g_triple_pid_ctrl.params.gyro_kd);
        // }
        
        esp_err_t ret = triple_pid_balance_loop(&g_triple_pid_ctrl,
                                                 pitch_for_control, imu.pitch_rate,
                                                 wheel_speed_avg, input.target_speed,
                                                 g_lqr_distance,
                                                 dt,
                                                 &g_triple_pid_output);
        
        if (ret != ESP_OK || g_triple_pid_output.emergency) {
            output.left_wheel_torque = 0;
            output.right_wheel_torque = 0;
            g_last_lqr_u = 0;
        } else {
            // 三环 PID 输出 (torque 或 speed，取决于 wheel_mode)
            float ctrl_output = g_triple_pid_output.torque;
            output.lqr_u = ctrl_output;
            
            // YAW 控制
            if (g_yaw_control_enabled) {
                lqr_yaw_loop(&g_lqr_ctrl, &input, &output);
                g_yaw_output = output.yaw_control * g_tpid_yaw_scale;  // 缩放到三环PID量级
                output.left_wheel_torque = ctrl_output + g_yaw_output;
                output.right_wheel_torque = ctrl_output - g_yaw_output;
            } else if (g_diff_speed_enabled && remote.joy_x != 0) {
                float diff = remote.joy_x * g_diff_speed_scale/100.0f;  // 需要根据实际量级调整缩放
                output.left_wheel_torque = ctrl_output - diff;
                output.right_wheel_torque = ctrl_output + diff;
                g_yaw_output = diff;
                // 差速转向时冻结位移零点并清除积分，彻底消除位移环干扰
                g_distance_zeropoint = g_lqr_distance;
                triple_pid_set_distance_zeropoint(&g_triple_pid_ctrl, g_distance_zeropoint);
                g_triple_pid_ctrl.pid_distance.integral = 0.0f;
            } else {
                output.left_wheel_torque = ctrl_output;
                output.right_wheel_torque = ctrl_output;
                g_yaw_output = 0.0f;
            }
            g_last_lqr_u = ctrl_output;
        }
        
        // 轮子离地保护
        if (g_wheel_off_ground) {
            g_distance_zeropoint = g_lqr_distance;
            lqr_set_distance_zeropoint(&g_lqr_ctrl, g_distance_zeropoint);
            triple_pid_set_distance_zeropoint(&g_triple_pid_ctrl, g_distance_zeropoint);
        }
        
        // 填充兼容字段用于波形显示
        output.angle_control = g_triple_pid_output.angle_p_out;
        output.gyro_control = g_triple_pid_output.angle_d_out;
        output.speed_control = g_triple_pid_output.speed_p_out;
        output.filtered_target_speed = g_triple_pid_output.wheel_speed_target;
        
    } else if (g_control_mode == CTRL_MODE_FULL_LQR && g_full_lqr_initialized) {
        // ======== 完整 LQR 控制模式 (同时输出 T 和 Tp) ========
        // 状态向量: [theta, d_theta, x, v, phi, phi_rate]
        // 输出: T (轮子扭矩), Tp (腿部摆动扭矩)
        // K 增益根据腿长 L0 实时插值 (三次多项式)
        // 控制律: u = -K * x (标准 LQR, 左右腿完全相同)
        
        // 1. 获取腿部状态 (L0, body_angle, body_angle_rate)
        //    统一约定 (左右腿相同):
        //      theta   = 90° + alpha + pitch
        //      d_theta = pitch_rate + d_alpha
        //      phi     = -pitch
        //      phi_rate= -pitch_rate
        float left_L0 = g_vmc_dual_output.left.current_leg_length;
        float right_L0 = g_vmc_dual_output.right.current_leg_length;
        float avg_L0 = (left_L0 + right_L0) / 2.0f;
        
        // 如果 VMC 还没运行过, 从 FK 缓存获取腿长
        if (avg_L0 < 0.01f) {
            leg_state_t ls, rs;
            if (leg_ctrl_get_state_cached(true, &ls) == ESP_OK && ls.valid) {
                left_L0 = ls.workspace.leg_length;
            }
            if (leg_ctrl_get_state_cached(false, &rs) == ESP_OK && rs.valid) {
                right_L0 = rs.workspace.leg_length;
            }
            avg_L0 = (left_L0 + right_L0) / 2.0f;
            if (avg_L0 < 0.01f) avg_L0 = 0.09f; // 安全默认值
        }
        
        // ======================================================================
        // theta / d_theta 计算 (左右腿完全相同!)
        // ======================================================================
        // 统一约定:
        //   theta   = 90° + alpha + pitch  (alpha = body_angle)
        //   d_theta = pitch_rate + d_alpha
        //   phi     = -pitch
        //   phi_rate= -pitch_rate
        // 左右腿使用完全相同的公式, 不再区分符号.
        // ======================================================================
        
        float left_body_angle = g_vmc_dual_output.left.current_body_angle;
        float right_body_angle = g_vmc_dual_output.right.current_body_angle;
        if (fabsf(left_body_angle) < 0.01f && fabsf(right_body_angle) < 0.01f) {
            // VMC 未运行, 从 FK 缓存获取
            leg_state_t ls2, rs2;
            if (leg_ctrl_get_state_cached(true, &ls2) == ESP_OK && ls2.valid) {
                left_body_angle = ls2.workspace.body_angle;
            }
            if (leg_ctrl_get_state_cached(false, &rs2) == ESP_OK && rs2.valid) {
                right_body_angle = rs2.workspace.body_angle;
            }
        }
        
        float pitch_rate_rad = DEG2RAD(imu.pitch_rate);
        float left_dalpha_rad = DEG2RAD(g_vmc_dual_output.left.current_body_angle_rate);
        float right_dalpha_rad = DEG2RAD(g_vmc_dual_output.right.current_body_angle_rate);
        
        // 左腿 theta/d_theta (统一约定: theta = 90° + alpha + pitch)
        float theta_left_lqr = DEG2RAD(90.0f + left_body_angle + imu.pitch);
        float d_theta_left_lqr = pitch_rate_rad + left_dalpha_rad;
        
        // 右腿 theta/d_theta (与左腿完全相同的公式!)
        float theta_right_lqr = DEG2RAD(90.0f + right_body_angle + imu.pitch);
        float d_theta_right_lqr = pitch_rate_rad + right_dalpha_rad;

        // 防劈叉用的 theta: 纯几何角 (不含 pitch, 避免 2*pitch 耦合)
        float theta_left_rad = DEG2RAD(left_body_angle + 90.0f);
        float theta_right_rad = DEG2RAD(right_body_angle + 90.0f);
        
        // 2. 构造输入并计算左腿 LQR (统一约定: phi = -pitch, phi_rate = -pitch_rate)
        full_lqr_input_t flqr_input_left = {
            .theta = theta_left_lqr,
            .d_theta = d_theta_left_lqr,
            .L0 = left_L0,
            .x = g_lqr_distance,
            .v = g_lqr_speed,
            .pitch = -DEG2RAD(imu.pitch),         // phi = -pitch
            .pitch_rate = -pitch_rate_rad,          // phi_rate = -pitch_rate
            .yaw_total = DEG2RAD(g_yaw_angle_total),
            .yaw_rate = DEG2RAD(imu.yaw_rate),
            .theta_left = theta_left_rad,
            .theta_right = theta_right_rad,
            .v_set = input.target_speed,
            .x_set = g_distance_zeropoint,
            .turn_set = DEG2RAD(g_yaw_angle_total), // 方向保持: turn_set = current_yaw
            .theta_set = 0.0f,
            .off_ground = g_wheel_off_ground,
            .dt = dt,
            .is_left = true,   // 已废弃, 不影响计算
        };
        
        // 3. 构造输入并计算右腿 LQR (与左腿完全相同的公式!)
        full_lqr_input_t flqr_input_right = flqr_input_left;
        flqr_input_right.theta = theta_right_lqr;      // 右腿: 90° + alpha_right + pitch
        flqr_input_right.d_theta = d_theta_right_lqr;  // 右腿: pitch_rate + d_alpha_right
        flqr_input_right.L0 = right_L0;
        flqr_input_right.is_left = false;  // 已废弃, 不影响计算
        
        // 分别计算左右腿
        esp_err_t ret_l = full_lqr_compute(&g_full_lqr_ctrl, &flqr_input_left, &g_full_lqr_output_left);
        esp_err_t ret_r = full_lqr_compute(&g_full_lqr_ctrl, &flqr_input_right, &g_full_lqr_output_right);
        
        if (ret_l != ESP_OK || ret_r != ESP_OK) {
            output.left_wheel_torque = 0;
            output.right_wheel_torque = 0;
            g_last_lqr_u = 0;
            g_full_lqr_Tp_left = 0;
            g_full_lqr_Tp_right = 0;
        } else {
            // 4. 轮子扭矩输出
           
            float T_left = g_full_lqr_output_left.wheel_torque; 
            float T_right = g_full_lqr_output_right.wheel_torque;
            
            // 转向差速 (使用右腿的 turn_torque, 左右对称)
            float turn_T = g_full_lqr_output_right.turn_torque;
            
            // 转向差速
            if (g_yaw_control_enabled) {
                // 更新 turn_set 为用户目标 (摇杆积分)
                // 无操作时保持当前角度 (方向保持)
                if (fabsf((float)remote.joy_x) > 5.0f) {
                    // 有转向输入时, 持续改变 turn_set
                    flqr_input_left.turn_set += DEG2RAD(-remote.joy_x * g_joy_yaw_scale * dt);
                    flqr_input_right.turn_set = flqr_input_left.turn_set;
                }
                g_yaw_output = turn_T;
                T_left += g_yaw_output;
                T_right -= g_yaw_output;
            } else if (g_diff_speed_enabled && remote.joy_x != 0) {
                float diff = remote.joy_x / 100.0f * g_diff_speed_scale;
                T_left += diff;
                T_right -= diff;
                g_yaw_output = diff;
            } else {
                g_yaw_output = 0.0f;
            }
            
            // 转向差速后再次限幅, 确保最终轮扭矩不超过电机极限
            if (T_left > WHEEL_TORQUE_LIMIT) T_left = WHEEL_TORQUE_LIMIT;
            if (T_left < -WHEEL_TORQUE_LIMIT) T_left = -WHEEL_TORQUE_LIMIT;
            if (T_right > WHEEL_TORQUE_LIMIT) T_right = WHEEL_TORQUE_LIMIT;
            if (T_right < -WHEEL_TORQUE_LIMIT) T_right = -WHEEL_TORQUE_LIMIT;
            
            output.left_wheel_torque = -T_left;//都要取反因为仿真和现实中不一样
            output.right_wheel_torque = -T_right;//取反！
            output.lqr_u = (T_left + T_right) / 2.0f;
            g_last_lqr_u = output.lqr_u;
            
            // 5. Tp 输出 (保存到全局, 由 VMC 计算时注入)
            // 左右腿 LQR 使用完全相同的公式, Tp 符号约定也完全相同, 不再需要取反.
            g_full_lqr_Tp_left = g_full_lqr_output_left.leg_torque;
            g_full_lqr_Tp_right = g_full_lqr_output_right.leg_torque;
            g_full_lqr_wheel_T_left = T_left;
            g_full_lqr_wheel_T_right = T_right;
        }
        
        // 轮子离地保护
        if (g_wheel_off_ground) {
            g_distance_zeropoint = g_lqr_distance;
        }
        
        // #FLQR 数据流输出
        if (g_full_lqr_stream_enable) {
            printf("#FLQR,%.3f,%.3f,%.4f,%.4f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                   g_full_lqr_wheel_T_left,      // 左轮 T
                   g_full_lqr_wheel_T_right,      // 右轮 T
                   g_full_lqr_Tp_left,            // 左腿 Tp
                   g_full_lqr_Tp_right,           // 右腿 Tp
                   avg_L0,                         // 平均腿长
                   RAD2DEG(theta_right_lqr),       // theta_right (度, 右腿约定)
                   RAD2DEG(d_theta_right_lqr),     // d_theta_right (度/s)
                   g_lqr_distance,                 // x (m)
                   g_lqr_speed,                    // v (m/s)
                   imu.pitch,                      // pitch (度)
                   imu.pitch_rate,                 // pitch_rate (度/s)
                   g_full_lqr_output_right.split_comp);  // 防劈叉
        }
        
        // 填充兼容字段用于波形显示
        output.angle_control = g_full_lqr_output_right.state_contrib[0];  // theta 分量
        output.gyro_control = g_full_lqr_output_right.state_contrib[1];   // d_theta 分量
        output.speed_control = g_full_lqr_output_right.state_contrib[3];  // v 分量
        output.filtered_target_speed = input.target_speed;
        
        // ======== Roll 控制 + X-Offset (Full LQR 模式) ========
        if (g_leg_control_enabled && roll_active) {
            lqr_roll_output_t roll_output;
            esp_err_t roll_ret = lqr_roll_loop(&g_lqr_ctrl, &input, &roll_output);

            if (roll_ret == ESP_OK) {
                g_roll_output = roll_output.roll_control;
                g_roll_left_delta = roll_output.left_leg_delta;
                g_roll_right_delta = roll_output.right_leg_delta;
                g_roll_filtered = roll_output.filtered_roll;

                float new_left_length = g_leg_base_length + roll_output.left_leg_delta;
                float new_right_length = g_leg_base_length + roll_output.right_leg_delta;
                float left_angle = g_leg_base_angle;
                float right_angle = g_leg_base_angle;

                if (g_xoffset_enabled) {
                    g_xoffset_debug_speed = g_lqr_speed;
                    g_xoffset_value = pid_compute(&g_xoffset_pid, g_lqr_speed, 0.0f, dt);
                } else {
                    g_xoffset_value = 0.0f;
                    g_xoffset_debug_speed = 0.0f;
                }

                apply_xoffset_and_ik(&new_left_length, &left_angle,
                                      &new_right_length, &right_angle,
                                      g_xoffset_value);
            }
        } else {
            g_roll_output = 0.0f;
            g_roll_left_delta = 0.0f;
            g_roll_right_delta = 0.0f;
            g_roll_filtered = 0.0f;
            g_xoffset_value = 0.0f;
            g_xoffset_debug_speed = 0.0f;
        }
        
    } else {
        // ======== LQR 多环控制模式 (默认) ========
        
        // 自动模式下根据状态切换 simple/full 模式
        // 手动模式下保持用户设置的环路使能状态
        // 修改: go 只影响 YAW 环路，其他环路始终按 full 模式运行 (除非失控)
        if (!g_loop_manual_mode) {
            if (g_uncontrolable != 0) {
                // 失控时，切换到简单平衡模式 (仅角度+角速度环)
                lqr_set_simple_balance_mode(&g_lqr_ctrl);
            } else {
                // 正常控制，使用完整平衡模式 (go 不再影响此逻辑)
                lqr_set_full_balance_mode(&g_lqr_ctrl);
            }
        }
        // 手动模式: 环路使能状态由 g_loop_enable_mask 控制，已在 balance_test_set_loop_gain() 中设置
        
        // 统一调用 LQR 平衡循环
        esp_err_t ret = lqr_balance_loop(&g_lqr_ctrl, &input, &output);
        if (ret != ESP_OK) {
            // 限流打印：每秒最多打印一次
            static uint32_t last_fail_log = 0;
            uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
            if (now - last_fail_log > 1000) {
                ESP_LOGW(TAG, "LQR balance loop failed");
                last_fail_log = now;
            }
            output.left_wheel_torque = 0;
            output.right_wheel_torque = 0;
            g_last_lqr_u = 0;
        } else {
        // ======== 轮子离地保护 (参考 shibo_wheel_leg) ========
        // 离地时: 位移零点重置，只输出角度+角速度(不输出位移/速度分量)，积分清零
        if (g_wheel_off_ground) {
            g_distance_zeropoint = g_lqr_distance;
            lqr_set_distance_zeropoint(&g_lqr_ctrl, g_distance_zeropoint);
            triple_pid_set_distance_zeropoint(&g_triple_pid_ctrl, g_distance_zeropoint);
            // 只保留角度+角速度控制，去掉位移和速度分量
            output.lqr_u = output.angle_control + output.gyro_control;
            pid_reset(&g_lqr_ctrl.pid_lqr_u);
        }
        
        // ======== YAW 轴转向控制 ========
        {
            // 注: 通用边沿检测已在 compute_balance_output 入口处完成
            bool yaw_active = g_yaw_control_enabled && g_uncontrolable == 0;
            
            if (yaw_active) {
            // 使用 lqr_yaw_loop 计算 YAW 控制量
            lqr_yaw_loop(&g_lqr_ctrl, &input, &output);
            
            // 保存 YAW 输出用于调试
            g_yaw_output = output.yaw_control;
            
            // 合成输出 (参考 shibo_wheel_leg)
            // 平衡控制输出 + YAW 差速
            // 正yaw_output → 右轮加速 → 左转(CCW) → yaw_total增大
            float lqr_u = output.lqr_u;
            output.left_wheel_torque = lqr_u - g_yaw_output;
            output.right_wheel_torque = lqr_u + g_yaw_output;
        } else if (g_diff_speed_enabled && remote.joy_x != 0) {
            float lqr_u = output.lqr_u;
            float diff = remote.joy_x / 100.0f * g_diff_speed_scale;
            output.left_wheel_torque = lqr_u - diff;
            output.right_wheel_torque = lqr_u + diff;
            g_yaw_output = diff;
        } else {
            // 简单模式/YAW禁用时，直接使用 LQR 输出，无 YAW 控制
            float lqr_u = output.lqr_u;
            output.left_wheel_torque = lqr_u;
            output.right_wheel_torque = lqr_u;
            g_yaw_output = 0.0f;
        }
        }  // end YAW block
        g_last_lqr_u = output.lqr_u;  // 保存用于波形显示
        
        // ======== X-Offset 计算 (腿部速度自适应偏移) ========
        // 在 Roll 控制之前计算 x_offset，后面 Roll 控制中使用偏移后的角度
        // 输入: 当前轮速 (g_lqr_speed, 所有模式都计算)
        // 输出: g_xoffset_value (笛卡尔 x 方向偏移, 米)
        if (g_xoffset_enabled && g_leg_control_enabled) {
            g_xoffset_debug_speed = g_lqr_speed;
            // PID: setpoint=当前速度, measurement=0
            // 速度>0(前进) → error>0 → x_offset>0 (腿向后摆)
            g_xoffset_value = pid_compute(&g_xoffset_pid, g_lqr_speed, 0.0f, dt);
        } else {
            g_xoffset_value = 0.0f;
            g_xoffset_debug_speed = 0.0f;
            if (!g_xoffset_enabled) {
                pid_reset(&g_xoffset_pid);
            }
        }
        
        // ======== Roll 控制 (腿长调节) ========
        if (g_leg_control_enabled && roll_active) {
            lqr_roll_output_t roll_output;
            esp_err_t roll_ret = lqr_roll_loop(&g_lqr_ctrl, &input, &roll_output);

            if (roll_ret == ESP_OK) {
                g_roll_output = roll_output.roll_control;
                g_roll_left_delta = roll_output.left_leg_delta;
                g_roll_right_delta = roll_output.right_leg_delta;
                g_roll_filtered = roll_output.filtered_roll;

                float new_left_length = g_leg_base_length + roll_output.left_leg_delta;
                float new_right_length = g_leg_base_length + roll_output.right_leg_delta;
                float left_angle = g_leg_base_angle;
                float right_angle = g_leg_base_angle;

                apply_xoffset_and_ik(&new_left_length, &left_angle,
                                      &new_right_length, &right_angle,
                                      g_xoffset_value);
            }
        } else if (!jump_is_active() && !standup_is_active()) {
            // Roll 控制未启用时，使用基础腿长 (左右对称)
            // 跳跃/起身期间跳过: 腿目标由状态机接管
            float base_length = g_leg_base_length;
            float base_angle = g_leg_base_angle;

            // X-Offset: 即使 Roll 未启用，也可以应用 x_offset
            if (g_leg_control_enabled && fabsf(g_xoffset_value) > 0.0001f) {
                apply_xoffset_single(base_length, base_angle, g_xoffset_value);
            } else {
                g_leg_left_target_length = base_length;
                g_leg_right_target_length = base_length;
                g_leg_left_target_angle = base_angle;
                g_leg_right_target_angle = base_angle;
            }
            
            // 清零调试变量
            g_roll_output = 0.0f;
            g_roll_left_delta = 0.0f;
            g_roll_right_delta = 0.0f;
            g_roll_filtered = 0.0f;
        }
        }  // end of LQR balance_loop success
    }  // end of LQR mode
    
    // ======== 角度零点自动调整 (重心补偿, 所有模式通用) ========
    // 原理: 静止时如果角度持续偏离零点, 说明重心不在正上方,
    //       缓慢修正 angle_zeropoint 使平均角度误差趋向零.
    // 参数通过 Commander ID='I' (zeropoint_kp/ki/kd) 可调.
    {
        // 计算当前角度误差 (所有模式统一使用 pitch_for_control)
        float angle_err_for_auto = pitch_for_control - g_angle_zeropoint;
        
        float zp_raw = 0.0f, zp_filtered = 0.0f;
        float zp_delta = lqr_zeropoint_auto_adjust(&g_lqr_ctrl, angle_err_for_auto,
                                                     g_lqr_speed, g_zp_speed_threshold,
                                                     dt,
                                                     &zp_raw, &zp_filtered);
        
        // 保存零点自适应调试变量
        g_zp_pitch_for_ctrl = pitch_for_control;
        g_zp_angle_error = angle_err_for_auto;
        g_zp_pid_raw = zp_raw;
        g_zp_pid_filtered = zp_filtered;
        g_zp_active = (fabsf(g_lqr_speed) < g_zp_speed_threshold);  // 与函数内部判断条件一致
        
        if (fabsf(zp_delta) > 0.0f) {
            // 累加到全局角度零点, 并同步到所有控制器
            g_angle_zeropoint += zp_delta;
            lqr_set_angle_zeropoint(&g_lqr_ctrl, g_angle_zeropoint);
            if (g_dual_pid_initialized) {
                dual_pid_set_angle_zeropoint(&g_dual_pid_ctrl, g_angle_zeropoint);
            }
            if (g_single_pid_initialized) {
                single_pid_set_angle_zeropoint(&g_single_pid_ctrl, g_angle_zeropoint);
            }
        }
        
        // 保存到 output 用于波形显示 (I/J 通道)
        output.zeropoint_adjust_raw = zp_raw;
        output.zeropoint_adjust_filtered = zp_filtered;
    }
    
    // ======== X-Offset 计算 (非 LQR/FULL_LQR 模式: Dual PID / Single PID) ========
    // LQR 和 FULL_LQR 模式的 x_offset 已在上面计算, 这里处理其他模式
    if (g_control_mode != CTRL_MODE_LQR && g_control_mode != CTRL_MODE_FULL_LQR && g_xoffset_enabled && g_leg_control_enabled) {
        g_xoffset_debug_speed = g_lqr_speed;
        g_xoffset_value = pid_compute(&g_xoffset_pid, g_lqr_speed, 0.0f, dt);
    } else if (g_control_mode != CTRL_MODE_LQR && g_control_mode != CTRL_MODE_FULL_LQR) {
        g_xoffset_value = 0.0f;
        g_xoffset_debug_speed = 0.0f;
        if (!g_xoffset_enabled) {
            pid_reset(&g_xoffset_pid);
        }
    }
    
    // ======== Roll 控制 + X-Offset (非 LQR 模式通用: Dual PID / Single PID) ========
    // 注: 双环PID和单环PID模式下也可以使用 Roll 控制和 X-Offset
    bool is_non_lqr = (g_control_mode == CTRL_MODE_DUAL_PID || g_control_mode == CTRL_MODE_SINGLE_PID || g_control_mode == CTRL_MODE_CAR || g_control_mode == CTRL_MODE_TRIPLE_PID);
    if (is_non_lqr && g_leg_control_enabled && roll_active) {
        lqr_roll_output_t roll_output;
        esp_err_t roll_ret = lqr_roll_loop(&g_lqr_ctrl, &input, &roll_output);

        if (roll_ret == ESP_OK) {
            g_roll_output = roll_output.roll_control;
            g_roll_left_delta = roll_output.left_leg_delta;
            g_roll_right_delta = roll_output.right_leg_delta;
            g_roll_filtered = roll_output.filtered_roll;

            float new_left_length = g_leg_base_length + roll_output.left_leg_delta;
            float new_right_length = g_leg_base_length + roll_output.right_leg_delta;
            float left_angle = g_leg_base_angle;
            float right_angle = g_leg_base_angle;

            apply_xoffset_and_ik(&new_left_length, &left_angle,
                                  &new_right_length, &right_angle,
                                  g_xoffset_value);
        }
    } else if (is_non_lqr && g_leg_control_enabled && !roll_active) {
        apply_xoffset_single(g_leg_base_length, g_leg_base_angle, g_xoffset_value);
    }
    
    // ======== 关节电机速度滤波 (每帧更新, 不依赖 VMC) ========
    // 读取关节电机原始速度并滤波, 结果存入全局变量供 VMC 和 #JOINT 数据流使用
    // 支持两种模式: 0=中值滤波(Median), 1=限幅滤波(SlewRate)
    {
        bool lh_ok = (g_motor_left_hip != NULL);
        bool lk_ok = (g_motor_left_knee != NULL);
        bool rh_ok = (g_motor_right_hip != NULL);
        bool rk_ok = (g_motor_right_knee != NULL);
        
        float lh_raw = lh_ok ? can_motor_read_speed(g_motor_left_hip)   * 6.0f : 0.0f;
        float lk_raw = lk_ok ? can_motor_read_speed(g_motor_left_knee)  * 6.0f : 0.0f;
        float rh_raw = rh_ok ? can_motor_read_speed(g_motor_right_hip)  * 6.0f : 0.0f;
        float rk_raw = rk_ok ? can_motor_read_speed(g_motor_right_knee) * 6.0f : 0.0f;
        
        if (g_joint_speed_filter_enable) {
            if (g_joint_speed_filter_mode == 0) {
                // 中值滤波
                g_joint_lh_spd_filtered = median_compute(&g_mf_joint_lh, lh_raw);
                g_joint_lk_spd_filtered = median_compute(&g_mf_joint_lk, lk_raw);
                g_joint_rh_spd_filtered = median_compute(&g_mf_joint_rh, rh_raw);
                g_joint_rk_spd_filtered = median_compute(&g_mf_joint_rk, rk_raw);
            } else {
                // 限幅滤波
                g_joint_lh_spd_filtered = slewrate_compute_dt(&g_sr_joint_lh, lh_raw, dt);
                g_joint_lk_spd_filtered = slewrate_compute_dt(&g_sr_joint_lk, lk_raw, dt);
                g_joint_rh_spd_filtered = slewrate_compute_dt(&g_sr_joint_rh, rh_raw, dt);
                g_joint_rk_spd_filtered = slewrate_compute_dt(&g_sr_joint_rk, rk_raw, dt);
            }
        } else {
            g_joint_lh_spd_filtered = lh_raw;
            g_joint_lk_spd_filtered = lk_raw;
            g_joint_rh_spd_filtered = rh_raw;
            g_joint_rk_spd_filtered = rk_raw;
        }
    }
    
    // ======== VMC 力控计算 (在 Roll 控制之后) ========
    // VMC 模式下：
    //   1. 获取 Roll 控制的腿长输出 (如果启用)
    //   2. 计算当前状态和 VMC 扭矩
    // 注: VMC 的 target_y 使用 Roll 调节后的腿长
    if (g_vmc_enabled && g_leg_control_enabled) {
        // 从 Roll 控制获取左右腿目标高度 (腿长近似等于高度)
        // Roll 控制已将 g_leg_left_target_length 和 g_leg_right_target_length 更新
        // 这些值在 VMC 中用作 target_y
        vmc_compute_leg_state(&input);
    }

    // ======== 支持力估计 (任意模式, 仅需关节电机有效) ========
    compute_support_force();
    
    // ======== 输出波形数据 (用于 Qt 调参面板) ========
    output_plot_data(&input, &output);
    
    // ======== 输出 PID 调试信息 ========
    output_pid_debug(&input);
    
    // ======== 支持力离地检测 (F_L 阈値, 左右轮独立) ========
    {
        // 左腿
        if (g_support_force_left_FL < SFORCE_FL_THRESHOLD) {
            if (g_sforce_left_off_cnt < SFORCE_OFF_ENTER_CNT) g_sforce_left_off_cnt++;
        } else {
            if (g_sforce_left_off_cnt > -SFORCE_OFF_EXIT_CNT) g_sforce_left_off_cnt--;
        }
        bool left_off_new  = g_sforce_left_off ? (g_sforce_left_off_cnt > -SFORCE_OFF_EXIT_CNT)
                                                : (g_sforce_left_off_cnt >= SFORCE_OFF_ENTER_CNT);
        if (left_off_new && !g_sforce_left_off)
            ESP_LOGW(TAG, "SFORCE: Left wheel off ground (F_L=%.2f N)", g_support_force_left_FL);
        else if (!left_off_new && g_sforce_left_off)
            ESP_LOGI(TAG, "SFORCE: Left wheel on ground (F_L=%.2f N)", g_support_force_left_FL);
        g_sforce_left_off = left_off_new;

        // 右腿
        if (g_support_force_right_FL < SFORCE_FL_THRESHOLD) {
            if (g_sforce_right_off_cnt < SFORCE_OFF_ENTER_CNT) g_sforce_right_off_cnt++;
        } else {
            if (g_sforce_right_off_cnt > -SFORCE_OFF_EXIT_CNT) g_sforce_right_off_cnt--;
        }
        bool right_off_new = g_sforce_right_off ? (g_sforce_right_off_cnt > -SFORCE_OFF_EXIT_CNT)
                                                 : (g_sforce_right_off_cnt >= SFORCE_OFF_ENTER_CNT);
        if (right_off_new && !g_sforce_right_off)
            ESP_LOGW(TAG, "SFORCE: Right wheel off ground (F_L=%.2f N)", g_support_force_right_FL);
        else if (!right_off_new && g_sforce_right_off)
            ESP_LOGI(TAG, "SFORCE: Right wheel on ground (F_L=%.2f N)", g_support_force_right_FL);
        g_sforce_right_off = right_off_new;
    }

    // ======== 更新轮命令 ========
    xSemaphoreTake(g_wheel_cmd_mutex, portMAX_DELAY);
    if (g_wheel_off_ground || jump_wants_zero_wheel()) {
        // 离地 或 跳跃空中阶段: 清零输出，防止轮子空转
        g_wheel_cmd.left_torque = 0;
        g_wheel_cmd.right_torque = 0;
    } else {
        // 左右轮分别判断: F_L 小于阈値则该轮失能
        g_wheel_cmd.left_torque  = g_sforce_left_off  ? 0.0f : output.left_wheel_torque;
        g_wheel_cmd.right_torque = g_sforce_right_off ? 0.0f : output.right_wheel_torque;
    }
    // 单环 PID 模式、小车模式使用速度模式；三环 PID 的 WHEEL_SPEED 模式也使用速度模式
    g_wheel_cmd.use_speed_mode = (g_control_mode == CTRL_MODE_SINGLE_PID || g_control_mode == CTRL_MODE_CAR
                                  || (g_control_mode == CTRL_MODE_TRIPLE_PID && g_triple_pid_ctrl.params.wheel_mode == TRIPLE_PID_WHEEL_SPEED));
    // enabled 状态由 balance_test_enable/disable 控制
    xSemaphoreGive(g_wheel_cmd_mutex);
}

/**
 * @brief 发送电机命令
 */
void apply_motor_commands(void) {
    shared_wheel_cmd_t cmd;
    // 每条腿的轮电机是否已进入 idle (F_L 离地检测触发)
    static bool s_left_idled  = false;
    static bool s_right_idled = false;

    // BAL_TO_CAR_TILT 阶段: 绕过所有保护 (sforce/uncontrolable), 直接发速度命令
    // 此时机身正在后倾, sforce 和 uncontrolable 检测会误判, 必须强制输出
    if (g_bal_to_car_state == BAL_TO_CAR_TILT && g_state == BALANCE_TEST_RUNNING) {
        const float tilt_rpm = -(BAL_TO_CAR_TILT_SPEED_RPM + 2.0f);  // 负 = 向前, +2 死区补偿
        can_motor_set_speed(g_motor_left,  tilt_rpm);
        can_motor_set_speed(g_motor_right, tilt_rpm);
        s_left_idled  = false;  // 重置 idle 标志, 防止后续误判
        s_right_idled = false;
        return;
    }

    xSemaphoreTake(g_wheel_cmd_mutex, portMAX_DELAY);
    memcpy(&cmd, &g_wheel_cmd, sizeof(cmd));
    xSemaphoreGive(g_wheel_cmd_mutex);

    // ── 辅助 lambda: 对单个轮电机应用命令, 同时处理 F_L 离地 idle/re-enable ──
    // 使用宏简化左右对称逻辑 (避免重复代码)
#define APPLY_WHEEL_CMD(motor, speed_rpm, torque_val, sforce_off, idled_flag)   \
    do {                                                                          \
        if (sforce_off) {                                                         \
            /* F_L 离地: 首次发送 speed=0 / torque=0 制动, 然后进入 idle */      \
            if (!idled_flag) {                                                    \
                if (cmd.use_speed_mode) {                                         \
                    can_motor_set_speed((motor), 0.0f);                           \
                }                                                                 \
                can_motor_set_idle((motor));                                      \
                idled_flag = true;                                                \
            }                                                                     \
            /* 已 idle 时不再发命令, 避免覆盖 idle 状态 */                       \
        } else {                                                                  \
            if (idled_flag) {                                                     \
                /* 重新着地: 恢复闭环, 并重置位移零点防止累积误差 */            \
                can_motor_enter_closed_loop((motor));                             \
                g_distance_zeropoint = g_lqr_distance;                           \
                lqr_set_distance_zeropoint(&g_lqr_ctrl, g_distance_zeropoint);   \
                triple_pid_set_distance_zeropoint(&g_triple_pid_ctrl,            \
                                                  g_distance_zeropoint);         \
                idled_flag = false;                                               \
            }                                                                     \
            /* 正常发命令 */                                                      \
            if (cmd.use_speed_mode) {                                             \
                can_motor_set_speed((motor), (speed_rpm));                        \
            } else {                                                              \
                can_motor_set_torque((motor), (torque_val));                      \
            }                                                                     \
        }                                                                         \
    } while (0)

    if (g_state == BALANCE_TEST_EMERGENCY) {
        // E-stop: 强制发送扭矩 0
        can_motor_set_torque(g_motor_left, 0);
        can_motor_set_torque(g_motor_right, 0);
    } else if (cmd.enabled && g_state == BALANCE_TEST_RUNNING) {
        if (g_uncontrolable != 0) {
            // IMU 角度超限: 强制发送扭矩 0
            can_motor_set_torque(g_motor_left, 0);
            can_motor_set_torque(g_motor_right, 0);
        } else {
            // 计算速度模式下的 RPM (速度模式时 left_torque 实际存储 rad/s)
            float left_speed_rpm  = 0.0f;
            float right_speed_rpm = 0.0f;
            if (cmd.use_speed_mode) {
                // 转换: rad/s → rpm (rpm = rad/s * 60 / 2π ≈ rad/s * 9.5493)
                left_speed_rpm  = cmd.left_torque  * 9.5493f;
                right_speed_rpm = cmd.right_torque * 9.5493f;
                // 死区补偿: 非零指令时叠加 DEADZONE, 确保电机能动
                const float SPEED_DEADZONE_RPM = 2.0f;
                if      (left_speed_rpm  > 0.0f) left_speed_rpm  += SPEED_DEADZONE_RPM;
                else if (left_speed_rpm  < 0.0f) left_speed_rpm  -= SPEED_DEADZONE_RPM;
                if      (right_speed_rpm > 0.0f) right_speed_rpm += SPEED_DEADZONE_RPM;
                else if (right_speed_rpm < 0.0f) right_speed_rpm -= SPEED_DEADZONE_RPM;
            }
            // CAR 模式: 轮子始终着地, 不需要 sforce 检测, 直接绕过 idle 逻辑
            bool sforce_en = (g_control_mode != CTRL_MODE_CAR);
            // 左轮: F_L 离地 → idle; 着地 → 恢复闭环
            APPLY_WHEEL_CMD(g_motor_left,  left_speed_rpm,  cmd.left_torque,
                            sforce_en && g_sforce_left_off,  s_left_idled);
            // 右轮: F_L 离地 → idle; 着地 → 恢复闭环
            APPLY_WHEEL_CMD(g_motor_right, right_speed_rpm, cmd.right_torque,
                            sforce_en && g_sforce_right_off, s_right_idled);
        }
    } else {
        // 系统停止时重置 idle 标志 (下次启动时不会误判)
        s_left_idled  = false;
        s_right_idled = false;
    }

#undef APPLY_WHEEL_CMD
    // READY/IDLE 状态: 不发送任何轮电机命令, 不占用 CAN 总线, 允许手动测试
    
    // 记录电机命令发送时间
    g_motor_send_time_us = esp_timer_get_time();
    
    // 计算控制到电机的延迟
    if (g_ctrl_end_time_us > 0) {
        g_latency_ctrl_to_motor_us = (float)(g_motor_send_time_us - g_ctrl_end_time_us);
    }
    
    // 计算 IMU 总延迟 (使用实际使用的 IMU 数据时间戳 -> 电机发送)
    if (g_used_imu_time_us > 0) {
        float total = (float)(g_motor_send_time_us - g_used_imu_time_us);
        g_latency_total_us = total;
        
        // 更新统计数据
        g_latency_sample_count++;
        if (g_latency_sample_count == 1) {
            g_latency_total_avg_us = total;
            g_latency_total_min_us = total;
            g_latency_total_max_us = total;
        } else {
            // 指数移动平均 (alpha = 0.02 ~ 50次平均)
            g_latency_total_avg_us = g_latency_total_avg_us * 0.98f + total * 0.02f;
            if (total < g_latency_total_min_us) g_latency_total_min_us = total;
            if (total > g_latency_total_max_us) g_latency_total_max_us = total;
        }
    }
    
    // 计算 WiFi 遥控延迟 (WiFi接收 -> 电机发送)
    if (g_used_wifi_time_us > 0) {
        // WiFi → 控制延迟
        g_latency_wifi_to_ctrl_us = (float)(g_ctrl_start_time_us - g_used_wifi_time_us);
        
        // WiFi 总延迟
        float wifi_total = (float)(g_motor_send_time_us - g_used_wifi_time_us);
        g_latency_wifi_total_us = wifi_total;
        
        // 更新统计 (只在有新 WiFi 数据时更新，避免统计旧数据)
        static uint64_t last_wifi_time = 0;
        if (g_used_wifi_time_us != last_wifi_time && wifi_total < 1000000.0f) {  // 小于1秒才是有效数据
            last_wifi_time = g_used_wifi_time_us;
            g_latency_wifi_sample_count++;
            
            if (g_latency_wifi_sample_count == 1) {
                g_latency_wifi_avg_us = wifi_total;
                g_latency_wifi_min_us = wifi_total;
                g_latency_wifi_max_us = wifi_total;
            } else {
                g_latency_wifi_avg_us = g_latency_wifi_avg_us * 0.95f + wifi_total * 0.05f;
                if (wifi_total < g_latency_wifi_min_us) g_latency_wifi_min_us = wifi_total;
                if (wifi_total > g_latency_wifi_max_us) g_latency_wifi_max_us = wifi_total;
            }
        }
    }
}


