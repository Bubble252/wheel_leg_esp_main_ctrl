/**
 * @file balance_plot.c
 * @brief 波形输出 + PID 调试输出
 *
 * 从 balance_test.c 拆分出来。包含:
 * - output_plot_data(): #DATA 波形输出
 * - output_pid_debug(): PID 调试数据输出
 * - balance_test_set_plot/get_enabled: 波形控制
 */

#include "balance_plot.h"
#include "balance_test.h"
#include "balance_types.h"
#include "balance_vmc.h"
#include "balance_observer.h"
#include "lqr_balance.h"
#include "full_lqr.h"
#include "leg_kinematics.h"
#include "can_motor.h"

#include "esp_log.h"
#include <stdio.h>
#include <math.h>

static const char *TAG = "BAL_PLOT";

// ============================================================================
// extern 变量 (定义在 balance_test.c)
// ============================================================================

// 控制器
extern lqr_controller_t g_lqr_ctrl;
extern float g_lqr_distance;
extern float g_distance_zeropoint;
extern float g_angle_zeropoint;

// Dual PID
extern dual_pid_controller_t g_dual_pid_ctrl;
extern dual_pid_output_t g_dual_pid_output;

// Single PID
extern single_pid_output_t g_single_pid_output;

// Triple PID
extern triple_pid_controller_t g_triple_pid_ctrl;
extern triple_pid_output_t g_triple_pid_output;

// 控制模式
extern control_mode_t g_control_mode;

// 轮速/加速度
extern float g_left_wheel_speed_rad;
extern float g_right_wheel_speed_rad;
extern float g_left_wheel_accel;
extern float g_right_wheel_accel;

// Yaw
extern float g_yaw_angle_total;
extern float g_yaw_output;

// 关节速度
extern float g_joint_lh_spd_filtered;
extern float g_joint_lk_spd_filtered;
extern float g_joint_rh_spd_filtered;
extern float g_joint_rk_spd_filtered;

// Leg Sync
extern bool g_leg_sync_enabled;
extern float g_leg_sync_debug_diff;
extern float g_leg_sync_debug_correction;
extern float g_leg_sync_gain;
extern float g_leg_sync_max_correction;

// X-Offset
extern bool g_xoffset_enabled;
extern pid_controller_t g_xoffset_pid;
extern float g_xoffset_value;
extern float g_xoffset_limit;
extern float g_xoffset_debug_speed;

// 零点自适应
extern float g_zp_pitch_for_ctrl;
extern float g_zp_angle_error;
extern float g_zp_pid_raw;
extern float g_zp_pid_filtered;
extern bool g_zp_active;

// 数据流开关
extern bool g_joint_stream_enable;
extern bool g_mpow_stream_enable;
extern uint8_t g_mpow_volt_poll_idx;
extern uint8_t g_mpow_volt_poll_div;

// 电机句柄
extern can_motor_handle_t g_motor_left;
extern can_motor_handle_t g_motor_right;
extern can_motor_handle_t g_motor_left_hip;
extern can_motor_handle_t g_motor_left_knee;
extern can_motor_handle_t g_motor_right_hip;
extern can_motor_handle_t g_motor_right_knee;

// Full LQR
extern float g_full_lqr_Tp_left;
extern float g_full_lqr_Tp_right;

void balance_test_set_plot(bool enable) {
    g_plot_enabled = enable;
    g_plot_counter = 0;
    ESP_LOGI(TAG, "Plot output %s", enable ? "enabled" : "disabled");
}

/**
 * @brief 设置波形输出分频系数
 * @param divider 分频系数 (1-255), 每 N 次控制循环输出一次
 * @note 控制循环 200Hz, divider=10 时输出 20Hz
 */
void balance_test_set_plot_divider(uint8_t divider) {
    if (divider < 1) divider = 1;
    g_plot_divider = divider;
    ESP_LOGI(TAG, "Plot divider set to %d (%.1f Hz)", divider, 200.0f / divider);
}

/**
 * @brief 获取波形输出状态
 * @return true 已使能, false 已禁用
 */
bool balance_test_get_plot_enabled(void) {
    return g_plot_enabled;
}

/**
 * @brief 输出波形数据到串口
 * @note 格式: #DATA,<ID>,<Target>,<Control>
 */
void output_plot_data(const lqr_input_t *input, const lqr_output_t *output) {
    if (!g_plot_enabled) return;
    
    g_plot_counter++;
    if (g_plot_counter < g_plot_divider) return;
    g_plot_counter = 0;
    
    // 仅输出掩码中使能的通道
    if (PLOT_CH_ENABLED('A')) printf("#DATA,A,0.0,%.2f\n", input->pitch);
    if (PLOT_CH_ENABLED('B')) printf("#DATA,B,0.0,%.2f\n", input->pitch_rate);
    if (PLOT_CH_ENABLED('C')) printf("#DATA,C,%.2f,%.2f\n", g_distance_zeropoint, g_lqr_distance);
    if (PLOT_CH_ENABLED('D')) printf("#DATA,D,%.2f,%.2f\n", input->target_speed, input->lqr_speed);
    if (PLOT_CH_ENABLED('E')) printf("#DATA,E,%.2f,%.2f\n", g_lqr_ctrl.yaw_angle_target, g_yaw_angle_total);
    if (PLOT_CH_ENABLED('F')) printf("#DATA,F,%.2f,%.2f\n", input->target_yaw_rate, input->yaw_rate);
    if (PLOT_CH_ENABLED('G')) printf("#DATA,G,%.2f,%.2f\n", input->target_speed, output->filtered_target_speed);
    if (PLOT_CH_ENABLED('H')) printf("#DATA,H,0.0,%.2f\n", g_last_lqr_u);
    if (PLOT_CH_ENABLED('I')) printf("#DATA,I,%.3f,%.3f\n", g_zp_pitch_for_ctrl, g_angle_zeropoint);
    if (PLOT_CH_ENABLED('J')) printf("#DATA,J,%.4f,%.4f\n", g_zp_pid_raw, g_zp_pid_filtered);
    if (PLOT_CH_ENABLED('K')) printf("#DATA,K,0.0,%.2f\n", input->roll);
    if (PLOT_CH_ENABLED('L')) {
        float roll_filtered_display = lpf_compute_dt(&g_lqr_ctrl.lpf_roll, input->roll, input->dt);
        printf("#DATA,L,%.2f,%.2f\n", input->roll, roll_filtered_display);
    }
    if (PLOT_CH_ENABLED('M')) printf("#DATA,M,%.4f,%.4f\n", g_lqr_ctrl.params.speed_kp, g_lqr_ctrl.params.speed_kp_max);
    if (PLOT_CH_ENABLED('N')) printf("#DATA,N,%.2f,%.2f\n", input->pitch_rate, output->filtered_gyro);
    if (PLOT_CH_ENABLED('W')) printf("#DATA,W,%.3f,%.3f\n", input->lqr_speed, output->filtered_speed);
    if (PLOT_CH_ENABLED('O')) printf("#DATA,O,%.2f,%.2f\n", g_left_wheel_speed_rad, g_left_wheel_accel);
    if (PLOT_CH_ENABLED('P')) printf("#DATA,P,%.2f,%.2f\n", g_right_wheel_speed_rad, g_right_wheel_accel);
    if (PLOT_CH_ENABLED('Q')) printf("#DATA,Q,0.0,%.4f\n", output->angle_control);
    if (PLOT_CH_ENABLED('R')) printf("#DATA,R,0.0,%.4f\n", output->gyro_control);
    if (PLOT_CH_ENABLED('S')) {
        float dist_ctrl = (g_control_mode == CTRL_MODE_TRIPLE_PID) ? 
                           g_triple_pid_output.distance_control : output->distance_control;
        printf("#DATA,S,0.0,%.4f\n", dist_ctrl);
    }
    if (PLOT_CH_ENABLED('T')) printf("#DATA,T,0.0,%.4f\n", output->speed_control);
    if (PLOT_CH_ENABLED('U')) printf("#DATA,U,0.0,%.4f\n", g_yaw_output);
    if (PLOT_CH_ENABLED('V')) printf("#DATA,V,%.4f,%.4f\n", output->lqr_u_raw, output->lqr_u);
    if (PLOT_CH_ENABLED('X')) printf("#DATA,X,%.3f,%.3f\n", can_motor_read_current(g_motor_left), can_motor_read_current(g_motor_right));
    if (PLOT_CH_ENABLED('Y')) printf("#DATA,Y,%.2f,%.2f\n", g_lqr_ctrl.yaw_angle_target, g_yaw_angle_total);
    if (PLOT_CH_ENABLED('Y')) {
        printf("#YAW_DBG,out=%.3f,err=%.2f,hold=%d,rate=%.2f\n", 
               g_yaw_output, 
               g_yaw_angle_total - g_lqr_ctrl.yaw_angle_target,
               g_lqr_ctrl.yaw_holding ? 1 : 0,
               input->yaw_rate);
    }
    if (PLOT_CH_ENABLED('Z')) printf("#DATA,Z,%.3f,%.1f\n", g_zp_angle_error, g_zp_active ? 1.0f : 0.0f);
    
    // 关节电机数据流 (独立使能, 复用 plot 分频)
    // 每次输出时独立读取并滤波，不依赖 VMC 是否启用
    // 格式: #JOINT,LH_pos,LH_spd,LH_cur,LH_spd_f,LK_pos,LK_spd,LK_cur,LK_spd_f,RH_pos,RH_spd,RH_cur,RH_spd_f,RK_pos,RK_spd,RK_cur,RK_spd_f
    if (g_joint_stream_enable) {
        // 原始速度也乘以 6.0 转为 °/s, 与滤波后值单位一致, 方便 UI 波形对比
        float lh_spd = g_motor_left_hip   ? can_motor_read_speed(g_motor_left_hip)   * 6.0f : 0.0f;
        float lk_spd = g_motor_left_knee  ? can_motor_read_speed(g_motor_left_knee)  * 6.0f : 0.0f;
        float rh_spd = g_motor_right_hip   ? can_motor_read_speed(g_motor_right_hip)   * 6.0f : 0.0f;
        float rk_spd = g_motor_right_knee  ? can_motor_read_speed(g_motor_right_knee)  * 6.0f : 0.0f;
        
        // 使用全局滤波后的值 (由 compute_balance_output 每帧更新)
        printf("#JOINT,%.2f,%.1f,%.3f,%.1f,%.2f,%.1f,%.3f,%.1f,%.2f,%.1f,%.3f,%.1f,%.2f,%.1f,%.3f,%.1f\n",
               g_motor_left_hip  ? can_motor_read_position(g_motor_left_hip)  : 0.0f,
               lh_spd,
               g_motor_left_hip  ? can_motor_read_current(g_motor_left_hip)   : 0.0f,
               g_joint_lh_spd_filtered,
               g_motor_left_knee ? can_motor_read_position(g_motor_left_knee) : 0.0f,
               lk_spd,
               g_motor_left_knee ? can_motor_read_current(g_motor_left_knee)  : 0.0f,
               g_joint_lk_spd_filtered,
               g_motor_right_hip  ? can_motor_read_position(g_motor_right_hip)  : 0.0f,
               rh_spd,
               g_motor_right_hip  ? can_motor_read_current(g_motor_right_hip)   : 0.0f,
               g_joint_rh_spd_filtered,
               g_motor_right_knee ? can_motor_read_position(g_motor_right_knee) : 0.0f,
               rk_spd,
               g_motor_right_knee ? can_motor_read_current(g_motor_right_knee)  : 0.0f,
               g_joint_rk_spd_filtered);
    }
    
    // 电机功率数据流 (独立使能, 复用 plot 分频)
    // 格式: #MPOW,LH_cur,LK_cur,LW_cur,RH_cur,RK_cur,RW_cur,LH_vol,LK_vol,LW_vol,RH_vol,RK_vol,RW_vol
    if (g_mpow_stream_enable) {
        // --- 低频电压轮询: 每 25 次 plot 周期请求 1 个电机的电压 (约 2Hz, 6个电机一轮 3 秒) ---
        g_mpow_volt_poll_div++;
        if (g_mpow_volt_poll_div >= 25) {
            g_mpow_volt_poll_div = 0;
            can_motor_handle_t motors[6] = {
                g_motor_left_hip, g_motor_left_knee, g_motor_left,
                g_motor_right_hip, g_motor_right_knee, g_motor_right
            };
            if (motors[g_mpow_volt_poll_idx]) {
                can_motor_request_voltage(motors[g_mpow_volt_poll_idx]);
            }
            g_mpow_volt_poll_idx = (g_mpow_volt_poll_idx + 1) % 6;
        }

        float lh_cur = g_motor_left_hip   ? can_motor_read_current(g_motor_left_hip)   : 0.0f;
        float lk_cur = g_motor_left_knee  ? can_motor_read_current(g_motor_left_knee)  : 0.0f;
        float lw_cur = g_motor_left       ? can_motor_read_current(g_motor_left)       : 0.0f;
        float rh_cur = g_motor_right_hip  ? can_motor_read_current(g_motor_right_hip)  : 0.0f;
        float rk_cur = g_motor_right_knee ? can_motor_read_current(g_motor_right_knee) : 0.0f;
        float rw_cur = g_motor_right      ? can_motor_read_current(g_motor_right)      : 0.0f;
        float lh_vol = g_motor_left_hip   ? can_motor_read_voltage(g_motor_left_hip)   : 0.0f;
        float lk_vol = g_motor_left_knee  ? can_motor_read_voltage(g_motor_left_knee)  : 0.0f;
        float lw_vol = g_motor_left       ? can_motor_read_voltage(g_motor_left)       : 0.0f;
        float rh_vol = g_motor_right_hip  ? can_motor_read_voltage(g_motor_right_hip)  : 0.0f;
        float rk_vol = g_motor_right_knee ? can_motor_read_voltage(g_motor_right_knee) : 0.0f;
        float rw_vol = g_motor_right      ? can_motor_read_voltage(g_motor_right)      : 0.0f;
        printf("#MPOW,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
               lh_cur, lk_cur, lw_cur, rh_cur, rk_cur, rw_cur,
               lh_vol, lk_vol, lw_vol, rh_vol, rk_vol, rw_vol);
    }
    
    // 观测器数据流 (独立使能, 复用 plot 分频)
    // 格式: #OBSV,v_raw,v_encoder,v_filter,x_filter,a_imu
    if (g_obsv_stream_enable) {
        printf("#OBSV,%.4f,%.4f,%.4f,%.4f,%.4f\n",
               g_obsv_wheel_v_raw,     // 原始轮速 (无补偿)
               g_obsv_v_encoder,       // 运动学补偿后的编码器速度
               g_obsv_v_filter,        // 卡尔曼滤波后的速度
               g_obsv_x_filter,        // 滤波积分位移
               g_obsv_a_imu);          // IMU 前进方向加速度
    }

    // 支持力数据流 (独立使能, 复用 plot 分频)
    // 格式: #SFORCE,L_FL(N),L_Fa(Nm),R_FL(N),R_Fa(Nm)
    if (g_sforce_stream_enable) {
        printf("#SFORCE,%.2f,%.3f,%.2f,%.3f\n",
               g_support_force_left_FL, g_support_force_left_Fa,
               g_support_force_right_FL, g_support_force_right_Fa);
    }
}

/**
 * @brief 输出 PID 调试信息 (实时打印)
 */
void output_pid_debug(const lqr_input_t *input) {
    if (!g_pid_debug_enabled) return;
    
    g_pid_debug_counter++;
    if (g_pid_debug_counter < g_pid_debug_divider) return;
    g_pid_debug_counter = 0;
    
    float pitch = input->pitch;
    float pitch_rate = input->pitch_rate;
    float wheel_speed = input->lqr_speed;
    
    if (g_control_mode == CTRL_MODE_DUAL_PID) {
        // 双环 PID 调试输出
        if (g_dual_pid_ctrl.params.loop_order == DUAL_PID_SPEED_FIRST) {
            // 速度优先模式: 速度环(外)→角度环(内)
            printf("[DPID-SF] pitch=%.2f° spd=%.2f | "
                   "Speed(外): err=%.2f P=%.2f I=%.3f D=%.3f → tgt_pitch=%.2f° | "
                   "Angle(内): err=%.2f P=%.2f I=%.3f D=%.3f → torque=%.3f\n",
                   pitch, wheel_speed,
                   g_dual_pid_output.speed_error,
                   g_dual_pid_output.speed_p_out,
                   g_dual_pid_output.speed_i_out,
                   g_dual_pid_output.speed_d_out,
                   g_dual_pid_output.target_speed,
                   g_dual_pid_output.angle_error,
                   g_dual_pid_output.angle_p_out,
                   g_dual_pid_output.angle_i_out,
                   g_dual_pid_output.angle_d_out,
                   g_dual_pid_output.torque);
        } else {
            // 角度优先模式: 角度环(外)→速度环(内)
            printf("[DPID-AF] pitch=%.2f° err=%.2f° rate=%.1f°/s | "
                   "Angle(外): P=%.2f I=%.3f D=%.3f → tgt_spd=%.2f | "
                   "Speed(内): err=%.2f P=%.2f I=%.3f D=%.3f → torque=%.3f\n",
                   pitch,
                   g_dual_pid_output.angle_error,
                   pitch_rate,
                   g_dual_pid_output.angle_p_out,
                   g_dual_pid_output.angle_i_out,
                   g_dual_pid_output.angle_d_out,
                   g_dual_pid_output.target_speed,
                   g_dual_pid_output.speed_error,
                   g_dual_pid_output.speed_p_out,
                   g_dual_pid_output.speed_i_out,
                   g_dual_pid_output.speed_d_out,
                   g_dual_pid_output.torque);
        }
               
    } else if (g_control_mode == CTRL_MODE_SINGLE_PID) {
        // 单环 PID 调试输出
        float speed_rpm = g_single_pid_output.target_speed * 9.5493f;
        printf("[SPID] pitch=%.2f° err=%.2f° rate=%.1f°/s | "
               "Angle: P=%.2f I=%.3f D=%.3f → speed=%.2f rad/s (%.0f rpm)\n",
               pitch,
               g_single_pid_output.angle_error,
               pitch_rate,
               g_single_pid_output.angle_p_out,
               g_single_pid_output.angle_i_out,
               g_single_pid_output.angle_d_out,
               g_single_pid_output.target_speed,
               speed_rpm);
               
    } else if (g_control_mode == CTRL_MODE_TRIPLE_PID) {
        // 四环 PID 调试输出
        if (g_triple_pid_ctrl.params.distance_enable) {
            printf("[TPID] pitch=%.2f° spd=%.2f | "
                   "Dist(最外): err=%.3f → spd_corr=%.3f | "
                   "Speed(外): err=%.2f → pitch_tgt=%.2f° | "
                   "Angle(中): err=%.2f → whl_tgt=%.2f | "
                   "Wheel(内): err=%.2f → out=%.3f [%s]\n",
                   pitch, wheel_speed,
                   g_triple_pid_output.distance_error,
                   g_triple_pid_output.distance_control,
                   g_triple_pid_output.speed_error,
                   g_triple_pid_output.pitch_target,
                   g_triple_pid_output.angle_error,
                   g_triple_pid_output.wheel_speed_target,
                   g_triple_pid_output.wheel_speed_error,
                   g_triple_pid_output.torque,
                   g_triple_pid_ctrl.params.wheel_mode == TRIPLE_PID_WHEEL_SPEED ? "spd" : "trq");
        } else {
            printf("[TPID] pitch=%.2f° spd=%.2f | "
                   "Speed(外): err=%.2f → pitch_tgt=%.2f° | "
                   "Angle(中): err=%.2f → whl_tgt=%.2f | "
                   "Wheel(内): err=%.2f → out=%.3f [%s]\n",
                   pitch, wheel_speed,
                   g_triple_pid_output.speed_error,
                   g_triple_pid_output.pitch_target,
                   g_triple_pid_output.angle_error,
                   g_triple_pid_output.wheel_speed_target,
                   g_triple_pid_output.wheel_speed_error,
                   g_triple_pid_output.torque,
                   g_triple_pid_ctrl.params.wheel_mode == TRIPLE_PID_WHEEL_SPEED ? "spd" : "trq");
        }
               
    } else {
        // LQR 模式调试输出
        printf("[LQR] pitch=%.2f° rate=%.1f°/s dist=%.3f spd=%.2f | "
               "u=%.3f yaw=%.3f\n",
               pitch, pitch_rate,
               g_lqr_distance, wheel_speed,
               g_last_lqr_u, g_yaw_output);
    }
    
    // X-Offset 调试输出 (附加行，仅在启用时显示)
    if (g_xoffset_enabled) {
        printf("[XOFF] spd=%.3f → x_off=%.4fm (Kp=%.4f Ki=%.4f Kd=%.4f lim=%.3f)\n",
               g_xoffset_debug_speed, g_xoffset_value,
               g_xoffset_pid.kp, g_xoffset_pid.ki, g_xoffset_pid.kd, g_xoffset_limit);
    }
    
    // Leg Sync 调试输出 (附加行，仅在启用时显示)
    if (g_leg_sync_enabled) {
        printf("[SYNC] diff=%.2f° → corr=%.2f° (gain=%.2f max=%.1f°)\n",
               g_leg_sync_debug_diff, g_leg_sync_debug_correction,
               g_leg_sync_gain, g_leg_sync_max_correction);
    }
}

