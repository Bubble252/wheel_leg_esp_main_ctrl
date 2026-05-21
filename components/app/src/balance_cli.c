/**
 * @file balance_cli.c
 * @brief CLI 命令处理 - 解析和执行所有 balance 命令
 *
 * 从 balance_test.c 拆分出来，包含 20+ 个子命令的处理逻辑。
 */

#include "balance_test.h"
#include "balance_types.h"
#include "balance_vmc.h"
#include "balance_observer.h"
#include "balance_plot.h"
#include "config.h"
#include "lqr_balance.h"
#include "full_lqr.h"
#include "leg_kinematics.h"
#include "can_motor.h"
#include "pi_comm.h"

#include <stdio.h>
#include <string.h>
#include <strings.h>
#include <stdlib.h>
#include <math.h>

// 角度-弧度转换
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define DEG2RAD(d) ((d) * M_PI / 180.0f)
#define RAD2DEG(r) ((r) * 180.0f / M_PI)

// 任务周期 (与 balance_test.c 保持一致)
#undef IMU_READ_PERIOD_MS
#undef BALANCE_CTRL_PERIOD_MS
#undef MOTOR_COMM_PERIOD_MS
#define IMU_READ_PERIOD_MS          3
#define BALANCE_CTRL_PERIOD_MS      2
#define MOTOR_COMM_PERIOD_MS        2

// ============================================================================
// extern 变量 (定义在 balance_test.c)
// ============================================================================

// 状态
extern balance_test_state_t g_state;
extern bool g_initialized;
extern bool g_use_unified_task;
extern bool g_uncontrolable_check_enabled;
extern balance_test_stats_t g_stats;
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
extern bool g_full_lqr_stream_enable;
extern float g_full_lqr_wheel_T_left;
extern float g_full_lqr_wheel_T_right;

// 延迟测量
extern float g_latency_imu_to_ctrl_us;
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

// 角度零点自适应
extern float g_angle_zeropoint;
extern float g_zp_speed_threshold;
extern float g_zp_pitch_for_ctrl;
extern float g_zp_angle_error;
extern float g_zp_pid_raw;
extern float g_zp_pid_filtered;
extern bool g_zp_active;

// 波形输出
extern bool g_plot_enabled;
extern uint8_t g_plot_divider;
extern uint32_t g_plot_channel_mask;

// PID 调试
extern bool g_pid_debug_enabled;
extern uint8_t g_pid_debug_divider;
extern uint8_t g_pid_debug_counter;

// 环路使能
extern uint8_t g_loop_enable_mask;

// Yaw 控制
extern bool g_yaw_control_enabled;
extern bool g_yaw_force_enable;

// Roll 控制
extern bool g_roll_control_enabled;

// 腿部补偿
extern bool g_pitch_leg_comp_enabled;

// X-Offset
extern bool g_xoffset_enabled;
extern pid_controller_t g_xoffset_pid;
extern float g_xoffset_value;
extern float g_xoffset_limit;
extern float g_xoffset_debug_speed;

// Leg Sync
extern bool g_leg_sync_enabled;
extern float g_leg_sync_gain;
extern float g_leg_sync_max_correction;
extern float g_leg_sync_debug_diff;
extern float g_leg_sync_debug_correction;

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
extern float g_leg_move_speed;
extern float g_leg_length_min;
extern float g_leg_length_max;

// 关节速度滤波
extern bool g_joint_speed_filter_enable;
extern int g_joint_speed_filter_mode;
extern float g_joint_speed_slew_rate;
extern int g_joint_median_window;
extern slewrate_filter_t g_sr_joint_lh;
extern slewrate_filter_t g_sr_joint_lk;
extern slewrate_filter_t g_sr_joint_rh;
extern slewrate_filter_t g_sr_joint_rk;
extern median_filter_t g_mf_joint_lh;
extern median_filter_t g_mf_joint_lk;
extern median_filter_t g_mf_joint_rh;
extern median_filter_t g_mf_joint_rk;
extern float g_joint_lh_spd_filtered;
extern float g_joint_lk_spd_filtered;
extern float g_joint_rh_spd_filtered;
extern float g_joint_rk_spd_filtered;

// 轮速/WMA
extern weighted_ma_filter_t g_wheel_speed_wma;
extern bool g_wma_enabled;
extern float g_left_wheel_speed_rad;
extern float g_right_wheel_speed_rad;
extern float g_left_wheel_accel;
extern float g_right_wheel_accel;
extern bool g_wheel_off_ground;

// 遥控器/手柄
extern float g_joy_speed_scale;
extern float g_joy_yaw_scale;

// 小车模式
extern control_mode_t g_control_mode;
extern control_mode_t g_car_mode_prev_mode;
extern float g_car_mode_prev_base_angle;
extern float g_car_mode_prev_base_length;

// 数据流开关
extern bool g_joint_stream_enable;
extern bool g_mpow_stream_enable;

// PI Comm
extern bool g_pi_comm_enabled;

// 电机句柄
extern can_motor_handle_t g_motor_left;
extern can_motor_handle_t g_motor_right;
extern can_motor_handle_t g_motor_left_hip;
extern can_motor_handle_t g_motor_left_knee;
extern can_motor_handle_t g_motor_right_hip;
extern can_motor_handle_t g_motor_right_knee;

// Full LQR Tp (已在 balance_vmc.h 中有部分, 这里补充调试用)
extern float g_full_lqr_Tp_left;
extern float g_full_lqr_Tp_right;

// 失控标志
extern int g_uncontrolable;

void balance_test_process_cmd(const char *cmd_str) {
    if (cmd_str == NULL) return;
    
    char cmd[64];
    strncpy(cmd, cmd_str, sizeof(cmd) - 1);
    cmd[sizeof(cmd) - 1] = '\0';
    
    char *token = strtok(cmd, " \t\n\r");
    if (token == NULL) {
        printf("Usage: balance [init|start|stop|enable|disable|estop|reset|status|zero|plot|debug|leg|roll|pitchcomp|xoffset|sync|vmc|...]\n");
        return;
    }
    
    if (strcmp(token, "init") == 0) {
        esp_err_t ret = balance_test_init();
        printf("Balance test init: %s\n", ret == ESP_OK ? "OK" : "FAILED");
    }
    else if (strcmp(token, "start") == 0) {
        esp_err_t ret = balance_test_start();
        printf("Balance test start: %s\n", ret == ESP_OK ? "OK" : "FAILED");
    }
    else if (strcmp(token, "stop") == 0) {
        balance_test_stop();
        printf("Balance test stopped\n");
    }
    else if (strcmp(token, "enable") == 0) {
        balance_test_enable();
        printf("Balance enabled\n");
    }
    else if (strcmp(token, "disable") == 0) {
        balance_test_disable();
        printf("Balance disabled\n");
    }
    else if (strcmp(token, "estop") == 0) {
        balance_test_emergency_stop();
        printf("Emergency stop!\n");
    }
    else if (strcmp(token, "reset") == 0) {
        balance_test_reset_emergency();
        printf("Emergency reset\n");
    }
    else if (strcmp(token, "status") == 0) {
        balance_test_print_status();
    }
    else if (strcmp(token, "freq") == 0) {
        // 查询任务频率 - 输出格式便于 Qt 解析
        printf("FREQ:IMU=%.1f,CTRL=%.1f,MOTOR=%.1f,LEG=%.1f\n",
               g_stats.imu_freq_hz, g_stats.control_freq_hz,
               g_stats.motor_freq_hz, g_stats.leg_freq_hz);
    }
    else if (strcmp(token, "latency") == 0) {
        // 查询控制回路延迟
        printf("=== Control Loop Latency (Precise) ===\n");
        printf("IMU -> Ctrl:    %.1f us (%.2f ms)\n", g_latency_imu_to_ctrl_us, g_latency_imu_to_ctrl_us / 1000.0f);
        printf("Ctrl calc:      %.1f us (%.2f ms)\n", g_latency_ctrl_calc_us, g_latency_ctrl_calc_us / 1000.0f);
        printf("Ctrl -> Motor:  %.1f us (%.2f ms)\n", g_latency_ctrl_to_motor_us, g_latency_ctrl_to_motor_us / 1000.0f);
        printf("Total (IMU->Motor):\n");
        printf("  Current: %.1f us (%.2f ms)\n", g_latency_total_us, g_latency_total_us / 1000.0f);
        printf("  Average: %.1f us (%.2f ms)\n", g_latency_total_avg_us, g_latency_total_avg_us / 1000.0f);
        printf("  Min:     %.1f us (%.2f ms)\n", g_latency_total_min_us, g_latency_total_min_us / 1000.0f);
        printf("  Max:     %.1f us (%.2f ms)\n", g_latency_total_max_us, g_latency_total_max_us / 1000.0f);
        printf("  Samples: %lu\n", g_latency_sample_count);
        printf("---\n");
        printf("Task periods: IMU=%dms, Ctrl=%dms, Motor=%dms\n", 
               IMU_READ_PERIOD_MS, BALANCE_CTRL_PERIOD_MS, MOTOR_COMM_PERIOD_MS);
        // 输出便于 UI 解析的格式 (添加统计信息)
        printf("LATENCY:IMU_CTRL=%.1f,CALC=%.1f,CTRL_MOTOR=%.1f,TOTAL=%.1f,AVG=%.1f,MIN=%.1f,MAX=%.1f\n",
               g_latency_imu_to_ctrl_us, g_latency_ctrl_calc_us, 
               g_latency_ctrl_to_motor_us, g_latency_total_us,
               g_latency_total_avg_us, g_latency_total_min_us, g_latency_total_max_us);
        
        // WiFi 遥控延迟
        printf("\n=== WiFi Remote Latency ===\n");
        printf("WiFi -> Ctrl:   %.1f us (%.2f ms)\n", g_latency_wifi_to_ctrl_us, g_latency_wifi_to_ctrl_us / 1000.0f);
        printf("WiFi Total (WiFi->Motor):\n");
        printf("  Current: %.1f us (%.2f ms)\n", g_latency_wifi_total_us, g_latency_wifi_total_us / 1000.0f);
        printf("  Average: %.1f us (%.2f ms)\n", g_latency_wifi_avg_us, g_latency_wifi_avg_us / 1000.0f);
        printf("  Min:     %.1f us (%.2f ms)\n", g_latency_wifi_min_us, g_latency_wifi_min_us / 1000.0f);
        printf("  Max:     %.1f us (%.2f ms)\n", g_latency_wifi_max_us, g_latency_wifi_max_us / 1000.0f);
        printf("  Samples: %lu\n", g_latency_wifi_sample_count);
        // WiFi 延迟 UI 解析格式
        printf("WIFI_LATENCY:WIFI_CTRL=%.1f,TOTAL=%.1f,AVG=%.1f,MIN=%.1f,MAX=%.1f\n",
               g_latency_wifi_to_ctrl_us, g_latency_wifi_total_us,
               g_latency_wifi_avg_us, g_latency_wifi_min_us, g_latency_wifi_max_us);
    }
    else if (strcmp(token, "zero") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            // 无参数: 显示完整零点自适应状态
            printf("=== 角度零点自适应状态 ===\n");
            printf("  当前零点: %.3f°\n", g_angle_zeropoint);
            printf("  当前pitch: %.3f°\n", g_zp_pitch_for_ctrl);
            printf("  角度误差: %.3f° (pitch - zeropoint)\n", g_zp_angle_error);
            printf("  PID输出: raw=%.6f, filtered=%.6f\n", g_zp_pid_raw, g_zp_pid_filtered);
            printf("  轮速阈值: %.3f m/s (当前轮速: %.3f)\n", g_zp_speed_threshold, g_lqr_speed);
            printf("  PID激活: %s\n", g_zp_active ? "YES (轮速<阈值)" : "NO (运动中)");
            printf("  PID参数: kp=%.6f, ki=%.6f, kd=%.6f\n", 
                   g_lqr_ctrl.pid_zeropoint.kp, g_lqr_ctrl.pid_zeropoint.ki, g_lqr_ctrl.pid_zeropoint.kd);
            printf("Usage: balance zero <degrees>       - 手动设置零点\n");
            printf("       balance zero threshold <val> - 设置轮速阈值 (m/s)\n");
        } else if (strcmp(token, "threshold") == 0 || strcmp(token, "thr") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                float thr = atof(token);
                if (thr > 0.0f && thr < 10.0f) {
                    g_zp_speed_threshold = thr;
                    printf("Zeropoint speed threshold set to %.3f m/s\n", g_zp_speed_threshold);
                } else {
                    printf("Invalid threshold, range: 0.001 ~ 10.0 m/s\n");
                }
            } else {
                printf("Current speed threshold: %.3f m/s\n", g_zp_speed_threshold);
                printf("Usage: balance zero threshold <value>\n");
            }
        } else {
            float zero = atof(token);
            balance_test_set_angle_zeropoint(zero);
            printf("Angle zeropoint set to %.2f\n", zero);
        }
    }
    else if (strcmp(token, "plot") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            printf("Plot output: %s\n", g_plot_enabled ? "enabled" : "disabled");
            printf("Plot divider: %d (%.1f Hz)\n", g_plot_divider, 500.0f / g_plot_divider);
            printf("Channel mask: 0x%08lX\n", (unsigned long)g_plot_channel_mask);
            // 打印已启用的通道列表
            printf("Enabled channels: ");
            const char all_ch[] = "ABCDEFGHIJKLMNOPQRSTUVWY";
            for (int i = 0; all_ch[i]; i++) {
                if (PLOT_CH_ENABLED(all_ch[i])) printf("%c", all_ch[i]);
            }
            printf("\n");
            printf("Usage: balance plot [on|off|div <N>|ch <ABCD...>|ch none|ch all]\n");
        } else if (strcmp(token, "on") == 0) {
            balance_test_set_plot(true);
        } else if (strcmp(token, "off") == 0) {
            balance_test_set_plot(false);
        } else if (strcmp(token, "div") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                int div = atoi(token);
                if (div >= 1 && div <= 255) {
                    balance_test_set_plot_divider((uint8_t)div);
                } else {
                    printf("Divider must be 1-255\n");
                }
            } else {
                printf("Current divider: %d\n", g_plot_divider);
                printf("Usage: balance plot div <1-255>\n");
            }
        } else if (strcmp(token, "ch") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token == NULL) {
                printf("Channel mask: 0x%08lX\n", (unsigned long)g_plot_channel_mask);
                printf("Usage: balance plot ch <ABCD...>  (e.g. 'ABDH' for angle+gyro+speed+output)\n");
                printf("       balance plot ch all        (enable all channels)\n");
                printf("       balance plot ch none       (disable all channels)\n");
            } else if (strcmp(token, "all") == 0) {
                g_plot_channel_mask = 0xFFFFFFFF;
                printf("All plot channels enabled\n");
            } else if (strcmp(token, "none") == 0) {
                g_plot_channel_mask = 0;
                printf("All plot channels disabled\n");
            } else {
                // 解析通道字母列表: "ABDH" -> 只开 A,B,D,H
                uint32_t mask = 0;
                for (int i = 0; token[i]; i++) {
                    char ch = token[i];
                    if (ch >= 'a' && ch <= 'z') ch -= 32;  // 转大写
                    if (ch >= 'A' && ch <= 'Z') {
                        mask |= plot_ch_bit(ch);
                    }
                }
                g_plot_channel_mask = mask;
                printf("Plot channels set to: ");
                for (int i = 0; i < 26; i++) {
                    if (mask & (1U << i)) printf("%c", 'A' + i);
                }
                printf(" (mask=0x%08lX)\n", (unsigned long)mask);
            }
        } else {
            printf("Unknown plot command: %s\n", token);
            printf("Usage: balance plot [on|off|div <N>|ch <ABCD...>]\n");
        }
    }
    // ===== PID 调试输出命令 =====
    else if (strcmp(token, "debug") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            printf("PID debug output: %s\n", g_pid_debug_enabled ? "enabled" : "disabled");
            printf("Debug divider: %d (%.1f Hz @ 200Hz ctrl)\n", g_pid_debug_divider, 200.0f / g_pid_debug_divider);
            printf("Control mode: %s\n", 
                   g_control_mode == CTRL_MODE_LQR ? "LQR" :
                   g_control_mode == CTRL_MODE_DUAL_PID ? "DUAL_PID" : 
                   g_control_mode == CTRL_MODE_CAR ? "CAR" : "SINGLE_PID");
            printf("Usage: balance debug [on|off|div <N>]\n");
        } else if (strcmp(token, "on") == 0) {
            g_pid_debug_enabled = true;
            g_pid_debug_counter = 0;
            printf("PID debug output enabled (div=%d, %.1f Hz)\n", 
                   g_pid_debug_divider, 200.0f / g_pid_debug_divider);
        } else if (strcmp(token, "off") == 0) {
            g_pid_debug_enabled = false;
            printf("PID debug output disabled\n");
        } else if (strcmp(token, "div") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                int div = atoi(token);
                if (div >= 1 && div <= 255) {
                    g_pid_debug_divider = (uint8_t)div;
                    printf("Debug divider set to %d (%.1f Hz @ 200Hz ctrl)\n", div, 200.0f / div);
                } else {
                    printf("Divider must be 1-255\n");
                }
            } else {
                printf("Current divider: %d (%.1f Hz)\n", g_pid_debug_divider, 200.0f / g_pid_debug_divider);
                printf("Usage: balance debug div <1-255>\n");
            }
        } else {
            printf("Unknown debug command: %s\n", token);
            printf("Usage: balance debug [on|off|div <N>]\n");
        }
    }
    // ===== 腿部电机控制命令 =====
    else if (strcmp(token, "leg") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            printf("Leg control: %s\n", g_leg_control_enabled ? "enabled" : "disabled");
            printf("Motor angles: L_Hip=%.1f, L_Knee=%.1f, R_Hip=%.1f, R_Knee=%.1f\n",
                   g_leg_left_hip_angle, g_leg_left_knee_angle,
                   g_leg_right_hip_angle, g_leg_right_knee_angle);
            printf("Target state: L(%.3fm, %.1fdeg) R(%.3fm, %.1fdeg)\n",
                   g_leg_left_target_length, g_leg_left_target_angle,
                   g_leg_right_target_length, g_leg_right_target_angle);
            printf("Leg length range: %.3f ~ %.3f m\n", g_leg_length_min, g_leg_length_max);
            printf("Leg speed: %.1f rpm\n", g_leg_move_speed);
            printf("Usage: balance leg [on|off|status|set|target|speed|range|test]\n");
        } else if (strcmp(token, "on") == 0) {
            balance_test_set_leg_control(true);
            printf("Leg motors enabled\n");
        } else if (strcmp(token, "off") == 0) {
            balance_test_set_leg_control(false);
            printf("Leg motors disabled\n");
        } else if (strcmp(token, "status") == 0) {
            // 显示详细腿部状态
            leg_ctrl_print_status();
        } else if (strcmp(token, "set") == 0) {
            // balance leg set <left_hip> <left_knee> <right_hip> <right_knee>
            // 直接设置电机角度 (原始模式)
            float lh = 0, lk = 0, rh = 0, rk = 0;
            token = strtok(NULL, " \t\n\r");
            if (token) lh = atof(token);
            token = strtok(NULL, " \t\n\r");
            if (token) lk = atof(token);
            token = strtok(NULL, " \t\n\r");
            if (token) rh = atof(token);
            token = strtok(NULL, " \t\n\r");
            if (token) rk = atof(token);
            balance_test_set_leg_angles(lh, lk, rh, rk);
            printf("Motor angles set: L_Hip=%.1f, L_Knee=%.1f, R_Hip=%.1f, R_Knee=%.1f\n",
                   lh, lk, rh, rk);
        } else if (strcmp(token, "target") == 0) {
            // balance leg target <length> <angle> [left|right|both]
            // 使用运动学设置目标腿长和身体夹角
            token = strtok(NULL, " \t\n\r");
            if (token == NULL) {
                printf("Usage: balance leg target <length_m> <angle_deg> [left|right|both]\n");
                printf("  length: leg length in meters (%.3f ~ %.3f)\n", g_leg_length_min, g_leg_length_max);
                printf("  angle: body angle in degrees (%.1f ~ %.1f), forward=positive\n", 
                       LEG_BODY_ANGLE_MIN, LEG_BODY_ANGLE_MAX);
                printf("Example: balance leg target 0.15 0 both\n");
            } else {
                float length = atof(token);
                float angle = 0;
                char side = 'b';  // default both
                
                token = strtok(NULL, " \t\n\r");
                if (token) angle = atof(token);
                
                token = strtok(NULL, " \t\n\r");
                if (token) {
                    if (strcmp(token, "left") == 0 || strcmp(token, "l") == 0) side = 'l';
                    else if (strcmp(token, "right") == 0 || strcmp(token, "r") == 0) side = 'r';
                }
                
                esp_err_t ret = ESP_OK;
                if (side == 'l') {
                    ret = leg_ctrl_set_target(true, length, angle);
                } else if (side == 'r') {
                    ret = leg_ctrl_set_target(false, length, angle);
                } else {
                    ret = leg_ctrl_set_both(length, angle, length, angle);
                }
                
                if (ret == ESP_OK) {
                    printf("Target set: Length=%.3fm, Angle=%.1fdeg\n", length, angle);
                    
                    // 立即发送一次电机命令 (方便调试，不需要等待 task_motor_comm)
                    if (g_leg_control_enabled) {
                        if (g_motor_left_hip) {
                            can_motor_set_position(g_motor_left_hip, g_leg_left_hip_angle, g_leg_move_speed);
                        }
                        if (g_motor_left_knee) {
                            can_motor_set_position(g_motor_left_knee, g_leg_left_knee_angle, g_leg_move_speed);
                        }
                        if (g_motor_right_hip) {
                            can_motor_set_position(g_motor_right_hip, g_leg_right_hip_angle, g_leg_move_speed);
                        }
                        if (g_motor_right_knee) {
                            can_motor_set_position(g_motor_right_knee, g_leg_right_knee_angle, g_leg_move_speed);
                        }
                        printf("Motor commands sent immediately\n");
                    } else {
                        printf("Note: Leg control disabled, use 'balance leg on' to enable\n");
                    }
                } else {
                    printf("Failed to set target (out of range)\n");
                }
            }
        } else if (strcmp(token, "speed") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                float spd = atof(token);
                balance_test_set_leg_speed(spd);
                printf("Leg speed set to %.1f rpm\n", spd);
            } else {
                printf("Current leg speed: %.1f rpm\n", g_leg_move_speed);
                printf("Usage: balance leg speed <rpm>\n");
            }
        } else if (strcmp(token, "range") == 0) {
            // balance leg range [<min> <max>]
            // 设置腿长范围限制
            token = strtok(NULL, " \t\n\r");
            if (token == NULL) {
                // 显示当前范围
                printf("Leg length range: %.3f ~ %.3f m\n", g_leg_length_min, g_leg_length_max);
                printf("Usage: balance leg range <min_m> <max_m>\n");
                printf("  Default: 0.07 ~ 0.17 m\n");
            } else {
                float new_min = atof(token);
                token = strtok(NULL, " \t\n\r");
                if (token) {
                    float new_max = atof(token);
                    if (new_min >= new_max) {
                        printf("Error: min (%.3f) must be less than max (%.3f)\n", new_min, new_max);
                    } else if (new_min < 0.05f || new_max > 0.20f) {
                        printf("Error: range out of hardware limits (0.05 ~ 0.20 m)\n");
                    } else {
                        g_leg_length_min = new_min;
                        g_leg_length_max = new_max;
                        printf("Leg length range set: %.3f ~ %.3f m\n", g_leg_length_min, g_leg_length_max);
                    }
                } else {
                    printf("Usage: balance leg range <min_m> <max_m>\n");
                }
            }
        } else if (strcmp(token, "test") == 0) {
            // 测试运动学正逆解 (调用 algorithm 模块)
            token = strtok(NULL, " \t\n\r");
            if (token == NULL) {
                printf("Usage: balance leg test fk <hip> <knee> [left|right]\n");
                printf("       balance leg test ik <length> <angle> [left|right]\n");
            } else if (strcmp(token, "fk") == 0) {
                // 正运动学测试
                float hip = 0, knee = 0;
                bool is_left = true;
                token = strtok(NULL, " \t\n\r");
                if (token) hip = atof(token);
                token = strtok(NULL, " \t\n\r");
                if (token) knee = atof(token);
                token = strtok(NULL, " \t\n\r");
                if (token && (strcmp(token, "right") == 0 || strcmp(token, "r") == 0)) {
                    is_left = false;
                }
                
                leg_joint_state_t joint = { .hip_angle = hip, .knee_angle = knee };
                leg_workspace_state_t ws;
                if (leg_kin_forward(&joint, is_left, NULL, &ws) == ESP_OK) {
                    printf("FK (%s): Hip=%.1f, Knee=%.1f -> Length=%.3fm, Angle=%.1fdeg\n",
                           is_left ? "left" : "right", hip, knee, ws.leg_length, ws.body_angle);
                } else {
                    printf("FK failed\n");
                }
            } else if (strcmp(token, "ik") == 0) {
                // 逆运动学测试
                float length = 0.15f, angle = 0;
                bool is_left = true;
                token = strtok(NULL, " \t\n\r");
                if (token) length = atof(token);
                token = strtok(NULL, " \t\n\r");
                if (token) angle = atof(token);
                token = strtok(NULL, " \t\n\r");
                if (token && (strcmp(token, "right") == 0 || strcmp(token, "r") == 0)) {
                    is_left = false;
                }
                
                leg_workspace_state_t ws = { .leg_length = length, .body_angle = angle };
                leg_joint_state_t joint;
                if (leg_kin_inverse(&ws, is_left, NULL, &joint) == ESP_OK) {
                    printf("IK (%s): Length=%.3fm, Angle=%.1fdeg -> Hip=%.1f, Knee=%.1f\n",
                           is_left ? "left" : "right", length, angle, joint.hip_angle, joint.knee_angle);
                } else {
                    printf("IK failed (target unreachable)\n");
                }
            }
        } else {
            printf("Unknown leg command: %s\n", token);
            printf("Usage: balance leg [on|off|status|set|target|speed|range|test]\n");
            printf("  on/off  - Enable/disable leg motors\n");
            printf("  status  - Show detailed leg status\n");
            printf("  set <lh> <lk> <rh> <rk> - Set motor angles directly\n");
            printf("  target <length> <angle> [left|right|both] - Set using kinematics\n");
            printf("  speed <rpm> - Set leg motor speed\n");
            printf("  range [<min> <max>] - Get/set leg length range (m)\n");
            printf("  test fk/ik ... - Test kinematics\n");
        }
    }
    // ===== Roll 闭环控制开关 =====
    else if (strcmp(token, "roll") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            printf("Roll control: %s\n", g_roll_control_enabled ? "enabled" : "disabled");
            printf("Usage: balance roll [on|off]\n");
        } else if (strcmp(token, "on") == 0) {
            balance_test_set_roll_control(true);
            printf("Roll control enabled\n");
        } else if (strcmp(token, "off") == 0) {
            balance_test_set_roll_control(false);
            printf("Roll control disabled\n");
        } else {
            printf("Unknown roll command: %s\n", token);
            printf("Usage: balance roll [on|off]\n");
        }
    }
    // ===== Pitch 腿部角度补偿开关 =====
    else if (strcmp(token, "pitchcomp") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            printf("Pitch leg compensation: %s\n", g_pitch_leg_comp_enabled ? "enabled" : "disabled");
            printf("  When enabled: pitch_for_control = imu.pitch + body_angle + 90\n");
            printf("  When disabled: pitch_for_control = imu.pitch (raw IMU)\n");
            printf("Usage: balance pitchcomp [on|off]\n");
        } else if (strcmp(token, "on") == 0) {
            balance_test_set_pitch_comp(true);
            printf("Pitch leg compensation enabled\n");
        } else if (strcmp(token, "off") == 0) {
            balance_test_set_pitch_comp(false);
            printf("Pitch leg compensation disabled (using raw IMU pitch)\n");
        } else {
            printf("Unknown pitchcomp command: %s\n", token);
            printf("Usage: balance pitchcomp [on|off]\n");
        }
    }
    // ===== X-Offset 腿部速度自适应偏移 =====
    else if (strcmp(token, "xoffset") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            // 显示状态
            printf("=== X-Offset Status ===\n");
            printf("X-Offset: %s\n", g_xoffset_enabled ? "ENABLED" : "DISABLED");
            printf("PID: Kp=%.4f Ki=%.4f Kd=%.4f\n", 
                   g_xoffset_pid.kp, g_xoffset_pid.ki, g_xoffset_pid.kd);
            printf("Limit: %.3f m (%.1f cm)\n", g_xoffset_limit, g_xoffset_limit * 100.0f);
            printf("Current value: %.4f m (speed=%.3f m/s)\n", 
                   g_xoffset_value, g_xoffset_debug_speed);
            printf("Usage: balance xoffset [on|off|kp|ki|kd|limit]\n");
        } else if (strcmp(token, "on") == 0) {
            balance_test_set_xoffset(true);
            printf("X-Offset enabled\n");
        } else if (strcmp(token, "off") == 0) {
            balance_test_set_xoffset(false);
            printf("X-Offset disabled\n");
        } else if (strcmp(token, "kp") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                float kp = atof(token);
                balance_test_set_xoffset_pid(kp, g_xoffset_pid.ki, g_xoffset_pid.kd);
                printf("X-Offset Kp = %.4f\n", kp);
            } else {
                printf("Current Kp = %.4f\n", g_xoffset_pid.kp);
            }
        } else if (strcmp(token, "ki") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                float ki = atof(token);
                balance_test_set_xoffset_pid(g_xoffset_pid.kp, ki, g_xoffset_pid.kd);
                printf("X-Offset Ki = %.4f\n", ki);
            } else {
                printf("Current Ki = %.4f\n", g_xoffset_pid.ki);
            }
        } else if (strcmp(token, "kd") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                float kd = atof(token);
                balance_test_set_xoffset_pid(g_xoffset_pid.kp, g_xoffset_pid.ki, kd);
                printf("X-Offset Kd = %.4f\n", kd);
            } else {
                printf("Current Kd = %.4f\n", g_xoffset_pid.kd);
            }
        } else if (strcmp(token, "limit") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                float limit = atof(token);
                balance_test_set_xoffset_limit(limit);
                printf("X-Offset limit = %.3f m\n", limit);
            } else {
                printf("Current limit = %.3f m (%.1f cm)\n", g_xoffset_limit, g_xoffset_limit * 100.0f);
            }
        } else {
            printf("Unknown xoffset command: %s\n", token);
            printf("Usage: balance xoffset [on|off|kp <val>|ki <val>|kd <val>|limit <val>]\n");
        }
    }
    // ===== Leg Sync (防劈叉) =====
    else if (strcmp(token, "sync") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            // 显示 Sync 状态
            printf("=== Leg Sync Status ===\n");
            printf("Leg Sync: %s\n", g_leg_sync_enabled ? "ENABLED" : "DISABLED");
            printf("Gain: %.2f\n", g_leg_sync_gain);
            printf("Max correction: %.1f°\n", g_leg_sync_max_correction);
            printf("Current: diff=%.2f° corr=%.2f°\n",
                   g_leg_sync_debug_diff, g_leg_sync_debug_correction);
            printf("Usage: balance sync [on|off|gain <0~1>|max <deg>]\n");
        } else if (strcmp(token, "on") == 0) {
            balance_test_set_leg_sync(true);
            printf("Leg Sync enabled\n");
        } else if (strcmp(token, "off") == 0) {
            balance_test_set_leg_sync(false);
            printf("Leg Sync disabled\n");
        } else if (strcmp(token, "gain") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                float gain = atof(token);
                balance_test_set_leg_sync_gain(gain);
                printf("Leg Sync gain = %.2f\n", gain);
            } else {
                printf("Current gain = %.2f\n", g_leg_sync_gain);
            }
        } else if (strcmp(token, "max") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                float max_deg = atof(token);
                balance_test_set_leg_sync_max(max_deg);
                printf("Leg Sync max correction = %.1f°\n", max_deg);
            } else {
                printf("Current max correction = %.1f°\n", g_leg_sync_max_correction);
            }
        } else {
            printf("Unknown sync command: %s\n", token);
            printf("Usage: balance sync [on|off|gain <0~1>|max <deg>]\n");
        }
    }
    // ===== VMC 力控模式 =====
    else if (strcmp(token, "vmc") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            // 显示 VMC 状态
            printf("=== VMC Status ===\n");
            printf("VMC mode: %s\n", g_vmc_enabled ? "ENABLED (force control)" : "DISABLED (position control)");
            printf("Coord type: %s\n", g_vmc_params.coord_type == VMC_COORD_BODY ? "BODY" : "WORLD");
            printf("Target height: %.3f m\n", g_vmc_target_y);
            printf("Target vx: %.3f m/s\n", g_vmc_target_vx);
            printf("Params: K_vx=%.1f, K_y=%.1f, D_y=%.1f, gc=%.2f, mass=%.1fkg\n",
                   g_vmc_params.K_vx, g_vmc_params.K_y, g_vmc_params.D_y,
                   g_vmc_params.gravity_comp, g_vmc_params.robot_mass);
            printf("Torque limits: hip=%.1fNm, knee=%.1fNm\n",
                   g_vmc_params.max_hip_torque, g_vmc_params.max_knee_torque);
            printf("Leg Sync: %s (Kp=%.3f, Kd=%.3f)\n", 
                   g_vmc_params.sync_enable ? "ON" : "OFF", g_vmc_params.K_sync, g_vmc_params.D_sync);
            if (g_vmc_enabled) {
                printf("Left:  F_x=%.2fN, F_y=%.2fN, tau_hip=%.2fNm, tau_knee=%.2fNm\n",
                       g_vmc_dual_output.left.debug.F_x, g_vmc_dual_output.left.debug.F_y,
                       g_vmc_dual_output.left.hip_torque, g_vmc_dual_output.left.knee_torque);
                printf("Right: F_x=%.2fN, F_y=%.2fN, tau_hip=%.2fNm, tau_knee=%.2fNm\n",
                       g_vmc_dual_output.right.debug.F_x, g_vmc_dual_output.right.debug.F_y,
                       g_vmc_dual_output.right.hip_torque, g_vmc_dual_output.right.knee_torque);
                if (g_vmc_params.sync_enable) {
                    printf("Sync: diff=%.2fdeg, rate=%.1fdeg/s, out=%.3fNm\n",
                           g_vmc_dual_output.angle_diff_deg, g_vmc_dual_output.angle_diff_rate_deg, 
                           g_vmc_dual_output.F_sync);
                }
            }
            printf("Usage: balance vmc [on|off|kvx|ky|dy|gc|mass|height|status|sync]\n");
        } else if (strcmp(token, "on") == 0) {
            if (!g_leg_control_enabled) {
                printf("Error: Enable leg control first (balance leg on)\n");
            } else {
                // 切换腿部电机到力矩模式
                if (g_motor_left_hip) can_motor_set_mode(g_motor_left_hip, MODE_TORQUE);
                if (g_motor_left_knee) can_motor_set_mode(g_motor_left_knee, MODE_TORQUE);
                if (g_motor_right_hip) can_motor_set_mode(g_motor_right_hip, MODE_TORQUE);
                if (g_motor_right_knee) can_motor_set_mode(g_motor_right_knee, MODE_TORQUE);
                
                g_vmc_enabled = true;
                printf("VMC force control ENABLED\n");
                printf("Note: Leg motors now in torque mode!\n");
            }
        } else if (strcmp(token, "off") == 0) {
            g_vmc_enabled = false;
            
            // 切换腿部电机回位置模式
            if (g_motor_left_hip) can_motor_set_mode(g_motor_left_hip, MODE_POS_FILTER);
            if (g_motor_left_knee) can_motor_set_mode(g_motor_left_knee, MODE_POS_FILTER);
            if (g_motor_right_hip) can_motor_set_mode(g_motor_right_hip, MODE_POS_FILTER);
            if (g_motor_right_knee) can_motor_set_mode(g_motor_right_knee, MODE_POS_FILTER);
            
            printf("VMC force control DISABLED (back to position control)\n");
        } else if (strcmp(token, "kvx") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                g_vmc_params.K_vx = atof(token);
                printf("VMC K_vx = %.1f Ns/m\n", g_vmc_params.K_vx);
            } else {
                printf("VMC K_vx = %.1f Ns/m\n", g_vmc_params.K_vx);
                printf("Usage: balance vmc kvx <value>\n");
            }
        } else if (strcmp(token, "ky") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                g_vmc_params.K_y = atof(token);
                printf("VMC K_y = %.1f N/m\n", g_vmc_params.K_y);
            } else {
                printf("VMC K_y = %.1f N/m\n", g_vmc_params.K_y);
                printf("Usage: balance vmc ky <value>\n");
            }
        } else if (strcmp(token, "dy") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                g_vmc_params.D_y = atof(token);
                printf("VMC D_y = %.1f Ns/m\n", g_vmc_params.D_y);
            } else {
                printf("VMC D_y = %.1f Ns/m\n", g_vmc_params.D_y);
                printf("Usage: balance vmc dy <value>\n");
            }
        } else if (strcmp(token, "gc") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                g_vmc_params.gravity_comp = atof(token);
                if (g_vmc_params.gravity_comp < 0) g_vmc_params.gravity_comp = 0;
                if (g_vmc_params.gravity_comp > 1.5f) g_vmc_params.gravity_comp = 1.5f;
                printf("VMC gravity_comp = %.2f\n", g_vmc_params.gravity_comp);
            } else {
                printf("VMC gravity_comp = %.2f\n", g_vmc_params.gravity_comp);
                printf("Usage: balance vmc gc <0~1.5>\n");
            }
        } else if (strcmp(token, "mass") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                g_vmc_params.robot_mass = atof(token);
                printf("VMC robot_mass = %.1f kg\n", g_vmc_params.robot_mass);
            } else {
                printf("VMC robot_mass = %.1f kg\n", g_vmc_params.robot_mass);
                printf("Usage: balance vmc mass <kg>\n");
            }
        } else if (strcmp(token, "height") == 0 || strcmp(token, "y") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                g_vmc_target_y = atof(token);
                // 限制在合理范围
                if (g_vmc_target_y < 0.07f) g_vmc_target_y = 0.07f;
                if (g_vmc_target_y > 0.19f) g_vmc_target_y = 0.19f;
                printf("VMC target height = %.3f m\n", g_vmc_target_y);
            } else {
                printf("VMC target height = %.3f m\n", g_vmc_target_y);
                printf("Usage: balance vmc height <meters>\n");
            }
        } else if (strcmp(token, "vx") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                g_vmc_target_vx = atof(token);
                printf("VMC target vx = %.3f m/s\n", g_vmc_target_vx);
            } else {
                printf("VMC target vx = %.3f m/s\n", g_vmc_target_vx);
                printf("Usage: balance vmc vx <m/s>\n");
            }
        } else if (strcmp(token, "status") == 0) {
            // 详细状态
            printf("=== VMC Detailed Status ===\n");
            printf("Mode: %s\n", g_vmc_enabled ? "FORCE CONTROL" : "POSITION CONTROL");
            printf("Coordinate: %s\n", g_vmc_params.coord_type == VMC_COORD_WORLD ? "WORLD (x-y)" : "BODY (L-α)");
            printf("Diff method: %s\n", g_vmc_params.vmc_diff_method == VMC_DIFF_JACOBIAN ? "Jacobian (关节速度)" : "Numeric (数值微分)");
            printf("Target: height=%.3fm, vx=%.3fm/s\n", g_vmc_target_y, g_vmc_target_vx);
            printf("Params (World Coord):\n");
            printf("  K_vx = %.1f Ns/m (horizontal velocity gain)\n", g_vmc_params.K_vx);
            printf("  K_y  = %.1f N/m (vertical stiffness)\n", g_vmc_params.K_y);
            printf("  D_y  = %.1f Ns/m (vertical damping)\n", g_vmc_params.D_y);
            printf("Params (Body Coord):\n");
            printf("  K_L  = %.1f N/m (leg stiffness)\n", g_vmc_params.K_L);
            printf("  D_L  = %.1f Ns/m (leg damping)\n", g_vmc_params.D_L);
            printf("  K_α  = %.2f Nm/rad (angle stiffness)\n", g_vmc_params.K_alpha);
            printf("  D_α  = %.2f Nm·s/rad (angle damping)\n", g_vmc_params.D_alpha);
            printf("Common:\n");
            printf("  gravity_comp = %.2f (0~1)\n", g_vmc_params.gravity_comp);
            printf("  robot_mass = %.1f kg\n", g_vmc_params.robot_mass);
            printf("  max_hip_torque = %.1f Nm\n", g_vmc_params.max_hip_torque);
            printf("  max_knee_torque = %.1f Nm\n", g_vmc_params.max_knee_torque);
            printf("Pitch Control: %s\n", g_vmc_params.pitch_ctrl_enable ? "ENABLED" : "DISABLED");
            printf("  K_pitch = %.2f Nm/rad\n", g_vmc_params.K_pitch);
            printf("  D_pitch = %.2f Nm·s/rad\n", g_vmc_params.D_pitch);
            printf("  target_pitch = %.2f deg\n", RAD2DEG(g_vmc_params.target_pitch));
            if (g_vmc_enabled && g_motor_left_hip && g_motor_left_knee) {
                // 计算当前状态
                leg_joint_state_t joint = {
                    .hip_angle = can_motor_read_position(g_motor_left_hip),
                    .knee_angle = can_motor_read_position(g_motor_left_knee)
                };
                leg_workspace_state_t ws;
                leg_kin_forward(&joint, true, NULL, &ws);
                float x, y;
                leg_kin_forward_cartesian(&joint, true, NULL, 0.0f, &x, &y);
                printf("Left leg state:\n");
                printf("  Body coord: L=%.3fm, α=%.1f°\n", ws.leg_length, ws.body_angle);
                printf("  World coord: y=%.3fm, x=%.3fm (pitch=0)\n", y, x);
                if (g_vmc_params.coord_type == VMC_COORD_BODY) {
                    printf("  Output: F_L=%.2fN, F_α=%.3fNm (gravity=%.2fN)\n",
                           g_vmc_dual_output.left.debug.F_L, g_vmc_dual_output.left.debug.F_alpha, g_vmc_dual_output.left.debug.F_gravity);
                } else {
                    printf("  Output: F_x=%.2fN, F_y=%.2fN (gravity=%.2fN)\n",
                           g_vmc_dual_output.left.debug.F_x, g_vmc_dual_output.left.debug.F_y, g_vmc_dual_output.left.debug.F_gravity);
                }
                printf("  Torque: hip=%.2fNm (vmc=%.2f, pitch=%.2f), knee=%.2fNm\n",
                       g_vmc_dual_output.left.hip_torque, g_vmc_dual_output.left.debug.tau_hip_vmc, 
                       g_vmc_dual_output.left.debug.tau_hip_pitch, g_vmc_dual_output.left.knee_torque);
            }
        } else if (strcmp(token, "coord") == 0) {
            // 坐标系切换
            token = strtok(NULL, " \t\n\r");
            if (token == NULL) {
                printf("VMC Coordinate: %s\n", g_vmc_params.coord_type == VMC_COORD_WORLD ? "WORLD (x-y)" : "BODY (L-α)");
                printf("Usage: balance vmc coord [world|body]\n");
            } else if (strcmp(token, "world") == 0) {
                g_vmc_params.coord_type = VMC_COORD_WORLD;
                printf("VMC coordinate set to WORLD (x-y)\n");
                printf("  Controls: horizontal velocity (vx), vertical height (y)\n");
            } else if (strcmp(token, "body") == 0) {
                g_vmc_params.coord_type = VMC_COORD_BODY;
                printf("VMC coordinate set to BODY (L-α)\n");
                printf("  Controls: leg length (L), body angle (α)\n");
            } else {
                printf("Unknown coordinate type: %s\n", token);
                printf("Usage: balance vmc coord [world|body]\n");
            }
        } else if (strcmp(token, "kl") == 0) {
            // 腿长刚度 (机身坐标系)
            token = strtok(NULL, " \t\n\r");
            if (token) {
                g_vmc_params.K_L = atof(token);
                printf("VMC K_L = %.1f N/m\n", g_vmc_params.K_L);
            } else {
                printf("VMC K_L = %.1f N/m\n", g_vmc_params.K_L);
                printf("Usage: balance vmc kl <N/m>\n");
            }
        } else if (strcmp(token, "dl") == 0) {
            // 腿长阻尼 (机身坐标系)
            token = strtok(NULL, " \t\n\r");
            if (token) {
                g_vmc_params.D_L = atof(token);
                printf("VMC D_L = %.1f Ns/m\n", g_vmc_params.D_L);
            } else {
                printf("VMC D_L = %.1f Ns/m\n", g_vmc_params.D_L);
                printf("Usage: balance vmc dl <Ns/m>\n");
            }
        } else if (strcmp(token, "ka") == 0) {
            // 身体角度刚度 (机身坐标系)
            token = strtok(NULL, " \t\n\r");
            if (token) {
                g_vmc_params.K_alpha = atof(token);
                printf("VMC K_alpha = %.2f Nm/rad\n", g_vmc_params.K_alpha);
            } else {
                printf("VMC K_alpha = %.2f Nm/rad\n", g_vmc_params.K_alpha);
                printf("Usage: balance vmc ka <Nm/rad>\n");
            }
        } else if (strcmp(token, "da") == 0) {
            // 身体角度阻尼 (机身坐标系)
            token = strtok(NULL, " \t\n\r");
            if (token) {
                g_vmc_params.D_alpha = atof(token);
                printf("VMC D_alpha = %.2f Nm·s/rad\n", g_vmc_params.D_alpha);
            } else {
                printf("VMC D_alpha = %.2f Nm·s/rad\n", g_vmc_params.D_alpha);
                printf("Usage: balance vmc da <Nm·s/rad>\n");
            }
        } else if (strcmp(token, "soft") == 0) {
            // 柔软预设
            g_vmc_params.K_y = 500.0f;
            g_vmc_params.D_y = 80.0f;
            printf("VMC preset: SOFT (K_y=500, D_y=80)\n");
        } else if (strcmp(token, "stiff") == 0) {
            // 刚硬预设
            g_vmc_params.K_y = 1500.0f;
            g_vmc_params.D_y = 30.0f;
            printf("VMC preset: STIFF (K_y=1500, D_y=30)\n");
        } else if (strcmp(token, "pitch") == 0) {
            // Pitch 控制子命令
            token = strtok(NULL, " \t\n\r");
            if (token == NULL) {
                printf("VMC Pitch Control: %s\n", g_vmc_params.pitch_ctrl_enable ? "ENABLED" : "DISABLED");
                printf("  K_pitch = %.2f Nm/rad\n", g_vmc_params.K_pitch);
                printf("  D_pitch = %.2f Nm·s/rad\n", g_vmc_params.D_pitch);
                printf("  target_pitch = %.2f deg\n", RAD2DEG(g_vmc_params.target_pitch));
                if (g_vmc_enabled && g_vmc_input_valid) {
                    printf("  tau_hip_pitch (L) = %.3f Nm\n", g_vmc_dual_output.left.debug.tau_hip_pitch);
                    printf("  tau_hip_pitch (R) = %.3f Nm\n", g_vmc_dual_output.right.debug.tau_hip_pitch);
                }
                printf("Usage: balance vmc pitch [on|off|kp|kd|target]\n");
            } else if (strcmp(token, "on") == 0) {
                g_vmc_params.pitch_ctrl_enable = true;
                printf("VMC pitch control ENABLED\n");
            } else if (strcmp(token, "off") == 0) {
                g_vmc_params.pitch_ctrl_enable = false;
                printf("VMC pitch control DISABLED\n");
            } else if (strcmp(token, "kp") == 0) {
                token = strtok(NULL, " \t\n\r");
                if (token) {
                    g_vmc_params.K_pitch = atof(token);
                    printf("VMC K_pitch = %.2f Nm/rad\n", g_vmc_params.K_pitch);
                } else {
                    printf("VMC K_pitch = %.2f Nm/rad\n", g_vmc_params.K_pitch);
                    printf("Usage: balance vmc pitch kp <value>\n");
                }
            } else if (strcmp(token, "kd") == 0) {
                token = strtok(NULL, " \t\n\r");
                if (token) {
                    g_vmc_params.D_pitch = atof(token);
                    printf("VMC D_pitch = %.2f Nm·s/rad\n", g_vmc_params.D_pitch);
                } else {
                    printf("VMC D_pitch = %.2f Nm·s/rad\n", g_vmc_params.D_pitch);
                    printf("Usage: balance vmc pitch kd <value>\n");
                }
            } else if (strcmp(token, "target") == 0) {
                token = strtok(NULL, " \t\n\r");
                if (token) {
                    g_vmc_params.target_pitch = DEG2RAD(atof(token));
                    printf("VMC target_pitch = %.2f deg\n", RAD2DEG(g_vmc_params.target_pitch));
                } else {
                    printf("VMC target_pitch = %.2f deg\n", RAD2DEG(g_vmc_params.target_pitch));
                    printf("Usage: balance vmc pitch target <deg>\n");
                }
            } else {
                printf("Unknown vmc pitch command: %s\n", token);
                printf("Usage: balance vmc pitch [on|off|kp|kd|target]\n");
            }
        } else if (strcmp(token, "sync") == 0) {
            // 双腿协调控制 (Leg Sync) 子命令
            token = strtok(NULL, " \t\n\r");
            if (token == NULL) {
                printf("=== Leg Sync Control ===\n");
                printf("Status: %s\n", g_vmc_params.sync_enable ? "ENABLED" : "DISABLED");
                printf("  Kp = %.3f Nm/rad\n", g_vmc_params.K_sync);
                printf("  Kd = %.3f Nm·s/rad\n", g_vmc_params.D_sync);
                if (g_vmc_enabled && g_vmc_params.sync_enable) {
                    printf("  Left angle  = %.2f deg\n", g_vmc_dual_output.left.current_body_angle);
                    printf("  Right angle = %.2f deg\n", g_vmc_dual_output.right.current_body_angle);
                    printf("  Angle diff  = %.2f deg\n", g_vmc_dual_output.angle_diff_deg);
                    printf("  Diff rate   = %.2f deg/s\n", g_vmc_dual_output.angle_diff_rate_deg);
                    printf("  Sync output = %.3f Nm\n", g_vmc_dual_output.F_sync);
                }
                printf("Usage: balance vmc sync [on|off|kp|kd]\n");
            } else if (strcmp(token, "on") == 0) {
                g_vmc_params.sync_enable = true;
                printf("Leg sync control ENABLED\n");
            } else if (strcmp(token, "off") == 0) {
                g_vmc_params.sync_enable = false;
                printf("Leg sync control DISABLED\n");
            } else if (strcmp(token, "kp") == 0) {
                token = strtok(NULL, " \t\n\r");
                if (token) {
                    g_vmc_params.K_sync = atof(token);
                    printf("Leg sync Kp = %.3f Nm/rad\n", g_vmc_params.K_sync);
                } else {
                    printf("Leg sync Kp = %.3f Nm/rad\n", g_vmc_params.K_sync);
                    printf("Usage: balance vmc sync kp <value>\n");
                }
            } else if (strcmp(token, "kd") == 0) {
                token = strtok(NULL, " \t\n\r");
                if (token) {
                    g_vmc_params.D_sync = atof(token);
                    printf("Leg sync Kd = %.3f Nm·s/rad\n", g_vmc_params.D_sync);
                } else {
                    printf("Leg sync Kd = %.3f Nm·s/rad\n", g_vmc_params.D_sync);
                    printf("Usage: balance vmc sync kd <value>\n");
                }
            } else {
                printf("Unknown vmc sync command: %s\n", token);
                printf("Usage: balance vmc sync [on|off|kp|kd]\n");
            }
        } else if (strcmp(token, "stream") == 0) {
            // VMC 数据流输出控制
            token = strtok(NULL, " \t\n\r");
            if (token == NULL) {
                printf("VMC stream: %s\n", g_vmc_stream_enable ? "ENABLED" : "DISABLED");
                printf("Format: #VMC,L_len,L_ang,L_FL,L_Fa,L_hip,L_knee,R_len,R_ang,R_FL,R_Fa,R_hip,R_knee,diff,Fsync\n");
                printf("Usage: balance vmc stream [on|off]\n");
            } else if (strcmp(token, "on") == 0 || strcmp(token, "1") == 0) {
                g_vmc_stream_enable = true;
                printf("VMC stream ENABLED\n");
            } else if (strcmp(token, "off") == 0 || strcmp(token, "0") == 0) {
                g_vmc_stream_enable = false;
                printf("VMC stream DISABLED\n");
            } else {
                printf("Unknown vmc stream command: %s\n", token);
                printf("Usage: balance vmc stream [on|off]\n");
            }
        } else if (strcmp(token, "diff") == 0) {
            // VMC 速度估计方法切换
            token = strtok(NULL, " \t\n\r");
            if (token == NULL) {
                printf("VMC diff method: %s\n",
                       g_vmc_params.vmc_diff_method == VMC_DIFF_JACOBIAN ? "Jacobian (关节速度)" : "Numeric (数值微分)");
                printf("Sync diff method: %s\n",
                       g_vmc_params.sync_diff_method == VMC_DIFF_JACOBIAN ? "Jacobian" : "Numeric");
                printf("Usage: balance vmc diff [jacobian|numeric]\n");
            } else if (strcasecmp(token, "jacobian") == 0 || strcmp(token, "0") == 0) {
                g_vmc_params.vmc_diff_method = VMC_DIFF_JACOBIAN;
                g_vmc_params.sync_diff_method = VMC_DIFF_JACOBIAN;
                printf("VMC diff method = Jacobian (关节速度 → 雅可比映射)\n");
            } else if (strcasecmp(token, "numeric") == 0 || strcmp(token, "1") == 0) {
                g_vmc_params.vmc_diff_method = VMC_DIFF_NUMERIC;
                g_vmc_params.sync_diff_method = VMC_DIFF_NUMERIC;
                printf("VMC diff method = Numeric (位置数值微分, 使用实际dt)\n");
            } else {
                printf("Unknown diff method: %s\n", token);
                printf("Usage: balance vmc diff [jacobian|numeric]\n");
            }
        } else {
            printf("Unknown vmc command: %s\n", token);
            printf("Usage: balance vmc [on|off|status|coord|soft|stiff|pitch|sync|stream|diff]\n");
            printf("  World coord: kvx|ky|dy\n");
            printf("  Body coord:  kl|dl|ka|da\n");
            printf("  Common:      gc|mass|height|vx\n");
            printf("  Leg sync:    sync [on|off|kp|kd]\n");
            printf("  Stream:      stream [on|off]  (for UI debug)\n");
            printf("  Diff method: diff [jacobian|numeric]  (speed estimation)\n");
        }
    }
    // ===== 关节电机数据流 & 速度滤波 =====
    else if (strcmp(token, "joint") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            printf("Joint stream: %s\n", g_joint_stream_enable ? "ENABLED" : "DISABLED");
            printf("Joint speed filter: %s, mode=%s\n",
                   g_joint_speed_filter_enable ? "ON" : "OFF",
                   g_joint_speed_filter_mode == 0 ? "Median" : "SlewRate");
            if (g_joint_speed_filter_mode == 0) {
                printf("  Median window: %d\n", g_joint_median_window);
            } else {
                printf("  Slew rate: %.0f deg/s^2\n", g_joint_speed_slew_rate);
            }
            printf("Filtered speeds (deg/s): LH=%.1f LK=%.1f RH=%.1f RK=%.1f\n",
                   g_joint_lh_spd_filtered, g_joint_lk_spd_filtered,
                   g_joint_rh_spd_filtered, g_joint_rk_spd_filtered);
            printf("Usage: balance joint [on|off|filter|mode|rate|window]\n");
        } else if (strcmp(token, "stream") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token == NULL) {
                printf("Joint stream: %s\n", g_joint_stream_enable ? "ENABLED" : "DISABLED");
                printf("Format: #JOINT,LH_pos,LH_spd,LH_cur,LH_spd_f,...(x4)\n");
                printf("Usage: balance joint stream [on|off]\n");
            } else if (strcmp(token, "on") == 0 || strcmp(token, "1") == 0) {
                g_joint_stream_enable = true;
                printf("Joint stream ENABLED\n");
            } else if (strcmp(token, "off") == 0 || strcmp(token, "0") == 0) {
                g_joint_stream_enable = false;
                printf("Joint stream DISABLED\n");
            } else {
                printf("Usage: balance joint stream [on|off]\n");
            }
        } else if (strcmp(token, "on") == 0 || strcmp(token, "1") == 0) {
            g_joint_stream_enable = true;
            printf("Joint stream ENABLED\n");
        } else if (strcmp(token, "off") == 0 || strcmp(token, "0") == 0) {
            g_joint_stream_enable = false;
            printf("Joint stream DISABLED\n");
        } else if (strcmp(token, "filter") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token == NULL) {
                printf("Joint speed filter: %s, mode=%s\n",
                       g_joint_speed_filter_enable ? "ON" : "OFF",
                       g_joint_speed_filter_mode == 0 ? "Median" : "SlewRate");
                printf("Usage: balance joint filter [on|off]\n");
            } else if (strcmp(token, "on") == 0 || strcmp(token, "1") == 0) {
                g_joint_speed_filter_enable = true;
                // 重置所有滤波器以避免首次跳变
                slewrate_init(&g_sr_joint_lh, g_joint_speed_slew_rate);
                slewrate_init(&g_sr_joint_lk, g_joint_speed_slew_rate);
                slewrate_init(&g_sr_joint_rh, g_joint_speed_slew_rate);
                slewrate_init(&g_sr_joint_rk, g_joint_speed_slew_rate);
                median_reset(&g_mf_joint_lh);
                median_reset(&g_mf_joint_lk);
                median_reset(&g_mf_joint_rh);
                median_reset(&g_mf_joint_rk);
                printf("Joint speed filter ENABLED (mode=%s)\n",
                       g_joint_speed_filter_mode == 0 ? "Median" : "SlewRate");
            } else if (strcmp(token, "off") == 0 || strcmp(token, "0") == 0) {
                g_joint_speed_filter_enable = false;
                printf("Joint speed filter DISABLED\n");
            } else {
                printf("Usage: balance joint filter [on|off]\n");
            }
        } else if (strcmp(token, "rate") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token == NULL) {
                printf("Joint speed slew rate: %.0f deg/s^2\n", g_joint_speed_slew_rate);
                printf("Usage: balance joint rate <value>\n");
            } else {
                g_joint_speed_slew_rate = atof(token);
                slewrate_set_max_rate(&g_sr_joint_lh, g_joint_speed_slew_rate);
                slewrate_set_max_rate(&g_sr_joint_lk, g_joint_speed_slew_rate);
                slewrate_set_max_rate(&g_sr_joint_rh, g_joint_speed_slew_rate);
                slewrate_set_max_rate(&g_sr_joint_rk, g_joint_speed_slew_rate);
                printf("Joint speed slew rate = %.0f deg/s^2\n", g_joint_speed_slew_rate);
            }
        } else if (strcmp(token, "mode") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token == NULL) {
                printf("Joint filter mode: %d (%s)\n", g_joint_speed_filter_mode,
                       g_joint_speed_filter_mode == 0 ? "Median" : "SlewRate");
                printf("Usage: balance joint mode [0|1|median|slew]\n");
            } else if (strcmp(token, "0") == 0 || strcasecmp(token, "median") == 0) {
                g_joint_speed_filter_mode = 0;
                median_reset(&g_mf_joint_lh);
                median_reset(&g_mf_joint_lk);
                median_reset(&g_mf_joint_rh);
                median_reset(&g_mf_joint_rk);
                printf("Joint filter mode = Median (window=%d)\n", g_joint_median_window);
            } else if (strcmp(token, "1") == 0 || strcasecmp(token, "slew") == 0) {
                g_joint_speed_filter_mode = 1;
                slewrate_init(&g_sr_joint_lh, g_joint_speed_slew_rate);
                slewrate_init(&g_sr_joint_lk, g_joint_speed_slew_rate);
                slewrate_init(&g_sr_joint_rh, g_joint_speed_slew_rate);
                slewrate_init(&g_sr_joint_rk, g_joint_speed_slew_rate);
                printf("Joint filter mode = SlewRate (rate=%.0f)\n", g_joint_speed_slew_rate);
            } else {
                printf("Usage: balance joint mode [0|1|median|slew]\n");
            }
        } else if (strcmp(token, "window") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token == NULL) {
                printf("Median window: %d\n", g_joint_median_window);
                printf("Usage: balance joint window <3|5|7|9>\n");
            } else {
                int w = atoi(token);
                if (w >= 3 && w <= MEDIAN_FILTER_MAX_WINDOW) {
                    g_joint_median_window = w;
                    median_set_window(&g_mf_joint_lh, w);
                    median_set_window(&g_mf_joint_lk, w);
                    median_set_window(&g_mf_joint_rh, w);
                    median_set_window(&g_mf_joint_rk, w);
                    printf("Median window = %d\n", g_joint_median_window);
                } else {
                    printf("Invalid window size (must be odd, 3~%d)\n", MEDIAN_FILTER_MAX_WINDOW);
                }
            }
        } else {
            printf("Unknown joint command: %s\n", token);
            printf("Usage: balance joint [on|off|stream|filter|mode|rate|window]\n");
        }
    }
    // ===== 电机功率数据流 =====
    else if (strcmp(token, "mpow") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            printf("Motor power stream: %s\n", g_mpow_stream_enable ? "ENABLED" : "DISABLED");
            printf("Format: #MPOW,LH_cur,LK_cur,LW_cur,RH_cur,RK_cur,RW_cur,LH_vol,LK_vol,LW_vol,RH_vol,RK_vol,RW_vol\n");
            printf("Usage: balance mpow [on|off]\n");
        } else if (strcmp(token, "on") == 0 || strcmp(token, "1") == 0) {
            g_mpow_stream_enable = true;
            printf("Motor power stream ENABLED\n");
        } else if (strcmp(token, "off") == 0 || strcmp(token, "0") == 0) {
            g_mpow_stream_enable = false;
            printf("Motor power stream DISABLED\n");
        } else {
            printf("Usage: balance mpow [on|off]\n");
        }
    }
    // ===== 速度/位移观测器 =====
    else if (strcmp(token, "obsv") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            printf("=== Velocity Observer (KF) ===\n");
            printf("Observer: %s\n", g_observer_enabled ? "ENABLED" : "DISABLED");
            printf("Stream:   %s\n", g_obsv_stream_enable ? "ON" : "OFF");
            printf("Task:     %d ms (%.0f Hz)\n", g_obsv_period_ms, 1000.0f / g_obsv_period_ms);
            printf("KF Q: v=%.4f  a=%.4f\n", g_kf_Q_v, g_kf_Q_a);
            printf("KF R: v=%.2f  a=%.2f\n", g_kf_R_v, g_kf_R_a);
            printf("--- Current values ---\n");
            printf("  v_raw(wheel)   = %.4f m/s\n", g_obsv_wheel_v_raw);
            printf("  v_encoder(comp)= %.4f m/s\n", g_obsv_v_encoder);
            printf("  v_filter(KF)   = %.4f m/s\n", g_obsv_v_filter);
            printf("  x_filter       = %.4f m\n",   g_obsv_x_filter);
            printf("  a_imu          = %.4f m/s2\n", g_obsv_a_imu);
            printf("Usage: balance obsv [on|off|stream|qv|qa|rv|ra|hz|reset]\n");
        } else if (strcmp(token, "on") == 0 || strcmp(token, "1") == 0) {
            g_observer_enabled = true;
            printf("Velocity observer ENABLED (KF replaces raw speed/distance)\n");
        } else if (strcmp(token, "off") == 0 || strcmp(token, "0") == 0) {
            g_observer_enabled = false;
            printf("Velocity observer DISABLED (using raw wheel speed)\n");
        } else if (strcmp(token, "stream") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token == NULL) {
                printf("Observer stream: %s\n", g_obsv_stream_enable ? "ON" : "OFF");
                printf("Usage: balance obsv stream [on|off]\n");
            } else if (strcmp(token, "on") == 0 || strcmp(token, "1") == 0) {
                g_obsv_stream_enable = true;
                printf("Observer stream ON\n");
            } else if (strcmp(token, "off") == 0 || strcmp(token, "0") == 0) {
                g_obsv_stream_enable = false;
                printf("Observer stream OFF\n");
            }
        } else if (strcmp(token, "qv") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) { g_kf_Q_v = atof(token); printf("KF Q_v = %.4f\n", g_kf_Q_v); }
            else { printf("KF Q_v = %.4f\nUsage: balance obsv qv <value>\n", g_kf_Q_v); }
        } else if (strcmp(token, "qa") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) { g_kf_Q_a = atof(token); printf("KF Q_a = %.4f\n", g_kf_Q_a); }
            else { printf("KF Q_a = %.4f\nUsage: balance obsv qa <value>\n", g_kf_Q_a); }
        } else if (strcmp(token, "rv") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) { g_kf_R_v = atof(token); printf("KF R_v = %.2f\n", g_kf_R_v); }
            else { printf("KF R_v = %.2f\nUsage: balance obsv rv <value>\n", g_kf_R_v); }
        } else if (strcmp(token, "ra") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) { g_kf_R_a = atof(token); printf("KF R_a = %.2f\n", g_kf_R_a); }
            else { printf("KF R_a = %.2f\nUsage: balance obsv ra <value>\n", g_kf_R_a); }
        } else if (strcmp(token, "reset") == 0) {
            kf_observer_reset();
            printf("Observer state reset\n");
        } else if (strcmp(token, "hz") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                int ms = atoi(token);
                if (ms >= 1 && ms <= 20) {
                    g_obsv_period_ms = ms;
                    printf("Observer period = %d ms (%.0f Hz)\n", ms, 1000.0f / ms);
                } else {
                    printf("Invalid period (1-20 ms)\n");
                }
            } else {
                printf("Observer period = %d ms (%.0f Hz)\nUsage: balance obsv hz <ms>\n",
                       g_obsv_period_ms, 1000.0f / g_obsv_period_ms);
            }
        } else {
            printf("Unknown obsv command: %s\n", token);
            printf("Usage: balance obsv [on|off|stream|qv|qa|rv|ra|hz|reset]\n");
        }
    }
    // ===== 树莓派通信开关 =====
    else if (strcmp(token, "picomm") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            printf("Pi comm: %s\n", g_pi_comm_enabled ? "enabled" : "disabled");
            printf("Usage: balance picomm [on|off]\n");
        } else if (strcmp(token, "on") == 0 || strcmp(token, "1") == 0) {
            if (!g_pi_comm_enabled) {
                g_pi_comm_enabled = true;
                // 如果系统已初始化，立即初始化 pi_comm
                if (g_initialized) {
                    esp_err_t ret = pi_comm_init();
                    if (ret == ESP_OK) {
                        printf("Pi comm enabled and initialized\n");
                    } else {
                        printf("Pi comm enabled but init failed: %s\n", esp_err_to_name(ret));
                    }
                } else {
                    printf("Pi comm enabled (will init on balance init)\n");
                }
            } else {
                printf("Pi comm already enabled\n");
            }
        } else if (strcmp(token, "off") == 0 || strcmp(token, "0") == 0) {
            g_pi_comm_enabled = false;
            printf("Pi comm disabled\n");
        } else {
            printf("Unknown picomm command: %s\n", token);
            printf("Usage: balance picomm [on|off]\n");
        }
    }
    // ===== 电机零点设置命令 =====
    else if (strcmp(token, "mzero") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            printf("Set motor position as origin (zero point)\n");
            printf("Usage: balance mzero [left|right|all|<motor_id>]\n");
            printf("  left  - Set left wheel (ID=3) origin\n");
            printf("  right - Set right wheel (ID=6) origin\n");
            printf("  all   - Set both wheels origin\n");
            printf("  <1-6> - Set specific motor origin by ID\n");
        } else if (strcmp(token, "left") == 0) {
            if (g_motor_left) {
                esp_err_t ret = can_motor_set_origin(g_motor_left);
                printf("Left wheel origin set: %s\n", ret == ESP_OK ? "OK" : "FAILED");
            } else {
                printf("Left wheel motor not initialized\n");
            }
        } else if (strcmp(token, "right") == 0) {
            if (g_motor_right) {
                esp_err_t ret = can_motor_set_origin(g_motor_right);
                printf("Right wheel origin set: %s\n", ret == ESP_OK ? "OK" : "FAILED");
            } else {
                printf("Right wheel motor not initialized\n");
            }
        } else if (strcmp(token, "all") == 0) {
            bool ok = true;
            if (g_motor_left) {
                if (can_motor_set_origin(g_motor_left) != ESP_OK) ok = false;
                vTaskDelay(pdMS_TO_TICKS(10));
            }
            if (g_motor_right) {
                if (can_motor_set_origin(g_motor_right) != ESP_OK) ok = false;
            }
            printf("Both wheels origin set: %s\n", ok ? "OK" : "FAILED");
        } else {
            // 尝试解析为电机ID
            int motor_id = atoi(token);
            if (motor_id >= 1 && motor_id <= 6) {
                can_motor_handle_t motor = can_motor_create(motor_id);
                if (motor) {
                    esp_err_t ret = can_motor_set_origin(motor);
                    printf("Motor %d origin set: %s\n", motor_id, ret == ESP_OK ? "OK" : "FAILED");
                    can_motor_destroy(motor);
                } else {
                    printf("Failed to access motor %d\n", motor_id);
                }
            } else {
                printf("Invalid motor ID: %s (must be 1-6)\n", token);
            }
        }
    }
    // ===== 环路使能控制命令 =====
    else if (strcmp(token, "loop") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            balance_test_print_loop_status();
            printf("Usage: balance loop [status|simple|full|none|<mask>]\n");
            printf("       balance loop <A|B|C|D|H|Y> [on|off|<0.0-1.0>]\n");
            printf("  Loops: A=Angle B=Gyro C=Dist D=Speed H=LQR_U Y=Yaw\n");
        } else if (strcmp(token, "status") == 0) {
            balance_test_print_loop_status();
        } else if (strcmp(token, "simple") == 0) {
            balance_test_set_loop_enable(LOOP_SIMPLE);
            printf("Simple balance mode: Angle + Gyro only\n");
        } else if (strcmp(token, "full") == 0) {
            balance_test_set_loop_enable(LOOP_FULL);
            printf("Full balance mode: All loops enabled\n");
        } else if (strcmp(token, "none") == 0) {
            balance_test_set_loop_enable(LOOP_NONE);
            printf("All loops DISABLED (motor will output 0)\n");
        } else if (token[0] == '0' && token[1] == 'x') {
            // 十六进制掩码 (如 0x03)
            uint8_t mask = (uint8_t)strtol(token, NULL, 16);
            balance_test_set_loop_enable(mask);
        } else if (strlen(token) == 1) {
            // 单个环路控制 (A/B/C/D/H/Y)
            loop_enable_mask_t loop = LOOP_NONE;
            switch (token[0]) {
                case 'A': case 'a': loop = LOOP_ANGLE; break;
                case 'B': case 'b': loop = LOOP_GYRO; break;
                case 'C': case 'c': loop = LOOP_DISTANCE; break;
                case 'D': case 'd': loop = LOOP_SPEED; break;
                case 'H': case 'h': loop = LOOP_LQR_U; break;
                case 'Y': case 'y': loop = LOOP_YAW; break;
                default:
                    printf("Unknown loop: %s\n", token);
                    printf("Valid loops: A=Angle B=Gyro C=Dist D=Speed H=LQR_U Y=Yaw\n");
                    return;
            }
            
            // 获取 on/off/value
            token = strtok(NULL, " \t\n\r");
            if (token == NULL) {
                // 显示当前状态
                float gain = balance_test_get_loop_gain(loop);
                printf("Loop %c: %.2f (%s)\n", 
                       "ABCDHY"[__builtin_ctz(loop)], 
                       gain, gain > 0.5f ? "ON" : "OFF");
            } else if (strcmp(token, "on") == 0) {
                balance_test_set_loop_gain(loop, 1.0f);
            } else if (strcmp(token, "off") == 0) {
                balance_test_set_loop_gain(loop, 0.0f);
            } else {
                float val = atof(token);
                balance_test_set_loop_gain(loop, val);
            }
        } else {
            printf("Unknown loop command: %s\n", token);
            printf("Usage: balance loop [status|simple|full|none|0x<mask>]\n");
            printf("       balance loop <A|B|C|D|H|Y> [on|off|<0.0-1.0>]\n");
        }
    }
    // ===== 任务架构切换命令 =====
    else if (strcmp(token, "task") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            printf("Task architecture: %s\n", g_use_unified_task ? "UNIFIED" : "SEPARATE");
            printf("  UNIFIED:  IMU + Ctrl + Motor in ONE task (lower latency)\n");
            printf("  SEPARATE: IMU, Ctrl, Motor in separate tasks (default)\n");
            printf("Usage: balance task [unified|separate]\n");
            printf("Note: Must 'balance stop' first, then change, then 'balance start'\n");
        } else if (strcmp(token, "unified") == 0 || strcmp(token, "1") == 0) {
            if (g_tasks_running) {
                printf("Error: Stop tasks first with 'balance stop'\n");
            } else {
                g_use_unified_task = true;
                printf("Task architecture set to UNIFIED\n");
                printf("  Run 'balance start' to apply\n");
            }
        } else if (strcmp(token, "separate") == 0 || strcmp(token, "0") == 0) {
            if (g_tasks_running) {
                printf("Error: Stop tasks first with 'balance stop'\n");
            } else {
                g_use_unified_task = false;
                printf("Task architecture set to SEPARATE (default)\n");
                printf("  Run 'balance start' to apply\n");
            }
        } else {
            printf("Unknown task command: %s\n", token);
            printf("Usage: balance task [unified|separate]\n");
        }
    }
    // ===== 失控检测开关命令 =====
    else if (strcmp(token, "safety") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            printf("Safety (uncontrolable check): %s\n", g_uncontrolable_check_enabled ? "ENABLED" : "DISABLED");
            printf("  ENABLED:  Pitch > 45° triggers emergency mode (simple balance)\n");
            printf("  DISABLED: No pitch limit check, always full balance mode\n");
            printf("Usage: balance safety [on|off]\n");
        } else if (strcmp(token, "on") == 0 || strcmp(token, "1") == 0 || strcmp(token, "enable") == 0) {
            g_uncontrolable_check_enabled = true;
            g_uncontrolable = 0;  // 清除失控状态
            printf("Safety check ENABLED\n");
        } else if (strcmp(token, "off") == 0 || strcmp(token, "0") == 0 || strcmp(token, "disable") == 0) {
            g_uncontrolable_check_enabled = false;
            g_uncontrolable = 0;  // 清除失控状态
            printf("Safety check DISABLED - use with caution!\n");
        } else {
            printf("Unknown safety command: %s\n", token);
            printf("Usage: balance safety [on|off]\n");
        }
    }
    // ===== YAW 强制使能命令 =====
    else if (strcmp(token, "yaw") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            printf("YAW force enable: %s\n", g_yaw_force_enable ? "ON" : "OFF");
            printf("YAW loop enabled: %s\n", g_yaw_control_enabled ? "YES" : "NO");
            printf("TPID yaw scale:  %.1f\n", g_tpid_yaw_scale);
            printf("  When FORCE ON, YAW works regardless of WiFi yaw_enable\n");
            printf("Usage: balance yaw [on|off|scale <value>]\n");
        } else if (strcmp(token, "on") == 0 || strcmp(token, "1") == 0 || strcmp(token, "enable") == 0) {
            g_yaw_force_enable = true;
            printf("YAW force enable ON - YAW active without remote\n");
        } else if (strcmp(token, "off") == 0 || strcmp(token, "0") == 0 || strcmp(token, "disable") == 0) {
            g_yaw_force_enable = false;
            printf("YAW force enable OFF - YAW controlled by WiFi yaw_enable\n");
        } else if (strcmp(token, "scale") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                g_tpid_yaw_scale = atof(token);
                printf("TPID yaw scale = %.1f\n", g_tpid_yaw_scale);
            } else {
                printf("Current TPID yaw scale = %.1f\n", g_tpid_yaw_scale);
                printf("Usage: balance yaw scale <value>  (default 500)\n");
            }
        } else {
            printf("Unknown yaw command: %s\n", token);
            printf("Usage: balance yaw [on|off|scale <value>]\n");
        }
    }
    // ===== 离地检测命令 =====
    else if (strcmp(token, "airborne") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            printf("=== Wheel Off-Ground Detection ===\n");
            printf("Status: %s\n", g_wheel_off_ground ? "OFF GROUND" : "ON GROUND");
            printf("Speed threshold:  %.1f rad/s (current: L=%.1f R=%.1f)\n",
                   g_lqr_ctrl.params.wheel_off_ground_speed_threshold,
                   g_left_wheel_speed_rad, g_right_wheel_speed_rad);
            printf("Accel threshold:  %.1f rad/s² (current: L=%.1f R=%.1f)\n",
                   g_lqr_ctrl.params.wheel_off_ground_accel_threshold,
                   g_left_wheel_accel, g_right_wheel_accel);
            printf("Usage: balance airborne [speed <val>|accel <val>]\n");
        } else if (strcmp(token, "speed") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                g_lqr_ctrl.params.wheel_off_ground_speed_threshold = atof(token);
                printf("Off-ground speed threshold = %.1f rad/s\n",
                       g_lqr_ctrl.params.wheel_off_ground_speed_threshold);
            }
        } else if (strcmp(token, "accel") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                g_lqr_ctrl.params.wheel_off_ground_accel_threshold = atof(token);
                printf("Off-ground accel threshold = %.1f rad/s²\n",
                       g_lqr_ctrl.params.wheel_off_ground_accel_threshold);
            }
        } else {
            printf("Usage: balance airborne [speed <val>|accel <val>]\n");
        }
    }
    // ===== 控制模式切换命令 =====
    else if (strcmp(token, "mode") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            const char *mode_str = (g_control_mode == CTRL_MODE_LQR) ? "LQR" : 
                                   (g_control_mode == CTRL_MODE_DUAL_PID) ? "DUAL_PID" : 
                                   (g_control_mode == CTRL_MODE_CAR) ? "CAR" :
                                   (g_control_mode == CTRL_MODE_TRIPLE_PID) ? "TRIPLE_PID" :
                                   (g_control_mode == CTRL_MODE_FULL_LQR) ? "FULL_LQR" : "SINGLE_PID";
            printf("Control mode: %s\n", mode_str);
            printf("CTRL_MODE:%s\n", mode_str);
            printf("  LQR:        Multi-loop LQR control (angle+gyro+dist+speed) → torque\n");
            printf("  DUAL_PID:   Dual-loop PID (angle→speed→torque) → torque mode\n");
            printf("  SINGLE_PID: Single-loop PID (angle→speed) → speed mode\n");
            printf("  CAR:        Car mode (no balance, direct speed control)\n");
            printf("  TRIPLE_PID: Triple-loop PID (speed→angle→wheel) → torque/speed mode\n");
            printf("  FULL_LQR:   Full 6-state LQR (T+Tp, K poly-interp by L0) → torque\n");
            printf("Usage: balance mode [lqr|pid|spid|car|tpid|flqr]\n");
        } else if (strcmp(token, "lqr") == 0 || strcmp(token, "0") == 0) {
            g_control_mode = CTRL_MODE_LQR;
            // 如果正在运行，切换电机到扭矩模式
            if (g_state == BALANCE_TEST_RUNNING) {
                can_motor_set_mode(g_motor_left, MODE_TORQUE);
                can_motor_set_mode(g_motor_right, MODE_TORQUE);
            }
            printf("Control mode set to LQR (torque mode)\n");
            printf("CTRL_MODE:LQR\n");
        } else if (strcmp(token, "pid") == 0 || strcmp(token, "dual") == 0 || strcmp(token, "1") == 0) {
            if (!g_dual_pid_initialized) {
                printf("Error: Dual PID controller not initialized\n");
            } else {
                g_control_mode = CTRL_MODE_DUAL_PID;
                dual_pid_reset(&g_dual_pid_ctrl);  // 切换时重置
                // 如果正在运行，切换电机到扭矩模式
                if (g_state == BALANCE_TEST_RUNNING) {
                    can_motor_set_mode(g_motor_left, MODE_TORQUE);
                    can_motor_set_mode(g_motor_right, MODE_TORQUE);
                }
                printf("Control mode set to DUAL_PID (torque mode, %s)\n",
                       g_dual_pid_ctrl.params.loop_order == DUAL_PID_SPEED_FIRST 
                           ? "SPEED_FIRST" : "ANGLE_FIRST");
                printf("CTRL_MODE:DUAL_PID\n");
            }
        } else if (strcmp(token, "spid") == 0 || strcmp(token, "single") == 0 || strcmp(token, "2") == 0) {
            if (!g_single_pid_initialized) {
                printf("Error: Single PID controller not initialized\n");
            } else {
                g_control_mode = CTRL_MODE_SINGLE_PID;
                single_pid_reset(&g_single_pid_ctrl);  // 切换时重置
                // 如果正在运行，切换电机到速度模式
                if (g_state == BALANCE_TEST_RUNNING) {
                    can_motor_set_mode(g_motor_left, MODE_SPEED);
                    can_motor_set_mode(g_motor_right, MODE_SPEED);
                }
                printf("Control mode set to SINGLE_PID (speed mode)\n");
                printf("CTRL_MODE:SINGLE_PID\n");
            }
        } else if (strcmp(token, "car") == 0 || strcmp(token, "3") == 0) {
            // 进入小车模式: 保存当前状态, 设置腿部角度, 切电机速度模式
            g_car_mode_prev_mode = g_control_mode;
            g_car_mode_prev_base_angle = g_leg_base_angle;
            g_car_mode_prev_base_length = g_leg_base_length;
            g_control_mode = CTRL_MODE_CAR;
            
            // 切换电机到速度模式
            if (g_state == BALANCE_TEST_RUNNING) {
                can_motor_set_mode(g_motor_left, MODE_SPEED);
                can_motor_set_mode(g_motor_right, MODE_SPEED);
            }
            
            // 设置腿部趴下姿态
            if (g_leg_control_enabled) {
                leg_ctrl_set_target(true, CAR_MODE_LEG_LENGTH, CAR_MODE_BODY_ANGLE);
                leg_ctrl_set_target(false, CAR_MODE_LEG_LENGTH, CAR_MODE_BODY_ANGLE);
            }
            
            printf("Control mode set to CAR (speed mode, body_angle=%.0f°, leg=%.0fmm)\n",
                   CAR_MODE_BODY_ANGLE, CAR_MODE_LEG_LENGTH * 1000.0f);
            printf("CTRL_MODE:CAR\n");
        } else if (strcmp(token, "tpid") == 0 || strcmp(token, "triple") == 0 || strcmp(token, "4") == 0) {
            if (!g_triple_pid_initialized) {
                printf("Error: Triple PID controller not initialized\n");
            } else {
                g_control_mode = CTRL_MODE_TRIPLE_PID;
                triple_pid_reset(&g_triple_pid_ctrl);
                // 初始化位移零点为当前位置
                triple_pid_set_distance_zeropoint(&g_triple_pid_ctrl, g_lqr_distance);
                g_distance_zeropoint = g_lqr_distance;
                // 根据轮速环模式设置电机模式
                if (g_state == BALANCE_TEST_RUNNING) {
                    if (g_triple_pid_ctrl.params.wheel_mode == TRIPLE_PID_WHEEL_SPEED) {
                        can_motor_set_mode(g_motor_left, MODE_SPEED);
                        can_motor_set_mode(g_motor_right, MODE_SPEED);
                    } else {
                        can_motor_set_mode(g_motor_left, MODE_TORQUE);
                        can_motor_set_mode(g_motor_right, MODE_TORQUE);
                    }
                }
                printf("Control mode set to TRIPLE_PID (%s mode)\n",
                       g_triple_pid_ctrl.params.wheel_mode == TRIPLE_PID_WHEEL_SPEED 
                           ? "speed" : "torque");
                printf("CTRL_MODE:TRIPLE_PID\n");
            }
        } else if (strcmp(token, "flqr") == 0 || strcmp(token, "full_lqr") == 0 || strcmp(token, "5") == 0) {
            if (!g_full_lqr_initialized) {
                printf("Error: Full LQR controller not initialized\n");
            } else {
                g_control_mode = CTRL_MODE_FULL_LQR;
                full_lqr_reset(&g_full_lqr_ctrl);
                g_distance_zeropoint = g_lqr_distance;
                g_full_lqr_Tp_left = 0.0f;
                g_full_lqr_Tp_right = 0.0f;
                // 切换电机到扭矩模式
                if (g_state == BALANCE_TEST_RUNNING) {
                    can_motor_set_mode(g_motor_left, MODE_TORQUE);
                    can_motor_set_mode(g_motor_right, MODE_TORQUE);
                }
                printf("Control mode set to FULL_LQR (6-state LQR, T+Tp, torque mode)\n");
                printf("CTRL_MODE:FULL_LQR\n");
            }
        } else if (strcmp(token, "exit_car") == 0) {
            // 退出小车模式: 恢复之前的模式和腿部姿态
            if (g_control_mode != CTRL_MODE_CAR) {
                printf("Not in car mode\n");
            } else {
                // 先停止轮子
                if (g_state == BALANCE_TEST_RUNNING) {
                    can_motor_set_speed(g_motor_left, 0);
                    can_motor_set_speed(g_motor_right, 0);
                }
                
                // 恢复腿部姿态 (固定 72mm, 不恢复进入 car 前的腿长)
                if (g_leg_control_enabled) {
                    leg_ctrl_set_target(true,  CAR_TO_BAL_LEG_LENGTH, -90.0f);
                    leg_ctrl_set_target(false, CAR_TO_BAL_LEG_LENGTH, -90.0f);
                }
                
                // 恢复控制模式
                g_control_mode = g_car_mode_prev_mode;
                
                // 恢复电机模式
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
                printf("Exited car mode, restored to %s\n", restored_str);
                printf("CTRL_MODE:%s\n", restored_str);
            }
        } else {
            printf("Unknown mode: %s\n", token);
            printf("Usage: balance mode [lqr|pid|spid|car|tpid|flqr|exit_car]\n");
        }
    }
    // ===== 双环 PID 调参命令 =====
    else if (strcmp(token, "dpid") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            // 显示当前参数
            const char *order_str = g_dual_pid_ctrl.params.loop_order == DUAL_PID_SPEED_FIRST
                ? "SPEED_FIRST (速度环外→角度环内)" : "ANGLE_FIRST (角度环外→速度环内)";
            printf("=== Dual PID Parameters ===\n");
            printf("Loop order: %s\n", order_str);
            if (g_dual_pid_ctrl.params.loop_order == DUAL_PID_SPEED_FIRST) {
                printf("Speed PID (outer): kp=%.2f ki=%.2f kd=%.3f limit=%.1f (max pitch target °)\n",
                       g_dual_pid_ctrl.params.speed_kp, g_dual_pid_ctrl.params.speed_ki,
                       g_dual_pid_ctrl.params.speed_kd, g_dual_pid_ctrl.params.speed_limit);
                printf("Angle PID (inner): kp=%.2f ki=%.2f kd=%.3f limit=%.1f (max torque)\n",
                       g_dual_pid_ctrl.params.angle_kp, g_dual_pid_ctrl.params.angle_ki,
                       g_dual_pid_ctrl.params.angle_kd, g_dual_pid_ctrl.params.angle_limit);
            } else {
                printf("Angle PID (outer): kp=%.2f ki=%.2f kd=%.3f limit=%.1f (max target speed)\n",
                       g_dual_pid_ctrl.params.angle_kp, g_dual_pid_ctrl.params.angle_ki,
                       g_dual_pid_ctrl.params.angle_kd, g_dual_pid_ctrl.params.angle_limit);
                printf("Speed PID (inner): kp=%.2f ki=%.2f kd=%.3f limit=%.1f (max torque)\n",
                       g_dual_pid_ctrl.params.speed_kp, g_dual_pid_ctrl.params.speed_ki,
                       g_dual_pid_ctrl.params.speed_kd, g_dual_pid_ctrl.params.speed_limit);
            }
            printf("Angle zeropoint: %.2f deg\n", g_dual_pid_ctrl.params.angle_zeropoint);
            printf("Gyro PID (damping): kp=%.4f ki=%.4f kd=%.6f limit=%.1f\n",
                   g_dual_pid_ctrl.params.gyro_kp, g_dual_pid_ctrl.params.gyro_ki,
                   g_dual_pid_ctrl.params.gyro_kd, g_dual_pid_ctrl.params.gyro_limit);
            printf("Speed cmd gain: %.1f (SPEED_FIRST 外环指令增益)\n", g_dual_pid_ctrl.params.speed_cmd_gain);
            printf("Max torque: %.1f Nm\n", g_dual_pid_ctrl.params.max_torque);
            printf("\nUsage: balance dpid angle <kp> <ki> <kd>\n");
            printf("       balance dpid speed <kp> <ki> <kd>\n");
            printf("       balance dpid gain <value>  (speed_cmd_gain for SPEED_FIRST)\n");
            printf("       balance dpid gyro <kp> <ki> <kd>  (角速度阻尼PID)\n");
            printf("       balance dpid zero <degrees>\n");
            printf("       balance dpid order <0|1>  (0=ANGLE_FIRST, 1=SPEED_FIRST)\n");
            printf("       balance dpid reset\n");
            printf("       balance dpid status\n");
        } else if (strcmp(token, "angle") == 0) {
            // 设置直立环 PID
            float kp = g_dual_pid_ctrl.params.angle_kp;
            float ki = g_dual_pid_ctrl.params.angle_ki;
            float kd = g_dual_pid_ctrl.params.angle_kd;
            
            token = strtok(NULL, " \t\n\r");
            if (token) kp = atof(token);
            token = strtok(NULL, " \t\n\r");
            if (token) ki = atof(token);
            token = strtok(NULL, " \t\n\r");
            if (token) kd = atof(token);
            
            dual_pid_set_angle_gains(&g_dual_pid_ctrl, kp, ki, kd);
            printf("Angle PID set: kp=%.2f ki=%.2f kd=%.3f\n", kp, ki, kd);
            // 输出 Qt 可解析格式
            printf("DPID:ANGLE,%.4f,%.4f,%.4f\n", kp, ki, kd);
        } else if (strcmp(token, "speed") == 0) {
            // 设置速度环 PID
            float kp = g_dual_pid_ctrl.params.speed_kp;
            float ki = g_dual_pid_ctrl.params.speed_ki;
            float kd = g_dual_pid_ctrl.params.speed_kd;
            
            token = strtok(NULL, " \t\n\r");
            if (token) kp = atof(token);
            token = strtok(NULL, " \t\n\r");
            if (token) ki = atof(token);
            token = strtok(NULL, " \t\n\r");
            if (token) kd = atof(token);
            
            dual_pid_set_speed_gains(&g_dual_pid_ctrl, kp, ki, kd);
            printf("Speed PID set: kp=%.2f ki=%.2f kd=%.3f\n", kp, ki, kd);
            // 输出 Qt 可解析格式
            printf("DPID:SPEED,%.4f,%.4f,%.4f\n", kp, ki, kd);
        } else if (strcmp(token, "gain") == 0) {
            // 设置 speed_cmd_gain (SPEED_FIRST 外环指令增益)
            token = strtok(NULL, " \t\n\r");
            if (token) {
                float gain = atof(token);
                g_dual_pid_ctrl.params.speed_cmd_gain = gain;
                printf("Speed cmd gain set to %.1f\n", gain);
                printf("DPID:GAIN,%.4f\n", gain);
            } else {
                printf("Current speed_cmd_gain: %.1f\n", g_dual_pid_ctrl.params.speed_cmd_gain);
                printf("Usage: balance dpid gain <value>\n");
                printf("  在 SPEED_FIRST 模式中, target_speed 会乘以此增益再送入速度外环\n");
            }
        } else if (strcmp(token, "gyro") == 0) {
            // 设置角速度阻尼PID
            float kp = g_dual_pid_ctrl.params.gyro_kp;
            float ki = g_dual_pid_ctrl.params.gyro_ki;
            float kd = g_dual_pid_ctrl.params.gyro_kd;
            token = strtok(NULL, " \t\n\r");
            if (token) kp = atof(token);
            token = strtok(NULL, " \t\n\r");
            if (token) ki = atof(token);
            token = strtok(NULL, " \t\n\r");
            if (token) kd = atof(token);
            dual_pid_set_gyro_gains(&g_dual_pid_ctrl, kp, ki, kd);
            printf("Gyro PID set: kp=%.4f ki=%.4f kd=%.6f\n", kp, ki, kd);
            printf("DPID:GYRO,%.4f,%.4f,%.6f\n", kp, ki, kd);
        } else if (strcmp(token, "zero") == 0) {
            // 设置角度零点 (同步到所有控制器)
            token = strtok(NULL, " \t\n\r");
            if (token) {
                float zero = atof(token);
                balance_test_set_angle_zeropoint(zero);
                printf("Angle zeropoint set to %.2f (synced to all controllers)\n", zero);
            } else {
                printf("Current angle zeropoint: %.2f\n", g_angle_zeropoint);
            }
        } else if (strcmp(token, "reset") == 0) {
            dual_pid_reset(&g_dual_pid_ctrl);
            printf("Dual PID controller reset\n");
        } else if (strcmp(token, "order") == 0) {
            // 设置环路顺序
            token = strtok(NULL, " \t\n\r");
            if (token) {
                int order = atoi(token);
                if (order == 0 || order == 1) {
                    dual_pid_set_loop_order(&g_dual_pid_ctrl, (uint8_t)order);
                    printf("Dual PID loop order set to %s (%d)\n",
                           order == DUAL_PID_SPEED_FIRST ? "SPEED_FIRST (速度环外,角度环内)" 
                                                         : "ANGLE_FIRST (角度环外,速度环内)", order);
                } else {
                    printf("Invalid order. Use 0=ANGLE_FIRST, 1=SPEED_FIRST\n");
                }
            } else {
                printf("Current loop order: %s (%d)\n",
                       g_dual_pid_ctrl.params.loop_order == DUAL_PID_SPEED_FIRST 
                           ? "SPEED_FIRST (速度环外,角度环内)" 
                           : "ANGLE_FIRST (角度环外,速度环内)",
                       g_dual_pid_ctrl.params.loop_order);
                printf("Usage: balance dpid order <0|1>  (0=ANGLE_FIRST, 1=SPEED_FIRST)\n");
            }
        } else if (strcmp(token, "status") == 0) {
            // 显示实时状态
            printf("=== Dual PID Status ===\n");
            printf("Control mode: %s\n", g_control_mode == CTRL_MODE_DUAL_PID ? "ACTIVE" : "INACTIVE");
            printf("Loop order: %s\n", 
                   g_dual_pid_ctrl.params.loop_order == DUAL_PID_SPEED_FIRST 
                       ? "SPEED_FIRST (速度环外→角度环内)" : "ANGLE_FIRST (角度环外→速度环内)");
            if (g_dual_pid_ctrl.params.loop_order == DUAL_PID_SPEED_FIRST) {
                printf("Speed error: %.2f rad/s\n", g_dual_pid_output.speed_error);
                printf("Pitch target: %.2f deg\n", g_dual_pid_output.target_speed);
                printf("Angle error: %.2f deg\n", g_dual_pid_output.angle_error);
            } else {
                printf("Angle error: %.2f deg\n", g_dual_pid_output.angle_error);
                printf("Target speed: %.2f rad/s\n", g_dual_pid_output.target_speed);
                printf("Speed error: %.2f rad/s\n", g_dual_pid_output.speed_error);
            }
            printf("Output torque: %.2f Nm\n", g_dual_pid_output.torque);
            printf("Angle PID: P=%.2f I=%.2f D=%.3f\n",
                   g_dual_pid_output.angle_p_out, g_dual_pid_output.angle_i_out, g_dual_pid_output.angle_d_out);
            printf("Speed PID: P=%.2f I=%.2f D=%.3f\n",
                   g_dual_pid_output.speed_p_out, g_dual_pid_output.speed_i_out, g_dual_pid_output.speed_d_out);
            printf("Gyro PID:  P=%.4f I=%.4f D=%.6f (kp=%.4f ki=%.4f kd=%.6f)\n",
                   g_dual_pid_output.gyro_p_out, g_dual_pid_output.gyro_i_out, g_dual_pid_output.gyro_d_out,
                   g_dual_pid_ctrl.params.gyro_kp, g_dual_pid_ctrl.params.gyro_ki, g_dual_pid_ctrl.params.gyro_kd);
            // 输出 Qt 可解析格式
            printf("DPID_STATUS:PITCH_ERR=%.2f,TGT_SPD=%.2f,SPD_ERR=%.2f,TORQUE=%.2f,ORDER=%d\n",
                   g_dual_pid_output.angle_error, g_dual_pid_output.target_speed,
                   g_dual_pid_output.speed_error, g_dual_pid_output.torque,
                   g_dual_pid_ctrl.params.loop_order);
        } else {
            printf("Unknown dpid command: %s\n", token);
            printf("Usage: balance dpid [angle|speed|gain|gyro|zero|order|reset|status]\n");
        }
    }
    // ===== 单环 PID 调参命令 =====
    else if (strcmp(token, "spid") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            // 显示当前参数
            printf("=== Single PID Parameters (Speed Output Mode) ===\n");
            printf("Angle PID: kp=%.2f ki=%.2f kd=%.3f limit=%.1f rad/s\n",
                   g_single_pid_ctrl.params.angle_kp, g_single_pid_ctrl.params.angle_ki,
                   g_single_pid_ctrl.params.angle_kd, g_single_pid_ctrl.params.angle_limit);
            printf("Angle zeropoint: %.2f deg\n", g_single_pid_ctrl.params.angle_zeropoint);
            printf("Emergency angle: %.1f deg\n", g_single_pid_ctrl.params.emergency_angle);
            printf("\nUsage: balance spid angle <kp> <ki> <kd>\n");
            printf("       balance spid limit <max_speed_rad_s>\n");
            printf("       balance spid zero <degrees>\n");
            printf("       balance spid reset\n");
            printf("       balance spid status\n");
        } else if (strcmp(token, "angle") == 0) {
            // 设置直立环 PID
            float kp = g_single_pid_ctrl.params.angle_kp;
            float ki = g_single_pid_ctrl.params.angle_ki;
            float kd = g_single_pid_ctrl.params.angle_kd;
            
            token = strtok(NULL, " \t\n\r");
            if (token) kp = atof(token);
            token = strtok(NULL, " \t\n\r");
            if (token) ki = atof(token);
            token = strtok(NULL, " \t\n\r");
            if (token) kd = atof(token);
            
            single_pid_set_angle_gains(&g_single_pid_ctrl, kp, ki, kd);
            printf("Single PID Angle set: kp=%.2f ki=%.2f kd=%.3f\n", kp, ki, kd);
            // 输出 Qt 可解析格式
            printf("SPID:ANGLE,%.4f,%.4f,%.4f\n", kp, ki, kd);
        } else if (strcmp(token, "limit") == 0) {
            // 设置输出限幅 (最大速度)
            token = strtok(NULL, " \t\n\r");
            if (token) {
                float limit = atof(token);
                g_single_pid_ctrl.params.angle_limit = limit;
                pid_set_output_limits(&g_single_pid_ctrl.pid_angle, -limit, limit);
                printf("Single PID output limit set to %.1f rad/s (%.1f rpm)\n", limit, limit * 9.5493f);
            } else {
                printf("Current output limit: %.1f rad/s (%.1f rpm)\n", 
                       g_single_pid_ctrl.params.angle_limit,
                       g_single_pid_ctrl.params.angle_limit * 9.5493f);
            }
        } else if (strcmp(token, "zero") == 0) {
            // 设置角度零点 (同步到所有控制器)
            token = strtok(NULL, " \t\n\r");
            if (token) {
                float zero = atof(token);
                balance_test_set_angle_zeropoint(zero);
                printf("Angle zeropoint set to %.2f (synced to all controllers)\n", zero);
            } else {
                printf("Current angle zeropoint: %.2f\n", g_angle_zeropoint);
            }
        } else if (strcmp(token, "reset") == 0) {
            single_pid_reset(&g_single_pid_ctrl);
            printf("Single PID controller reset\n");
        } else if (strcmp(token, "status") == 0) {
            // 显示实时状态
            printf("=== Single PID Status ===\n");
            printf("Control mode: %s\n", g_control_mode == CTRL_MODE_SINGLE_PID ? "ACTIVE" : "INACTIVE");
            printf("Angle error: %.2f deg\n", g_single_pid_output.angle_error);
            printf("Output speed: %.2f rad/s (%.1f rpm)\n", 
                   g_single_pid_output.target_speed,
                   g_single_pid_output.target_speed * 9.5493f);
            printf("Angle PID: P=%.2f I=%.2f D=%.3f\n",
                   g_single_pid_output.angle_p_out, g_single_pid_output.angle_i_out, g_single_pid_output.angle_d_out);
            // 输出 Qt 可解析格式
            printf("SPID_STATUS:PITCH_ERR=%.2f,TGT_SPD=%.2f\n",
                   g_single_pid_output.angle_error, g_single_pid_output.target_speed);
        } else {
            printf("Unknown spid command: %s\n", token);
            printf("Usage: balance spid [angle|limit|zero|reset|status]\n");
        }
    }
    // ===== 三环 PID 调参命令 =====
    else if (strcmp(token, "tpid") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            const char *wm_str = g_triple_pid_ctrl.params.wheel_mode == TRIPLE_PID_WHEEL_SPEED
                ? "SPEED_CMD (电机速度模式)" : "TORQUE_PID (软件PID→扭矩)";
            printf("=== Triple PID Parameters ===\n");
            printf("Architecture: Speed(outer) → Angle(mid) → Wheel(inner)\n");
            printf("Wheel mode: %s\n", wm_str);
            printf("Speed PID (outer): kp=%.6f ki=%.6f kd=%.6f limit=%.1f\n",
                   g_triple_pid_ctrl.params.speed_kp, g_triple_pid_ctrl.params.speed_ki,
                   g_triple_pid_ctrl.params.speed_kd, g_triple_pid_ctrl.params.speed_limit);
            printf("Angle PID (mid):   kp=%.6f ki=%.6f kd=%.6f limit=%.1f\n",
                   g_triple_pid_ctrl.params.angle_kp, g_triple_pid_ctrl.params.angle_ki,
                   g_triple_pid_ctrl.params.angle_kd, g_triple_pid_ctrl.params.angle_limit);
            printf("Wheel PID (inner): kp=%.4f ki=%.4f kd=%.4f limit=%.1f\n",
                   g_triple_pid_ctrl.params.wheel_kp, g_triple_pid_ctrl.params.wheel_ki,
                   g_triple_pid_ctrl.params.wheel_kd, g_triple_pid_ctrl.params.wheel_limit);
            printf("Gyro PID (damping): kp=%.4f ki=%.4f kd=%.6f limit=%.1f\n",
                   g_triple_pid_ctrl.params.gyro_kp, g_triple_pid_ctrl.params.gyro_ki,
                   g_triple_pid_ctrl.params.gyro_kd, g_triple_pid_ctrl.params.gyro_limit);
            printf("Angle zeropoint: %.2f deg\n", g_triple_pid_ctrl.params.angle_zeropoint);
            printf("Speed cmd gain: %.1f\n", g_triple_pid_ctrl.params.speed_cmd_gain);
            printf("Max torque: %.1f Nm\n", g_triple_pid_ctrl.params.max_torque);
            printf("Distance PID: kp=%.4f ki=%.4f kd=%.4f (limit=%.1f) [%s]\n",
                   g_triple_pid_ctrl.params.distance_kp, g_triple_pid_ctrl.params.distance_ki,
                   g_triple_pid_ctrl.params.distance_kd, g_triple_pid_ctrl.params.distance_limit,
                   g_triple_pid_ctrl.params.distance_enable ? "启用" : "关闭");
            printf("\nUsage: balance tpid angle <kp> <ki> <kd>\n");
            printf("       balance tpid speed <kp> <ki> <kd>\n");
            printf("       balance tpid wheel <kp> <ki> <kd>\n");
            printf("       balance tpid gyro <kp> <ki> <kd>  (角速度阻尼PID)\n");
            printf("       balance tpid distance <kp> <ki> <kd>  (位移环PID)\n");
            printf("       balance tpid disten <0|1>  (位移环开关)\n");
            printf("       balance tpid limit <speed|angle|wheel|gyro|distance> <value>\n");
            printf("       balance tpid gain <value>  (speed_cmd_gain)\n");
            printf("       balance tpid wmode <0|1>  (0=SPEED_CMD, 1=TORQUE_PID)\n");
            printf("       balance tpid zero <degrees>\n");
            printf("       balance tpid reset\n");
            printf("       balance tpid status\n");
        } else if (strcmp(token, "angle") == 0) {
            float kp = g_triple_pid_ctrl.params.angle_kp;
            float ki = g_triple_pid_ctrl.params.angle_ki;
            float kd = g_triple_pid_ctrl.params.angle_kd;
            token = strtok(NULL, " \t\n\r");
            if (token) kp = atof(token);
            token = strtok(NULL, " \t\n\r");
            if (token) ki = atof(token);
            token = strtok(NULL, " \t\n\r");
            if (token) kd = atof(token);
            triple_pid_set_angle_gains(&g_triple_pid_ctrl, kp, ki, kd);
            printf("Triple PID Angle set: kp=%.6f ki=%.6f kd=%.6f\n", kp, ki, kd);
            printf("TPID:ANGLE,%.6f,%.6f,%.6f\n", kp, ki, kd);
        } else if (strcmp(token, "speed") == 0) {
            float kp = g_triple_pid_ctrl.params.speed_kp;
            float ki = g_triple_pid_ctrl.params.speed_ki;
            float kd = g_triple_pid_ctrl.params.speed_kd;
            token = strtok(NULL, " \t\n\r");
            if (token) kp = atof(token);
            token = strtok(NULL, " \t\n\r");
            if (token) ki = atof(token);
            token = strtok(NULL, " \t\n\r");
            if (token) kd = atof(token);
            triple_pid_set_speed_gains(&g_triple_pid_ctrl, kp, ki, kd);
            printf("Triple PID Speed set: kp=%.6f ki=%.6f kd=%.6f\n", kp, ki, kd);
            printf("TPID:SPEED,%.6f,%.6f,%.6f\n", kp, ki, kd);
        } else if (strcmp(token, "wheel") == 0) {
            float kp = g_triple_pid_ctrl.params.wheel_kp;
            float ki = g_triple_pid_ctrl.params.wheel_ki;
            float kd = g_triple_pid_ctrl.params.wheel_kd;
            token = strtok(NULL, " \t\n\r");
            if (token) kp = atof(token);
            token = strtok(NULL, " \t\n\r");
            if (token) ki = atof(token);
            token = strtok(NULL, " \t\n\r");
            if (token) kd = atof(token);
            triple_pid_set_wheel_gains(&g_triple_pid_ctrl, kp, ki, kd);
            printf("Triple PID Wheel set: kp=%.4f ki=%.4f kd=%.4f\n", kp, ki, kd);
            printf("TPID:WHEEL,%.4f,%.4f,%.4f\n", kp, ki, kd);
        } else if (strcmp(token, "gyro") == 0) {
            // 设置角速度阻尼PID
            float kp = g_triple_pid_ctrl.params.gyro_kp;
            float ki = g_triple_pid_ctrl.params.gyro_ki;
            float kd = g_triple_pid_ctrl.params.gyro_kd;
            token = strtok(NULL, " \t\n\r");
            if (token) kp = atof(token);
            token = strtok(NULL, " \t\n\r");
            if (token) ki = atof(token);
            token = strtok(NULL, " \t\n\r");
            if (token) kd = atof(token);
            triple_pid_set_gyro_gains(&g_triple_pid_ctrl, kp, ki, kd);
            printf("Triple PID Gyro set: kp=%.4f ki=%.4f kd=%.6f\n", kp, ki, kd);
            printf("TPID:GYRO,%.4f,%.4f,%.6f\n", kp, ki, kd);
        } else if (strcmp(token, "limit") == 0) {
            // 设置各环输出限幅
            token = strtok(NULL, " \t\n\r");
            if (token == NULL) {
                printf("=== Triple PID Limits ===\n");
                printf("Speed limit:  %.1f (max pitch target, deg)\n", g_triple_pid_ctrl.params.speed_limit);
                printf("Angle limit:  %.1f (max wheel_speed_target, rad/s)\n", g_triple_pid_ctrl.params.angle_limit);
                printf("Wheel limit:  %.1f (max torque, Nm)\n", g_triple_pid_ctrl.params.wheel_limit);
                printf("Gyro limit:   %.1f\n", g_triple_pid_ctrl.params.gyro_limit);
                printf("Distance limit: %.1f (max speed correction, rad/s)\n", g_triple_pid_ctrl.params.distance_limit);
                printf("TPID:LIMIT,%.1f,%.1f,%.1f,%.1f,%.1f\n",
                       g_triple_pid_ctrl.params.speed_limit, g_triple_pid_ctrl.params.angle_limit,
                       g_triple_pid_ctrl.params.wheel_limit, g_triple_pid_ctrl.params.gyro_limit,
                       g_triple_pid_ctrl.params.distance_limit);
                printf("Usage: balance tpid limit <speed|angle|wheel|gyro|distance> <value>\n");
            } else if (strcmp(token, "speed") == 0) {
                token = strtok(NULL, " \t\n\r");
                if (token) {
                    float limit = atof(token);
                    g_triple_pid_ctrl.params.speed_limit = limit;
                    pid_set_output_limits(&g_triple_pid_ctrl.pid_speed, -limit, limit);
                    printf("Triple PID speed limit set to %.1f\n", limit);
                    printf("TPID:LIMIT,%.1f,%.1f,%.1f,%.1f\n",
                           g_triple_pid_ctrl.params.speed_limit, g_triple_pid_ctrl.params.angle_limit,
                           g_triple_pid_ctrl.params.wheel_limit, g_triple_pid_ctrl.params.gyro_limit);
                }
            } else if (strcmp(token, "angle") == 0) {
                token = strtok(NULL, " \t\n\r");
                if (token) {
                    float limit = atof(token);
                    g_triple_pid_ctrl.params.angle_limit = limit;
                    pid_set_output_limits(&g_triple_pid_ctrl.pid_angle, -limit, limit);
                    printf("Triple PID angle limit set to %.1f\n", limit);
                    printf("TPID:LIMIT,%.1f,%.1f,%.1f,%.1f\n",
                           g_triple_pid_ctrl.params.speed_limit, g_triple_pid_ctrl.params.angle_limit,
                           g_triple_pid_ctrl.params.wheel_limit, g_triple_pid_ctrl.params.gyro_limit);
                }
            } else if (strcmp(token, "wheel") == 0) {
                token = strtok(NULL, " \t\n\r");
                if (token) {
                    float limit = atof(token);
                    g_triple_pid_ctrl.params.wheel_limit = limit;
                    pid_set_output_limits(&g_triple_pid_ctrl.pid_wheel, -limit, limit);
                    printf("Triple PID wheel limit set to %.1f\n", limit);
                    printf("TPID:LIMIT,%.1f,%.1f,%.1f,%.1f\n",
                           g_triple_pid_ctrl.params.speed_limit, g_triple_pid_ctrl.params.angle_limit,
                           g_triple_pid_ctrl.params.wheel_limit, g_triple_pid_ctrl.params.gyro_limit);
                }
            } else if (strcmp(token, "gyro") == 0) {
                token = strtok(NULL, " \t\n\r");
                if (token) {
                    float limit = atof(token);
                    g_triple_pid_ctrl.params.gyro_limit = limit;
                    pid_set_output_limits(&g_triple_pid_ctrl.pid_gyro, -limit, limit);
                    printf("Triple PID gyro limit set to %.1f\n", limit);
                    printf("TPID:LIMIT,%.1f,%.1f,%.1f,%.1f\n",
                           g_triple_pid_ctrl.params.speed_limit, g_triple_pid_ctrl.params.angle_limit,
                           g_triple_pid_ctrl.params.wheel_limit, g_triple_pid_ctrl.params.gyro_limit);
                }
            } else if (strcmp(token, "distance") == 0 || strcmp(token, "dist") == 0) {
                token = strtok(NULL, " \t\n\r");
                if (token) {
                    float limit = atof(token);
                    g_triple_pid_ctrl.params.distance_limit = limit;
                    pid_set_output_limits(&g_triple_pid_ctrl.pid_distance, -limit, limit);
                    printf("Triple PID distance limit set to %.1f\n", limit);
                    printf("TPID:LIMIT,%.1f,%.1f,%.1f,%.1f,%.1f\n",
                           g_triple_pid_ctrl.params.speed_limit, g_triple_pid_ctrl.params.angle_limit,
                           g_triple_pid_ctrl.params.wheel_limit, g_triple_pid_ctrl.params.gyro_limit,
                           g_triple_pid_ctrl.params.distance_limit);
                }
            } else {
                printf("Unknown limit target: %s\n", token);
                printf("Usage: balance tpid limit <speed|angle|wheel|gyro|distance> <value>\n");
            }
        } else if (strcmp(token, "gain") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                float gain = atof(token);
                g_triple_pid_ctrl.params.speed_cmd_gain = gain;
                printf("Triple PID speed cmd gain set to %.1f\n", gain);
                printf("TPID:GAIN,%.4f\n", gain);
            } else {
                printf("Current speed_cmd_gain: %.1f\n", g_triple_pid_ctrl.params.speed_cmd_gain);
            }
        } else if (strcmp(token, "wmode") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                int mode = atoi(token);
                if (mode == 0 || mode == 1) {
                    triple_pid_set_wheel_mode(&g_triple_pid_ctrl, (uint8_t)mode);
                    // 如果当前是三环PID模式且正在运行，实时切换电机模式
                    if (g_control_mode == CTRL_MODE_TRIPLE_PID && g_state == BALANCE_TEST_RUNNING) {
                        if (mode == TRIPLE_PID_WHEEL_SPEED) {
                            can_motor_set_mode(g_motor_left, MODE_SPEED);
                            can_motor_set_mode(g_motor_right, MODE_SPEED);
                        } else {
                            can_motor_set_mode(g_motor_left, MODE_TORQUE);
                            can_motor_set_mode(g_motor_right, MODE_TORQUE);
                        }
                    }
                    printf("Triple PID wheel mode set to %s (%d)\n",
                           mode == TRIPLE_PID_WHEEL_SPEED ? "SPEED_CMD (电机速度模式)" 
                                                          : "TORQUE_PID (软件PID→扭矩)", mode);
                } else {
                    printf("Invalid mode. Use 0=SPEED_CMD, 1=TORQUE_PID\n");
                }
            } else {
                printf("Current wheel mode: %s (%d)\n",
                       g_triple_pid_ctrl.params.wheel_mode == TRIPLE_PID_WHEEL_SPEED 
                           ? "SPEED_CMD (电机速度模式)" : "TORQUE_PID (软件PID→扭矩)",
                       g_triple_pid_ctrl.params.wheel_mode);
                printf("Usage: balance tpid wmode <0|1>  (0=SPEED_CMD, 1=TORQUE_PID)\n");
            }
        } else if (strcmp(token, "zero") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                float zero = atof(token);
                balance_test_set_angle_zeropoint(zero);
                printf("Angle zeropoint set to %.2f (synced to all controllers)\n", zero);
            } else {
                printf("Current angle zeropoint: %.2f\n", g_angle_zeropoint);
            }
        } else if (strcmp(token, "reset") == 0) {
            triple_pid_reset(&g_triple_pid_ctrl);
            printf("Triple PID controller reset\n");
        } else if (strcmp(token, "status") == 0) {
            printf("=== Triple PID Status ===\n");
            printf("Control mode: %s\n", g_control_mode == CTRL_MODE_TRIPLE_PID ? "ACTIVE" : "INACTIVE");
            printf("Wheel mode: %s\n",
                   g_triple_pid_ctrl.params.wheel_mode == TRIPLE_PID_WHEEL_SPEED 
                       ? "SPEED_CMD (电机速度模式)" : "TORQUE_PID (软件PID→扭矩)");
            printf("Speed error: %.2f rad/s\n", g_triple_pid_output.speed_error);
            printf("Pitch target: %.2f deg\n", g_triple_pid_output.pitch_target);
            printf("Angle error: %.2f deg\n", g_triple_pid_output.angle_error);
            printf("Wheel speed target: %.2f rad/s\n", g_triple_pid_output.wheel_speed_target);
            printf("Wheel speed error: %.2f rad/s\n", g_triple_pid_output.wheel_speed_error);
            printf("Output: %.2f %s\n", g_triple_pid_output.torque,
                   g_triple_pid_ctrl.params.wheel_mode == TRIPLE_PID_WHEEL_SPEED ? "rad/s" : "Nm");
            printf("Speed PID: P=%.4f I=%.4f D=%.6f\n",
                   g_triple_pid_output.speed_p_out, g_triple_pid_output.speed_i_out, g_triple_pid_output.speed_d_out);
            printf("Angle PID: P=%.4f I=%.4f D=%.6f\n",
                   g_triple_pid_output.angle_p_out, g_triple_pid_output.angle_i_out, g_triple_pid_output.angle_d_out);
            printf("Wheel PID: P=%.4f I=%.4f D=%.6f\n",
                   g_triple_pid_output.wheel_p_out, g_triple_pid_output.wheel_i_out, g_triple_pid_output.wheel_d_out);
            printf("Gyro PID:  P=%.4f I=%.4f D=%.6f (kp=%.4f ki=%.4f kd=%.6f)\n",
                   g_triple_pid_output.gyro_p_out, g_triple_pid_output.gyro_i_out, g_triple_pid_output.gyro_d_out,
                   g_triple_pid_ctrl.params.gyro_kp, g_triple_pid_ctrl.params.gyro_ki, g_triple_pid_ctrl.params.gyro_kd);
            printf("TPID_STATUS:PITCH_TGT=%.2f,WHL_TGT=%.2f,TORQUE=%.2f,WMODE=%d\n",
                   g_triple_pid_output.pitch_target, g_triple_pid_output.wheel_speed_target,
                   g_triple_pid_output.torque, g_triple_pid_ctrl.params.wheel_mode);
            printf("Distance PID: kp=%.4f ki=%.4f kd=%.4f (limit=%.1f) [%s]\n",
                   g_triple_pid_ctrl.params.distance_kp, g_triple_pid_ctrl.params.distance_ki,
                   g_triple_pid_ctrl.params.distance_kd, g_triple_pid_ctrl.params.distance_limit,
                   g_triple_pid_ctrl.params.distance_enable ? "启用" : "关闭");
            printf("Distance zeropoint: %.3f, current: %.3f, error: %.3f\n",
                   g_triple_pid_ctrl.distance_zeropoint, g_lqr_distance,
                   g_triple_pid_output.distance_error);
            printf("Distance control: %.4f\n", g_triple_pid_output.distance_control);
        } else if (strcmp(token, "distance") == 0 || strcmp(token, "dist") == 0) {
            // 设置位移环PID参数
            float kp = g_triple_pid_ctrl.params.distance_kp;
            float ki = g_triple_pid_ctrl.params.distance_ki;
            float kd = g_triple_pid_ctrl.params.distance_kd;
            token = strtok(NULL, " \t\n\r");
            if (token) kp = atof(token);
            token = strtok(NULL, " \t\n\r");
            if (token) ki = atof(token);
            token = strtok(NULL, " \t\n\r");
            if (token) kd = atof(token);
            triple_pid_set_distance_gains(&g_triple_pid_ctrl, kp, ki, kd);
            printf("Triple PID Distance set: kp=%.4f ki=%.4f kd=%.4f\n", kp, ki, kd);
            printf("TPID:DIST,%.4f,%.4f,%.4f\n", kp, ki, kd);
        } else if (strcmp(token, "disten") == 0) {
            // 位移环使能/禁用
            token = strtok(NULL, " \t\n\r");
            if (token) {
                int en = atoi(token);
                g_triple_pid_ctrl.params.distance_enable = (uint8_t)(en ? 1 : 0);
                if (en) {
                    // 启用时重置位移零点为当前位置，避免跳变
                    triple_pid_set_distance_zeropoint(&g_triple_pid_ctrl, g_lqr_distance);
                    g_distance_zeropoint = g_lqr_distance;
                    pid_reset(&g_triple_pid_ctrl.pid_distance);
                }
                printf("Triple PID distance loop %s\n", en ? "ENABLED" : "DISABLED");
                printf("TPID:DISTEN,%d\n", en ? 1 : 0);
            } else {
                printf("Distance loop: %s\n", g_triple_pid_ctrl.params.distance_enable ? "ENABLED" : "DISABLED");
                printf("TPID:DISTEN,%d\n", g_triple_pid_ctrl.params.distance_enable);
            }
        } else {
            printf("Unknown tpid command: %s\n", token);
            printf("Usage: balance tpid [angle|speed|wheel|gyro|distance|disten|gain|wmode|zero|reset|status]\n");
        }
    }
    // ===== 轮速加权滑动平均滤波器 =====
    else if (strcmp(token, "wma") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            printf("=== Wheel Speed WMA Filter ===\n");
            printf("Status: %s\n", g_wma_enabled ? "ENABLED" : "DISABLED");
            printf("Window: %d points\n", WMA_WINDOW_SIZE);
            printf("Usage: balance wma [on|off]\n");
            printf("WMA_STATUS:%d\n", g_wma_enabled ? 1 : 0);
        } else if (strcmp(token, "on") == 0 || strcmp(token, "1") == 0) {
            wma_reset(&g_wheel_speed_wma);
            g_wma_enabled = true;
            printf("WMA filter ENABLED (%d-point weighted moving average)\n", WMA_WINDOW_SIZE);
            printf("WMA_STATUS:1\n");
        } else if (strcmp(token, "off") == 0 || strcmp(token, "0") == 0) {
            g_wma_enabled = false;
            printf("WMA filter DISABLED (raw wheel speed)\n");
            printf("WMA_STATUS:0\n");
        } else {
            printf("Unknown wma command: %s\n", token);
            printf("Usage: balance wma [on|off]\n");
        }
    }
    // ===== 遥杆映射比例调节 =====
    else if (strcmp(token, "joy") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            printf("=== Joystick Scale ===\n");
            printf("Speed scale: %.6f (joy_y * scale → target_speed, max ±%.3f)\n",
                   g_joy_speed_scale, 100.0f * g_joy_speed_scale);
            printf("Yaw scale:   %.6f (joy_x * scale → target_yaw_rate, max ±%.3f)\n",
                   g_joy_yaw_scale, 100.0f * g_joy_yaw_scale);
            printf("Usage: balance joy [speed <scale>|yaw <scale>]\n");
        } else if (strcmp(token, "speed") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                g_joy_speed_scale = atof(token);
                printf("Joy speed scale = %.6f (max speed ±%.3f)\n",
                       g_joy_speed_scale, 100.0f * g_joy_speed_scale);
            } else {
                printf("Current speed scale = %.6f (max speed ±%.3f)\n",
                       g_joy_speed_scale, 100.0f * g_joy_speed_scale);
            }
        } else if (strcmp(token, "yaw") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                g_joy_yaw_scale = atof(token);
                printf("Joy yaw scale = %.6f (max yaw rate ±%.3f)\n",
                       g_joy_yaw_scale, 100.0f * g_joy_yaw_scale);
            } else {
                printf("Current yaw scale = %.6f (max yaw rate ±%.3f)\n",
                       g_joy_yaw_scale, 100.0f * g_joy_yaw_scale);
            }
        } else {
            printf("Unknown joy command: %s\n", token);
            printf("Usage: balance joy [speed <scale>|yaw <scale>]\n");
        }
    }
    // ===== Full LQR 调参和数据流命令 =====
    else if (strcmp(token, "flqr") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            // 显示当前 Full LQR 参数
            printf("=== Full LQR Parameters ===\n");
            printf("Initialized: %s\n", g_full_lqr_initialized ? "YES" : "NO");
            printf("Pitch offset: %.4f rad (%.2f deg)\n",
                   g_full_lqr_ctrl.params.pitch_offset,
                   RAD2DEG(g_full_lqr_ctrl.params.pitch_offset));
            printf("V set scale: %.2f\n", g_full_lqr_ctrl.params.v_set_scale);
            printf("Max wheel torque: %.2f Nm\n", g_full_lqr_ctrl.params.max_wheel_torque);
            printf("Max leg torque: %.2f Nm\n", g_full_lqr_ctrl.params.max_leg_torque);
            printf("Split PD: Kp=%.3f Kd=%.3f limit=%.2f\n",
                   g_full_lqr_ctrl.params.split_kp,
                   g_full_lqr_ctrl.params.split_kd,
                   g_full_lqr_ctrl.params.split_limit);
            printf("Turn PD: Kp=%.3f Kd=%.4f limit=%.2f\n",
                   g_full_lqr_ctrl.params.turn_kp,
                   g_full_lqr_ctrl.params.turn_kd,
                   g_full_lqr_ctrl.params.turn_limit);
            printf("Current Tp: L=%.3f R=%.3f  T: L=%.3f R=%.3f\n",
                   g_full_lqr_Tp_left, g_full_lqr_Tp_right,
                   g_full_lqr_wheel_T_left, g_full_lqr_wheel_T_right);
            printf("Current K gains (R leg, L0 interp):\n");
            for (int i = 0; i < 12; i++) {
                printf("  K[%2d] = %+10.4f  (%s)\n", i, g_full_lqr_ctrl.K[i],
                       i < 6 ? "T" : "Tp");
            }
            printf("Usage: balance flqr [stream|pitch_offset|v_scale|max_t|max_tp|split|turn|coeff]\n");
        } else if (strcmp(token, "stream") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token == NULL) {
                g_full_lqr_stream_enable = !g_full_lqr_stream_enable;
            } else if (strcmp(token, "on") == 0 || strcmp(token, "1") == 0) {
                g_full_lqr_stream_enable = true;
            } else {
                g_full_lqr_stream_enable = false;
            }
            printf("Full LQR stream: %s\n", g_full_lqr_stream_enable ? "ON" : "OFF");
        } else if (strcmp(token, "pitch_offset") == 0 || strcmp(token, "po") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                g_full_lqr_ctrl.params.pitch_offset = atof(token);
                printf("Pitch offset = %.4f rad (%.2f deg)\n",
                       g_full_lqr_ctrl.params.pitch_offset,
                       RAD2DEG(g_full_lqr_ctrl.params.pitch_offset));
            } else {
                printf("Current pitch_offset = %.4f rad\n", g_full_lqr_ctrl.params.pitch_offset);
            }
        } else if (strcmp(token, "v_scale") == 0 || strcmp(token, "vs") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                g_full_lqr_ctrl.params.v_set_scale = atof(token);
                printf("V set scale = %.3f\n", g_full_lqr_ctrl.params.v_set_scale);
            } else {
                printf("Current v_scale = %.3f\n", g_full_lqr_ctrl.params.v_set_scale);
            }
        } else if (strcmp(token, "max_t") == 0 || strcmp(token, "mt") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                g_full_lqr_ctrl.params.max_wheel_torque = atof(token);
                printf("Max wheel torque = %.2f Nm\n", g_full_lqr_ctrl.params.max_wheel_torque);
            } else {
                printf("Current max_wheel_torque = %.2f Nm\n", g_full_lqr_ctrl.params.max_wheel_torque);
            }
        } else if (strcmp(token, "max_tp") == 0 || strcmp(token, "mtp") == 0) {
            token = strtok(NULL, " \t\n\r");
            if (token) {
                g_full_lqr_ctrl.params.max_leg_torque = atof(token);
                printf("Max leg torque = %.2f Nm\n", g_full_lqr_ctrl.params.max_leg_torque);
            } else {
                printf("Current max_leg_torque = %.2f Nm\n", g_full_lqr_ctrl.params.max_leg_torque);
            }
        } else if (strcmp(token, "split") == 0) {
            // balance flqr split <kp> <kd> [limit]
            token = strtok(NULL, " \t\n\r");
            if (token) {
                g_full_lqr_ctrl.params.split_kp = atof(token);
                token = strtok(NULL, " \t\n\r");
                if (token) g_full_lqr_ctrl.params.split_kd = atof(token);
                token = strtok(NULL, " \t\n\r");
                if (token) g_full_lqr_ctrl.params.split_limit = atof(token);
            }
            printf("Split PD: Kp=%.3f Kd=%.3f limit=%.2f\n",
                   g_full_lqr_ctrl.params.split_kp,
                   g_full_lqr_ctrl.params.split_kd,
                   g_full_lqr_ctrl.params.split_limit);
        } else if (strcmp(token, "turn") == 0) {
            // balance flqr turn <kp> <kd> [limit]
            token = strtok(NULL, " \t\n\r");
            if (token) {
                g_full_lqr_ctrl.params.turn_kp = atof(token);
                token = strtok(NULL, " \t\n\r");
                if (token) g_full_lqr_ctrl.params.turn_kd = atof(token);
                token = strtok(NULL, " \t\n\r");
                if (token) g_full_lqr_ctrl.params.turn_limit = atof(token);
            }
            printf("Turn PD: Kp=%.3f Kd=%.4f limit=%.2f\n",
                   g_full_lqr_ctrl.params.turn_kp,
                   g_full_lqr_ctrl.params.turn_kd,
                   g_full_lqr_ctrl.params.turn_limit);
        } else if (strcmp(token, "coeff") == 0) {
            // balance flqr coeff [<row>] [<c0> <c1> <c2> <c3>]
            token = strtok(NULL, " \t\n\r");
            if (token == NULL) {
                // 显示所有多项式系数
                printf("Poly coefficients [12][4]:\n");
                for (int i = 0; i < 12; i++) {
                    printf("  K[%2d]: %+12.4f %+12.4f %+12.4f %+12.4f\n",
                           i,
                           g_full_lqr_ctrl.params.poly_coeff[i][0],
                           g_full_lqr_ctrl.params.poly_coeff[i][1],
                           g_full_lqr_ctrl.params.poly_coeff[i][2],
                           g_full_lqr_ctrl.params.poly_coeff[i][3]);
                }
            } else {
                int row = atoi(token);
                if (row < 0 || row >= 12) {
                    printf("Row must be 0-11\n");
                } else {
                    token = strtok(NULL, " \t\n\r");
                    if (token) {
                        g_full_lqr_ctrl.params.poly_coeff[row][0] = atof(token);
                        token = strtok(NULL, " \t\n\r");
                        if (token) g_full_lqr_ctrl.params.poly_coeff[row][1] = atof(token);
                        token = strtok(NULL, " \t\n\r");
                        if (token) g_full_lqr_ctrl.params.poly_coeff[row][2] = atof(token);
                        token = strtok(NULL, " \t\n\r");
                        if (token) g_full_lqr_ctrl.params.poly_coeff[row][3] = atof(token);
                        printf("K[%d] coeff set: %.4f %.4f %.4f %.4f\n", row,
                               g_full_lqr_ctrl.params.poly_coeff[row][0],
                               g_full_lqr_ctrl.params.poly_coeff[row][1],
                               g_full_lqr_ctrl.params.poly_coeff[row][2],
                               g_full_lqr_ctrl.params.poly_coeff[row][3]);
                    } else {
                        printf("K[%d]: %.4f %.4f %.4f %.4f\n", row,
                               g_full_lqr_ctrl.params.poly_coeff[row][0],
                               g_full_lqr_ctrl.params.poly_coeff[row][1],
                               g_full_lqr_ctrl.params.poly_coeff[row][2],
                               g_full_lqr_ctrl.params.poly_coeff[row][3]);
                    }
                }
            }
        } else {
            printf("Unknown flqr sub-command: %s\n", token);
            printf("Usage: balance flqr [stream|pitch_offset|v_scale|max_t|max_tp|split|turn|coeff]\n");
        }
    }
    // ===== 支持力数据流控制 =====
    else if (strcmp(token, "sforce") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            g_sforce_stream_enable = !g_sforce_stream_enable;
        } else if (strcmp(token, "on") == 0 || strcmp(token, "1") == 0) {
            g_sforce_stream_enable = true;
        } else if (strcmp(token, "off") == 0 || strcmp(token, "0") == 0) {
            g_sforce_stream_enable = false;
        }
        printf("Support force stream: %s\n", g_sforce_stream_enable ? "ON" : "OFF");
        printf("Format: #SFORCE,L_FL(N),L_Fa(Nm),R_FL(N),R_Fa(Nm)\n");
    }
    else {
        printf("Unknown command: %s\n", token);
        printf("Usage: balance [init|start|stop|enable|disable|estop|reset|status|zero|plot|debug|leg|roll|mzero|loop|task|safety|airborne|mode|dpid|spid|wma|joy|flqr|sforce]\n");
    }
}
