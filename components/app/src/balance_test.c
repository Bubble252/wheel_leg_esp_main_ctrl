/**
 * @file balance_test.c
 * @brief 平衡控制测试模块实现 - 仅轮电机
 * @author Bubble
 * @date 2026-01-17
 * @note 参考 shibo_wheel_leg 项目的 wl_pro_robot_freertos.cpp
 * 
 * FreeRTOS 任务架构:
 * ┌─────────────────────────────────────────────────────────────┐
 * │ Core 0 (低优先级)                                           │
 * │   task_wifi_remote    - WiFi + WebSocket (优先级 10)        │
 * │   task_remote_watchdog - 遥控超时检测 (优先级 8)            │
 * ├─────────────────────────────────────────────────────────────┤
 * │ Core 1 (高优先级, 实时)                                      │
 * │   task_imu_read       - IMU 数据读取 (优先级 22, 2ms)       │
 * │   task_observer       - 速度观测器 (优先级 21, 3ms)          │
 * │   task_balance_ctrl   - LQR 平衡控制 (优先级 24, 5ms)       │
 * │   task_motor_comm     - CAN 电机通信 (优先级 20, 2ms)       │
 * └─────────────────────────────────────────────────────────────┘
 */

#include "balance_test.h"
#include "balance_roll.h"
#include "balance_jump.h"
#include "balance_standup.h"
#include "balance_observer.h"
#include "balance_vmc.h"
#include "balance_types.h"
#include "balance_cli.h"
#include "balance_plot.h"
#include "balance_control.h"
#include "balance_manager.h"
#include "config.h"
#include "types.h"
#include "can_motor.h"
#include "can_motor_stw_regs.h"
#include "imu_driver.h"
#include "wit_reg.h"      // RRATE_200HZ 定义
#include "wifi_remote.h"
#include "lqr_balance.h"
#include "full_lqr.h"
#include "lowpass_filter.h"
#include "leg_kinematics.h"
#include "power_detect.h"
#include "commander_parser.h"
#include "pi_comm.h"      // 树莓派串口通信

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <strings.h>     // strcasecmp
#include <stdlib.h>
#include <math.h>

static const char *TAG = "BAL_TEST";

// 角度-弧度转换
#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif
#define DEG2RAD(d) ((d) * M_PI / 180.0f)
#define RAD2DEG(r) ((r) * 180.0f / M_PI)

// ============================================================================
// 任务配置
// ============================================================================

// 任务周期 (ms) - 覆盖 config.h 中的默认值
// FreeRTOS tick = 1ms (CONFIG_FREERTOS_HZ=1000)
#undef IMU_READ_PERIOD_MS
#undef BALANCE_CTRL_PERIOD_MS
#undef MOTOR_COMM_PERIOD_MS
#define IMU_READ_PERIOD_MS          3       // 333Hz IMU 读取 (仅分离模式使用)
#define BALANCE_CTRL_PERIOD_MS      2       // 500Hz 平衡控制
#define MOTOR_COMM_PERIOD_MS        2       // 500Hz 电机通信 (与控制同步)
#define LEG_MOTOR_DIVIDER           2       // 腿电机分频 (500Hz / 2 = 250Hz, CAN利用率约50%)
#define WATCHDOG_PERIOD_MS          100     // 10Hz
#define OBSERVER_PERIOD_MS          3       // 333Hz 速度观测器 (独立任务)

// 电机力矩安全限幅 (Nm) - 基于实际电机硬件极限
#define WHEEL_TORQUE_LIMIT          0.3f    // 轮电机力矩上限
#define HIP_TORQUE_LIMIT            1.6f    // Hip 关节力矩上限
#define KNEE_TORQUE_LIMIT           3.5f    // Knee 关节力矩上限

// ===== 合并任务配置 =====
// 将 IMU读取 + 控制算法 + 电机通信 合并到一个任务中运行
// 默认使用分离任务架构，可通过 CLI 切换
#define UNIFIED_TASK_PERIOD_MS      2       // 合并任务基础周期 (500Hz)
#define UNIFIED_TASK_STACK          12288   // 合并任务栈大小 (需要更大)
#define UNIFIED_TASK_PRIO           24      // 合并任务优先级 (最高)

// 任务栈大小 - 覆盖 config.h 中的默认值
// 注意: 1000Hz tick 下 FreeRTOS 开销增加，需要更大的栈
#undef TASK_STACK_IMU
#undef TASK_STACK_BALANCE
#undef TASK_STACK_MOTOR
#define TASK_STACK_IMU              4096
#define TASK_STACK_BALANCE          8192
#define TASK_STACK_MOTOR            4096
#define TASK_STACK_WATCHDOG         4096    // 增加到 4KB，避免栈溢出
#define TASK_STACK_OBSERVER         4096    // 观测器任务栈

// 任务优先级
#define TASK_PRIO_IMU               22
#define TASK_PRIO_BALANCE           24
#define TASK_PRIO_MOTOR             20
#define TASK_PRIO_WATCHDOG          8
#define TASK_PRIO_OBSERVER          21      // 低于 IMU(22) 和控制(24), 高于电机(20)

// 遥控超时 (ms)
#define REMOTE_TIMEOUT_MS           1000

// 紧急停止角度
#define EMERGENCY_ANGLE_DEG         45.0f

// ============================================================================
// 共享数据结构 (线程安全)
// ============================================================================

// shared_imu_data_t → balance_types.h
// shared_remote_data_t → balance_types.h
// shared_wheel_cmd_t → balance_types.h
// shared_wheel_state_t → balance_types.h

// ============================================================================
// 全局变量
// ============================================================================

// 共享数据
shared_imu_data_t g_imu_data = {0};
shared_remote_data_t g_remote_data = {0};
shared_wheel_cmd_t g_wheel_cmd = {0};
shared_wheel_state_t g_wheel_state = {0};

// 互斥锁
SemaphoreHandle_t g_imu_mutex = NULL;
SemaphoreHandle_t g_remote_mutex = NULL;
SemaphoreHandle_t g_wheel_cmd_mutex = NULL;
SemaphoreHandle_t g_wheel_state_mutex = NULL;

// LQR 控制器
lqr_controller_t g_lqr_ctrl;

// 电机句柄
can_motor_handle_t g_motor_left = NULL;   // ID=3
can_motor_handle_t g_motor_right = NULL;  // ID=6

// 任务句柄
TaskHandle_t g_task_imu = NULL;
TaskHandle_t g_task_balance = NULL;
TaskHandle_t g_task_motor = NULL;
TaskHandle_t g_task_watchdog = NULL;
TaskHandle_t g_task_unified = NULL;        // 合并任务句柄
TaskHandle_t g_task_observer = NULL;       // 观测器独立任务句柄

// 任务架构选择
bool g_use_unified_task = true;            // true=使用合并任务(默认), false=使用分离任务

// 功能开关
bool g_uncontrolable_check_enabled = true;  // true=启用失控检测(默认), false=禁用失控检测

// control_mode_t 已移至 balance_types.h
control_mode_t g_control_mode = CTRL_MODE_TRIPLE_PID;  // 默认三环 PID 模式

// 普通小车模式参数 (部分已移至 balance_types.h)
#define CAR_MODE_MAX_SPEED      (200.0f)    // 小车模式最大速度 (rpm)
#define CAR_MODE_YAW_GAIN       (80.0f)     // 小车模式转向增益 (rpm per joy_x unit)
control_mode_t g_car_mode_prev_mode = CTRL_MODE_LQR;  // 进入小车模式前的模式 (用于退出时恢复)
float g_car_mode_prev_base_angle = -90.0f;            // 进入小车模式前的身体夹角
float g_car_mode_prev_base_length = 0.09f;            // 进入小车模式前的腿长

// ============================================================================
// bal_to_car_state_t, g_bal_to_car_state 等已移至 balance_standup.c
// bal_to_car 过渡参数已移至 balance_types.h
// CAR_TO_BAL_LEG_LENGTH, BAL_TO_CAR_RETRACT_LENGTH 等通过 balance_types.h 包含

// 双环 PID 控制器
dual_pid_controller_t g_dual_pid_ctrl;
bool g_dual_pid_initialized = false;
dual_pid_output_t g_dual_pid_output;       // 保存输出用于调试

// 三环 PID 控制器 (速度环→角度环→轮速环)
triple_pid_controller_t g_triple_pid_ctrl;
bool g_triple_pid_initialized = false;
triple_pid_output_t g_triple_pid_output;   // 保存输出用于调试

// 完整 LQR 控制器 (同时输出 T 和 Tp, K 随腿长插值)
bool g_auto_enable_inhibited = true;        // true=禁止自动使能, 需手动 balance enable

full_lqr_controller_t g_full_lqr_ctrl;
bool g_full_lqr_initialized = false;
full_lqr_output_t g_full_lqr_output_left;   // 左腿输出 (调试)
full_lqr_output_t g_full_lqr_output_right;  // 右腿输出 (调试)
bool g_full_lqr_stream_enable = false;       // #FLQR 数据流开关
float g_full_lqr_Tp_left = 0.0f;             // 左腿 Tp 输出 (用于 VMC 注入)
float g_full_lqr_Tp_right = 0.0f;            // 右腿 Tp 输出 (用于 VMC 注入)
float g_full_lqr_wheel_T_left = 0.0f;        // 左轮 T 输出 (调试)
float g_full_lqr_wheel_T_right = 0.0f;       // 右轮 T 输出 (调试)

// 轮速加权滑动平均滤波器 (用于双环/三环 PID 模式)
weighted_ma_filter_t g_wheel_speed_wma;
bool g_wma_enabled = true;    // 默认开启 WMA 滤波

// 单环 PID 控制器 (输出速度，适合电机速度模式)
single_pid_controller_t g_single_pid_ctrl;
bool g_single_pid_initialized = false;
single_pid_output_t g_single_pid_output;   // 保存输出用于调试

// 状态
balance_test_state_t g_state = BALANCE_TEST_IDLE;
bool g_initialized = false;
volatile bool g_tasks_running = false;

// 统计
balance_test_stats_t g_stats = {0};
uint32_t g_imu_count_per_sec = 0;
uint32_t g_ctrl_count_per_sec = 0;
uint32_t g_motor_count_per_sec = 0;
uint32_t g_leg_count_per_sec = 0;
uint32_t g_last_stat_time = 0;

// 延迟诊断 (微秒级) - 精确测量
// 核心思路: 追踪实际被使用的 IMU 数据从读取到电机发送的真实延迟
volatile uint64_t g_used_imu_time_us = 0;      // 控制任务实际使用的 IMU 数据的读取时间
volatile uint64_t g_ctrl_start_time_us = 0;    // 控制计算开始时间
volatile uint64_t g_ctrl_end_time_us = 0;      // 控制计算结束时间
volatile uint64_t g_motor_send_time_us = 0;    // 电机命令发送时间
float g_latency_imu_to_ctrl_us = 0.0f;         // IMU数据等待时间 (数据读取 → 控制开始使用)
float g_latency_ctrl_calc_us = 0.0f;           // 控制计算耗时
float g_latency_ctrl_to_motor_us = 0.0f;       // 控制输出等待时间 (控制完成 → 电机发送)
float g_latency_total_us = 0.0f;               // 总延迟 (IMU读取 → 电机发送)
uint32_t g_latency_sample_count = 0;           // 延迟采样计数
float g_latency_total_avg_us = 0.0f;           // 总延迟平均值
float g_latency_total_max_us = 0.0f;           // 总延迟最大值
float g_latency_total_min_us = 999999.0f;      // 总延迟最小值

// WiFi 遥控延迟诊断 (微秒级)
volatile uint64_t g_used_wifi_time_us = 0;     // 控制任务实际使用的 WiFi 数据的接收时间
float g_latency_wifi_to_ctrl_us = 0.0f;        // WiFi数据等待时间 (WiFi接收 → 控制使用)
float g_latency_wifi_total_us = 0.0f;          // WiFi总延迟 (WiFi接收 → 电机发送)
float g_latency_wifi_avg_us = 0.0f;            // WiFi延迟平均值
float g_latency_wifi_max_us = 0.0f;            // WiFi延迟最大值
float g_latency_wifi_min_us = 999999.0f;       // WiFi延迟最小值
uint32_t g_latency_wifi_sample_count = 0;      // WiFi延迟采样计数

// LQR 内部状态 (来自 shibo_wheel_leg)
float g_lqr_distance = 0.0f;         // 累积位移 (机器人前进方向为正)
float g_lqr_speed = 0.0f;            // 当前速度 (机器人前进方向为正)
float g_distance_zeropoint = 0.0f;   // 位移零点
float g_angle_zeropoint = -1.5f;     // 角度零点 (需要根据实际机器人调整)
int g_move_stop_flag = 0;            // 停止标志
int g_uncontrolable = 0;             // 失控标志

// 遥杆映射比例 (可通过 UI/CLI 在线调节)
float g_joy_speed_scale = 0.003f;    // joy_y → target_speed 比例 (默认 0.003, max ±0.3)
float g_joy_yaw_scale = 0.9f;       // joy_x → target_yaw_rate 比例 (默认 0.9)
float g_tpid_yaw_scale = 500.0f;     // 三环PID yaw输出缩放 (LQR yaw输出太小, 需放大)

// 轮速加速度计算 (用于离地检测)
float g_left_wheel_speed_rad = 0.0f;     // 左轮速度 (rad/s)
float g_right_wheel_speed_rad = 0.0f;    // 右轮速度 (rad/s)
float g_left_wheel_accel = 0.0f;         // 左轮加速度 (rad/s²)
float g_right_wheel_accel = 0.0f;        // 右轮加速度 (rad/s²)
float g_prev_left_wheel_speed = 0.0f;    // 上次左轮速度
float g_prev_right_wheel_speed = 0.0f;   // 上次右轮速度
bool g_wheel_off_ground = false;          // 轮子离地标志
int g_off_ground_counter = 0;             // 离地检测去抖计数器

// 支持力离地检测 → 已移至 balance_vmc.c (变量和宏)

// 离地检测去抖参数
#define OFF_GROUND_ENTER_COUNT  8   // 连续 8 帧判定离地 (16ms@500Hz)
#define OFF_GROUND_EXIT_COUNT   13  // 连续 13 帧判定着地 (26ms@500Hz)

// YAW 轴控制 (带过零处理)
float g_yaw_angle_last = 0.0f;       // 上一次 YAW 角度 (用于过零处理)
float g_yaw_angle_total = 0.0f;      // YAW 累积角度 (经过过零处理)
float g_yaw_output = 0.0f;           // YAW 控制输出
bool g_yaw_first_run = true;         // YAW 首次运行标志

// Roll 控制 (腿长调节)
float g_roll_output = 0.0f;          // Roll 控制原始输出
float g_roll_left_delta = 0.0f;      // 左腿长度增量
float g_roll_right_delta = 0.0f;     // 右腿长度增量
float g_roll_filtered = 0.0f;        // 滤波后的 Roll 角度

// 波形数据输出 (用于 Qt 调参面板)
bool g_plot_enabled = false;         // 波形输出使能
uint8_t g_plot_divider = 10;         // 输出分频 (每N次控制循环输出一次)
uint8_t g_plot_counter = 0;          // 分频计数器
float g_last_lqr_u = 0.0f;           // 保存 LQR 输出用于波形显示
uint32_t g_plot_channel_mask = 0xFFFFFFFF;  // 通道使能掩码 (默认全开)

// plot_ch_bit 和 PLOT_CH_ENABLED 宏已移至 balance_plot.h

// PID 调试输出 (用于实时 debug)
bool g_pid_debug_enabled = false;    // PID 调试输出使能
uint8_t g_pid_debug_divider = 50;    // 输出分频 (每N次控制循环输出一次，默认约4Hz@200Hz)
uint8_t g_pid_debug_counter = 0;     // 分频计数器

// 环路使能控制 (用于单环调试)
uint8_t g_loop_enable_mask = LOOP_FULL;  // 默认全部启用
bool g_yaw_control_enabled = false;      // YAW 控制独立开关 (默认关, 需手动开启)
bool g_yaw_force_enable = false;         // YAW 强制使能 (无需遥控器 go, 通过 CLI 控制)
bool g_diff_speed_enabled = false;       // 差速转向使能 (Yaw失能时的开环差速)
float g_diff_speed_scale = 0.9f;        // 差速转向增益 (joy_x * scale)
bool g_loop_manual_mode = false;         // 手动模式 (禁止自动切换 simple/full)

// ============================================================================
// 腿部电机控制
// ============================================================================
bool g_leg_control_enabled = false;  // 腿部电机使能
can_motor_handle_t g_motor_left_hip = NULL;    // ID=1 左大腿 (extern for balance_jump.c)
can_motor_handle_t g_motor_left_knee = NULL;   // ID=2 左小腿
can_motor_handle_t g_motor_right_hip = NULL;   // ID=4 右大腿
can_motor_handle_t g_motor_right_knee = NULL;  // ID=5 右小腿

// 腿部电机目标角度 (度) - 由 leg_ctrl_init() 通过逆运动学计算
float g_leg_left_hip_angle = 0.0f;    // 左大腿角度 (extern for balance_roll.c)
float g_leg_left_knee_angle = 0.0f;   // 左小腿角度
float g_leg_right_hip_angle = 0.0f;   // 右大腿角度
float g_leg_right_knee_angle = 0.0f;  // 右小腿角度
float g_leg_move_speed = 50.0f;        // 腿部电机运动速度 (rpm)

// 腿长范围限制 (可通过 UI 或 CLI 调节, extern for balance_roll.c)
float g_leg_length_min = 0.065f;       // 最小腿长 (m), 默认与 LEG_LENGTH_MIN 一致
float g_leg_length_max = 0.11f;        // 最大腿长 (m), 默认与 LEG_LENGTH_MAX 一致

// 腿部目标状态 (运动学空间) (extern for balance_jump.c)
// 基础腿长/角度: 用户设定的"高度"，Roll控制不修改这些值
float g_leg_base_length = 0.09f;           // 基础腿长 (米) - 决定机器人高度
float g_leg_base_angle = -90.0f;          // 基础身体夹角 (度), -90=垂直向下

// 实际发送给电机的目标 (= 基础值 + Roll调整) (extern for balance_roll.c)
float g_leg_left_target_length = 0.09f;   // 左腿实际目标腿长 (米)
float g_leg_left_target_angle = -90.0f;   // 左腿实际目标身体夹角 (度)
float g_leg_right_target_length = 0.09f;  // 右腿实际目标腿长 (米)
float g_leg_right_target_angle = -90.0f;  // 右腿实际目标身体夹角 (度), -90=垂直向下

// Roll 闭环控制开关 (与腿长相关，纯轮测试时禁用)
bool g_roll_control_enabled = false;  // 默认禁用

// Pitch 腿部角度补偿开关
bool g_pitch_leg_comp_enabled = false; // 默认禁用腿部角度补偿

// ============================================================================
// 零点自适应 PID 调试变量
// ============================================================================
float g_zp_speed_threshold = 0.1f;         // 轮速阈值 (m/s), 低于此值才启用零点自适应
float g_zp_pitch_for_ctrl = 0.0f;         // 当前 pitch_for_control (用于零点调试)
float g_zp_angle_error = 0.0f;            // 零点自适应的角度误差
float g_zp_pid_raw = 0.0f;                // PID 原始输出 (滤波前)
float g_zp_pid_filtered = 0.0f;           // PID 滤波后输出
bool  g_zp_active = false;                // 是否进入了零点PID计算 (轮速<阈值)

// ============================================================================
// X-Offset 腿部速度自适应偏移 (独立腿部姿态控制)
// ============================================================================
// 功能: 根据当前轮速，用 PID 控制腿脚在笛卡尔 x 方向的偏移
//   速度>0 (前进) → x_offset>0 (腿脚后摆) → 类似人跑步时支撑腿后蹬
//   速度=0 → x_offset=0 (腿回中位)
// 与平衡控制完全独立，只改变腿的几何形状，不影响轮力矩
bool g_xoffset_enabled = false;           // X-Offset 使能开关
pid_controller_t g_xoffset_pid;           // X-Offset PID 控制器
float g_xoffset_value = 0.0f;             // 当前 x_offset 输出 (米)
float g_xoffset_limit = 0.03f;            // X-Offset 限幅 (米), 默认 ±3cm
float g_xoffset_debug_speed = 0.0f;       // 调试用: 当时的速度输入

// ============================================================================
// Leg Sync 左右腿同步控制 (防劈叉)
// ============================================================================
// 功能: 读取左右腿实际 body_angle，用交叉耦合补偿消除差异
//   左右腿角度差 → 按比例修正各自的目标角度
//   相当于在两条腿之间加一根"虚拟弹簧"
// 与 X-Offset 正交: X-Offset 是两腿同方向偏移，Sync 是反方向补偿
bool g_leg_sync_enabled = false;           // Leg Sync 使能开关
float g_leg_sync_gain = 0.3f;              // 同步增益 (0~1), 0.3 = 修正30%的差异
float g_leg_sync_max_correction = 15.0f;   // 最大修正量 (度), 防止过大跳变
float g_leg_sync_debug_diff = 0.0f;        // 调试: 左右腿角度差 (度)
float g_leg_sync_debug_correction = 0.0f;  // 调试: 实际修正量 (度)

// ============================================================================
// 跳跃状态机 → 已移至 components/balance/src/balance_jump.c
// extern MIT 参数 (供 apply_leg_motor_commands 使用)
// ============================================================================
extern bool g_jump_mit_active;
extern float g_jump_mit_kp;
extern float g_jump_mit_kd;
extern float g_jump_mit_ff_torque;
extern float g_jump_mit_target_rad[4];
extern float g_jump_mit_ff_sign[4];

float g_joint_normal_max_speed_rpm = 300.0f;  // 正常关节最大速度 (RPM) (extern for balance_jump.c)
float g_joint_normal_pos_kp = 0.0f;           // 正常位置环 Kp (启动时读取)

// ============================================================================
// 起身状态机 → 已移至 balance_standup.c (standup/car_standup/bal_to_car)
// extern 声明在 balance_standup.h 中
// ============================================================================

// ============================================================================
// VMC 变量 → 已移至 balance_vmc.c
// extern 声明在 balance_vmc.h 中
// ============================================================================

// 数据流输出控制 (仍在 balance_test.c 中定义, 用于 plot/stream)
bool g_joint_stream_enable = false;
bool g_mpow_stream_enable = false;
uint8_t g_mpow_volt_poll_idx = 0;
uint8_t g_mpow_volt_poll_div = 0;

// 离地检测变量 → 已移至 balance_vmc.c
// g_sforce_left_off_cnt, g_sforce_right_off_cnt 等

// ============================================================================
// 观测器变量 → 已移至 balance_observer.c
// extern 声明在 balance_observer.h 中
// ============================================================================

// ============================================================================
// 关节电机速度滤波 (支持中值滤波 / 限幅滤波切换)
// ============================================================================
bool g_joint_speed_filter_enable = true;    // 关节速度滤波使能 (默认开启)
int  g_joint_speed_filter_mode = 1;         // 0=中值滤波(Median), 1=限幅滤波(SlewRate)
// Slew-Rate 参数
float g_joint_speed_slew_rate = 3000.0f;    // 最大变化率 (°/s²), 默认3000
slewrate_filter_t g_sr_joint_lh;             // 左髋速度限幅滤波器
slewrate_filter_t g_sr_joint_lk;             // 左膝速度限幅滤波器
slewrate_filter_t g_sr_joint_rh;             // 右髋速度限幅滤波器
slewrate_filter_t g_sr_joint_rk;             // 右膝速度限幅滤波器
// Median 参数
int g_joint_median_window = 3;               // 中值滤波窗口大小 (3/5/7/9)
median_filter_t g_mf_joint_lh;               // 左髋速度中值滤波器
median_filter_t g_mf_joint_lk;               // 左膝速度中值滤波器
median_filter_t g_mf_joint_rh;               // 右髋速度中值滤波器
median_filter_t g_mf_joint_rk;               // 右膝速度中值滤波器
// 滤波后的速度值 (用于波形输出和 VMC 输入)
float g_joint_lh_spd_filtered = 0.0f;
float g_joint_lk_spd_filtered = 0.0f;
float g_joint_rh_spd_filtered = 0.0f;
float g_joint_rk_spd_filtered = 0.0f;

// 树莓派通信开关 (调试时可禁用以减少串口占用)
bool g_pi_comm_enabled = false;  // 默认禁用

// ============================================================================
// apply_xoffset_and_ik / apply_xoffset_single → 已移至 components/balance/src/balance_roll.c
// ============================================================================

// ============================================================================
// Commander 参数回调 - 将调参面板的参数同步到 LQR 控制器
// ============================================================================

/**
 * @brief Commander 参数更新回调
 * @note 当从串口接收到调参命令时调用，将参数同步到 LQR 控制器
 */
void commander_param_callback(char controller_id, char param_char, float value)
{
    if (!g_initialized) return;
    
    lqr_params_t params;
    memcpy(&params, &g_lqr_ctrl.params, sizeof(lqr_params_t));
    bool updated = false;
    
    switch (controller_id) {
        case CTRL_ID_ANGLE:  // A - 角度控制
            switch (param_char) {
                case 'P': params.angle_kp = value; updated = true; break;
                case 'I': params.angle_ki = value; updated = true; break;
                case 'D': params.angle_kd = value; updated = true; break;
                case 'L': params.angle_limit = value; updated = true; break;
            }
            break;
            
        case CTRL_ID_GYRO:  // B - 角速度控制
            switch (param_char) {
                case 'P': params.gyro_kp = value; updated = true; break;
                case 'I': params.gyro_ki = value; updated = true; break;
                case 'D': params.gyro_kd = value; updated = true; break;
                case 'L': params.gyro_limit = value; updated = true; break;
            }
            break;
            
        case CTRL_ID_DISTANCE:  // C - 位移控制 (仅LQR, Triple PID位移环独立)
            switch (param_char) {
                case 'P': params.distance_kp = value; updated = true; break;
                case 'I': params.distance_ki = value; updated = true; break;
                case 'D': params.distance_kd = value; updated = true; break;
                case 'L': params.distance_limit = value; updated = true; break;
            }
            break;
            
        case CTRL_ID_SPEED:  // D - 速度控制
            switch (param_char) {
                case 'P': params.speed_kp = value; updated = true; break;
                case 'I': params.speed_ki = value; updated = true; break;
                case 'D': params.speed_kd = value; updated = true; break;
                case 'L': params.speed_limit = value; updated = true; break;
            }
            break;
            
        case CTRL_ID_YAW_ANGLE:  // E - YAW角度控制
            switch (param_char) {
                case 'P': params.yaw_angle_kp = value; updated = true; break;
                case 'I': params.yaw_angle_ki = value; updated = true; break;
                case 'D': params.yaw_angle_kd = value; updated = true; break;
                case 'L': params.yaw_angle_limit = value; updated = true; break;
            }
            break;
            
        case CTRL_ID_YAW_GYRO:  // F - YAW角速度控制
            switch (param_char) {
                case 'P': params.yaw_gyro_kp = value; updated = true; break;
                case 'I': params.yaw_gyro_ki = value; updated = true; break;
                case 'D': params.yaw_gyro_kd = value; updated = true; break;
                case 'L': params.yaw_gyro_limit = value; updated = true; break;
            }
            break;
            
        case CTRL_ID_JOYY_LPF:  // G - 摇杆Y轴滤波
            if (param_char == 'T') {
                params.lpf_joyy_tf = value;
                updated = true;
                // 同步更新 Dual PID 的遥杆滤波器
                g_dual_pid_ctrl.params.lpf_joyy_tf = value;
                lpf_set_tf(&g_dual_pid_ctrl.lpf_joyy, value);
            }
            break;
            
        case CTRL_ID_LQR_U:  // H - LQR输出补偿
            switch (param_char) {
                case 'P': params.lqr_u_kp = value; updated = true; break;
                case 'I': params.lqr_u_ki = value; updated = true; break;
                case 'D': params.lqr_u_kd = value; updated = true; break;
                case 'L': params.lqr_u_limit = value; updated = true; break;
            }
            break;
            
        case CTRL_ID_ZEROPOINT:  // I - 零点自适应
            switch (param_char) {
                case 'P': params.zeropoint_kp = value; updated = true; break;
                case 'I': params.zeropoint_ki = value; updated = true; break;
                case 'D': params.zeropoint_kd = value; updated = true; break;
                case 'L': params.zeropoint_limit = value; updated = true; break;
            }
            break;
            
        case CTRL_ID_ZERO_LPF:  // J - 零点滤波
            if (param_char == 'T') {
                params.lpf_zeropoint_tf = value;
                updated = true;
            }
            break;
            
        case CTRL_ID_ROLL_ANGLE:  // K - Roll轴平衡
            switch (param_char) {
                case 'P': params.roll_kp = value; updated = true; break;
                case 'I': params.roll_ki = value; updated = true; break;
                case 'D': params.roll_kd = value; updated = true; break;
                case 'L': params.roll_limit = value; updated = true; break;
            }
            break;
            
        case CTRL_ID_ROLL_LPF:  // L - Roll角度滤波
            if (param_char == 'T') {
                params.lpf_roll_tf = value;
                updated = true;
            }
            break;
            
        case CTRL_ID_SPEED_ADAPT:  // M - 速度自适应
            switch (param_char) {
                case 'L': params.speed_kp_min = value; updated = true; break;  // Kp_Min
                case 'H': params.speed_kp_max = value; updated = true; break;  // Kp_Max
            }
            break;
            
        case CTRL_ID_GYRO_LPF:  // N - 角速度滤波
            switch (param_char) {
                case 'T': params.lpf_gyro_tf = value; updated = true; break;     // LPF Tf
                case 'M': params.gyro_filter_mode = (uint8_t)value; updated = true; break;  // 模式: 0=LPF, 1=限幅
                case 'R': params.gyro_slew_rate = value; updated = true; break;   // 限幅最大变化率
            }
            break;
            
        case CTRL_ID_SPEED_LPF:  // W - 轮速滤波
            switch (param_char) {
                case 'T': params.lpf_speed_tf = value; updated = true; break;     // LPF Tf
                case 'M': params.speed_filter_mode = (uint8_t)value; updated = true; break;  // 模式: 0=LPF, 1=限幅
                case 'R': params.speed_slew_rate = value; updated = true; break;   // 限幅最大变化率
            }
            break;
            
        case CTRL_ID_JOY_SCALE:  // X - 遥杆映射比例
            switch (param_char) {
                case 'P': g_joy_speed_scale = value; break;  // joy_y → speed 比例
                case 'I': g_joy_yaw_scale = value; break;    // joy_x → yaw 比例
            }
            // 不走 lqr_set_params, 直接生效
            ESP_LOGI(TAG, "Joy scale updated: speed=%.6f, yaw=%.6f", g_joy_speed_scale, g_joy_yaw_scale);
            return;  // 直接返回, 不调 lqr_set_params
    }
    
    if (updated) {
        lqr_set_params(&g_lqr_ctrl, &params);
        ESP_LOGI(TAG, "LQR params updated from Commander");
    }
}

/**
 * @brief Commander 参数查询回调
 * @note 当从串口接收到查询命令时调用，返回 LQR 控制器的实际参数
 * @param controller_id 控制器 ID (A-M)
 * @param params 输出参数结构
 * @return true=成功获取参数
 */
bool commander_query_callback(char controller_id, commander_pid_params_t *params)
{
    if (!g_initialized || !params) return false;
    
    const lqr_params_t *p = &g_lqr_ctrl.params;
    
    // 初始化默认值
    params->p = 0.0f;
    params->i = 0.0f;
    params->d = 0.0f;
    params->limit = 0.0f;
    params->ramp = 1000.0f;
    params->lpf_tf = 0.0f;
    
    switch (controller_id) {
        case CTRL_ID_ANGLE:  // A - 角度控制
            params->p = p->angle_kp;
            params->i = p->angle_ki;
            params->d = p->angle_kd;
            params->limit = p->angle_limit;
            return true;
            
        case CTRL_ID_GYRO:  // B - 角速度控制
            params->p = p->gyro_kp;
            params->i = p->gyro_ki;
            params->d = p->gyro_kd;
            params->limit = p->gyro_limit;
            return true;
            
        case CTRL_ID_DISTANCE:  // C - 位移控制 (LQR)
            params->p = p->distance_kp;
            params->i = p->distance_ki;
            params->d = p->distance_kd;
            params->limit = p->distance_limit;
            return true;
            
        case CTRL_ID_SPEED:  // D - 速度控制
            params->p = p->speed_kp;
            params->i = p->speed_ki;
            params->d = p->speed_kd;
            params->limit = p->speed_limit;
            return true;
            
        case CTRL_ID_YAW_ANGLE:  // E - YAW角度控制
            params->p = p->yaw_angle_kp;
            params->i = p->yaw_angle_ki;
            params->d = p->yaw_angle_kd;
            params->limit = p->yaw_angle_limit;
            return true;
            
        case CTRL_ID_YAW_GYRO:  // F - YAW角速度控制
            params->p = p->yaw_gyro_kp;
            params->i = p->yaw_gyro_ki;
            params->d = p->yaw_gyro_kd;
            params->limit = p->yaw_gyro_limit;
            return true;
            
        case CTRL_ID_JOYY_LPF:  // G - 摇杆Y轴滤波
            params->lpf_tf = p->lpf_joyy_tf;
            return true;
            
        case CTRL_ID_LQR_U:  // H - LQR输出补偿
            params->p = p->lqr_u_kp;
            params->i = p->lqr_u_ki;
            params->d = p->lqr_u_kd;
            params->limit = p->lqr_u_limit;
            return true;
            
        case CTRL_ID_ZEROPOINT:  // I - 零点自适应
            params->p = p->zeropoint_kp;
            params->i = p->zeropoint_ki;
            params->d = p->zeropoint_kd;
            params->limit = p->zeropoint_limit;
            return true;
            
        case CTRL_ID_ZERO_LPF:  // J - 零点滤波
            params->lpf_tf = p->lpf_zeropoint_tf;
            return true;
            
        case CTRL_ID_ROLL_ANGLE:  // K - Roll轴平衡
            params->p = p->roll_kp;
            params->i = p->roll_ki;
            params->d = p->roll_kd;
            params->limit = p->roll_limit;
            return true;
            
        case CTRL_ID_ROLL_LPF:  // L - Roll角度滤波
            params->lpf_tf = p->lpf_roll_tf;
            return true;
            
        case CTRL_ID_SPEED_ADAPT:  // M - 速度自适应
            // 复用 p 和 i 字段存储 kp_min/max
            params->p = p->speed_kp_min;
            params->i = p->speed_kp_max;
            return true;
            
        case CTRL_ID_GYRO_LPF:  // N - 角速度滤波
            params->lpf_tf = p->lpf_gyro_tf;
            // 复用 p 和 i 字段存储 mode 和 slew_rate
            params->p = (float)p->gyro_filter_mode;
            params->i = p->gyro_slew_rate;
            return true;
            
        case CTRL_ID_SPEED_LPF:  // W - 轮速滤波
            params->lpf_tf = p->lpf_speed_tf;
            // 复用 p 和 i 字段存储 mode 和 slew_rate
            params->p = (float)p->speed_filter_mode;
            params->i = p->speed_slew_rate;
            return true;
            
        case CTRL_ID_JOY_SCALE:  // X - 遥杆映射比例
            params->p = g_joy_speed_scale;
            params->i = g_joy_yaw_scale;
            return true;
            
        default:
            return false;
    }
}

// ============================================================================
// 波形数据输出 (用于 Qt 调参面板绘图)
// ============================================================================

/**
 * @brief 更新并上报机器人状态到 Pi
 * @note 由控制任务周期性调用，pi_comm 内部控制实际上报频率
 */
void update_pi_comm_state(void) {
    pi_robot_state_t state = {0};
    
    // 模式和状态
    if (g_wheel_cmd.enabled) {
        state.mode = MODE_STAND;
        state.status = STATUS_RUNNING;
    } else {
        state.mode = MODE_IDLE;
        state.status = STATUS_IDLE;
    }
    
    // 标志位
    if (g_wheel_cmd.enabled) {
        state.flags |= FLAG_MOTOR_ENABLED | FLAG_BALANCE_ACTIVE;
    }
    if (g_lqr_ctrl.yaw_holding) {
        state.flags |= FLAG_YAW_HOLDING;
    }
    if (pi_comm_is_connected()) {
        state.flags |= FLAG_PI_CONNECTED;
    }
    
    // IMU 数据 (度)
    state.pitch = g_imu_data.pitch;
    state.roll = g_imu_data.roll;
    state.yaw = g_imu_data.yaw;
    
    // 速度计算: rpm -> rad/s -> m/s
    // wheel_speed (rpm) * (2π/60) = rad/s, 再乘轮子半径得到线速度
    const float rpm_to_rad_s = 3.14159f / 30.0f;
    const float wheel_radius_m = WHEEL_RADIUS / 1000.0f;  // mm -> m
    float avg_wheel_rpm = (g_wheel_state.left_speed + g_wheel_state.right_speed) / 2.0f;
    state.vx_actual = avg_wheel_rpm * rpm_to_rad_s * wheel_radius_m;
    state.yaw_rate_actual = g_imu_data.yaw_rate;
    
    // 高度 (取左右腿平均目标值)
    state.height_actual = (g_leg_left_target_length + g_leg_right_target_length) / 2.0f;
    
    // 电池电压 (TODO: 从实际传感器读取)
    state.battery_voltage = 24.0f;
    
    pi_comm_update_state(&state);
}

/**
 * @brief 使能/禁用波形数据输出
 * @param enable 是否使能
 */
// ============================================================================
// 环路使能控制 (用于单环调试)
// ============================================================================

/**
 * @brief 同步环路使能掩码到 LQR 控制器
 */
static void sync_loop_enable_to_lqr(void) {
    if (!g_initialized) return;
    
    float angle_en    = (g_loop_enable_mask & LOOP_ANGLE)    ? 1.0f : 0.0f;
    float gyro_en     = (g_loop_enable_mask & LOOP_GYRO)     ? 1.0f : 0.0f;
    float distance_en = (g_loop_enable_mask & LOOP_DISTANCE) ? 1.0f : 0.0f;
    float speed_en    = (g_loop_enable_mask & LOOP_SPEED)    ? 1.0f : 0.0f;
    float lqr_u_en    = (g_loop_enable_mask & LOOP_LQR_U)    ? 1.0f : 0.0f;
    
    lqr_set_loop_enable(&g_lqr_ctrl, angle_en, gyro_en, distance_en, speed_en, lqr_u_en);
    
    // YAW 控制独立管理
    bool yaw_was_enabled = g_yaw_control_enabled;
    g_yaw_control_enabled = (g_loop_enable_mask & LOOP_YAW) ? true : false;
    
    // 如果 YAW 从关闭变为开启，重置状态
    if (!yaw_was_enabled && g_yaw_control_enabled) {
        g_lqr_ctrl.yaw_holding = false;
        g_lqr_ctrl.yaw_angle_target = g_yaw_angle_total;
        pid_reset(&g_lqr_ctrl.pid_yaw_angle);
        pid_reset(&g_lqr_ctrl.pid_yaw_gyro);
        ESP_LOGI(TAG, "YAW enabled via mask, target locked to %.2f", g_yaw_angle_total);
    }
}

void balance_test_set_loop_enable(uint8_t mask) {
    g_loop_enable_mask = mask;
    sync_loop_enable_to_lqr();
    
    // 使用预设模式时退出手动模式
    if (mask == LOOP_FULL || mask == LOOP_SIMPLE || mask == LOOP_NONE) {
        g_loop_manual_mode = false;
        ESP_LOGI(TAG, "Loop enable mask: 0x%02X (auto mode)", mask);
    } else {
        g_loop_manual_mode = true;
        ESP_LOGI(TAG, "Loop enable mask: 0x%02X (manual mode)", mask);
    }
    balance_test_print_loop_status();
}

uint8_t balance_test_get_loop_enable(void) {
    return g_loop_enable_mask;
}

void balance_test_set_loop_gain(loop_enable_mask_t loop, float enable) {
    if (!g_initialized) return;
    
    // 进入手动模式 (禁止自动切换 simple/full)
    g_loop_manual_mode = true;
    
    // Clamp to 0.0~5.0
    if (enable < 0.0f) enable = 0.0f;
    if (enable > 5.0f) enable = 5.0f;
    
    // 直接设置到 LQR 参数 (支持渐变)
    switch (loop) {
        case LOOP_ANGLE:
            g_lqr_ctrl.params.angle_enable = enable;
            if (enable > 0.5f) g_loop_enable_mask |= LOOP_ANGLE;
            else g_loop_enable_mask &= ~LOOP_ANGLE;
            break;
        case LOOP_GYRO:
            g_lqr_ctrl.params.gyro_enable = enable;
            if (enable > 0.5f) g_loop_enable_mask |= LOOP_GYRO;
            else g_loop_enable_mask &= ~LOOP_GYRO;
            break;
        case LOOP_DISTANCE:
            g_lqr_ctrl.params.distance_enable = enable;
            if (enable > 0.5f) g_loop_enable_mask |= LOOP_DISTANCE;
            else g_loop_enable_mask &= ~LOOP_DISTANCE;
            break;
        case LOOP_SPEED:
            g_lqr_ctrl.params.speed_enable = enable;
            if (enable > 0.5f) g_loop_enable_mask |= LOOP_SPEED;
            else g_loop_enable_mask &= ~LOOP_SPEED;
            break;
        case LOOP_LQR_U:
            g_lqr_ctrl.params.lqr_u_enable = enable;
            if (enable > 0.5f) g_loop_enable_mask |= LOOP_LQR_U;
            else g_loop_enable_mask &= ~LOOP_LQR_U;
            break;
        case LOOP_YAW:
            g_yaw_control_enabled = (enable > 0.5f);
            if (enable > 0.5f) {
                g_loop_enable_mask |= LOOP_YAW;
                // 启用 YAW 时，重置状态，让它重新锁定当前角度
                g_lqr_ctrl.yaw_holding = false;
                g_lqr_ctrl.yaw_angle_target = g_yaw_angle_total;
                pid_reset(&g_lqr_ctrl.pid_yaw_angle);
                pid_reset(&g_lqr_ctrl.pid_yaw_gyro);
                ESP_LOGI(TAG, "YAW enabled, target locked to %.2f", g_yaw_angle_total);
            } else {
                g_loop_enable_mask &= ~LOOP_YAW;
            }
            break;
        default:
            ESP_LOGW(TAG, "Unknown loop: 0x%02X", loop);
            return;
    }
    
    ESP_LOGI(TAG, "Loop 0x%02X gain set to %.2f (manual mode)", loop, enable);
}

float balance_test_get_loop_gain(loop_enable_mask_t loop) {
    if (!g_initialized) return 0.0f;
    
    switch (loop) {
        case LOOP_ANGLE:    return g_lqr_ctrl.params.angle_enable;
        case LOOP_GYRO:     return g_lqr_ctrl.params.gyro_enable;
        case LOOP_DISTANCE: return g_lqr_ctrl.params.distance_enable;
        case LOOP_SPEED:    return g_lqr_ctrl.params.speed_enable;
        case LOOP_LQR_U:    return g_lqr_ctrl.params.lqr_u_enable;
        case LOOP_YAW:      return g_yaw_control_enabled ? 1.0f : 0.0f;
        default:            return 0.0f;
    }
}

void balance_test_print_loop_status(void) {
    printf("\n=== Loop Enable Status ===\n");
    printf("Mode: %s\n", g_loop_manual_mode ? "MANUAL (user control)" : "AUTO (simple/full switch)");
    printf("  [%c] A - Angle (pitch)      : %.2f\n", 
           (g_loop_enable_mask & LOOP_ANGLE) ? 'X' : ' ',
           g_initialized ? g_lqr_ctrl.params.angle_enable : 0.0f);
    printf("  [%c] B - Gyro (pitch_rate)  : %.2f\n", 
           (g_loop_enable_mask & LOOP_GYRO) ? 'X' : ' ',
           g_initialized ? g_lqr_ctrl.params.gyro_enable : 0.0f);
    printf("  [%c] C - Distance           : %.2f\n", 
           (g_loop_enable_mask & LOOP_DISTANCE) ? 'X' : ' ',
           g_initialized ? g_lqr_ctrl.params.distance_enable : 0.0f);
    printf("  [%c] D - Speed              : %.2f\n", 
           (g_loop_enable_mask & LOOP_SPEED) ? 'X' : ' ',
           g_initialized ? g_lqr_ctrl.params.speed_enable : 0.0f);
    printf("  [%c] H - LQR_U (integral)   : %.2f\n", 
           (g_loop_enable_mask & LOOP_LQR_U) ? 'X' : ' ',
           g_initialized ? g_lqr_ctrl.params.lqr_u_enable : 0.0f);
    printf("  [%c] Y - YAW (turning)      : %s%s\n", 
           (g_loop_enable_mask & LOOP_YAW) ? 'X' : ' ',
           g_yaw_control_enabled ? "ON" : "OFF",
           g_yaw_force_enable ? " [FORCE]" : "");
    printf("==========================\n");
    printf("Mask: 0x%02X\n", g_loop_enable_mask);
    printf("Presets: simple=0x03, full=0x3F\n");
    printf("Tip: Use 'balance loop full' to return to auto mode\n\n");
}

// ============================================================================
// 腿部电机固定角度控制
// ============================================================================

/**
 * @brief 使能/禁用腿部电机控制
 * @param enable true=使能, false=禁用
 */
void balance_test_set_leg_control(bool enable) {
    if (enable && !g_leg_control_enabled) {
        // 使能腿部电机
        int enabled_count = 0;
        if (g_motor_left_hip) {
            can_motor_set_mode(g_motor_left_hip, MODE_POS_FILTER);
            can_motor_enter_closed_loop(g_motor_left_hip);
            ESP_LOGI(TAG, "  Left Hip (ID1) enabled");
            enabled_count++;
        } else {
            ESP_LOGW(TAG, "  Left Hip (ID1) NOT available");
        }
        if (g_motor_left_knee) {
            can_motor_set_mode(g_motor_left_knee, MODE_POS_FILTER);
            can_motor_enter_closed_loop(g_motor_left_knee);
            ESP_LOGI(TAG, "  Left Knee (ID2) enabled");
            enabled_count++;
        } else {
            ESP_LOGW(TAG, "  Left Knee (ID2) NOT available");
        }
        if (g_motor_right_hip) {
            can_motor_set_mode(g_motor_right_hip, MODE_POS_FILTER);
            can_motor_enter_closed_loop(g_motor_right_hip);
            ESP_LOGI(TAG, "  Right Hip (ID4) enabled");
            enabled_count++;
        } else {
            ESP_LOGW(TAG, "  Right Hip (ID4) NOT available");
        }
        if (g_motor_right_knee) {
            can_motor_set_mode(g_motor_right_knee, MODE_POS_FILTER);
            can_motor_enter_closed_loop(g_motor_right_knee);
            ESP_LOGI(TAG, "  Right Knee (ID5) enabled");
            enabled_count++;
        } else {
            ESP_LOGW(TAG, "  Right Knee (ID5) NOT available");
        }
        ESP_LOGI(TAG, "Leg motors ENABLED (%d/4 motors)", enabled_count);

        // 使能后自动设置初始姿态: 110mm, 0°
        g_leg_control_enabled = true;  // 先置 true 让 leg_ctrl_set_target 生效
        leg_ctrl_set_target(true,  0.110f, 0.0f);
        leg_ctrl_set_target(false, 0.110f, 0.0f);
        g_leg_base_length = 0.110f;
        g_leg_base_angle  = 0.0f;
        ESP_LOGI(TAG, "Leg target: 110mm, 0 deg");
    } else if (!enable && g_leg_control_enabled) {
        // 禁用腿部电机
        if (g_motor_left_hip) can_motor_set_idle(g_motor_left_hip);
        if (g_motor_left_knee) can_motor_set_idle(g_motor_left_knee);
        if (g_motor_right_hip) can_motor_set_idle(g_motor_right_hip);
        if (g_motor_right_knee) can_motor_set_idle(g_motor_right_knee);
        ESP_LOGI(TAG, "Leg motors DISABLED");
    }
    g_leg_control_enabled = enable;
}

/**
 * @brief 设置腿部电机角度
 * @param left_hip 左大腿角度 (度)
 * @param left_knee 左小腿角度 (度)
 * @param right_hip 右大腿角度 (度)
 * @param right_knee 右小腿角度 (度)
 */
void balance_test_set_leg_angles(float left_hip, float left_knee, 
                                  float right_hip, float right_knee) {
    g_leg_left_hip_angle = left_hip;
    g_leg_left_knee_angle = left_knee;
    g_leg_right_hip_angle = right_hip;
    g_leg_right_knee_angle = right_knee;
    
    ESP_LOGI(TAG, "Leg angles set: L_Hip=%.1f, L_Knee=%.1f, R_Hip=%.1f, R_Knee=%.1f",
             left_hip, left_knee, right_hip, right_knee);
}

/**
 * @brief 设置腿部电机运动速度
 * @param speed 速度 (rpm)
 */
void balance_test_set_leg_speed(float speed) {
    g_leg_move_speed = speed;
    ESP_LOGI(TAG, "Leg move speed set to %.1f rpm", speed);
}


// Forward declaration for leg sync

/**
 * @brief 发送腿部电机命令 (在 motor_comm 任务中调用)
 * @note 支持两种模式:
 *       - 位置控制模式 (!g_vmc_enabled): 发送位置命令
 *       - VMC 力控模式 (g_vmc_enabled): 发送已计算好的扭矩命令
 */
void apply_leg_motor_commands(void) {
    if (!g_leg_control_enabled) return;
    
    if (g_vmc_enabled && g_vmc_input_valid) {
        // ===== VMC 力控模式: 发送已计算好的扭矩 =====
        // STW 电机: 使用 MIT 前馈电流控制 (kp=0, kd=0), 更直接的力矩控制
        // JuCi 电机: 使用 0xC0 Q轴电流命令 + 非线性补偿
        
        // 关节力矩安全限幅 (在发送电机命令前)
        if (g_vmc_dual_output.left.hip_torque > HIP_TORQUE_LIMIT) g_vmc_dual_output.left.hip_torque = HIP_TORQUE_LIMIT;
        if (g_vmc_dual_output.left.hip_torque < -HIP_TORQUE_LIMIT) g_vmc_dual_output.left.hip_torque = -HIP_TORQUE_LIMIT;
        if (g_vmc_dual_output.left.knee_torque > KNEE_TORQUE_LIMIT) g_vmc_dual_output.left.knee_torque = KNEE_TORQUE_LIMIT;
        if (g_vmc_dual_output.left.knee_torque < -KNEE_TORQUE_LIMIT) g_vmc_dual_output.left.knee_torque = -KNEE_TORQUE_LIMIT;
        if (g_vmc_dual_output.right.hip_torque > HIP_TORQUE_LIMIT) g_vmc_dual_output.right.hip_torque = HIP_TORQUE_LIMIT;
        if (g_vmc_dual_output.right.hip_torque < -HIP_TORQUE_LIMIT) g_vmc_dual_output.right.hip_torque = -HIP_TORQUE_LIMIT;
        if (g_vmc_dual_output.right.knee_torque > KNEE_TORQUE_LIMIT) g_vmc_dual_output.right.knee_torque = KNEE_TORQUE_LIMIT;
        if (g_vmc_dual_output.right.knee_torque < -KNEE_TORQUE_LIMIT) g_vmc_dual_output.right.knee_torque = -KNEE_TORQUE_LIMIT;
        
        if (g_motor_left_hip) {
            can_motor_stw_mit_control(g_motor_left_hip, 0, 0, 0, 0,
                g_vmc_dual_output.left.hip_torque);
        }
        if (g_motor_left_knee) {
            can_motor_stw_mit_control(g_motor_left_knee, 0, 0, 0, 0,
                g_vmc_dual_output.left.knee_torque);
        }
        if (g_motor_right_hip) {
            can_motor_stw_mit_control(g_motor_right_hip, 0, 0, 0, 0,
                g_vmc_dual_output.right.hip_torque);
        }
        if (g_motor_right_knee) {
            can_motor_stw_mit_control(g_motor_right_knee, 0, 0, 0, 0,
                g_vmc_dual_output.right.knee_torque);
        }
        // MIT 反馈帧精度有限 (±pos_max 范围编码), 主动发送 0xA3 获取多圈绝对角度
        // 确保 FK / leg_sync 使用精确位置数据
        if (g_motor_left_hip)   can_motor_request_angle(g_motor_left_hip);
        if (g_motor_left_knee)  can_motor_request_angle(g_motor_left_knee);
        if (g_motor_right_hip)  can_motor_request_angle(g_motor_right_hip);
        if (g_motor_right_knee) can_motor_request_angle(g_motor_right_knee);
    } else if (g_jump_mit_active) {
        // ===== 跳跃蹬伸模式: 大腿位置控制 + 小腿 MIT 前馈 =====
        // 大腿用位置控制 (更稳定, 不过冲), 小腿用 MIT 前馈加速蹬伸
        if (g_motor_left_hip) {
            can_motor_set_position(g_motor_left_hip, g_leg_left_hip_angle, g_leg_move_speed);
        }
        if (g_motor_left_knee) {
            can_motor_stw_mit_control(g_motor_left_knee,
                g_jump_mit_target_rad[1], 0,
                g_jump_mit_kp, g_jump_mit_kd, g_jump_mit_ff_torque * g_jump_mit_ff_sign[1]);
        }
        if (g_motor_right_hip) {
            can_motor_set_position(g_motor_right_hip, g_leg_right_hip_angle, g_leg_move_speed);
        }
        if (g_motor_right_knee) {
            can_motor_stw_mit_control(g_motor_right_knee,
                g_jump_mit_target_rad[3], 0,
                g_jump_mit_kp, g_jump_mit_kd, g_jump_mit_ff_torque * g_jump_mit_ff_sign[3]);
        }
    } else if (g_standup_mit_active) {
        // ===== 起身 MIT 模式: 选择性逐电机控制 =====
        // 仅激活的 hip 发 MIT (kp=0,kd=0,纯前馈), 其余关节保持位置控制
        if (g_motor_left_hip) {
            if (g_standup_mit_left_hip) {
                can_motor_stw_mit_control(g_motor_left_hip,
                    g_standup_mit_target_left_hip, 0,
                    g_standup_mit_kp, g_standup_mit_kd,
                    g_standup_mit_ff_torque * g_standup_ff_sign_left);
            } else {
                can_motor_set_position(g_motor_left_hip, g_leg_left_hip_angle, g_leg_move_speed);
            }
        }
        if (g_motor_left_knee) {
            can_motor_set_position(g_motor_left_knee, g_leg_left_knee_angle, g_leg_move_speed);
        }
        if (g_motor_right_hip) {
            if (g_standup_mit_right_hip) {
                can_motor_stw_mit_control(g_motor_right_hip,
                    g_standup_mit_target_right_hip, 0,
                    g_standup_mit_kp, g_standup_mit_kd,
                    g_standup_mit_ff_torque * g_standup_ff_sign_right);
            } else {
                can_motor_set_position(g_motor_right_hip, g_leg_right_hip_angle, g_leg_move_speed);
            }
        }
        if (g_motor_right_knee) {
            can_motor_set_position(g_motor_right_knee, g_leg_right_knee_angle, g_leg_move_speed);
        }
    } else if (!g_vmc_enabled) {
        // ===== 位置控制模式 =====
        
        // ---- Leg Sync: 左右腿同步补偿 (防劈叉) ----
        float left_hip_cmd = g_leg_left_hip_angle;
        float left_knee_cmd = g_leg_left_knee_angle;
        float right_hip_cmd = g_leg_right_hip_angle;
        float right_knee_cmd = g_leg_right_knee_angle;
        
        if (g_leg_sync_enabled) {
            // 读取左右腿实际 body_angle (从电机缓存，无阻塞)
            leg_state_t left_state, right_state;
            if (leg_ctrl_get_state_cached(true, &left_state) == ESP_OK &&
                leg_ctrl_get_state_cached(false, &right_state) == ESP_OK &&
                left_state.valid && right_state.valid) {
                
                float left_actual = left_state.workspace.body_angle;
                float right_actual = right_state.workspace.body_angle;
                float angle_diff = left_actual - right_actual;  // 正值=左腿比右腿更偏前
                
                // 计算修正量 (按比例修正差异)
                float correction = angle_diff * g_leg_sync_gain;
                
                // 限幅
                if (correction > g_leg_sync_max_correction) correction = g_leg_sync_max_correction;
                if (correction < -g_leg_sync_max_correction) correction = -g_leg_sync_max_correction;
                
                // 保存调试值
                g_leg_sync_debug_diff = angle_diff;
                g_leg_sync_debug_correction = correction;
                
                // 应用修正: 通过修改 body_angle → 重新 IK
                // 左腿角度偏大 → 减小左腿目标角度, 增大右腿目标角度
                if (fabsf(correction) > 0.1f) {
                    float left_target_angle = g_leg_left_target_angle - correction;
                    float right_target_angle = g_leg_right_target_angle + correction;
                    
                    // 重新 IK 计算修正后的关节角度
                    leg_workspace_state_t left_ws = { .leg_length = g_leg_left_target_length, .body_angle = left_target_angle };
                    leg_workspace_state_t right_ws = { .leg_length = g_leg_right_target_length, .body_angle = right_target_angle };
                    
                    leg_kin_clamp_workspace(&left_ws.leg_length, &left_ws.body_angle, NULL);
                    leg_kin_clamp_workspace(&right_ws.leg_length, &right_ws.body_angle, NULL);
                    
                    leg_joint_state_t left_joint, right_joint;
                    if (leg_kin_inverse(&left_ws, true, NULL, &left_joint) == ESP_OK) {
                        left_hip_cmd = left_joint.hip_angle;
                        left_knee_cmd = left_joint.knee_angle;
                    }
                    if (leg_kin_inverse(&right_ws, false, NULL, &right_joint) == ESP_OK) {
                        right_hip_cmd = right_joint.hip_angle;
                        right_knee_cmd = right_joint.knee_angle;
                    }
                }
            }
        } else {
            g_leg_sync_debug_diff = 0.0f;
            g_leg_sync_debug_correction = 0.0f;
        }
        
        if (g_motor_left_hip) {
            can_motor_set_position(g_motor_left_hip, left_hip_cmd, g_leg_move_speed);
        }
        if (g_motor_left_knee) {
            can_motor_set_position(g_motor_left_knee, left_knee_cmd, g_leg_move_speed);
        }
        if (g_motor_right_hip) {
            can_motor_set_position(g_motor_right_hip, right_hip_cmd, g_leg_move_speed);
        }
        if (g_motor_right_knee) {
            can_motor_set_position(g_motor_right_knee, right_knee_cmd, g_leg_move_speed);
        }
        // ===== 位置模式电流轮询 =====
        // 0xC2 位置命令的应答只含角度，需额外请求 0xA1 获取 Q 轴电流
        // 否则 can_motor_read_current() 始终为 0，支持力估算无效
        if (g_motor_left_hip)   can_motor_request_current(g_motor_left_hip);
        if (g_motor_left_knee)  can_motor_request_current(g_motor_left_knee);
        if (g_motor_right_hip)  can_motor_request_current(g_motor_right_hip);
        if (g_motor_right_knee) can_motor_request_current(g_motor_right_knee);
    }
}
// ============================================================================
// Roll 闭环控制开关
// ============================================================================

/**
 * @brief 使能/禁用 Roll 闭环控制
 * @param enable true=使能, false=禁用
 * @note Roll 控制与腿长相关，纯轮电机测试时建议禁用
 */
void balance_test_set_roll_control(bool enable) {
    g_roll_control_enabled = enable;
    ESP_LOGI(TAG, "Roll control %s", enable ? "ENABLED" : "DISABLED");
}

/**
 * @brief 获取 Roll 闭环控制状态
 * @return true=已使能, false=已禁用
 */
bool balance_test_get_roll_control(void) {
    return g_roll_control_enabled;
}

// ============================================================================
// Pitch 腿部角度补偿开关
// ============================================================================

/**
 * @brief 使能/禁用 Pitch 腿部角度补偿
 * @param enable true=使能, false=禁用
 * @note 禁用后直接使用 IMU 原始 pitch，不考虑腿部角度
 */
void balance_test_set_pitch_comp(bool enable) {
    g_pitch_leg_comp_enabled = enable;
    ESP_LOGI(TAG, "Pitch leg compensation %s", enable ? "ENABLED" : "DISABLED");
}

/**
 * @brief 获取 Pitch 腿部角度补偿状态
 * @return true=已使能, false=已禁用
 */
bool balance_test_get_pitch_comp(void) {
    return g_pitch_leg_comp_enabled;
}

// ============================================================================
// X-Offset 腿部速度自适应偏移 API
// ============================================================================

/**
 * @brief 使能/禁用 X-Offset
 */
void balance_test_set_xoffset(bool enable) {
    g_xoffset_enabled = enable;
    if (!enable) {
        pid_reset(&g_xoffset_pid);
        g_xoffset_value = 0.0f;
    }
    ESP_LOGI(TAG, "X-Offset %s", enable ? "ENABLED" : "DISABLED");
}

/**
 * @brief 获取 X-Offset 状态
 */
bool balance_test_get_xoffset(void) {
    return g_xoffset_enabled;
}

/**
 * @brief 设置 X-Offset PID 增益
 */
void balance_test_set_xoffset_pid(float kp, float ki, float kd) {
    pid_set_gains(&g_xoffset_pid, kp, ki, kd);
    ESP_LOGI(TAG, "X-Offset PID: Kp=%.4f Ki=%.4f Kd=%.4f", kp, ki, kd);
}

/**
 * @brief 设置 X-Offset 限幅
 */
void balance_test_set_xoffset_limit(float limit) {
    if (limit < 0.001f) limit = 0.001f;
    if (limit > 0.08f) limit = 0.08f;  // 最大 8cm
    g_xoffset_limit = limit;
    pid_set_output_limits(&g_xoffset_pid, -limit, limit);
    ESP_LOGI(TAG, "X-Offset limit: %.3f m", limit);
}

// ============================================================================
// Leg Sync (防劈叉) API
// ============================================================================

/**
 * @brief 使能/禁用 Leg Sync
 */
void balance_test_set_leg_sync(bool enable) {
    g_leg_sync_enabled = enable;
    if (!enable) {
        g_leg_sync_debug_diff = 0.0f;
        g_leg_sync_debug_correction = 0.0f;
    }
    ESP_LOGI(TAG, "Leg Sync %s (gain=%.2f max=%.1f°)",
             enable ? "ENABLED" : "DISABLED",
             g_leg_sync_gain, g_leg_sync_max_correction);
}

/**
 * @brief 获取 Leg Sync 状态
 */
bool balance_test_get_leg_sync(void) {
    return g_leg_sync_enabled;
}

/**
 * @brief 设置 Leg Sync 增益
 */
void balance_test_set_leg_sync_gain(float gain) {
    if (gain < 0.0f) gain = 0.0f;
    if (gain > 1.0f) gain = 1.0f;
    g_leg_sync_gain = gain;
    ESP_LOGI(TAG, "Leg Sync gain: %.2f", gain);
}

/**
 * @brief 设置 Leg Sync 最大修正量
 */
void balance_test_set_leg_sync_max(float max_deg) {
    if (max_deg < 1.0f) max_deg = 1.0f;
    if (max_deg > 45.0f) max_deg = 45.0f;
    g_leg_sync_max_correction = max_deg;
    ESP_LOGI(TAG, "Leg Sync max correction: %.1f°", max_deg);
}

// ============================================================================
