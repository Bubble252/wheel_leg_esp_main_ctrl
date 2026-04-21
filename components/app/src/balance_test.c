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

// IMU 数据 (由 task_imu_read 写入, task_balance_ctrl 读取)
typedef struct {
    float pitch;            // 俯仰角 (度)
    float pitch_rate;       // 俯仰角速度 (度/秒)
    float roll;             // 横滚角 (度)
    float roll_rate;        // 横滚角速度 (度/秒)
    float yaw;              // 偏航角 (度)
    float yaw_rate;         // 偏航角速度 (度/秒)
    float accel_x;          // 加速度 X (g)
    float accel_y;          // 加速度 Y (g)
    float accel_z;          // 加速度 Z (g)
    uint32_t timestamp;     // 时间戳 (ms)
    uint64_t read_time_us;  // 精确读取时间 (us) - 用于延迟测量
    bool valid;             // 数据有效
} shared_imu_data_t;

// 遥控数据 (由 WiFi 回调写入, task_balance_ctrl 读取)
typedef struct {
    int16_t joy_x;          // 摇杆 X (-100~100) 转向
    int16_t joy_y;          // 摇杆 Y (-100~100) 速度
    int16_t joy_x_last;     // 上一次摇杆 X
    int16_t joy_y_last;     // 上一次摇杆 Y
    bool go;                // 使能开关
    bool car_mode;          // 小车模式开关 (来自 Web UI)
    uint32_t last_update;   // 最后更新时间 (ms)
    uint64_t receive_time_us; // 精确接收时间 (us) - 用于延迟测量
} shared_remote_data_t;

// 轮电机命令 (由 task_balance_ctrl 写入, task_motor_comm 读取)
typedef struct {
    float left_torque;      // 左轮力矩 (扭矩模式) / 左轮速度 (速度模式)
    float right_torque;     // 右轮力矩 (扭矩模式) / 右轮速度 (速度模式)
    bool enabled;           // 使能标志
    bool use_speed_mode;    // true=速度模式, false=扭矩模式
} shared_wheel_cmd_t;

// 轮电机状态 (由 task_motor_comm 写入, task_balance_ctrl 读取)
typedef struct {
    float left_position;    // 左轮位置 (度)
    float right_position;   // 右轮位置 (度)
    float left_speed;       // 左轮速度 (rpm)
    float right_speed;      // 右轮速度 (rpm)
    bool left_online;       // 左轮在线
    bool right_online;      // 右轮在线
    uint32_t timestamp;     // 时间戳 (ms)
} shared_wheel_state_t;

// ============================================================================
// 全局变量
// ============================================================================

// 共享数据
static shared_imu_data_t g_imu_data = {0};
static shared_remote_data_t g_remote_data = {0};
static shared_wheel_cmd_t g_wheel_cmd = {0};
static shared_wheel_state_t g_wheel_state = {0};

// 互斥锁
static SemaphoreHandle_t g_imu_mutex = NULL;
static SemaphoreHandle_t g_remote_mutex = NULL;
static SemaphoreHandle_t g_wheel_cmd_mutex = NULL;
static SemaphoreHandle_t g_wheel_state_mutex = NULL;

// LQR 控制器
static lqr_controller_t g_lqr_ctrl;

// 电机句柄
static can_motor_handle_t g_motor_left = NULL;   // ID=3
static can_motor_handle_t g_motor_right = NULL;  // ID=6

// 任务句柄
static TaskHandle_t g_task_imu = NULL;
static TaskHandle_t g_task_balance = NULL;
static TaskHandle_t g_task_motor = NULL;
static TaskHandle_t g_task_watchdog = NULL;
static TaskHandle_t g_task_unified = NULL;        // 合并任务句柄
static TaskHandle_t g_task_observer = NULL;       // 观测器独立任务句柄

// 任务架构选择
static bool g_use_unified_task = true;            // true=使用合并任务(默认), false=使用分离任务

// 功能开关
static bool g_uncontrolable_check_enabled = true;  // true=启用失控检测(默认), false=禁用失控检测

// 控制模式选择
typedef enum {
    CTRL_MODE_LQR = 0,      // LQR 多环控制 (默认)
    CTRL_MODE_DUAL_PID,     // 双环 PID 控制 (直立环+速度环) - 扭矩模式
    CTRL_MODE_SINGLE_PID,   // 单环 PID 控制 (直立环→速度) - 速度模式
    CTRL_MODE_CAR,          // 普通小车模式 (无直立环, 趴下跑)
    CTRL_MODE_TRIPLE_PID,   // 三环 PID 控制 (速度环→角度环→轮速环)
    CTRL_MODE_FULL_LQR,     // 完整 LQR 控制 (同时输出 T 和 Tp, K 随腿长插值)
} control_mode_t;
static control_mode_t g_control_mode = CTRL_MODE_TRIPLE_PID;  // 默认三环 PID 模式

// 普通小车模式参数
#define CAR_MODE_BODY_ANGLE     (-130.0f)    // 小车模式身体夹角 (度), 趴下
#define CAR_MODE_LEG_LENGTH     (0.067f)    // 小车模式腿长 (米)
#define CAR_MODE_MAX_SPEED      (200.0f)    // 小车模式最大速度 (rpm)
#define CAR_MODE_YAW_GAIN       (80.0f)     // 小车模式转向增益 (rpm per joy_x unit)
static control_mode_t g_car_mode_prev_mode = CTRL_MODE_LQR;  // 进入小车模式前的模式 (用于退出时恢复)
static float g_car_mode_prev_base_angle = -90.0f;            // 进入小车模式前的身体夹角
static float g_car_mode_prev_base_length = 0.09f;            // 进入小车模式前的腿长

// 双环 PID 控制器
static dual_pid_controller_t g_dual_pid_ctrl;
static bool g_dual_pid_initialized = false;
static dual_pid_output_t g_dual_pid_output;       // 保存输出用于调试

// 三环 PID 控制器 (速度环→角度环→轮速环)
static triple_pid_controller_t g_triple_pid_ctrl;
static bool g_triple_pid_initialized = false;
static triple_pid_output_t g_triple_pid_output;   // 保存输出用于调试

// 完整 LQR 控制器 (同时输出 T 和 Tp, K 随腿长插值)
static bool g_auto_enable_inhibited = true;        // true=禁止自动使能, 需手动 balance enable

static full_lqr_controller_t g_full_lqr_ctrl;
static bool g_full_lqr_initialized = false;
static full_lqr_output_t g_full_lqr_output_left;   // 左腿输出 (调试)
static full_lqr_output_t g_full_lqr_output_right;  // 右腿输出 (调试)
static bool g_full_lqr_stream_enable = false;       // #FLQR 数据流开关
static float g_full_lqr_Tp_left = 0.0f;             // 左腿 Tp 输出 (用于 VMC 注入)
static float g_full_lqr_Tp_right = 0.0f;            // 右腿 Tp 输出 (用于 VMC 注入)
static float g_full_lqr_wheel_T_left = 0.0f;        // 左轮 T 输出 (调试)
static float g_full_lqr_wheel_T_right = 0.0f;       // 右轮 T 输出 (调试)

// 轮速加权滑动平均滤波器 (用于双环/三环 PID 模式)
static weighted_ma_filter_t g_wheel_speed_wma;
static bool g_wma_enabled = true;    // 默认开启 WMA 滤波

// 单环 PID 控制器 (输出速度，适合电机速度模式)
static single_pid_controller_t g_single_pid_ctrl;
static bool g_single_pid_initialized = false;
static single_pid_output_t g_single_pid_output;   // 保存输出用于调试

// 状态
static balance_test_state_t g_state = BALANCE_TEST_IDLE;
static bool g_initialized = false;
static bool g_tasks_running = false;

// 统计
static balance_test_stats_t g_stats = {0};
static uint32_t g_imu_count_per_sec = 0;
static uint32_t g_ctrl_count_per_sec = 0;
static uint32_t g_motor_count_per_sec = 0;
static uint32_t g_leg_count_per_sec = 0;
static uint32_t g_last_stat_time = 0;

// 延迟诊断 (微秒级) - 精确测量
// 核心思路: 追踪实际被使用的 IMU 数据从读取到电机发送的真实延迟
static volatile uint64_t g_used_imu_time_us = 0;      // 控制任务实际使用的 IMU 数据的读取时间
static volatile uint64_t g_ctrl_start_time_us = 0;    // 控制计算开始时间
static volatile uint64_t g_ctrl_end_time_us = 0;      // 控制计算结束时间
static volatile uint64_t g_motor_send_time_us = 0;    // 电机命令发送时间
static float g_latency_imu_to_ctrl_us = 0.0f;         // IMU数据等待时间 (数据读取 → 控制开始使用)
static float g_latency_ctrl_calc_us = 0.0f;           // 控制计算耗时
static float g_latency_ctrl_to_motor_us = 0.0f;       // 控制输出等待时间 (控制完成 → 电机发送)
static float g_latency_total_us = 0.0f;               // 总延迟 (IMU读取 → 电机发送)
static uint32_t g_latency_sample_count = 0;           // 延迟采样计数
static float g_latency_total_avg_us = 0.0f;           // 总延迟平均值
static float g_latency_total_max_us = 0.0f;           // 总延迟最大值
static float g_latency_total_min_us = 999999.0f;      // 总延迟最小值

// WiFi 遥控延迟诊断 (微秒级)
static volatile uint64_t g_used_wifi_time_us = 0;     // 控制任务实际使用的 WiFi 数据的接收时间
static float g_latency_wifi_to_ctrl_us = 0.0f;        // WiFi数据等待时间 (WiFi接收 → 控制使用)
static float g_latency_wifi_total_us = 0.0f;          // WiFi总延迟 (WiFi接收 → 电机发送)
static float g_latency_wifi_avg_us = 0.0f;            // WiFi延迟平均值
static float g_latency_wifi_max_us = 0.0f;            // WiFi延迟最大值
static float g_latency_wifi_min_us = 999999.0f;       // WiFi延迟最小值
static uint32_t g_latency_wifi_sample_count = 0;      // WiFi延迟采样计数

// LQR 内部状态 (来自 shibo_wheel_leg)
static float g_lqr_distance = 0.0f;         // 累积位移 (机器人前进方向为正)
static float g_lqr_speed = 0.0f;            // 当前速度 (机器人前进方向为正)
static float g_distance_zeropoint = 0.0f;   // 位移零点
static float g_angle_zeropoint = 0.0f;      // 角度零点 (需要根据实际机器人调整)
static int g_move_stop_flag = 0;            // 停止标志
static int g_uncontrolable = 0;             // 失控标志

// 遥杆映射比例 (可通过 UI/CLI 在线调节)
static float g_joy_speed_scale = 0.003f;    // joy_y → target_speed 比例 (默认 0.003, max ±0.3)
static float g_joy_yaw_scale = 0.9f;       // joy_x → target_yaw_rate 比例 (默认 0.9)
static float g_tpid_yaw_scale = 500.0f;     // 三环PID yaw输出缩放 (LQR yaw输出太小, 需放大)

// 轮速加速度计算 (用于离地检测)
static float g_left_wheel_speed_rad = 0.0f;     // 左轮速度 (rad/s)
static float g_right_wheel_speed_rad = 0.0f;    // 右轮速度 (rad/s)
static float g_left_wheel_accel = 0.0f;         // 左轮加速度 (rad/s²)
static float g_right_wheel_accel = 0.0f;        // 右轮加速度 (rad/s²)
static float g_prev_left_wheel_speed = 0.0f;    // 上次左轮速度
static float g_prev_right_wheel_speed = 0.0f;   // 上次右轮速度
static bool g_wheel_off_ground = false;          // 轮子离地标志
static int g_off_ground_counter = 0;             // 离地检测去抖计数器

// 支持力离地检测去抖 (左右腿独立)
static int g_sforce_left_off_cnt  = 0;           // 左腿支持力离地计数器
static int g_sforce_right_off_cnt = 0;           // 右腿支持力离地计数器
static bool g_sforce_left_off  = false;          // 左轮小力离地标志
static bool g_sforce_right_off = false;          // 右轮小力离地标志
#define SFORCE_FL_THRESHOLD   1.0f              // F_L 离地判定阈値 (N)
#define SFORCE_OFF_ENTER_CNT  5                 // 连续 5 帧判定离地 (10ms@500Hz)
#define SFORCE_OFF_EXIT_CNT   10                // 连续 10 帧判定着地 (20ms@500Hz)

// 离地检测去抖参数
#define OFF_GROUND_ENTER_COUNT  8   // 连续 8 帧判定离地 (16ms@500Hz)
#define OFF_GROUND_EXIT_COUNT   13  // 连续 13 帧判定着地 (26ms@500Hz)

// YAW 轴控制 (带过零处理)
static float g_yaw_angle_last = 0.0f;       // 上一次 YAW 角度 (用于过零处理)
static float g_yaw_angle_total = 0.0f;      // YAW 累积角度 (经过过零处理)
static float g_yaw_output = 0.0f;           // YAW 控制输出
static bool g_yaw_first_run = true;         // YAW 首次运行标志

// Roll 控制 (腿长调节)
static float g_roll_output = 0.0f;          // Roll 控制原始输出
static float g_roll_left_delta = 0.0f;      // 左腿长度增量
static float g_roll_right_delta = 0.0f;     // 右腿长度增量
static float g_roll_filtered = 0.0f;        // 滤波后的 Roll 角度

// 波形数据输出 (用于 Qt 调参面板)
static bool g_plot_enabled = false;         // 波形输出使能
static uint8_t g_plot_divider = 10;         // 输出分频 (每N次控制循环输出一次)
static uint8_t g_plot_counter = 0;          // 分频计数器
static float g_last_lqr_u = 0.0f;           // 保存 LQR 输出用于波形显示
static uint32_t g_plot_channel_mask = 0xFFFFFFFF;  // 通道使能掩码 (默认全开)

// 通道ID到bit位的映射: A=0, B=1, ..., Y=24, W=22
static inline uint32_t plot_ch_bit(char ch) {
    if (ch >= 'A' && ch <= 'Z') return 1U << (ch - 'A');
    return 0;
}
#define PLOT_CH_ENABLED(ch) (g_plot_channel_mask & plot_ch_bit(ch))

// PID 调试输出 (用于实时 debug)
static bool g_pid_debug_enabled = false;    // PID 调试输出使能
static uint8_t g_pid_debug_divider = 50;    // 输出分频 (每N次控制循环输出一次，默认约4Hz@200Hz)
static uint8_t g_pid_debug_counter = 0;     // 分频计数器

// 环路使能控制 (用于单环调试)
static uint8_t g_loop_enable_mask = LOOP_FULL;  // 默认全部启用
static bool g_yaw_control_enabled = false;      // YAW 控制独立开关 (默认关, 需手动开启)
static bool g_yaw_force_enable = false;         // YAW 强制使能 (无需遥控器 go, 通过 CLI 控制)
static bool g_diff_speed_enabled = false;       // 差速转向使能 (Yaw失能时的开环差速)
static float g_diff_speed_scale = 0.9f;        // 差速转向增益 (joy_x * scale)
static bool g_loop_manual_mode = false;         // 手动模式 (禁止自动切换 simple/full)

// ============================================================================
// 腿部电机控制
// ============================================================================
static bool g_leg_control_enabled = false;  // 腿部电机使能
static can_motor_handle_t g_motor_left_hip = NULL;    // ID=1 左大腿
static can_motor_handle_t g_motor_left_knee = NULL;   // ID=2 左小腿
static can_motor_handle_t g_motor_right_hip = NULL;   // ID=4 右大腿
static can_motor_handle_t g_motor_right_knee = NULL;  // ID=5 右小腿

// 腿部电机目标角度 (度) - 由 leg_ctrl_init() 通过逆运动学计算
static float g_leg_left_hip_angle = 0.0f;    // 左大腿角度
static float g_leg_left_knee_angle = 0.0f;   // 左小腿角度
static float g_leg_right_hip_angle = 0.0f;   // 右大腿角度
static float g_leg_right_knee_angle = 0.0f;  // 右小腿角度
static float g_leg_move_speed = 50.0f;        // 腿部电机运动速度 (rpm)

// 腿长范围限制 (可通过 UI 或 CLI 调节)
static float g_leg_length_min = 0.065f;       // 最小腿长 (m), 默认与 LEG_LENGTH_MIN 一致
static float g_leg_length_max = 0.11f;        // 最大腿长 (m), 默认与 LEG_LENGTH_MAX 一致

// 腿部目标状态 (运动学空间)
// 基础腿长/角度: 用户设定的"高度"，Roll控制不修改这些值
static float g_leg_base_length = 0.09f;           // 基础腿长 (米) - 决定机器人高度
static float g_leg_base_angle = -90.0f;          // 基础身体夹角 (度), -90=垂直向下

// 实际发送给电机的目标 (= 基础值 + Roll调整)
static float g_leg_left_target_length = 0.09f;   // 左腿实际目标腿长 (米)
static float g_leg_left_target_angle = -90.0f;   // 左腿实际目标身体夹角 (度)
static float g_leg_right_target_length = 0.09f;  // 右腿实际目标腿长 (米)
static float g_leg_right_target_angle = -90.0f;  // 右腿实际目标身体夹角 (度), -90=垂直向下

// Roll 闭环控制开关 (与腿长相关，纯轮测试时禁用)
static bool g_roll_control_enabled = false;  // 默认禁用

// Pitch 腿部角度补偿开关
static bool g_pitch_leg_comp_enabled = false; // 默认禁用腿部角度补偿

// ============================================================================
// 零点自适应 PID 调试变量
// ============================================================================
static float g_zp_speed_threshold = 0.1f;         // 轮速阈值 (m/s), 低于此值才启用零点自适应
static float g_zp_pitch_for_ctrl = 0.0f;         // 当前 pitch_for_control (用于零点调试)
static float g_zp_angle_error = 0.0f;            // 零点自适应的角度误差
static float g_zp_pid_raw = 0.0f;                // PID 原始输出 (滤波前)
static float g_zp_pid_filtered = 0.0f;           // PID 滤波后输出
static bool  g_zp_active = false;                // 是否进入了零点PID计算 (轮速<阈值)

// ============================================================================
// X-Offset 腿部速度自适应偏移 (独立腿部姿态控制)
// ============================================================================
// 功能: 根据当前轮速，用 PID 控制腿脚在笛卡尔 x 方向的偏移
//   速度>0 (前进) → x_offset>0 (腿脚后摆) → 类似人跑步时支撑腿后蹬
//   速度=0 → x_offset=0 (腿回中位)
// 与平衡控制完全独立，只改变腿的几何形状，不影响轮力矩
static bool g_xoffset_enabled = false;           // X-Offset 使能开关
static pid_controller_t g_xoffset_pid;           // X-Offset PID 控制器
static float g_xoffset_value = 0.0f;             // 当前 x_offset 输出 (米)
static float g_xoffset_limit = 0.03f;            // X-Offset 限幅 (米), 默认 ±3cm
static float g_xoffset_debug_speed = 0.0f;       // 调试用: 当时的速度输入

// ============================================================================
// Leg Sync 左右腿同步控制 (防劈叉)
// ============================================================================
// 功能: 读取左右腿实际 body_angle，用交叉耦合补偿消除差异
//   左右腿角度差 → 按比例修正各自的目标角度
//   相当于在两条腿之间加一根"虚拟弹簧"
// 与 X-Offset 正交: X-Offset 是两腿同方向偏移，Sync 是反方向补偿
static bool g_leg_sync_enabled = false;           // Leg Sync 使能开关
static float g_leg_sync_gain = 0.3f;              // 同步增益 (0~1), 0.3 = 修正30%的差异
static float g_leg_sync_max_correction = 15.0f;   // 最大修正量 (度), 防止过大跳变
static float g_leg_sync_debug_diff = 0.0f;        // 调试: 左右腿角度差 (度)
static float g_leg_sync_debug_correction = 0.0f;  // 调试: 实际修正量 (度)

// ============================================================================
// 跳跃状态机
// ============================================================================
typedef enum {
    JUMP_IDLE = 0,          // 空闲 (等待指令)
    JUMP_CROUCH,            // 蹲下蓄力 (腿长→68mm)
    JUMP_EXTEND,            // 蹬伸起跳 (腿长→110mm)
    JUMP_AIR_RETRACT,       // 空中收腿 (腿长→68mm, 轮速=0)
    JUMP_RECOVER,           // 着地恢复 (恢复正常腿长, 恢复平衡)
} jump_state_t;

static jump_state_t g_jump_state = JUMP_IDLE;
static uint32_t g_jump_state_enter_ms = 0;         // 进入当前状态的时间 (ms)
static float g_jump_saved_leg_length = 0.09f;      // 跳跃前保存的腿长 (m)
static float g_jump_saved_base_angle = -90.0f;      // 跳跃前保存的身体夹角 (度)
static bool g_jump_last_btn = false;                // 上一帧跳跃按钮状态 (用于上升沿检测)

// 跳跃参数 (可调)
static float g_jump_crouch_length = 0.068f;         // 蹲下腿长 (m)
static float g_jump_extend_length = 0.110f;         // 蹬伸腿长 (m)
static float g_jump_retract_length = 0.068f;        // 空中收腿腿长 (m)
static float g_jump_pos_threshold = 0.008f;         // 位置到达阈值 (m), 8mm
static uint32_t g_jump_timeout_ms = 2000;           // 单阶段安全超时 (ms)
static uint32_t g_jump_extend_hold_ms = 150;        // 蹬伸后最短保持时间 (ms), 确保离地
static uint32_t g_jump_air_hold_ms = 300;           // 空中收腿保持时间 (ms)
// g_jump_recover_hold_ms removed (RECOVER stage eliminated)
static float g_jump_max_speed_rpm = 400.0f;          // 跳跃时关节最大速度 (RPM)
static float g_jump_retract_pos_kp = 2.0f;           // 收腿时位置环 Kp
static float g_joint_normal_max_speed_rpm = 300.0f;  // 正常关节最大速度 (RPM)
static float g_joint_normal_pos_kp = 0.0f;           // 正常位置环 Kp (启动时读取)

// MIT 模式蹬伸参数
static bool g_jump_mit_active = false;               // MIT 蹬伸模式激活标志
static float g_jump_mit_kp = 3.0f;                   // MIT 位置刚度 (0~500)
static float g_jump_mit_kd = 0.1f;                   // MIT 速度阻尼 (0~5)
static float g_jump_mit_ff_torque = 1.5f;            // MIT 前馈力矩 (Nm)
static float g_jump_mit_vel_max = 41.9f;             // MIT 最大速度 (rad/s), ≈400RPM
static float g_jump_mit_target_rad[4] = {0};         // MIT 目标角度 [LH,LK,RH,RK] (rad)
static float g_jump_mit_ff_sign[4] = {1,1,1,1};      // MIT 前馈力矩方向 [LH,LK,RH,RK] (+1/-1)

// ============================================================================
// 起身 (Standup) 状态机
// ============================================================================
typedef enum {
    STANDUP_IDLE = 0,           // 空闲 (等待指令)
    STANDUP_RETRACT,            // 收腿 (双腿缩到最短, 保持向后)
    STANDUP_LEFT_ROLL,          // 左腿翻转 (左hip MIT + 左轮慢转)
    STANDUP_LEFT_EXTEND,        // 左腿伸长 (hip位置闭环, 伸长腿长)
    STANDUP_WAIT,               // 等待 (左腿伸长后稳定一段时间)
    STANDUP_RIGHT_ROLL,         // 右腿翻转 (右hip MIT + 右轮慢转)
    STANDUP_DONE,               // 完成, 进入小车模式
} standup_state_t;

static standup_state_t g_standup_state = STANDUP_IDLE;
static uint32_t g_standup_state_enter_ms = 0;
static bool g_standup_last_btn = false;             // 上一帧起身按钮状态 (上升沿检测)

// 起身参数 (可调)
static float g_standup_retract_length = 0.065f;     // 收腿目标腿长 (m)
static float g_standup_retract_angle = 0.0f;        // 收腿时身体夹角 (度), 运行时由 FK 读取
static float g_standup_mit_ff_torque = 0.08f;        // MIT 前馈力矩 (Nm)
static float g_standup_wheel_speed = -200.0f;        // 轮子速度 (RPM, 左轮为负)
static float g_standup_angle_threshold = 30.0f;     // 到达 car mode 角度的阈值 (度)
static uint32_t g_standup_retract_timeout_ms = 1000;// 收腿超时 (ms)
static uint32_t g_standup_roll_timeout_ms = 2000;   // 翻转超时 (ms)
static float g_standup_zero_threshold = 45.0f;      // 零位检查阈值 (度)
static float g_standup_extend_length = 0.105f;      // 翻转后伸腿目标腿长 (m)
static uint32_t g_standup_extend_timeout_ms = 500; // 伸腿超时 (ms)
static uint32_t g_standup_wait_ms = 1000;           // 左腿伸长后等待时间 (ms)

// 起身 MIT 控制状态
static bool g_standup_mit_active = false;            // MIT 模式激活标志
static bool g_standup_mit_left_hip = false;          // 左 hip MIT 激活
static bool g_standup_mit_right_hip = false;         // 右 hip MIT 激活
static float g_standup_ff_sign_left = 1.0f;          // 左 hip 前馈力矩方向
static float g_standup_ff_sign_right = 1.0f;         // 右 hip 前馈力矩方向
static float g_standup_mit_kp = 0.3f;                // MIT 位置刚度
static float g_standup_mit_kd = 0.03f;               // MIT 速度阻尼
static float g_standup_mit_target_left_hip = 0.0f;   // 左 hip MIT 目标位置 (rad)
static float g_standup_mit_target_right_hip = 0.0f;  // 右 hip MIT 目标位置 (rad)

// ============================================================================
// VMC (Virtual Model Control) 力控模式
// ============================================================================
static bool g_vmc_enabled = false;           // VMC 力控使能 (与位置控制互斥)
static vmc_params_t g_vmc_params;            // VMC 参数
static float g_vmc_target_vx = 0.0f;         // 目标水平速度 (m/s), 由平衡控制给出
static float g_vmc_target_y = 0.09f;         // 目标机身高度 (m), 默认 0.09m

// VMC 调试输出
static vmc_dual_output_t g_vmc_dual_output = {0};  // 双腿 VMC 输出 (包含协调控制)
static bool g_vmc_input_valid = false;             // VMC 输入是否有效
static bool g_vmc_stream_enable = false;           // VMC 数据流输出使能 (用于 UI 调试)
static bool g_joint_stream_enable = false;         // 关节电机数据流使能 (角度/速度/电流)
static bool g_mpow_stream_enable = false;          // 电机功率数据流使能 (电压/电流)
static uint8_t g_mpow_volt_poll_idx = 0;           // 电压轮询索引 (0-5, 轮流请求6个电机)
static uint8_t g_mpow_volt_poll_div = 0;            // 电压轮询分频计数器

// 支持力估计 (从电机电流通过逆雅可比反解, 始终计算)
static float g_support_force_left_FL = 0.0f;       // 左腿实际 F_L (N), 沿腿方向
static float g_support_force_right_FL = 0.0f;      // 右腿实际 F_L (N), 沿腿方向
static float g_support_force_left_Fa = 0.0f;       // 左腿实际 F_alpha (Nm)
static float g_support_force_right_Fa = 0.0f;      // 右腿实际 F_alpha (Nm)
static bool  g_sforce_stream_enable = false;        // 支持力数据流使能

// ============================================================================
// 速度/位移观测器 (卡尔曼滤波融合: 编码器 + IMU 加速度)
// ============================================================================
static bool g_observer_enabled = true;             // 观测器使能 (默认开启, 与参考代码一致)
static bool g_tpid_use_observer_speed = false;     // 三环PID速度源: true=观测器, false=滤波轮速
static bool g_obsv_stream_enable = false;          // 观测器数据流输出使能
static int  g_obsv_period_ms = OBSERVER_PERIOD_MS; // 观测器任务周期 (ms, 可 CLI 调参)

// 2x2 卡尔曼滤波器状态: x = [v, a]^T
static float g_kf_x[2] = {0};                     // 状态: [速度(m/s), 加速度(m/s²)]
static float g_kf_P[4] = {1, 0, 0, 1};            // 协方差 P (2x2, 行主序)
// KF 参数 (可在线调参)
static float g_kf_Q_v = 1.0f;                      // 过程噪声-速度 (对标参考代码 Q=I)
static float g_kf_Q_a = 1.0f;                      // 过程噪声-加速度
static float g_kf_R_v = 200.0f;                    // 观测噪声-编码器速度 (对标参考代码 R=200I)
static float g_kf_R_a = 200.0f;                    // 观测噪声-IMU加速度

// 观测器输出 (可选用于替代原始轮速)
static float g_obsv_v_encoder = 0.0f;              // 运动学补偿后的编码器速度 (m/s)
static float g_obsv_v_filter = 0.0f;               // 卡尔曼滤波后的速度 (m/s)
static float g_obsv_x_filter = 0.0f;               // 滤波速度积分位移 (m)
static float g_obsv_a_imu = 0.0f;                  // IMU 前进方向加速度 (m/s²), 已去重力
static float g_obsv_wheel_v_raw = 0.0f;            // 原始轮速 (无补偿) (m/s)

// 加速度计零偏校准 + 低通滤波 + 死区
static float g_accel_bias = 0.0f;                  // 启动时校准的加速度偏置 (m/s²)
static bool  g_accel_bias_calibrated = false;       // 偏置校准完成标志
static float g_accel_lpf = 0.0f;                   // 低通滤波后的加速度 (m/s²)
static float g_accel_lpf_tau = 0.009f;             // LPF 时间常数 (s), 对标参考代码
static float g_accel_deadzone = 0.01f;              // 死区阈值 (m/s²), 约 0.06g

// ============================================================================
// 关节电机速度滤波 (支持中值滤波 / 限幅滤波切换)
// ============================================================================
static bool g_joint_speed_filter_enable = true;    // 关节速度滤波使能 (默认开启)
static int  g_joint_speed_filter_mode = 1;         // 0=中值滤波(Median), 1=限幅滤波(SlewRate)
// Slew-Rate 参数
static float g_joint_speed_slew_rate = 3000.0f;    // 最大变化率 (°/s²), 默认3000
static slewrate_filter_t g_sr_joint_lh;             // 左髋速度限幅滤波器
static slewrate_filter_t g_sr_joint_lk;             // 左膝速度限幅滤波器
static slewrate_filter_t g_sr_joint_rh;             // 右髋速度限幅滤波器
static slewrate_filter_t g_sr_joint_rk;             // 右膝速度限幅滤波器
// Median 参数
static int g_joint_median_window = 3;               // 中值滤波窗口大小 (3/5/7/9)
static median_filter_t g_mf_joint_lh;               // 左髋速度中值滤波器
static median_filter_t g_mf_joint_lk;               // 左膝速度中值滤波器
static median_filter_t g_mf_joint_rh;               // 右髋速度中值滤波器
static median_filter_t g_mf_joint_rk;               // 右膝速度中值滤波器
// 滤波后的速度值 (用于波形输出和 VMC 输入)
static float g_joint_lh_spd_filtered = 0.0f;
static float g_joint_lk_spd_filtered = 0.0f;
static float g_joint_rh_spd_filtered = 0.0f;
static float g_joint_rk_spd_filtered = 0.0f;

// 树莓派通信开关 (调试时可禁用以减少串口占用)
static bool g_pi_comm_enabled = false;  // 默认禁用

// ============================================================================
// Commander 参数回调 - 将调参面板的参数同步到 LQR 控制器
// ============================================================================

/**
 * @brief Commander 参数更新回调
 * @note 当从串口接收到调参命令时调用，将参数同步到 LQR 控制器
 */
static void commander_param_callback(char controller_id, char param_char, float value)
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
static bool commander_query_callback(char controller_id, commander_pid_params_t *params)
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
static void update_pi_comm_state(void) {
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
static void output_plot_data(const lqr_input_t *input, const lqr_output_t *output) {
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
static void output_pid_debug(const lqr_input_t *input) {
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

/**
 * @brief 计算 VMC 输入状态和输出扭矩 (在平衡控制循环中调用)
 * @note 此函数负责:
 *       1. 收集传感器数据 (电机角度/速度、IMU、轮速)
 *       2. 设置目标值 (腿长由 Roll 控制、身体角度默认垂直)
 *       3. 调用 vmc_ctrl_compute() 完成所有计算
 *       4. 保存输出用于电机控制
 * 
 * 所有 FK、雅可比、VMC 计算都封装在 leg_kinematics.c 中
 */

/**
 * @brief 支持力估计 (任意模式下均可调用)
 * @note 仅依赖关节电机有效, 与 VMC/LQR 模式无关
 *       若 VMC 未启用则自行刷新 CAN 缓冲区
 *       [F_L; F_α] = (J^T)^{-1} × [τ_hip; τ_knee]
 */
static void compute_support_force(void) {
    bool left_valid  = (g_motor_left_hip  && g_motor_left_knee);
    bool right_valid = (g_motor_right_hip && g_motor_right_knee);

    if (!left_valid && !right_valid) return;

    // VMC 未启用时需要在此刷新 CAN 接收缓冲区
    if (!g_vmc_enabled || !g_leg_control_enabled) {
        can_motor_process_rx();
    }

    float l_hip_actual_Nm  = left_valid  ? vmc_current_to_torque(can_motor_read_current(g_motor_left_hip))  : 0;
    float l_knee_actual_Nm = left_valid  ? vmc_current_to_torque(can_motor_read_current(g_motor_left_knee)) : 0;
    float r_hip_actual_Nm  = right_valid ? vmc_current_to_torque(can_motor_read_current(g_motor_right_hip))  : 0;
    float r_knee_actual_Nm = right_valid ? vmc_current_to_torque(can_motor_read_current(g_motor_right_knee)) : 0;

    // 右腿扭矩方向修正 (与 vmc_ctrl_compute 中的取反对应)
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

static void vmc_compute_leg_state(const lqr_input_t *lqr_input) {
    if (!g_vmc_enabled || !g_leg_control_enabled) {
        g_vmc_input_valid = false;
        return;
    }
    
    // 刷新 CAN 接收缓冲区，确保读取到最新的电机回复数据
    // (上一轮发送命令后电机的回复可能还在 RX 队列中未处理)
    can_motor_process_rx();
    
    // ===== 1. 获取公共数据 =====
    // IMU 数据
    float pitch_deg = 0.0f, pitch_rate_deg = 0.0f;
    if (lqr_input != NULL) {
        pitch_deg = lqr_input->raw_pitch;
        pitch_rate_deg = lqr_input->pitch_rate;
    }
    
    // 轮子速度 → 机器人水平速度
    const float rpm_to_rad_s = 3.14159f / 30.0f;
    xSemaphoreTake(g_wheel_state_mutex, portMAX_DELAY);
    float avg_wheel_rpm = (g_wheel_state.left_speed + g_wheel_state.right_speed) / 2.0f;
    xSemaphoreGive(g_wheel_state_mutex);
    float robot_vx = -avg_wheel_rpm * rpm_to_rad_s * WHEEL_RADIUS_M;  // 向后为正
    
    // ===== 2. 使用 vmc_dual_compute 计算双腿 (包含协调控制) =====
    bool left_valid = (g_motor_left_hip && g_motor_left_knee);
    bool right_valid = (g_motor_right_hip && g_motor_right_knee);
    
    if (left_valid || right_valid) {
        // 数值微分模式下不需要读取电机速度 (减少 CAN 通信)
        bool need_velocity = (g_vmc_params.vmc_diff_method == VMC_DIFF_JACOBIAN);
        
        // 关节速度使用全局滤波后的值 (由 compute_balance_output 每帧更新)
        // 若非 Jacobian 模式则不需要速度
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
            
            // ===== Full LQR Tp 注入 (替代 F_alpha) =====
            // 在 Full LQR 模式下, Tp 完全取代 VMC 的 F_alpha
            // VMC 仍然负责 F_L 腿长控制, 但 F_alpha 方向完全由 Full LQR 的 Tp 控制
            // 做法: 先减去 VMC 已计算的 J^T × F_alpha, 再加上 J^T × Tp
            //
            // 统一约定: Tp > 0 = F_alpha > 0 = 增大 alpha = 向后摆
            // 左右腿 LQR 使用完全相同的公式, Tp 含义也完全相同.
            if (g_control_mode == CTRL_MODE_FULL_LQR && g_full_lqr_initialized) {
                // 获取当前关节雅可比矩阵
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
                
                // 获取 VMC 已计算的 F_alpha (需要减去)
                float F_alpha_left  = g_vmc_dual_output.left.debug.F_alpha;
                float F_alpha_right = g_vmc_dual_output.right.debug.F_alpha;
                
                // 左腿: 用 Tp 替代 F_alpha
                // Tp 与 F_alpha 符号约定相同 (正 = 向后摆), 直接替代
                float Tp_left = g_full_lqr_Tp_left;
                float delta_left = Tp_left - F_alpha_left;
                g_vmc_dual_output.left.hip_torque  += J_left[2] * delta_left;
                g_vmc_dual_output.left.knee_torque += J_left[3] * delta_left;
                
                // 右腿: 用 Tp 替代 F_alpha
                // 注意: vmc_ctrl_compute 已对右腿输出做了整体取反,
                //   所以这里 J^T 的注入也需要取反才能正确叠加.
                float Tp_right = g_full_lqr_Tp_right;
                float delta_right = Tp_right - F_alpha_right;
                g_vmc_dual_output.right.hip_torque  += -(J_right[2] * delta_right);
                g_vmc_dual_output.right.knee_torque += -(J_right[3] * delta_right);
            }
            
            // 支持力估计已移至 compute_support_force(), 在 compute_balance_output() 中无条件调用

                // VMC 数据流输出 (用于 UI 调试)
                // 格式: #VMC,L_len,L_ang,L_FL,L_Fa,L_hip,L_knee,R_len,R_ang,R_FL,R_Fa,R_hip,R_knee,diff,Fsync,L_aFL,L_aFa,R_aFL,R_aFa
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

// Forward declaration for leg sync
static esp_err_t leg_ctrl_get_state_cached(bool is_left, leg_state_t *state);

/**
 * @brief 发送腿部电机命令 (在 motor_comm 任务中调用)
 * @note 支持两种模式:
 *       - 位置控制模式 (!g_vmc_enabled): 发送位置命令
 *       - VMC 力控模式 (g_vmc_enabled): 发送已计算好的扭矩命令
 */
static void apply_leg_motor_commands(void) {
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
        // ===== STW 电机位置轮询 =====
        // MIT 模式的应答不含位置信息 (与 JUCI 的 0x2A 回复不同),
        // 必须额外发送 0xA3 读角度命令, 否则 motor->state.position 不更新,
        // 导致 VMC alpha_error 始终为 0, 输出力矩为 0.
        if (g_motor_left_hip)   can_motor_request_angle(g_motor_left_hip);
        if (g_motor_left_knee)  can_motor_request_angle(g_motor_left_knee);
        if (g_motor_right_hip)  can_motor_request_angle(g_motor_right_hip);
        if (g_motor_right_knee) can_motor_request_angle(g_motor_right_knee);
    } else if (g_jump_mit_active) {
        // ===== 跳跃 MIT 蹬伸模式: 直接发 MIT 控制帧 =====
        // 前馈力矩按每个电机的运动方向签名, 避免左右腿镜像导致反向
        if (g_motor_left_hip) {
            can_motor_stw_mit_control(g_motor_left_hip,
                g_jump_mit_target_rad[0], 0,
                g_jump_mit_kp, g_jump_mit_kd, g_jump_mit_ff_torque * g_jump_mit_ff_sign[0]);
        }
        if (g_motor_left_knee) {
            can_motor_stw_mit_control(g_motor_left_knee,
                g_jump_mit_target_rad[1], 0,
                g_jump_mit_kp, g_jump_mit_kd, g_jump_mit_ff_torque * g_jump_mit_ff_sign[1]);
        }
        if (g_motor_right_hip) {
            can_motor_stw_mit_control(g_motor_right_hip,
                g_jump_mit_target_rad[2], 0,
                g_jump_mit_kp, g_jump_mit_kd, g_jump_mit_ff_torque * g_jump_mit_ff_sign[2]);
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
// 内部函数声明
// ============================================================================

static void task_imu_read(void *arg);
static void task_balance_ctrl(void *arg);
static void task_motor_comm(void *arg);
static void task_remote_watchdog(void *arg);
static void task_unified_control(void *arg);      // 合并任务 (IMU + 控制 + 电机)
static void task_observer(void *arg);             // 速度观测器独立任务

static void velocity_observer_update(float dt, const shared_imu_data_t *imu,
                                      float left_vel_rad, float right_vel_rad);

static void update_remote_from_wifi(void);
static void compute_balance_output(float dt);
static void apply_motor_commands(void);
static void jump_state_machine_update(void);
static void standup_state_machine_update(void);
static bool standup_is_active(void);
static esp_err_t leg_ctrl_get_state_cached(bool is_left, leg_state_t *state);
esp_err_t leg_ctrl_set_target(bool is_left, float leg_length, float body_angle);

// ============================================================================
// 初始化
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
        
        // 腿电机开机校零：将当前位置设为 0°
        // 注意：开机前需确保腿部在标定位置！
        ESP_LOGI(TAG, "Setting leg motors origin (current position -> 0)...");
        can_motor_set_origin(g_motor_left_hip);
        vTaskDelay(pdMS_TO_TICKS(10));
        can_motor_set_origin(g_motor_left_knee);
        vTaskDelay(pdMS_TO_TICKS(10));
        can_motor_set_origin(g_motor_right_hip);
        vTaskDelay(pdMS_TO_TICKS(10));
        can_motor_set_origin(g_motor_right_knee);
        vTaskDelay(pdMS_TO_TICKS(50));  // 等待电机处理
        ESP_LOGI(TAG, "Leg motors origin set complete");
        
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
static void task_imu_read(void *arg) {
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
static void task_balance_ctrl(void *arg) {
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
static void task_motor_comm(void *arg) {
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
static void task_unified_control(void *arg) {
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
 * @brief 速度观测器独立任务 (Core 1)
 * 
 * 独立于控制循环运行, 可以有自己的频率 (默认 333Hz)
 * 读取: IMU 数据, 轮电机状态, VMC 输出
 * 写入: g_obsv_v_filter, g_obsv_x_filter 等
 * 控制任务只读取观测器输出, 二者不会互相阻塞
 */
static void task_observer(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    
    ESP_LOGI(TAG, "[task_observer] Started on Core %d, period %d ms (%.0f Hz)",
             xPortGetCoreID(), g_obsv_period_ms, 1000.0f / g_obsv_period_ms);
    
    // --- 启动时加速度计零偏校准 (采集 200 次取平均, ~0.6s) ---
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
        
        // 只有使能时才运算, 否则空转等待
        if (g_observer_enabled) {
            // 读取 IMU 数据 (线程安全)
            shared_imu_data_t imu;
            xSemaphoreTake(g_imu_mutex, portMAX_DELAY);
            memcpy(&imu, &g_imu_data, sizeof(imu));
            xSemaphoreGive(g_imu_mutex);
            
            // 读取轮电机速度 (线程安全)
            float left_vel_rad, right_vel_rad;
            xSemaphoreTake(g_wheel_state_mutex, portMAX_DELAY);
            left_vel_rad  = g_wheel_state.left_speed  * 0.10472f;   // rpm → rad/s
            right_vel_rad = g_wheel_state.right_speed * 0.10472f;
            xSemaphoreGive(g_wheel_state_mutex);
            
            // 执行观测器更新 (内部读取 g_vmc_dual_output 等全局变量)
            velocity_observer_update(dt, &imu, left_vel_rad, right_vel_rad);
        }
        
        vTaskDelayUntil(&last_wake, period);
    }
    
    ESP_LOGI(TAG, "[task_observer] Stopped");
    g_task_observer = NULL;
    vTaskDelete(NULL);
}

/**
 * @brief 遥控看门狗任务 (Core 0, 100ms 周期)
 */
static void task_remote_watchdog(void *arg) {
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

// ============================================================================
// 跳跃状态机
// ============================================================================

/**
 * @brief 跳跃状态机更新 (在控制循环中调用)
 * 
 * 状态转移:
 *   IDLE → (按钮上升沿) → CROUCH → (保持时间到) → EXTEND → (保持时间到) →
 *   AIR_RETRACT → (保持时间到) → RECOVER → (保持时间到) → IDLE
 * 
 * CROUCH:      蹲下蓄力, 腿长→68mm
 * EXTEND:      蹬伸起跳, 腿长→110mm
 * AIR_RETRACT: 空中收腿, 腿长→68mm, 轮速强制为0
 * RECOVER:     着地恢复, 恢复原始腿长, 恢复正常平衡
 */
static void jump_state_machine_update(void) {
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

/**
 * @brief 跳跃状态机是否要求轮速为零
 * @return true: 空中阶段, 轮速应强制为0
 */
static inline bool jump_wants_zero_wheel(void) {
    return (g_jump_state == JUMP_AIR_RETRACT);
}

/**
 * @brief 跳跃状态机是否正在执行 (非 IDLE)
 * @return true: 跳跃过程中
 */
static inline bool jump_is_active(void) {
    return (g_jump_state != JUMP_IDLE);
}

// ============================================================================
// 起身 (Standup) 状态机
// ============================================================================

static bool standup_is_active(void) {
    return g_standup_state != STANDUP_IDLE;
}

static void standup_state_machine_update(void) {
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
            // 收腿: 等双腿到达最短位置, 或超时
            bool reached = fk_valid &&
                (fabsf(left_state.workspace.leg_length - g_standup_retract_length) < 0.008f) &&
                (fabsf(right_state.workspace.leg_length - g_standup_retract_length) < 0.008f);
            bool timeout = (elapsed >= g_standup_retract_timeout_ms);
            if (reached || timeout) {
                // 进入左腿翻转
                g_standup_state = STANDUP_LEFT_ROLL;
                g_standup_state_enter_ms = now_ms;
                
                // 激活左 hip MIT
                g_standup_mit_active = true;
                g_standup_mit_left_hip = true;
                g_standup_mit_right_hip = false;
                
                // 计算前馈方向: 从当前 hip 角度朝 car mode hip 角度
                float current_hip = can_motor_read_position(g_motor_left_hip);
                leg_workspace_state_t target_ws = { .leg_length = g_standup_retract_length, .body_angle = CAR_MODE_BODY_ANGLE };
                leg_joint_state_t target_joint;
                if (leg_kin_inverse(&target_ws, true, NULL, &target_joint) == ESP_OK) {
                    g_standup_ff_sign_left = (target_joint.hip_angle >= current_hip) ? 1.0f : -1.0f;
                    g_standup_mit_target_left_hip = DEG2RAD(target_joint.hip_angle);
                }
                
                // 左轮进入速度模式, 慢转
                can_motor_set_mode(g_motor_left, MODE_SPEED);
                can_motor_set_speed(g_motor_left, g_standup_wheel_speed);
                
                ESP_LOGW(TAG, "STANDUP: LEFT_ROLL! hip=%.1f ff_sign=%.0f wheel=%.0f (%s)",
                         current_hip, g_standup_ff_sign_left, g_standup_wheel_speed,
                         reached ? "reached" : "timeout");
            }
            break;
        }
            
        case STANDUP_LEFT_ROLL: {
            // 等左腿 body_angle 到达 car mode 角度附近
            bool reached = fk_valid &&
                (fabsf(left_state.workspace.body_angle - CAR_MODE_BODY_ANGLE) < g_standup_angle_threshold);
            bool timeout = (elapsed >= g_standup_roll_timeout_ms);
            if (reached || timeout) {
                // 停左轮
                can_motor_set_speed(g_motor_left, 0);
                
                // 左 hip 退出 MIT, 设左腿到 car mode 角度 + 收腿腿长 (先保持短腿)
                g_standup_mit_left_hip = false;
                g_standup_mit_active = false;
                leg_ctrl_set_target(true, g_standup_retract_length, CAR_MODE_BODY_ANGLE);
                
                // 进入左腿伸长阶段
                g_standup_state = STANDUP_LEFT_EXTEND;
                g_standup_state_enter_ms = now_ms;
                
                ESP_LOGW(TAG, "STANDUP: LEFT_EXTEND! target_len=%.3f (%s)",
                         g_standup_extend_length, reached ? "reached" : "timeout");
            }
            break;
        }
            
        case STANDUP_LEFT_EXTEND: {
            // 伸长左腿到目标腿长
            leg_ctrl_set_target(true, g_standup_extend_length, CAR_MODE_BODY_ANGLE);
            
            bool reached = fk_valid &&
                (fabsf(left_state.workspace.leg_length - g_standup_extend_length) < 0.008f);
            bool timeout = (elapsed >= g_standup_extend_timeout_ms);
            if (reached || timeout) {
                // 进入等待阶段
                g_standup_state = STANDUP_WAIT;
                g_standup_state_enter_ms = now_ms;
                
                ESP_LOGW(TAG, "STANDUP: WAIT %lums before RIGHT_ROLL (%s)",
                         (unsigned long)g_standup_wait_ms, reached ? "reached" : "timeout");
            }
            break;
        }
            
        case STANDUP_WAIT: {
            // 等待一段时间后进入右腿翻转
            if (elapsed >= g_standup_wait_ms) {
                // 进入右腿翻转
                g_standup_state = STANDUP_RIGHT_ROLL;
                g_standup_state_enter_ms = now_ms;
                
                // 激活右 hip MIT
                g_standup_mit_active = true;
                g_standup_mit_right_hip = true;
                
                // 计算右 hip 前馈方向
                float current_hip = can_motor_read_position(g_motor_right_hip);
                leg_workspace_state_t target_ws = { .leg_length = g_standup_retract_length, .body_angle = CAR_MODE_BODY_ANGLE };
                leg_joint_state_t target_joint;
                if (leg_kin_inverse(&target_ws, false, NULL, &target_joint) == ESP_OK) {
                    g_standup_ff_sign_right = (target_joint.hip_angle >= current_hip) ? 1.0f : -1.0f;
                    g_standup_mit_target_right_hip = DEG2RAD(target_joint.hip_angle);
                }
                
                // 右轮进入速度模式, 慢转
                can_motor_set_mode(g_motor_right, MODE_SPEED);
                can_motor_set_speed(g_motor_right, g_standup_wheel_speed);
                
                ESP_LOGW(TAG, "STANDUP: RIGHT_ROLL! hip=%.1f ff_sign=%.0f wheel=%.0f",
                         current_hip, g_standup_ff_sign_right, g_standup_wheel_speed);
            }
            break;
        }
            
        case STANDUP_RIGHT_ROLL: {
            // 等右腿 body_angle 到达 car mode 角度附近
            bool reached = fk_valid &&
                (fabsf(right_state.workspace.body_angle - CAR_MODE_BODY_ANGLE) < g_standup_angle_threshold);
            bool timeout = (elapsed >= g_standup_roll_timeout_ms);
            if (reached || timeout) {
                // 停右轮
                can_motor_set_speed(g_motor_right, 0);
                
                // 退出 MIT
                g_standup_mit_active = false;
                g_standup_mit_right_hip = false;
                
                // 设右腿到 car mode 角度 + 收腿腿长 (先保持短腿)
                leg_ctrl_set_target(false, g_standup_retract_length, CAR_MODE_BODY_ANGLE);
                
                // 右腿不需要伸长，直接进入 DONE
                g_standup_state = STANDUP_DONE;
                g_standup_state_enter_ms = now_ms;
                
                ESP_LOGW(TAG, "STANDUP: DONE! right leg rolled (%s)",
                         reached ? "reached" : "timeout");
            }
            break;
        }
            

            
        case STANDUP_DONE: {
            // 短暂稳定后进入小车模式
            if (elapsed >= 100) {
                g_car_mode_prev_mode = g_control_mode;
                g_car_mode_prev_base_angle = g_leg_base_angle;
                g_car_mode_prev_base_length = g_leg_base_length;
                g_control_mode = CTRL_MODE_CAR;
                
                // 两腿都收缩到 retract 腿长
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

/**
 * @brief 从 WiFi 模块更新遥控数据
 */
static void update_remote_from_wifi(void) {
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
        ESP_LOGW(TAG, "WiFi: Auto-exit CAR mode (set by standup), restored to %s", restored_str);
        printf("CTRL_MODE:%s\n", restored_str);
    }
    
    if (wifi_data->car_mode && !last_car_mode && !wifi_data->estop) {
        // 进入小车模式: 保存当前状态, 设置腿部角度, 切电机速度模式
        g_car_mode_prev_mode = g_control_mode;
        g_car_mode_prev_base_angle = g_leg_base_angle;
        g_car_mode_prev_base_length = g_leg_base_length;
        g_control_mode = CTRL_MODE_CAR;
        
        if (g_state == BALANCE_TEST_RUNNING) {
            can_motor_set_mode(g_motor_left, MODE_SPEED);
            can_motor_set_mode(g_motor_right, MODE_SPEED);
        }
        
        if (g_leg_control_enabled) {
            leg_ctrl_set_target(true, CAR_MODE_LEG_LENGTH, CAR_MODE_BODY_ANGLE);
            leg_ctrl_set_target(false, CAR_MODE_LEG_LENGTH, CAR_MODE_BODY_ANGLE);
        }
        
        ESP_LOGI(TAG, "WiFi: CAR mode ON (body_angle=%.0f°, leg=%.0fmm)",
                 CAR_MODE_BODY_ANGLE, CAR_MODE_LEG_LENGTH * 1000.0f);
        printf("CTRL_MODE:CAR\n");
    } else if (!wifi_data->car_mode && last_car_mode) {
        // 退出小车模式: 恢复之前的模式和腿部姿态
        if (g_control_mode == CTRL_MODE_CAR) {
            if (g_state == BALANCE_TEST_RUNNING) {
                can_motor_set_speed(g_motor_left, 0);
                can_motor_set_speed(g_motor_right, 0);
            }
            
            if (g_leg_control_enabled) {
                leg_ctrl_set_target(true, g_car_mode_prev_base_length, g_car_mode_prev_base_angle);
                leg_ctrl_set_target(false, g_car_mode_prev_base_length, g_car_mode_prev_base_angle);
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
        }
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
    static float last_angle_zero = 7.4f;
    if (fabsf(wifi_data->angle_zero - last_angle_zero) > 0.01f) {
        balance_test_set_angle_zeropoint(wifi_data->angle_zero);
        last_angle_zero = wifi_data->angle_zero;
    }
    
    // ======== 处理跳跃按钮 (上升沿触发) ========
    {
        bool jump_btn = wifi_data->jump || (wifi_data->dir == DIR_JUMP);
        if (jump_btn && !g_jump_last_btn && g_jump_state == JUMP_IDLE) {
            // 上升沿 + 当前空闲 → 启动跳跃序列 (不要求平衡使能, 只要腿部使能即可)
            // 需双腿均着地 (F_L 检测), 悬空时禁止触发, 防止空中乱动
            bool both_on_ground = (!g_sforce_left_off && !g_sforce_right_off);
            if (g_leg_control_enabled && !wifi_data->estop && both_on_ground) {
                g_jump_saved_leg_length = g_leg_base_length;  // 保存当前腿长
                g_jump_saved_base_angle = g_leg_base_angle;   // 保存当前夹角 (防止跳跃中被覆盖)
                g_jump_state = JUMP_CROUCH;
                g_jump_state_enter_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
                // 跳跃前提升关节最大速度
                can_motor_stw_set_max_speed(g_motor_left_hip, g_jump_max_speed_rpm);
                can_motor_stw_set_max_speed(g_motor_left_knee, g_jump_max_speed_rpm);
                can_motor_stw_set_max_speed(g_motor_right_hip, g_jump_max_speed_rpm);
                can_motor_stw_set_max_speed(g_motor_right_knee, g_jump_max_speed_rpm);
                // 蹲下蓄力
                leg_ctrl_set_target(true, g_jump_crouch_length, g_leg_base_angle);
                leg_ctrl_set_target(false, g_jump_crouch_length, g_leg_base_angle);
                ESP_LOGW(TAG, "JUMP: START! saved_leg=%.0fmm, crouch=%.0fmm, spd=%.0f",
                         g_jump_saved_leg_length * 1000.0f, g_jump_crouch_length * 1000.0f,
                         g_jump_max_speed_rpm);
            } else if (!both_on_ground) {
                ESP_LOGW(TAG, "JUMP: IGNORED - not both on ground (L=%d R=%d)",
                         g_sforce_left_off, g_sforce_right_off);
            }
        }
        g_jump_last_btn = jump_btn;
    }
    
    // ======== 处理起身按钮 (上升沿触发) ========
    {
        bool standup_btn = wifi_data->standup;
        if (standup_btn && !g_standup_last_btn && g_standup_state == STANDUP_IDLE) {
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
    
    // ======== 处理腿部使能 ========
    static bool last_leg_enable = false;
    if (wifi_data->leg_enable != last_leg_enable && !wifi_data->estop) {
        balance_test_set_leg_control(wifi_data->leg_enable);
        ESP_LOGI(TAG, "WiFi: Leg control %s", wifi_data->leg_enable ? "ENABLED" : "DISABLED");
    }
    last_leg_enable = wifi_data->leg_enable;
    
    // ======== 处理腿部角度和长度 ========
    // 跳跃/起身过程中不允许遥控覆盖腿长 (由状态机接管)
    if (g_leg_control_enabled && !wifi_data->estop && !jump_is_active() && !standup_is_active()) {
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

// ============================================================================
// 速度/位移观测器 (2x2 卡尔曼滤波)
// ============================================================================
// 状态: x = [v, a]^T  (速度 m/s, 加速度 m/s²)
// 状态转移: F = [1, dt; 0, 1]  (匀加速模型)
// 观测: z = [v_encoder, a_imu]^T,  H = I
// 参考: /home/bubble/wheel-legged/docs/速度与位移观测器设计指南.md

/**
 * @brief 2x2 卡尔曼滤波器更新 (标量展开, 无矩阵库依赖)
 * @param dt 时间步长 (秒)
 * @param z_v 编码器速度观测 (m/s)
 * @param z_a IMU加速度观测 (m/s²)
 */
static void kf_observer_update(float dt, float z_v, float z_a) {
    // ===== Step 1: 状态预测 x' = F * x =====
    float v_pred = g_kf_x[0] + g_kf_x[1] * dt;
    float a_pred = g_kf_x[1];
    
    // ===== Step 2: 协方差预测 P' = F * P * F^T + Q =====
    // F = [1, dt; 0, 1],  F^T = [1, 0; dt, 1]
    // P = [p00, p01; p10, p11]
    float p00 = g_kf_P[0], p01 = g_kf_P[1], p10 = g_kf_P[2], p11 = g_kf_P[3];
    
    // FP = F * P
    float fp00 = p00 + dt * p10;
    float fp01 = p01 + dt * p11;
    float fp10 = p10;
    float fp11 = p11;
    
    // P' = FP * F^T + Q
    float pp00 = fp00 + fp01 * dt + g_kf_Q_v;
    float pp01 = fp01;
    float pp10 = fp10 + fp11 * dt;
    float pp11 = fp11 + g_kf_Q_a;
    
    // ===== Step 3: 卡尔曼增益 K = P' * H^T * (H * P' * H^T + R)^{-1} =====
    // H = I, 所以 S = P' + R
    float s00 = pp00 + g_kf_R_v;
    float s01 = pp01;
    float s10 = pp10;
    float s11 = pp11 + g_kf_R_a;
    
    // S^{-1} (2x2 矩阵求逆)
    float det = s00 * s11 - s01 * s10;
    if (fabsf(det) < 1e-10f) det = 1e-10f;  // 防止除零
    float inv_det = 1.0f / det;
    float si00 =  s11 * inv_det;
    float si01 = -s01 * inv_det;
    float si10 = -s10 * inv_det;
    float si11 =  s00 * inv_det;
    
    // K = P' * S^{-1}  (因为 H = I)
    float k00 = pp00 * si00 + pp01 * si10;
    float k01 = pp00 * si01 + pp01 * si11;
    float k10 = pp10 * si00 + pp11 * si10;
    float k11 = pp10 * si01 + pp11 * si11;
    
    // ===== Step 4: 状态更新 x = x' + K * (z - H * x') =====
    float innov_v = z_v - v_pred;
    float innov_a = z_a - a_pred;
    
    g_kf_x[0] = v_pred + k00 * innov_v + k01 * innov_a;
    g_kf_x[1] = a_pred + k10 * innov_v + k11 * innov_a;
    
    // ===== Step 5: 协方差更新 P = (I - K * H) * P' =====
    // (I - K) * P'  (因为 H = I)
    float ikh00 = 1.0f - k00, ikh01 = -k01;
    float ikh10 = -k10,       ikh11 = 1.0f - k11;
    
    g_kf_P[0] = ikh00 * pp00 + ikh01 * pp10;
    g_kf_P[1] = ikh00 * pp01 + ikh01 * pp11;
    g_kf_P[2] = ikh10 * pp00 + ikh11 * pp10;
    g_kf_P[3] = ikh10 * pp01 + ikh11 * pp11;
    
    // 防止协方差过度收敛 (最小方差约束)
    if (g_kf_P[0] < 0.001f) g_kf_P[0] = 0.001f;
    if (g_kf_P[3] < 0.001f) g_kf_P[3] = 0.001f;
}

/**
 * @brief 重置观测器状态
 */
static void kf_observer_reset(void) {
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

/**
 * @brief 速度观测器主函数 (在 compute_balance_output 中每帧调用)
 * 
 * 运动学补偿:
 *   ω_ground = ω_motor - pitch_rate + d_alpha  (轮子对地绝对角速度)
 *   v_body = ω_ground * R + L0 * d_theta * cos(theta) + d_L0 * sin(theta)
 * 
 * 注意正方向: 本项目 left_vel_rad/right_vel_rad 均为 rpm→rad/s,
 *   向前运动时左轮为负/右轮为负, 取 -0.5*(L+R) 为正方向
 */
static void velocity_observer_update(float dt, const shared_imu_data_t *imu,
                                      float left_vel_rad, float right_vel_rad) {
    if (!g_observer_enabled) return;
    
    // --- 1) 原始轮速 (无补偿, 保留用于对比) ---
    g_obsv_wheel_v_raw = (-0.5f) * (left_vel_rad + right_vel_rad) * WHEEL_RADIUS_M;
    
    // --- 2) 运动学补偿: 减去机体 pitch 旋转 ---
    // pitch_rate 单位: °/s → 需要转 rad/s
    float pitch_rate_rad = imu->pitch_rate * 0.0174533f;  // deg/s → rad/s
    
    // 轮子对地绝对角速度 (减去机体旋转)
    // 两轮编码器方向统一 (都是负=前进), 但物理安装镜像对称
    // pitch 前倾时: 左轮编码器假读正值, 右轮编码器假读负值 (从各自轴端看旋转方向相反)
    // 但由于最终取 (-0.5)*(L+R), 左+右- 的对称补偿会被抵消
    // 所以两轮都用减号, 使 pitch 补偿不被取平均消掉:
    //   (-0.5)*((L-p)+(R-p)) = (-0.5)*(L+R) + p  → 补偿保留
    float left_ground_rad  = left_vel_rad  - pitch_rate_rad;   // 左轮: - pitch_rate
    float right_ground_rad = right_vel_rad - pitch_rate_rad;   // 右轮: - pitch_rate
    
    // --- 3) 腿部动力学补偿 (仅在 VMC 使能时) ---
    float left_vbody = left_ground_rad * WHEEL_RADIUS_M;
    float right_vbody = right_ground_rad * WHEEL_RADIUS_M;
    
    if (g_leg_control_enabled && g_vmc_input_valid) {
        // 左腿: d_alpha, L0, d_L0, body_angle (需转换为 theta = 与竖直方向的夹角)
        float l_d_alpha = g_vmc_dual_output.left.current_body_angle_rate * 0.0174533f;  // deg/s → rad/s
        float l_L0 = g_vmc_dual_output.left.current_leg_length;                          // m
        float l_d_L0 = g_vmc_dual_output.left.current_leg_length_rate;                   // m/s
        // body_angle: 腿与机身夹角(度), -90°=垂直向下
        // theta (与竖直方向夹角) = pitch + body_angle + 90
        float l_theta_deg = imu->pitch + g_vmc_dual_output.left.current_body_angle + 90.0f;
        float l_theta = l_theta_deg * 0.0174533f;
        // d_theta ≈ pitch_rate + d_alpha (简化, 忽略高阶项)
        float l_d_theta = pitch_rate_rad + l_d_alpha;
        
        // 补偿: 加入 d_alpha 和腿部运动学 (同号, 与 pitch_rate 同理)
        left_ground_rad -= l_d_alpha;  // 轮轴定子随腿摆动
        left_vbody = left_ground_rad * WHEEL_RADIUS_M
                   + l_L0 * l_d_theta * cosf(l_theta)
                   + l_d_L0 * sinf(l_theta);
        
        // 右腿: 类似处理
        float r_d_alpha = g_vmc_dual_output.right.current_body_angle_rate * 0.0174533f;
        float r_L0 = g_vmc_dual_output.right.current_leg_length;
        float r_d_L0 = g_vmc_dual_output.right.current_leg_length_rate;
        float r_theta_deg = imu->pitch + g_vmc_dual_output.right.current_body_angle + 90.0f;
        float r_theta = r_theta_deg * 0.0174533f;
        float r_d_theta = pitch_rate_rad + r_d_alpha;
        
        right_ground_rad -= r_d_alpha;  // 与左腿同号, 避免取平均时抵消
        right_vbody = right_ground_rad * WHEEL_RADIUS_M
                    + r_L0 * r_d_theta * cosf(r_theta)
                    + r_d_L0 * sinf(r_theta);
    }
    
    // --- 4) 双轮取平均 ---
    // 本项目: 两轮均为负=前进, 取 -0.5*(L+R) 为前进正方向
    g_obsv_v_encoder = (-0.5f) * (left_vbody + right_vbody);
    
    // --- 5) IMU 加速度 (前进方向, 去除重力) ---
    // WIT IMU 加速度单位: g,  需要乘以 9.81 得到 m/s²
    // accel_x 对应前进方向 (已确认), pitch 正=前倾
    // 加速度计测量比力 (specific force), 机体→世界旋转自动抵消重力:
    //   a_forward = (accel_x * cos(pitch) - accel_z * sin(pitch)) * 9.81
    float pitch_rad = imu->pitch * 0.0174533f;
    float a_raw = ((imu->accel_x + sinf(pitch_rad)) * cosf(pitch_rad) - (imu->accel_z - cosf(pitch_rad)) * sinf(pitch_rad)) * 9.81f;
    
    // 5a) 零偏补偿
    if (g_accel_bias_calibrated) {
        a_raw -= g_accel_bias;
    }
    
    // 5b) 低通滤波 (一阶 IIR, 对标参考代码 AccelLPF ≈ 0.009s)
    float alpha = dt / (g_accel_lpf_tau + dt);
    g_accel_lpf = g_accel_lpf * (1.0f - alpha) + a_raw * alpha;
    
    // 5c) 死区滤波 (消除静止时的微小偏移)
    if (fabsf(g_accel_lpf) < g_accel_deadzone) {
        g_obsv_a_imu = 0.0f;
    } else {
        g_obsv_a_imu = g_accel_lpf;
    }
    
    // --- 6) 卡尔曼滤波融合 ---
    kf_observer_update(dt, g_obsv_v_encoder, g_obsv_a_imu);
    
    g_obsv_v_filter = g_kf_x[0];
    
    // --- 7) 位移积分 ---
    g_obsv_x_filter += g_obsv_v_filter * dt;
}

static void compute_balance_output(float dt) {
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
    }
    if ((g_move_stop_flag == 1) && (fabsf(g_lqr_speed) < 0.5f)) {
        g_distance_zeropoint = g_lqr_distance;
        lqr_set_distance_zeropoint(&g_lqr_ctrl, g_distance_zeropoint);
        triple_pid_set_distance_zeropoint(&g_triple_pid_ctrl, g_distance_zeropoint);
        g_move_stop_flag = 0;
    }
    
    // 被快速推动时的原地停车处理
    if (fabsf(g_lqr_speed) > 0.6f) {
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
        
        float speed_cmd = remote.joy_y / 100.0f * CAR_MODE_MAX_SPEED;  // -MAX ~ +MAX rpm
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
        
        // 根据腿长线性插值角度环Kp: L0=0.06→Kp=1.3, L0=0.11→Kp=0.9
        {
            float avg_L0 = (g_leg_left_target_length + g_leg_right_target_length) * 0.5f;
            if (avg_L0 < 0.06f) avg_L0 = 0.06f;
            if (avg_L0 > 0.11f) avg_L0 = 0.11f;
            float interp_angle_kp = 1.25f + (1.15f - 1.25f) * (avg_L0 - 0.06f) / (0.11f - 0.06f);
            triple_pid_set_angle_gains(&g_triple_pid_ctrl, interp_angle_kp,
                                       g_triple_pid_ctrl.params.angle_ki,
                                       g_triple_pid_ctrl.params.angle_kd);
        }
        
        // 根据腿长线性插值角速度环Kp: L0=0.06→Kp=0.11, L0=0.11→Kp=0.09
        {
            float avg_L0 = (g_leg_left_target_length + g_leg_right_target_length) * 0.5f;
            if (avg_L0 < 0.06f) avg_L0 = 0.06f;
            if (avg_L0 > 0.11f) avg_L0 = 0.11f;
            float interp_gyro_kp = 0.105f + (0.095f - 0.105f) * (avg_L0 - 0.06f) / (0.11f - 0.06f);
            triple_pid_set_gyro_gains(&g_triple_pid_ctrl, interp_gyro_kp,
                                      g_triple_pid_ctrl.params.gyro_ki,
                                      g_triple_pid_ctrl.params.gyro_kd);
        }
        
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
        // Full LQR 模式下 Roll/X-Offset 与 LQR 模式相同
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
                
                // X-Offset
                if (g_xoffset_enabled) {
                    g_xoffset_debug_speed = g_lqr_speed;
                    g_xoffset_value = pid_compute(&g_xoffset_pid, g_lqr_speed, 0.0f, dt);
                } else {
                    g_xoffset_value = 0.0f;
                    g_xoffset_debug_speed = 0.0f;
                }
                
                if (fabsf(g_xoffset_value) > 0.0001f) {
                    float lx, ly;
                    leg_kin_polar_to_cartesian(new_left_length, left_angle, &lx, &ly);
                    lx += g_xoffset_value;
                    leg_kin_clamp_cartesian_body(&lx, &ly, NULL);
                    leg_kin_cartesian_to_polar(lx, ly, &new_left_length, &left_angle);
                    
                    float rx, ry;
                    leg_kin_polar_to_cartesian(new_right_length, right_angle, &rx, &ry);
                    rx += g_xoffset_value;
                    leg_kin_clamp_cartesian_body(&rx, &ry, NULL);
                    leg_kin_cartesian_to_polar(rx, ry, &new_right_length, &right_angle);
                }
                
                if (new_left_length < g_leg_length_min) new_left_length = g_leg_length_min;
                if (new_left_length > g_leg_length_max) new_left_length = g_leg_length_max;
                if (new_right_length < g_leg_length_min) new_right_length = g_leg_length_min;
                if (new_right_length > g_leg_length_max) new_right_length = g_leg_length_max;
                
                leg_kin_clamp_workspace(&new_left_length, &left_angle, NULL);
                leg_kin_clamp_workspace(&new_right_length, &right_angle, NULL);
                
                leg_workspace_state_t left_ws = { .leg_length = new_left_length, .body_angle = left_angle };
                leg_workspace_state_t right_ws = { .leg_length = new_right_length, .body_angle = right_angle };
                leg_joint_state_t left_joint, right_joint;
                
                if (leg_kin_inverse(&left_ws, true, NULL, &left_joint) == ESP_OK) {
                    g_leg_left_target_length = new_left_length;
                    g_leg_left_target_angle = left_angle;
                    g_leg_left_hip_angle = left_joint.hip_angle;
                    g_leg_left_knee_angle = left_joint.knee_angle;
                }
                if (leg_kin_inverse(&right_ws, false, NULL, &right_joint) == ESP_OK) {
                    g_leg_right_target_length = new_right_length;
                    g_leg_right_target_angle = right_angle;
                    g_leg_right_hip_angle = right_joint.hip_angle;
                    g_leg_right_knee_angle = right_joint.knee_angle;
                }
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
        // 仅在腿控制使能且 Roll 控制使能时执行
        // Roll 控制原理: 在基础腿长上对称调节左右腿长度
        //   - roll > 0 (右倾) -> 左腿伸长, 右腿缩短
        //   - roll < 0 (左倾) -> 左腿缩短, 右腿伸长
        if (g_leg_control_enabled && roll_active) {
            lqr_roll_output_t roll_output;
            esp_err_t roll_ret = lqr_roll_loop(&g_lqr_ctrl, &input, &roll_output);
            
            if (roll_ret == ESP_OK) {
                // 保存调试变量
                g_roll_output = roll_output.roll_control;
                g_roll_left_delta = roll_output.left_leg_delta;
                g_roll_right_delta = roll_output.right_leg_delta;
                g_roll_filtered = roll_output.filtered_roll;
                
                // 基于基础腿长对称调节 (不修改基础值)
                float new_left_length = g_leg_base_length + roll_output.left_leg_delta;
                float new_right_length = g_leg_base_length + roll_output.right_leg_delta;
                float left_angle = g_leg_base_angle;
                float right_angle = g_leg_base_angle;
                
                // ---- X-Offset: 在笛卡尔空间偏移腿脚 x 位置 ----
                if (fabsf(g_xoffset_value) > 0.0001f) {
                    // 左腿: (L, α) → (x, y) → x += offset → (L', α')
                    float lx, ly;
                    leg_kin_polar_to_cartesian(new_left_length, left_angle, &lx, &ly);
                    lx += g_xoffset_value;
                    leg_kin_clamp_cartesian_body(&lx, &ly, NULL);
                    leg_kin_cartesian_to_polar(lx, ly, &new_left_length, &left_angle);
                    
                    // 右腿: 同样偏移
                    float rx, ry;
                    leg_kin_polar_to_cartesian(new_right_length, right_angle, &rx, &ry);
                    rx += g_xoffset_value;
                    leg_kin_clamp_cartesian_body(&rx, &ry, NULL);
                    leg_kin_cartesian_to_polar(rx, ry, &new_right_length, &right_angle);
                }
                
                // 使用动态可调的腿长范围做限幅
                if (new_left_length < g_leg_length_min) new_left_length = g_leg_length_min;
                if (new_left_length > g_leg_length_max) new_left_length = g_leg_length_max;
                if (new_right_length < g_leg_length_min) new_right_length = g_leg_length_min;
                if (new_right_length > g_leg_length_max) new_right_length = g_leg_length_max;
                
                // 使用 leg_kinematics 的函数进行工作空间限幅 (几何限制)
                leg_kin_clamp_workspace(&new_left_length, &left_angle, NULL);
                leg_kin_clamp_workspace(&new_right_length, &right_angle, NULL);
                
                // 更新实际目标 (不调用 leg_ctrl_set_target 以避免修改基础值)
                // 直接计算 IK 并更新电机角度目标
                leg_workspace_state_t left_ws = { .leg_length = new_left_length, .body_angle = left_angle };
                leg_workspace_state_t right_ws = { .leg_length = new_right_length, .body_angle = right_angle };
                leg_joint_state_t left_joint, right_joint;
                
                if (leg_kin_inverse(&left_ws, true, NULL, &left_joint) == ESP_OK) {
                    g_leg_left_target_length = new_left_length;
                    g_leg_left_target_angle = left_angle;
                    g_leg_left_hip_angle = left_joint.hip_angle;
                    g_leg_left_knee_angle = left_joint.knee_angle;
                }
                
                if (leg_kin_inverse(&right_ws, false, NULL, &right_joint) == ESP_OK) {
                    g_leg_right_target_length = new_right_length;
                    g_leg_right_target_angle = right_angle;
                    g_leg_right_hip_angle = right_joint.hip_angle;
                    g_leg_right_knee_angle = right_joint.knee_angle;
                }
            }
        } else if (!jump_is_active() && !standup_is_active()) {
            // Roll 控制未启用时，使用基础腿长 (左右对称)
            // 跳跃/起身期间跳过: 腿目标由状态机接管
            float base_length = g_leg_base_length;
            float base_angle = g_leg_base_angle;
            
            // X-Offset: 即使 Roll 未启用，也可以应用 x_offset
            if (g_leg_control_enabled && fabsf(g_xoffset_value) > 0.0001f) {
                float lx, ly;
                leg_kin_polar_to_cartesian(base_length, base_angle, &lx, &ly);
                lx += g_xoffset_value;
                leg_kin_clamp_cartesian_body(&lx, &ly, NULL);
                
                float new_length, new_angle;
                leg_kin_cartesian_to_polar(lx, ly, &new_length, &new_angle);
                
                // IK 计算偏移后的关节角度
                leg_workspace_state_t left_ws = { .leg_length = new_length, .body_angle = new_angle };
                leg_workspace_state_t right_ws = { .leg_length = new_length, .body_angle = new_angle };
                leg_joint_state_t left_joint, right_joint;
                
                if (leg_kin_inverse(&left_ws, true, NULL, &left_joint) == ESP_OK) {
                    g_leg_left_target_length = new_length;
                    g_leg_left_target_angle = new_angle;
                    g_leg_left_hip_angle = left_joint.hip_angle;
                    g_leg_left_knee_angle = left_joint.knee_angle;
                }
                if (leg_kin_inverse(&right_ws, false, NULL, &right_joint) == ESP_OK) {
                    g_leg_right_target_length = new_length;
                    g_leg_right_target_angle = new_angle;
                    g_leg_right_hip_angle = right_joint.hip_angle;
                    g_leg_right_knee_angle = right_joint.knee_angle;
                }
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
            
            if (new_left_length < g_leg_length_min) new_left_length = g_leg_length_min;
            if (new_left_length > g_leg_length_max) new_left_length = g_leg_length_max;
            if (new_right_length < g_leg_length_min) new_right_length = g_leg_length_min;
            if (new_right_length > g_leg_length_max) new_right_length = g_leg_length_max;
            
            // ---- X-Offset: 笛卡尔空间偏移 (非 LQR 模式) ----
            if (fabsf(g_xoffset_value) > 0.0001f) {
                float lx, ly;
                leg_kin_polar_to_cartesian(new_left_length, left_angle, &lx, &ly);
                lx += g_xoffset_value;
                leg_kin_clamp_cartesian_body(&lx, &ly, NULL);
                leg_kin_cartesian_to_polar(lx, ly, &new_left_length, &left_angle);
                
                float rx, ry;
                leg_kin_polar_to_cartesian(new_right_length, right_angle, &rx, &ry);
                rx += g_xoffset_value;
                leg_kin_clamp_cartesian_body(&rx, &ry, NULL);
                leg_kin_cartesian_to_polar(rx, ry, &new_right_length, &right_angle);
            }
            
            // Clamp + IK (非 LQR 模式也需要完整 IK)
            leg_kin_clamp_workspace(&new_left_length, &left_angle, NULL);
            leg_kin_clamp_workspace(&new_right_length, &right_angle, NULL);
            
            leg_workspace_state_t left_ws = { .leg_length = new_left_length, .body_angle = left_angle };
            leg_workspace_state_t right_ws = { .leg_length = new_right_length, .body_angle = right_angle };
            leg_joint_state_t left_joint, right_joint;
            
            if (leg_kin_inverse(&left_ws, true, NULL, &left_joint) == ESP_OK) {
                g_leg_left_target_length = new_left_length;
                g_leg_left_target_angle = left_angle;
                g_leg_left_hip_angle = left_joint.hip_angle;
                g_leg_left_knee_angle = left_joint.knee_angle;
            }
            if (leg_kin_inverse(&right_ws, false, NULL, &right_joint) == ESP_OK) {
                g_leg_right_target_length = new_right_length;
                g_leg_right_target_angle = right_angle;
                g_leg_right_hip_angle = right_joint.hip_angle;
                g_leg_right_knee_angle = right_joint.knee_angle;
            }
        }
    } else if (is_non_lqr && g_leg_control_enabled && !roll_active) {
        // 非 LQR 模式, Roll 未启用, 但 x_offset 可能仍然有效
        if (fabsf(g_xoffset_value) > 0.0001f) {
            float base_length = g_leg_base_length;
            float base_angle = g_leg_base_angle;
            float lx, ly;
            leg_kin_polar_to_cartesian(base_length, base_angle, &lx, &ly);
            lx += g_xoffset_value;
            leg_kin_clamp_cartesian_body(&lx, &ly, NULL);
            
            float new_length, new_angle;
            leg_kin_cartesian_to_polar(lx, ly, &new_length, &new_angle);
            
            leg_workspace_state_t left_ws = { .leg_length = new_length, .body_angle = new_angle };
            leg_workspace_state_t right_ws = { .leg_length = new_length, .body_angle = new_angle };
            leg_joint_state_t left_joint, right_joint;
            
            if (leg_kin_inverse(&left_ws, true, NULL, &left_joint) == ESP_OK) {
                g_leg_left_target_length = new_length;
                g_leg_left_target_angle = new_angle;
                g_leg_left_hip_angle = left_joint.hip_angle;
                g_leg_left_knee_angle = left_joint.knee_angle;
            }
            if (leg_kin_inverse(&right_ws, false, NULL, &right_joint) == ESP_OK) {
                g_leg_right_target_length = new_length;
                g_leg_right_target_angle = new_angle;
                g_leg_right_hip_angle = right_joint.hip_angle;
                g_leg_right_knee_angle = right_joint.knee_angle;
            }
        }
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
static void apply_motor_commands(void) {
    shared_wheel_cmd_t cmd;
    // 每条腿的轮电机是否已进入 idle (F_L 离地检测触发)
    static bool s_left_idled  = false;
    static bool s_right_idled = false;

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
            // 左轮: F_L 离地 → idle; 着地 → 恢复闭环
            APPLY_WHEEL_CMD(g_motor_left,  left_speed_rpm,  cmd.left_torque,
                            g_sforce_left_off,  s_left_idled);
            // 右轮: F_L 离地 → idle; 着地 → 恢复闭环
            APPLY_WHEEL_CMD(g_motor_right, right_speed_rpm, cmd.right_torque,
                            g_sforce_right_off, s_right_idled);
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

// ============================================================================
// 命令处理
// ============================================================================

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
                
                // 恢复腿部姿态
                if (g_leg_control_enabled) {
                    leg_ctrl_set_target(true, g_car_mode_prev_base_length, g_car_mode_prev_base_angle);
                    leg_ctrl_set_target(false, g_car_mode_prev_base_length, g_car_mode_prev_base_angle);
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

// ============================================================================
// 腿部控制上层接口 (调用 leg_kinematics 模块)
// ============================================================================

/**
 * @brief 初始化腿部控制
 */
void leg_ctrl_init(void) {
    // 设置初始目标为站立姿态 (垂直向下)
    g_leg_left_target_length = 0.09f;
    g_leg_left_target_angle = -90.0f;  // 垂直向下
    g_leg_right_target_length = 0.09f;
    g_leg_right_target_angle = -90.0f; // 垂直向下
    
    // 用逆运动学计算初始电机角度
    leg_workspace_state_t workspace;
    leg_joint_state_t joint;
    
    // 左腿
    workspace.leg_length = g_leg_left_target_length;
    workspace.body_angle = g_leg_left_target_angle;
    if (leg_kin_inverse(&workspace, true, NULL, &joint) == ESP_OK) {
        g_leg_left_hip_angle = joint.hip_angle;
        g_leg_left_knee_angle = joint.knee_angle;
        ESP_LOGI(TAG, "Left leg init: L=%.3f, A=%.1f -> Hip=%.1f, Knee=%.1f",
                 workspace.leg_length, workspace.body_angle,
                 g_leg_left_hip_angle, g_leg_left_knee_angle);
    }
    
    // 右腿
    workspace.leg_length = g_leg_right_target_length;
    workspace.body_angle = g_leg_right_target_angle;
    if (leg_kin_inverse(&workspace, false, NULL, &joint) == ESP_OK) {
        g_leg_right_hip_angle = joint.hip_angle;
        g_leg_right_knee_angle = joint.knee_angle;
        ESP_LOGI(TAG, "Right leg init: L=%.3f, A=%.1f -> Hip=%.1f, Knee=%.1f",
                 workspace.leg_length, workspace.body_angle,
                 g_leg_right_hip_angle, g_leg_right_knee_angle);
    }
    
    // 打印运动学参数
    leg_kin_print_params(NULL, true);
    leg_kin_print_params(NULL, false);
    
    ESP_LOGI(TAG, "Leg control initialized");
}

/**
 * @brief 主动请求腿部电机位置更新
 * @param is_left 是否为左腿
 * @return ESP_OK 成功
 */
static esp_err_t leg_ctrl_request_position(bool is_left) {
    can_motor_handle_t hip, knee;
    
    if (is_left) {
        hip = g_motor_left_hip;
        knee = g_motor_left_knee;
    } else {
        hip = g_motor_right_hip;
        knee = g_motor_right_knee;
    }
    
    if (hip == NULL || knee == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    // 请求读取位置寄存器
    extern esp_err_t can_motor_request_status(can_motor_handle_t motor);
    can_motor_request_status(hip);
    can_motor_request_status(knee);
    
    // 等待 CAN 回复并处理
    vTaskDelay(pdMS_TO_TICKS(20));  // 给 CAN 接收任务时间处理回复
    
    return ESP_OK;
}

/**
 * @brief 读取当前腿部状态 (从编码器)
 * @note 会自动请求电机位置更新，确保读取到最新数据
 * @warning 此函数包含 CAN 请求和阻塞延时 (~48ms/次)，不要在控制循环中使用！
 *          控制循环中请使用 leg_ctrl_get_state_cached()
 */
esp_err_t leg_ctrl_get_state(bool is_left, leg_state_t *state) {
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    // 先请求位置更新
    leg_ctrl_request_position(is_left);
    
    leg_joint_state_t joint;
    
    if (is_left) {
        if (g_motor_left_hip == NULL || g_motor_left_knee == NULL) {
            state->valid = false;
            return ESP_ERR_INVALID_STATE;
        }
        joint.hip_angle = can_motor_read_position(g_motor_left_hip);
        joint.knee_angle = can_motor_read_position(g_motor_left_knee);
    } else {
        if (g_motor_right_hip == NULL || g_motor_right_knee == NULL) {
            state->valid = false;
            return ESP_ERR_INVALID_STATE;
        }
        joint.hip_angle = can_motor_read_position(g_motor_right_hip);
        joint.knee_angle = can_motor_read_position(g_motor_right_knee);
    }
    
    state->joint = joint;
    state->is_left = is_left;
    
    // 调用运动学正解
    esp_err_t ret = leg_kin_forward(&joint, is_left, NULL, &state->workspace);
    state->valid = (ret == ESP_OK);
    
    return ret;
}

/**
 * @brief 读取当前腿部状态 (从缓存，无阻塞)
 * @note 直接从电机驱动缓存中读取位置，不发送 CAN 请求。
 *       适合在 200Hz 控制循环中使用，延迟 < 1us。
 *       数据由 can_motor_process_rx() 在电机通信中自动更新。
 */
static esp_err_t leg_ctrl_get_state_cached(bool is_left, leg_state_t *state) {
    if (state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    leg_joint_state_t joint;
    
    if (is_left) {
        if (g_motor_left_hip == NULL || g_motor_left_knee == NULL) {
            state->valid = false;
            return ESP_ERR_INVALID_STATE;
        }
        joint.hip_angle = can_motor_read_position(g_motor_left_hip);
        joint.knee_angle = can_motor_read_position(g_motor_left_knee);
    } else {
        if (g_motor_right_hip == NULL || g_motor_right_knee == NULL) {
            state->valid = false;
            return ESP_ERR_INVALID_STATE;
        }
        joint.hip_angle = can_motor_read_position(g_motor_right_hip);
        joint.knee_angle = can_motor_read_position(g_motor_right_knee);
    }
    
    state->joint = joint;
    state->is_left = is_left;
    
    // 调用运动学正解 (纯数学计算，无阻塞)
    esp_err_t ret = leg_kin_forward(&joint, is_left, NULL, &state->workspace);
    state->valid = (ret == ESP_OK);
    
    return ret;
}

/**
 * @brief 设置目标腿部状态 (腿长 + 身体夹角)
 * @note 此函数会同时更新基础腿长，用于用户手动设置高度
 *       Roll 控制内部不应调用此函数，以避免修改基础值
 */
esp_err_t leg_ctrl_set_target(bool is_left, float leg_length, float body_angle) {
    // 使用动态可调的腿长范围做限幅
    if (leg_length < g_leg_length_min) leg_length = g_leg_length_min;
    if (leg_length > g_leg_length_max) leg_length = g_leg_length_max;
    
    // 运动学库的 clamp (包含几何限制)
    leg_kin_clamp_workspace(&leg_length, &body_angle, NULL);
    
    // 检查可达性
    if (!leg_kin_is_reachable(leg_length, body_angle, NULL)) {
        ESP_LOGW(TAG, "Target unreachable: L=%.3f, A=%.1f", leg_length, body_angle);
        return ESP_ERR_INVALID_ARG;
    }
    
    // 调用运动学逆解
    leg_workspace_state_t workspace = { .leg_length = leg_length, .body_angle = body_angle };
    leg_joint_state_t joint;
    
    esp_err_t ret = leg_kin_inverse(&workspace, is_left, NULL, &joint);
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 更新基础腿长 (用户设定的高度)
    // 当用户手动设置时，左右腿应该设置相同的基础值
    g_leg_base_length = leg_length;
    g_leg_base_angle = body_angle;
    
    // 更新实际目标
    if (is_left) {
        g_leg_left_target_length = leg_length;
        g_leg_left_target_angle = body_angle;
        g_leg_left_hip_angle = joint.hip_angle;
        g_leg_left_knee_angle = joint.knee_angle;
    } else {
        g_leg_right_target_length = leg_length;
        g_leg_right_target_angle = body_angle;
        g_leg_right_hip_angle = joint.hip_angle;
        g_leg_right_knee_angle = joint.knee_angle;
    }
    
    // 限流日志：仅在腿长变化超过阈值时打印
    static float s_last_log_left_len = 0, s_last_log_right_len = 0;
    float *p_last_len = is_left ? &s_last_log_left_len : &s_last_log_right_len;
    if (fabsf(leg_length - *p_last_len) > 0.005f) {  // 变化超过 5mm 才打印
        ESP_LOGI(TAG, "%s leg target: L=%.3fm, A=%.1fdeg -> Hip=%.1f, Knee=%.1f",
                 is_left ? "Left" : "Right", leg_length, body_angle, 
                 joint.hip_angle, joint.knee_angle);
        *p_last_len = leg_length;
    }
    
    return ESP_OK;
}

/**
 * @brief 设置双腿目标状态
 */
esp_err_t leg_ctrl_set_both(float left_length, float left_angle,
                             float right_length, float right_angle) {
    esp_err_t ret1 = leg_ctrl_set_target(true, left_length, left_angle);
    esp_err_t ret2 = leg_ctrl_set_target(false, right_length, right_angle);
    
    if (ret1 != ESP_OK) return ret1;
    if (ret2 != ESP_OK) return ret2;
    
    return ESP_OK;
}

/**
 * @brief 打印当前腿部状态
 */
void leg_ctrl_print_status(void) {
    leg_state_t left, right;
    
    // 获取当前状态
    bool left_valid = (leg_ctrl_get_state(true, &left) == ESP_OK && left.valid);
    bool right_valid = (leg_ctrl_get_state(false, &right) == ESP_OK && right.valid);
    
    // 输出机器可解析格式 (供 Qt 面板使用)
    // 格式: LEG_STATE: L_Len=xxx L_Ang=xxx L_Hip=xxx L_Knee=xxx R_Len=xxx R_Ang=xxx R_Hip=xxx R_Knee=xxx
    printf("LEG_STATE: L_Len=%.3f L_Ang=%.1f L_Hip=%.1f L_Knee=%.1f R_Len=%.3f R_Ang=%.1f R_Hip=%.1f R_Knee=%.1f\n",
           left_valid ? left.workspace.leg_length : 0.0f,
           left_valid ? left.workspace.body_angle : 0.0f,
           left_valid ? left.joint.hip_angle : 0.0f,
           left_valid ? left.joint.knee_angle : 0.0f,
           right_valid ? right.workspace.leg_length : 0.0f,
           right_valid ? right.workspace.body_angle : 0.0f,
           right_valid ? right.joint.hip_angle : 0.0f,
           right_valid ? right.joint.knee_angle : 0.0f);
    
    // 输出人类可读格式
    printf("=== Leg Control Status ===\n");
    printf("Control: %s\n", g_leg_control_enabled ? "ENABLED" : "DISABLED");
    printf("Speed: %.1f rpm\n\n", g_leg_move_speed);
    
    // 左腿
    if (left_valid) {
        printf("Left Leg (current):\n");
        printf("  Motor: Hip=%.1f deg, Knee=%.1f deg\n", 
               left.joint.hip_angle, left.joint.knee_angle);
        printf("  State: Length=%.3f m, BodyAngle=%.1f deg\n", 
               left.workspace.leg_length, left.workspace.body_angle);
    } else {
        printf("Left Leg: NOT AVAILABLE\n");
    }
    printf("  Target: Length=%.3f m, BodyAngle=%.1f deg\n", 
           g_leg_left_target_length, g_leg_left_target_angle);
    printf("  Target Motor: Hip=%.1f, Knee=%.1f\n\n", 
           g_leg_left_hip_angle, g_leg_left_knee_angle);
    
    // 右腿
    if (right_valid) {
        printf("Right Leg (current):\n");
        printf("  Motor: Hip=%.1f deg, Knee=%.1f deg\n", 
               right.joint.hip_angle, right.joint.knee_angle);
        printf("  State: Length=%.3f m, BodyAngle=%.1f deg\n", 
               right.workspace.leg_length, right.workspace.body_angle);
    } else {
        printf("Right Leg: NOT AVAILABLE\n");
    }
    printf("  Target: Length=%.3f m, BodyAngle=%.1f deg\n", 
           g_leg_right_target_length, g_leg_right_target_angle);
    printf("  Target Motor: Hip=%.1f, Knee=%.1f\n", 
           g_leg_right_hip_angle, g_leg_right_knee_angle);
}

