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
 * │   task_balance_ctrl   - LQR 平衡控制 (优先级 24, 5ms)       │
 * │   task_motor_comm     - CAN 电机通信 (优先级 20, 2ms)       │
 * └─────────────────────────────────────────────────────────────┘
 */

#include "balance_test.h"
#include "config.h"
#include "types.h"
#include "can_motor.h"
#include "imu_driver.h"
#include "wit_reg.h"      // RRATE_200HZ 定义
#include "wifi_remote.h"
#include "lqr_balance.h"
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
#define IMU_READ_PERIOD_MS          3       // 333Hz IMU 读取 (≈300Hz)
#define BALANCE_CTRL_PERIOD_MS      5       // 200Hz 平衡控制
#define MOTOR_COMM_PERIOD_MS        5       // 200Hz 电机通信 (与控制同步)
#define LEG_MOTOR_DIVIDER           1       // 腿电机分频 (200Hz / 1 = 200Hz)
#define WATCHDOG_PERIOD_MS          100     // 10Hz

// ===== 合并任务配置 =====
// 将 IMU读取 + 控制算法 + 电机通信 合并到一个任务中运行
// 默认使用分离任务架构，可通过 CLI 切换
#define UNIFIED_TASK_PERIOD_MS      5       // 合并任务基础周期 (200Hz)
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

// 任务优先级
#define TASK_PRIO_IMU               22
#define TASK_PRIO_BALANCE           24
#define TASK_PRIO_MOTOR             20
#define TASK_PRIO_WATCHDOG          8

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

// 任务架构选择
static bool g_use_unified_task = true;            // true=使用合并任务(默认), false=使用分离任务

// 功能开关
static bool g_uncontrolable_check_enabled = false; // true=启用失控检测(默认), false=禁用失控检测

// 控制模式选择
typedef enum {
    CTRL_MODE_LQR = 0,      // LQR 多环控制 (默认)
    CTRL_MODE_DUAL_PID,     // 双环 PID 控制 (直立环+速度环) - 扭矩模式
    CTRL_MODE_SINGLE_PID,   // 单环 PID 控制 (直立环→速度) - 速度模式
} control_mode_t;
static control_mode_t g_control_mode = CTRL_MODE_LQR;  // 默认 LQR 模式

// 双环 PID 控制器
static dual_pid_controller_t g_dual_pid_ctrl;
static bool g_dual_pid_initialized = false;
static dual_pid_output_t g_dual_pid_output;       // 保存输出用于调试

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

// 轮速加速度计算 (用于离地检测)
static float g_left_wheel_speed_rad = 0.0f;     // 左轮速度 (rad/s)
static float g_right_wheel_speed_rad = 0.0f;    // 右轮速度 (rad/s)
static float g_left_wheel_accel = 0.0f;         // 左轮加速度 (rad/s²)
static float g_right_wheel_accel = 0.0f;        // 右轮加速度 (rad/s²)
static float g_prev_left_wheel_speed = 0.0f;    // 上次左轮速度
static float g_prev_right_wheel_speed = 0.0f;   // 上次右轮速度
static bool g_wheel_off_ground = false;          // 轮子离地标志

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

// PID 调试输出 (用于实时 debug)
static bool g_pid_debug_enabled = false;    // PID 调试输出使能
static uint8_t g_pid_debug_divider = 50;    // 输出分频 (每N次控制循环输出一次，默认约4Hz@200Hz)
static uint8_t g_pid_debug_counter = 0;     // 分频计数器

// 环路使能控制 (用于单环调试)
static uint8_t g_loop_enable_mask = LOOP_FULL;  // 默认全部启用
static bool g_yaw_control_enabled = true;       // YAW 控制独立开关
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
static float g_leg_length_min = 0.045f;       // 最小腿长 (m), 默认与 LEG_LENGTH_MIN 一致
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
            
        case CTRL_ID_DISTANCE:  // C - 位移控制
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
            
        case CTRL_ID_DISTANCE:  // C - 位移控制
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
    
    // 输出各通道数据
    // A - 角度: target=0(平衡点), control=当前角度
    printf("#DATA,A,0.0,%.2f\n", input->pitch);
    
    // B - 角速度: target=0, control=当前角速度  
    printf("#DATA,B,0.0,%.2f\n", input->pitch_rate);
    
    // C - 位移: target=目标位移, control=当前位移
    printf("#DATA,C,%.2f,%.2f\n", g_distance_zeropoint, g_lqr_distance);
    
    // D - 速度: target=目标速度(原始), control=当前速度
    // 注: 蓝色=原始目标速度, 红色=当前速度
    printf("#DATA,D,%.2f,%.2f\n", input->target_speed, input->lqr_speed);
    
    // E - YAW 角度: target=目标角度, control=当前累积角度
    printf("#DATA,E,%.2f,%.2f\n", g_lqr_ctrl.yaw_angle_target, g_yaw_angle_total);
    
    // F - YAW 角速度: target=目标角速度, control=当前角速度
    printf("#DATA,F,%.2f,%.2f\n", input->target_yaw_rate, input->yaw_rate);
    
    // G - 摇杆滤波: target=滤波前(原始), control=滤波后
    printf("#DATA,G,%.2f,%.2f\n", input->target_speed, output->filtered_target_speed);
    
    // H - LQR 输出: target=0, control=LQR_u
    printf("#DATA,H,0.0,%.2f\n", g_last_lqr_u);
    
    // I - 零点调整: target=0, control=当前零点偏移
    printf("#DATA,I,0.0,%.3f\n", g_lqr_ctrl.distance_zeropoint);
    
    // J - 零点滤波: target=滤波前(原始), control=滤波后
    printf("#DATA,J,%.4f,%.4f\n", output->zeropoint_adjust_raw, output->zeropoint_adjust_filtered);
    
    // K - Roll 角度: target=0, control=当前 Roll
    printf("#DATA,K,0.0,%.2f\n", input->roll);
    
    // L - Roll 滤波: target=滤波前(原始 Roll), control=滤波后
    // 使用控制器内的 lpf_roll 滤波器，但不影响主控制（仅用于显示）
    float roll_filtered_display = lpf_compute_dt(&g_lqr_ctrl.lpf_roll, input->roll, input->dt);
    printf("#DATA,L,%.2f,%.2f\n", input->roll, roll_filtered_display);
    
    // O - 左轮: target=速度(rad/s), control=加速度(rad/s²)
    // 用于调试轮子离地检测阈值
    printf("#DATA,O,%.2f,%.2f\n", g_left_wheel_speed_rad, g_left_wheel_accel);
    
    // P - 右轮: target=速度(rad/s), control=加速度(rad/s²)
    printf("#DATA,P,%.2f,%.2f\n", g_right_wheel_speed_rad, g_right_wheel_accel);
    
    // ===== 各环路控制输出量 (用于观察每个环路的贡献) =====
    // Q - 角度环输出: target=0, control=angle_control
    printf("#DATA,Q,0.0,%.4f\n", output->angle_control);
    
    // R - 角速度环输出: target=0, control=gyro_control
    printf("#DATA,R,0.0,%.4f\n", output->gyro_control);
    
    // S - 位移环输出: target=0, control=distance_control
    printf("#DATA,S,0.0,%.4f\n", output->distance_control);
    
    // T - 速度环输出: target=0, control=speed_control
    printf("#DATA,T,0.0,%.4f\n", output->speed_control);
    
    // U - YAW 控制输出: target=0, control=yaw_control
    printf("#DATA,U,0.0,%.4f\n", g_yaw_output);
    
    // V - LQR 输出 PID 处理前后对比: target=处理前(raw), control=处理后(final)
    printf("#DATA,V,%.4f,%.4f\n", output->lqr_u_raw, output->lqr_u);
    
    // Y - YAW 调试输出 (专用通道，用于 YAW 面板)
    printf("#DATA,Y,%.2f,%.2f\n", g_lqr_ctrl.yaw_angle_target, g_yaw_angle_total);
    
    // YAW 输出和状态 (详细调试)
    printf("#YAW_DBG,out=%.3f,err=%.2f,hold=%d,rate=%.2f\n", 
           g_yaw_output, 
           g_yaw_angle_total - g_lqr_ctrl.yaw_angle_target,
           g_lqr_ctrl.yaw_holding ? 1 : 0,
           input->yaw_rate);
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
    
    // Clamp to 0.0~1.0
    if (enable < 0.0f) enable = 0.0f;
    if (enable > 1.0f) enable = 1.0f;
    
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
    printf("  [%c] Y - YAW (turning)      : %s\n", 
           (g_loop_enable_mask & LOOP_YAW) ? 'X' : ' ',
           g_yaw_control_enabled ? "ON" : "OFF");
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
        
        vmc_dual_input_t dual_input = {
            .pitch_deg = pitch_deg,
            .pitch_rate_deg = pitch_rate_deg,
            .robot_vx = robot_vx,
            .target_vx = g_vmc_target_vx,
            .left = {
                .target_leg_length = g_leg_left_target_length,
                .target_body_angle_deg = -90.0f,  // 垂直向下
                .sensor = {
                    .hip_angle = left_valid ? can_motor_read_position(g_motor_left_hip) : 0,
                    .knee_angle = left_valid ? can_motor_read_position(g_motor_left_knee) : 0,
                    .hip_velocity = (left_valid && need_velocity) ? can_motor_read_speed(g_motor_left_hip) * 6.0f : 0,
                    .knee_velocity = (left_valid && need_velocity) ? can_motor_read_speed(g_motor_left_knee) * 6.0f : 0
                }
            },
            .right = {
                .target_leg_length = g_leg_right_target_length,
                .target_body_angle_deg = -90.0f,
                .sensor = {
                    .hip_angle = right_valid ? can_motor_read_position(g_motor_right_hip) : 0,
                    .knee_angle = right_valid ? can_motor_read_position(g_motor_right_knee) : 0,
                    .hip_velocity = (right_valid && need_velocity) ? can_motor_read_speed(g_motor_right_hip) * 6.0f : 0,
                    .knee_velocity = (right_valid && need_velocity) ? can_motor_read_speed(g_motor_right_knee) * 6.0f : 0
                }
            }
        };
        
        if (vmc_dual_compute(&g_vmc_params, &dual_input, &g_vmc_dual_output) == ESP_OK) {
            g_vmc_input_valid = true;
            
            // VMC 数据流输出 (用于 UI 调试)
            // 格式: #VMC,L_len,L_ang,L_FL,L_Fa,L_hip,L_knee,R_len,R_ang,R_FL,R_Fa,R_hip,R_knee,diff,Fsync
            if (g_vmc_stream_enable) {
                printf("#VMC,%.3f,%.1f,%.2f,%.3f,%.2f,%.2f,%.3f,%.1f,%.2f,%.3f,%.2f,%.2f,%.2f,%.3f\n",
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
                       g_vmc_dual_output.F_sync);
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
        if (g_motor_left_hip) {
            can_motor_set_torque(g_motor_left_hip, g_vmc_dual_output.left.hip_torque);
        }
        if (g_motor_left_knee) {
            can_motor_set_torque(g_motor_left_knee, g_vmc_dual_output.left.knee_torque);
        }
        if (g_motor_right_hip) {
            can_motor_set_torque(g_motor_right_hip, g_vmc_dual_output.right.hip_torque);
        }
        if (g_motor_right_knee) {
            can_motor_set_torque(g_motor_right_knee, g_vmc_dual_output.right.knee_torque);
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

static void update_remote_from_wifi(void);
static void compute_balance_output(float dt);
static void apply_motor_commands(void);
static esp_err_t leg_ctrl_get_state_cached(bool is_left, leg_state_t *state);

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
    
    // 初始化 Commander 解析器
    // - set_callback: 收到设置命令时更新 LQR 参数
    // - query_callback: 收到查询命令时返回 LQR 实际参数
    commander_parser_init(commander_param_callback, commander_query_callback);
    ESP_LOGI(TAG, "Commander parser initialized with LQR callbacks");
    
    // 初始化 X-Offset PID (腿部速度自适应偏移)
    {
        pid_params_t xoffset_params = {
            .kp = 0.01f,           // 默认比例增益 (m/s → m)
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
    
    g_tasks_running = true;
    g_state = BALANCE_TEST_READY;
    
    ESP_LOGI(TAG, "Balance test tasks started (%s mode)", 
             g_use_unified_task ? "unified" : "separate");
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "Connect to WiFi: WL-PRO (password: 12345678)");
    ESP_LOGI(TAG, "Open http://192.168.4.1 in browser");
    ESP_LOGI(TAG, "Toggle 'Robot Go!' switch to enable balance");
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
    
    // 重置控制器
    lqr_reset(&g_lqr_ctrl);
    g_distance_zeropoint = g_lqr_distance;
    lqr_set_distance_zeropoint(&g_lqr_ctrl, g_distance_zeropoint);  // 同步到 LQR 控制器
    
    // 重置 YAW 累积角度，并初始化为当前方向 (避免启动跳变)
    g_yaw_angle_total = 0.0f;
    g_yaw_first_run = true;  // 让下一帧重新初始化 yaw_angle_last
    g_uncontrolable = 0;
    
    // 电机进入闭环 (根据控制模式选择电机模式)
    if (g_control_mode == CTRL_MODE_SINGLE_PID) {
        // 单环 PID 使用速度模式
        can_motor_set_mode(g_motor_left, MODE_SPEED);
        can_motor_set_mode(g_motor_right, MODE_SPEED);
        ESP_LOGI(TAG, "Motor mode: SPEED (single PID)");
    } else {
        // LQR / 双环 PID 使用扭矩模式
        can_motor_set_mode(g_motor_left, MODE_TORQUE);
        can_motor_set_mode(g_motor_right, MODE_TORQUE);
        ESP_LOGI(TAG, "Motor mode: TORQUE (LQR/dual PID)");
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
    ESP_LOGI(TAG, "Angle zeropoint set to %.2f (synced to all controllers)", zeropoint);
}

float balance_test_get_angle_zeropoint(void) {
    return g_angle_zeropoint;
}

void balance_test_print_status(void) {
    const char *state_names[] = {"IDLE", "READY", "RUNNING", "EMERGENCY", "ERROR"};
    const char *mode_names[] = {"LQR", "DUAL_PID", "SINGLE_PID"};
    
    ESP_LOGI(TAG, "=== Balance Test Status ===");
    ESP_LOGI(TAG, "State: %s", state_names[g_state]);
    ESP_LOGI(TAG, "Task mode: %s", g_use_unified_task ? "UNIFIED" : "SEPARATE");
    ESP_LOGI(TAG, "Control mode: %s (%s)", 
             mode_names[g_control_mode],
             g_control_mode == CTRL_MODE_SINGLE_PID ? "speed output" : "torque output");
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
 * @brief 合并任务: IMU读取 + 控制算法 + 电机通信 (Core 1, 5ms 周期)
 * 
 * 将三个任务合并到一个任务中执行，减少任务切换开销和同步延迟。
 * 执行顺序: 1.发送电机命令 → 2.读IMU → 3.处理CAN接收 → 4.计算控制
 * 先发命令让电机有时间回复，IMU读取期间(~0.3-1ms)电机回复到达，
 * 然后 process_rx 获取最新数据，控制计算用的是最新电机状态。
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
            g_imu_data.timestamp = imu_raw.timestamp;
            g_imu_data.read_time_us = cycle_start;
            g_imu_data.valid = true;
            
            g_stats.imu_read_count++;
            g_imu_count_per_sec++;
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
static void task_remote_watchdog(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(WATCHDOG_PERIOD_MS);
    
    ESP_LOGI(TAG, "[task_remote_watchdog] Started on Core %d", xPortGetCoreID());
    
    while (g_tasks_running) {
        // 检查遥控超时
        if (wifi_remote_check_timeout(REMOTE_TIMEOUT_MS)) {
            // 超时，禁用遥控输入
            xSemaphoreTake(g_remote_mutex, portMAX_DELAY);
            if (g_remote_data.go) {
                ESP_LOGW(TAG, "Remote timeout, disabling go");
                g_remote_data.go = false;
            }
            xSemaphoreGive(g_remote_mutex);
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
    g_remote_data.last_update = wifi_data->last_update_ms;
    g_remote_data.receive_time_us = wifi_data->receive_time_us;  // 复制精确时间戳
    
    xSemaphoreGive(g_remote_mutex);
    
    // 根据 go 状态切换平衡控制
    static bool last_go = false;
    if (wifi_data->go && !last_go) {
        // go 从 false 变为 true
        balance_test_enable();
    } else if (!wifi_data->go && last_go) {
        // go 从 true 变为 false
        balance_test_disable();
    }
    last_go = wifi_data->go;
}

/**
 * @brief 计算平衡控制输出
 * @param dt 时间步长 (秒)
 * @note 参考 shibo_wheel_leg 的 lqr_balance_loop
 */
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
    {
        float speed_th = g_lqr_ctrl.params.wheel_off_ground_speed_threshold;
        float accel_th = g_lqr_ctrl.params.wheel_off_ground_accel_threshold;
        bool left_off = (fabsf(left_vel_rad) > speed_th) || (fabsf(g_left_wheel_accel) > accel_th);
        bool right_off = (fabsf(right_vel_rad) > speed_th) || (fabsf(g_right_wheel_accel) > accel_th);
        bool off_ground_now = left_off || right_off;
        
        if (off_ground_now && !g_wheel_off_ground) {
            ESP_LOGW(TAG, "WHEEL OFF GROUND! L: spd=%.1f acc=%.1f  R: spd=%.1f acc=%.1f",
                     left_vel_rad, g_left_wheel_accel, right_vel_rad, g_right_wheel_accel);
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
        .target_speed = remote.joy_y * 0.1f,    // joy_y (-100~100) -> target speed
        .target_yaw_rate = -remote.joy_x * 0.01f,  // joy_x -> target yaw rate
        .dt = dt,
    };
    
    // ======== 运动细节优化 (参考 shibo_wheel_leg) ========
    
    // 有前后方向运动指令时，重置位移零点 (仅在刚开始移动时)
    static bool was_moving = false;
    bool is_moving = (remote.joy_y != 0);
    if (is_moving && !was_moving) {
        // joy_y 从 0 变为非 0，开始移动
        g_distance_zeropoint = g_lqr_distance;
        lqr_set_distance_zeropoint(&g_lqr_ctrl, g_distance_zeropoint);
        lqr_reset(&g_lqr_ctrl);  // 仅重置一次
    }
    if (is_moving) {
        // 移动过程中持续更新位移零点 (防止位移环干扰)
        g_distance_zeropoint = g_lqr_distance;
        lqr_set_distance_zeropoint(&g_lqr_ctrl, g_distance_zeropoint);
    }
    was_moving = is_moving;
    
    // 运动指令复零时的原地停车处理
    if ((remote.joy_x_last != 0 && remote.joy_x == 0) ||
        (remote.joy_y_last != 0 && remote.joy_y == 0)) {
        g_move_stop_flag = 1;
    }
    if ((g_move_stop_flag == 1) && (fabsf(g_lqr_speed) < 0.5f)) {
        g_distance_zeropoint = g_lqr_distance;
        lqr_set_distance_zeropoint(&g_lqr_ctrl, g_distance_zeropoint);  // 同步到 LQR 控制器
        g_move_stop_flag = 0;
    }
    
    // 被快速推动时的原地停车处理
    if (fabsf(g_lqr_speed) > 15.0f) {
        g_distance_zeropoint = g_lqr_distance;
        lqr_set_distance_zeropoint(&g_lqr_ctrl, g_distance_zeropoint);  // 同步到 LQR 控制器
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
        
        esp_err_t ret = dual_pid_balance_loop(&g_dual_pid_ctrl, 
                                               pitch_for_control, imu.pitch_rate,
                                               wheel_speed_avg, dt,
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
            
            // YAW 控制 (可选，仅在 go=true 时)
            if (remote.go && g_yaw_control_enabled) {
                lqr_yaw_loop(&g_lqr_ctrl, &input, &output);
                g_yaw_output = output.yaw_control;
                output.left_wheel_torque = torque + g_yaw_output;
                output.right_wheel_torque = torque - g_yaw_output;
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
            
            // YAW 控制 (可选，仅在 go=true 时)
            // 单环模式下 YAW 也是速度差速
            if (remote.go && g_yaw_control_enabled) {
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
        }
        
        // 填充兼容字段用于波形显示
        output.angle_control = g_single_pid_output.angle_p_out;
        output.gyro_control = g_single_pid_output.angle_d_out;
        output.speed_control = 0;  // 单环无速度环
        output.filtered_target_speed = g_single_pid_output.target_speed;
        
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
            // 只保留角度+角速度控制，去掉位移和速度分量
            output.lqr_u = output.angle_control + output.gyro_control;
            pid_reset(&g_lqr_ctrl.pid_lqr_u);
        }
        
        // ======== YAW 轴转向控制 (仅在 go=true 且 YAW 环使能时生效) ========
        // 修改: go 只控制 YAW 环路是否加入
        if (remote.go && g_uncontrolable == 0 && g_yaw_control_enabled) {
            // 使用 lqr_yaw_loop 计算 YAW 控制量
            // input.target_yaw_rate 已经在前面设置为 remote.joy_x * 0.02f
            lqr_yaw_loop(&g_lqr_ctrl, &input, &output);
            
            // 保存 YAW 输出用于调试
            g_yaw_output = output.yaw_control;
            
            // 合成输出 (参考 shibo_wheel_leg)
            // 平衡控制输出 + YAW 差速
            float lqr_u = output.lqr_u;
            output.left_wheel_torque = lqr_u + g_yaw_output;
            output.right_wheel_torque = lqr_u - g_yaw_output;
        } else {
            // 简单模式/YAW禁用时，直接使用 LQR 输出，无 YAW 控制
            float lqr_u = output.lqr_u;
            output.left_wheel_torque = lqr_u;
            output.right_wheel_torque = lqr_u;
            g_yaw_output = 0.0f;
        }
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
        if (g_leg_control_enabled && g_roll_control_enabled) {
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
        } else {
            // Roll 控制未启用时，使用基础腿长 (左右对称)
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
    
    // ======== X-Offset 计算 (非 LQR 模式: Dual PID / Single PID) ========
    // LQR 模式的 x_offset 已在上面计算, 这里处理 Dual PID 和 Single PID 模式
    if (g_control_mode != CTRL_MODE_LQR && g_xoffset_enabled && g_leg_control_enabled) {
        g_xoffset_debug_speed = g_lqr_speed;
        g_xoffset_value = pid_compute(&g_xoffset_pid, g_lqr_speed, 0.0f, dt);
    } else if (g_control_mode != CTRL_MODE_LQR) {
        g_xoffset_value = 0.0f;
        g_xoffset_debug_speed = 0.0f;
        if (!g_xoffset_enabled) {
            pid_reset(&g_xoffset_pid);
        }
    }
    
    // ======== Roll 控制 + X-Offset (非 LQR 模式通用: Dual PID / Single PID) ========
    // 注: 双环PID和单环PID模式下也可以使用 Roll 控制和 X-Offset
    bool is_non_lqr = (g_control_mode == CTRL_MODE_DUAL_PID || g_control_mode == CTRL_MODE_SINGLE_PID);
    if (is_non_lqr && g_leg_control_enabled && g_roll_control_enabled) {
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
    } else if (is_non_lqr && g_leg_control_enabled && !g_roll_control_enabled) {
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
    
    // ======== 输出波形数据 (用于 Qt 调参面板) ========
    output_plot_data(&input, &output);
    
    // ======== 输出 PID 调试信息 ========
    output_pid_debug(&input);
    
    // ======== 更新轮命令 ========
    xSemaphoreTake(g_wheel_cmd_mutex, portMAX_DELAY);
    if (g_wheel_off_ground) {
        // 离地时清零输出，防止轮子空转
        g_wheel_cmd.left_torque = 0;
        g_wheel_cmd.right_torque = 0;
    } else {
        g_wheel_cmd.left_torque = output.left_wheel_torque;
        g_wheel_cmd.right_torque = output.right_wheel_torque;
    }
    // 单环 PID 模式使用速度模式，其他模式使用扭矩模式
    g_wheel_cmd.use_speed_mode = (g_control_mode == CTRL_MODE_SINGLE_PID);
    // enabled 状态由 balance_test_enable/disable 控制
    xSemaphoreGive(g_wheel_cmd_mutex);
}

/**
 * @brief 发送电机命令
 */
static void apply_motor_commands(void) {
    shared_wheel_cmd_t cmd;
    
    xSemaphoreTake(g_wheel_cmd_mutex, portMAX_DELAY);
    memcpy(&cmd, &g_wheel_cmd, sizeof(cmd));
    xSemaphoreGive(g_wheel_cmd_mutex);
    
    if (cmd.enabled && g_state == BALANCE_TEST_RUNNING) {
        if (cmd.use_speed_mode) {
            // 速度模式 (单环 PID 输出): left_torque/right_torque 实际存储的是速度 (rad/s)
            // 转换: rad/s → rpm (rpm = rad/s * 60 / 2π ≈ rad/s * 9.5493)
            float left_speed_rpm = cmd.left_torque * 9.5493f;
            float right_speed_rpm = cmd.right_torque * 9.5493f;
            can_motor_set_speed(g_motor_left, left_speed_rpm);
            can_motor_set_speed(g_motor_right, right_speed_rpm);
        } else {
            // 扭矩模式 (LQR / 双环 PID 输出)
            can_motor_set_torque(g_motor_left, cmd.left_torque);
            can_motor_set_torque(g_motor_right, cmd.right_torque);
        }
    } else {
        // 停止时使用扭矩模式发送 0
        can_motor_set_torque(g_motor_left, 0);
        can_motor_set_torque(g_motor_right, 0);
    }
    
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
        if (token) {
            float zero = atof(token);
            balance_test_set_angle_zeropoint(zero);
            printf("Angle zeropoint set to %.2f\n", zero);
        } else {
            printf("Current angle zeropoint: %.2f\n", balance_test_get_angle_zeropoint());
            printf("Usage: balance zero <degrees>\n");
        }
    }
    else if (strcmp(token, "plot") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            printf("Plot output: %s\n", g_plot_enabled ? "enabled" : "disabled");
            printf("Plot divider: %d (%.1f Hz)\n", g_plot_divider, 200.0f / g_plot_divider);
            printf("Usage: balance plot [on|off|div <N>]\n");
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
        } else {
            printf("Unknown plot command: %s\n", token);
            printf("Usage: balance plot [on|off|div <N>]\n");
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
                   g_control_mode == CTRL_MODE_DUAL_PID ? "DUAL_PID" : "SINGLE_PID");
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
        } else {
            printf("Unknown vmc command: %s\n", token);
            printf("Usage: balance vmc [on|off|status|coord|soft|stiff|pitch|sync|stream]\n");
            printf("  World coord: kvx|ky|dy\n");
            printf("  Body coord:  kl|dl|ka|da\n");
            printf("  Common:      gc|mass|height|vx\n");
            printf("  Leg sync:    sync [on|off|kp|kd]\n");
            printf("  Stream:      stream [on|off]  (for UI debug)\n");
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
                                   (g_control_mode == CTRL_MODE_DUAL_PID) ? "DUAL_PID" : "SINGLE_PID";
            printf("Control mode: %s\n", mode_str);
            printf("CTRL_MODE:%s\n", mode_str);
            printf("  LQR:        Multi-loop LQR control (angle+gyro+dist+speed) → torque\n");
            printf("  DUAL_PID:   Dual-loop PID (angle→speed→torque) → torque mode\n");
            printf("  SINGLE_PID: Single-loop PID (angle→speed) → speed mode\n");
            printf("Usage: balance mode [lqr|pid|spid]\n");
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
        } else {
            printf("Unknown mode: %s\n", token);
            printf("Usage: balance mode [lqr|pid|spid]\n");
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
            printf("Max torque: %.1f Nm\n", g_dual_pid_ctrl.params.max_torque);
            printf("\nUsage: balance dpid angle <kp> <ki> <kd>\n");
            printf("       balance dpid speed <kp> <ki> <kd>\n");
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
        } else if (strcmp(token, "zero") == 0) {
            // 设置角度零点
            token = strtok(NULL, " \t\n\r");
            if (token) {
                float zero = atof(token);
                dual_pid_set_angle_zeropoint(&g_dual_pid_ctrl, zero);
                printf("Dual PID angle zeropoint set to %.2f\n", zero);
            } else {
                printf("Current angle zeropoint: %.2f\n", g_dual_pid_ctrl.params.angle_zeropoint);
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
            // 输出 Qt 可解析格式
            printf("DPID_STATUS:PITCH_ERR=%.2f,TGT_SPD=%.2f,SPD_ERR=%.2f,TORQUE=%.2f,ORDER=%d\n",
                   g_dual_pid_output.angle_error, g_dual_pid_output.target_speed,
                   g_dual_pid_output.speed_error, g_dual_pid_output.torque,
                   g_dual_pid_ctrl.params.loop_order);
        } else {
            printf("Unknown dpid command: %s\n", token);
            printf("Usage: balance dpid [angle|speed|zero|order|reset|status]\n");
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
            // 设置角度零点
            token = strtok(NULL, " \t\n\r");
            if (token) {
                float zero = atof(token);
                single_pid_set_angle_zeropoint(&g_single_pid_ctrl, zero);
                printf("Single PID angle zeropoint set to %.2f\n", zero);
            } else {
                printf("Current angle zeropoint: %.2f\n", g_single_pid_ctrl.params.angle_zeropoint);
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
    else {
        printf("Unknown command: %s\n", token);
        printf("Usage: balance [init|start|stop|enable|disable|estop|reset|status|zero|plot|debug|leg|roll|mzero|loop|task|safety|airborne|mode|dpid|spid]\n");
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

