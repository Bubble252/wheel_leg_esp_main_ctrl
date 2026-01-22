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
#include "wifi_remote.h"
#include "lqr_balance.h"
#include "power_detect.h"
#include "commander_parser.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "BAL_TEST";

// ============================================================================
// 任务配置
// ============================================================================

// 任务周期 (ms) - 覆盖 config.h 中的默认值
// FreeRTOS tick = 1ms (CONFIG_FREERTOS_HZ=1000)
#undef IMU_READ_PERIOD_MS
#undef BALANCE_CTRL_PERIOD_MS
#undef MOTOR_COMM_PERIOD_MS
#define IMU_READ_PERIOD_MS          2       // 500Hz IMU 读取
#define BALANCE_CTRL_PERIOD_MS      5       // 200Hz 平衡控制
#define MOTOR_COMM_PERIOD_MS        2       // 500Hz 电机通信
#define WATCHDOG_PERIOD_MS          100     // 10Hz

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
} shared_remote_data_t;

// 轮电机命令 (由 task_balance_ctrl 写入, task_motor_comm 读取)
typedef struct {
    float left_torque;      // 左轮力矩
    float right_torque;     // 右轮力矩
    bool enabled;           // 使能标志
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

// 状态
static balance_test_state_t g_state = BALANCE_TEST_IDLE;
static bool g_initialized = false;
static bool g_tasks_running = false;

// 统计
static balance_test_stats_t g_stats = {0};
static uint32_t g_imu_count_per_sec = 0;
static uint32_t g_ctrl_count_per_sec = 0;
static uint32_t g_last_stat_time = 0;

// LQR 内部状态 (来自 shibo_wheel_leg)
static float g_lqr_distance = 0.0f;         // 累积位移 (机器人前进方向为正)
static float g_lqr_speed = 0.0f;            // 当前速度 (机器人前进方向为正)
static float g_distance_zeropoint = 0.0f;   // 位移零点
static float g_angle_zeropoint = 7.4f;      // 角度零点 (需要根据实际机器人调整)
static int g_move_stop_flag = 0;            // 停止标志
static int g_uncontrolable = 0;             // 失控标志

// YAW 轴控制 (带过零处理)
static float g_yaw_angle_last = 0.0f;       // 上一次 YAW 角度 (用于过零处理)
static float g_yaw_angle_total = 0.0f;      // YAW 累积角度 (经过过零处理)
static float g_yaw_output = 0.0f;           // YAW 控制输出
static bool g_yaw_first_run = true;         // YAW 首次运行标志

// 波形数据输出 (用于 Qt 调参面板)
static bool g_plot_enabled = false;         // 波形输出使能
static uint8_t g_plot_divider = 10;         // 输出分频 (每N次控制循环输出一次)
static uint8_t g_plot_counter = 0;          // 分频计数器
static float g_last_lqr_u = 0.0f;           // 保存 LQR 输出用于波形显示

// ============================================================================
// 腿部电机控制 (固定角度模式)
// ============================================================================
static bool g_leg_control_enabled = false;  // 腿部电机使能
static can_motor_handle_t g_motor_left_hip = NULL;    // ID=1 左大腿
static can_motor_handle_t g_motor_left_knee = NULL;   // ID=2 左小腿
static can_motor_handle_t g_motor_right_hip = NULL;   // ID=4 右大腿
static can_motor_handle_t g_motor_right_knee = NULL;  // ID=5 右小腿

// 腿部电机目标角度 (度) - 可通过命令修改
static float g_leg_left_hip_angle = 0.0f;    // 左大腿角度
static float g_leg_left_knee_angle = 0.0f;   // 左小腿角度
static float g_leg_right_hip_angle = 0.0f;   // 右大腿角度
static float g_leg_right_knee_angle = 0.0f;  // 右小腿角度
static float g_leg_move_speed = 50.0f;       // 腿部电机运动速度 (rpm)

// Roll 闭环控制开关 (与腿长相关，纯轮测试时禁用)
static bool g_roll_control_enabled = false;  // 默认禁用

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
    
    // D - 速度: target=目标速度, control=当前速度
    printf("#DATA,D,%.2f,%.2f\n", input->target_speed, input->lqr_speed);
    
    // H - LQR 输出: target=0, control=LQR_u
    printf("#DATA,H,0.0,%.2f\n", g_last_lqr_u);
    
    // K - Roll 角度: target=0, control=当前 Roll
    printf("#DATA,K,0.0,%.2f\n", input->roll);
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
        if (g_motor_left_hip) {
            can_motor_set_mode(g_motor_left_hip, MODE_POS_FILTER);
            can_motor_enter_closed_loop(g_motor_left_hip);
        }
        if (g_motor_left_knee) {
            can_motor_set_mode(g_motor_left_knee, MODE_POS_FILTER);
            can_motor_enter_closed_loop(g_motor_left_knee);
        }
        if (g_motor_right_hip) {
            can_motor_set_mode(g_motor_right_hip, MODE_POS_FILTER);
            can_motor_enter_closed_loop(g_motor_right_hip);
        }
        if (g_motor_right_knee) {
            can_motor_set_mode(g_motor_right_knee, MODE_POS_FILTER);
            can_motor_enter_closed_loop(g_motor_right_knee);
        }
        ESP_LOGI(TAG, "Leg motors ENABLED");
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
 * @brief 发送腿部电机位置命令 (在 motor_comm 任务中调用)
 */
static void apply_leg_motor_commands(void) {
    if (!g_leg_control_enabled) return;
    
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
// 内部函数声明
// ============================================================================

static void task_imu_read(void *arg);
static void task_balance_ctrl(void *arg);
static void task_motor_comm(void *arg);
static void task_remote_watchdog(void *arg);

static void update_remote_from_wifi(void);
static void compute_balance_output(float dt);
static void apply_motor_commands(void);

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
    }
    
    // 初始化 IMU
    ret = imu_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "IMU init failed");
        return ret;
    }
    
    // 初始化 LQR 控制器 (使用默认参数)
    ret = lqr_init(&g_lqr_ctrl, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "LQR init failed");
        return ret;
    }
    
    // 设置角度零点 (可根据实际机器人调整)
    lqr_set_angle_zeropoint(&g_lqr_ctrl, g_angle_zeropoint);
    
    // 初始化 Commander 解析器
    // - set_callback: 收到设置命令时更新 LQR 参数
    // - query_callback: 收到查询命令时返回 LQR 实际参数
    commander_parser_init(commander_param_callback, commander_query_callback);
    ESP_LOGI(TAG, "Commander parser initialized with LQR callbacks");
    
    // 初始化 WiFi 遥控 (但不启动)
    ret = wifi_remote_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "WiFi remote init failed, continuing anyway");
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
    
    // 创建任务 (Core 1 - 实时控制)
    xTaskCreatePinnedToCore(task_imu_read, "imu_read", TASK_STACK_IMU,
                            NULL, TASK_PRIO_IMU, &g_task_imu, 1);
    xTaskCreatePinnedToCore(task_balance_ctrl, "balance_ctrl", TASK_STACK_BALANCE,
                            NULL, TASK_PRIO_BALANCE, &g_task_balance, 1);
    xTaskCreatePinnedToCore(task_motor_comm, "motor_comm", TASK_STACK_MOTOR,
                            NULL, TASK_PRIO_MOTOR, &g_task_motor, 1);
    
    // 创建任务 (Core 0 - 非实时)
    xTaskCreatePinnedToCore(task_remote_watchdog, "watchdog", TASK_STACK_WATCHDOG,
                            NULL, TASK_PRIO_WATCHDOG, &g_task_watchdog, 0);
    
    g_tasks_running = true;
    g_state = BALANCE_TEST_READY;
    
    ESP_LOGI(TAG, "Balance test tasks started");
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
    
    // 删除任务
    if (g_task_imu) { vTaskDelete(g_task_imu); g_task_imu = NULL; }
    if (g_task_balance) { vTaskDelete(g_task_balance); g_task_balance = NULL; }
    if (g_task_motor) { vTaskDelete(g_task_motor); g_task_motor = NULL; }
    if (g_task_watchdog) { vTaskDelete(g_task_watchdog); g_task_watchdog = NULL; }
    
    // 停止 WiFi
    wifi_remote_stop();
    
    g_tasks_running = false;
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
    
    // 电机进入闭环
    can_motor_set_mode(g_motor_left, MODE_TORQUE);
    can_motor_set_mode(g_motor_right, MODE_TORQUE);
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
    ESP_LOGI(TAG, "Angle zeropoint set to %.2f", zeropoint);
}

float balance_test_get_angle_zeropoint(void) {
    return g_angle_zeropoint;
}

void balance_test_print_status(void) {
    const char *state_names[] = {"IDLE", "READY", "RUNNING", "EMERGENCY", "ERROR"};
    
    ESP_LOGI(TAG, "=== Balance Test Status ===");
    ESP_LOGI(TAG, "State: %s", state_names[g_state]);
    ESP_LOGI(TAG, "Angle zeropoint: %.2f deg", g_angle_zeropoint);
    ESP_LOGI(TAG, "Roll control: %s", g_roll_control_enabled ? "ENABLED" : "DISABLED");
    ESP_LOGI(TAG, "Leg control: %s", g_leg_control_enabled ? "ENABLED" : "DISABLED");
    if (g_leg_control_enabled) {
        ESP_LOGI(TAG, "  Leg angles: L_Hip=%.1f, L_Knee=%.1f, R_Hip=%.1f, R_Knee=%.1f",
                 g_leg_left_hip_angle, g_leg_left_knee_angle,
                 g_leg_right_hip_angle, g_leg_right_knee_angle);
    }
    ESP_LOGI(TAG, "Control freq: %.1f Hz", g_stats.control_freq_hz);
    ESP_LOGI(TAG, "IMU freq: %.1f Hz", g_stats.imu_freq_hz);
    
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
    
    while (1) {
        // 读取 IMU 数据
        if (imu_read_data(&imu) == ESP_OK && imu.is_valid) {
            xSemaphoreTake(g_imu_mutex, portMAX_DELAY);
            g_imu_data.pitch = imu.pitch;
            g_imu_data.pitch_rate = imu.gyro_y;
            g_imu_data.roll = imu.roll;
            g_imu_data.roll_rate = imu.gyro_x;
            g_imu_data.yaw = imu.yaw;
            g_imu_data.yaw_rate = imu.gyro_z;
            g_imu_data.timestamp = imu.timestamp;
            g_imu_data.valid = true;
            xSemaphoreGive(g_imu_mutex);
            
            g_stats.imu_read_count++;
            g_imu_count_per_sec++;
        }
        
        vTaskDelayUntil(&last_wake, period);
    }
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
    
    while (1) {
        // 更新遥控数据
        update_remote_from_wifi();
        
        // 计算平衡控制输出
        compute_balance_output(dt);
        
        // 更新统计
        g_stats.control_loop_count++;
        g_ctrl_count_per_sec++;
        
        // 每秒更新一次频率统计
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - g_last_stat_time >= 1000) {
            g_stats.imu_freq_hz = (float)g_imu_count_per_sec;
            g_stats.control_freq_hz = (float)g_ctrl_count_per_sec;
            g_imu_count_per_sec = 0;
            g_ctrl_count_per_sec = 0;
            g_last_stat_time = now;
        }
        
        vTaskDelayUntil(&last_wake, period);
    }
}

/**
 * @brief 电机通信任务 (Core 1, 2ms 周期)
 */
static void task_motor_comm(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(MOTOR_COMM_PERIOD_MS);
    
    ESP_LOGI(TAG, "[task_motor_comm] Started on Core %d", xPortGetCoreID());
    
    while (1) {
        // 处理 CAN 接收
        can_motor_process_rx();
        
        // 读取轮电机状态
        xSemaphoreTake(g_wheel_state_mutex, portMAX_DELAY);
        g_wheel_state.left_position = can_motor_read_position(g_motor_left);
        g_wheel_state.left_speed = can_motor_read_speed(g_motor_left);
        g_wheel_state.left_online = can_motor_is_online(g_motor_left, 100);
        g_wheel_state.right_position = can_motor_read_position(g_motor_right);
        g_wheel_state.right_speed = can_motor_read_speed(g_motor_right);
        g_wheel_state.right_online = can_motor_is_online(g_motor_right, 100);
        g_wheel_state.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
        xSemaphoreGive(g_wheel_state_mutex);
        
        // 发送电机力矩命令
        apply_motor_commands();
        
        // 发送腿部电机位置命令 (固定角度)
        apply_leg_motor_commands();
        
        g_stats.motor_cmd_count++;
        
        vTaskDelayUntil(&last_wake, period);
    }
}

/**
 * @brief 遥控看门狗任务 (Core 0, 100ms 周期)
 */
static void task_remote_watchdog(void *arg) {
    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(WATCHDOG_PERIOD_MS);
    
    ESP_LOGI(TAG, "[task_remote_watchdog] Started on Core %d", xPortGetCoreID());
    
    while (1) {
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
    
    xSemaphoreTake(g_remote_mutex, portMAX_DELAY);
    memcpy(&remote, &g_remote_data, sizeof(remote));
    xSemaphoreGive(g_remote_mutex);
    
    xSemaphoreTake(g_wheel_state_mutex, portMAX_DELAY);
    memcpy(&wheel, &g_wheel_state, sizeof(wheel));
    xSemaphoreGive(g_wheel_state_mutex);
    
    // ======== 检查紧急停止 ========
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
    
    // ======== 计算 LQR 状态量 ========
    // 位置: 使用电机位置 (度 -> 弧度)
    // 注: 电机返回的是累积角度，不会有过零问题
    float left_pos_rad = wheel.left_position * 0.0174533f;   // deg to rad
    float right_pos_rad = wheel.right_position * 0.0174533f;
    
    // 速度: rpm -> rad/s
    float left_vel_rad = wheel.left_speed * 0.10472f;   // rpm to rad/s
    float right_vel_rad = wheel.right_speed * 0.10472f;
    
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
    lqr_input_t input = {
        .pitch = imu.pitch,
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
        .target_yaw_rate = remote.joy_x * 0.02f,  // joy_x -> target yaw rate
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
    
    // ======== 计算 LQR 输出 ========
    lqr_output_t output = {0};
    
    if (!remote.go || g_uncontrolable != 0) {
        // 未使能或失控，切换到简单平衡模式 (仅角度+角速度环)
        lqr_set_simple_balance_mode(&g_lqr_ctrl);
    } else {
        // 正常控制，使用完整平衡模式
        lqr_set_full_balance_mode(&g_lqr_ctrl);
    }
    
    // 统一调用 LQR 平衡循环
    esp_err_t ret = lqr_balance_loop(&g_lqr_ctrl, &input, &output);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "LQR balance loop failed");
        output.left_wheel_torque = 0;
        output.right_wheel_torque = 0;
        g_last_lqr_u = 0;
    } else {
        // ======== YAW 轴转向控制 (仅在正常模式下) ========
        if (remote.go && g_uncontrolable == 0) {
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
            // 简单模式下，直接使用 LQR 输出，无 YAW 控制
            float lqr_u = output.lqr_u;
            output.left_wheel_torque = lqr_u;
            output.right_wheel_torque = lqr_u;
            g_yaw_output = 0.0f;
        }
        g_last_lqr_u = output.lqr_u;  // 保存用于波形显示
    }
    
    // ======== 输出波形数据 (用于 Qt 调参面板) ========
    output_plot_data(&input, &output);
    
    // ======== 更新轮命令 ========
    xSemaphoreTake(g_wheel_cmd_mutex, portMAX_DELAY);
    g_wheel_cmd.left_torque = output.left_wheel_torque;
    g_wheel_cmd.right_torque = output.right_wheel_torque;
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
        can_motor_set_torque(g_motor_left, cmd.left_torque);
        can_motor_set_torque(g_motor_right, cmd.right_torque);
    } else {
        can_motor_set_torque(g_motor_left, 0);
        can_motor_set_torque(g_motor_right, 0);
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
        printf("Usage: balance [init|start|stop|enable|disable|estop|reset|status|zero <deg>]\n");
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
    // ===== 腿部电机控制命令 =====
    else if (strcmp(token, "leg") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token == NULL) {
            printf("Leg control: %s\n", g_leg_control_enabled ? "enabled" : "disabled");
            printf("Leg angles: L_Hip=%.1f, L_Knee=%.1f, R_Hip=%.1f, R_Knee=%.1f\n",
                   g_leg_left_hip_angle, g_leg_left_knee_angle,
                   g_leg_right_hip_angle, g_leg_right_knee_angle);
            printf("Leg speed: %.1f rpm\n", g_leg_move_speed);
            printf("Usage: balance leg [on|off|set <lh> <lk> <rh> <rk>|speed <rpm>]\n");
        } else if (strcmp(token, "on") == 0) {
            balance_test_set_leg_control(true);
            printf("Leg motors enabled\n");
        } else if (strcmp(token, "off") == 0) {
            balance_test_set_leg_control(false);
            printf("Leg motors disabled\n");
        } else if (strcmp(token, "set") == 0) {
            // balance leg set <left_hip> <left_knee> <right_hip> <right_knee>
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
            printf("Leg angles set: L_Hip=%.1f, L_Knee=%.1f, R_Hip=%.1f, R_Knee=%.1f\n",
                   lh, lk, rh, rk);
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
        } else {
            printf("Unknown leg command: %s\n", token);
            printf("Usage: balance leg [on|off|set <lh> <lk> <rh> <rk>|speed <rpm>]\n");
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
    else {
        printf("Unknown command: %s\n", token);
        printf("Usage: balance [init|start|stop|enable|disable|estop|reset|status|zero|plot|leg|roll|mzero]\n");
    }
}
