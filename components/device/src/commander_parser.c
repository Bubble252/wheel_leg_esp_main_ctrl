/**
 * @file commander_parser.c
 * @brief Commander命令解析器实现 - 与pid_tuner.py配合使用
 * 
 * 设计理念：
 * - 参数设置时，通过 set_callback 通知 LQR 控制器更新参数
 * - 参数查询时，通过 query_callback 从 LQR 控制器读取真实参数
 * - Commander 不存储参数副本，避免同步问题
 */

#include "commander_parser.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

#define TAG "COMMANDER"

/* ==================== 静态变量 ==================== */

// 参数设置回调 - 当收到设置命令时调用
static commander_callback_t s_set_callback = NULL;

// 参数查询回调 - 当收到查询命令时调用
static commander_query_callback_t s_query_callback = NULL;

// 接收缓冲区
#define RECV_BUFFER_SIZE 256
static char s_recv_buffer[RECV_BUFFER_SIZE];
static size_t s_recv_len = 0;

/* ==================== 内部函数 ==================== */

/**
 * @brief 获取控制器名称
 */
static const char* get_controller_name(char controller_id)
{
    switch (controller_id) {
        case CTRL_ID_ANGLE:      return "Angle";
        case CTRL_ID_GYRO:       return "Gyro";
        case CTRL_ID_DISTANCE:   return "Distance";
        case CTRL_ID_SPEED:      return "Speed";
        case CTRL_ID_YAW_ANGLE:  return "YawAngle";
        case CTRL_ID_YAW_GYRO:   return "YawGyro";
        case CTRL_ID_JOYY_LPF:   return "JoyyLPF";
        case CTRL_ID_LQR_U:      return "LqrU";
        case CTRL_ID_ZEROPOINT:  return "Zeropoint";
        case CTRL_ID_ZERO_LPF:   return "ZeropointLPF";
        case CTRL_ID_ROLL_ANGLE: return "RollAngle";
        case CTRL_ID_ROLL_LPF:   return "RollLPF";
        case CTRL_ID_SPEED_ADAPT:return "SpeedAdapt";
        default: return "Unknown";
    }
}

/**
 * @brief 处理PID参数命令
 */
static void handle_pid_command(char controller_id, char param, float value)
{
    const char *name = get_controller_name(controller_id);
    
    if (param == PARAM_QUERY) {
        // 查询 - 通过回调从 LQR 读取真实参数
        if (s_query_callback) {
            commander_pid_params_t params;
            if (s_query_callback(controller_id, &params)) {
                // 格式与 pid_tuner.py 正则表达式匹配
                printf("PID: P: %.4f I: %.4f D: %.4f R: %.4f L: %.4f\n",
                       params.p, params.i, params.d, params.ramp, params.limit);
            } else {
                printf("[%s] Query callback failed for %s\n", TAG, name);
            }
        } else {
            printf("[%s] No query callback registered for %s\n", TAG, name);
        }
        return;
    }
    
    // 设置命令 - 打印日志，然后通过回调通知 LQR
    switch (param) {
        case PARAM_PID_P:
            printf("[%s] %s.P = %.4f\n", TAG, name, value);
            break;
        case PARAM_PID_I:
            printf("[%s] %s.I = %.4f\n", TAG, name, value);
            break;
        case PARAM_PID_D:
            printf("[%s] %s.D = %.4f\n", TAG, name, value);
            break;
        case PARAM_PID_LIMIT:
            printf("[%s] %s.Limit = %.4f\n", TAG, name, value);
            break;
        case PARAM_PID_RAMP:
            printf("[%s] %s.Ramp = %.4f\n", TAG, name, value);
            break;
        case PARAM_PID_LPF_TF:
            printf("[%s] %s.LPF_Tf = %.4f\n", TAG, name, value);
            break;
        default:
            printf("[%s] Unknown param for %s: %c\n", TAG, name, param);
            return;
    }
    
    // 调用设置回调
    if (s_set_callback) {
        s_set_callback(controller_id, param, value);
    }
}

/**
 * @brief 处理LPF参数命令
 */
static void handle_lpf_command(char controller_id, char param, float value)
{
    const char *name = get_controller_name(controller_id);
    
    if (param == PARAM_QUERY) {
        // 查询 - 通过回调从 LQR 读取真实参数
        if (s_query_callback) {
            commander_pid_params_t params;  // 复用结构，只用 lpf_tf 字段
            if (s_query_callback(controller_id, &params)) {
                // 格式与 pid_tuner.py 正则表达式匹配
                printf("LPF: Tf: %.4f\n", params.lpf_tf);
            } else {
                printf("[%s] Query callback failed for %s\n", TAG, name);
            }
        } else {
            printf("[%s] No query callback registered for %s\n", TAG, name);
        }
        return;
    }
    
    // 设置命令
    if (param == PARAM_LPF_TF) {
        printf("[%s] %s.Tf = %.4f\n", TAG, name, value);
        if (s_set_callback) {
            s_set_callback(controller_id, param, value);
        }
    } else {
        printf("[%s] Unknown param for %s: %c\n", TAG, name, param);
    }
}

/**
 * @brief 处理速度自适应参数命令
 */
static void handle_speed_adaptive_command(char param, float value)
{
    if (param == PARAM_QUERY) {
        // 查询 - 通过回调从 LQR 读取真实参数
        if (s_query_callback) {
            commander_pid_params_t params;  // 复用结构，用 p 和 i 存储 kp_min/max
            if (s_query_callback(CTRL_ID_SPEED_ADAPT, &params)) {
                // 格式与 pid_tuner.py 正则表达式匹配
                printf("Speed Adaptive P: Low=%.4f High=%.4f\n", params.p, params.i);
            } else {
                printf("[%s] Query callback failed for SpeedAdapt\n", TAG);
            }
        } else {
            printf("[%s] No query callback registered for SpeedAdapt\n", TAG);
        }
        return;
    }
    
    // 设置命令
    switch (param) {
        case PARAM_SPEED_KP_MIN:
            printf("[%s] SpeedAdapt.Kp_Min = %.4f (高姿态)\n", TAG, value);
            break;
        case PARAM_SPEED_KP_MAX:
            printf("[%s] SpeedAdapt.Kp_Max = %.4f (低姿态)\n", TAG, value);
            break;
        default:
            printf("[%s] Unknown SpeedAdapt param: %c\n", TAG, param);
            return;
    }
    
    if (s_set_callback) {
        s_set_callback(CTRL_ID_SPEED_ADAPT, param, value);
    }
}

/**
 * @brief 处理角速度滤波命令 (双模式: LPF / 限幅)
 * 协议:
 *   N? = 查询 → "GyroFilter: Mode=1 Tf=0.0050 Rate=500.0000"
 *   NT<value> = 设置 LPF Tf
 *   NM<value> = 设置模式 (0=LPF, 1=限幅)
 *   NR<value> = 设置限幅最大变化率
 */
static void handle_gyro_filter_command(char param, float value)
{
    if (param == PARAM_QUERY) {
        if (s_query_callback) {
            commander_pid_params_t params;
            if (s_query_callback(CTRL_ID_GYRO_LPF, &params)) {
                // params.lpf_tf = Tf, params.p = mode, params.i = slew_rate
                printf("GyroFilter: Mode=%d Tf=%.4f Rate=%.4f\n",
                       (int)params.p, params.lpf_tf, params.i);
            } else {
                printf("[%s] Query callback failed for GyroFilter\n", TAG);
            }
        } else {
            printf("[%s] No query callback registered for GyroFilter\n", TAG);
        }
        return;
    }

    switch (param) {
        case 'T':
            printf("[%s] GyroFilter.Tf = %.4f\n", TAG, value);
            break;
        case 'M':
            printf("[%s] GyroFilter.Mode = %d (%s)\n", TAG, (int)value,
                   (int)value == 0 ? "LPF" : "SlewRate");
            break;
        case 'R':
            printf("[%s] GyroFilter.Rate = %.4f\n", TAG, value);
            break;
        default:
            printf("[%s] Unknown GyroFilter param: %c\n", TAG, param);
            return;
    }

    if (s_set_callback) {
        s_set_callback(CTRL_ID_GYRO_LPF, param, value);
    }
}

/**
 * @brief 处理轮速滤波命令 (双模式: LPF / 限幅)
 * 协议:
 *   W? = 查询 → "SpeedFilter: Mode=0 Tf=0.0100 Rate=50.0000"
 *   WT<value> = 设置 LPF Tf
 *   WM<value> = 设置模式 (0=LPF, 1=限幅)
 *   WR<value> = 设置限幅最大变化率
 */
static void handle_speed_filter_command(char param, float value)
{
    if (param == PARAM_QUERY) {
        if (s_query_callback) {
            commander_pid_params_t params;
            if (s_query_callback(CTRL_ID_SPEED_LPF, &params)) {
                // params.lpf_tf = Tf, params.p = mode, params.i = slew_rate
                printf("SpeedFilter: Mode=%d Tf=%.4f Rate=%.4f\n",
                       (int)params.p, params.lpf_tf, params.i);
            } else {
                printf("[%s] Query callback failed for SpeedFilter\n", TAG);
            }
        } else {
            printf("[%s] No query callback registered for SpeedFilter\n", TAG);
        }
        return;
    }

    switch (param) {
        case 'T':
            printf("[%s] SpeedFilter.Tf = %.4f\n", TAG, value);
            break;
        case 'M':
            printf("[%s] SpeedFilter.Mode = %d (%s)\n", TAG, (int)value,
                   (int)value == 0 ? "LPF" : "SlewRate");
            break;
        case 'R':
            printf("[%s] SpeedFilter.Rate = %.4f\n", TAG, value);
            break;
        default:
            printf("[%s] Unknown SpeedFilter param: %c\n", TAG, param);
            return;
    }

    if (s_set_callback) {
        s_set_callback(CTRL_ID_SPEED_LPF, param, value);
    }
}

/**
 * @brief 应用解析后的命令
 */
static void apply_command(const commander_cmd_t *cmd)
{
    // 根据控制器类型分发
    switch (cmd->controller_id) {
        // PID控制器
        case CTRL_ID_ANGLE:
        case CTRL_ID_GYRO:
        case CTRL_ID_DISTANCE:
        case CTRL_ID_SPEED:
        case CTRL_ID_YAW_ANGLE:
        case CTRL_ID_YAW_GYRO:
        case CTRL_ID_LQR_U:
        case CTRL_ID_ZEROPOINT:
        case CTRL_ID_ROLL_ANGLE:
            handle_pid_command(cmd->controller_id, cmd->param_char, cmd->value);
            break;
            
        // LPF控制器
        case CTRL_ID_JOYY_LPF:
        case CTRL_ID_ZERO_LPF:
        case CTRL_ID_ROLL_LPF:
            handle_lpf_command(cmd->controller_id, cmd->param_char, cmd->value);
            break;
            
        // 角速度滤波 (双模式: LPF / 限幅)
        case CTRL_ID_GYRO_LPF:
            handle_gyro_filter_command(cmd->param_char, cmd->value);
            break;
            
        // 轮速滤波 (双模式: LPF / 限幅)
        case CTRL_ID_SPEED_LPF:
            handle_speed_filter_command(cmd->param_char, cmd->value);
            break;
            
        // 速度自适应
        case CTRL_ID_SPEED_ADAPT:
            handle_speed_adaptive_command(cmd->param_char, cmd->value);
            break;
            
        default:
            printf("[%s] Unknown controller: %c\n", TAG, cmd->controller_id);
            break;
    }
}

/* ==================== API实现 ==================== */

int commander_parser_init(commander_callback_t set_callback, 
                          commander_query_callback_t query_callback)
{
    s_set_callback = set_callback;
    s_query_callback = query_callback;
    s_recv_len = 0;
    memset(s_recv_buffer, 0, sizeof(s_recv_buffer));
    
    printf("[%s] Commander parser initialized\n", TAG);
    printf("[%s] Supported controllers: A-F (PID), G/J/L (LPF), H/I/K (PID), M (SpeedAdapt), N (GyroFilter), W (SpeedFilter)\n", TAG);
    return 0;
}

bool commander_parse(const char *cmd, commander_cmd_t *result)
{
    if (!cmd || !result || strlen(cmd) < 2) {
        result->valid = false;
        return false;
    }
    
    // 跳过前导空白
    while (*cmd && isspace((unsigned char)*cmd)) {
        cmd++;
    }
    
    if (strlen(cmd) < 2) {
        result->valid = false;
        return false;
    }
    
    // 第一个字符是控制器ID
    result->controller_id = cmd[0];
    
    // 第二个字符是参数字符
    result->param_char = cmd[1];
    
    // 如果是查询命令，不需要数值
    if (result->param_char == PARAM_QUERY) {
        result->value = 0.0f;
        result->valid = true;
        return true;
    }
    
    // 后面是数值
    if (strlen(cmd) < 3) {
        result->valid = false;
        return false;
    }
    
    char *endptr = NULL;
    result->value = strtof(cmd + 2, &endptr);
    
    // 检查是否成功解析数值
    if (endptr == cmd + 2) {
        result->valid = false;
        return false;
    }
    
    result->valid = true;
    return true;
}

bool commander_process_line(const char *line)
{
    commander_cmd_t cmd;
    
    if (commander_parse(line, &cmd)) {
        apply_command(&cmd);
        return true;
    }
    
    return false;
}

int commander_process_data(const char *data, size_t len)
{
    int cmd_count = 0;
    
    for (size_t i = 0; i < len; i++) {
        char c = data[i];
        
        if (c == '\n' || c == '\r') {
            // 行结束，处理缓冲区
            if (s_recv_len > 0) {
                s_recv_buffer[s_recv_len] = '\0';
                
                if (commander_process_line(s_recv_buffer)) {
                    cmd_count++;
                }
                
                s_recv_len = 0;
            }
        } else {
            // 添加到缓冲区
            if (s_recv_len < RECV_BUFFER_SIZE - 1) {
                s_recv_buffer[s_recv_len++] = c;
            } else {
                // 缓冲区满，丢弃
                s_recv_len = 0;
            }
        }
    }
    
    return cmd_count;
}

int commander_send_data(const char *id, float target, float control,
                        int (*send_func)(const char *data, size_t len))
{
    if (!send_func || !id) {
        return -1;
    }
    
    char buffer[64];
    int len = snprintf(buffer, sizeof(buffer), "#DATA,%s,%.4f,%.4f\n", id, target, control);
    
    if (len > 0 && len < (int)sizeof(buffer)) {
        return send_func(buffer, len);
    }
    
    return -1;
}

int commander_send_web_status(int go, int dir, float joyx, float joyy, float height,
                              int (*send_func)(const char *data, size_t len))
{
    if (!send_func) {
        return -1;
    }
    
    char buffer[64];
    int len = snprintf(buffer, sizeof(buffer), "#WEB,%d,%d,%.2f,%.2f,%.2f\n", 
                       go, dir, joyx, joyy, height);
    
    if (len > 0 && len < (int)sizeof(buffer)) {
        return send_func(buffer, len);
    }
    
    return -1;
}

void commander_set_callback(commander_callback_t callback)
{
    s_set_callback = callback;
}

void commander_print_params(void)
{
    printf("\n========== Commander Parameters ==========\n");
    printf("Note: Use query command (e.g., 'A?') to get actual parameters\n");
    
    if (s_query_callback) {
        commander_pid_params_t params;
        
        printf("\n--- PID Controllers ---\n");
        
        const char* pid_ids[] = {"A-Angle", "B-Gyro", "C-Distance", "D-Speed", 
                                  "E-YawAngle", "F-YawGyro", "H-LqrU", 
                                  "I-Zeropoint", "K-RollAngle"};
        const char pid_chars[] = {CTRL_ID_ANGLE, CTRL_ID_GYRO, CTRL_ID_DISTANCE, 
                                   CTRL_ID_SPEED, CTRL_ID_YAW_ANGLE, CTRL_ID_YAW_GYRO,
                                   CTRL_ID_LQR_U, CTRL_ID_ZEROPOINT, CTRL_ID_ROLL_ANGLE};
        
        for (int i = 0; i < 9; i++) {
            if (s_query_callback(pid_chars[i], &params)) {
                printf("[%s] P=%.4f I=%.4f D=%.4f L=%.4f R=%.4f\n",
                       pid_ids[i], params.p, params.i, params.d, 
                       params.limit, params.ramp);
            }
        }
        
        printf("\n--- LPF Controllers ---\n");
        if (s_query_callback(CTRL_ID_JOYY_LPF, &params)) {
            printf("[G-JoyyLPF] Tf=%.4f\n", params.lpf_tf);
        }
        if (s_query_callback(CTRL_ID_ZERO_LPF, &params)) {
            printf("[J-ZeropointLPF] Tf=%.4f\n", params.lpf_tf);
        }
        if (s_query_callback(CTRL_ID_ROLL_LPF, &params)) {
            printf("[L-RollLPF] Tf=%.4f\n", params.lpf_tf);
        }
        
        printf("\n--- Gyro Filter ---\n");
        if (s_query_callback(CTRL_ID_GYRO_LPF, &params)) {
            printf("[N-GyroFilter] Mode=%d Tf=%.4f Rate=%.4f\n",
                   (int)params.p, params.lpf_tf, params.i);
        }
        
        printf("\n--- Speed Filter ---\n");
        if (s_query_callback(CTRL_ID_SPEED_LPF, &params)) {
            printf("[W-SpeedFilter] Mode=%d Tf=%.4f Rate=%.4f\n",
                   (int)params.p, params.lpf_tf, params.i);
        }
        
        printf("\n--- Speed Adaptive ---\n");
        if (s_query_callback(CTRL_ID_SPEED_ADAPT, &params)) {
            printf("[M-SpeedAdapt] Kp_Min=%.4f (高姿态) Kp_Max=%.4f (低姿态)\n",
                   params.p, params.i);
        }
    } else {
        printf("No query callback registered - cannot retrieve parameters\n");
    }
    
    printf("\n==========================================\n\n");
}
