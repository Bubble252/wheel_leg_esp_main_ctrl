/**
 * @file commander_parser.h
 * @brief Commander命令解析器 - 用于PID调参面板通信
 * 
 * 协议格式: <控制器ID><参数字符><数值>
 * 例如: AP1.5 = 设置角度控制器的P值为1.5
 *       ET0.01 = 设置YAW角度LPF的Tf为0.01
 *       ML0.7 = 设置速度自适应低姿态P为0.7
 * 
 * 控制器ID (与pid_tuner.py对应):
 *   A - 角度控制 (Angle)
 *   B - 角速度控制 (Gyro)
 *   C - 位移控制 (Distance)
 *   D - 速度控制 (Speed)
 *   E - YAW角度控制
 *   F - YAW角速度控制
 *   G - 摇杆Y轴滤波 (LPF)
 *   H - LQR输出补偿
 *   I - 零点自适应
 *   J - 零点滤波 (LPF)
 *   K - Roll轴平衡
 *   L - Roll角度滤波 (LPF)
 *   M - 速度自适应参数
 */

#ifndef COMMANDER_PARSER_H
#define COMMANDER_PARSER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ==================== 控制器ID定义 ==================== */
/* 与 pid_tuner.py 中的定义一一对应 */

// PID控制器
#define CTRL_ID_ANGLE       'A'     // 角度控制 (Angle PID)
#define CTRL_ID_GYRO        'B'     // 角速度控制 (Gyro PID)
#define CTRL_ID_DISTANCE    'C'     // 位移控制 (Distance PID)
#define CTRL_ID_SPEED       'D'     // 速度控制 (Speed PID)
#define CTRL_ID_YAW_ANGLE   'E'     // YAW角度控制
#define CTRL_ID_YAW_GYRO    'F'     // YAW角速度控制

// LPF控制器
#define CTRL_ID_JOYY_LPF    'G'     // 摇杆Y轴滤波
#define CTRL_ID_LQR_U       'H'     // LQR输出补偿
#define CTRL_ID_ZEROPOINT   'I'     // 零点自适应
#define CTRL_ID_ZERO_LPF    'J'     // 零点滤波
#define CTRL_ID_ROLL_ANGLE  'K'     // Roll轴平衡
#define CTRL_ID_ROLL_LPF    'L'     // Roll角度滤波

// 速度自适应
#define CTRL_ID_SPEED_ADAPT 'M'     // 速度自适应参数

// 反馈信号滤波
#define CTRL_ID_GYRO_LPF    'N'     // 角速度滤波
#define CTRL_ID_SPEED_LPF   'W'     // 轮速滤波

/* ==================== 参数字符定义 ==================== */

// PID参数 (用于A,B,C,D,E,F,H,I,K控制器)
#define PARAM_PID_P         'P'     // P值
#define PARAM_PID_I         'I'     // I值
#define PARAM_PID_D         'D'     // D值
#define PARAM_PID_LIMIT     'L'     // 输出限幅
#define PARAM_PID_RAMP      'R'     // 斜坡限制
#define PARAM_PID_LPF_TF    'T'     // 低通滤波时间常数
#define PARAM_QUERY         '?'     // 查询参数

// LPF参数 (用于G,J,L控制器)
#define PARAM_LPF_TF        'T'     // 滤波时间常数

// 速度自适应参数 (用于M控制器)
#define PARAM_SPEED_KP_MIN  'L'     // Kp_Min (高姿态时)
#define PARAM_SPEED_KP_MAX  'H'     // Kp_Max (低姿态时)

/* ==================== 数据结构定义 ==================== */

/**
 * @brief 解析后的命令结构
 */
typedef struct {
    char controller_id;     // 控制器ID
    char param_char;        // 参数字符
    float value;            // 参数值
    bool valid;             // 是否有效
} commander_cmd_t;

/**
 * @brief PID参数结构 (用于A,B,C,D,E,F,H,I,K控制器)
 */
typedef struct {
    float p;                // P值
    float i;                // I值
    float d;                // D值
    float limit;            // 输出限幅
    float ramp;             // 斜坡限制
    float lpf_tf;           // 低通滤波时间常数
} commander_pid_params_t;

/**
 * @brief LPF参数结构 (用于G,J,L控制器)
 */
typedef struct {
    float tf;               // 滤波时间常数
} commander_lpf_params_t;

/**
 * @brief 速度自适应参数结构 (用于M控制器)
 * @note 对应 lqr_params_t 中的 speed_kp_min 和 speed_kp_max
 */
typedef struct {
    float kp_min;           // 高姿态时的最小P值 (稳定性优先)
    float kp_max;           // 低姿态时的最大P值 (响应速度优先)
} commander_speed_adaptive_t;

/**
 * @brief 参数更新回调函数类型 (设置参数时调用)
 */
typedef void (*commander_callback_t)(char controller_id, char param_char, float value);

/**
 * @brief 参数查询回调函数类型 (查询参数时调用，返回实际参数)
 * @param controller_id 控制器ID
 * @param params 输出参数结构指针
 * @return true=成功获取参数
 */
typedef bool (*commander_query_callback_t)(char controller_id, commander_pid_params_t *params);

/* ==================== API函数 ==================== */

/**
 * @brief 初始化Commander解析器
 * @param set_callback 参数设置回调函数（可以为NULL）
 * @param query_callback 参数查询回调函数（可以为NULL，使用内部存储）
 * @return 0=成功
 */
int commander_parser_init(commander_callback_t set_callback, 
                          commander_query_callback_t query_callback);

/**
 * @brief 解析单个命令字符串
 * @param cmd 命令字符串，如 "AP1.5"
 * @param result 解析结果输出
 * @return true=解析成功
 */
bool commander_parse(const char *cmd, commander_cmd_t *result);

/**
 * @brief 处理接收到的数据（支持缓冲区，处理不完整数据）
 * @param data 接收到的数据
 * @param len 数据长度
 * @return 成功处理的命令数量
 */
int commander_process_data(const char *data, size_t len);

/**
 * @brief 处理单行命令
 * @param line 命令行字符串
 * @return true=处理成功
 */
bool commander_process_line(const char *line);

/**
 * @brief 设置参数更新回调
 * @param callback 回调函数
 * @deprecated 请使用 commander_parser_init 设置回调
 */
void commander_set_callback(commander_callback_t callback);

/**
 * @brief 发送数据流（用于上位机绘图）
 * @param id 数据标识
 * @param target 目标值
 * @param control 控制输出
 * @param send_func 发送函数
 * @return 发送的字节数
 * 
 * 格式: #DATA,<ID>,<Target>,<Control>\n
 */
int commander_send_data(const char *id, float target, float control,
                        int (*send_func)(const char *data, size_t len));

/**
 * @brief 发送Web遥控器状态
 * @param go 前进标志
 * @param dir 方向 (-1/0/1)
 * @param joyx X轴摇杆
 * @param joyy Y轴摇杆
 * @param height 高度
 * @param send_func 发送函数
 * @return 发送的字节数
 * 
 * 格式: #WEB,<go>,<dir>,<joyx>,<joyy>,<height>\n
 */
int commander_send_web_status(int go, int dir, float joyx, float joyy, float height,
                              int (*send_func)(const char *data, size_t len));

/**
 * @brief 打印当前所有参数 (调试用)
 * @note 需要先设置 query_callback 才能打印实际参数
 */
void commander_print_params(void);

#ifdef __cplusplus
}
#endif

#endif // COMMANDER_PARSER_H
