/**
 * @file pi_protocol.h
 * @brief 树莓派通信协议定义
 * @author Bubble
 * @date 2026-01-27
 * 
 * 协议特性:
 * - 帧格式: [0xAA 0x55] [LEN] [SEQ] [CMD] [DATA...] [CRC16]
 * - 字节序: 大端序 (Big-Endian)
 * - CRC: CRC16-CCITT (0x1021, 初值 0xFFFF)
 */

#ifndef PI_PROTOCOL_H
#define PI_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 帧格式常量
// ============================================================================

#define PI_FRAME_HEADER_0       0xAA
#define PI_FRAME_HEADER_1       0x55
#define PI_FRAME_HEADER_SIZE    2
#define PI_FRAME_MIN_SIZE       7       // HEAD(2) + LEN(1) + SEQ(1) + CMD(1) + CRC(2)
#define PI_FRAME_MAX_SIZE       257     // 最大帧长度
#define PI_FRAME_MAX_DATA_SIZE  250     // 最大数据长度

// ============================================================================
// 命令码定义
// ============================================================================

// 系统命令 (0x00-0x1F)
#define CMD_HEARTBEAT           0x01    // 心跳包
#define CMD_HEARTBEAT_ACK       0x02    // 心跳响应
#define CMD_HANDSHAKE           0x03    // 握手请求
#define CMD_HANDSHAKE_ACK       0x04    // 握手响应
#define CMD_GET_VERSION         0x05    // 获取版本
#define CMD_VERSION_REPORT      0x06    // 版本上报
#define CMD_RESET               0x07    // 软复位
#define CMD_ENTER_DEBUG         0x08    // 进入调试模式
#define CMD_EXIT_DEBUG          0x09    // 退出调试模式
#define CMD_SET_LOG_LEVEL       0x0A    // 设置日志等级

// 控制命令 (0x20-0x3F)
#define CMD_SET_VELOCITY        0x20    // 设置速度
#define CMD_SET_MODE            0x21    // 设置模式
#define CMD_SET_HEIGHT          0x22    // 设置高度
#define CMD_EMERGENCY_STOP      0x23    // 紧急停止
#define CMD_MOTOR_ENABLE        0x24    // 电机使能
#define CMD_SET_PITCH           0x25    // 设置 Pitch
#define CMD_SET_ROLL            0x26    // 设置 Roll
#define CMD_SET_POSE            0x27    // 设置完整姿态
#define CMD_JUMP                0x28    // 跳跃 (预留)

// 查询命令 (0x40-0x5F)
#define CMD_GET_STATUS          0x40    // 获取状态
#define CMD_GET_IMU             0x41    // 获取 IMU
#define CMD_GET_MOTOR           0x42    // 获取电机
#define CMD_GET_BATTERY         0x43    // 获取电池
#define CMD_GET_ERROR           0x44    // 获取错误

// 上报命令 (0x60-0x7F)
#define CMD_STATUS_REPORT       0x60    // 状态上报
#define CMD_IMU_REPORT          0x61    // IMU 上报
#define CMD_MOTOR_REPORT        0x62    // 电机上报
#define CMD_ERROR_REPORT        0x63    // 错误上报
#define CMD_EVENT_REPORT        0x64    // 事件上报

// 参数配置 (0x80-0x9F)
#define CMD_SET_PID             0x80    // 设置 PID
#define CMD_GET_PID             0x81    // 获取 PID
#define CMD_SET_LQR             0x82    // 设置 LQR
#define CMD_GET_LQR             0x83    // 获取 LQR
#define CMD_SAVE_CONFIG         0x84    // 保存配置
#define CMD_LOAD_CONFIG         0x85    // 加载配置
#define CMD_RESET_CONFIG        0x86    // 重置配置

// 通用响应 (0xF0-0xFF)
#define CMD_ACK                 0xF0    // 确认
#define CMD_NACK                0xF1    // 拒绝

// ============================================================================
// 运动模式定义
// ============================================================================

typedef enum {
    MODE_IDLE       = 0,    // 空闲 (电机失能)
    MODE_STAND      = 1,    // 站立 (平衡使能)
    MODE_WALK       = 2,    // 行走 (接受速度指令)
    MODE_CROUCH     = 3,    // 蹲下
    MODE_JUMP       = 4,    // 跳跃 (预留)
    MODE_RECOVERY   = 5,    // 恢复/自起 (预留)
} motion_mode_t;

// ============================================================================
// 系统状态定义
// ============================================================================

typedef enum {
    STATUS_BOOT         = 0,    // 启动中
    STATUS_IDLE         = 1,    // 空闲
    STATUS_RUNNING      = 2,    // 运行中
    STATUS_ERROR        = 3,    // 错误
    STATUS_ESTOP        = 4,    // 急停
    STATUS_RECOVERING   = 5,    // 恢复中
} system_status_t;

// 状态标志位
#define FLAG_MOTOR_ENABLED      0x01
#define FLAG_BALANCE_ACTIVE     0x02
#define FLAG_YAW_HOLDING        0x04
#define FLAG_PI_CONNECTED       0x08
#define FLAG_LOW_BATTERY        0x10
#define FLAG_OVER_TILT          0x20

// ============================================================================
// 错误码定义
// ============================================================================

#define ERR_NONE            0x00
#define ERR_IMU_FAIL        0x01
#define ERR_MOTOR_FAIL      0x02
#define ERR_OVER_TILT       0x03
#define ERR_LOW_BATTERY     0x04
#define ERR_COMM_TIMEOUT    0x05
#define ERR_MOTOR_OVERLOAD  0x06
#define ERR_SENSOR_FAIL     0x07

// NACK 错误码
#define NACK_UNKNOWN_CMD    0x01
#define NACK_INVALID_PARAM  0x02
#define NACK_WRONG_STATE    0x03
#define NACK_BUSY           0x04
#define NACK_CRC_ERROR      0x05

// ============================================================================
// 数据结构定义 (所有多字节字段使用大端序)
// ============================================================================

// 心跳请求
typedef struct __attribute__((packed)) {
    uint32_t timestamp;         // 发送时间戳 (ms)
} heartbeat_req_t;              // 4 bytes

// 心跳响应
typedef struct __attribute__((packed)) {
    uint32_t timestamp;         // 回显时间戳
    uint32_t esp_time;          // ESP32 时间
    uint8_t  cpu_load;          // CPU 负载 (0-100)
    uint8_t  status;            // 系统状态
} heartbeat_ack_t;              // 10 bytes

// 握手请求
typedef struct __attribute__((packed)) {
    uint8_t  protocol_ver;      // 协议版本
    uint32_t capabilities;      // 能力位图
} handshake_req_t;              // 5 bytes

// 握手响应
typedef struct __attribute__((packed)) {
    uint8_t  result;            // 结果 (0=成功)
    uint8_t  protocol_ver;      // 协议版本
    char     version[16];       // 固件版本
    uint32_t capabilities;      // 能力位图
} handshake_ack_t;              // 22 bytes

// 速度指令
typedef struct __attribute__((packed)) {
    float    vx;                // 前进速度 (m/s)
    float    yaw_rate;          // 偏航角速度 (rad/s)
    uint8_t  priority;          // 优先级
    uint8_t  flags;             // 标志位
} velocity_cmd_t;               // 10 bytes

#define VEL_FLAG_SMOOTH     0x01
#define VEL_FLAG_OVERRIDE   0x02

// 模式指令
typedef struct __attribute__((packed)) {
    uint8_t  mode;              // 运动模式
    uint8_t  sub_mode;          // 子模式
} mode_cmd_t;                   // 2 bytes

// 高度指令
typedef struct __attribute__((packed)) {
    float    height;            // 高度 (m)
    float    duration;          // 过渡时间 (s)
} height_cmd_t;                 // 8 bytes

// Pitch 指令
typedef struct __attribute__((packed)) {
    float    pitch;             // Pitch 角度 (度)
    float    duration;          // 过渡时间 (s)
} pitch_cmd_t;                  // 8 bytes

// Roll 指令
typedef struct __attribute__((packed)) {
    float    roll;              // Roll 角度 (度)
    float    duration;          // 过渡时间 (s)
} roll_cmd_t;                   // 8 bytes

// 完整姿态指令
typedef struct __attribute__((packed)) {
    float    pitch;             // Pitch 角度 (度)
    float    roll;              // Roll 角度 (度)
    float    height;            // 高度 (m)
    float    duration;          // 过渡时间 (s)
} pose_cmd_t;                   // 16 bytes

// 电机使能指令
typedef struct __attribute__((packed)) {
    uint8_t  enable;            // 0=失能, 1=使能
} motor_enable_cmd_t;           // 1 byte

// 紧急停止
typedef struct __attribute__((packed)) {
    uint8_t  reason;            // 停止原因
} estop_cmd_t;                  // 1 byte

// 综合状态上报
typedef struct __attribute__((packed)) {
    uint32_t timestamp;         // ESP32 时间 (ms)
    uint8_t  mode;              // 当前模式
    uint8_t  status;            // 系统状态
    uint8_t  error_code;        // 错误码
    uint8_t  flags;             // 状态标志
    float    pitch;             // 俯仰角 (度)
    float    roll;              // 横滚角 (度)
    float    yaw;               // 偏航角 (度)
    float    vx_actual;         // 实际前进速度 (m/s)
    float    yaw_rate_actual;   // 实际偏航角速度 (rad/s)
    float    battery_voltage;   // 电池电压 (V)
    float    height_actual;     // 实际高度 (m)
} status_report_t;              // 40 bytes

// IMU 上报
typedef struct __attribute__((packed)) {
    uint32_t timestamp;         // 时间戳
    float    pitch;             // 俯仰角 (度)
    float    roll;              // 横滚角 (度)
    float    yaw;               // 偏航角 (度)
    float    pitch_rate;        // 俯仰角速度 (度/s)
    float    roll_rate;         // 横滚角速度 (度/s)
    float    yaw_rate;          // 偏航角速度 (度/s)
    float    accel_x;           // X 加速度 (g)
    float    accel_y;           // Y 加速度 (g)
    float    accel_z;           // Z 加速度 (g)
} imu_report_t;                 // 40 bytes

// 错误上报
typedef struct __attribute__((packed)) {
    uint32_t timestamp;         // 时间戳
    uint8_t  error_code;        // 错误码
    uint8_t  severity;          // 严重程度
    uint8_t  source;            // 错误来源
    char     message[32];       // 错误描述
} error_report_t;               // 39 bytes

// ACK
typedef struct __attribute__((packed)) {
    uint8_t  seq;               // 请求序列号
    uint8_t  cmd;               // 命令码
} ack_t;                        // 2 bytes

// NACK
typedef struct __attribute__((packed)) {
    uint8_t  seq;               // 请求序列号
    uint8_t  cmd;               // 命令码
    uint8_t  error_code;        // 错误码
} nack_t;                       // 3 bytes

// ============================================================================
// 字节序转换宏 (ESP32 是小端序，协议使用大端序)
// ============================================================================

#define HOST_TO_BE16(x)  __builtin_bswap16(x)
#define HOST_TO_BE32(x)  __builtin_bswap32(x)
#define BE_TO_HOST16(x)  __builtin_bswap16(x)
#define BE_TO_HOST32(x)  __builtin_bswap32(x)

// 浮点数转换 (输入是已转换为 host 字节序的 uint32)
static inline float be_to_float(uint32_t host_val) {
    float result;
    __builtin_memcpy(&result, &host_val, sizeof(float));
    return result;
}

// float 转换为 uint32 (host 字节序, 供 write_be32 使用)
static inline uint32_t float_to_be(float f) {
    uint32_t host_val;
    __builtin_memcpy(&host_val, &f, sizeof(float));
    return host_val;  // write_be32 会处理字节序
}

// 从大端序缓冲区读取
static inline uint16_t read_be16(const uint8_t *buf) {
    return ((uint16_t)buf[0] << 8) | buf[1];
}

static inline uint32_t read_be32(const uint8_t *buf) {
    return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
           ((uint32_t)buf[2] << 8) | buf[3];
}

static inline float read_be_float(const uint8_t *buf) {
    return be_to_float(read_be32(buf));
}

// 向大端序缓冲区写入
static inline void write_be16(uint8_t *buf, uint16_t val) {
    buf[0] = (val >> 8) & 0xFF;
    buf[1] = val & 0xFF;
}

static inline void write_be32(uint8_t *buf, uint32_t val) {
    buf[0] = (val >> 24) & 0xFF;
    buf[1] = (val >> 16) & 0xFF;
    buf[2] = (val >> 8) & 0xFF;
    buf[3] = val & 0xFF;
}

static inline void write_be_float(uint8_t *buf, float val) {
    write_be32(buf, float_to_be(val));
}

#ifdef __cplusplus
}
#endif

#endif // PI_PROTOCOL_H
