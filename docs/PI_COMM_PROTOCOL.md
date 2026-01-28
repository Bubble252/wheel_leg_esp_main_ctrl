# 树莓派与ESP32串口通信协议设计

> 创建日期: 2026-01-26
> 版本: v1.0 (设计阶段)
> 状态: 📋 设计中

---

## 一、系统架构

### 1.1 整体架构

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                           Raspberry Pi (上位机)                              │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │  视觉处理   │  │  路径规划   │  │  高级决策   │  │  远程通信   │        │
│  └─────────────┘  └─────────────┘  └─────────────┘  └─────────────┘        │
│                              │                                              │
│                              ▼                                              │
│                    ┌─────────────────┐                                      │
│                    │  ESP32Protocol  │  ← Python 通信库                     │
│                    │  (pi_comm.py)   │                                      │
│                    └────────┬────────┘                                      │
└─────────────────────────────┼───────────────────────────────────────────────┘
                              │
                              │ UART / USB-Serial
                              │ 115200 bps, 8N1
                              │ 二进制协议
                              │
┌─────────────────────────────┼───────────────────────────────────────────────┐
│                             ▼                                    ESP32-S3   │
│                    ┌─────────────────┐                                      │
│                    │  pi_comm_task   │  ← FreeRTOS 任务 (Core 0)            │
│                    │  协议解析/心跳  │                                      │
│                    └────────┬────────┘                                      │
│                             │ FreeRTOS Queue                                │
│                             ▼                                               │
│  ┌─────────────┐  ┌─────────────────┐  ┌─────────────┐  ┌─────────────┐    │
│  │  imu_task   │  │  control_task   │  │ motor_task  │  │  wifi_task  │    │
│  │  (Core 1)   │  │   (Core 1)      │  │  (Core 1)   │  │  (Core 0)   │    │
│  │   5ms       │  │    5ms          │  │   2ms       │  │             │    │
│  └─────────────┘  └─────────────────┘  └─────────────┘  └─────────────┘    │
└─────────────────────────────────────────────────────────────────────────────┘
```

### 1.2 设计原则

| 原则 | 说明 |
|------|------|
| **可靠性** | CRC 校验 + ACK 机制 + 重传 |
| **实时性** | 二进制协议，最小化解析延迟 |
| **安全性** | 心跳检测，断连自动停止 |
| **可扩展** | 预留命令码空间，易于添加新功能 |
| **可调试** | 支持切换到文本模式调试 |

---

## 二、帧结构设计

### 2.1 帧格式

```
┌──────┬──────┬──────┬──────┬─────────┬──────┬──────┐
│ HEAD │ HEAD │ LEN  │ SEQ  │ CMD     │ DATA │ CRC  │
│ 0xAA │ 0x55 │ 1B   │ 1B   │ 1B      │ 0-250B│ 2B  │
└──────┴──────┴──────┴──────┴─────────┴──────┴──────┘
  同步头      长度   序列号  命令码   数据载荷  校验
```

| 字段 | 偏移 | 长度 | 说明 |
|------|------|------|------|
| HEAD | 0 | 2B | 帧头 `0xAA 0x55`，用于字节流同步 |
| LEN | 2 | 1B | SEQ + CMD + DATA 的长度 (2-253) |
| SEQ | 3 | 1B | 序列号 (0-255 循环)，用于 ACK 匹配 |
| CMD | 4 | 1B | 命令码 |
| DATA | 5 | 0-250B | 数据载荷 (**大端序**) |
| CRC | 5+N | 2B | CRC16-CCITT (SEQ + CMD + DATA)，**大端序** |

**最小帧**: 7 字节 (无数据)
**最大帧**: 257 字节 (250 字节数据)

### 2.2 CRC 校验

使用 **CRC16-CCITT** (多项式 0x1021，初值 0xFFFF):

```c
uint16_t crc16_ccitt(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    while (len--) {
        crc ^= (*data++) << 8;
        for (int i = 0; i < 8; i++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : crc << 1;
        }
    }
    return crc;
}
```

### 2.3 字节序

- **所有多字节数据**: 大端序 (Big-Endian / 网络字节序)
- **浮点数**: IEEE 754 单精度 (4 字节)，大端序
- **CRC**: 大端序 (高字节在前)

**示例**:
```
float 1.5f = 0x3FC00000
  大端序传输: 3F C0 00 00  ← 高字节在前，人类可读

uint16_t 0x1234
  大端序传输: 12 34        ← 和书写顺序一致

uint32_t 0x12345678
  大端序传输: 12 34 56 78  ← 调试时直观
```

**为什么选择大端序**:
1. ✅ **网络标准**: 与 TCP/IP 网络字节序一致，便于后续扩展
2. ✅ **调试直观**: 串口监视器看到的字节顺序与数值书写顺序一致
3. ✅ **跨平台**: 不依赖 CPU 原生字节序，代码更通用
4. ⚠️ **需要转换**: ESP32/Pi 都是小端序 CPU，需要手动转换

**转换宏** (ESP32 C):
```c
#include <arpa/inet.h>  // 或自定义

// 主机序 → 大端序 (发送前)
#define HOST_TO_BE16(x)  __builtin_bswap16(x)
#define HOST_TO_BE32(x)  __builtin_bswap32(x)

// 大端序 → 主机序 (接收后)
#define BE_TO_HOST16(x)  __builtin_bswap16(x)
#define BE_TO_HOST32(x)  __builtin_bswap32(x)

// 浮点数转换
static inline float be_to_float(uint32_t be_val) {
    uint32_t host_val = BE_TO_HOST32(be_val);
    return *(float *)&host_val;
}

static inline uint32_t float_to_be(float f) {
    uint32_t host_val = *(uint32_t *)&f;
    return HOST_TO_BE32(host_val);
}
```

**转换函数** (Python):
```python
import struct

# Python struct 格式符:
#   '>' = 大端序 (Big-Endian)
#   '<' = 小端序 (Little-Endian)

# 打包 (发送)
data = struct.pack('>ff', vx, yaw_rate)  # 大端序

# 解包 (接收)
vx, yaw_rate = struct.unpack('>ff', data)  # 大端序
```

---

## 三、命令码定义

### 3.1 命令码分配表

```c
// ============================================================================
// 系统命令 (0x00-0x1F) - 连接管理、心跳、版本
// ============================================================================
#define CMD_HEARTBEAT           0x01    // 心跳包 (双向)
#define CMD_HEARTBEAT_ACK       0x02    // 心跳响应
#define CMD_HANDSHAKE           0x03    // 握手请求
#define CMD_HANDSHAKE_ACK       0x04    // 握手响应
#define CMD_GET_VERSION         0x05    // 获取版本信息
#define CMD_VERSION_REPORT      0x06    // 版本信息响应
#define CMD_RESET               0x07    // 软复位 ESP32
#define CMD_ENTER_DEBUG         0x08    // 进入调试模式 (切换到文本协议)
#define CMD_EXIT_DEBUG          0x09    // 退出调试模式

// ============================================================================
// 控制命令 (0x20-0x3F) - 树莓派 → ESP32
// ============================================================================
#define CMD_SET_VELOCITY        0x20    // 设置速度 (vx, yaw_rate)
#define CMD_SET_MODE            0x21    // 设置运动模式
#define CMD_SET_HEIGHT          0x22    // 设置腿部高度
#define CMD_EMERGENCY_STOP      0x23    // 紧急停止 (最高优先级)
#define CMD_MOTOR_ENABLE        0x24    // 电机使能/失能
#define CMD_SET_PITCH           0x25    // 设置目标 Pitch 角度
#define CMD_SET_ROLL            0x26    // 设置目标 Roll 角度
#define CMD_SET_POSE            0x27    // 设置完整姿态 (pitch + roll + height)
#define CMD_JUMP                0x28    // 跳跃命令 (预留)

// ============================================================================
// 查询命令 (0x40-0x5F) - 树莓派 → ESP32 (请求)
// ============================================================================
#define CMD_GET_STATUS          0x40    // 获取综合状态
#define CMD_GET_IMU             0x41    // 获取 IMU 数据
#define CMD_GET_MOTOR           0x42    // 获取电机数据
#define CMD_GET_BATTERY         0x43    // 获取电池电量
#define CMD_GET_ERROR           0x44    // 获取错误信息

// ============================================================================
// 上报命令 (0x60-0x7F) - ESP32 → 树莓派 (主动/响应)
// ============================================================================
#define CMD_STATUS_REPORT       0x60    // 综合状态上报
#define CMD_IMU_REPORT          0x61    // IMU 数据上报
#define CMD_MOTOR_REPORT        0x62    // 电机数据上报
#define CMD_ERROR_REPORT        0x63    // 错误/告警上报
#define CMD_EVENT_REPORT        0x64    // 事件上报 (模式切换等)

// ============================================================================
// 参数配置 (0x80-0x9F) - 调参用
// ============================================================================
#define CMD_SET_PID             0x80    // 设置 PID 参数
#define CMD_GET_PID             0x81    // 获取 PID 参数
#define CMD_SET_LQR             0x82    // 设置 LQR 参数
#define CMD_GET_LQR             0x83    // 获取 LQR 参数
#define CMD_SAVE_CONFIG         0x84    // 保存配置到 Flash
#define CMD_LOAD_CONFIG         0x85    // 从 Flash 加载配置
#define CMD_RESET_CONFIG        0x86    // 恢复默认配置

// ============================================================================
// 通用响应 (0xF0-0xFF)
// ============================================================================
#define CMD_ACK                 0xF0    // 通用确认 (成功)
#define CMD_NACK                0xF1    // 通用拒绝 (失败)
```

### 3.2 命令优先级

| 优先级 | 命令 | 说明 |
|--------|------|------|
| 最高 | `CMD_EMERGENCY_STOP` | 立即停止，跳过队列 |
| 高 | `CMD_SET_VELOCITY`, `CMD_MOTOR_ENABLE` | 实时控制 |
| 中 | `CMD_HEARTBEAT`, `CMD_STATUS_REPORT` | 连接维护 |
| 低 | `CMD_SET_PID`, `CMD_SAVE_CONFIG` | 配置类 |

---

## 四、数据结构定义

### 4.1 系统命令

#### 心跳包 (CMD_HEARTBEAT)

```c
// 树莓派 → ESP32
typedef struct __attribute__((packed)) {
    uint32_t timestamp;     // 发送时间戳 (ms)
} heartbeat_req_t;          // 4 bytes

// ESP32 → 树莓派 (CMD_HEARTBEAT_ACK)
typedef struct __attribute__((packed)) {
    uint32_t timestamp;     // 回显请求时间戳
    uint32_t esp_time;      // ESP32 当前时间 (ms)
    uint8_t  cpu_load;      // CPU 负载 (0-100%)
    uint8_t  status;        // 系统状态 (见 system_status_t)
} heartbeat_ack_t;          // 10 bytes
```

#### 握手请求/响应 (CMD_HANDSHAKE)

```c
// 树莓派 → ESP32
typedef struct __attribute__((packed)) {
    uint8_t  protocol_ver;  // 协议版本 (当前 0x01)
    uint32_t capabilities;  // 能力位图 (预留)
} handshake_req_t;          // 5 bytes

// ESP32 → 树莓派 (CMD_HANDSHAKE_ACK)
typedef struct __attribute__((packed)) {
    uint8_t  result;        // 0=成功, 其他=错误码
    uint8_t  protocol_ver;  // ESP32 支持的协议版本
    char     version[16];   // 固件版本字符串
    uint32_t capabilities;  // ESP32 能力位图
} handshake_ack_t;          // 22 bytes
```

### 4.2 控制命令

#### 速度设置 (CMD_SET_VELOCITY) ⭐ 核心命令

```c
// 树莓派 → ESP32
typedef struct __attribute__((packed)) {
    float    vx;            // 前进速度 (m/s), 正=前进, 负=后退
    float    yaw_rate;      // 偏航角速度 (rad/s), 正=左转, 负=右转
    uint8_t  priority;      // 优先级 (0-255, 255=最高)
    uint8_t  flags;         // 标志位 (预留)
} velocity_cmd_t;           // 10 bytes

// flags 位定义
#define VEL_FLAG_SMOOTH     0x01    // 使用平滑插值
#define VEL_FLAG_OVERRIDE   0x02    // 覆盖当前命令
```

#### 模式设置 (CMD_SET_MODE)

```c
typedef struct __attribute__((packed)) {
    uint8_t  mode;          // 运动模式 (见 motion_mode_t)
    uint8_t  sub_mode;      // 子模式 (预留)
} mode_cmd_t;               // 2 bytes

// 运动模式定义
typedef enum {
    MODE_IDLE       = 0,    // 空闲 (电机失能)
    MODE_STAND      = 1,    // 站立 (平衡使能)
    MODE_WALK       = 2,    // 行走 (接受速度指令)
    MODE_CROUCH     = 3,    // 蹲下
    MODE_JUMP       = 4,    // 跳跃 (预留)
    MODE_RECOVERY   = 5,    // 恢复/自起 (预留)
} motion_mode_t;
```

#### 高度设置 (CMD_SET_HEIGHT)

```c
typedef struct __attribute__((packed)) {
    float    height;        // 腿部高度 (m), 范围 0.02-0.06
    float    duration;      // 过渡时间 (s), 0=立即
} height_cmd_t;             // 8 bytes
```

#### Pitch 角度设置 (CMD_SET_PITCH)

```c
typedef struct __attribute__((packed)) {
    float    pitch;         // 目标 Pitch 角度 (度), 范围 -15 ~ +15
    float    duration;      // 过渡时间 (s), 0=立即
} pitch_cmd_t;              // 8 bytes
```

#### Roll 角度设置 (CMD_SET_ROLL)

```c
typedef struct __attribute__((packed)) {
    float    roll;          // 目标 Roll 角度 (度), 范围 -10 ~ +10
    float    duration;      // 过渡时间 (s), 0=立即
} roll_cmd_t;               // 8 bytes
```

#### 完整姿态设置 (CMD_SET_POSE)

```c
typedef struct __attribute__((packed)) {
    float    pitch;         // 目标 Pitch 角度 (度)
    float    roll;          // 目标 Roll 角度 (度)
    float    height;        // 目标腿部高度 (m)
    float    duration;      // 过渡时间 (s), 0=立即
} pose_cmd_t;               // 16 bytes
```

#### 紧急停止 (CMD_EMERGENCY_STOP)

```c
typedef struct __attribute__((packed)) {
    uint8_t  reason;        // 停止原因 (可选)
} estop_cmd_t;              // 1 byte

// 响应: CMD_ACK (无数据)
```

### 4.3 状态上报

#### 综合状态 (CMD_STATUS_REPORT) ⭐ 主要状态

```c
typedef struct __attribute__((packed)) {
    // 时间戳
    uint32_t timestamp;         // ESP32 时间 (ms)
    
    // 系统状态
    uint8_t  mode;              // 当前运动模式
    uint8_t  status;            // 系统状态 (见下)
    uint8_t  error_code;        // 错误码 (0=正常)
    uint8_t  flags;             // 状态标志
    
    // 姿态
    float    pitch;             // 俯仰角 (度)
    float    roll;              // 横滚角 (度)
    float    yaw;               // 偏航角 (度)
    
    // 速度
    float    vx_actual;         // 实际前进速度 (m/s)
    float    yaw_rate_actual;   // 实际偏航角速度 (rad/s)
    
    // 其他
    float    battery_voltage;   // 电池电压 (V)
    float    height_actual;     // 实际腿部高度 (m)
    
} status_report_t;              // 40 bytes

// 系统状态定义
typedef enum {
    STATUS_BOOT         = 0,    // 启动中
    STATUS_IDLE         = 1,    // 空闲
    STATUS_RUNNING      = 2,    // 运行中
    STATUS_ERROR        = 3,    // 错误
    STATUS_ESTOP        = 4,    // 急停
    STATUS_RECOVERING   = 5,    // 恢复中
} system_status_t;

// 状态标志位
#define FLAG_MOTOR_ENABLED      0x01    // 电机已使能
#define FLAG_BALANCE_ACTIVE     0x02    // 平衡控制激活
#define FLAG_YAW_HOLDING        0x04    // YAW 保持模式
#define FLAG_PI_CONNECTED       0x08    // 树莓派已连接
#define FLAG_LOW_BATTERY        0x10    // 低电量警告
#define FLAG_OVER_TILT          0x20    // 倾角过大
```

#### IMU 数据 (CMD_IMU_REPORT)

```c
typedef struct __attribute__((packed)) {
    uint32_t timestamp;     // 时间戳 (ms)
    float    pitch;         // 俯仰角 (度)
    float    roll;          // 横滚角 (度)
    float    yaw;           // 偏航角 (度)
    float    pitch_rate;    // 俯仰角速度 (度/s)
    float    roll_rate;     // 横滚角速度 (度/s)
    float    yaw_rate;      // 偏航角速度 (度/s)
    float    accel_x;       // X 轴加速度 (g)
    float    accel_y;       // Y 轴加速度 (g)
    float    accel_z;       // Z 轴加速度 (g)
} imu_report_t;             // 40 bytes
```

#### 错误上报 (CMD_ERROR_REPORT)

```c
typedef struct __attribute__((packed)) {
    uint32_t timestamp;     // 时间戳
    uint8_t  error_code;    // 错误码
    uint8_t  severity;      // 严重程度 (0=info, 1=warn, 2=error, 3=fatal)
    uint8_t  source;        // 错误来源 (0=系统, 1=IMU, 2=电机, 3=通信)
    char     message[32];   // 错误描述 (可选)
} error_report_t;           // 39 bytes

// 错误码定义
#define ERR_NONE            0x00    // 无错误
#define ERR_IMU_FAIL        0x01    // IMU 故障
#define ERR_MOTOR_FAIL      0x02    // 电机故障
#define ERR_OVER_TILT       0x03    // 倾角过大
#define ERR_LOW_BATTERY     0x04    // 电池电量低
#define ERR_COMM_TIMEOUT    0x05    // 通信超时
#define ERR_MOTOR_OVERLOAD  0x06    // 电机过载
#define ERR_SENSOR_FAIL     0x07    // 传感器故障
```

### 4.4 通用响应

#### ACK (CMD_ACK)

```c
typedef struct __attribute__((packed)) {
    uint8_t  seq;           // 对应的请求序列号
    uint8_t  cmd;           // 对应的命令码
} ack_t;                    // 2 bytes
```

#### NACK (CMD_NACK)

```c
typedef struct __attribute__((packed)) {
    uint8_t  seq;           // 对应的请求序列号
    uint8_t  cmd;           // 对应的命令码
    uint8_t  error_code;    // 拒绝原因
} nack_t;                   // 3 bytes

// NACK 错误码
#define NACK_UNKNOWN_CMD    0x01    // 未知命令
#define NACK_INVALID_PARAM  0x02    // 参数无效
#define NACK_WRONG_STATE    0x03    // 状态不允许
#define NACK_BUSY           0x04    // 设备忙
#define NACK_CRC_ERROR      0x05    // CRC 错误
```

---

## 五、连接管理

### 5.1 状态机

```
                        ┌──────────────────┐
                        │                  │
                        ▼                  │ 握手成功
    ┌──────────┐    握手请求    ┌──────────┴───┐
    │ 断开     │──────────────►│  握手中       │
    │ DISCONN  │               │  HANDSHAKING  │
    └──────────┘               └───────┬───────┘
         ▲                             │
         │                             │ 握手超时/失败
         │                             ▼
         │                     ┌───────────────┐
         │                     │               │
         │ 心跳超时 (300ms)    │               │
         │                     ▼               │
    ┌────┴─────┐  心跳正常  ┌──────────────┐   │
    │ 重连中   │◄───────────│   已连接     │◄──┘
    │RECONNECT │            │  CONNECTED   │
    └────┬─────┘            └──────┬───────┘
         │                         │
         │ 重试 3 次失败           │ 收到急停/断开命令
         │                         ▼
         │                  ┌──────────────┐
         └─────────────────►│   安全停止   │
                            │  SAFE_STOP   │
                            └──────────────┘
```

### 5.2 心跳机制

```
时间线 (树莓派为主)
──────────────────────────────────────────────────────────────►
    │         │         │         │         │
    ▼         ▼         ▼         ▼         ▼
   HB        HB        HB        HB        HB       树莓派发送 (100ms)
    │         │         │         │         │
    ▼         ▼         ▼         ▼         ▼
   ACK       ACK       ACK      (丢失)    (丢失)    ESP32 响应
                                  │         │
                                  └────┬────┘
                                       │
                                 超时计数 +1
                                       │
                            连续 3 次超时 → 断连
```

### 5.3 超时参数

| 参数 | 值 | 说明 |
|------|-----|------|
| 心跳间隔 | 100 ms | 树莓派发送心跳的频率 |
| 心跳超时 | 300 ms | 连续未收到心跳则断连 |
| 命令超时 | 50 ms | 需要 ACK 的命令等待时间 |
| 重传次数 | 3 次 | 命令重传最大次数 |
| 重连间隔 | 500 ms | 断连后重新握手的间隔 |
| 握手超时 | 1000 ms | 握手等待响应时间 |

### 5.4 断连保护

当 ESP32 检测到与树莓派断连时:

```c
void pi_comm_on_disconnect(void) {
    // 1. 立即停止运动
    set_target_velocity(0, 0);
    
    // 2. 根据配置决定后续动作
    if (g_config.disconnect_action == DISCONNECT_STOP) {
        // 保持平衡，等待重连
        set_mode(MODE_STAND);
    } else if (g_config.disconnect_action == DISCONNECT_DISABLE) {
        // 失能电机
        motor_disable_all();
    }
    
    // 3. 触发告警
    error_report(ERR_COMM_TIMEOUT, "Pi disconnected");
}
```

---

## 六、数据流设计

### 6.1 下行数据流 (树莓派 → ESP32)

| 数据类型 | 频率 | 命令 | 需要 ACK |
|----------|------|------|----------|
| 心跳 | 10 Hz | `CMD_HEARTBEAT` | ✅ |
| 速度指令 | 20-50 Hz | `CMD_SET_VELOCITY` | ❌ (最新值覆盖) |
| 模式切换 | 事件触发 | `CMD_SET_MODE` | ✅ |
| 高度调整 | 事件触发 | `CMD_SET_HEIGHT` | ✅ |
| 紧急停止 | 事件触发 | `CMD_EMERGENCY_STOP` | ✅ |

### 6.2 上行数据流 (ESP32 → 树莓派)

| 数据类型 | 频率 | 命令 | 触发方式 |
|----------|------|------|----------|
| 状态上报 | 20 Hz | `CMD_STATUS_REPORT` | 周期性 |
| IMU 数据 | 50-100 Hz | `CMD_IMU_REPORT` | 可选开启 |
| 心跳响应 | 10 Hz | `CMD_HEARTBEAT_ACK` | 响应心跳 |
| 错误告警 | 事件触发 | `CMD_ERROR_REPORT` | 立即上报 |
| ACK/NACK | 按需 | `CMD_ACK/NACK` | 响应控制命令 |

### 6.3 带宽估算

假设 115200 bps, 8N1:

```
下行:
  心跳:      10 Hz × 11 B = 110 B/s
  速度指令:  50 Hz × 17 B = 850 B/s
  ─────────────────────────────────
  合计:      ~1 KB/s (约 8% 带宽)

上行:
  状态上报:  20 Hz × 47 B = 940 B/s
  心跳响应:  10 Hz × 17 B = 170 B/s
  ─────────────────────────────────
  合计:      ~1.1 KB/s (约 9% 带宽)
```

**结论**: 115200 bps 带宽充足，可以支持更高频率。

---

## 七、实现架构

### 7.1 ESP32 端 (C)

```
components/
├── pi_comm/                        # 新组件
│   ├── include/
│   │   ├── pi_comm.h               # 公共接口
│   │   ├── pi_protocol.h           # 协议定义 (命令码、数据结构)
│   │   └── pi_frame.h              # 帧解析器
│   ├── src/
│   │   ├── pi_comm.c               # 通信任务主逻辑
│   │   ├── pi_protocol.c           # 命令处理函数
│   │   ├── pi_frame.c              # 帧解析/构建
│   │   └── pi_crc.c                # CRC 计算
│   └── CMakeLists.txt
│
└── app/
    └── src/
        └── balance_test.c          # 集成 pi_comm
```

#### 核心任务

```c
// pi_comm.c
static void pi_comm_task(void *arg) {
    // 初始化 UART
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
    };
    uart_driver_install(UART_NUM_1, 1024, 1024, 20, &uart_queue, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    
    while (1) {
        // 1. 接收数据
        int len = uart_read_bytes(UART_NUM_1, rx_buf, sizeof(rx_buf), 10 / portTICK_PERIOD_MS);
        if (len > 0) {
            // 2. 帧解析
            pi_frame_t frame;
            while (frame_parser_feed(rx_buf, len, &frame)) {
                // 3. 命令处理
                pi_protocol_handle(&frame);
            }
        }
        
        // 4. 心跳检测
        if (heartbeat_check_timeout()) {
            pi_comm_on_disconnect();
        }
        
        // 5. 周期性状态上报
        if (should_report_status()) {
            pi_send_status_report();
        }
    }
}
```

### 7.2 树莓派端 (Python)

```python
# pi_comm.py
import serial
import struct
import threading
from enum import IntEnum
from dataclasses import dataclass

class CMD(IntEnum):
    HEARTBEAT = 0x01
    SET_VELOCITY = 0x20
    STATUS_REPORT = 0x60
    # ...

@dataclass
class StatusReport:
    timestamp: int
    mode: int
    pitch: float
    roll: float
    yaw: float
    vx_actual: float
    yaw_rate_actual: float
    battery_voltage: float

class ESP32Protocol:
    HEADER = bytes([0xAA, 0x55])
    
    def __init__(self, port='/dev/ttyUSB0', baudrate=115200):
        self.ser = serial.Serial(port, baudrate, timeout=0.1)
        self.seq = 0
        self.connected = False
        self.callbacks = {}
        
        # 启动接收线程
        self.rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        self.rx_thread.start()
        
        # 启动心跳线程
        self.hb_thread = threading.Thread(target=self._heartbeat_loop, daemon=True)
        self.hb_thread.start()
    
    def send_velocity(self, vx: float, yaw_rate: float):
        """发送速度指令"""
        data = struct.pack('>ffBB', vx, yaw_rate, 0, 0)  # 大端序
        self._send_frame(CMD.SET_VELOCITY, data)
    
    def on_status(self, callback):
        """注册状态回调"""
        self.callbacks['status'] = callback
    
    def _send_frame(self, cmd: int, data: bytes = b''):
        self.seq = (self.seq + 1) & 0xFF
        payload = bytes([self.seq, cmd]) + data
        crc = self._crc16(payload)
        frame = self.HEADER + bytes([len(payload)]) + payload + struct.pack('>H', crc)  # CRC 大端序
        self.ser.write(frame)
    
    def _crc16(self, data: bytes) -> int:
        crc = 0xFFFF
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                crc = (crc << 1) ^ 0x1021 if crc & 0x8000 else crc << 1
            crc &= 0xFFFF
        return crc
    
    def _rx_loop(self):
        """接收线程"""
        buffer = bytearray()
        while True:
            data = self.ser.read(256)
            if data:
                buffer.extend(data)
                # 解析帧...
    
    def _heartbeat_loop(self):
        """心跳线程"""
        import time
        while True:
            self._send_frame(CMD.HEARTBEAT, struct.pack('>I', int(time.time() * 1000)))  # 大端序
            time.sleep(0.1)
```

---

## 八、调试支持

### 8.1 调试模式切换

发送 `CMD_ENTER_DEBUG` 后，ESP32 切换到文本协议模式:
- 可以使用现有的 Commander 协议调试
- 所有二进制命令被忽略
- 发送 `CMD_EXIT_DEBUG` 或重启恢复

### 8.2 日志等级

```c
typedef enum {
    LOG_NONE = 0,
    LOG_ERROR = 1,
    LOG_WARN = 2,
    LOG_INFO = 3,
    LOG_DEBUG = 4,
    LOG_VERBOSE = 5,
} pi_log_level_t;

// 可通过命令动态设置日志等级
#define CMD_SET_LOG_LEVEL   0x0A
```

### 8.3 协议分析

推荐使用 Wireshark 或自定义脚本分析串口数据:

```python
# 简单的协议分析脚本
def parse_frame(data):
    if data[0:2] != b'\xAA\x55':
        return None
    length = data[2]
    seq = data[3]
    cmd = data[4]
    payload = data[5:3+length]
    crc = struct.unpack('>H', data[3+length:5+length])[0]  # 大端序
    print(f"SEQ={seq:02X} CMD={cmd:02X} LEN={length} CRC={crc:04X}")
    print(f"  Payload: {payload.hex()}")
```

---

## 九、扩展预留

### 9.1 预留命令码

| 范围 | 用途 |
|------|------|
| 0x30-0x3F | 高级运动控制 (跳跃、特殊动作) |
| 0x50-0x5F | 传感器查询 (距离、触觉等) |
| 0x70-0x7F | 高级上报 (视觉标定、地图等) |
| 0xA0-0xBF | 自定义扩展 |

### 9.2 能力协商

在握手时交换能力位图，用于版本兼容:

```c
// 能力位定义
#define CAP_BASIC_CONTROL   (1 << 0)    // 基本运动控制
#define CAP_HEIGHT_CONTROL  (1 << 1)    // 高度控制
#define CAP_YAW_CONTROL     (1 << 2)    // YAW 控制
#define CAP_JUMP            (1 << 3)    // 跳跃 (预留)
#define CAP_VISION          (1 << 4)    // 视觉接口 (预留)
#define CAP_HIGH_RATE_IMU   (1 << 5)    // 高频 IMU (预留)
```

---

## 十、实现计划

### Phase 1: 基础框架 (1-2 天)
- [ ] 创建 `pi_comm` 组件
- [ ] 实现帧解析器 (`pi_frame.c`)
- [ ] 实现 CRC 校验 (`pi_crc.c`)
- [ ] 基本 UART 收发

### Phase 2: 核心功能 (2-3 天)
- [ ] 实现握手流程
- [ ] 实现心跳机制
- [ ] 实现速度指令 (`CMD_SET_VELOCITY`)
- [ ] 实现状态上报 (`CMD_STATUS_REPORT`)
- [ ] 断连保护

### Phase 3: 完善功能 (1-2 天)
- [ ] 实现模式切换
- [ ] 实现紧急停止
- [ ] 实现错误上报
- [ ] 调试模式支持

### Phase 4: 树莓派端 (1-2 天)
- [ ] Python 协议库
- [ ] 简单测试脚本
- [ ] 文档完善

---

## 十一、参考资料

- [ESP-IDF UART 驱动](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/uart.html)
- [FreeRTOS 队列](https://www.freertos.org/Embedded-RTOS-Queues.html)
- [CRC16-CCITT 算法](https://www.lammertbies.nl/comm/info/crc-calculation)
- [PySerial 文档](https://pyserial.readthedocs.io/)

---

## 修改历史

| 日期 | 版本 | 修改内容 |
|------|------|----------|
| 2026-01-26 | v1.0 | 初始设计文档 |
