# 双轮足机器人项目文档

> 项目名称: wheel_leg_final  
> 平台: ESP32-S3  
> 框架: ESP-IDF v5.5  
> 作者: Bubble  
> 创建日期: 2026-01-15

---

## 一、项目概述

这是一个双轮足机器人项目，每条腿包含：
- **2个关节电机** (大腿 + 小腿)
- **1个轮电机**

共计 **6个电机**，通过单路 CAN 总线控制，使用不同 ID 区分左右腿。

---

## 二、硬件引脚分配

### 2.1 CAN 总线 (电机控制)

| 功能 | GPIO | 说明 |
|------|------|------|
| CAN_TX | IO18 | CAN 发送 |
| CAN_RX | IO17 | CAN 接收 |

> ✅ **采用单 CAN 总线方案**: 6 个电机通过不同 ID 区分，共用一条 CAN 总线。

### 2.2 按键输入

| 功能 | GPIO | 说明 |
|------|------|------|
| 按键1 | IO13 | 外部上拉，按下为低电平 |
| 按键2 | IO9 | 外部上拉，按下为低电平 |

### 2.3 电源检测

| 功能 | GPIO | 说明 |
|------|------|------|
| 电源检测 | IO3 | 高电平=电池供电，低电平=USB供电 |

### 2.4 电机供电控制

| 功能 | GPIO | 说明 |
|------|------|------|
| 电机供电使能 | IO8 | 高电平=电机供电，低电平=断开电机供电 |

### 2.5 串口通信 (树莓派)

| 功能 | GPIO | 说明 |
|------|------|------|
| UART_TX | IO4 | 发送到树莓派 |
| UART_RX | IO5 | 接收树莓派数据 |

> ⚠️ **注意**: 通信协议格式尚未定义

### 2.6 I2C 总线

#### I2C1 - 温湿度传感器 (SHT30)

| 功能 | GPIO | 说明 |
|------|------|------|
| I2C1_SCL | IO1 | SHT30 时钟线 |
| I2C1_SDA | IO2 | SHT30 数据线 |

#### I2C2 - IMU 陀螺仪 (维特智能)

| 功能 | GPIO | 说明 |
|------|------|------|
| I2C2_SCL | IO11 | 陀螺仪时钟线 |
| I2C2_SDA | IO12 | 陀螺仪数据线 |

---

## 三、电机配置

### 3.1 电机 ID 分配

采用单 CAN 总线，6 个电机通过不同 ID 区分：

| 腿部 | 电机 | ID | 说明 |
|------|------|-----|------|
| **左腿** | 大腿电机 | 1 | 髋关节/大腿关节 |
| **左腿** | 小腿电机 | 2 | 膝关节/小腿关节 |
| **左腿** | 轮电机 | 3 | 轮毂电机 |
| **右腿** | 大腿电机 | 4 | 髋关节/大腿关节 |
| **右腿** | 小腿电机 | 5 | 膝关节/小腿关节 |
| **右腿** | 轮电机 | 6 | 轮毂电机 |

**代码中的定义:**
```c
// common/config.h

// 左腿电机 ID
#define MOTOR_ID_LEFT_HIP     1   // 左大腿
#define MOTOR_ID_LEFT_KNEE    2   // 左小腿
#define MOTOR_ID_LEFT_WHEEL   3   // 左轮

// 右腿电机 ID
#define MOTOR_ID_RIGHT_HIP    4   // 右大腿
#define MOTOR_ID_RIGHT_KNEE   5   // 右小腿
#define MOTOR_ID_RIGHT_WHEEL  6   // 右轮

// CAN 帧 ID
#define CAN_TX_BASE_ID        0x600   // 发送帧基地址
#define CAN_RX_BASE_ID        0x580   // 接收帧基地址
```

### 3.2 CAN 通信协议

基于 CAN2.0A 标准帧协议，波特率 **1Mbps**。

> ⚠️ **重要**: 所有寄存器地址和数据均采用**大端序 (Big-Endian)**，即高字节在前！

#### 帧 ID 规则

- **发送帧 ID**: `0x600 + 电机ID`
- **接收帧 ID**: `0x580 + 电机ID`

#### 命令字说明

| 命令字 | 功能 | 数据长度 |
|--------|------|----------|
| 0x4B | 读取 1 个寄存器 (2字节) | 8 |
| 0x43 | 读取 2 个寄存器 (4字节) | 8 |
| 0x2B | 写入 1 个寄存器 (2字节) | 8 |
| 0x23 | 写入 2 个寄存器 (4字节) | 8 |
| 0x24 | PV 指令 (位置+速度) | 8 |
| 0x25 | PVT 指令 (位置+速度+力矩%) | 8 |
| 0x2A | 驱动器写入回复 | 8 |

#### CAN 帧格式详解

##### 读取 1 个寄存器 (0x4B) - 2字节数据

**发送:**
| data[0] | data[1] | data[2] | data[3] | data[4] | data[5] | data[6] | data[7] |
|---------|---------|---------|---------|---------|---------|---------|---------|
| 0x4B | RegH | RegL | 0x00 | 0x00 | 0x00 | 0x00 | 0x00 |

**回复:**
| data[0] | data[1] | data[2] | data[3] | data[4] | data[5] | data[6] | data[7] |
|---------|---------|---------|---------|---------|---------|---------|---------|
| 0x4B | RegH | RegL | 0x00 | DataH | DataL | 0x00 | 0x00 |

**示例 - 读取电压 (REG_VOLTAGE = 0x0004):**
- 发送: `4B 00 04 00 00 00 00 00`
- 回复: `4B 00 04 00 00 7B 00 00` → 0x007B = 123 → 12.3V

##### 读取 2 个寄存器 (0x43) - 4字节数据

**发送:**
| data[0] | data[1] | data[2] | data[3] | data[4] | data[5] | data[6] | data[7] |
|---------|---------|---------|---------|---------|---------|---------|---------|
| 0x43 | RegH | RegL | 0x00 | 0x00 | 0x00 | 0x00 | 0x00 |

**回复:**
| data[0] | data[1] | data[2] | data[3] | data[4] | data[5] | data[6] | data[7] |
|---------|---------|---------|---------|---------|---------|---------|---------|
| 0x43 | RegH | RegL | 0x00 | Data1 | Data2 | Data3 | Data4 |

**示例 - 读取位置 (REG_POSITION = 0x0008):**
- 发送: `43 00 08 00 00 00 00 00`
- 回复: `43 00 08 00 00 00 8C A0` → 0x00008CA0 = 36000 → 360.00°

##### 写入 1 个寄存器 (0x2B) - 2字节数据

**发送:**
| data[0] | data[1] | data[2] | data[3] | data[4] | data[5] | data[6] | data[7] |
|---------|---------|---------|---------|---------|---------|---------|---------|
| 0x2B | RegH | RegL | 0x00 | DataH | DataL | 0x00 | 0x00 |

**回复 (0x2A):**
| data[0] | data[1] | data[2] | data[3] | data[4] | data[5] | data[6] | data[7] |
|---------|---------|---------|---------|---------|---------|---------|---------|
| 0x2A | PosH | PosM | PosL | SpdH | SpdL | CurH | CurL |

- 位置: 3字节有符号数 (×100)
- 速度: 2字节有符号数 (rpm)
- 电流: 2字节有符号数 (×100)

##### 写入 2 个寄存器 (0x23) - 4字节数据

**发送:**
| data[0] | data[1] | data[2] | data[3] | data[4] | data[5] | data[6] | data[7] |
|---------|---------|---------|---------|---------|---------|---------|---------|
| 0x23 | RegH | RegL | 0x00 | Data1 | Data2 | Data3 | Data4 |

**示例 - 设置速度 500rpm (REG_SET_SPEED = 0x0021):**
- 500 × 100 = 50000 = 0x0000C350
- 发送: `23 00 21 00 00 00 C3 50`

##### PV 指令 (0x24) - 位置+速度

| data[0] | data[1] | data[2] | data[3] | data[4] | data[5] | data[6] | data[7] |
|---------|---------|---------|---------|---------|---------|---------|---------|
| 0x24 | PosH | PosM | PosM | PosL | SpdH | SpdL | 0x00 |

- 位置: 4字节有符号数 (×100)
- 速度: 2字节有符号数 (rpm, 不放大)

**示例 - 以150rpm转到-360°:**
- 位置: -360 × 100 = -36000 = 0xFFFF7360
- 速度: 150 = 0x0096
- 发送: `24 FF FF 73 60 00 96 00`

#### 常用寄存器

| 寄存器地址 | 名称 | 读/写 | 说明 |
|------------|------|-------|------|
| 0x0004 | REG_VOLTAGE | R | 电源电压 (×10) |
| 0x0005 | REG_CURRENT | R | 母线电流 (×100) |
| 0x0006 | REG_SPEED | R | 实时速度 (×100) |
| 0x0008 | REG_POSITION | R | 实时位置 (×100) |
| 0x000A | REG_DRIVER_TEMP | R | 驱动器温度 (×10) |
| 0x000B | REG_MOTOR_TEMP | R | 电机温度 (×10) |
| 0x000C | REG_ERROR | R | 错误信息 |
| 0x0020 | REG_SET_TORQUE | RW | 设置力矩 (×100) |
| 0x0021 | REG_SET_SPEED | RW | 设置速度 (×100) |
| 0x0023 | REG_SET_ABS_POS | RW | 设置绝对位置 (×100) |
| 0x0060 | REG_CONTROL_MODE | RW | 控制模式 |
| 0x00A0 | REG_IDLE | W | 进入空闲状态 |
| 0x00A2 | REG_CLOSE_LOOP | W | 进入闭环控制 |

#### 控制模式

| 值 | 模式 |
|----|------|
| 0 | MODE_TORQUE - 力矩模式 |
| 1 | MODE_SPEED - 速度模式 |
| 2 | MODE_POS_TRAP - 位置梯形轨迹 |
| 3 | MODE_POS_FILTER - 位置滤波模式 |
| 4 | MODE_POS_DIRECT - 位置直通模式 |
| 5 | MODE_LOW_SPEED - 低速大扭模式 |

---

## 四、代码框架设计

### 4.1 目录结构规划

```
wheel_leg_final/
├── CMakeLists.txt
├── sdkconfig
├── docs/
│   └── PROJECT_NOTES.md              # 本文档
│
├── main/
│   ├── CMakeLists.txt
│   └── main.c                        # 主入口 (系统初始化 + 任务创建)
│
└── components/
    │
    ├── device/                       # ========== 硬件驱动层 ==========
    │   ├── CMakeLists.txt
    │   ├── include/
    │   │   ├── can_motor.h           # CAN 电机驱动
    │   │   ├── can_motor_regs.h      # 电机寄存器定义
    │   │   ├── imu_driver.h          # IMU 陀螺仪驱动
    │   │   ├── sht30_driver.h        # 温湿度传感器驱动
    │   │   ├── button_driver.h       # 按键驱动
    │   │   ├── power_detect.h        # 电源检测
    │   │   └── uart_comm.h           # UART 通信接口
    │   └── src/
    │       ├── can_motor.c
    │       ├── imu_driver.c
    │       ├── sht30_driver.c
    │       ├── button_driver.c
    │       ├── power_detect.c
    │       └── uart_comm.c
    │
    ├── wifi_remote/                  # ========== WiFi 遥控模块 ==========
    │   ├── CMakeLists.txt            # (移植自 shibo_wheel_leg)
    │   ├── include/
    │   │   ├── wifi_remote.h         # WiFi 遥控主接口
    │   │   ├── wifi_config.h         # AP/STA 配置
    │   │   └── web_page.h            # HTML 页面 (原 basic_web.h)
    │   └── src/
    │       ├── wifi_remote.c         # WiFi + HTTP Server + WebSocket
    │       └── json_parser.c         # JSON 命令解析 (替代 ArduinoJson)
    │
    ├── algorithm/                    # ========== 算法层 ==========
    │   ├── CMakeLists.txt
    │   ├── include/
    │   │   ├── balance_ctrl.h        # 平衡控制算法 (LQR/PID)
    │   │   ├── leg_kinematics.h      # 腿部正逆运动学
    │   │   ├── motion_planner.h      # 运动规划
    │   │   └── filter.h              # 滤波器 (卡尔曼/互补滤波)
    │   └── src/
    │       ├── balance_ctrl.c
    │       ├── leg_kinematics.c
    │       ├── motion_planner.c
    │       └── filter.c
    │
    ├── control/                      # ========== 控制逻辑层 ==========
    │   ├── CMakeLists.txt
    │   ├── include/
    │   │   ├── leg_controller.h      # 单腿控制器
    │   │   ├── robot_controller.h    # 整机控制器
    │   │   └── motor_manager.h       # 电机管理 (6电机统一接口)
    │   └── src/
    │       ├── leg_controller.c
    │       ├── robot_controller.c
    │       └── motor_manager.c
    │
    ├── app/                          # ========== 应用层 ==========
    │   ├── CMakeLists.txt
    │   ├── include/
    │   │   ├── state_machine.h       # 状态机 (待机/站立/运动/急停等)
    │   │   ├── task_manager.h        # RTOS 任务管理
    │   │   ├── cmd_parser.h          # 命令解析 (来自树莓派)
    │   │   └── safety_monitor.h      # 安全监控 (过温/过流/倾倒保护)
    │   └── src/
    │       ├── state_machine.c
    │       ├── task_manager.c
    │       ├── cmd_parser.c
    │       └── safety_monitor.c
    │
    └── common/                       # ========== 公共模块 ==========
        ├── CMakeLists.txt
        ├── include/
        │   ├── config.h              # 全局配置 (引脚定义/参数)
        │   ├── types.h               # 自定义类型
        │   └── utils.h               # 工具函数
        └── src/
            └── utils.c
```

### 4.2 分层架构说明

```
┌─────────────────────────────────────────────────────────────┐
│                      应用层 (app/)                          │
│   状态机 │ 命令解析 │ 安全监控 │ 任务管理                    │
├─────────────────────────────────────────────────────────────┤
│                     控制逻辑层 (control/)                   │
│   整机控制器 │ 单腿控制器 │ 电机管理                         │
├─────────────────────────────────────────────────────────────┤
│                      算法层 (algorithm/)                    │
│   平衡控制 │ 运动学 │ 运动规划 │ 滤波器                      │
├─────────────────────────────────────────────────────────────┤
│                    硬件驱动层 (device/)                     │
│   CAN电机 │ IMU │ SHT30 │ 按键 │ 电源检测 │ UART           │
├─────────────────────────────────────────────────────────────┤
│              WiFi遥控模块 (wifi_remote/)                    │
│   WiFi AP │ HTTP Server │ WebSocket │ JSON解析             │
├─────────────────────────────────────────────────────────────┤
│                   ESP-IDF / FreeRTOS                        │
└─────────────────────────────────────────────────────────────┘
```

---

## 五、FreeRTOS 任务设计

### 5.1 ESP-IDF 与 FreeRTOS

> ✅ **ESP-IDF 默认使用 FreeRTOS**，无需额外配置即可使用多任务。

ESP32-S3 是**双核** CPU：
- **Core 0 (PRO_CPU)**: 通常运行 WiFi/BT 协议栈
- **Core 1 (APP_CPU)**: 运行用户应用程序

可以使用 `xTaskCreatePinnedToCore()` 指定任务运行在哪个核心。

### 5.2 任务规划

| 任务名称 | 优先级 | 周期 | CPU核心 | 栈大小 | 说明 |
|----------|--------|------|---------|--------|------|
| `task_balance_ctrl` | **最高 (24)** | 1-5ms | Core 1 | 4KB | 平衡控制 (实时性要求最高) |
| `task_motor_comm` | 高 (20) | 1-2ms | Core 1 | 4KB | CAN 电机通信 (发送指令+接收状态) |
| `task_imu_read` | 高 (18) | 1-5ms | Core 1 | 2KB | IMU 数据读取 |
| `task_state_machine` | 中 (12) | 10ms | Core 0 | 4KB | 状态机管理 |
| `task_wifi_remote` | 中 (10) | 100ms | Core 0 | 8KB | WiFi 遥控 (HTTP+WebSocket) |
| `task_uart_comm` | 中 (10) | 20ms | Core 0 | 4KB | 树莓派通信 |
| `task_safety_monitor` | 中 (10) | 100ms | Core 0 | 2KB | 安全监控 (温度/电流) |
| `task_sensor_read` | 低 (5) | 1000ms | Core 0 | 2KB | 温湿度等慢速传感器 |
| `task_button_handle` | 低 (5) | 50ms | Core 0 | 2KB | 按键检测 (或用中断) |

### 5.3 任务优先级原则

```
优先级高 ─────────────────────────────────────────► 优先级低

平衡控制 > 电机通信 > IMU读取 > 状态机 > 通信 > 传感器 > 按键
   24        20         18       12      10       5        5
```

**原则**:
1. **实时控制任务** (平衡、电机) 优先级最高，运行在 Core 1
2. **通信任务** (UART、WiFi) 运行在 Core 0，避免干扰控制
3. **慢速传感器** 优先级最低

### 5.4 任务代码示例

```c
// main.c 或 task_manager.c

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"

#define TAG "TASK"

// ==================== 任务函数声明 ====================

void task_balance_ctrl(void *pvParameters);
void task_motor_comm(void *pvParameters);
void task_imu_read(void *pvParameters);
void task_state_machine(void *pvParameters);
void task_uart_comm(void *pvParameters);
void task_safety_monitor(void *pvParameters);

// ==================== 任务句柄 ====================

TaskHandle_t h_balance_ctrl = NULL;
TaskHandle_t h_motor_comm = NULL;
TaskHandle_t h_imu_read = NULL;
TaskHandle_t h_state_machine = NULL;
TaskHandle_t h_uart_comm = NULL;
TaskHandle_t h_safety_monitor = NULL;

// ==================== 创建所有任务 ====================

void create_all_tasks(void) {
    ESP_LOGI(TAG, "Creating tasks...");
    
    // 平衡控制 - 最高优先级, Core 1
    xTaskCreatePinnedToCore(
        task_balance_ctrl,      // 任务函数
        "balance_ctrl",         // 任务名称
        4096,                   // 栈大小 (字节)
        NULL,                   // 参数
        24,                     // 优先级
        &h_balance_ctrl,        // 句柄
        1                       // Core 1
    );
    
    // CAN 电机通信 - 高优先级, Core 1
    xTaskCreatePinnedToCore(
        task_motor_comm,
        "motor_comm",
        4096,
        NULL,
        20,
        &h_motor_comm,
        1
    );
    
    // IMU 读取 - 高优先级, Core 1
    xTaskCreatePinnedToCore(
        task_imu_read,
        "imu_read",
        2048,
        NULL,
        18,
        &h_imu_read,
        1
    );
    
    // 状态机 - 中优先级, Core 0
    xTaskCreatePinnedToCore(
        task_state_machine,
        "state_machine",
        4096,
        NULL,
        12,
        &h_state_machine,
        0
    );
    
    // UART 通信 - 中优先级, Core 0
    xTaskCreatePinnedToCore(
        task_uart_comm,
        "uart_comm",
        4096,
        NULL,
        10,
        &h_uart_comm,
        0
    );
    
    // 安全监控 - 中优先级, Core 0
    xTaskCreatePinnedToCore(
        task_safety_monitor,
        "safety_monitor",
        2048,
        NULL,
        10,
        &h_safety_monitor,
        0
    );
    
    ESP_LOGI(TAG, "All tasks created");
}

// ==================== 任务实现示例 ====================

// 平衡控制任务 (1-5ms 周期)
void task_balance_ctrl(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(2);  // 2ms = 500Hz
    
    while (1) {
        // 1. 获取 IMU 数据 (从共享变量或队列)
        // 2. 运行平衡算法
        // 3. 计算电机目标值
        // 4. 发送到电机控制队列
        
        // 精确延时，保证周期稳定
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

// CAN 电机通信任务
void task_motor_comm(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(1);  // 1ms
    
    while (1) {
        // 1. 从队列获取电机指令
        // 2. 通过 CAN 发送指令
        // 3. 接收电机状态反馈
        // 4. 更新电机状态到共享变量
        
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

// IMU 读取任务
void task_imu_read(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xPeriod = pdMS_TO_TICKS(2);  // 2ms = 500Hz
    
    while (1) {
        // 1. 读取 IMU 原始数据
        // 2. 滤波处理
        // 3. 更新姿态数据到共享变量
        
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

// 状态机任务
void task_state_machine(void *pvParameters) {
    while (1) {
        // 1. 检查当前状态
        // 2. 处理状态转换
        // 3. 执行状态动作
        
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms
    }
}
```

### 5.5 任务间通信机制

| 机制 | 用途 | 示例 |
|------|------|------|
| **队列 (Queue)** | 任务间传递数据 | 电机指令队列、命令队列 |
| **信号量 (Semaphore)** | 同步/互斥 | I2C 总线访问互斥 |
| **事件组 (Event Group)** | 多事件同步 | 等待多个初始化完成 |
| **任务通知 (Task Notify)** | 轻量级通知 | 唤醒特定任务 |
| **共享变量 + 互斥锁** | 共享状态数据 | IMU 姿态数据、电机状态 |

```c
// 示例: 电机指令队列
QueueHandle_t motor_cmd_queue;

void init_queues(void) {
    motor_cmd_queue = xQueueCreate(10, sizeof(motor_cmd_t));
}

// 平衡控制任务发送指令
void task_balance_ctrl(void *pvParameters) {
    motor_cmd_t cmd;
    while (1) {
        // 计算电机指令
        cmd.motor_id = 1;
        cmd.speed = 100.0f;
        
        // 发送到队列
        xQueueSend(motor_cmd_queue, &cmd, 0);
        
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}

// 电机通信任务接收指令
void task_motor_comm(void *pvParameters) {
    motor_cmd_t cmd;
    while (1) {
        // 从队列接收指令
        if (xQueueReceive(motor_cmd_queue, &cmd, pdMS_TO_TICKS(1)) == pdTRUE) {
            // 发送 CAN 指令
            can_motor_set_speed(cmd.motor_id, cmd.speed);
        }
        
        vTaskDelayUntil(&xLastWakeTime, xPeriod);
    }
}
```

### 5.6 共享数据结构

```c
// types.h

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

// IMU 数据结构
typedef struct {
    float roll;           // 横滚角 (°)
    float pitch;          // 俯仰角 (°)
    float yaw;            // 偏航角 (°)
    float gyro_x;         // 角速度 X (°/s)
    float gyro_y;         // 角速度 Y (°/s)
    float gyro_z;         // 角速度 Z (°/s)
    float accel_x;        // 加速度 X (g)
    float accel_y;        // 加速度 Y (g)
    float accel_z;        // 加速度 Z (g)
    uint32_t timestamp;   // 时间戳 (ms)
} imu_data_t;

// 电机状态
typedef struct {
    float position;       // 位置 (°)
    float speed;          // 速度 (rpm)
    float current;        // 电流 (A)
    float temperature;    // 温度 (°C)
    uint32_t error_code;  // 错误码
    bool is_online;       // 在线状态
} motor_state_t;

// 机器人全局状态
typedef struct {
    imu_data_t imu;
    motor_state_t motors[6];    // 6个电机
    float battery_voltage;
    bool is_battery_powered;
    SemaphoreHandle_t mutex;    // 互斥锁
} robot_state_t;

extern robot_state_t g_robot_state;

// 安全访问宏
#define ROBOT_STATE_LOCK()    xSemaphoreTake(g_robot_state.mutex, portMAX_DELAY)
#define ROBOT_STATE_UNLOCK()  xSemaphoreGive(g_robot_state.mutex)
```

### 5.7 CPU 资源分配建议

```
┌────────────────────────────────────────────────────────────────┐
│                          Core 1 (APP_CPU)                      │
│                      实时控制任务专用                           │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐             │
│  │ 平衡控制    │  │ 电机通信    │  │ IMU读取     │             │
│  │ 2ms/500Hz  │  │ 1ms/1000Hz │  │ 2ms/500Hz  │             │
│  │ 优先级:24  │  │ 优先级:20  │  │ 优先级:18  │             │
│  └─────────────┘  └─────────────┘  └─────────────┘             │
├────────────────────────────────────────────────────────────────┤
│                          Core 0 (PRO_CPU)                      │
│                      非实时任务 + 系统服务                       │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐             │
│  │ 状态机      │  │ UART通信    │  │ 安全监控    │             │
│  │ 10ms       │  │ 20ms       │  │ 100ms      │             │
│  │ 优先级:12  │  │ 优先级:10  │  │ 优先级:10  │             │
│  └─────────────┘  └─────────────┘  └─────────────┘             │
│                                                                │
│  ┌─────────────┐  ┌─────────────┐                              │
│  │ 温湿度读取  │  │ 按键处理    │  + ESP-IDF系统任务            │
│  │ 1000ms     │  │ 50ms/中断  │                              │
│  │ 优先级:5   │  │ 优先级:5   │                              │
│  └─────────────┘  └─────────────┘                              │
└────────────────────────────────────────────────────────────────┘
```

---

## 六、组件详细说明

### 6.1 device/ - 硬件驱动层

#### can_motor (CAN 电机驱动)

**职责**:
- 初始化 TWAI (CAN) 驱动
- 封装电机读取/写入命令
- 电机状态管理

**关键 API**:
```c
// 初始化 CAN 总线
esp_err_t can_motor_init(gpio_num_t tx_pin, gpio_num_t rx_pin);

// 创建电机实例
can_motor_handle_t can_motor_create(uint8_t motor_id);

// 读取状态
float can_motor_read_position(can_motor_handle_t motor);
float can_motor_read_speed(can_motor_handle_t motor);
float can_motor_read_current(can_motor_handle_t motor);

// 控制命令
esp_err_t can_motor_set_speed(can_motor_handle_t motor, float speed_rpm);
esp_err_t can_motor_set_position(can_motor_handle_t motor, float angle_deg, float speed_rpm);
esp_err_t can_motor_set_torque(can_motor_handle_t motor, float torque_nm);
esp_err_t can_motor_enter_closed_loop(can_motor_handle_t motor);
esp_err_t can_motor_set_idle(can_motor_handle_t motor);
```

#### imu_driver (IMU 驱动)

**职责**:
- 通过 I2C2 与维特智能陀螺仪通信
- 读取姿态角 (Roll, Pitch, Yaw)
- 读取角速度、加速度

#### sht30_driver (温湿度传感器驱动)

**职责**:
- 通过 I2C1 与 SHT30 通信
- 读取温度和湿度数据

#### button_driver (按键驱动)

**职责**:
- 初始化按键 GPIO
- 提供按键状态读取接口

#### power_detect (电源检测)

**职责**:
- 初始化电源检测 GPIO
- 提供电源状态读取接口

#### uart_comm (串口通信接口)

**职责**:
- 初始化 UART
- 提供与树莓派通信的接口

---

### 6.2 wifi_remote/ - WiFi 遥控器模块 (新增)

> 参考项目: https://github.com/Bubble252/shibo_wheel_leg

#### 原 Arduino 实现分析

你之前的项目使用以下技术栈：

| 组件 | Arduino 库 | ESP-IDF 替代方案 |
|------|-----------|------------------|
| WiFi AP | `WiFi.h` | `esp_wifi.h` (原生支持) |
| HTTP Server | `WebServer.h` | `esp_http_server.h` |
| WebSocket | `WebSocketsServer.h` | `esp_websocket` 或自实现 |
| JSON 解析 | `ArduinoJson.h` | `cJSON.h` (ESP-IDF 内置) |

#### JSON 通信协议 (保持兼容)

Web 端发送的 JSON 格式:
```json
{
    "mode": "basic",
    "dir": "forward",     // stop/forward/back/left/right/jump
    "height": 38,         // 腿高度 (32-85mm)
    "roll": 0,            // Roll 角度 (-30~30°)
    "linear": 0,          // 线速度
    "angular": 0,         // 角速度
    "stable": 1,          // 使能开关 (go)
    "joy_x": 0,           // 摇杆 X (-100~100)
    "joy_y": 0            // 摇杆 Y (-100~100)
}
```

#### 遥控数据结构 (移植到 ESP-IDF)

```c
// common/types.h

// 遥控命令类型
typedef enum {
    DIR_FORWARD = 0,
    DIR_BACK,
    DIR_RIGHT,
    DIR_LEFT,
    DIR_STOP,
    DIR_JUMP,
} direction_t;

// 遥控器数据 (来自 Web/手机)
typedef struct {
    direction_t dir;          // 方向命令
    direction_t dir_last;     // 上一次方向
    int16_t height;           // 腿高度 (mm)
    int16_t roll;             // Roll 目标角度 (°)
    int16_t linear;           // 线速度
    int16_t angular;          // 角速度
    int16_t joy_x;            // 摇杆 X (-100~100)
    int16_t joy_y;            // 摇杆 Y (-100~100)
    int16_t joy_x_last;       // 上一次摇杆 X
    int16_t joy_y_last;       // 上一次摇杆 Y
    bool go;                  // 使能开关
    uint32_t last_update_ms;  // 最后更新时间 (用于超时检测)
} remote_cmd_t;

extern remote_cmd_t g_remote_cmd;
```

#### ESP-IDF WiFi 遥控器实现

**目录结构:**
```
components/
└── wifi_remote/
    ├── CMakeLists.txt
    ├── include/
    │   ├── wifi_remote.h      # 主接口
    │   ├── wifi_config.h      # WiFi 配置
    │   └── web_page.h         # HTML 页面 (从 basic_web.h 移植)
    └── src/
        ├── wifi_remote.c      # WiFi + HTTP Server + WebSocket
        └── json_parser.c      # JSON 解析 (替代 ArduinoJson)
```

**ESP-IDF HTTP Server + WebSocket 示例:**

```c
// wifi_remote.c

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_http_server.h"
#include "cJSON.h"
#include "wifi_remote.h"

#define AP_SSID      "WHEEL_LEG"
#define AP_PASSWORD  "12345678"
#define AP_CHANNEL   1
#define MAX_STA_CONN 4

static const char *TAG = "WIFI_REMOTE";
static httpd_handle_t server = NULL;

// ==================== WiFi AP 初始化 ====================

esp_err_t wifi_init_ap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = strlen(AP_SSID),
            .channel = AP_CHANNEL,
            .password = AP_PASSWORD,
            .max_connection = MAX_STA_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
        },
    };
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "WiFi AP started. SSID:%s password:%s", AP_SSID, AP_PASSWORD);
    return ESP_OK;
}

// ==================== HTTP 页面处理 ====================

// 返回遥控网页 (从 basic_web.h 移植)
static esp_err_t root_get_handler(httpd_req_t *req) {
    extern const char web_page_html[];  // 定义在 web_page.h
    httpd_resp_set_type(req, "text/html");
    httpd_resp_send(req, web_page_html, strlen(web_page_html));
    return ESP_OK;
}

// ==================== WebSocket 处理 ====================

static esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WebSocket handshake");
        return ESP_OK;
    }
    
    // 接收 WebSocket 数据
    httpd_ws_frame_t ws_pkt;
    uint8_t *buf = NULL;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) return ret;
    
    buf = calloc(1, ws_pkt.len + 1);
    ws_pkt.payload = buf;
    ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
    
    if (ret == ESP_OK) {
        // 解析 JSON
        parse_remote_json((char *)buf);
    }
    
    free(buf);
    return ret;
}

// ==================== JSON 解析 ====================

void parse_remote_json(const char *json_str) {
    cJSON *json = cJSON_Parse(json_str);
    if (json == NULL) return;
    
    // 解析 mode
    cJSON *mode = cJSON_GetObjectItem(json, "mode");
    if (mode && strcmp(mode->valuestring, "basic") == 0) {
        
        // 保存上一次状态
        g_remote_cmd.dir_last = g_remote_cmd.dir;
        g_remote_cmd.joy_x_last = g_remote_cmd.joy_x;
        g_remote_cmd.joy_y_last = g_remote_cmd.joy_y;
        
        // 解析 dir
        cJSON *dir = cJSON_GetObjectItem(json, "dir");
        if (dir && cJSON_IsString(dir)) {
            if (strcmp(dir->valuestring, "forward") == 0) g_remote_cmd.dir = DIR_FORWARD;
            else if (strcmp(dir->valuestring, "back") == 0) g_remote_cmd.dir = DIR_BACK;
            else if (strcmp(dir->valuestring, "left") == 0) g_remote_cmd.dir = DIR_LEFT;
            else if (strcmp(dir->valuestring, "right") == 0) g_remote_cmd.dir = DIR_RIGHT;
            else if (strcmp(dir->valuestring, "jump") == 0) g_remote_cmd.dir = DIR_JUMP;
            else g_remote_cmd.dir = DIR_STOP;
        }
        
        // 解析其他参数
        cJSON *height = cJSON_GetObjectItem(json, "height");
        if (height) g_remote_cmd.height = height->valueint;
        
        cJSON *roll = cJSON_GetObjectItem(json, "roll");
        if (roll) g_remote_cmd.roll = roll->valueint;
        
        cJSON *stable = cJSON_GetObjectItem(json, "stable");
        if (stable) g_remote_cmd.go = (stable->valueint == 1);
        
        cJSON *joy_x = cJSON_GetObjectItem(json, "joy_x");
        if (joy_x) g_remote_cmd.joy_x = joy_x->valueint;
        
        cJSON *joy_y = cJSON_GetObjectItem(json, "joy_y");
        if (joy_y) g_remote_cmd.joy_y = joy_y->valueint;
        
        // 更新时间戳
        g_remote_cmd.last_update_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    }
    
    cJSON_Delete(json);
}

// ==================== 启动 HTTP Server ====================

httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;  // WebSocket 需要较大栈
    
    if (httpd_start(&server, &config) == ESP_OK) {
        // 注册页面路由
        httpd_uri_t root = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_get_handler,
        };
        httpd_register_uri_handler(server, &root);
        
        // 注册 WebSocket
        httpd_uri_t ws = {
            .uri       = "/ws",
            .method    = HTTP_GET,
            .handler   = ws_handler,
            .is_websocket = true,
        };
        httpd_register_uri_handler(server, &ws);
        
        ESP_LOGI(TAG, "HTTP Server started on port %d", config.server_port);
    }
    
    return server;
}
```

#### 与现有框架协调

WiFi 遥控任务放在 **Core 0** 运行 (与实时控制隔离):

```c
// 任务规划更新
// task_wifi_remote - 中优先级, Core 0, 非实时

void task_wifi_remote(void *pvParameters) {
    // 初始化
    wifi_init_ap();
    start_webserver();
    
    while (1) {
        // WebSocket 处理由 HTTP Server 异步完成
        // 这里可以做超时检测
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - g_remote_cmd.last_update_ms > 1000) {
            // 1秒无数据，安全停止
            g_remote_cmd.go = false;
        }
        
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
```

#### HTML 页面移植

`basic_web.h` 可以直接复用，只需修改 WebSocket 连接地址：

```javascript
// 原 Arduino 版本
socket = new WebSocket('ws://' + window.location.hostname + ':81/');

// ESP-IDF 版本 (使用同一端口)
socket = new WebSocket('ws://' + window.location.hostname + '/ws');
```

#### 数据流

```
┌─────────────────┐      WebSocket       ┌──────────────────────┐
│   手机浏览器    │ ─────────────────────► │   task_wifi_remote   │
│ (192.168.4.1)  │      JSON 数据        │      (Core 0)        │
└─────────────────┘                       └──────────┬───────────┘
                                                     │
                                           g_remote_cmd (共享变量)
                                                     │
                                                     ▼
┌────────────────────────────────────────────────────────────────┐
│                      task_state_machine (Core 0)               │
│                   读取 g_remote_cmd, 执行状态转换               │
└────────────────────────────────────────────────────────────────┘
                                                     │
                                                     ▼
┌────────────────────────────────────────────────────────────────┐
│                    task_balance_ctrl (Core 1)                  │
│         根据 g_remote_cmd.joy_y 计算目标速度                    │
│         根据 g_remote_cmd.joy_x 计算转向                        │
└────────────────────────────────────────────────────────────────┘
```

### 6.3 algorithm/ - 算法层

#### balance_ctrl (平衡控制)

**职责**:
- LQR 或 PID 平衡控制器
- 轮式倒立摆模型
- 输出电机控制量

```c
// 平衡控制器接口
typedef struct {
    float kp_angle;      // 角度 P 增益
    float kd_angle;      // 角度 D 增益 (角速度)
    float kp_speed;      // 速度 P 增益
    float ki_speed;      // 速度 I 增益
    float kp_turn;       // 转向 P 增益
} balance_params_t;

void balance_ctrl_init(const balance_params_t *params);
void balance_ctrl_update(const imu_data_t *imu, float target_speed, float target_turn,
                         float *left_wheel_out, float *right_wheel_out);
```

#### leg_kinematics (腿部运动学) - v2.3 实现

**文件位置:**
- `components/algorithm/include/leg_kinematics.h`
- `components/algorithm/src/leg_kinematics.c`

**机构说明:**
```
       机身 (Body)
          │
   ┌──────┴──────┐
   │  Hip Motor  │  ← 大腿电机 (θ1)
   └──────┬──────┘
          │ L1 = 0.10m (大腿长度)
          │
   ┌──────┴──────┐
   │ Knee Motor  │  ← 小腿电机 (θ2)
   └──────┬──────┘
          │ L2 = 0.10m (小腿长度)
          │
   ┌──────┴──────┐
   │ Wheel Motor │  ← 轮电机
   └─────────────┘
```

**工作空间定义:**
- **腿长** (leg_length): 大腿电机轴心到轮电机轴心的直线距离 [0.07m ~ 0.17m]
- **身体夹角** (body_angle): 该直线与机身垂直向下方向的夹角 [-160° ~ -20°]，垂直向下=-90°

**电机零点偏移:**
| 腿 | 髋关节偏移 | 膝关节偏移 | 说明 |
|---|---|---|---|
| 左腿 | -25° | +120° | 电机Hip=-115°,Knee=30°时腿垂直向下且大小腿呈90° |
| 右腿 | +25° | -120° | 镜像对称 |

**运动学角度定义:**
- `theta1`: 大腿相对机身的角度，向前为正
- `theta2`: 小腿相对大腿的角度，0°=伸直，顺时针(弯曲)为负

**API 函数:**

```c
// 获取默认腿部参数
void leg_kin_get_default_params(bool is_left, leg_kin_params_t *params);

// 正运动学: 关节空间 -> 工作空间
// 输入: 电机角度 (hip_angle, knee_angle)
// 输出: 腿长 + 身体夹角
esp_err_t leg_kin_forward(const leg_joint_state_t *joint, bool is_left,
                          const leg_kin_params_t *params,
                          leg_workspace_state_t *workspace);

// 逆运动学: 工作空间 -> 关节空间
// 输入: 目标腿长 + 身体夹角
// 输出: 电机角度 (hip_angle, knee_angle)
esp_err_t leg_kin_inverse(const leg_workspace_state_t *workspace, bool is_left,
                          const leg_kin_params_t *params,
                          leg_joint_state_t *joint);

// 检查工作空间目标是否可达
bool leg_kin_is_reachable(float leg_length, float body_angle, 
                          const leg_kin_params_t *params);

// 限制工作空间目标到可达范围
void leg_kin_clamp_workspace(float *leg_length, float *body_angle,
                             const leg_kin_params_t *params);

// 计算雅可比矩阵 (用于 VMC)
// J[0]=∂L/∂θ1, J[1]=∂L/∂θ2, J[2]=∂α/∂θ1, J[3]=∂α/∂θ2
esp_err_t leg_kin_jacobian(const leg_joint_state_t *joint, bool is_left,
                           const leg_kin_params_t *params, float J[4]);
```

**数学公式:**

正运动学:
```
θ1 = hip_motor - hip_offset    (大腿相对机身角度)
θ2 = knee_motor - knee_offset  (小腿相对大腿角度, 0=伸直)

L = sqrt(L1² + L2² + 2·L1·L2·cos(θ2))     (余弦定理)
β = atan2(L2·sin(θ2), L1 + L2·cos(θ2))    (小腿引起的角度偏移)
α = θ1 + β                                (身体夹角)
```

逆运动学:
```
θ2 = acos((L² - L1² - L2²) / (2·L1·L2))   (余弦定理求膝关节角)
β = atan2(L2·sin(θ2), L1 + L2·cos(θ2))
θ1 = α - β

hip_motor = θ1 + hip_offset
knee_motor = θ2 + knee_offset
```

**应用层封装 (balance_test.c):**

```c
// 初始化腿部控制
void leg_ctrl_init(void);

// 读取当前腿部状态 (从编码器)
esp_err_t leg_ctrl_get_state(bool is_left, leg_state_t *state);

// 设置目标腿部状态 (腿长 + 身体夹角)
esp_err_t leg_ctrl_set_target(bool is_left, float leg_length, float body_angle);

// 设置双腿目标状态
esp_err_t leg_ctrl_set_both(float left_length, float left_angle,
                             float right_length, float right_angle);

// 打印当前腿部状态
void leg_ctrl_print_status(void);
```

**串口命令:**

| 命令 | 说明 |
|------|------|
| `balance leg on/off` | 使能/禁用腿部电机 |
| `balance leg status` | 查询腿部状态 (输出 LEG_STATE 格式) |
| `balance leg set <lh> <lk> <rh> <rk>` | 直接设置电机角度 (度) |
| `balance leg target <L> <A> [left\|right\|both]` | 运动学目标设置 |
| `balance leg speed <rpm>` | 设置腿部移动速度 |
| `balance leg test fk <hip> <knee> [left\|right]` | 正运动学测试 |
| `balance leg test ik <L> <A> [left\|right]` | 逆运动学测试 |

**数据输出格式 (供 Qt 面板解析):**

```
LEG_STATE: L_Len=0.150 L_Ang=0.0 L_Hip=-105.0 L_Knee=60.0 R_Len=0.150 R_Ang=0.0 R_Hip=105.0 R_Knee=-60.0
FK (left): Hip=-105.0, Knee=60.0 -> Length=0.150m, Angle=0.0deg
IK (left): Length=0.150m, Angle=0.0deg -> Hip=-105.0, Knee=60.0
```

### 6.3 control/ - 控制逻辑层

#### leg_controller (单腿控制器)

**职责**:
- 管理单条腿的 3 个电机
- 腿长控制
- 足端轨迹跟踪

```c
// 初始化腿部控制器
esp_err_t leg_init(leg_t leg_id);  // LEG_LEFT 或 LEG_RIGHT

// 关节角度控制
esp_err_t leg_set_joint_angles(leg_t leg_id, float hip_angle, float knee_angle);

// 足端位置控制
esp_err_t leg_set_foot_position(leg_t leg_id, float x, float y);

// 轮速控制
esp_err_t leg_set_wheel_speed(leg_t leg_id, float speed_rpm);

// 获取状态
leg_state_t leg_get_state(leg_t leg_id);
```

#### robot_controller (整机控制器)

**职责**:
- 协调左右腿
- 整机平衡控制
- 运动模式管理

---

## 七、ESP-IDF 组件使用指南

### 7.1 TWAI (CAN) 驱动

ESP-IDF 原生支持 CAN (TWAI) 驱动。

**头文件**:
```c
#include "driver/twai.h"
```

**初始化示例** (参考你的 Arduino 代码):
```c
#include "driver/twai.h"

esp_err_t can_bus_init(gpio_num_t tx_pin, gpio_num_t rx_pin) {
    // 通用配置
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_pin, rx_pin, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 10;
    g_config.tx_queue_len = 10;
    
    // 波特率配置 - 1Mbps
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    
    // 过滤器配置 - 接收所有帧
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    // 安装驱动
    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    
    // 启动 TWAI
    ESP_ERROR_CHECK(twai_start());
    
    return ESP_OK;
}
```

**发送帧**:
```c
esp_err_t can_send_frame(uint32_t id, uint8_t *data, uint8_t len) {
    twai_message_t msg = {
        .identifier = id,
        .data_length_code = 8,
        .flags = TWAI_MSG_FLAG_NONE,  // 标准帧
    };
    memcpy(msg.data, data, len);
    
    return twai_transmit(&msg, pdMS_TO_TICKS(100));
}
```

**接收帧**:
```c
esp_err_t can_receive_frame(uint32_t *id, uint8_t *data, uint32_t timeout_ms) {
    twai_message_t msg;
    esp_err_t ret = twai_receive(&msg, pdMS_TO_TICKS(timeout_ms));
    
    if (ret == ESP_OK) {
        *id = msg.identifier;
        memcpy(data, msg.data, msg.data_length_code);
    }
    return ret;
}
```

### 7.2 双路 CAN 实现方案

ESP32-S3 只有 **1 个 TWAI 控制器**，要实现双路 CAN 有以下方案：

#### 方案 A: 软件分时复用 (不推荐)
切换 GPIO 引脚，分时使用单个控制器。延迟高，不适合实时控制。

#### 方案 B: 外部 CAN 控制器
使用 SPI 接口的外部 CAN 控制器如 MCP2515。

#### 方案 C: 使用 ESP32-S3 的 TWAI + 软件 CAN
一路硬件 TWAI，另一路用 GPIO 模拟 (复杂度高)。

#### ✅ 方案 D: 单 CAN 总线 + 不同电机 ID (已采用)
将所有 6 个电机接到同一条 CAN 总线，通过不同 ID 区分。

**当前配置:**
- CAN 引脚: TX=IO18, RX=IO17
- 左腿电机 ID: 1, 2, 3
- 右腿电机 ID: 4, 5, 6
- 波特率: 1Mbps

### 7.3 I2C 驱动

**头文件**:
```c
#include "driver/i2c.h"
// 或使用新版 API
#include "driver/i2c_master.h"
```

**I2C 初始化示例**:
```c
#include "driver/i2c_master.h"

i2c_master_bus_handle_t i2c1_bus;
i2c_master_bus_handle_t i2c2_bus;

void i2c_init(void) {
    // I2C1 - SHT30
    i2c_master_bus_config_t i2c1_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = GPIO_NUM_1,
        .sda_io_num = GPIO_NUM_2,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,  // 外部已有上拉
    };
    i2c_new_master_bus(&i2c1_config, &i2c1_bus);
    
    // I2C2 - IMU
    i2c_master_bus_config_t i2c2_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_1,
        .scl_io_num = GPIO_NUM_11,
        .sda_io_num = GPIO_NUM_12,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = false,
    };
    i2c_new_master_bus(&i2c2_config, &i2c2_bus);
}
```

### 7.4 GPIO 配置

```c
#include "driver/gpio.h"

void gpio_init(void) {
    // 按键输入 (外部上拉)
    gpio_config_t btn_config = {
        .pin_bit_mask = (1ULL << GPIO_NUM_13) | (1ULL << GPIO_NUM_9),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,    // 外部已有上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE,       // 下降沿中断
    };
    gpio_config(&btn_config);
    
    // 电源检测输入
    gpio_config_t pwr_config = {
        .pin_bit_mask = (1ULL << GPIO_NUM_3),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&pwr_config);
    
    // 电机供电控制输出
    gpio_config_t motor_pwr_config = {
        .pin_bit_mask = (1ULL << GPIO_NUM_8),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&motor_pwr_config);
    gpio_set_level(GPIO_NUM_8, 0);  // 初始断电
}
```

### 7.5 UART (树莓派通信)

```c
#include "driver/uart.h"

void uart_rpi_init(void) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    
    uart_driver_install(UART_NUM_1, 256, 256, 0, NULL, 0);
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, GPIO_NUM_4, GPIO_NUM_5, -1, -1);
}
```

---

## 八、参考资源

### 8.1 原有 Arduino 项目

#### CAN 电机驱动
仓库地址: https://github.com/Bubble252/esp_can_driver

关键文件:
- `lib/CANServo/CANServo.h` - API 定义
- `lib/CANServo/CANServo.cpp` - 驱动实现
- `lib/CANServo/CANServoRegs.h` - 寄存器定义
- `doc/指令说明.md` - 通信协议详细说明

#### WiFi 遥控器 (低自由度轮足)
仓库地址: https://github.com/Bubble252/shibo_wheel_leg

关键文件:
- `src/basic_web.h` - HTML 遥控页面 (含虚拟摇杆)
- `src/wifi_config.cpp` - WiFi AP/STA 配置
- `src/robot.h/.cpp` - 遥控数据结构 & JSON 解析
- `src/wl_pro_robot_freertos.cpp` - FreeRTOS 多任务版本 (参考任务分配)

### 8.2 ESP-IDF 官方文档

- [TWAI 驱动](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/twai.html)
- [I2C 驱动](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/i2c.html)
- [GPIO 驱动](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/gpio.html)
- [UART 驱动](https://docs.espressif.com/projects/esp-idf/en/latest/esp32s3/api-reference/peripherals/uart.html)

### 8.3 传感器文档

- **维特智能 IMU**: 参考官方 SDK 和数据手册
- **SHT30 温湿度传感器**: I2C 地址 0x44 或 0x45

---

## 九、平衡控制测试模块 (Balance Test)

> 创建日期: 2026-01-17
> 参考: shibo_wheel_leg 项目的 wl_pro_robot_freertos.cpp

### 9.1 功能说明

平衡控制测试模块是一个简化版的平衡控制实现，仅控制**轮电机** (ID=3, ID=6)，不涉及腿部关节电机。用于测试和调试 LQR 平衡算法。

### 9.2 FreeRTOS 任务架构

```
┌─────────────────────────────────────────────────────────────┐
│ Core 0 (低优先级)                                           │
│   task_remote_watchdog - 遥控超时检测 (优先级 8, 100ms)     │
│   (WiFi/WebSocket 由 HTTP Server 异步处理)                  │
├─────────────────────────────────────────────────────────────┤
│ Core 1 (高优先级, 实时)                                      │
│   task_imu_read       - IMU 数据读取 (优先级 22, 2ms)       │
│   task_balance_ctrl   - LQR 平衡控制 (优先级 24, 5ms)       │
│   task_motor_comm     - CAN 电机通信 (优先级 20, 2ms)       │
└─────────────────────────────────────────────────────────────┘
```

### 9.3 使用方法

**串口命令:**

```bash
# 初始化模块
balance init

# 启动测试 (创建任务, 启动 WiFi AP)
balance start

# 此时连接 WiFi: SSID=WL-PRO, 密码=12345678
# 打开浏览器访问 http://192.168.4.1
# 打开 "Robot Go!" 开关使能平衡控制

# 手动使能/禁用 (也可以通过网页开关)
balance enable
balance disable

# 紧急停止
balance estop

# 复位紧急状态
balance reset

# 查看状态
balance status

# 设置角度零点 (根据实际机器人重心调整)
balance zero 7.4

# 停止测试
balance stop
```

**快捷命令别名:**
- `bal` = `balance`

### 9.4 控制逻辑

参考 `shibo_wheel_leg/src/wl_pro_robot_freertos.cpp`:

```c
// ==================== LQR 平衡算法 ====================
// LQR_u = angle_control + gyro_control + distance_control + speed_control
//
// 各控制环说明:
//   angle_control    = PID(pitch - angle_zeropoint)  // 角度环
//   gyro_control     = PID(pitch_rate)               // 角速度环
//   distance_control = PID(lqr_distance - distance_zeropoint)  // 位移环
//   speed_control    = PID(lqr_speed - target_speed) // 速度环

// ==================== 位移/速度计算 ====================
// 电机方向: 右轮顺时针为负机器人前进，左轮逆时针为负机器人前进
// 所以两轮都为负时，机器人前进；取负后位移为正表示前进
// 位移公式: s = r * θ (θ为弧度, r为轮子半径)
// 速度公式: v = r * ω (ω为角速度 rad/s)
// 轮子半径: WHEEL_RADIUS_M = 0.03m (直径60mm)
g_lqr_distance = (-0.5f) * (left_pos_rad + right_pos_rad) * WHEEL_RADIUS_M;  // 单位: m
g_lqr_speed = (-0.5f) * (left_vel_rad + right_vel_rad) * WHEEL_RADIUS_M;     // 单位: m/s

// ==================== YAW 方向保持控制 ====================
// 有转向输入时: 角速度控制模式
//   yaw_control = PID(yaw_rate - target_yaw_rate)
// 松手时: 方向保持模式
//   yaw_control = PID(yaw_total - yaw_target) + PID(yaw_rate)
//   (自动锁定当前方向)

// ==================== 最终电机输出 (力矩模式) ====================
// 左轮: LQR_u + YAW_output
// 右轮: LQR_u - YAW_output

// ==================== 遥控输入映射 ====================
// joy_y (-100~100) -> target_speed = joy_y * 0.1
// joy_x (-100~100) -> target_yaw_rate = joy_x * 0.02
```

### 9.5 运动细节优化

```c
// 1. 开始移动时: 重置位移零点和积分 (仅一次)
if (joy_y 从0变为非0) {
    distance_zeropoint = lqr_distance;
    lqr_reset();  // 重置积分
}

// 2. 移动过程中: 持续更新位移零点 (防止位移环干扰速度控制)
if (joy_y != 0) {
    distance_zeropoint = lqr_distance;
}

// 3. 松手停车: 等速度降低后锁定位置
if (松手 && 速度 < 0.5) {
    distance_zeropoint = lqr_distance;
}

// 4. 被推动时: 重置位移零点 (防止猛烈反应)
if (速度 > 15.0) {
    distance_zeropoint = lqr_distance;
}

// 5. 失控恢复: 角度恢复后延迟0.5秒再恢复控制
if (失控 && 角度 < 10°) {
    计数器++;
    if (计数器 > 100) 恢复控制;
} else if (角度 > 10°) {
    计数器 = 1;  // 重置，防止意外恢复
}
```

### 9.6 关键参数

| 参数 | 默认值 | 说明 |
|------|--------|------|
| angle_zeropoint | 7.4° | 俯仰角零点，根据机器人重心调整 |
| emergency_angle | 45° | 紧急停止角度阈值 |
| REMOTE_TIMEOUT_MS | 1000ms | 遥控超时时间 |
| BALANCE_CTRL_PERIOD_MS | 5ms | 平衡控制周期 (200Hz) |
| IMU_READ_PERIOD_MS | 2ms | IMU 读取周期 (500Hz) |

### 9.7 LQR 默认 PID 参数

| 控制环 | Kp | Ki | Kd | Limit | 说明 |
|--------|-----|-----|-----|-------|------|
| angle | 0.7 | 0.55 | 0.001 | 10.0 | 角度环 |
| gyro | 0.05 | 0.0 | 0.0 | 8.0 | 角速度环 |
| distance | 0.5 | 0.0 | 0.0 | 8.0 | 位移环 |
| speed | 0.7 | 0.0 | 0.0 | 8.0 | 速度环 |
| lqr_u | 1.0 | 8.5 | 0.0 | 8.0 | LQR输出积分 |
| yaw_angle | 0.3 | 0.0 | 0.0 | 5.0 | YAW角度环(方向保持) |
| yaw_gyro | 0.1 | 0.0 | 0.0 | 3.0 | YAW角速度环 |
| roll | 0.2 | 0.0 | 0.0 | 5.0 | Roll环(腿长控制用) |

### 9.8 数据流

```
┌─────────────────┐      WebSocket       ┌──────────────────────┐
│   手机浏览器    │ ─────────────────────► │   HTTP Server       │
│ (192.168.4.1)  │      JSON 数据        │   (异步回调)          │
└─────────────────┘                       └──────────┬───────────┘
                                                     │
                                           g_remote_data (共享)
                                                     │
                                                     ▼
┌────────────────────────────────────────────────────────────────┐
│                      task_balance_ctrl (Core 1, 200Hz)         │
│   1. 从 g_imu_data 读取姿态                                     │
│   2. 从 g_wheel_state 读取轮速                                  │
│   3. 从 g_remote_data 读取遥控命令                              │
│   4. 执行 LQR 平衡计算                                          │
│   5. 写入 g_wheel_cmd (力矩命令)                                │
└────────────────────────────────────────────────────────────────┘
                                                     │
                                                     ▼
┌────────────────────────────────────────────────────────────────┐
│                      task_motor_comm (Core 1, 500Hz)           │
│   1. 处理 CAN 接收 (更新 g_wheel_state)                         │
│   2. 发送力矩命令到轮电机 (ID=3, ID=6)                          │
└────────────────────────────────────────────────────────────────┘
```

### 9.9 调试技巧

1. **角度零点调整**: 机器人静止时应保持垂直，如有倾斜调整 `balance zero <deg>`
2. **WiFi 连接问题**: 检查手机是否正确连接到 WL-PRO 网络
3. **电机无响应**: 检查 `mpower on` 是否已执行，CAN 总线是否正常
4. **紧急停止触发**: 倾斜超过45°会自动停止，需要 `balance reset` 后重新 `balance enable`

### 9.10 电机零点设置

测试前需要将电机当前位置设为零点（原点），确保位移计算正确：

```bash
# 设置轮电机零点
balance mzero left      # 设置左轮 (ID=3) 当前位置为零点
balance mzero right     # 设置右轮 (ID=6) 当前位置为零点
balance mzero all       # 同时设置两个轮电机零点

# 设置任意电机零点 (按ID)
balance mzero 1         # 设置电机 ID=1 (左大腿) 零点
balance mzero 4         # 设置电机 ID=4 (右大腿) 零点
```

**推荐测试流程**:
1. 手动将机器人摆到期望的初始姿态
2. 执行 `balance mzero all` 设置轮电机零点
3. 执行 `balance init` 和 `balance start`
4. 通过网页或 `balance enable` 使能平衡

### 9.11 调参命令 (Qt调参面板)

通过串口发送 Commander 命令可实时调整 PID 参数：

```bash
# 格式: @<控制器ID><参数><值>
# 控制器ID: A=角度, B=角速度, C=位移, D=速度, E=YAW角度, F=YAW角速度, H=LQR_U
# 参数: P=Kp, I=Ki, D=Kd, L=Limit

# 示例
@AP0.8    # 设置角度环 Kp = 0.8
@BI0.6    # 设置角速度环 Ki = 0.6
@DL10.0   # 设置速度环 Limit = 10.0

# 波形输出控制
balance plot on     # 开启波形输出
balance plot off    # 关闭波形输出
balance plot div 5  # 设置输出分频 (200Hz / 5 = 40Hz)
```

---

## 十、开发计划 (TODO)

### Phase 1: 基础框架
- [x] 创建 components 目录结构
- [x] 实现 common/config.h 引脚定义
- [x] 移植 CAN 电机驱动到 ESP-IDF (device/can_motor)
- [x] 实现 IMU 驱动 (device/imu_driver)
- [x] 实现 GPIO 驱动 (按键、电源检测、电机供电)

### Phase 2: WiFi 遥控器移植 (来自 shibo_wheel_leg)
- [x] 移植 WiFi AP 初始化 (wifi_config.cpp → wifi_remote.c)
- [x] 移植 HTTP Server (替代 WebServer.h)
- [x] 移植 WebSocket (替代 WebSocketsServer.h)
- [x] 移植 JSON 解析 (ArduinoJson → cJSON)
- [x] 移植 HTML 页面 (basic_web.h → web_page.h)
- [x] 测试 Web 遥控器连接

### Phase 3: 控制算法
- [x] 实现腿部正逆运动学 (algorithm/leg_kinematics) ← **v2.3 完成**
- [x] 实现滤波器 (algorithm/filter)
- [x] 实现 LQR 平衡控制算法 (algorithm/lqr_balance)
- [x] 实现平衡测试模块 (app/balance_test) ← **NEW**
- [ ] 实现腿部控制器 (control/leg_controller) → VMC 待实现

### Phase 4: 任务管理
- [x] 创建 FreeRTOS 任务框架 (balance_test.c)
- [ ] 实现完整状态机 (app/state_machine)
- [ ] 实现安全监控 (app/safety_monitor)

### Phase 5: 通信与集成
- [ ] 定义树莓派通信协议
- [ ] 实现 UART 通信模块
- [ ] 整机测试与调试

### Phase 6: Qt 调参面板 (已完成)
- [x] 移植 pid_tuner.py 框架
- [x] 实现 Commander 协议解析器 (device/commander_parser)
- [x] 实现 PID/LPF 参数调节面板
- [x] 实现实时波形显示
- [x] 实现 Web 遥控器状态监控
- [x] 新增设备控制面板 (电机/IMU/平衡/传感器/终端)

---

## 十一、修改历史

| 日期 | 版本 | 修改内容 |
|------|------|----------|
| 2026-01-15 | v1.0 | 初始版本，硬件配置与代码框架规划 |
| 2026-01-15 | v1.1 | 添加分层架构、FreeRTOS 任务设计、状态机 |
| 2026-01-15 | v1.2 | 添加 WiFi 遥控器模块移植方案 (来自 shibo_wheel_leg) |
| 2026-01-15 | v1.3 | 采用单 CAN 总线方案，更新电机 ID 分配 (左腿 1-3，右腿 4-6) |
| 2026-01-16 | v1.4 | 添加 CAN 帧格式详解，明确大端序字节顺序 |
| 2026-01-17 | v1.5 | 添加平衡控制测试模块 (balance_test)，参考 shibo_wheel_leg |
| 2026-01-21 | v1.6 | 完善 LQR 控制逻辑：修复位移计算、添加 YAW 方向保持、优化运动细节 |
| 2026-01-21 | v1.7 | 代码审查修复：去掉0.5系数、修复每帧重置积分、修复恢复检测逻辑 |
| 2026-01-22 | v1.8 | 添加电机零点设置功能 (setOrigin)，新增 `balance mzero` 命令 |
| 2026-01-22 | v1.9 | 完善电机驱动API：新增 PVT指令、相对位置、低速模式、电压/错误读取、擦除参数等 |
| 2026-01-22 | v2.0 | 新增 Qt 调参面板与 Commander 通信协议完整文档 |
| 2026-01-22 | v2.1 | 位移/速度计算加入轮子半径 (r=0.03m)，单位改为 m 和 m/s |
| 2026-01-24 | v2.2 | 串口命令处理改为独立任务 (console_task)，支持与平衡控制并行运行 |
| 2026-01-24 | v2.3 | 新增腿部运动学模块 (leg_kinematics)，更新 Qt 腿部控制面板 |
| 2026-01-24 | v2.3.1 | 新增 Qt 平衡面板环路调试控制 (单环/组合调试) |
| 2026-01-26 | v2.4 | 新增 YAW 调试面板，修复 YAW 使能时角度锁定问题 |
| 2026-01-26 | v2.5 | 扩展波形通道 (E/F/G/I/J/L/Y)，LPF 面板显示滤波前后对比 |

---

## 十二、Qt 调参面板与 Commander 通信协议

> 创建日期: 2026-01-22
> 文件位置: `tools/qt_tuner/pid_tuner.py`
> 设备端实现: `components/device/src/commander_parser.c`

### 12.1 系统架构概述

Qt 调参面板是一个 PyQt5 桌面应用，通过串口与 ESP32 进行双向通信，实现:
- **参数调节**: 实时修改 LQR 平衡控制器的 PID/LPF 参数
- **波形显示**: 实时绘制控制过程的目标值和实际值
- **设备控制**: 替代命令行的图形化设备控制界面
- **状态监控**: 显示 Web 遥控器、IMU、电机等设备状态

```
┌─────────────────────────────────────────────────────────────────────────┐
│                         Qt 调参面板 (pid_tuner.py)                       │
│  ┌──────────────┐ ┌──────────────┐ ┌──────────────┐ ┌──────────────┐    │
│  │ PID 调参面板 │ │ 设备控制面板 │ │ 波形显示面板 │ │ 终端命令面板 │    │
│  │ A-M 控制器  │ │ 电机/IMU/...│ │ pyqtgraph   │ │ 直接发送命令 │    │
│  └──────────────┘ └──────────────┘ └──────────────┘ └──────────────┘    │
└────────────────────────────────────┬────────────────────────────────────┘
                                     │ 串口 (115200 bps)
                                     │ 命令: <ID><Param><Value>\n
                                     │ 波形: #DATA,<ID>,<Target>,<Control>\n
                                     ▼
┌─────────────────────────────────────────────────────────────────────────┐
│                         ESP32 (balance_test.c)                          │
│  ┌──────────────────────────────────────────────────────────────────┐  │
│  │              Commander 解析器 (commander_parser.c)                │  │
│  │   - commander_param_callback(): 参数设置 -> LQR 控制器            │  │
│  │   - commander_query_callback(): 参数查询 <- LQR 控制器            │  │
│  └──────────────────────────────────────────────────────────────────┘  │
│                                    │                                    │
│                                    ▼                                    │
│  ┌──────────────────────────────────────────────────────────────────┐  │
│  │               LQR 平衡控制器 (lqr_balance.c)                       │  │
│  │   - lqr_params_t: 所有 PID/LPF 参数                               │  │
│  │   - lqr_balance_loop(): 200Hz 平衡控制循环                        │  │
│  │   - output_plot_data(): 波形数据输出                              │  │
│  └──────────────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────────────┘
```

### 12.2 Commander 通信协议

#### 12.2.1 命令格式

```
<控制器ID><参数字符><数值>\n    # 设置参数
<控制器ID>?\n                   # 查询参数
```

**示例:**
```bash
AP1.5      # 设置角度控制器的 P 值为 1.5
BI0.55     # 设置角速度控制器的 I 值为 0.55
GT0.01     # 设置摇杆Y轴滤波的 Tf 为 0.01
ML0.3      # 设置速度自适应低姿态 P 为 0.3
A?         # 查询角度控制器参数
```

#### 12.2.2 控制器 ID 定义

| ID | 名称 | 类型 | 说明 | 对应 lqr_params_t 参数 |
|----|------|------|------|------------------------|
| **A** | Angle | PID | 角度控制 | `angle_kp/ki/kd/limit` |
| **B** | Gyro | PID | 角速度控制 | `gyro_kp/ki/kd/limit` |
| **C** | Distance | PID | 位移控制 | `distance_kp/ki/kd/limit` |
| **D** | Speed | PID | 速度控制 | `speed_kp/ki/kd/limit` |
| **E** | YawAngle | PID | YAW角度控制 | `yaw_angle_kp/ki/kd/limit` |
| **F** | YawGyro | PID | YAW角速度控制 | `yaw_gyro_kp/ki/kd/limit` |
| **G** | JoyyLPF | LPF | 摇杆Y轴滤波 | `lpf_joyy_tf` |
| **H** | LqrU | PID | LQR输出补偿 | `lqr_u_kp/ki/kd/limit` |
| **I** | Zeropoint | PID | 零点自适应 | `zeropoint_kp/ki/kd/limit` |
| **J** | ZeroLPF | LPF | 零点滤波 | `lpf_zeropoint_tf` |
| **K** | RollAngle | PID | Roll轴平衡 | `roll_kp/ki/kd/limit` |
| **L** | RollLPF | LPF | Roll角度滤波 | `lpf_roll_tf` |
| **M** | SpeedAdapt | 特殊 | 速度自适应P | `speed_kp_min/max` |

#### 12.2.3 参数字符定义

**PID 控制器 (A/B/C/D/E/F/H/I/K):**
| 字符 | 说明 | 示例 |
|------|------|------|
| `P` | 比例增益 | `AP0.7` = 角度 Kp=0.7 |
| `I` | 积分增益 | `AI0.55` = 角度 Ki=0.55 |
| `D` | 微分增益 | `AD0.001` = 角度 Kd=0.001 |
| `L` | 输出限幅 | `AL10.0` = 角度 Limit=10.0 |
| `R` | 斜坡限制 | `AR1000` = 角度 Ramp=1000 |
| `?` | 查询参数 | `A?` = 查询角度控制器 |

**LPF 控制器 (G/J/L):**
| 字符 | 说明 | 示例 |
|------|------|------|
| `T` 或直接数值 | 滤波时间常数 Tf | `GT0.01` 或 `G0.01` |
| `?` | 查询参数 | `G?` = 查询摇杆滤波 |

**速度自适应 (M):**
| 字符 | 说明 | 示例 |
|------|------|------|
| `L` | Kp_Min (高姿态) | `ML0.3` = 高姿态 P=0.3 |
| `H` | Kp_Max (低姿态) | `MH1.0` = 低姿态 P=1.0 |
| `?` | 查询参数 | `M?` = 查询自适应参数 |

#### 12.2.4 响应格式

**PID 参数查询响应:**
```
PID: P: 0.7000 I: 0.5500 D: 0.0010 R: 1000.0000 L: 10.0000
```

**LPF 参数查询响应:**
```
LPF: Tf: 0.0100
```

**速度自适应参数查询响应:**
```
Speed Adaptive P: Low=0.3000 High=1.0000
```

#### 12.2.5 波形数据格式 (ESP32 → PC)

```
#DATA,<ID>,<Target>,<Control>\n
```

| 通道 | ID | Target (蓝色线) | Control (红色线) | 说明 |
|------|-----|-----------------|------------------|------|
| **A** | A | 0.0 | pitch角度 | 角度偏差 (目标=0°) |
| **B** | B | 0.0 | pitch_rate | 角速度 (目标=0) |
| **C** | C | distance_zeropoint | lqr_distance | 位移偏差 |
| **D** | D | target_speed (原始) | lqr_speed (实际) | 速度追踪 |
| **E** | E | yaw_angle_target | yaw_angle_total | YAW角度 (目标 vs 累积) |
| **F** | F | target_yaw_rate | yaw_rate | YAW角速度 (目标 vs 实际) |
| **G** | G | target_speed (滤波前) | filtered_target_speed (滤波后) | 摇杆Y滤波效果 |
| **H** | H | 0.0 | lqr_u | LQR输出 |
| **I** | I | 0.0 | distance_zeropoint | 零点偏移累积值 |
| **J** | J | zeropoint_adjust_raw | zeropoint_adjust_filtered | 零点滤波效果 |
| **K** | K | 0.0 | roll | Roll角度 |
| **L** | L | roll (滤波前) | roll_filtered (滤波后) | Roll滤波效果 |
| **Y** | Y | yaw_angle_target | yaw_angle_total | YAW专用通道 (用于YAW面板) |

**LPF 滤波面板 (G/J/L) 显示说明:**
- **蓝色线**: 滤波前的原始值
- **红色线**: 滤波后的平滑值
- 调节 Tf 参数可实时观察滤波效果变化

**YAW 调试输出格式:**
```
#YAW_DBG,out=<yaw_output>,err=<angle_error>,hold=<0/1>,rate=<yaw_rate>\n
```

**波形输出控制命令:**
```bash
balance plot on      # 开启波形输出
balance plot off     # 关闭波形输出
balance plot div 10  # 设置分频 (200Hz / 10 = 20Hz 输出)
```

#### 12.2.6 Web 遥控器状态格式 (ESP32 → PC)

```
#WEB,<go>,<dir>,<joyx>,<joyy>,<height>\n
```

| 字段 | 说明 | 示例值 |
|------|------|--------|
| go | 使能开关 | 0/1 |
| dir | 方向 | 0-5 (STOP/FWD/BACK/LEFT/RIGHT/JUMP) |
| joyx | 摇杆X (-100~100) | -50 |
| joyy | 摇杆Y (-100~100) | 80 |
| height | 腿高度 (mm) | 38 |

### 12.3 Qt 调参面板组件

#### 12.3.1 面板结构

```
pid_tuner.py
├── SerialThread            # 串口通信线程
│   ├── data_received       # 数据接收信号
│   ├── send_command()      # 发送命令
│   └── 缓冲区行处理
│
├── PIDControlPanel         # PID 参数调节面板 (9个)
│   ├── P/I/D/Limit/Ramp 输入框 + 设置按钮
│   ├── 当前参数显示
│   ├── 查询/发送全部/重置 按钮
│   └── 波形绘图区 (pyqtgraph)
│
├── LPFControlPanel         # LPF 参数调节面板 (3个: G/J/L)
│   ├── Tf 输入框 + 设置按钮
│   ├── 波形绘图区 (双曲线)
│   │   ├── 蓝色线: 滤波前原始值
│   │   └── 红色线: 滤波后平滑值
│   └── 实时对比滤波效果
│
├── YawDebugPanel           # YAW 调试面板 (v2.4 新增)
│   ├── 实时状态显示
│   │   ├── 目标角度 / 当前角度 / 误差
│   │   ├── 输出量 / 保持模式
│   │   └── 当前角速度
│   ├── 自动诊断 (连接/数据/误差/静止检测)
│   ├── YAW PID 快捷调参 (E: 角度, F: 角速度)
│   └── 控制说明 (工作模式/常见问题)
│
├── SpeedAdaptivePanel      # 速度自适应面板
│   ├── Kp_Max/Kp_Min 输入框
│   ├── 高度范围设置
│   └── 实时Kp计算预览
│
├── LegControlPanel         # 腿部控制面板 (v2.3 重构)
│   ├── 腿部电机使能 (on/off/status)
│   ├── 运动学控制 (腿长 + 身体夹角, 带滑块)
│   ├── 直接角度控制 (髋/膝电机角度)
│   ├── 预设姿态 (站立/半蹲/蹲下/伸直/前倾/后倾)
│   ├── 运动学测试 (FK/IK)
│   └── 腿部状态显示 (双腿腿长/夹角/关节角度)
│
├── WebMonitorPanel         # Web 遥控器监控面板
│   ├── Go/Dir/JoyX/JoyY/Height 显示
│   └── 命令历史
│
├── BalanceControlPanel     # 平衡控制面板 (v2.3 更新)
│   ├── 初始化控制 (balance init)
│   ├── 运行控制 (start/stop)
│   ├── 使能控制 (enable/disable/estop)
│   ├── Roll 平衡控制 (侧倾补偿)
│   ├── 角度零点设置
│   ├── 波形输出控制
│   └── 🆕 环路调试控制 (单环/组合调试)
│       ├── 快捷预设 (Full/Simple/None)
│       ├── LQR 各环独立开关 (A:角度/B:角速度/C:位移/D:速度/H:总输出)
│       ├── Yaw 转向控制
│       └── 增益精调 (0.0-1.0)
│
├── MotorControlPanel       # 电机控制面板 (新增)
│   ├── 全局控制 (扫描/使能/停止/读取)
│   ├── 单电机控制 (模式/速度/位置/力矩)
│   ├── 高级操作 (设零点/保存/重启/错误码)
│   └── 电机状态表
│
├── IMUControlPanel         # IMU 控制面板 (新增)
│   ├── 初始化控制
│   ├── 数据读取 (开始/停止/单次)
│   ├── 输出速率设置
│   ├── 校准 (加速度计/磁力计)
│   └── IMU 数据显示
│
├── SensorPanel             # 传感器控制面板 (新增)
│   ├── 电源监控 (读取/电机供电开关)
│   ├── 按键控制
│   ├── 温湿度传感器 (SHT30)
│   └── WiFi 遥控器控制
│
├── TerminalPanel           # 终端面板 (新增)
│   ├── 命令输入框
│   └── 快捷命令按钮
│
└── PIDTunerUI              # 主窗口
    ├── 串口连接控制
    ├── Tab 页面切换
    ├── 日志显示
    └── process_serial_data()  # 数据解析分发
```

#### 12.3.2 数据流

```
用户操作 (点击设置按钮)
    │
    ▼
PIDControlPanel.set_param('P', 0.7)
    │
    ▼
PIDTunerUI.send_command("AP0.7")
    │
    ▼
SerialThread.send_command("AP0.7\n")
    │
    ▼ 串口发送
────────────────────────────────────────────────────
    │
    ▼ ESP32 接收
commander_process_line("AP0.7")
    │
    ▼
commander_param_callback('A', 'P', 0.7)
    │
    ▼
lqr_set_params(&g_lqr_ctrl, &params)  // 更新 LQR 控制器参数
    │
    ▼
printf("[COMMANDER] Angle.P = 0.7000\n")  // 日志输出
```

```
ESP32 波形输出 (200Hz / divider)
    │
    ▼
output_plot_data()
    │
    ▼
printf("#DATA,A,0.0,%.2f\n", pitch)
    │
    ▼ 串口发送
────────────────────────────────────────────────────
    │
    ▼ Qt 接收
SerialThread.data_received.emit("#DATA,A,0.0,5.32")
    │
    ▼
PIDTunerUI.process_serial_data()
    │
    ▼
pid_panels['angle'].update_plot(0.0, 5.32)
    │
    ▼
pyqtgraph 实时更新波形
```

### 12.4 ESP32 端 Commander 实现

#### 12.4.1 初始化流程

```c
// balance_test.c -> balance_test_init()

// 初始化 Commander 解析器
// - set_callback: 收到设置命令时更新 LQR 参数
// - query_callback: 收到查询命令时返回 LQR 实际参数
commander_parser_init(commander_param_callback, commander_query_callback);
```

#### 12.4.2 参数设置回调

```c
// balance_test.c

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
        // ... 其他控制器
    }
    
    if (updated) {
        lqr_set_params(&g_lqr_ctrl, &params);  // 同步到 LQR 控制器
    }
}
```

#### 12.4.3 参数查询回调

```c
// balance_test.c

static bool commander_query_callback(char controller_id, commander_pid_params_t *params)
{
    if (!g_initialized || !params) return false;
    
    const lqr_params_t *p = &g_lqr_ctrl.params;  // 从 LQR 控制器读取
    
    switch (controller_id) {
        case CTRL_ID_ANGLE:  // A - 角度控制
            params->p = p->angle_kp;
            params->i = p->angle_ki;
            params->d = p->angle_kd;
            params->limit = p->angle_limit;
            return true;
        // ... 其他控制器
    }
    return false;
}
```

#### 12.4.4 波形数据输出

```c
// balance_test.c

static void output_plot_data(const lqr_input_t *input, const lqr_output_t *output) {
    if (!g_plot_enabled) return;
    
    g_plot_counter++;
    if (g_plot_counter < g_plot_divider) return;
    g_plot_counter = 0;
    
    // 输出各通道数据
    printf("#DATA,A,0.0,%.2f\n", input->pitch);
    printf("#DATA,B,0.0,%.2f\n", input->pitch_rate);
    printf("#DATA,C,%.2f,%.2f\n", g_distance_zeropoint, g_lqr_distance);
    printf("#DATA,D,%.2f,%.2f\n", input->target_speed, input->lqr_speed);
    printf("#DATA,H,0.0,%.2f\n", g_last_lqr_u);
    printf("#DATA,K,0.0,%.2f\n", input->roll);
}
```

### 12.5 使用流程

#### 12.5.1 快速开始

```bash
# 1. 启动 Qt 调参面板
cd tools/qt_tuner
python3 pid_tuner.py

# 2. 连接串口 (选择 /dev/ttyACM0 或对应端口)

# 3. 初始化平衡系统 (切换到"平衡控制"标签页)
#    点击 "初始化平衡系统" 按钮
#    或在终端面板输入: balance init

# 4. 启动平衡测试
#    点击 "启动" 按钮
#    或输入: balance start

# 5. 开启波形输出
#    在"平衡控制"页面点击 "开启波形"
#    或输入: balance plot on

# 6. 切换到需要调节的 PID 标签页 (如 A-角度PID)
#    点击 "查询当前参数" 获取设备当前值
#    修改参数并点击 "设置" 按钮

# 7. 观察波形变化，迭代调参
```

#### 12.5.2 调参技巧

1. **从 P 开始**: 先调 P 值，从小增大，直到出现轻微振荡
2. **加入 I**: 增加 I 值消除稳态误差，但注意积分饱和
3. **加入 D**: 增加 D 值抑制振荡，提高响应速度
4. **调整 Limit**: 根据电机能力设置输出限幅
5. **观察波形**: 关注超调量、调节时间、稳态误差
6. **轻推测试**: 轻推机器人观察恢复情况

#### 12.5.3 LQR 默认参数 (参考值)

| 控制器 | Kp | Ki | Kd | Limit | 说明 |
|--------|-----|-----|-----|-------|------|
| A-Angle | 0.7 | 0.55 | 0.001 | 10.0 | 角度控制 (核心) |
| B-Gyro | 0.05 | 0.0 | 0.0 | 8.0 | 角速度阻尼 |
| C-Distance | 0.5 | 0.0 | 0.0 | 8.0 | 位移控制 |
| D-Speed | 0.7 | 0.0 | 0.0 | 8.0 | 速度跟踪 |
| E-YawAngle | 0.3 | 0.0 | 0.0 | 5.0 | 方向保持 |
| F-YawGyro | 0.1 | 0.0 | 0.0 | 3.0 | 转向阻尼 |
| H-LqrU | 1.0 | 8.5 | 0.0 | 8.0 | LQR输出积分 |

### 12.6 设备控制面板命令映射

#### 12.6.1 平衡控制面板

| 界面按钮 | 发送命令 | 说明 |
|----------|----------|------|
| 初始化平衡系统 | `balance init` | 初始化 CAN、IMU、LQR、WiFi |
| 启动 | `balance start` | 创建任务、启动 WiFi AP |
| 停止 | `balance stop` | 删除任务、停止电机 |
| 使能平衡 | `balance enable` | 进入闭环控制 |
| 禁用平衡 | `balance disable` | 退出闭环、电机清零 |
| 紧急停止 | `balance estop` | 立即停止电机 |
| 重置 | `balance reset` | 复位紧急状态 |
| 状态 | `balance status` | 打印当前状态 |
| 设置零点 | `balance zero <deg>` | 设置俯仰角零点 |
| 开启波形 | `balance plot on` | 使能波形输出 |
| 关闭波形 | `balance plot off` | 禁用波形输出 |
| 设置分频 | `balance plot div <N>` | 波形输出分频 |
| Roll 开启 | `balance roll on` | 开启侧倾补偿 |
| Roll 关闭 | `balance roll off` | 关闭侧倾补偿 |
| 环路全开 | `balance loop full` | 启用所有环路 (ABCDHY) |
| 环路简单 | `balance loop simple` | 仅角度+角速度环 (AB) |
| 环路全关 | `balance loop none` | 关闭所有环路 |
| 环路状态 | `balance loop status` | 显示当前环路状态 |
| 环路开关 | `balance loop <X> on/off` | X=A/B/C/D/H/Y |
| 环路增益 | `balance loop <X> <0.0-1.0>` | 设置环路增益 |

#### 12.6.2 电机控制面板

| 界面按钮 | 发送命令 | 说明 |
|----------|----------|------|
| 扫描电机 | `scan` | 扫描在线电机 |
| 使能全部 | `enable all` | 所有电机进入闭环 |
| 停止全部 | `stop all` | 所有电机停止 |
| 读取全部 | `read all` | 读取所有电机状态 |
| 设置模式 | `mode <id> <mode>` | 0-力矩/1-速度/2-位置... |
| 设置速度 | `speed <id> <rpm>` | 速度模式目标速度 |
| 设置位置 | `pos <id> <deg>` | 位置模式目标角度 |
| 设置力矩 | `torque <id> <val>` | 力矩模式目标力矩 |
| 使能 | `enable <id>` | 单个电机进入闭环 |
| 空闲 | `idle <id>` | 单个电机进入空闲 |
| 停止 | `stop <id>` | 单个电机停止 |
| 读取 | `read <id>` | 读取单个电机状态 |
| 设零点 | `motor <id> origin` | 设置当前位置为零点 |
| 保存 | `motor <id> save` | 保存参数到Flash |
| 重启 | `motor <id> reboot` | 重启驱动器 |
| 错误码 | `motor <id> error` | 读取错误码 |

#### 12.6.3 IMU 控制面板

| 界面按钮 | 发送命令 | 说明 |
|----------|----------|------|
| 初始化 IMU | `imu init` | 初始化 I2C 和 IMU |
| 关闭 IMU | `imu deinit` | 释放 IMU 资源 |
| 查看状态 | `imu status` | 打印 IMU 状态 |
| 开始连续读取 | `imu start` | 启动 IMU 读取任务 |
| 停止读取 | `imu stop` | 停止 IMU 读取任务 |
| 单次读取 | `imu read` | 读取一次 IMU 数据 |
| 打印数据 (开) | `imu print on` | 使能数据打印 |
| 打印数据 (关) | `imu print off` | 禁用数据打印 |
| 设置速率 | `imu rate <N>` | 0-10 对应不同输出速率 |
| 加速度计校准 | `imu cali acc` | 启动加速度计校准 |
| 磁力计校准 | `imu cali mag` | 启动磁力计校准 |
| 停止校准 | `imu cali stop` | 停止校准过程 |

#### 12.6.4 传感器面板

| 界面按钮 | 发送命令 | 说明 |
|----------|----------|------|
| 读取电源状态 | `power` | 读取电源状态 |
| 电机供电 ON | `mpower on` | 使能电机供电 |
| 电机供电 OFF | `mpower off` | 关闭电机供电 |
| 按键初始化 | `btn init` | 初始化按键 GPIO |
| 按键读取 | `btn read` | 读取按键状态 |
| 按键开始监控 | `btn start` | 启动按键监控任务 |
| 按键停止 | `btn stop` | 停止按键监控任务 |
| SHT30 初始化 | `sht init` | 初始化温湿度传感器 |
| SHT30 读取 | `sht read` | 读取温湿度 |
| SHT30 开始监控 | `sht start` | 启动周期读取 |
| SHT30 停止 | `sht stop` | 停止周期读取 |
| 扫描 I2C | `sht scan` | 扫描 I2C 设备 |
| 启动 WiFi AP | `wifi start` | 启动 WiFi 热点 |
| 停止 WiFi | `wifi stop` | 关闭 WiFi |
| 查看 WiFi 状态 | `wifi status` | 显示 WiFi 连接状态 |

### 12.7 常见问题排查

#### 12.7.1 连接问题

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| 串口打开失败 | 权限不足 | `sudo chmod 666 /dev/ttyACM0` |
| 无响应 | 波特率不对 | 确认 115200 bps |
| 乱码 | 未初始化 | 先执行 `balance init` |
| 查询无返回 | 回调未注册 | 检查 `commander_parser_init()` |

#### 12.7.2 调参问题

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| 参数不生效 | 未同步到 LQR | 检查 `lqr_set_params()` |
| 波形不更新 | 波形未开启 | `balance plot on` |
| 波形太慢 | 分频太大 | `balance plot div 5` |
| 参数回读为 0 | 查询回调失败 | 检查 `commander_query_callback()` |

#### 12.7.3 控制问题

| 问题 | 可能原因 | 解决方案 |
|------|----------|----------|
| 电机不转 | 未使能 | `balance enable` |
| 电机不转 | 电源未开 | `mpower on` |
| 紧急停止 | 角度过大 | `balance reset` 然后 `balance enable` |
| 方向相反 | 电机方向错 | 调整位移计算符号 |

### 12.8 文件路径汇总

| 组件 | 文件路径 | 说明 |
|------|----------|------|
| Qt 调参面板 | `tools/qt_tuner/pid_tuner.py` | PyQt5 主程序 |
| Commander 解析器 | `components/device/src/commander_parser.c` | 命令解析实现 |
| Commander 头文件 | `components/device/include/commander_parser.h` | 协议定义 |
| 平衡测试模块 | `components/app/src/balance_test.c` | 主控制逻辑 |
| LQR 控制器 | `components/algorithm/src/lqr_balance.c` | LQR 算法实现 |
| LQR 参数结构 | `components/algorithm/include/lqr_balance.h` | lqr_params_t 定义 |
