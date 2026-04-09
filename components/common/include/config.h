/**
 * @file config.h
 * @brief 全局配置 - 引脚定义与系统参数
 * @author Bubble
 * @date 2026-01-15
 */

#ifndef __CONFIG_H__
#define __CONFIG_H__

#include "driver/gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 版本信息
// ============================================================================

#define PROJECT_NAME        "wheel_leg_final"
#define PROJECT_VERSION     "1.0.0"

// ============================================================================
// 电机品牌选择 (宏定义切换)
// ============================================================================

#define MOTOR_BRAND_JUCI    0           // 俱瓷科技 (JuCi) 电机
#define MOTOR_BRAND_STW     1           // 伺泰威 (STW) 电机

// 当前使用的电机品牌 - 修改此宏即可切换电机类型
// 支持单品牌 (MOTOR_BRAND_JUCI / MOTOR_BRAND_STW) 或混合品牌:
//   在混合模式下, 通过 MOTOR_BRAND_OF(id) 宏为每个电机指定品牌
#define MOTOR_BRAND    MOTOR_BRAND_STW

// ============================================================================
// 混合品牌配置 (按电机 ID 指定品牌)
// ============================================================================
// 轮电机 (ID 3,6) = JuCi, 关节电机 (ID 1,2,4,5) = STW
// 修改此处即可切换各电机品牌
#define MOTOR_BRAND_ID1     MOTOR_BRAND_STW     // 左大腿(关节)
#define MOTOR_BRAND_ID2     MOTOR_BRAND_STW     // 左小腿(关节)
#define MOTOR_BRAND_ID3     MOTOR_BRAND_JUCI    // 左轮
#define MOTOR_BRAND_ID4     MOTOR_BRAND_STW     // 右大腿(关节)
#define MOTOR_BRAND_ID5     MOTOR_BRAND_STW     // 右小腿(关节)
#define MOTOR_BRAND_ID6     MOTOR_BRAND_JUCI    // 右轮

// 查询表: 根据 motor_id (1~6) 获取品牌
#define MOTOR_BRAND_OF(id) ( \
    (id) == 1 ? MOTOR_BRAND_ID1 : \
    (id) == 2 ? MOTOR_BRAND_ID2 : \
    (id) == 3 ? MOTOR_BRAND_ID3 : \
    (id) == 4 ? MOTOR_BRAND_ID4 : \
    (id) == 5 ? MOTOR_BRAND_ID5 : \
    (id) == 6 ? MOTOR_BRAND_ID6 : MOTOR_BRAND_STW)

// ============================================================================
// CAN 总线配置
// ============================================================================

#define CAN_TX_PIN          GPIO_NUM_18
#define CAN_RX_PIN          GPIO_NUM_17
#define CAN_BAUDRATE        1000000     // 1Mbps

// CAN 帧 ID 基地址 (俱瓷科技, 混合模式下也需要)
#define CAN_TX_BASE_ID      0x600       // 发送帧基地址
#define CAN_RX_BASE_ID      0x580       // 接收帧基地址

// ============================================================================
// 电机 ID 分配
// ============================================================================

// 左腿电机 ID
#define MOTOR_ID_LEFT_HIP       1       // 左大腿
#define MOTOR_ID_LEFT_KNEE      2       // 左小腿
#define MOTOR_ID_LEFT_WHEEL     3       // 左轮

// 右腿电机 ID
#define MOTOR_ID_RIGHT_HIP      4       // 右大腿
#define MOTOR_ID_RIGHT_KNEE     5       // 右小腿
#define MOTOR_ID_RIGHT_WHEEL    6       // 右轮

// 电机总数
#define MOTOR_COUNT             6

// ============================================================================
// I2C 配置
// ============================================================================

// I2C1 - SHT30 温湿度传感器
#define I2C1_SCL_PIN        GPIO_NUM_1
#define I2C1_SDA_PIN        GPIO_NUM_2
#define I2C1_FREQ_HZ        100000      // 100kHz

// I2C2 - IMU 陀螺仪 (维特智能)
#define I2C2_SCL_PIN        GPIO_NUM_11
#define I2C2_SDA_PIN        GPIO_NUM_12
#define I2C2_FREQ_HZ        400000      // 400kHz

// I2C 设备地址
#define SHT30_I2C_ADDR      0x44
#define IMU_I2C_ADDR        0x50        // 维特智能默认地址，需确认

// ============================================================================
// UART 配置 (树莓派通信)
// ============================================================================

#define UART_RPI_TX_PIN     GPIO_NUM_4
#define UART_RPI_RX_PIN     GPIO_NUM_5
#define UART_RPI_BAUDRATE   115200
#define UART_RPI_NUM        UART_NUM_1

// ============================================================================
// GPIO 配置
// ============================================================================

// 按键
#define BUTTON1_PIN         GPIO_NUM_13     // 外部上拉，按下为低
#define BUTTON2_PIN         GPIO_NUM_9      // 外部上拉，按下为低

// 电源检测
#define POWER_DETECT_PIN    GPIO_NUM_3      // 高=电池，低=USB

// 电机供电控制
#define MOTOR_POWER_PIN     GPIO_NUM_8      // 高=供电，低=断电
#define MOTOR_POWER_EN_PIN  GPIO_NUM_8      // 别名

// ============================================================================
// WiFi 配置
// ============================================================================

#define WIFI_AP_SSID        "WHEEL_LEG"
#define WIFI_AP_PASSWORD    "12345678"
#define WIFI_AP_CHANNEL     1
#define WIFI_AP_MAX_CONN    4

// ============================================================================
// 机械参数 (单位: mm)
// ============================================================================

#define LEG_LINK1_LENGTH    100.0f      // 大腿长度 (mm) - 需根据实际修改
#define LEG_LINK2_LENGTH    100.0f      // 小腿长度 (mm) - 需根据实际修改
#define WHEEL_RADIUS        30.0f       // 轮半径 (mm) - 直径60mm
#define WHEEL_RADIUS_M      0.03f       // 轮半径 (m) - 用于位移/速度计算

// ============================================================================
// 控制参数
// ============================================================================

// 平衡控制周期
#define BALANCE_CTRL_PERIOD_MS      2       // 2ms = 500Hz
#define MOTOR_COMM_PERIOD_MS        1       // 1ms = 1000Hz
#define IMU_READ_PERIOD_MS          2       // 2ms = 500Hz
#define CONTROL_LOOP_FREQ_HZ        500     // 控制环频率 (Hz)

// 安全限制
#define MAX_PITCH_ANGLE             35.0f   // 最大俯仰角 (°)
#define MAX_ROLL_ANGLE              30.0f   // 最大横滚角 (°)
#define MAX_MOTOR_TEMP              80.0f   // 电机最高温度 (°C)
#define MAX_DRIVER_TEMP             85.0f   // 驱动器最高温度 (°C)

// ============================================================================
// FreeRTOS 任务优先级
// ============================================================================

#define TASK_PRIORITY_BALANCE       24
#define TASK_PRIORITY_MOTOR_COMM    20
#define TASK_PRIORITY_IMU_READ      18
#define TASK_PRIORITY_STATE_MACHINE 12
#define TASK_PRIORITY_WIFI_REMOTE   10
#define TASK_PRIORITY_UART_COMM     10
#define TASK_PRIORITY_SAFETY        10
#define TASK_PRIORITY_CONTROL       15
#define TASK_PRIORITY_SENSOR        5
#define TASK_PRIORITY_BUTTON        5

// ============================================================================
// FreeRTOS 任务栈大小
// ============================================================================

#define TASK_STACK_BALANCE          4096
#define TASK_STACK_MOTOR_COMM       4096
#define TASK_STACK_IMU_READ         2048
#define TASK_STACK_STATE_MACHINE    4096
#define TASK_STACK_WIFI_REMOTE      8192
#define TASK_STACK_UART_COMM        4096
#define TASK_STACK_SAFETY           2048
#define TASK_STACK_CONTROL          4096
#define TASK_STACK_SENSOR           2048
#define TASK_STACK_BUTTON           2048

#ifdef __cplusplus
}
#endif

#endif // __CONFIG_H__
