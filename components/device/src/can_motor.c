/**
 * @file can_motor.c
 * @brief CAN 电机驱动实现
 * @author Bubble
 * @date 2026-01-15
 * @note 移植自 Arduino 版本 CANServo
 */

#include "can_motor.h"
#include "can_motor_regs.h"
#include "config.h"
#include "driver/twai.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "CAN_MOTOR";

// ============================================================================
// 内部数据结构
// ============================================================================

// 电机实例结构体
struct can_motor {
    uint8_t motor_id;
    motor_state_t state;
    uint32_t tx_frame_id;
    uint32_t rx_frame_id;
};

// 全局电机实例数组
static can_motor_handle_t g_motors[MOTOR_COUNT] = {NULL};
static bool g_can_initialized = false;

// ============================================================================
// 内部函数
// ============================================================================

/**
 * @brief 发送 CAN 帧
 */
static esp_err_t can_send_frame(uint32_t id, uint8_t *data, uint8_t len) {
    if (!g_can_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    twai_message_t msg = {
        .identifier = id,
        .data_length_code = 8,
        .flags = TWAI_MSG_FLAG_NONE,
    };
    
    memset(msg.data, 0, 8);
    memcpy(msg.data, data, len > 8 ? 8 : len);
    
    return twai_transmit(&msg, pdMS_TO_TICKS(10));
}

/**
 * @brief 写入 1 个寄存器 (2字节)
 * @note 寄存器地址和数据都是大端序 (高字节在前)
 */
static esp_err_t write_reg_1(can_motor_handle_t motor, uint16_t reg_addr, int16_t value) {
    uint8_t data[8] = {0};
    
    data[0] = CAN_CMD_WRITE_1REG;
    data[1] = (reg_addr >> 8) & 0xFF;  // 寄存器高字节
    data[2] = reg_addr & 0xFF;          // 寄存器低字节
    data[3] = 0x00;
    data[4] = (value >> 8) & 0xFF;      // 数据高字节
    data[5] = value & 0xFF;              // 数据低字节
    data[6] = 0x00;
    data[7] = 0x00;
    
    return can_send_frame(motor->tx_frame_id, data, 8);
}

/**
 * @brief 写入 2 个寄存器 (4字节)
 * @note 寄存器地址和数据都是大端序 (高字节在前)
 */
static esp_err_t write_reg_2(can_motor_handle_t motor, uint16_t reg_addr, int32_t value) {
    uint8_t data[8] = {0};
    
    data[0] = CAN_CMD_WRITE_2REG;
    data[1] = (reg_addr >> 8) & 0xFF;  // 寄存器高字节
    data[2] = reg_addr & 0xFF;          // 寄存器低字节
    data[3] = 0x00;
    data[4] = (value >> 24) & 0xFF;     // 数据字节1 (最高)
    data[5] = (value >> 16) & 0xFF;     // 数据字节2
    data[6] = (value >> 8) & 0xFF;      // 数据字节3
    data[7] = value & 0xFF;              // 数据字节4 (最低)
    
    return can_send_frame(motor->tx_frame_id, data, 8);
}

/**
 * @brief 读取 1 个寄存器请求 (2字节)
 * @note 寄存器地址是大端序 (高字节在前)
 */
static esp_err_t read_reg_1_request(can_motor_handle_t motor, uint16_t reg_addr) {
    uint8_t data[8] = {0};
    
    data[0] = CAN_CMD_READ_1REG;
    data[1] = (reg_addr >> 8) & 0xFF;  // 寄存器高字节
    data[2] = reg_addr & 0xFF;          // 寄存器低字节
    
    return can_send_frame(motor->tx_frame_id, data, 8);
}

/**
 * @brief 读取 2 个寄存器请求 (4字节)
 * @note 寄存器地址是大端序 (高字节在前)
 */
static esp_err_t read_reg_2_request(can_motor_handle_t motor, uint16_t reg_addr) {
    uint8_t data[8] = {0};
    
    data[0] = CAN_CMD_READ_2REG;
    data[1] = (reg_addr >> 8) & 0xFF;  // 寄存器高字节
    data[2] = reg_addr & 0xFF;          // 寄存器低字节
    
    return can_send_frame(motor->tx_frame_id, data, 8);
}

/**
 * @brief 根据帧 ID 查找电机实例
 */
static can_motor_handle_t find_motor_by_rx_id(uint32_t rx_id) {
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (g_motors[i] && g_motors[i]->rx_frame_id == rx_id) {
            return g_motors[i];
        }
    }
    return NULL;
}

// ============================================================================
// CAN 总线初始化
// ============================================================================

esp_err_t can_bus_init(gpio_num_t tx_pin, gpio_num_t rx_pin) {
    if (g_can_initialized) {
        ESP_LOGW(TAG, "CAN bus already initialized");
        return ESP_OK;
    }
    
    // 通用配置
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_pin, rx_pin, TWAI_MODE_NORMAL);
    g_config.rx_queue_len = 16;
    g_config.tx_queue_len = 16;
    
    // 波特率配置 - 1Mbps
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
    
    // 过滤器配置 - 接收所有帧
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    
    // 安装驱动
    esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install TWAI driver: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 启动 TWAI
    ret = twai_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start TWAI: %s", esp_err_to_name(ret));
        twai_driver_uninstall();
        return ret;
    }
    
    g_can_initialized = true;
    ESP_LOGI(TAG, "CAN bus initialized (TX: GPIO%d, RX: GPIO%d, 1Mbps)", tx_pin, rx_pin);
    
    return ESP_OK;
}

esp_err_t can_bus_deinit(void) {
    if (!g_can_initialized) {
        return ESP_OK;
    }
    
    twai_stop();
    twai_driver_uninstall();
    g_can_initialized = false;
    
    ESP_LOGI(TAG, "CAN bus deinitialized");
    return ESP_OK;
}

// ============================================================================
// 电机实例管理
// ============================================================================

can_motor_handle_t can_motor_create(uint8_t motor_id) {
    if (motor_id < 1 || motor_id > MOTOR_COUNT) {
        ESP_LOGE(TAG, "Invalid motor ID: %d (valid: 1-%d)", motor_id, MOTOR_COUNT);
        return NULL;
    }
    
    // 检查是否已存在
    int index = motor_id - 1;
    if (g_motors[index] != NULL) {
        ESP_LOGW(TAG, "Motor %d already exists", motor_id);
        return g_motors[index];
    }
    
    // 创建实例
    can_motor_handle_t motor = calloc(1, sizeof(struct can_motor));
    if (motor == NULL) {
        ESP_LOGE(TAG, "Failed to allocate motor instance");
        return NULL;
    }
    
    motor->motor_id = motor_id;
    motor->tx_frame_id = CAN_TX_BASE_ID + motor_id;
    motor->rx_frame_id = CAN_RX_BASE_ID + motor_id;
    motor->state.motor_id = motor_id;
    motor->state.is_online = false;
    
    g_motors[index] = motor;
    
    ESP_LOGI(TAG, "Motor %d created (TX: 0x%03lX, RX: 0x%03lX)", 
             motor_id, motor->tx_frame_id, motor->rx_frame_id);
    
    return motor;
}

void can_motor_destroy(can_motor_handle_t motor) {
    if (motor == NULL) return;
    
    int index = motor->motor_id - 1;
    if (index >= 0 && index < MOTOR_COUNT) {
        g_motors[index] = NULL;
    }
    
    free(motor);
}

uint8_t can_motor_get_id(can_motor_handle_t motor) {
    return motor ? motor->motor_id : 0;
}

// ============================================================================
// 状态读取
// ============================================================================

esp_err_t can_motor_request_status(can_motor_handle_t motor) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    
    esp_err_t ret = ESP_OK;
    
    // 请求读取所有状态寄存器
    // 每个请求之间需要短暂延时，避免总线冲突
    
    ret = read_reg_1_request(motor, REG_VOLTAGE);
    vTaskDelay(pdMS_TO_TICKS(2));
    
    ret = read_reg_1_request(motor, REG_CURRENT);
    vTaskDelay(pdMS_TO_TICKS(2));
    
    ret = read_reg_2_request(motor, REG_SPEED);  // 速度是4字节
    vTaskDelay(pdMS_TO_TICKS(2));
    
    ret = read_reg_2_request(motor, REG_POSITION);  // 位置是4字节
    vTaskDelay(pdMS_TO_TICKS(2));
    
    ret = read_reg_1_request(motor, REG_DRIVER_TEMP);
    vTaskDelay(pdMS_TO_TICKS(2));
    
    ret = read_reg_1_request(motor, REG_MOTOR_TEMP);
    vTaskDelay(pdMS_TO_TICKS(2));
    
    ret = read_reg_2_request(motor, REG_ERROR);  // 错误码是4字节
    
    return ret;
}

esp_err_t can_motor_process_rx(void) {
    if (!g_can_initialized) return ESP_ERR_INVALID_STATE;
    
    twai_message_t msg;
    
    // 非阻塞接收
    while (twai_receive(&msg, 0) == ESP_OK) {
        // 查找对应的电机
        can_motor_handle_t motor = find_motor_by_rx_id(msg.identifier);
        if (motor == NULL) {
            continue;
        }
        
        // 解析数据 - 注意大端序
        uint8_t cmd = msg.data[0];
        // 寄存器地址: data[1]=高字节, data[2]=低字节
        uint16_t reg_addr = (msg.data[1] << 8) | msg.data[2];
        
        if (cmd == CAN_CMD_READ_1REG) {
            // 2字节数据: data[4]=高字节, data[5]=低字节
            int16_t value = (int16_t)((msg.data[4] << 8) | msg.data[5]);
            
            switch (reg_addr) {
                case REG_VOLTAGE:
                    motor->state.voltage = value / SCALE_VOLTAGE;
                    break;
                case REG_CURRENT:
                    motor->state.current = value / SCALE_CURRENT;
                    break;
                case REG_DRIVER_TEMP:
                    motor->state.driver_temp = value / SCALE_TEMPERATURE;
                    break;
                case REG_MOTOR_TEMP:
                    motor->state.motor_temp = value / SCALE_TEMPERATURE;
                    break;
            }
            motor->state.is_online = true;
            motor->state.last_update = xTaskGetTickCount() * portTICK_PERIOD_MS;
            
        } else if (cmd == CAN_CMD_READ_2REG) {
            // 4字节数据: data[4]=最高字节, data[7]=最低字节
            int32_t value = ((int32_t)msg.data[4] << 24) | 
                           ((int32_t)msg.data[5] << 16) | 
                           ((int32_t)msg.data[6] << 8)  | 
                           (int32_t)msg.data[7];
            
            switch (reg_addr) {
                case REG_SPEED:
                    motor->state.speed = value / SCALE_SPEED;
                    break;
                case REG_POSITION:
                    motor->state.position = value / SCALE_POSITION;
                    break;
                case REG_ERROR:
                    motor->state.error_code = (uint32_t)value;
                    break;
            }
            motor->state.is_online = true;
            motor->state.last_update = xTaskGetTickCount() * portTICK_PERIOD_MS;
            
        } else if (cmd == CAN_CMD_REPLY) {
            // 写操作回复: [0x2A, PosH, PosM, PosL, SpdH, SpdL, CurH, CurL]
            // 位置: 3字节有符号数 (x100)
            int32_t pos_raw = ((int32_t)(int8_t)msg.data[1] << 16) | 
                             ((int32_t)msg.data[2] << 8) | 
                             (int32_t)msg.data[3];
            motor->state.position = pos_raw / SCALE_POSITION;
            
            // 速度: 2字节有符号数 (rpm)
            int16_t spd_raw = (int16_t)((msg.data[4] << 8) | msg.data[5]);
            motor->state.speed = (float)spd_raw;
            
            // 电流: 2字节有符号数 (x100)
            int16_t cur_raw = (int16_t)((msg.data[6] << 8) | msg.data[7]);
            motor->state.current = cur_raw / SCALE_CURRENT;
            
            motor->state.is_online = true;
            motor->state.last_update = xTaskGetTickCount() * portTICK_PERIOD_MS;
        }
    }
    
    return ESP_OK;
}

esp_err_t can_motor_get_state(can_motor_handle_t motor, motor_state_t *state) {
    if (motor == NULL || state == NULL) return ESP_ERR_INVALID_ARG;
    memcpy(state, &motor->state, sizeof(motor_state_t));
    return ESP_OK;
}

float can_motor_read_position(can_motor_handle_t motor) {
    return motor ? motor->state.position : 0.0f;
}

float can_motor_read_speed(can_motor_handle_t motor) {
    return motor ? motor->state.speed : 0.0f;
}

float can_motor_read_current(can_motor_handle_t motor) {
    return motor ? motor->state.current : 0.0f;
}

float can_motor_read_voltage(can_motor_handle_t motor) {
    return motor ? motor->state.voltage : 0.0f;
}

uint32_t can_motor_read_error(can_motor_handle_t motor) {
    return motor ? motor->state.error_code : 0;
}

void can_motor_print_error(uint32_t error_code) {
    if (error_code == 0) {
        printf("No error\n");
        return;
    }
    
    printf("Error code: 0x%08lX\n", (unsigned long)error_code);
    
    if (error_code & ERR_POWER_SMALL)       printf("  - Power too small\n");
    if (error_code & ERR_PHASE_RES_HIGH)    printf("  - Phase resistance high\n");
    if (error_code & ERR_CURRENT_FLUCTUATE) printf("  - Current fluctuation\n");
    if (error_code & ERR_INDUCTANCE_HIGH)   printf("  - Inductance high\n");
    if (error_code & ERR_ENCODER_BW)        printf("  - Encoder bandwidth issue\n");
    if (error_code & ERR_ENCODER_SPI)       printf("  - Encoder SPI error\n");
    if (error_code & ERR_ENCODER_TYPE)      printf("  - Encoder type error\n");
    if (error_code & ERR_HALL_NOT_CALIB)    printf("  - Hall not calibrated\n");
    if (error_code & ERR_ENCODER_NO_DATA)   printf("  - No encoder data\n");
    if (error_code & ERR_CPR_ERROR)         printf("  - CPR setting error\n");
    if (error_code & ERR_RUN_STATE)         printf("  - Run state error\n");
    if (error_code & ERR_HALL_SIGNAL)       printf("  - Hall signal error\n");
    if (error_code & ERR_ENCODER2)          printf("  - Second encoder error\n");
    if (error_code & ERR_DRIVER_JC2804)     printf("  - JC2804 driver error\n");
    if (error_code & ERR_MOS_OVERHEAT)      printf("  - MOS overheat\n");
    if (error_code & ERR_MOTOR_OVERHEAT)    printf("  - Motor overheat\n");
    if (error_code & ERR_UNDERVOLTAGE)      printf("  - Undervoltage\n");
    if (error_code & ERR_OVERVOLTAGE)       printf("  - Overvoltage\n");
    if (error_code & ERR_OVERCURRENT)       printf("  - Overcurrent\n");
}

float can_motor_read_motor_temp(can_motor_handle_t motor) {
    return motor ? motor->state.motor_temp : 0.0f;
}

float can_motor_read_driver_temp(can_motor_handle_t motor) {
    return motor ? motor->state.driver_temp : 0.0f;
}

bool can_motor_is_online(can_motor_handle_t motor, uint32_t timeout_ms) {
    if (motor == NULL) return false;
    
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    return motor->state.is_online && 
           (now - motor->state.last_update < timeout_ms);
}

// ============================================================================
// 控制命令
// ============================================================================

esp_err_t can_motor_set_mode(can_motor_handle_t motor, motor_mode_t mode) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    return write_reg_1(motor, REG_CONTROL_MODE, (int16_t)mode);
}

esp_err_t can_motor_set_speed(can_motor_handle_t motor, float speed_rpm) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    // 速度用 4 字节 (32位), 放大 100 倍
    int32_t speed_raw = (int32_t)(speed_rpm * SCALE_SPEED);
    return write_reg_2(motor, REG_SET_SPEED, speed_raw);
}

esp_err_t can_motor_set_position(can_motor_handle_t motor, float angle_deg, float speed_rpm) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    
    // 先设置位置
    int32_t pos_raw = (int32_t)(angle_deg * SCALE_POSITION);
    esp_err_t ret = write_reg_2(motor, REG_SET_ABS_POS, pos_raw);
    
    return ret;
}

esp_err_t can_motor_set_torque(can_motor_handle_t motor, float torque) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    int16_t torque_raw = (int16_t)(torque * SCALE_TORQUE);
    return write_reg_1(motor, REG_SET_TORQUE, torque_raw);
}

esp_err_t can_motor_pv_cmd(can_motor_handle_t motor, float position_deg, float speed_rpm) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    
    uint8_t data[8] = {0};
    
    int32_t pos_raw = (int32_t)(position_deg * SCALE_POSITION);
    int16_t speed_raw = (int16_t)speed_rpm;  // 速度不放大
    
    // PV 指令格式: [0x24, PosH, PosM, PosM, PosL, SpdH, SpdL, 0x00]
    data[0] = CAN_CMD_PV;
    data[1] = (pos_raw >> 24) & 0xFF;  // 位置 H
    data[2] = (pos_raw >> 16) & 0xFF;  // 位置 M
    data[3] = (pos_raw >> 8) & 0xFF;   // 位置 M
    data[4] = pos_raw & 0xFF;           // 位置 L
    data[5] = (speed_raw >> 8) & 0xFF;  // 速度 H
    data[6] = speed_raw & 0xFF;          // 速度 L
    data[7] = 0x00;
    
    return can_send_frame(motor->tx_frame_id, data, 8);
}

esp_err_t can_motor_pvt_cmd(can_motor_handle_t motor, float position_deg, float speed_rpm, uint8_t torque_percent) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    
    uint8_t data[8] = {0};
    
    int32_t pos_raw = (int32_t)(position_deg * SCALE_POSITION);
    int16_t speed_raw = (int16_t)speed_rpm;  // 速度不放大
    
    // PVT 指令格式: [0x25, PosH, PosM, PosM, PosL, SpdH, SpdL, Torque%]
    data[0] = CAN_CMD_PVT;
    data[1] = (pos_raw >> 24) & 0xFF;  // 位置 H
    data[2] = (pos_raw >> 16) & 0xFF;  // 位置 M
    data[3] = (pos_raw >> 8) & 0xFF;   // 位置 M
    data[4] = pos_raw & 0xFF;           // 位置 L
    data[5] = (speed_raw >> 8) & 0xFF;  // 速度 H
    data[6] = speed_raw & 0xFF;          // 速度 L
    data[7] = torque_percent;            // 力矩百分比 (0-100)
    
    return can_send_frame(motor->tx_frame_id, data, 8);
}

esp_err_t can_motor_set_rel_position(can_motor_handle_t motor, float angle_deg) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    int32_t pos_raw = (int32_t)(angle_deg * SCALE_POSITION);
    return write_reg_2(motor, REG_SET_REL_POS, pos_raw);
}

esp_err_t can_motor_set_low_speed(can_motor_handle_t motor, float speed_rpm) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    // 低速模式：范围 ±300rpm, 放大100倍
    int16_t speed_raw = (int16_t)(speed_rpm * SCALE_SPEED);
    return write_reg_1(motor, REG_SET_LOW_SPEED, speed_raw);
}

esp_err_t can_motor_enter_closed_loop(can_motor_handle_t motor) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    return write_reg_1(motor, REG_CLOSE_LOOP, 0x0001);  // 写入1执行命令
}

esp_err_t can_motor_set_idle(can_motor_handle_t motor) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    return write_reg_1(motor, REG_IDLE, 0x0001);  // 写入1执行命令
}

esp_err_t can_motor_set_origin(can_motor_handle_t motor) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    ESP_LOGI(TAG, "Motor %d: Setting current position as origin (zero)", motor->motor_id);
    return write_reg_1(motor, REG_SET_ORIGIN, 0x0001);  // 写入1执行命令
}

esp_err_t can_motor_reboot(can_motor_handle_t motor) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    ESP_LOGI(TAG, "Motor %d: Rebooting", motor->motor_id);
    return write_reg_1(motor, REG_REBOOT, 0x0001);  // 写入1执行命令
}

esp_err_t can_motor_save(can_motor_handle_t motor) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    ESP_LOGI(TAG, "Motor %d: Saving parameters to Flash", motor->motor_id);
    return write_reg_1(motor, REG_SAVE, 0x0001);  // 写入1执行命令
}

esp_err_t can_motor_calibrate(can_motor_handle_t motor) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    ESP_LOGW(TAG, "Motor %d: Starting calibration - motor will rotate!", motor->motor_id);
    return write_reg_1(motor, REG_CALIBRATE, 0x0001);  // 写入1执行命令
}

esp_err_t can_motor_erase(can_motor_handle_t motor) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    ESP_LOGW(TAG, "Motor %d: Erasing parameters - will need recalibration!", motor->motor_id);
    return write_reg_1(motor, REG_ERASE, 0x0001);  // 写入1执行命令
}

// ============================================================================
// 批量操作
// ============================================================================

esp_err_t can_motor_set_all_speeds(float speeds[6]) {
    esp_err_t ret = ESP_OK;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (g_motors[i] != NULL) {
            esp_err_t r = can_motor_set_speed(g_motors[i], speeds[i]);
            if (r != ESP_OK) ret = r;
            vTaskDelay(1);  // 短暂延时避免总线冲突
        }
    }
    return ret;
}

esp_err_t can_motor_all_enter_closed_loop(void) {
    esp_err_t ret = ESP_OK;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (g_motors[i] != NULL) {
            esp_err_t r = can_motor_enter_closed_loop(g_motors[i]);
            if (r != ESP_OK) ret = r;
            vTaskDelay(1);
        }
    }
    return ret;
}

esp_err_t can_motor_all_set_idle(void) {
    esp_err_t ret = ESP_OK;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (g_motors[i] != NULL) {
            esp_err_t r = can_motor_set_idle(g_motors[i]);
            if (r != ESP_OK) ret = r;
            vTaskDelay(1);
        }
    }
    return ret;
}
