/**
 * @file can_motor.c
 * @brief CAN 电机驱动实现
 * @author Bubble
 * @date 2026-01-15
 * @note 移植自 Arduino 版本 CANServo
 */

#include "can_motor.h"
#include "config.h"
#include "can_motor_regs.h"
#include "can_motor_stw_regs.h"

#include "driver/twai.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>
#include <stdlib.h>
#include <math.h>

static const char *TAG = "CAN_MOTOR";

// ============================================================================
// 角度安全处理配置
// ============================================================================

// 单次位置命令最大角度变化量 (度)
// 设为 0 表示不限制步长，只计算最短路径
// 注意：步长限制只对高频连续调用有效，单次命令应设为 0
#define MOTOR_MAX_ANGLE_STEP        0.0f

// 是否启用角度跨越保护 (最短路径计算)
// 警告：此功能仅适用于编码器在 ±180° 内回绕的电机
// 对于多圈累计编码器（如当前使用的电机），必须设为 0！
#define MOTOR_ANGLE_WRAP_PROTECTION 0

// ============================================================================
// 内部数据结构
// ============================================================================

// 电机实例结构体
struct can_motor {
    uint8_t motor_id;
    uint8_t brand;              // MOTOR_BRAND_JUCI 或 MOTOR_BRAND_STW
    int8_t  dir;                // 方向因子: +1 (JuCi/CCW正) 或 -1 (STW/CW正)
    motor_state_t state;
    uint32_t tx_frame_id;
    uint32_t rx_frame_id;
    // STW 专用字段 (混合模式下始终存在, JuCi 电机不使用)
    // 0xB0 电机信息
    stw_motor_info_t stw_info;
    bool stw_info_valid;
    // 0xB6-0xB9 PID 参数缓存
    float stw_pid[4];           // [0]=pos_kp, [1]=pos_ki, [2]=spd_kp, [3]=spd_ki
    bool  stw_pid_valid[4];
    // MIT 运控
    stw_mit_state_t stw_mit_state;
    bool stw_mit_state_valid;
    float stw_mit_pos_max;      // 当前配置的最大值 (用于解码)
    float stw_mit_vel_max;
    float stw_mit_t_max;
};

// 全局电机实例数组
static can_motor_handle_t g_motors[MOTOR_COUNT] = {NULL};
static bool g_can_initialized = false;

// CAN Bus-Off 恢复统计
static uint32_t g_can_busoff_count = 0;
static uint32_t g_can_tx_error_count = 0;
static uint32_t g_can_recovery_count = 0;

// CAN 调试开关 (打印每帧 TX/RX)
static bool g_can_debug = false;
static uint32_t g_can_debug_filter_id = 0;  // 0=全部, >0=只打印指定 motor_id

void can_motor_set_debug(bool enable, uint32_t filter_motor_id) {
    g_can_debug = enable;
    g_can_debug_filter_id = filter_motor_id;
    printf("CAN debug %s (filter: %s)\n", enable ? "ON" : "OFF",
           filter_motor_id ? "motor specified" : "all");
}

// 保存初始化引脚，用于恢复失败时重新初始化
static gpio_num_t g_can_tx_pin = GPIO_NUM_NC;
static gpio_num_t g_can_rx_pin = GPIO_NUM_NC;

// ============================================================================
// 角度安全处理辅助函数
// ============================================================================

/**
 * @brief 将角度规范化到 [-180, 180) 范围
 * @param angle 输入角度 (度)
 * @return 规范化后的角度
 */
static float normalize_angle_180(float angle) {
    // 使用 fmodf 处理大角度，避免循环次数过多
    angle = fmodf(angle, 360.0f);
    if (angle >= 180.0f) {
        angle -= 360.0f;
    } else if (angle < -180.0f) {
        angle += 360.0f;
    }
    return angle;
}

/**
 * @brief 计算从当前角度到目标角度的最短路径增量
 * @param current 当前角度 (度)
 * @param target 目标角度 (度)
 * @return 最短路径增量 (范围 [-180, 180])
 * 
 * @example 
 *   shortest_angle_delta(330, -40) = -10  (而不是 -370)
 *   shortest_angle_delta(-30, 40) = 70
 *   shortest_angle_delta(10, 350) = -20   (而不是 340)
 */
static float shortest_angle_delta(float current, float target) {
    float delta = target - current;
    return normalize_angle_180(delta);
}

/**
 * @brief 限制角度变化量
 * @param delta 原始变化量
 * @param max_step 最大允许变化量
 * @return 限制后的变化量
 */
static float clamp_angle_delta(float delta, float max_step) {
    if (delta > max_step) {
        return max_step;
    } else if (delta < -max_step) {
        return -max_step;
    }
    return delta;
}

/**
 * @brief 计算安全的目标角度 (最短路径 + 变化量限制)
 * @param current 当前电机角度 (度)
 * @param target 期望目标角度 (度)
 * @param max_step 单次最大变化量 (度), 0 表示不限制
 * @return 安全的目标角度
 */
static float compute_safe_target_angle(float current, float target, float max_step) {
#if MOTOR_ANGLE_WRAP_PROTECTION
    // 计算最短路径增量
    float delta = shortest_angle_delta(current, target);
    
    // 应用变化量限制
    if (max_step > 0.0f) {
        delta = clamp_angle_delta(delta, max_step);
    }
    
    // 返回基于当前位置的新目标
    return current + delta;
#else
    // 保护功能禁用时，直接返回原始目标
    (void)current;
    (void)max_step;
    return target;
#endif
}

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
        .data_length_code = len,
        .flags = TWAI_MSG_FLAG_NONE,
    };
    
    memset(msg.data, 0, 8);
    memcpy(msg.data, data, len > 8 ? 8 : len);
    
    esp_err_t ret = twai_transmit(&msg, pdMS_TO_TICKS(1));  // 1ms 超时，避免阻塞高频控制循环
    if (g_can_debug) {
        // 过滤: 只打印指定电机的帧
        bool show = (g_can_debug_filter_id == 0) ||
                    (id == CAN_TX_BASE_ID + g_can_debug_filter_id) ||
                    (id == CAN_RX_BASE_ID + g_can_debug_filter_id) ||
                    (id == (STW_TX_ID_OFFSET | g_can_debug_filter_id)) ||
                    (id == g_can_debug_filter_id);
        if (show) {
            printf("CAN TX 0x%03lX [%d]: %02X %02X %02X %02X %02X %02X %02X %02X %s\n",
                   id, len, data[0], data[1], data[2], data[3],
                   data[4], data[5], data[6], data[7],
                   (ret == ESP_OK) ? "OK" : esp_err_to_name(ret));
        }
    }
    if (ret != ESP_OK) {
        g_can_tx_error_count++;
        // TX 失败可能是 Bus-Off，尝试检查和恢复
        if (ret == ESP_ERR_INVALID_STATE || ret == ESP_ERR_NOT_FOUND) {
            // ESP_ERR_INVALID_STATE: 驱动不在运行状态 (可能 Bus-Off)
            // ESP_ERR_NOT_FOUND: TX 队列被禁用
            can_bus_check_and_recover();
        }
        // 每 100 次错误打印一次警告，避免日志洪泛
        if ((g_can_tx_error_count % 100) == 1) {
            ESP_LOGW(TAG, "CAN TX error #%lu: %s (id=0x%03lx)", 
                     g_can_tx_error_count, esp_err_to_name(ret), id);
        }
    }
    return ret;
}

/**
 * @brief 写入 1 个寄存器 (2字节)
 * @note 寄存器地址和数据都是大端序 (高字节在前) [仅俱瓷]
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
 * @note 寄存器地址和数据都是大端序 (高字节在前) [仅俱瓷]
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
 * @note 寄存器地址是大端序 (高字节在前) [仅俱瓷]
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
 * @note 寄存器地址是大端序 (高字节在前) [仅俱瓷]
 */
static esp_err_t read_reg_2_request(can_motor_handle_t motor, uint16_t reg_addr) {
    uint8_t data[8] = {0};
    
    data[0] = CAN_CMD_READ_2REG;
    data[1] = (reg_addr >> 8) & 0xFF;  // 寄存器高字节
    data[2] = reg_addr & 0xFF;          // 寄存器低字节
    
    return can_send_frame(motor->tx_frame_id, data, 8);
}

// ============================================================================
// STW 内部辅助函数 (小端序, 命令码协议)
// ============================================================================

/**
 * @brief 发送 STW 简单命令 (仅命令码, DLC=1)
 */
static esp_err_t stw_send_cmd_simple(can_motor_handle_t motor, uint8_t cmd) {
    uint8_t data[1] = { cmd };
    return can_send_frame(motor->tx_frame_id, data, 1);
}

/**
 * @brief 发送 STW 命令 + 4字节有符号数据 (小端序, DLC=5)
 */
static esp_err_t stw_send_cmd_4s(can_motor_handle_t motor, uint8_t cmd, int32_t value) {
    uint8_t data[5];
    data[0] = cmd;
    data[1] = (uint8_t)(value & 0xFF);           // 低字节
    data[2] = (uint8_t)((value >> 8) & 0xFF);
    data[3] = (uint8_t)((value >> 16) & 0xFF);
    data[4] = (uint8_t)((value >> 24) & 0xFF);   // 高字节
    return can_send_frame(motor->tx_frame_id, data, 5);
}

/**
 * @brief 发送 STW 命令 + 4字节无符号数据 (小端序, DLC=5)
 */
static esp_err_t __attribute__((unused)) stw_send_cmd_4u(can_motor_handle_t motor, uint8_t cmd, uint32_t value) {
    uint8_t data[5];
    data[0] = cmd;
    data[1] = (uint8_t)(value & 0xFF);
    data[2] = (uint8_t)((value >> 8) & 0xFF);
    data[3] = (uint8_t)((value >> 16) & 0xFF);
    data[4] = (uint8_t)((value >> 24) & 0xFF);
    return can_send_frame(motor->tx_frame_id, data, 5);
}

/**
 * @brief 发送 STW 命令 + 4字节 float 数据 (IEEE 754, 小端序, DLC=5)
 */
static esp_err_t stw_send_cmd_float(can_motor_handle_t motor, uint8_t cmd, float value) {
    uint8_t data[5];
    uint32_t raw;
    memcpy(&raw, &value, sizeof(raw));
    data[0] = cmd;
    data[1] = (uint8_t)(raw & 0xFF);
    data[2] = (uint8_t)((raw >> 8) & 0xFF);
    data[3] = (uint8_t)((raw >> 16) & 0xFF);
    data[4] = (uint8_t)((raw >> 24) & 0xFF);
    return can_send_frame(motor->tx_frame_id, data, 5);
}

/**
 * @brief 将 PID 命令码转为数组索引 (0xB6→0, 0xB7→1, 0xB8→2, 0xB9→3)
 * @return 0~3 成功, -1 无效
 */
static int stw_pid_cmd_to_index(uint8_t cmd) {
    if (cmd >= STW_CMD_POS_KP && cmd <= STW_CMD_SPD_KI) {
        return cmd - STW_CMD_POS_KP;
    }
    return -1;
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
    g_config.rx_queue_len = 32;   // 增大 RX 队列，防止 500Hz 下丢帧
    g_config.tx_queue_len = 8;    // TX 队列适当大小，避免堆积过多
    // 启用 Bus-Off 和错误告警，用于自动恢复
    g_config.alerts_enabled = TWAI_ALERT_BUS_OFF | TWAI_ALERT_ERR_PASS 
                            | TWAI_ALERT_BUS_RECOVERED | TWAI_ALERT_ABOVE_ERR_WARN;
    
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
    g_can_tx_pin = tx_pin;
    g_can_rx_pin = rx_pin;
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

/**
 * @brief 检查 CAN 总线状态并在 Bus-Off 时自动恢复
 * @return ESP_OK: 总线正常, ESP_ERR_INVALID_STATE: 恢复失败
 */
esp_err_t can_bus_check_and_recover(void) {
    if (!g_can_initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    
    uint32_t alerts;
    // 非阻塞读取告警 (timeout = 0)
    if (twai_read_alerts(&alerts, 0) != ESP_OK) {
        return ESP_OK;  // 没有告警，总线正常
    }
    
    if (alerts & TWAI_ALERT_ABOVE_ERR_WARN) {
        // 限流打印：每秒最多1次，避免日志洪泛
        static uint32_t last_warn_log = 0;
        static uint32_t warn_suppressed = 0;
        uint32_t now = (uint32_t)(esp_log_timestamp());
        if (now - last_warn_log > 1000) {
            if (warn_suppressed > 0) {
                ESP_LOGW(TAG, "CAN bus error warning (TX/RX error counter high) [suppressed %lu times]", warn_suppressed);
            } else {
                ESP_LOGW(TAG, "CAN bus error warning (TX/RX error counter high)");
            }
            last_warn_log = now;
            warn_suppressed = 0;
        } else {
            warn_suppressed++;
        }
    }
    
    if (alerts & TWAI_ALERT_ERR_PASS) {
        // 限流打印：每秒最多1次
        static uint32_t last_pass_log = 0;
        static uint32_t pass_suppressed = 0;
        uint32_t now = (uint32_t)(esp_log_timestamp());
        if (now - last_pass_log > 1000) {
            if (pass_suppressed > 0) {
                ESP_LOGW(TAG, "CAN bus entered error-passive state [suppressed %lu times]", pass_suppressed);
            } else {
                ESP_LOGW(TAG, "CAN bus entered error-passive state");
            }
            last_pass_log = now;
            pass_suppressed = 0;
        } else {
            pass_suppressed++;
        }
    }
    
    if (alerts & TWAI_ALERT_BUS_RECOVERED) {
        g_can_recovery_count++;
        ESP_LOGI(TAG, "CAN bus recovered from Bus-Off (recovery #%lu)", g_can_recovery_count);
        // 恢复后需要重新启动
        esp_err_t ret = twai_start();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "CAN bus restarted successfully after recovery");
            return ESP_OK;
        } else {
            ESP_LOGE(TAG, "Failed to restart CAN after recovery: %s", esp_err_to_name(ret));
            return ESP_ERR_INVALID_STATE;
        }
    }
    
    if (alerts & TWAI_ALERT_BUS_OFF) {
        g_can_busoff_count++;
        ESP_LOGE(TAG, "CAN Bus-Off detected! (count: %lu) Initiating recovery...", g_can_busoff_count);
        
        // 发起恢复流程 (CAN 控制器将等待 128 * 11 个连续隐性位)
        esp_err_t ret = twai_initiate_recovery();
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to initiate CAN recovery: %s", esp_err_to_name(ret));
            // 强制重初始化
            twai_stop();
            twai_driver_uninstall();
            g_can_initialized = false;
            if (g_can_tx_pin != GPIO_NUM_NC && g_can_rx_pin != GPIO_NUM_NC) {
                esp_err_t reinit_ret = can_bus_init(g_can_tx_pin, g_can_rx_pin);
                if (reinit_ret == ESP_OK) {
                    g_can_recovery_count++;
                    ESP_LOGI(TAG, "CAN bus re-initialized successfully (recovery #%lu)", g_can_recovery_count);
                    return ESP_OK;
                }
            }
            return ESP_ERR_INVALID_STATE;
        }
        
        // 非阻塞恢复: 记录时间戳，下次调用时检查是否已恢复
        // 不再在此处阻塞等待，让控制循环继续运行
        static uint32_t recovery_start_ms = 0;
        recovery_start_ms = (uint32_t)(esp_log_timestamp());
        
        // 只做一次短等待 (1ms)，大多数情况下 Bus-Off 恢复很快
        vTaskDelay(pdMS_TO_TICKS(1));
        if (twai_read_alerts(&alerts, 0) == ESP_OK && (alerts & TWAI_ALERT_BUS_RECOVERED)) {
            g_can_recovery_count++;
            ESP_LOGI(TAG, "CAN bus recovered quickly (recovery #%lu)", g_can_recovery_count);
            ret = twai_start();
            if (ret == ESP_OK) {
                ESP_LOGI(TAG, "CAN bus restarted successfully");
                return ESP_OK;
            }
        }
        
        // 如果 1ms 内没恢复，等待稍长一些 (最多 20ms，不再等 500ms)
        for (int i = 0; i < 4; i++) {
            vTaskDelay(pdMS_TO_TICKS(5));
            if (twai_read_alerts(&alerts, 0) == ESP_OK) {
                if (alerts & TWAI_ALERT_BUS_RECOVERED) {
                    g_can_recovery_count++;
                    uint32_t elapsed = (uint32_t)(esp_log_timestamp()) - recovery_start_ms;
                    ESP_LOGI(TAG, "CAN bus recovered after %lu ms (recovery #%lu)", 
                             elapsed, g_can_recovery_count);
                    ret = twai_start();
                    if (ret == ESP_OK) {
                        ESP_LOGI(TAG, "CAN bus restarted successfully");
                        return ESP_OK;
                    }
                    ESP_LOGE(TAG, "Failed to restart CAN: %s", esp_err_to_name(ret));
                    return ESP_ERR_INVALID_STATE;
                }
            }
        }
        
        ESP_LOGE(TAG, "CAN recovery timeout (20ms), forcing re-init...");
        
        // 恢复超时，强制卸载再重装 TWAI 驱动 (硬重启 CAN 控制器)
        twai_stop();           // 可能失败，忽略
        twai_driver_uninstall();
        g_can_initialized = false;
        
        // 重新初始化
        if (g_can_tx_pin != GPIO_NUM_NC && g_can_rx_pin != GPIO_NUM_NC) {
            esp_err_t reinit_ret = can_bus_init(g_can_tx_pin, g_can_rx_pin);
            if (reinit_ret == ESP_OK) {
                g_can_recovery_count++;
                ESP_LOGI(TAG, "CAN bus re-initialized successfully (recovery #%lu)", g_can_recovery_count);
                return ESP_OK;
            } else {
                ESP_LOGE(TAG, "CAN bus re-init failed: %s", esp_err_to_name(reinit_ret));
            }
        }
        
        return ESP_ERR_TIMEOUT;
    }
    
    return ESP_OK;
}

/**
 * @brief 获取 CAN 总线错误统计
 */
void can_bus_get_error_stats(uint32_t *busoff_count, uint32_t *tx_error_count, uint32_t *recovery_count) {
    if (busoff_count) *busoff_count = g_can_busoff_count;
    if (tx_error_count) *tx_error_count = g_can_tx_error_count;
    if (recovery_count) *recovery_count = g_can_recovery_count;
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
    motor->brand = MOTOR_BRAND_OF(motor_id);
    motor->dir   = (motor->brand == MOTOR_BRAND_STW) ? -1 : 1;
    
    if (motor->brand == MOTOR_BRAND_JUCI) {
        motor->tx_frame_id = CAN_TX_BASE_ID + motor_id;
        motor->rx_frame_id = CAN_RX_BASE_ID + motor_id;
    } else {
        // STW: 主机发送 (0x100 | Dev_addr), 从机应答 Dev_addr
        motor->tx_frame_id = STW_TX_ID_OFFSET | motor_id;
        motor->rx_frame_id = motor_id;
        // 初始化 MIT 最大值为默认值 (用于解码)
        motor->stw_mit_pos_max = STW_MIT_DEFAULT_POS_MAX;
        motor->stw_mit_vel_max = STW_MIT_DEFAULT_VEL_MAX;
        motor->stw_mit_t_max   = STW_MIT_DEFAULT_TORQUE_MAX;
    }
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
    
    if (motor->brand == MOTOR_BRAND_JUCI) {
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
    } else {
        // STW: 0xAE 只返回电压/温度/模式/故障, 需额外请求位置/速度/电流
        ret = stw_send_cmd_simple(motor, STW_CMD_READ_ANGLE);    // 0xA3: 多圈角度
        vTaskDelay(pdMS_TO_TICKS(2));
        ret = stw_send_cmd_simple(motor, STW_CMD_READ_SPEED);    // 0xA2: 速度
        vTaskDelay(pdMS_TO_TICKS(2));
        ret = stw_send_cmd_simple(motor, STW_CMD_READ_CURRENT);  // 0xA1: Q轴电流
        vTaskDelay(pdMS_TO_TICKS(2));
        ret = stw_send_cmd_simple(motor, STW_CMD_READ_STATUS);   // 0xAE: 电压/温度/模式/故障
    }
    
    return ret;
}

esp_err_t can_motor_process_rx(void) {
    if (!g_can_initialized) return ESP_ERR_INVALID_STATE;
    
    // 每次处理 RX 时顺带检查 Bus-Off 状态 (非阻塞)
    can_bus_check_and_recover();
    
    twai_message_t msg;
    
    // 非阻塞接收
    while (twai_receive(&msg, 0) == ESP_OK) {
        // CAN RX 调试打印
        if (g_can_debug) {
            uint32_t rx_id = msg.identifier;
            bool show = (g_can_debug_filter_id == 0);
            if (!show) {
                // 检查是否匹配过滤的电机
                show = (rx_id == CAN_RX_BASE_ID + g_can_debug_filter_id) ||
                       (rx_id == CAN_TX_BASE_ID + g_can_debug_filter_id) ||
                       (rx_id == g_can_debug_filter_id) ||
                       (rx_id == (STW_TX_ID_OFFSET | g_can_debug_filter_id));
            }
            if (show) {
                printf("CAN RX 0x%03lX [%d]: %02X %02X %02X %02X %02X %02X %02X %02X\n",
                       rx_id, msg.data_length_code,
                       msg.data[0], msg.data[1], msg.data[2], msg.data[3],
                       msg.data[4], msg.data[5], msg.data[6], msg.data[7]);
            }
        }

        // 查找对应的电机
        can_motor_handle_t motor = find_motor_by_rx_id(msg.identifier);
        if (motor == NULL) {
            continue;
        }

        if (motor->brand == MOTOR_BRAND_JUCI) {
        // ---- 俱瓷协议解析 (大端序, 寄存器协议) ----
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

        } else {
        // ---- 伺泰威协议解析 (小端序, 命令码协议) ----
        uint8_t cmd = msg.data[0];
        
        switch (cmd) {
            case STW_CMD_READ_CURRENT:   // 0xA1
            case STW_CMD_TORQUE: {       // 0xC0 (应答与 0xA1 格式一致)
                // [1-4]: 4s Q轴电流, 单位 0.001A, 小端
                int32_t cur_raw = (int32_t)((uint32_t)msg.data[1] |
                                            ((uint32_t)msg.data[2] << 8) |
                                            ((uint32_t)msg.data[3] << 16) |
                                            ((uint32_t)msg.data[4] << 24));
                motor->state.current = cur_raw / STW_SCALE_CURRENT * motor->dir;
                motor->state.is_online = true;
                motor->state.last_update = xTaskGetTickCount() * portTICK_PERIOD_MS;
                break;
            }
            
            case STW_CMD_READ_SPEED:     // 0xA2
            case STW_CMD_SPEED: {        // 0xC1 (应答与 0xA2 格式一致)
                // [1-4]: 4s 速度, 单位 0.01RPM, 小端
                int32_t spd_raw = (int32_t)((uint32_t)msg.data[1] |
                                            ((uint32_t)msg.data[2] << 8) |
                                            ((uint32_t)msg.data[3] << 16) |
                                            ((uint32_t)msg.data[4] << 24));
                motor->state.speed = spd_raw / STW_SCALE_SPEED * motor->dir;
                motor->state.is_online = true;
                motor->state.last_update = xTaskGetTickCount() * portTICK_PERIOD_MS;
                break;
            }
            
            case STW_CMD_READ_ANGLE:     // 0xA3
            case STW_CMD_ABS_POS:        // 0xC2
            case STW_CMD_REL_POS:        // 0xC3
            case STW_CMD_RETURN_ORIGIN: {// 0xC4 (应答与 0xA3 格式一致)
                // [1-2]: 2u 单圈绝对值角度 (小端)
                // [3-6]: 4s 多圈绝对值角度 (小端), count → degree
                int32_t multi_raw = (int32_t)((uint32_t)msg.data[3] |
                                              ((uint32_t)msg.data[4] << 8) |
                                              ((uint32_t)msg.data[5] << 16) |
                                              ((uint32_t)msg.data[6] << 24));
                motor->state.position = multi_raw * STW_POS_COUNT_TO_DEG * motor->dir;
                motor->state.is_online = true;
                motor->state.last_update = xTaskGetTickCount() * portTICK_PERIOD_MS;
                break;
            }
            
            case STW_CMD_READ_COMPACT: { // 0xA4
                // [1]: 1u 温度 (℃)
                // [2-3]: 2s Q轴电流 0.001A (小端)
                // [4-5]: 2s 速度 0.01RPM (小端)
                // [6-7]: 2u 单圈角度 (小端)
                motor->state.motor_temp = (float)msg.data[1];
                int16_t cur_raw = (int16_t)((uint16_t)msg.data[2] |
                                            ((uint16_t)msg.data[3] << 8));
                motor->state.current = cur_raw / STW_SCALE_CURRENT * motor->dir;
                int16_t spd_raw = (int16_t)((uint16_t)msg.data[4] |
                                            ((uint16_t)msg.data[5] << 8));
                motor->state.speed = spd_raw / STW_SCALE_SPEED * motor->dir;
                uint16_t ang_raw = (uint16_t)msg.data[6] |
                                   ((uint16_t)msg.data[7] << 8);
                // 单圈角度 (不更新多圈 position, 仅补充信息)
                (void)ang_raw;
                motor->state.is_online = true;
                motor->state.last_update = xTaskGetTickCount() * portTICK_PERIOD_MS;
                break;
            }
            
            case STW_CMD_READ_STATUS:    // 0xAE
            case STW_CMD_IDLE: {         // 0xCF (应答与 0xAE 格式一致)
                // [1-2]: 2u 母线电压 0.01V (小端)
                // [3-4]: 2u 母线电流 0.01A (小端)
                // [5]: 1u 温度 (℃)
                // [6]: 1u 运行模式
                // [7]: 1u 故障码
                uint16_t vol_raw = (uint16_t)msg.data[1] |
                                   ((uint16_t)msg.data[2] << 8);
                motor->state.voltage = vol_raw / STW_SCALE_VOLTAGE;
                uint16_t bus_cur = (uint16_t)msg.data[3] |
                                   ((uint16_t)msg.data[4] << 8);
                motor->state.current = bus_cur / STW_SCALE_BUS_CURRENT;
                motor->state.motor_temp = (float)msg.data[5];
                motor->state.driver_temp = (float)msg.data[5]; // STW 只有一个温度
                // 故障码: 位映射转为 uint32_t
                motor->state.error_code = (uint32_t)msg.data[7];
                motor->state.is_online = true;
                motor->state.last_update = xTaskGetTickCount() * portTICK_PERIOD_MS;
                break;
            }
            
            case STW_CMD_CLEAR_FAULT: {  // 0xAF
                motor->state.error_code = (uint32_t)msg.data[1];
                motor->state.is_online = true;
                motor->state.last_update = xTaskGetTickCount() * portTICK_PERIOD_MS;
                break;
            }
            
            case STW_CMD_READ_MOTOR_INFO: { // 0xB0
                // DLC=7: [1]=pole_pairs(1u), [2-5]=Kt(4f LE), [6]=gear_ratio(1u)
                if (msg.data_length_code >= 7) {
                    motor->stw_info.pole_pairs = msg.data[1];
                    uint32_t kt_raw = (uint32_t)msg.data[2] |
                                      ((uint32_t)msg.data[3] << 8) |
                                      ((uint32_t)msg.data[4] << 16) |
                                      ((uint32_t)msg.data[5] << 24);
                    memcpy(&motor->stw_info.kt, &kt_raw, sizeof(float));
                    motor->stw_info.gear_ratio = msg.data[6];
                    motor->stw_info_valid = true;
                }
                motor->state.is_online = true;
                motor->state.last_update = xTaskGetTickCount() * portTICK_PERIOD_MS;
                break;
            }

            case STW_CMD_SET_MAX_SPEED:     // 0xB2
            case STW_CMD_SET_MAX_CURRENT:   // 0xB3
            case STW_CMD_SET_TORQUE_SLOPE:  // 0xB4
            case STW_CMD_SET_ACCEL: {       // 0xB5
                // 应答格式与发送一致 (DLC=5, [1-4]=4u), 确认收到即可
                motor->state.is_online = true;
                motor->state.last_update = xTaskGetTickCount() * portTICK_PERIOD_MS;
                break;
            }

            case STW_CMD_POS_KP:    // 0xB6
            case STW_CMD_POS_KI:    // 0xB7
            case STW_CMD_SPD_KP:    // 0xB8
            case STW_CMD_SPD_KI: {  // 0xB9
                // DLC=5: [1-4]=value(4f IEEE 754 LE)
                if (msg.data_length_code >= 5) {
                    uint32_t f_raw = (uint32_t)msg.data[1] |
                                     ((uint32_t)msg.data[2] << 8) |
                                     ((uint32_t)msg.data[3] << 16) |
                                     ((uint32_t)msg.data[4] << 24);
                    float val;
                    memcpy(&val, &f_raw, sizeof(float));
                    int idx = stw_pid_cmd_to_index(cmd);
                    if (idx >= 0) {
                        motor->stw_pid[idx] = val;
                        motor->stw_pid_valid[idx] = true;
                    }
                }
                motor->state.is_online = true;
                motor->state.last_update = xTaskGetTickCount() * portTICK_PERIOD_MS;
                break;
            }

            case STW_CMD_MIT_CONFIG: {  // 0xF0
                // DLC=7 应答: [1-2]=Pos_Max(2u LE, 0.1rad), [3-4]=Vel_Max, [5-6]=T_Max
                if (msg.data_length_code >= 7) {
                    uint16_t pm = (uint16_t)msg.data[1] | ((uint16_t)msg.data[2] << 8);
                    uint16_t vm = (uint16_t)msg.data[3] | ((uint16_t)msg.data[4] << 8);
                    uint16_t tm = (uint16_t)msg.data[5] | ((uint16_t)msg.data[6] << 8);
                    motor->stw_mit_pos_max = pm / STW_MIT_SCALE_POS;
                    motor->stw_mit_vel_max = vm / STW_MIT_SCALE_VEL;
                    motor->stw_mit_t_max   = tm / STW_MIT_SCALE_TORQUE;
                }
                motor->state.is_online = true;
                motor->state.last_update = xTaskGetTickCount() * portTICK_PERIOD_MS;
                break;
            }

            case STW_CMD_MIT_READ_STATE: {  // 0xF1
                // DLC=7: [1-2]=pos(16bit), [3]-[4]b7:4=vel(12bit),
                //        [4]b3:0-[5]=torque(12bit), [6]=status(1u)
                if (msg.data_length_code >= 7) {
                    uint16_t pos_raw = (uint16_t)msg.data[1] | ((uint16_t)msg.data[2] << 8);
                    uint16_t vel_raw = ((uint16_t)msg.data[3] << 4) | ((msg.data[4] >> 4) & 0x0F);
                    uint16_t trq_raw = ((uint16_t)(msg.data[4] & 0x0F) << 8) | msg.data[5];
                    float pm = motor->stw_mit_pos_max;
                    float vm = motor->stw_mit_vel_max;
                    float tm = motor->stw_mit_t_max;
                    motor->stw_mit_state.position = ((float)pos_raw / 65535.0f * 2.0f * pm - pm) * motor->dir;
                    motor->stw_mit_state.velocity = ((float)vel_raw / 4095.0f * 2.0f * vm - vm) * motor->dir;
                    motor->stw_mit_state.torque   = ((float)trq_raw / 4095.0f * 2.0f * tm - tm) * motor->dir;
                    motor->stw_mit_state.status   = msg.data[6];
                    motor->stw_mit_state_valid = true;
                }
                motor->state.is_online = true;
                motor->state.last_update = xTaskGetTickCount() * portTICK_PERIOD_MS;
                break;
            }
            
            default:
                // 其他命令应答 (版本/参数等), 仅更新在线状态
                motor->state.is_online = true;
                motor->state.last_update = xTaskGetTickCount() * portTICK_PERIOD_MS;
                break;
        }
        } // end else (STW)
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

esp_err_t can_motor_request_angle(can_motor_handle_t motor) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    if (motor->brand == MOTOR_BRAND_JUCI) {
        return read_reg_2_request(motor, REG_POSITION);
    } else {
        // STW: 0xA3 读取多圈角度
        return stw_send_cmd_simple(motor, STW_CMD_READ_ANGLE);
    }
}

esp_err_t can_motor_request_current(can_motor_handle_t motor) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    if (motor->brand == MOTOR_BRAND_JUCI) {
        return read_reg_1_request(motor, REG_CURRENT);
    } else {
        // STW: 0xA1 读取 Q 轴电流
        return stw_send_cmd_simple(motor, STW_CMD_READ_CURRENT);
    }
}

esp_err_t can_motor_request_voltage(can_motor_handle_t motor) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    if (motor->brand == MOTOR_BRAND_JUCI) {
        return read_reg_1_request(motor, REG_VOLTAGE);
    } else {
        // STW: 0xAE 命令返回电压在内的完整状态
        return stw_send_cmd_simple(motor, STW_CMD_READ_STATUS);
    }
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
    
    // JuCi 错误码 (位映射)
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
    // STW 错误码 (位映射, 低6位)
    if (error_code & STW_FAULT_VOLTAGE)     printf("  - Voltage fault\n");
    if (error_code & STW_FAULT_CURRENT)     printf("  - Current fault\n");
    if (error_code & STW_FAULT_TEMP)        printf("  - Temperature fault\n");
    if (error_code & STW_FAULT_ENCODER)     printf("  - Encoder fault\n");
    if (error_code & STW_FAULT_HARDWARE)    printf("  - Hardware fault\n");
    if (error_code & STW_FAULT_SOFTWARE)    printf("  - Software fault\n");
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
    if (motor->brand == MOTOR_BRAND_JUCI) {
        return write_reg_1(motor, REG_CONTROL_MODE, (int16_t)mode);
    } else {
        // STW 无显式模式切换命令, 模式由控制命令 (0xC0/0xC1/0xC2) 隐式切换
        ESP_LOGD(TAG, "Motor %d: STW mode set to %d (implicit)", motor->motor_id, mode);
        return ESP_OK;
    }
}

esp_err_t can_motor_set_speed(can_motor_handle_t motor, float speed_rpm) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    if (motor->brand == MOTOR_BRAND_JUCI) {
        // 速度用 4 字节 (32位), 放大 100 倍
        int32_t speed_raw = (int32_t)(speed_rpm * SCALE_SPEED);
        return write_reg_2(motor, REG_SET_SPEED, speed_raw);
    } else {
        // STW 0xC1: 速度, 单位 0.01RPM, 4s 小端
        int32_t speed_raw = (int32_t)(speed_rpm * motor->dir * STW_SCALE_SPEED);
        return stw_send_cmd_4s(motor, STW_CMD_SPEED, speed_raw);
    }
}

esp_err_t can_motor_set_position(can_motor_handle_t motor, float angle_deg, float speed_rpm) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    
    // 获取当前电机位置
    float current_pos = motor->state.position;
    
    // 计算安全的目标角度 (最短路径 + 变化量限制)
    float safe_target = compute_safe_target_angle(current_pos, angle_deg, MOTOR_MAX_ANGLE_STEP);
    
    if (motor->brand == MOTOR_BRAND_JUCI) {
        // 发送位置命令
        int32_t pos_raw = (int32_t)(safe_target * SCALE_POSITION);
        return write_reg_2(motor, REG_SET_ABS_POS, pos_raw);
    } else {
        // STW 0xC2: 绝对位置, 单位 count (16384 count/rev), 4s 小端
        (void)speed_rpm;  // STW 绝对位置命令不含速度, 需通过 0xB2 预设
        int32_t pos_raw = (int32_t)(safe_target * motor->dir * STW_POS_DEG_TO_COUNT);
        return stw_send_cmd_4s(motor, STW_CMD_ABS_POS, pos_raw);
    }
}

esp_err_t can_motor_set_torque(can_motor_handle_t motor, float torque) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    if (motor->brand == MOTOR_BRAND_JUCI) {
        int16_t torque_raw = (int16_t)(torque * SCALE_TORQUE);
        return write_reg_1(motor, REG_SET_TORQUE, torque_raw);
    } else {
        // STW 0xC0: Q 轴电流, 单位 0.001A, 4s 小端
        // torque 参数为力矩 (Nm), 先除以力矩常数 Kt 得到电流 (A)
        float current_A = torque * motor->dir / STW_TORQUE_CONSTANT;
        int32_t cur_raw = (int32_t)(current_A * STW_SCALE_CURRENT);
        return stw_send_cmd_4s(motor, STW_CMD_TORQUE, cur_raw);
    }
}

esp_err_t can_motor_pv_cmd(can_motor_handle_t motor, float position_deg, float speed_rpm) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    
    if (motor->brand == MOTOR_BRAND_JUCI) {
        // 获取当前电机位置并计算安全目标
        float current_pos = motor->state.position;
        float safe_target = compute_safe_target_angle(current_pos, position_deg, MOTOR_MAX_ANGLE_STEP);
        
        uint8_t data[8] = {0};
        
        int32_t pos_raw = (int32_t)(safe_target * SCALE_POSITION);
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
    } else {
        // STW 无 PV 组合命令, 退化为绝对位置命令
        return can_motor_set_position(motor, position_deg, speed_rpm);
    }
}

esp_err_t can_motor_pvt_cmd(can_motor_handle_t motor, float position_deg, float speed_rpm, uint8_t torque_percent) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    
    if (motor->brand == MOTOR_BRAND_JUCI) {
        // 获取当前电机位置并计算安全目标
        float current_pos = motor->state.position;
        float safe_target = compute_safe_target_angle(current_pos, position_deg, MOTOR_MAX_ANGLE_STEP);
        
        uint8_t data[8] = {0};
        
        int32_t pos_raw = (int32_t)(safe_target * SCALE_POSITION);
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
    } else {
        // STW 无 PVT 组合命令, 退化为绝对位置命令
        (void)torque_percent;
        return can_motor_set_position(motor, position_deg, speed_rpm);
    }
}

esp_err_t can_motor_set_rel_position(can_motor_handle_t motor, float angle_deg) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    if (motor->brand == MOTOR_BRAND_JUCI) {
        int32_t pos_raw = (int32_t)(angle_deg * SCALE_POSITION);
        return write_reg_2(motor, REG_SET_REL_POS, pos_raw);
    } else {
        // STW 0xC3: 相对位置, 单位 count, 4s 小端
        int32_t pos_raw = (int32_t)(angle_deg * motor->dir * STW_POS_DEG_TO_COUNT);
        return stw_send_cmd_4s(motor, STW_CMD_REL_POS, pos_raw);
    }
}

esp_err_t can_motor_set_low_speed(can_motor_handle_t motor, float speed_rpm) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    if (motor->brand == MOTOR_BRAND_JUCI) {
        // 低速模式：范围 ±300rpm, 放大100倍
        int16_t speed_raw = (int16_t)(speed_rpm * SCALE_SPEED);
        return write_reg_1(motor, REG_SET_LOW_SPEED, speed_raw);
    } else {
        // STW 无低速大扭模式, 用普通速度命令替代
        return can_motor_set_speed(motor, speed_rpm);
    }
}

esp_err_t can_motor_enter_closed_loop(can_motor_handle_t motor) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    if (motor->brand == MOTOR_BRAND_JUCI) {
        return write_reg_1(motor, REG_CLOSE_LOOP, 0x0001);
    } else {
        // STW 无显式闭环命令, 发送零电流命令激活电机
        return stw_send_cmd_4s(motor, STW_CMD_TORQUE, 0);
    }
}

esp_err_t can_motor_set_idle(can_motor_handle_t motor) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    if (motor->brand == MOTOR_BRAND_JUCI) {
        return write_reg_1(motor, REG_IDLE, 0x0001);
    } else {
        // STW 0xCF: 关闭输出, 电机进入自由态
        return stw_send_cmd_simple(motor, STW_CMD_IDLE);
    }
}

esp_err_t can_motor_set_origin(can_motor_handle_t motor) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    ESP_LOGI(TAG, "Motor %d: Setting current position as origin (zero)", motor->motor_id);
    if (motor->brand == MOTOR_BRAND_JUCI) {
        return write_reg_1(motor, REG_SET_ORIGIN, 0x0001);
    } else {
        // STW 0xB1: 设置当前位置为原点, 断电保存
        return stw_send_cmd_simple(motor, STW_CMD_SET_ORIGIN);
    }
}

esp_err_t can_motor_reboot(can_motor_handle_t motor) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    ESP_LOGI(TAG, "Motor %d: Rebooting", motor->motor_id);
    if (motor->brand == MOTOR_BRAND_JUCI) {
        return write_reg_1(motor, REG_REBOOT, 0x0001);
    } else {
        // STW 0x00: 重启, 附带固定数据 0xFF00FF00FF00FF, 不应答
        uint8_t data[8] = {
            STW_CMD_REBOOT,
            STW_REBOOT_DATA_1, STW_REBOOT_DATA_2,
            STW_REBOOT_DATA_3, STW_REBOOT_DATA_4,
            STW_REBOOT_DATA_5, STW_REBOOT_DATA_6,
            STW_REBOOT_DATA_7
        };
        return can_send_frame(motor->tx_frame_id, data, 8);
    }
}

esp_err_t can_motor_save(can_motor_handle_t motor) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    ESP_LOGI(TAG, "Motor %d: Saving parameters to Flash", motor->motor_id);
    if (motor->brand == MOTOR_BRAND_JUCI) {
        return write_reg_1(motor, REG_SAVE, 0x0001);
    } else {
        // STW 参数保存通过上位机, CAN 协议无此命令
        ESP_LOGW(TAG, "Motor %d: STW motor save not supported via CAN", motor->motor_id);
        return ESP_ERR_NOT_SUPPORTED;
    }
}

esp_err_t can_motor_calibrate(can_motor_handle_t motor) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    ESP_LOGW(TAG, "Motor %d: Starting calibration - motor will rotate!", motor->motor_id);
    if (motor->brand == MOTOR_BRAND_JUCI) {
        return write_reg_1(motor, REG_CALIBRATE, 0x0001);
    } else {
        // STW 校准通过上位机, CAN 协议无此命令
        ESP_LOGW(TAG, "Motor %d: STW motor calibrate not supported via CAN", motor->motor_id);
        return ESP_ERR_NOT_SUPPORTED;
    }
}

esp_err_t can_motor_erase(can_motor_handle_t motor) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    ESP_LOGW(TAG, "Motor %d: Erasing parameters - will need recalibration!", motor->motor_id);
    if (motor->brand == MOTOR_BRAND_JUCI) {
        return write_reg_1(motor, REG_ERASE, 0x0001);
    } else {
        // STW 擦除通过上位机, CAN 协议无此命令
        ESP_LOGW(TAG, "Motor %d: STW motor erase not supported via CAN", motor->motor_id);
        return ESP_ERR_NOT_SUPPORTED;
    }
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

// ============================================================================
// STW 专用命令实现
// ============================================================================

// ---------- 0xB0: 读取电机信息 ----------

esp_err_t can_motor_stw_request_motor_info(can_motor_handle_t motor) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    if (motor->brand == MOTOR_BRAND_STW) {
        return stw_send_cmd_simple(motor, STW_CMD_READ_MOTOR_INFO);
    }
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t can_motor_stw_get_motor_info(can_motor_handle_t motor, stw_motor_info_t *info) {
    if (motor == NULL || info == NULL) return ESP_ERR_INVALID_ARG;
    if (motor->brand == MOTOR_BRAND_STW) {
        if (!motor->stw_info_valid) return ESP_ERR_NOT_FOUND;
        *info = motor->stw_info;
        return ESP_OK;
    }
    return ESP_ERR_NOT_SUPPORTED;
}

// ---------- 0xB2~0xB5: 参数设置 ----------

esp_err_t can_motor_stw_set_max_speed(can_motor_handle_t motor, float speed_rpm) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    if (motor->brand == MOTOR_BRAND_STW) {
        uint32_t raw = (uint32_t)(speed_rpm * STW_SCALE_SPEED);
        return stw_send_cmd_4u(motor, STW_CMD_SET_MAX_SPEED, raw);
    }
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t can_motor_stw_set_max_current(can_motor_handle_t motor, float current_a) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    if (motor->brand == MOTOR_BRAND_STW) {
        uint32_t raw = (uint32_t)(current_a * STW_SCALE_CURRENT);
        return stw_send_cmd_4u(motor, STW_CMD_SET_MAX_CURRENT, raw);
    }
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t can_motor_stw_set_torque_slope(can_motor_handle_t motor, float slope) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    if (motor->brand == MOTOR_BRAND_STW) {
        uint32_t raw = (uint32_t)(slope * STW_SCALE_CURRENT);  // 单位 0.001A/s
        return stw_send_cmd_4u(motor, STW_CMD_SET_TORQUE_SLOPE, raw);
    }
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t can_motor_stw_set_accel(can_motor_handle_t motor, float accel) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    if (motor->brand == MOTOR_BRAND_STW) {
        uint32_t raw = (uint32_t)(accel * STW_SCALE_SPEED);  // 单位 0.01RPM/s
        return stw_send_cmd_4u(motor, STW_CMD_SET_ACCEL, raw);
    }
    return ESP_ERR_NOT_SUPPORTED;
}

// ---------- 0xB6~0xB9: PID 参数读/写 ----------

esp_err_t can_motor_stw_request_pid(can_motor_handle_t motor, uint8_t cmd) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    if (motor->brand == MOTOR_BRAND_STW) {
        if (stw_pid_cmd_to_index(cmd) < 0) return ESP_ERR_INVALID_ARG;
        return stw_send_cmd_simple(motor, cmd);
    }
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t can_motor_stw_write_pid(can_motor_handle_t motor, uint8_t cmd, float value) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    if (motor->brand == MOTOR_BRAND_STW) {
        if (stw_pid_cmd_to_index(cmd) < 0) return ESP_ERR_INVALID_ARG;
        return stw_send_cmd_float(motor, cmd, value);
    }
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t can_motor_stw_get_pid(can_motor_handle_t motor, uint8_t cmd, float *value) {
    if (motor == NULL || value == NULL) return ESP_ERR_INVALID_ARG;
    if (motor->brand == MOTOR_BRAND_STW) {
        int idx = stw_pid_cmd_to_index(cmd);
        if (idx < 0) return ESP_ERR_INVALID_ARG;
        if (!motor->stw_pid_valid[idx]) return ESP_ERR_NOT_FOUND;
        *value = motor->stw_pid[idx];
        return ESP_OK;
    }
    return ESP_ERR_NOT_SUPPORTED;
}

// ---------- MIT 运控模式 ----------

esp_err_t can_motor_stw_mit_set_config(can_motor_handle_t motor,
                                       float pos_max, float vel_max, float t_max) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    if (motor->brand == MOTOR_BRAND_STW) {
        uint8_t data[7];
        data[0] = STW_CMD_MIT_CONFIG;
        uint16_t pos_raw = (uint16_t)(pos_max * STW_MIT_SCALE_POS);    // 0.1 rad
        uint16_t vel_raw = (uint16_t)(vel_max * STW_MIT_SCALE_VEL);    // 0.01 rad/s
        uint16_t t_raw   = (uint16_t)(t_max   * STW_MIT_SCALE_TORQUE); // 0.01 Nm
        data[1] = (uint8_t)(pos_raw & 0xFF);
        data[2] = (uint8_t)((pos_raw >> 8) & 0xFF);
        data[3] = (uint8_t)(vel_raw & 0xFF);
        data[4] = (uint8_t)((vel_raw >> 8) & 0xFF);
        data[5] = (uint8_t)(t_raw & 0xFF);
        data[6] = (uint8_t)((t_raw >> 8) & 0xFF);
        // 更新本地缓存 (用于后续解码)
        motor->stw_mit_pos_max = pos_max;
        motor->stw_mit_vel_max = vel_max;
        motor->stw_mit_t_max   = t_max;
        return can_send_frame(motor->tx_frame_id, data, 7);
    }
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t can_motor_stw_mit_request_config(can_motor_handle_t motor) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    if (motor->brand == MOTOR_BRAND_STW) {
        return stw_send_cmd_simple(motor, STW_CMD_MIT_CONFIG);
    }
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t can_motor_stw_mit_request_state(can_motor_handle_t motor) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    if (motor->brand == MOTOR_BRAND_STW) {
        return stw_send_cmd_simple(motor, STW_CMD_MIT_READ_STATE);
    }
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t can_motor_stw_mit_get_state(can_motor_handle_t motor, stw_mit_state_t *state) {
    if (motor == NULL || state == NULL) return ESP_ERR_INVALID_ARG;
    if (motor->brand == MOTOR_BRAND_STW) {
        if (!motor->stw_mit_state_valid) return ESP_ERR_NOT_FOUND;
        *state = motor->stw_mit_state;
        return ESP_OK;
    }
    return ESP_ERR_NOT_SUPPORTED;
}

esp_err_t can_motor_stw_mit_control(can_motor_handle_t motor,
                                    float target_pos, float target_vel,
                                    float kp, float kd, float target_torque) {
    if (motor == NULL) return ESP_ERR_INVALID_ARG;
    if (motor->brand == MOTOR_BRAND_STW) {
        float pm = motor->stw_mit_pos_max;
        float vm = motor->stw_mit_vel_max;
        float tm = motor->stw_mit_t_max;

        // 方向取反: 算法 CCW 正 → STW CW 正
        float dir_pos = target_pos * motor->dir;
        float dir_vel = target_vel * motor->dir;
        float dir_trq = target_torque * motor->dir;

        uint16_t pos_int = (uint16_t)((dir_pos + pm) / (2.0f * pm) * 65535.0f);
        uint16_t vel_int = (uint16_t)((dir_vel + vm) / (2.0f * vm) * 4095.0f);
        uint16_t kp_int  = (uint16_t)(kp / 500.0f * 4095.0f);
        uint16_t kd_int  = (uint16_t)(kd / 5.0f   * 4095.0f);
        uint16_t trq_int = (uint16_t)((dir_trq + tm) / (2.0f * tm) * 4095.0f);

        uint8_t data[8];
        data[0] = (uint8_t)(pos_int >> 8);
        data[1] = (uint8_t)(pos_int & 0xFF);
        data[2] = (uint8_t)(vel_int >> 4);
        data[3] = (uint8_t)(((vel_int & 0x0F) << 4) | ((kp_int >> 8) & 0x0F));
        data[4] = (uint8_t)(kp_int & 0xFF);
        data[5] = (uint8_t)(kd_int >> 4);
        data[6] = (uint8_t)(((kd_int & 0x0F) << 4) | ((trq_int >> 8) & 0x0F));
        data[7] = (uint8_t)(trq_int & 0xFF);

        uint32_t mit_id = STW_MIT_ID_OFFSET | motor->motor_id;
        return can_send_frame(mit_id, data, 8);
    }
    return ESP_ERR_NOT_SUPPORTED;
}
