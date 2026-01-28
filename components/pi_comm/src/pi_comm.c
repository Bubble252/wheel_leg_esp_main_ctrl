/**
 * @file pi_comm.c
 * @brief 树莓派通信模块主实现
 * @author Bubble
 * @date 2026-01-27
 */

#include "pi_comm.h"
#include "pi_frame.h"
#include "pi_protocol.h"

#include "driver/uart.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "PI_COMM";

// ============================================================================
// 版本信息
// ============================================================================

#define FIRMWARE_VERSION    "1.0.0"
#define PROTOCOL_VERSION    0x01

// ============================================================================
// 私有变量
// ============================================================================

static bool g_initialized = false;
static TaskHandle_t g_task_handle = NULL;
static pi_frame_parser_t g_parser;
static pi_comm_callbacks_t g_callbacks = {0};
static SemaphoreHandle_t g_state_mutex = NULL;

// 连接状态
static pi_conn_state_t g_conn_state = PI_CONN_DISCONNECTED;
static uint32_t g_last_heartbeat_time = 0;
static uint8_t g_tx_seq = 0;

// 机器人状态 (用于上报)
static pi_robot_state_t g_robot_state = {0};

// 统计
static pi_comm_stats_t g_stats = {0};

// TX 缓冲区
static uint8_t g_tx_buf[PI_FRAME_MAX_SIZE];

// ============================================================================
// 私有函数声明
// ============================================================================

static void pi_comm_task(void *arg);
static void handle_frame(const pi_frame_t *frame);
static void handle_heartbeat(const pi_frame_t *frame);
static void handle_handshake(const pi_frame_t *frame);
static void handle_velocity(const pi_frame_t *frame);
static void handle_mode(const pi_frame_t *frame);
static void handle_height(const pi_frame_t *frame);
static void handle_pitch(const pi_frame_t *frame);
static void handle_roll(const pi_frame_t *frame);
static void handle_pose(const pi_frame_t *frame);
static void handle_motor_enable(const pi_frame_t *frame);
static void handle_estop(const pi_frame_t *frame);
static void handle_get_status(const pi_frame_t *frame);

static void send_frame(uint8_t cmd, const uint8_t *data, size_t data_len);
static void send_ack(uint8_t seq, uint8_t cmd);
static void send_nack(uint8_t seq, uint8_t cmd, uint8_t error_code);
static void send_status_report(void);
static void check_heartbeat_timeout(void);

static uint32_t get_time_ms(void);

// ============================================================================
// 公共函数实现
// ============================================================================

esp_err_t pi_comm_init(void) {
    if (g_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }
    
    // 创建互斥锁
    g_state_mutex = xSemaphoreCreateMutex();
    if (!g_state_mutex) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    
    // 配置 UART
    uart_config_t uart_config = {
        .baud_rate = PI_COMM_UART_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    
    esp_err_t ret = uart_driver_install(PI_COMM_UART_NUM, 1024, 1024, 0, NULL, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(ret));
        vSemaphoreDelete(g_state_mutex);
        return ret;
    }
    
    ret = uart_param_config(PI_COMM_UART_NUM, &uart_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to config UART: %s", esp_err_to_name(ret));
        uart_driver_delete(PI_COMM_UART_NUM);
        vSemaphoreDelete(g_state_mutex);
        return ret;
    }
    
    ret = uart_set_pin(PI_COMM_UART_NUM, PI_COMM_UART_TX_PIN, PI_COMM_UART_RX_PIN,
                       UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set UART pins: %s", esp_err_to_name(ret));
        uart_driver_delete(PI_COMM_UART_NUM);
        vSemaphoreDelete(g_state_mutex);
        return ret;
    }
    
    // 初始化解析器
    pi_frame_parser_init(&g_parser);
    
    // 创建任务
    BaseType_t xret = xTaskCreatePinnedToCore(
        pi_comm_task,
        "pi_comm",
        PI_COMM_TASK_STACK_SIZE,
        NULL,
        PI_COMM_TASK_PRIORITY,
        &g_task_handle,
        PI_COMM_TASK_CORE
    );
    
    if (xret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create task");
        uart_driver_delete(PI_COMM_UART_NUM);
        vSemaphoreDelete(g_state_mutex);
        return ESP_ERR_NO_MEM;
    }
    
    g_initialized = true;
    ESP_LOGI(TAG, "Pi comm initialized (UART%d, TX=%d, RX=%d, %d bps)",
             PI_COMM_UART_NUM, PI_COMM_UART_TX_PIN, PI_COMM_UART_RX_PIN,
             PI_COMM_UART_BAUD);
    
    return ESP_OK;
}

void pi_comm_deinit(void) {
    if (!g_initialized) return;
    
    if (g_task_handle) {
        vTaskDelete(g_task_handle);
        g_task_handle = NULL;
    }
    
    uart_driver_delete(PI_COMM_UART_NUM);
    
    if (g_state_mutex) {
        vSemaphoreDelete(g_state_mutex);
        g_state_mutex = NULL;
    }
    
    g_initialized = false;
    ESP_LOGI(TAG, "Pi comm deinitialized");
}

void pi_comm_register_callbacks(const pi_comm_callbacks_t *callbacks) {
    if (callbacks) {
        memcpy(&g_callbacks, callbacks, sizeof(pi_comm_callbacks_t));
    }
}

pi_conn_state_t pi_comm_get_state(void) {
    return g_conn_state;
}

bool pi_comm_is_connected(void) {
    return g_conn_state == PI_CONN_CONNECTED;
}

void pi_comm_update_state(const pi_robot_state_t *state) {
    if (!state) return;
    
    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        memcpy(&g_robot_state, state, sizeof(pi_robot_state_t));
        xSemaphoreGive(g_state_mutex);
    }
}

void pi_comm_send_error(uint8_t error_code, uint8_t severity, 
                        uint8_t source, const char *message) {
    uint8_t data[39] = {0};
    
    // timestamp (大端序)
    uint32_t ts = get_time_ms();
    write_be32(&data[0], ts);
    
    data[4] = error_code;
    data[5] = severity;
    data[6] = source;
    
    if (message) {
        strncpy((char *)&data[7], message, 32);
    }
    
    send_frame(CMD_ERROR_REPORT, data, sizeof(data));
}

void pi_comm_send_event(uint8_t event_type, const uint8_t *data, size_t data_len) {
    uint8_t buf[64] = {0};
    
    // timestamp
    uint32_t ts = get_time_ms();
    write_be32(&buf[0], ts);
    
    buf[4] = event_type;
    
    if (data && data_len > 0 && data_len <= 59) {
        memcpy(&buf[5], data, data_len);
    }
    
    send_frame(CMD_EVENT_REPORT, buf, 5 + data_len);
}

void pi_comm_get_stats(pi_comm_stats_t *stats) {
    if (stats) {
        memcpy(stats, &g_stats, sizeof(pi_comm_stats_t));
    }
}

// ============================================================================
// 私有函数实现
// ============================================================================

static uint32_t get_time_ms(void) {
    return xTaskGetTickCount() * portTICK_PERIOD_MS;
}

static void pi_comm_task(void *arg) {
    uint8_t rx_buf[256];
    pi_frame_t frame;
    uint32_t last_status_time = 0;
    
    ESP_LOGI(TAG, "Pi comm task started");
    
    while (1) {
        // 读取 UART 数据
        int len = uart_read_bytes(PI_COMM_UART_NUM, rx_buf, sizeof(rx_buf), 
                                  pdMS_TO_TICKS(10));
        
        if (len > 0) {
            // 解析帧
            for (int i = 0; i < len; i++) {
                if (pi_frame_parser_feed(&g_parser, rx_buf[i], &frame)) {
                    g_stats.rx_frames++;
                    g_stats.last_rx_time = get_time_ms();
                    handle_frame(&frame);
                }
            }
        }
        
        // 检查心跳超时
        check_heartbeat_timeout();
        
        // 周期性状态上报 (仅在连接时)
        if (g_conn_state == PI_CONN_CONNECTED) {
            uint32_t now = get_time_ms();
            if (now - last_status_time >= PI_COMM_STATUS_REPORT_INTERVAL) {
                send_status_report();
                last_status_time = now;
            }
        }
    }
}

static void handle_frame(const pi_frame_t *frame) {
    ESP_LOGD(TAG, "RX: SEQ=%02X CMD=%02X LEN=%d", frame->seq, frame->cmd, frame->data_len);
    
    switch (frame->cmd) {
        case CMD_HEARTBEAT:
            handle_heartbeat(frame);
            break;
            
        case CMD_HANDSHAKE:
            handle_handshake(frame);
            break;
            
        case CMD_SET_VELOCITY:
            handle_velocity(frame);
            break;
            
        case CMD_SET_MODE:
            handle_mode(frame);
            break;
            
        case CMD_SET_HEIGHT:
            handle_height(frame);
            break;
            
        case CMD_SET_PITCH:
            handle_pitch(frame);
            break;
            
        case CMD_SET_ROLL:
            handle_roll(frame);
            break;
            
        case CMD_SET_POSE:
            handle_pose(frame);
            break;
            
        case CMD_MOTOR_ENABLE:
            handle_motor_enable(frame);
            break;
            
        case CMD_EMERGENCY_STOP:
            handle_estop(frame);
            break;
            
        case CMD_GET_STATUS:
            handle_get_status(frame);
            break;
            
        case CMD_RESET:
            send_ack(frame->seq, frame->cmd);
            vTaskDelay(pdMS_TO_TICKS(100));
            esp_restart();
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown command: 0x%02X", frame->cmd);
            send_nack(frame->seq, frame->cmd, NACK_UNKNOWN_CMD);
            break;
    }
}

static void handle_heartbeat(const pi_frame_t *frame) {
    if (frame->data_len < sizeof(heartbeat_req_t)) {
        return;
    }
    
    g_last_heartbeat_time = get_time_ms();
    g_stats.heartbeat_count++;
    
    // 如果是断开状态，切换到已连接
    if (g_conn_state == PI_CONN_DISCONNECTED || g_conn_state == PI_CONN_RECONNECTING) {
        g_conn_state = PI_CONN_CONNECTED;
        ESP_LOGI(TAG, "Pi connected (via heartbeat)");
    }
    
    // 解析请求时间戳 (大端序)
    uint32_t req_timestamp = read_be32(frame->data);
    
    // 构建响应
    uint8_t resp[10];
    write_be32(&resp[0], req_timestamp);        // 回显时间戳
    write_be32(&resp[4], get_time_ms());        // ESP32 时间
    resp[8] = 0;                                // CPU 负载 (TODO)
    resp[9] = g_robot_state.status;             // 系统状态
    
    // 发送响应
    size_t len = pi_frame_build(g_tx_buf, sizeof(g_tx_buf), 
                                frame->seq, CMD_HEARTBEAT_ACK, resp, sizeof(resp));
    if (len > 0) {
        uart_write_bytes(PI_COMM_UART_NUM, g_tx_buf, len);
        g_stats.tx_frames++;
    }
}

static void handle_handshake(const pi_frame_t *frame) {
    if (frame->data_len < sizeof(handshake_req_t)) {
        send_nack(frame->seq, frame->cmd, NACK_INVALID_PARAM);
        return;
    }
    
    // 解析请求
    uint8_t protocol_ver = frame->data[0];
    // uint32_t capabilities = read_be32(&frame->data[1]);
    
    ESP_LOGI(TAG, "Handshake request: protocol v%d", protocol_ver);
    
    // 构建响应
    uint8_t resp[22] = {0};
    resp[0] = 0;                                // result = 成功
    resp[1] = PROTOCOL_VERSION;                 // 协议版本
    strncpy((char *)&resp[2], FIRMWARE_VERSION, 16);  // 固件版本
    write_be32(&resp[18], 0x0F);                // 能力位图
    
    // 发送响应
    size_t len = pi_frame_build(g_tx_buf, sizeof(g_tx_buf),
                                frame->seq, CMD_HANDSHAKE_ACK, resp, sizeof(resp));
    if (len > 0) {
        uart_write_bytes(PI_COMM_UART_NUM, g_tx_buf, len);
        g_stats.tx_frames++;
    }
    
    g_conn_state = PI_CONN_CONNECTED;
    g_last_heartbeat_time = get_time_ms();
    ESP_LOGI(TAG, "Pi connected (handshake)");
}

static void handle_velocity(const pi_frame_t *frame) {
    if (frame->data_len < 8) {  // 至少需要 vx + yaw_rate
        send_nack(frame->seq, frame->cmd, NACK_INVALID_PARAM);
        return;
    }
    
    // 解析 (大端序)
    float vx = read_be_float(&frame->data[0]);
    float yaw_rate = read_be_float(&frame->data[4]);
    
    ESP_LOGD(TAG, "Velocity: vx=%.3f, yaw_rate=%.3f", vx, yaw_rate);
    
    if (g_callbacks.on_velocity) {
        g_callbacks.on_velocity(vx, yaw_rate);
    }
    
    // 速度指令不需要 ACK (最新值覆盖)
}

static void handle_mode(const pi_frame_t *frame) {
    if (frame->data_len < sizeof(mode_cmd_t)) {
        send_nack(frame->seq, frame->cmd, NACK_INVALID_PARAM);
        return;
    }
    
    uint8_t mode = frame->data[0];
    
    ESP_LOGI(TAG, "Mode: %d", mode);
    
    if (g_callbacks.on_mode) {
        g_callbacks.on_mode(mode);
    }
    
    send_ack(frame->seq, frame->cmd);
}

static void handle_height(const pi_frame_t *frame) {
    if (frame->data_len < sizeof(height_cmd_t)) {
        send_nack(frame->seq, frame->cmd, NACK_INVALID_PARAM);
        return;
    }
    
    float height = read_be_float(&frame->data[0]);
    float duration = read_be_float(&frame->data[4]);
    
    ESP_LOGI(TAG, "Height: %.3f m, duration: %.2f s", height, duration);
    
    if (g_callbacks.on_height) {
        g_callbacks.on_height(height, duration);
    }
    
    send_ack(frame->seq, frame->cmd);
}

static void handle_pitch(const pi_frame_t *frame) {
    if (frame->data_len < sizeof(pitch_cmd_t)) {
        send_nack(frame->seq, frame->cmd, NACK_INVALID_PARAM);
        return;
    }
    
    float pitch = read_be_float(&frame->data[0]);
    float duration = read_be_float(&frame->data[4]);
    
    ESP_LOGI(TAG, "Pitch: %.2f deg, duration: %.2f s", pitch, duration);
    
    if (g_callbacks.on_pitch) {
        g_callbacks.on_pitch(pitch, duration);
    }
    
    send_ack(frame->seq, frame->cmd);
}

static void handle_roll(const pi_frame_t *frame) {
    if (frame->data_len < sizeof(roll_cmd_t)) {
        send_nack(frame->seq, frame->cmd, NACK_INVALID_PARAM);
        return;
    }
    
    float roll = read_be_float(&frame->data[0]);
    float duration = read_be_float(&frame->data[4]);
    
    ESP_LOGI(TAG, "Roll: %.2f deg, duration: %.2f s", roll, duration);
    
    if (g_callbacks.on_roll) {
        g_callbacks.on_roll(roll, duration);
    }
    
    send_ack(frame->seq, frame->cmd);
}

static void handle_pose(const pi_frame_t *frame) {
    if (frame->data_len < sizeof(pose_cmd_t)) {
        send_nack(frame->seq, frame->cmd, NACK_INVALID_PARAM);
        return;
    }
    
    float pitch = read_be_float(&frame->data[0]);
    float roll = read_be_float(&frame->data[4]);
    float height = read_be_float(&frame->data[8]);
    float duration = read_be_float(&frame->data[12]);
    
    ESP_LOGI(TAG, "Pose: pitch=%.2f, roll=%.2f, height=%.3f, duration=%.2f",
             pitch, roll, height, duration);
    
    if (g_callbacks.on_pose) {
        g_callbacks.on_pose(pitch, roll, height, duration);
    }
    
    send_ack(frame->seq, frame->cmd);
}

static void handle_motor_enable(const pi_frame_t *frame) {
    if (frame->data_len < sizeof(motor_enable_cmd_t)) {
        send_nack(frame->seq, frame->cmd, NACK_INVALID_PARAM);
        return;
    }
    
    bool enable = frame->data[0] != 0;
    
    ESP_LOGI(TAG, "Motor enable: %s", enable ? "ON" : "OFF");
    
    if (g_callbacks.on_motor_enable) {
        g_callbacks.on_motor_enable(enable);
    }
    
    send_ack(frame->seq, frame->cmd);
}

static void handle_estop(const pi_frame_t *frame) {
    uint8_t reason = (frame->data_len > 0) ? frame->data[0] : 0;
    
    ESP_LOGW(TAG, "EMERGENCY STOP! reason=%d", reason);
    
    if (g_callbacks.on_estop) {
        g_callbacks.on_estop(reason);
    }
    
    send_ack(frame->seq, frame->cmd);
}

static void handle_get_status(const pi_frame_t *frame) {
    send_status_report();
}

static void send_frame(uint8_t cmd, const uint8_t *data, size_t data_len) {
    g_tx_seq++;
    
    size_t len = pi_frame_build(g_tx_buf, sizeof(g_tx_buf), g_tx_seq, cmd, data, data_len);
    if (len > 0) {
        uart_write_bytes(PI_COMM_UART_NUM, g_tx_buf, len);
        g_stats.tx_frames++;
    }
}

static void send_ack(uint8_t seq, uint8_t cmd) {
    uint8_t data[2] = {seq, cmd};
    
    size_t len = pi_frame_build(g_tx_buf, sizeof(g_tx_buf), seq, CMD_ACK, data, sizeof(data));
    if (len > 0) {
        uart_write_bytes(PI_COMM_UART_NUM, g_tx_buf, len);
        g_stats.tx_frames++;
    }
}

static void send_nack(uint8_t seq, uint8_t cmd, uint8_t error_code) {
    uint8_t data[3] = {seq, cmd, error_code};
    
    size_t len = pi_frame_build(g_tx_buf, sizeof(g_tx_buf), seq, CMD_NACK, data, sizeof(data));
    if (len > 0) {
        uart_write_bytes(PI_COMM_UART_NUM, g_tx_buf, len);
        g_stats.tx_frames++;
    }
    
    g_stats.rx_errors++;
}

static void send_status_report(void) {
    uint8_t data[40];
    
    if (xSemaphoreTake(g_state_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
        // timestamp
        write_be32(&data[0], get_time_ms());
        
        // 状态字段
        data[4] = g_robot_state.mode;
        data[5] = g_robot_state.status;
        data[6] = g_robot_state.error_code;
        data[7] = g_robot_state.flags;
        
        // 姿态
        write_be_float(&data[8], g_robot_state.pitch);
        write_be_float(&data[12], g_robot_state.roll);
        write_be_float(&data[16], g_robot_state.yaw);
        
        // 速度
        write_be_float(&data[20], g_robot_state.vx_actual);
        write_be_float(&data[24], g_robot_state.yaw_rate_actual);
        
        // 其他
        write_be_float(&data[28], g_robot_state.battery_voltage);
        write_be_float(&data[32], g_robot_state.height_actual);
        
        // 填充到 40 字节
        write_be32(&data[36], 0);
        
        xSemaphoreGive(g_state_mutex);
    } else {
        memset(data, 0, sizeof(data));
        write_be32(&data[0], get_time_ms());
    }
    
    send_frame(CMD_STATUS_REPORT, data, 40);
}

static void check_heartbeat_timeout(void) {
    if (g_conn_state != PI_CONN_CONNECTED) {
        return;
    }
    
    uint32_t now = get_time_ms();
    if (now - g_last_heartbeat_time > PI_COMM_HEARTBEAT_TIMEOUT_MS) {
        ESP_LOGW(TAG, "Heartbeat timeout! Disconnecting...");
        
        g_conn_state = PI_CONN_DISCONNECTED;
        
        // 触发断连回调
        if (g_callbacks.on_disconnect) {
            g_callbacks.on_disconnect();
        }
    }
}
