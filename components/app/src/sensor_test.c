/**
 * @file sensor_test.c
 * @brief 传感器测试模块实现 (按键、温湿度等)
 * @author Bubble
 * @date 2026-01-16
 * 
 * 使用方法：
 * 1. 在串口终端输入 sensor help 查看命令
 * 2. 测试按键: btn init, btn read, btn start
 * 3. 测试温湿度: sht init, sht read, sht start
 */

#include "sensor_test.h"
#include "button_driver.h"
#include "sht30_driver.h"
#include "power_detect.h"
#include "config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

static const char *TAG = "SENSOR_TEST";

// ============================================================================
// 按键测试
// ============================================================================

static bool g_btn_initialized = false;
static bool g_btn_monitoring = false;
static TaskHandle_t g_btn_task = NULL;

// 按键回调函数
static void button_callback(button_id_t button, button_event_t event) {
    const char *btn_name = (button == BUTTON_1) ? "BTN1" : "BTN2";
    const char *event_name;
    
    switch (event) {
        case BUTTON_EVENT_PRESS:
            event_name = "PRESSED";
            break;
        case BUTTON_EVENT_RELEASE:
            event_name = "RELEASED";
            break;
        case BUTTON_EVENT_LONG_PRESS:
            event_name = "LONG_PRESS";
            break;
        case BUTTON_EVENT_DOUBLE_CLICK:
            event_name = "DOUBLE_CLICK";
            break;
        default:
            event_name = "UNKNOWN";
    }
    
    printf("[%s] %s\n", btn_name, event_name);
}

// 按键监控任务
static void btn_monitor_task(void *arg) {
    while (g_btn_monitoring) {
        button_poll();
        vTaskDelay(pdMS_TO_TICKS(20));  // 20ms 轮询间隔
    }
    vTaskDelete(NULL);
    g_btn_task = NULL;
}

static void cmd_btn_init(void) {
    if (g_btn_initialized) {
        printf("Button already initialized\n");
        return;
    }
    
    esp_err_t ret = button_init();
    if (ret == ESP_OK) {
        button_register_callback(button_callback);
        g_btn_initialized = true;
        printf("Button initialized (BTN1=GPIO%d, BTN2=GPIO%d)\n", BUTTON1_PIN, BUTTON2_PIN);
    } else {
        printf("Button init failed: %s\n", esp_err_to_name(ret));
    }
}

static void cmd_btn_deinit(void) {
    if (!g_btn_initialized) {
        printf("Button not initialized\n");
        return;
    }
    
    // 停止监控
    g_btn_monitoring = false;
    if (g_btn_task) {
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    
    button_deinit();
    g_btn_initialized = false;
    printf("Button deinitialized\n");
}

static void cmd_btn_read(void) {
    if (!g_btn_initialized) {
        printf("Button not initialized, run 'btn init' first\n");
        return;
    }
    
    bool btn1 = button_is_pressed(BUTTON_1);
    bool btn2 = button_is_pressed(BUTTON_2);
    
    printf("Button status:\n");
    printf("  BTN1 (GPIO%d): %s\n", BUTTON1_PIN, btn1 ? "PRESSED" : "RELEASED");
    printf("  BTN2 (GPIO%d): %s\n", BUTTON2_PIN, btn2 ? "PRESSED" : "RELEASED");
}

static void cmd_btn_start(void) {
    if (!g_btn_initialized) {
        printf("Button not initialized, run 'btn init' first\n");
        return;
    }
    
    if (g_btn_monitoring) {
        printf("Button monitoring already running\n");
        return;
    }
    
    g_btn_monitoring = true;
    xTaskCreate(btn_monitor_task, "btn_mon", 4096, NULL, 5, &g_btn_task);
    printf("Button monitoring started. Press any button...\n");
    printf("(Use 'btn stop' to stop)\n");
}

static void cmd_btn_stop(void) {
    if (!g_btn_monitoring) {
        printf("Button monitoring not running\n");
        return;
    }
    
    g_btn_monitoring = false;
    printf("Button monitoring stopped\n");
}

// ============================================================================
// SHT30 温湿度传感器测试
// ============================================================================

static bool g_sht_initialized = false;
static bool g_sht_monitoring = false;
static TaskHandle_t g_sht_task = NULL;

// 电源检测初始化状态
static bool g_power_initialized = false;

// SHT30 监控任务
static void sht_monitor_task(void *arg) {
    while (g_sht_monitoring) {
        float temp, hum;
        esp_err_t ret = sht30_read(&temp, &hum);
        
        if (ret == ESP_OK) {
            printf("[SHT30] Temp: %.2f°C, Humidity: %.2f%%\n", temp, hum);
        } else {
            printf("[SHT30] Read error: %s\n", esp_err_to_name(ret));
        }
        
        vTaskDelay(pdMS_TO_TICKS(1000));  // 1秒间隔
    }
    vTaskDelete(NULL);
    g_sht_task = NULL;
}

static void cmd_sht_init(void) {
    if (g_sht_initialized) {
        printf("SHT30 already initialized\n");
        return;
    }
    
    printf("Initializing SHT30 (SDA=GPIO%d, SCL=GPIO%d, Addr=0x%02X)...\n",
           I2C1_SDA_PIN, I2C1_SCL_PIN, SHT30_I2C_ADDR);
    
    esp_err_t ret = sht30_init(I2C1_SDA_PIN, I2C1_SCL_PIN, SHT30_I2C_ADDR);
    if (ret == ESP_OK) {
        g_sht_initialized = true;
        printf("SHT30 initialized successfully\n");
        
        // 尝试读取一次
        float temp, hum;
        ret = sht30_read(&temp, &hum);
        if (ret == ESP_OK) {
            printf("Initial reading: Temp=%.2f°C, Humidity=%.2f%%\n", temp, hum);
        } else {
            printf("Warning: Initial read failed (%s)\n", esp_err_to_name(ret));
            printf("Check if sensor is connected properly\n");
        }
    } else {
        printf("SHT30 init failed: %s\n", esp_err_to_name(ret));
    }
}

static void cmd_sht_deinit(void) {
    if (!g_sht_initialized) {
        printf("SHT30 not initialized\n");
        return;
    }
    
    // 停止监控
    g_sht_monitoring = false;
    if (g_sht_task) {
        vTaskDelay(pdMS_TO_TICKS(1100));
    }
    
    sht30_deinit();
    g_sht_initialized = false;
    printf("SHT30 deinitialized\n");
}

static void cmd_sht_read(void) {
    if (!g_sht_initialized) {
        printf("SHT30 not initialized, run 'sht init' first\n");
        return;
    }
    
    sht30_data_t data;
    esp_err_t ret = sht30_read_single(&data, SHT30_REPEATABILITY_HIGH);
    
    if (ret == ESP_OK && data.valid) {
        printf("SHT30 Reading:\n");
        printf("  Temperature: %.2f °C (%.2f °F)\n", data.temperature, 
               data.temperature * 9.0f / 5.0f + 32.0f);
        printf("  Humidity:    %.2f %%RH\n", data.humidity);
        printf("  Timestamp:   %lu ms\n", data.timestamp_ms);
    } else {
        printf("SHT30 read failed: %s\n", esp_err_to_name(ret));
    }
}

static void cmd_sht_start(void) {
    if (!g_sht_initialized) {
        printf("SHT30 not initialized, run 'sht init' first\n");
        return;
    }
    
    if (g_sht_monitoring) {
        printf("SHT30 monitoring already running\n");
        return;
    }
    
    g_sht_monitoring = true;
    xTaskCreate(sht_monitor_task, "sht_mon", 4096, NULL, 5, &g_sht_task);
    printf("SHT30 monitoring started (1 second interval)\n");
    printf("(Use 'sht stop' to stop)\n");
}

static void cmd_sht_stop(void) {
    if (!g_sht_monitoring) {
        printf("SHT30 monitoring not running\n");
        return;
    }
    
    g_sht_monitoring = false;
    printf("SHT30 monitoring stopped\n");
}

static void cmd_sht_status(void) {
    if (!g_sht_initialized) {
        printf("SHT30 not initialized\n");
        return;
    }
    
    uint16_t status;
    esp_err_t ret = sht30_read_status(&status);
    
    if (ret == ESP_OK) {
        printf("SHT30 Status Register: 0x%04X\n", status);
        printf("  Alert pending:     %s\n", (status & 0x8000) ? "Yes" : "No");
        printf("  Heater:            %s\n", (status & 0x2000) ? "ON" : "OFF");
        printf("  RH tracking alert: %s\n", (status & 0x0800) ? "Yes" : "No");
        printf("  T tracking alert:  %s\n", (status & 0x0400) ? "Yes" : "No");
        printf("  System reset:      %s\n", (status & 0x0010) ? "Yes" : "No");
        printf("  Command status:    %s\n", (status & 0x0002) ? "Failed" : "OK");
        printf("  Checksum status:   %s\n", (status & 0x0001) ? "Failed" : "OK");
    } else {
        printf("Failed to read status: %s\n", esp_err_to_name(ret));
    }
}

static void cmd_sht_heater(const char *state) {
    if (!g_sht_initialized) {
        printf("SHT30 not initialized, run 'sht init' first\n");
        return;
    }
    
    bool enable = (strcmp(state, "on") == 0);
    esp_err_t ret = sht30_heater(enable);
    
    if (ret == ESP_OK) {
        printf("SHT30 heater %s\n", enable ? "ON" : "OFF");
    } else {
        printf("Failed to set heater: %s\n", esp_err_to_name(ret));
    }
}

static void cmd_sht_reset(void) {
    if (!g_sht_initialized) {
        printf("SHT30 not initialized, run 'sht init' first\n");
        return;
    }
    
    esp_err_t ret = sht30_soft_reset();
    if (ret == ESP_OK) {
        printf("SHT30 soft reset done\n");
    } else {
        printf("Failed to reset: %s\n", esp_err_to_name(ret));
    }
}

static void cmd_sht_scan(void) {
    if (!g_sht_initialized) {
        printf("SHT30 not initialized, run 'sht init' first\n");
        return;
    }
    
    sht30_scan_i2c();
}

// ============================================================================
// 电源检测测试
// ============================================================================

// 确保电源模块已初始化
static esp_err_t ensure_power_init(void) {
    if (g_power_initialized) {
        return ESP_OK;
    }
    
    esp_err_t ret = power_init();
    if (ret == ESP_OK) {
        g_power_initialized = true;
        printf("Power module initialized\n");
    }
    return ret;
}

static void cmd_power_status(void) {
    // 确保初始化
    ensure_power_init();
    
    bool is_battery = power_is_battery();
    bool motor_enabled = power_motor_is_enabled();
    
    printf("=== Power Status ===\n");
    printf("Power source: %s\n", is_battery ? "BATTERY" : "USB");
    printf("  Detection GPIO: %d\n", POWER_DETECT_PIN);
    printf("  Level: %s (Battery=HIGH, USB=LOW)\n", is_battery ? "HIGH" : "LOW");
    printf("Motor power: %s\n", motor_enabled ? "ENABLED" : "DISABLED");
    printf("  Control GPIO: %d\n", MOTOR_POWER_PIN);
}

static void cmd_motor_power(bool enable) {
    // 确保电源模块已初始化
    esp_err_t ret = ensure_power_init();
    if (ret != ESP_OK) {
        printf("Failed to init power module: %s\n", esp_err_to_name(ret));
        return;
    }
    
    ret = power_motor_enable(enable);
    if (ret == ESP_OK) {
        printf("Motor power %s (GPIO%d = %s)\n", 
               enable ? "ENABLED" : "DISABLED",
               MOTOR_POWER_PIN,
               enable ? "HIGH" : "LOW");
    } else {
        printf("Failed to set motor power: %s\n", esp_err_to_name(ret));
    }
}

// ============================================================================
// 命令处理
// ============================================================================

void sensor_test_print_help(void) {
    printf("\n");
    printf("========== Sensor Test Commands =========\n");
    printf("--- Button Commands ---\n");
    printf("  btn init              - Initialize buttons\n");
    printf("  btn deinit            - Deinitialize buttons\n");
    printf("  btn read              - Read button status\n");
    printf("  btn start             - Start button monitoring\n");
    printf("  btn stop              - Stop button monitoring\n");
    printf("\n");
    printf("--- SHT30 Temp/Humidity Commands ---\n");
    printf("  sht init              - Initialize SHT30\n");
    printf("  sht deinit            - Deinitialize SHT30\n");
    printf("  sht read              - Read temp & humidity\n");
    printf("  sht start             - Start continuous reading\n");
    printf("  sht stop              - Stop continuous reading\n");
    printf("  sht status            - Read status register\n");
    printf("  sht heater [on|off]   - Control heater\n");
    printf("  sht reset             - Soft reset sensor\n");
    printf("  sht scan              - Scan I2C bus\n");
    printf("\n");
    printf("--- Power & Motor Power ---\n");
    printf("  power                 - Show power status\n");
    printf("  mpower on             - Enable motor power\n");
    printf("  mpower off            - Disable motor power\n");
    printf("==========================================\n\n");
}

void sensor_test_process_cmd(const char *cmd_line) {
    if (!cmd_line || strlen(cmd_line) == 0) {
        sensor_test_print_help();
        return;
    }
    
    char line[128];
    strncpy(line, cmd_line, sizeof(line) - 1);
    line[sizeof(line) - 1] = '\0';
    
    char *token = strtok(line, " \t\n\r");
    if (!token) {
        sensor_test_print_help();
        return;
    }
    
    // ========== 按键命令 ==========
    if (strcmp(token, "init") == 0) {
        // 后向兼容: sensor init = btn init + sht init
        cmd_btn_init();
        cmd_sht_init();
    }
    else if (strcmp(token, "help") == 0) {
        sensor_test_print_help();
    }
    // 其他情况不处理，由 motor_test 路由
}

// 供 motor_test.c 调用的独立命令处理
void btn_process_cmd(const char *cmd_line) {
    if (!cmd_line) return;
    
    char line[64];
    strncpy(line, cmd_line, sizeof(line) - 1);
    line[sizeof(line) - 1] = '\0';
    
    char *token = strtok(line, " \t\n\r");
    if (!token) {
        printf("Usage: btn [init|deinit|read|start|stop]\n");
        return;
    }
    
    if (strcmp(token, "init") == 0) {
        cmd_btn_init();
    } else if (strcmp(token, "deinit") == 0) {
        cmd_btn_deinit();
    } else if (strcmp(token, "read") == 0) {
        cmd_btn_read();
    } else if (strcmp(token, "start") == 0) {
        cmd_btn_start();
    } else if (strcmp(token, "stop") == 0) {
        cmd_btn_stop();
    } else {
        printf("Unknown btn command: %s\n", token);
        printf("Usage: btn [init|deinit|read|start|stop]\n");
    }
}

void sht_process_cmd(const char *cmd_line) {
    if (!cmd_line) return;
    
    char line[64];
    strncpy(line, cmd_line, sizeof(line) - 1);
    line[sizeof(line) - 1] = '\0';
    
    char *token = strtok(line, " \t\n\r");
    if (!token) {
        printf("Usage: sht [init|deinit|read|start|stop|status|heater|reset]\n");
        return;
    }
    
    if (strcmp(token, "init") == 0) {
        cmd_sht_init();
    } else if (strcmp(token, "deinit") == 0) {
        cmd_sht_deinit();
    } else if (strcmp(token, "read") == 0) {
        cmd_sht_read();
    } else if (strcmp(token, "start") == 0) {
        cmd_sht_start();
    } else if (strcmp(token, "stop") == 0) {
        cmd_sht_stop();
    } else if (strcmp(token, "status") == 0) {
        cmd_sht_status();
    } else if (strcmp(token, "heater") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token && (strcmp(token, "on") == 0 || strcmp(token, "off") == 0)) {
            cmd_sht_heater(token);
        } else {
            printf("Usage: sht heater [on|off]\n");
        }
    } else if (strcmp(token, "reset") == 0) {
        cmd_sht_reset();
    } else if (strcmp(token, "scan") == 0) {
        cmd_sht_scan();
    } else {
        printf("Unknown sht command: %s\n", token);
        printf("Usage: sht [init|deinit|read|start|stop|status|heater|reset|scan]\n");
    }
}

void power_process_cmd(void) {
    cmd_power_status();
}

void motor_power_process_cmd(const char *cmd_line) {
    if (!cmd_line) return;
    
    char line[64];
    strncpy(line, cmd_line, sizeof(line) - 1);
    line[sizeof(line) - 1] = '\0';
    
    char *token = strtok(line, " \t\n\r");
    if (!token) {
        // 没有参数，显示状态
        ensure_power_init();
        bool enabled = power_motor_is_enabled();
        printf("Motor power is %s (GPIO%d)\n", enabled ? "ENABLED" : "DISABLED", MOTOR_POWER_PIN);
        printf("Usage: mpower [on|off]\n");
        return;
    }
    
    if (strcmp(token, "on") == 0) {
        cmd_motor_power(true);
    } else if (strcmp(token, "off") == 0) {
        cmd_motor_power(false);
    } else {
        printf("Unknown mpower command: %s\n", token);
        printf("Usage: mpower [on|off]\n");
    }
}

// ============================================================================
// 初始化和反初始化
// ============================================================================

esp_err_t sensor_test_init(void) {
    ESP_LOGI(TAG, "Sensor test module initialized");
    return ESP_OK;
}

void sensor_test_deinit(void) {
    // 清理按键
    if (g_btn_monitoring) {
        g_btn_monitoring = false;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    if (g_btn_initialized) {
        button_deinit();
        g_btn_initialized = false;
    }
    
    // 清理 SHT30
    if (g_sht_monitoring) {
        g_sht_monitoring = false;
        vTaskDelay(pdMS_TO_TICKS(1100));
    }
    if (g_sht_initialized) {
        sht30_deinit();
        g_sht_initialized = false;
    }
    
    ESP_LOGI(TAG, "Sensor test module deinitialized");
}
