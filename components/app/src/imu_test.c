/**
 * @file imu_test.c
 * @brief IMU 测试模块 - 串口命令交互
 * @author Bubble
 * @date 2026-01-16
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "wit_imu.h"
#include "wit_c_sdk.h"

static const char *TAG = "IMU_TEST";

// ============ 配置参数 ============
#define IMU_TEST_TASK_STACK     4096
#define IMU_TEST_TASK_PRIORITY  5
#define IMU_CMD_BUF_SIZE        128
#define IMU_DEFAULT_RATE        RRATE_50HZ      // 默认50Hz输出
#define IMU_DEFAULT_BANDWIDTH   BANDWIDTH_21HZ  // 默认21Hz带宽

// ============ 状态变量 ============
static TaskHandle_t s_imu_read_task = NULL;
static volatile bool s_running = false;
static volatile bool s_print_enabled = false;
static volatile uint32_t s_print_interval_ms = 100;  // 默认100ms打印一次 (10Hz显示)
static volatile uint32_t s_update_count = 0;
static volatile uint32_t s_last_count = 0;
static volatile float s_actual_rate = 0.0f;

// ============ 数据缓存 ============
static float s_acc[3] = {0};
static float s_gyro[3] = {0};
static float s_angle[3] = {0};

// ============ 帮助信息 ============
static void print_help(void) {
    printf("\n========== IMU Test Commands ==========\n");
    printf("  init          - 初始化 IMU\n");
    printf("  deinit        - 反初始化 IMU\n");
    printf("  scan          - 扫描 I2C 设备\n");
    printf("  start         - 开始持续读取\n");
    printf("  stop          - 停止读取\n");
    printf("  print [on|off]- 开启/关闭数据打印\n");
    printf("  interval <ms> - 设置打印间隔 (ms)\n");
    printf("  read          - 单次读取数据\n");
    printf("  quat          - 读取四元数\n");
    printf("  temp          - 读取温度\n");
    printf("  rate <0-12>   - 设置输出速率\n");
    printf("                  0=0.2Hz, 1=0.5Hz, 2=1Hz, 3=2Hz\n");
    printf("                  4=5Hz, 5=10Hz, 6=20Hz, 7=50Hz\n");
    printf("                  8=100Hz, 9=125Hz, 10=200Hz\n");
    printf("                  11=单次, 12=不输出\n");
    printf("  bw <0-6>      - 设置滤波带宽\n");
    printf("                  0=256Hz, 1=184Hz, 2=94Hz, 3=44Hz\n");
    printf("                  4=21Hz, 5=10Hz, 6=5Hz\n");
    printf("  cali acc      - 加速度计校准 (保持水平静止)\n");
    printf("  cali mag      - 磁力计校准 (缓慢旋转)\n");
    printf("  cali stop     - 停止校准\n");
    printf("  status        - 显示当前状态\n");
    printf("  help          - 显示帮助\n");
    printf("========================================\n\n");
}

// ============ 状态显示 ============
static void print_status(void) {
    printf("\n========== IMU Status ==========\n");
    printf("  初始化: %s\n", wit_imu_is_initialized() ? "是" : "否");
    printf("  运行中: %s\n", s_running ? "是" : "否");
    printf("  打印:   %s\n", s_print_enabled ? "开启" : "关闭");
    printf("  打印间隔: %lu ms\n", s_print_interval_ms);
    printf("  实际刷新率: %.1f Hz\n", s_actual_rate);
    printf("  总更新次数: %lu\n", s_update_count);
    printf("================================\n\n");
}

// ============ IMU 读取任务 ============
static void imu_read_task(void *arg) {
    TickType_t last_print_tick = xTaskGetTickCount();
    TickType_t last_rate_tick = xTaskGetTickCount();
    const TickType_t update_period = pdMS_TO_TICKS(20);  // 50Hz 更新
    
    ESP_LOGI(TAG, "IMU read task started");
    
    while (s_running) {
        // 更新数据
        if (wit_imu_update() == 0) {
            wit_imu_get_acc(&s_acc[0], &s_acc[1], &s_acc[2]);
            wit_imu_get_gyro(&s_gyro[0], &s_gyro[1], &s_gyro[2]);
            wit_imu_get_angle(&s_angle[0], &s_angle[1], &s_angle[2]);
            s_update_count++;
        }
        
        // 计算实际刷新率 (每秒计算一次)
        TickType_t now = xTaskGetTickCount();
        if ((now - last_rate_tick) >= pdMS_TO_TICKS(1000)) {
            s_actual_rate = (float)(s_update_count - s_last_count);
            s_last_count = s_update_count;
            last_rate_tick = now;
        }
        
        // 打印数据
        if (s_print_enabled && ((now - last_print_tick) >= pdMS_TO_TICKS(s_print_interval_ms))) {
            printf("\033[2J\033[H");  // 清屏
            printf("============ IMU Data (%.1f Hz) ============\n", s_actual_rate);
            printf("Acc:   X=%+7.3f  Y=%+7.3f  Z=%+7.3f (g)\n", 
                   s_acc[0], s_acc[1], s_acc[2]);
            printf("Gyro:  X=%+8.2f  Y=%+8.2f  Z=%+8.2f (°/s)\n", 
                   s_gyro[0], s_gyro[1], s_gyro[2]);
            printf("Angle: Roll=%+7.2f  Pitch=%+7.2f  Yaw=%+7.2f (°)\n", 
                   s_angle[0], s_angle[1], s_angle[2]);
            printf("=============================================\n");
            printf("Commands: stop, print off, help\n");
            last_print_tick = now;
        }
        
        vTaskDelay(update_period);
    }
    
    ESP_LOGI(TAG, "IMU read task stopped");
    s_imu_read_task = NULL;
    vTaskDelete(NULL);
}

// ============ 命令处理 ============
static void cmd_init(void) {
    if (wit_imu_is_initialized()) {
        printf("IMU already initialized\n");
        return;
    }
    
    printf("Initializing IMU...\n");
    if (wit_imu_init() == 0) {
        // 设置默认参数
        vTaskDelay(pdMS_TO_TICKS(100));
        wit_imu_set_output_rate(IMU_DEFAULT_RATE);
        vTaskDelay(pdMS_TO_TICKS(100));
        wit_imu_set_bandwidth(IMU_DEFAULT_BANDWIDTH);
        printf("IMU initialized successfully\n");
        printf("Default: Rate=50Hz, Bandwidth=21Hz\n");
    } else {
        printf("IMU initialization failed!\n");
    }
}

static void cmd_deinit(void) {
    if (s_running) {
        printf("Please stop reading first\n");
        return;
    }
    wit_imu_deinit();
    printf("IMU deinitialized\n");
}

static void cmd_scan(void) {
    printf("Scanning I2C bus for IMU...\n");
    uint8_t addr = 0;
    if (wit_imu_scan(&addr) == 0) {
        printf("Found IMU at address 0x%02X\n", addr);
    } else {
        printf("No IMU found\n");
    }
}

static void cmd_start(void) {
    if (!wit_imu_is_initialized()) {
        printf("Please init first\n");
        return;
    }
    if (s_running) {
        printf("Already running\n");
        return;
    }
    
    s_running = true;
    s_update_count = 0;
    s_last_count = 0;
    
    xTaskCreate(imu_read_task, "imu_read", IMU_TEST_TASK_STACK, 
                NULL, IMU_TEST_TASK_PRIORITY, &s_imu_read_task);
    printf("IMU reading started (50Hz update)\n");
}

static void cmd_stop(void) {
    if (!s_running) {
        printf("Not running\n");
        return;
    }
    s_running = false;
    vTaskDelay(pdMS_TO_TICKS(100));
    printf("IMU reading stopped\n");
}

static void cmd_print(const char *arg) {
    if (arg == NULL || strcmp(arg, "on") == 0) {
        s_print_enabled = true;
        printf("Print enabled\n");
    } else if (strcmp(arg, "off") == 0) {
        s_print_enabled = false;
        printf("Print disabled\n");
    } else {
        printf("Usage: print [on|off]\n");
    }
}

static void cmd_interval(int ms) {
    if (ms < 20) ms = 20;
    if (ms > 5000) ms = 5000;
    s_print_interval_ms = ms;
    printf("Print interval set to %d ms\n", ms);
}

static void cmd_read(void) {
    if (!wit_imu_is_initialized()) {
        printf("Please init first\n");
        return;
    }
    
    if (wit_imu_update() == 0) {
        wit_imu_get_acc(&s_acc[0], &s_acc[1], &s_acc[2]);
        wit_imu_get_gyro(&s_gyro[0], &s_gyro[1], &s_gyro[2]);
        wit_imu_get_angle(&s_angle[0], &s_angle[1], &s_angle[2]);
        
        printf("Acc:   %.3f, %.3f, %.3f (g)\n", s_acc[0], s_acc[1], s_acc[2]);
        printf("Gyro:  %.2f, %.2f, %.2f (°/s)\n", s_gyro[0], s_gyro[1], s_gyro[2]);
        printf("Angle: Roll=%.2f, Pitch=%.2f, Yaw=%.2f (°)\n", 
               s_angle[0], s_angle[1], s_angle[2]);
    } else {
        printf("Read failed\n");
    }
}

static void cmd_quat(void) {
    if (!wit_imu_is_initialized()) {
        printf("Please init first\n");
        return;
    }
    
    float q[4];
    wit_imu_get_quaternion(&q[0], &q[1], &q[2], &q[3]);
    printf("Quaternion: w=%.4f, x=%.4f, y=%.4f, z=%.4f\n", q[0], q[1], q[2], q[3]);
}

static void cmd_temp(void) {
    if (!wit_imu_is_initialized()) {
        printf("Please init first\n");
        return;
    }
    
    float temp = wit_imu_get_temperature();
    printf("Temperature: %.2f °C\n", temp);
}

static void cmd_rate(int rate) {
    if (!wit_imu_is_initialized()) {
        printf("Please init first\n");
        return;
    }
    
    if (rate < 0 || rate > 12) {
        printf("Invalid rate (0-12)\n");
        return;
    }
    
    const char *rate_names[] = {
        "0.2Hz", "0.5Hz", "1Hz", "2Hz", "5Hz", "10Hz",
        "20Hz", "50Hz", "100Hz", "125Hz", "200Hz", "单次", "不输出"
    };
    
    if (wit_imu_set_output_rate(rate) == 0) {
        printf("Output rate set to %s\n", rate_names[rate]);
    } else {
        printf("Set rate failed\n");
    }
}

static void cmd_bandwidth(int bw) {
    if (!wit_imu_is_initialized()) {
        printf("Please init first\n");
        return;
    }
    
    if (bw < 0 || bw > 6) {
        printf("Invalid bandwidth (0-6)\n");
        return;
    }
    
    const char *bw_names[] = {
        "256Hz", "184Hz", "94Hz", "44Hz", "21Hz", "10Hz", "5Hz"
    };
    
    if (wit_imu_set_bandwidth(bw) == 0) {
        printf("Bandwidth set to %s\n", bw_names[bw]);
    } else {
        printf("Set bandwidth failed\n");
    }
}

static void cmd_cali(const char *type) {
    if (!wit_imu_is_initialized()) {
        printf("Please init first\n");
        return;
    }
    
    if (strcmp(type, "acc") == 0) {
        printf("Starting accelerometer calibration...\n");
        printf("Keep sensor horizontal and still!\n");
        if (wit_imu_acc_calibrate() == 0) {
            printf("Calibrating... wait 5 seconds then use 'cali stop'\n");
        } else {
            printf("Calibration start failed\n");
        }
    } else if (strcmp(type, "mag") == 0) {
        printf("Starting magnetometer calibration...\n");
        printf("Rotate sensor slowly in all directions!\n");
        if (wit_imu_mag_calibrate() == 0) {
            printf("Calibrating... rotate then use 'cali stop'\n");
        } else {
            printf("Calibration start failed\n");
        }
    } else if (strcmp(type, "stop") == 0) {
        printf("Stopping calibration and saving...\n");
        wit_imu_acc_calibrate_stop();
        wit_imu_mag_calibrate_stop();
        printf("Calibration stopped\n");
    } else {
        printf("Usage: cali [acc|mag|stop]\n");
    }
}

// ============ 命令解析 ============
void imu_test_process_cmd(const char *cmd) {
    char buf[IMU_CMD_BUF_SIZE];
    strncpy(buf, cmd, sizeof(buf) - 1);
    buf[sizeof(buf) - 1] = '\0';
    
    // 解析命令
    char *token = strtok(buf, " \t\r\n");
    if (token == NULL) return;
    
    if (strcmp(token, "help") == 0 || strcmp(token, "?") == 0) {
        print_help();
    } else if (strcmp(token, "init") == 0) {
        cmd_init();
    } else if (strcmp(token, "deinit") == 0) {
        cmd_deinit();
    } else if (strcmp(token, "scan") == 0) {
        cmd_scan();
    } else if (strcmp(token, "start") == 0) {
        cmd_start();
    } else if (strcmp(token, "stop") == 0) {
        cmd_stop();
    } else if (strcmp(token, "print") == 0) {
        token = strtok(NULL, " \t\r\n");
        cmd_print(token);
    } else if (strcmp(token, "interval") == 0) {
        token = strtok(NULL, " \t\r\n");
        if (token) {
            cmd_interval(atoi(token));
        } else {
            printf("Usage: interval <ms>\n");
        }
    } else if (strcmp(token, "read") == 0) {
        cmd_read();
    } else if (strcmp(token, "quat") == 0) {
        cmd_quat();
    } else if (strcmp(token, "temp") == 0) {
        cmd_temp();
    } else if (strcmp(token, "rate") == 0) {
        token = strtok(NULL, " \t\r\n");
        if (token) {
            cmd_rate(atoi(token));
        } else {
            printf("Usage: rate <0-12>\n");
        }
    } else if (strcmp(token, "bw") == 0) {
        token = strtok(NULL, " \t\r\n");
        if (token) {
            cmd_bandwidth(atoi(token));
        } else {
            printf("Usage: bw <0-6>\n");
        }
    } else if (strcmp(token, "cali") == 0) {
        token = strtok(NULL, " \t\r\n");
        if (token) {
            cmd_cali(token);
        } else {
            printf("Usage: cali [acc|mag|stop]\n");
        }
    } else if (strcmp(token, "status") == 0) {
        print_status();
    } else {
        printf("Unknown command: %s (type 'help')\n", token);
    }
}

// ============ 初始化 ============
void imu_test_init(void) {
    ESP_LOGI(TAG, "IMU test module initialized");
    printf("\n");
    printf("========================================\n");
    printf("       WIT IMU Test Module\n");
    printf("========================================\n");
    printf("Default parameters:\n");
    printf("  - Output Rate: 50Hz (RRATE_50HZ = 7)\n");
    printf("  - Bandwidth:   21Hz (BANDWIDTH_21HZ = 4)\n");
    printf("  - I2C Address: 0x50\n");
    printf("  - I2C SCL:     GPIO11\n");
    printf("  - I2C SDA:     GPIO12\n");
    printf("  - I2C Speed:   100kHz\n");
    printf("========================================\n");
    printf("Type 'help' for commands\n\n");
}
