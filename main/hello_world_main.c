/**
 * @file main.c
 * @brief 轮腿机器人主程序
 * @author Bubble
 * @date 2026-01-15
 * @version 1.0
 * 
 * 编译选项：
 *   - 定义 MOTOR_TEST_MODE=1 启用电机测试模式
 *   - 不定义则进入正常机器人控制模式
 */

#include <stdio.h>
#include <inttypes.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_chip_info.h"
#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"

// 项目组件头文件
#include "config.h"
#include "types.h"
#include "robot_controller.h"
#include "button_driver.h"
#include "power_detect.h"
#include "motor_test.h"

// ============================================================================
// 编译选项: 设为1启用电机测试模式
// ============================================================================
#define MOTOR_TEST_MODE     1

static const char *TAG = "MAIN";

#if !MOTOR_TEST_MODE
// 按钮回调 (正常模式)
static void button_callback(button_id_t button, button_event_t event) {
    ESP_LOGI(TAG, "Button %d event: %d", button, event);
    
    if (button == BUTTON_1 && event == BUTTON_EVENT_PRESS) {
        // 按钮1: 站立
        robot_controller_stand_up();
    } else if (button == BUTTON_2 && event == BUTTON_EVENT_PRESS) {
        // 按钮2: 坐下
        robot_controller_sit_down();
    } else if (event == BUTTON_EVENT_LONG_PRESS) {
        // 长按任意按钮: 紧急停止
        robot_controller_emergency_stop();
    }
}
#endif

void app_main(void)
{
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "  Wheel-Leg Robot Starting...");
    ESP_LOGI(TAG, "=================================");

    // 打印芯片信息
    esp_chip_info_t chip_info;
    uint32_t flash_size;
    esp_chip_info(&chip_info);
    ESP_LOGI(TAG, "%s chip, %d core(s), revision v%d.%d",
             CONFIG_IDF_TARGET,
             chip_info.cores,
             chip_info.revision / 100,
             chip_info.revision % 100);
    
    if (esp_flash_get_size(NULL, &flash_size) == ESP_OK) {
        ESP_LOGI(TAG, "Flash: %" PRIu32 "MB", flash_size / (1024 * 1024));
    }
    
    ESP_LOGI(TAG, "Free heap: %" PRIu32 " bytes", esp_get_minimum_free_heap_size());

#if MOTOR_TEST_MODE
    // ============================================
    // 电机测试模式
    // ============================================
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "  MOTOR TEST MODE");
    ESP_LOGI(TAG, "  Type 'help' for commands");
    ESP_LOGI(TAG, "=================================");
    
    motor_test_start();  // 阻塞在这里，处理命令行
    
#else
    // ============================================
    // 正常机器人控制模式
    // ============================================
    
    // 初始化电源检测
    esp_err_t ret = power_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Power init failed");
    } else {
        ESP_LOGI(TAG, "Power available: %s", power_is_available() ? "YES" : "NO");
    }
    
    // 初始化按钮
    ret = button_init();
    if (ret == ESP_OK) {
        button_register_callback(button_callback);
    }
    
    // 初始化机器人控制器
    ret = robot_controller_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Robot controller init failed!");
        return;
    }
    
    // 启动控制器
    ret = robot_controller_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Robot controller start failed!");
        return;
    }
    
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "  Robot Ready!");
    ESP_LOGI(TAG, "  Button 1: Stand up");
    ESP_LOGI(TAG, "  Button 2: Sit down");
    ESP_LOGI(TAG, "  Long press: Emergency stop");
    ESP_LOGI(TAG, "=================================");
    
    // 主循环 - 按钮轮询和状态监控
    while (1) {
        // 轮询按钮
        button_poll();
        
        // 打印状态 (每5秒)
        static uint32_t last_print = 0;
        uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
        if (now - last_print >= 5000) {
            robot_data_t data;
            robot_controller_get_data(&data);
            ESP_LOGI(TAG, "State: %d, Pitch: %.2f, Free heap: %" PRIu32,
                     data.state, data.imu.pitch, esp_get_free_heap_size());
            last_print = now;
        }
        
        vTaskDelay(pdMS_TO_TICKS(20));
    }
#endif
}
