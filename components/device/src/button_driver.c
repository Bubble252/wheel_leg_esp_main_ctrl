/**
 * @file button_driver.c
 * @brief 按钮驱动实现
 * @author Bubble
 * @date 2026-01-15
 */

#include "button_driver.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BUTTON";

// 按钮 GPIO 映射
static const gpio_num_t button_pins[BUTTON_COUNT] = {
    BUTTON1_PIN,
    BUTTON2_PIN
};

// 按钮状态
static bool g_button_state[BUTTON_COUNT] = {false, false};
static uint32_t g_button_press_time[BUTTON_COUNT] = {0, 0};
static button_callback_t g_callback = NULL;
static bool g_initialized = false;

// 长按阈值 (ms)
#define LONG_PRESS_THRESHOLD_MS 1000

esp_err_t button_init(void) {
    if (g_initialized) {
        return ESP_OK;
    }
    
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    
    for (int i = 0; i < BUTTON_COUNT; i++) {
        io_conf.pin_bit_mask = (1ULL << button_pins[i]);
        esp_err_t ret = gpio_config(&io_conf);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to config button %d: %s", i, esp_err_to_name(ret));
            return ret;
        }
    }
    
    g_initialized = true;
    ESP_LOGI(TAG, "Buttons initialized (GPIO%d, GPIO%d)", BUTTON1_PIN, BUTTON2_PIN);
    
    return ESP_OK;
}

void button_deinit(void) {
    g_initialized = false;
}

void button_register_callback(button_callback_t callback) {
    g_callback = callback;
}

bool button_is_pressed(button_id_t button) {
    if (button >= BUTTON_COUNT) return false;
    
    // 按钮低电平有效 (上拉 + 按下接地)
    return gpio_get_level(button_pins[button]) == 0;
}

void button_poll(void) {
    if (!g_initialized) return;
    
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    for (int i = 0; i < BUTTON_COUNT; i++) {
        bool pressed = button_is_pressed((button_id_t)i);
        
        if (pressed && !g_button_state[i]) {
            // 按下事件
            g_button_state[i] = true;
            g_button_press_time[i] = now;
            
            if (g_callback) {
                g_callback((button_id_t)i, BUTTON_EVENT_PRESS);
            }
        } else if (!pressed && g_button_state[i]) {
            // 释放事件
            g_button_state[i] = false;
            
            uint32_t press_duration = now - g_button_press_time[i];
            
            if (press_duration >= LONG_PRESS_THRESHOLD_MS) {
                if (g_callback) {
                    g_callback((button_id_t)i, BUTTON_EVENT_LONG_PRESS);
                }
            } else {
                if (g_callback) {
                    g_callback((button_id_t)i, BUTTON_EVENT_RELEASE);
                }
            }
        }
    }
}
