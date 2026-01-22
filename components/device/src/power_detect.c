/**
 * @file power_detect.c
 * @brief 电源检测驱动实现
 * @author Bubble
 * @date 2026-01-15
 */

#include "power_detect.h"
#include "config.h"
#include "driver/gpio.h"
#include "esp_log.h"

static const char *TAG = "POWER";

static bool g_initialized = false;
static bool g_motor_enabled = false;

esp_err_t power_init(void) {
    if (g_initialized) {
        return ESP_OK;
    }
    
    // 配置电源检测引脚 (输入)
    gpio_config_t input_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL << POWER_DETECT_PIN),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    
    esp_err_t ret = gpio_config(&input_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to config power detect pin: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 配置电机使能引脚 (输出)
    gpio_config_t output_conf = {
        .intr_type = GPIO_INTR_DISABLE,
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = (1ULL << MOTOR_POWER_EN_PIN),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    
    ret = gpio_config(&output_conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to config motor enable pin: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 默认开启电机电源
    gpio_set_level(MOTOR_POWER_EN_PIN, 1);
    g_motor_enabled = true;
    
    g_initialized = true;
    ESP_LOGI(TAG, "Power detect initialized (DETECT: GPIO%d, EN: GPIO%d)", 
             POWER_DETECT_PIN, MOTOR_POWER_EN_PIN);
    
    return ESP_OK;
}

void power_deinit(void) {
    if (!g_initialized) return;
    
    // 关闭电机电源
    gpio_set_level(MOTOR_POWER_EN_PIN, 0);
    g_motor_enabled = false;
    g_initialized = false;
}

bool power_is_available(void) {
    if (!g_initialized) return false;
    
    // 高电平表示有电
    return gpio_get_level(POWER_DETECT_PIN) == 1;
}

bool power_is_battery(void) {
    // 根据 PROJECT_NOTES.md: 高电平=电池供电，低电平=USB供电
    // 即使未初始化也可以直接读取 GPIO
    return gpio_get_level(POWER_DETECT_PIN) == 1;
}

esp_err_t power_motor_enable(bool enable) {
    if (!g_initialized) return ESP_ERR_INVALID_STATE;
    
    // 直接控制电机电源，不做安全检查（后续整理逻辑时再添加）
    gpio_set_level(MOTOR_POWER_EN_PIN, enable ? 1 : 0);
    g_motor_enabled = enable;
    
    ESP_LOGI(TAG, "Motor power %s", enable ? "ENABLED" : "DISABLED");
    
    return ESP_OK;
}

bool power_motor_is_enabled(void) {
    return g_motor_enabled;
}
