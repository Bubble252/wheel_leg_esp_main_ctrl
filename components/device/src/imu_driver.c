/**
 * @file imu_driver.c
 * @brief IMU 驱动实现 (使用 WIT Motion SDK)
 * @author Bubble
 * @date 2026-01-15
 * @note 基于 WIT Motion IMU SDK 移植
 */

#include "imu_driver.h"
#include "config.h"
#include "wit_imu.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "IMU";

// IMU 数据缓存
static imu_data_t g_imu_data = {0};
static bool g_imu_initialized = false;

esp_err_t imu_init(void) {
    if (g_imu_initialized) {
        return ESP_OK;
    }
    
    // 使用 WIT IMU SDK 初始化
    if (wit_imu_init() != 0) {
        ESP_LOGE(TAG, "WIT IMU initialization failed");
        return ESP_FAIL;
    }
    
    g_imu_initialized = true;
    ESP_LOGI(TAG, "IMU initialized using WIT Motion SDK");
    
    return ESP_OK;
}

void imu_deinit(void) {
    if (!g_imu_initialized) return;
    
    wit_imu_deinit();
    g_imu_initialized = false;
    ESP_LOGI(TAG, "IMU deinitialized");
}

esp_err_t imu_read_data(imu_data_t *data) {
    if (data == NULL) return ESP_ERR_INVALID_ARG;
    if (!g_imu_initialized) return ESP_ERR_INVALID_STATE;
    
    // 从 WIT IMU 读取数据
    if (wit_imu_update() != 0) {
        g_imu_data.is_valid = false;
        return ESP_FAIL;
    }
    
    // 获取加速度数据
    wit_imu_get_acc(&g_imu_data.accel_x, &g_imu_data.accel_y, &g_imu_data.accel_z);
    
    // 获取陀螺仪数据
    wit_imu_get_gyro(&g_imu_data.gyro_x, &g_imu_data.gyro_y, &g_imu_data.gyro_z);
    
    // 获取姿态角数据
    wit_imu_get_angle(&g_imu_data.roll, &g_imu_data.pitch, &g_imu_data.yaw);
    
    g_imu_data.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
    g_imu_data.is_valid = true;
    
    memcpy(data, &g_imu_data, sizeof(imu_data_t));
    return ESP_OK;
}

float imu_get_roll(void) {
    return g_imu_data.roll;
}

float imu_get_pitch(void) {
    return g_imu_data.pitch;
}

float imu_get_yaw(void) {
    return g_imu_data.yaw;
}

bool imu_is_online(void) {
    return g_imu_initialized && g_imu_data.is_valid;
}

esp_err_t imu_calibrate(void) {
    if (!g_imu_initialized) return ESP_ERR_INVALID_STATE;
    
    ESP_LOGI(TAG, "Starting IMU calibration...");
    
    // 使用 WIT SDK 进行加速度计校准
    if (wit_imu_acc_calibrate() != 0) {
        ESP_LOGE(TAG, "Failed to start accelerometer calibration");
        return ESP_FAIL;
    }
    
    // 等待校准完成 (约5秒)
    vTaskDelay(pdMS_TO_TICKS(5000));
    
    if (wit_imu_acc_calibrate_stop() != 0) {
        ESP_LOGE(TAG, "Failed to stop accelerometer calibration");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "IMU calibration complete");
    return ESP_OK;
}

// ============ 扩展接口 ============

esp_err_t imu_get_quaternion(float *pq0, float *pq1, float *pq2, float *pq3) {
    if (!g_imu_initialized) return ESP_ERR_INVALID_STATE;
    
    wit_imu_get_quaternion(pq0, pq1, pq2, pq3);
    return ESP_OK;
}

float imu_get_temperature(void) {
    if (!g_imu_initialized) return 0.0f;
    
    return wit_imu_get_temperature();
}

esp_err_t imu_mag_calibrate(void) {
    if (!g_imu_initialized) return ESP_ERR_INVALID_STATE;
    
    ESP_LOGI(TAG, "Starting magnetometer calibration...");
    ESP_LOGI(TAG, "Please rotate sensor slowly in all directions");
    
    if (wit_imu_mag_calibrate() != 0) {
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

esp_err_t imu_mag_calibrate_stop(void) {
    if (!g_imu_initialized) return ESP_ERR_INVALID_STATE;
    
    if (wit_imu_mag_calibrate_stop() != 0) {
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Magnetometer calibration complete");
    return ESP_OK;
}

esp_err_t imu_set_output_rate(int32_t rate) {
    if (!g_imu_initialized) return ESP_ERR_INVALID_STATE;
    
    return (wit_imu_set_output_rate(rate) == 0) ? ESP_OK : ESP_FAIL;
}

void imu_print_data(void) {
    wit_imu_print_data();
}
