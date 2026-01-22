/**
 * @file sht30_driver.h
 * @brief SHT30 温湿度传感器驱动
 * @author Bubble
 * @date 2026-01-16
 * 
 * 功能：
 * - I2C 通信读取温湿度
 * - 支持单次测量和周期测量模式
 * - CRC 校验
 */

#ifndef SHT30_DRIVER_H
#define SHT30_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief SHT30 测量精度
 */
typedef enum {
    SHT30_REPEATABILITY_HIGH,       // 高精度 (15ms)
    SHT30_REPEATABILITY_MEDIUM,     // 中精度 (6ms)
    SHT30_REPEATABILITY_LOW,        // 低精度 (4ms)
} sht30_repeatability_t;

/**
 * @brief SHT30 数据结构
 */
typedef struct {
    float temperature;      // 温度 (°C)
    float humidity;         // 相对湿度 (%)
    bool valid;             // 数据是否有效
    uint32_t timestamp_ms;  // 采集时间戳 (ms)
} sht30_data_t;

/**
 * @brief 初始化 SHT30
 * @param sda_pin SDA 引脚
 * @param scl_pin SCL 引脚
 * @param i2c_addr I2C 地址 (0x44 或 0x45)
 * @return ESP_OK 成功
 */
esp_err_t sht30_init(int sda_pin, int scl_pin, uint8_t i2c_addr);

/**
 * @brief 反初始化 SHT30
 */
void sht30_deinit(void);

/**
 * @brief 软复位 SHT30
 * @return ESP_OK 成功
 */
esp_err_t sht30_soft_reset(void);

/**
 * @brief 单次测量温湿度
 * @param data 输出数据
 * @param repeatability 测量精度
 * @return ESP_OK 成功
 */
esp_err_t sht30_read_single(sht30_data_t *data, sht30_repeatability_t repeatability);

/**
 * @brief 读取温湿度 (使用高精度)
 * @param temperature 输出温度 (°C)
 * @param humidity 输出湿度 (%)
 * @return ESP_OK 成功
 */
esp_err_t sht30_read(float *temperature, float *humidity);

/**
 * @brief 启动周期测量模式
 * @param mps 每秒测量次数 (0.5, 1, 2, 4, 10)
 * @param repeatability 测量精度
 * @return ESP_OK 成功
 */
esp_err_t sht30_start_periodic(float mps, sht30_repeatability_t repeatability);

/**
 * @brief 停止周期测量模式
 * @return ESP_OK 成功
 */
esp_err_t sht30_stop_periodic(void);

/**
 * @brief 从周期模式读取数据
 * @param data 输出数据
 * @return ESP_OK 成功
 */
esp_err_t sht30_fetch_data(sht30_data_t *data);

/**
 * @brief 读取状态寄存器
 * @param status 输出状态值
 * @return ESP_OK 成功
 */
esp_err_t sht30_read_status(uint16_t *status);

/**
 * @brief 清除状态寄存器
 * @return ESP_OK 成功
 */
esp_err_t sht30_clear_status(void);

/**
 * @brief 开启加热器 (用于测试/除湿)
 * @param enable true 开启, false 关闭
 * @return ESP_OK 成功
 */
esp_err_t sht30_heater(bool enable);

/**
 * @brief 检查 SHT30 是否在线
 * @return true 在线
 */
bool sht30_is_online(void);

/**
 * @brief 扫描 I2C 总线上的设备
 * @return ESP_OK 成功
 */
esp_err_t sht30_scan_i2c(void);

/**
 * @brief 获取最后一次读取的数据
 * @param data 输出数据
 */
void sht30_get_last_data(sht30_data_t *data);

#ifdef __cplusplus
}
#endif

#endif // SHT30_DRIVER_H
