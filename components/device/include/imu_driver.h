/**
 * @file imu_driver.h
 * @brief IMU 驱动 (I2C)
 * @author Bubble
 * @date 2026-01-15
 */

#ifndef IMU_DRIVER_H
#define IMU_DRIVER_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 IMU (I2C)
 * @return ESP_OK 成功
 */
esp_err_t imu_init(void);

/**
 * @brief 反初始化 IMU
 */
void imu_deinit(void);

/**
 * @brief 读取 IMU 数据
 * @param data 输出数据结构
 * @return ESP_OK 成功
 */
esp_err_t imu_read_data(imu_data_t *data);

/**
 * @brief 获取滚转角
 * @return 滚转角 (度)
 */
float imu_get_roll(void);

/**
 * @brief 获取俯仰角
 * @return 俯仰角 (度)
 */
float imu_get_pitch(void);

/**
 * @brief 获取偏航角
 * @return 偏航角 (度)
 */
float imu_get_yaw(void);

/**
 * @brief 检查 IMU 是否在线
 * @return true 在线
 */
bool imu_is_online(void);

/**
 * @brief 校准 IMU (需要静置)
 * @return ESP_OK 成功
 */
esp_err_t imu_calibrate(void);

/**
 * @brief 获取四元数
 * @param pq0 四元数w分量
 * @param pq1 四元数x分量
 * @param pq2 四元数y分量
 * @param pq3 四元数z分量
 * @return ESP_OK 成功
 */
esp_err_t imu_get_quaternion(float *pq0, float *pq1, float *pq2, float *pq3);

/**
 * @brief 获取温度
 * @return 温度值 (°C)
 */
float imu_get_temperature(void);

/**
 * @brief 开始磁力计校准
 * @return ESP_OK 成功
 */
esp_err_t imu_mag_calibrate(void);

/**
 * @brief 停止磁力计校准
 * @return ESP_OK 成功
 */
esp_err_t imu_mag_calibrate_stop(void);

/**
 * @brief 设置输出速率
 * @param rate 速率索引 (参见 wit_c_sdk.h RRATE_* 定义)
 * @return ESP_OK 成功
 */
esp_err_t imu_set_output_rate(int32_t rate);

/**
 * @brief 打印 IMU 数据
 */
void imu_print_data(void);

#ifdef __cplusplus
}
#endif

#endif // IMU_DRIVER_H
