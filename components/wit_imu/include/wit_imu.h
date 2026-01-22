/**
 * @file wit_imu.h
 * @brief WIT Motion IMU ESP-IDF驱动接口
 * @note 移植自 https://github.com/Bubble252/WIT_imu_idf
 * @author Bubble
 */

#ifndef __WIT_IMU_H__
#define __WIT_IMU_H__

#include <stdint.h>
#include <stdbool.h>
#include "wit_c_sdk.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化WIT IMU传感器
 * @return 0成功，-1失败
 */
int32_t wit_imu_init(void);

/**
 * @brief 反初始化WIT IMU传感器
 */
void wit_imu_deinit(void);

/**
 * @brief 扫描I2C总线上的WIT IMU传感器
 * @param found_addr 找到的传感器地址
 * @return 0成功，-1失败
 */
int32_t wit_imu_scan(uint8_t *found_addr);

/**
 * @brief 更新传感器数据（从传感器读取最新数据）
 * @return 0成功，-1失败
 */
int32_t wit_imu_update(void);

/**
 * @brief 获取加速度数据 (单位: g)
 * @param acc_x X轴加速度
 * @param acc_y Y轴加速度
 * @param acc_z Z轴加速度
 */
void wit_imu_get_acc(float *acc_x, float *acc_y, float *acc_z);

/**
 * @brief 获取陀螺仪数据 (单位: °/s)
 * @param gyro_x X轴角速度
 * @param gyro_y Y轴角速度
 * @param gyro_z Z轴角速度
 */
void wit_imu_get_gyro(float *gyro_x, float *gyro_y, float *gyro_z);

/**
 * @brief 获取姿态角数据 (单位: °)
 * @param roll 横滚角
 * @param pitch 俯仰角
 * @param yaw 偏航角
 */
void wit_imu_get_angle(float *roll, float *pitch, float *yaw);

/**
 * @brief 获取温度 (单位: °C)
 * @return 温度值
 */
float wit_imu_get_temperature(void);

/**
 * @brief 获取四元数
 * @param pq0 四元数w分量
 * @param pq1 四元数x分量
 * @param pq2 四元数y分量
 * @param pq3 四元数z分量
 */
void wit_imu_get_quaternion(float *pq0, float *pq1, float *pq2, float *pq3);

/**
 * @brief 开始加速度计校准
 * @note 校准时需保持传感器水平静止
 * @return 0成功，-1失败
 */
int32_t wit_imu_acc_calibrate(void);

/**
 * @brief 停止加速度计校准并保存
 * @return 0成功，-1失败
 */
int32_t wit_imu_acc_calibrate_stop(void);

/**
 * @brief 开始磁力计校准
 * @note 校准时需缓慢旋转传感器
 * @return 0成功，-1失败
 */
int32_t wit_imu_mag_calibrate(void);

/**
 * @brief 停止磁力计校准并保存
 * @return 0成功，-1失败
 */
int32_t wit_imu_mag_calibrate_stop(void);

/**
 * @brief 设置输出速率
 * @param rate 输出速率索引 (见 wit_c_sdk.h 中的 RRATE_* 定义)
 *        - RRATE_02HZ: 0.2Hz
 *        - RRATE_05HZ: 0.5Hz
 *        - RRATE_1HZ: 1Hz
 *        - RRATE_2HZ: 2Hz
 *        - RRATE_5HZ: 5Hz
 *        - RRATE_10HZ: 10Hz (默认)
 *        - RRATE_20HZ: 20Hz
 *        - RRATE_50HZ: 50Hz
 *        - RRATE_100HZ: 100Hz
 *        - RRATE_125HZ: 125Hz
 *        - RRATE_200HZ: 200Hz
 *        - RRATE_SINGLE: 单次
 *        - RRATE_NOOUTPUT: 不输出
 * @return 0成功，-1失败
 */
int32_t wit_imu_set_output_rate(int32_t rate);

/**
 * @brief 设置低通滤波带宽
 * @param bandwidth 带宽索引 (见 wit_c_sdk.h 中的 BANDWIDTH_* 定义)
 *        - BANDWIDTH_256HZ: 256Hz
 *        - BANDWIDTH_184HZ: 184Hz
 *        - BANDWIDTH_94HZ: 94Hz
 *        - BANDWIDTH_44HZ: 44Hz
 *        - BANDWIDTH_21HZ: 21Hz
 *        - BANDWIDTH_10HZ: 10Hz
 *        - BANDWIDTH_5HZ: 5Hz
 * @return 0成功，-1失败
 */
int32_t wit_imu_set_bandwidth(int32_t bandwidth);

/**
 * @brief 打印传感器数据到控制台
 */
void wit_imu_print_data(void);

/**
 * @brief 获取原始寄存器值
 * @param reg 寄存器地址
 * @return 寄存器值
 */
int16_t wit_imu_get_raw_reg(uint8_t reg);

/**
 * @brief 检查IMU是否已初始化
 * @return true已初始化，false未初始化
 */
bool wit_imu_is_initialized(void);

#ifdef __cplusplus
}
#endif

#endif /* __WIT_IMU_H__ */
