/**
 * @file imu_test.h
 * @brief IMU 测试模块接口
 * @author Bubble
 * @date 2026-01-16
 */

#ifndef IMU_TEST_H
#define IMU_TEST_H

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 初始化 IMU 测试模块
 */
void imu_test_init(void);

/**
 * @brief 处理 IMU 测试命令
 * @param cmd 命令字符串
 */
void imu_test_process_cmd(const char *cmd);

#ifdef __cplusplus
}
#endif

#endif // IMU_TEST_H
