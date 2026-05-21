#pragma once

/**
 * @brief CLI 命令处理模块
 *
 * 包含: balance_test_process_cmd() - 解析和执行所有 balance CLI 命令
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 处理 balance CLI 命令
 * @param cmd_str 命令字符串
 */
void balance_test_process_cmd(const char *cmd_str);

#ifdef __cplusplus
}
#endif
