/**
 * @file wifi_remote.h
 * @brief WiFi 遥控器模块
 * @author Bubble
 * @date 2026-01-16
 * @note 移植自 shibo_wheel_leg 项目
 */

#ifndef WIFI_REMOTE_H
#define WIFI_REMOTE_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"
#include "types.h"  // 使用公共类型定义中的 direction_t

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 遥控器数据结构
 */
typedef struct {
    // 方向命令 (使用公共 direction_t)
    direction_t dir;            // 当前方向
    direction_t dir_last;       // 上次方向
    
    // 摇杆数据 (-100 ~ 100)
    int16_t joy_x;              // 摇杆 X (左右转向)
    int16_t joy_y;              // 摇杆 Y (前后速度)
    int16_t joy_x_last;         // 上次摇杆 X
    int16_t joy_y_last;         // 上次摇杆 Y
    
    // 滑块数据
    int16_t height;             // 腿部高度 (32-85 mm)
    int16_t roll;               // Roll 目标角度 (-30~30°)
    int16_t linear;             // 线速度 (-200~200 mm/s)
    int16_t angular;            // 角速度 (-100~100 °/s)
    
    // 控制开关
    bool go;                    // 使能开关 (Robot Go!)
    
    // 状态
    uint32_t last_update_ms;    // 最后更新时间 (毫秒)
    uint64_t receive_time_us;   // 精确接收时间 (微秒) - 用于延迟测量
    uint32_t msg_count;         // 接收消息计数
    bool connected;             // WebSocket 连接状态
} remote_data_t;

/**
 * @brief 获取遥控器数据指针
 * @return 遥控器数据指针
 */
remote_data_t* wifi_remote_get_data(void);

/**
 * @brief 初始化 WiFi 遥控模块
 * @return ESP_OK 成功
 */
esp_err_t wifi_remote_init(void);

/**
 * @brief 启动 WiFi AP 和 Web 服务器
 * @return ESP_OK 成功
 */
esp_err_t wifi_remote_start(void);

/**
 * @brief 停止 WiFi 遥控模块
 */
void wifi_remote_stop(void);

/**
 * @brief 检查连接超时
 * @param timeout_ms 超时时间 (毫秒)
 * @return true 超时, false 正常
 */
bool wifi_remote_check_timeout(uint32_t timeout_ms);

/**
 * @brief 打印遥控器状态
 */
void wifi_remote_print_status(void);

/**
 * @brief 获取方向名称字符串
 * @param dir 方向枚举
 * @return 方向名称
 */
const char* wifi_remote_dir_to_str(direction_t dir);

#ifdef __cplusplus
}
#endif

#endif // WIFI_REMOTE_H
