/**
 * @file can_motor.h
 * @brief CAN 电机驱动接口
 * @author Bubble
 * @date 2026-01-15
 * @note 移植自 Arduino 版本 CANServo
 */

#ifndef __CAN_MOTOR_H__
#define __CAN_MOTOR_H__

#include "esp_err.h"
#include "driver/gpio.h"
#include "types.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 类型定义
// ============================================================================

typedef struct can_motor* can_motor_handle_t;

// ============================================================================
// CAN 总线初始化
// ============================================================================

/**
 * @brief 初始化 CAN (TWAI) 总线
 * @param tx_pin 发送引脚
 * @param rx_pin 接收引脚
 * @return ESP_OK 成功, 其他失败
 */
esp_err_t can_bus_init(gpio_num_t tx_pin, gpio_num_t rx_pin);

/**
 * @brief 关闭 CAN 总线
 */
esp_err_t can_bus_deinit(void);

/**
 * @brief 检查 CAN 总线状态并在 Bus-Off 时自动恢复
 * @return ESP_OK: 总线正常或已恢复, ESP_ERR_INVALID_STATE/ESP_ERR_TIMEOUT: 恢复失败
 * @note 非阻塞调用 (除 Bus-Off 恢复外)。Bus-Off 恢复最多等待 500ms。
 */
esp_err_t can_bus_check_and_recover(void);

/**
 * @brief 获取 CAN 总线错误统计
 * @param busoff_count  Bus-Off 次数 (可为 NULL)
 * @param tx_error_count TX 失败次数 (可为 NULL)
 * @param recovery_count 恢复成功次数 (可为 NULL)
 */
void can_bus_get_error_stats(uint32_t *busoff_count, uint32_t *tx_error_count, uint32_t *recovery_count);

// ============================================================================
// 电机实例管理
// ============================================================================

/**
 * @brief 创建电机实例
 * @param motor_id 电机 ID (1-6)
 * @return 电机句柄, NULL 表示失败
 */
can_motor_handle_t can_motor_create(uint8_t motor_id);

/**
 * @brief 销毁电机实例
 * @param motor 电机句柄
 */
void can_motor_destroy(can_motor_handle_t motor);

/**
 * @brief 获取电机 ID
 */
uint8_t can_motor_get_id(can_motor_handle_t motor);

// ============================================================================
// 状态读取
// ============================================================================

/**
 * @brief 请求读取电机状态 (非阻塞)
 * @param motor 电机句柄
 * @return ESP_OK 成功
 */
esp_err_t can_motor_request_status(can_motor_handle_t motor);

/**
 * @brief 处理接收到的 CAN 帧，更新电机状态
 * @note 应在接收任务中调用
 */
esp_err_t can_motor_process_rx(void);

/**
 * @brief 获取电机状态 (从缓存)
 * @param motor 电机句柄
 * @param state 输出状态结构体
 */
esp_err_t can_motor_get_state(can_motor_handle_t motor, motor_state_t *state);

/**
 * @brief 读取实时位置
 * @return 位置 (°)
 */
float can_motor_read_position(can_motor_handle_t motor);

/**
 * @brief 读取实时速度
 * @return 速度 (rpm)
 */
float can_motor_read_speed(can_motor_handle_t motor);

/**
 * @brief 读取实时电流
 * @return 电流 (A)
 */
float can_motor_read_current(can_motor_handle_t motor);

/**
 * @brief 请求读取角度/位置寄存器 (非阻塞, 轻量)
 * @note 只发送一次 CAN 读请求, 结果通过 process_rx 更新到 state.position
 *       STW: 发送 0xA3 多圈角度读取; JUCI: 发送位置寄存器读取
 * @param motor 电机句柄
 * @return ESP_OK 成功
 */
esp_err_t can_motor_request_angle(can_motor_handle_t motor);
esp_err_t can_motor_request_current(can_motor_handle_t motor);

/**
 * @brief 请求读取电源电压寄存器 (非阻塞, 轻量)
 * @note 只发送一次 CAN 读请求, 结果通过 process_rx 更新到 state.voltage
 * @param motor 电机句柄
 * @return ESP_OK 成功
 */
esp_err_t can_motor_request_voltage(can_motor_handle_t motor);

/**
 * @brief 读取电源电压
 * @param motor 电机句柄
 * @return 电压 (V)
 */
float can_motor_read_voltage(can_motor_handle_t motor);

/**
 * @brief 读取错误码
 * @param motor 电机句柄
 * @return 32位错误码，0表示无错误
 */
uint32_t can_motor_read_error(can_motor_handle_t motor);

/**
 * @brief 打印错误码含义
 * @param error_code 错误码
 */
void can_motor_print_error(uint32_t error_code);

/**
 * @brief 读取电机温度
 * @return 温度 (°C)
 */
float can_motor_read_motor_temp(can_motor_handle_t motor);

/**
 * @brief 读取驱动器温度
 * @return 温度 (°C)
 */
float can_motor_read_driver_temp(can_motor_handle_t motor);

/**
 * @brief 检查电机是否在线
 * @param motor 电机句柄
 * @param timeout_ms 超时时间 (ms)
 * @return true 在线, false 离线
 */
bool can_motor_is_online(can_motor_handle_t motor, uint32_t timeout_ms);

// ============================================================================
// 控制命令
// ============================================================================

/**
 * @brief 设置控制模式
 * @param motor 电机句柄
 * @param mode 控制模式
 */
esp_err_t can_motor_set_mode(can_motor_handle_t motor, motor_mode_t mode);

/**
 * @brief 设置速度 (速度模式)
 * @param motor 电机句柄
 * @param speed_rpm 目标速度 (rpm)
 */
esp_err_t can_motor_set_speed(can_motor_handle_t motor, float speed_rpm);

/**
 * @brief 设置位置 (位置模式)
 * @param motor 电机句柄
 * @param angle_deg 目标角度 (°)
 * @param speed_rpm 运动速度限制 (rpm)
 */
esp_err_t can_motor_set_position(can_motor_handle_t motor, float angle_deg, float speed_rpm);

/**
 * @brief 设置力矩 (力矩模式)
 * @param motor 电机句柄
 * @param torque 力矩值 (单位由驱动器定义)
 */
esp_err_t can_motor_set_torque(can_motor_handle_t motor, float torque);

/**
 * @brief PV 指令 (位置+速度)
 * @param motor 电机句柄
 * @param position_deg 目标位置 (°)
 * @param speed_rpm 目标速度 (rpm)
 */
esp_err_t can_motor_pv_cmd(can_motor_handle_t motor, float position_deg, float speed_rpm);

/**
 * @brief PVT 指令 (位置+速度+力矩限制)
 * @param motor 电机句柄
 * @param position_deg 目标位置 (°)
 * @param speed_rpm 目标速度 (rpm)
 * @param torque_percent 力矩限制百分比 (0-100)
 */
esp_err_t can_motor_pvt_cmd(can_motor_handle_t motor, float position_deg, float speed_rpm, uint8_t torque_percent);

/**
 * @brief 设置相对位置
 * @param motor 电机句柄
 * @param angle_deg 相对角度 (°)
 * @return ESP_OK 成功
 */
esp_err_t can_motor_set_rel_position(can_motor_handle_t motor, float angle_deg);

/**
 * @brief 设置低速模式速度 (低速大扭矩模式)
 * @param motor 电机句柄
 * @param speed_rpm 速度 (rpm)，范围 ±300rpm
 * @return ESP_OK 成功
 */
esp_err_t can_motor_set_low_speed(can_motor_handle_t motor, float speed_rpm);

/**
 * @brief 进入闭环控制
 */
esp_err_t can_motor_enter_closed_loop(can_motor_handle_t motor);

/**
 * @brief 进入空闲状态 (电机失力)
 */
esp_err_t can_motor_set_idle(can_motor_handle_t motor);

/**
 * @brief 设置当前位置为原点 (零点)
 * @param motor 电机句柄
 * @return ESP_OK 成功
 * @note 调用后电机当前位置变为 0°
 */
esp_err_t can_motor_set_origin(can_motor_handle_t motor);

/**
 * @brief 重启电机驱动器
 * @param motor 电机句柄
 * @return ESP_OK 成功
 */
esp_err_t can_motor_reboot(can_motor_handle_t motor);

/**
 * @brief 保存参数到Flash
 * @param motor 电机句柄
 * @return ESP_OK 成功
 */
esp_err_t can_motor_save(can_motor_handle_t motor);

/**
 * @brief 校准电机
 * @param motor 电机句柄
 * @return ESP_OK 成功
 * @warning 校准过程中电机会旋转，确保安全！
 */
esp_err_t can_motor_calibrate(can_motor_handle_t motor);

/**
 * @brief 擦除电机参数
 * @param motor 电机句柄
 * @return ESP_OK 成功
 * @warning 擦除后需要重新校准！
 */
esp_err_t can_motor_erase(can_motor_handle_t motor);

// ============================================================================
// 批量操作 (用于同步控制)
// ============================================================================

/**
 * @brief 设置所有电机速度
 * @param speeds 速度数组 [6] (rpm)
 */
esp_err_t can_motor_set_all_speeds(float speeds[6]);

/**
 * @brief 所有电机进入闭环
 */
esp_err_t can_motor_all_enter_closed_loop(void);

/**
 * @brief 所有电机进入空闲
 */
esp_err_t can_motor_all_set_idle(void);

// ============================================================================
// STW 专用命令 (仅 MOTOR_BRAND_STW 时有效, JUCI 返回 NOT_SUPPORTED)
// ============================================================================

/**
 * @brief STW 电机信息 (0xB0 应答)
 */
typedef struct {
    uint8_t pole_pairs;     // 极对数
    float   kt;             // 力矩常数 Kt (Nm/A)
    uint8_t gear_ratio;     // 减速比
} stw_motor_info_t;

/**
 * @brief STW MIT 实时状态 (0xF1 应答)
 */
typedef struct {
    float   position;       // 位置 (rad)
    float   velocity;       // 速度 (rad/s)
    float   torque;         // 力矩 (Nm)
    uint8_t status;         // 状态: Bit0=MIT模式, Bit1=故障
} stw_mit_state_t;

/**
 * @brief 请求读取电机信息 (极对数/Kt/减速比) — 0xB0
 * @note 结果通过 RX 解析后存入内部, 用 can_motor_stw_get_motor_info() 读取
 */
esp_err_t can_motor_stw_request_motor_info(can_motor_handle_t motor);

/**
 * @brief 获取上次 0xB0 应答的电机信息
 */
esp_err_t can_motor_stw_get_motor_info(can_motor_handle_t motor, stw_motor_info_t *info);

/**
 * @brief 设置位置模式最大速度 — 0xB2
 * @param speed_rpm 最大速度 (RPM)
 */
esp_err_t can_motor_stw_set_max_speed(can_motor_handle_t motor, float speed_rpm);

/**
 * @brief 设置最大 Q 轴电流 — 0xB3
 * @param current_a 最大电流 (A)
 */
esp_err_t can_motor_stw_set_max_current(can_motor_handle_t motor, float current_a);

/**
 * @brief 设置电流斜率 — 0xB4
 * @param slope 电流斜率 (A/s)
 */
esp_err_t can_motor_stw_set_torque_slope(can_motor_handle_t motor, float slope);

/**
 * @brief 设置速度模式加速度 — 0xB5
 * @param accel 加速度 (RPM/s)
 */
esp_err_t can_motor_stw_set_accel(can_motor_handle_t motor, float accel);

/**
 * @brief 读取 PID 参数 — 0xB6~0xB9 (DLC=1 请求)
 * @param cmd STW_CMD_POS_KP / POS_KI / SPD_KP / SPD_KI
 * @note 结果通过 RX 解析后存入内部, 用 can_motor_stw_get_pid() 读取
 */
esp_err_t can_motor_stw_request_pid(can_motor_handle_t motor, uint8_t cmd);

/**
 * @brief 写入 PID 参数 — 0xB6~0xB9 (DLC=5 写入)
 * @param cmd STW_CMD_POS_KP / POS_KI / SPD_KP / SPD_KI
 * @param value PID 参数值 (IEEE 754 float)
 */
esp_err_t can_motor_stw_write_pid(can_motor_handle_t motor, uint8_t cmd, float value);

/**
 * @brief 获取上次 PID 读取结果
 * @param cmd STW_CMD_POS_KP / POS_KI / SPD_KP / SPD_KI
 * @param[out] value 参数值
 */
esp_err_t can_motor_stw_get_pid(can_motor_handle_t motor, uint8_t cmd, float *value);

/**
 * @brief 配置 MIT 运控参数 — 0xF0 (DLC=7 写入)
 * @param pos_max 位置最大值 (rad)
 * @param vel_max 速度最大值 (rad/s)
 * @param t_max 力矩最大值 (Nm)
 */
esp_err_t can_motor_stw_mit_set_config(can_motor_handle_t motor,
                                       float pos_max, float vel_max, float t_max);

/**
 * @brief 读取 MIT 配置 — 0xF0 (DLC=1 请求)
 */
esp_err_t can_motor_stw_mit_request_config(can_motor_handle_t motor);

/**
 * @brief 请求 MIT 实时状态 — 0xF1 (DLC=1 请求)
 * @note 结果通过 RX 解析后存入内部, 用 can_motor_stw_mit_get_state() 读取
 */
esp_err_t can_motor_stw_mit_request_state(can_motor_handle_t motor);

/**
 * @brief 获取上次 MIT 状态应答
 */
esp_err_t can_motor_stw_mit_get_state(can_motor_handle_t motor, stw_mit_state_t *state);

/**
 * @brief MIT 运控控制命令 — StdID=(0x400|addr), DLC=8
 * @param target_pos 目标位置 (rad)
 * @param target_vel 目标速度 (rad/s)
 * @param kp 位置刚度 (0~500)
 * @param kd 速度阻尼 (0~5)
 * @param target_torque 前馈力矩 (Nm)
 */
esp_err_t can_motor_stw_mit_control(can_motor_handle_t motor,
                                    float target_pos, float target_vel,
                                    float kp, float kd, float target_torque);

/**
 * @brief 开启/关闭 CAN 帧调试打印 (TX+RX hex dump)
 * @param enable true=开启, false=关闭
 * @param filter_motor_id 0=全部电机, >0=只打印指定电机ID的帧
 */
void can_motor_set_debug(bool enable, uint32_t filter_motor_id);

#ifdef __cplusplus
}
#endif

#endif // __CAN_MOTOR_H__
