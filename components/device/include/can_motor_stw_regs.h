/**
 * @file can_motor_stw_regs.h
 * @brief 伺泰威 (STW) 电机 CAN 命令码与协议定义
 * @author Bubble
 * @date 2026-01-15
 * @note 基于《自定义CAN通信协议_V3.07b0》
 */

#ifndef __CAN_MOTOR_STW_REGS_H__
#define __CAN_MOTOR_STW_REGS_H__

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// STW CAN 地址方案
// ============================================================================

// 主机发送: StdID = Dev_addr 或 (0x100 | Dev_addr)
// 从机应答: StdID = Dev_addr
// 广播地址: 0x00 (所有从机执行但不应答)
// 公共地址: 0xFF (所有从机响应并应答)
// MIT 运控: StdID = (0x400 | Dev_addr)

#define STW_TX_ID_OFFSET    0x100       // 主机发送偏移 (用于区分方向)
#define STW_MIT_ID_OFFSET   0x400       // MIT 运控模式 StdID 偏移

// ============================================================================
// STW 命令码 - 系统类
// ============================================================================

#define STW_CMD_REBOOT          0x00    // 重启从机 (不应答)
#define STW_CMD_READ_VERSION    0xA0    // 读 Boot/软件/硬件/协议版本
#define STW_CMD_READ_CURRENT    0xA1    // 读实时 Q 轴电流
#define STW_CMD_READ_SPEED      0xA2    // 读实时旋转速度
#define STW_CMD_READ_ANGLE      0xA3    // 读实时单圈/多圈角度
#define STW_CMD_READ_COMPACT    0xA4    // 读温度+电流+速度+角度 (紧凑)
#define STW_CMD_READ_STATUS     0xAE    // 读电压/电流/温度/模式/故障
#define STW_CMD_CLEAR_FAULT     0xAF    // 清除故障

// ============================================================================
// STW 命令码 - 参数类
// ============================================================================

#define STW_CMD_READ_MOTOR_INFO 0xB0    // 读极对数/力矩常数/减速比
#define STW_CMD_SET_ORIGIN      0xB1    // 设置当前位置为原点
#define STW_CMD_SET_MAX_SPEED   0xB2    // 设置位置模式最大速度
#define STW_CMD_SET_MAX_CURRENT 0xB3    // 设置最大 Q 轴电流
#define STW_CMD_SET_TORQUE_SLOPE 0xB4   // 设置电流斜率
#define STW_CMD_SET_ACCEL       0xB5    // 设置速度模式加速度
#define STW_CMD_POS_KP          0xB6    // 读/设位置控制 Kp
#define STW_CMD_POS_KI          0xB7    // 读/设位置控制 Ki
#define STW_CMD_SPD_KP          0xB8    // 读/设速度控制 Kp
#define STW_CMD_SPD_KI          0xB9    // 读/设速度控制 Ki

// ============================================================================
// STW 命令码 - 控制类
// ============================================================================

#define STW_CMD_TORQUE          0xC0    // Q 轴电流控制 (力矩)
#define STW_CMD_SPEED           0xC1    // 速度控制
#define STW_CMD_ABS_POS         0xC2    // 绝对值位置控制
#define STW_CMD_REL_POS         0xC3    // 相对位置控制
#define STW_CMD_RETURN_ORIGIN   0xC4    // 最短距离回原点
#define STW_CMD_BRAKE           0xCE    // 抱闸控制
#define STW_CMD_IDLE            0xCF    // 关闭输出 (自由态)

// ============================================================================
// STW 命令码 - MIT 运控模式
// ============================================================================

#define STW_CMD_MIT_CONFIG      0xF0    // MIT 参数配置 (Pos_Max/Vel_Max/T_Max)
#define STW_CMD_MIT_READ_STATE  0xF1    // 读取 MIT 实时状态

// ============================================================================
// STW 重启命令附加数据
// ============================================================================

#define STW_REBOOT_DATA_1       0xFF
#define STW_REBOOT_DATA_2       0x00
#define STW_REBOOT_DATA_3       0xFF
#define STW_REBOOT_DATA_4       0x00
#define STW_REBOOT_DATA_5       0xFF
#define STW_REBOOT_DATA_6       0x00
#define STW_REBOOT_DATA_7       0xFF

// ============================================================================
// STW 数据缩放因子
// ============================================================================

#define STW_SCALE_CURRENT       1000.0f     // 电流: 单位 0.001A
#define STW_SCALE_SPEED         100.0f      // 速度: 单位 0.01RPM
#define STW_SCALE_VOLTAGE       100.0f      // 电压: 单位 0.01V
#define STW_SCALE_BUS_CURRENT   100.0f      // 母线电流: 单位 0.01A

// 力矩常数 Kt (Nm/A): 力矩 = Kt × Q轴电流
// !! 请根据实际电机规格修改此值, 可通过 0xB0 命令从电机读取 !!
#define STW_TORQUE_CONSTANT     0.44f       // Kt (Nm/A), 从 0xB0 实测: poles=11, Kt=0.44, gear=8

// 位置: 16384 count/rev → 度
#define STW_POS_COUNTS_PER_REV  16384.0f
#define STW_POS_DEG_TO_COUNT    (STW_POS_COUNTS_PER_REV / 360.0f)  // ≈45.511
#define STW_POS_COUNT_TO_DEG    (360.0f / STW_POS_COUNTS_PER_REV)  // ≈0.02197

// MIT 运控模式缩放
#define STW_MIT_SCALE_POS       10.0f       // 0.1 rad
#define STW_MIT_SCALE_VEL       100.0f      // 0.01 rad/s
#define STW_MIT_SCALE_TORQUE    100.0f      // 0.01 Nm

// MIT 默认最大值
#define STW_MIT_DEFAULT_POS_MAX     95.5f   // rad (955 * 0.1)
#define STW_MIT_DEFAULT_VEL_MAX     45.0f   // rad/s (4500 * 0.01)
#define STW_MIT_DEFAULT_TORQUE_MAX  18.0f   // Nm (1800 * 0.01)

// ============================================================================
// STW 运行模式值 (0xAE 应答 Data[6])
// ============================================================================

#define STW_MODE_OFF            0       // 关闭状态
#define STW_MODE_VOLTAGE        1       // 电压控制
#define STW_MODE_TORQUE         2       // Q 轴电流控制
#define STW_MODE_SPEED          3       // 速度控制
#define STW_MODE_POSITION       4       // 位置控制

// ============================================================================
// STW 故障码位定义 (0xAE/0xAF 应答 Data[7])
// ============================================================================

#define STW_FAULT_VOLTAGE       (1 << 0)    // Bit0: 电压故障
#define STW_FAULT_CURRENT       (1 << 1)    // Bit1: 电流故障
#define STW_FAULT_TEMP          (1 << 2)    // Bit2: 温度故障
#define STW_FAULT_ENCODER       (1 << 3)    // Bit3: 编码器故障
#define STW_FAULT_HARDWARE      (1 << 6)    // Bit6: 硬件故障
#define STW_FAULT_SOFTWARE      (1 << 7)    // Bit7: 软件故障

#ifdef __cplusplus
}
#endif

#endif // __CAN_MOTOR_STW_REGS_H__
