/**
 * @file can_motor_regs.h
 * @brief CAN 电机寄存器定义
 * @author Bubble
 * @date 2026-01-15
 * @note 移植自 Arduino 版本 CANServoRegs.h
 */

#ifndef __CAN_MOTOR_REGS_H__
#define __CAN_MOTOR_REGS_H__

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// CAN 命令字
// ============================================================================

#define CAN_CMD_READ_1REG       0x4B    // 读取 1 个寄存器 (2字节)
#define CAN_CMD_READ_2REG       0x43    // 读取 2 个寄存器 (4字节)
#define CAN_CMD_WRITE_1REG      0x2B    // 写入 1 个寄存器 (2字节)
#define CAN_CMD_WRITE_2REG      0x23    // 写入 2 个寄存器 (4字节)
#define CAN_CMD_PV              0x24    // PV 指令 (位置+速度)
#define CAN_CMD_PVT             0x25    // PVT 指令 (位置+速度+力矩%)
#define CAN_CMD_REPLY           0x2A    // 驱动器写入回复

// ============================================================================
// 寄存器地址 - 只读状态
// ============================================================================

#define REG_HW_VERSION          0x0002  // 硬件版本
#define REG_FW_VERSION          0x0003  // 固件版本
#define REG_VOLTAGE             0x0004  // 电源电压 (×10)
#define REG_CURRENT             0x0005  // 母线电流 (×100)
#define REG_SPEED               0x0006  // 实时速度 (×100)
#define REG_POSITION            0x0008  // 实时位置 (×100) - 32位
#define REG_CLOCK_POS           0x0009  // 时钟位置 (×100)
#define REG_DRIVER_TEMP         0x000A  // 驱动器温度 (×10)
#define REG_MOTOR_TEMP          0x000B  // 电机温度 (×10)
#define REG_ERROR               0x000C  // 错误信息

// ============================================================================
// 寄存器地址 - 读写控制
// ============================================================================

#define REG_SET_TORQUE          0x0020  // 设置力矩 (×100)
#define REG_SET_SPEED           0x0021  // 设置速度 (×100)
#define REG_SET_ABS_POS         0x0023  // 设置绝对位置 (×100) - 32位
#define REG_SET_REL_POS         0x0025  // 设置相对位置 (×100) - 32位
#define REG_SET_LOW_SPEED       0x0027  // 设置低速 (×100) - 范围±300rpm
#define REG_CONTROL_MODE        0x0060  // 控制模式

// ============================================================================
// 寄存器地址 - 只写控制
// ============================================================================

#define REG_IDLE                0x00A0  // 进入空闲状态
#define REG_CALIBRATE           0x00A1  // 校准电机
#define REG_CLOSE_LOOP          0x00A2  // 进入闭环控制
#define REG_ERASE               0x00A3  // 擦除参数
#define REG_SAVE                0x00A4  // 保存参数到Flash
#define REG_REBOOT              0x00A5  // 重启驱动器
#define REG_SET_ORIGIN          0x00A6  // 设置当前位置为原点 (零点)

// ============================================================================
// 控制模式值
// ============================================================================

#define MODE_TORQUE_VAL         0       // 力矩模式
#define MODE_SPEED_VAL          1       // 速度模式
#define MODE_POS_TRAP_VAL       2       // 位置梯形轨迹
#define MODE_POS_FILTER_VAL     3       // 位置滤波模式
#define MODE_POS_DIRECT_VAL     4       // 位置直通模式
#define MODE_LOW_SPEED_VAL      5       // 低速大扭模式

// ============================================================================
// 错误码定义 (完整版，来自原始 Arduino 项目)
// ============================================================================

#define ERR_NONE                0x00000000  // 无错误
#define ERR_POWER_SMALL         0x00000001  // 电源功率小
#define ERR_PHASE_RES_HIGH      0x00000002  // 相电阻偏大
#define ERR_CURRENT_FLUCTUATE   0x00000008  // 电流波动大
#define ERR_INDUCTANCE_HIGH     0x00000010  // 电感偏大
#define ERR_ENCODER_BW          0x00000020  // 编码器带宽不合适
#define ERR_ENCODER_SPI         0x00000040  // 编码器SPI通信错误
#define ERR_ENCODER_TYPE        0x00000080  // 编码器型号错误
#define ERR_HALL_NOT_CALIB      0x00000100  // Hall电机未校准
#define ERR_ENCODER_NO_DATA     0x00000200  // 未读到编码器数据
#define ERR_CPR_ERROR           0x00000400  // CPR设置错误
#define ERR_RUN_STATE           0x00000800  // 运行状态错误
#define ERR_HALL_SIGNAL         0x00008000  // Hall信号错误
#define ERR_ENCODER2            0x00020000  // 第二编码器错误
#define ERR_DRIVER_JC2804       0x00080000  // JC2804驱动错误
#define ERR_MOS_OVERHEAT        0x00100000  // MOS高温报警
#define ERR_MOTOR_OVERHEAT      0x00200000  // 电机高温报警
#define ERR_UNDERVOLTAGE        0x00400000  // 欠压报警
#define ERR_OVERVOLTAGE         0x00800000  // 过压报警
#define ERR_OVERCURRENT         0x01000000  // 过流报警

// ============================================================================
// 数据缩放因子
// ============================================================================

#define SCALE_VOLTAGE           10.0f   // 电压: 实际值 = 寄存器值 / 10
#define SCALE_CURRENT           100.0f  // 电流: 实际值 = 寄存器值 / 100
#define SCALE_SPEED             100.0f  // 速度: 实际值 = 寄存器值 / 100
#define SCALE_POSITION          100.0f  // 位置: 实际值 = 寄存器值 / 100
#define SCALE_TEMPERATURE       10.0f   // 温度: 实际值 = 寄存器值 / 10
#define SCALE_TORQUE            100.0f  // 力矩: 实际值 = 寄存器值 / 100

#ifdef __cplusplus
}
#endif

#endif // __CAN_MOTOR_REGS_H__
