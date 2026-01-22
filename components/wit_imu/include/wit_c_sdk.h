/**
 * @file wit_c_sdk.h
 * @brief WIT Motion SDK 核心接口
 * @note 移植自 https://github.com/Bubble252/WIT_imu_idf
 * @author WIT Motion / 移植: Bubble
 */

#ifndef __WIT_C_SDK_H
#define __WIT_C_SDK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "wit_reg.h"

// ============ 返回值定义 ============
#define WIT_HAL_OK      (0)     // 成功
#define WIT_HAL_BUSY    (-1)    // 忙
#define WIT_HAL_TIMEOUT (-2)    // 超时
#define WIT_HAL_ERROR   (-3)    // 通用错误
#define WIT_HAL_NOMEM   (-4)    // 内存不足
#define WIT_HAL_EMPTY   (-5)    // 资源为空
#define WIT_HAL_INVAL   (-6)    // 无效参数

// ============ 数据缓冲区大小 ============
#define WIT_DATA_BUFF_SIZE  256

// ============ 通信协议类型 ============
#define WIT_PROTOCOL_NORMAL 0   // 标准协议
#define WIT_PROTOCOL_MODBUS 1   // Modbus协议
#define WIT_PROTOCOL_CAN    2   // CAN协议
#define WIT_PROTOCOL_I2C    3   // I2C协议

// ============ 数据更新标志 ============
#define ACC_UPDATE      0x01
#define GYRO_UPDATE     0x02
#define MAG_UPDATE      0x04
#define ANGLE_UPDATE    0x08
#define READ_UPDATE     0x80

// ============ 函数指针类型定义 ============

/**
 * @brief 串口写函数类型
 */
typedef void (*SerialWrite)(uint8_t *p_ucData, uint32_t uiLen);

/**
 * @brief I2C写函数类型
 * @param ucAddr I2C设备地址 (已左移1位)
 * @param ucReg 寄存器地址
 * @param p_ucVal 数据指针
 * @param uiLen 数据长度
 * @return 0成功，非0失败
 */
typedef int32_t (*WitI2cWrite)(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);

/**
 * @brief I2C读函数类型
 * @param ucAddr I2C设备地址 (已左移1位)
 * @param ucReg 寄存器地址
 * @param p_ucVal 数据缓冲区
 * @param uiLen 读取长度
 * @return 0成功，非0失败
 */
typedef int32_t (*WitI2cRead)(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);

/**
 * @brief CAN写函数类型
 */
typedef void (*CanWrite)(uint8_t ucStdId, uint8_t *p_ucData, uint32_t uiLen);

/**
 * @brief 延时函数类型 (毫秒)
 */
typedef void (*DelaymsCb)(uint16_t ucMs);

/**
 * @brief 寄存器更新回调函数类型
 * @param uiReg 起始寄存器地址
 * @param uiRegNum 寄存器数量
 */
typedef void (*RegUpdateCb)(uint32_t uiReg, uint32_t uiRegNum);

// ============ 函数注册接口 ============

/**
 * @brief 注册串口写函数
 */
int32_t WitSerialWriteRegister(SerialWrite write_func);

/**
 * @brief 串口数据输入 (逐字节解析)
 */
void WitSerialDataIn(uint8_t ucData);

/**
 * @brief 注册I2C读写函数
 */
int32_t WitI2cFuncRegister(WitI2cWrite write_func, WitI2cRead read_func);

/**
 * @brief 注册CAN写函数
 */
int32_t WitCanWriteRegister(CanWrite write_func);

/**
 * @brief CAN数据输入
 */
void WitCanDataIn(uint8_t ucData[8], uint8_t ucLen);

/**
 * @brief 注册延时函数
 */
int32_t WitDelayMsRegister(DelaymsCb delayms_func);

/**
 * @brief 注册寄存器更新回调
 */
int32_t WitRegisterCallBack(RegUpdateCb update_func);

// ============ 核心接口 ============

/**
 * @brief 初始化WIT SDK
 * @param uiProtocol 通信协议 (WIT_PROTOCOL_*)
 * @param ucAddr 设备地址 (I2C: 7位地址, 如0x50)
 * @return WIT_HAL_OK 成功
 */
int32_t WitInit(uint32_t uiProtocol, uint8_t ucAddr);

/**
 * @brief 反初始化WIT SDK
 */
void WitDeInit(void);

/**
 * @brief 写寄存器
 * @param uiReg 寄存器地址
 * @param usData 数据
 * @return WIT_HAL_OK 成功
 */
int32_t WitWriteReg(uint32_t uiReg, uint16_t usData);

/**
 * @brief 读寄存器
 * @param uiReg 起始寄存器地址
 * @param uiReadNum 读取寄存器数量
 * @return WIT_HAL_OK 成功
 */
int32_t WitReadReg(uint32_t uiReg, uint32_t uiReadNum);

// ============ 校准接口 ============

/**
 * @brief 开始加速度校准
 * @note 需要将设备水平放置
 */
int32_t WitStartAccCali(void);

/**
 * @brief 停止加速度校准
 */
int32_t WitStopAccCali(void);

/**
 * @brief 开始磁力计校准
 * @note 校准过程中需要绕3个轴旋转
 */
int32_t WitStartMagCali(void);

/**
 * @brief 停止磁力计校准
 */
int32_t WitStopMagCali(void);

// ============ 配置接口 ============

/**
 * @brief 设置串口波特率
 * @param uiBaudIndex WIT_BAUD_* 值
 */
int32_t WitSetUartBaud(int32_t uiBaudIndex);

/**
 * @brief 设置CAN波特率
 * @param uiBaudIndex CAN_BAUD_* 值
 */
int32_t WitSetCanBaud(int32_t uiBaudIndex);

/**
 * @brief 设置带宽
 * @param uiBaudWidth BANDWIDTH_* 值
 */
int32_t WitSetBandwidth(int32_t uiBaudWidth);

/**
 * @brief 设置输出速率
 * @param uiRate RRATE_* 值
 */
int32_t WitSetOutputRate(int32_t uiRate);

/**
 * @brief 设置输出内容
 * @param uiRsw RSW_* 标志组合
 */
int32_t WitSetContent(int32_t uiRsw);

// ============ 工具函数 ============

/**
 * @brief 检查范围
 */
char CheckRange(short sTemp, short sMin, short sMax);

// ============ 全局寄存器数组 ============
extern int16_t sReg[REGSIZE];

#ifdef __cplusplus
}
#endif

#endif /* __WIT_C_SDK_H */
