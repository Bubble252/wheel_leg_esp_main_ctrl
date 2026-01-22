/**
 * @file REG.h
 * @brief WIT IMU 寄存器定义
 * @note 移植自 https://github.com/Bubble252/WIT_imu_idf
 */

#ifndef __AHRSREG_H
#define __AHRSREG_H

#ifdef __cplusplus
extern "C" {
#endif

#define REGSIZE 0x90

// ============ 配置寄存器 (可读写) ============
#define SAVE        0x00    // 保存设置
#define CALSW       0x01    // 校准开关
#define RSW         0x02    // 输出内容选择
#define RRATE       0x03    // 输出速率
#define BAUD        0x04    // 波特率
#define AXOFFSET    0x05    // 加速度X偏移
#define AYOFFSET    0x06    // 加速度Y偏移
#define AZOFFSET    0x07    // 加速度Z偏移
#define GXOFFSET    0x08    // 陀螺仪X偏移
#define GYOFFSET    0x09    // 陀螺仪Y偏移
#define GZOFFSET    0x0a    // 陀螺仪Z偏移
#define HXOFFSET    0x0b    // 磁力计X偏移
#define HYOFFSET    0x0c    // 磁力计Y偏移
#define HZOFFSET    0x0d    // 磁力计Z偏移
#define D0MODE      0x0e    // D0模式
#define D1MODE      0x0f    // D1模式
#define D2MODE      0x10    // D2模式
#define D3MODE      0x11    // D3模式
#define D0PWMH      0x12    // D0 PWM高
#define D1PWMH      0x13    // D1 PWM高
#define D2PWMH      0x14    // D2 PWM高
#define D3PWMH      0x15    // D3 PWM高
#define D0PWMT      0x16    // D0 PWM周期
#define D1PWMT      0x17    // D1 PWM周期
#define D2PWMT      0x18    // D2 PWM周期
#define D3PWMT      0x19    // D3 PWM周期
#define IICADDR     0x1a    // I2C地址
#define LEDOFF      0x1b    // LED开关
#define MAGRANGX    0x1c    // 磁力计X范围
#define MAGRANGY    0x1d    // 磁力计Y范围
#define MAGRANGZ    0x1e    // 磁力计Z范围
#define BANDWIDTH   0x1f    // 带宽
#define GYRORANGE   0x20    // 陀螺仪量程
#define ACCRANGE    0x21    // 加速度量程
#define SLEEP       0x22    // 休眠
#define ORIENT      0x23    // 安装方向
#define AXIS6       0x24    // 6轴/9轴算法
#define FILTK       0x25    // 滤波系数
#define GPSBAUD     0x26    // GPS波特率
#define READADDR    0x27    // 读取地址
#define BWSCALE     0x28    // 带宽缩放
#define MOVETHR     0x28    // 运动阈值
#define MOVESTA     0x29    // 运动状态
#define ACCFILT     0x2A    // 加速度滤波
#define GYROFILT    0x2b    // 陀螺仪滤波
#define MAGFILT     0x2c    // 磁力计滤波
#define POWONSEND   0x2d    // 上电发送
#define VERSION     0x2e    // 版本
#define CCBW        0x2f    // CC带宽

// ============ 数据寄存器 (只读) ============
#define YYMM        0x30    // 年月
#define DDHH        0x31    // 日时
#define MMSS        0x32    // 分秒
#define MS          0x33    // 毫秒
#define AX          0x34    // 加速度X
#define AY          0x35    // 加速度Y
#define AZ          0x36    // 加速度Z
#define GX          0x37    // 陀螺仪X
#define GY          0x38    // 陀螺仪Y
#define GZ          0x39    // 陀螺仪Z
#define HX          0x3a    // 磁力计X
#define HY          0x3b    // 磁力计Y
#define HZ          0x3c    // 磁力计Z
#define Roll        0x3d    // 横滚角
#define Pitch       0x3e    // 俯仰角
#define Yaw         0x3f    // 偏航角
#define TEMP        0x40    // 温度
#define D0Status    0x41    // D0状态
#define D1Status    0x42    // D1状态
#define D2Status    0x43    // D2状态
#define D3Status    0x44    // D3状态
#define PressureL   0x45    // 气压低位
#define PressureH   0x46    // 气压高位
#define HeightL     0x47    // 高度低位
#define HeightH     0x48    // 高度高位
#define LonL        0x49    // 经度低位
#define LonH        0x4a    // 经度高位
#define LatL        0x4b    // 纬度低位
#define LatH        0x4c    // 纬度高位
#define GPSHeight   0x4d    // GPS高度
#define GPSYAW      0x4e    // GPS航向
#define GPSVL       0x4f    // GPS速度低位
#define GPSVH       0x50    // GPS速度高位
#define Q0          0x51    // 四元数q0
#define Q1          0x52    // 四元数q1
#define Q2          0x53    // 四元数q2
#define Q3          0x54    // 四元数q3
#define SVNUM       0x55    // 卫星数
#define PDOP        0x56    // 位置精度因子
#define HDOP        0x57    // 水平精度因子
#define VDOP        0x58    // 垂直精度因子
#define DELAYT      0x59    // 延时

#define XMIN        0x5a
#define XMAX        0x5b
#define BATVAL      0x5c
#define ALARMPIN    0x5d
#define YMIN        0x5e
#define YMAX        0x5f
#define GYROZSCALE  0x60
#define GYROCALITHR 0x61
#define ALARMLEVEL  0x62
#define GYROCALTIME 0x63
#define REFROLL     0x64
#define REFPITCH    0x65
#define REFYAW      0x66
#define GPSTYPE     0x67
#define TRIGTIME    0x68
#define KEY         0x69    // 解锁密钥
#define WERROR      0x6a
#define TIMEZONE    0x6b
#define CALICNT     0x6c
#define WZCNT       0x6d
#define WZTIME      0x6e
#define WZSTATIC    0x6f
#define ACCSENSOR   0x70
#define GYROSENSOR  0x71
#define MAGSENSOR   0x72
#define PRESSENSOR  0x73
#define MODDELAY    0x74
#define ANGLEAXIS   0x75
#define XRSCALE     0x76
#define YRSCALE     0x77
#define ZRSCALE     0x78
#define XREFROLL    0x79
#define YREFPITCH   0x7a
#define ZREFYAW     0x7b
#define ANGXOFFSET  0x7c
#define ANGYOFFSET  0x7d
#define ANGZOFFSET  0x7e
#define NUMBERID1   0x7f
#define NUMBERID2   0x80
#define NUMBERID3   0x81
#define NUMBERID4   0x82
#define NUMBERID5   0x83
#define NUMBERID6   0x84
#define XA85PSCALE  0x85
#define XA85NSCALE  0x86
#define YA85PSCALE  0x87
#define YA85NSCALE  0x88
#define XA30PSCALE  0x89
#define XA30NSCALE  0x8a
#define YA30PSCALE  0x8b
#define YA30NSCALE  0x8c
#define CHIPIDL     0x8D
#define CHIPIDH     0x8E
#define REGINITFLAG (REGSIZE-1)

// ============ 算法选择 ============
#define ALGRITHM9   0       // 9轴算法
#define ALGRITHM6   1       // 6轴算法

// ============ 校准开关 (CALSW) ============
#define NORMAL          0x00    // 正常模式
#define CALGYROACC      0x01    // 加速度/陀螺仪校准
#define CALMAG          0x02    // 磁力计校准
#define CALALTITUDE     0x03    // 高度校准
#define CALANGLEZ       0x04    // Z轴角度校准
#define CALACCL         0x05    // 加速度L校准
#define CALACCR         0x06    // 加速度R校准
#define CALMAGMM        0x07    // 磁力计MM校准
#define CALREFANGLE     0x08    // 参考角度校准
#define CALMAG2STEP     0x09    // 两步磁力计校准
#define CALHEXAHEDRON   0x12    // 六面体校准

// ============ 输出内容标志 (RSW) ============
#define RSW_TIME    0x01
#define RSW_ACC     0x02
#define RSW_GYRO    0x04
#define RSW_ANGLE   0x08
#define RSW_MAG     0x10
#define RSW_PORT    0x20
#define RSW_PRESS   0x40
#define RSW_GPS     0x80
#define RSW_V       0x100
#define RSW_Q       0x200
#define RSW_GSA     0x400
#define RSW_MASK    0xfff

// ============ 输出速率 (RRATE) ============
#define RRATE_NONE      0x0d
#define RRATE_02HZ      0x01
#define RRATE_05HZ      0x02
#define RRATE_1HZ       0x03
#define RRATE_2HZ       0x04
#define RRATE_5HZ       0x05
#define RRATE_10HZ      0x06
#define RRATE_20HZ      0x07
#define RRATE_50HZ      0x08
#define RRATE_100HZ     0x09
#define RRATE_125HZ     0x0a
#define RRATE_200HZ     0x0b
#define RRATE_SINGLE    0x0c
#define RRATE_NOOUTPUT  0x0d

// ============ 波特率 (BAUD) ============
#define WIT_BAUD_4800       1
#define WIT_BAUD_9600       2
#define WIT_BAUD_19200      3
#define WIT_BAUD_38400      4
#define WIT_BAUD_57600      5
#define WIT_BAUD_115200     6
#define WIT_BAUD_230400     7
#define WIT_BAUD_460800     8
#define WIT_BAUD_921600     9

// ============ CAN 波特率 ============
#define CAN_BAUD_1000000    0
#define CAN_BAUD_800000     1
#define CAN_BAUD_500000     2
#define CAN_BAUD_400000     3
#define CAN_BAUD_250000     4
#define CAN_BAUD_200000     5
#define CAN_BAUD_125000     6
#define CAN_BAUD_100000     7
#define CAN_BAUD_50000      8
#define CAN_BAUD_20000      9
#define CAN_BAUD_10000      10
#define CAN_BAUD_5000       11
#define CAN_BAUD_3000       12

// ============ 带宽 (BANDWIDTH) ============
#define BANDWIDTH_256HZ     0
#define BANDWIDTH_184HZ     1
#define BANDWIDTH_94HZ      2
#define BANDWIDTH_44HZ      3
#define BANDWIDTH_21HZ      4
#define BANDWIDTH_10HZ      5
#define BANDWIDTH_5HZ       6

// ============ 输出头 ============
#define WIT_TIME        0x50
#define WIT_ACC         0x51
#define WIT_GYRO        0x52
#define WIT_ANGLE       0x53
#define WIT_MAGNETIC    0x54
#define WIT_DPORT       0x55
#define WIT_PRESS       0x56
#define WIT_GPS         0x57
#define WIT_VELOCITY    0x58
#define WIT_QUATER      0x59
#define WIT_GSA         0x5A
#define WIT_REGVALUE    0x5F

// ============ 保存设置 (SAVE) ============
#define SAVE_PARAM      0x00    // 保存参数
#define SAVE_SWAVE      0x01    // 保存波形

// ============ 解锁密钥 ============
#define KEY_UNLOCK      0xB588

#ifdef __cplusplus
}
#endif

#endif /* __AHRSREG_H */
