/**
 * @file wit_c_sdk.c
 * @brief WIT Motion SDK 核心实现
 * @note 移植自 https://github.com/Bubble252/WIT_imu_idf
 * @author WIT Motion / 移植: Bubble
 */

#include "wit_c_sdk.h"

// ============ 静态变量 ============
static SerialWrite p_WitSerialWriteFunc = NULL;
static WitI2cWrite p_WitI2cWriteFunc = NULL;
static WitI2cRead p_WitI2cReadFunc = NULL;
static CanWrite p_WitCanWriteFunc = NULL;
static RegUpdateCb p_WitRegUpdateCbFunc = NULL;
static DelaymsCb p_WitDelaymsFunc = NULL;

static uint8_t s_ucAddr = 0xff;
static uint8_t s_ucWitDataBuff[WIT_DATA_BUFF_SIZE];
static uint32_t s_uiWitDataCnt = 0, s_uiProtoclo = 0, s_uiReadRegIndex = 0;

// 全局寄存器数组
int16_t sReg[REGSIZE];

// ============ CRC16 查找表 ============
#define FuncW 0x06
#define FuncR 0x03

static const uint8_t __auchCRCHi[256] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01,
    0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0,
    0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01,
    0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81,
    0x40
};

static const uint8_t __auchCRCLo[256] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4,
    0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09,
    0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD,
    0x1D, 0x1C, 0xDC, 0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7,
    0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A,
    0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE,
    0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2,
    0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F,
    0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB,
    0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 0x90, 0x91,
    0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C,
    0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88,
    0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80,
    0x40
};

// ============ CRC16 计算 ============
static uint16_t __CRC16(uint8_t *puchMsg, uint16_t usDataLen) {
    uint8_t uchCRCHi = 0xFF;
    uint8_t uchCRCLo = 0xFF;
    uint32_t uIndex;
    
    while (usDataLen--) {
        uIndex = uchCRCHi ^ *puchMsg++;
        uchCRCHi = uchCRCLo ^ __auchCRCHi[uIndex];
        uchCRCLo = __auchCRCLo[uIndex];
    }
    return (uchCRCHi << 8 | uchCRCLo);
}

// ============ 解析标准协议数据帧 ============
static void CopeWitData(uint8_t ucIndex, uint16_t *p_usReg1Val, uint16_t *p_usReg2Val) {
    uint32_t uiReg1 = 0, uiReg2 = 0, uiReg1Len = 4, uiReg2Len = 0, uiLen = 4;
    
    if (p_WitRegUpdateCbFunc == NULL) return;
    
    uiReg1Len = 4;
    switch (ucIndex) {
        case WIT_ACC:   uiReg1 = AX;   uiReg1Len = 3; uiReg2 = TEMP; uiReg2Len = 1; break;
        case WIT_ANGLE: uiReg1 = Roll; uiReg1Len = 3; uiReg2 = VERSION; uiReg2Len = 1; break;
        case WIT_TIME:  uiReg1 = YYMM; break;
        case WIT_GYRO:  uiReg1 = GX;   uiLen = 3; break;
        case WIT_MAGNETIC: uiReg1 = HX; uiLen = 3; break;
        case WIT_DPORT: uiReg1 = D0Status; break;
        case WIT_PRESS: uiReg1 = PressureL; break;
        case WIT_GPS:   uiReg1 = LonL; break;
        case WIT_VELOCITY: uiReg1 = GPSHeight; break;
        case WIT_QUATER: uiReg1 = Q0; break;
        case WIT_GSA:   uiReg1 = SVNUM; break;
        case WIT_REGVALUE: uiReg1 = s_uiReadRegIndex; break;
        default:
            return;
    }
    
    if (uiLen == 3) {
        uiReg1Len = 3;
        uiReg2Len = 0;
    }
    
    if (uiReg1Len) {
        memcpy(&sReg[uiReg1], p_usReg1Val, uiReg1Len << 1);
        p_WitRegUpdateCbFunc(uiReg1, uiReg1Len);
    }
    if (uiReg2Len) {
        memcpy(&sReg[uiReg2], p_usReg2Val, uiReg2Len << 1);
        p_WitRegUpdateCbFunc(uiReg2, uiReg2Len);
    }
}

// ============ 串口数据输入 (逐字节解析) ============
void WitSerialDataIn(uint8_t ucData) {
    uint16_t usCRC16, usTemp, i, usData[4];
    
    s_ucWitDataBuff[s_uiWitDataCnt++] = ucData;
    
    switch (s_uiProtoclo) {
        case WIT_PROTOCOL_NORMAL:
            // 查找帧头 0x55
            while ((s_uiWitDataCnt >= 2) && 
                   !((s_ucWitDataBuff[0] == 0x55) && (s_ucWitDataBuff[1] >= 0x50) && (s_ucWitDataBuff[1] < 0x60))) {
                s_uiWitDataCnt--;
                memcpy(s_ucWitDataBuff, s_ucWitDataBuff + 1, s_uiWitDataCnt);
            }
            // 等待完整帧
            if (s_uiWitDataCnt >= 11) {
                // 校验和
                uint8_t ucSum = 0;
                for (i = 0; i < 10; i++) ucSum += s_ucWitDataBuff[i];
                if (ucSum == s_ucWitDataBuff[10]) {
                    for (i = 0; i < 4; i++) {
                        usData[i] = ((uint16_t)s_ucWitDataBuff[i * 2 + 3] << 8) | s_ucWitDataBuff[i * 2 + 2];
                    }
                    CopeWitData(s_ucWitDataBuff[1], usData, usData + 3);
                }
                s_uiWitDataCnt = 0;
            }
            break;
            
        case WIT_PROTOCOL_MODBUS:
            // 查找Modbus帧头
            while ((s_uiWitDataCnt >= 2) && 
                   !((s_ucWitDataBuff[0] == s_ucAddr) && (s_ucWitDataBuff[1] == FuncR))) {
                s_uiWitDataCnt--;
                memcpy(s_ucWitDataBuff, s_ucWitDataBuff + 1, s_uiWitDataCnt);
            }
            // 等待完整帧
            if (s_uiWitDataCnt >= 3) {
                usTemp = s_ucWitDataBuff[2];
                if (s_uiWitDataCnt >= (usTemp + 5)) {
                    usCRC16 = __CRC16(s_ucWitDataBuff, usTemp + 3);
                    if ((s_ucWitDataBuff[usTemp + 3] == (usCRC16 >> 8)) && 
                        (s_ucWitDataBuff[usTemp + 4] == (usCRC16 & 0xFF))) {
                        usTemp = usTemp >> 1;
                        for (i = 0; i < usTemp; i++) {
                            sReg[i + s_uiReadRegIndex] = ((uint16_t)s_ucWitDataBuff[(i << 1) + 3] << 8) | 
                                                          s_ucWitDataBuff[(i << 1) + 4];
                        }
                        p_WitRegUpdateCbFunc(s_uiReadRegIndex, usTemp);
                        s_uiWitDataCnt = 0;
                    }
                }
            }
            break;
            
        case WIT_PROTOCOL_CAN:
        case WIT_PROTOCOL_I2C:
            s_uiWitDataCnt = 0;
            break;
    }
    
    if (s_uiWitDataCnt == WIT_DATA_BUFF_SIZE) s_uiWitDataCnt = 0;
}

// ============ 函数注册 ============
int32_t WitSerialWriteRegister(SerialWrite write_func) {
    if (!write_func) return WIT_HAL_INVAL;
    p_WitSerialWriteFunc = write_func;
    return WIT_HAL_OK;
}

int32_t WitI2cFuncRegister(WitI2cWrite write_func, WitI2cRead read_func) {
    if (!write_func) return WIT_HAL_INVAL;
    if (!read_func) return WIT_HAL_INVAL;
    p_WitI2cWriteFunc = write_func;
    p_WitI2cReadFunc = read_func;
    return WIT_HAL_OK;
}

int32_t WitCanWriteRegister(CanWrite write_func) {
    if (!write_func) return WIT_HAL_INVAL;
    p_WitCanWriteFunc = write_func;
    return WIT_HAL_OK;
}

int32_t WitDelayMsRegister(DelaymsCb delayms_func) {
    if (!delayms_func) return WIT_HAL_INVAL;
    p_WitDelaymsFunc = delayms_func;
    return WIT_HAL_OK;
}

int32_t WitRegisterCallBack(RegUpdateCb update_func) {
    if (!update_func) return WIT_HAL_INVAL;
    p_WitRegUpdateCbFunc = update_func;
    return WIT_HAL_OK;
}

// ============ CAN数据输入 ============
void WitCanDataIn(uint8_t ucData[8], uint8_t ucLen) {
    uint16_t usData[4];
    uint8_t i;
    
    if (ucLen == 8) {
        if ((ucData[0] == 0x55) && (ucData[1] >= 0x50) && (ucData[1] < 0x60)) {
            for (i = 0; i < 3; i++) {
                usData[i] = ((uint16_t)ucData[i * 2 + 3] << 8) | ucData[i * 2 + 2];
            }
            CopeWitData(ucData[1], usData, usData);
        }
    }
}

// ============ 写寄存器 ============
int32_t WitWriteReg(uint32_t uiReg, uint16_t usData) {
    uint16_t usTemp;
    uint8_t ucBuff[8];
    
    if (uiReg >= REGSIZE) return WIT_HAL_INVAL;
    
    switch (s_uiProtoclo) {
        case WIT_PROTOCOL_NORMAL:
            if (p_WitSerialWriteFunc == NULL) return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = uiReg & 0xFF;
            ucBuff[3] = usData & 0xFF;
            ucBuff[4] = usData >> 8;
            p_WitSerialWriteFunc(ucBuff, 5);
            break;
            
        case WIT_PROTOCOL_MODBUS:
            if (p_WitSerialWriteFunc == NULL) return WIT_HAL_EMPTY;
            ucBuff[0] = s_ucAddr;
            ucBuff[1] = FuncW;
            ucBuff[2] = uiReg >> 8;
            ucBuff[3] = uiReg & 0xFF;
            ucBuff[4] = usData >> 8;
            ucBuff[5] = usData & 0xFF;
            usTemp = __CRC16(ucBuff, 6);
            ucBuff[6] = usTemp >> 8;
            ucBuff[7] = usTemp & 0xFF;
            p_WitSerialWriteFunc(ucBuff, 8);
            break;
            
        case WIT_PROTOCOL_CAN:
            if (p_WitCanWriteFunc == NULL) return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = uiReg & 0xFF;
            ucBuff[3] = usData & 0xFF;
            ucBuff[4] = usData >> 8;
            p_WitCanWriteFunc(s_ucAddr, ucBuff, 5);
            break;
            
        case WIT_PROTOCOL_I2C:
            if (p_WitI2cWriteFunc == NULL) return WIT_HAL_EMPTY;
            ucBuff[0] = usData & 0xFF;
            ucBuff[1] = usData >> 8;
            if (p_WitI2cWriteFunc(s_ucAddr << 1, uiReg, ucBuff, 2) != 0) {
                // I2C write failed
            }
            break;
            
        default:
            return WIT_HAL_INVAL;
    }
    
    return WIT_HAL_OK;
}

// ============ 读寄存器 ============
int32_t WitReadReg(uint32_t uiReg, uint32_t uiReadNum) {
    uint16_t usTemp, i;
    uint8_t ucBuff[8];
    
    if ((uiReg + uiReadNum) >= REGSIZE) return WIT_HAL_INVAL;
    
    switch (s_uiProtoclo) {
        case WIT_PROTOCOL_NORMAL:
            if (uiReadNum > 4) return WIT_HAL_INVAL;
            if (p_WitSerialWriteFunc == NULL) return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = 0x27;
            ucBuff[3] = uiReg & 0xFF;
            ucBuff[4] = uiReg >> 8;
            p_WitSerialWriteFunc(ucBuff, 5);
            break;
            
        case WIT_PROTOCOL_MODBUS:
            if (p_WitSerialWriteFunc == NULL) return WIT_HAL_EMPTY;
            usTemp = uiReadNum << 1;
            if ((usTemp + 5) > WIT_DATA_BUFF_SIZE) return WIT_HAL_NOMEM;
            ucBuff[0] = s_ucAddr;
            ucBuff[1] = FuncR;
            ucBuff[2] = uiReg >> 8;
            ucBuff[3] = uiReg & 0xFF;
            ucBuff[4] = uiReadNum >> 8;
            ucBuff[5] = uiReadNum & 0xFF;
            usTemp = __CRC16(ucBuff, 6);
            ucBuff[6] = usTemp >> 8;
            ucBuff[7] = usTemp & 0xFF;
            p_WitSerialWriteFunc(ucBuff, 8);
            break;
            
        case WIT_PROTOCOL_CAN:
            if (uiReadNum > 3) return WIT_HAL_INVAL;
            if (p_WitCanWriteFunc == NULL) return WIT_HAL_EMPTY;
            ucBuff[0] = 0xFF;
            ucBuff[1] = 0xAA;
            ucBuff[2] = 0x27;
            ucBuff[3] = uiReg & 0xFF;
            ucBuff[4] = uiReg >> 8;
            p_WitCanWriteFunc(s_ucAddr, ucBuff, 5);
            break;
            
        case WIT_PROTOCOL_I2C:
            if (p_WitI2cReadFunc == NULL) return WIT_HAL_EMPTY;
            usTemp = uiReadNum << 1;
            if (WIT_DATA_BUFF_SIZE < usTemp) return WIT_HAL_NOMEM;
            if (p_WitI2cReadFunc(s_ucAddr << 1, uiReg, s_ucWitDataBuff, usTemp) == 0) {
                if (p_WitRegUpdateCbFunc == NULL) return WIT_HAL_EMPTY;
                for (i = 0; i < uiReadNum; i++) {
                    sReg[i + uiReg] = ((uint16_t)s_ucWitDataBuff[(i << 1) + 1] << 8) | 
                                       s_ucWitDataBuff[i << 1];
                }
                p_WitRegUpdateCbFunc(uiReg, uiReadNum);
            }
            break;
            
        default:
            return WIT_HAL_INVAL;
    }
    
    s_uiReadRegIndex = uiReg;
    return WIT_HAL_OK;
}

// ============ 初始化/反初始化 ============
int32_t WitInit(uint32_t uiProtocol, uint8_t ucAddr) {
    if (uiProtocol > WIT_PROTOCOL_I2C) return WIT_HAL_INVAL;
    s_uiProtoclo = uiProtocol;
    s_ucAddr = ucAddr;
    s_uiWitDataCnt = 0;
    return WIT_HAL_OK;
}

void WitDeInit(void) {
    p_WitSerialWriteFunc = NULL;
    p_WitI2cWriteFunc = NULL;
    p_WitI2cReadFunc = NULL;
    p_WitCanWriteFunc = NULL;
    p_WitRegUpdateCbFunc = NULL;
    s_ucAddr = 0xFF;
    s_uiWitDataCnt = 0;
    s_uiProtoclo = 0;
}

// ============ 范围检查 ============
char CheckRange(short sTemp, short sMin, short sMax) {
    if ((sTemp >= sMin) && (sTemp <= sMax)) return 1;
    else return 0;
}

// ============ 校准接口 ============
int32_t WitStartAccCali(void) {
    if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK) return WIT_HAL_ERROR;
    if (s_uiProtoclo == WIT_PROTOCOL_MODBUS) p_WitDelaymsFunc(20);
    else if (s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(10);
    if (WitWriteReg(CALSW, CALGYROACC) != WIT_HAL_OK) return WIT_HAL_ERROR;
    return WIT_HAL_OK;
}

int32_t WitStopAccCali(void) {
    if (WitWriteReg(CALSW, NORMAL) != WIT_HAL_OK) return WIT_HAL_ERROR;
    if (s_uiProtoclo == WIT_PROTOCOL_MODBUS) p_WitDelaymsFunc(20);
    else if (s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(10);
    if (WitWriteReg(SAVE, SAVE_PARAM) != WIT_HAL_OK) return WIT_HAL_ERROR;
    return WIT_HAL_OK;
}

int32_t WitStartMagCali(void) {
    if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK) return WIT_HAL_ERROR;
    if (s_uiProtoclo == WIT_PROTOCOL_MODBUS) p_WitDelaymsFunc(20);
    else if (s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(10);
    if (WitWriteReg(CALSW, CALMAGMM) != WIT_HAL_OK) return WIT_HAL_ERROR;
    return WIT_HAL_OK;
}

int32_t WitStopMagCali(void) {
    if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK) return WIT_HAL_ERROR;
    if (s_uiProtoclo == WIT_PROTOCOL_MODBUS) p_WitDelaymsFunc(20);
    else if (s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(10);
    if (WitWriteReg(CALSW, NORMAL) != WIT_HAL_OK) return WIT_HAL_ERROR;
    return WIT_HAL_OK;
}

// ============ 配置接口 ============
int32_t WitSetUartBaud(int32_t uiBaudIndex) {
    if (!CheckRange(uiBaudIndex, WIT_BAUD_4800, WIT_BAUD_921600)) {
        return WIT_HAL_INVAL;
    }
    if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK) return WIT_HAL_ERROR;
    if (s_uiProtoclo == WIT_PROTOCOL_MODBUS) p_WitDelaymsFunc(20);
    else if (s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(10);
    if (WitWriteReg(BAUD, uiBaudIndex) != WIT_HAL_OK) return WIT_HAL_ERROR;
    p_WitDelaymsFunc(500);
    return WIT_HAL_OK;
}

int32_t WitSetCanBaud(int32_t uiBaudIndex) {
    if (!CheckRange(uiBaudIndex, CAN_BAUD_1000000, CAN_BAUD_3000)) {
        return WIT_HAL_INVAL;
    }
    if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK) return WIT_HAL_ERROR;
    if (s_uiProtoclo == WIT_PROTOCOL_MODBUS) p_WitDelaymsFunc(20);
    else if (s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(10);
    if (WitWriteReg(BAUD, uiBaudIndex) != WIT_HAL_OK) return WIT_HAL_ERROR;
    return WIT_HAL_OK;
}

int32_t WitSetBandwidth(int32_t uiBaudWidth) {
    if (!CheckRange(uiBaudWidth, BANDWIDTH_256HZ, BANDWIDTH_5HZ)) {
        return WIT_HAL_INVAL;
    }
    if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK) return WIT_HAL_ERROR;
    if (s_uiProtoclo == WIT_PROTOCOL_MODBUS) p_WitDelaymsFunc(20);
    else if (s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(10);
    if (WitWriteReg(BANDWIDTH, uiBaudWidth) != WIT_HAL_OK) return WIT_HAL_ERROR;
    return WIT_HAL_OK;
}

int32_t WitSetOutputRate(int32_t uiRate) {
    if (!CheckRange(uiRate, RRATE_02HZ, RRATE_NOOUTPUT)) {
        return WIT_HAL_INVAL;
    }
    if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK) return WIT_HAL_ERROR;
    if (s_uiProtoclo == WIT_PROTOCOL_MODBUS) p_WitDelaymsFunc(20);
    else if (s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(10);
    if (WitWriteReg(RRATE, uiRate) != WIT_HAL_OK) return WIT_HAL_ERROR;
    return WIT_HAL_OK;
}

int32_t WitSetContent(int32_t uiRsw) {
    if (WitWriteReg(KEY, KEY_UNLOCK) != WIT_HAL_OK) return WIT_HAL_ERROR;
    if (s_uiProtoclo == WIT_PROTOCOL_MODBUS) p_WitDelaymsFunc(20);
    else if (s_uiProtoclo == WIT_PROTOCOL_NORMAL) p_WitDelaymsFunc(10);
    if (WitWriteReg(RSW, uiRsw & RSW_MASK) != WIT_HAL_OK) return WIT_HAL_ERROR;
    return WIT_HAL_OK;
}
