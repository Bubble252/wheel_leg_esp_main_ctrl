/**
 * @file wit_imu.c
 * @brief WIT Motion IMU ESP-IDF驱动实现
 * @note 移植自 https://github.com/Bubble252/WIT_imu_idf
 * @author Bubble
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "wit_c_sdk.h"

static const char *TAG = "WIT_IMU";

// ============ 配置参数 ============
#define WIT_I2C_SCL_GPIO        GPIO_NUM_11
#define WIT_I2C_SDA_GPIO        GPIO_NUM_12
#define WIT_I2C_FREQ_HZ         400000      // 400kHz (Fast Mode, 支持500Hz控制频率)
#define WIT_I2C_TIMEOUT_MS      1000
#define WIT_DEFAULT_ADDR        0x50

// ============ 私有变量 ============
static i2c_master_bus_handle_t s_i2c_bus = NULL;
static i2c_master_dev_handle_t s_i2c_dev = NULL;
static volatile char s_cDataUpdate = 0;
static volatile bool s_bInit = false;
static uint8_t s_ucDevAddr = WIT_DEFAULT_ADDR;

// 传感器数据
static float s_fAcc[3], s_fGyro[3], s_fAngle[3];

// ============ 外部引用全局寄存器 ============
extern int16_t sReg[REGSIZE];

// ============ 私有函数声明 ============
static int32_t WitIICWrite(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);
static int32_t WitIICRead(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen);
static void Delayms(uint16_t usMs);
static void SensorDataUpdate(uint32_t uiReg, uint32_t uiRegNum);

// ============ I2C底层读写实现 ============
static int32_t WitIICWrite(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen) {
    if (s_i2c_dev == NULL) {
        ESP_LOGE(TAG, "I2C device not initialized");
        return -1;
    }
    
    // 构建写入数据: [寄存器地址][数据...]
    uint8_t *write_buf = malloc(uiLen + 1);
    if (write_buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate write buffer");
        return -1;
    }
    
    write_buf[0] = ucReg;
    memcpy(&write_buf[1], p_ucVal, uiLen);
    
    esp_err_t ret = i2c_master_transmit(s_i2c_dev, write_buf, uiLen + 1, WIT_I2C_TIMEOUT_MS);
    free(write_buf);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C write failed: %s", esp_err_to_name(ret));
        return -1;
    }
    
    return 0;
}

static int32_t WitIICRead(uint8_t ucAddr, uint8_t ucReg, uint8_t *p_ucVal, uint32_t uiLen) {
    if (s_i2c_dev == NULL) {
        ESP_LOGE(TAG, "I2C device not initialized");
        return -1;
    }
    
    // 写入寄存器地址，然后读取数据
    esp_err_t ret = i2c_master_transmit_receive(s_i2c_dev, &ucReg, 1, p_ucVal, uiLen, WIT_I2C_TIMEOUT_MS);
    
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C read failed: %s", esp_err_to_name(ret));
        return -1;
    }
    
    return 0;
}

// ============ 延时回调 ============
static void Delayms(uint16_t usMs) {
    vTaskDelay(pdMS_TO_TICKS(usMs));
}

// ============ 数据更新回调 ============
static void SensorDataUpdate(uint32_t uiReg, uint32_t uiRegNum) {
    int i;
    for (i = 0; i < (int)uiRegNum; i++) {
        switch (uiReg) {
            case AX:
            case AX + 1:
            case AX + 2:
                s_cDataUpdate |= ACC_UPDATE;
                break;
            case GX:
            case GX + 1:
            case GX + 2:
                s_cDataUpdate |= GYRO_UPDATE;
                break;
            case HX:
            case HX + 1:
            case HX + 2:
                s_cDataUpdate |= MAG_UPDATE;
                break;
            case Roll:
            case Roll + 1:
            case Roll + 2:
                s_cDataUpdate |= ANGLE_UPDATE;
                break;
            default:
                s_cDataUpdate |= READ_UPDATE;
                break;
        }
        uiReg++;
    }
}

// ============ I2C总线初始化 ============
static esp_err_t wit_i2c_init(void) {
    // 配置I2C主机总线
    i2c_master_bus_config_t i2c_bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = WIT_I2C_SCL_GPIO,
        .sda_io_num = WIT_I2C_SDA_GPIO,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    
    esp_err_t ret = i2c_new_master_bus(&i2c_bus_config, &s_i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "I2C bus initialized (SCL=%d, SDA=%d)", WIT_I2C_SCL_GPIO, WIT_I2C_SDA_GPIO);
    return ESP_OK;
}

// ============ 添加I2C设备 ============
static esp_err_t wit_i2c_add_device(uint8_t addr) {
    if (s_i2c_bus == NULL) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_FAIL;
    }
    
    // 如果已有设备，先删除
    if (s_i2c_dev != NULL) {
        i2c_master_bus_rm_device(s_i2c_dev);
        s_i2c_dev = NULL;
    }
    
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = WIT_I2C_FREQ_HZ,
    };
    
    esp_err_t ret = i2c_master_bus_add_device(s_i2c_bus, &dev_config, &s_i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device at 0x%02X: %s", addr, esp_err_to_name(ret));
        return ret;
    }
    
    s_ucDevAddr = addr;
    ESP_LOGI(TAG, "I2C device added at address 0x%02X", addr);
    return ESP_OK;
}

// ============ 自动扫描传感器 ============
int32_t wit_imu_scan(uint8_t *found_addr) {
    if (s_i2c_bus == NULL) {
        if (wit_i2c_init() != ESP_OK) {
            return -1;
        }
    }
    
    ESP_LOGI(TAG, "Scanning for WIT IMU sensor...");
    
    // 扫描地址范围 0x50-0x53
    for (uint8_t addr = 0x50; addr <= 0x53; addr++) {
        if (wit_i2c_add_device(addr) == ESP_OK) {
            // 尝试读取设备
            uint8_t data[2];
            if (WitIICRead(addr << 1, AX << 1, data, 2) == 0) {
                ESP_LOGI(TAG, "Found WIT IMU at address 0x%02X", addr);
                if (found_addr) *found_addr = addr;
                return 0;
            }
            i2c_master_bus_rm_device(s_i2c_dev);
            s_i2c_dev = NULL;
        }
    }
    
    ESP_LOGE(TAG, "No WIT IMU sensor found");
    return -1;
}

// ============ 初始化 ============
int32_t wit_imu_init(void) {
    esp_err_t ret;
    
    if (s_bInit) {
        ESP_LOGW(TAG, "WIT IMU already initialized");
        return 0;
    }
    
    // 初始化I2C总线
    if (s_i2c_bus == NULL) {
        ret = wit_i2c_init();
        if (ret != ESP_OK) {
            return -1;
        }
    }
    
    // 添加默认设备
    ret = wit_i2c_add_device(WIT_DEFAULT_ADDR);
    if (ret != ESP_OK) {
        // 尝试扫描
        uint8_t found_addr = 0;
        if (wit_imu_scan(&found_addr) != 0) {
            return -1;
        }
        ret = wit_i2c_add_device(found_addr);
        if (ret != ESP_OK) {
            return -1;
        }
    }
    
    // 初始化WIT SDK
    WitInit(WIT_PROTOCOL_I2C, s_ucDevAddr);
    WitI2cFuncRegister(WitIICWrite, WitIICRead);
    WitDelayMsRegister(Delayms);
    WitRegisterCallBack(SensorDataUpdate);
    
    ESP_LOGI(TAG, "WIT IMU initialized successfully");
    s_bInit = true;
    
    return 0;
}

// ============ 反初始化 ============
void wit_imu_deinit(void) {
    if (!s_bInit) return;
    
    WitDeInit();
    
    if (s_i2c_dev != NULL) {
        i2c_master_bus_rm_device(s_i2c_dev);
        s_i2c_dev = NULL;
    }
    
    if (s_i2c_bus != NULL) {
        i2c_del_master_bus(s_i2c_bus);
        s_i2c_bus = NULL;
    }
    
    s_bInit = false;
    ESP_LOGI(TAG, "WIT IMU deinitialized");
}

// ============ 读取传感器数据 ============
int32_t wit_imu_update(void) {
    if (!s_bInit) {
        ESP_LOGE(TAG, "WIT IMU not initialized");
        return -1;
    }
    
    // 读取加速度、陀螺仪、姿态角 (从AX开始读取12个寄存器)
    if (WitReadReg(AX, 12) != WIT_HAL_OK) {
        ESP_LOGE(TAG, "Failed to read sensor data");
        return -1;
    }
    
    // 转换数据
    if (s_cDataUpdate & ACC_UPDATE) {
        for (int i = 0; i < 3; i++) {
            s_fAcc[i] = sReg[AX + i] / 32768.0f * 16.0f;  // ±16g
        }
        s_cDataUpdate &= ~ACC_UPDATE;
    }
    
    if (s_cDataUpdate & GYRO_UPDATE) {
        for (int i = 0; i < 3; i++) {
            s_fGyro[i] = sReg[GX + i] / 32768.0f * 2000.0f;  // ±2000°/s
        }
        s_cDataUpdate &= ~GYRO_UPDATE;
    }
    
    if (s_cDataUpdate & ANGLE_UPDATE) {
        for (int i = 0; i < 3; i++) {
            s_fAngle[i] = sReg[Roll + i] / 32768.0f * 180.0f;  // ±180°
        }
        s_cDataUpdate &= ~ANGLE_UPDATE;
    }
    
    return 0;
}

// ============ 获取加速度数据 ============
void wit_imu_get_acc(float *acc_x, float *acc_y, float *acc_z) {
    if (acc_x) *acc_x = s_fAcc[0];
    if (acc_y) *acc_y = s_fAcc[1];
    if (acc_z) *acc_z = s_fAcc[2];
}

// ============ 获取陀螺仪数据 ============
void wit_imu_get_gyro(float *gyro_x, float *gyro_y, float *gyro_z) {
    if (gyro_x) *gyro_x = s_fGyro[0];
    if (gyro_y) *gyro_y = s_fGyro[1];
    if (gyro_z) *gyro_z = s_fGyro[2];
}

// ============ 获取姿态角数据 ============
void wit_imu_get_angle(float *roll, float *pitch, float *yaw) {
    if (roll) *roll = s_fAngle[0];
    if (pitch) *pitch = s_fAngle[1];
    if (yaw) *yaw = s_fAngle[2];
}

// ============ 获取温度 ============
float wit_imu_get_temperature(void) {
    return sReg[TEMP] / 100.0f;  // 温度，单位：°C
}

// ============ 获取四元数 ============
void wit_imu_get_quaternion(float *pq0, float *pq1, float *pq2, float *pq3) {
    if (WitReadReg(Q0, 4) == WIT_HAL_OK) {
        Delayms(5);
        if (pq0) *pq0 = sReg[Q0] / 32768.0f;
        if (pq1) *pq1 = sReg[Q1] / 32768.0f;
        if (pq2) *pq2 = sReg[Q2] / 32768.0f;
        if (pq3) *pq3 = sReg[Q3] / 32768.0f;
    }
}

// ============ 校准接口 ============
int32_t wit_imu_acc_calibrate(void) {
    ESP_LOGI(TAG, "Starting accelerometer calibration...");
    ESP_LOGI(TAG, "Please keep sensor horizontal and still");
    
    if (WitStartAccCali() != WIT_HAL_OK) {
        ESP_LOGE(TAG, "Failed to start accelerometer calibration");
        return -1;
    }
    
    return 0;
}

int32_t wit_imu_acc_calibrate_stop(void) {
    if (WitStopAccCali() != WIT_HAL_OK) {
        ESP_LOGE(TAG, "Failed to stop accelerometer calibration");
        return -1;
    }
    
    ESP_LOGI(TAG, "Accelerometer calibration completed");
    return 0;
}

int32_t wit_imu_mag_calibrate(void) {
    ESP_LOGI(TAG, "Starting magnetometer calibration...");
    ESP_LOGI(TAG, "Please rotate sensor slowly in all directions");
    
    if (WitStartMagCali() != WIT_HAL_OK) {
        ESP_LOGE(TAG, "Failed to start magnetometer calibration");
        return -1;
    }
    
    return 0;
}

int32_t wit_imu_mag_calibrate_stop(void) {
    if (WitStopMagCali() != WIT_HAL_OK) {
        ESP_LOGE(TAG, "Failed to stop magnetometer calibration");
        return -1;
    }
    
    ESP_LOGI(TAG, "Magnetometer calibration completed");
    return 0;
}

// ============ 配置接口 ============
int32_t wit_imu_set_output_rate(int32_t rate) {
    if (WitSetOutputRate(rate) != WIT_HAL_OK) {
        ESP_LOGE(TAG, "Failed to set output rate");
        return -1;
    }
    
    ESP_LOGI(TAG, "Output rate set to %d", rate);
    return 0;
}

int32_t wit_imu_set_bandwidth(int32_t bandwidth) {
    if (WitSetBandwidth(bandwidth) != WIT_HAL_OK) {
        ESP_LOGE(TAG, "Failed to set bandwidth");
        return -1;
    }
    
    ESP_LOGI(TAG, "Bandwidth set to %d", bandwidth);
    return 0;
}

// ============ 打印传感器数据 ============
void wit_imu_print_data(void) {
    printf("Acc: %.3f, %.3f, %.3f (g)\n", s_fAcc[0], s_fAcc[1], s_fAcc[2]);
    printf("Gyro: %.3f, %.3f, %.3f (°/s)\n", s_fGyro[0], s_fGyro[1], s_fGyro[2]);
    printf("Angle: Roll=%.3f, Pitch=%.3f, Yaw=%.3f (°)\n", s_fAngle[0], s_fAngle[1], s_fAngle[2]);
}

// ============ 获取原始寄存器值 ============
int16_t wit_imu_get_raw_reg(uint8_t reg) {
    if (reg >= REGSIZE) return 0;
    return sReg[reg];
}

// ============ 检查初始化状态 ============
bool wit_imu_is_initialized(void) {
    return s_bInit;
}
