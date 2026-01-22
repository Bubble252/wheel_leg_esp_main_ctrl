/**
 * @file sht30_driver.c
 * @brief SHT30 温湿度传感器驱动实现
 * @author Bubble
 * @date 2026-01-16
 */

#include "sht30_driver.h"
#include "config.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "SHT30";

// ============================================================================
// SHT30 命令定义
// ============================================================================

// 单次测量命令 (Clock Stretching Disabled)
#define SHT30_CMD_MEAS_HIGH_REP     0x2400  // 高精度
#define SHT30_CMD_MEAS_MED_REP      0x240B  // 中精度
#define SHT30_CMD_MEAS_LOW_REP      0x2416  // 低精度

// 单次测量命令 (Clock Stretching Enabled)
#define SHT30_CMD_MEAS_CS_HIGH      0x2C06  // 高精度 + Clock Stretching
#define SHT30_CMD_MEAS_CS_MED       0x2C0D  // 中精度 + Clock Stretching
#define SHT30_CMD_MEAS_CS_LOW       0x2C10  // 低精度 + Clock Stretching

// 周期测量命令 (mps = measurements per second)
#define SHT30_CMD_PERIODIC_05_HIGH  0x2032  // 0.5 mps, 高精度
#define SHT30_CMD_PERIODIC_05_MED   0x2024
#define SHT30_CMD_PERIODIC_05_LOW   0x202F
#define SHT30_CMD_PERIODIC_1_HIGH   0x2130  // 1 mps, 高精度
#define SHT30_CMD_PERIODIC_1_MED    0x2126
#define SHT30_CMD_PERIODIC_1_LOW    0x212D
#define SHT30_CMD_PERIODIC_2_HIGH   0x2236  // 2 mps, 高精度
#define SHT30_CMD_PERIODIC_2_MED    0x2220
#define SHT30_CMD_PERIODIC_2_LOW    0x222B
#define SHT30_CMD_PERIODIC_4_HIGH   0x2334  // 4 mps, 高精度
#define SHT30_CMD_PERIODIC_4_MED    0x2322
#define SHT30_CMD_PERIODIC_4_LOW    0x2329
#define SHT30_CMD_PERIODIC_10_HIGH  0x2737  // 10 mps, 高精度
#define SHT30_CMD_PERIODIC_10_MED   0x2721
#define SHT30_CMD_PERIODIC_10_LOW   0x272A

// 其他命令
#define SHT30_CMD_FETCH_DATA        0xE000  // 读取周期测量数据
#define SHT30_CMD_STOP_PERIODIC     0x3093  // 停止周期测量
#define SHT30_CMD_SOFT_RESET        0x30A2  // 软复位
#define SHT30_CMD_HEATER_ON         0x306D  // 开启加热器
#define SHT30_CMD_HEATER_OFF        0x3066  // 关闭加热器
#define SHT30_CMD_READ_STATUS       0xF32D  // 读取状态寄存器
#define SHT30_CMD_CLEAR_STATUS      0x3041  // 清除状态寄存器

// ============================================================================
// 私有变量
// ============================================================================

static i2c_master_bus_handle_t g_i2c_bus = NULL;
static i2c_master_dev_handle_t g_i2c_dev = NULL;
static uint8_t g_i2c_addr = 0x44;
static bool g_initialized = false;
static bool g_periodic_mode = false;
static sht30_data_t g_last_data = {0};

// ============================================================================
// CRC-8 校验 (多项式 0x31, 初值 0xFF)
// ============================================================================

static uint8_t crc8(const uint8_t *data, size_t len) {
    uint8_t crc = 0xFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (int j = 0; j < 8; j++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x31;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}

// ============================================================================
// I2C 通信函数
// ============================================================================

static esp_err_t sht30_write_cmd(uint16_t cmd) {
    if (!g_i2c_dev) {
        ESP_LOGE(TAG, "I2C device handle is NULL");
        return ESP_ERR_INVALID_STATE;
    }
    uint8_t buf[2] = {cmd >> 8, cmd & 0xFF};
    esp_err_t ret = i2c_master_transmit(g_i2c_dev, buf, 2, 100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C transmit cmd 0x%04X failed: %s", cmd, esp_err_to_name(ret));
    }
    return ret;
}

static esp_err_t sht30_read_bytes(uint8_t *data, size_t len) {
    if (!g_i2c_dev) {
        ESP_LOGE(TAG, "I2C device handle is NULL");
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t ret = i2c_master_receive(g_i2c_dev, data, len, 100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C receive %d bytes failed: %s", len, esp_err_to_name(ret));
    }
    return ret;
}

// ============================================================================
// 公共函数
// ============================================================================

esp_err_t sht30_init(int sda_pin, int scl_pin, uint8_t i2c_addr) {
    if (g_initialized) {
        ESP_LOGW(TAG, "Already initialized");
        return ESP_OK;
    }
    
    g_i2c_addr = i2c_addr;
    
    // 初始化 I2C 主机总线
    i2c_master_bus_config_t bus_config = {
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .i2c_port = I2C_NUM_0,
        .scl_io_num = scl_pin,
        .sda_io_num = sda_pin,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,  // 使能内部上拉
    };
    
    esp_err_t ret = i2c_new_master_bus(&bus_config, &g_i2c_bus);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 添加 SHT30 设备
    i2c_device_config_t dev_config = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = g_i2c_addr,
        .scl_speed_hz = I2C1_FREQ_HZ,
    };
    
    ret = i2c_master_bus_add_device(g_i2c_bus, &dev_config, &g_i2c_dev);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add I2C device: %s", esp_err_to_name(ret));
        i2c_del_master_bus(g_i2c_bus);
        g_i2c_bus = NULL;
        return ret;
    }
    
    g_initialized = true;
    
    // 软复位
    ret = sht30_soft_reset();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Soft reset failed (sensor may not be connected)");
    }
    
    // 等待传感器就绪
    vTaskDelay(pdMS_TO_TICKS(10));
    
    ESP_LOGI(TAG, "SHT30 initialized (I2C: SDA=%d, SCL=%d, Addr=0x%02X)", 
             sda_pin, scl_pin, g_i2c_addr);
    
    return ESP_OK;
}

void sht30_deinit(void) {
    if (!g_initialized) return;
    
    if (g_periodic_mode) {
        sht30_stop_periodic();
    }
    
    if (g_i2c_dev) {
        i2c_master_bus_rm_device(g_i2c_dev);
        g_i2c_dev = NULL;
    }
    
    if (g_i2c_bus) {
        i2c_del_master_bus(g_i2c_bus);
        g_i2c_bus = NULL;
    }
    
    g_initialized = false;
    ESP_LOGI(TAG, "SHT30 deinitialized");
}

esp_err_t sht30_soft_reset(void) {
    if (!g_initialized) return ESP_ERR_INVALID_STATE;
    
    esp_err_t ret = sht30_write_cmd(SHT30_CMD_SOFT_RESET);
    if (ret == ESP_OK) {
        vTaskDelay(pdMS_TO_TICKS(2));  // 等待复位完成
    }
    return ret;
}

esp_err_t sht30_read_single(sht30_data_t *data, sht30_repeatability_t repeatability) {
    if (!g_initialized) return ESP_ERR_INVALID_STATE;
    if (!data) return ESP_ERR_INVALID_ARG;
    if (!g_i2c_dev) {
        ESP_LOGE(TAG, "I2C device handle is NULL");
        return ESP_ERR_INVALID_STATE;
    }
    
    // 选择命令和等待时间
    uint16_t cmd;
    uint32_t wait_ms;
    
    switch (repeatability) {
        case SHT30_REPEATABILITY_HIGH:
            cmd = SHT30_CMD_MEAS_HIGH_REP;
            wait_ms = 16;
            break;
        case SHT30_REPEATABILITY_MEDIUM:
            cmd = SHT30_CMD_MEAS_MED_REP;
            wait_ms = 7;
            break;
        case SHT30_REPEATABILITY_LOW:
            cmd = SHT30_CMD_MEAS_LOW_REP;
            wait_ms = 5;
            break;
        default:
            return ESP_ERR_INVALID_ARG;
    }
    
    // 发送测量命令
    uint8_t cmd_buf[2] = {cmd >> 8, cmd & 0xFF};
    esp_err_t ret = i2c_master_transmit(g_i2c_dev, cmd_buf, 2, 100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Write cmd 0x%04X failed: %s", cmd, esp_err_to_name(ret));
        data->valid = false;
        return ret;
    }
    
    // 等待测量完成
    vTaskDelay(pdMS_TO_TICKS(wait_ms));
    
    // 读取数据 (6 字节: TempH, TempL, TempCRC, HumH, HumL, HumCRC)
    uint8_t buf[6];
    ret = i2c_master_receive(g_i2c_dev, buf, 6, 100);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Read 6 bytes failed: %s (check if sensor is connected)", esp_err_to_name(ret));
        data->valid = false;
        return ret;
    }
    
    // 校验 CRC
    if (crc8(&buf[0], 2) != buf[2] || crc8(&buf[3], 2) != buf[5]) {
        ESP_LOGW(TAG, "CRC error");
        data->valid = false;
        return ESP_ERR_INVALID_CRC;
    }
    
    // 转换温度 (公式: T = -45 + 175 * raw / 65535)
    uint16_t raw_temp = (buf[0] << 8) | buf[1];
    data->temperature = -45.0f + 175.0f * (float)raw_temp / 65535.0f;
    
    // 转换湿度 (公式: RH = 100 * raw / 65535)
    uint16_t raw_hum = (buf[3] << 8) | buf[4];
    data->humidity = 100.0f * (float)raw_hum / 65535.0f;
    
    // 限制湿度范围
    if (data->humidity > 100.0f) data->humidity = 100.0f;
    if (data->humidity < 0.0f) data->humidity = 0.0f;
    
    data->valid = true;
    data->timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    // 保存最后数据
    memcpy(&g_last_data, data, sizeof(sht30_data_t));
    
    return ESP_OK;
}

esp_err_t sht30_read(float *temperature, float *humidity) {
    sht30_data_t data;
    esp_err_t ret = sht30_read_single(&data, SHT30_REPEATABILITY_HIGH);
    
    if (ret == ESP_OK && data.valid) {
        if (temperature) *temperature = data.temperature;
        if (humidity) *humidity = data.humidity;
    }
    
    return ret;
}

esp_err_t sht30_start_periodic(float mps, sht30_repeatability_t repeatability) {
    if (!g_initialized) return ESP_ERR_INVALID_STATE;
    
    uint16_t cmd = 0;
    
    // 根据 mps 和精度选择命令
    if (mps <= 0.5f) {
        cmd = (repeatability == SHT30_REPEATABILITY_HIGH) ? SHT30_CMD_PERIODIC_05_HIGH :
              (repeatability == SHT30_REPEATABILITY_MEDIUM) ? SHT30_CMD_PERIODIC_05_MED :
              SHT30_CMD_PERIODIC_05_LOW;
    } else if (mps <= 1.0f) {
        cmd = (repeatability == SHT30_REPEATABILITY_HIGH) ? SHT30_CMD_PERIODIC_1_HIGH :
              (repeatability == SHT30_REPEATABILITY_MEDIUM) ? SHT30_CMD_PERIODIC_1_MED :
              SHT30_CMD_PERIODIC_1_LOW;
    } else if (mps <= 2.0f) {
        cmd = (repeatability == SHT30_REPEATABILITY_HIGH) ? SHT30_CMD_PERIODIC_2_HIGH :
              (repeatability == SHT30_REPEATABILITY_MEDIUM) ? SHT30_CMD_PERIODIC_2_MED :
              SHT30_CMD_PERIODIC_2_LOW;
    } else if (mps <= 4.0f) {
        cmd = (repeatability == SHT30_REPEATABILITY_HIGH) ? SHT30_CMD_PERIODIC_4_HIGH :
              (repeatability == SHT30_REPEATABILITY_MEDIUM) ? SHT30_CMD_PERIODIC_4_MED :
              SHT30_CMD_PERIODIC_4_LOW;
    } else {
        cmd = (repeatability == SHT30_REPEATABILITY_HIGH) ? SHT30_CMD_PERIODIC_10_HIGH :
              (repeatability == SHT30_REPEATABILITY_MEDIUM) ? SHT30_CMD_PERIODIC_10_MED :
              SHT30_CMD_PERIODIC_10_LOW;
    }
    
    esp_err_t ret = sht30_write_cmd(cmd);
    if (ret == ESP_OK) {
        g_periodic_mode = true;
    }
    
    return ret;
}

esp_err_t sht30_stop_periodic(void) {
    if (!g_initialized) return ESP_ERR_INVALID_STATE;
    
    esp_err_t ret = sht30_write_cmd(SHT30_CMD_STOP_PERIODIC);
    if (ret == ESP_OK) {
        g_periodic_mode = false;
    }
    
    return ret;
}

esp_err_t sht30_fetch_data(sht30_data_t *data) {
    if (!g_initialized) return ESP_ERR_INVALID_STATE;
    if (!data) return ESP_ERR_INVALID_ARG;
    if (!g_periodic_mode) return ESP_ERR_INVALID_STATE;
    
    // 发送读取命令
    esp_err_t ret = sht30_write_cmd(SHT30_CMD_FETCH_DATA);
    if (ret != ESP_OK) {
        data->valid = false;
        return ret;
    }
    
    // 读取数据
    uint8_t buf[6];
    ret = sht30_read_bytes(buf, 6);
    if (ret != ESP_OK) {
        data->valid = false;
        return ret;
    }
    
    // 校验 CRC
    if (crc8(&buf[0], 2) != buf[2] || crc8(&buf[3], 2) != buf[5]) {
        ESP_LOGW(TAG, "CRC error");
        data->valid = false;
        return ESP_ERR_INVALID_CRC;
    }
    
    // 转换数据
    uint16_t raw_temp = (buf[0] << 8) | buf[1];
    data->temperature = -45.0f + 175.0f * (float)raw_temp / 65535.0f;
    
    uint16_t raw_hum = (buf[3] << 8) | buf[4];
    data->humidity = 100.0f * (float)raw_hum / 65535.0f;
    
    if (data->humidity > 100.0f) data->humidity = 100.0f;
    if (data->humidity < 0.0f) data->humidity = 0.0f;
    
    data->valid = true;
    data->timestamp_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    
    memcpy(&g_last_data, data, sizeof(sht30_data_t));
    
    return ESP_OK;
}

esp_err_t sht30_read_status(uint16_t *status) {
    if (!g_initialized) return ESP_ERR_INVALID_STATE;
    if (!status) return ESP_ERR_INVALID_ARG;
    
    esp_err_t ret = sht30_write_cmd(SHT30_CMD_READ_STATUS);
    if (ret != ESP_OK) return ret;
    
    uint8_t buf[3];
    ret = sht30_read_bytes(buf, 3);
    if (ret != ESP_OK) return ret;
    
    // 校验 CRC
    if (crc8(&buf[0], 2) != buf[2]) {
        return ESP_ERR_INVALID_CRC;
    }
    
    *status = (buf[0] << 8) | buf[1];
    return ESP_OK;
}

esp_err_t sht30_clear_status(void) {
    if (!g_initialized) return ESP_ERR_INVALID_STATE;
    return sht30_write_cmd(SHT30_CMD_CLEAR_STATUS);
}

esp_err_t sht30_heater(bool enable) {
    if (!g_initialized) return ESP_ERR_INVALID_STATE;
    return sht30_write_cmd(enable ? SHT30_CMD_HEATER_ON : SHT30_CMD_HEATER_OFF);
}

bool sht30_is_online(void) {
    if (!g_initialized) return false;
    
    uint16_t status;
    return sht30_read_status(&status) == ESP_OK;
}

void sht30_get_last_data(sht30_data_t *data) {
    if (data) {
        memcpy(data, &g_last_data, sizeof(sht30_data_t));
    }
}

esp_err_t sht30_scan_i2c(void) {
    if (!g_i2c_bus) {
        ESP_LOGE(TAG, "I2C bus not initialized");
        return ESP_ERR_INVALID_STATE;
    }
    
    printf("Scanning I2C bus for devices...\n");
    printf("     0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");
    
    int found = 0;
    for (int i = 0; i < 128; i += 16) {
        printf("%02X:", i);
        for (int j = 0; j < 16; j++) {
            uint8_t addr = i + j;
            if (addr < 0x08 || addr > 0x77) {
                printf("   ");
                continue;
            }
            
            // 尝试添加设备并发送空数据
            i2c_device_config_t dev_config = {
                .dev_addr_length = I2C_ADDR_BIT_LEN_7,
                .device_address = addr,
                .scl_speed_hz = 100000,
            };
            
            i2c_master_dev_handle_t dev;
            esp_err_t ret = i2c_master_bus_add_device(g_i2c_bus, &dev_config, &dev);
            if (ret != ESP_OK) {
                printf(" --");
                continue;
            }
            
            // 尝试发送空数据检测 ACK
            uint8_t dummy = 0;
            ret = i2c_master_transmit(dev, &dummy, 0, 50);
            i2c_master_bus_rm_device(dev);
            
            if (ret == ESP_OK) {
                printf(" %02X", addr);
                found++;
            } else {
                printf(" --");
            }
        }
        printf("\n");
    }
    
    printf("\nFound %d device(s)\n", found);
    if (found == 0) {
        printf("No I2C devices found. Check wiring:\n");
        printf("  SDA: GPIO%d\n", I2C1_SDA_PIN);
        printf("  SCL: GPIO%d\n", I2C1_SCL_PIN);
    }
    
    return ESP_OK;
}
