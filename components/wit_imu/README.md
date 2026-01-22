# WIT Motion IMU SDK Component

移植自 [WIT_imu_idf](https://github.com/Bubble252/WIT_imu_idf)

## 功能特性

- 支持 WIT Motion 系列 IMU (维特智能)
- I2C 通信协议 (默认地址 0x50)
- 获取加速度、陀螺仪、姿态角数据
- 获取四元数数据
- 加速度计和磁力计校准
- 可配置输出速率和滤波带宽

## 硬件连接

| ESP32-S3 | IMU |
|----------|-----|
| GPIO 11  | SCL |
| GPIO 12  | SDA |
| 3.3V     | VCC |
| GND      | GND |

## 使用方法

### 方式一：通过 imu_driver 接口 (推荐)

```c
#include "imu_driver.h"

// 初始化
imu_init();

// 读取数据
imu_data_t data;
imu_read_data(&data);

printf("Roll: %.2f, Pitch: %.2f, Yaw: %.2f\n", 
       data.roll, data.pitch, data.yaw);
printf("Accel: %.2f, %.2f, %.2f (g)\n",
       data.accel_x, data.accel_y, data.accel_z);

// 校准
imu_calibrate();  // 加速度计校准
imu_mag_calibrate();  // 磁力计校准 (需要旋转)
vTaskDelay(pdMS_TO_TICKS(10000));
imu_mag_calibrate_stop();
```

### 方式二：直接使用 WIT SDK

```c
#include "wit_imu.h"

// 初始化
wit_imu_init();

// 更新数据
wit_imu_update();

// 获取数据
float roll, pitch, yaw;
wit_imu_get_angle(&roll, &pitch, &yaw);

float acc_x, acc_y, acc_z;
wit_imu_get_acc(&acc_x, &acc_y, &acc_z);

float gyro_x, gyro_y, gyro_z;
wit_imu_get_gyro(&gyro_x, &gyro_y, &gyro_z);

// 打印数据
wit_imu_print_data();

// 校准
wit_imu_acc_calibrate();
vTaskDelay(pdMS_TO_TICKS(5000));
wit_imu_acc_calibrate_stop();
```

## 数据单位

| 数据类型 | 单位 | 范围 |
|----------|------|------|
| 加速度 | g | ±16g |
| 角速度 | °/s | ±2000°/s |
| 姿态角 | ° | ±180° |
| 温度 | °C | - |
| 四元数 | - | -1~1 |

## 输出速率配置

```c
#include "wit_c_sdk.h"

// 可用的输出速率
// RRATE_02HZ, RRATE_05HZ, RRATE_1HZ, RRATE_2HZ
// RRATE_5HZ, RRATE_10HZ, RRATE_20HZ, RRATE_50HZ
// RRATE_100HZ, RRATE_125HZ, RRATE_200HZ

wit_imu_set_output_rate(RRATE_100HZ);
```

## 滤波带宽配置

```c
// 可用的带宽
// BANDWIDTH_256HZ, BANDWIDTH_184HZ, BANDWIDTH_94HZ
// BANDWIDTH_44HZ, BANDWIDTH_21HZ, BANDWIDTH_10HZ, BANDWIDTH_5HZ

wit_imu_set_bandwidth(BANDWIDTH_21HZ);
```

## 文件结构

```
components/wit_imu/
├── CMakeLists.txt
├── README.md
├── include/
│   ├── wit_c_sdk.h      # SDK 核心接口
│   ├── wit_imu.h        # 高层封装接口
│   └── wit_reg.h        # 寄存器定义
└── src/
    ├── wit_c_sdk.c      # SDK 实现
    └── wit_imu.c        # ESP-IDF I2C 驱动
```

## 参考资料

- [维特智能官网](https://www.wit-motion.com/)
- [原始 Arduino 仓库](https://github.com/Bubble252/WIT_imu_idf)
