/**
 * @file motor_test.c
 * @brief 电机测试模块 - 通过串口命令测试电机
 * @author Bubble
 * @date 2026-01-15
 * 
 * 使用方法：
 * 1. 烧录后打开串口监视器
 * 2. 输入命令测试电机，例如：
 *    - help                    显示帮助
 *    - scan                    扫描在线电机
 *    - read 1                  读取电机1状态
 *    - speed 1 100             电机1速度100rpm
 *    - pos 1 90                电机1转到90度
 *    - torque 1 0.5            电机1力矩0.5
 *    - stop 1                  停止电机1
 *    - stop all                停止所有电机
 *    - enable 1                使能电机1闭环
 *    - idle 1                  电机1进入空闲
 */

#include "motor_test.h"
#include "imu_test.h"
#include "sensor_test.h"
#include "balance_test.h"
#include "can_motor.h"
#include "can_motor_stw_regs.h"
#include "wifi_remote.h"
#include "commander_parser.h"
#include "config.h"
#include "esp_log.h"
#include "esp_console.h"
#include "esp_vfs_dev.h"
#include "esp_vfs_usb_serial_jtag.h"
#include "driver/usb_serial_jtag.h"
#include "driver/usb_serial_jtag_vfs.h"
#include "esp_task_wdt.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <fcntl.h>

static const char *TAG = "MOTOR_TEST";

// ============================================================================
// 编译选项: 设为1时程序启动后自动执行 balance init + balance start
// ============================================================================
#define AUTO_BALANCE_START  1

// 电机句柄
static can_motor_handle_t g_motors[MOTOR_COUNT] = {NULL};
static TaskHandle_t g_rx_task = NULL;
static TaskHandle_t g_status_task = NULL;
static TaskHandle_t g_console_task = NULL;  // 串口命令任务
static bool g_running = false;
static bool g_show_status = false;
static bool g_console_initialized = false;  // USB Serial 初始化标志

// 前向声明
static void process_command(char *cmd);

// ============================================================================
// CAN 接收任务 - 绑定到 CPU1 避免影响 CPU0 的 IDLE 任务
// ============================================================================
static void can_rx_task(void *arg) {
    while (g_running) {
        can_motor_process_rx();
        vTaskDelay(pdMS_TO_TICKS(10));  // 10ms 间隔，足够让出 CPU
    }
    vTaskDelete(NULL);
}

// ============================================================================
// 串口命令任务 - 绑定到 CPU0，低优先级，不影响实时控制
// ============================================================================
static void console_task(void *arg) {
    char line[128];
    int idx = 0;
    uint8_t rx_buf[8];
    
    ESP_LOGI(TAG, "[console_task] Started on Core %d", xPortGetCoreID());
    
    printf("\nmotor> ");
    fflush(stdout);
    
    while (g_running) {
        // 从 USB Serial JTAG 读取 (非阻塞，超时20ms)
        int len = usb_serial_jtag_read_bytes(rx_buf, sizeof(rx_buf), pdMS_TO_TICKS(20));
        
        for (int i = 0; i < len; i++) {
            int c = rx_buf[i];
            if (c == '\n' || c == '\r') {
                printf("\n");
                line[idx] = '\0';
                if (idx > 0) {
                    process_command(line);
                }
                idx = 0;
                printf("motor> ");
                fflush(stdout);
            } else if (c == '\b' || c == 127) {  // Backspace
                if (idx > 0) {
                    idx--;
                    printf("\b \b");
                    fflush(stdout);
                }
            } else if (c >= 32 && c < 127 && idx < sizeof(line) - 1) {
                line[idx++] = c;
                printf("%c", c);  // 回显
                fflush(stdout);
            }
        }
        
        // 没有数据时让出 CPU
        if (len == 0) {
            vTaskDelay(pdMS_TO_TICKS(10));
        }
    }
    
    ESP_LOGI(TAG, "[console_task] Stopped");
    vTaskDelete(NULL);
}

// ============================================================================
// 状态显示任务
// ============================================================================
static void status_display_task(void *arg) {
    while (g_running) {
        if (g_show_status) {
            printf("\n--- Motor Status ---\n");
            for (int i = 0; i < MOTOR_COUNT; i++) {
                if (g_motors[i]) {
                    motor_state_t state;
                    can_motor_get_state(g_motors[i], &state);
                    printf("M%d: pos=%.1f° spd=%.1f rpm cur=%.2fA %s\n",
                           i + 1,
                           state.position,
                           state.speed,
                           state.current,
                           state.is_online ? "ONLINE" : "OFFLINE");
                }
            }
            printf("--------------------\n");
        }
        vTaskDelay(pdMS_TO_TICKS(500));
    }
    vTaskDelete(NULL);
}

// ============================================================================
// 命令处理函数
// ============================================================================

static void print_help(void) {
    printf("\n");
    printf("========== Motor Test Commands ==========\n");
    printf("  help                  - Show this help\n");
    printf("  scan                  - Scan online motors\n");
    printf("  read <id>             - Read motor status\n");
    printf("  read all              - Read all motors\n");
    printf("  speed <id> <rpm>      - Set speed (rpm)\n");
    printf("  pos <id> <deg>        - Set position (degrees)\n");
    printf("  torque <id> <val>     - Set torque\n");
    printf("  torque_loop <id> <val> [ms] - Continuous torque (default 2000ms)\n");
    printf("  torque_debug <id> [val]     - Full torque debug (idle->mode->enable->torque+CAN dump)\n");
    printf("  mode <id> <0-5>       - Set control mode\n");
    printf("      0=Torque, 1=Speed, 2=PosTrap, 3=PosFilter, 4=PosDirect, 5=LowSpeed\n");
    printf("  enable <id>           - Enable closed loop\n");
    printf("  enable all            - Enable all motors\n");
    printf("  idle <id>             - Set motor idle\n");
    printf("  stop <id>             - Stop motor (speed=0)\n");
    printf("  stop all              - Stop all motors\n");
    printf("  status on/off         - Toggle status display\n");
    printf("  can debug [id]        - CAN frame hex dump (TX+RX)\n");
    printf("  can off               - Stop CAN debug\n");
    printf("  test <id>             - Quick test motor\n");
    printf("  motor <id> origin     - Set current pos as zero\n");
    printf("  motor <id> save       - Save params to Flash\n");
    printf("  motor <id> reboot     - Reboot motor driver\n");
    printf("  motor <id> calib      - Calibrate motor (CAUTION!)\n");
    printf("  motor <id> erase      - Erase params (CAUTION!)\n");
    printf("  motor <id> error      - Read error code\n");
    printf("==========================================\n");
    printf("Motor IDs: 1-6\n");
    printf("  1=L_Hip, 2=L_Knee, 3=L_Wheel\n");
    printf("  4=R_Hip, 5=R_Knee, 6=R_Wheel\n");
    printf("\n");
    printf("========== IMU Commands =================\n");
    printf("  imu help              - Show IMU help\n");
    printf("  imu init              - Initialize IMU\n");
    printf("  imu start             - Start continuous read\n");
    printf("  imu stop              - Stop reading\n");
    printf("  imu print [on|off]    - Toggle data print\n");
    printf("  imu read              - Single read\n");
    printf("  imu rate <0-12>       - Set output rate\n");
    printf("  imu cali [acc|mag]    - Calibration\n");
    printf("==========================================\n");
    printf("\n");
    printf("========== WiFi Remote Commands =========\n");
    printf("  wifi [start]          - Start WiFi AP\n");
    printf("  wifi stop             - Stop WiFi AP\n");
    printf("  wifi status           - Show remote status\n");
    printf("==========================================\n\n");
    printf("========== Sensor Test Commands =========\n");
    printf("  btn init              - Initialize buttons\n");
    printf("  btn read              - Read button status\n");
    printf("  btn start/stop        - Start/stop monitoring\n");
    printf("  sht init              - Initialize SHT30\n");
    printf("  sht read              - Read temp & humidity\n");
    printf("  sht start/stop        - Start/stop monitoring\n");
    printf("  sht scan              - Scan I2C bus\n");
    printf("  power                 - Show power status\n");
    printf("  mpower on/off         - Motor power control\n");
    printf("==========================================\n\n");
    printf("========== Balance Test Commands ========\n");
    printf("  balance init          - Initialize balance module\n");
    printf("  balance start         - Start balance test (WiFi + tasks)\n");
    printf("  balance stop          - Stop balance test\n");
    printf("  balance enable        - Enable balance control\n");
    printf("  balance disable       - Disable balance control\n");
    printf("  balance estop         - Emergency stop\n");
    printf("  balance reset         - Reset from emergency\n");
    printf("  balance status        - Show balance status\n");
    printf("  balance zero <deg>    - Set angle zeropoint\n");
    printf("==========================================\n\n");
    printf("========== PID Tuner Commands ===========\n");
    printf("  Commander format: <ID><Param><Value>\n");
    printf("  Examples:\n");
    printf("    AP1.5    - Set Angle PID P=1.5\n");
    printf("    BD0.01   - Set Gyro PID D=0.01\n");
    printf("    DL8.0    - Set Speed PID Limit=8.0\n");
    printf("    GT0.05   - Set JoyY LPF Tf=0.05\n");
    printf("    MH1.0    - Set SpeedAdapt Kp_Max=1.0\n");
    printf("    A?       - Query Angle PID params\n");
    printf("  PID IDs: A=Angle B=Gyro C=Dist D=Speed\n");
    printf("           E=YawAngle F=YawGyro H=LqrU\n");
    printf("           I=Zeropoint K=RollAngle\n");
    printf("  LPF IDs: G=JoyY J=Zero L=Roll\n");
    printf("  Params: P/I/D/L(limit)/R(ramp)/T(lpf)\n");
    printf("  tune status   - Print all parameters\n");
    printf("==========================================\n\n");
    printf("========== STW Motor Commands ============\n");
    printf("  stw info <id>              - Read motor info (poles/Kt/gear)\n");
    printf("  stw maxspeed <id> <rpm>    - Set position max speed\n");
    printf("  stw maxcur <id> <A>        - Set max Q-axis current\n");
    printf("  stw slope <id> <A/s>       - Set torque current slope\n");
    printf("  stw accel <id> <RPM/s>     - Set speed mode accel\n");
    printf("  stw pid <id> read          - Read all 4 PID params\n");
    printf("  stw pid <id> <type> <val>  - Write PID param\n");
    printf("      type: pos_kp pos_ki spd_kp spd_ki\n");
    printf("  stw mit config <id> <pmax> <vmax> <tmax>\n");
    printf("                             - Set MIT limits\n");
    printf("  stw mit state <id>         - Read MIT state\n");
    printf("  stw mit ctrl <id> <pos> <vel> <kp> <kd> <torque>\n");
    printf("                             - MIT impedance control\n");
    printf("==========================================\n\n");
}

static void cmd_scan(void) {
    printf("Scanning motors...\n");
    
    // 依次请求每个电机状态
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (g_motors[i]) {
            // 只请求电压来检测在线状态
            can_motor_request_status(g_motors[i]);
            // 等待并处理接收
            for (int j = 0; j < 5; j++) {
                vTaskDelay(pdMS_TO_TICKS(10));
                can_motor_process_rx();
            }
        }
    }
    
    // 最后再处理一次
    vTaskDelay(pdMS_TO_TICKS(50));
    can_motor_process_rx();
    
    printf("Online motors: ");
    int count = 0;
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (g_motors[i] && can_motor_is_online(g_motors[i], 500)) {
            printf("%d ", i + 1);
            count++;
        }
    }
    if (count == 0) {
        printf("NONE");
    }
    printf("\n");
}

static void cmd_read(int id) {
    if (id < 1 || id > MOTOR_COUNT) {
        printf("Invalid motor ID: %d (1-%d)\n", id, MOTOR_COUNT);
        return;
    }
    
    can_motor_handle_t motor = g_motors[id - 1];
    if (!motor) {
        printf("Motor %d not initialized\n", id);
        return;
    }
    
    // 发送状态请求
    can_motor_request_status(motor);
    
    // 等待并处理接收数据
    for (int i = 0; i < 10; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
        can_motor_process_rx();
    }
    
    motor_state_t state;
    can_motor_get_state(motor, &state);
    
    printf("\n=== Motor %d Status ===\n", id);
    printf("  Position:     %.2f °\n", state.position);
    printf("  Speed:        %.2f rpm\n", state.speed);
    printf("  Current:      %.3f A\n", state.current);
    printf("  Voltage:      %.2f V\n", state.voltage);
    printf("  Motor Temp:   %.1f °C\n", state.motor_temp);
    printf("  Driver Temp:  %.1f °C\n", state.driver_temp);
    printf("  Error Code:   0x%04lX\n", state.error_code);
    printf("  Online:       %s\n", state.is_online ? "YES" : "NO");
    printf("=======================\n");
}

static void cmd_read_all(void) {
    printf("\n");
    
    // 依次请求每个电机的状态
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (g_motors[i]) {
            can_motor_request_status(g_motors[i]);
            // 等待并处理接收数据
            for (int j = 0; j < 5; j++) {
                vTaskDelay(pdMS_TO_TICKS(10));
                can_motor_process_rx();
            }
        }
    }
    
    // 最后再处理一次
    vTaskDelay(pdMS_TO_TICKS(50));
    can_motor_process_rx();
    
    printf("ID  | Position  | Speed     | Current | Voltage | Online\n");
    printf("----|-----------|-----------|---------|---------|-------\n");
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (g_motors[i]) {
            motor_state_t state;
            can_motor_get_state(g_motors[i], &state);
            printf(" %d  | %8.2f° | %7.1f rpm | %5.2f A | %5.1f V | %s\n",
                   i + 1,
                   state.position,
                   state.speed,
                   state.current,
                   state.voltage,
                   state.is_online ? "YES" : "NO");
        }
    }
    printf("\n");
}

static void cmd_speed(int id, float speed) {
    if (id < 1 || id > MOTOR_COUNT) {
        printf("Invalid motor ID: %d\n", id);
        return;
    }
    
    can_motor_handle_t motor = g_motors[id - 1];
    if (!motor) {
        printf("Motor %d not initialized\n", id);
        return;
    }
    
    esp_err_t ret = can_motor_set_speed(motor, speed);
    if (ret == ESP_OK) {
        printf("Motor %d: speed = %.1f rpm\n", id, speed);
    } else {
        printf("Failed to set speed: %s\n", esp_err_to_name(ret));
    }
}

static void cmd_position(int id, float pos) {
    if (id < 1 || id > MOTOR_COUNT) {
        printf("Invalid motor ID: %d\n", id);
        return;
    }
    
    can_motor_handle_t motor = g_motors[id - 1];
    if (!motor) {
        printf("Motor %d not initialized\n", id);
        return;
    }
    
    esp_err_t ret = can_motor_set_position(motor, pos, 100.0f);
    if (ret == ESP_OK) {
        printf("Motor %d: position = %.1f °\n", id, pos);
    } else {
        printf("Failed to set position: %s\n", esp_err_to_name(ret));
    }
}

static void cmd_torque(int id, float torque) {
    if (id < 1 || id > MOTOR_COUNT) {
        printf("Invalid motor ID: %d\n", id);
        return;
    }
    
    can_motor_handle_t motor = g_motors[id - 1];
    if (!motor) {
        printf("Motor %d not initialized\n", id);
        return;
    }
    
    int16_t torque_raw = (int16_t)(torque * 100.0f);
    printf("Motor %d: torque = %.2f, raw = %d (0x%04X)\n", id, torque, torque_raw, (uint16_t)torque_raw);
    
    esp_err_t ret = can_motor_set_torque(motor, torque);
    if (ret == ESP_OK) {
        printf("  CAN send OK\n");
    } else {
        printf("  CAN send FAILED: %s\n", esp_err_to_name(ret));
    }
    
    // 读回电流确认电机是否响应
    vTaskDelay(pdMS_TO_TICKS(50));
    can_motor_request_status(motor);
    for (int i = 0; i < 5; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
        can_motor_process_rx();
    }
    motor_state_t state;
    can_motor_get_state(motor, &state);
    printf("  Readback -> Current: %.3f A, Speed: %.1f rpm\n", state.current, state.speed);
}

static void cmd_mode(int id, int mode) {
    if (id < 1 || id > MOTOR_COUNT) {
        printf("Invalid motor ID: %d\n", id);
        return;
    }
    
    if (mode < 0 || mode > 5) {
        printf("Invalid mode: %d (valid: 0-5)\n", mode);
        return;
    }
    
    can_motor_handle_t motor = g_motors[id - 1];
    if (!motor) {
        printf("Motor %d not initialized\n", id);
        return;
    }
    
    const char* mode_names[] = {
        "Torque", "Speed", "PosTrap", "PosFilter", "PosDirect", "LowSpeed"
    };
    
    esp_err_t ret = can_motor_set_mode(motor, (motor_mode_t)mode);
    if (ret == ESP_OK) {
        printf("Motor %d: mode = %d (%s)\n", id, mode, mode_names[mode]);
        printf("  NOTE: 建议顺序: mode -> enable -> torque/speed\n");
    } else {
        printf("Failed to set mode: %s\n", esp_err_to_name(ret));
    }
}

// 力矩模式连续发送测试 (排除通信超时问题)
static void cmd_torque_loop(int id, float torque, int duration_ms) {
    if (id < 1 || id > MOTOR_COUNT) {
        printf("Invalid motor ID: %d\n", id);
        return;
    }
    
    can_motor_handle_t motor = g_motors[id - 1];
    if (!motor) {
        printf("Motor %d not initialized\n", id);
        return;
    }
    
    printf("Motor %d: sending torque=%.2f for %d ms (20Hz)...\n", id, torque, duration_ms);
    
    int count = duration_ms / 50;
    if (count < 1) count = 1;
    
    for (int i = 0; i < count; i++) {
        can_motor_set_torque(motor, torque);
        vTaskDelay(pdMS_TO_TICKS(50));
        can_motor_process_rx();
    }
    
    // 读状态
    can_motor_request_status(motor);
    for (int i = 0; i < 5; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
        can_motor_process_rx();
    }
    motor_state_t state;
    can_motor_get_state(motor, &state);
    printf("  After loop -> Current: %.3f A, Speed: %.1f rpm\n", state.current, state.speed);
    
    // 停止
    can_motor_set_torque(motor, 0);
    printf("  Torque loop done, set torque=0\n");
}

// 力矩模式全流程调试 (自动开启CAN debug, 执行 mode→enable→torque)
static void cmd_torque_debug(int id, float torque) {
    if (id < 1 || id > MOTOR_COUNT) {
        printf("Invalid motor ID: %d\n", id);
        return;
    }
    
    can_motor_handle_t motor = g_motors[id - 1];
    if (!motor) {
        printf("Motor %d not initialized\n", id);
        return;
    }
    
    printf("=== Torque Debug: Motor %d, torque=%.2f ===\n", id, torque);
    can_motor_set_debug(true, id);
    
    // Step 1: 先 idle 复位
    printf("\n[1] Set idle...\n");
    can_motor_set_idle(motor);
    vTaskDelay(pdMS_TO_TICKS(100));
    can_motor_process_rx();
    
    // Step 2: 设置模式
    printf("\n[2] Set mode=0 (Torque)...\n");
    can_motor_set_mode(motor, (motor_mode_t)0);
    vTaskDelay(pdMS_TO_TICKS(100));
    can_motor_process_rx();
    
    // Step 3: 进入闭环
    printf("\n[3] Enter closed loop...\n");
    can_motor_enter_closed_loop(motor);
    vTaskDelay(pdMS_TO_TICKS(100));
    can_motor_process_rx();
    
    // Step 4: 发送力矩
    printf("\n[4] Set torque=%.2f...\n", torque);
    can_motor_set_torque(motor, torque);
    vTaskDelay(pdMS_TO_TICKS(100));
    can_motor_process_rx();
    
    // Step 5: 连续发送力矩 1秒
    printf("\n[5] Continuous torque for 1s (20Hz)...\n");
    for (int i = 0; i < 20; i++) {
        can_motor_set_torque(motor, torque);
        vTaskDelay(pdMS_TO_TICKS(50));
        can_motor_process_rx();
    }
    
    // Step 6: 读取状态
    printf("\n[6] Read status...\n");
    can_motor_request_status(motor);
    for (int i = 0; i < 5; i++) {
        vTaskDelay(pdMS_TO_TICKS(10));
        can_motor_process_rx();
    }
    motor_state_t state;
    can_motor_get_state(motor, &state);
    printf("  Current: %.3f A, Speed: %.1f rpm, Error: 0x%04lX\n",
           state.current, state.speed, state.error_code);
    
    // 清理
    can_motor_set_torque(motor, 0);
    can_motor_set_debug(false, 0);
    printf("\n=== Torque Debug Done ===\n");
}

static void cmd_enable(int id) {
    if (id < 1 || id > MOTOR_COUNT) {
        printf("Invalid motor ID: %d\n", id);
        return;
    }
    
    can_motor_handle_t motor = g_motors[id - 1];
    if (!motor) {
        printf("Motor %d not initialized\n", id);
        return;
    }
    
    esp_err_t ret = can_motor_enter_closed_loop(motor);
    if (ret == ESP_OK) {
        printf("Motor %d: closed loop enabled\n", id);
    } else {
        printf("Failed to enable: %s\n", esp_err_to_name(ret));
    }
}

static void cmd_idle(int id) {
    if (id < 1 || id > MOTOR_COUNT) {
        printf("Invalid motor ID: %d\n", id);
        return;
    }
    
    can_motor_handle_t motor = g_motors[id - 1];
    if (!motor) {
        printf("Motor %d not initialized\n", id);
        return;
    }
    
    esp_err_t ret = can_motor_set_idle(motor);
    if (ret == ESP_OK) {
        printf("Motor %d: set to idle\n", id);
    } else {
        printf("Failed: %s\n", esp_err_to_name(ret));
    }
}

static void cmd_stop(int id) {
    if (id < 1 || id > MOTOR_COUNT) {
        printf("Invalid motor ID: %d\n", id);
        return;
    }
    
    can_motor_handle_t motor = g_motors[id - 1];
    if (!motor) {
        printf("Motor %d not initialized\n", id);
        return;
    }
    
    can_motor_set_speed(motor, 0);
    printf("Motor %d: stopped\n", id);
}

static void cmd_stop_all(void) {
    printf("Stopping all motors...\n");
    
    // 如果平衡控制正在运行，先禁用平衡输出，否则控制循环会立即覆盖停止命令
    balance_test_state_t state = balance_test_get_state();
    if (state == BALANCE_TEST_RUNNING) {
        printf("  Balance control is running, disabling first...\n");
        balance_test_disable();
    }
    
    can_motor_all_set_idle();
    printf("All motors stopped\n");
}

static void cmd_enable_all(void) {
    printf("Enabling all motors...\n");
    can_motor_all_enter_closed_loop();
    printf("All motors enabled\n");
}

static void cmd_test(int id) {
    if (id < 1 || id > MOTOR_COUNT) {
        printf("Invalid motor ID: %d\n", id);
        return;
    }
    
    printf("Quick test motor %d...\n", id);
    
    // 1. 使能闭环
    printf("  1. Enable closed loop...\n");
    cmd_enable(id);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // 2. 读取当前位置
    printf("  2. Read current position...\n");
    cmd_read(id);
    
    // 3. 低速转动
    printf("  3. Rotate at 50 rpm for 2s...\n");
    cmd_speed(id, 50);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // 4. 反向
    printf("  4. Rotate at -50 rpm for 2s...\n");
    cmd_speed(id, -50);
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // 5. 停止
    printf("  5. Stop...\n");
    cmd_speed(id, 0);
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // 6. 读取最终位置
    printf("  6. Read final position...\n");
    cmd_read(id);
    
    printf("Test complete!\n");
}

// ============================================================================
// 命令解析
// ============================================================================
static void process_command(char *cmd) {
    char *token = strtok(cmd, " \t\n\r");
    if (!token) return;
    
    if (strcmp(token, "help") == 0 || strcmp(token, "?") == 0) {
        print_help();
    }
    else if (strcmp(token, "scan") == 0) {
        cmd_scan();
    }
    else if (strcmp(token, "read") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token) {
            if (strcmp(token, "all") == 0) {
                cmd_read_all();
            } else {
                cmd_read(atoi(token));
            }
        } else {
            printf("Usage: read <id> or read all\n");
        }
    }
    else if (strcmp(token, "speed") == 0) {
        char *id_str = strtok(NULL, " \t\n\r");
        char *val_str = strtok(NULL, " \t\n\r");
        if (id_str && val_str) {
            cmd_speed(atoi(id_str), atof(val_str));
        } else {
            printf("Usage: speed <id> <rpm>\n");
        }
    }
    else if (strcmp(token, "pos") == 0) {
        char *id_str = strtok(NULL, " \t\n\r");
        char *val_str = strtok(NULL, " \t\n\r");
        if (id_str && val_str) {
            cmd_position(atoi(id_str), atof(val_str));
        } else {
            printf("Usage: pos <id> <degrees>\n");
        }
    }
    else if (strcmp(token, "torque") == 0) {
        char *id_str = strtok(NULL, " \t\n\r");
        char *val_str = strtok(NULL, " \t\n\r");
        if (id_str && val_str) {
            cmd_torque(atoi(id_str), atof(val_str));
        } else {
            printf("Usage: torque <id> <value>\n");
        }
    }
    else if (strcmp(token, "torque_loop") == 0) {
        char *id_str = strtok(NULL, " \t\n\r");
        char *val_str = strtok(NULL, " \t\n\r");
        char *ms_str  = strtok(NULL, " \t\n\r");
        if (id_str && val_str) {
            int ms = ms_str ? atoi(ms_str) : 2000;
            cmd_torque_loop(atoi(id_str), atof(val_str), ms);
        } else {
            printf("Usage: torque_loop <id> <value> [duration_ms=2000]\n");
        }
    }
    else if (strcmp(token, "torque_debug") == 0) {
        char *id_str = strtok(NULL, " \t\n\r");
        char *val_str = strtok(NULL, " \t\n\r");
        if (id_str) {
            float t = val_str ? atof(val_str) : 0.3f;
            cmd_torque_debug(atoi(id_str), t);
        } else {
            printf("Usage: torque_debug <id> [torque=0.3]\n");
        }
    }
    else if (strcmp(token, "mode") == 0) {
        char *id_str = strtok(NULL, " \t\n\r");
        char *val_str = strtok(NULL, " \t\n\r");
        if (id_str && val_str) {
            cmd_mode(atoi(id_str), atoi(val_str));
        } else {
            printf("Usage: mode <id> <0-5>\n");
            printf("  0=Torque, 1=Speed, 2=PosTrap, 3=PosFilter, 4=PosDirect, 5=LowSpeed\n");
        }
    }
    else if (strcmp(token, "enable") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token) {
            if (strcmp(token, "all") == 0) {
                cmd_enable_all();
            } else {
                cmd_enable(atoi(token));
            }
        } else {
            printf("Usage: enable <id> or enable all\n");
        }
    }
    else if (strcmp(token, "idle") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token) {
            cmd_idle(atoi(token));
        } else {
            printf("Usage: idle <id>\n");
        }
    }
    else if (strcmp(token, "stop") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token) {
            if (strcmp(token, "all") == 0) {
                cmd_stop_all();
            } else {
                cmd_stop(atoi(token));
            }
        } else {
            printf("Usage: stop <id> or stop all\n");
        }
    }
    else if (strcmp(token, "status") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token) {
            if (strcmp(token, "on") == 0) {
                g_show_status = true;
                printf("Status display ON\n");
            } else if (strcmp(token, "off") == 0) {
                g_show_status = false;
                printf("Status display OFF\n");
            }
        } else {
            printf("Usage: status on/off\n");
        }
    }
    else if (strcmp(token, "can") == 0) {
        // can debug [motor_id] — 开启CAN帧调试
        // can off             — 关闭CAN帧调试
        token = strtok(NULL, " \t\n\r");
        if (token && strcmp(token, "debug") == 0) {
            char *id_str = strtok(NULL, " \t\n\r");
            uint32_t filter = id_str ? (uint32_t)atoi(id_str) : 0;
            can_motor_set_debug(true, filter);
        } else if (token && strcmp(token, "off") == 0) {
            can_motor_set_debug(false, 0);
        } else {
            printf("Usage: can debug [motor_id]  |  can off\n");
        }
    }
    else if (strcmp(token, "test") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (token) {
            cmd_test(atoi(token));
        } else {
            printf("Usage: test <id>\n");
        }
    }
    // ========== 电机系统命令 (设置零点/保存/重启等) ==========
    else if (strcmp(token, "motor") == 0) {
        char *id_str = strtok(NULL, " \t\n\r");
        char *action = strtok(NULL, " \t\n\r");
        if (id_str && action) {
            int id = atoi(id_str);
            if (id < 1 || id > MOTOR_COUNT) {
                printf("Invalid motor ID: %d (valid: 1-6)\n", id);
            } else {
                can_motor_handle_t motor = g_motors[id - 1];
                if (!motor) {
                    printf("Motor %d not initialized\n", id);
                } else if (strcmp(action, "origin") == 0 || strcmp(action, "zero") == 0) {
                    esp_err_t ret = can_motor_set_origin(motor);
                    if (ret == ESP_OK) {
                        printf("Motor %d: Zero point SET (current pos -> 0)\n", id);
                        printf("  NOTE: Use 'motor %d save' to save to Flash\n", id);
                    } else {
                        printf("Failed: %s\n", esp_err_to_name(ret));
                    }
                } else if (strcmp(action, "save") == 0) {
                    printf("Motor %d: Saving params to Flash...\n", id);
                    esp_err_t ret = can_motor_save(motor);
                    if (ret == ESP_OK) {
                        printf("Motor %d: Params SAVED to Flash\n", id);
                    } else {
                        printf("Failed: %s\n", esp_err_to_name(ret));
                    }
                } else if (strcmp(action, "reboot") == 0) {
                    printf("Motor %d: Rebooting...\n", id);
                    esp_err_t ret = can_motor_reboot(motor);
                    if (ret == ESP_OK) {
                        printf("Motor %d: Reboot command sent\n", id);
                    } else {
                        printf("Failed: %s\n", esp_err_to_name(ret));
                    }
                } else if (strcmp(action, "calib") == 0 || strcmp(action, "calibrate") == 0) {
                    printf("Motor %d: Starting calibration...\n", id);
                    printf("  WARNING: Motor will spin! Make sure it's safe!\n");
                    esp_err_t ret = can_motor_calibrate(motor);
                    if (ret == ESP_OK) {
                        printf("Motor %d: Calibration started\n", id);
                    } else {
                        printf("Failed: %s\n", esp_err_to_name(ret));
                    }
                } else if (strcmp(action, "erase") == 0) {
                    printf("Motor %d: Erasing params...\n", id);
                    printf("  WARNING: This will reset all motor parameters!\n");
                    esp_err_t ret = can_motor_erase(motor);
                    if (ret == ESP_OK) {
                        printf("Motor %d: Params ERASED\n", id);
                    } else {
                        printf("Failed: %s\n", esp_err_to_name(ret));
                    }
                } else if (strcmp(action, "error") == 0) {
                    uint32_t err_code = can_motor_read_error(motor);
                    printf("Motor %d error code: 0x%08lX\n", id, (unsigned long)err_code);
                    can_motor_print_error(err_code);
                } else {
                    printf("Unknown action: %s\n", action);
                    printf("Valid actions: origin, save, reboot, calib, erase, error\n");
                }
            }
        } else {
            printf("Usage: motor <id> <action>\n");
            printf("Actions: origin, save, reboot, calib, erase, error\n");
        }
    }
    // ========== IMU 命令路由 ==========
    else if (strcmp(token, "imu") == 0) {
        // 获取剩余命令字符串
        char *rest = strtok(NULL, "");
        if (rest) {
            imu_test_process_cmd(rest);
        } else {
            printf("IMU commands:\n");
            printf("  imu init/deinit/scan/start/stop\n");
            printf("  imu read/quat/temp/status/help\n");
            printf("  imu print [on|off]\n");
            printf("  imu rate <0-12> / bw <0-6>\n");
            printf("  imu cali [acc|mag|stop]\n");
        }
    }
    // ========== WiFi 遥控器命令 ==========
    else if (strcmp(token, "wifi") == 0) {
        token = strtok(NULL, " \t\n\r");
        if (!token || strcmp(token, "start") == 0) {
            printf("Starting WiFi remote...\n");
            printf("SSID: WL-PRO  Password: 12345678\n");
            printf("Connect and open http://192.168.4.1\n");
            wifi_remote_start();
        } else if (strcmp(token, "stop") == 0) {
            printf("Stopping WiFi remote...\n");
            wifi_remote_stop();
        } else if (strcmp(token, "status") == 0) {
            wifi_remote_print_status();
        } else {
            printf("Usage: wifi [start|stop|status]\n");
        }
    }
    // ========== 按键测试命令 ==========
    else if (strcmp(token, "btn") == 0) {
        char *rest = strtok(NULL, "");
        if (rest) {
            btn_process_cmd(rest);
        } else {
            printf("Usage: btn [init|deinit|read|start|stop]\n");
        }
    }
    // ========== SHT30 温湿度传感器命令 ==========
    else if (strcmp(token, "sht") == 0) {
        char *rest = strtok(NULL, "");
        if (rest) {
            sht_process_cmd(rest);
        } else {
            printf("Usage: sht [init|deinit|read|start|stop|status|heater|reset]\n");
        }
    }
    // ========== 电源检测命令 ==========
    else if (strcmp(token, "power") == 0) {
        power_process_cmd();
    }
    // ========== 电机供电控制命令 ==========
    else if (strcmp(token, "mpower") == 0) {
        char *rest = strtok(NULL, "");
        motor_power_process_cmd(rest);
    }
    // ========== 传感器帮助命令 ==========
    else if (strcmp(token, "sensor") == 0) {
        char *rest = strtok(NULL, "");
        if (rest) {
            sensor_test_process_cmd(rest);
        } else {
            sensor_test_print_help();
        }
    }
    // ========== 平衡控制测试命令 ==========
    else if (strcmp(token, "balance") == 0 || strcmp(token, "bal") == 0) {
        char *rest = strtok(NULL, "");
        if (rest) {
            balance_test_process_cmd(rest);
        } else {
            printf("Balance test commands:\n");
            printf("  balance init      - Initialize balance test module\n");
            printf("  balance start     - Start balance test (creates tasks, starts WiFi)\n");
            printf("  balance stop      - Stop balance test (deletes tasks)\n");
            printf("  balance enable    - Enable balance control\n");
            printf("  balance disable   - Disable balance control\n");
            printf("  balance estop     - Emergency stop\n");
            printf("  balance reset     - Reset from emergency\n");
            printf("  balance status    - Print current status\n");
            printf("  balance zero <deg> - Set/get angle zeropoint\n");
            printf("  balance plot [on|off] - Enable/disable waveform output\n");
            printf("  balance plot div <N> - Set plot divider (1-255, default 10=20Hz)\n");
        }
    }
    // ========== STW 电机专属命令 ==========
    else if (strcmp(token, "stw") == 0) {
        char *sub = strtok(NULL, " \t\n\r");
        if (!sub) {
            printf("Usage: stw <info|maxspeed|maxcur|slope|accel|pid|mit> ...\n");
        }
        // --- stw info <id> ---
        else if (strcmp(sub, "info") == 0) {
            char *id_str = strtok(NULL, " \t\n\r");
            if (!id_str) { printf("Usage: stw info <id>\n"); }
            else {
                int id = atoi(id_str);
                if (id < 1 || id > MOTOR_COUNT || !g_motors[id-1]) {
                    printf("Invalid motor %d\n", id);
                } else {
                    can_motor_handle_t m = g_motors[id-1];
                    printf("Motor %d: Requesting info...\n", id);
                    esp_err_t ret = can_motor_stw_request_motor_info(m);
                    if (ret == ESP_OK) {
                        vTaskDelay(pdMS_TO_TICKS(100));
                        stw_motor_info_t info;
                        if (can_motor_stw_get_motor_info(m, &info) == ESP_OK) {
                            printf("  Pole pairs : %u\n", info.pole_pairs);
                            printf("  Kt (Nm/A)  : %.4f\n", info.kt);
                            printf("  Gear ratio : %u\n", info.gear_ratio);
                        } else {
                            printf("  No info received yet\n");
                        }
                    } else {
                        printf("Request failed: %s\n", esp_err_to_name(ret));
                    }
                }
            }
        }
        // --- stw maxspeed <id> <val> ---
        else if (strcmp(sub, "maxspeed") == 0) {
            char *id_str = strtok(NULL, " \t\n\r");
            char *val_str = strtok(NULL, " \t\n\r");
            if (!id_str || !val_str) { printf("Usage: stw maxspeed <id> <rpm>\n"); }
            else {
                int id = atoi(id_str);
                if (id < 1 || id > MOTOR_COUNT || !g_motors[id-1]) {
                    printf("Invalid motor %d\n", id);
                } else {
                    esp_err_t ret = can_motor_stw_set_max_speed(g_motors[id-1], atof(val_str));
                    printf("Motor %d: set max speed = %.1f rpm -> %s\n", id, atof(val_str), esp_err_to_name(ret));
                }
            }
        }
        // --- stw maxcur <id> <val> ---
        else if (strcmp(sub, "maxcur") == 0) {
            char *id_str = strtok(NULL, " \t\n\r");
            char *val_str = strtok(NULL, " \t\n\r");
            if (!id_str || !val_str) { printf("Usage: stw maxcur <id> <A>\n"); }
            else {
                int id = atoi(id_str);
                if (id < 1 || id > MOTOR_COUNT || !g_motors[id-1]) {
                    printf("Invalid motor %d\n", id);
                } else {
                    esp_err_t ret = can_motor_stw_set_max_current(g_motors[id-1], atof(val_str));
                    printf("Motor %d: set max current = %.2f A -> %s\n", id, atof(val_str), esp_err_to_name(ret));
                }
            }
        }
        // --- stw slope <id> <val> ---
        else if (strcmp(sub, "slope") == 0) {
            char *id_str = strtok(NULL, " \t\n\r");
            char *val_str = strtok(NULL, " \t\n\r");
            if (!id_str || !val_str) { printf("Usage: stw slope <id> <A/s>\n"); }
            else {
                int id = atoi(id_str);
                if (id < 1 || id > MOTOR_COUNT || !g_motors[id-1]) {
                    printf("Invalid motor %d\n", id);
                } else {
                    esp_err_t ret = can_motor_stw_set_torque_slope(g_motors[id-1], atof(val_str));
                    printf("Motor %d: set torque slope = %.2f A/s -> %s\n", id, atof(val_str), esp_err_to_name(ret));
                }
            }
        }
        // --- stw accel <id> <val> ---
        else if (strcmp(sub, "accel") == 0) {
            char *id_str = strtok(NULL, " \t\n\r");
            char *val_str = strtok(NULL, " \t\n\r");
            if (!id_str || !val_str) { printf("Usage: stw accel <id> <RPM/s>\n"); }
            else {
                int id = atoi(id_str);
                if (id < 1 || id > MOTOR_COUNT || !g_motors[id-1]) {
                    printf("Invalid motor %d\n", id);
                } else {
                    esp_err_t ret = can_motor_stw_set_accel(g_motors[id-1], atof(val_str));
                    printf("Motor %d: set accel = %.1f RPM/s -> %s\n", id, atof(val_str), esp_err_to_name(ret));
                }
            }
        }
        // --- stw pid <id> read / stw pid <id> <type> <val> ---
        else if (strcmp(sub, "pid") == 0) {
            char *id_str  = strtok(NULL, " \t\n\r");
            char *action  = strtok(NULL, " \t\n\r");
            if (!id_str || !action) {
                printf("Usage: stw pid <id> read\n");
                printf("       stw pid <id> <pos_kp|pos_ki|spd_kp|spd_ki> <val>\n");
            } else {
                int id = atoi(id_str);
                if (id < 1 || id > MOTOR_COUNT || !g_motors[id-1]) {
                    printf("Invalid motor %d\n", id);
                } else {
                    can_motor_handle_t m = g_motors[id-1];
                    if (strcmp(action, "read") == 0) {
                        // 读取全部4个PID参数
                        static const uint8_t cmds[] = {STW_CMD_POS_KP, STW_CMD_POS_KI, STW_CMD_SPD_KP, STW_CMD_SPD_KI};
                        static const char *names[] = {"pos_kp", "pos_ki", "spd_kp", "spd_ki"};
                        printf("Motor %d PID params:\n", id);
                        for (int i = 0; i < 4; i++) {
                            can_motor_stw_request_pid(m, cmds[i]);
                            vTaskDelay(pdMS_TO_TICKS(50));
                            float val = 0;
                            if (can_motor_stw_get_pid(m, cmds[i], &val) == ESP_OK) {
                                printf("  %-6s = %.6f\n", names[i], val);
                            } else {
                                printf("  %-6s = (no reply)\n", names[i]);
                            }
                        }
                    } else {
                        // 写入单个PID参数
                        uint8_t cmd = 0;
                        if (strcmp(action, "pos_kp") == 0) cmd = STW_CMD_POS_KP;
                        else if (strcmp(action, "pos_ki") == 0) cmd = STW_CMD_POS_KI;
                        else if (strcmp(action, "spd_kp") == 0) cmd = STW_CMD_SPD_KP;
                        else if (strcmp(action, "spd_ki") == 0) cmd = STW_CMD_SPD_KI;
                        else {
                            printf("Unknown PID type: %s (use pos_kp/pos_ki/spd_kp/spd_ki)\n", action);
                            goto stw_done;
                        }
                        char *val_str = strtok(NULL, " \t\n\r");
                        if (!val_str) {
                            printf("Usage: stw pid %d %s <value>\n", id, action);
                        } else {
                            float val = atof(val_str);
                            esp_err_t ret = can_motor_stw_write_pid(m, cmd, val);
                            printf("Motor %d: set %s = %.6f -> %s\n", id, action, val, esp_err_to_name(ret));
                        }
                    }
                }
            }
        }
        // --- stw mit <subcmd> ---
        else if (strcmp(sub, "mit") == 0) {
            char *mit_sub = strtok(NULL, " \t\n\r");
            if (!mit_sub) {
                printf("Usage: stw mit config <id> <pmax> <vmax> <tmax>\n");
                printf("       stw mit state <id>\n");
                printf("       stw mit ctrl <id> <pos> <vel> <kp> <kd> <torque>\n");
            }
            else if (strcmp(mit_sub, "config") == 0) {
                char *id_str = strtok(NULL, " \t\n\r");
                char *p_str  = strtok(NULL, " \t\n\r");
                char *v_str  = strtok(NULL, " \t\n\r");
                char *t_str  = strtok(NULL, " \t\n\r");
                if (!id_str || !p_str || !v_str || !t_str) {
                    printf("Usage: stw mit config <id> <pos_max> <vel_max> <t_max>\n");
                } else {
                    int id = atoi(id_str);
                    if (id < 1 || id > MOTOR_COUNT || !g_motors[id-1]) {
                        printf("Invalid motor %d\n", id);
                    } else {
                        float pm = atof(p_str), vm = atof(v_str), tm = atof(t_str);
                        esp_err_t ret = can_motor_stw_mit_set_config(g_motors[id-1], pm, vm, tm);
                        printf("Motor %d: MIT config pos_max=%.2f vel_max=%.2f t_max=%.2f -> %s\n",
                               id, pm, vm, tm, esp_err_to_name(ret));
                    }
                }
            }
            else if (strcmp(mit_sub, "state") == 0) {
                char *id_str = strtok(NULL, " \t\n\r");
                if (!id_str) { printf("Usage: stw mit state <id>\n"); }
                else {
                    int id = atoi(id_str);
                    if (id < 1 || id > MOTOR_COUNT || !g_motors[id-1]) {
                        printf("Invalid motor %d\n", id);
                    } else {
                        can_motor_handle_t m = g_motors[id-1];
                        can_motor_stw_mit_request_state(m);
                        vTaskDelay(pdMS_TO_TICKS(100));
                        stw_mit_state_t st;
                        if (can_motor_stw_mit_get_state(m, &st) == ESP_OK) {
                            printf("Motor %d MIT state:\n", id);
                            printf("  Position : %.4f rad\n", st.position);
                            printf("  Velocity : %.4f rad/s\n", st.velocity);
                            printf("  Torque   : %.4f Nm\n", st.torque);
                            printf("  Status   : 0x%02X (MIT=%d Fault=%d)\n",
                                   st.status, st.status & 1, (st.status >> 1) & 1);
                        } else {
                            printf("  No MIT state received yet\n");
                        }
                    }
                }
            }
            else if (strcmp(mit_sub, "ctrl") == 0) {
                char *id_str = strtok(NULL, " \t\n\r");
                char *pos_s  = strtok(NULL, " \t\n\r");
                char *vel_s  = strtok(NULL, " \t\n\r");
                char *kp_s   = strtok(NULL, " \t\n\r");
                char *kd_s   = strtok(NULL, " \t\n\r");
                char *t_s    = strtok(NULL, " \t\n\r");
                if (!id_str || !pos_s || !vel_s || !kp_s || !kd_s || !t_s) {
                    printf("Usage: stw mit ctrl <id> <pos> <vel> <kp> <kd> <torque>\n");
                } else {
                    int id = atoi(id_str);
                    if (id < 1 || id > MOTOR_COUNT || !g_motors[id-1]) {
                        printf("Invalid motor %d\n", id);
                    } else {
                        float pos = atof(pos_s), vel = atof(vel_s);
                        float kp = atof(kp_s), kd = atof(kd_s), t = atof(t_s);
                        esp_err_t ret = can_motor_stw_mit_control(g_motors[id-1], pos, vel, kp, kd, t);
                        printf("Motor %d: MIT ctrl pos=%.3f vel=%.3f kp=%.1f kd=%.2f t=%.3f -> %s\n",
                               id, pos, vel, kp, kd, t, esp_err_to_name(ret));
                    }
                }
            }
            else {
                printf("Unknown MIT sub-command: %s\n", mit_sub);
            }
        }
        else {
            printf("Unknown STW sub-command: %s\n", sub);
        }
        stw_done: ;
    }
    // ========== PID 调参命令 (Commander 协议) ==========
    else if (strcmp(token, "tune") == 0) {
        char *rest = strtok(NULL, " \t\n\r");
        if (rest && strcmp(rest, "status") == 0) {
            commander_print_params();
        } else {
            printf("Tuning commands:\n");
            printf("  tune status    - Print all parameters\n");
            printf("  Or use Commander format directly:\n");
            printf("  <ID><Param><Value> e.g. AP1.5, BD0.01, MH1.0\n");
        }
    }
    // ========== Commander 协议直接解析 (用于PID调参面板) ==========
    else if (strlen(token) >= 2 && 
             ((token[0] >= 'A' && token[0] <= 'N') || token[0] == 'W') &&
             (token[1] == 'P' || token[1] == 'I' || token[1] == 'D' || 
              token[1] == 'L' || token[1] == 'R' || token[1] == 'T' ||
              token[1] == 'H' || token[1] == 'M' || token[1] == '?')) {
        // 看起来像 Commander 命令 (如 AP1.5, BD0.01, MH1.0, A?)
        // 注意: 使用 token 而不是 cmd，因为 strtok 已修改 cmd
        if (commander_process_line(token)) {
            // 命令处理成功，参数会通过回调同步到 LQR 控制器
        } else {
            printf("Invalid Commander command: %s\n", token);
        }
    }
    else {
        printf("Unknown command: %s (type 'help' for commands)\n", token);
    }
}

// ============================================================================
// 公共接口
// ============================================================================

/**
 * @brief 初始化串口控制台 (仅初始化一次)
 */
static esp_err_t init_console(void) {
    if (g_console_initialized) {
        return ESP_OK;  // 已初始化
    }
    
    // 初始化 USB Serial JTAG 用于串口输入
    usb_serial_jtag_driver_config_t usb_serial_config = {
        .rx_buffer_size = 256,
        .tx_buffer_size = 256,
    };
    esp_err_t ret = usb_serial_jtag_driver_install(&usb_serial_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "USB Serial JTAG install failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 将 stdin/stdout 重定向到 USB Serial JTAG
    usb_serial_jtag_vfs_use_driver();
    
    // 设置 stdin 为非阻塞模式
    fcntl(fileno(stdin), F_SETFL, O_NONBLOCK);
    
    g_console_initialized = true;
    ESP_LOGI(TAG, "USB Serial JTAG console initialized");
    
    return ESP_OK;
}

esp_err_t motor_test_start(void) {
    ESP_LOGI(TAG, "Starting motor test mode...");
    
    // 初始化控制台
    esp_err_t ret = init_console();
    if (ret != ESP_OK) {
        return ret;
    }
    
    // 初始化并打开电机电源 (IO8)
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << MOTOR_POWER_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
    gpio_set_level(MOTOR_POWER_PIN, 1);  // 打开电机电源
    ESP_LOGI(TAG, "Motor power enabled (GPIO%d = HIGH)", MOTOR_POWER_PIN);
    vTaskDelay(pdMS_TO_TICKS(100));  // 等待电源稳定
    
    // 初始化 CAN 总线
    ret = can_bus_init(CAN_TX_PIN, CAN_RX_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "CAN bus init failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    // 创建电机实例
    for (int i = 0; i < MOTOR_COUNT; i++) {
        g_motors[i] = can_motor_create(i + 1);
        if (g_motors[i] == NULL) {
            ESP_LOGE(TAG, "Failed to create motor %d", i + 1);
        }
    }
    
    g_running = true;
    
    // 启动 CAN 接收任务 - 绑定到 CPU1，优先级2
    xTaskCreatePinnedToCore(can_rx_task, "can_rx", 4096, NULL, 2, &g_rx_task, 1);
    
    // 启动状态显示任务 - 绑定到 CPU1，优先级2
    xTaskCreatePinnedToCore(status_display_task, "status", 2048, NULL, 2, &g_status_task, 1);
    
    // 启动串口命令任务 - 绑定到 CPU0，优先级5 (低于实时控制任务)
    xTaskCreatePinnedToCore(console_task, "console", 4096, NULL, 5, &g_console_task, 0);
    
    // 打印帮助
    print_help();
    
    ESP_LOGI(TAG, "Motor test started (non-blocking)");
    ESP_LOGI(TAG, "Tasks: can_rx(CPU1), status(CPU1), console(CPU0)");
    
#if AUTO_BALANCE_START
    // 自动初始化并启动平衡模块
    ESP_LOGI(TAG, "AUTO_BALANCE_START enabled, auto-running balance init + start...");
    vTaskDelay(pdMS_TO_TICKS(500));  // 等待 CAN 总线稳定
    
    ret = balance_test_init();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Auto balance_test_init: OK");
        ret = balance_test_start();
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "Auto balance_test_start: OK");
        } else {
            ESP_LOGE(TAG, "Auto balance_test_start: FAILED (%s)", esp_err_to_name(ret));
        }
    } else {
        ESP_LOGE(TAG, "Auto balance_test_init: FAILED (%s)", esp_err_to_name(ret));
    }
#endif
    
    return ESP_OK;
}

void motor_test_stop(void) {
    g_running = false;
    
    // 停止所有电机
    can_motor_all_set_idle();
    
    // 等待任务结束
    vTaskDelay(pdMS_TO_TICKS(200));
    
    // 任务会自行删除，清空句柄
    g_rx_task = NULL;
    g_status_task = NULL;
    g_console_task = NULL;
    
    // 销毁电机实例
    for (int i = 0; i < MOTOR_COUNT; i++) {
        if (g_motors[i]) {
            can_motor_destroy(g_motors[i]);
            g_motors[i] = NULL;
        }
    }
    
    // 关闭 CAN
    can_bus_deinit();
    
    // 关闭电机电源
    gpio_set_level(MOTOR_POWER_PIN, 0);
    ESP_LOGI(TAG, "Motor power disabled (GPIO%d = LOW)", MOTOR_POWER_PIN);
    
    ESP_LOGI(TAG, "Motor test stopped");
}
