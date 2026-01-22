/**
 * @file robot_controller.c
 * @brief 机器人主控制器实现
 * @author Bubble
 * @date 2026-01-15
 */

#include "robot_controller.h"
#include "state_machine.h"
#include "can_motor.h"
#include "imu_driver.h"
#include "balance_algorithm.h"
#include "config.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "ROBOT_CTRL";

// 控制任务句柄
static TaskHandle_t g_control_task = NULL;

// 电机句柄
static can_motor_handle_t g_motors[MOTOR_COUNT] = {NULL};

// 遥控命令
static remote_cmd_t g_remote_cmd = {0};
static SemaphoreHandle_t g_cmd_mutex = NULL;

// 机器人数据
static robot_data_t g_robot_data = {0};
static SemaphoreHandle_t g_data_mutex = NULL;

// 控制循环任务
static void control_loop_task(void *arg);

esp_err_t robot_controller_init(void) {
    esp_err_t ret;
    
    // 创建互斥锁
    g_cmd_mutex = xSemaphoreCreateMutex();
    g_data_mutex = xSemaphoreCreateMutex();
    if (g_cmd_mutex == NULL || g_data_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutexes");
        return ESP_ERR_NO_MEM;
    }
    
    // 初始化状态机
    ret = state_machine_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init state machine");
        return ret;
    }
    
    // 初始化 CAN 总线
    ret = can_bus_init(CAN_TX_PIN, CAN_RX_PIN);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init CAN bus");
        return ret;
    }
    
    // 创建电机实例
    for (int i = 0; i < MOTOR_COUNT; i++) {
        g_motors[i] = can_motor_create(i + 1);
        if (g_motors[i] == NULL) {
            ESP_LOGE(TAG, "Failed to create motor %d", i + 1);
            return ESP_FAIL;
        }
    }
    
    // 初始化 IMU
    ret = imu_init();
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to init IMU, continuing anyway");
    }
    
    // 初始化平衡算法
    ret = balance_init(NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to init balance algorithm");
        return ret;
    }
    
    // 切换到 IDLE 状态
    state_machine_request(STATE_IDLE);
    
    ESP_LOGI(TAG, "Robot controller initialized");
    return ESP_OK;
}

esp_err_t robot_controller_start(void) {
    if (g_control_task != NULL) {
        ESP_LOGW(TAG, "Control task already running");
        return ESP_OK;
    }
    
    xTaskCreatePinnedToCore(
        control_loop_task,
        "control_loop",
        TASK_STACK_CONTROL,
        NULL,
        TASK_PRIORITY_CONTROL,
        &g_control_task,
        1  // 运行在 Core 1
    );
    
    if (g_control_task == NULL) {
        ESP_LOGE(TAG, "Failed to create control task");
        return ESP_FAIL;
    }
    
    ESP_LOGI(TAG, "Robot controller started");
    return ESP_OK;
}

void robot_controller_stop(void) {
    if (g_control_task != NULL) {
        vTaskDelete(g_control_task);
        g_control_task = NULL;
    }
    
    // 停止所有电机
    can_motor_all_set_idle();
    
    ESP_LOGI(TAG, "Robot controller stopped");
}

void robot_controller_set_command(const remote_cmd_t *cmd) {
    if (cmd == NULL) return;
    
    xSemaphoreTake(g_cmd_mutex, portMAX_DELAY);
    memcpy(&g_remote_cmd, cmd, sizeof(remote_cmd_t));
    xSemaphoreGive(g_cmd_mutex);
}

esp_err_t robot_controller_get_data(robot_data_t *data) {
    if (data == NULL) return ESP_ERR_INVALID_ARG;
    
    xSemaphoreTake(g_data_mutex, portMAX_DELAY);
    memcpy(data, &g_robot_data, sizeof(robot_data_t));
    xSemaphoreGive(g_data_mutex);
    
    return ESP_OK;
}

esp_err_t robot_controller_stand_up(void) {
    return state_machine_request(STATE_STANDING_UP);
}

esp_err_t robot_controller_sit_down(void) {
    return state_machine_request(STATE_SITTING_DOWN);
}

void robot_controller_emergency_stop(void) {
    state_machine_emergency_stop("User request");
    can_motor_all_set_idle();
}

// ============================================================================
// 控制循环
// ============================================================================

static void control_loop_task(void *arg) {
    TickType_t last_wake_time = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(1000 / CONTROL_LOOP_FREQ_HZ);
    const float dt = 1.0f / CONTROL_LOOP_FREQ_HZ;
    
    ESP_LOGI(TAG, "Control loop started at %d Hz", CONTROL_LOOP_FREQ_HZ);
    
    while (1) {
        robot_state_t state = state_machine_get_state();
        
        // 处理 CAN 接收
        can_motor_process_rx();
        
        // 读取 IMU
        imu_data_t imu_data;
        imu_read_data(&imu_data);
        
        // 读取遥控命令
        remote_cmd_t cmd;
        xSemaphoreTake(g_cmd_mutex, portMAX_DELAY);
        memcpy(&cmd, &g_remote_cmd, sizeof(remote_cmd_t));
        xSemaphoreGive(g_cmd_mutex);
        
        // 根据状态执行不同逻辑
        switch (state) {
            case STATE_IDLE:
                // 空闲状态，等待命令
                break;
                
            case STATE_STANDING_UP:
                // TODO: 站立过程
                // 完成后切换到 BALANCING
                can_motor_all_enter_closed_loop();
                state_machine_request(STATE_BALANCING);
                break;
                
            case STATE_BALANCING: {
                // 平衡控制
                balance_input_t input = {
                    .pitch = imu_data.pitch,
                    .pitch_rate = imu_data.gyro_y,
                    .roll = imu_data.roll,
                    .roll_rate = imu_data.gyro_x,
                    .yaw_rate = imu_data.gyro_z,
                    .target_speed = cmd.speed,
                    .target_yaw_rate = cmd.turn,
                    .target_height = cmd.height,
                };
                
                // 获取轮子速度
                input.left_wheel_speed = can_motor_read_speed(g_motors[MOTOR_ID_LEFT_WHEEL - 1]);
                input.right_wheel_speed = can_motor_read_speed(g_motors[MOTOR_ID_RIGHT_WHEEL - 1]);
                
                balance_output_t output;
                esp_err_t ret = balance_compute(&input, &output, dt);
                
                if (ret == ESP_OK) {
                    // 发送轮子速度命令
                    can_motor_set_speed(g_motors[MOTOR_ID_LEFT_WHEEL - 1], output.left_wheel_speed);
                    can_motor_set_speed(g_motors[MOTOR_ID_RIGHT_WHEEL - 1], output.right_wheel_speed);
                } else {
                    // 紧急停止
                    state_machine_emergency_stop("Balance failed");
                    can_motor_all_set_idle();
                }
                break;
            }
                
            case STATE_SITTING_DOWN:
                // TODO: 坐下过程
                can_motor_all_set_idle();
                state_machine_request(STATE_IDLE);
                break;
                
            case STATE_ERROR:
            case STATE_EMERGENCY_STOP:
                // 确保电机停止
                can_motor_all_set_idle();
                break;
                
            default:
                break;
        }
        
        // 更新机器人数据
        xSemaphoreTake(g_data_mutex, portMAX_DELAY);
        memcpy(&g_robot_data.imu, &imu_data, sizeof(imu_data_t));
        for (int i = 0; i < MOTOR_COUNT; i++) {
            can_motor_get_state(g_motors[i], &g_robot_data.motors[i]);
        }
        g_robot_data.state = state;
        g_robot_data.timestamp = xTaskGetTickCount() * portTICK_PERIOD_MS;
        xSemaphoreGive(g_data_mutex);
        
        // 等待下一周期
        vTaskDelayUntil(&last_wake_time, period);
    }
}
