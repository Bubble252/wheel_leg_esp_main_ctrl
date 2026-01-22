/**
 * @file state_machine.c
 * @brief 机器人状态机实现
 * @author Bubble
 * @date 2026-01-15
 */

#include "state_machine.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include <string.h>

static const char *TAG = "STATE";

// 当前状态
static robot_state_t g_current_state = STATE_INIT;
static SemaphoreHandle_t g_state_mutex = NULL;
static state_change_callback_t g_callback = NULL;

// 状态名称
static const char *state_names[] = {
    [STATE_INIT] = "INIT",
    [STATE_IDLE] = "IDLE",
    [STATE_STANDING_UP] = "STANDING_UP",
    [STATE_BALANCING] = "BALANCING",
    [STATE_SITTING_DOWN] = "SITTING_DOWN",
    [STATE_ERROR] = "ERROR",
    [STATE_EMERGENCY_STOP] = "EMERGENCY_STOP",
};

// 状态转换表 [当前状态][目标状态] = 是否允许
static const bool transition_table[7][7] = {
    // INIT  IDLE  STAND BALANCE SIT  ERROR EMERG
    {false, true, false, false, false, true, true},  // FROM INIT
    {false, false, true, false, false, true, true},  // FROM IDLE
    {false, true, false, true, false, true, true},   // FROM STANDING_UP
    {false, false, false, false, true, true, true},  // FROM BALANCING
    {false, true, false, false, false, true, true},  // FROM SITTING_DOWN
    {false, true, false, false, false, false, true}, // FROM ERROR
    {false, true, false, false, false, false, false},// FROM EMERGENCY_STOP
};

esp_err_t state_machine_init(void) {
    g_state_mutex = xSemaphoreCreateMutex();
    if (g_state_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create mutex");
        return ESP_ERR_NO_MEM;
    }
    
    g_current_state = STATE_INIT;
    ESP_LOGI(TAG, "State machine initialized");
    
    return ESP_OK;
}

robot_state_t state_machine_get_state(void) {
    robot_state_t state;
    
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    state = g_current_state;
    xSemaphoreGive(g_state_mutex);
    
    return state;
}

esp_err_t state_machine_request(robot_state_t new_state) {
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    
    robot_state_t old_state = g_current_state;
    
    // 检查是否允许切换
    if (!transition_table[old_state][new_state]) {
        ESP_LOGW(TAG, "Transition %s -> %s not allowed",
                 state_names[old_state], state_names[new_state]);
        xSemaphoreGive(g_state_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    // 执行状态切换
    g_current_state = new_state;
    
    ESP_LOGI(TAG, "State: %s -> %s", state_names[old_state], state_names[new_state]);
    
    xSemaphoreGive(g_state_mutex);
    
    // 调用回调
    if (g_callback != NULL) {
        g_callback(old_state, new_state);
    }
    
    return ESP_OK;
}

void state_machine_emergency_stop(const char *reason) {
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    
    robot_state_t old_state = g_current_state;
    g_current_state = STATE_EMERGENCY_STOP;
    
    ESP_LOGE(TAG, "EMERGENCY STOP! Reason: %s", reason ? reason : "unknown");
    ESP_LOGE(TAG, "State: %s -> EMERGENCY_STOP", state_names[old_state]);
    
    xSemaphoreGive(g_state_mutex);
    
    // 调用回调
    if (g_callback != NULL) {
        g_callback(old_state, STATE_EMERGENCY_STOP);
    }
}

esp_err_t state_machine_clear_emergency(void) {
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    
    if (g_current_state != STATE_EMERGENCY_STOP) {
        xSemaphoreGive(g_state_mutex);
        return ESP_ERR_INVALID_STATE;
    }
    
    g_current_state = STATE_IDLE;
    ESP_LOGI(TAG, "Emergency cleared, state -> IDLE");
    
    xSemaphoreGive(g_state_mutex);
    
    if (g_callback != NULL) {
        g_callback(STATE_EMERGENCY_STOP, STATE_IDLE);
    }
    
    return ESP_OK;
}

void state_machine_register_callback(state_change_callback_t callback) {
    g_callback = callback;
}

bool state_machine_can_transition(robot_state_t target) {
    xSemaphoreTake(g_state_mutex, portMAX_DELAY);
    bool result = transition_table[g_current_state][target];
    xSemaphoreGive(g_state_mutex);
    return result;
}

const char *state_machine_get_state_name(robot_state_t state) {
    if (state >= 0 && state < sizeof(state_names) / sizeof(state_names[0])) {
        return state_names[state];
    }
    return "UNKNOWN";
}
