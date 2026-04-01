/**
 * @file wifi_remote.c
 * @brief WiFi 遥控器模块实现
 * @author Bubble
 * @date 2026-01-16
 * @note 移植自 shibo_wheel_leg 项目
 */

#include "wifi_remote.h"
#include "web_page.h"

#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_mac.h"
#include "esp_netif.h"
#include "esp_http_server.h"
#include "esp_timer.h"  // 用于精确时间测量
#include "cJSON.h"
#include "nvs_flash.h"
#include "hal/brownout_ll.h"  // 用于禁用欠压检测

static const char *TAG = "WIFI_REMOTE";

// ============== 配置 ==============
#define AP_SSID         "WL-PRO"
#define AP_PASSWORD     "12345678"
#define AP_CHANNEL      1
#define AP_MAX_CONN     4

// ============== 全局变量 ==============
static remote_data_t g_remote_data = {
    .dir = DIR_STOP,
    .dir_last = DIR_STOP,
    .joy_x = 0,
    .joy_y = 0,
    .joy_x_last = 0,
    .joy_y_last = 0,
    .height = 38,
    .roll = 0,
    .linear = 0,
    .angular = 0,
    .go = false,
    .car_mode = false,
    .balance_enable = false,
    .estop = false,
    .control_mode = 4,         // 默认 TRIPLE_PID
    .pitch_comp = false,
    .leg_enable = false,
    .leg_angle = -90.0f,
    .leg_length = 0.09f,
    .detail_mode = false,
    .detail_sync = true,
    .detail_left_length = 0.09f,
    .detail_left_angle = -90.0f,
    .detail_left_speed = 0.0f,
    .detail_right_length = 0.09f,
    .detail_right_angle = -90.0f,
    .detail_right_speed = 0.0f,
    .joy_speed_gain = 0.003f,
    .joy_yaw_gain = 0.03f,
    .dist_enable = true,       // 默认开启位移环
    .yaw_enable = true,        // 默认开启 Yaw 控制
    .last_update_ms = 0,
    .msg_count = 0,
    .connected = false,
};

static httpd_handle_t g_server = NULL;
static bool g_initialized = false;

// ============== 方向名称 ==============
static const char* dir_names[] = {
    "forward", "back", "right", "left", "stop", "jump"
};

const char* wifi_remote_dir_to_str(direction_t dir) {
    if (dir >= 0 && dir <= DIR_JUMP) {
        return dir_names[dir];
    }
    return "unknown";
}

remote_data_t* wifi_remote_get_data(void) {
    return &g_remote_data;
}

// ============== JSON 解析 ==============
static void parse_json_command(const char *json_str) {
    cJSON *json = cJSON_Parse(json_str);
    if (json == NULL) {
        ESP_LOGW(TAG, "JSON parse error");
        return;
    }
    
    // 检查 mode
    cJSON *mode = cJSON_GetObjectItem(json, "mode");
    if (!mode || !cJSON_IsString(mode) || strcmp(mode->valuestring, "basic") != 0) {
        cJSON_Delete(json);
        return;
    }
    
    // 保存上次状态
    g_remote_data.dir_last = g_remote_data.dir;
    g_remote_data.joy_x_last = g_remote_data.joy_x;
    g_remote_data.joy_y_last = g_remote_data.joy_y;
    
    // 解析 dir
    cJSON *dir = cJSON_GetObjectItem(json, "dir");
    if (dir && cJSON_IsString(dir)) {
        if (strcmp(dir->valuestring, "forward") == 0) {
            g_remote_data.dir = DIR_FORWARD;
        } else if (strcmp(dir->valuestring, "back") == 0) {
            g_remote_data.dir = DIR_BACK;
        } else if (strcmp(dir->valuestring, "left") == 0) {
            g_remote_data.dir = DIR_LEFT;
        } else if (strcmp(dir->valuestring, "right") == 0) {
            g_remote_data.dir = DIR_RIGHT;
        } else if (strcmp(dir->valuestring, "jump") == 0) {
            g_remote_data.dir = DIR_JUMP;
        } else {
            g_remote_data.dir = DIR_STOP;
        }
    }
    
    // 解析摇杆
    cJSON *joy_x = cJSON_GetObjectItem(json, "joy_x");
    if (joy_x && cJSON_IsNumber(joy_x)) {
        g_remote_data.joy_x = (int16_t)joy_x->valueint;
    }
    
    cJSON *joy_y = cJSON_GetObjectItem(json, "joy_y");
    if (joy_y && cJSON_IsNumber(joy_y)) {
        g_remote_data.joy_y = (int16_t)joy_y->valueint;
    }
    
    // 解析滑块
    cJSON *height = cJSON_GetObjectItem(json, "height");
    if (height && cJSON_IsNumber(height)) {
        g_remote_data.height = (int16_t)height->valueint;
    }
    
    cJSON *roll = cJSON_GetObjectItem(json, "roll");
    if (roll && cJSON_IsNumber(roll)) {
        g_remote_data.roll = (int16_t)roll->valueint;
    }
    
    cJSON *linear = cJSON_GetObjectItem(json, "linear");
    if (linear && cJSON_IsNumber(linear)) {
        g_remote_data.linear = (int16_t)linear->valueint;
    }
    
    cJSON *angular = cJSON_GetObjectItem(json, "angular");
    if (angular && cJSON_IsNumber(angular)) {
        g_remote_data.angular = (int16_t)angular->valueint;
    }
    
    // 解析使能开关
    cJSON *stable = cJSON_GetObjectItem(json, "stable");
    if (stable && cJSON_IsNumber(stable)) {
        g_remote_data.go = (stable->valueint == 1);
    }
    
    // 解析小车模式开关
    cJSON *car_mode = cJSON_GetObjectItem(json, "car_mode");
    if (car_mode && cJSON_IsNumber(car_mode)) {
        g_remote_data.car_mode = (car_mode->valueint == 1);
    }
    
    // 解析扩展控制字段
    cJSON *balance_enable = cJSON_GetObjectItem(json, "balance_enable");
    if (balance_enable && cJSON_IsNumber(balance_enable)) {
        g_remote_data.balance_enable = (balance_enable->valueint == 1);
    }
    
    cJSON *estop = cJSON_GetObjectItem(json, "estop");
    if (estop && cJSON_IsNumber(estop)) {
        g_remote_data.estop = (estop->valueint == 1);
    }
    
    cJSON *ctrl_mode = cJSON_GetObjectItem(json, "control_mode");
    if (ctrl_mode && cJSON_IsNumber(ctrl_mode)) {
        g_remote_data.control_mode = (int8_t)ctrl_mode->valueint;
    }
    
    cJSON *pitch_comp = cJSON_GetObjectItem(json, "pitch_comp");
    if (pitch_comp && cJSON_IsNumber(pitch_comp)) {
        g_remote_data.pitch_comp = (pitch_comp->valueint == 1);
    }
    
    cJSON *leg_enable = cJSON_GetObjectItem(json, "leg_enable");
    if (leg_enable && cJSON_IsNumber(leg_enable)) {
        g_remote_data.leg_enable = (leg_enable->valueint == 1);
    }
    
    cJSON *leg_angle = cJSON_GetObjectItem(json, "leg_angle");
    if (leg_angle && cJSON_IsNumber(leg_angle)) {
        g_remote_data.leg_angle = (float)leg_angle->valuedouble;
    }
    
    cJSON *leg_length = cJSON_GetObjectItem(json, "leg_length");
    if (leg_length && cJSON_IsNumber(leg_length)) {
        g_remote_data.leg_length = (float)leg_length->valuedouble;
    }
    
    // 解析详细调控模式字段
    cJSON *detail_mode = cJSON_GetObjectItem(json, "detail_mode");
    if (detail_mode && cJSON_IsNumber(detail_mode)) {
        g_remote_data.detail_mode = (detail_mode->valueint == 1);
    }
    
    cJSON *detail_sync = cJSON_GetObjectItem(json, "detail_sync");
    if (detail_sync && cJSON_IsNumber(detail_sync)) {
        g_remote_data.detail_sync = (detail_sync->valueint == 1);
    }
    
    cJSON *dl_len = cJSON_GetObjectItem(json, "detail_left_length");
    if (dl_len && cJSON_IsNumber(dl_len)) {
        g_remote_data.detail_left_length = (float)dl_len->valuedouble;
    }
    
    cJSON *dl_ang = cJSON_GetObjectItem(json, "detail_left_angle");
    if (dl_ang && cJSON_IsNumber(dl_ang)) {
        g_remote_data.detail_left_angle = (float)dl_ang->valuedouble;
    }
    
    cJSON *dl_spd = cJSON_GetObjectItem(json, "detail_left_speed");
    if (dl_spd && cJSON_IsNumber(dl_spd)) {
        g_remote_data.detail_left_speed = (float)dl_spd->valuedouble;
    }
    
    cJSON *dr_len = cJSON_GetObjectItem(json, "detail_right_length");
    if (dr_len && cJSON_IsNumber(dr_len)) {
        g_remote_data.detail_right_length = (float)dr_len->valuedouble;
    }
    
    cJSON *dr_ang = cJSON_GetObjectItem(json, "detail_right_angle");
    if (dr_ang && cJSON_IsNumber(dr_ang)) {
        g_remote_data.detail_right_angle = (float)dr_ang->valuedouble;
    }
    
    cJSON *dr_spd = cJSON_GetObjectItem(json, "detail_right_speed");
    if (dr_spd && cJSON_IsNumber(dr_spd)) {
        g_remote_data.detail_right_speed = (float)dr_spd->valuedouble;
    }
    
    // 解析遥杆增益
    cJSON *joy_spd_gain = cJSON_GetObjectItem(json, "joy_speed_gain");
    if (joy_spd_gain && cJSON_IsNumber(joy_spd_gain)) {
        g_remote_data.joy_speed_gain = (float)joy_spd_gain->valuedouble;
    }
    
    cJSON *joy_yaw_gain = cJSON_GetObjectItem(json, "joy_yaw_gain");
    if (joy_yaw_gain && cJSON_IsNumber(joy_yaw_gain)) {
        g_remote_data.joy_yaw_gain = (float)joy_yaw_gain->valuedouble;
    }
    
    // 解析位移环开关
    cJSON *dist_enable = cJSON_GetObjectItem(json, "dist_enable");
    if (dist_enable && cJSON_IsNumber(dist_enable)) {
        g_remote_data.dist_enable = (dist_enable->valueint == 1);
    }
    
    // 解析 Yaw 闭环开关
    cJSON *yaw_enable = cJSON_GetObjectItem(json, "yaw_enable");
    if (yaw_enable && cJSON_IsNumber(yaw_enable)) {
        g_remote_data.yaw_enable = (yaw_enable->valueint == 1);
    }
    
    // 更新时间戳和计数
    g_remote_data.last_update_ms = xTaskGetTickCount() * portTICK_PERIOD_MS;
    g_remote_data.receive_time_us = esp_timer_get_time();  // 精确微秒时间戳
    g_remote_data.msg_count++;
    
    // 打印接收到的命令 (限流: 每秒最多打印一次)
    static uint32_t last_rx_log_ms = 0;
    uint32_t now_ms = g_remote_data.last_update_ms;
    if (now_ms - last_rx_log_ms >= 1000) {
        ESP_LOGI(TAG, "[%lu] RX: dir=%s joy=(%d,%d) height=%d roll=%d go=%d",
                 g_remote_data.msg_count,
                 wifi_remote_dir_to_str(g_remote_data.dir),
                 g_remote_data.joy_x, g_remote_data.joy_y,
                 g_remote_data.height, g_remote_data.roll,
                 g_remote_data.go);
        last_rx_log_ms = now_ms;
    }
    
    cJSON_Delete(json);
}

// ============== HTTP 处理器 ==============

// 根路径 - 返回网页
static esp_err_t root_handler(httpd_req_t *req) {
    ESP_LOGI(TAG, "Serving web page (size=%d bytes)", strlen(web_page_html));
    
    // 设置响应头
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Connection", "close");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    
    // 分块发送大页面
    const char *data = web_page_html;
    size_t remaining = strlen(web_page_html);
    const size_t chunk_size = 1024;  // 每次发送 1KB
    
    while (remaining > 0) {
        size_t send_size = (remaining > chunk_size) ? chunk_size : remaining;
        esp_err_t ret = httpd_resp_send_chunk(req, data, send_size);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to send chunk: %d", ret);
            httpd_resp_send_chunk(req, NULL, 0);  // 结束
            return ret;
        }
        data += send_size;
        remaining -= send_size;
    }
    
    // 发送结束标记
    httpd_resp_send_chunk(req, NULL, 0);
    ESP_LOGI(TAG, "Web page sent successfully");
    return ESP_OK;
}

// WebSocket 处理器
static esp_err_t ws_handler(httpd_req_t *req) {
    if (req->method == HTTP_GET) {
        ESP_LOGI(TAG, "WebSocket handshake from client");
        g_remote_data.connected = true;
        return ESP_OK;
    }
    
    // 接收 WebSocket 帧
    httpd_ws_frame_t ws_pkt;
    memset(&ws_pkt, 0, sizeof(httpd_ws_frame_t));
    ws_pkt.type = HTTPD_WS_TYPE_TEXT;
    
    // 第一次调用获取长度
    esp_err_t ret = httpd_ws_recv_frame(req, &ws_pkt, 0);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed: %d", ret);
        return ret;
    }
    
    if (ws_pkt.len == 0) {
        return ESP_OK;
    }
    
    // 分配缓冲区并接收数据
    uint8_t *buf = calloc(1, ws_pkt.len + 1);
    if (buf == NULL) {
        ESP_LOGE(TAG, "Failed to allocate memory");
        return ESP_ERR_NO_MEM;
    }
    
    ws_pkt.payload = buf;
    ret = httpd_ws_recv_frame(req, &ws_pkt, ws_pkt.len);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_ws_recv_frame failed: %d", ret);
        free(buf);
        return ret;
    }
    
    // 解析 JSON
    if (ws_pkt.type == HTTPD_WS_TYPE_TEXT) {
        parse_json_command((char *)buf);
    }
    
    free(buf);
    return ESP_OK;
}

// ============== WiFi 事件处理 ==============
static void wifi_event_handler(void *arg, esp_event_base_t event_base,
                               int32_t event_id, void *event_data) {
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_AP_START:
                ESP_LOGI(TAG, "WiFi AP started");
                break;
            case WIFI_EVENT_AP_STACONNECTED: {
                wifi_event_ap_staconnected_t *event = (wifi_event_ap_staconnected_t *)event_data;
                ESP_LOGI(TAG, "Station connected, MAC: " MACSTR ", AID: %d",
                         MAC2STR(event->mac), event->aid);
                break;
            }
            case WIFI_EVENT_AP_STADISCONNECTED: {
                wifi_event_ap_stadisconnected_t *event = (wifi_event_ap_stadisconnected_t *)event_data;
                ESP_LOGI(TAG, "Station disconnected, MAC: " MACSTR ", AID: %d",
                         MAC2STR(event->mac), event->aid);
                g_remote_data.connected = false;
                break;
            }
            default:
                break;
        }
    }
}

// ============== 初始化函数 ==============

esp_err_t wifi_remote_init(void) {
    if (g_initialized) {
        return ESP_OK;
    }
    
    ESP_LOGI(TAG, "Initializing WiFi remote...");
    
    // 初始化 NVS (WiFi 需要)
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    
    // 初始化网络接口
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();
    
    // 初始化 WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    
    // 注册事件处理器
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                         ESP_EVENT_ANY_ID,
                                                         &wifi_event_handler,
                                                         NULL, NULL));
    
    g_initialized = true;
    ESP_LOGI(TAG, "WiFi remote initialized");
    
    return ESP_OK;
}

esp_err_t wifi_remote_start(void) {
    if (!g_initialized) {
        esp_err_t ret = wifi_remote_init();
        if (ret != ESP_OK) {
            return ret;
        }
    }
    
    ESP_LOGI(TAG, "Starting WiFi AP...");
    
    // 临时禁用欠压检测，防止 WiFi 启动时重启
    // 正式产品请确保电源充足后移除此代码
    brownout_ll_intr_enable(false);
    ESP_LOGW(TAG, "Brownout detector disabled for WiFi startup");
    
    // 配置 AP
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = AP_SSID,
            .ssid_len = strlen(AP_SSID),
            .channel = AP_CHANNEL,
            .password = AP_PASSWORD,
            .max_connection = AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .required = false,
            },
        },
    };
    
    if (strlen(AP_PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }
    
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    // 设置 WiFi 发射功率
    // 注意：必须在 esp_wifi_start() 之后调用
    // 参数单位: 0.25 dBm，即 value=78 → 78×0.25=19.5dBm
    // 可选值: 8(2dBm), 20(5dBm), 28(7dBm), 34(8.5dBm), 44(11dBm), 52(13dBm), 60(15dBm), 68(17dBm), 76(19dBm), 78(19.5dBm), 84(21dBm)
    // 
    // 功率与 CAN 总线稳定性的权衡:
    //   20 (5dBm/3mW)   → CAN稳定但WiFi信号太弱，电机运行时手机丢失AP
    //   52 (13dBm/20mW) → 信号比5dBm强4倍，PA电流~200mA，CAN可接受
    //   78 (19.5dBm/89mW) → PA峰值~350mA，3.3V跌落导致CAN bus-off
    // 如果仍有CAN错误，降至 44(11dBm) 或 34(8.5dBm)
    esp_wifi_set_max_tx_power(20);  // 5dBm (~3mW)，优先保证CAN总线稳定
    
    int8_t power;
    esp_wifi_get_max_tx_power(&power);
    ESP_LOGI(TAG, "WiFi TX power: %d (0.25dBm units)", power);
    
    // 注意：测试阶段暂不重新启用欠压检测
    // 正式产品如果电源稳定，可以取消下面的注释
    // vTaskDelay(pdMS_TO_TICKS(1000));  // 等待1秒稳定
    // brownout_ll_intr_enable(true);
    // ESP_LOGI(TAG, "Brownout detector re-enabled");
    ESP_LOGW(TAG, "Brownout detector remains DISABLED (debug mode)");
    
    ESP_LOGI(TAG, "WiFi AP started. SSID: %s, Password: %s", AP_SSID, AP_PASSWORD);
    ESP_LOGI(TAG, "Connect to http://192.168.4.1");
    
    // 启动 HTTP 服务器
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 8192;
    config.lru_purge_enable = true;
    config.recv_wait_timeout = 10;      // 接收超时 10 秒
    config.send_wait_timeout = 10;      // 发送超时 10 秒
    config.max_resp_headers = 16;       // 增加响应头数量
    
    ESP_LOGI(TAG, "Starting HTTP server on port %d", config.server_port);
    
    if (httpd_start(&g_server, &config) == ESP_OK) {
        // 注册根路径
        httpd_uri_t root_uri = {
            .uri       = "/",
            .method    = HTTP_GET,
            .handler   = root_handler,
            .user_ctx  = NULL,
        };
        httpd_register_uri_handler(g_server, &root_uri);
        
        // 注册 WebSocket (ESP-IDF v5.x 方式)
        static const httpd_uri_t ws_uri = {
            .uri       = "/ws",
            .method    = HTTP_GET,
            .handler   = ws_handler,
            .user_ctx  = NULL,
            .is_websocket = true,
            .handle_ws_control_frames = false,
        };
        httpd_register_uri_handler(g_server, &ws_uri);
        
        ESP_LOGI(TAG, "HTTP server started successfully");
    } else {
        ESP_LOGE(TAG, "Failed to start HTTP server");
        return ESP_FAIL;
    }
    
    return ESP_OK;
}

void wifi_remote_stop(void) {
    if (g_server) {
        httpd_stop(g_server);
        g_server = NULL;
    }
    esp_wifi_stop();
    ESP_LOGI(TAG, "WiFi remote stopped");
}

bool wifi_remote_check_timeout(uint32_t timeout_ms) {
    uint32_t now = xTaskGetTickCount() * portTICK_PERIOD_MS;
    if (g_remote_data.last_update_ms == 0) {
        return true;  // 从未收到数据
    }
    return (now - g_remote_data.last_update_ms) > timeout_ms;
}

void wifi_remote_print_status(void) {
    ESP_LOGI(TAG, "=== WiFi Remote Status ===");
    ESP_LOGI(TAG, "Connected: %s", g_remote_data.connected ? "Yes" : "No");
    ESP_LOGI(TAG, "Messages: %lu", g_remote_data.msg_count);
    ESP_LOGI(TAG, "Go: %s", g_remote_data.go ? "ON" : "OFF");
    ESP_LOGI(TAG, "Dir: %s", wifi_remote_dir_to_str(g_remote_data.dir));
    ESP_LOGI(TAG, "Joystick: X=%d, Y=%d", g_remote_data.joy_x, g_remote_data.joy_y);
    ESP_LOGI(TAG, "Height: %d mm", g_remote_data.height);
    ESP_LOGI(TAG, "Roll: %d deg", g_remote_data.roll);
    ESP_LOGI(TAG, "==========================");
}
