/**
 * @file robot_state.c
 * @brief 机器人状态计算实现 - 统一的传感器数据处理
 * @author Bubble
 * @date 2026-02-04
 * 
 * 功能:
 *   - 所有单位转换 (deg→rad, rpm→rad/s)
 *   - 正运动学计算 (FK)
 *   - 雅可比矩阵计算
 *   - 一阶导 (速度) 和二阶导 (加速度) 计算
 *   - 世界坐标系变换
 *   - Pitch 补偿 (考虑腿部角度)
 *   - YAW 过零处理
 *   - 位移累积
 */

#include "robot_state.h"
#include "leg_kinematics.h"
#include <string.h>
#include <math.h>
#include "esp_log.h"

static const char *TAG = "ROBOT_STATE";

// ============================================================================
// 内部状态 (用于累积量、微分和历史数据)
// ============================================================================
static struct {
    // YAW 过零处理
    float yaw_last;             // 上一次 YAW 角度 (rad)
    float yaw_total;            // YAW 累积角度 (rad)
    
    // 位移累积
    float lqr_distance;         // LQR 累积位移 (m)
    
    // IMU 历史数据 (用于数值微分计算角加速度)
    float pitch_rate_last;      // 上一次 pitch 角速度 (rad/s)
    float roll_rate_last;       // 上一次 roll 角速度 (rad/s)
    float pitch_acc_filtered;   // 滤波后的 pitch 角加速度 (rad/s²)
    float roll_acc_filtered;    // 滤波后的 roll 角加速度 (rad/s²)
    
    // 左腿历史数据 (用于数值微分计算加速度)
    float left_hip_vel_last;    // 上一次髋关节速度 (rad/s)
    float left_knee_vel_last;   // 上一次膝关节速度 (rad/s)
    float left_dL_last;         // 上一次腿长变化率 (m/s)
    float left_dalpha_last;     // 上一次身体角速度 (rad/s)
    float left_world_vy_last;   // 上一次世界坐标系垂直速度 (m/s)
    float left_dtheta_world_last; // 上一次 theta_world 角速度 (rad/s)
    
    // 右腿历史数据
    float right_hip_vel_last;
    float right_knee_vel_last;
    float right_dL_last;
    float right_dalpha_last;
    float right_world_vy_last;
    float right_dtheta_world_last;
    
    // 状态标志
    bool first_run;             // 首次运行标志
    bool initialized;           // 初始化标志
} s_state = {0};

// 角加速度低通滤波系数 (0~1, 越小滤波越强，延迟越大)
// 对于 5ms 周期，0.3 对应约 15ms 的时间常数
#define ACC_FILTER_ALPHA  0.1f

// ============================================================================
// 内部辅助函数
// ============================================================================

/**
 * @brief YAW 过零处理
 */
static float yaw_unwrap(float current_yaw_rad, float last_yaw_rad, float *total) {
    float delta = current_yaw_rad - last_yaw_rad;
    
    // 处理 ±π 跳变
    if (delta > M_PI) {
        delta -= 2.0f * M_PI;
    } else if (delta < -M_PI) {
        delta += 2.0f * M_PI;
    }
    
    *total += delta;
    return delta;
}

/**
 * @brief 数值微分 (计算加速度)
 * @param current 当前值
 * @param last 上一次值
 * @param dt 时间间隔
 * @return 导数
 */
static inline float numerical_diff(float current, float last, float dt) {
    if (dt > 1e-6f) {
        return (current - last) / dt;
    }
    return 0.0f;
}

/**
 * @brief 计算单腿状态 (FK + 雅可比 + 世界坐标系 + 加速度)
 * @param raw 原始传感器数据
 * @param is_left 是否为左腿
 * @param pitch_rad IMU pitch 角度 (rad)
 * @param pitch_rate_rad IMU pitch 角速度 (rad/s)
 * @param pitch_acc_rad IMU pitch 角加速度 (rad/s²)
 * @param dt 控制周期 (s)
 * @param state 输出状态
 */
static void compute_leg_state(const leg_raw_data_t *raw, 
                               bool is_left, 
                               float pitch_rad,
                               float pitch_rate_rad,
                               float pitch_acc_rad,
                               float dt,
                               leg_extended_state_t *state) {
    if (raw == NULL || state == NULL) return;
    
    // 清零
    memset(state, 0, sizeof(leg_extended_state_t));
    
    // 检查有效性
    if (!raw->hip_online || !raw->knee_online) {
        state->valid = false;
        return;
    }
    
    // === 1. 关节空间数据转换 ===
    // 保存原始角度 (度) - 用于 VMC 接口
    state->hip_pos_deg = raw->hip_pos_deg;
    state->knee_pos_deg = raw->knee_pos_deg;
    
    // 转换为弧度
    state->hip_pos = deg2rad(raw->hip_pos_deg);
    state->knee_pos = deg2rad(raw->knee_pos_deg);
    
    // 转换速度: rpm → rad/s
    state->hip_vel = rpm2rads(raw->hip_vel_rpm);
    state->knee_vel = rpm2rads(raw->knee_vel_rpm);
    
    // 获取历史数据指针
    float *hip_vel_last, *knee_vel_last, *dL_last, *dalpha_last, *world_vy_last, *dtheta_world_last;
    if (is_left) {
        hip_vel_last = &s_state.left_hip_vel_last;
        knee_vel_last = &s_state.left_knee_vel_last;
        dL_last = &s_state.left_dL_last;
        dalpha_last = &s_state.left_dalpha_last;
        world_vy_last = &s_state.left_world_vy_last;
        dtheta_world_last = &s_state.left_dtheta_world_last;
    } else {
        hip_vel_last = &s_state.right_hip_vel_last;
        knee_vel_last = &s_state.right_knee_vel_last;
        dL_last = &s_state.right_dL_last;
        dalpha_last = &s_state.right_dalpha_last;
        world_vy_last = &s_state.right_world_vy_last;
        dtheta_world_last = &s_state.right_dtheta_world_last;
    }
    
    // 关节加速度 (数值微分)
    if (!s_state.first_run) {
        state->hip_acc = numerical_diff(state->hip_vel, *hip_vel_last, dt);
        state->knee_acc = numerical_diff(state->knee_vel, *knee_vel_last, dt);
    }
    
    // === 2. 正运动学: 关节空间 → 工作空间 (机身坐标系) ===
    leg_joint_state_t joint = {
        .hip_angle = raw->hip_pos_deg,
        .knee_angle = raw->knee_pos_deg
    };
    
    leg_workspace_state_t workspace;
    if (leg_kin_forward(&joint, is_left, NULL, &workspace) != ESP_OK) {
        state->valid = false;
        return;
    }
    
    state->leg_length = workspace.leg_length;
    state->body_angle = deg2rad(workspace.body_angle);  // 转换为弧度
    
    // === 3. 雅可比矩阵 (机身坐标系) ===
    if (leg_kin_jacobian(&joint, is_left, NULL, state->J) != ESP_OK) {
        state->valid = false;
        return;
    }
    
    // 计算工作空间速度: [dL; dalpha] = J * [hip_vel; knee_vel]
    state->dL = state->J[0] * state->hip_vel + state->J[1] * state->knee_vel;
    state->dalpha = state->J[2] * state->hip_vel + state->J[3] * state->knee_vel;
    
    // 工作空间加速度 (数值微分)
    if (!s_state.first_run) {
        state->ddL = numerical_diff(state->dL, *dL_last, dt);
        state->ddalpha = numerical_diff(state->dalpha, *dalpha_last, dt);
    }
    
    // === 4. 腿相对世界竖直方向的角度及其导数 ===
    // theta_world = pitch + body_angle + π/2
    // 当 body_angle = -π/2（垂直向下）且 pitch = 0 时，theta_world = 0
    state->theta_world = pitch_rad + state->body_angle + (M_PI / 2.0f);
    
    // 一阶导: dtheta_world = pitch_rate + dalpha
    // 直接使用传感器数据和已计算的 dalpha
    state->dtheta_world = pitch_rate_rad + state->dalpha;
    
    // 二阶导: ddtheta_world = pitch_acc + ddalpha
    // 使用 IMU 提供的 pitch_acc 和数值微分得到的 ddalpha
    // 也可以通过数值微分 dtheta_world 来计算，这里两种方法都支持
    if (!s_state.first_run) {
        // 方法1: 直接使用已有的加速度数据
        state->ddtheta_world = pitch_acc_rad + state->ddalpha;
        
        // 方法2 (备用): 如果 pitch_acc 不可靠，可以用数值微分
        // float ddtheta_world_diff = numerical_diff(state->dtheta_world, *dtheta_world_last, dt);
        // state->ddtheta_world = ddtheta_world_diff;
    }
    
    // === 5. 世界坐标系位置和速度 ===
    // 使用世界坐标系雅可比
    if (leg_kin_jacobian_cartesian(&joint, is_left, NULL, pitch_rad, state->J_world) == ESP_OK) {
        // 世界坐标系位置
        float x, y;
        if (leg_kin_forward_cartesian(&joint, is_left, NULL, pitch_rad, &x, &y) == ESP_OK) {
            state->world_x = x;  // 向后为正
            state->world_y = y;  // 向上为正 (通常为负，因为轮子在下方)
        }
        
        // 世界坐标系速度: [vx; vy] = J_world * [hip_vel; knee_vel]
        state->world_vx = state->J_world[0] * state->hip_vel + state->J_world[1] * state->knee_vel;
        state->world_vy = state->J_world[2] * state->hip_vel + state->J_world[3] * state->knee_vel;
        
        // 世界坐标系垂直加速度 (数值微分)
        if (!s_state.first_run) {
            state->world_ay = numerical_diff(state->world_vy, *world_vy_last, dt);
        }
    }
    
    // === 6. 更新历史数据 ===
    *hip_vel_last = state->hip_vel;
    *knee_vel_last = state->knee_vel;
    *dL_last = state->dL;
    *dalpha_last = state->dalpha;
    *world_vy_last = state->world_vy;
    *dtheta_world_last = state->dtheta_world;
    
    state->valid = true;
}

// ============================================================================
// 公共 API 实现
// ============================================================================

esp_err_t robot_state_init(void) {
    memset(&s_state, 0, sizeof(s_state));
    s_state.first_run = true;
    s_state.initialized = true;
    
    ESP_LOGI(TAG, "Robot state module initialized");
    return ESP_OK;
}

esp_err_t robot_state_update(const sensor_raw_data_t *raw, 
                              robot_state_t *state,
                              bool enable_leg_comp) {
    if (raw == NULL || state == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    
    if (!s_state.initialized) {
        robot_state_init();
    }
    
    // 清零状态
    memset(state, 0, sizeof(robot_state_t));
    
    // 时间信息
    state->dt = raw->dt;
    state->timestamp_ms = raw->timestamp_ms;
    
    // =========================================================================
    // 1. IMU 单位转换 (deg → rad)
    // =========================================================================
    if (raw->imu.valid) {
        state->imu.pitch = deg2rad(raw->imu.pitch_deg);
        state->imu.pitch_rate = deg2rad(raw->imu.pitch_rate_dps);
        state->imu.roll = deg2rad(raw->imu.roll_deg);
        state->imu.roll_rate = deg2rad(raw->imu.roll_rate_dps);
        state->imu.yaw = deg2rad(raw->imu.yaw_deg);
        state->imu.yaw_rate = deg2rad(raw->imu.yaw_rate_dps);
        
        // IMU 角加速度计算 (数值微分 + 低通滤波)
        // 注意: IMU 只能测量角速度，角加速度必须通过微分计算
        // 数值微分会放大噪声，因此需要滤波
        if (!s_state.first_run && raw->dt > 1e-6f) {
            // 原始数值微分
            float pitch_acc_raw = numerical_diff(state->imu.pitch_rate, s_state.pitch_rate_last, raw->dt);
            float roll_acc_raw = numerical_diff(state->imu.roll_rate, s_state.roll_rate_last, raw->dt);
            
            // 一阶低通滤波: y = α*x + (1-α)*y_last
            s_state.pitch_acc_filtered = ACC_FILTER_ALPHA * pitch_acc_raw + (1.0f - ACC_FILTER_ALPHA) * s_state.pitch_acc_filtered;
            s_state.roll_acc_filtered = ACC_FILTER_ALPHA * roll_acc_raw + (1.0f - ACC_FILTER_ALPHA) * s_state.roll_acc_filtered;
            
            state->imu.pitch_acc = s_state.pitch_acc_filtered;
            state->imu.roll_acc = s_state.roll_acc_filtered;
        }
        // 更新历史数据
        s_state.pitch_rate_last = state->imu.pitch_rate;
        s_state.roll_rate_last = state->imu.roll_rate;
        
        // YAW 过零处理
        if (s_state.first_run) {
            s_state.yaw_last = state->imu.yaw;
        }
        yaw_unwrap(state->imu.yaw, s_state.yaw_last, &s_state.yaw_total);
        s_state.yaw_last = state->imu.yaw;
        state->imu.yaw_total = s_state.yaw_total;
        
        state->imu_valid = true;
    }
    
    // =========================================================================
    // 2. 轮电机单位转换 (deg → rad, rpm → rad/s)
    // =========================================================================
    state->wheel.left_pos = deg2rad(raw->wheel.left_pos_deg);
    state->wheel.left_vel = rpm2rads(raw->wheel.left_vel_rpm);
    state->wheel.right_pos = deg2rad(raw->wheel.right_pos_deg);
    state->wheel.right_vel = rpm2rads(raw->wheel.right_vel_rpm);
    state->wheel_valid = raw->wheel.left_online && raw->wheel.right_online;
    
    // =========================================================================
    // 3. 轮速 → 机器人速度 + 位移累积 (所有控制模式共用)
    // =========================================================================
    // 电机方向约定: 轮子正转时机器人后退
    // robot_vx: 向后为正 (与轮速同向)
    // lqr_speed: 前进为正 (与轮速反向)
    float avg_wheel_vel = (state->wheel.left_vel + state->wheel.right_vel) / 2.0f;
    state->kinematics.wheel_vel_avg = -avg_wheel_vel;  // 取反: 前进为正
    state->kinematics.robot_vx = avg_wheel_vel * WHEEL_RADIUS_M;  // 向后为正
    state->kinematics.lqr_speed = -state->kinematics.robot_vx;    // 前进为正
    
    // 平均轮位置
    float avg_wheel_pos = (state->wheel.left_pos + state->wheel.right_pos) / 2.0f;
    state->kinematics.wheel_pos_avg = avg_wheel_pos;
    
    // 累积位移 (前进为正)
    s_state.lqr_distance = -avg_wheel_pos * WHEEL_RADIUS_M;  // 前进为正
    state->kinematics.lqr_distance = s_state.lqr_distance;
    
    // 单轮位移 (用于更精确的控制)
    state->kinematics.left_wheel_distance = -state->wheel.left_pos * WHEEL_RADIUS_M;
    state->kinematics.right_wheel_distance = -state->wheel.right_pos * WHEEL_RADIUS_M;
    
    // 差速 (用于 YAW 控制)
    state->kinematics.wheel_vel_diff = state->wheel.left_vel - state->wheel.right_vel;
    state->kinematics.wheel_pos_diff = state->wheel.left_pos - state->wheel.right_pos;
    
    // =========================================================================
    // 4. 腿部 FK + 雅可比 + 世界坐标系 + 加速度计算
    // =========================================================================
    compute_leg_state(&raw->left_leg, true, state->imu.pitch, state->imu.pitch_rate, state->imu.pitch_acc, raw->dt, &state->left_leg);
    compute_leg_state(&raw->right_leg, false, state->imu.pitch, state->imu.pitch_rate, state->imu.pitch_acc, raw->dt, &state->right_leg);
    state->leg_valid = state->left_leg.valid && state->right_leg.valid;
    
    // =========================================================================
    // 5. 双腿平均值
    // =========================================================================
    if (state->leg_valid) {
        state->avg_leg_length = (state->left_leg.leg_length + state->right_leg.leg_length) / 2.0f;
        state->avg_body_angle = (state->left_leg.body_angle + state->right_leg.body_angle) / 2.0f;
        state->avg_dL = (state->left_leg.dL + state->right_leg.dL) / 2.0f;
        state->avg_dalpha = (state->left_leg.dalpha + state->right_leg.dalpha) / 2.0f;
        
        // 机身高度估计 (世界坐标系)
        // body_height = -avg(world_y)，因为 world_y 是轮子相对髋关节，向上为正
        // 轮子在下方，所以 world_y < 0，body_height = -world_y > 0
        state->kinematics.body_height = -(state->left_leg.world_y + state->right_leg.world_y) / 2.0f;
        
        // 机身垂直加速度 (世界坐标系)
        // 轮子的 world_ay 是相对髋关节的，机身加速度 = -轮子加速度
        state->body_ay_world = -(state->left_leg.world_ay + state->right_leg.world_ay) / 2.0f;
    }
    
    // =========================================================================
    // 6. Pitch 补偿 (考虑腿部相对世界竖直面的偏移)
    // =========================================================================
    // theta3 = pitch(IMU) + body_angle + π/2
    // 当 body_angle = -π/2（垂直向下）时，theta3 = pitch (无补偿)
    // 当腿向前摆（body_angle > -π/2）时，等效 pitch 增大
    state->pitch_compensated = state->imu.pitch;  // 默认不补偿
    state->pitch_comp_left = state->imu.pitch;
    state->pitch_comp_right = state->imu.pitch;
    
    if (enable_leg_comp && state->leg_valid) {
        // 单腿补偿
        state->pitch_comp_left = state->imu.pitch + state->left_leg.body_angle + (M_PI / 2.0f);
        state->pitch_comp_right = state->imu.pitch + state->right_leg.body_angle + (M_PI / 2.0f);
        
        // 双腿平均补偿
        state->pitch_compensated = state->imu.pitch + state->avg_body_angle + (M_PI / 2.0f);
    }
    
    // =========================================================================
    // 7. 遥控器数据归一化
    // =========================================================================
    // 注意: 实际映射系数由上层应用决定，这里只做简单归一化
    state->remote.target_speed = raw->remote.joy_y * 0.01f;      // -1 ~ +1
    state->remote.target_yaw_rate = -raw->remote.joy_x * 0.01f;  // 取反，-1 ~ +1
    state->remote.enabled = raw->remote.go;
    
    // =========================================================================
    // 8. 更新首次运行标志
    // =========================================================================
    if (s_state.first_run) {
        s_state.first_run = false;
    }
    
    return ESP_OK;
}

void robot_state_reset(robot_state_t *state) {
    // 重置所有累积量和历史数据
    s_state.yaw_total = 0.0f;
    s_state.lqr_distance = 0.0f;
    s_state.first_run = true;
    
    // 重置历史速度/加速度数据
    s_state.left_hip_vel_last = 0.0f;
    s_state.left_knee_vel_last = 0.0f;
    s_state.left_dL_last = 0.0f;
    s_state.left_dalpha_last = 0.0f;
    s_state.left_world_vy_last = 0.0f;
    
    s_state.right_hip_vel_last = 0.0f;
    s_state.right_knee_vel_last = 0.0f;
    s_state.right_dL_last = 0.0f;
    s_state.right_dalpha_last = 0.0f;
    s_state.right_world_vy_last = 0.0f;
    
    if (state != NULL) {
        state->imu.yaw_total = 0.0f;
        state->kinematics.lqr_distance = 0.0f;
    }
    
    ESP_LOGI(TAG, "Robot state reset");
}

void robot_state_reset_distance(robot_state_t *state) {
    s_state.lqr_distance = 0.0f;
    if (state != NULL) {
        state->kinematics.lqr_distance = 0.0f;
    }
}

void robot_state_reset_yaw(robot_state_t *state) {
    s_state.yaw_total = 0.0f;
    if (state != NULL) {
        state->imu.yaw_total = 0.0f;
    }
}

// ============================================================================
// 调试辅助函数
// ============================================================================

void robot_state_print(const robot_state_t *state) {
    if (state == NULL) return;
    
    ESP_LOGI(TAG, "=== Robot State ===");
    ESP_LOGI(TAG, "IMU: pitch=%.2f° rate=%.2f°/s acc=%.2f°/s² roll=%.2f° yaw=%.2f°",
             state->imu.pitch * RAD_TO_DEG,
             state->imu.pitch_rate * RAD_TO_DEG,
             state->imu.pitch_acc * RAD_TO_DEG,
             state->imu.roll * RAD_TO_DEG,
             state->imu.yaw * RAD_TO_DEG);
    
    ESP_LOGI(TAG, "Pitch comp: %.2f° (left=%.2f° right=%.2f°)",
             state->pitch_compensated * RAD_TO_DEG,
             state->pitch_comp_left * RAD_TO_DEG,
             state->pitch_comp_right * RAD_TO_DEG);
    
    ESP_LOGI(TAG, "Wheel: Lvel=%.2f Rvel=%.2f rad/s, speed=%.3f m/s",
             state->wheel.left_vel, state->wheel.right_vel,
             state->kinematics.lqr_speed);
    
    if (state->leg_valid) {
        ESP_LOGI(TAG, "Left leg: L=%.3fm α=%.1f° dL=%.3f dalpha=%.2f",
                 state->left_leg.leg_length,
                 state->left_leg.body_angle * RAD_TO_DEG,
                 state->left_leg.dL,
                 state->left_leg.dalpha);
        
        ESP_LOGI(TAG, "Left leg theta_w: %.1f° dtheta=%.2f°/s ddtheta=%.2f°/s²",
                 state->left_leg.theta_world * RAD_TO_DEG,
                 state->left_leg.dtheta_world * RAD_TO_DEG,
                 state->left_leg.ddtheta_world * RAD_TO_DEG);
        
        ESP_LOGI(TAG, "Left leg world: x=%.3f y=%.3f vx=%.3f vy=%.3f ay=%.3f",
                 state->left_leg.world_x, state->left_leg.world_y,
                 state->left_leg.world_vx, state->left_leg.world_vy,
                 state->left_leg.world_ay);
        
        ESP_LOGI(TAG, "Right leg: L=%.3fm α=%.1f° dL=%.3f dalpha=%.2f",
                 state->right_leg.leg_length,
                 state->right_leg.body_angle * RAD_TO_DEG,
                 state->right_leg.dL,
                 state->right_leg.dalpha);
        
        ESP_LOGI(TAG, "Right leg theta_w: %.1f° dtheta=%.2f°/s ddtheta=%.2f°/s²",
                 state->right_leg.theta_world * RAD_TO_DEG,
                 state->right_leg.dtheta_world * RAD_TO_DEG,
                 state->right_leg.ddtheta_world * RAD_TO_DEG);
        
        ESP_LOGI(TAG, "Body: height=%.3fm ay_world=%.3f m/s²",
                 state->kinematics.body_height,
                 state->body_ay_world);
    }
}
