/**
 * @file leg_kinematics.h
 * @brief 腿部运动学模块 - 串联二连杆正逆运动学
 * @author Bubble
 * @date 2026-01-24
 * 
 * 机构说明:
 *   串联二连杆轮腿结构，每条腿由大腿电机(Hip)和小腿电机(Knee)驱动
 *   
 *         机身 (Body)
 *            │
 *     ┌──────┴──────┐
 *     │  Hip Motor  │  ← 大腿电机 (θ1)
 *     └──────┬──────┘
 *            │ L1 (大腿长度)
 *            │
 *     ┌──────┴──────┐
 *     │ Knee Motor  │  ← 小腿电机 (θ2)
 *     └──────┬──────┘
 *            │ L2 (小腿长度)
 *            │
 *     ┌──────┴──────┐
 *     │ Wheel Motor │  ← 轮电机
 *     └─────────────┘
 *   
 * 运动学空间:
 *   - 腿长 (leg_length): 大腿电机轴心到轮电机轴心的直线距离
 *   - 身体夹角 (body_angle): 该直线与机身垂直向下方向的夹角，向前为正
 */

#ifndef LEG_KINEMATICS_H
#define LEG_KINEMATICS_H

#include <stdbool.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

// ============================================================================
// 配置参数 (可在 menuconfig 或代码中覆盖)
// ============================================================================

#ifndef LEG_THIGH_LENGTH
#define LEG_THIGH_LENGTH            0.10f   // 大腿长度 (米)
#endif

#ifndef LEG_SHANK_LENGTH
#define LEG_SHANK_LENGTH            0.10f   // 小腿长度 (米)
#endif

// 电机零点偏移 (将电机编码器角度转换为运动学角度)
// 运动学定义: theta2=0 表示小腿伸直，顺时针弯曲为负
// 左腿校准: 电机读数 Hip=-60°, Knee=-55° 时，腿部垂直向下(body_angle=-90°)，大小腿夹角90°
// 此时: theta2 = -90°, beta = -45°, theta1 = body_angle - beta = -90 - (-45) = -45°
// theta_knee = motor_knee - knee_offset  =>  -90 = -55 - offset  =>  offset = 35
// theta_hip = motor_hip - hip_offset  =>  -45 = -60 - offset  =>  offset = -15
#ifndef LEG_LEFT_HIP_OFFSET
#define LEG_LEFT_HIP_OFFSET         (-15.0f)
#endif

#ifndef LEG_LEFT_KNEE_OFFSET
#define LEG_LEFT_KNEE_OFFSET        (35.0f)
#endif

// 右腿: 镜像对称
#ifndef LEG_RIGHT_HIP_OFFSET
#define LEG_RIGHT_HIP_OFFSET        (15.0f)
#endif

#ifndef LEG_RIGHT_KNEE_OFFSET
#define LEG_RIGHT_KNEE_OFFSET       (-35.0f)
#endif

// 腿部工作空间限位
#ifndef LEG_LENGTH_MIN
#define LEG_LENGTH_MIN              0.07f   // 最小腿长 (米)
#endif

#ifndef LEG_LENGTH_MAX
#define LEG_LENGTH_MAX              0.17f   // 最大腿长 (米)
#endif

#ifndef LEG_BODY_ANGLE_MIN
#define LEG_BODY_ANGLE_MIN          (-160.0f) // 身体夹角最小值 (度), 向前蹬腿
#endif

#ifndef LEG_BODY_ANGLE_MAX
#define LEG_BODY_ANGLE_MAX          (-20.0f)  // 身体夹角最大值 (度), 向后蹬腿
#endif

// ============================================================================
// 数据结构
// ============================================================================

/**
 * @brief 腿部运动学参数
 */
typedef struct {
    float thigh_length;     // 大腿长度 (米)
    float shank_length;     // 小腿长度 (米)
    float hip_offset;       // 大腿电机零点偏移 (度)
    float knee_offset;      // 小腿电机零点偏移 (度)
} leg_kin_params_t;

/**
 * @brief 腿部关节空间状态 (电机角度)
 */
typedef struct {
    float hip_angle;        // 大腿电机角度 (度)
    float knee_angle;       // 小腿电机角度 (度)
} leg_joint_state_t;

/**
 * @brief 腿部工作空间状态 (腿长+身体夹角)
 */
typedef struct {
    float leg_length;       // 腿长 (米)
    float body_angle;       // 身体夹角 (度), 向前为正
} leg_workspace_state_t;

/**
 * @brief 完整腿部状态
 */
typedef struct {
    leg_joint_state_t joint;        // 关节空间
    leg_workspace_state_t workspace; // 工作空间
    bool is_left;                    // 是否为左腿
    bool valid;                      // 数据是否有效
} leg_state_t;

// ============================================================================
// API 函数
// ============================================================================

/**
 * @brief 获取默认腿部参数
 * @param is_left 是否为左腿
 * @param params 输出参数
 */
void leg_kin_get_default_params(bool is_left, leg_kin_params_t *params);

/**
 * @brief 正运动学: 关节空间 -> 工作空间
 * @param joint 关节空间状态 (电机角度)
 * @param is_left 是否为左腿
 * @param params 运动学参数 (NULL 使用默认参数)
 * @param workspace 输出工作空间状态
 * @return ESP_OK 成功
 */
esp_err_t leg_kin_forward(const leg_joint_state_t *joint, bool is_left,
                          const leg_kin_params_t *params,
                          leg_workspace_state_t *workspace);

/**
 * @brief 逆运动学: 工作空间 -> 关节空间
 * @param workspace 工作空间状态 (腿长+身体夹角)
 * @param is_left 是否为左腿
 * @param params 运动学参数 (NULL 使用默认参数)
 * @param joint 输出关节空间状态
 * @return ESP_OK 成功, ESP_ERR_INVALID_ARG 目标不可达
 */
esp_err_t leg_kin_inverse(const leg_workspace_state_t *workspace, bool is_left,
                          const leg_kin_params_t *params,
                          leg_joint_state_t *joint);

/**
 * @brief 检查工作空间目标是否可达
 * @param leg_length 目标腿长 (米)
 * @param body_angle 目标身体夹角 (度)
 * @param params 运动学参数 (NULL 使用默认参数)
 * @return true 可达, false 不可达
 */
bool leg_kin_is_reachable(float leg_length, float body_angle, 
                          const leg_kin_params_t *params);

/**
 * @brief 限制工作空间目标到可达范围
 * @param leg_length 输入/输出腿长 (米)
 * @param body_angle 输入/输出身体夹角 (度)
 * @param params 运动学参数 (NULL 使用默认参数)
 */
void leg_kin_clamp_workspace(float *leg_length, float *body_angle,
                             const leg_kin_params_t *params);

/**
 * @brief 计算腿部雅可比矩阵 (用于速度/力映射)
 * @param joint 当前关节状态
 * @param is_left 是否为左腿
 * @param params 运动学参数 (NULL 使用默认参数)
 * @param J 输出 2x2 雅可比矩阵 [J11, J12; J21, J22]
 * @return ESP_OK 成功
 * @note J 将关节速度映射到工作空间速度: [dL; dα] = J * [dθ1; dθ2]
 */
esp_err_t leg_kin_jacobian(const leg_joint_state_t *joint, bool is_left,
                           const leg_kin_params_t *params,
                           float J[4]);

// ============================================================================
// VMC (Virtual Model Control) 数据结构和 API
// ============================================================================

/**
 * @brief VMC 坐标系类型
 * 
 * VMC_COORD_WORLD: 世界坐标系 (x-水平, y-垂直)
 *   - 直接控制离地高度和水平速度
 *   - 需要 pitch 补偿 (θ1_world = θ1_body + pitch)
 *   - 适合需要精确高度控制的场景
 * 
 * VMC_COORD_BODY: 机身坐标系 (L-腿长, α-身体角度)
 *   - 控制腿部弹簧特性和身体姿态
 *   - 不需要 pitch 补偿，使用现有雅可比
 *   - 与 LQR 职责分离更清晰
 *   - 适合轮腿机器人 (LQR管平衡，VMC管腿部)
 */
typedef enum {
    VMC_COORD_WORLD = 0,    // 世界坐标系 (x-y)
    VMC_COORD_BODY = 1      // 机身坐标系 (L-α)
} vmc_coord_type_t;

/**
 * @brief VMC 控制参数 (世界坐标系)
 * 
 * 控制策略:
 *   F_x = K_vx × (target_vx - current_vx)              // 水平: 纯速度控制
 *   F_y = K_y × (target_y - current_y) + D_y × (target_vy - current_vy) + F_gravity
 *                                                      // 垂直: 弹簧-阻尼 + 重力补偿
 *   τ_hip_pitch = K_pitch × (0 - pitch) + D_pitch × (0 - pitch_rate)
 *                                                      // 机身姿态: PD 控制
 */
typedef struct {
    // 坐标系选择
    vmc_coord_type_t coord_type;  // VMC_COORD_WORLD 或 VMC_COORD_BODY
    
    // === 世界坐标系参数 (coord_type == VMC_COORD_WORLD) ===
    // 水平方向 (X) - 速度控制
    float K_vx;             // 水平速度增益 (Ns/m), 典型值: 10~100
    
    // 垂直方向 (Y) - 弹簧阻尼位置控制  
    float K_y;              // 垂直刚度 (N/m), 典型值: 500~2000
    float D_y;              // 垂直阻尼 (Ns/m), 典型值: 20~100
    
    // === 机身坐标系参数 (coord_type == VMC_COORD_BODY) ===
    // 腿长方向 (L) - 弹簧阻尼
    float K_L;              // 腿长刚度 (N/m), 典型值: 500~2000
    float D_L;              // 腿长阻尼 (Ns/m), 典型值: 20~100
    
    // 身体角度方向 (α) - 弹簧阻尼
    float K_alpha;          // 身体角度刚度 (Nm/rad), 典型值: 1~20
    float D_alpha;          // 身体角度阻尼 (Nm·s/rad), 典型值: 0.1~2
    
    // === 通用参数 ===
    // 机身 Pitch 控制 - 通过髋关节内力矩控制机身姿态
    float K_pitch;          // Pitch 刚度 (Nm/rad), 典型值: 1~10
    float D_pitch;          // Pitch 阻尼 (Nm·s/rad), 典型值: 0.1~1
    float target_pitch;     // 目标 pitch 角度 (rad), 通常为 0
    bool pitch_ctrl_enable; // Pitch 控制使能
    
    // 双腿协调控制 (Leg Sync) - 消除左右腿身体角度差异
    float K_sync;           // 协调控制 P 增益 (Nm/rad), 典型值: 0.5~2
    float D_sync;           // 协调控制 D 增益 (Nm·s/rad), 典型值: 0.05~0.2
    bool sync_enable;       // 协调控制使能
    
    // 重力补偿
    float gravity_comp;     // 重力补偿系数 (0~1), 1.0=完全补偿
    float robot_mass;       // 机器人质量 (kg), 用于计算重力补偿
    
    // 输出限幅
    float max_hip_torque;   // 髋关节最大扭矩 (Nm)
    float max_knee_torque;  // 膝关节最大扭矩 (Nm)
} vmc_params_t;

/**
 * @brief VMC 输入状态
 * 
 * 世界坐标系 (coord_type == VMC_COORD_WORLD):
 *   x: 水平方向，向后为正
 *   y: 垂直方向，向上为正 (机身高度方向)
 *   原点: 髋关节轴心
 *   
 * 机身坐标系 (coord_type == VMC_COORD_BODY):
 *   L: 腿长 (髋关节到轮子的直线距离)
 *   α: 身体角度 (腿相对机身垂直向下的夹角，向前为正)
 */
typedef struct {
    // === 世界坐标系输入 (coord_type == VMC_COORD_WORLD) ===
    float target_vx;        // 目标水平速度 (m/s)，向后为正
    float target_y;         // 目标机身高度 (m)，向上为正
    float target_vy;        // 目标垂直速度 (m/s)，一般为 0
    float current_y;        // 当前机身高度 (m)
    float current_vx;       // 当前水平速度 (m/s)
    float current_vy;       // 当前垂直速度 (m/s)
    
    // === 机身坐标系输入 (coord_type == VMC_COORD_BODY) ===
    float target_L;         // 目标腿长 (m)
    float target_dL;        // 目标腿长变化率 (m/s)，一般为 0
    float target_alpha;     // 目标身体角度 (rad)
    float target_dalpha;    // 目标身体角速度 (rad/s)，一般为 0
    float current_L;        // 当前腿长 (m)
    float current_dL;       // 当前腿长变化率 (m/s)
    float current_alpha;    // 当前身体角度 (rad)
    float current_dalpha;   // 当前身体角速度 (rad/s)
    
    // === 通用输入: IMU 姿态 (用于 pitch 控制和世界坐标系转换) ===
    float current_pitch;    // 当前 pitch 角度 (rad), 前倾为正
    float current_pitch_rate; // 当前 pitch 角速度 (rad/s)
} vmc_input_t;

/**
 * @brief VMC 输出 (关节扭矩)
 */
typedef struct {
    float hip_torque;       // 髋关节扭矩 (Nm) - 总扭矩 (VMC + pitch)
    float knee_torque;      // 膝关节扭矩 (Nm)
    
    // 调试信息 - 世界坐标系虚拟力 (coord_type == VMC_COORD_WORLD)
    float F_x;              // 水平虚拟力 (N), 向后为正
    float F_y;              // 垂直虚拟力 (N), 向上为正 (支撑力)
    
    // 调试信息 - 机身坐标系虚拟力 (coord_type == VMC_COORD_BODY)
    float F_L;              // 沿腿方向虚拟力 (N), 伸展为正
    float F_alpha;          // 身体角度虚拟力矩 (Nm), 向前为正
    
    // 通用调试信息
    float F_gravity;        // 重力补偿力 (N)
    float tau_hip_vmc;      // VMC 计算的髋关节扭矩 (Nm)
    float tau_hip_pitch;    // Pitch 控制的髋关节扭矩 (Nm)
} vmc_output_t;

/**
 * @brief 计算笛卡尔坐标系雅可比矩阵 (世界坐标系)
 * @param joint 当前关节状态
 * @param is_left 是否为左腿
 * @param params 运动学参数 (NULL 使用默认参数)
 * @param pitch_rad IMU 机身俯仰角 (弧度), 前倾为正
 * @param J 输出 2x2 雅可比矩阵 [J00, J01; J10, J11]
 * @return ESP_OK 成功
 * @note J 将关节速度映射到笛卡尔速度: [dx; dy] = J × [dθ1; dθ2]
 *       θ1_world = θ1_body + pitch (世界坐标系下的大腿角度)
 *       x = -L1*sin(θ1_world) - L2*sin(θ1_world+θ2)  (向后为正)
 *       y = L1*cos(θ1_world) + L2*cos(θ1_world+θ2)   (向上为正)
 */
esp_err_t leg_kin_jacobian_cartesian(const leg_joint_state_t *joint, 
                                      bool is_left,
                                      const leg_kin_params_t *params,
                                      float pitch_rad,
                                      float J[4]);

/**
 * @brief 正运动学: 关节角度 -> 笛卡尔位置 (世界坐标系)
 * @param joint 关节角度
 * @param is_left 是否为左腿
 * @param params 运动学参数 (NULL 使用默认参数)
 * @param pitch_rad IMU 机身俯仰角 (弧度), 前倾为正
 * @param x 输出水平位置 (m), 向后为正
 * @param y 输出垂直位置 (m), 向上为正 (即机身高度)
 * @return ESP_OK 成功
 */
esp_err_t leg_kin_forward_cartesian(const leg_joint_state_t *joint,
                                     bool is_left,
                                     const leg_kin_params_t *params,
                                     float pitch_rad,
                                     float *x, float *y);

/**
 * @brief VMC 计算关节扭矩 (世界坐标系输入)
 * @param params VMC 参数
 * @param input VMC 输入 (世界坐标系速度和位置)
 * @param joint 当前关节状态
 * @param is_left 是否为左腿
 * @param pitch_rad IMU 机身俯仰角 (弧度), 前倾为正
 * @param output VMC 输出 (关节扭矩)
 * @return ESP_OK 成功
 * 
 * 控制公式:
 *   F_x = K_vx × (target_vx - current_vx)
 *   F_y = K_y × (target_y - current_y) + D_y × (target_vy - current_vy) + gravity_comp × m × g
 *   [τ_hip; τ_knee] = -J_cart^T × [F_x; F_y]
 * 
 * 注意: 雅可比在世界坐标系下计算，θ1_world = θ1_body + pitch
 */
esp_err_t vmc_compute_torque(const vmc_params_t *params,
                              const vmc_input_t *input,
                              const leg_joint_state_t *joint,
                              bool is_left,
                              float pitch_rad,
                              vmc_output_t *output);

/**
 * @brief 获取默认 VMC 参数
 * @param params 输出参数
 */
void vmc_get_default_params(vmc_params_t *params);

/**
 * @brief 打印腿部参数信息
 * @param params 参数 (NULL 打印默认参数)
 * @param is_left 是否为左腿
 */
void leg_kin_print_params(const leg_kin_params_t *params, bool is_left);

// ============================================================================
// VMC 控制器高层接口 (封装完整的单腿 VMC 计算流程)
// ============================================================================

/**
 * @brief 单腿传感器数据 (应用层提供)
 */
typedef struct {
    float hip_angle;        // 髋关节电机角度 (度)
    float knee_angle;       // 膝关节电机角度 (度)
    float hip_velocity;     // 髋关节电机速度 (rpm)
    float knee_velocity;    // 膝关节电机速度 (rpm)
} vmc_leg_sensor_t;

/**
 * @brief VMC 控制器输入 (应用层提供)
 */
typedef struct {
    // IMU 数据
    float pitch_deg;        // 机身俯仰角 (度), 前倾为正
    float pitch_rate_deg;   // 机身俯仰角速度 (度/秒)
    
    // 轮子数据 (用于估计水平速度)
    float robot_vx;         // 机器人对地水平速度 (m/s), 向后为正
    
    // 目标值 (由上层控制器给出)
    float target_leg_length;    // 目标腿长 (m), 通常由 Roll 控制调整
    float target_body_angle_deg;// 目标身体夹角 (度), 默认 -90 (垂直向下)
    float target_vx;        // 目标水平速度 (m/s), 默认 0
    
    // 单腿传感器数据
    vmc_leg_sensor_t sensor;
} vmc_ctrl_input_t;

/**
 * @brief VMC 控制器输出 (应用层使用)
 */
typedef struct {
    float hip_torque;       // 髋关节输出扭矩 (Nm)
    float knee_torque;      // 膝关节输出扭矩 (Nm)
    
    // 状态估计 (可选使用)
    float current_leg_length;   // 当前腿长 (m)
    float current_body_angle;   // 当前身体夹角 (度)
    float current_height;       // 当前离地高度 (m), 世界坐标系
    
    // 调试信息
    vmc_output_t debug;     // 详细调试输出
} vmc_ctrl_output_t;

/**
 * @brief VMC 单腿控制器计算 (高层接口)
 * 
 * 完整流程:
 *   1. 从传感器数据计算关节状态
 *   2. 正运动学: 计算当前腿长、身体角度、世界坐标高度
 *   3. 雅可比 + 速度计算
 *   4. 填充 VMC 输入
 *   5. 调用 vmc_compute_torque() 计算扭矩
 *   6. 返回输出扭矩和状态估计
 * 
 * @param params VMC 控制参数
 * @param input 控制器输入 (传感器数据 + 目标值)
 * @param is_left 是否为左腿
 * @param output 控制器输出 (扭矩 + 状态)
 * @return ESP_OK 成功
 * 
 * 使用示例:
 * @code
 *   vmc_ctrl_input_t input = {
 *       .pitch_deg = imu.pitch,
 *       .pitch_rate_deg = imu.pitch_rate,
 *       .robot_vx = wheel_speed * WHEEL_RADIUS,
 *       .target_leg_length = 0.14f,
 *       .target_body_angle_deg = -90.0f,
 *       .sensor = {
 *           .hip_angle = motor_read_pos(hip),
 *           .knee_angle = motor_read_pos(knee),
 *           .hip_velocity = motor_read_vel(hip),
 *           .knee_velocity = motor_read_vel(knee)
 *       }
 *   };
 *   vmc_ctrl_output_t output;
 *   vmc_ctrl_compute(&params, &input, true, &output);
 *   motor_set_torque(hip, output.hip_torque);
 *   motor_set_torque(knee, output.knee_torque);
 * @endcode
 */
esp_err_t vmc_ctrl_compute(const vmc_params_t *params,
                            const vmc_ctrl_input_t *input,
                            bool is_left,
                            vmc_ctrl_output_t *output);

// ============================================================================
// 双腿 VMC 控制器 (包含双腿协调控制)
// ============================================================================

/**
 * @brief 单腿 VMC 输入 (用于双腿计算的简化结构)
 */
typedef struct {
    float target_leg_length;    // 目标腿长 (m)
    float target_body_angle_deg;// 目标身体夹角 (度)
    vmc_leg_sensor_t sensor;    // 腿部传感器数据
} vmc_leg_input_t;

/**
 * @brief 双腿 VMC 输入
 */
typedef struct {
    // 公共数据 (IMU + 速度)
    float pitch_deg;            // 机身俯仰角 (度)
    float pitch_rate_deg;       // 机身俯仰角速度 (度/秒)
    float robot_vx;             // 机器人对地水平速度 (m/s)
    float target_vx;            // 目标水平速度 (m/s)
    
    // 左右腿单独数据
    vmc_leg_input_t left;       // 左腿输入
    vmc_leg_input_t right;      // 右腿输入
} vmc_dual_input_t;

/**
 * @brief 双腿 VMC 输出
 */
typedef struct {
    vmc_ctrl_output_t left;     // 左腿输出
    vmc_ctrl_output_t right;    // 右腿输出
    
    // 协调控制调试信息
    float angle_diff_deg;       // 左右腿身体角度差 (度)
    float angle_diff_rate_deg;  // 角度差变化率 (度/秒)
    float F_sync;               // 协调补偿虚拟力 (N)
} vmc_dual_output_t;

/**
 * @brief 双腿 VMC 控制器计算 (包含双腿协调控制)
 * 
 * 功能:
 *   1. 分别计算左右腿的 VMC 扭矩
 *   2. 如果启用协调控制 (sync_enable)，计算左右腿角度差的 PD 补偿
 *   3. 将补偿量通过雅可比转换，叠加到关节扭矩
 * 
 * 协调控制原理:
 *   - 角度差 = left_body_angle - right_body_angle
 *   - F_sync = K_sync × 角度差 + D_sync × 角度差变化率
 *   - 左腿: ΔF_alpha = -F_sync (角度大时减小)
 *   - 右腿: ΔF_alpha = +F_sync (角度小时增大)
 *   - 通过雅可比转换: Δτ = J^T × [0; ΔF_alpha]
 * 
 * @param params VMC 控制参数 (包含 sync_enable, K_sync, D_sync)
 * @param input 双腿输入数据
 * @param output 双腿输出数据 (扭矩已包含协调补偿)
 * @return ESP_OK 成功
 */
esp_err_t vmc_dual_compute(const vmc_params_t *params,
                           const vmc_dual_input_t *input,
                           vmc_dual_output_t *output);

#ifdef __cplusplus
}
#endif

#endif // LEG_KINEMATICS_H
