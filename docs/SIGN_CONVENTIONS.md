# 符号约定与正方向对照表

> 本文档对比 **MATLAB 仿真**、**参考嵌入式代码** (DM_Balance_Hy_V1_0) 和 **我们的代码** 三套系统中，各观测变量和控制变量的正方向定义，以及 K 矩阵如何作用于它们。

## 核心关系

```
theta_参考 = theta_我们 + π
```

两者只差常数偏移 π，**动态变化方向完全一致**，不需要取反。

---

## 1. MATLAB 仿真模型 (`get_k_length.m`)

### 1.1 坐标系与变量定义

| 变量 | 定义 | 正方向 | 平衡点 |
|------|------|--------|--------|
| **θ (theta)** | 腿杆与竖直向上方向的夹角 | 逆时针为正 (向后摆) | θ₀ = 0 (倒立摆, 不稳定) |
| **dθ** | θ 的时间导数 | 同 θ 正方向 | 0 |
| **x** | 轮子水平位移 | 前进为正 | 0 |
| **v (dx/dt)** | 轮子速度 | 前进为正 | 0 |
| **φ (phi)** | 机体绕质心的俯仰角 | 前倾为正 *(从水平面量起)* | φ₀ = 0 |
| **dφ** | φ 的时间导数 | 同 φ 正方向 | 0 |
| **T** | 轮子电机扭矩 | T > 0 → 驱动轮子前进 (x 增大) | 0 |
| **Tp** | 髋关节扭矩 | Tp > 0 → 使 θ 增大 (腿向后摆) 且使 φ 增大 | 0 |

### 1.2 动力学方程

```
eqn1: d²x/dt² = (T - N·R) / (Iw/R + mw·R)
eqn2: Ip·d²θ/dt² = (P·L + PM·LM)·sin(θ) - (N·L + NM·LM)·cos(θ) - T + Tp
eqn3: IM·d²φ/dt² = Tp + NM·l·cos(φ) + PM·l·sin(φ)
```

其中 N、P 是腿杆对轮子的约束力 (水平、竖直分量)。

### 1.3 状态空间与 LQR

- **状态向量**: `[θ, dθ, x, v, φ, dφ]`
- **控制向量**: `[T, Tp]`
- **线性化点**: θ₀=0, φ₀=0, 所有速度=0, T=Tp=0
- **Q** = diag([1, 0.07, 10, 5, 300, 0.6])
- **R** = [20, 0; 0, 1]
- **K** = lqr(A, B, Q, R) → 2×6 矩阵

### 1.4 K 矩阵作用方式

```
[T ]     [K[0] K[1] K[2] K[3] K[4]  K[5] ] [θ  ]
[  ] = - [                                ] [dθ ]
[Tp]     [K[6] K[7] K[8] K[9] K[10] K[11]] [x  ]
                                             [v  ]
                                             [φ  ]
                                             [dφ ]
```

> **注意**: MATLAB 的 `lqr()` 返回的 K 满足 u = -K·x，但参考代码中直接用 `T = K[0]*θ + ...`。
> 经查证 `get_k.m`，多项式拟合的是 **+K_matlab**（没有取反），即系数**没有**吸收负号。
>
> 参考代码 `T = +K*state` 之所以正确，有两个独立的原因:
> 1. **φ/dφ 这两项**: Pitch = -φ_matlab（定义方向相反），所以 `+K[4]*Pitch = -K[4]*φ`，天然提供了负号。
> 2. **θ/dθ/x/v 这四项**: θ 在 π 附近运行（而非 MATLAB 的 0），sin(π+δ)=-sin(δ) 使得线性化结果翻转，
>    +K*state 在 θ≈π 时等价于 -K*deviation。
>
> 不是系数吸收了负号，也不是 IMU 装反了。

---

## 2. 参考嵌入式代码 (DM_Balance_Hy_V1_0)

### 2.1 IMU 约定 (Mahony 滤波器)

```c
// mahony_filter.c
mahony_filter->pitch = -asinf(mahony_filter->rMat[2][0]);
```

- **Pitch > 0**: 机体后仰 (鼻子朝上)
- **Pitch < 0**: 机体前倾
- **Gyro[1]**: pitch 方向角速度，后仰为正
- 视角: **从底盘上往下看**，右侧为右腿

### 2.2 右腿变量 (`chassisR_task.c` + `VMC_calc.c`)

| 变量 | 代码 | 正方向 | 平衡值 | 说明 |
|------|------|--------|--------|------|
| **PitchR** | `ins->Pitch` | 后仰为正 | ≈0 | 直接用 IMU Pitch |
| **PitchGyroR** | `ins->Gyro[1]` | 后仰为正 | 0 | 直接用 IMU 角速度 |
| **phi1** | `π/2 + motor[0].pos` | — | — | 右上角 = 前方电机 |
| **phi4** | `π/2 + motor[1].pos` | — | — | 右下角 = 后方电机 |
| **phi0** | `atan2(YC, XC-l5/2)` | 从 A→E 中点到 C 点的极角 | ≈ -π/2 (腿垂直) | 五连杆虚拟角 |
| **alpha** | `π/2 - phi0` | α=0 水平向后, α=-π/2 垂直向下 | ≈ π (腿垂直) | 与我们的 α 定义一致 |
| **theta** | `π/2 - PitchR - phi0` | 增大 = 向后摆 | π (= MATLAB θ₀) | LQR 状态变量 |
| **d_theta** | `-PitchGyroR - d_phi0` | 同 theta 正方向 | 0 | |
| **x_filter** | Kalman 滤波后的位移 | 前进为正 | 0 | 由观测器得到 |
| **v_filter** | Kalman 滤波后的速度 | 前进为正 | 0 | 由观测器得到 |
| **wheel_T** | LQR 输出 | T > 0 → 前进 | 0 | 轮子扭矩 |
| **Tp** | LQR 输出 | Tp > 0 → 腿向后摆 | 0 | 髋关节扭矩 |

### 2.3 右腿 K 矩阵公式

```c
// chassisR_task.c - 右腿控制环
wheel_T = K[0]*(theta - 0)
        + K[1]*(d_theta - 0)
        + K[2]*(x_filter - x_set)        // x 误差: 实际 - 目标
        + K[3]*(v_filter - 0.4*v_set)    // v 误差: 实际 - 目标
        + K[4]*(PitchR - 0.04 - phi_set) // pitch 误差: 实际 - 偏移
        + K[5]*(PitchGyroR - 0);         // pitch_rate 误差

Tp     = K[6]*(theta - 0)
        + K[7]*(d_theta - 0)
        + K[8]*(x_filter - x_set)
        + K[9]*(v_filter - 0.4*v_set)
        + K[10]*(PitchR - 0.04 - phi_set)
        + K[11]*(PitchGyroR - 0);

Tp += leg_tp;                  // 防劈叉补偿
wheel_T -= turn_T;             // 转向差速 (右轮减去)
```

**K 系数随腿长 L0 变化** (三次多项式拟合):
```c
K[i] = Poly_Coefficient[i][0]*L0³ + ...[1]*L0² + ...[2]*L0 + ...[3]
```

### 2.4 左腿变量 (`chassisL_task.c` + `VMC_calc.c`)

参考代码的左腿通过 **翻转 pitch 和 x/v** 来实现镜像，使用 **同一组 K 系数**。

| 变量 | 代码 | 与右腿的关系 | 说明 |
|------|------|-------------|------|
| **PitchL** | `0 - ins->Pitch` | **取反** | 左腿看到的 pitch 方向相反 |
| **PitchGyroL** | `0 - ins->Gyro[1]` | **取反** | |
| **phi1** | `π/2 + motor[2].pos` | 左下角 = **后方**! | 五连杆镜像: A/E 互换 |
| **phi4** | `π/2 + motor[3].pos` | 左上角 = **前方**! | 五连杆镜像: A/E 互换 |
| **theta** | `π/2 - PitchL - phi0` | 用取反后的 PitchL | 因为机构+IMU双翻转，theta 增大仍=后摆 |
| **d_theta** | `-PitchGyroL - d_phi0` | 用取反后的 PitchGyroL | |

### 2.5 左腿 K 矩阵公式 (注意 x/v 翻转!)

```c
// chassisL_task.c - 左腿控制环
wheel_T = K[0]*(theta - 0)
        + K[1]*(d_theta - 0)
        + K[2]*(x_set - x_filter)            // ← 翻转! (目标 - 实际)
        + K[3]*(0.4*v_set - v_filter)        // ← 翻转! (目标 - 实际)
        + K[4]*(PitchL - (-0.04))            // PitchL = -Pitch, 偏移也取反
        + K[5]*(PitchGyroL - 0);

Tp     = K[6]*(theta - 0 + theta_set)        // 防劈叉 theta_set
        + K[7]*(d_theta - 0)
        + K[8]*(x_set - x_filter)            // ← 翻转!
        + K[9]*(0.4*v_set - v_filter)        // ← 翻转!
        + K[10]*(PitchL - (-0.04))
        + K[11]*(PitchGyroL - 0);

Tp += leg_tp;
wheel_T -= turn_T;              // 左轮也是减去 turn_T!
```

### 2.6 为什么左腿要翻转 x/v 和 pitch?

从**底盘上方往下看**：
- 右轮顺时针转 → 机器人前进，x 增大
- 左轮逆时针转 → 机器人前进，但从左轮自身角度看是反向的

所以 K 矩阵是基于右腿推导的，左腿使用时需要翻转：
- `x_err = x_set - x` (而非 `x - x_set`)
- `v_err = v_set - v` (而非 `v - v_set`)
- `PitchL = -Pitch` (pitch 反号)

这样 K 的增益就能正确地控制左腿。

### 2.7 观测器 (`observe_task.c`)

```c
// 右轮体速度 (轮子 + 车体运动学)
wr = -motor[0].vel - INS.Gyro[1] + right.d_alpha;
vrb = wr * 0.0603 + right.L0 * right.d_theta * cos(right.theta)
    + right.d_L0 * sin(right.theta);

// 左轮体速度
wl = -motor[1].vel + INS.Gyro[1] + left.d_alpha;   // 注意 Gyro 符号反!
vlb = wl * 0.0603 + left.L0 * left.d_theta * cos(left.theta)
    + left.d_L0 * sin(left.theta);

aver_v = (vrb - vlb) / 2.0;    // 右减左取平均
x_filter += v_filter * dt;      // 积分得到位移
```

### 2.8 VMC: Tp 和 F0 如何映射到关节扭矩

```c
// VMC_calc_2() - 对右腿和左腿通用
torque_set[0] = j11 * F0 + j12 * Tp;   // 前方关节 (RightFront / LeftBack)
torque_set[1] = j21 * F0 + j22 * Tp;   // 后方关节 (RightBack / LeftFront)
```

Tp > 0 → 腿向后摆。F0 > 0 → 支撑力向上。
Jacobian `j11, j12, j21, j22` 根据五连杆几何自动计算。

### 2.9 转向与防劈叉

```c
// 转向
turn_T = Kp*(turn_set - yaw) - Kd*Gyro[2];
wheel_T_right -= turn_T;    // 右轮减
wheel_T_left  -= turn_T;    // 左轮也减! (参考代码两轮同号)

// 防劈叉
theta_err = 0 - (right.theta + left.theta);   // 两腿 theta 之和趋零
leg_tp = PID(theta_err, 0);                    // 加到两腿的 Tp 上
```

---

## 3. 我们的代码 (wheel_leg_esp_main_ctrl)

### 3.1 IMU 约定

我们的 IMU 使用的是 WIT 惯导或类似模块：
- **imu.pitch > 0**: 机体后仰 (与参考代码相同)
- **imu.pitch_rate > 0**: 后仰角速度
- **imu.yaw_rate**: 偏航角速度

### 3.2 五连杆约定

| 属性 | 我们的定义 | 参考代码定义 | 是否一致 |
|------|-----------|-------------|---------|
| **α (alpha)** | 0°=水平向后, -90°=垂直向下, -180°=水平向前 | alpha = π/2 - phi0, 同方向 | ✅ 一致 |
| **body_angle** | α 的角度值 (deg), 平衡≈-90° | phi0 (rad) | 本质相同 |
| **右腿 A 点** | 前方 | 前方 (phi1 = 右上角) | ✅ 一致 |
| **左腿 A 点** | 后方 (已镜像) | 后方 (phi1 = 左下角) | ✅ 一致 |

### 3.3 观测变量对照表

| 变量 | 我们的代码 | 平衡值 | 与参考代码对比 | 🔍 评估 |
|------|-----------|--------|---------------|---------|
| **theta_right** | `DEG2RAD(-pitch + body_angle + 90)` | 0 | 参考: `π/2 - Pitch - phi0` ≈ π | ✅ **差 π，方向一致** |
| **d_theta_right** | `-pitch_rate_rad + d_alpha_rad` | 0 | 参考: `-PitchGyroR - d_phi0` | ✅ **完全一致** |
| **theta_left** | `DEG2RAD(+pitch + body_angle + 90)` | 0 | 参考: `π/2 - PitchL - phi0` (PitchL=-Pitch) | ✅ **差 π，方向一致** |
| **d_theta_left** | `+pitch_rate_rad + d_alpha_rad` | 0 | 参考: `-PitchGyroL - d_phi0` (PitchGyroL=-Gyro) | ✅ **完全一致** |
| **x (g_lqr_distance)** | `(-0.5)*(left_pos+right_pos)*R` | 0 | 前进为正 | ✅ 一致 |
| **v (g_lqr_speed)** | `(-0.5)*(left_vel+right_vel)*R` 或观测器 | 0 | 前进为正 | ✅ 一致 |
| **pitch** | `imu.pitch` (直接传入 LQR) | 0 | 后仰为正 | ✅ 一致 |
| **pitch_rate** | `imu.pitch_rate → rad/s` | 0 | 后仰为正 | ✅ 一致 |

### 3.4 控制变量对照表

| 变量 | 我们的代码 | 与参考代码对比 | 🔍 评估 |
|------|-----------|---------------|---------|
| **T_right (wheel_T)** | `full_lqr_output_right.wheel_torque` 直接使用 | 参考: `wheel_motor[0].wheel_T` | ✅ 正常 |
| **T_left (wheel_T)** | **`-`**`full_lqr_output_left.wheel_torque` (取反!) | 参考代码左右轮不取反 | ⚠️ 见下方分析 |
| **Tp_right** | `full_lqr_output_right.leg_torque` 直接使用 | 参考: `vmcr->Tp` | ✅ 正常 |
| **Tp_left** | **`-`**`full_lqr_output_left.leg_torque` (取反!) | 参考代码左腿不取反 | ⚠️ 见下方分析 |

### 3.5 full_lqr_compute() 中的 K 矩阵作用 (`full_lqr.c`)

#### 右腿 (is_left = false):
```c
x_err          = x - x_set;           // 与参考一致
v_err          = v - v_scale*v_set;    // 与参考一致
pitch_err      = pitch - offset;       // 与参考一致 (PitchR - 0.04)
pitch_rate_err = pitch_rate;           // 与参考一致

T  = K[0]*theta + K[1]*d_theta + K[2]*x_err + K[3]*v_err + K[4]*pitch_err + K[5]*pitch_rate_err
Tp = K[6]*(theta+theta_set) + K[7]*d_theta + K[8]*x_err + K[9]*v_err + K[10]*pitch_err + K[11]*pitch_rate_err
```

#### 左腿 (is_left = true):
```c
x_err          = x_set - x;           // ← 翻转! 与参考一致
v_err          = v_scale*v_set - v;    // ← 翻转! 与参考一致
pitch_err      = -pitch - (-offset);   // = -pitch + offset，与参考一致 (PitchL = -Pitch)
pitch_rate_err = -pitch_rate;          // ← 翻转! 与参考一致

T  = K[0]*theta + K[1]*d_theta + K[2]*x_err + K[3]*v_err + K[4]*pitch_err + K[5]*pitch_rate_err
Tp = K[6]*(theta+theta_set) + ...
```

> ✅ **full_lqr.c 内部的符号处理与参考代码完全一致**。左腿通过翻转 x/v/pitch 实现镜像。

## 4. 批判性分析: 我们代码与参考代码的差异

### 4.1 ⚠️ 左轮扭矩取反 (`T_left = -LQR_output`)

**我们的代码** (`balance_test.c` ~L3784):
```c
float T_left = -g_full_lqr_output_left.wheel_torque;  // 取反!
float T_right = g_full_lqr_output_right.wheel_torque;
```

**参考代码**: 左右轮 wheel_T 都不取反，直接使用 LQR 输出。

**为什么我们需要取反？**

参考代码中，左右轮的电机是**镜像安装**的 (从底盘上方看，右轮 CW 前进，左轮 CCW 前进)。
LQR 左腿已经通过翻转 x/v 使得输出的 T 符号与右腿相反，刚好适配镜像电机。

但**我们的两个轮子电机正方向相同** (+T = 都是同一个旋转方向)。所以需要对左轮 T 取反，
才能让两轮物理上做同方向运动。

> ✅ **取反是正确的**，原因是硬件电机安装方式不同。

### 4.2 ⚠️ 左腿 Tp 取反 (`Tp_left = -LQR_output`)

**我们的代码** (`balance_test.c` ~L1447):
```c
float Tp_left = -g_full_lqr_Tp_left;    // 取反!
float Tp_right = g_full_lqr_Tp_right;
```

**参考代码**: 左右腿 Tp 都不取反，直接给 VMC。

**为什么我们需要取反？**

与轮子扭矩同理: LQR 左腿的 Tp 符号是在参考代码的电机约定下正确的。
我们的左腿关节电机安装方向可能不同，需要取反才能使 Tp 正确作用于五连杆。

具体来说：参考代码的左腿 phi1=后方, phi4=前方 (镜像)，而我们的左腿 A=后方, E=前方 (也镜像了)，
但关节电机的正方向约定可能不同，导致 Tp 需要取反。

> ✅ **取反是正确的**，但建议在代码中添加更详细的注释说明原因。

### 4.3 ⚠️ 右腿 Tp→VMC 注入的额外取反

**我们的代码** (`balance_test.c` ~L1455):
```c
// 左腿
delta_left = Tp_left - F_alpha_left;
hip_torque  += J[2] * delta_left;      // 正常
knee_torque += J[3] * delta_left;      // 正常

// 右腿
delta_right = Tp_right - F_alpha_right;
hip_torque  += -(J[2] * delta_right);  // ← 额外取反!
knee_torque += -(J[3] * delta_right);  // ← 额外取反!
```

**参考代码**: 左右腿都用 `VMC_calc_2()` 统一计算 `torque = J * [F0; Tp]`，没有额外取反。

**为什么右腿有额外取反？**

这可能是因为我们的右腿 Jacobian `J[2], J[3]` 的符号约定与 Tp 方向不匹配。
需要确认: 如果 Tp > 0 应该使腿向后摆，而 `J[2]*Tp` 计算出的关节扭矩方向是否正确。

> ⚠️ **需要验证**: 这个额外取反是否正确取决于我们的 Jacobian 计算约定。
> 建议添加单元测试: 给 Tp > 0，检查关节扭矩是否真的使腿向后摆。

### 4.4 ⚠️ 转向差速

**参考代码**:
```c
// 右轮和左轮都减去 turn_T
wheel_T_right -= turn_T;
wheel_T_left  -= turn_T;
```

**我们的代码**:
```c
T_left  += turn_T;
T_right -= turn_T;
```

两者的转向逻辑不同。参考代码两轮同号减去 turn_T，我们是差速 (一加一减)。
这会导致转向行为不同：
- 参考代码: turn_T 影响整体前后运动 + 转向
- 我们的代码: turn_T 只产生纯差速转向

> ⚠️ **行为不同但不一定是 bug**。参考代码中两轮电机镜像安装，
> 两轮同减 turn_T 物理上等价于差速。**如果我们的两轮电机正方向相同**，
> 那么我们的一加一减才是正确的差速。

### 4.5 ⚠️ 防劈叉

**参考代码**:
```c
theta_err = 0 - (right.theta + left.theta);   // 两腿 theta 之和趋零
leg_tp = PID(theta_err, 0);                    // 加到两腿的 Tp 上
```

**我们的代码**:
```c
theta_err = theta_left - theta_right;
// 用的是纯几何角 (不含 pitch), 防止 2*pitch 耦合
// theta_left ≈ 0, theta_right ≈ 0
// 劈叉时 theta_err ≠ 0
```

> ⚠️ **实现不同但目的相同**。我们用的是 theta_left - theta_right (差值)，
> 参考代码用的是 -(sum)。参考代码的方式会受到 pitch 影响 (因为 theta 包含 pitch)。
> 我们单独用纯几何角可能更合理，避免 pitch 的干扰。

---

## 5. 三套系统对照总结

### 5.1 状态变量正方向统一表

| 状态变量 | MATLAB 仿真 | 参考代码 (右腿) | 我们的代码 (右腿) | 三者一致? |
|---------|------------|----------------|------------------|----------|
| θ 增大 | 腿向后摆 | 腿向后摆 | 腿向后摆 | ✅ 一致 |
| dθ 增大 | 向后摆加速 | 向后摆加速 | 向后摆加速 | ✅ 一致 |
| x 增大 | 前进 | 前进 | 前进 | ✅ 一致 |
| v 增大 | 前进加速 | 前进加速 | 前进加速 | ✅ 一致 |
| **φ/pitch 增大** | **前倾 (φ>0)** | **后仰 (Pitch>0)** | **后仰 (pitch>0)** | ❌ **方向相反!** |
| **dφ/pitch_rate 增大** | **前倾加速** | **后仰加速** | **后仰加速** | ❌ **方向相反!** |
| T > 0 | 驱动前进 | 驱动前进 | 驱动前进 | ✅ 一致 |
| Tp > 0 | 腿向后摆 | 腿向后摆 | 腿向后摆 | ✅ 一致 |

> **⚠️ 关于 φ/Pitch 方向相反的解释:**
>
> MATLAB 建模时选择了 φ **前倾为正**（数学上逆时针为正，从右侧看）。
> IMU 的 Pitch 是标准航空约定：**后仰为正**。两者天然差一个负号：**Pitch = -φ_matlab**。
>
> 这**不是** IMU 安装特殊，也**不是**多项式系数"吸收"了负号。
> 这就是两套坐标系的定义方向本来就是反的。
>
> **参考代码如何处理这个差异:**
>
> MATLAB 理论控制律 `u = -K*x`（6 个状态全展开）:
> ```
> T = -K[0]*θ - K[1]*dθ - K[2]*x - K[3]*v - K[4]*φ - K[5]*dφ
> ```
> 注意：**全部 6 项前面都是负号**，这是 `u = -K*x` 的直接展开。
>
> 参考代码右腿实际写的:
> ```
> T = +K[0]*theta + K[1]*d_theta + K[2]*x + K[3]*v + K[4]*Pitch + K[5]*PitchGyro
> ```
> 注意：**全部 6 项前面都是正号**，即 `T = +K*state`。
>
> 那为什么 `+K*state` 能等价于 `-K*x`？分两组来看：
>
> **第一组: φ/dφ 这两项** — 靠 Pitch = -φ 提供负号
> - `+K[4]*Pitch = +K[4]*(-φ) = -K[4]*φ` ✅ 与 MATLAB 一致
> - `+K[5]*PitchGyro = +K[5]*(-dφ) = -K[5]*dφ` ✅ 与 MATLAB 一致
>
> **第二组: θ/dθ/x/v 这四项** — 靠 θ 在 π 附近运行提供负号
> - 参考代码 theta ≈ π，MATLAB θ ≈ 0，两者关系: theta_参考 = θ_matlab + π
> - 在 θ=π 附近线性化时，sin(π+δ)=-sin(δ), cos(π+δ)=-cos(δ)
> - 这使得 A 矩阵中与 θ 相关的项翻号，等价于 K 矩阵前的符号翻转
> - 所以 `+K[0]*theta_参考` 在 θ≈π 附近的效果 = `-K[0]*θ_matlab` 在 θ≈0 附近的效果
> - x/v 项的翻号也源于同样的机制（θ 在 π 改变了动力学方程中 x 的耦合方向）
>
> **总结: 6 个状态中，φ 和 dφ 的负号由 Pitch=-φ 自然提供；
> θ/dθ/x/v 的负号由 θ 在 π 附近运行来提供。两组负号来源不同，但都是自洽的。**### 5.2 θ 平衡值对照

| 系统 | θ 平衡值 | 说明 |
|------|---------|------|
| MATLAB | 0 | 倒立摆顶部 (不稳定平衡) |
| 参考代码 | π | theta = π/2 - 0 - (-π/2) = π |
| 我们的代码 | 0 | theta = DEG2RAD(0 + (-90) + 90) = 0 |

`theta_参考 = theta_我们 + π`，动态部分完全相同。

### 5.3 K 矩阵数值示例 (L0 = 0.14m)

| K 索引 | 状态→控制 | K 值 | -K (实际作用) | 物理含义 (MATLAB 坐标系) |
|--------|----------|------|-------------|------------------------|
| K[0] | θ → T | -1.45 | **+1.45** | θ↑(后摆) → T↑(前进) = 追回腿 ✅ |
| K[1] | dθ → T | -0.17 | +0.17 | 后摆加速 → 前进阻尼 ✅ |
| K[2] | x → T | -0.64 | +0.64 | 超前 → 前进 = 倒立摆"先推再倾" ✅ |
| K[3] | v → T | -0.64 | +0.64 | 速度过快 → 前进 = 同上 ✅ |
| K[4] | φ → T | **+1.68** | **-1.68** | 前倾 → 后退 = 推回正 ✅ |
| K[5] | dφ → T | +0.09 | -0.09 | 前倾加速 → 后退阻尼 ✅ |
| K[6] | θ → Tp | +2.64 | **-2.64** | 后摆 → 腿前摆 = 回正 ✅ |
| K[7] | dθ → Tp | +0.36 | -0.36 | 后摆加速 → 前摆阻尼 ✅ |
| K[8] | x → Tp | +1.37 | -1.37 | 超前 → 腿前摆 = 前移支撑点 ✅ |
| K[9] | v → Tp | +1.36 | -1.36 | 速度过快 → 腿前摆 ✅ |
| K[10] | φ → Tp | **+15.62** | **-15.62** | 前倾 → 腿强力前摆 = 支撑点追重心 ✅ |
| K[11] | dφ → Tp | +0.71 | -0.71 | 前倾加速 → 前摆阻尼 ✅ |

> **注 1**: 以上数值由多项式拟合在 L0=0.14m (典型站立腿长) 处计算。
> 
> **注 2**: K[10]=15.62 是所有增益中绝对值最大的。这与 Q(φ)=300 (最高权重)
> 和 R(Tp)=1 (最便宜控制) 完全一致 — LQR 优先用便宜的 Tp 来修正最重要的 φ。
>
> **注 3**: K[2] 和 K[3] 的 -K 为正 (x/v 偏大时 T 前进) 看似反直觉，
> 但这是倒立摆特有行为: 要后退必须先前推制造后仰，再靠 φ 反馈完成后退。
> 详细分析见 `REFERENCE_VS_SIMULATION_DIRECTIONS.md` 第七节。

---

## 6. 已修复的 Bug 记录

| # | Bug 描述 | 文件 | 修复方式 |
|---|---------|------|---------|
| 1 | theta/d_theta 没有区分左右腿 | `balance_test.c` | 左腿: +pitch, 右腿: -pitch |
| 2 | 左轮 T 没有取反 | `balance_test.c` | `T_left = -output.wheel_torque` |
| 3 | 左腿 Tp 没有取反 | `balance_test.c` | `Tp_left = -g_full_lqr_Tp_left` |

---

## 7. 待验证项

- [ ] 右腿 Tp→VMC 注入时的额外取反 (`-(J[2]*delta)`) 是否正确
- [ ] 转向差速逻辑 (一加一减 vs 同号减) 是否在当前电机配置下正确
- [ ] 防劈叉用纯几何角 vs 含 pitch 角, 哪种效果更好
- [x] ~~MATLAB φ (前倾正) vs 代码 Pitch (后仰正) 的符号变换~~ → **已解决**: φ=-Pitch, 天然提供 `u=-K*x` 中 φ 项的负号, 不需要额外处理
- [ ] 观测器 (Kalman) 的 x/v 正方向是否与 LQR 需要的一致

---

*文档创建于 2026-03-17，基于代码分支 `feature/lqr-optimize`*
