# Full LQR 使用说明与符号链路逐行分析

> 创建日期: 2026-03-20  
> 分支: `feature/lqr-optimize`  
> 目标: 逐步拆解 `full_lqr.c` 的完整数据流，明确 K 矩阵如何将观测变量转化为控制变量，并标记所有可疑之处。

---

## 一、Full LQR 是什么、怎么用

### 1.1 基本概念

Full LQR 是一个 **6 状态 × 2 输出** 的线性状态反馈控制器。

- **输入**: 6 维状态向量 `[θ, dθ, x_err, v_err, pitch_err, pitch_rate_err]`
- **输出**: 2 个力矩
  - `T` (wheel_torque): 轮子电机扭矩，直接驱动轮毂电机
  - `Tp` (leg_torque): 腿部摆动扭矩，通过 VMC 的 J^T 转换为 hip/knee 关节扭矩

### 1.2 调用流程

```
balance_test.c (上层)
    │
    ├─ 1. 计算 theta/d_theta (左右腿分别计算，含 pitch 和 body_angle)
    ├─ 2. 构造 full_lqr_input_t (填入所有状态)
    ├─ 3. 分别对左右腿调用 full_lqr_compute()
    │      ├─ 根据 L0 插值 K 增益 (12 个)
    │      ├─ 根据 is_left 翻转 x/v/pitch 的符号
    │      ├─ T  = K[0..5]  × state
    │      ├─ Tp = K[6..11] × state + 防劈叉
    │      ├─ 计算 turn_torque (转向 PD)
    │      └─ 输出 T, Tp, turn_torque
    ├─ 4. 轮子扭矩: T_left = -output_left.T,  T_right = output_right.T
    ├─ 5. 转向差速: T_left += turn_T,  T_right -= turn_T
    ├─ 6. Tp 保存到全局变量，在 VMC 计算后注入:
    │      Tp_left_global  = output_left.Tp
    │      Tp_right_global = -output_right.Tp  ← 右腿 Tp 取反
    └─ 7. VMC 注入时:
           Tp_left_inject  = -Tp_left_global   ← 再取反
           Tp_right_inject = Tp_right_global
           delta = Tp_inject - F_alpha_vmc
           hip  += J[2] * delta      (左腿)
           hip  += -(J[2] * delta)   (右腿, 额外取反)
```

### 1.3 API

```c
// 初始化
full_lqr_controller_t ctrl;
full_lqr_init(&ctrl, NULL);  // NULL = 使用默认参数

// 每个控制周期, 对左右腿各调用一次:
full_lqr_input_t input = { .theta = ..., .is_left = true, ... };
full_lqr_output_t output;
full_lqr_compute(&ctrl, &input, &output);

// 输出:
//   output.wheel_torque  → 轮子扭矩
//   output.leg_torque    → 腿部 Tp
//   output.turn_torque   → 转向差速
//   output.split_comp    → 防劈叉补偿值
//   output.K[0..11]      → 当前插值后的 K 增益
//   output.state_contrib[0..5] → 各状态对 T 的贡献 (调试用)
```

---

## 二、K 矩阵的来源与含义

### 2.1 MATLAB 原始推导

在 MATLAB `get_k_length.m` 中，对不同腿长 L0 分别做 LQR 设计:

```matlab
% 状态向量: [θ, dθ, x, v, φ, dφ]
% 控制向量: [T, Tp]
% MATLAB: u = -K * x
K = lqr(A, B, Q, R);  % 返回 2×6 矩阵
```

**MATLAB 的约定**:
- θ = 0: 腿竖直向上 (倒立摆不稳定平衡点)
- θ 增大: 腿向后摆 (逆时针, 从右侧看)
- φ > 0: 机体前倾
- T > 0: 驱动轮子前进 (x 增大)
- Tp > 0: 使 θ 增大 (腿向后摆)

### 2.2 多项式拟合 — 负号到底在哪里?

对多个 L0 值 (如 0.10~0.21m) 分别求 K, 然后对每个 K_i 用三次多项式拟合:

```
K_i(L0) = c[0]*L0³ + c[1]*L0² + c[2]*L0 + c[3]
```

#### 完整链路追踪

**Step 1: MATLAB `get_k_length.m`**
```matlab
K = lqr(A, B, Q, R);   % MATLAB 返回 K, 满足 u = -K*x
```
MATLAB 的 `lqr()` 返回的 K 使得最优控制律为 `u = -K*x`。

**Step 2: MATLAB `get_k.m`**
```matlab
k = get_k_length(i);    % 得到 K (2×6)
k11(j) = k(1,1);        % 直接取值, 没有取反!
...
a11 = polyfit(leg, k11, 3);  % 直接拟合 +K, 没有取反!
```
多项式系数 `a11` 拟合的是 **+K_matlab** (没有吸收负号)。

**Step 3: 参考代码 `chassisR_task.c`**
```c
// 多项式系数直接从 MATLAB 复制过来:
float Poly_Coefficient[12][4] = {{-213.6885, 153.3306, ...}, ...};

// K 增益插值:
LQR_K[i] = LQR_K_calc(&Poly_Coefficient[i][0], vmcr->L0);
// 得到的 LQR_K[i] = +K_matlab[i]

// 控制律:
wheel_T = LQR_K[0]*theta + LQR_K[1]*d_theta + ...  // 直接用 +K*state
```

**这看起来矛盾**: MATLAB 说 `u = -K*x`, 但参考代码用的是 `T = +K*x`。

#### 🔑 矛盾的解释

**实际上并不矛盾**, 原因在于 MATLAB 模型中 theta 的平衡点:

- MATLAB 线性化点: `theta₀ = 0` (腿竖直向上, 不稳定平衡)
- 参考代码平衡点: `theta₀ = π` (腿竖直向下, 差了 π)

在 `theta₀ = 0` 处线性化时, MATLAB 的 A 矩阵包含了重力作为"不稳定极点"。
`lqr()` 返回的 K 使得:
```
u = -K * x_deviation   (其中 x_deviation = x - x_equilibrium)
```

但在参考代码中, theta 在 π 附近运行, 偏差 `delta_theta = theta - π`。
由于 `sin(π + δ) = -sin(δ)`, `cos(π + δ) = -cos(δ)`, 在 θ=π 附近线性化的 A 矩阵
与 θ=0 处的 A 矩阵中, theta 行列的符号会翻转。

**更简洁的理解**: K_matlab 已经编码了正确的反馈方向。
在参考代码中 `T = K[0]*theta`, 当 theta 偏离 π 时:
- `theta > π` (腿偏后) → K[0] < 0 → T < 0 → 轮子减速/后退
- `theta < π` (腿偏前) → K[0] < 0 但 theta 仍 > 0 → 需看数值

**实际上, 由于 theta 在 π 附近, K[0] 的数值大小和符号已经在 MATLAB 优化中被正确确定。**
参考代码直接用 `+K*x` 而不是 `-K*x`, 是因为:
1. 多项式拟合的系数本身就编码了正确的符号关系
2. 状态变量的定义 (theta 在 π 附近而非 0 附近) 使得符号自然正确

> 💡 **总结**: 多项式系数 = +K_matlab (没有吸收负号)。
> 参考代码和我们的代码都直接写 `T = K[0]*theta + ...`。
> 这在数值上是正确的, 因为 K 的符号由 MATLAB LQR 求解器保证。
> **不要再说"系数吸收了负号"—— 它没有。**

#### 我们的代码

我们的 `DEFAULT_POLY_COEFF` 与参考代码的 `Poly_Coefficient` **完全相同** (逐个数字对比):

```c
// 我们的 full_lqr.c:
static const float DEFAULT_POLY_COEFF[12][4] = {
    {-213.6885f, 153.3306f, -50.978f, -0.13318f},  // K[0]
    ...
};

// 参考代码 chassisR_task.c:
float Poly_Coefficient[12][4] = {
    {-213.6885,  153.3306,  -50.978,  -0.13318},   // K[0]
    ...
};
```

**完全一致。** 使用方式也相同:
```c
// 我们的代码:
T = ctrl->K[0]*theta + ctrl->K[1]*d_theta + ...  // +K*state

// 参考代码:
wheel_T = LQR_K[0]*theta + LQR_K[1]*d_theta + ...  // +K*state
```

**结论: 多项式系数和 K×state 的使用方式, 我们的代码与参考代码完全一致。
差异只出现在 state 的构造上 (theta/pitch/x 等的符号处理)。**

### 2.3 K 增益的物理直觉 (以 L0=0.12m 参考代码平衡点 θ≈π 为例)

在参考代码中，theta 在 π 附近运行。以右腿为例，控制律 `T = K[0]*theta + ...`:

#### 轮子扭矩 T 的 6 个增益 K[0..5]:

| K | 状态 | L0=0.12 时 | 直觉解释 (参考代码, theta≈π) |
|---|------|-----------|------|
| K[0] | θ→T | ≈ -5.3 | theta 在 π 附近, K[0]<0, 所以 T≈K[0]*π<0 是"偏置"; 偏离时 δθ>0(后摆) → T 更负(减速追回) |
| K[1] | dθ→T | ≈ -0.8 | 后摆加速(dθ>0) → T<0 减速 (阻尼) |
| K[2] | x_err→T | ≈ -1.3 | x>x_set(前进过多) → T<0 后退修正 |
| K[3] | v_err→T | ≈ -1.3 | v>v_set(速度过快) → T<0 减速 (阻尼) |
| K[4] | pitch→T | ≈ +3.0 | 后仰(pitch>0) → T>0 前进追回重心 ✅ |
| K[5] | dφ→T | ≈ +0.3 | 后仰加速 → 前进补偿 |

> 💡 K[4] 的物理含义最直观: **后仰时轮子前进追重心**, 这是倒立摆的核心控制逻辑。
> 注意: 参考代码直接用 `K[4]*(Pitch - 0.04)`, Pitch 后仰为正, K[4]>0,
> 所以后仰 → T>0 → 前进, 符号正确。

#### 腿部扭矩 Tp 的 6 个增益 K[6..11]:

| K | 状态 | L0=0.12 时 | 直觉解释 |
|---|------|-----------|---------|
| K[6] | θ→Tp | ≈ -0.3 | theta 偏离 → Tp 修正腿部姿态 |
| K[7] | dθ→Tp | ≈ -0.3 | 阻尼 |
| K[8] | x_err→Tp | ≈ +0.3 | 前进过多 → 腿后摆(配合 T 减速) |
| K[9] | v_err→Tp | ≈ +0.2 | 速度偏大 → 腿后摆 |
| K[10] | pitch→Tp | ≈ +10.7 | 后仰 → Tp>0 腿后摆, 制造前倾力矩 |
| K[11] | dφ→Tp | ≈ +0.3 | 阻尼 |

---

## 三、坐标系与符号约定

### 3.1 全局约定

```
              机器人俯视图
              
              前进方向 →
              
         ┌─────────────┐
         │    机 身     │
    左腿  │             │  右腿
         │   IMU ●     │
         └──┬──────┬───┘
            │      │
            ●      ●  ← 轮子
         左轮    右轮
```

| 量 | 正方向 | 说明 |
|----|--------|------|
| x | 前进 | 编码器/观测器 |
| v | 前进 | dx/dt |
| pitch | 后仰 | IMU 直接读数, 鼻子朝上 > 0 |
| pitch_rate | 后仰角速度 | IMU 直接读数 |
| yaw | 逆时针(俯视) | IMU 约定 |
| T > 0 | 驱动轮前进 | 参考 MATLAB 约定 |
| Tp > 0 | 使腿向后摆 | 参考 MATLAB 约定 |

### 3.2 theta 的定义: 腿部摆角

theta 是整个腿杆 (从髋关节到轮子) 相对于某个基准的摆角。

**MATLAB/参考代码**: θ 从竖直向上量起, 逆时针为正。平衡时 θ₀ = 0 (MATLAB) 或 π (参考代码)。

**本项目**: theta 从水平向后量起 (即 body_angle + 90°), 加上 pitch 的影响。

#### 右腿 theta 的推导

```
参考代码: theta_R = π/2 - Pitch_R - phi0_R
其中:
  Pitch_R = IMU.Pitch (后仰为正)
  phi0_R  = atan2(YC, XC - l5/2), 平衡时 ≈ -π/2

所以平衡时: theta_R = π/2 - 0 - (-π/2) = π

本项目: theta_R = DEG2RAD(-pitch - body_angle - 90)
其中:
  pitch = IMU.pitch (后仰为正, 单位: 度)
  body_angle = alpha (度), 平衡时 ≈ -90°

平衡时: theta_R = DEG2RAD(0 - (-90) - 90) = DEG2RAD(0) = 0
```

关系: `theta_参考 = theta_本项目 + π`, 动态变化方向完全一致。

#### 左腿 theta 的推导

```
参考代码: theta_L = π/2 - PitchL - phi0_L
其中 PitchL = -IMU.Pitch (pitch 取反!)

本项目: theta_L = DEG2RAD(+pitch + body_angle + 90)
平衡时: theta_L = DEG2RAD(0 + (-90) + 90) = 0
```

#### d_theta 推导

```
参考代码:
  d_theta_R = -PitchGyro_R - d_phi0 = -pitch_rate - d_alpha
  d_theta_L = -PitchGyro_L - d_phi0 = -(-pitch_rate) - d_alpha = pitch_rate - d_alpha

本项目:
  d_theta_R = -pitch_rate_rad - right_dalpha_rad  ← ✅ 一致
  d_theta_L = +pitch_rate_rad + left_dalpha_rad   ← ⚠️ 见可疑点 #1
```

---

## 四、左右腿 K 矩阵作用的完整链路

K 矩阵是基于 **右腿** 推导的。左腿通过翻转部分状态变量的符号, 使同一组 K 增益能正确地控制左腿。

### 4.1 右腿 (is_left = false)

#### Step 1: 状态准备 (balance_test.c)

```c
theta       = DEG2RAD(-pitch - body_angle - 90)   // 平衡时 = 0
d_theta     = -pitch_rate_rad - dalpha_rad         // 平衡时 = 0
x           = g_lqr_distance                        // 前进为正
v           = g_lqr_speed                           // 前进为正
pitch       = DEG2RAD(imu.pitch)                    // 后仰为正
pitch_rate  = DEG2RAD(imu.pitch_rate)               // 后仰为正
```

#### Step 2: full_lqr_compute 内部符号变换

```c
// 右腿分支 (is_left = false):
x_err          = x - x_set              // 前进过多 → 正
v_err          = v - v_scale * v_set     // 速度过快 → 正
pitch_err      = -pitch - (-offset)      // = -pitch + offset
pitch_rate_err = -pitch_rate
```

**⚠️ 可疑点 #2: 右腿 pitch_err 和 pitch_rate_err 为什么取反?**

代码注释说"因为原参考代码从右边看逆时针为正"。让我们验证:

参考代码右腿:
```c
pitch_err = PitchR - 0.04   // PitchR = IMU.Pitch, 后仰为正
```

本项目右腿:
```c
pitch_err = -input->pitch - (-pitch_offset) = -pitch + offset
```

**这里 `input->pitch` = `DEG2RAD(imu.pitch)` (后仰为正)。**

所以本项目: `pitch_err = -pitch + 0.04`
参考代码: `pitch_err = pitch - 0.04`

**两者符号完全相反!** 这意味着本项目对右腿 pitch 做了额外的取反。

**为什么?** 可能的解释:
1. MATLAB 中 φ (前倾为正) 与 IMU Pitch (后仰为正) 方向相反
2. 有人可能认为多项式系数吸收了 φ→Pitch 的符号翻转, 所以需要传入 `-pitch`

**但是, 我们已经确认多项式系数 = +K_matlab, 并未吸收任何额外符号。**

**参考代码直接用 `PitchR - 0.04`, 没有取反!** 
参考代码的 `myPithR = ins->Pitch` (后仰为正), 控制律:
```c
K[4] * (myPithR - 0.04)   // 即 K[4] * (pitch - offset)
```
这说明 MATLAB 求出的 K[4] 的数值和符号, 在直接乘以 IMU 的 Pitch (后仰为正) 时, 就已经能产生正确的控制效果。

**→ 参考代码右腿: `pitch_err = pitch - offset` (后仰为正, 直接用)**
**→ 本项目右腿: `pitch_err = -pitch + offset` (额外取反了!)**
**→ 如果多项式系数与参考代码完全相同 (已确认), 则右腿不应取反。**

> 🔴 **结论**: 右腿的 `pitch_err = -pitch + offset` 和 `pitch_rate_err = -pitch_rate`
> 相对于参考代码多了一次取反。这是一个**真实的符号差异**, 不是被某处"吸收"了。
>
> **但是**, 如果当前代码在实际机器人上已经能正常平衡, 那可能有其他环节 (如 Tp 注入时的额外取反, 或者 theta 构造方式的差异) 恰好补偿了这个差异。需要实机测试验证。

#### Step 3: K×state 计算

```c
T  = K[0]*θ + K[1]*dθ + K[2]*x_err + K[3]*v_err + K[4]*pitch_err + K[5]*pitch_rate_err
Tp = K[6]*(θ + θ_set) + K[7]*dθ + K[8]*x_err + K[9]*v_err + K[10]*pitch_err + K[11]*pitch_rate_err
```

**⚠️ 可疑点 #3: Tp 的 theta 项**

T 用的是 `K[0]*theta`, 但 Tp 用的是 `K[6]*(theta + theta_set)`。

参考代码:
- 右腿: `K[6] * theta` (没有 theta_set)
- 左腿: `K[6] * (theta + theta_set)` (有 theta_set, 用于防劈叉)

本项目: **左右腿都用 `K[6]*(theta + theta_set)`**, 但 `theta_set` 默认 = 0, 所以实际效果一样。只是如果未来给 theta_set 赋非零值, 右腿也会受影响, 这与参考代码不同。

#### Step 4: 输出使用 (balance_test.c)

```c
T_right = output_right.wheel_torque;     // 直接使用
Tp_right_global = -output_right.leg_torque;  // 取反!
```

#### Step 5: Tp 注入 VMC (balance_test.c ~L1453)

```c
Tp_right_inject = Tp_right_global;       // = -output_right.leg_torque
delta_right = Tp_right_inject - F_alpha_right;
hip  += -(J_right[2] * delta_right);     // 额外取反!
knee += -(J_right[3] * delta_right);     // 额外取反!
```

**右腿 Tp 的符号链**: `output.Tp → 取反 → inject → J^T×delta → 再取反`

净效果: `hip += J[2] * (output.Tp + F_alpha)` ... 不对, 让我仔细算:

```
Tp_right_global = -output.Tp
Tp_right_inject = Tp_right_global = -output.Tp
delta = -output.Tp - F_alpha
hip += -(J[2] * delta) = -(J[2] * (-output.Tp - F_alpha))
     = J[2] * output.Tp + J[2] * F_alpha
```

而 VMC 原本已经算了 `hip += J[2] * F_alpha`, 所以总效果是:
```
hip_total = hip_vmc + J[2]*output.Tp + J[2]*F_alpha
          = (J[0]*F_L + J[2]*F_alpha) + J[2]*output.Tp + J[2]*F_alpha
          = J[0]*F_L + 2*J[2]*F_alpha + J[2]*output.Tp
```

**⚠️ 可疑点 #4: F_alpha 被加了两次!**

原意是用 Tp 替代 F_alpha: `delta = Tp - F_alpha`, 使得最终 `hip = J[0]*F_L + J[2]*Tp`。
但由于右腿的两次取反, 实际变成了 `hip = J[0]*F_L + 2*J[2]*F_alpha + J[2]*Tp`。

让我重新核实代码逻辑...

```c
// VMC 已计算: hip = J[0]*F_L + J[2]*F_alpha  (这是 VMC 的初始输出)
// 然后:
delta_right = Tp_right - F_alpha_right;
hip += -(J[2] * delta_right);
// 展开:
// hip = (J[0]*F_L + J[2]*F_alpha) + (-(J[2]*(Tp_right - F_alpha)))
// hip = J[0]*F_L + J[2]*F_alpha - J[2]*Tp_right + J[2]*F_alpha
// hip = J[0]*F_L + 2*J[2]*F_alpha - J[2]*Tp_right

// 其中 Tp_right = Tp_right_global = -output.Tp_right
// hip = J[0]*F_L + 2*J[2]*F_alpha - J[2]*(-output.Tp)
// hip = J[0]*F_L + 2*J[2]*F_alpha + J[2]*output.Tp
```

**确认: F_alpha 被加了两次, 这几乎肯定是 bug, 除非右腿的 J[2] 本身有取反来补偿。**

但根据 VMC_RIGHT_LEG_ANALYSIS.md, 右腿的 J^T 输出扭矩最终会在 `vmc_ctrl_compute` 中被取反 (因为镜像)。所以情况更复杂, 需要看完整的 VMC 输出链。

> 🔴 **这个链路非常混乱, 多处取反叠加, 容易出错。建议重构为更清晰的结构。**

---

### 4.2 左腿 (is_left = true)

#### Step 1: 状态准备

```c
theta       = DEG2RAD(+pitch + body_angle + 90)    // 平衡时 = 0
d_theta     = +pitch_rate_rad + left_dalpha_rad     // 平衡时 = 0
x, v, pitch, pitch_rate: 与右腿相同
```

**⚠️ 可疑点 #1 详解: 左腿 d_theta 的符号**

参考代码:
```c
d_theta_L = -PitchGyroL - d_phi0
// PitchGyroL = -IMU.Gyro[1] (取反)
// 所以: d_theta_L = -(-pitch_rate) - d_phi0 = pitch_rate - d_phi0
```

本项目:
```c
d_theta_left_lqr = pitch_rate_rad + left_dalpha_rad
```

参考代码是 `pitch_rate - d_phi0`, 本项目是 `pitch_rate + d_alpha`。

关键问题: **d_phi0 和 d_alpha 是什么关系?**

- `phi0 = atan2(YC, XC - l5/2)`, 平衡时 phi0 ≈ -π/2
- `alpha = π/2 - phi0`, 所以 `d_alpha = -d_phi0`

因此: `pitch_rate - d_phi0 = pitch_rate + d_alpha` ✅ **一致!**

#### Step 2: full_lqr_compute 内部符号变换

```c
// 左腿分支 (is_left = true):
x_err          = x_set - x              // 翻转: 前进过多 → 负 (与右腿相反)
v_err          = v_scale * v_set - v     // 翻转
pitch_err      = pitch - pitch_offset    // = pitch - 0.04
pitch_rate_err = pitch_rate
```

**左腿 pitch 的验证:**

参考代码左腿:
```c
PitchL = -IMU.Pitch;   // pitch 取反
pitch_err = PitchL - (-0.04) = -pitch + 0.04
```

本项目左腿:
```c
pitch_err = input->pitch - pitch_offset = pitch - 0.04
```

**input->pitch = DEG2RAD(imu.pitch)**, 后仰为正。

参考代码: `pitch_err = -pitch + 0.04`
本项目:   `pitch_err = pitch - 0.04`

**⚠️ 可疑点 #5: 左腿 pitch_err 也是反的!**

参考代码 = `-pitch + offset`, 本项目 = `+pitch - offset`。符号相反。

注释写的是 `"= -pitch + offset"`, 但代码实际上是 `pitch - offset`。**注释与代码不一致!**

**同样, 左腿 pitch_rate_err:**
- 参考代码: `PitchGyroL = -pitch_rate`, 即 `-pitch_rate`
- 本项目: `pitch_rate_err = pitch_rate` (没有取反)

参考代码 = `-pitch_rate`, 本项目 = `+pitch_rate`。**也是反的!**

> 🔴 **左腿和右腿的 pitch_err / pitch_rate_err 与参考代码都是符号相反的。**
>
> 但奇怪的是, 左腿和右腿之间是一致的:
> - 参考代码: 右腿 `pitch - offset`, 左腿 `-pitch + offset` (相反)
> - 本项目: 右腿 `-pitch + offset`, 左腿 `pitch - offset` (也是相反)
>
> 也就是说, **本项目把参考代码的"右腿"和"左腿"对 pitch 的处理交换了**。
> 左腿用了参考代码右腿的公式, 右腿用了参考代码左腿的公式。

**可能的解释**: 本项目的 theta 计算方式已经吸收了 pitch 的翻转。
- 参考代码右腿: `theta = π/2 - Pitch - phi0` (Pitch 是减号)
- 本项目右腿: `theta = -pitch - body_angle - 90` = `-(pitch + body_angle + 90)` (pitch 也是减号)

在 theta 中, 右腿的 pitch 已经被取反了。如果 pitch_err 再取反一次, 就等于"不翻转"。

**结合 theta 和 pitch_err 一起看**, 整体符号可能是对的。但这依赖于多项式系数是否正确配合。

> ⚠️ **建议**: 添加单元测试, 给一个已知的状态 (如 pitch=5°, body_angle=-90°), 分别用参考代码和本项目计算 T/Tp, 验证数值是否一致。

#### Step 3: K×state (与右腿相同的公式)

#### Step 4: 输出使用

```c
T_left = -output_left.wheel_torque;      // 取反! (因为电机安装方向)
Tp_left_global = output_left.leg_torque;  // 不取反
```

#### Step 5: Tp 注入 VMC

```c
Tp_left_inject = -Tp_left_global = -output_left.leg_torque;  // 取反!
delta_left = Tp_left_inject - F_alpha_left;
hip  += J_left[2] * delta_left;      // 不额外取反
knee += J_left[3] * delta_left;
```

左腿 Tp 链:
```
output.Tp → global (不变) → inject (取反) → delta = -Tp - F_alpha → J^T×delta
```

最终:
```
hip_total = (J[0]*F_L + J[2]*F_alpha) + J[2]*(-Tp - F_alpha)
          = J[0]*F_L + J[2]*F_alpha - J[2]*Tp - J[2]*F_alpha
          = J[0]*F_L - J[2]*Tp
```

**等一下, 这个结果意味着: F_alpha 被正确消去了, 但 Tp 是负号!**

预期的目标是: `hip = J[0]*F_L + J[2]*Tp`
实际结果是: `hip = J[0]*F_L - J[2]*Tp`

> 🟡 **可疑点 #6**: 左腿 Tp 注入后, Tp 前面有一个意外的负号。
> 这可能是故意的 (如果 LQR 输出的 Tp 方向约定与 VMC 的 F_alpha 方向相反),
> 也可能是 bug。需要结合实机行为判断。

---

## 五、可疑点汇总

| # | 位置 | 描述 | 严重性 | 建议 |
|---|------|------|--------|------|
| 1 | `balance_test.c:3727` | 左腿 `d_theta = pitch_rate + d_alpha`, 参考代码是 `pitch_rate - d_phi0`。经推导 `d_alpha = -d_phi0`, **所以实际一致**。 | ✅ 无问题 | - |
| 2 | `full_lqr.c:222-223` | 右腿 `pitch_err = -pitch + offset`, 参考代码是 `pitch - offset`。**符号相反。** 多项式系数确认与参考代码完全相同且未吸收额外负号, 所以这是一个真实差异。 | 🔴 高 | 改为 `pitch - offset`, 或验证其他环节是否补偿 |
| 3 | `full_lqr.c:241` | Tp 对 theta 的处理: `K[6]*(theta + theta_set)`, 右腿参考代码无 theta_set。默认 theta_set=0 所以无影响, 但含义不同。 | 🟡 低 | 明确 theta_set 仅用于左腿 |
| 4 | `balance_test.c:1455-1456` | 右腿 Tp 注入 VMC 时有额外取反 `-(J[2]*delta)`, 导致 F_alpha 被加了两次而非替代。 | 🔴 高 | 重新推导右腿 Tp 注入逻辑 |
| 5 | `full_lqr.c:215-216` | 左腿 `pitch_err = pitch - offset`, 注释写 `"= -pitch + offset"`, **注释与代码不一致**, 且与参考代码符号相反 (参考代码左腿: `-pitch + offset`)。 | 🔴 高 | 改为 `-pitch + offset`, 或验证其他环节 |
| 6 | `balance_test.c:1447-1449` | 左腿 Tp 注入: 最终效果是 `hip = J[0]*F_L - J[2]*Tp`, Tp 前有意外负号。 | 🟡 中 | 需要确认 Tp 方向约定 |
| 7 | `balance_test.c:3811` | `Tp_right_global = -output.Tp`, 注释说"由于右腿和左腿的符号相反"。但在 `full_lqr_compute` 内部, 左腿已经做了符号翻转。这里再取反可能是双重翻转。 | 🟡 中 | 梳理完整链路确认 |

---

## 六、转向差速与防劈叉

### 6.1 转向

```c
// full_lqr.c 内部:
turn_torque = Kp * (turn_set - yaw_total) - Kd * yaw_rate;

// balance_test.c 上层:
T_left  += turn_T;
T_right -= turn_T;
```

参考代码: `wheel_T_right -= turn_T; wheel_T_left -= turn_T;` (两轮同减)

**区别**: 参考代码两轮同减是因为两轮电机镜像安装 (同减 = 物理差速)。
本项目两轮电机同向安装, 所以一加一减 = 物理差速。**逻辑正确。**

### 6.2 防劈叉

```c
// full_lqr.c:
theta_err = input->theta_left - input->theta_right;  // 纯几何角之差
split_comp = Kp * theta_err;
Tp += split_comp;
```

参考代码:
```c
theta_err = 0 - (right.theta + left.theta);  // 含 pitch 的 theta 之和
```

**区别**: 
- 参考代码: 两腿 theta (含 pitch) 之和趋零
- 本项目: 两腿纯几何角之差趋零

本项目的方式避免了 pitch 的耦合干扰, 可能更合理。但注意:
- 两者 theta_err 的定义不同, 所以 Kp 的最优值也不同
- 参考代码的 theta 平衡值 = π, 两者之和 = 2π ≠ 0, 需要 theta_set 补偿

### 6.3 离地保护

```c
if (off_ground) {
    T = K[0]*theta + K[1]*d_theta;  // 仅保留 theta 项
    Tp = K[6]*theta + K[7]*d_theta + split_comp;
    T = 0.0f;  // ← 然后又清零了!
}
```

**⚠️ 可疑点 #8**: T 先算了一个值, 然后立刻被清零。第一行计算是多余的。代码逻辑正确 (离地时轮子不出力), 但写法容易误导。

---

## 七、完整的变量方向总结表

### 7.1 传入 full_lqr_compute 前 (观测方向)

| 变量 | 右腿传入值 | 左腿传入值 | 方向说明 |
|------|-----------|-----------|---------|
| theta | -pitch - body_angle - 90 | +pitch + body_angle + 90 | 平衡时=0, pitch 符号相反 |
| d_theta | -pitch_rate - d_alpha | +pitch_rate + d_alpha | pitch_rate 符号相反 |
| x | g_lqr_distance | g_lqr_distance | 相同, 前进为正 |
| v | g_lqr_speed | g_lqr_speed | 相同, 前进为正 |
| pitch | DEG2RAD(imu.pitch) | DEG2RAD(imu.pitch) | 相同, 后仰为正 |
| pitch_rate | DEG2RAD(imu.pitch_rate) | DEG2RAD(imu.pitch_rate) | 相同, 后仰为正 |

### 7.2 full_lqr_compute 内部变换后 (送入 K×state)

| 变量 | 右腿 | 左腿 | 参考代码右腿 | 参考代码左腿 |
|------|------|------|------------|------------|
| theta | 原样 | 原样 | 原样 | 原样 |
| d_theta | 原样 | 原样 | 原样 | 原样 |
| x_err | x - x_set | x_set - x | x - x_set ✅ | x_set - x ✅ |
| v_err | v - v_s*v_set | v_s*v_set - v | v - v_s*v_set ✅ | v_s*v_set - v ✅ |
| pitch_err | **-pitch + offset** | **+pitch - offset** | pitch - offset ❌ | -pitch + offset ❌ |
| pitch_rate_err | **-pitch_rate** | **+pitch_rate** | pitch_rate ❌ | -pitch_rate ❌ |

> 🔴 **关键发现**: pitch_err 和 pitch_rate_err 在左右腿之间的分配与参考代码完全对调了。
> 本项目右腿用了参考代码左腿的公式, 左腿用了参考代码右腿的公式。

### 7.3 控制输出方向 (K×state 之后)

| 输出 | 含义 | 右腿后处理 | 左腿后处理 |
|------|------|-----------|-----------|
| T (wheel_torque) | 轮子扭矩 | 直接使用 | **取反** (电机方向) |
| Tp (leg_torque) | 腿部扭矩 | **取反** → 再经 VMC 额外取反 | 不取反 → 经 VMC **取反**注入 |
| turn_torque | 转向差速 | T_right -= turn | T_left += turn |

---

## 八、给调试者的建议

1. **最关键的验证**: 在静止平衡状态下, 施加一个小扰动 (如手推一下), 观察 T 和 Tp 的方向是否正确:
   - 向前推 (pitch 变负/前倾): T 应该变负(后退追回), Tp 应该使腿前摆
   - 向后拉 (pitch 变正/后仰): T 应该变正(前进追回)

2. **pitch_err 符号验证**: 在 #FLQR 数据流中, 观察 `state_contrib[4]` (K[4]*pitch_err):
   - 后仰时 state_contrib[4] 应该为正 (驱动前进)
   - 如果为负, 说明 pitch_err 符号可能有问题

3. **Tp 注入验证**: 暂时关闭 Tp (设 `max_tp = 0`), 只用 T 做平衡控制。如果平衡正常, 再开启 Tp 观察是否改善。如果开启 Tp 后反而不稳定, 检查 Tp 注入链路的符号。

4. **单独测试防劈叉**: 设置较大的 `split_kp`, 手动拨动一条腿, 观察另一条腿是否做出对称修正。

---

*文档创建于 2026-03-20, 基于 `full_lqr.c` 和 `balance_test.c` 的代码分析。*
*所有"可疑点"需要结合实机测试验证, 不能仅凭代码分析得出最终结论。*

---

## 附录 A: 多项式系数负号问题的最终结论

这是一个容易产生误解的核心问题, 在此做最终澄清。

### 链路追踪

```
MATLAB lqr(A,B,Q,R)
    │  返回 K (满足 u = -K*x 的最优增益)
    ▼
get_k.m: polyfit(leg, k_ij, 3)
    │  直接拟合 +K, 没有取反
    ▼
chassisR_task.c: Poly_Coefficient[12][4] = {...}
    │  数值 = polyfit 的输出 = +K_matlab
    ▼
我们的 full_lqr.c: DEFAULT_POLY_COEFF[12][4] = {...}
    │  数值与参考代码完全相同 (逐个对比确认)
    ▼
控制律: T = K[0]*theta + K[1]*d_theta + ...
    │  直接 +K * state, 没有额外负号
    ▼
参考代码也是: wheel_T = LQR_K[0]*theta + ...
              (同样没有负号)
```

### 为什么 +K*x 而不是 -K*x?

MATLAB 的 `u = -K*x` 是标准教科书写法, 其中 x 是**偏差** (state - equilibrium)。

在参考代码中:
- theta 平衡值 = π (不是 0)
- 如果严格写偏差: `u = -K * (theta - π) = -K*theta + K*π`
- 但参考代码写的是: `T = K*theta` (没有 -π 项)

**这不矛盾**, 因为:
1. K*π 是一个常数偏置, 在平衡时被抵消 (平衡时 T=0, 所有项的和为零)
2. 关键是**动态响应**: 当 theta 增大 δ, T 的变化量 = K[0]*δ
3. K[0] < 0, 所以 theta 增大 → T 减小 → 轮子减速 → 正确的负反馈

**简言之**: `+K*state` 和 `-K*deviation` 在动态上是等价的, 因为常数项在平衡时抵消。

### 最终结论

**poly_coeff = +K_matlab。代码中 T = +K*state。不存在"吸收负号"。**
本项目与参考代码在系数和公式结构上完全一致, 差异仅在 state 的构造上 (pitch_err 符号等)。
