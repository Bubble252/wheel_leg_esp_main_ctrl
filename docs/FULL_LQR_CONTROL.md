# 完整 LQR 控制模式 (Full LQR)

> 添加日期: 2026-03-14  
> 参考项目: DM-balance 五连杆轮足机器人

---

## 一、概述

Full LQR 是在原有简易 LQR (4环PID级联) 基础上新增的 **6状态完整 LQR 控制器**。

### 与简易 LQR 的主要区别

| 特性 | 简易 LQR (CTRL_MODE_LQR) | 完整 LQR (CTRL_MODE_FULL_LQR) |
|------|--------------------------|-------------------------------|
| 状态维度 | 4 (角度/角速度/位移/速度) | 6 (+theta/d_theta 腿摆角) |
| 控制方式 | 4个PID级联 | 真正的状态反馈 K×x |
| 轮子扭矩 T | ✅ | ✅ |
| 腿部扭矩 Tp | ❌ (VMC独立控制) | ✅ (LQR计算Tp, 注入VMC的J^T) |
| K增益 | 固定PID参数 | 随腿长L0实时插值 (三次多项式) |
| 防劈叉 | VMC sync (PD on angle_diff) | LQR内建 Tp_Pid (PD on theta_diff) |

---

## 二、状态向量

```
x = [theta, d_theta, x, v, pitch, pitch_rate]  (6维)
```

| 状态 | 符号 | 说明 | 来源 |
|------|------|------|------|
| theta | θ | 腿部摆角 (rad) | pitch + body_angle + 90° |
| d_theta | dθ | 腿部摆角速度 (rad/s) | pitch_rate + body_angle_rate |
| x | x | 机器人位移 (m) | 编码器/观测器 |
| v | v | 机器人速度 (m/s) | 编码器/观测器 |
| pitch | φ | 机身俯仰角 (rad) | IMU |
| pitch_rate | dφ | 机身俯仰角速度 (rad/s) | IMU |

---

## 三、控制输出

### 3.1 轮子扭矩 T
```
T = K[0]*θ + K[1]*dθ + K[2]*x_err + K[3]*v_err + K[4]*φ_err + K[5]*dφ
```
直接发送给轮毂电机 (扭矩模式)。

### 3.2 腿部摆动扭矩 Tp
```
Tp = K[6]*θ + K[7]*dθ + K[8]*x_err + K[9]*v_err + K[10]*φ_err + K[11]*dφ
Tp += split_comp  (防劈叉补偿)
```
通过 VMC 的雅可比矩阵转换为关节扭矩, **完全替代** VMC 原有的 F_alpha:
```
delta = Tp - F_alpha  (先减去 VMC 已计算的 F_alpha, 再加上 Tp)
τ_hip  += J[2] * delta
τ_knee += J[3] * delta
```
最终效果等价于: τ = J^T × [F_L; Tp] (F_L 仍由 VMC 控制, F_alpha 被 Tp 替代)

### 3.3 转向差速
```
turn_T = Kp * (turn_set - yaw_total) - Kd * yaw_rate
left_T  += turn_T
right_T -= turn_T
```

---

## 四、K 增益插值

12 个 K 增益根据当前腿长 L0 实时计算 (三次多项式拟合):

```
K_i(L0) = c[0]*L0³ + c[1]*L0² + c[2]*L0 + c[3]
```

默认多项式系数来自 DM-balance 项目 (适用于其机械结构, 需根据实际机器人重新拟合):

| K[i] | 作用 | c[0] | c[1] | c[2] | c[3] |
|------|------|------|------|------|------|
| K[0] | θ→T | -213.69 | 153.33 | -50.98 | -0.13 |
| K[1] | dθ→T | -1.14 | 1.25 | -3.63 | 0.06 |
| K[2] | x→T | -82.31 | 49.84 | -10.67 | -0.73 |
| K[3] | v→T | -70.35 | 43.31 | -10.20 | -0.65 |
| K[4] | φ→T | -246.36 | 173.91 | -47.66 | 6.13 |
| K[5] | dφ→T | -13.19 | 10.23 | -3.17 | 0.52 |
| K[6] | θ→Tp | 114.43 | -51.76 | 2.83 | 2.60 |
| K[7] | dθ→Tp | 14.42 | -8.56 | 1.62 | 0.13 |
| K[8] | x→Tp | -154.00 | 107.39 | -28.83 | 3.50 |
| K[9] | v→Tp | -128.91 | 90.02 | -24.30 | 3.04 |
| K[10] | φ→Tp | 577.61 | -351.56 | 75.96 | 4.24 |
| K[11] | dφ→Tp | 46.46 | -29.02 | 6.54 | 0.06 |

---

## 五、左右腿符号约定

参考代码中, K 增益是基于右腿推导的。左腿通过翻转部分状态符号实现对称:

| 状态分量 | 右腿 | 左腿 |
|----------|------|------|
| theta | θ | θ (相同) |
| d_theta | dθ | dθ (相同) |
| x_err | x - x_set | x_set - x |
| v_err | v - v_scale*v_set | v_scale*v_set - v |
| pitch_err | pitch - offset | -pitch + offset |
| pitch_rate | pitch_rate | -pitch_rate |

---

## 六、Tp 与 VMC 的集成

在 `vmc_compute_leg_state()` 中, Full LQR 的 Tp 在 `vmc_dual_compute()` 之后注入:

```
VMC 计算: F_L (弹簧阻尼+重力补偿) + F_alpha (身体角度PD)
         → J^T × [F_L; F_alpha] → hip_torque, knee_torque

Full LQR Tp 替代 F_alpha:
         delta = Tp - F_alpha
         hip_torque  += J[2] * delta
         knee_torque += J[3] * delta
```

这意味着:
- VMC 继续负责 **腿长控制** (F_L: 弹簧阻尼 + 重力补偿)
- VMC 的 F_alpha (身体角度 PD) 被 Full LQR 的 Tp **完全替代** (不是叠加)
- 最终关节扭矩 = J^T × [F_L; Tp], 其中 F_L 来自 VMC, Tp 来自 Full LQR

---

## 七、CLI 命令

```bash
# 切换到 Full LQR 模式
balance mode flqr

# 查看 Full LQR 参数和当前 K 增益
balance flqr

# 开启/关闭 #FLQR 数据流 (Qt 调参用)
balance flqr stream on
balance flqr stream off

# 调整 pitch 零点偏移 (rad)
balance flqr pitch_offset 0.04

# 调整目标速度缩放 (参考代码用 0.4)
balance flqr v_scale 0.4

# 调整轮子扭矩限幅 (Nm)
balance flqr max_t 2.0

# 调整腿部扭矩 Tp 限幅 (Nm)
balance flqr max_tp 8.0

# 调整防劈叉 PD 参数
balance flqr split <kp> <kd> [limit]

# 调整转向 PD 参数
balance flqr turn <kp> <kd> [limit]

# 查看/修改多项式系数
balance flqr coeff           # 查看全部 12×4 系数
balance flqr coeff 0         # 查看第0行
balance flqr coeff 0 -213.69 153.33 -50.98 -0.13   # 修改第0行

# 切回简易 LQR
balance mode lqr
```

---

## 八、#FLQR 数据流格式

```
#FLQR,T_left,T_right,Tp_left,Tp_right,L0,theta_deg,d_theta_deg,x,v,pitch,pitch_rate,split_comp
```

| 字段 | 说明 | 单位 |
|------|------|------|
| T_left | 左轮扭矩 | Nm |
| T_right | 右轮扭矩 | Nm |
| Tp_left | 左腿 Tp | Nm |
| Tp_right | 右腿 Tp | Nm |
| L0 | 平均腿长 | m |
| theta_deg | 腿部摆角 | 度 |
| d_theta_deg | 腿部摆角速度 | 度/s |
| x | 位移 | m |
| v | 速度 | m/s |
| pitch | 俯仰角 | 度 |
| pitch_rate | 俯仰角速度 | 度/s |
| split_comp | 防劈叉补偿 | Nm |

---

## 九、调试建议

1. **首次使用**: 先在简易 LQR 或 Triple PID 模式下验证平衡正常
2. **切换模式**: `balance mode flqr`, 观察 `#FLQR` 数据流
3. **减小 max_t**: 先从小值 (如 0.5 Nm) 开始, 逐步增大
4. **检查 K 增益**: `balance flqr` 查看当前 L0 下的 K 值是否合理
5. **Tp 调试**: 可先将 `max_tp 0` 禁用 Tp, 仅测试 T (轮子扭矩)
6. **多项式系数**: 默认系数来自参考项目, 需根据实际机器人的质量、惯量、尺寸重新拟合
