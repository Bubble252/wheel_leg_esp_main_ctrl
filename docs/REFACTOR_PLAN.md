# Balance Test 重构计划

> 创建日期: 2026-05-19
> 状态: Phase 2+3 完成
> 关联分支: feature/lqr-optimize

---

## 一、问题分析

### 1.1 现状

`balance_test.c` 从原始 **8418 行** 重构为 **2899 行**，拆分为 10 个独立文件。

| 指标 | 重构前 | 当前 |
|---|---|---|
| balance_test.c 行数 | 8418 | **2899** |
| balance_cli.c | - | 2545 |
| balance_control.c | - | 1486 |
| balance_roll.c | - | 107 |
| balance_jump.c | - | 234 |
| balance_standup.c | - | 448 |
| balance_observer.c | - | 276 |
| balance_vmc.c | - | 240 |
| balance_plot.c | - | 389 |
| balance_leg.c | - | 338 |
| balance_types.h (共享类型) | - | 115 |
| extern 变量 | 0 | ~130 |
| 重复代码块 (Roll+X-Offset+IK) | 4 处 | **0 处** |

### 1.2 具体问题

**P0 - 致命:**
- 单文件 8418 行，无法有效 review，改一处怕崩全局
- `compute_balance_output()` 是巨型 if/else 分支树 (5种控制模式)，逻辑嵌套深
- Roll + X-Offset + IK 代码块复制粘贴 4 次，改一处需要同步 4 处

**P1 - 严重:**
- 211 个散落的 `static g_xxx` 全局变量，状态管理混乱
- 5 个状态机 (跳跃/起身/小车起身/BAL→CAR/主状态) 共享全局变量，互相直接改对方状态
- CLI 命令处理 2600 行纯 if/else 瀑布

**P2 - 中等:**
- `lqr_params_t` 包含 50+ 个 PID 参数全部平铺，没有层级
- 延迟诊断代码 ~30 个全局变量未封装
- 调试输出变量 (g_zp_xxx, g_roll_xxx 等) 与业务逻辑混杂

### 1.3 参考项目

`/home/bubble/无线imu/vqf_esp32_dist` 项目的组件设计值得借鉴:
- 每组件 <500 行，职责单一
- 通过回调解耦模块间依赖
- 配置集中在独立头文件
- main.c 只做初始化 + 任务启动

---

## 二、重构目标

1. **每个源文件 <500 行**，职责单一
2. **消灭重复代码** — Roll+X-Offset+IK 只写一次
3. **全局变量按功能封装为结构体**
4. **状态机独立** — 每个状态机在自己的文件里
5. **CLI 改用命令注册表**
6. **重构后行为完全不变** — 纯结构重组，不改控制逻辑

---

## 三、目标结构

```
components/balance/               ← 新组件目录
├── CMakeLists.txt
├── include/
│   ├── balance_manager.h         主状态机 + 任务管理接口
│   ├── balance_config.h          控制模式 / 任务周期 / 安全限幅等配置
│   └── balance_shared.h          跨子模块共享类型和 extern 声明
│
└── src/
    ├── balance_manager.c         主状态机 + 任务创建/销毁 (~300行)
    │                             包含: balance_test_init/start/stop/enable/disable
    │                                   task_unified_control, update_remote_from_wifi
    │
    ├── balance_control.c         控制模式分发 + 核心计算 (~400行)
    │                             包含: compute_balance_output (拆解后)
    │                                   各模式的控制输入构造和输出合成
    │
    ├── balance_roll.c            Roll 控制 + X-Offset (~150行)
    │                             包含: apply_roll_and_xoffset() — 消灭4处重复
    │                                   Roll 闭环 PID, X-Offset PID
    │
    ├── balance_leg.c             腿部位置控制 + Leg Sync (~250行)
    │                             包含: leg_ctrl_init/set_target/set_both
    │                                   Leg Sync 防劈叉补偿
    │                                   关节电机速度滤波
    │
    ├── balance_jump.c            跳跃状态机 (~300行)
    │                             包含: jump_state_machine_update
    │                                   MIT 蹬伸参数
    │
    ├── balance_standup.c         起身相关状态机 (~400行)
    │                             包含: standup_state_machine_update
    │                                   car_standup_state_machine_update
    │                                   bal_to_car_state_machine_update
    │
    ├── balance_cli.c             CLI 命令处理 (~400行)
    │                             包含: balance_test_process_cmd
    │                                   命令注册表 + 各命令处理函数
    │
    ├── balance_plot.c            波形输出 + PID 调试 (~200行)
    │                             包含: output_plot_data, output_pid_debug
    │                                   通道掩码管理
    │
    ├── balance_observer.c        卡尔曼速度观测器 (~200行)
    │                             包含: velocity_observer_update
    │                                   KF 状态, 加速度零偏校准
    │
    ├── balance_vmc.c             VMC 力控 (~200行)
    │                             包含: vmc_compute_leg_state
    │                                   支持力估计, 离地检测
    │
    └── balance_diagnostics.c     延迟测量 + 统计 (~200行)
                                  包含: latency_profiler_t
                                        balance_test_get_stats
                                        频率统计
```

原有 `components/app/` 中的 `balance_test.c` 和 `balance_test.h` 在重构完成后删除，
由 `balance_manager.h` 提供对外的统一接口。

---

## 四、重构步骤

### Phase 1: 拆分最独立的模块 (风险最低)

> **策略调整**: 原计划直接拆分为独立组件 (components/balance/)，但发现 static 全局变量跨文件访问问题。
> 改为先在 balance_test.c 内部提取辅助函数消灭重复，验证编译通过后再做文件拆分。

#### Step 1.1 — 在 balance_test.c 内部消灭 Roll+X-Offset+IK 重复代码 ✅

- [x] 提取 `apply_xoffset_and_ik()` 辅助函数 (640-686行) — 统一 X-Offset 笛卡尔偏移 + 腿长限幅 + 工作空间限幅 + 逆运动学
- [x] 提取 `apply_xoffset_single()` 辅助函数 (691-724行) — 对称腿的简化版本 (Roll 未启用时)
- [x] 替换 Full LQR 模式 Roll 块 (~5207行) → `apply_xoffset_and_ik()`
- [x] 替换 LQR 模式 Roll 块 (~5337行) → `apply_xoffset_and_ik()`
- [x] 替换 LQR 模式无 Roll 块 (~5357行) → `apply_xoffset_single()`
- [x] 替换 非 LQR 模式 Roll 块 (~5460行) → `apply_xoffset_and_ik()`
- [x] 替换 非 LQR 模式无 Roll 块 (~5516行) → `apply_xoffset_single()`
- [x] **验证: `idf.py build` 编译通过，无新增 warning**

**效果**: 4处重复代码 (每处~60行) → 2个辅助函数 (共~85行)，净减 ~78 行，8418→8340

#### Step 1.2 — 跳跃状态机代码审查 ✅ (无需改动)

- [x] 审查跳跃状态机代码组织
- 结论: 跳跃相关代码已良好组织 — 变量集中在 465-492 行，函数在 3258-3396 行，有清晰的分区注释
- 暂不需要在文件内进一步整理，等 Phase 2 文件拆分时再提取

#### Step 1.3 — 延迟诊断代码审查 ✅ (无需改动)

- [x] 审查延迟诊断代码组织
- 结论: 诊断变量集中在 275-304 行，CLI 输出在 5767-5807 行，命名规范 (g_latency_*)
- 暂不需要在文件内进一步整理，等 Phase 2 文件拆分时再提取

#### Step 1.4 — 创建 balance 组件骨架 ✅ (部分完成)

- [x] 创建 `components/balance/` 目录结构
- [x] 创建 `CMakeLists.txt`
- [x] 创建 `balance_roll.h` / `balance_roll.c` — 移入 `apply_xoffset_and_ik()` 和 `apply_xoffset_single()`
- [x] 在 balance_test.c 中将 10 个共享变量从 `static` 改为 extern 可见
- [x] **验证: `idf.py build` 编译通过**
- [ ] ~~将跳跃状态机移入 `balance_jump.c`~~ (暂缓: 跳跃状态机依赖 `leg_ctrl_set_target` 等 static 函数，交叉依赖复杂)
- [ ] ~~将延迟诊断移入 `balance_diagnostics.c`~~ (暂缓: 延迟计算内联在控制循环中，拆分需大量修改)

---

### Phase 2: 拆分状态机和 CLI

#### Step 2.1 — 拆出 balance_standup.c ✅

- [x] 提取起身状态机: `STANDUP_IDLE → RETRACT → LEFT_ROLL → LEFT_EXTEND → WAIT → RIGHT_ROLL → DONE`
- [x] 提取小车起身状态机: `CAR_STANDUP_IDLE → SWING → DONE`
- [x] 提取 BAL→CAR 过渡状态机: `BAL_TO_CAR_IDLE → RETRACT → TILT → SETTLE`
- [x] 封装各自的状态变量为结构体
- [x] **验证: `idf.py build` 编译通过**

#### Step 2.2 — 拆出 balance_cli.c ✅

- [x] 将 2326 行 CLI 代码移至 `components/app/src/balance_cli.c`
- [x] 81 个 static 变量改为 extern，在 CLI 文件中 extern 声明
- [x] 保留原有 if/else 结构 (命令注册表留待 Phase 4)
- [x] **验证: `idf.py build` 编译通过**

#### Step 2.3 — 拆出 balance_plot.c ✅

- [x] 提取波形输出: `output_plot_data()`, 通道掩码管理
- [x] 提取 PID 调试输出: `output_pid_debug()`
- [x] 提取 `plot_ch_bit()` 和 `PLOT_CH_ENABLED` 宏到 `balance_plot.h`
- [x] **验证: `idf.py build` 编译通过**

---

### Phase 3: 拆分核心控制逻辑

#### Step 3.1 — 拆出 balance_observer.c ✅

- [x] 提取卡尔曼滤波器: `velocity_observer_update()`
- [x] 提取观测器任务: `task_observer()`
- [x] **验证: `idf.py build` 编译通过**

#### Step 3.2 — 拆出 balance_vmc.c ✅

- [x] 提取 VMC 力控计算: `vmc_compute_leg_state()`
- [x] 提取支持力估计: `compute_support_force()`
- [x] 提取离地检测 (轮加速度 + 支持力 双重检测)
- [x] **验证: `idf.py build` 编译通过**

#### Step 3.3 — 拆出 balance_leg.c ✅

- [x] 提取腿部位置控制: `leg_ctrl_init/set_target/set_both/get_state`
- [x] 提取腿部状态打印: `leg_ctrl_print_status()`
- [x] **验证: `idf.py build` 编译通过**

#### Step 3.4 — 拆分 balance_control.c ✅

- [x] 提取 `compute_balance_output()` (1066 行) 到 `components/app/src/balance_control.c`
- [x] 提取 `apply_motor_commands()` (152 行)
- [x] 23 个 static 变量改为 extern
- [x] **验证: `idf.py build` 编译通过**

#### Step 3.5 — balance_manager.c (待做)

- [ ] 提取主状态机: `BALANCE_TEST_IDLE → READY → RUNNING → EMERGENCY`
- [ ] 提取任务创建/销毁: unified task, 分离 tasks
- [ ] 提取 `update_remote_from_wifi()`, Commander 回调
- [ ] **验证: 完整启动/停止/使能/紧急停止流程正常**

---

### Phase 4: 结构优化 (锦上添花)

#### Step 4.1 — 全局变量封装

- [ ] Roll 相关 → `roll_control_state_t`
- [ ] 延迟诊断 → `latency_profiler_t`
- [ ] 跳跃 → `jump_state_machine_t`
- [ ] 起身 → `standup_state_machine_t`
- [ ] 观测器 → `observer_state_t`
- [ ] Yaw → `yaw_control_state_t`

#### Step 4.2 — 控制模式策略化 (可选)

```c
typedef struct {
    void (*init)(void);
    void (*compute)(const lqr_input_t *in, lqr_output_t *out, float dt);
    void (*reset)(void);
} control_strategy_t;

// 运行时切换模式只改函数指针
static control_strategy_t *g_active_strategy;
```

#### Step 4.3 — 删除旧文件

- [ ] 确认所有功能迁移完成后，删除 `components/app/src/balance_test.c`
- [ ] 更新 `components/app/include/balance_test.h` → redirect 到 `balance_manager.h`
- [ ] 更新 `app_main.c` 的 include

---

## 五、测试验证清单

每完成一个 Step，必须验证:

### 基本功能
- [ ] `balance init` → `balance start` → `balance enable` 正常启动
- [ ] 倒立平衡稳定 (各控制模式分别测试)
- [ ] 遥控器转向响应正常
- [ ] Roll 控制正常 (左倾右倾时腿长对称调节)
- [ ] 腿长/角度调节正常

### 状态机
- [ ] 跳跃: 下蹲 → 起跳 → 落地恢复
- [ ] 起身: 从趴着站起来
- [ ] 小车模式: balance → car → balance 切换
- [ ] 紧急停止和恢复

### 调试功能
- [ ] `balance plot on/off` 波形输出
- [ ] `balance latency` 延迟统计
- [ ] `balance freq` 频率统计
- [ ] Commander 调参正常
- [ ] WiFi Web UI 调参正常

### 安全性
- [ ] 倾斜超过 45° 触发紧急停止
- [ ] 离地检测正常 (轮速加速度 + 支持力)
- [ ] 电机力矩限幅正常

---

## 六、风险控制

1. **每步只做一件事** — 不混合重构和功能修改
2. **每步都上机测试** — 控制代码容错率低
3. **保持 git 可回退** — 每个步骤一个 commit，方便 revert
4. **先拆最独立的模块** — 跳跃/CLI/诊断不参与核心平衡循环，风险最低
5. **最后动核心** — `compute_balance_output()` 放在最后拆，此时已经积累了经验

---

## 七、参考资料

- 参考: `/home/bubble/无线imu/vqf_esp32_dist` — 组件划分、回调解耦
- 参考: `shibo_wheel_leg` — 原始控制算法
- 相关文档: `docs/SIGN_CONVENTIONS.md` — 符号约定
- 相关文档: `docs/VMC_LEG_CONTROL.md` — VMC 腿部控制
- 相关文档: `docs/FULL_LQR_CONTROL.md` — Full LQR 控制
