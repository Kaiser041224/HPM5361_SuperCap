# HPM5361 SuperCap

面向 RoboMaster 比赛的超级电容控制器。基于 HPMicro HPM5361 (RISC-V, 480MHz)，通过四开关 Buck-Boost 变换器管理 VIN / VOUT / VCAP 三端口能量，以**恒功率 (CW) 削峰填谷**策略解耦底盘瞬时功率波动，实现裁判系统侧输入功率的平滑可控。

---

## 设计概览

### 三端口拓扑

```text
   VIN ──[IIN 检流电阻]── VOUT ──[四开关 Buck-Boost]── VCAP (超级电容电容组)
          (2mΩ)            │
                     底盘负载 (瞬时功率波动)
```

- **VIN**：裁判系统供电端口，当前硬件未直接采样。
- **VOUT**：超级电容控制器输出电压，与 VIN 仅通过 IIN 检流电阻相连，直接挂接底盘负载。
- **VCAP**：超级电容电容组电压，位于 Buck-Boost 的 cap 侧。
- **IIN**：VIN → VOUT 检流电阻上的电流 (`INA241A3, 50V/V`)，用于 CW 功率闭环反馈。

### 核心策略：恒功率削峰填谷

| 工况 | 行为 |
|------|------|
| 底盘功率需求较低 | Buck-Boost 充电：VOUT → VCAP，向超级电容电容组储能 |
| 底盘功率瞬时激增 | Buck-Boost 放电：VCAP → VOUT，超级电容释放能量补给负载 |
| 稳态 | CW 功率环调节充/放平衡，保持 VIN 端口**输入功率恒定** |

通过上述削峰填谷，VIN 端口的功率纹波被超级电容吸收，裁判系统观测到的功率保持平稳，避免了因瞬时超功率触发判罚。

### 控制级联结构

```text
CW 功率外环 (25kHz): PI(p_target, p_in) → v_out_target_cmd
    │
    ▼
VOUT 电压外环 + cap 侧 CC 竞争环 (50kHz): PI(v_out_target, vout) ⟷ PI(i_cap_target, i_cap_est)
    │  (CV/CC 竞争输出较保守的 current_ref)
    ▼
电感电流内环 (200kHz): PI(current_ref, i_l) → V_L_cmd → VOUT/VCAP 前馈 → generalized_duty → DA, DB
```

- **CW 外环**：增量式 PI，输出为有符号 VOUT 电压目标，双向控制充电/放电。
- **电压/Cap CC 竞争环**：VOUT 电压环与 cap 侧限流环取更保守的电流命令，防止过充或过放。
- **电流内环**：增量式 PI 输出平均电感电压命令，由 VOUT/VCAP 前馈转换为归一化占空比 `generalized_duty`，再经单输入调制器生成 A/B 半桥 PWM 占空比。
- **VCAP 动态限幅**：仅在 cap 侧进入过压区时收缩 `generalized_duty` 上限，低压区释放以保持最大调制性能。

### 物理量约定

| 名称 | 含义 | 用途 |
|------|------|------|
| VOUT | 超级电容控制器输出电压，底盘负载供电 | 电压环反馈、功率计算、Buck-Boost 前馈 |
| VCAP | 超级电容电容组电压 | cap 侧监测、限幅参考、Buck-Boost 前馈 |
| I_IN | VIN→VOUT 输入电流，按硬件正方向换算 | CW 环输入功率/电流反馈 |
| I_L | 电感电流 | 电流内环反馈；cap 侧充/放电电流由 I_L 与占空比估算，仅用于 CC 竞争环 |
| VIN | 系统总输入电压 | 当前硬件未采样；VIN 与 VOUT 仅通过 IIN 检流电阻连接 |

---

## 快速开始

### 环境约定

| 项目 | 默认路径 |
|------|----------|
| 当前工程 | `/workspace/HPM5361_SuperCap` |
| HPM SDK | `/workspace/hpm_sdk` |
| OpenOCD | `/workspace/tools/openocd-hpm/install/bin/openocd` |
| RISC-V 工具链 | `/opt/riscv32-gnu-toolchain-elf-bin/bin` |

### 常用命令

```bash
make build
make artifacts
make flash
make clean
```

`make artifacts` 会把 HPM SDK 默认生成的 `build/output/demo.*` 复制为当前工程名产物：

```text
output/HPM5361_SuperCap.elf
output/HPM5361_SuperCap.bin
output/HPM5361_SuperCap.asm
output/HPM5361_SuperCap.map
```

VS Code F5 调试会自动执行：

```bash
make artifacts DEBUG_PATH_MODE=container
```

---

## 硬件与调试

| 项目 | 配置 |
|------|------|
| MCU | HPM5361 |
| 默认板级目录 | `Board/HPM5361_SuperCap_board` |
| 默认板级配置 | `HPM5361_SuperCap_board.yaml` / `HPM5361_SuperCap_board.cfg` |
| OpenOCD SoC 配置 | `soc/hpm5300.cfg` |
| 默认 Probe | CMSIS-DAP / DAPLink |
| J-Link Device | `HPM5361xEGx` |

调试配置位于：

- `.vscode/launch.json`：J-Link RTT 与 OpenOCD DAPLink 两套 Cortex-Debug 配置
- `.vscode/tasks.json`：F5 前置构建、芯片 reset 任务
- `doc/build_flash_debug_guide.md`：编译、下载、路径映射与调试说明

---

## 软件架构

工程采用 **App / Interface / Driver / Board** 解耦结构，App 层不直接包含 `hpm_*` 头文件。

```text
Application -> Control -> Platform -> Interface -> Driver -> Board
            \-> Platform -> Interface -> Driver -> Board
Control     -> Algorithm
Debug       -> Platform / Control / Interface
```

### 目录结构

```text
HPM5361_SuperCap/
├── App/
│   ├── Application/      # 应用编排：入口、状态机、通信编排
│   ├── Control/          # 控制器：Buck-Boost、故障管理
│   ├── Algorithm/        # 纯算法：PID、PLL、滤波、RMS、Ramp、FFD
│   ├── Platform/         # 应用级硬件能力封装
│   ├── Debug/            # RTT、自检、bring-up 调试入口
│   └── main.c
├── Interface/            # C17 接口契约层
├── Driver/
│   ├── hpm_impl/         # HPM SDK 适配层
│   ├── WS2812/           # WS2812 协议驱动
│   └── IrqProfiler/      # 中断占用分析辅助
├── Board/
│   └── HPM5361_SuperCap_board/
├── doc/                  # 驱动设计、PWM/ADC/调试文档
├── linkers/              # 可选自定义链接脚本
├── Makefile
└── CMakeLists.txt
```

---

## 开发体验建议

- 优先使用 `make artifacts` 作为本地构建入口，确保 `output/` 下产物名称与工程名一致。
- VS Code Remote / Dev Container 内调试使用 `DEBUG_PATH_MODE=container`；宿主机 Ozone 等外部调试可使用默认 `host` 模式。
- 新增或迁移模块时遵循 `AGENTS.md` 分层约束：业务逻辑进入 `App/`，硬件契约进入 `Interface/`，SDK 适配进入 `Driver/hpm_impl/`。
- 临时调试和自动化会话文件应留在 `.sisyphus/`、`.mimocode/`、`build/`、`output/`，不要纳入版本管理。

---

## 参考文档

- `AGENTS.md`：工程分层、C17 编码和性能规范
- `App/README.md`：App 层内部划分说明
- `doc/build_flash_debug_guide.md`：编译、下载和 VS Code 调试说明
- `doc/adc_driver_design.md`：ADC 驱动设计说明
- `doc/hrpwm_driver_design.md`：HRPWM / PWM 驱动设计说明
- `doc/hpm_pwm_gptmr_pwm_guide.md`：HPM PWM 与 GPTMR PWM 使用梳理
