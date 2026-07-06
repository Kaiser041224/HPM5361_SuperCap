# HPM5361 SuperCap

基于 HPMicro HPM5361 (RISC-V, 480MHz) 的超级电容功率控制工程。当前工程的项目命名、板级路径、调试产物和 VS Code 配置已统一到 `HPM5361_SuperCap` / `HPM5361_SuperCap_board`。

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

## 当前命名迁移状态

已检查并确认以下关键位置使用当前项目命名：

| 范围 | 当前值 |
|------|--------|
| 工程目录 | `HPM5361_SuperCap` |
| VS Code workspace | `HPM5361_SuperCap.code-workspace` |
| 默认 board | `HPM5361_SuperCap_board` |
| OpenOCD board cfg | `Board/HPM5361_SuperCap_board/HPM5361_SuperCap_board.cfg` |
| 输出 ELF | `output/HPM5361_SuperCap.elf` |
| 启动日志 | `[MAIN] HPM5361 SuperCap started` |

源码、文档和配置中未发现旧项目命名残留。

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
