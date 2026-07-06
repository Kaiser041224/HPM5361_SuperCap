# 编译、下载与 VS Code 调试配置说明

本文整理当前工程的编译、OpenOCD 下载、GDB Server、VS Code Cortex-Debug F5 调试及源码路径映射配置。

适用环境：

- 工程目录：`/workspace/HPM5361_SuperCap`
- HPM SDK：`/workspace/hpm_sdk`
- HPM OpenOCD：`/workspace/tools/openocd-hpm/install/bin/openocd`
- RISC-V 工具链：`/opt/riscv32-gnu-toolchain-elf-bin/bin`
- 调试器：J-Link（支持 RTT）、DAPLink / CMSIS-DAP（不支持 RTT）

---

## 1. 关键文件

| 文件 | 作用 |
|------|------|
| `Makefile` | 统一编译、导出产物、OpenOCD 下载、OpenOCD GDB Server 启动 |
| `.vscode/tasks.json` | VS Code F5 前自动执行 `make artifacts DEBUG_PATH_MODE=container` |
| `.vscode/launch.json` | Cortex-Debug 启动 OpenOCD、连接 GDB、加载 ELF 并停到 `main` |
| `Board/HPM5361_SuperCap_board/HPM5361_SuperCap_board.cfg` | 当前板子的 OpenOCD flash bank、时钟初始化、GDB attach 行为 |
| `Board/HPM5361_SuperCap_board/HPM5361_SuperCap_board.yaml` | 板级 SoC、OpenOCD probe、flash size 等元信息 |

---

## 2. Makefile 编译配置

### 2.1 默认板卡与构建参数

当前默认配置：

```makefile
BOARD ?= HPM5361_SuperCap_board
RV_ARCH ?= rv32imafdc
RV_ABI ?= ilp32d
CMAKE_BUILD_TYPE ?= Debug
HPM_BUILD_TYPE ?= flash_xip
OPT_LEVEL ?= -O0
```

常用编译命令：

```bash
make build
make artifacts
```

区别：

| 命令 | 作用 |
|------|------|
| `make build` | 配置 CMake 并编译 |
| `make artifacts` | 编译后把 `build/output/demo.*` 复制到 `output/HPM5361_SuperCap.*` |

`make artifacts` 成功后会生成：

```text
output/HPM5361_SuperCap.elf
output/HPM5361_SuperCap.bin
output/HPM5361_SuperCap.asm
output/HPM5361_SuperCap.map
```

---

## 3. 调试源码路径模式

当前工程支持两种源码路径模式：

```makefile
DEBUG_PATH_MODE ?= host
HOST_WORKSPACE_DIR ?= D:/Codes/HPM_dev/Alliance-HPM-Dev
```

### 3.1 `host` 模式：默认模式

普通命令默认使用 `host` 模式：

```bash
make artifacts
```

此时 ELF 调试信息中的源码路径会被映射到宿主机路径：

```text
D:/Codes/HPM_dev/Alliance-HPM-Dev/HPM5361_SuperCap
D:/Codes/HPM_dev/Alliance-HPM-Dev/hpm_sdk
```

用途：

- 宿主机侧调试器
- Ozone 等容器外调试场景

如宿主机工作区位置变化，只需要修改：

```makefile
HOST_WORKSPACE_DIR ?= D:/Codes/HPM_dev/Alliance-HPM-Dev
```

### 3.2 `container` 模式：VS Code 容器内 F5 调试

容器内 VS Code 调试必须使用容器内源码路径：

```bash
make artifacts DEBUG_PATH_MODE=container
```

此时 ELF 调试信息中的源码路径为：

```text
/workspace/HPM5361_SuperCap
/workspace/hpm_sdk
```

`.vscode/tasks.json` 已经在 F5 前自动使用该模式：

```json
"args": [
  "artifacts",
  "DEBUG_PATH_MODE=container"
]
```

因此：

- 手动普通编译：默认保留宿主机路径。
- VS Code F5 调试：自动切换为容器路径。

---

## 4. OpenOCD 下载配置

### 4.1 Makefile 默认 OpenOCD 参数

```makefile
FLASH_TOOL ?= openocd
OPENOCD_BIN ?= $(if $(HPM_OPENOCD_PREFIX),$(HPM_OPENOCD_PREFIX)/bin/openocd,openocd)
OCD_SCRIPTS ?= $(if $(HPM_OCD_SCRIPTS),$(HPM_OCD_SCRIPTS),$(ROOT_DIR)/../hpm_sdk/boards/openocd)
PROBE_CFG ?= probes/cmsis_dap.cfg
SOC_CFG ?= soc/hpm5300.cfg
BOARD_CFG ?= $(ROOT_DIR)/Board/HPM5361_SuperCap_board/HPM5361_SuperCap_board.cfg
OPENOCD_SPEED ?= 1000
OPENOCD_RISCV_TIMEOUT ?= 60
```

DAPLink 在 OpenOCD 中使用 CMSIS-DAP 配置：

```text
probes/cmsis_dap.cfg
```

HPM5361 使用 HPM5300 系列 SoC 配置：

```text
soc/hpm5300.cfg
```

### 4.2 下载命令

```bash
make flash
```

等价于：

```bash
make flash-openocd
```

执行内容：

```bash
/workspace/tools/openocd-hpm/install/bin/openocd \
  -s /workspace/hpm_sdk/boards/openocd \
  -f probes/cmsis_dap.cfg \
  -f soc/hpm5300.cfg \
  -f /workspace/HPM5361_SuperCap/Board/HPM5361_SuperCap_board/HPM5361_SuperCap_board.cfg \
  -c "adapter speed 1000" \
  -c "riscv set_command_timeout_sec 60" \
  -c "reset_config srst_only srst_nogate connect_deassert_srst" \
  -c "init; reset halt; program output/HPM5361_SuperCap.elf verify reset exit"
```

如果 DAPLink/JTAG 不稳定，可降低速率：

```bash
make flash OPENOCD_SPEED=400
make flash OPENOCD_SPEED=200
```

成功下载的关键日志：

```text
** Programming Finished **
** Verify Started **
** Verified OK **
```

---

## 5. 当前板级 OpenOCD cfg

文件：

```text
Board/HPM5361_SuperCap_board/HPM5361_SuperCap_board.cfg
```

核心 flash bank：

```tcl
flash bank xpi0 hpm_xpi 0x80000000 0x100000 1 1 $_TARGET0 0xF3000000 0x5 0x1000
```

参数说明：

| 参数 | 值 | 说明 |
|------|----|------|
| Flash base | `0x80000000` | XIP Flash 映射基地址 |
| Flash size | `0x100000` | 1MiB，与当前 board yaml 保持一致 |
| Controller | `0xF3000000` | HPM5361 `XPI0` 控制器地址 |
| option0 | `0x5` | 与 `board.c` 的 `nor_cfg_option` 对齐 |
| option1 | `0x1000` | 与 `board.c` 的 `nor_cfg_option` 对齐 |

时钟初始化：

```tcl
proc init_clock {} {
    mww 0xF4000800 0xFFFFFFFF
    mww 0xF4000810 0xFFFFFFFF
    mww 0xF4000820 0xFFFFFFFF
    mww 0xF4000830 0xFFFFFFFF
    echo "clocks has been enabled!"
}
```

OpenOCD 在 `reset-init` 时会调用：

```tcl
$_TARGET0 configure -event reset-init {
    init_clock
}
```

调试连接与断开事件：

```tcl
# GDB 连接时：复位并暂停芯片，等待调试命令
$_TARGET0 configure -event gdb-attach {
    reset halt
}

# GDB 断开时：复位芯片并恢复运行
$_TARGET0 configure -event gdb-disconnected {
    reset run
}
```

`gdb-disconnected` 事件确保退出调试后芯片自动复位并正常运行。

---

## 6. 单独启动 OpenOCD GDB Server

如需只启动 GDB Server，不下载程序：

```bash
make debug-openocd
```

默认监听：

```text
localhost:3333
```

另开终端连接：

```bash
riscv32-unknown-elf-gdb output/HPM5361_SuperCap.elf
```

GDB 内执行：

```gdb
target extended-remote localhost:3333
monitor reset halt
load
break main
continue
```

如果程序已经下载，不想重新 `load`：

```gdb
target extended-remote localhost:3333
monitor reset halt
break main
continue
```

---

## 7. VS Code F5 调试配置

### 7.1 自动编译任务

文件：`.vscode/tasks.json`

核心配置：

```json
{
  "label": "make artifacts",
  "type": "shell",
  "command": "make",
  "args": [
    "artifacts",
    "DEBUG_PATH_MODE=container"
  ],
  "options": {
    "cwd": "${workspaceFolder}"
  },
  "problemMatcher": []
}
```

作用：

- 每次 F5 前自动编译最新代码。
- 使用 `DEBUG_PATH_MODE=container`，确保源码路径可在容器内打开。
- 产物输出到 `output/HPM5361_SuperCap.elf`。

### 7.2 Cortex-Debug 启动配置

文件：`.vscode/launch.json`

核心配置：

```json
{
  "name": "HPM5361 Debug - OpenOCD DAPLink",
  "type": "cortex-debug",
  "request": "launch",
  "servertype": "openocd",
  "cwd": "${workspaceFolder}",
  "executable": "${workspaceFolder}/output/HPM5361_SuperCap.elf",
  "preLaunchTask": "make artifacts",
  "runToEntryPoint": "main"
}
```

工具链路径：

```json
"gdbPath": "/opt/riscv32-gnu-toolchain-elf-bin/bin/riscv32-unknown-elf-gdb",
"toolchainPrefix": "riscv32-unknown-elf",
"objdumpPath": "/opt/riscv32-gnu-toolchain-elf-bin/bin/riscv32-unknown-elf-objdump",
"nmPath": "/opt/riscv32-gnu-toolchain-elf-bin/bin/riscv32-unknown-elf-nm"
```

OpenOCD 路径与配置：

```json
"serverpath": "/workspace/tools/openocd-hpm/install/bin/openocd",
"searchDir": [
  "/workspace/hpm_sdk/boards/openocd"
],
"configFiles": [
  "probes/cmsis_dap.cfg",
  "soc/hpm5300.cfg",
  "${workspaceFolder}/Board/HPM5361_SuperCap_board/HPM5361_SuperCap_board.cfg"
]
```

OpenOCD 额外参数：

```json
"serverArgs": [
  "-c",
  "bindto 127.0.0.1",
  "-c",
  "adapter speed 1000",
  "-c",
  "riscv set_command_timeout_sec 60",
  "-c",
  "reset_config srst_only srst_nogate connect_deassert_srst"
]
```

说明：

- `bindto 127.0.0.1`：限制 OpenOCD 端口仅本机访问。
- `adapter speed 1000`：JTAG 速率。若不稳定，可改为 `400` 或 `200`。
- `riscv set_command_timeout_sec 60`：避免 DAPLink 下载/擦写较慢时超时。

### 7.3 源码路径兜底映射

`launch.json` 中配置：

```json
"sourceFileMap": {
  "D:/Codes/HPM_dev/Alliance-HPM-Dev/HPM5361_SuperCap": "${workspaceFolder}",
  "D:/Codes/HPM_dev/Alliance-HPM-Dev/hpm_sdk": "/workspace/hpm_sdk"
}
```

作用：

- 主要靠 `DEBUG_PATH_MODE=container` 让 F5 编译出容器路径 ELF。
- `sourceFileMap` 是兜底：如果 ELF 中仍残留宿主机路径，VS Code 也能映射回容器路径打开源码。

---

## 8. 推荐工作流

### 8.1 日常容器内 F5 调试

在 VS Code 中按：

```text
F5
```

自动完成：

1. `make artifacts DEBUG_PATH_MODE=container`
2. 生成最新 `output/HPM5361_SuperCap.elf`
3. Cortex-Debug 启动 OpenOCD
4. GDB 连接 `localhost:3333`
5. 下载/加载 ELF
6. 停在 `main`

### 8.2 命令行编译

默认宿主机路径模式：

```bash
make artifacts
```

容器路径模式：

```bash
make artifacts DEBUG_PATH_MODE=container
```

### 8.3 命令行下载

```bash
make flash
```

链路不稳定时：

```bash
make flash OPENOCD_SPEED=400
make flash OPENOCD_SPEED=200
```

### 8.4 命令行调试

终端 1：

```bash
make debug-openocd OPENOCD_SPEED=400
```

终端 2：

```bash
riscv32-unknown-elf-gdb output/HPM5361_SuperCap.elf
```

GDB：

```gdb
target extended-remote localhost:3333
monitor reset halt
break main
continue
```

---

## 9. 常见日志与处理

### 9.1 烧录成功标志

```text
** Programming Finished **
** Verify Started **
** Verified OK **
```

出现以上日志说明烧录和校验成功。

### 9.2 JTAG warning

可能出现：

```text
Unexpected idcode after end of chain
IR capture error at bit 5
```

如果后续仍出现：

```text
Target successfully examined
** Verified OK **
```

则本次烧录/调试可以继续，但说明 JTAG 链路质量一般。建议：

1. 降低速度：`OPENOCD_SPEED=400` 或 `OPENOCD_SPEED=200`
2. 检查 DAPLink 与目标板是否可靠共地
3. 检查 `VTref` 是否接入目标板 `3V3`
4. 检查 `TCK/TMS/TDI/TDO` 是否接反或虚焊
5. 缩短 JTAG 线
6. 确认目标板复位脚没有被外部电路干扰

### 9.3 找不到源码文件

现象：

```text
由于找不到该文件，因此无法打开编辑器
```

原因通常是 ELF 中的调试路径指向宿主机，而 VS Code 运行在容器内。

处理：

- F5 调试：确认 `.vscode/tasks.json` 使用 `DEBUG_PATH_MODE=container`
- 手动调试：先运行

```bash
make artifacts DEBUG_PATH_MODE=container
```

### 9.4 退出调试后 VS Code 卡在调试状态

正常情况下在 VS Code 中按 **Shift+F5** 或点红色停止按钮，Cortex-Debug 会自动关闭 OpenOCD 和 GDB。

如果 VS Code 卡在调试状态，通常是 OpenOCD 或 GDB 进程未正常退出。手动清理：

```bash
# 关闭 OpenOCD
pkill -f 'openocd.*HPM5361_SuperCap'

# 关闭 GDB
pkill -f 'riscv32-unknown-elf-gdb.*HPM5361_SuperCap'
```

或一键清理：

```bash
pkill -f 'openocd.*HPM5361_SuperCap' || true; pkill -f 'riscv32-unknown-elf-gdb.*HPM5361_SuperCap' || true
```

清理后 VS Code 调试状态会自动恢复。

> **注意**：不要在 `.vscode/tasks.json` 的 `postDebugTask` 中直接放 `pkill -f` 命令。`pkill -f` 会匹配完整命令行，可能匹配到 bash 自身，导致 task 永远不结束，反而让 VS Code 卡在"正在运行任务"状态。

### 9.5 退出调试后芯片不运行

Cortex-Debug v1.12.1 没有 `overrideDetachCommands`，OpenOCD 的 `gdb-disconnected` 事件会干扰启动流程，因此退出调试后芯片会停在 halt 状态。

**手动复位**：`Ctrl+Shift+P` → `Tasks: Run Task` → `reset chip`

或通过 telnet：

```bash
telnet localhost 4444
> reset run
> exit
```

---

## 10. SEGGER RTT 实时调试输出

### 10.1 概述

SEGGER RTT 通过调试器的 JTAG/SWD 接口实现零 CPU 开销的实时日志输出，无需额外串口。

当前工程支持情况：

| 调试器 | RTT 支持 | 说明 |
|--------|:--------:|------|
| J-Link | ✅ | J-Link GDB Server 原生支持 RTT |
| DAPLink / OpenOCD | ❌ | HPM OpenOCD fork 缺少 `rtt setup` 命令 |

### 10.2 固件集成

`CMakeLists.txt` 中手动引入 SEGGER RTT 源码（不含 syscalls），避免与 UART debug console 的 `_write` 冲突：

```cmake
set(RTT_DIR $ENV{HPM_SDK_BASE}/middleware/segger_rtt)
sdk_app_inc(${RTT_DIR}/Config)
sdk_app_inc(${RTT_DIR}/RTT)
sdk_app_src(${RTT_DIR}/RTT/SEGGER_RTT.c)
sdk_app_src(${RTT_DIR}/RTT/SEGGER_RTT_printf.c)
```

代码中使用 `SEGGER_RTT_printf` 输出 RTT 日志，标准 `printf` 保持走 UART 不受影响：

```c
#include "SEGGER_RTT.h"

SEGGER_RTT_WriteString(0, "Hello RTT\r\n");
SEGGER_RTT_printf(0, "value: %d\r\n", some_var);
```

### 10.3 VS Code 配置

`.vscode/launch.json` 中有两个调试配置：

- **HPM5361 Debug - J-Link (RTT)**：RTT 启用，VS Code 自动打开 RTT Console
- **HPM5361 Debug - OpenOCD DAPLink**：RTT 禁用

使用时在调试下拉中选择对应配置。

### 10.4 为什么不使用 `CONFIG_SEGGER_RTT=1`

SDK 的 `CONFIG_SEGGER_RTT=1` 会引入 `SEGGER_RTT_Syscalls_GCC.c`，该文件定义 `_write` 函数将 `printf` 重定向到 RTT。这会与 `hpm_debug_console.c` 的 UART `_write` 冲突（`multiple definition`）。

解决方案对比：

| 方案 | `_write` 冲突 | UART printf | RTT printf |
|------|:---:|:---:|:---:|
| `CONFIG_SEGGER_RTT=1` 单独 | ❌ 冲突 | - | - |
| `CONFIG_SEGGER_RTT=1` + `CONFIG_NDEBUG_CONSOLE=1` | ✅ | ❌ 禁用 | ✅ |
| **手动引入（当前方案）** | ✅ | ✅ 保留 | ✅ |

---

## 11. 配置速查

| 需求 | 命令/配置 |
|------|-----------|
| 普通编译，宿主机路径 | `make artifacts` |
| 容器调试编译 | `make artifacts DEBUG_PATH_MODE=container` |
| OpenOCD 下载 | `make flash` |
| 降速下载 | `make flash OPENOCD_SPEED=400` |
| 启动 GDB Server | `make debug-openocd` |
| 降速启动 GDB Server | `make debug-openocd OPENOCD_SPEED=400` |
| VS Code F5 调试 | 选择 `HPM5361 Debug - J-Link (RTT)` 或 `OpenOCD DAPLink` |
| 退出调试后复位芯片 | `Tasks: Run Task` → `reset chip` |
| 修改宿主机源码根路径 | `HOST_WORKSPACE_DIR ?= ...` |
| 修改 F5 调试源码路径模式 | `.vscode/tasks.json` 中 `DEBUG_PATH_MODE=container` |
| 修改 JTAG 速度 | `OPENOCD_SPEED=<kHz>` 或 `launch.json` 的 `adapter speed` |
