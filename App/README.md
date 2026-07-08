# App 目录说明

`App/` 层用于组织应用级代码。本工程是一个面向 RoboMaster 的超级电容控制器，通过四开关 Buck-Boost 变换器管理 VIN / VOUT / VCAP 三端口能量，实现恒功率削峰填谷。

## 控制架构

```text
Application/  (系统状态机、运行模式编排、故障响应)
    │
    ▼
Control/     (控制器：Buck-Boost 电流/电压/CW/CC 环、故障管理)
    │
    ▼
Algorithm/   (纯算法：PID、滤波、斜坡、PLL 等)
```

## 子层划分

- `Application/`：系统启动桥接 (`main.c`)、运行状态机与模式编排 (`app_control`)、CAN 通信 (`app_comm`)。
- `Control/`：四开关 Buck-Boost 控制器 (`ctrl_buckboost`)，含电流内环 (200kHz)、VOUT 电压外环 + cap 侧 CC 竞争环 (50kHz)、CW 功率外环 (25kHz)；故障保护管理 (`ctrl_fault`)。
- `Algorithm/`：纯算法库 (PID / PLL / Ramp / RMS / FFD / Filter)，不依赖 `app_*` 或 `hpm_*`。
- `Platform/`：应用级硬件能力封装 — HRPWM、ADC/PMT 采样链、模拟量调理与滤波、CAN、GPIO。
- `Debug/`：RTT 输出 (`app_debug_rtt`)，按域拆分调试入口 (`app_debug_adc` / `app_debug_can` / `app_debug_hrpwm`)。

## 分层约束

遵循 `AGENTS.md` 定义的四层解耦架构：

- `Application/` 负责"何时做、处于什么模式"。
- `Control/` 负责"控制怎么算、如何保护、输出什么量"。
- `Platform/` 负责"硬件能力如何被安全地提供给上层"。
- 所有 App 层禁止直接包含 `hpm_*.h`，仅通过 `Interface/` 契约层访问硬件抽象。
