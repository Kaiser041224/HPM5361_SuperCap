# App 目录说明

`App/` 层用于组织应用级代码，当前划分为以下子层：

- `Application/`：应用入口 (`app_entry`)、系统控制编排 (`app_control`)、CAN 通信 (`app_comm`)。
- `Control/`：面向对象控制器 — `ctrl_buckboost` (四开关 Buck-Boost)、`ctrl_fault` (故障管理)。
- `Algorithm/`：纯算法库 (PID / PLL / Ramp / RMS / FFD / Filter)，不依赖 `app_*` 或 `hpm_*`。
- `Platform/`：应用级平台能力封装 (`app_hrpwm`、`app_adc`、`app_can`、`app_gpio`)。
- `Debug/`：RTT 输出 (`app_debug_rtt`)，按域拆分测试用例 (`app_debug_adc` / `app_debug_can` / `app_debug_hrpwm`)。

`main.c` 承担系统启动桥接，测试阶段调用 `app_init()` + `app_debug_run_once()`，生产阶段改为 `app_init()` + `app_run_once()`。
