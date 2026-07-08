# HRPWM / PWM 平台说明

当前工程仅将 App 层功率 PWM 绑定到 `PWM1` 的两组互补输出。

## 当前 pinmux

| 引脚 | 功能 | App 语义 |
|------|------|----------|
| `PA28` | `PWM1_P_4` | Buck-Boost A 高/低边互补 pair 起始通道 |
| `PA29` | `PWM1_P_5` | Buck-Boost A 互补输出 |
| `PA30` | `PWM1_P_6` | Buck-Boost B 高/低边互补 pair 起始通道 |
| `PA31` | `PWM1_P_7` | Buck-Boost B 互补输出 |
| `PA25` | `PWM0_P_1` | 板级保留；App HRPWM 平台当前不管理 |

## App 平台语义

`App/Platform/Inc/app_hrpwm.h` 仅暴露：

- `HRPWM_BUCKBOOST_A -> PWM1 pair0 -> ch4/ch5`
- `HRPWM_BUCKBOOST_B -> PWM1 pair1 -> ch6/ch7`
- `HRPWM_INST_BUCKBOOST -> PWM1`

相位控制接口保留为空实现，仅用于兼容旧调试入口；当前控制链不再使用移相输出。

## ADC 触发

`PWM1 CMP10` 在载波中心触发 ADC0 PMT，用于 `IL` 电流内环采样。
`PWM1 CMP11` 在载波中心触发 ADC1 PMT，用于 `VOUT`、`VCAP`、`IIN` 外环/功率采样。
