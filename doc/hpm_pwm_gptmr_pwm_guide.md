# HPM PWM / GPTMR 当前使用说明

本文只记录当前工程的实际使用方式。更完整的 SDK API 行为请参考 `/workspace/hpm_sdk` 示例和驱动头文件。

## PWM1：Buck-Boost 功率 PWM

| App 对象 | PWM 实例 | 输出通道 | 引脚 |
|----------|----------|----------|------|
| `HRPWM_BUCKBOOST_A` | `PWM1` | `P_4/P_5` | `PA28/PA29` |
| `HRPWM_BUCKBOOST_B` | `PWM1` | `P_6/P_7` | `PA30/PA31` |

`PWM1 CMP10` 用作 ADC0 PMT 触发比较点，对应 `IL` 载波中心采样。
`PWM1 CMP11` 用作 ADC1 PMT 触发比较点，对应 `VOUT`、`VCAP`、`IIN` 载波中心采样。

## GPTMR0：蜂鸣器

| App 对象 | GPTMR 通道 | 引脚 | 功能 |
|----------|------------|------|------|
| `app_buzzer` | global ch2 (`GPTMR0 CH2`) | `PB10` | 蜂鸣器 PWM 输出 |

## GPTMR1：控制外环定时器

| App 通道 | GPTMR 通道 | 频率 | 用途 |
|----------|------------|------|------|
| `APP_GPTMR_CH_VOLTAGE` | `GPTMR1 CH0` | 50 kHz | VOUT 电压外环 + cap 侧估算电流 CC 竞争环 |
| `APP_GPTMR_CH_POWER` | `GPTMR1 CH1` | 25 kHz | 功率估算 / 功率环 |

当前工程不再使用额外的谐振控制外环定时器。
