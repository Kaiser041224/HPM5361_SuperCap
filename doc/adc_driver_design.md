# ADC 驱动与当前采样映射说明

本文记录当前 `HPM5361_SuperCap` 的 ADC 使用约定，重点是板级 `pinmux.c` 与 App 层逻辑通道之间的映射。

## 当前硬件映射

| 引脚 | ADC 实例 / 通道 | 逻辑通道 | 物理量 | 采样用途 |
|------|------------------|----------|--------|----------|
| `PB11` | `ADC1 ch3` | `ADC_CH_VOUT` | 超级电容控制器输出电压 | VOUT 电压环反馈、CW 功率计算、Buck-Boost 前馈 |
| `PB12` | `ADC1 ch4` | `ADC_CH_VCAP` | 超级电容电容组电压 | Buck-Boost cap 侧前馈、监测、限幅参考 |
| `PB13` | `ADC1 ch5` | `ADC_CH_I_IN` | 输入电流，`INA241A3` (`50V/V`) + `2mΩ` 采样电阻 | CW 环输入电流/功率反馈与滤波 |
| `PB14` | `ADC0 ch6` | `ADC_CH_I_L` | 电感电流，`INA241A3` (`50V/V`) + `1mΩ` 采样电阻 | ADC0 PMT 回调驱动电流内环 |

`PB10` 当前不再作为 ADC 输入使用，而是配置为 `GPTMR0_COMP_2` 蜂鸣器输出。

## PMT 触发链

- ADC0 由 `PWM1 CMP10` 在载波中心触发；ADC1 由 `PWM1 CMP11` 在同样的载波中心位置触发。
- ADC0 PMT 列表：dummy + `IL`。
- ADC1 PMT 列表：dummy + `VOUT` + `VCAP` + `IIN`。
- ADC0 PMT 完成回调负责触发 Buck-Boost 电流内环；ADC1 PMT 回调刷新 `VCAP/VOUT/IIN` 缓存，供外环、前馈和功率计算使用。

## 命名约定

当前工程直接使用当前硬件物理节点名，不再引入源工程的别名映射：

- `VOUT`：超级电容控制器输出电压，和系统 `VIN` 仅通过 `IIN` 检流电阻相连；`VIN` 当前未采样。
- `VCAP`：超级电容电容组电压，位于四开关 Buck-Boost cap 侧。
- `I_IN`：输入电流，`INA241A3` (`50V/V`)、`2mΩ` 采样电阻，按硬件正方向直接换算，不做软件极性反向。
- `I_L`：电感电流，`INA241A3` (`50V/V`)、`1mΩ` 采样电阻，驱动 Buck-Boost 电流内环；cap 侧充电/放电电流由 `I_L` 和占空比估算，仅用于 CC 竞争环。

## 分层约束

- `App/Platform/Src/app_adc.c` 负责逻辑通道到 ADC 实例/通道的映射。
- `App/Platform/Src/app_analog_signal.c` 负责 raw → 物理量换算、滤波和快照。
- `App/Application/Src/app_control.c` 只消费逻辑通道，不直接依赖物理引脚名。
