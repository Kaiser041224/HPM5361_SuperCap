# ADC 驱动与当前采样映射说明

本文记录当前 `HPM5361_SuperCap` 的 ADC 使用约定，重点是板级 `pinmux.c` 与 App 层逻辑通道之间的映射。

## 当前硬件映射

| 引脚 | ADC 实例 / 通道 | 逻辑通道 | 物理量 | 采样用途 |
|------|------------------|----------|--------|----------|
| `PB11` | `ADC1 ch3` | `ADC_CH_VCAP` / `APP_ADC_CH_VLINK` | 超级电容电压；控制模型 `VLINK` | 电压环反馈、Buck-Boost 前馈 |
| `PB12` | `ADC1 ch4` | `ADC_CH_VOUT` / `APP_ADC_CH_VIN` | 三端口 `VOUT`；控制模型 `VIN` | 输入功率估算 / Buck-Boost 前馈 / 调试观测 |
| `PB13` | `ADC1 ch5` | `ADC_CH_I_IN` | 输入电流 | 功率环输入电流反馈与滤波 |
| `PB14` | `ADC0 ch6` | `ADC_CH_I_L` | 电感电流 | ADC0 PMT 回调驱动电流内环 |

`PB10` 当前不再作为 ADC 输入使用，而是配置为 `GPTMR0_COMP_2` 蜂鸣器输出。

## PMT 触发链

- ADC0 由 `PWM1 CMP10` 在载波中心触发；ADC1 由 `PWM1 CMP11` 在载波 0 位置触发。
- ADC0 PMT 列表：dummy + `IL`。
- ADC1 PMT 列表：dummy + `VCAP` + `VOUT` + `IIN`。
- ADC0 PMT 完成回调负责触发 Buck-Boost 电流内环；ADC1 PMT 回调刷新 `VCAP/VOUT/IIN` 缓存，供外环和输入功率计算使用。

## 命名约定

当前三端口超级电容工程的硬件命名与源工程 / 控制模型不同：

- 当前 `VCAP` 等价于控制模型 `VLINK`，表示超级电容端电压。
- 当前 `VOUT` 等价于控制模型 `VIN`，表示三端口外部电压，参与输入功率和 Buck-Boost 前馈计算。
- `I_IN`、`I_L` 物理语义保持不变。

## 分层约束

- `App/Platform/Src/app_adc.c` 负责逻辑通道到 ADC 实例/通道的映射。
- `App/Platform/Src/app_analog_signal.c` 负责 raw → 物理量换算、滤波和快照。
- `App/Application/Src/app_control.c` 只消费逻辑通道，不直接依赖物理引脚名。
