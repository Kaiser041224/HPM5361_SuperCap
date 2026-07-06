# AGENTS.md - 嵌入式 C17 解耦架构开发指南 (完全体)

## 1. 核心设计哲学 (Core Philosophy)
本工程采用 **“物理隔离 + 契约驱动”** 模式。其核心目标是实现业务逻辑（App）与芯片外设（SDK/HAL）的深度解耦。
- **原子目标**：App 层代码与硬件外设深度解耦，仅通过 Interface 层访问。
- **现代化 C**：强制使用 **C17** 标准（匿名结构体、泛型选择 `_Generic`、静态断言 `static_assert`）。
- **高性能**：针对 HPM 系列 RISC-V 架构优化，强制执行 L1 Cache 对齐与 ILM (RAMFUNC) 部署。

---

## 2. 目录分层与依赖守则 (Strict Layering)

当前工程维持 **App 内部分层 + Interface 契约层 + Driver 适配层 + Board 板级层** 的结构，不再额外引入独立 `Module/` 目录。

### 2.1 顶层分层

| 目录层级 | 职责 (Responsibility) | 允许包含的头文件 | 禁止项 (Hard Bans) |
| :--- | :--- | :--- | :--- |
| **1. App/** | 应用级代码总入口；内部再细分为 `Application/`、`Control/`、`Algorithm/`、`Platform/`、`Debug/` | `App/*`, `Interface/`, `<stdint.h>`, `<stdbool.h>`, `<stddef.h>` | 禁止直接包含任何 `hpm_*.h`；禁止跨层直接依赖 `Board/` 私有实现 |
| **2. Interface/** | **契约定义层**。定义硬件抽象对象 (HAL) 与跨层访问协议 | 纯 C 标准库头文件 | 禁止包含私有变量、静态函数或驱动实现 |
| **3. Driver/** | SDK 适配实现。将物理寄存器操作映射至 Interface 契约 | `hpm_sdk.h`, `board_pins.h`, `Interface/` | 禁止在此处编写业务逻辑、控制策略或状态机 |
| **4. Board/** | 硬件底表。定义原始引脚、时钟、板级资源与 IOCFG | `hpm_soc.h` (仅宏定义) | 禁止承载业务逻辑、控制算法或应用编排 |

### 2.2 App/ 内部分层

| 子目录 | 职责 | 允许依赖 | 禁止项 |
| :--- | :--- | :--- | :--- |
| **App/Application/** | 应用编排层；负责系统启动后的业务流程、状态机、模态切换、任务组织、上位机通讯编排 | `App/Control/`, `App/Platform/`, `App/Algorithm/`, `Interface/` | 禁止直接调用 `Driver/` 私有接口；禁止直接操作寄存器或 `hpm_*` API |
| **App/Control/** | 实际控制器层；负责闭环控制、保护策略、控制状态、调节器组合与控制输出决策 | `App/Algorithm/`, `App/Platform/`, `Interface/` | 禁止承担上位机协议编排；禁止直接依赖 `Driver/` 私有接口 |
| **App/Algorithm/** | 纯算法库；提供 PID、PLL、滤波、RMS、斜坡等可复用计算模块 | C 标准库、`<math.h>`、`<float.h>`、必要的通用类型头 | 禁止依赖 `Driver/`、`Board/`，尽量避免依赖具体业务状态机 |
| **App/Platform/** | 面向应用的硬件能力封装；把 `Interface/` 组合成可供 `Application/Control/` 使用的稳定能力 | `Interface/`, 必要的 App 公共类型头 | 禁止在此处写具体业务状态机；禁止把 SDK/HAL 细节向上泄漏 |
| **App/Debug/** | 调试、验证、自检、RTT 输出、临时 bring-up 入口 | `Interface/`, `App/Platform/`, `App/Control/` | 禁止成为正式业务默认路径；禁止让调试逻辑反向主导生产代码结构 |

### 2.3 推荐调用方向

推荐依赖方向如下：

```text
Application -> Control -> Platform -> Interface -> Driver -> Board
            \-> Platform -> Interface -> Driver -> Board
Control     -> Algorithm
Debug       -> Platform / Control / Interface
```

补充约束：

- `Application/` 负责“做什么、什么时候做、处于什么模式”。
- `Control/` 负责“控制怎么算、如何保护、输出什么控制量”。
- `Platform/` 负责“硬件能力如何被安全、稳定地提供给上层”。
- `Debug/` 仅作为验证辅助层，不能长期承载正式运行主流程。
- 若某个对象天然属于控制语义（如功率级控制器、采样同步控制器、保护控制器），优先落在 `Control/`，而不是额外拆出 `Module/`。

---

## 3. C17 编码与性能规范 (Coding Standards)

### 3.1 接口对象化 (Object-Oriented C17)
接口定义必须使用 **匿名结构体**。所有物理参数需在驱动层完成归一化（例如：占空比统一为 `float` [0.0-1.0]）。
```c
// 示例：Interface/intf_pwm.h
typedef struct {
    uint8_t instance_id;
    struct {
        status_t (*init)(void);
        status_t (*set_duty)(float duty); 
    }; // 匿名结构体：允许对象通过 dev->set_duty() 直接调用
} pwm_if_t;
