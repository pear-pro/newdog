# 工程变更交接文档 - 四足机器人项目 (STM32F427)

> 分支: `test` | 基线: commit `18280f1` (移植4310)
> 日期: 2026-06-05

---

## 一、变更概述

本次变更的核心目标：**新增 UART8 串口通信模块，实现树莓派向 STM32 下发速度指令，控制四足机器人行走。**

---

## 二、新增文件

### 1. `Core/Src/usart_demo.c` + `Core/Inc/usart_demo.h`

UART8 串口通信完整模块，包含：

- **协议**: 8 字节定长帧 `[0x55][0xAA][CTRL][V_H][V_L][W_H][W_L][CKSUM]`
- **接收**: DMA 循环模式 + IDLE 中断 → 环形缓冲区 (256B)
- **解析**: 三状态机 (SYNC_1 → SYNC_2 → COLLECT)，双字节帧头同步
- **分发**: 支持 `CTRL_SPEED (0x01)` 指令，将线速度/角速度注入 `rcData`
- **发送**: 预留 `UART8_Demo_SendResponse()` 应答函数（当前未调用）
- **超时保护**: 代码已写好但注释掉了（1000ms 无数据回站立）

---

## 三、修改文件清单

### 3.1 硬件/驱动层

| 文件 | 变更内容 |
|------|----------|
| `Core/Src/usart.c` | 新增 `MX_UART8_Init()` (115200, 8N1)；MSP 层配置 PE0(RX)/PE1(TX)，GPIO AF8；DMA RX=CIRCULAR (DMA1_Stream6, CH5)，TX=NORMAL (DMA1_Stream0, CH5)；NVIC 优先级 3,0 |
| `Core/Src/dma.c` | 新增 DMA1_Stream0 和 DMA1_Stream6 的中断优先级配置 |
| `Core/Src/stm32f4xx_it.c` | 新增三个中断处理函数：`DMA1_Stream0_IRQHandler` (UART8 TX DMA)、`DMA1_Stream6_IRQHandler` (UART8 RX DMA)、`UART8_IRQHandler` (IDLE 检测 + 标志置位) |
| `Core/Inc/usart.h` | 新增 `MX_UART8_Init()` 声明 |
| `Core/Inc/stm32f4xx_it.h` | 新增 `DMA1_Stream0_IRQHandler` 和 `DMA1_Stream6_IRQHandler` 声明 |
| `Core/Src/tim.c` | 微调（未涉及核心逻辑） |
| `Core/Src/gpio.c` | 微调（未涉及核心逻辑） |

### 3.2 应用层

| 文件 | 变更内容 |
|------|----------|
| `Core/Src/main.c` | 主循环中调用 `UART8_Demo_Init()` 和 `UART8_Demo_Process()`；注释掉了机械臂调试死循环；注释掉了 `motion_Jump` 调用；新增 `MX_UART8_Init()` 调用 |
| `Core/Src/gait.c` | 步态算法调整（与串口无关，属步态优化） |
| `Core/Src/motor.c` | 电机参数微调 |
| `Core/Inc/gait.h` | 步态头文件微调 |

### 3.3 构建系统

| 文件 | 变更内容 |
|------|----------|
| `CMakeLists.txt` | `target_sources` 中新增 `Core/Src/usart_demo.c` |
| `build/Debug/compile_commands.json` | 手动新增 `usart_demo.c` 的编译条目（供 clangd/IntelliSense 使用） |

### 3.4 STM32CubeMX 配置

| 文件 | 变更内容 |
|------|----------|
| `1.ioc` | 新增 UART8 外设配置（引脚、DMA、NVIC） |
| `MDK-ARM/1.uvprojx` | Keil 工程文件同步更新 |

---

## 四、数据流架构

```
树莓派 (115200bps)
    │
    ▼ (8字节帧)
UART8 PE0(RX) + PE1(TX)
    │
    ▼ (DMA 循环写入)
dma_rx_buf[256]  ←── DMA1_Stream6 自动写入
    │
    ▼ (IDLE 中断触发)
UART8_IRQHandler() → uart8_idle_flag = 1
    │
    ▼ (主循环轮询)
UART8_Demo_Process()
    │
    ├─ RingBuf_ReadByte() ← 逐字节从环形缓冲区读取
    ├─ Parser_FeedByte()  ← 状态机解析帧头+数据
    ├─ Parser_ValidateFrame() ← 校验和验证
    └─ DispatchCommand()  → rcData.R_y / rcData.R_x (速度注入)
                                │
                                ▼
                           步态状态机消费速度值
```

---

## 五、需要关注的问题和风险点

### 5.1 高优先级

1. **超时保护已禁用**: `usart_demo.c:252-258` 中 1000ms 无数据回站立的逻辑被注释。如果树莓派断开或通信中断，机器人会保持最后的速度指令一直运动，**有安全风险**。建议根据实际需求启用或实现其他安全机制。

2. **`uart8_walk_request` 未接入主循环**: `main.c` 中对应的 sw5/sw7 置位代码被注释掉（`main.c:280-285`）。当前树莓派下发的速度数据只注入了 `rcData.R_x` 和 `rcData.R_y`，但**没有触发行走状态机**（需要 sw5==0x0320 && sw7==0x0320 才进入 `temp_state=1`）。需确认：是通过树莓派指令触发，还是通过物理遥控器触发？

3. **NVIC 优先级冲突风险**: UART8 中断优先级设为 (3, 0)。需确认与其他外设（CAN、TIM、UART1/7）的优先级不冲突，特别是电机通信和遥控器接收的实时性要求。

### 5.2 中优先级

4. **步态参数未验证**: `gait.c` 有大量修改（967 行变更），属于步态算法调整。需确认步态在实机上已测试通过。

5. **`motion_Jump` 被注释**: `main.c` 中 case 7 的跳跃动作被注释。如果是有意禁用请确认，否则跳跃功能不可用。

6. **机械臂调试代码已注释**: `main.c` 中 `gimbal_send_unitree` 死循环被注释掉，机械臂功能当前未启用。

7. **`compile_commands.json` 手动维护**: 当前 `build/Debug/compile_commands.json` 是手动添加了 `usart_demo.c` 条目。下次 CMake 重新生成时会覆盖。CMakeLists.txt 已正确添加源文件，所以重生成不会有问题，但需注意不要手动编辑该文件中的其他条目。

### 5.3 低优先级

8. **DMA RX 缓冲区大小**: 当前 256 字节，对于 8 字节帧、低频指令绰绰有余。但如果未来扩展高频数据流，需评估是否够用。

9. **发送功能未使用**: `UART8_Demo_SendResponse()` 已实现但未被调用。如果树莓派需要应答，需在 `DispatchCommand()` 中添加调用。

10. **校验和算法**: 当前只对 Byte2-Byte6 求和取低 8 位，未包含帧头。安全性足够，但确认与树莓派端协议一致。

---

## 六、硬件连接确认项

| 项目 | 值 |
|------|-----|
| UART8 TX | PE1 (AF8) |
| UART8 RX | PE0 (AF8) |
| 波特率 | 115200, 8N1 |
| DMA RX | DMA1 Stream6 Channel5 (CIRCULAR) |
| DMA TX | DMA1 Stream0 Channel5 (NORMAL) |
| 硬件流控 | 无 |

**需确认**: 树莓派端的 TX/RX 是否与 STM32 交叉连接（RPi TX → PE0, RPi TX ← PE1）。

---

## 七、编译和烧录

- **构建工具**: CMake + ARM GCC (STM32CubeIDE bundles)
- **构建命令**: `cmake -DCMAKE_BUILD_TYPE=Debug -B build/Debug && cmake --build build/Debug`
- **Keil 工程**: `MDK-ARM/1.uvprojx` 已同步更新
- **目标芯片**: STM32F427xx (Cortex-M4, FPU)

---

## 八、测试建议

1. **通信回环测试**: 用串口工具直接连接 UART8 (PE0/PE1)，发送 `[55 AA 01 00 64 00 00 65]` 验证能否触发行走
2. **校验和拒收测试**: 发送错误校验和的帧，确认被丢弃
3. **帧头重同步测试**: 在数据流中随机插入 0x55，确认解析器能重新同步
4. **超时保护测试**: 确认通信中断后机器人的行为（当前会保持最后速度）
5. **DMA 溢出测试**: 高速连续发送，确认环形缓冲区不溢出
6. **实机行走测试**: 树莓派下发速度指令，确认步态正常响应
