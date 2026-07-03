# UART8 串口调试经验总结

> 项目：四足机器人 STM32F427
> 模块：`Core/Src/usart_demo.c` — 树莓派速度指令接收
> 日期：2026-06-07
> 相关文件：`REVIEW_FOR_NEXT_TEAM.md`

---

## 一、问题现象

树莓派通过 UART8 下发速度指令。调试时发现：

| 测试场景 | 现象 |
|----------|------|
| 第一轮发送（0→0.6 连续数据） | 正常接收，`front_speed` 跟随变化 |
| 停止发送，等几秒 | 正常 |
| 第二轮发送（同样数据） | **只收到前几帧，之后 `dbg_frame_ok_count` 停止增长** |
| 停下来再发 | 完全收不到 |

`dbg_frame_ok_count` 停在一个固定值不再变化，`dbg_frame_err_count` 也不增加——说明不是校验失败，而是**数据根本不再被处理**。

---

## 二、排查过程复盘（含错误判断）

### 路线 1：怀疑校验和计算 ⚠️ 走了弯路
- 检查 `CalcChecksum8()` 和树莓派 PDF 协议
- 验证测试帧 `55 AA 01 03 E8 00 00 EC`：校验正确
- 结论：不是校验问题

### 路线 2：怀疑 0x55 数据字节 ⚠️ 部分正确但不致命
- 发现 `PARSE_COLLECT` 阶段遇到 `0x55` 会误判为帧头（数据区 velocity_v 低字节可能等于 0x55）
- 修复了此 bug，但没有解决根本问题
- 结论：数据区 0x55 是有问题，但卡死另有原因

### 路线 3：怀疑 IDLE 标志依赖 ⚠️ 有改进但不根本
- 原代码 `if (uart8_idle_flag)` 才处理数据，IDLE 中断不触发 = 数据不处理
- 改为轮询 `RingBuf_Available() > 0` 就处理
- 结论：改进了可靠性和处理速度，但不能解决卡死

### 路线 4：怀疑超时逻辑、环形缓冲区溢出、DMA 配置 ⚠️ 都不是

### ✅ 真正原因：UART Overrun Error (ORE)
- **排查中一直遗漏的地方：`huart8.ErrorCode`、`huart8.Instance->CR3` 的 `DMAR` 位、`huart8.RxState`**
- 加入自动检测后，`dbg_uart_recover_count` 在第二轮发送时确实增加
- 确实证明：UART 的 ORE 在第二轮发送时触发

---

## 三、根因分析：UART Overrun Error (ORE)

### 什么是 ORE？

STM32F4 的 UART 有一个接收移位寄存器和接收数据寄存器（DR）。接收流程：

```
RX 引脚 → 移位寄存器（逐位接收）→ DR（接收数据寄存器）→ DMA → 内存缓冲区
```

如果电路收到一个新字节，但 DR 中的上一个字节还没被 DMA/CPU 读走，硬件就会在状态寄存器（SR）中置 ORE 标志位。**一旦 ORE 被置位，UART 硬件会停止接收后续所有字节**，直到软件明确清除 ORE 标志。

### 为什么第一轮正常、第二轮才触发？

第一轮连续发送时，DMA 持续在搬运数据，数据不会积压。

第二轮开始发送时：
- 主循环正在执行电机通信（`Motor_SendCmd_AllAngle()` 耗时约 4.4ms）
- 此时 UART8 收到数据，DMA 还没来得及读走
- 紧接着下一个字节到达 → **ORE 触发**
- ORE 一触发，后续所有数据被丢弃
- 因此只收到恢复前到达的前几帧，后面全丢

### HAL 库不会自动处理 ORE

STM32 HAL 的 UART 中断处理函数 `HAL_UART_IRQHandler()` 会检测 ORE，但 DMA 接收模式下如果处理不当，ErrorCode 会永久保存，RxState 可能变成 HAL_UART_STATE_ERROR，导致 `HAL_UART_Receive_DMA()` 返回 BUSY 无法重新启动。

---

## 四、解决方案：`UART8_Demo_RestartRx()` 自动恢复

在 `UART8_Demo_Process()` 开头增加 3 项健康检查：

```c
/* 情况 1：UART 硬件出错（ORE/FE/NE/PE） */
if (huart8.ErrorCode != HAL_UART_ERROR_NONE) {
    UART8_Demo_RestartRx(2u);
    return;
}

/* 情况 2：DMA 接收位被 HAL/错误流程关闭 */
if ((huart8.Instance->CR3 & USART_CR3_DMAR) == 0u) {
    UART8_Demo_RestartRx(3u);
    return;
}

/* 情况 3：RxState 不是 BUSY_RX（HAL 状态机异常） */
if (huart8.RxState != HAL_UART_STATE_BUSY_RX) {
    UART8_Demo_RestartRx(4u);
    return;
}
```

`UART8_Demo_RestartRx()` 执行的操作：
1. `HAL_UART_DMAStop()` — 停掉旧的 DMA 接收
2. 清除所有 UART 错误标志（ORE、FE、NE、PE、IDLE）
3. 恢复 HAL 状态（ErrorCode=NONE, RxState=READY, Lock=UNLOCKED）
4. 清空环形缓冲区（rx_tail=0, memset）
5. 重置解析器（Parser_Reset）
6. 强制设置 DMA 为 Circular 模式（防止被改回 Normal）
7. 重新启动 `HAL_UART_Receive_DMA()`
8. 重新使能 IDLE 中断

---

## 五、核心经验教训

### 1. 串口调试第一原则：先查硬件错误标志

串口出问题时，**第一步就检查**：
```
huart8.ErrorCode != HAL_UART_ERROR_NONE ？
huart8.Instance->SR（状态寄存器）？
huart8.RxState == HAL_UART_STATE_BUSY_RX ？
huart8.Instance->CR3 & USART_CR3_DMAR ？
```

不要先扎进软件逻辑（校验、缓冲、状态机）。

### 2. 识别经验模式

| 现象 | 对应硬件问题 |
|------|-------------|
| "能收几帧然后永久卡死" | **ORE（Overrun Error）** — UART 溢出 |
| "发送正常但全收不到" | DMA 未启动、CR3_DMAR 位被关、或 FE/NE |
| "已停止发送但 `dbg_byte_count` 还在增加" | 噪声数据，检查接线/接地 |
| "校验失败帧大量增加" | 波特率不匹配或帧格式不一致 |

### 3. 不要假设 HAL 会自动兜底

- HAL 的 UART 错误处理不完整，DMA 接收模式下 ORE 不会自动恢复
- 关键状态变量（ErrorCode、RxState、Lock）需要手动重置
- DMA 模式可能在错误流程中被 HAL 改回 Normal

### 4. Part 不应该花时间在错误的方向上

本次对话花大量时间纠结于：
- 速度映射公式 `/1000` 还是 `/10000`
- `V_MAX`、`V_MAX_INV`、`3.448f` 的数值
- 注释写反了的细节

这些问题虽然也要解决，但是当"接收卡死"这个问题存在时，应该先解决硬件接收问题，再调映射参数。

---

## 六、快速排查清单（给下一届）

串口通信出问题时，按顺序检查：

- [ ] **1.** `huart8.ErrorCode == 0` ？（0 表示无硬件错误）
- [ ] **2.** `huart8.Instance->CR3 & USART_CR3_DMAR` ？（非 0 表示 DMA 接收已使能）
- [ ] **3.** `huart8.RxState == HAL_UART_STATE_BUSY_RX` ？（BUSY_RX 表示正在接收）
- [ ] **4.** `dbg_byte_count` 在增加吗？（DMA 确实在搬数据）
- [ ] **5.** `dbg_dma_head` 和 `dbg_dma_tail` 在变化吗？（环形缓冲区正常读写）
- [ ] **6.** `dbg_parser_state` 是 0 还是 2 ？（0=在找帧头，2=在收数据）
- [ ] **7.** `dbg_frame_ok_total` 在增加吗？（校验通过）
- [ ] **8.** `dbg_frame_err_total` 在增加吗？（校验失败，检查波特率/帧格式）
- [ ] **9.** `dbg_uart_recover_count` > 0 ？（自动恢复被触发过，说明有 UART 错误）

---

## 七、代码架构速览

### 数据流

```
树莓派 (115200, 8N1)
    │ 8 字节帧
    ▼
UART8 PE0(RX) ← DMA1_Stream6 Circular 写入
    │
    ▼
dma_rx_buf[256]  ←── DMA 硬件自动写入（无 CPU 干预）
    │                  head = DMA counter(反算写入位置)
    ▼                  tail = 软件读取位置(rx_tail)
环形缓冲区读取(RingBuf_ReadByte)
    │
    ▼
Parser_FeedByte() — 三状态机逐字节解析
    ├─ SYNC_1: 等 0x55
    ├─ SYNC_2: 等 0xAA → 进入收集
    └─ COLLECT: 收 6 字节 → 帧完整
        │
        ▼
Parser_ValidateFrame() — 校验和验证
    │ 通过
    ▼
DispatchCommand() — 提取 velocity_v/velocity_w
    │ front_speed = velocity_v / 1000
    │ turn_omega  = velocity_w / 1000
    │ clamp 到 [-1, 1]
    ▼
motion_Mix() — 步态执行
```

### 关键变量速查

| 变量 | 类型 | 含义 | 正常范围 |
|------|------|------|----------|
| `dbg_byte_count` | u32 | 从缓冲区读出的总字节数 | 持续递增 |
| `dbg_frame_ok_total` | u32 | 校验通过的帧数 | 32bit 不回绕 |
| `dbg_frame_err_total` | u32 | 校验失败的帧数 | 应为 0 |
| `dbg_uart_recover_count` | u32 | 自动恢复次数 | **应始终为 0** |
| `dbg_uart_recover_reason` | u32 | 恢复原因(1-4) | 1=Init 正常 |
| `dbg_dma_avail` | u32 | DMA 缓冲区中可读字节数 | 0~255 |
| `dbg_parser_state` | u32 | 0=找0x55, 1=找0xAA, 2=收数据 | 正常在 0↔2 切换 |
| `front_speed` | float | 下发到步态的线速度 | [-1.0, 1.0] |
| `turn_omega` | float | 下发到步态的角速度 | [-1.0, 1.0] |

### 文件列表

| 文件 | 作用 |
|------|------|
| `Core/Src/usart_demo.c` | 主模块（接收、解析、分发） |
| `Core/Inc/usart_demo.h` | 协议常量、结构体、API 声明 |
| `Core/Src/usart.c` | UART8 硬件初始化 |
| `Core/Src/stm32f4xx_it.c` | IDLE 中断处理 |
| `Core/Src/main.c` | 调用 `UART8_Demo_Process()` |

---

## 八、原版代码 vs 稳定恢复版 对比

| 项目 | 原版 (V1) | 稳定恢复版 (V2) |
|------|----------|----------------|
| 数据触发 | 依赖 `uart8_idle_flag` 才解析 | 轮询 `RingBuf_Available()`，有数据就处理 |
| 数据区 0x55 | PARSE_COLLECT 遇到 0x55 重同步 | 直接收，不重同步 |
| 环形缓冲区 | 位掩码，要求 2 的幂 | 通用取模 |
| UART 错误恢复 | **无**，卡死后永久失效 | 每次 Process() 检查 3 种异常并自动恢复 |
| DMA 模式 | 假设 Circular | 恢复时强制 Circular |
| 调试变量 | 8 位计数（易回绕） | 32 位计数（不回绕）+ 状态快照 |
| 超时保护 | 永久注释 | 条件编译开关 |
