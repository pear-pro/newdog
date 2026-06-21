# IMU 调试检查清单

## 问题描述
- 探针（GPIOF GPIO_PIN_14）不亮
- IMU_Data 数组无法接收到数据

---

## 根本原因分析

### ? 已修复的问题

#### 1. **GPIOF GPIO_PIN_14 GPIO 未配置** 
- **文件**: `Core/Src/gpio.c`
- **症状**: 探针完全无响应，即使CAN中断触发也看不到闪烁
- **原因**: `MX_GPIO_Init()` 函数中缺少 GPIOF 时钟使能和引脚初始化
- **修复**: 
  ```c
  // 已添加:
  __HAL_RCC_GPIOF_CLK_ENABLE();  // GPIOF 时钟使能
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOF, &GPIO_InitStruct);
  ```

---

## 待诊断的潜在问题

### 2. **CAN数据源问题** ??
如果修复GPIO后探针仍不闪烁，进行以下检查：

#### 2.1 验证IMU设备ID和波特率
- **检查点**: 
  - IMU硬件实际发送的CAN ID是否为 `0x50`？
  - IMU和STM32的CAN波特率是否匹配？
  
- **配置位置**: `Core/Src/can.c` (第48-58行)
  ```c
  can_filter_st.FilterIdHigh = (0x50 << 5);     // 期望接收 CAN ID 0x50
  can_filter_st.FilterMaskIdHigh = (0x7FF << 5); // 掩码设置
  ```

#### 2.2 CAN物理连接检查
- CAN_H 和 CAN_N 线是否正确连接？
- 是否有 120Ω 终端电阻？
- CAN总线是否被其他设备占用或冲突？

#### 2.3 检查CAN中断链路
```
CAN1_RX0_IRQHandler (stm32f4xx_it.c:240)
   ↓
HAL_CAN_IRQHandler(&hcan1)
   ↓
HAL_CAN_RxFifo0MsgPendingCallback (stm32f4xx_it.c:314)
   ↓
switch (rx_header.StdId) → case 0x50
   ↓
IMU_Parse_CAN(rx_header.StdId, rx_data)
   ↓
HAL_GPIO_TogglePin(GPIOF, GPIO_PIN_14)  ← 探针翻转
```

---

## 验证步骤

### 步骤1: 编译并烧写修复后的代码
```bash
# 在项目根目录执行
cd build
cmake --build .
# 使用你的编程工具（如J-Link或ST-Link）烧写生成的固件
```

### 步骤2: 观察探针现象
- **正常情况**: 连接IMU后，探针每20帧CAN消息翻转一次（在[imu.c](Core/Src/imu.c#L99)裡有计数器）
- **预期频率**: IMU运行频率通常为100Hz，所以探针应该以0.2秒（200ms）周期闪烁

### 步骤3: 如果探针仍不闪烁
进行以下调试：

#### 3.1 添加调试代码验证CAN中断是否被触发
```c
// 临时在 HAL_CAN_RxFifo0MsgPendingCallback 中添加
static uint32_t can_int_count = 0;
can_int_count++;
HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_1);  // 用GPIOG的已配置引脚替代
```

#### 3.2 检查IMU发送的实际CAN ID
连接逻辑分析仪或CAN总线分析工具，查看：
- IMU是否在发送数据？
- 实际CAN ID是多少？
- 数据帧格式是否匹配协议？

---

## 文件修改清单

| 文件 | 修改内容 | 状态 |
|------|--------|------|
| `Core/Src/gpio.c` | 添加GPIOF时钟使能和GPIO_PIN_14初始化 | ? 已修复 |
| `Core/Src/can.c` | CAN过滤器配置（检查CAN ID 0x50） | ?? 需验证 |
| `Core/Src/stm32f4xx_it.c` | CAN中断处理回调 | ? 配置正确 |
| `Core/Src/imu.c` | IMU数据解析函数 | ? 代码正确 |

---

## 相关代码位置

- **GPIO配置**: [Core/Src/gpio.c](Core/Src/gpio.c)
- **CAN初始化**: [Core/Src/can.c](Core/Src/can.c#L48)
- **CAN中断处理**: [Core/Src/stm32f4xx_it.c](Core/Src/stm32f4xx_it.c#L240)
- **IMU数据解析**: [Core/Src/imu.c](Core/Src/imu.c#L33)
- **IMU头文件**: [Core/Inc/imu.h](Core/Inc/imu.h)

---

## 常见问题 (FAQ)

**Q: 修复GPIO后仍然没有数据？**
A: 检查IMU设备是否实际连接并通电，CAN总线是否有数据在传输，查看硬件连接。

**Q: 如何判断是硬件问题还是软件问题？**
A: 使用CAN分析工具（如PCAN-View）监控总线，如果看不到0x50 ID的报文，则是硬件连接或IMU配置问题。

**Q: IMU数据帧的预期内容是什么？**
A: 详见[imu.c](Core/Src/imu.c#L56)的IMU_Parse_CAN函数，支持0x51(加速度)、0x52(角速度)、0x53(欧拉角)三种报文类型。

