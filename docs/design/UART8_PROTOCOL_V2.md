# UART8 协议扩展设计 — V2 模式指令分离

> 状态：设计阶段，待算法团队对齐后实施
> 日期：2026-06-07
> 依赖：当前 V1 协议稳定运行

---

## 一、动机

### 当前 V1 协议的局限

```
V1 帧：[0x55][0xAA][CTRL=0x01][V_H][V_L][W_H][W_L][CKSUM]
```

V1 只有一个控制模式 `CTRL_SPEED(0x01)`。模式切换（走/站/跳）完全依赖遥控器物理开关（sw5/sw8），树莓派无法通过协议控制模式。

这导致以下问题：
- 树莓派想狗站立，只能发 `V=0,W=0` 然后停发，由 STM32 侧超时判断
- "发零速"和"站立姿态"是两件事，不应混为一谈
- 未来如果需要树莓派控制跳跃、匍匐等模式，没有扩展空间

### 参考设计

宇树 Unitree Go2 SDK 的设计：

```
SportClient.Move(vx, vy, vyaw)   → 运动指令（持续发送）
SportClient.StandUp()            → 姿态指令（单次调用）
SportClient.StandDown()          → 姿态指令（单次调用）
```

运动指令和姿态/模式指令是**独立的 API**，对应独立的协议字段。

---

## 二、V2 协议设计

### 帧格式保持不变

```
[0x55][0xAA][CTRL][V_H][V_L][W_H][W_L][CKSUM]
```

### 新增 CTRL 模式码

| 模式码 | 宏名 | 含义 | V/W 字段语义 |
|--------|------|------|-------------|
| `0x01` | `CTRL_SPEED` | 速度行走（V1 已有） | 目标速度 m/s×1000 |
| `0x02` | `CTRL_STAND` | 站立姿态 | 忽略（可填 0） |
| `0x03` | `CTRL_STAND_DOWN` | 匍匐/趴下 | 忽略（可填 0） |
| `0x04` | `CTRL_JUMP` | 跳跃 | 忽略（可填 0） |
| `0x10`~`0x1F` | — | 预留自定义模式 | — |

### 示例帧

```
55 AA 01 03 E8 00 00 EC  → 行走, V=1000(1.0m/s), W=0
55 AA 02 00 00 00 00 02  → 站立, V/W 忽略
55 AA 04 00 00 00 00 04  → 跳跃
```

### 校验和公式不变

```
CKSUM = (CTRL + V_H + V_L + W_H + W_L) & 0xFF
```

---

## 三、STM32 侧改动

### usart_demo.c — DispatchCommand()

```c
static void DispatchCommand(const CmdFrame_TypeDef *cmd)
{
    switch (cmd->ctrl_mode) {

    case CTRL_SPEED:   // 0x01 — 行走
        front_speed = ...;
        turn_omega  = ...;
        uart8_walk_request = 1;
        last_rx_tick = HAL_GetTick();
        break;

    case CTRL_STAND:   // 0x02 — 站立
        front_speed = 0.0f;
        turn_omega  = 0.0f;
        uart8_mode_request = MODE_STAND;
        last_rx_tick = HAL_GetTick();
        break;

    case CTRL_JUMP:    // 0x04 — 跳跃
        uart8_mode_request = MODE_JUMP;
        last_rx_tick = HAL_GetTick();
        break;

    default:
        dbg_bad_ctrl_count++;
        break;
    }
}
```

### main.c — 状态机

```c
// 遥控 RPi 模式入口（已有）
if (rcData.sw5 == 0x0320 && rcData.sw8 == 0x0000) {
    if (uart8_mode_request == MODE_STAND) {
        temp_state = 12;   // RPi 站立
    } else if (uart8_mode_request == MODE_JUMP) {
        temp_state = 13;   // RPi 跳跃
    } else {
        temp_state = uart8_walk_request ? 2 : 12;
    }
}
```

### 新增变量

```c
// usart_demo.h 或 main.c
#define MODE_NONE   0
#define MODE_STAND  1
#define MODE_JUMP   2

volatile uint8_t uart8_mode_request = MODE_NONE;
```

---

## 四、树莓派侧改动

算法团队需要在发送侧增加一个模式字段：

```python
# 当前（V1）
def send_speed(v, w):
    frame = build_frame(CTRL_SPEED, v, w)
    uart.write(frame)

# V2 新增
def send_stand():
    frame = build_frame(CTRL_STAND, 0, 0)
    uart.write(frame)

def send_jump():
    frame = build_frame(CTRL_JUMP, 0, 0)
    uart.write(frame)
```

---

## 五、实施计划

| 阶段 | 内容 | 依赖 |
|------|------|------|
| **Phase 1（当前）** | 方案 D 超时站立，仅改 STM32 | 已完成 |
| **Phase 2** | 与算法团队对齐协议，确认 V2 帧格式 | 算法配合 |
| **Phase 3** | STM32 侧实现 CTRL_STAND/CTRL_JUMP | Phase 2 |
| **Phase 4** | 树莓派侧实现对应发送函数 | Phase 3 |
| **Phase 5** | 联调测试，逐步替换超时方案 | Phase 4 |

---

## 六、向后兼容

- `CTRL_SPEED(0x01)` 行为完全不变
- 树莓派不升级 → 只发 0x01 → 和 V1 行为完全一致（超时站立仍生效）
- 树莓派升级但 STM32 未升级 → 发 0x02 会被 `default:` 计数到 `dbg_bad_ctrl_count`，不影响行走
- 双方都升级 → 完整模式控制
