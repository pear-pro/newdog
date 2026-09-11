#ifndef __VMC__
#define __VMC__

#include <stdint.h>

/* ── VMC 总开关（代码开关）：上机验证稳定后再考虑绑遥控器 ── */
#define VMC_ENABLE 1

/* VMC 是否作用于当前模式（由 main.c 按 temp_state 维护：跳跃等爆发动作置 0） */
extern uint8_t vmc_active;

/* 4 腿支撑/摆动状态：1=支撑 0=摆动，由 gait.c set_Motor_Kp 每轮更新
 * 腿序: [0]=FR(hposition1) [1]=BR(hposition2) [2]=BL(hposition3) [3]=FL(hposition4) */
extern uint8_t vmc_leg_state[4];

/* VMC 主更新函数（TIM10 100Hz 中断调用）：
 * 支撑腿足端虚拟力 → JᵀF 力矩前馈 → 写入 hmotorX.Tau_ff（N·m），
 * 与电机固件内的位置 PD（Kp/Kw）叠加构成支撑相控制律 */
void VMC_Update(void);

#endif
