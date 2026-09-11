/* vmc.c — VMC（虚拟模型控制）模块
 *
 * 原理：把每条支撑腿看成一个"重力补偿 + 虚拟弹簧-阻尼"元件，
 *       在足端构造期望虚拟力 F = (Fx, Fz)，通过力雅可比转置 Jᵀ 映射为
 *       两个髋电机的力矩前馈 τ = Jᵀ·F，与电机固件内的位置 PD 叠加：
 *           支撑相控制律 = Kp·(θ_des−θ) + Kw·(0−ω) + Tau_ff(=JᵀF)
 *       摆动相保持纯位置控制（Tau_ff = 0，由 set_Motor_Kp 置位）。
 *
 * 坐标系：腿平面 (x 向前, z 向下)，与 kinematic.c 逆解帧一致
 *        （hposition.B_y > 0 表示足端在髋轴心下方）。
 * 角度约定：hposition.alpha/beta 为"度"，0 = 曲柄竖直向下（逆解输出）。
 *        本模块内部把关节角换算为从 +x 指向 +z(下) 的 θ 再喂给雅可比公式：
 *            θ_α = α + 90° ，θ_β = 90° − β
 *        （由 kinematic.c 逆解中的 fmod(alpha−90) / fmod(90−beta) 推出）。
 * 执行位置：TIM10 100Hz 中断（tim.c），固定周期、与自由运行的主循环解耦；
 *        只写 hmotorX.Tau_ff（32 位字原子写），主循环打包下发时取最新值。
 */
#include "vmc.h"
#include <math.h>
#include <stdint.h>
#include "gait.h"
#include "imu.h"
#include "global_var.h"
#include "motor.h"
#include "tim.h"

/* ════════════════════ 调参区（沿用 gait.c 全局调参风格）════════════════════ */

float VMC_MASS_KG = 3.0f;    // 整机质量(kg)，重力补偿 = m·g / 支撑腿数
#define VMC_G 9.8f            // 重力加速度


float vmc_kp_roll  = 0.0f;    // 横滚虚拟弹簧 N/deg（悬吊/站立测试通过后再调大）
float vmc_kd_roll  = 0.0f;    // 横滚虚拟阻尼 N/(deg/s)，输入 GyroX
float vmc_kp_pitch = 0.0f;    // 俯仰虚拟弹簧 N/deg
float vmc_kd_pitch = 0.0f;    // 俯仰虚拟阻尼 N/(deg/s)，输入 GyroY
float vmc_roll_dir  =  1.0f;  // 横滚补偿整体方向 ±1（IMU 安装方向实测后定：
                              //   若补偿方向反了，把对应 dir 取反再烧录验证）
float vmc_pitch_dir =  1.0f;  // 俯仰补偿整体方向 ±1（同上）
float vmc_kv = 0.0f;          // 足端水平阻尼 N/(m/s)，默认关（VeloY 为加速度积分，漂移较大）

#define VMC_DELTA_F_MAX 10.0f // 姿态补偿力限幅 N（防 IMU 异常时冲击）
#define VMC_TAU_MAX     1.0f  // 单电机 VMC 力矩限幅 N·m
                              // 参考值：10kg 四足站立 α 电机 ≈0.95 N·m，trot 双支撑 ≈1.9 N·m

/* 连杆长度(cm)，与 kinematic.c 的 L1..L4 完全一致：
 *   rod[0]=L1=12.5 α曲柄; rod[1]=L2=26 α侧连杆; rod[2]=L4=12.5 β曲柄; rod[3]=L3=26 β侧连杆 */
static const float vmc_rod[4] = {12.5f, 26.0f, 12.5f, 26.0f};

/* ════════════════════════ 每腿参数表 ════════════════════════
 * 腿序 0..3 = FR/BR/BL/FL = hposition1..4
 *   alpha_dir : α 电机力矩方向。对应 motor.c Motor_SendCmd_AllAngle 中
 *               Theta_des 与 hposition.alpha 的符号关系（1,2,5,6 号电机取反）
 *   beta_dir  : β 电机力矩方向（同上）
 *   x_sign    : 腿系 +x 与机体前方的关系。inverseKinematic_All 对后腿(2,3)取反 B_x，
 *               故后腿腿系 +x 指向身体后方 → x_sign = −1
 *   roll_sign : 横滚补偿力分配（右腿 +，左腿 −）
 *   pitch_sign: 俯仰补偿力分配（前腿 +，后腿 −）
 * 悬吊实验若发现某腿补偿方向反了，改该腿的 alpha_dir/beta_dir 符号即可 */
typedef struct {
    float alpha_dir;
    float beta_dir;
    float x_sign;
    float roll_sign;
    float pitch_sign;
} VmcLegParam_t;

static const VmcLegParam_t vmc_leg_param[4] = {
    /*          FR(h1)          BR(h2)          BL(h3)          FL(h4)   */
    { -1.0f, -1.0f, +1.0f, +1.0f, +1.0f },   /* FR: hmotor1(α) hmotor2(β) 均取反 */
    { +1.0f, +1.0f, -1.0f, +1.0f, -1.0f },   /* BR: hmotor3(α) hmotor4(β) */
    { -1.0f, -1.0f, -1.0f, -1.0f, -1.0f },   /* BL: hmotor5(α) hmotor6(β) 均取反 */
    { +1.0f, +1.0f, +1.0f, -1.0f, +1.0f },   /* FL: hmotor7(α) hmotor8(β) */
};

/* ════════════════════════ 全局状态 ════════════════════════ */

uint8_t vmc_active = 1;         /* main.c 按 temp_state 维护 */
uint8_t vmc_leg_state[4] = {0}; /* gait.c set_Motor_Kp 每轮更新 */
static uint8_t vmc_was_active = 0; /* 上一周期活跃标志（退出时清力矩用） */

/* 腿与电机句柄的映射（与 main.c 注释的 hposition↔hmotor 关系一致） */
static Motor_HandleTypeDef* const vmc_motor_alpha[4] = { &hmotor1, &hmotor3, &hmotor5, &hmotor7 };
static Motor_HandleTypeDef* const vmc_motor_beta[4]  = { &hmotor2, &hmotor4, &hmotor6, &hmotor8 };
static Position_HandleTypeDef* const vmc_hposition[4] = { &hposition1, &hposition2, &hposition3, &hposition4 };

/* ════════════════════════ 内部函数 ════════════════════════ */

/* 清空全部腿电机力矩前馈（退出 VMC / 无支撑腿 / 急停时调用，防残留力矩突变） */
static void vmc_clear_torque(void)
{
    hmotor1.Tau_ff = 0.0f;
    hmotor2.Tau_ff = 0.0f;
    hmotor3.Tau_ff = 0.0f;
    hmotor4.Tau_ff = 0.0f;
    hmotor5.Tau_ff = 0.0f;
    hmotor6.Tau_ff = 0.0f;
    hmotor7.Tau_ff = 0.0f;
    hmotor8.Tau_ff = 0.0f;
}

/* ── 五连杆腿雅可比：足端虚拟力 (Fx,Fz) → 关节力矩 (tau_a, tau_b) ──
 * 复现参考版 VMC_Jacobi_Matrix，与参考版的差异（均已数值验证）：
 * 1) 参考版第 1 行括号内为 sin_Phi2·cos(Beta) − sin(Beta)·cos_Phi1，
 *    数学推导应为 sin(Φ2−β) = sinΦ2·cosβ − cosΦ2·sinβ，
 *    已修正 cos_Phi1 → cos_Phi2（有限差分数值验证：修正后与数值导数误差 ~1e-6）。
 * 2) 参考版 sin_Phi1 = (−C − A·cosΦ1)/B 在 B=0（两曲柄末端等高）时除零；
 *    改用代数等价的对称式 sin_Phi1 = (−C·B + A·√Δ)/(A²+B²)，B=0 时数值安全。
 * 3) 分支符号：本模块腿系 z 向下，足端侧对应 "+√" 分支
 *    （参考代码 z 向上坐标系里的 −√ 即本坐标系的 +√）。
 * 4) β 行雅可比整体取负：θ_β = 90°−β_local → ∂/∂β_local = −∂/∂θ_β。
 * 5) 增加机械奇异防护（两曲柄平行/腿完全伸直时雅可比无穷大），触发时返回 0，
 *    调用方保持该腿上次力矩（力矩连续不突变）。
 * 全链路已通过 PC 数值验证：真实逆解输出 → θ换算 → 分支重建足端 → 虚功原理，
 * 73 个工作区网格点全部一致。
 *
 * 数学推导：设 C 为足端，|C−B|=rod[1]、|C−D|=rod[3] 两圆交点，
 *   代入 C = B + rod[1]·(cosΦ1, sinΦ1) 消元得 A·cosΦ1 + B·sinΦ1 = −C，
 *   标准闭式解取"+√"分支 = 足端下垂构型（z 向下坐标系，与逆解所选分支一致；
 *   参考代码 z 向上坐标系中的 −√ 对应本坐标系的 +√）。
 *   雅可比 J = [∂(xc,zc)/∂α; ∂(xc,zc)/∂β]，力矩 τ = Jᵀ·F（虚功原理：
 *   F·ẋ = F·J·q̇ = (JᵀF)·q̇ = τ·q̇）。
 *
 * 入参：alpha_deg/beta_deg = hposition 指令角（度，逆解输出）；
 *       Fx/Fz = 足端虚拟力（N，腿系：x 向前、z 向下，Fz>0 = 向下推地）。
 * 出参：tau_a/tau_b = α/β 电机力矩前馈（N·m，已乘方向系数并限幅）。
 * 返回：1=有效，0=奇异/不可达（保持上次力矩）。 */
static uint8_t vmc_leg_torque(int leg_idx, float alpha_deg, float beta_deg,
                              float Fx, float Fz, float *tau_a, float *tau_b)
{
    /* 关节角(度) → 从 +x 指向 +z(下) 的 θ（换算依据见文件头注释） */
    float alpha = (alpha_deg + 90.0f) * 3.141592f / 180.0f;
    float beta  = (90.0f - beta_deg)  * 3.141592f / 180.0f;

    float xb, xd, zb, zd;
    float A, B, C;
    float sin_Phi1, sin_Phi2, cos_Phi1, cos_Phi2;

    /* 求 Φ1、Φ2（和运动学正解一模一样：两圆交点闭式解） */
    xb = vmc_rod[0] * cosf(alpha);
    xd = vmc_rod[2] * cosf(beta);
    zb = vmc_rod[0] * sinf(alpha);
    zd = vmc_rod[2] * sinf(beta);

    A = 2.0f * vmc_rod[1] * (xb - xd);
    B = 2.0f * vmc_rod[1] * (zb - zd);
    C = vmc_rod[1]*vmc_rod[1] + (xb-xd)*(xb-xd) + (zb-zd)*(zb-zd)
      - vmc_rod[3]*vmc_rod[3];

    /* 防护1：两曲柄末端重合（B==D，如 α=−β 位形）→ A²+B²=0 除零 */
    float R2 = A*A + B*B;
    if (R2 < 1e-6f) return 0;
    /* 防护2：两圆不相交（判别式<0，编码器噪声/不可达位形） */
    float disc = R2 - C*C;
    if (disc < 0.0f) return 0;

    /* 闭式解：取 "+√" 分支 = 足端下垂构型（与逆解所选分支一致）。
     * 注意：本模块腿系 z 向下，足端侧对应 +√ 分支；
     * 参考代码（z 向上坐标系）用的 −√ 在 z 向下坐标系里是轴心上方那支。
     * sin 用与 cos 配对的对称式，B=0 时数值安全 */
    cos_Phi1 = (-C*A + B*sqrtf(disc)) / R2;
    sin_Phi1 = (-C*B - A*sqrtf(disc)) / R2;

    cos_Phi2 = (xb - xd + vmc_rod[1]*cos_Phi1) / vmc_rod[3];
    sin_Phi2 = (zb - zd + vmc_rod[1]*sin_Phi1) / vmc_rod[3];

    /* 防护3：sin(Φ1−Φ2)→0（两连杆共线，腿伸直/折叠的奇异位形）→ 雅可比无穷大 */
    float sin_Phi1_Phi2 = sin_Phi1*cos_Phi2 - sin_Phi2*cos_Phi1;
    if (fabsf(sin_Phi1_Phi2) < 0.05f) return 0;

    /* 构造雅可比矩阵（∂(xc,zc)/∂关节角，关节角 = hposition 的 α/β）
     * [ Jacobi[0][0] Jacobi[0][1] ] [Fx]
     * [ Jacobi[1][0] Jacobi[1][1] ] [Fz]
     * 行0 = ∂(xc,zc)/∂α；行1 = ∂(xc,zc)/∂β
     * 注意：θ_β = 90°−β_local → ∂/∂β_local = −∂/∂θ_β，
     *       故行1在 θ_β 求导结果基础上整体取负 */
    float Jacobi[2][2];
    Jacobi[0][0] =  vmc_rod[0]*sin_Phi2*(sinf(alpha)*cos_Phi1 - sin_Phi1*cosf(alpha))/sin_Phi1_Phi2;
    Jacobi[1][0] = -vmc_rod[2]*sin_Phi1*(sin_Phi2*cosf(beta)  - cos_Phi2*sinf(beta))/sin_Phi1_Phi2;
    Jacobi[0][1] = -vmc_rod[0]*cos_Phi2*(sinf(alpha)*cos_Phi1 - sin_Phi1*cosf(alpha))/sin_Phi1_Phi2;
    Jacobi[1][1] =  vmc_rod[2]*cos_Phi1*(sin_Phi2*cosf(beta)  - cos_Phi2*sinf(beta))/sin_Phi1_Phi2;

    /* τ = Jᵀ·F；杆长单位 cm → 力矩 N·cm → ×0.01 转 N·m（Tau_ff 协议单位）
     * 再乘电机方向系数（与 motor.c Theta_des 符号表一致），最后限幅 */
    float tau_alpha = (Fx*Jacobi[0][0] + Fz*Jacobi[0][1]) * 0.01f * vmc_leg_param[leg_idx].alpha_dir;
    float tau_beta  = (Fx*Jacobi[1][0] + Fz*Jacobi[1][1]) * 0.01f * vmc_leg_param[leg_idx].beta_dir;

    if (tau_alpha >  VMC_TAU_MAX) tau_alpha =  VMC_TAU_MAX;
    if (tau_alpha < -VMC_TAU_MAX) tau_alpha = -VMC_TAU_MAX;
    if (tau_beta  >  VMC_TAU_MAX) tau_beta  =  VMC_TAU_MAX;
    if (tau_beta  < -VMC_TAU_MAX) tau_beta  = -VMC_TAU_MAX;

    *tau_a = tau_alpha;
    *tau_b = tau_beta;
    return 1;
}

/* ════════════════════════ 对外接口 ════════════════════════ */

/* VMC 主更新函数：TIM10 100Hz 中断调用 */
void VMC_Update(void)
{
    /* 未启用/非 VMC 模式/急停 → 退出并清力矩（防恢复时残留力矩突变） */
    if (!VMC_ENABLE || !vmc_active || emergency_stop) {
        if (vmc_was_active) {
            vmc_clear_torque();
        }
        vmc_was_active = 0;
        return;
    }
    vmc_was_active = 1;

    /* 统计支撑腿数（重力补偿按支撑腿数均摊：
     * 四足站立 = 24.5N/腿；trot 双支撑 = 49N/腿） */
    uint8_t n_support = 0;
    for (uint8_t i = 0; i < 4; i++) {
        if (vmc_leg_state[i]) n_support++;
    }
    if (n_support == 0) {
        vmc_clear_torque();
        return;
    }

    /* 姿态虚拟力（PD）：body_roll/body_pitch 单位度、GyroX/Y 单位 °/s
     * roll 分配到左右腿、pitch 分配到前后腿（符号见 vmc_leg_param 表）
     * vmc_roll_dir/vmc_pitch_dir 为整体方向修正（IMU 安装方向实测后定） */
    float dF_roll  = vmc_roll_dir  * (vmc_kp_roll  * body_roll  + vmc_kd_roll  * GyroX);
    float dF_pitch = vmc_pitch_dir * (vmc_kp_pitch * body_pitch + vmc_kd_pitch * GyroY);
    if (dF_roll  >  VMC_DELTA_F_MAX) dF_roll  =  VMC_DELTA_F_MAX;
    if (dF_roll  < -VMC_DELTA_F_MAX) dF_roll  = -VMC_DELTA_F_MAX;
    if (dF_pitch >  VMC_DELTA_F_MAX) dF_pitch =  VMC_DELTA_F_MAX;
    if (dF_pitch < -VMC_DELTA_F_MAX) dF_pitch = -VMC_DELTA_F_MAX;

    float Fz_grav = VMC_MASS_KG * VMC_G / (float)n_support;

    for (uint8_t i = 0; i < 4; i++) {
        if (!vmc_leg_state[i]) {
            continue;   /* 摆动腿不写（set_Motor_Kp 已置 Tau_ff=0） */
        }

        /* 足端虚拟力（腿系：x 向前、z 向下；Fz>0 = 向下推地 = 支撑身体） */
        float Fx = -vmc_kv * VeloY * vmc_leg_param[i].x_sign;  /* 水平阻尼，默认关 */
        float Fz = Fz_grav
                 + vmc_leg_param[i].roll_sign  * dF_roll
                 + vmc_leg_param[i].pitch_sign * dF_pitch;

        float tau_a = 0.0f, tau_b = 0.0f;
        if (vmc_leg_torque(i, vmc_hposition[i]->alpha, vmc_hposition[i]->beta,
                           Fx, Fz, &tau_a, &tau_b)) {
            vmc_motor_alpha[i]->Tau_ff = tau_a;
            vmc_motor_beta[i]->Tau_ff  = tau_b;
        }
        /* 奇异防护触发 → 保持该腿上次力矩 */
    }
}
