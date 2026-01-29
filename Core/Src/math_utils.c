#include "math_utils.h"

// 取绝对值；注意 INT8_MIN 会溢出 1（补码特性）
int8_t math_absi8(int8_t x)
{
    return (x < 0) ? (int8_t)(-x) : x;
}

// 取绝对值；注意 INT16_MIN 会溢出 1（补码特性）
int16_t math_absi16(int16_t x)
{
    return (x < 0) ? (int16_t)(-x) : x;
}

// 取绝对值；注意 INT32_MIN 会溢出 1（补码特性）
int32_t math_absi32(int32_t x)
{
    return (x < 0) ? -x : x;
}

// 浮点绝对值
float math_absf(float x)
{
    return (x < 0.0f) ? -x : x;
}

// 有符号整型限幅到 [min, max]
int32_t math_clampi32(int32_t x, int32_t min, int32_t max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

// 无符号整型限幅到 [min, max]
uint32_t math_clampu32(uint32_t x, uint32_t min, uint32_t max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

// 浮点限幅到 [min, max]
float math_clampf(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}

// 将 x 从 [in_min, in_max] 归一化到 [0, 1]；区间为 0 时返回 0
float math_normalize01f(float x, float in_min, float in_max)
{
    float denom = in_max - in_min;
    if (denom == 0.0f) return 0.0f;
    return (x - in_min) / denom;
}

// 先归一化，再映射到 [out_min, out_max]
float math_normalize_rangef(float x, float in_min, float in_max, float out_min, float out_max)
{
    float t = math_normalize01f(x, in_min, in_max);
    return out_min + t * (out_max - out_min);
}
