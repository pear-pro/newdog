#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MATH_MIN(a, b) (((a) < (b)) ? (a) : (b))
#define MATH_MAX(a, b) (((a) > (b)) ? (a) : (b))
// 将 x 限幅到 [lo, hi]（含边界）
#define MATH_CLAMP(x, lo, hi) (MATH_MIN(MATH_MAX((x), (lo)), (hi)))

// 有符号/浮点绝对值
int8_t  math_absi8(int8_t x);
int16_t math_absi16(int16_t x);
int32_t math_absi32(int32_t x);
float   math_absf(float x);

// 限幅函数
int32_t  math_clampi32(int32_t x, int32_t min, int32_t max);
uint32_t math_clampu32(uint32_t x, uint32_t min, uint32_t max);
float    math_clampf(float x, float min, float max);

// 归一化到 [0, 1] 或映射到 [out_min, out_max]
float math_normalize01f(float x, float in_min, float in_max);
float math_normalize_rangef(float x, float in_min, float in_max, float out_min, float out_max);

#ifdef __cplusplus
}
#endif

#endif
