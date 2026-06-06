#ifndef PG_LED_H
#define PG_LED_H

#include "stm32f4xx_hal.h"

/* PG1-PG8 are active-low LEDs (low = on). */

#define PG_LED_PINS (GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3 | GPIO_PIN_4 | \
                     GPIO_PIN_5 | GPIO_PIN_6 | GPIO_PIN_7 | GPIO_PIN_8)

void Leds_On(void);
void Leds_Off(void);
void Leds_Toggle(void);
void Led_Toggle(char c);
void Led_Set(char c);
void LED_Off(char c);

#endif
