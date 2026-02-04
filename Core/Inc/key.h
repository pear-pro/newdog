#ifndef KEY_H
#define KEY_H

#include "main.h"

#ifdef __cplusplus
extern "C" {
#endif

#define KEY_PB2_PORT GPIOB
#define KEY_PB2_PIN  GPIO_PIN_2
#define KEY_DEBOUNCE_MS 20U

typedef enum
{
    KEY_RELEASED = 0,
    KEY_PRESSED  = 1
} key_state_t;

void key_init(void);
void key_update(void);
key_state_t key_get_state(void);
uint8_t key_is_pressed(void);
uint8_t key_pressed_event(void);
uint8_t key_released_event(void);
void key_exti_callback(uint16_t gpio_pin);

#ifdef __cplusplus
}
#endif

#endif
