#include "key.h"
#include "pg_led.h"
#include "motor.h"

static key_state_t key_state = KEY_RELEASED;
static GPIO_PinState raw_state = GPIO_PIN_RESET;
static uint32_t last_change_ms = 0;
static uint8_t pressed_event = 0;
static uint8_t released_event = 0;
static uint32_t last_irq_ms = 0;

void key_init(void)
{
    key_state = KEY_RELEASED;
    raw_state = HAL_GPIO_ReadPin(KEY_PB2_PORT, KEY_PB2_PIN);
    last_change_ms = HAL_GetTick();
    pressed_event = 0;
    released_event = 0;
}

void key_update(void)
{
    GPIO_PinState raw = HAL_GPIO_ReadPin(KEY_PB2_PORT, KEY_PB2_PIN);
    uint32_t now = HAL_GetTick();

    if (raw != raw_state)
    {
        raw_state = raw;
        last_change_ms = now;
    }

    if ((now - last_change_ms) >= KEY_DEBOUNCE_MS)
    {
        key_state_t new_state = (raw_state == GPIO_PIN_SET) ? KEY_PRESSED : KEY_RELEASED;
        if (new_state != key_state)
        {
            key_state = new_state;
            if (key_state == KEY_PRESSED)
            {
                pressed_event = 1;
            }
            else
            {
                released_event = 1;
            }
        }
    }
}

key_state_t key_get_state(void)
{
    return key_state;
}

uint8_t key_is_pressed(void)
{
    return (key_state == KEY_PRESSED) ? 1U : 0U;
}

uint8_t key_pressed_event(void)
{
    uint8_t ret = pressed_event;
    pressed_event = 0;
    return ret;
}

uint8_t key_released_event(void)
{
    uint8_t ret = released_event;
    released_event = 0;
    return ret;
}

void key_exti_callback(uint16_t gpio_pin)
{
    if (gpio_pin != KEY_PB2_PIN)
    {
        return;
    }

    uint32_t now = HAL_GetTick();
    if ((now - last_irq_ms) < KEY_DEBOUNCE_MS)
    {
        return;
    }
    last_irq_ms = now;

    raw_state = HAL_GPIO_ReadPin(KEY_PB2_PORT, KEY_PB2_PIN);
    if (raw_state == GPIO_PIN_SET)
    {
        key_state = KEY_PRESSED;
        pressed_event = 1;
        Led_Set('A');
    }
    else
    {
        key_state = KEY_RELEASED;
        released_event = 1;
		Led_Set('a');
    }
}
