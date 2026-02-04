#include "pg_led.h"

void Leds_On(void)
{
    HAL_GPIO_WritePin(GPIOG, PG_LED_PINS, GPIO_PIN_RESET);
}

void Leds_Off(void)
{
    HAL_GPIO_WritePin(GPIOG, PG_LED_PINS, GPIO_PIN_SET);
}

void Leds_Toggle(void)
{
    HAL_GPIO_TogglePin(GPIOG, PG_LED_PINS);
}

void Led_Toggle(char c)
{
    switch (c)
    {
        case 'A':
        case 'a':
            HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_1);
            break;
        case 'B':
        case 'b':
            HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_2);
            break;
        case 'C':
        case 'c':
            HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_3);
            break;
        case 'D':
        case 'd':
            HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_4);
            break;
        case 'E':
        case 'e':
            HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_5);
            break;
        case 'F':
        case 'f':
            HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_6);
            break;
        case 'G':
        case 'g':
            HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_7);
            break;
        case 'H':
        case 'h':
            HAL_GPIO_TogglePin(GPIOG, GPIO_PIN_8);
            break;
        default:
            break;
    }
}

void Led_Set(char c)
{
    GPIO_PinState state;

    if (c >= 'A' && c <= 'H')
    {
        state = GPIO_PIN_RESET; // active-low: ON
    }
    else if (c >= 'a' && c <= 'h')
    {
        state = GPIO_PIN_SET;   // active-low: OFF
    }
    else
    {
        return;
    }

    switch (c)
    {
        case 'A':
        case 'a':
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, state);
            break;
        case 'B':
        case 'b':
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, state);
            break;
        case 'C':
        case 'c':
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, state);
            break;
        case 'D':
        case 'd':
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, state);
            break;
        case 'E':
        case 'e':
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, state);
            break;
        case 'F':
        case 'f':
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, state);
            break;
        case 'G':
        case 'g':
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, state);
            break;
        case 'H':
        case 'h':
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, state);
            break;
        default:
            break;
    }
}

void LED_Off(char c)
{
    if (c < 'a' || c > 'h')
    {
        return;
    }

    switch (c)
    {
        case 'a':
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_1, GPIO_PIN_SET);
            break;
        case 'b':
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_2, GPIO_PIN_SET);
            break;
        case 'c':
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET);
            break;
        case 'd':
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_4, GPIO_PIN_SET);
            break;
        case 'e':
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_SET);
            break;
        case 'f':
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);
            break;
        case 'g':
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_7, GPIO_PIN_SET);
            break;
        case 'h':
            HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, GPIO_PIN_SET);
            break;
        default:
            break;
    }
}
