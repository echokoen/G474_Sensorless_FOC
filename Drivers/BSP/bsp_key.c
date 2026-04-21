/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    bsp_key.c
  * @brief   Start/Stop key processing.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "bsp_key.h"
#include "app_foc.h"
#include "foc_config.h"
#include "stm32g4xx_hal.h"
#include <stdio.h>

#define BSP_KEY_DEBOUNCE_MS          (20u)
#define BSP_KEY_STARTSTOP_DEBOUNCE_MS (300u)
#define BSP_KEY_LONGPRESS_MS         (500u)
#define BSP_KEY_REPEAT_MS            (120u)
#define BSP_KEY_SPEED_STEP_SHORT_HZ  (0.5f)
#define BSP_KEY_SPEED_STEP_LONG_HZ   (0.2f)
#define BSP_KEY_SPEED_MIN_HZ         (16.667f)   /* 1000 rpm */
#define BSP_KEY_SPEED_MAX_HZ         (50.0f)     /* 3000 rpm */

typedef struct
{
    uint8_t stable_pressed;
    uint8_t longpress_started;
    uint32_t debounce_tick;
    uint32_t press_tick;
    uint32_t repeat_tick;
} BSP_KeyRepeatState_t;

static uint32_t g_key_last_tick = 0u;
static BSP_KeyRepeatState_t g_key0_state = {0};
static BSP_KeyRepeatState_t g_key1_state = {0};
static float g_speed_target_mech_hz = FOC_SPEED_REF_MECH_HZ;

static void BSP_KEY_ProgramSpeedTarget(float mech_hz);

static float bsp_key_clampf(float x, float lo, float hi)
{
    if (x < lo)
    {
        return lo;
    }
    if (x > hi)
    {
        return hi;
    }
    return x;
}

static void BSP_KEY_HandleSpeedStep(BSP_KeyRepeatState_t *state, uint8_t pressed, float short_step_hz, float long_step_hz)
{
    uint32_t now_tick = HAL_GetTick();

    if (pressed == 0u)
    {
        state->stable_pressed = 0u;
        state->longpress_started = 0u;
        state->debounce_tick = now_tick;
        state->press_tick = 0u;
        state->repeat_tick = 0u;
        return;
    }

    if (state->stable_pressed == 0u)
    {
        if ((now_tick - state->debounce_tick) < BSP_KEY_DEBOUNCE_MS)
        {
            return;
        }

        state->stable_pressed = 1u;
        state->longpress_started = 0u;
        state->press_tick = now_tick;
        state->repeat_tick = now_tick;
        BSP_KEY_ProgramSpeedTarget(g_speed_target_mech_hz + short_step_hz);
        return;
    }

    if (state->longpress_started == 0u)
    {
        if ((now_tick - state->press_tick) >= BSP_KEY_LONGPRESS_MS)
        {
            state->longpress_started = 1u;
            state->repeat_tick = now_tick;
        }
        return;
    }

    if ((now_tick - state->repeat_tick) >= BSP_KEY_REPEAT_MS)
    {
        state->repeat_tick = now_tick;
        BSP_KEY_ProgramSpeedTarget(g_speed_target_mech_hz + long_step_hz);
    }
}

static void BSP_KEY_ProgramSpeedTarget(float mech_hz)
{
    g_speed_target_mech_hz = bsp_key_clampf(mech_hz,
                                            BSP_KEY_SPEED_MIN_HZ,
                                            BSP_KEY_SPEED_MAX_HZ);
    AppFoc_SetSpeedTargetMechHz(g_speed_target_mech_hz);
    printf("[KEY] speed_target=%.2f Hz\r\n", g_speed_target_mech_hz);
}

void BSP_KEY_Init(void)
{
    g_key_last_tick = 0u;
    g_key0_state = (BSP_KeyRepeatState_t){0};
    g_key1_state = (BSP_KeyRepeatState_t){0};
    g_speed_target_mech_hz = FOC_SPEED_REF_MECH_HZ;
}

void BSP_KEY_SetSpeedTargetHz(float mech_hz)
{
    g_speed_target_mech_hz = bsp_key_clampf(mech_hz,
                                            BSP_KEY_SPEED_MIN_HZ,
                                            BSP_KEY_SPEED_MAX_HZ);
}

void BSP_KEY_Task(void)
{
    uint8_t key0_pressed = (HAL_GPIO_ReadPin(KEY0_GPIO_Port, KEY0_Pin) == GPIO_PIN_RESET) ? 1u : 0u;
    uint8_t key1_pressed = (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET) ? 1u : 0u;

    BSP_KEY_HandleSpeedStep(&g_key0_state,
                            key0_pressed,
                            BSP_KEY_SPEED_STEP_SHORT_HZ,
                            BSP_KEY_SPEED_STEP_LONG_HZ);
    BSP_KEY_HandleSpeedStep(&g_key1_state,
                            key1_pressed,
                            -BSP_KEY_SPEED_STEP_SHORT_HZ,
                            -BSP_KEY_SPEED_STEP_LONG_HZ);
}

void BSP_KEY_EXTI_Callback(uint16_t GPIO_Pin)
{
    uint32_t now_tick;

    if (GPIO_Pin != Start_Stop_Pin)
    {
        return;
    }

    now_tick = HAL_GetTick();
    if ((now_tick - g_key_last_tick) < BSP_KEY_STARTSTOP_DEBOUNCE_MS)
    {
        return;
    }

    if (HAL_GPIO_ReadPin(Start_Stop_GPIO_Port, Start_Stop_Pin) != GPIO_PIN_RESET)
    {
        return;
    }

    g_key_last_tick = now_tick;

    /* 直接依据当前状态决定启停，避免上电自动启动后状态不同步。 */
    if (AppFoc_GetState() == FOC_STATE_IDLE)
    {
        AppFoc_Start();
        printf("[KEY] AppFoc_Start()\r\n");
    }
    else
    {
        AppFoc_Stop();
        printf("[KEY] AppFoc_Stop()\r\n");
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    BSP_KEY_EXTI_Callback(GPIO_Pin);
}
