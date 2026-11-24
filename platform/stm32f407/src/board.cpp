#include "stm32f407/board.hpp"

#include <cstdint>

extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

#include "stm32f4xx_hal.h"

namespace {

std::uint32_t clock_cycles_per_millisecond;

void initialize_hal_timebase()
{
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
    clock_cycles_per_millisecond = SystemCoreClock / 1000U;
}

} // namespace

namespace {

void isolate_unused_discovery_chips() { //basically tri-state the unused chips on the discovery board to reduce power consumption and avoid bus contention
    __HAL_RCC_GPIOD_CLK_ENABLE();
    __HAL_RCC_GPIOE_CLK_ENABLE();

    // 1) Hold Discovery CS43L22 Audio DAC (I2C1 addr 0x4A) in reset: PD4 = LOW
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_4, GPIO_PIN_RESET);
    GPIO_InitTypeDef pd4{};
    pd4.Pin   = GPIO_PIN_4;
    pd4.Mode  = GPIO_MODE_OUTPUT_PP;
    pd4.Pull  = GPIO_NOPULL;
    pd4.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &pd4);

    // 2) Tri-state Discovery LIS302DL/LIS3DSH accel (shares PA5 SCK / PA7 RMII): PE3 = HIGH
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_3, GPIO_PIN_SET);
    GPIO_InitTypeDef pe3{};
    pe3.Pin   = GPIO_PIN_3;
    pe3.Mode  = GPIO_MODE_OUTPUT_PP;
    pe3.Pull  = GPIO_NOPULL;
    pe3.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOE, &pe3);
}

} // namespace

extern "C" std::uint32_t HAL_GetTick()
{
    if (xTaskGetSchedulerState() == taskSCHEDULER_RUNNING) {
        return xTaskGetTickCount();
    }

    return DWT->CYCCNT / clock_cycles_per_millisecond;
}

namespace stm32f407 {

bool initialize_system_clock()
{
    initialize_hal_timebase();

    if (HAL_Init() != HAL_OK) {
        return false;
    }

    if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK) {
        return false;
    }

    RCC_OscInitTypeDef oscillator{};
    oscillator.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    oscillator.HSEState = RCC_HSE_ON;
    oscillator.PLL.PLLState = RCC_PLL_ON;
    oscillator.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    oscillator.PLL.PLLM = 8U;
    oscillator.PLL.PLLN = 336U;
    oscillator.PLL.PLLP = RCC_PLLP_DIV2;
    oscillator.PLL.PLLQ = 7U;

    if (HAL_RCC_OscConfig(&oscillator) != HAL_OK) {
        return false;
    }

    RCC_ClkInitTypeDef clock{};
    clock.ClockType = RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK |
                      RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clock.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    clock.AHBCLKDivider = RCC_SYSCLK_DIV1;
    clock.APB1CLKDivider = RCC_HCLK_DIV4;
    clock.APB2CLKDivider = RCC_HCLK_DIV2;

    if (HAL_RCC_ClockConfig(&clock, FLASH_ACR_LATENCY_5WS) != HAL_OK) {
        return false;
    }

    clock_cycles_per_millisecond = SystemCoreClock / 1000U;
    return true;
}

void initialize_board()
{
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef gpio{};
    gpio.Pin = GPIO_PIN_12;
    gpio.Mode = GPIO_MODE_OUTPUT_PP;
    gpio.Pull = GPIO_NOPULL;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(GPIOD, &gpio);
    HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_RESET);
}

void toggle_green_led()
{
    HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
}

[[noreturn]] void halt()
{
    __asm volatile ("cpsid i");
    for (;;) {
    }
}

} // namespace stm32f407
