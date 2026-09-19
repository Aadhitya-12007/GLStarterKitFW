#include "stm32f407/console.hpp"

#include <cstddef>
#include <cstdint>

extern "C" {
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "task.h"
}

#include "stm32f4xx_hal.h"

namespace {

constexpr std::size_t queue_size = 1024U;
constexpr std::size_t chunk_size = 64U;
constexpr configSTACK_DEPTH_TYPE console_stack_depth = 256U;
constexpr UBaseType_t console_task_priority = tskIDLE_PRIORITY;
constexpr std::uint32_t baud_rate = 115200U;

StreamBufferHandle_t queue;
TaskHandle_t console_task_handle;
UART_HandleTypeDef uart;

void configure_pins()
{
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef gpio{};
    gpio.Pin = GPIO_PIN_8;
    gpio.Mode = GPIO_MODE_AF_PP;
    gpio.Pull = GPIO_PULLUP;
    gpio.Speed = GPIO_SPEED_FREQ_LOW;
    gpio.Alternate = GPIO_AF7_USART3;
    HAL_GPIO_Init(GPIOD, &gpio);
}

bool configure_uart()
{
    constexpr std::uint32_t interrupt_priority =
        configMAX_SYSCALL_INTERRUPT_PRIORITY >> (8U - configPRIO_BITS);

    __HAL_RCC_USART3_CLK_ENABLE();

    uart.Instance = USART3;
    uart.Init.BaudRate = baud_rate;
    uart.Init.WordLength = UART_WORDLENGTH_8B;
    uart.Init.StopBits = UART_STOPBITS_1;
    uart.Init.Parity = UART_PARITY_NONE;
    uart.Init.Mode = UART_MODE_TX;
    uart.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    uart.Init.OverSampling = UART_OVERSAMPLING_16;

    if (HAL_UART_Init(&uart) != HAL_OK) {
        return false;
    }

    HAL_NVIC_SetPriority(USART3_IRQn, interrupt_priority, 0U);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
    return true;
}

void task_entry(void*)
{
    std::uint8_t chunk[chunk_size];

    for (;;) {
        const std::size_t length =
            xStreamBufferReceive(queue, chunk, sizeof(chunk), portMAX_DELAY);

        if (HAL_UART_Transmit_IT(&uart, chunk, static_cast<std::uint16_t>(length)) == HAL_OK) {
            ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        }
    }
}

} // namespace

extern "C" void USART3_IRQHandler()
{
    HAL_UART_IRQHandler(&uart);
}

extern "C" void HAL_UART_TxCpltCallback(UART_HandleTypeDef*)
{
    BaseType_t higher_priority_task_woken = pdFALSE;
    vTaskNotifyGiveFromISR(console_task_handle, &higher_priority_task_woken);
    portYIELD_FROM_ISR(higher_priority_task_woken);
}

// Output backend of eyalroz/printf. Never blocks: a character that does not fit the
// queue is dropped.
extern "C" void putchar_(char character)
{
    if (queue == nullptr) {
        return;
    }

    // A stream buffer allows one writer; the critical section admits many tasks.
    taskENTER_CRITICAL();
    xStreamBufferSend(queue, &character, 1U, 0U);
    taskEXIT_CRITICAL();
}

namespace stm32f407 {

bool start_console()
{
    configure_pins();
    if (!configure_uart()) {
        return false;
    }

    queue = xStreamBufferCreate(queue_size, 1U);
    if (queue == nullptr) {
        return false;
    }

    return xTaskCreate(task_entry,
                       "console",
                       console_stack_depth,
                       nullptr,
                       console_task_priority,
                       &console_task_handle) == pdPASS;
}

} // namespace stm32f407
