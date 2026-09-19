extern "C" {
#include "FreeRTOS.h"
#include "task.h"
}

#include "stm32f407/board.hpp"

extern "C" {

void vApplicationMallocFailedHook()
{
    stm32f407::halt();
}

void vApplicationStackOverflowHook(TaskHandle_t, char*)
{
    stm32f407::halt();
}

// coreMQTT uses assert(); newlib's default handler would pull in stdio.
void __assert_func(const char*, int, const char*, const char*)
{
    stm32f407::halt();
}

} // extern "C"
