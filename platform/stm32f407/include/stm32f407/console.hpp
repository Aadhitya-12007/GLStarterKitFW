#pragma once

namespace stm32f407 {

// Routes printf output (eyalroz/printf, via putchar_) to USART3 TX on PD8. Output is
// queued without blocking and sent by a task at the lowest priority; bytes that do not
// fit the queue are dropped. Call once, before the scheduler starts and before anything
// prints.
bool start_console();

} // namespace stm32f407
