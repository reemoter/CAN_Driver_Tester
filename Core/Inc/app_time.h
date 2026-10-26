#ifndef APP_TIME_H
#define APP_TIME_H
#include <stdint.h>

/* TIM6 ISR is the sole writer. Unsigned elapsed-time subtraction handles wrap.
 * Never reset this clock when a drive is reset or its power is cycled. */
void AppTime_Tick1ms(void);
uint32_t AppTime_NowMs(void);
extern volatile uint32_t app_time_ms; /* Read-only diagnostic; only TIM6 writes. */
#endif
