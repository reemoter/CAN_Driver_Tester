#include "app_time.h"

volatile uint32_t app_time_ms;

void AppTime_Tick1ms(void) { app_time_ms++; }
uint32_t AppTime_NowMs(void) { return app_time_ms; }
