#ifndef APP_SCHEDULER_H
#define APP_SCHEDULER_H
#include <stdint.h>

enum { APP_TASK_LIMIT_SENSOR, APP_TASK_LIFT, APP_TASK_MOTOR, APP_TASK_ETH_LINK, APP_TASK_COUNT };
typedef struct {
    uint32_t period_ms, runs, missed, max_lateness_ms, last_run_ms;
} AppTaskDiagnostics;
extern volatile AppTaskDiagnostics app_task_diagnostics[APP_TASK_COUNT];

void AppScheduler_Init(void); /* once, before TIM6 starts */
void AppScheduler_Process(void); /* main loop only; never runs catch-up bursts */
#endif
