#include "app_scheduler.h"
#include "app_time.h"
#include "app_timing.h"
#include "motor.h"
#include "ether_phy.h"
#include "limit_sensor.h"
#include "lift_motion.h"

typedef struct { uint32_t period_ms; void (*run)(void); } AppTask;

/* Call application-owned PHY glue directly. CubeMX regenerates the BSP
 * ethernet_link_check_state() body as empty; it is not our extension point. */
static void check_ethernet_link(void) { EthernetPhy_Process(); }

/* Periodic task registry: sensor sampling, motor transactions, link maintenance.
 * CAN cycle launches and response-dependent SDO deadlines remain in motor.c;
 * every one uses AppTime and the settings in app_timing.h. */
static const AppTask tasks[APP_TASK_COUNT] = {
    [APP_TASK_LIMIT_SENSOR] = {APP_LIMIT_SENSOR_SAMPLE_MS, LimitSensor_Process},
    [APP_TASK_LIFT] = {APP_LIFT_SERVICE_MS, Lift_Process},
    [APP_TASK_MOTOR] = {APP_MOTOR_SERVICE_MS, Motor_Process},
    [APP_TASK_ETH_LINK] = {APP_ETH_LINK_CHECK_MS, check_ethernet_link}
};
static uint32_t last_run[APP_TASK_COUNT];
volatile AppTaskDiagnostics app_task_diagnostics[APP_TASK_COUNT];

void AppScheduler_Init(void)
{
    LimitSensor_Init();
    Lift_Init();
    uint32_t now = AppTime_NowMs();
    for (uint32_t i = 0; i < APP_TASK_COUNT; i++) {
        last_run[i] = now;
        app_task_diagnostics[i] = (AppTaskDiagnostics){.period_ms = tasks[i].period_ms};
    }
}

void AppScheduler_Process(void)
{
    for (uint32_t i = 0; i < APP_TASK_COUNT; i++) {
        uint32_t now = AppTime_NowMs();
        uint32_t elapsed = now - last_run[i];
        uint32_t period = tasks[i].period_ms;
        if (elapsed < period) continue;
        uint32_t lateness = elapsed - period;
        last_run[i] = now;
        app_task_diagnostics[i].runs++;
        app_task_diagnostics[i].missed += elapsed / period - 1U;
        app_task_diagnostics[i].last_run_ms = now;
        if (lateness > app_task_diagnostics[i].max_lateness_ms)
            app_task_diagnostics[i].max_lateness_ms = lateness;
        tasks[i].run();
    }
}
