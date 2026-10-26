#include <assert.h>
#include <stdio.h>
#include <stdint.h>
#include "app_time.c"
#include "app_scheduler.c"

struct netif gnetif;
static uint32_t motor_calls, phy_calls;
void Motor_Process(void) { motor_calls++; }
void ethernet_link_check_state(struct netif *netif)
{
    assert(netif == &gnetif);
    assert(motor_calls > 0); /* motor work is ordered before PHY maintenance */
    phy_calls++;
}
static void run_ticks(uint32_t count)
{
    for (uint32_t i = 0; i < count; i++) {
        AppTime_Tick1ms();
        AppScheduler_Process();
        AppScheduler_Process(); /* repeated main-loop calls never duplicate work */
    }
}
int main(void)
{
    app_time_ms = 0;
    AppScheduler_Init();
    AppScheduler_Process();
    assert(!motor_calls && !phy_calls);
    run_ticks(1000);
    assert(AppTime_NowMs() == 1000 && motor_calls == 1000);
    assert(phy_calls == 1000 / APP_ETH_LINK_CHECK_MS);
    assert(app_task_diagnostics[APP_TASK_MOTOR].missed == 0);
    assert(app_task_diagnostics[APP_TASK_ETH_LINK].period_ms == APP_ETH_LINK_CHECK_MS);

    /* Long pauses cause one execution per task, never a burst of old cycles. */
    AppScheduler_Init();
    uint32_t previous_motor = motor_calls, previous_phy = phy_calls;
    for (uint32_t i = 0; i < 250; i++) AppTime_Tick1ms();
    AppScheduler_Process(); AppScheduler_Process();
    assert(motor_calls == previous_motor + 1 && phy_calls == previous_phy + 1);
    assert(app_task_diagnostics[APP_TASK_MOTOR].missed == 249);
    assert(app_task_diagnostics[APP_TASK_MOTOR].max_lateness_ms == 249);
    assert(app_task_diagnostics[APP_TASK_ETH_LINK].missed == 250 / APP_ETH_LINK_CHECK_MS - 1);

    /* 32-bit millisecond rollover preserves elapsed-time scheduling. */
    app_time_ms = UINT32_MAX - 50;
    AppScheduler_Init();
    previous_motor = motor_calls; previous_phy = phy_calls;
    run_ticks(2 * APP_ETH_LINK_CHECK_MS);
    assert(motor_calls == previous_motor + 2 * APP_ETH_LINK_CHECK_MS);
    assert(phy_calls == previous_phy + 2);
    assert(!app_task_diagnostics[APP_TASK_MOTOR].missed);
    assert(!app_task_diagnostics[APP_TASK_ETH_LINK].max_lateness_ms);
    uint32_t now = AppTime_NowMs();
    AppScheduler_Init(); assert(AppTime_NowMs() == now);
    puts("Shared timebase/scheduler tests passed");
    return 0;
}
