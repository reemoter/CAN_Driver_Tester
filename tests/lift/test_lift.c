#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "lift_motion.c"
static uint32_t now, moves, stops, quick;
volatile MotorStatus motor_status[1];
volatile LimitSensorStatus limit_sensors[2];
volatile LimitSensorDiagnostics limit_sensor_diagnostics;
uint32_t AppTime_NowMs(void) { return now; }
uint8_t Motor_SetVelocity(uint32_t i, int32_t v)
{ assert(i==0); moves++; motor_status[0].target_velocity=v; return 1; }
uint8_t Motor_ZeroTarget(uint32_t i)
{ assert(i==0); stops++; motor_status[0].target_velocity=0; return 1; }
uint8_t Motor_QuickStop(uint32_t i)
{ assert(i==0); quick++; return 1; }
static void tick(uint32_t ms)
{
    now += ms;
    motor_status[0].pdo_ms = now;
    limit_sensor_diagnostics.sample_ms = now;
    Lift_Process();
}
static void reset(void)
{
    now=100; moves=stops=quick=0;
    memset((void *)motor_status,0,sizeof(motor_status));
    memset((void *)limit_sensors,0,sizeof(limit_sensors));
    motor_status[0].state=MOTOR_ENABLED;
    motor_status[0].pdo_rx_count=1;
    motor_status[0].pdo_ms=now;
    limit_sensor_diagnostics.sample_ms=now;
    limit_sensors[0].valid=limit_sensors[1].valid=1;
    Lift_Init();
}
int main(void)
{
    reset(); assert(Lift_Start(0,1,150)==0);
    assert(motor_status[0].target_velocity==-150 && lift_status.state==LIFT_UP);
    assert(Lift_Start(0,1,150)==0); /* same direction accepted */
    limit_sensors[1].active=1; tick(1); assert(!stops); /* opposite limit ignored */
    limit_sensors[1].active=0; limit_sensors[0].active=1;
    tick(1); assert(stops==1 && !motor_status[0].target_velocity && !quick);
    assert(lift_status.state==LIFT_STOPPING);
    tick(1); tick(APP_REVERSAL_ZERO_HOLD_MS);
    assert(lift_status.state==LIFT_DONE && motor_status[0].state==MOTOR_ENABLED);
    assert(Lift_Start(0,0,150)==0 && motor_status[0].target_velocity==150);
    Lift_Cancel(); assert(!Lift_Busy() && !motor_status[0].target_velocity);
    reset(); limit_sensors[0].active=1;
    assert(!Lift_Start(0,1,150) && !moves && lift_status.state==LIFT_DONE);
    limit_sensors[1].active=1; assert(Lift_Start(0,0,150)==10);
    reset(); limit_sensors[0].valid=0; assert(Lift_Start(0,1,150)==9);
    reset(); now+=APP_LIMIT_SENSOR_MAX_GAP_MS+1; assert(Lift_Start(0,1,150)==9);
    reset(); assert(Lift_Start(1,1,150)==3 && Lift_Start(0,1,0)==5);
    motor_status[0].actual_velocity=20; assert(Lift_Start(0,1,150)==4);
    reset(); assert(!Lift_Start(0,0,150)); tick(APP_LIFT_MOVE_TIMEOUT_MS);
    assert(lift_status.reason==LIFT_TIMEOUT && stops==1);
    reset(); assert(!Lift_Start(0,1,150)); limit_sensors[0].valid=0; tick(1);
    assert(lift_status.reason==LIFT_SENSOR_INVALID && stops==1);
    reset(); assert(!Lift_Start(0,1,150));
    limit_sensors[0].active=limit_sensors[1].active=1; tick(1);
    assert(lift_status.reason==LIFT_SENSOR_CONFLICT);
    reset(); assert(!Lift_Start(0,1,150)); motor_status[0].state=MOTOR_ERROR; tick(1);
    assert(lift_status.reason==LIFT_MOTOR_ERROR && !Lift_Busy());
    reset(); assert(!Lift_Start(0,1,150)); limit_sensors[0].active=1; tick(1);
    motor_status[0].actual_velocity=20; tick(APP_STATE_TIMEOUT_MS);
    assert(lift_status.reason==LIFT_STOP_TIMEOUT && quick==1);
    reset(); now=UINT32_MAX-9;
    motor_status[0].pdo_ms=limit_sensor_diagnostics.sample_ms=now;
    assert(!Lift_Start(0,0,150)); tick(APP_LIFT_MOVE_TIMEOUT_MS);
    assert(lift_status.reason==LIFT_TIMEOUT);
    /* Interrupt Up before S1: immediately zero, then fresh stable standstill. */
    reset(); assert(!Lift_Start(0,1,150));
    motor_status[0].actual_velocity=-150;
    assert(!Lift_Start(0,0,150));
    assert(lift_status.state==LIFT_REVERSING && !motor_status[0].target_velocity);
    tick(1); assert(moves==1); /* cannot reverse while still moving */
    motor_status[0].actual_velocity=0; tick(1);
    tick(APP_REVERSAL_ZERO_HOLD_MS-1); assert(lift_status.state==LIFT_REVERSING);
    tick(1); assert(lift_status.state==LIFT_DOWN && motor_status[0].target_velocity==150);
    /* Retarget while stopping: most recent direction, original bounded wait. */
    assert(!Lift_Start(0,1,150));
    uint32_t deadline_base=stopped_ms;
    tick(1); assert(!Lift_Start(0,0,200));
    assert(stopped_ms==deadline_base);
    tick(APP_REVERSAL_ZERO_HOLD_MS);
    assert(lift_status.state==LIFT_DOWN && motor_status[0].target_velocity==200);
    /* A newly requested destination already active never starts into it. */
    assert(!Lift_Start(0,1,150)); limit_sensors[0].active=1;
    tick(1); tick(APP_REVERSAL_ZERO_HOLD_MS);
    assert(lift_status.state==LIFT_DONE && !motor_status[0].target_velocity);
    /* Reversal stop timeout cannot be extended by new direction commands. */
    reset(); assert(!Lift_Start(0,1,150)); motor_status[0].actual_velocity=-50;
    assert(!Lift_Start(0,0,150)); tick(APP_STATE_TIMEOUT_MS-1);
    assert(!Lift_Start(0,1,150)); tick(1);
    assert(lift_status.reason==LIFT_STOP_TIMEOUT && quick==1);
    reset(); assert(!Lift_Start(0,1,150)); assert(!Lift_Start(0,0,150));
    Lift_Cancel(); tick(200); assert(!Lift_Busy() && !motor_status[0].target_velocity);
    puts("Lift motion: directions, limits, stop/restart, conflicts, timeout, errors passed");
    return 0;
}
