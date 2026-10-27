#ifndef LIFT_MOTION_H
#define LIFT_MOTION_H
#include <stdint.h>
#define LIFT_MOTOR_INDEX 0U
typedef enum { LIFT_IDLE, LIFT_UP, LIFT_DOWN, LIFT_STOPPING,
               LIFT_DONE, LIFT_CANCELLED, LIFT_ERROR, LIFT_REVERSING } LiftState;
typedef enum { LIFT_REASON_NONE, LIFT_LIMIT, LIFT_USER_STOP, LIFT_TIMEOUT,
               LIFT_SENSOR_INVALID, LIFT_SENSOR_CONFLICT, LIFT_MOTOR_ERROR,
               LIFT_STOP_TIMEOUT, LIFT_INTERRUPTED } LiftReason;
typedef struct {
    LiftState state;
    LiftReason reason;
    uint32_t started_ms, elapsed_ms;
    int32_t speed;
} LiftStatus;
extern volatile LiftStatus lift_status;
void Lift_Init(void);
void Lift_Process(void);
uint8_t Lift_Busy(void);
/* UDP result: 0 accepted, 3 wrong motor, 4 busy/state, 5 bad speed,
 * 9 invalid/stale sensor, 10 both limits active. No implicit enable. */
uint8_t Lift_Start(uint32_t index, uint8_t up, int32_t magnitude);
void Lift_Cancel(void);
#endif
