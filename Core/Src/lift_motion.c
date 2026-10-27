#include "lift_motion.h"
#include "limit_sensor.h"
#include "motor.h"
#include "app_time.h"
#include "app_timing.h"

volatile LiftStatus lift_status;
static uint32_t stopped_ms, zero_ms;
static uint8_t zero_seen;

uint8_t Lift_Busy(void)
{ return lift_status.state == LIFT_UP || lift_status.state == LIFT_DOWN ||
         lift_status.state == LIFT_STOPPING || lift_status.state == LIFT_REVERSING; }

void Lift_Init(void) { lift_status = (LiftStatus){0}; zero_seen = 0; }

static uint8_t sensors_valid(uint32_t now)
{
    return limit_sensors[0].valid && limit_sensors[1].valid &&
        now - limit_sensor_diagnostics.sample_ms <= APP_LIMIT_SENSOR_MAX_GAP_MS;
}

static uint8_t stationary(uint32_t now)
{
    volatile MotorStatus *s = &motor_status[LIFT_MOTOR_INDEX];
    return s->pdo_rx_count && now - s->pdo_ms < APP_PDO_TIMEOUT_MS &&
        s->actual_velocity >= -MOTOR_REVERSAL_ZERO_BAND &&
        s->actual_velocity <= MOTOR_REVERSAL_ZERO_BAND;
}

uint8_t Lift_Start(uint32_t index, uint8_t up, int32_t magnitude)
{
    if (index != LIFT_MOTOR_INDEX) return 3;
    if (magnitude <= 0) return 5;
    uint32_t now = AppTime_NowMs();
    volatile MotorStatus *s = &motor_status[index];
    if (s->state != MOTOR_ENABLED) return 4;
    if (!sensors_valid(now)) return 9;
    if (limit_sensors[0].active && limit_sensors[1].active) return 10;
    int32_t speed = up ? -magnitude : magnitude;
    if (Lift_Busy()) {
        if (lift_status.state == LIFT_STOPPING && lift_status.reason != LIFT_LIMIT) return 4;
        if ((lift_status.state == LIFT_UP && up) || (lift_status.state == LIFT_DOWN && !up)) {
            /* Speed update in the same direction; do not extend travel timeout. */
            if (!Motor_SetVelocity(index, speed)) return 4;
            lift_status.speed = speed;
            return 0;
        }
        if (lift_status.state != LIFT_REVERSING) {
            if (!Motor_ZeroTarget(index)) return 4;
            stopped_ms = now;
            zero_seen = 0;
            lift_status.started_ms = now;
            lift_status.elapsed_ms = 0;
        }
        /* While stopping, the most recent Up/Down wins. Do not reset the
         * standstill timer/deadline on repeated commands. */
        lift_status.speed = speed;
        lift_status.reason = LIFT_REASON_NONE;
        lift_status.state = LIFT_REVERSING;
        return 0;
    }
    if (s->target_velocity || s->requested_velocity || s->reversal_pending || !stationary(now)) return 4;
    lift_status = (LiftStatus){.started_ms = now};
    if (limit_sensors[up ? LIMIT_SENSOR_S1 : LIMIT_SENSOR_S2].active) {
        lift_status.state = LIFT_DONE;
        lift_status.reason = LIFT_LIMIT;
        return 0; /* already at destination; never move further into the limit */
    }
    if (!Motor_SetVelocity(index, speed)) return 4;
    lift_status.speed = speed;
    lift_status.state = up ? LIFT_UP : LIFT_DOWN;
    return 0;
}

static void stop_at(LiftReason reason, uint32_t now)
{
    lift_status.reason = reason;
    lift_status.elapsed_ms = now - lift_status.started_ms;
    if (!Motor_ZeroTarget(LIFT_MOTOR_INDEX)) {
        (void)Motor_QuickStop(LIFT_MOTOR_INDEX);
        lift_status.state = LIFT_ERROR;
        lift_status.reason = LIFT_MOTOR_ERROR;
        return;
    }
    stopped_ms = now;
    zero_seen = 0;
    lift_status.state = LIFT_STOPPING;
}

void Lift_Cancel(void)
{
    if (!Lift_Busy()) return;
    (void)Motor_ZeroTarget(LIFT_MOTOR_INDEX);
    lift_status.elapsed_ms = AppTime_NowMs() - lift_status.started_ms;
    lift_status.state = LIFT_CANCELLED;
    lift_status.reason = LIFT_USER_STOP;
}

void Lift_Process(void)
{
    if (!Lift_Busy()) return;
    uint32_t now = AppTime_NowMs();
    volatile MotorStatus *s = &motor_status[LIFT_MOTOR_INDEX];
    if (s->state != MOTOR_ENABLED) {
        lift_status.state = LIFT_ERROR;
        lift_status.reason = LIFT_MOTOR_ERROR;
        return; /* motor error/lease/quick-stop path already owns stopping */
    }
    lift_status.elapsed_ms = now - lift_status.started_ms;
    if (lift_status.state == LIFT_REVERSING) {
        if (!sensors_valid(now)) { stop_at(LIFT_SENSOR_INVALID, now); return; }
        if (limit_sensors[0].active && limit_sensors[1].active) {
            stop_at(LIFT_SENSOR_CONFLICT, now); return;
        }
        if (s->target_velocity || s->requested_velocity || s->reversal_pending) {
            stop_at(LIFT_INTERRUPTED, now); return;
        }
        if (stationary(now) && (int32_t)(s->pdo_ms - stopped_ms) >= 0) {
            if (!zero_seen) { zero_seen = 1; zero_ms = now; }
            if (now - zero_ms >= APP_REVERSAL_ZERO_HOLD_MS) {
                uint8_t up = lift_status.speed < 0;
                if (limit_sensors[up ? LIMIT_SENSOR_S1 : LIMIT_SENSOR_S2].active) {
                    lift_status.state = LIFT_DONE;
                    lift_status.reason = LIFT_LIMIT;
                } else if (Motor_SetVelocity(LIFT_MOTOR_INDEX, lift_status.speed)) {
                    lift_status.state = up ? LIFT_UP : LIFT_DOWN;
                } else stop_at(LIFT_MOTOR_ERROR, now);
                return;
            }
        } else zero_seen = 0;
        if (now - stopped_ms >= APP_STATE_TIMEOUT_MS) {
            (void)Motor_QuickStop(LIFT_MOTOR_INDEX);
            lift_status.state = LIFT_ERROR;
            lift_status.reason = LIFT_STOP_TIMEOUT;
        }
        return;
    }
    if (lift_status.state == LIFT_STOPPING) {
        if (s->target_velocity || s->requested_velocity || s->reversal_pending) {
            stop_at(LIFT_INTERRUPTED, now);
            return;
        }
        if (stationary(now) && (int32_t)(s->pdo_ms - stopped_ms) >= 0) {
            if (!zero_seen) { zero_seen = 1; zero_ms = now; }
            if (now - zero_ms >= APP_REVERSAL_ZERO_HOLD_MS) {
                lift_status.state = lift_status.reason == LIFT_LIMIT ? LIFT_DONE : LIFT_ERROR;
                return;
            }
        } else zero_seen = 0;
        if (now - stopped_ms >= APP_STATE_TIMEOUT_MS) {
            (void)Motor_QuickStop(LIFT_MOTOR_INDEX);
            lift_status.state = LIFT_ERROR;
            lift_status.reason = LIFT_STOP_TIMEOUT;
        }
        return;
    }
    if (!sensors_valid(now)) { stop_at(LIFT_SENSOR_INVALID, now); return; }
    if (limit_sensors[0].active && limit_sensors[1].active) {
        stop_at(LIFT_SENSOR_CONFLICT, now); return;
    }
    uint32_t destination = lift_status.state == LIFT_UP ? LIMIT_SENSOR_S1 : LIMIT_SENSOR_S2;
    if (limit_sensors[destination].active) { stop_at(LIFT_LIMIT, now); return; }
    if (now - lift_status.started_ms >= APP_LIFT_MOVE_TIMEOUT_MS) {
        stop_at(LIFT_TIMEOUT, now); return;
    }
    if (s->target_velocity != lift_status.speed || s->reversal_pending)
        stop_at(LIFT_INTERRUPTED, now);
    /* Never renew command lease here. PC keepalive remains mandatory. */
}
