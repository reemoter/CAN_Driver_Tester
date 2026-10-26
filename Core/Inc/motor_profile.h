#ifndef MOTOR_PROFILE_H
#define MOTOR_PROFILE_H
#include <stdint.h>

/* MCU rest-to-rest trajectory peak acceleration: 300 rpm/s.
 * Quintic S curve needs 1.875 * delta / rate: 150 rpm takes about 0.94 s.
 * The EPOS linear ramp is a secondary ceiling, not the trajectory generator.
 * Units assume commissioned rpm and rpm/s scaling. No Flash/PI changes.
 */
#define MOTOR_SCURVE_RATE 300U
#define MOTOR_SCURVE_MIN_MS 100U
#define MOTOR_PROFILE_ACCELERATION 10000U
#define MOTOR_PROFILE_DECELERATION 10000U

typedef struct {
    double value, acceleration, origin, slope, c3, c4, c5;
    int32_t goal;
    uint32_t started, duration;
} MotorProfile;

static void profile_reset(MotorProfile *p, int32_t velocity)
{
    *p = (MotorProfile){0};
    p->value = velocity;
    p->goal = velocity;
}

static int32_t profile_sample(MotorProfile *p, uint32_t now)
{
    uint32_t elapsed = now - p->started;
    if (!p->duration || elapsed >= p->duration) {
        p->value = p->goal;
        p->acceleration = 0;
        p->duration = 0;
    } else {
        double u = (double)elapsed / p->duration;
        p->value = p->origin + p->slope*u + u*u*u*(p->c3 + u*(p->c4 + u*p->c5));
        p->acceleration = (p->slope + u*u*(3*p->c3 + u*(4*p->c4 + u*5*p->c5)))
                            * 1000.0 / p->duration;
    }
    /* Protect conversion at signed API endpoints and on mid-ramp replanning. */
    if (p->value >= INT32_MAX) return INT32_MAX;
    if (p->value <= INT32_MIN) return INT32_MIN;
    return (int32_t)p->value;
}

static void profile_goal(MotorProfile *p, int32_t goal, uint32_t now)
{
    (void)profile_sample(p, now);
    if (p->goal == goal) return; /* duplicate UDP targets must not restart ramp */
    double delta = (double)goal - p->value;
    double distance = delta < 0 ? -delta : delta;
    double ms = distance * 1875.0 / MOTOR_SCURVE_RATE;
    if (ms < MOTOR_SCURVE_MIN_MS) ms = MOTOR_SCURVE_MIN_MS;
    if (ms > 2147483000.0) ms = 2147483000.0;
    ms = (double)(uint32_t)(ms + 0.999);
    p->origin = p->value;
    p->slope = p->acceleration * ms / 1000.0;
    /* Match current velocity/acceleration, finish with zero acceleration/jerk.
     * For rest-to-rest transitions this is 10u^3 - 15u^4 + 6u^5.
     * Retargeting preserves acceleration; jerk may change at the replan. */
    p->c3 = 10*delta - 6*p->slope;
    p->c4 = -15*delta + 8*p->slope;
    p->c5 = 6*delta - 3*p->slope;
    p->goal = goal;
    p->started = now;
    p->duration = (uint32_t)ms;
}

_Static_assert(MOTOR_PROFILE_ACCELERATION > 0U && MOTOR_PROFILE_DECELERATION > 0U,
               "Profile ramp rates must be positive");
_Static_assert(MOTOR_SCURVE_RATE > 0U && MOTOR_SCURVE_MIN_MS > 0U,
               "S curve settings must be positive");
#endif
