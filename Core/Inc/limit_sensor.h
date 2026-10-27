#ifndef LIMIT_SENSOR_H
#define LIMIT_SENSOR_H
#include <stdint.h>

enum { LIMIT_SENSOR_S1, LIMIT_SENSOR_S2, LIMIT_SENSOR_COUNT };
typedef struct {
    uint8_t raw_high;       /* electrical level, 0 = detected */
    uint8_t active;         /* filtered detection, 1 = detected */
    uint8_t valid;          /* first stable input has been confirmed */
    uint32_t detected_count, released_count, changed_ms;
    uint32_t stable_ms;     /* credited same-level samples; capped at filter time */
} LimitSensorStatus;
typedef struct {
    uint32_t samples, sample_ms, max_sample_gap_ms, gap_resets;
} LimitSensorDiagnostics;

/* Main-loop-owned diagnostics; observe only from debugger. Index 0=S1, 1=S2.
 * Until valid=1, active=0 is unknown, not a confirmed clear limit. */
extern volatile LimitSensorStatus limit_sensors[LIMIT_SENSOR_COUNT];
extern volatile LimitSensorDiagnostics limit_sensor_diagnostics;
void LimitSensor_Init(void); /* after MX_GPIO_Init, before scheduler ticks */
void LimitSensor_Process(void); /* nonblocking, called by application scheduler */
#endif
