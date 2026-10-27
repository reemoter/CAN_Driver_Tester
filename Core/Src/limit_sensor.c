#include "limit_sensor.h"
#include "main.h"
#include "app_time.h"
#include "app_timing.h"
#include <string.h>

volatile LimitSensorStatus limit_sensors[LIMIT_SENSOR_COUNT];
volatile LimitSensorDiagnostics limit_sensor_diagnostics;
static uint8_t candidate_high[LIMIT_SENSOR_COUNT], sampled;
static uint32_t last_sample_ms;

void LimitSensor_Init(void)
{
    memset((void *)limit_sensors, 0, sizeof(limit_sensors));
    memset((void *)&limit_sensor_diagnostics, 0, sizeof(limit_sensor_diagnostics));
    sampled = 0;
    LimitSensor_Process();
}

void LimitSensor_Process(void)
{
    uint32_t now = AppTime_NowMs();
    if (sampled && now == last_sample_ms) return;
    uint32_t elapsed = sampled ? now - last_sample_ms : 0;
    uint8_t gap = !sampled || elapsed > APP_LIMIT_SENSOR_MAX_GAP_MS;
    uint32_t credit = elapsed < APP_LIMIT_SENSOR_SAMPLE_MS ? elapsed : APP_LIMIT_SENSOR_SAMPLE_MS;
    if (sampled) {
        if (elapsed > limit_sensor_diagnostics.max_sample_gap_ms)
            limit_sensor_diagnostics.max_sample_gap_ms = elapsed;
        if (gap) limit_sensor_diagnostics.gap_resets++;
    }
    uint8_t levels[LIMIT_SENSOR_COUNT] = {
        HAL_GPIO_ReadPin(INPUT_S1_GPIO_Port, INPUT_S1_Pin) == GPIO_PIN_SET,
        HAL_GPIO_ReadPin(INPUT_S2_GPIO_Port, INPUT_S2_Pin) == GPIO_PIN_SET
    };
    last_sample_ms = now;
    sampled = 1;
    limit_sensor_diagnostics.sample_ms = now;
    limit_sensor_diagnostics.samples++;
    for (uint32_t i = 0; i < LIMIT_SENSOR_COUNT; i++) {
        volatile LimitSensorStatus *s = &limit_sensors[i];
        s->raw_high = levels[i];
        /* Tolerate main-loop jitter, but never credit unobserved time.
         * A small gap contributes at most one nominal sample interval. */
        if (gap || candidate_high[i] != levels[i]) {
            candidate_high[i] = levels[i];
            s->stable_ms = 0;
        } else if (s->stable_ms < APP_LIMIT_SENSOR_FILTER_MS) {
            uint32_t remaining = APP_LIMIT_SENSOR_FILTER_MS - s->stable_ms;
            s->stable_ms += credit < remaining ? credit : remaining;
        }
        if (s->stable_ms < APP_LIMIT_SENSOR_FILTER_MS) continue;
        uint8_t active = !levels[i];
        if (!s->valid || s->active != active) {
            uint8_t was_valid = s->valid;
            s->active = active;
            s->changed_ms = now;
            if (active) s->detected_count++; /* includes active at boot */
            else if (was_valid) s->released_count++;
            s->valid = 1;
        }
    }
}
