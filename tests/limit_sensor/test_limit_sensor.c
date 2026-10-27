#include <assert.h>
#include <stdio.h>
#include "limit_sensor.c"
static uint32_t now;
static GPIO_PinState s1 = GPIO_PIN_SET, s2 = GPIO_PIN_SET;
uint32_t AppTime_NowMs(void) { return now; }
GPIO_PinState HAL_GPIO_ReadPin(int port, int pin)
{ assert(port == 0); return pin == 1 ? s1 : s2; }
static void ticks(uint32_t count)
{ while (count--) { now += APP_LIMIT_SENSOR_SAMPLE_MS; LimitSensor_Process(); } }
int main(void)
{
    LimitSensor_Init();
    assert(!limit_sensors[0].valid && limit_sensors[0].raw_high);
    ticks(APP_LIMIT_SENSOR_FILTER_MS);
    assert(limit_sensors[0].valid && !limit_sensors[0].active);
    assert(!limit_sensors[0].released_count);
    /* Main-loop jitter must not starve qualification with 2 ms spacing. */
    s1 = GPIO_PIN_RESET;
    for (uint32_t i = 0; i <= APP_LIMIT_SENSOR_FILTER_MS; i++) {
        now += 2; LimitSensor_Process();
    }
    assert(limit_sensors[0].active && limit_sensors[0].valid);
    s1 = GPIO_PIN_SET; LimitSensor_Init(); ticks(APP_LIMIT_SENSOR_FILTER_MS);
    /* Short low pulse does not become a detection. */
    s1 = GPIO_PIN_RESET; ticks(1); ticks(APP_LIMIT_SENSOR_FILTER_MS-1);
    assert(!limit_sensors[0].active);
    s1 = GPIO_PIN_SET; ticks(APP_LIMIT_SENSOR_FILTER_MS+1);
    assert(!limit_sensors[0].detected_count);
    s1 = GPIO_PIN_RESET; ticks(APP_LIMIT_SENSOR_FILTER_MS+1);
    assert(limit_sensors[0].active && limit_sensors[0].detected_count == 1);
    assert(!limit_sensors[1].active);
    ticks(20); assert(limit_sensors[0].detected_count == 1);
    s1 = GPIO_PIN_SET; ticks(APP_LIMIT_SENSOR_FILTER_MS+1);
    assert(!limit_sensors[0].active && limit_sensors[0].released_count == 1);
    /* Already active at boot must qualify, including both sensors together. */
    s1 = s2 = GPIO_PIN_RESET; LimitSensor_Init();
    ticks(APP_LIMIT_SENSOR_FILTER_MS);
    assert(limit_sensors[0].valid && limit_sensors[1].active);
    assert(limit_sensors[0].detected_count == 1 && limit_sensors[1].detected_count == 1);
    /* Missing sampling time does not count as input stability. */
    s1 = GPIO_PIN_SET; ticks(1);
    now += 100; LimitSensor_Process();
    assert(limit_sensors[0].active);
    ticks(APP_LIMIT_SENSOR_FILTER_MS); assert(!limit_sensors[0].active);
    now = UINT32_MAX-2; s1 = GPIO_PIN_RESET; LimitSensor_Init();
    ticks(APP_LIMIT_SENSOR_FILTER_MS);
    assert(limit_sensors[0].valid && limit_sensors[0].active);
    uint32_t samples = limit_sensor_diagnostics.samples;
    LimitSensor_Process(); assert(limit_sensor_diagnostics.samples == samples);
    puts("Limit sensors: active-low, startup, filter, bounce, gaps, wrap passed");
    return 0;
}
