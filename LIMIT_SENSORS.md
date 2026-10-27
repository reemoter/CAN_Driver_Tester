# Limit sensor sampling

- S1 = PD1 / INPUT_S1, S2 = PD0 / INPUT_S2. GPIO configuration remains owned
  by the .ioc (input, no pull). Active-low: High clear, Low detected.
- AppScheduler_Init initializes the module after generated GPIO startup.
  Task order is LIMIT_SENSOR, MOTOR, ETH_LINK. Enum indexes changed accordingly.
- APP_LIMIT_SENSOR_SAMPLE_MS=1, APP_LIMIT_SENSOR_FILTER_MS=5 in app_timing.h.
  Five milliseconds of same-level sample credit is required. Small scheduling
  delays credit at most one nominal interval; gaps over APP_LIMIT_SENSOR_MAX_GAP_MS
  (10 ms) restart qualification. No HAL_Delay or busy wait.
- limit_sensors[0] is S1, limit_sensors[1] is S2. raw_high is electrical level;
  active is filtered detection. valid remains zero until the first qualified
  level. Do not interpret active=0 as clear while valid=0.
- detected_count counts confirmed clear-to-active transitions, including an
  active input at startup. released_count counts confirmed active-to-clear
  transitions; a clear startup does not increment it. changed_ms is the time
  of qualification, not the physical edge timestamp.
- limit_sensor_diagnostics exposes samples/sample_ms. Scheduler timing can be
  observed at app_task_diagnostics[APP_TASK_LIMIT_SENSOR].
- Debugger observation only at this stage: no UDP payload/PC GUI changes, no
  motor commands, automatic stops or sequence actions. Sensor disconnection
  cannot be distinguished reliably from other levels without circuit support.
- All application logic is in separate files, with no generated C/H changes.
  makefile.defs handles the new source until CubeIDE discovers it on refresh.

Manual check: after downloading and running firmware, wait for valid=1. With
no metal present expect raw_high=1, active=0. Present metal for over 5 ms:
raw_high=0, active=1 and detected_count increments once. Remove it:
raw_high=1, active=0 and released_count increments once. Check each sensor
individually and together. Input voltage/wiring must match MCU requirements.

Diagnostics added: stable_ms is capped sample credit (0..5 by default).
max_sample_gap_ms and gap_resets expose scheduling gaps. A 2 ms sampling
regression reproduces the old filter starvation and passes with the fix.
