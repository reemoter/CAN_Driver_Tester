# Up / Down limit sequences

Motor index 0 only (LIFT_MOTOR_INDEX in lift_motion.h). Up is Reverse, negative
velocity, destination S1 (PD1). Down is Forward, positive velocity, destination
S2 (PD0). Both sensors are filtered active-low inputs. The opposite limit may
be active when moving away; both limits active is a conflict.

Workflow: Connect -> Power ON -> Enable at zero -> enter a positive RPM
magnitude -> Up (S1) or Down (S2). Direction dropdown applies only to manual
velocity, not Up/Down. The GUI does not automatically enable, power on or reset.
Start requires Enabled, fresh TPDO at near-zero velocity, zero manual target,
valid fresh sensors, and no active sequence. Already at destination completes
without movement. Up/Down may interrupt an active move. An opposite direction immediately sends
target zero, waits for fresh near-zero feedback held for 100 ms, then starts
the newest requested direction. It never waits for the old destination sensor.
Same-direction commands update speed without extending the move timeout.
Direction changes while already stopping update the pending direction without
resetting the 3-second stop deadline. A pending destination already active
finishes without motion. Error-stop sequences cannot be overridden.

Task order: sensor -> lift sequence -> motor -> PHY. Lift runs every 1 ms.
On destination detection, Motor_ZeroTarget cancels the MCU S curve and any
pending RPDO/SYNC batch, then publishes target 0 with controlword 15. EPOS's
configured deceleration still applies; this is not instantaneous physical stop.
The sequence waits for fresh near-zero feedback for 100 ms before DONE.
Enabled is retained, so the next move does not require re-enable/reset.

Limits are filtered (default 5 ms sample credit); communication and mechanics
add latency. A sensor is a sequence destination, not a hardware emergency stop.
Start still uses S-curve acceleration. Raw/filtered sensor masks are displayed.

APP_LIFT_MOVE_TIMEOUT_MS=10000 bounds travel before target-zero stop.
Invalid/stale sensors or both sensors active also request target zero and end
in sequence ERROR after stopping. If near-zero is not confirmed within
APP_STATE_TIMEOUT_MS (3000), fall back to Quick Stop (separate reason).
Motor errors and link/lease failures retain the existing protection path.
The sequence never renews the motor lease: the PC's 100 ms keepalive remains
mandatory. Normal DONE remains enabled, with keepalive while the GUI is open.
No stall detection or automatic retry/restart is introduced.

During a sequence, UDP manual velocity/enable/power/reset commands are rejected.
Up/Down remain accepted for direction changes, including during normal limit stop.
Manual velocity 0 cancels it using immediate target-zero; Quick Stop cancels it
and invokes the existing Quick Stop path. Direct debugger mailbox commands are
not a supported concurrent controller; unexpected targets cause cancellation.

## UDP additions (request remains 16 bytes)

- Command 10: Up, int32 value=positive speed magnitude.
- Command 11: Down, same magnitude format.
- Command 12: read-only extended status, 192 bytes.
- New motion commands use the same owner, replay and duplicate cache as 1..7.
  Duplicate packets do not restart movement or refresh the lease.
- Results 9: invalid/stale sensor; 10: both sensors active. Existing result 4
  also means moving/busy/not enabled; result 5 means invalid magnitude.
- Commands 0, 8, 9 retain their original lengths and field layout. New GUI falls
  back to command 9 if the firmware does not support command 12.

Command 12: bytes 0..167 match command 9. New fields:

| Offset | Type | Meaning |
|---|---|---|
| 168 | u8 | state: IDLE/UP/DOWN/STOPPING/DONE/CANCELLED/ERROR/REVERSING = 0..7 |
| 169 | u8 | reason: NONE/LIMIT/USER_STOP/TIMEOUT/SENSOR_INVALID/SENSOR_CONFLICT/MOTOR_ERROR/STOP_TIMEOUT/INTERRUPTED = 0..8 |
| 170/171/172 | u8 each | raw-high / active / valid masks; bit0=S1, bit1=S2 |
| 173 | u8 | controlled motor index (0) |
| 174 | u8 | lift protocol revision (1) |
| 175 | u8 | reserved zero |
| 176/180 | u32 LE | started_ms / elapsed_ms |
| 184 | i32 LE | signed sequence speed |
| 188 | u32 LE | move timeout ms |

Firmware and PC tests cover direction, already-active destination, conflict,
timeout, stop confirmation/restart, immediate-zero CAN output, task ordering,
UDP ownership/replay/cache and response decoding. Hardware motion not tested.
Download firmware and restart the PC tool; verify sensors with motor disabled
first. Code is in application-owned files, safe from CubeMX regeneration.
