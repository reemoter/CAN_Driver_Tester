# CAN / EPOS4 first bring-up

## Configuration

- CAN1 PA11(RX), PA12(TX), normal mode, 1 Mbps (APB1 42 MHz / 2 / 21).
- TIM6 provides a 1 ms software clock. ISR only increments the clock.
- Default: one EPOS4, CANopen node ID 1. `MOTOR_COUNT=2` enables independent node 2.
- Boot leaves EPOS power output low. No automatic Enable, Fault Reset, Reset Node,
  PDO reconfiguration, or motion. Existing drive motor/feedback parameters remain required.
- First version uses acknowledged expedited SDO (8-byte frames), not cyclic PDO.
  The existing Keil PDO mapping is therefore not a prerequisite for connection tests.
  Profile Velocity Mode is 3; target/actual velocity are raw device units.

## CAN functions

| Direction | CAN ID | Use |
|---|---|---|
| TX | 0x600 + node | SDO request, one outstanding request per node |
| RX | 0x580 + node | SDO response / abort |
| RX | 0x700 + node | Optional producer heartbeat / boot-up |
| RX | 0x080 + node | EMCY code |
| TX | 0x000 | Explicit enable sends NMT Start to the selected node only |

SDO objects: 0x6041 statusword, 0x6040 controlword, 0x6060 mode,
0x6061 mode display, 0x60FF target velocity, 0x606C actual velocity.
Enable sequence: zero target -> mode 3 -> verify mode -> shutdown 0x0006 ->
verify ready -> switch on 0x0007 -> verify switched on -> enable 0x000F ->
verify enabled. Status is masked by 0x006F, not compared as a complete word.
The main loop handles RX decoding and SDO state transitions without busy waits.

## Debugger test

Observe `motor_status[0]`, `motor_diagnostics`, and `motor_debug` in Live Expressions.
Write motor_index, velocity and command first. Submit by incrementing request_id last. Firmware never modifies these input fields. Wait for completed_id to equal request_id before changing arguments or submitting another request. `accepted=1` means accepted by firmware;
check `state` and `error` for actual completion. Index is zero based.

1. With mechanical motion prevented as needed for the setup, verify CAN H/L, common
   reference, transceiver power/standby, and termination at the two bus ends.
2. Confirm EPOS ID 1, 1 Mbps, and motor/feedback commissioning in EPOS Studio.
3. Set `motor_debug.command=4` to enable the board's EPOS power output.
4. After 1.5 s startup allowance, SDO status requests begin. Expect `MOTOR_MONITOR`.
   `rx_count=0` / timeout is a communication failure, not evidence of a motor failure.
5. Set `motor_debug.motor_index=0`, then `command=1`: enable at zero target.
   Expect `MOTOR_ENABLED`; inspect statusword, mode display, and abort code on failure.
6. Only after checking the configured velocity unit (0x60A9), set `velocity` to a
   suitable small value and `command=2`. Limit is +/-100 raw units. This is not
   guaranteed to mean rpm and is not a torque/speed safety limit.
7. `command=3` requests Quick Stop 0x0002. A pending SDO is completed first.
   The stopped state is confirmed by statusword (quick-stop active / disabled).
   Actual mechanical stopping depends on the configured drive quick-stop parameters.
8. `command=5` powers off only while idle/stopped/error. To renew a running command,
   submit command 2 again with a new request_id within 1000 ms; otherwise a lease error requests Quick Stop.

Errors latch until MCU reset. There is no automatic re-enable or fault reset.
Lease expiry issues a best-effort stop and enters ERROR; it does not claim a confirmed
stop. Communication loss prevents a guaranteed software stop. Configure EPOS-side
communication supervision/quick-stop behavior before sustained motion tests.
Heartbeat supervision starts only if a heartbeat has been observed (fixed 1500 ms
timeout); set this to match the drive's producer period when commissioning.
SDO response timeout is 200 ms; state transition timeout is 3000 ms per phase.
No retries are issued after a timeout, avoiding accidental matching of delayed replies.

## Timing and scope

`Motor_Process()` is called continuously, but advances at most once per TIM6 tick.
Missed iterations are counted and not replayed. SDO request spacing is at least 5 ms
after each response; it is a diagnostic/first-motion path, not a deterministic 5 ms
PDO control loop. A blocking Ethernet handler can still delay main-context work.
TIM5/TIM7 user's trigger flags remain available for future encoder/general tasks.
SysTick remains the HAL timebase. CAN auto retransmission/bus-off recovery remain as
generated (disabled); error passive/bus off stop further commands and latch an error.
RX ISR is bounded to three frames; software queue overflow also latches errors.

Next stage after hardware validation: explicitly configure/verify PDO mapping,
producer heartbeat and drive communication watchdog, then add timer-driven PDO/SYNC.
No hardware flashing or motion test was performed by this change.

## Debugger write verification fix

The old firmware cleared command immediately after reading it. J-Link could write 4,
then verify RAM after firmware had already cleared it to 0, report Verify failed,
and halt at whichever main-loop instruction was executing (including Ethernet).
The new request_id/completed_id handshake avoids firmware writes to debugger inputs.
It does not disable J-Link memory verification, exception catching, or Ethernet.

Terminate the old debug session, refresh/rebuild the project, then launch a fresh
session to load the new ELF and symbols. Remove terminated sessions from the Debug
view and select the active Thread #1 before using Resume. A stale IDE run-state
may still require GDB continue; the mailbox fix addresses the observed verify-failure
trigger, not all possible debugger/IDE issues.

Power-only test (no Enable/motion):
- motor_index=0, velocity=0, command=4; these writes alone do nothing.
- request_id=1 commits the request after reset.
- Expect completed_id=1 and accepted=1; command must remain 4 and request_id 1.
- Leave running for 3 s: expect MONITOR and increasing RX count.
- To power off: command=5, then request_id=2; expect completed_id=2.
- Reusing request_id=2 does nothing. Each new request needs a different ID.
- Never write completed_id or accepted; these are firmware-owned outputs.

The 1000 ms enabled-state command lease is unchanged. Keep motion tests separate
until the debugger write/continue behavior has been validated on hardware.