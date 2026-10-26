# UDP motor bench control v1

## Rotation direction

The GUI accepts a nonnegative speed magnitude and Forward (+) / Reverse (-).
Selecting a direction alone does not send motion: press Set velocity to apply.
Set zero speed commands zero while remaining enabled; Quick Stop retains its
existing stop workflow. Positive/negative follow the configured drive convention;
they do not establish a universal CW/CCW view of the mechanism.

Command 2 still uses a signed 32-bit velocity. No packet lengths or command IDs
change. Firmware detects reversal against the previous commanded direction or
opposite measured motion, first commands zero, then requires fresh TPDO velocity
within +/-5 configured velocity units for at least 100 ms before applying the
requested opposite sign. Current drive units are rpm. Zero tolerance is
MOTOR_REVERSAL_ZERO_BAND; hold/timeout settings are APP_REVERSAL_ZERO_HOLD_MS
and APP_REVERSAL_TIMEOUT_MS in app_timing.h (defaults 100 / 5000 ms).
The EPOS profile deceleration controls slowing down; MCU adds no outer PID.
Failure to confirm standstill latches REVERSAL_TIMEOUT (12).
Keepalive/lease and all existing communication supervision remain active.
Zero speed, Quick Stop, fault, power cycle and Reset drivers cancel queued reversal.
No deferred motion is restored after reconnect/error/reset.

motor_status.requested_velocity records the user request; target_velocity remains
zero during the interlock. Direction changes do not silently renew the lease.
Extended reply byte 103, formerly reserved: bit 7 indicates reversal support,
bit 0 indicates a pending reversal. GUI/CSV expose both flags; older firmware
reports no support and the updated GUI displays an update message.
Update both firmware and PC tool before testing reversal at a commissioned speed.

Firmware timing is centralized in Core/Inc/app_timing.h, using the TIM6 AppTime
clock for motor, control ownership, UDP timestamps and PHY diagnostics.
See [SCHEDULING.md](SCHEDULING.md). HAL/lwIP internal clocks are unchanged.

Motor motion now uses synchronous PDO/SYNC (10 ms); commissioning and detailed
diagnostics remain SDO. See [PDO_CANOPEN.md](PDO_CANOPEN.md) for mappings,
supervision and first hardware test. Command 9 reports PDO configured at offset
130 (uint16) and TPDO RX count at offset 164 (uint32), formerly reserved.

## First test (read-only)

MCU defaults to 192.168.0.24/24, UDP port 2000.
Set BOARD_CONFIG_ID in Core/Inc/board_identity.h and rebuild/download.
Default is 2; compiler -DBOARD_CONFIG_ID overrides the header default.
PD0/PD1 are sensor inputs, never sampled for network identity. The old internal
pulldowns are removed (GPIO input, no pull, also persisted in the .ioc).

| BOARD_CONFIG_ID | IPv4 | MAC |
|---|---|---|
| 1 | 192.168.0.23 | 00:80:E1:00:00:01 |
| 2 (default) | 192.168.0.24 | 00:80:E1:00:00:02 |
| 3 | 192.168.0.25 | 00:80:E1:00:00:03 |
| 4 | 192.168.0.26 | 00:80:E1:00:00:04 |

The identity is compile-time configured: no switch sampling, boot delay or
legacy Flash configuration. Invalid IDs fail compilation. Diagnostics keep
raw_selector=255 (unused), samples=0, stable=1 (valid fixed configuration).
Each board on the network must use a different compiled ID. MAC storage has
static lifetime. Reset drivers does not change network identity.

Set the PC Ethernet adapter to an unused address on the same subnet, for example
192.168.0.10/24. Do not assign .23 through .26 to the PC. A direct Ethernet link does not need
a gateway. If multiple adapters overlap, use --bind with the PC Ethernet address.
Allow the Python application on the intended private network if Windows asks.

Refresh/rebuild in CubeIDE and download the new firmware using a fresh debug
session. Leave the CPU running. Live Expressions are read-only observations;
do not write values while running with J-Link.

    python tools/udp_motor.py --status
    python tools/udp_motor.py

Optional: --host 192.168.0.23 --bind 192.168.0.10 --port 2000.
Reply bytes 53/54/55 expose board ID, raw selector (255=unused), and flags
(bit 0 initialized, bit 1 stable); packet lengths remain unchanged.
Python 3 standard library only; GUI requires tkinter. On Connect, only status
queries are sent. OFFLINE and zero CAN counters are expected before Power ON.
GUI IP edits take effect only after Connect. Stop before switching motor indices.

Then: Power ON -> wait at least 1.5 s -> MONITOR/error NONE -> Enable at zero ->
ENABLED/mode 3 -> only after confirming velocity units, Set velocity -> Quick Stop.
There is no automatic power-on, enable, fault reset, or reconnect/re-enable.
There is no MCU/PC application speed cap; velocity is signed 32-bit in the configured EPOS units (confirmed rpm for this drive).
Enable energizes the motor even at zero target. Only test motion after commissioning.
Quick Stop confirmation refers to drive state; actual mechanical stop depends on
drive configuration. Actual velocity is only refreshed while ENABLED and may be
stale in other states. Errors latch until an explicit Reset drivers recovery or MCU reset.

## Link supervision

After an accepted Enable, the GUI renews the node's lease every ~100 ms with a
new sequence number. Keepalive does not change speed. The enabled-state lease
remains 1000 ms. A failed exchange disables automatic renewal and makes one
best-effort Quick Stop request. No automatic re-enable occurs on recovery.
Closing the GUI requests Quick Stop best-effort; if that packet is lost the lease
expires. A broken CAN link or halted MCU cannot guarantee a remote stop; EPOS-side
communication supervision is still required. Stopping the CPU is not a motor stop.

Only one source IP/UDP port can mutate state at a time. Queries are unrestricted.
Another controller is rejected while the owner was active in the last 3 seconds,
or while any motor is enabling/enabled/stopping. Read-only queries do not renew
motor lease or ownership. Use this unauthenticated bench protocol on an isolated
trusted network. It is not intended for Internet exposure.

## Wire format

All integers little endian. Request is exactly 16 bytes. Unknown magic or invalid
length is silently dropped. Chained pbufs are copied with pbuf_copy_partial.

| Offset | Size | Request |
|---|---|---|
| 0 | 4 | ASCII ABS1 |
| 4 | 4 | sequence number |
| 8 | 1 | command |
| 9 | 1 | zero-based motor index |
| 10 | 2 | reserved, zero |
| 12 | 4 | signed velocity (command 2 only) |

Commands: 0=query, 1=enable at zero, 2=set velocity, 3=quick stop,
4=power on, 5=power off, 6=renew lease, 7=reset all drivers. Power affects the shared board output.
The default motor count is one; index 1 requires MOTOR_COUNT=2.

Result codes: 0=accepted, 1=reserved field invalid, 2=unknown command,
3=invalid motor index, 4=state not allowed or transient busy, 5=reserved (former application velocity cap),
7=stale/reused sequence with different content, 8=another controller owns control.
Accepted means API acceptance, not completed state transition. Poll state/error.
Enable can be transiently rejected while a background SDO is pending; retry manually
with a new sequence after checking MONITOR. The client never retries mutations itself.

The most recent mutating request/reply is cached per owner. An exact duplicate
returns its original response without repeating the action or renewing lease.
Older sequence numbers are rejected (signed modular ordering, half-range window).
Fresh queries always return current status and do not affect mutation sequencing.

Response is exactly 56 bytes, sent to the source address/port:

| Offset | Size | Response |
|---|---|---|
| 0 | 4 | ASCII ABR1 |
| 4 | 4 | echoed sequence |
| 8..10 | 1 each | echoed command, index, result |
| 11..15 | 1 each | state, error, node ID, heartbeat, signed mode |
| 16 | 2 | statusword |
| 18 | 2 | EMCY code |
| 20,24 | 4 each | signed target, actual velocity |
| 28 | 4 | SDO abort code |
| 32,36 | 4 each | CAN RX frames, TX registrations (not bus success) |
| 40 | 4 | CAN ESR |
| 44 | 4 | application uptime ms (TIM6, starts after initialization) |
| 48 | 4 | command lease ms |
| 52 | 1 | configured motor count |
| 53..55 | 1 each | board ID, unused selector (255), identity flags (initialized=1, stable=2) |

lwIP raw API is called only from main context. CAN motor timing still uses TIM6.
Firmware contains no UDP processing in timer/CAN interrupts. Long Ethernet HAL
calls can still delay main-context supervision; this is a bench implementation.

## Validation

Run `python -m unittest discover -s tools -p test_udp_motor.py` for protocol tests.
Firmware and PC encoding must use the offsets above; firmware is compiled with the
STM32 ARM toolchain. Physical Ethernet/CAN/motion tests require the actual board.

## Startup acquisition update
Power ON allows up to 8000 ms to obtain the first 0x6041 response. First probe is no earlier than 1500 ms; a usable heartbeat allows it then, otherwise the fixed grace is 3000 ms. Only this initial read-only query retries, 250 ms after a 200 ms response timeout. Once a valid response is received, ordinary SDO timeouts remain 200 ms and latch errors. No enable/speed write is retried. Inspect motor_status startup_retries, failed_object, failure_ms, failure_can_esr if startup fails. Do not press Enable until MONITOR/error NONE.

## Reset drivers (command 7)

Use the new Reset drivers button to recover from MCU-latched errors without
restarting the MCU or Ethernet connection. Reset is accepted only when every
configured motor is OFFLINE, MONITOR, STOPPED or ERROR. ENABLED, ENABLING,
STOPPING, or a reset already in progress is rejected (STATE_OR_BUSY).

The board has a shared EPOS power output, so this operation always affects all
configured motors, even if the GUI selected a single index:
1. Set target velocity to zero, cancel old transactions and enter RESETTING (7).
2. Drive EPOS power output OFF for 1000 ms without blocking main/Ethernet.
3. Deinitialize/reinitialize CAN, discard stale mailboxes/RX, clear MCU node state.
4. Drive EPOS power ON and run the initial read-only acquisition again (up to 8 s).
5. Reach MONITOR if healthy. No automatic Enable or speed restoration.

Command OK means reset accepted, not completed. Poll state/error. The GUI turns
keepalive off on reset acceptance. Old error details may remain visible while
RESETTING, and are cleared when CAN initialization succeeds. If CAN recovery
fails, power remains OFF and CAN error is latched. Counters are retained.

This is a physical power-output cycle rather than CANopen Fault Reset/NMT Reset
Node. If EPOS logic is independently powered (e.g. another supply), this output
may not fully power-cycle it. Persistent drive faults/wiring/mechanical problems
can recur and must be diagnosed; reset does not resolve their underlying cause.
Restart the PC program after updating to show the new button.
## Extended feedback diagnostics (read-only command 8)

Request still uses ABS1/16 bytes. Command 8 is read-only like command 0 and does
not acquire ownership, renew the motion lease, or change any drive settings.
Reply is ABR1/128 bytes. First 56 bytes preserve the existing layout. Ordinary
commands still reply with 56 bytes for compatibility. Restart the PC program
and load the new firmware together. The GUI polls command 9 every ~100 ms.

While ENABLED target/controlword writes and status/actual-speed feedback use
synchronous PDO. The acknowledged diagnostic SDO cycle is target readback ->
speed demand -> controlword readback
-> position 0x6064:0 -> current demand 0x30D0:0 -> actual current 0x30D1:2 -> torque
0x6077:0. Each request waits for ACK and at least 5 ms before the next request.
This is sequential bench telemetry, not simultaneous high-rate acquisition.
Only read-only telemetry was added; no gains/current limits/profile parameters
were modified. Current is signed mA. Torque is signed per mille of configured
rated torque, calculated from current and motor settings, not a torque sensor.
Position is in configured device position units. Delta is per firmware position
sample, not per GUI refresh, and handles signed counter wrap modulo 32 bits.

Valid mask bits: 0=position, 1=current demand, 2=actual current, 3=torque,
4=target readback, 5=speed demand, 6=controlword, 7=actual speed, 8=statusword.
A value without a valid bit is 'not sampled', not zero. Optional object/subindex
not-found SDO aborts (0x06020000/0x06090011) mark unsupported and skip that object
until reset, without latching a motion error. Other protocol/timeout/drive faults
still use normal error handling. Raw diagnostic abort/object/subindex are shown.
Samples are only refreshed while ENABLED; after stop/error, ages increase and
previous values must not be interpreted as current output. General statusword
Warning bit 7 / Internal limit bit 11 and Controlword Halt bit 8 are displayed.
These flags do not prove which physical component or parameter caused a stall.

Extension offsets:
- 56 i32 target readback; 60 i32 velocity demand
- 64 u16 controlword; 66 i16 torque per mille
- 68 i32 current demand mA; 72 i32 actual current mA
- 76 i32 position; 80 i32 position delta
- 84 u32 valid mask; 88 u32 unsupported mask; 92 u32 position sample ms
- 96 u32 diagnostic abort; 100 u16 diagnostic object; 102 u8 subindex; 103 u8 motion flags (bit7 reversal supported, bit0 pending)
- 104 u32 current sample ms; 108 u32 torque sample ms; 112 u32 motor clock ms
- 116 u32 demand sample ms; 120 u32 controlword sample ms; 124 u32 target sample ms

Save diagnostics CSV exports the most recent 600 extended replies (~60 s) with PC
time, state, raw values, validity flags and timestamps. Capture a short test at
10 rpm, then Quick Stop and save. Do not raise speed/torque simply to overcome
unknown mechanical resistance. Read-only diagnostics cannot prove commissioning
or fix a wiring/mechanical problem on their own.

Definitions verified against EPOS4 Firmware Specification and maxon support:
https://www.maxonmotor.com/medias/sys_master/root/8830265393182/EPOS4-Firmware-Specification-En.pdf
https://support.maxongroup.com/hc/en-us/articles/360011690019-EPOS4-IDX-Object-Torque-actual-value
## Application speed cap removed
The user-requested update removes the former +/-100 test cap from Motor_SetVelocity, UDP validation and the GUI. Only the signed 32-bit wire-format range remains. Drive-side maximum speed/current limits are unchanged. Restart the PC tool and download the rebuilt firmware together.

## Passive Hall / encoder hand-turn measurement

Update MCU firmware and restart the PC tool together. New read-only command 9
returns 168 bytes: the existing 128-byte feedback plus Hall/configuration fields.
Commands 0 and 8 remain compatible. Command 9 never renews lease or acquires control.

Reset drivers, or Power ON from a fresh boot, and wait for MONITOR with statusword
(state mask 0x006F) 0x0040 = Switch on disabled. DO NOT press Enable or Set velocity.
Power to the drive/sensors is necessary; motor output must remain disabled. Quick
Stop alone is not a guarantee of de-energized output, so STOPPED is not accepted.
The firmware acquires passive data only while MONITOR and recent disabled status
is confirmed: status -> configured encoder pulses 0x3010:1 (u32) -> configured pole
pairs 0x3001:3 (u8) -> position unit 0x60A8:0 (u32) -> position 0x6064 -> Hall pattern
0x301A:2 (u16) -> position again. All are reads. Optional unsupported objects are
marked N/A; no gain, commutation, direction, or pole-count setting is written.

1. Wait for fresh Hall pattern (001..110), position and configuration values.
2. Mark motor shaft position physically. Keep clear of pinch points in attached mechanism.
3. Click Begin one hand turn (disabled).
4. Gently rotate exactly ONE motor-shaft mechanical revolution in ONE direction,
   taking at least 30 seconds. Do not force a resistant mechanism or reverse direction.
5. Return to the physical shaft mark, stop and click End one hand turn promptly.
6. Save CSV and share the result line and CSV. Repeat the same direction for consistency.

For one complete mechanical turn, six Hall transitions per electrical revolution
imply pole-pair candidate = transitions / 6. Encoder PPR candidate = absolute
position delta / 4 ONLY with position units 0x00B50000 (increments) and motor-shaft
incremental feedback as configured here. Absolute direction/polarity and motor-phase
commutation cannot be determined from a single count ratio. No automatic changes.
If the turn is partial, total turns uncertain, skipped transitions occurred, or
invalid 000/111 patterns are seen, the result is inconclusive. Even a one-bit
transition stream can miss complete cycles if motion is too fast; rotate slowly.
The GUI rejects stale data, non-disabled output and node changes. State changes
while measuring invalidate the capture. Counters reset on Reset drivers.

Extension (command 9, 168-byte response):
128 u16 Hall pattern; 130 u16 PDO configured; 132 u32 Hall sample time;
136 u32 observed Hall changes; 140 u32 non-single-bit changes (missed/invalid);
144 u32 invalid 000/111 samples; 148 u32 configured encoder PPR;
152 u32 configured pole pairs; 156 u32 position SI unit; 160 u32 status sample time;
164 u32 TPDO receive count. Existing valid/unsupported bits add 9=Hall,10=PPR,11=poles,12=SI unit.
CSV adds these fields and turn_phase (0 before capture,1 during,2 after). GUI keeps
~60 s; perform a 30-40 s turn and export promptly. Estimates need independent physical
full-turn confirmation and consistent repeated measurements.
