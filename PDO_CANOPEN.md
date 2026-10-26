# EPOS4 synchronous PDO commissioning

The application keeps Profile Velocity Mode (6060=3). UDP commands and the
explicit enable/stop/reset workflow are unchanged. Board 2 remains 192.168.0.24.
The first Enable after driver power-on now configures PDOs through acknowledged
SDO writes, before the existing zero-target DS402 enable sequence.

## Bus configuration

One controller is the SYNC producer on its CAN bus. Node 127 is reserved for its
heartbeat; drive nodes are 1 (default) and optionally 2 (MOTOR_COUNT in motor.h).
Do not add a second SYNC producer or another node 127 on that bus.

| Frame | CAN ID | Payload, in wire order | Timing |
|---|---|---|---|
| RPDO1 | 0x200 + node | int32 target velocity, uint16 controlword | Before each SYNC |
| SYNC | 0x080 | Empty | 10 ms nominal |
| TPDO1 | 0x180 + node | uint16 statusword, int8 mode display, int32 actual velocity | Each SYNC |
| Master heartbeat | 0x77F | 0x05 | 100 ms, while cyclic service succeeds |
| Drive heartbeat | 0x700 + node | NMT state | 100 ms after configuration |

All multi-byte values are little endian. RPDO transmission type and TPDO
transmission type are 1. The six-byte RPDO places target before controlword,
because EPOS4 PVM adopts a new target on a subsequent controlword write.
Unused RPDO2-4 and TPDO2-4 are disabled to prevent inherited mappings from
applying competing commands or generating unrelated traffic.

Configuration requires a fresh pre-operational heartbeat after the directed
NMT pre-op command. The current drive already produces heartbeats. If heartbeat
production is disabled in another drive's saved configuration, enable it in
EPOS Studio before commissioning. Configuration then disables PDOs, clears
mapping counts, writes entries and counts, sets synchronous types, and enables
the configured COB-IDs. Every step waits for its matching SDO acknowledgement.
Any abort/timeout latches an error; there is no automatic SDO motion fallback.
The controller sends directed NMT start and performs the existing zero-speed
SDO enable sequence. A fresh TPDO must arrive before accepting velocity commands.

These changes remain in drive RAM. No Store parameters command is issued, and
motor/encoder/pole-pair settings are untouched. Full drive power cycling restores
its saved communication configuration. After a normal Quick Stop, re-enable
reuses the mapping. Reset drivers rebuilds it on the next explicit Enable.

## Scheduling and supervision

TIM6 supplies the existing 1 ms timestamp. Main-context Motor_Process performs
the nonblocking transaction; ISR code does not perform CANopen configuration.
Application timing settings now live in Core/Inc/app_timing.h; AppScheduler
dispatches motor service and PHY checks using AppTime. See SCHEDULING.md.
RPDOs are transmitted one at a time and checked for actual bxCAN TXOK before
SYNC is queued. This prevents SYNC's higher arbitration priority from overtaking
RPDOs. Both configured, enabled motors share the same SYNC. A single enabled
motor works without enabling the other one. No missed cycle is replayed.
This is cooperative scheduling, not a hard real-time guarantee; inspect
motor_diagnostics.scheduler_late and cycle_missed under Ethernet load before
reducing APP_CAN_CYCLE_MS. The initial 10 ms leaves room for two RPDOs,
SYNC, TPDOs and diagnostics with the existing 1 ms service resolution.

Primary feedback and velocity commands use PDO while enabled. Read-only SDO
diagnostics run at a slower pace; commissioning and passive hand-turn acquisition
continue using SDO. Quick Stop also retains a confirmed SDO stop path.

- Missing TPDO for 100 ms, including the first one: MOTOR_ERR_PDO_TIMEOUT (11).
- Cyclic mailbox busy/completion failure for 5 ms: MOTOR_ERR_TX.
- A quick stop or fault invalidates an outstanding cyclic batch. The next batch
  overwrites commands with zero velocity/controlword 2 before SYNC.
- Lease and PDO/heartbeat supervision run even with a diagnostic SDO pending.
- PC command lease remains 1000 ms; there is no automatic re-enable.
- The controller configures consumer 1016:1 for node 127 / 300 ms, disables
  consumer 1016:2, sets 1029:1=0 and 6007=3 (quick-stop ramp, then disable).
  Some drive firmware omits 1029:1; only object/subindex-not-found for that
  setting is recorded as an optional diagnostic and skipped. Consumer 1016,
  stop reaction 6007, PDO mapping and all other errors remain mandatory.
  Master heartbeat is sent only when cyclic service recently completed SYNC.
  This supervises a frozen/disconnected MCU at the drive, after the first
  heartbeat is received. It does not establish a verified mechanical stopping
  time; deceleration follows the drive's existing quick-stop parameters.

## Test on hardware

Download the rebuilt firmware, run the updated PC tool, and connect to the
selected board's address. Power ON, wait for MONITOR, then Enable at zero.
Wait for ENABLED / mode 3, `PDO configured=True`, and an increasing `TPDO
received` counter before applying velocity. Use the already commissioned speed
and verify Quick Stop, re-enable at zero and Reset drivers.

Read-only debugger fields: motor_status[i].pdo_ready, pdo_rx_count, pdo_ms;
motor_diagnostics.sync_count, sync_ms, rpdo_count, cycle_missed,
cycle_tx_errors. Use UDP for writes; J-Link Live Expressions remain read-only.
For timeout/abort, save CSV and record failed_object/sdo_abort rather than
repeatedly resuming motion. Actual on-wire timing, drive PDO adoption and
communication-loss stopping still require hardware validation.

UDP packet lengths stay 56/128/168 bytes. Command 9 reuses reserved offset
130 (uint16 PDO configured) and 164 (uint32 TPDO receive count). Updated GUI and
CSV expose both; older firmware reports zero in these reserved fields.

## Verification and references

`python tests/pdo/run_tests.py` compiles production motor.c against a mocked HAL
with MSVC, testing both MOTOR_COUNT=1 and 2. It checks NMT/configuration/enable,
mapping aborts, negative velocity layout, RPDO-before-SYNC TX completion,
missing ACK, busy mailboxes, stop invalidation, PDO/lease/heartbeat supervision,
first-feedback gating and successful-SYNC heartbeat gating. This is a host
state-machine test, not a hardware or bus timing measurement.

Official EPOS4 Firmware Specification (2025-05), sections 3.4, 6.2.9,
6.2.12, 6.2.15-30 and 6.2.92:
https://www.maxongroup.co.uk/medias/sys_master/root/9444047912990/EPOS4-Firmware-Specification-En.pdf

Official communication-loss guidance:
https://support.maxongroup.com/hc/en-us/articles/360012982439-EPOS4-Motor-stop-at-communication-lost
