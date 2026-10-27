# 상위 제어기 UDP 연동 명세

기준: 2026-10-27 현재 control_udp.c, motor.h, lift_motion.h, app_timing.h.
상위 제어기 → 장비를 요청, 장비 → 상위 제어기를 응답으로 표기한다.
현재 구현을 기술한 문서이며 펌웨어 기능을 변경하지 않는다.

## 1. 통신 조건

- 장비 기본 주소: 192.168.0.24/24, UDP 목적지 포트 2000.
- BOARD_CONFIG_ID 1/2/3/4는 .23/.24/.25/.26에 대응. 현재 기본 ID=2.
- 상위 제어기는 장비와 통신 가능한 주소를 사용하고, 한 UDP 소켓을 유지한다.
  응답은 요청의 송신 IP·송신 포트로 돌아온다. 상위의 로컬 포트는 2000일 필요 없다.
- 바이너리 프로토콜. 모든 다중 바이트 정수는 Little Endian.
  i8/i16/i32는 부호 있음, u8/u16/u32는 부호 없음. 오프셋은 0부터.
- 1 요청 = 1 UDP datagram = 정확히 16바이트. 문자열 명령이나 줄바꿈은 없다.
- 응답은 요청에 대해서만 전송한다. 자동 상태 방송/완료 이벤트 전송은 없으므로 폴링한다.
- 별도 애플리케이션 CRC, 인증, 암호화는 없다. 신뢰 가능한 제어망에서 사용한다.

## 2. 요청 패킷: 16바이트

| 오프셋 | 길이 | 형식 | 내용 |
|---|---:|---|---|
| 0 | 4 | ASCII | ABS1 (41 42 53 31) |
| 4 | 4 | u32 | sequence |
| 8 | 1 | u8 | command |
| 9 | 1 | u8 | motor_index: 현재 0만 유효; CAN Node-ID 1과 다름 |
| 10 | 2 | u16 | reserved=0 |
| 12 | 4 | i32 | value: 명령 인자 |

길이 또는 ABS1이 틀리면 응답 없이 폐기한다. reserved가 0이 아니면 결과 1.
명령/인덱스 오류는 아래 결과 코드로 반환한다.

| command | 이름 | value | 응답 길이 | 동작 |
|---:|---|---|---:|---|
| 0 | STATUS | 0 | 56 | 기본 상태 조회 |
| 1 | ENABLE | 0 | 56 | 0속도로 Enable 시작; 자동 전원 인가 없음 |
| 2 | SET_VELOCITY | 부호 있는 속도 | 56 | +Forward/하강, -Reverse/상승; 0은 정지 목표 |
| 3 | QUICK_STOP | 0 | 56 | 시퀀스 취소 및 기존 Quick Stop 경로 |
| 4 | POWER_ON | 0 | 56 | 공유 드라이버 전원 ON |
| 5 | POWER_OFF | 0 | 56 | 공유 전원 OFF; Enabled 등 활성 상태에서는 거부 |
| 6 | KEEPALIVE | 0 | 56 | 명령 lease 유지; Enabling/Enabled에서만 허용 |
| 7 | RESET_DRIVERS | 0 | 56 | 전체 드라이버 공유 전원을 1초 OFF 후 재기동; 자동 Enable 없음 |
| 8 | DIAGNOSTICS | 0 | 128 | 확장 진단 |
| 9 | FULL_DIAGNOSTICS | 0 | 168 | Hall/PDO 포함 진단 |
| 10 | UP | 양수 속도 크기 | 56 | 음수 방향 상승, S1 감지 시 목표 즉시 0 |
| 11 | DOWN | 양수 속도 크기 | 56 | 양수 방향 하강, S2 감지 시 목표 즉시 0 |
| 12 | LIFT_STATUS | 0 | 192 | 명령 9 진단 + 센서/Up·Down 상태; 권장 폴링 명령 |

속도는 설정된 EPOS 속도 단위이며 현재 시운전 설정은 rpm이다. SET_VELOCITY는
i32 범위, UP/DOWN은 1..2147483647 범위다. 이 범위는 기구의 허용 속도를 뜻하지 않는다.
사용하지 않는 value도 0으로 보내는 것을 규칙으로 한다.

예: sequence=1, motor_index=0, UP 150rpm 요청:

```text
41 42 53 31  01 00 00 00  0A 00 00 00  96 00 00 00
```

Python 패킹 예 (전송하지 않는 패킷 생성 코드):

```python
import struct
packet = struct.pack('<4sIBBHi', b'ABS1', sequence, command, motor_index, 0, value)
```

## 3. 모든 응답의 공통부: 0..55

응답 헤더는 ABR1 (41 42 52 31). 수신 시 송신 장비 IP/포트, 길이, 헤더,
sequence·command·motor_index 일치를 먼저 확인한다. 지연된 다른 응답은 구분해 폐기한다.

| 오프셋 | 형식 | 내용 |
|---|---|---|
| 0..3 | ASCII | ABR1 |
| 4..7 | u32 | 요청 sequence 그대로 |
| 8 | u8 | 요청 command 그대로 |
| 9 | u8 | 요청 motor_index 그대로 |
| 10 | u8 | result: 명령 처리 결과 |
| 11 | u8 | motor_state |
| 12 | u8 | motor_error |
| 13 | u8 | CANopen node_id: 현재 1 |
| 14 | u8 | 마지막 CANopen heartbeat 상태 |
| 15 | i8 | mode_display: 정상 PVM=3 |
| 16..17 | u16 | CiA402 statusword |
| 18..19 | u16 | EMCY 코드 |
| 20..23 | i32 | target_velocity: 최종 목표; S자 중간 목표와 다름 |
| 24..27 | i32 | actual_velocity |
| 28..31 | u32 | SDO abort |
| 32..35 | u32 | 전체 CAN RX 카운트 |
| 36..39 | u32 | 전체 CAN TX 카운트 |
| 40..43 | u32 | CAN ESR |
| 44..47 | u32 | MCU 애플리케이션 시각 ms |
| 48..51 | u32 | command lease ms: 현재 1000 |
| 52 | u8 | 구성된 모터 수: 현재 1 |
| 53 | u8 | 고정 보드 ID: 현재 기본 2 |
| 54 | u8 | 과거 스위치 입력 자리: 현재 255=미사용 |
| 55 | u8 | 보드 flags: bit0 초기화됨, bit1 유효한 고정 설정 |

result=0은 명령 수락/처리 성공이며 모션 완료를 뜻하지 않는다. 상태 조회도
result=0이면서 motor_state=ERROR일 수 있다. motor_error와 lift_state/reason을 별도로 본다.

### result 코드

| 값 | 이름 | 의미 |
|---:|---|---|
| 0 | OK | 수락/처리 성공 |
| 1 | BAD_PACKET | 예약 필드가 0이 아님 |
| 2 | BAD_COMMAND | 미지원 명령 |
| 3 | BAD_INDEX | 모터 인덱스 오류; Up/Down은 0 전용 |
| 4 | STATE_OR_BUSY | 현재 상태에서 실행 불가/동작 충돌/피드백 미준비 |
| 5 | VELOCITY_RANGE | Up/Down 속도 크기가 0 이하 |
| 6 | RESERVED | 현재 사용 안 함 |
| 7 | STALE_SEQUENCE | 오래된 sequence 또는 같은 sequence에 다른 요청 |
| 8 | OTHER_CONTROLLER | 다른 IP·포트가 제어권 보유 |
| 9 | SENSOR_NOT_READY | 센서 초기 확인 미완료 또는 샘플이 오래됨 |
| 10 | BOTH_LIMITS_ACTIVE | S1·S2 동시 감지로 시작 거부 |

### motor_state / motor_error

state: 0 OFFLINE, 1 MONITOR, 2 ENABLING, 3 ENABLED, 4 STOPPING,
5 STOPPED, 6 ERROR, 7 RESETTING.

error: 0 NONE, 1 SDO_TIMEOUT, 2 SDO_ABORT, 3 PROTOCOL, 4 STATE_TIMEOUT,
5 DRIVE, 6 HEARTBEAT, 7 CAN, 8 RX_OVERFLOW, 9 LEASE, 10 TX,
11 PDO_TIMEOUT, 12 REVERSAL_TIMEOUT.

heartbeat: 0 boot-up, 4 stopped, 5 operational, 127 pre-operational.

## 4. 확장 진단: command 8/9/12 공통, 56..127

| 오프셋 | 형식 | 내용 |
|---|---|---|
| 56..59 | i32 | drive_target_velocity: 드라이버에서 읽은 S자 중간 목표 |
| 60..63 | i32 | demand_velocity: 드라이버 내부 프로파일 요구 속도 |
| 64..65 | u16 | drive controlword |
| 66..67 | i16 | 실제 토크, 정격 대비 천분율 |
| 68..71 | i32 | current_demand, mA |
| 72..75 | i32 | current_actual, mA |
| 76..79 | i32 | position_actual, 드라이버 위치 단위 |
| 80..83 | i32 | position_delta, SDO 샘플 사이 변화량 |
| 84..87 | u32 | feedback_valid 비트맵 |
| 88..91 | u32 | feedback_unsupported 비트맵 |
| 92..95 | u32 | position 갱신 시각 ms |
| 96..99 | u32 | 선택 진단의 SDO abort |
| 100..101 | u16 | 해당 진단 object index |
| 102 | u8 | 해당 진단 subindex |
| 103 | u8 | bit7 기존 수동 반전 interlock 지원, bit0 수동 반전 대기 |
| 104..107 | u32 | current 갱신 시각 ms |
| 108..111 | u32 | torque 갱신 시각 ms |
| 112..115 | u32 | MCU 애플리케이션 시각 ms |
| 116..119 | u32 | demand 갱신 시각 ms |
| 120..123 | u32 | controlword 갱신 시각 ms |
| 124..127 | u32 | drive_target 갱신 시각 ms |

valid/unsupported 비트맵: bit0 position, bit1 current demand, bit2 current actual,
bit3 torque, bit4 drive target, bit5 demand, bit6 controlword, bit7 actual velocity,
bit8 statusword, bit9 Hall, bit10 encoder PPR, bit11 pole pairs, bit12 position unit.
valid는 샘플 보유 여부이며 최신성 보장이 아니다. unsupported가 켜진 항목은 미지원.
나이 계산은 u32 모듈러 차 `(now - updated_ms) & 0xffffffff`를 사용한다.
정지/오류 중 전류 등 일부 SDO 진단은 갱신되지 않을 수 있어 오래된 값을 사용하면 안 된다.
바이트 103의 수동 반전 플래그와 Up/Down REVERSING은 별개다.

## 5. Hall/PDO 확장: command 9/12 공통, 128..167

| 오프셋 | 형식 | 내용 |
|---|---|---|
| 128..129 | u16 | Hall 패턴 |
| 130..131 | u16 | PDO ready |
| 132..135 | u32 | Hall 갱신 시각 ms |
| 136..139 | u32 | Hall edge 수 |
| 140..143 | u32 | Hall skipped 수 |
| 144..147 | u32 | Hall invalid 수 |
| 148..151 | u32 | encoder PPR |
| 152..155 | u32 | pole pairs |
| 156..159 | u32 | position unit 코드 |
| 160..163 | u32 | statusword/TPDO 갱신 시각 ms |
| 164..167 | u32 | PDO RX 카운트 |

## 6. 센서·Up/Down 확장: command 12 전용, 168..191

| 오프셋 | 형식 | 내용 |
|---|---|---|
| 168 | u8 | lift_state |
| 169 | u8 | lift_reason |
| 170 | u8 | sensor_raw_high 마스크 |
| 171 | u8 | sensor_active 마스크 |
| 172 | u8 | sensor_valid 마스크 |
| 173 | u8 | Up/Down 대상 모터 인덱스: 현재 0 |
| 174 | u8 | lift protocol revision: 현재 1 |
| 175 | u8 | 예약: 0 |
| 176..179 | u32 | 시퀀스 시작 시각 ms |
| 180..183 | u32 | 시퀀스 경과 시간 ms |
| 184..187 | i32 | 시퀀스 속도: Up 음수, Down 양수 |
| 188..191 | u32 | 이동 제한 시간 ms: 현재 10000 |

센서 마스크는 각각 bit0=S1, bit1=S2. raw는 High=1/Low=0,
active는 감지=1/미감지=0, valid는 초기 입력 확인 완료=1이다.
valid가 0인 센서의 active=0을 정상 미감지로 해석하지 않는다.

lift_state: 0 IDLE, 1 UP, 2 DOWN, 3 STOPPING, 4 DONE, 5 CANCELLED,
6 ERROR, 7 REVERSING.

lift_reason: 0 NONE, 1 LIMIT, 2 USER_STOP, 3 TIMEOUT, 4 SENSOR_INVALID,
5 SENSOR_CONFLICT, 6 MOTOR_ERROR, 7 STOP_TIMEOUT, 8 INTERRUPTED.

목적지 도착 완료 판단은 `lift_state=4(DONE)` 및 `lift_reason=1(LIMIT)`.
STOPPING은 정지 확인 중이지 완료가 아니다. 정상 완료 후 motor_state는 ENABLED 유지.

## 7. 상위 제어기 권장 제어 순서

1. UDP 소켓 생성 후 command 12 조회. 센서 valid와 모터·시퀀스 상태 확인.
2. command 4 POWER_ON. 응답 수락 이후 MONITOR까지 폴링.
3. command 1 ENABLE. command 6 KEEPALIVE를 100 ms마다 전송 시작.
   ENABLING에서 ENABLED까지 조회로 확인한다. 실패 시 자동 재Enable하지 않는다.
4. command 10 UP 또는 11 DOWN, value=양수 속도 크기.
5. KEEPALIVE 100 ms 주기 유지와 함께 command 12를 약 100 ms마다 조회.
6. 목적지 센서 확정 감지 시 펌웨어가 MCU S자 목표를 취소하고 즉시 0 목표를 전송.
   드라이버 감속/기계 관성까지 즉시 없어지는 의미는 아니다.
7. 실제 정지 확인 후 DONE/LIMIT. 이후에도 Enabled를 유지할 동안 KEEPALIVE 지속.
8. 정상 완료 뒤 반대 명령 가능. 구동 중 반대 명령도 수락하며 즉시 0 목표로 전환하고,
   정지 범위(현재 ±5rpm)에서 100 ms 확인 후 최신 요청 방향으로 S자 출발한다.
   기존 목적지 센서 도착까지 기다리지 않는다. REVERSING 중 명령 변경은 최신 방향이
   우선이나 정지 확인 제한 3초는 연장되지 않는다.

이미 목적지 센서가 active라면 추가 이동 없이 DONE. 양 센서 active는 시작 거부.
동일 방향 명령은 속도만 변경하며 기존 이동 제한 시간을 연장하지 않는다.
현재 전체 이동 제한은 10초이며 펌웨어 빌드 설정에서 변경 가능하다.
한계 도착/이동 시간 초과의 0 목표 정지 후 3초 내 실제 정지가 확인되지 않으면 Quick Stop.

### 정지·취소 및 복구

- Up/Down 중 command 2/value=0: 시퀀스 취소 + S자 우회 즉시 0 목표, Enabled 유지.
- Up/Down 외의 command 2/value=0: 기존 수동 속도 제어의 S자 감속 적용.
- command 3: 시퀀스 취소 + Quick Stop. 정상 0 목표 정지와 상태 전이가 다르다.
- 동작 중 수동 비영속도/Enable/Power/Reset 요청은 STATE_OR_BUSY로 거부한다.
- Quick Stop 후에는 STOPPED 확인 및 명시적 재Enable이 필요할 수 있다.
  드라이브/펌웨어 ERROR는 원인을 확인하고 필요한 경우 command 7로 공유 리셋한다.
- command 7은 Enabling/Enabled/Stopping/Resetting 중 거부한다. 자동 재Enable은 하지 않는다.
- 센서 자동 정지는 Up/Down 시퀀스 기능이다. 수동 SET_VELOCITY는 리미트 자동 정지 모드가 아니다.

## 8. KEEPALIVE, 소유권, sequence와 재전송

- 상태 조회 0/8/9/12는 lease를 갱신하지 않는다. KEEPALIVE를 별도로 보내야 한다.
- Enabled에서 명령 lease 1000 ms가 만료되면 LEASE 오류·기존 보호 정지 경로로 전환한다.
  정상 0속도/DONE 상태도 Enabled이면 KEEPALIVE가 필요하다. 시퀀스가 자체 갱신하지 않는다.
- 응답 제한은 상위 제어기 정책이다. 현재 PC 도구는 200 ms를 사용한다.
  timeout은 명령 미실행의 증거가 아니다. 응답 유실 후 장비 상태를 확인한다.
- 제어 명령 1..7/10/11은 송신 IP+송신 UDP 포트로 소유권을 구분한다.
  조회 0/8/9/12는 다른 소켓에서도 가능하지만 제어권은 얻지 않는다.
- 다른 제어기의 활성 소유자가 있으면 거부. 활성 상태가 없어도 마지막 소유자 요청 후
  3000 ms 동안은 다른 소켓의 제어를 거부한다. Enabled=0속도도 활성 상태다.
  기존 PC 도구와 상위 제어기를 동시에 구동하지 않는다.
- 새 요청마다 sequence를 u32로 1 증가. 소켓을 유지하고 제어·조회·KEEPALIVE에
  공통 카운터를 쓰는 방식을 권장한다. 한 소켓에서 한 번에 한 요청 처리 권장.
- 직전 캐시된 제어 요청과 sequence 및 16바이트 전체가 같으면 이전 56바이트 응답을
  재전송하고 명령을 재실행하지 않는다. lease도 갱신하지 않으며 응답 상태도 당시 값이다.
- 중복 보장은 마지막 캐시 제어 요청 1개뿐이다. 다른 제어 명령(KEEPALIVE 포함)이
  처리된 뒤 이전 요청을 재전송하면 STALE_SEQUENCE일 수 있다. 새 상태를 조회한다.
- 같은 sequence에 다른 인자를 붙이거나 이전 sequence를 쓰면 결과 7.
  순서는 signed 32bit 모듈러 차로 비교하므로 순방향 점프를 2^31 미만으로 유지한다.
- 결과 4/5/9/10 등 상태·범위 거부도 캐시된다. 조건이 바뀐 뒤 다시 시도할 때는 새 sequence.
- 프로세스 재시작으로 송신 포트가 바뀌면 다른 제어기로 취급된다. 기존 제어권을
  정상 해제할 수 있도록 종료 시 정지 및 장비 상태 확인 절차를 마련한다.

## 9. 구현 근거 파일

- Core/Src/control_udp.c: 요청 검증, 명령, 응답 오프셋, 소유권·중복 처리.
- Core/Inc/motor.h, Core/Src/motor.c: 모터 상태·오류·lease·정지 경로.
- Core/Inc/lift_motion.h, Core/Src/lift_motion.c: Up/Down 상태와 센서 동작.
- Core/Inc/app_timing.h: 주기·시간 제한.
- tools/udp_motor.py: 실제 송수신/파서 예제. 상위 연동에 GUI 부분은 필요 없다.
