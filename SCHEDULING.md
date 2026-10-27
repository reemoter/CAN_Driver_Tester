# 애플리케이션 시간과 스케줄링

애플리케이션의 시간 기준은 TIM6의 1ms 인터럽트 하나다.
주기·응답 후 대기·감시 제한시간은 `Core/Inc/app_timing.h`에서 설정한다.
기본값은 기존 정상 동작 버전과 동일하다. 설정 변경은 재빌드·다운로드 후 적용된다.

```text
TIM6 1ms ISR → AppTime_Tick1ms() → app_time_ms
                                      ↓
main loop → AppScheduler_Process() → Motor_Process(): 1ms 서비스
                                  → PHY 링크 확인: 100ms
          → MX_LWIP_Process(): 패킷·라이브러리 타임아웃 처리, 매 루프
```

## 설정 위치와 의미

| 설정 | 기본값 | 사용처 / 의미 |
|---|---:|---|
| APP_MOTOR_SERVICE_MS | 1ms | CAN 상태 머신과 송신 완료 확인 서비스 |
| APP_CAN_CYCLE_MS | 10ms | RPDO 묶음의 다음 실행 예약, 완료 후 SYNC |
| APP_MASTER_HEARTBEAT_MS | 100ms | MCU heartbeat 생산 |
| APP_DRIVE_HEARTBEAT_MS | 100ms | SDO로 설정하는 EPOS heartbeat 생산 |
| APP_ETH_LINK_CHECK_MS | 100ms | 공통 스케줄러의 PHY 확인 |
| APP_SDO_DIAGNOSTIC_GAP_MS | 20ms | 운전 중 진단 응답 후 다음 요청까지 최소 간격 |
| APP_SDO_STATE_GAP_MS | 5ms | Enable·정지·대기 조회 응답 후 최소 간격 |
| APP_SDO_CONFIG_GAP_MS | 1ms | PDO 설정 응답 후 최소 간격 |
| APP_NMT_START_SETTLE_MS | 10ms | NMT start 요청 후 Enable SDO 시작 전 대기 |
| APP_CAN_TX_TIMEOUT_MS | 5ms | CAN 주기 송신의 대기·완료 제한 |
| APP_PDO_TIMEOUT_MS | 100ms | 첫 TPDO 및 후속 TPDO 제한 |
| APP_SDO_TIMEOUT_MS | 200ms | SDO 응답 제한 |
| APP_STATE_TIMEOUT_MS | 3000ms | Enable·정지의 단계별 진행 제한 |
| APP_COMMAND_LEASE_MS | 1000ms | PC 명령 유효기간, UDP 응답에도 같은 값 제공 |
| APP_DRIVE_HB_TIMEOUT_MS | 1500ms | MCU에서 드라이버 heartbeat 지연 감시 |
| APP_CONSUMER_TIMEOUT_MS | 300ms | EPOS에서 MCU heartbeat 지연 감시 |
| APP_SYNC_HEALTH_WINDOW_MS | CAN 주기의 3배 | MCU heartbeat 전 최근 SYNC 성공 확인 |
| APP_CONTROL_OWNER_GRACE_MS | 3000ms | 비활성 상태에서도 직전 제어권 유지하는 시간 |
| APP_REVERSAL_ZERO_HOLD_MS | 100ms | 방향 반전 전 TPDO 실제 속도가 0 부근을 유지해야 하는 시간 |
| APP_REVERSAL_TIMEOUT_MS | 5000ms | 방향 반전의 정지 확인 대기 제한 |
| APP_POWER_ON_SETTLE_MS | 1500ms | 전원 인가 후 첫 조회 최소 대기 |
| APP_STARTUP_NO_HB_GRACE_MS | 3000ms | heartbeat가 없을 때 첫 조회 대기 |
| APP_STARTUP_TIMEOUT_MS | 8000ms | 초기 응답 확보 전체 제한 |
| APP_STARTUP_RETRY_GAP_MS | 250ms | 초기 읽기 조회 재시도 간격 |
| APP_DRIVER_RESET_OFF_MS | 1000ms | Reset drivers의 전원 OFF 유지 |
| APP_ETH_TX_TIMEOUT_MS | 20ms | HAL Ethernet 송신 제한시간, 주기 작업이 아님 |

## 실행 방식

`app_scheduler.c`의 작업 표가 모터 서비스와 PHY 작업을 호출한다.
CAN 사이클과 응답에 의존하는 SDO 예약은 모터 모듈이 관리하지만,
모두 `AppTime_NowMs()`와 `app_timing.h`만 사용한다.
UDP 제어권·응답 시간, PHY 진단 타임스탬프도 같은 시계를 사용한다.
시계는 드라이버 전원 순환·Reset drivers로 초기화되지 않는다.

주기 작업은 지연되더라도 한 번만 실행하고 현재 시점에서 다음 실행을 예약한다.
과거 작업을 몰아서 재실행하지 않는다. 32비트 unsigned 시간 차이를 사용하여
약 49.7일마다 발생하는 millisecond 카운터 넘침에도 동작한다.
SDO 20ms/5ms는 고정 주기가 아니라 **응답 시점부터의 최소 대기시간**이다.
RPDO→실제 TX 완료→SYNC 순서는 유지된다.

스케줄러는 메인 루프 기반이므로 하드 실시간 보장은 아니다.
Ethernet HAL 호출, 버스 지연 등의 영향을 받는다. 각 주기는 관련 감시 시간과
함께 조정해야 한다. 현재 모터 서비스는 1ms 고정이며, 2개 RPDO·SYNC·완료 확인에
서비스 슬롯이 필요하므로 CAN 주기를 4ms 미만으로 설정하면 컴파일 오류로 알린다.
설정 가능한 최솟값이 실제 하드웨어의 안정 동작을 보장하는 것은 아니다.

## SysTick과 미사용 타이머

- TIM5/TIM7은 CubeMX 초기화만 남긴다. 시작하지 않고 NVIC도 비활성화한다.
- TIM6은 CubeMX의 PSC=84-1, ARR=1000-1, 타이머 입력 84MHz 설정을 유지한다.
  시간 단위가 1ms인 구조이므로 TIM6 자체의 주기를 변경하지 않는다.
- HAL의 내부 timeout, lwIP 내부 시간(`sys_now`)은 기존 SysTick을 유지한다.
  이는 애플리케이션 주기 작업을 예약하는 시계가 아니다.
- CubeMX lwip.c의 기존 100ms PHY 스케줄은 USER CODE의 `#if 0`로 비활성화하여
  코드 재생성 후에도 이중 호출되지 않도록 한다. PHY 호출은 공통 스케줄러만 담당한다.
  스케줄러는 별도 파일의 `EthernetPhy_Process()`를 직접 호출한다.
  자동 생성되는 `ethernet_link_check_state()`가 비어 있어도 정상 동작한다.
- 보드 ID는 `board_identity.h`의 `BOARD_CONFIG_ID`로 지정한다(기본 2, IP .24).
  스위치 GPIO 판독과 부팅 안정화 대기는 제거했다.
- PC 도구는 별도 프로세스라 OS monotonic 시계를 사용한다. PC의 약 100ms 조회·
  keepalive와 200ms 응답 제한은 펌웨어 타이머와 별개다.

## 확인 방법

Live Expressions는 읽기 전용으로 사용한다.

- `app_time_ms`: TIM6 공통 시간, 증가 확인
- `app_task_diagnostics[0]`: 모터 작업의 period_ms/runs/missed/max_lateness_ms
- `app_task_diagnostics[1]`: PHY 작업의 동일 정보
- `motor_diagnostics.sync_count/sync_ms/cycle_missed/cycle_tx_errors`
- `motor_status[0].pdo_ready/pdo_ms/pdo_rx_count`

`python tests/scheduler/run_tests.py`는 실제 시계/스케줄러를 호스트에서 실행하여
기본/변경 PHY 주기, 중복 호출 방지, 지연 후 한 번 실행, 카운터 넘침을 검증한다.
`python tests/pdo/run_tests.py`는 공통 시간 API를 사용하도록 변경한 실제 motor.c의
모터 1개/2개 상태 전이와 오류 경로를 검증한다. 실제 보드의 부하별 지연 측정은 별도다.
