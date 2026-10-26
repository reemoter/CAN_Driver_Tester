#ifndef APP_TIMING_H
#define APP_TIMING_H

/* Firmware application timing, in milliseconds. Runtime timebase: TIM6, 1 ms.
 * HAL/SysTick and lwIP internal timers remain library-owned. PC GUI timing is
 * independent. TIM6's generated hardware configuration must remain 1 ms. */

/* Periodic work. Motor service advances nonblocking CAN transactions. */
#define APP_MOTOR_SERVICE_MS             1U
#define APP_CAN_CYCLE_MS                10U
#define APP_MASTER_HEARTBEAT_MS        100U
#define APP_DRIVE_HEARTBEAT_MS         100U
#define APP_ETH_LINK_CHECK_MS          100U
#define APP_ETH_TX_TIMEOUT_MS           20U /* HAL transmit timeout, not a task period */

/* Minimum gaps after the preceding SDO response; not fixed-rate tasks. */
#define APP_SDO_DIAGNOSTIC_GAP_MS       20U
#define APP_SDO_STATE_GAP_MS             5U
#define APP_SDO_CONFIG_GAP_MS            1U
#define APP_NMT_START_SETTLE_MS         10U

/* Communication/state supervision. */
#define APP_CAN_TX_TIMEOUT_MS            5U
#define APP_PDO_TIMEOUT_MS             100U
#define APP_SDO_TIMEOUT_MS             200U
#define APP_STATE_TIMEOUT_MS          3000U
#define APP_COMMAND_LEASE_MS          1000U
#define APP_DRIVE_HB_TIMEOUT_MS       1500U
#define APP_CONSUMER_TIMEOUT_MS        300U
#define APP_SYNC_HEALTH_WINDOW_MS      (3U * APP_CAN_CYCLE_MS)
#define APP_CONTROL_OWNER_GRACE_MS    3000U
#define APP_REVERSAL_ZERO_HOLD_MS      100U
#define APP_REVERSAL_TIMEOUT_MS       5000U

/* Driver power/startup. */
#define APP_POWER_ON_SETTLE_MS        1500U
#define APP_STARTUP_NO_HB_GRACE_MS    3000U
#define APP_STARTUP_TIMEOUT_MS        8000U
#define APP_STARTUP_RETRY_GAP_MS        250U
#define APP_DRIVER_RESET_OFF_MS       1000U

_Static_assert(APP_MOTOR_SERVICE_MS == 1U, "Motor transaction engine needs 1 ms service");
_Static_assert(APP_CAN_CYCLE_MS >= 4U * APP_MOTOR_SERVICE_MS && APP_CAN_CYCLE_MS < 0x20000000U,
               "Allow service slots for two RPDOs, SYNC and completion");
_Static_assert(APP_ETH_LINK_CHECK_MS > 0U && APP_ETH_LINK_CHECK_MS < 0x80000000U,
               "PHY period must fit a modular deadline");
_Static_assert(APP_PDO_TIMEOUT_MS > APP_CAN_CYCLE_MS,
               "PDO supervision must allow at least one CAN cycle");
_Static_assert(APP_CAN_TX_TIMEOUT_MS > APP_MOTOR_SERVICE_MS,
               "TX supervision must allow cooperative service");
_Static_assert(APP_MASTER_HEARTBEAT_MS > 0U && APP_DRIVE_HEARTBEAT_MS > 0U &&
               APP_DRIVE_HEARTBEAT_MS <= 65535U && APP_CONSUMER_TIMEOUT_MS <= 65535U &&
               APP_CONSUMER_TIMEOUT_MS >= APP_MASTER_HEARTBEAT_MS + 20U,
               "Heartbeat producer/consumer values are inconsistent");
_Static_assert(APP_DRIVE_HB_TIMEOUT_MS > APP_DRIVE_HEARTBEAT_MS,
               "MCU heartbeat supervision must exceed drive producer period");
_Static_assert(APP_REVERSAL_ZERO_HOLD_MS >= APP_CAN_CYCLE_MS &&
               APP_REVERSAL_TIMEOUT_MS > APP_REVERSAL_ZERO_HOLD_MS &&
               APP_REVERSAL_TIMEOUT_MS < 0x80000000U,
               "Reversal needs fresh zero-speed samples and a bounded wait");
_Static_assert(APP_STARTUP_TIMEOUT_MS > APP_STARTUP_NO_HB_GRACE_MS &&
               APP_STARTUP_NO_HB_GRACE_MS >= APP_POWER_ON_SETTLE_MS,
               "Startup windows must allow the initial probe");
#endif
