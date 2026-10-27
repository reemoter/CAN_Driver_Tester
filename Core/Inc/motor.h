#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"
#include "app_timing.h"

/* Configured nodes are independent. Set to 2 to include node 2. */
#define MOTOR_COUNT 1U
#define MOTOR_CAPACITY 2U
#define MOTOR_MASTER_NODE_ID 127U
#define MOTOR_REVERSAL_ZERO_BAND 5 /* Configured EPOS velocity units (rpm here). */
/* No application speed cap. Velocity uses signed 32-bit EPOS units (0x60A9). */

typedef enum {
    MOTOR_OFFLINE, MOTOR_MONITOR, MOTOR_ENABLING, MOTOR_ENABLED,
    MOTOR_STOPPING, MOTOR_STOPPED, MOTOR_ERROR, MOTOR_RESETTING
} MotorState;

typedef enum {
    MOTOR_ERR_NONE, MOTOR_ERR_SDO_TIMEOUT, MOTOR_ERR_SDO_ABORT,
    MOTOR_ERR_PROTOCOL, MOTOR_ERR_STATE_TIMEOUT, MOTOR_ERR_DRIVE,
    MOTOR_ERR_HEARTBEAT, MOTOR_ERR_CAN, MOTOR_ERR_RX_OVERFLOW,
    MOTOR_ERR_LEASE, MOTOR_ERR_TX, MOTOR_ERR_PDO_TIMEOUT, MOTOR_ERR_REVERSAL_TIMEOUT
} MotorError;

typedef struct {
    uint8_t node_id;
    MotorState state;
    MotorError error;
    uint16_t statusword;
    int32_t actual_velocity;
    int32_t target_velocity;
    int8_t mode_display;
    uint8_t heartbeat_state;
    uint8_t heartbeat_seen;
    uint32_t heartbeat_ms;
    uint32_t last_response_ms;
    uint32_t sdo_abort;
    uint16_t emcy_code;
    uint32_t rx_count;
    uint32_t startup_retries;
    uint16_t failed_object;
    uint32_t failure_ms;
    uint32_t failure_can_esr;
    int32_t drive_target_velocity; /* 0x60FF readback, not the MCU command */
    int32_t demand_velocity;       /* 0x606B, trajectory generator output */
    uint16_t drive_controlword;    /* 0x6040 readback */
    uint32_t drive_target_ms, demand_ms, controlword_ms;
    int32_t current_demand_ma, current_actual_ma, position_actual, position_delta;
    int16_t torque_actual_permille;
    uint32_t feedback_valid, feedback_unsupported;
    uint32_t position_ms, current_ms, torque_ms;
    uint32_t diagnostic_abort;
    uint16_t diagnostic_object;
    uint8_t diagnostic_subindex;
    uint16_t hall_pattern;
    uint32_t hall_ms, hall_edges, hall_skipped, hall_invalid;
    uint32_t encoder_ppr, configured_pole_pairs, position_unit, statusword_ms;
    uint32_t pdo_rx_count, pdo_ms;
    uint8_t pdo_ready;
    int32_t requested_velocity; /* User request; target stays zero during reversal. */
    uint8_t reversal_pending;
} MotorStatus;

typedef struct {
    uint32_t rx_count, tx_count, rx_overflow, tx_busy, can_error;
    uint32_t scheduler_late, can_esr;
    uint32_t sync_count, sync_ms, rpdo_count, cycle_missed, cycle_tx_errors;
} MotorDiagnostics;

/* Debugger mailbox: write motor_index/velocity/command first, request_id last.
 * 1=enable at zero speed, 2=set velocity/renew lease, 3=quick stop,
 * 4=power on, 5=power off. No automatic enable or fault reset.
 * Debugger owns all inputs including request_id; firmware never clears them.
 * Increment request_id for each request; wait for completed_id to match.
 * Do not edit arguments while a request is outstanding.
 */
typedef struct {
    uint32_t motor_index;
    int32_t velocity;
    uint32_t command;
    uint32_t request_id;
    uint32_t completed_id;
    uint32_t accepted;
} MotorDebugCommand;

extern volatile MotorStatus motor_status[MOTOR_CAPACITY];
extern volatile MotorDiagnostics motor_diagnostics;
extern volatile MotorDebugCommand motor_debug;

HAL_StatusTypeDef Motor_Init(void);
void Motor_Process(void); /* main context only */
uint8_t Motor_Enable(uint32_t index);
uint8_t Motor_SetVelocity(uint32_t index, int32_t velocity);
/* Cancel the trajectory and publish zero at the next CAN cycle; stay enabled. */
uint8_t Motor_ZeroTarget(uint32_t index);
uint8_t Motor_QuickStop(uint32_t index);
uint8_t Motor_SetPower(uint8_t on);
uint8_t Motor_KeepAlive(uint32_t index);
uint8_t Motor_ResetDrivers(void); /* Shared power: resets all configured nodes. */
uint32_t Motor_TimeMs(void);

#endif
