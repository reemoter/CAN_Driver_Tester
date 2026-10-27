#ifndef TEST_MOTOR_H
#define TEST_MOTOR_H
#include <stdint.h>
#define MOTOR_COUNT 1U
#define MOTOR_REVERSAL_ZERO_BAND 5
enum { MOTOR_OFFLINE, MOTOR_MONITOR, MOTOR_ENABLING, MOTOR_ENABLED,
       MOTOR_STOPPING, MOTOR_STOPPED, MOTOR_ERROR, MOTOR_RESETTING };
typedef struct {
    uint32_t state, pdo_rx_count, pdo_ms;
    int32_t actual_velocity, target_velocity, requested_velocity;
    uint8_t reversal_pending;
} MotorStatus;
extern volatile MotorStatus motor_status[1];
uint8_t Motor_SetVelocity(uint32_t i, int32_t v);
uint8_t Motor_ZeroTarget(uint32_t i);
uint8_t Motor_QuickStop(uint32_t i);
#endif
