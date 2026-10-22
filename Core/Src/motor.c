#include "motor.h"
#include "can.h"
#include <string.h>

_Static_assert(MOTOR_COUNT > 0U && MOTOR_COUNT <= MOTOR_CAPACITY,
               "MOTOR_COUNT must be 1 or 2");

volatile MotorStatus motor_status[MOTOR_CAPACITY];
volatile MotorDiagnostics motor_diagnostics;
volatile MotorDebugCommand motor_debug;

typedef struct { uint16_t id; uint8_t len; uint8_t data[8]; } Frame;
#define RX_SIZE 32U
static Frame rx_queue[RX_SIZE];
static volatile uint32_t rx_write, rx_read, clock_ms;
static uint32_t process_ms, overflow_seen;
static uint8_t power_on;
static uint32_t power_on_ms;
static uint8_t reset_pending;
static uint32_t reset_due_ms;
static uint32_t cycle_due_ms, cycle_sent_ms, cycle_mailbox, master_heartbeat_ms;
static uint8_t cycle_active, cycle_cursor, cycle_wait, cycle_sync, sync_seen;

typedef struct {
    uint8_t pending, read, size, phase, poll, stop_requested, subindex;
    uint16_t object;
    uint32_t sent_ms, phase_ms, next_ms, command_ms;
    uint8_t configuring, config_step, pdo_running;
    uint32_t pdo_started_ms;
} Node;
static Node nodes[MOTOR_CAPACITY];

static uint32_t le32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static HAL_StatusTypeDef send_tracked(uint16_t id, uint8_t len, uint8_t *data,
                                    uint32_t *tracked)
{
    CAN_TxHeaderTypeDef header = {0};
    uint32_t mailbox;
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) {
        motor_diagnostics.tx_busy++;
        return HAL_BUSY;
    }
    header.StdId = id;
    header.IDE = CAN_ID_STD;
    header.RTR = CAN_RTR_DATA;
    header.DLC = len;
    if (HAL_CAN_AddTxMessage(&hcan1, &header, data, &mailbox) != HAL_OK)
        return HAL_ERROR;
    motor_diagnostics.tx_count++;
    if (tracked) *tracked = mailbox;
    return HAL_OK;
}

static HAL_StatusTypeDef send_frame(uint16_t id, uint8_t len, uint8_t *data)
{
    /* Preserve the tracked mailbox completion flags until the cycle consumes
     * them; HAL_CAN_AddTxMessage would clear them when reusing that mailbox. */
    if (cycle_wait) return HAL_BUSY;
    return send_tracked(id, len, data, NULL);
}

/* A pending synchronous RPDO must never be followed by SYNC after its command
 * was invalidated. Rebuild the whole batch, including the other motor. */
static void cancel_cycle(void)
{
    if (cycle_wait) (void)HAL_CAN_AbortTxRequest(&hcan1, cycle_mailbox);
    cycle_active = cycle_wait = cycle_sync = 0;
    cycle_due_ms = clock_ms;
}

static void fail(uint32_t i, MotorError error)
{
    cancel_cycle();
    motor_status[i].failed_object = nodes[i].pending ? nodes[i].object : 0;
    motor_status[i].failure_ms = clock_ms;
    motor_status[i].failure_can_esr = hcan1.Instance->ESR;
    motor_status[i].error = error;
    motor_status[i].state = MOTOR_ERROR;
    motor_status[i].target_velocity = 0;
    nodes[i].pending = 0;
    /* Best effort only: a broken bus cannot guarantee a remote stop.
     * Never resume automatically after a communications error. */
    uint8_t stop[8] = {0x2B, 0x40, 0x60, 0, 2, 0, 0, 0};
    (void)send_frame((uint16_t)(0x600U + motor_status[i].node_id), 8, stop);
}

static void request_sub(uint32_t i, uint16_t object, uint8_t subindex, uint8_t size,
                        uint32_t value, uint8_t read)
{
    Node *n = &nodes[i];
    uint8_t data[8] = {0};
    data[0] = read ? 0x40U : (size == 1U ? 0x2FU : size == 2U ? 0x2BU : 0x23U);
    data[1] = (uint8_t)object;
    data[2] = (uint8_t)(object >> 8);
    data[3] = subindex;
    for (uint32_t j = 0; j < 4; j++) data[4+j] = (uint8_t)(value >> (8*j));
    HAL_StatusTypeDef result = send_frame((uint16_t)(0x600U + motor_status[i].node_id), 8, data);
    if (result == HAL_OK) {
        n->pending = 1;
        n->object = object;
        n->subindex = subindex;
        n->read = read;
        n->size = size;
        n->sent_ms = clock_ms;
    } else if (result == HAL_ERROR) fail(i, MOTOR_ERR_TX);
}

static void request(uint32_t i, uint16_t object, uint8_t size, uint32_t value, uint8_t read)
{ request_sub(i, object, 0, size, value, read); }

typedef struct { uint16_t object; uint8_t sub, size; uint32_t value; uint8_t node; } PdoSetting;
/* Volatile commissioning, exclusively in NMT pre-operational. No Flash store.
 * Target precedes controlword: EPOS4 PVM latches the target on controlword write. */
static const PdoSetting pdo_settings[] = {
    {0x1400,1,4,0x80000200U,1}, {0x1800,1,4,0xC0000180U,1},
    {0x1401,1,4,0x80000300U,1}, {0x1402,1,4,0x80000400U,1},
    {0x1403,1,4,0x80000500U,1}, {0x1801,1,4,0xC0000280U,1},
    {0x1802,1,4,0xC0000380U,1}, {0x1803,1,4,0xC0000480U,1},
    {0x1600,0,1,0,0}, {0x1600,1,4,0x60FF0020U,0},
    {0x1600,2,4,0x60400010U,0}, {0x1600,0,1,2,0},
    {0x1400,2,1,1,0},
    {0x1A00,0,1,0,0}, {0x1A00,1,4,0x60410010U,0},
    {0x1A00,2,4,0x60610008U,0}, {0x1A00,3,4,0x606C0020U,0},
    {0x1A00,0,1,3,0}, {0x1800,2,1,1,0}, {0x1800,3,2,0,0},
    {0x1005,0,4,0x80U,0}, {0x1017,0,2,100U,0},
    {0x6007,0,2,3U,0}, {0x1029,1,1,0U,0},
    {0x1016,2,4,0U,0},
    {0x1016,1,4,(MOTOR_MASTER_NODE_ID << 16) | MOTOR_CONSUMER_TIMEOUT_MS,0},
    {0x1400,1,4,0x00000200U,1}, {0x1800,1,4,0x40000180U,1}
};
#define PDO_SETTINGS_COUNT (sizeof(pdo_settings) / sizeof(pdo_settings[0]))

static void configure_pdo(uint32_t i)
{
    Node *n = &nodes[i];
    if (n->configuring == 1U) {
        /* NMT is unconfirmed; wait for a fresh pre-operational heartbeat. */
        if (motor_status[i].heartbeat_state == 0x7FU &&
            motor_status[i].heartbeat_seen &&
            (int32_t)(motor_status[i].heartbeat_ms - n->phase_ms) >= 0)
            n->configuring = 2U;
        return;
    }
    if (n->config_step < PDO_SETTINGS_COUNT) {
        const PdoSetting *p = &pdo_settings[n->config_step];
        request_sub(i, p->object, p->sub, p->size,
                    p->value + (p->node ? motor_status[i].node_id : 0U), 0);
        return;
    }
    uint8_t start[2] = {1U, motor_status[i].node_id};
    if (send_frame(0, 2, start) == HAL_OK) {
        n->configuring = 0;
        motor_status[i].pdo_ready = 1;
        n->phase = 0;
        n->phase_ms = clock_ms;
        n->next_ms = clock_ms + 10U;
    }
}

/* One tracked frame at a time. SYNC's lower CAN ID must not overtake queued
 * RPDOs: wait for actual TXOK, not merely a successful mailbox enqueue. */
static void process_pdo_cycle(uint32_t now)
{
    uint8_t any = 0;
    for (uint32_t i = 0; i < MOTOR_COUNT; i++) any |= nodes[i].pdo_running;
    if (!power_on || !any) { cancel_cycle(); return; }
    if (cycle_wait) {
        uint32_t index = cycle_mailbox == CAN_TX_MAILBOX0 ? 0U :
                         cycle_mailbox == CAN_TX_MAILBOX1 ? 1U : 2U;
        uint32_t tsr = hcan1.Instance->TSR >> (index * 8U);
        if (!(tsr & CAN_TSR_RQCP0)) {
            if (now - cycle_sent_ms < 5U) return;
        } else if (tsr & CAN_TSR_TXOK0) {
            cycle_wait = 0;
            if (cycle_sync) {
                motor_diagnostics.sync_count++;
                motor_diagnostics.sync_ms = now;
                sync_seen = 1;
                cycle_active = cycle_sync = 0;
                return;
            }
        } else {
            /* Arbitration loss/no ACK is a failed cycle with retransmit off. */
            cycle_sent_ms = now - 5U;
        }
        if (cycle_wait) {
            motor_diagnostics.cycle_tx_errors++;
            cancel_cycle();
            for (uint32_t i = 0; i < MOTOR_COUNT; i++)
                if (nodes[i].pdo_running && motor_status[i].state != MOTOR_ERROR)
                    fail(i, MOTOR_ERR_TX);
            cycle_due_ms = now + MOTOR_SYNC_PERIOD_MS;
            return;
        }
    }
    if (!cycle_active) {
        if ((int32_t)(now - cycle_due_ms) < 0) return;
        if (now - cycle_due_ms >= MOTOR_SYNC_PERIOD_MS) motor_diagnostics.cycle_missed++;
        cycle_due_ms = now + MOTOR_SYNC_PERIOD_MS; /* never replay missed cycles */
        cycle_active = 1;
        cycle_cursor = 0;
    }
    while (cycle_cursor < MOTOR_COUNT && !nodes[cycle_cursor].pdo_running) cycle_cursor++;
    uint8_t data[8] = {0};
    uint16_t id = 0x80U;
    uint8_t length = 0;
    if (cycle_cursor < MOTOR_COUNT) {
        uint32_t i = cycle_cursor;
        uint8_t enabled = motor_status[i].state == MOTOR_ENABLED && !nodes[i].stop_requested;
        uint32_t velocity = enabled ? (uint32_t)motor_status[i].target_velocity : 0U;
        for (uint32_t j = 0; j < 4; j++) data[j] = (uint8_t)(velocity >> (8U*j));
        data[4] = enabled ? 15U : 2U;
        id = (uint16_t)(0x200U + motor_status[i].node_id);
        length = 6;
    }
    HAL_StatusTypeDef result = send_tracked(id, length, data, &cycle_mailbox);
    if (result == HAL_OK) {
        cycle_sent_ms = now;
        cycle_wait = 1;
        if (length) { motor_diagnostics.rpdo_count++; cycle_cursor++; }
        else cycle_sync = 1;
    } else if (result == HAL_ERROR || now - (cycle_due_ms - MOTOR_SYNC_PERIOD_MS) >= 5U) {
        motor_diagnostics.cycle_tx_errors++;
        cancel_cycle();
        for (uint32_t i = 0; i < MOTOR_COUNT; i++)
            if (nodes[i].pdo_running && motor_status[i].state != MOTOR_ERROR) fail(i, MOTOR_ERR_TX);
        cycle_due_ms = now + MOTOR_SYNC_PERIOD_MS;
    }
}

static uint32_t feedback_bit(uint16_t object)
{
    switch (object) {
    case 0x6064: return 1U;
    case 0x30D0: return 2U;
    case 0x30D1: return 4U;
    case 0x6077: return 8U;
    case 0x60FF: return 16U;
    case 0x606B: return 32U;
    case 0x6040: return 64U;
    case 0x606C: return 128U;
    case 0x6041: return 256U;
    case 0x301A: return 512U;
    case 0x3010: return 1024U;
    case 0x3001: return 2048U;
    case 0x60A8: return 4096U;
    default: return 0;
    }
}

static void receive(const Frame *f)
{
    for (uint32_t i = 0; i < MOTOR_COUNT; i++) {
        volatile MotorStatus *s = &motor_status[i];
        Node *n = &nodes[i];
        if (f->id == 0x180U + s->node_id && n->pdo_running) {
            if (f->len != 7U) { fail(i, MOTOR_ERR_PROTOCOL); continue; }
            s->statusword = (uint16_t)(f->data[0] | (f->data[1] << 8));
            s->mode_display = (int8_t)f->data[2];
            s->actual_velocity = (int32_t)le32(f->data + 3);
            s->statusword_ms = s->pdo_ms = s->last_response_ms = clock_ms;
            s->feedback_valid |= 128U | 256U;
            s->pdo_rx_count++;
            s->rx_count++;
            if (s->state == MOTOR_ENABLED &&
                ((s->statusword & 0x6FU) != 0x27U || s->mode_display != 3))
                fail(i, MOTOR_ERR_DRIVE);
            else if (s->state == MOTOR_STOPPING &&
                     ((s->statusword & 0x6FU) == 0x07U ||
                      (s->statusword & 0x6FU) == 0x40U)) s->state = MOTOR_STOPPED;
            if ((s->statusword & 8U) && s->state != MOTOR_ERROR) fail(i, MOTOR_ERR_DRIVE);
        } else if (f->id == 0x700U + s->node_id && f->len == 1U) {
            s->heartbeat_seen = 1;
            s->heartbeat_ms = clock_ms;
            s->heartbeat_state = f->data[0];
            if (s->state == MOTOR_ENABLED && f->data[0] != 5U)
                fail(i, MOTOR_ERR_HEARTBEAT);
        } else if (f->id == 0x80U + s->node_id && f->len == 8U) {
            s->emcy_code = (uint16_t)(f->data[0] | (f->data[1] << 8));
            if (s->emcy_code != 0U) fail(i, MOTOR_ERR_DRIVE);
        } else if (f->id == 0x580U + s->node_id && f->len == 8U && n->pending) {
            uint16_t object = (uint16_t)(f->data[1] | (f->data[2] << 8));
            if (object != n->object || f->data[3] != n->subindex) continue;
            if (f->data[0] == 0x80U) {
                uint32_t abort = le32(&f->data[4]);
                /* Some EPOS4 firmware omits 1029:1 (NMT-state reaction).
                 * This does not replace the required 6007 quick-stop reaction
                 * or 1016 consumer watchdog. Skip only an absent 1029:1;
                 * mapping/watchdog/reaction failures must still latch errors. */
                if (n->configuring == 2U && !n->read && object == 0x1029U &&
                    n->subindex == 1U &&
                    (abort == 0x06020000U || abort == 0x06090011U)) {
                    s->diagnostic_abort = abort;
                    s->diagnostic_object = object;
                    s->diagnostic_subindex = n->subindex;
                    n->pending = 0;
                    n->config_step++;
                    n->next_ms = clock_ms + 1U;
                    continue;
                }
                uint32_t bit = feedback_bit(object);
                if (n->read && (bit & 0x1E0FU) &&
                    (abort == 0x06020000U || abort == 0x06090011U)) {
                    /* Unsupported optional telemetry must not break motion.
                     * Record N/A; never pretend that an unread value is zero. */
                    s->feedback_unsupported |= bit;
                    s->diagnostic_abort = abort;
                    s->diagnostic_object = object;
                    s->diagnostic_subindex = n->subindex;
                    n->pending = 0;
                    n->next_ms = clock_ms + 5U;
                    continue;
                }
                s->sdo_abort = le32(&f->data[4]);
                fail(i, MOTOR_ERR_SDO_ABORT);
                continue;
            }
            uint8_t expected = n->read ?
                (n->size == 1U ? 0x4FU : n->size == 2U ? 0x4BU : 0x43U) : 0x60U;
            if (f->data[0] != expected) { fail(i, MOTOR_ERR_PROTOCOL); continue; }
            uint32_t value = le32(&f->data[4]);
            n->pending = 0;
            s->rx_count++;
            s->last_response_ms = clock_ms;
            if (n->configuring == 2U) {
                n->config_step++;
                n->next_ms = clock_ms + 1U;
                continue;
            }
            uint32_t previous_valid = s->feedback_valid;
            if (n->read) s->feedback_valid |= feedback_bit(object);
            if (n->read && object == 0x301AU) {
                uint16_t pattern = (uint16_t)(value & 7U);
                if (pattern == 0U || pattern == 7U) s->hall_invalid++;
                else if ((previous_valid & 512U) && s->hall_pattern != 0U &&
                         s->hall_pattern != 7U && pattern != s->hall_pattern) {
                    uint16_t difference = pattern ^ s->hall_pattern;
                    if (difference & (difference - 1U)) s->hall_skipped++;
                    s->hall_edges++;
                }
                s->hall_pattern = pattern;
                s->hall_ms = clock_ms;
            }
            if (n->read && object == 0x3010U) s->encoder_ppr = value;
            if (n->read && object == 0x3001U) s->configured_pole_pairs = value;
            if (n->read && object == 0x60A8U) s->position_unit = value;
            if (n->read && object == 0x6064U) {
                s->position_delta = s->position_ms ?
                    (int32_t)(value - (uint32_t)s->position_actual) : 0;
                s->position_actual = (int32_t)value;
                s->position_ms = clock_ms;
            }
            if (n->read && object == 0x30D0U) s->current_demand_ma = (int32_t)value;
            if (n->read && object == 0x30D1U) {
                s->current_actual_ma = (int32_t)value;
                s->current_ms = clock_ms;
            }
            if (n->read && object == 0x6077U) {
                s->torque_actual_permille = (int16_t)value;
                s->torque_ms = clock_ms;
            }
            if (n->read && object == 0x6041U) {
                s->statusword = (uint16_t)value;
                s->statusword_ms = clock_ms;
            }
            if (n->read && object == 0x6061U) s->mode_display = (int8_t)value;
            if (n->read && object == 0x606CU) s->actual_velocity = (int32_t)value;
            if (n->read && object == 0x60FFU) {
                s->drive_target_velocity = (int32_t)value;
                s->drive_target_ms = clock_ms;
            }
            if (n->read && object == 0x606BU) {
                s->demand_velocity = (int32_t)value;
                s->demand_ms = clock_ms;
            }
            if (n->read && object == 0x6040U) {
                s->drive_controlword = (uint16_t)value;
                s->controlword_ms = clock_ms;
            }
            if ((s->statusword & 8U) != 0U) { fail(i, MOTOR_ERR_DRIVE); continue; }
            if (s->state == MOTOR_OFFLINE) s->state = MOTOR_MONITOR;
            else if (s->state == MOTOR_ENABLING) {
                uint8_t advance = 1;
                if (n->phase == 2U) advance = s->mode_display == 3;
                if (n->phase == 4U) advance = (s->statusword & 0x6FU) == 0x21U;
                if (n->phase == 6U) advance = (s->statusword & 0x6FU) == 0x23U;
                if (n->phase == 8U) advance = (s->statusword & 0x6FU) == 0x27U;
                if (advance) { n->phase++; n->phase_ms = clock_ms; }
                if (n->phase == 9U) {
                    s->state = MOTOR_ENABLED;
                    n->command_ms = clock_ms;
                    n->poll = 0;
                    n->pdo_running = 1;
                    n->pdo_started_ms = clock_ms;
                    s->pdo_rx_count = 0;
                    cycle_due_ms = clock_ms;
                }
            } else if (s->state == MOTOR_STOPPING) {
                if (n->phase == 0U) n->phase = 1;
                else if ((s->statusword & 0x6FU) == 0x07U ||
                         (s->statusword & 0x6FU) == 0x40U)
                    s->state = MOTOR_STOPPED;
            } else if (s->state == MOTOR_ENABLED && n->read && object == 0x6041U &&
                       (s->statusword & 0x6FU) != 0x27U) fail(i, MOTOR_ERR_DRIVE);
            n->next_ms = clock_ms + (s->state == MOTOR_ENABLED ? 20U : 5U);
        }
    }
}

HAL_StatusTypeDef Motor_Init(void)
{
    CAN_FilterTypeDef filter = {0};
    cancel_cycle();
    sync_seen = 0;
    master_heartbeat_ms = clock_ms;
    memset(nodes, 0, sizeof(nodes));
    memset((void *)motor_status, 0, sizeof(motor_status));
    uint32_t irq_mask = __get_PRIMASK();
    __disable_irq();
    rx_read = rx_write = 0;
    __set_PRIMASK(irq_mask);
    overflow_seen = motor_diagnostics.rx_overflow;
    for (uint32_t i = 0; i < MOTOR_CAPACITY; i++) motor_status[i].node_id = (uint8_t)(i + 1U);
    /* Accept standard data frames only; keep unrelated IDs for diagnostics. */
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterMaskIdLow = 6U;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK) return HAL_ERROR;
    if (HAL_CAN_Start(&hcan1) != HAL_OK) return HAL_ERROR;
    return HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if (hcan->Instance != CAN1) return;
    /* Hardware FIFO depth is three; bound ISR work even on a saturated bus. */
    for (uint32_t count = 0; count < 3U && HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0); count++) {
        CAN_RxHeaderTypeDef header;
        uint8_t data[8];
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &header, data) != HAL_OK) break;
        motor_diagnostics.rx_count++;
        if (header.IDE != CAN_ID_STD || header.RTR != CAN_RTR_DATA || header.DLC > 8U) continue;
        uint32_t next = (rx_write + 1U) % RX_SIZE;
        if (next == rx_read) { motor_diagnostics.rx_overflow++; continue; }
        rx_queue[rx_write].id = (uint16_t)header.StdId;
        rx_queue[rx_write].len = (uint8_t)header.DLC;
        memcpy(rx_queue[rx_write].data, data, 8);
        __DMB();
        rx_write = next;
    }
}

void Motor_Tick1ms(void) { clock_ms++; }
uint32_t Motor_TimeMs(void) { return clock_ms; }

uint8_t Motor_Enable(uint32_t i)
{
    if (i >= MOTOR_COUNT || !power_on || nodes[i].pending ||
        clock_ms - motor_status[i].last_response_ms >= MOTOR_SDO_TIMEOUT_MS ||
        (motor_status[i].state != MOTOR_MONITOR && motor_status[i].state != MOTOR_STOPPED)) return 0;
    uint8_t start[2] = {motor_status[i].pdo_ready ? 1U : 0x80U, motor_status[i].node_id};
    if (send_frame(0, 2, start) != HAL_OK) return 0;
    cancel_cycle();
    nodes[i].pdo_running = 0;
    nodes[i].configuring = motor_status[i].pdo_ready ? 0U : 1U;
    nodes[i].config_step = 0;
    if (nodes[i].configuring) motor_status[i].heartbeat_seen = 0;
    motor_status[i].target_velocity = 0;
    motor_status[i].state = MOTOR_ENABLING;
    nodes[i].phase = 0;
    nodes[i].phase_ms = clock_ms;
    nodes[i].next_ms = clock_ms;
    return 1;
}

uint8_t Motor_SetVelocity(uint32_t i, int32_t velocity)
{
    if (i >= MOTOR_COUNT || motor_status[i].state != MOTOR_ENABLED ||
        !motor_status[i].pdo_rx_count ||
        clock_ms - motor_status[i].pdo_ms >= MOTOR_PDO_TIMEOUT_MS) return 0;
    motor_status[i].target_velocity = velocity;
    nodes[i].command_ms = clock_ms;
    return 1;
}

uint8_t Motor_QuickStop(uint32_t i)
{
    if (i >= MOTOR_COUNT || motor_status[i].state == MOTOR_ERROR ||
        motor_status[i].state == MOTOR_OFFLINE) return 0;
    /* Finish an outstanding SDO before issuing another on this node. */
    motor_status[i].target_velocity = 0;
    nodes[i].stop_requested = 1;
    cancel_cycle();
    return 1;
}

uint8_t Motor_SetPower(uint8_t on)
{
    /* Power cycling is permitted only for idle/error states, never as a
     * substitute for a controlled stop. Use Reset drivers to clear errors. */
    if (reset_pending) return 0;
    for (uint32_t i = 0; i < MOTOR_COUNT; i++)
        if (motor_status[i].state == MOTOR_ENABLING || motor_status[i].state == MOTOR_ENABLED ||
            motor_status[i].state == MOTOR_STOPPING) return 0;
    HAL_GPIO_WritePin(OUTPUT_PWR_EPOS4_GPIO_Port, OUTPUT_PWR_EPOS4_Pin,
                      on ? GPIO_PIN_SET : GPIO_PIN_RESET);
    power_on = on != 0U;
    cancel_cycle();
    power_on_ms = clock_ms;
    for (uint32_t i = 0; i < MOTOR_COUNT; i++) {
        nodes[i].pending = 0;
        nodes[i].pdo_running = nodes[i].configuring = 0;
        motor_status[i].pdo_ready = 0;
        nodes[i].next_ms = clock_ms + 1500U;
        motor_status[i].heartbeat_seen = 0;
        motor_status[i].startup_retries = 0;
        if (motor_status[i].state != MOTOR_ERROR) motor_status[i].state = MOTOR_OFFLINE;
    }
    return 1;
}

uint8_t Motor_KeepAlive(uint32_t i)
{
    if (i >= MOTOR_COUNT) return 0;
    if (motor_status[i].state == MOTOR_ENABLING) return 1;
    if (motor_status[i].state != MOTOR_ENABLED) return 0;
    nodes[i].command_ms = clock_ms;
    return 1;
}

uint8_t Motor_ResetDrivers(void)
{
    if (reset_pending) return 0;
    for (uint32_t i = 0; i < MOTOR_COUNT; i++) {
        MotorState state = motor_status[i].state;
        if (state == MOTOR_ENABLING || state == MOTOR_ENABLED ||
            state == MOTOR_STOPPING || state == MOTOR_RESETTING) return 0;
    }
    /* This is an explicit shared power cycle, not an automatic fault retry.
     * Clear old requests before power can return. No enable is scheduled. */
    HAL_GPIO_WritePin(OUTPUT_PWR_EPOS4_GPIO_Port, OUTPUT_PWR_EPOS4_Pin, GPIO_PIN_RESET);
    power_on = 0;
    cancel_cycle();
    reset_pending = 1;
    reset_due_ms = clock_ms + 1000U;
    for (uint32_t i = 0; i < MOTOR_COUNT; i++) {
        memset(&nodes[i], 0, sizeof(nodes[i]));
        motor_status[i].target_velocity = 0;
        motor_status[i].state = MOTOR_RESETTING;
    }
    return 1;
}

void Motor_Process(void)
{
    uint32_t now = clock_ms;
    if (now == process_ms) return;
    if (now - process_ms > 1U) motor_diagnostics.scheduler_late++;
    process_ms = now;
    if (reset_pending) {
        if ((int32_t)(now - reset_due_ms) < 0) return;
        /* DeInit/Init resets bus-off/error state, stale mailboxes and RX FIFO.
         * The configured bitrate/pins are retained in hcan1.Init/MSP. */
        HAL_StatusTypeDef result = HAL_CAN_DeInit(&hcan1);
        if (result == HAL_OK) result = HAL_CAN_Init(&hcan1);
        if (result == HAL_OK) result = Motor_Init();
        reset_pending = 0;
        if (result != HAL_OK) {
            for (uint32_t i = 0; i < MOTOR_COUNT; i++) {
                motor_status[i].state = MOTOR_ERROR;
                motor_status[i].error = MOTOR_ERR_CAN;
                motor_status[i].failure_ms = now;
            }
            return; /* power stays off on recovery failure */
        }
        (void)Motor_SetPower(1);
        return;
    }
    /* Do not replay missed periods or burst obsolete commands. */
    motor_diagnostics.can_error = HAL_CAN_GetError(&hcan1);
    motor_diagnostics.can_esr = hcan1.Instance->ESR;
    uint8_t bus_error = __HAL_CAN_GET_FLAG(&hcan1, CAN_FLAG_BOF) ||
        __HAL_CAN_GET_FLAG(&hcan1, CAN_FLAG_EPV) ||
        (motor_diagnostics.can_error & HAL_CAN_ERROR_RX_FOV0);
    if (bus_error || motor_diagnostics.rx_overflow != overflow_seen) {
        overflow_seen = motor_diagnostics.rx_overflow;
        for (uint32_t i = 0; i < MOTOR_COUNT; i++)
            if (motor_status[i].state != MOTOR_ERROR)
                fail(i, bus_error ? MOTOR_ERR_CAN : MOTOR_ERR_RX_OVERFLOW);
    }
    for (uint32_t count = 0; count < RX_SIZE && rx_read != rx_write; count++) {
        __DMB();
        Frame frame = rx_queue[rx_read];
        __DMB();
        rx_read = (rx_read + 1U) % RX_SIZE;
        receive(&frame);
    }
    /* Input fields belong exclusively to the debugger. In particular, do not
     * clear command/request_id: J-Link may verify RAM writes after execution
     * has already consumed the command. A separate acknowledgement avoids that
     * write/readback race and lets the same command be submitted repeatedly. */
    uint32_t request_id = motor_debug.request_id;
    if (request_id != motor_debug.completed_id) {
        __DMB();
        uint32_t command = motor_debug.command;
        uint32_t i = motor_debug.motor_index;
        int32_t velocity = motor_debug.velocity;
        uint32_t accepted = 0;
        if (command == 1U) accepted = Motor_Enable(i);
        if (command == 2U) accepted = Motor_SetVelocity(i, velocity);
        if (command == 3U) accepted = Motor_QuickStop(i);
        if (command == 4U || command == 5U) {
            accepted = Motor_SetPower(command == 4U);
        }
        motor_debug.accepted = accepted;
        __DMB();
        motor_debug.completed_id = request_id;
    }
    for (uint32_t i = 0; i < MOTOR_COUNT; i++) {
        volatile MotorStatus *s = &motor_status[i];
        Node *n = &nodes[i];
        if (!power_on || s->state == MOTOR_ERROR) continue;
        /* Safety supervision is independent of outstanding diagnostic SDOs. */
        if (s->state == MOTOR_ENABLED && now - n->command_ms >= MOTOR_COMMAND_LEASE_MS) {
            fail(i, MOTOR_ERR_LEASE); continue;
        }
        if (n->pdo_running && now - (s->pdo_rx_count ? s->pdo_ms : n->pdo_started_ms)
                >= MOTOR_PDO_TIMEOUT_MS) { fail(i, MOTOR_ERR_PDO_TIMEOUT); continue; }
        if (s->heartbeat_seen && now - s->heartbeat_ms >= 1500U) {
            fail(i, MOTOR_ERR_HEARTBEAT); continue;
        }
        if (n->pending) {
            if (now - n->sent_ms >= MOTOR_SDO_TIMEOUT_MS) {
                if (s->state == MOTOR_OFFLINE && n->read && n->object == 0x6041U &&
                    now - power_on_ms < MOTOR_STARTUP_TIMEOUT_MS) {
                    /* Only the initial read-only probe may be repeated. A late
                     * response to an identical read is harmless; never repeat
                     * enable/speed writes or recover motion automatically. */
                    n->pending = 0;
                    n->next_ms = now + 250U;
                    s->startup_retries++;
                } else fail(i, MOTOR_ERR_SDO_TIMEOUT);
            }
            continue;
        }
        if (s->state == MOTOR_OFFLINE) {
            if (now - power_on_ms >= MOTOR_STARTUP_TIMEOUT_MS) {
                /* Report the initial probe even between retry attempts. */
                fail(i, MOTOR_ERR_SDO_TIMEOUT);
                s->failed_object = 0x6041U;
                continue;
            }
            /* Wait for a usable heartbeat when available. Drives without a
             * heartbeat producer are probed after a fixed 3 s grace period. */
            if (now - power_on_ms < 3000U &&
                (!s->heartbeat_seen || (s->heartbeat_state != 0x7FU && s->heartbeat_state != 5U)))
                continue;
        }
        if (n->stop_requested) {
            n->stop_requested = 0;
            s->state = MOTOR_STOPPING;
            n->phase = 0;
            n->phase_ms = now;
            n->next_ms = now;
            n->configuring = 0;
        }
        if ((s->state == MOTOR_ENABLING || s->state == MOTOR_STOPPING) &&
            now - n->phase_ms >= MOTOR_STATE_TIMEOUT_MS) { fail(i, MOTOR_ERR_STATE_TIMEOUT); continue; }
        if ((int32_t)(now - n->next_ms) < 0) continue;
        if (n->configuring) { configure_pdo(i); continue; }
        if (s->state == MOTOR_ENABLING) {
            switch (n->phase) {
            case 0: request(i, 0x60FF, 4, 0, 0); break;
            case 1: request(i, 0x6060, 1, 3, 0); break;
            case 2: request(i, 0x6061, 1, 0, 1); break;
            case 3: request(i, 0x6040, 2, 6, 0); break;
            case 4: request(i, 0x6041, 2, 0, 1); break;
            case 5: request(i, 0x6040, 2, 7, 0); break;
            case 6: request(i, 0x6041, 2, 0, 1); break;
            case 7: request(i, 0x6040, 2, 15, 0); break;
            case 8: request(i, 0x6041, 2, 0, 1); break;
            default: fail(i, MOTOR_ERR_PROTOCOL); break;
            }
        } else if (s->state == MOTOR_STOPPING) {
            request(i, n->phase == 0U ? 0x6040 : 0x6041, 2,
                    n->phase == 0U ? 2U : 0U, n->phase != 0U);
        } else if (s->state == MOTOR_ENABLED) {
            /* Motion and primary feedback use PDO only; SDO is slow telemetry. */
            if (n->poll == 0U) request(i, 0x60FF, 4, 0, 1);
            else if (n->poll == 1U) request(i, 0x606B, 4, 0, 1);
            else if (n->poll == 2U) request(i, 0x6040, 2, 0, 1);
            else {
                static const uint16_t objects[4] = {0x6064, 0x30D0, 0x30D1, 0x6077};
                uint32_t slot = n->poll - 3U;
                if (s->feedback_unsupported & (1U << slot)) {
                    n->poll = (uint8_t)((n->poll + 1U) % 7U);
                    continue;
                }
                request_sub(i, objects[slot], slot == 2U ? 2U : 0U,
                            slot == 3U ? 2U : 4U, 0, 1);
            }
            if (n->pending) {
                n->poll = (uint8_t)((n->poll + 1U) % 7U);
                n->next_ms = now + 20U;
            }
        } else if (s->state == MOTOR_MONITOR &&
                   (s->statusword & 0x6FU) == 0x40U &&
                   now - s->statusword_ms < MOTOR_SDO_TIMEOUT_MS) {
            /* Passive hand-turn acquisition: no output-enabling writes.
             * Read position before/after Hall, and continue confirming disabled. */
            static const uint16_t objects[7] = {0x6041, 0x3010, 0x3001, 0x60A8, 0x6064, 0x301A, 0x6064};
            uint8_t slot = n->poll % 7U;
            uint16_t object = objects[slot];
            if (!(s->feedback_unsupported & feedback_bit(object)))
                request_sub(i, object, slot == 1U ? 1U : slot == 2U ? 3U : slot == 5U ? 2U : 0U,
                            slot == 2U ? 1U : slot == 0U || slot == 5U ? 2U : 4U, 0, 1);
            if (n->pending || (s->feedback_unsupported & feedback_bit(object)))
                n->poll = (uint8_t)((slot + 1U) % 7U);
        } else {
            n->poll = 0;
            request(i, 0x6041, 2, 0, 1);
        }
    }
    process_pdo_cycle(now);
    /* Heartbeat production follows successful cyclic service. If the MCU
     * freezes or PDO service fails, the drive's 300 ms consumer stops it. */
    if (power_on && sync_seen && now - motor_diagnostics.sync_ms < 30U &&
        now - master_heartbeat_ms >= MOTOR_MASTER_HEARTBEAT_MS) {
        uint8_t state = 5;
        if (send_frame((uint16_t)(0x700U + MOTOR_MASTER_NODE_ID), 1, &state) == HAL_OK)
            master_heartbeat_ms = now;
    }
}
