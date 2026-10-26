/* Includes production code so the mocked HAL exercises its real state machine. */
#include <assert.h>
#include <stdio.h>
#include <stdint.h>
static uint32_t clock_ms;
#include "motor.c"
uint32_t AppTime_NowMs(void) { return clock_ms; }
CAN_TypeDef fake_can;
CAN_HandleTypeDef hcan1 = { &fake_can };
static Frame sent[2048];
static uint32_t sent_count, free_level = 3, abort_count;
uint32_t HAL_CAN_GetTxMailboxesFreeLevel(CAN_HandleTypeDef *h) { (void)h; return free_level; }
HAL_StatusTypeDef HAL_CAN_AddTxMessage(CAN_HandleTypeDef *h, CAN_TxHeaderTypeDef *header, uint8_t *data, uint32_t *mailbox)
{
    (void)h;
    assert(sent_count < 2048);
    sent[sent_count].id = (uint16_t)header->StdId;
    sent[sent_count].len = (uint8_t)header->DLC;
    memcpy(sent[sent_count++].data, data, header->DLC);
    *mailbox = CAN_TX_MAILBOX0;
    fake_can.TSR = 0; /* enqueue is not transmission completion */
    return HAL_OK;
}
HAL_StatusTypeDef HAL_CAN_AbortTxRequest(CAN_HandleTypeDef *h, uint32_t mailbox)
{ (void)h; (void)mailbox; abort_count++; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_ConfigFilter(CAN_HandleTypeDef *h, CAN_FilterTypeDef *filter)
{ (void)h; (void)filter; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_Start(CAN_HandleTypeDef *h) { (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_ActivateNotification(CAN_HandleTypeDef *h, uint32_t flags)
{ (void)h; (void)flags; return HAL_OK; }
uint32_t HAL_CAN_GetRxFifoFillLevel(CAN_HandleTypeDef *h, uint32_t fifo)
{ (void)h; (void)fifo; return 0; }
HAL_StatusTypeDef HAL_CAN_GetRxMessage(CAN_HandleTypeDef *h, uint32_t fifo, CAN_RxHeaderTypeDef *header, uint8_t *data)
{ (void)h; (void)fifo; (void)header; (void)data; return HAL_ERROR; }
uint32_t HAL_CAN_GetError(CAN_HandleTypeDef *h) { (void)h; return 0; }
HAL_StatusTypeDef HAL_CAN_DeInit(CAN_HandleTypeDef *h) { (void)h; return HAL_OK; }
HAL_StatusTypeDef HAL_CAN_Init(CAN_HandleTypeDef *h) { (void)h; return HAL_OK; }
void HAL_GPIO_WritePin(uint32_t port, uint32_t pin, uint32_t value)
{ (void)port; (void)pin; (void)value; }

static void reset_test(void)
{
    memset((void *)&motor_diagnostics, 0, sizeof(motor_diagnostics));
    memset((void *)&motor_debug, 0, sizeof(motor_debug));
    cycle_wait = cycle_active = 0;
    assert(Motor_Init() == HAL_OK);
    power_on = 1;
    clock_ms = 100;
    process_ms = 99;
    cycle_due_ms = clock_ms;
    sent_count = abort_count = 0;
    free_level = 3;
    fake_can.TSR = 0;
}
static void arm(uint32_t i)
{
    motor_status[i].state = MOTOR_ENABLED;
    motor_status[i].pdo_ready = 1;
    motor_status[i].pdo_rx_count = 1;
    motor_status[i].pdo_ms = clock_ms;
    motor_status[i].statusword = 0x27;
    motor_status[i].target_velocity = -250;
    profile_reset(&nodes[i].profile, -250); /* already running fixture */
    nodes[i].pdo_running = 1;
    nodes[i].command_ms = clock_ms;
    nodes[i].next_ms = clock_ms + 100;
}
static void tx_ok(void) { fake_can.TSR = CAN_TSR_RQCP0 | CAN_TSR_TXOK0; }
static void tpdo(uint32_t i, uint16_t sw, int32_t velocity, uint8_t len)
{
    Frame frame = { (uint16_t)(0x180 + motor_status[i].node_id), len, {0} };
    frame.data[0] = (uint8_t)sw; frame.data[1] = (uint8_t)(sw >> 8);
    frame.data[2] = 3;
    for (uint32_t j = 0; j < 4; j++) frame.data[3+j] = (uint8_t)((uint32_t)velocity >> (j*8));
    receive(&frame);
}
static void ack(uint32_t i, uint8_t code)
{
    Frame frame = { (uint16_t)(0x580 + motor_status[i].node_id), 8, {0} };
    frame.data[0] = code;
    frame.data[1] = (uint8_t)nodes[i].object;
    frame.data[2] = (uint8_t)(nodes[i].object >> 8);
    frame.data[3] = nodes[i].subindex;
    if (code == 0x80) frame.data[6] = 2, frame.data[7] = 6; /* 06020000 */
    receive(&frame);
}
int main(void)
{
    MotorProfile p = {0};
    profile_goal(&p,150,100);
    uint32_t duration = p.duration;
    assert(duration >= 937 && duration <= 938);
    assert(profile_sample(&p,100) == 0);
    assert(profile_sample(&p,150) == 0); /* soft start */
    assert(profile_sample(&p,100+duration/2) >= 74);
    profile_goal(&p,150,100+duration/2);
    assert(p.started == 100); /* duplicate UDP target cannot restart ramp */
    assert(profile_sample(&p,100+duration) == 150 && !p.duration && p.acceleration == 0);
    profile_goal(&p,0,2000);
    assert(profile_sample(&p,2000+duration/2) >= 75);
    assert(profile_sample(&p,2000+duration) == 0);
    profile_goal(&p,-150,4000);
    int32_t midpoint = profile_sample(&p,4000+duration/2);
    assert(midpoint <= -74 && midpoint >= -75);
    double before = p.acceleration;
    profile_goal(&p,-100,4000+duration/2);
    assert(profile_sample(&p,4000+duration/2) == midpoint && p.acceleration == before);
    profile_reset(&p,0);
    profile_goal(&p,150,UINT32_MAX-249U);
    duration = p.duration;
    assert(profile_sample(&p,UINT32_MAX-249U+duration) == 150);
    profile_goal(&p,INT32_MIN,2000);
    assert(profile_sample(&p,2000+p.duration) == INT32_MIN);
    /* A rejected ramp setting must never permit enable. */
    for (uint32_t step = 26; step <= 28; step++) {
        reset_test();
        motor_status[0].state = MOTOR_ENABLING;
        nodes[0].configuring = 2;
        nodes[0].config_step = (uint8_t)step;
        configure_pdo(0);
        assert(nodes[0].pending);
        ack(0, 0x80);
        assert(motor_status[0].state == MOTOR_ERROR &&
               motor_status[0].error == MOTOR_ERR_SDO_ABORT &&
               !motor_status[0].pdo_ready);
    }
    reset_test();
    motor_status[0].state = MOTOR_MONITOR;
    motor_status[0].last_response_ms = clock_ms;
    assert(Motor_Enable(0));
    assert(sent[0].id == 0 && sent[0].data[0] == 0x80);
    configure_pdo(0); assert(sent_count == 1); /* fresh pre-op confirmation required */
    Frame hb = {0x701,1,{0x7f}};
    receive(&hb); configure_pdo(0);
    for (uint32_t j = 0; j < PDO_SETTINGS_COUNT; j++) {
        configure_pdo(0);
        assert(nodes[0].pending && nodes[0].config_step == j);
        assert(sent[sent_count-1].id == 0x601);
        ack(0, pdo_settings[j].object == 0x1029 ? 0x80 : 0x60);
        assert(nodes[0].config_step == j+1 && motor_status[0].state == MOTOR_ENABLING);
    }
    configure_pdo(0);
    assert(motor_status[0].pdo_ready && !nodes[0].configuring);
    /* Ramp writes are mandatory commissioning steps, before NMT operational. */
    assert(pdo_settings[26].object == 0x6083 && pdo_settings[26].size == 4 &&
           pdo_settings[26].value == MOTOR_PROFILE_ACCELERATION);
    assert(pdo_settings[27].object == 0x6084 && pdo_settings[27].size == 4 &&
           pdo_settings[27].value == MOTOR_PROFILE_DECELERATION);
    assert(pdo_settings[28].object == 0x6086 && pdo_settings[28].size == 2 &&
           pdo_settings[28].value == 0);
    for (uint32_t j = 0; j < PDO_SETTINGS_COUNT; j++)
        assert(pdo_settings[j].object != 0x6085);
    assert(motor_status[0].diagnostic_object == 0x1029 &&
           motor_status[0].diagnostic_abort == 0x06020000);
    assert(sent[sent_count-1].id == 0 && sent[sent_count-1].data[0] == 1);
    assert(pdo_settings[9].value == 0x60ff0020 && pdo_settings[10].value == 0x60400010);
    assert(pdo_settings[25].value == ((MOTOR_MASTER_NODE_ID << 16) | APP_CONSUMER_TIMEOUT_MS));
    assert(pdo_settings[21].value == APP_DRIVE_HEARTBEAT_MS);
    /* The original zero-speed enable sequence remains SDO-confirmed. */
    for (uint32_t phase = 0; phase <= 8; phase++) {
        clock_ms += 20;
        nodes[0].next_ms = clock_ms;
        process_ms = clock_ms - 1;
        Motor_Process();
        assert(nodes[0].pending && nodes[0].phase == phase);
        Frame answer = {0x581,8,{0}};
        answer.data[0] = nodes[0].read ? (nodes[0].size == 1 ? 0x4f : 0x4b) : 0x60;
        answer.data[1] = (uint8_t)nodes[0].object;
        answer.data[2] = (uint8_t)(nodes[0].object >> 8);
        answer.data[4] = phase == 2 ? 3 : phase == 4 ? 0x21 : phase == 6 ? 0x23 : 0x27;
        receive(&answer);
    }
    assert(motor_status[0].state == MOTOR_ENABLED && nodes[0].pdo_running);
    assert(motor_status[0].target_velocity == 0 && !Motor_SetVelocity(0,250));

    reset_test();
    motor_status[0].state = MOTOR_MONITOR;
    motor_status[0].last_response_ms = clock_ms;
    assert(Motor_Enable(0)); receive(&hb); configure_pdo(0); configure_pdo(0);
    ack(0,0x80); assert(motor_status[0].error == MOTOR_ERR_SDO_ABORT);
    assert(!motor_status[0].pdo_ready && !nodes[0].pdo_running);

    /* Required watchdog and stop reaction must never inherit the optional skip. */
    const uint16_t required[] = {0x6007, 0x1016};
    for (uint32_t j = 0; j < 2; j++) {
        reset_test();
        motor_status[0].state = MOTOR_ENABLING;
        nodes[0].configuring = 2; nodes[0].pending = 1;
        nodes[0].object = required[j]; nodes[0].subindex = 1;
        ack(0,0x80);
        assert(motor_status[0].error == MOTOR_ERR_SDO_ABORT);
    }

    reset_test(); arm(0);
#if MOTOR_COUNT == 2
    arm(1);
#endif
    process_pdo_cycle(clock_ms);
    assert(sent_count == 1 && sent[0].id == 0x201 && sent[0].len == 6);
    assert((int32_t)le32(sent[0].data) == -250 && sent[0].data[4] == 15);
    process_pdo_cycle(++clock_ms); assert(sent_count == 1); /* must wait for TXOK */
    uint8_t dummy[8] = {0};
    assert(send_frame(0x601,8,dummy) == HAL_BUSY); /* no mailbox flag reuse */
    tx_ok(); process_pdo_cycle(++clock_ms);
#if MOTOR_COUNT == 2
    assert(sent[1].id == 0x202); /* both RPDOs precede shared SYNC */
    tx_ok(); process_pdo_cycle(++clock_ms);
#endif
    assert(sent[sent_count-1].id == 0x80 && sent[sent_count-1].len == 0);
    assert(motor_diagnostics.sync_count == 0);
    tx_ok(); process_pdo_cycle(++clock_ms);
    assert(motor_diagnostics.sync_count == 1);
    tpdo(0,0x27,-249,7);
    assert(motor_status[0].actual_velocity == -249 && motor_status[0].pdo_rx_count == 2);
    tpdo(0,0x27,0,6); assert(motor_status[0].error == MOTOR_ERR_PROTOCOL);

    reset_test(); arm(0); process_pdo_cycle(clock_ms);
    clock_ms += 5; process_pdo_cycle(clock_ms);
    assert(motor_status[0].error == MOTOR_ERR_TX && abort_count);
    for (uint32_t j = 0; j < sent_count; j++) assert(sent[j].id != 0x80);

    reset_test(); arm(0); free_level = 0;
    process_pdo_cycle(clock_ms); clock_ms += 5; process_pdo_cycle(clock_ms);
    assert(motor_status[0].error == MOTOR_ERR_TX && !sent_count);

    reset_test(); arm(0); process_pdo_cycle(clock_ms);
    fake_can.TSR = CAN_TSR_RQCP0; /* completed without ACK/TXOK */
    process_pdo_cycle(++clock_ms);
    assert(motor_status[0].error == MOTOR_ERR_TX);

    reset_test(); arm(0); process_pdo_cycle(clock_ms);
    assert(Motor_QuickStop(0)); assert(abort_count);
    process_pdo_cycle(++clock_ms);
    assert(sent[sent_count-1].id == 0x201 && sent[sent_count-1].data[4] == 2);
    assert(le32(sent[sent_count-1].data) == 0);

    reset_test(); arm(0);
    nodes[0].pending = 1; nodes[0].object = 0x6064; nodes[0].sent_ms = clock_ms;
    motor_status[0].pdo_ms = clock_ms - APP_PDO_TIMEOUT_MS;
    Motor_Process(); assert(motor_status[0].error == MOTOR_ERR_PDO_TIMEOUT);

    reset_test(); arm(0);
    nodes[0].pending = 1; nodes[0].sent_ms = clock_ms;
    nodes[0].command_ms = clock_ms - APP_COMMAND_LEASE_MS;
    Motor_Process(); assert(motor_status[0].error == MOTOR_ERR_LEASE);

    reset_test(); arm(0);
    motor_status[0].pdo_rx_count = 0;
    nodes[0].pdo_started_ms = clock_ms - APP_PDO_TIMEOUT_MS;
    Motor_Process(); assert(motor_status[0].error == MOTOR_ERR_PDO_TIMEOUT);

    reset_test(); arm(0);
    motor_status[0].heartbeat_seen = 1;
    motor_status[0].heartbeat_ms = clock_ms - 1500;
    nodes[0].pending = 1; nodes[0].sent_ms = clock_ms;
    Motor_Process(); assert(motor_status[0].error == MOTOR_ERR_HEARTBEAT);

    reset_test(); arm(0);
    motor_status[0].pdo_rx_count = 0;
    assert(!Motor_SetVelocity(0,250));
    tpdo(0,0x27,0,7); assert(Motor_SetVelocity(0,250));
    tpdo(0,0x07,0,7); assert(motor_status[0].error == MOTOR_ERR_DRIVE);

    reset_test(); arm(0); clock_ms += 37;
    process_pdo_cycle(clock_ms);
    assert(sent_count == 1 && motor_diagnostics.cycle_missed == 1);

    reset_test(); arm(0);
    cycle_due_ms = clock_ms + 100; /* heartbeat is only permitted after a real SYNC */
    master_heartbeat_ms = clock_ms - APP_MASTER_HEARTBEAT_MS;
    Motor_Process(); assert(!sent_count);
    sync_seen = 1; motor_diagnostics.sync_ms = ++clock_ms;
    Motor_Process(); assert(sent_count == 1 && sent[0].id == 0x77f);
    clock_ms += 100; motor_status[0].pdo_ms = clock_ms;
    nodes[0].command_ms = clock_ms; nodes[0].next_ms = clock_ms + 100;
    sent_count = 0; cycle_due_ms = clock_ms + 100;
    Motor_Process(); assert(!sent_count); /* stale SYNC suppresses heartbeat */
    /* Reverse requests ramp to zero, then wait for fresh stable feedback. */
    reset_test(); arm(0); assert(Motor_SetVelocity(0,250));
    profile_reset(&nodes[0].profile,250);
    tpdo(0,0x27,250,7);
    assert(Motor_SetVelocity(0,-250));
    assert(motor_status[0].reversal_pending && motor_status[0].target_velocity == 0);
    assert(motor_status[0].requested_velocity == -250);
    process_pdo_cycle(clock_ms);
    assert(sent[sent_count-1].id == 0x201 && le32(sent[sent_count-1].data) == 250);
    tpdo(0,0x27,0,7); /* actual zero alone cannot release unfinished ramp */
    assert(motor_status[0].reversal_pending);
    clock_ms += nodes[0].profile.duration;
    nodes[0].command_ms = clock_ms; /* keepalive during normal ramp */
    tpdo(0,0x27,0,7);
    clock_ms += APP_REVERSAL_ZERO_HOLD_MS - 1;
    tpdo(0,0x27,0,7); assert(motor_status[0].reversal_pending);
    clock_ms++; tpdo(0,0x27,-1,7);
    assert(!motor_status[0].reversal_pending && motor_status[0].target_velocity == -250);
    process_pdo_cycle(clock_ms);
    assert((int32_t)le32(sent[sent_count-1].data) == 0); /* reverse soft start */
    tpdo(0,0x27,-250,7); assert(Motor_SetVelocity(0,150));
    assert(motor_status[0].reversal_pending && motor_status[0].target_velocity == 0);
    assert(Motor_SetVelocity(0,0));
    clock_ms += APP_REVERSAL_ZERO_HOLD_MS; tpdo(0,0x27,0,7);
    assert(!motor_status[0].reversal_pending && motor_status[0].target_velocity == 0);

    reset_test(); arm(0); assert(Motor_SetVelocity(0,250));
    assert(Motor_SetVelocity(0,-250)); tpdo(0,0x27,0,7);
    clock_ms += APP_REVERSAL_ZERO_HOLD_MS - 1;
    tpdo(0,0x27,MOTOR_REVERSAL_ZERO_BAND+1,7); /* movement restarts hold */
    clock_ms++; tpdo(0,0x27,0,7);
    assert(motor_status[0].reversal_pending);
    assert(Motor_QuickStop(0));
    clock_ms += APP_REVERSAL_ZERO_HOLD_MS; tpdo(0,0x27,0,7);
    assert(!motor_status[0].reversal_pending && !motor_status[0].requested_velocity);

    reset_test(); arm(0); assert(Motor_SetVelocity(0,250));
    assert(Motor_SetVelocity(0,-250));
    clock_ms += APP_REVERSAL_TIMEOUT_MS;
    motor_status[0].pdo_ms = nodes[0].command_ms = clock_ms;
    Motor_Process(); assert(motor_status[0].error == MOTOR_ERR_REVERSAL_TIMEOUT);
    assert(!motor_status[0].reversal_pending && !motor_status[0].requested_velocity);

    reset_test(); arm(0); assert(Motor_SetVelocity(0,250));
    assert(Motor_SetVelocity(0,-250)); tpdo(0,0x27,0,7);
    clock_ms += APP_COMMAND_LEASE_MS;
    tpdo(0,0x27,0,7); assert(motor_status[0].target_velocity == 0);
    Motor_Process(); assert(motor_status[0].error == MOTOR_ERR_LEASE);

    reset_test(); arm(0); assert(Motor_SetVelocity(0,INT32_MIN));
    process_pdo_cycle(clock_ms); assert((int32_t)le32(sent[sent_count-1].data) == -250);
    tpdo(0,0x27,-250,7); assert(Motor_SetVelocity(0,250));
    assert(motor_status[0].reversal_pending);
#if MOTOR_COUNT == 2
    arm(1); assert(Motor_SetVelocity(1,333));
    assert(motor_status[1].target_velocity == 333 && !motor_status[1].reversal_pending);
#endif
    puts("PDO and direction-reversal state-machine tests passed");
    return 0;
}
