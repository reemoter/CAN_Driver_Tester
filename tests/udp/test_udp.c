#include <assert.h>
#include <stdlib.h>
#include <stdio.h>
#include "control_udp.c"
static uint32_t now=100, starts, cancels, quick_stops;
static uint8_t busy, capture[192];
static uint16_t capture_length;
volatile MotorStatus motor_status[MOTOR_CAPACITY];
volatile MotorDiagnostics motor_diagnostics;
volatile BoardIdentity board_identity;
volatile LimitSensorStatus limit_sensors[2];
volatile LimitSensorDiagnostics limit_sensor_diagnostics;
volatile LiftStatus lift_status;
uint32_t AppTime_NowMs(void) { return now; }
uint32_t Motor_TimeMs(void) { return now; }
uint8_t Motor_Enable(uint32_t i) { (void)i; return 1; }
uint8_t Motor_SetVelocity(uint32_t i,int32_t v) { (void)i; (void)v; return 1; }
uint8_t Motor_QuickStop(uint32_t i) { (void)i; quick_stops++; return 1; }
uint8_t Motor_SetPower(uint8_t on) { (void)on; return 1; }
uint8_t Motor_KeepAlive(uint32_t i) { (void)i; return 1; }
uint8_t Motor_ResetDrivers(void) { return 1; }
uint8_t Lift_Busy(void) { return busy; }
uint8_t Lift_Start(uint32_t i,uint8_t up,int32_t v)
{ assert(!i && v==150); starts++; busy=1; lift_status.state=up?LIFT_UP:LIFT_DOWN; return 0; }
void Lift_Cancel(void) { cancels++; busy=0; }
static struct udp_pcb fake_pcb;
struct udp_pcb *udp_new(void) { return &fake_pcb; }
err_t udp_bind(struct udp_pcb *p,const ip_addr_t *a,u16_t port)
{ (void)p;(void)a;assert(port==2000);return ERR_OK; }
void udp_remove(struct udp_pcb *p) { (void)p; }
void udp_recv(struct udp_pcb *p,udp_recv_fn fn,void *arg) { (void)p;(void)fn;(void)arg; }
struct pbuf *pbuf_alloc(int layer,u16_t len,int type)
{ (void)layer;(void)type;struct pbuf *p=calloc(1,sizeof(*p));assert(p);p->tot_len=len;return p; }
err_t pbuf_take(struct pbuf *p,const void *data,u16_t len)
{ memcpy(p->payload,data,len);return ERR_OK; }
u16_t pbuf_copy_partial(const struct pbuf *p,void *data,u16_t len,u16_t off)
{ memcpy(data,p->payload+off,len);return len; }
void pbuf_free(struct pbuf *p) { free(p); }
err_t udp_sendto(struct udp_pcb *pcb,struct pbuf *p,const ip_addr_t *a,u16_t port)
{ (void)pcb;(void)a;(void)port;capture_length=p->tot_len;memcpy(capture,p->payload,p->tot_len);return ERR_OK; }
static void command(uint32_t seq,uint8_t cmd,int32_t v,uint32_t addr)
{
    uint8_t request[16]={0};memcpy(request,"ABS1",4);put32(request+4,seq);
    request[8]=cmd;put32(request+12,(uint32_t)v);
    struct pbuf *p=pbuf_alloc(0,16,0);pbuf_take(p,request,16);
    ip_addr_t ip={addr};receive(NULL,server,p,&ip,1234);
}
int main(void)
{
    assert(ControlUdp_Init()==ERR_OK);
    motor_status[0].state=MOTOR_ENABLED;
    command(1,10,150,1);assert(starts==1 && capture[10]==0);
    command(1,10,150,1);assert(starts==1 && capture[10]==0); /* retry cached */
    command(1,11,150,1);assert(starts==1 && capture[10]==7);
    command(2,11,150,2);assert(starts==1 && capture[10]==8); /* other owner */
    command(2,2,150,1);assert(capture[10]==4 && busy);
    command(3,2,0,1);assert(!capture[10] && !busy && cancels==1);
    command(4,11,150,1);assert(starts==2 && busy);
    command(5,3,0,1);assert(quick_stops==1 && cancels==2 && !busy);
    limit_sensors[0].active=1;limit_sensors[0].valid=limit_sensors[1].valid=1;
    command(0,12,0,2);assert(capture_length==192 && !capture[10]);
    assert(capture[171]==1 && capture[172]==3 && capture[174]==1);
    command(0,9,0,2);assert(capture_length==168); /* existing diagnostics unchanged */
    command(0,8,0,2);assert(capture_length==128);
    puts("UDP lift: ownership, duplicate/replay, conflicts, stop and telemetry passed");
    return 0;
}
