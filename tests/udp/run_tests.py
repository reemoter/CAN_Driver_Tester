from pathlib import Path
import subprocess
import tempfile
root=Path(__file__).resolve().parents[2]
tests=Path(__file__).resolve().parent
vcvars=Path(r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat')
with tempfile.TemporaryDirectory(prefix='abs_udp_') as directory:
    work=Path(directory)
    (work/'lwip').mkdir()
    (work/'main.h').write_text('#include "fake_hal.h"\n')
    for name in ('motor.h','board_identity.h'):
        (work/name).write_bytes((root/'Core/Inc'/name).read_bytes())
    (work/'lwip/err.h').write_text('''
#ifndef FAKE_ERR_H
#define FAKE_ERR_H
typedef int err_t;
#define ERR_OK 0
#define ERR_MEM -1
#endif
''')
    (work/'lwip/pbuf.h').write_text('''
#ifndef FAKE_PBUF_H
#define FAKE_PBUF_H
#include <stdint.h>
#include "err.h"
typedef uint16_t u16_t;
struct pbuf { u16_t tot_len; uint8_t payload[192]; };
#define PBUF_TRANSPORT 0
#define PBUF_RAM 0
struct pbuf *pbuf_alloc(int layer,u16_t len,int type);
err_t pbuf_take(struct pbuf *p,const void *data,u16_t len);
u16_t pbuf_copy_partial(const struct pbuf *p,void *data,u16_t len,u16_t off);
void pbuf_free(struct pbuf *p);
#endif
''')
    (work/'lwip/udp.h').write_text('''
#include "pbuf.h"
typedef struct { uint32_t addr; } ip_addr_t;
struct udp_pcb { int dummy; };
typedef void (*udp_recv_fn)(void *,struct udp_pcb *,struct pbuf *,const ip_addr_t *,u16_t);
#define IP_ADDR_ANY ((const ip_addr_t *)0)
#define ip_addr_cmp(a,b) ((a)->addr==(b)->addr)
#define ip_addr_copy(a,b) ((a)=(b))
struct udp_pcb *udp_new(void);
err_t udp_bind(struct udp_pcb *p,const ip_addr_t *a,u16_t port);
void udp_remove(struct udp_pcb *p);
void udp_recv(struct udp_pcb *p,udp_recv_fn fn,void *arg);
err_t udp_sendto(struct udp_pcb *pcb,struct pbuf *p,const ip_addr_t *a,u16_t port);
''')
    command=(f'call "{vcvars}" >nul && cl /nologo /std:c11 /W3 /WX '
             f'/I"{work}" /I"{root / "tests/pdo"}" /I"{root / "Core/Src"}" /I"{root / "Core/Inc"}" '
             f'"{tests / "test_udp.c"}" /Fe:test.exe /Fo:test.obj && test.exe')
    subprocess.run(command,shell=True,check=True,cwd=work)
