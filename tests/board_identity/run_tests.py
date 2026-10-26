"""Compile/run fixed identity without HAL GPIO symbols; test all four IDs."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
vcvars = Path(r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat')
with tempfile.TemporaryDirectory(prefix='abs_identity_') as directory:
    work = Path(directory)
    for name in ('board_identity.h',):
        (work / name).write_bytes((root / 'Core/Inc' / name).read_bytes())
    (work / 'board_identity.c').write_bytes((root / 'Core/Src/board_identity.c').read_bytes())
    (work / 'main.h').write_text('#include <stdint.h>\ntypedef enum {HAL_OK, HAL_ERROR} HAL_StatusTypeDef;\n')
    (work / 'test.c').write_text('''
#include <assert.h>
#include <string.h>
#include "board_identity.c"
int main(void) {
    uint8_t ip[4], mask[4], gateway[4];
    assert(BoardIdentity_Init() == HAL_OK);
    BoardIdentity_CopyIPv4(ip, mask, gateway);
    assert(board_identity.id == BOARD_CONFIG_ID);
    assert(ip[0] == 192 && ip[1] == 168 && ip[2] == 0);
    assert(ip[3] == 22 + BOARD_CONFIG_ID);
    assert(mask[0] == 255 && mask[3] == 0 && gateway[3] == 1);
    assert(BoardIdentity_Mac()[5] == BOARD_CONFIG_ID);
    assert(!memcmp(BoardIdentity_Mac(), (const void *)board_identity.mac, 6));
    assert(board_identity.raw_selector == 255 && board_identity.samples == 0);
    assert(board_identity.initialized && board_identity.stable && !board_identity.error);
    assert(BoardIdentity_Init() == HAL_OK);
    return 0;
}
''')
    for board_id in (None, 1, 2, 3, 4, 0, 5):
        flag = '' if board_id is None else f'/DBOARD_CONFIG_ID={board_id}'
        command = (f'call "{vcvars}" >nul && cl /nologo /std:c11 /W3 /WX {flag} '
                   f'test.c /Fe:test.exe /Fo:test.obj')
        result = subprocess.run(command, shell=True, cwd=work, capture_output=True, text=True)
        if board_id in (0, 5):
            assert result.returncode != 0 and 'BOARD_CONFIG_ID must be' in result.stdout, result.stdout
        else:
            assert result.returncode == 0, result.stdout + result.stderr
            subprocess.run([str(work / 'test.exe')], cwd=work, check=True)
    print('Fixed board identity: default .24, IDs 1..4, invalid IDs rejected; no GPIO dependency')
