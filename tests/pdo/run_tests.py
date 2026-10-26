"""Windows host tests: production motor.c, mocked STM32 HAL, 1 and 2 nodes."""
from pathlib import Path
import subprocess
import tempfile
import re

root = Path(__file__).resolve().parents[2]
tests = Path(__file__).resolve().parent
vcvars = Path(r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat')
with tempfile.TemporaryDirectory(prefix='abs_pdo_') as directory:
    work = Path(directory)
    for count, changed in ((1, False), (2, False), (2, True)):
        timing = (root / 'Core/Inc/app_timing.h').read_text()
        if changed:
            for name, value in {'APP_CAN_CYCLE_MS': 20, 'APP_MASTER_HEARTBEAT_MS': 80,
                                'APP_DRIVE_HEARTBEAT_MS': 80, 'APP_CONSUMER_TIMEOUT_MS': 240,
                                'APP_SDO_DIAGNOSTIC_GAP_MS': 30, 'APP_SDO_STATE_GAP_MS': 8}.items():
                timing = re.sub(rf'(#define {name}\s+)\d+U', rf'\g<1>{value}U', timing)
        (work / 'app_timing.h').write_text(timing)
        header = (root / 'Core/Inc/motor.h').read_text()
        header = header.replace('#include "main.h"', '#include "fake_hal.h"')
        header = header.replace('#define MOTOR_COUNT 1U', f'#define MOTOR_COUNT {count}U')
        (work / 'motor.h').write_text(header)
        command = (f'call "{vcvars}" >nul && cl /nologo /std:c11 /W3 /WX '
                   f'/I"{work}" /I"{tests}" /I"{root / "Core/Src"}" /I"{root / "Core/Inc"}" '
                   f'"{tests / "test_motor_pdo.c"}" /Fe:"{work / "test.exe"}" '
                   f'/Fo:"{work / "test.obj"}" && "{work / "test.exe"}"')
        print(f'Testing {count} node(s), changed timing={changed}', flush=True)
        subprocess.run(command, shell=True, check=True, cwd=work)
