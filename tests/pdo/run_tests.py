"""Windows host tests: production motor.c, mocked STM32 HAL, 1 and 2 nodes."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
tests = Path(__file__).resolve().parent
vcvars = Path(r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat')
with tempfile.TemporaryDirectory(prefix='abs_pdo_') as directory:
    work = Path(directory)
    for count in (1, 2):
        header = (root / 'Core/Inc/motor.h').read_text()
        header = header.replace('#include "main.h"', '#include "fake_hal.h"')
        header = header.replace('#define MOTOR_COUNT 1U', f'#define MOTOR_COUNT {count}U')
        (work / 'motor.h').write_text(header)
        command = (f'call "{vcvars}" >nul && cl /nologo /std:c11 /W3 /WX '
                   f'/I"{work}" /I"{tests}" /I"{root / "Core/Src"}" '
                   f'"{tests / "test_motor_pdo.c"}" /Fe:"{work / "test.exe"}" '
                   f'/Fo:"{work / "test.obj"}" && "{work / "test.exe"}"')
        print(f'Testing {count} node(s)', flush=True)
        subprocess.run(command, shell=True, check=True, cwd=work)
