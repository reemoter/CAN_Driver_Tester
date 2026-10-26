"""Exercise the production scheduler/clock with default and changed PHY periods."""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
tests = Path(__file__).resolve().parent
vcvars = Path(r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat')
with tempfile.TemporaryDirectory(prefix='abs_scheduler_') as directory:
    work = Path(directory)
    for period in (100, 37):
        timing = (root / 'Core/Inc/app_timing.h').read_text()
        timing = timing.replace('APP_ETH_LINK_CHECK_MS          100U', f'APP_ETH_LINK_CHECK_MS          {period}U')
        (work / 'app_timing.h').write_text(timing)
        command = (f'call "{vcvars}" >nul && cl /nologo /std:c11 /W3 /WX '
                   f'/I"{work}" /I"{tests}" /I"{root / "Core/Src"}" /I"{root / "Core/Inc"}" '
                   f'"{tests / "test_scheduler.c"}" /Fe:"{work / "test.exe"}" '
                   f'/Fo:"{work / "test.obj"}" && "{work / "test.exe"}"')
        print(f'Testing PHY period={period} ms', flush=True)
        subprocess.run(command, shell=True, check=True, cwd=work)
