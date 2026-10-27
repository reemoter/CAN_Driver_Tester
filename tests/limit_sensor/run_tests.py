from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[2]
tests = Path(__file__).resolve().parent
vcvars = Path(r'C:\Program Files\Microsoft Visual Studio\2022\Community\VC\Auxiliary\Build\vcvars64.bat')
with tempfile.TemporaryDirectory(prefix='abs_limits_') as directory:
    command = (f'call "{vcvars}" >nul && cl /nologo /std:c11 /W3 /WX '
               f'/I"{tests}" /I"{root / "Core/Src"}" /I"{root / "Core/Inc"}" '
               f'"{tests / "test_limit_sensor.c"}" /Fe:test.exe /Fo:test.obj && test.exe')
    subprocess.run(command, shell=True, check=True, cwd=directory)
