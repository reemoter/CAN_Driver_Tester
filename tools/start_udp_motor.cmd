@echo off
setlocal
set "ABS_PYTHON=%USERPROFILE%\.cache\codex-runtimes\codex-primary-runtime\dependencies\python\python.exe"
if exist "%ABS_PYTHON%" (
    "%ABS_PYTHON%" "%~dp0udp_motor.py" %*
) else (
    py -3 "%~dp0udp_motor.py" %*
)
if errorlevel 1 pause
