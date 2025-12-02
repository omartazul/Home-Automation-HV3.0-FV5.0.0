@echo off
REM Batch wrapper to always run compute_fan_delays.py using system python
SET PY=python
where python >nul 2>nul
IF %ERRORLEVEL% NEQ 0 SET PY="C:\\Program Files\\Python312\\python.exe"
PUSHD "%~dp0"
REM By default, generate all outputs into the script folder
%PY% "compute_fan_delays.py" --generate-all
POPD
