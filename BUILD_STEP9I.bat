@echo off
setlocal
cd /d "%~dp0"
set "PIOEXE=%USERPROFILE%\.platformio\penv\Scripts\platformio.exe"
if not exist "%PIOEXE%" set "PIOEXE=pio"

echo Building supported Step9I environments...
"%PIOEXE%" run -e mega_v6_sensor_core_fast
if errorlevel 1 goto :fail
"%PIOEXE%" run -e mega_v6_he200_waveshare_readonly
if errorlevel 1 goto :fail
"%PIOEXE%" run -e mega_v6_he200_field_service
if errorlevel 1 goto :fail

echo.
echo ALL STEP9I BUILDS: SUCCESS
pause
exit /b 0

:fail
echo.
echo BUILD FAILED. Do not use the FIELD SERVICE firmware on the object.
pause
exit /b 1
