@echo off
setlocal
cd /d "%~dp0"
set "PIOEXE=%USERPROFILE%\.platformio\penv\Scripts\platformio.exe"
if not exist "%PIOEXE%" set "PIOEXE=pio"
"%PIOEXE%" run -e mega_v6_sensor_core_fast -e mega_v6_he200_waveshare_readonly -e mega_v6_he200_field_service
if errorlevel 1 goto :fail
echo ALL STEP9J BUILDS: SUCCESS. Diagnostic lock remains active.
pause
exit /b 0
:fail
echo BUILD FAILED. Do not upload.
pause
exit /b 1
