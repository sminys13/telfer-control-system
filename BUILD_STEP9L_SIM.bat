@echo off
setlocal
cd /d "%~dp0"
set "PIOEXE=%USERPROFILE%\.platformio\penv\Scripts\platformio.exe"
if not exist "%PIOEXE%" set "PIOEXE=pio"
"%PIOEXE%" run -e mega_v6_web_simulation
if errorlevel 1 exit /b 1
echo STEP9L SIM BUILD SUCCESS. Physical RS485 and sensor SPI disabled.
pause
