@echo off
setlocal
cd /d "%~dp0"
set "PIOEXE=%USERPROFILE%\.platformio\penv\Scripts\platformio.exe"
if not exist "%PIOEXE%" set "PIOEXE=pio"
"%PIOEXE%" run -e mega_v6_he200_web_control
if errorlevel 1 exit /b 1
echo STEP9K BUILD SUCCESS
pause
