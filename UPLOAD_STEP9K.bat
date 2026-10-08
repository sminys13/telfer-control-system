@echo off
setlocal
cd /d "%~dp0"
set "PIOEXE=%USERPROFILE%\.platformio\penv\Scripts\platformio.exe"
if not exist "%PIOEXE%" set "PIOEXE=pio"
set "MEGAPORT=%~1"
if not defined MEGAPORT set "MEGAPORT=COM8"
"%PIOEXE%" run -e mega_v6_he200_web_control -t upload --upload-port "%MEGAPORT%"
if errorlevel 1 exit /b 1
echo STEP9K UPLOAD SUCCESS. Controller boots DISARMED.
pause
