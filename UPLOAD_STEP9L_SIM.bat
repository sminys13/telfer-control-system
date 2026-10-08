@echo off
setlocal
cd /d "%~dp0"
if "%~1"=="" (
  echo Usage: UPLOAD_STEP9L_SIM.bat COM7
  echo Specify the verified Mega port and close other serial monitors.
  exit /b 2
)
set "PIOEXE=%USERPROFILE%\.platformio\penv\Scripts\platformio.exe"
if not exist "%PIOEXE%" set "PIOEXE=pio"
"%PIOEXE%" run -e mega_v6_web_simulation -t upload --upload-port "%~1"
if errorlevel 1 exit /b 1
echo STEP9L SIM UPLOAD SUCCESS. Controller boots DISARMED.
pause
