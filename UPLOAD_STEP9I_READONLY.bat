@echo off
setlocal
cd /d "%~dp0"
set "PIOEXE=%USERPROFILE%\.platformio\penv\Scripts\platformio.exe"
if not exist "%PIOEXE%" set "PIOEXE=pio"
set "PORT=%~1"
if not defined PORT set /p "PORT=Enter Arduino COM port, for example COM6: "
if not defined PORT exit /b 1
"%PIOEXE%" run -e mega_v6_he200_waveshare_readonly -t upload --upload-port %PORT%
if errorlevel 1 goto :fail
echo READ-ONLY UPLOAD SUCCESS
pause
exit /b 0
:fail
echo UPLOAD FAILED
pause
exit /b 1
