@echo off
setlocal
cd /d "%~dp0"
set "PIOEXE=%USERPROFILE%\.platformio\penv\Scripts\platformio.exe"
if not exist "%PIOEXE%" set "PIOEXE=pio"
echo Uploading READ-ONLY diagnostic firmware. Hardware must already be secured.
"%PIOEXE%" run -e mega_v6_he200_waveshare_readonly -t upload
if errorlevel 1 goto :fail
echo READ-ONLY upload completed.
pause
exit /b 0
:fail
echo Upload failed. Check board and COM port.
pause
exit /b 1
