@echo off
setlocal
cd /d "%~dp0"
set "PIOEXE=%USERPROFILE%\.platformio\penv\Scripts\platformio.exe"
if not exist "%PIOEXE%" set "PIOEXE=pio"
set "PORT=%~1"
if not defined PORT set /p "PORT=Enter Arduino COM port, for example COM6: "
if not defined PORT exit /b 1

echo ===============================================================
echo DANGER: FIELD SERVICE BUILD CAN WRITE HE200 AND MOVE MACHINERY.
echo AUTO/HOME remain blocked, but service pulse/assist can move drives.
echo Use only with verified NC E-STOP/safety chain and clear work area.
echo ===============================================================
set /p "CONFIRM=Type FIELD to continue: "
if /I not "%CONFIRM%"=="FIELD" exit /b 2
"%PIOEXE%" run -e mega_v6_he200_field_service -t upload --upload-port %PORT%
if errorlevel 1 goto :fail
echo FIELD SERVICE UPLOAD SUCCESS
pause
exit /b 0
:fail
echo UPLOAD FAILED
pause
exit /b 1
