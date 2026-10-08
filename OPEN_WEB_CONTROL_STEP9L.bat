@echo off
setlocal
cd /d "%~dp0tools\telfer_web_control"
set "PYEXE=%USERPROFILE%\.platformio\penv\Scripts\python.exe"
if not exist "%PYEXE%" set "PYEXE=python"
start "" powershell -NoProfile -WindowStyle Hidden -Command "Start-Sleep -Seconds 2; Start-Process 'http://127.0.0.1:8766/'"
echo Step9L web control: http://127.0.0.1:8766/
"%PYEXE%" -m http.server 8766 --bind 127.0.0.1
