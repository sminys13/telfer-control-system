@echo off
setlocal
cd /d "%~dp0"
set "PORT=8765"
set "PYEXE="
set "PYARGS="

if exist "%USERPROFILE%\.platformio\penv\Scripts\python.exe" (
  set "PYEXE=%USERPROFILE%\.platformio\penv\Scripts\python.exe"
) else (
  where py >nul 2>nul
  if not errorlevel 1 (
    set "PYEXE=py"
    set "PYARGS=-3"
  ) else (
    where python >nul 2>nul
    if not errorlevel 1 set "PYEXE=python"
  )
)

if not defined PYEXE (
  echo Python not found.
  echo PlatformIO normally installs Python here:
  echo %%USERPROFILE%%\.platformio\penv\Scripts\python.exe
  pause
  exit /b 1
)

echo Starting local service console at http://127.0.0.1:%PORT%/
start "" powershell -NoProfile -WindowStyle Hidden -Command "Start-Sleep -Seconds 2; Start-Process 'http://127.0.0.1:%PORT%/'"
"%PYEXE%" %PYARGS% -m http.server %PORT% --bind 127.0.0.1
endlocal
