@echo off
rem One-click start of the workbench on the Linux machine, from the Windows PC (owner request, 2026-11-25).
rem Opens the SSH tunnel, starts the workbench there and opens http://127.0.0.1:8765 in the browser.
rem Close this window (or press Ctrl-C in it) to stop the workbench; a tool still running gets its stop first.
rem Needs the SSH key login (ssh slosh-linux). Other host or folder: set SLOSH_HOST / SLOSH_DIR before running.
setlocal
if "%SLOSH_HOST%"=="" set "SLOSH_HOST=slosh-linux"
if "%SLOSH_DIR%"=="" set "SLOSH_DIR=~/Desktop/NMPC_slosh"
title Slosh workbench on %SLOSH_HOST%
echo Starting the workbench on %SLOSH_HOST% (%SLOSH_DIR%).
echo The page opens in your browser in a few seconds. Close this window to stop the workbench.
echo.
start "" /b powershell -NoProfile -Command "Start-Sleep 5; Start-Process 'http://127.0.0.1:8765'"
ssh -t -o ExitOnForwardFailure=yes -o ServerAliveInterval=15 -L 8765:127.0.0.1:8765 %SLOSH_HOST% "cd %SLOSH_DIR% && . .venv/bin/activate && exec python -m tools.workbench"
if errorlevel 1 (
  echo.
  echo The workbench did not start, or stopped with an error: see the lines above.
  echo Already running? Close the other workbench window or tunnel first.
  pause
)
