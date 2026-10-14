@echo off
REM Flash a Zephyr app to Seeed XIAO nRF54L15 via OpenOCD.
REM Writes RRAM unbuffered (fixes lost last bytes) and verifies.
REM Usage:  flash_xiao.bat beacon   |   flash_xiao.bat scanner

if "%~1"=="" (
  echo Usage: flash_xiao.bat ^<app folder^>
  exit /b 1
)

set APP=%~1
set ROOT=%~dp0
set OCD=%ROOT%tools\openocd\bin\openocd.exe
set CFG=C:/ncs/v341/zephyr/boards/seeed/xiao_nrf54l15/support/openocd.cfg
set HEX=%ROOT%%APP%\build\%APP%\zephyr\zephyr.hex
set HEX=%HEX:\=/%

if not exist "%OCD%" ( echo OpenOCD not found: %OCD% & exit /b 1 )
if not exist "%HEX%" ( echo Build not found: %HEX% - build the app first & exit /b 1 )

"%OCD%" -f "%CFG%" -c "init" -c "reset init" -c "mww 0x5004b500 0x1" -c "load_image {%HEX%}" -c "verify_image {%HEX%}" -c "reset run" -c "shutdown"
