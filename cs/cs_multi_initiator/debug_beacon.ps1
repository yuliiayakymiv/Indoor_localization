<#
.SYNOPSIS
  Debug firmware for beacon IL_B3: builds it and (with -Probe) flashes it.

.DESCRIPTION
  Put this file into cs\cs_multi_initiator (next to flash.ps1) and run it from the
  nRF Connect SDK terminal in that folder, after "git pull".

  Step 1: beacon 3 with the log on USB, external antenna (as in the real build).
  Step 2: the same, but with the onboard antenna (to rule out the antenna).

  After flashing, open the board's COM port at 115200 and press reset to see the log
  from the start. Check in the nRF Connect phone app whether IL_B3 shows up.

  Windows notes (fixes compared to the first version):
  - Build folders are short (b3log / b3int): with the long project path, longer names
    exceed the Windows path limit and the build fails in nrf_security/cracen.
  - The -D options are quoted: Windows PowerShell splits -DEXTRA_CONF_FILE=beacon.conf
    at the dot otherwise ("File not found: ...\beacon").
  - Flashing calls the Python of the pyOCD venv directly, with PYTHONPATH/PYTHONHOME cleared,
    so this works from the nRF Connect terminal without activating the venv (activating it
    breaks west, and the SDK's PYTHONPATH breaks the venv's pyOCD).

.EXAMPLE
  .\debug_beacon.ps1 -Step 1                    # only build
  .\debug_beacon.ps1 -Step 1 -Probe 3E7C21C1    # build and flash this board
  .\debug_beacon.ps1 -Step 2 -Probe 3E7C21C1
#>
param(
    [Parameter(Mandatory = $true)][ValidateSet(1, 2)][int]$Step,
    [string]$Probe,
    [string]$PyocdPython = "$HOME\pyocd-venv\Scripts\python.exe"
)

$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

if ($Step -eq 1) {
    $dir = "b3log"
    $antenna = "y"
} else {
    $dir = "b3int"
    $antenna = "n"
}

west build -b xiao_nrf54l15/nrf54l15/cpuapp -d $dir --pristine -- `
    "-DEXTRA_CONF_FILE=beacon.conf" `
    "-DCONFIG_IL_BEACON_ID=3" `
    "-DCONFIG_SERIAL=y" `
    "-DCONFIG_UART_CONSOLE=y" `
    "-DCONFIG_IL_EXTERNAL_ANTENNA=$antenna"

if ($LASTEXITCODE -ne 0) { Write-Error "Build failed" }

$hex = "$dir\cs_multi_initiator\zephyr\zephyr.hex"
Write-Host "Built: $hex"

if (-not $Probe) {
    Write-Host "To flash, run this script again with -Probe <ID>."
    exit 0
}

if (-not (Test-Path $PyocdPython)) {
    Write-Error "pyOCD venv not found at $PyocdPython (see the header of flash.ps1 for the setup)."
}

# The nRF Connect terminal sets PYTHONPATH (and possibly PYTHONHOME) to the SDK's Python 3.12
# packages. The venv's Python would then load the SDK's pyocd and fail with
# "Module use of python312.dll conflicts with this version of Python". Clear them for this call.
$savedPath = $env:PYTHONPATH
$savedHome = $env:PYTHONHOME
Remove-Item Env:PYTHONPATH -ErrorAction SilentlyContinue
Remove-Item Env:PYTHONHOME -ErrorAction SilentlyContinue
try {
    & $PyocdPython -m pyocd load $hex -u $Probe -t nrf54l
    $flashExit = $LASTEXITCODE
} finally {
    if ($null -ne $savedPath) { $env:PYTHONPATH = $savedPath }
    if ($null -ne $savedHome) { $env:PYTHONHOME = $savedHome }
}
if ($flashExit -ne 0) { Write-Error "Flashing failed" }
Write-Host "Done. Open the COM port at 115200 and press the board's reset button."
