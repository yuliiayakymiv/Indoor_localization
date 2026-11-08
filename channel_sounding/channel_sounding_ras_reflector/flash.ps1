<#
.SYNOPSIS
  Flash the built firmware to a Seeed XIAO nRF54L15 with pyOCD.

.DESCRIPTION
  Do NOT use "west flash" with the Seeed OpenOCD runner for this board: it can leave
  the last bytes of the image unwritten, which makes the firmware crash at boot
  (MPU FAULT in net_buf_alloc_len). pyOCD writes the image correctly.

  One-time setup (PowerShell):
    python -m venv $HOME\pyocd-venv
    & $HOME\pyocd-venv\Scripts\Activate.ps1
    pip install libusb libusb-package
    pip install git+https://github.com/pyocd/pyOCD.git

.EXAMPLE
  .\flash.ps1 -Probe 3E7C21C1          # flash one specific board
  .\flash.ps1 -Probe 3E7C21C1 -Erase   # mass-erase first, then flash
  .\flash.ps1 -List                    # show connected probes
#>
param(
    [string]$Probe,
    [string]$Hex = "build\channel_sounding_ras_reflector\zephyr\zephyr.hex",
    [switch]$Erase,
    [switch]$List
)

$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

if ($List) { python -m pyocd list; exit 0 }

if (-not (Test-Path $Hex)) {
    Write-Error "Firmware not found: $Hex. Build first (see README.md)."
}

$probeArgs = @()
if ($Probe) { $probeArgs = @("-u", $Probe) }
else { Write-Host "No -Probe given. If more than one board is plugged in, pass -Probe <ID> (see -List)." }

if ($Erase) { python -m pyocd erase @probeArgs -t nrf54l --mass }
python -m pyocd load $Hex @probeArgs -t nrf54l
Write-Host "Done. Press the board's reset button."
