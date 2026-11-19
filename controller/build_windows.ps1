# Build the native Windows x64 controller binary (run from PowerShell).
$ErrorActionPreference = "Stop"
Set-Location $PSScriptRoot

py -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install --upgrade pip
python -m pip install -r requirements.txt
python -m PyInstaller --clean --noconfirm --onefile --windowed `
  --name ground-controller src\ground_controller.py

Write-Host "Binary: $(Join-Path (Get-Location) 'dist\ground-controller.exe')"
