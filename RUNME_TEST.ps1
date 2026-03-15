param(
  [ValidateSet('Release','Debug')][string]$Config = 'Release',
  [switch]$Visualize,
  [switch]$SkipConfigure,
  [switch]$SkipBuild,
  [switch]$SkipCpp,
  [switch]$SkipPython,
  [switch]$SkipDepsCheck
)

$ErrorActionPreference = 'Stop'

function Write-Stage([string]$Message) {
  Write-Host "`n=== $Message ===" -ForegroundColor Cyan
}

function Require-Command([string]$Name) {
  if (-not (Get-Command $Name -ErrorAction SilentlyContinue)) {
    throw "Required command not found on PATH: $Name"
  }
}

$RepoRoot = Split-Path -Parent $MyInvocation.MyCommand.Path
if (-not $RepoRoot) {
  $RepoRoot = Get-Location | Select-Object -ExpandProperty Path
}

$VenvPython = Join-Path $RepoRoot '.venv\Scripts\python.exe'
$RunFlow = Join-Path $RepoRoot 'tests\pcl_hybrid\run_test_flow.ps1'
$CheckDeps = Join-Path $RepoRoot 'tests\pcl_hybrid\check_deps.py'
$PydPath = Join-Path $RepoRoot "tests\pcl_hybrid\build_py39\$Config\pcl_hybrid_py.cp39-win_amd64.pyd"

Write-Stage 'Preflight'
Require-Command 'cmake'

if (-not (Test-Path $RunFlow)) {
  throw "Expected test flow script not found: $RunFlow"
}
if (-not (Test-Path $CheckDeps)) {
  throw "Expected dependency checker not found: $CheckDeps"
}

Write-Host "Repo root: $RepoRoot" -ForegroundColor Gray
Write-Host "Config: $Config" -ForegroundColor Gray

Write-Stage 'Python Environment'
if (-not (Test-Path $VenvPython)) {
  Write-Host '.venv not found, creating virtual environment...' -ForegroundColor Yellow
  if (Get-Command py -ErrorAction SilentlyContinue) {
    & py -3.9 -m venv (Join-Path $RepoRoot '.venv')
  } elseif (Get-Command python -ErrorAction SilentlyContinue) {
    & python -m venv (Join-Path $RepoRoot '.venv')
  } else {
    throw 'Neither py nor python was found to create a virtual environment.'
  }
}

if (-not (Test-Path $VenvPython)) {
  throw "Virtual environment python not found after create attempt: $VenvPython"
}

Write-Host "Using Python: $VenvPython" -ForegroundColor Gray
& $VenvPython --version

Write-Stage 'Install Python Prerequisites'
& $VenvPython -m pip install --upgrade pip
& $VenvPython -m pip install numpy pytest pefile

Write-Stage 'Run Full Test Flow'
& $RunFlow `
  -RepoRoot $RepoRoot `
  -Config $Config `
  -PythonExe $VenvPython `
  -Visualize:$Visualize `
  -SkipConfigure:$SkipConfigure `
  -SkipBuild:$SkipBuild `
  -SkipCpp:$SkipCpp `
  -SkipPython:$SkipPython

if (-not $SkipDepsCheck) {
  Write-Stage 'Dependency Check (.pyd)'
  if (Test-Path $PydPath) {
    & $VenvPython $CheckDeps $PydPath
  } else {
    Write-Host "Skipping check_deps: module not found at $PydPath" -ForegroundColor DarkYellow
  }
}

Write-Stage 'Done'
Write-Host 'RUNME_TEST completed.' -ForegroundColor Green
