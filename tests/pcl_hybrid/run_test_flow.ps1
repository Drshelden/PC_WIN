param(
  [string]$RepoRoot = "C:\_LOCAL\GitHub\PC_WIN",
  [string]$PythonExe = "C:\Program Files\Python39\python.exe",
  [string]$VcpkgToolchain = "C:\vcpkg\scripts\buildsystems\vcpkg.cmake",
  [string]$VcpkgBin = "C:\vcpkg\installed\x64-windows\bin",
  [string]$VcpkgDebugBin = "C:\vcpkg\installed\x64-windows\debug\bin",
  [string]$PclConfigDir = "C:\vcpkg\installed\x64-windows\share\pcl",
  [string]$PclRuntimeBin = "C:\Program Files\PCL 1.14.1\bin",
  [string]$OpenNI2Bin = "C:\Program Files\OpenNI2\Redist",
  [string]$BuildDirName = "build_py39",
  [ValidateSet('Debug','Release')][string]$Config = "Release",
  [switch]$Visualize,
  [switch]$SkipConfigure,
  [switch]$SkipBuild,
  [switch]$SkipCpp,
  [switch]$SkipPython
)

$ErrorActionPreference = "Stop"

# In PowerShell 7+, native tools writing to stderr (for warnings) can be turned
# into terminating errors when ErrorActionPreference=Stop. We still validate
# success via LASTEXITCODE in Run-Step, so keep stderr warnings non-fatal.
if (Get-Variable -Name PSNativeCommandUseErrorActionPreference -ErrorAction SilentlyContinue) {
  $PSNativeCommandUseErrorActionPreference = $false
}

function Write-Stage($msg) {
  Write-Host "`n=== $msg ===" -ForegroundColor Cyan
}

function Assert-PathExists([string]$PathToCheck, [string]$What) {
  if (-not (Test-Path $PathToCheck)) {
    throw "$What not found: $PathToCheck"
  }
}

function Run-Step([string]$Name, [scriptblock]$Action) {
  Write-Host "[RUN] $Name" -ForegroundColor Yellow
  $global:LASTEXITCODE = 0
  $oldEap = $ErrorActionPreference
  try {
    # Native tools often emit warnings to stderr; treat exit code as source of truth.
    $ErrorActionPreference = "Continue"
    & $Action
  }
  finally {
    $ErrorActionPreference = $oldEap
  }
  if ($LASTEXITCODE -ne 0) {
    throw "Step failed with exit code ${LASTEXITCODE}: $Name"
  }
  Write-Host "[OK ] $Name" -ForegroundColor Green
}

function Get-MissingPythonModules([string]$Interpreter, [string[]]$Modules) {
  $joined = ($Modules -join "|")
  $cmd = "import importlib.util; mods='$joined'.split('|'); missing=[m for m in mods if importlib.util.find_spec(m) is None]; print(';'.join(missing))"
  $out = & $Interpreter -c $cmd
  if ($LASTEXITCODE -ne 0) {
    throw "Failed probing Python modules with interpreter: $Interpreter"
  }
  if (-not $out) {
    return @()
  }
  $text = ($out | Out-String).Trim()
  if (-not $text) {
    return @()
  }
  return $text.Split(';') | Where-Object { $_ -and $_.Trim().Length -gt 0 }
}

function Write-Banner() {
  $willConfigure = -not $SkipConfigure
  $willBuild = -not $SkipBuild
  $willCpp = -not $SkipCpp
  $willPy = -not $SkipPython
  $isComprehensive = $willConfigure -and $willBuild -and $willCpp -and $willPy

  Write-Host "" 
  Write-Host "pcl_hybrid test runner" -ForegroundColor Cyan
  Write-Host "Config: $Config" -ForegroundColor Gray
  Write-Host "Python: $PythonExe" -ForegroundColor Gray
  Write-Host "VISUALIZE: $(if ($Visualize) { 'ON' } else { 'OFF' })" -ForegroundColor Gray
  Write-Host "Comprehensive mode: $(if ($isComprehensive) { 'ON' } else { 'OFF' })" -ForegroundColor Gray
  Write-Host "Stages:" -ForegroundColor Gray
  Write-Host "  Configure: $(if ($willConfigure) { 'RUN' } else { 'SKIP' })" -ForegroundColor Gray
  Write-Host "  Build:     $(if ($willBuild) { 'RUN' } else { 'SKIP' })" -ForegroundColor Gray
  Write-Host "  C++ test:  $(if ($willCpp) { 'RUN' } else { 'SKIP' })" -ForegroundColor Gray
  Write-Host "  Python:    $(if ($willPy) { 'RUN' } else { 'SKIP' })" -ForegroundColor Gray
  Write-Host "" 
  Write-Host "Common options:" -ForegroundColor Yellow
  Write-Host "  Full default run: .\\tests\\pcl_hybrid\\run_test_flow.ps1 -Config Release" -ForegroundColor Gray
  Write-Host "  Debug run:        .\\tests\\pcl_hybrid\\run_test_flow.ps1 -Config Debug" -ForegroundColor Gray
  Write-Host "  Use venv python:  .\\tests\\pcl_hybrid\\run_test_flow.ps1 -PythonExe 'C:\\_LOCAL\\GitHub\\PC_WIN\\.venv\\Scripts\\python.exe'" -ForegroundColor Gray
  Write-Host "  Skip configure:   ... -SkipConfigure -SkipBuild" -ForegroundColor Gray
  Write-Host "  Python only:      ... -SkipConfigure -SkipBuild -SkipCpp" -ForegroundColor Gray
  Write-Host "  C++ only:         ... -SkipConfigure -SkipBuild -SkipPython" -ForegroundColor Gray
  Write-Host "  Enable visualize: ... -Visualize" -ForegroundColor Gray
  Write-Host ""
}

$PclDir = Join-Path $RepoRoot "tests\pcl_hybrid"
$BuildDir = Join-Path $PclDir $BuildDirName
$DataDir = Join-Path $PclDir "data"
$CppOut = Join-Path $DataDir "output_cpp_cli.json"
$PyOut = Join-Path $DataDir "output_py_cli.json"

# Prefer repo virtual environment automatically when caller did not pass -PythonExe.
if (-not $PSBoundParameters.ContainsKey('PythonExe')) {
  $VenvPython = Join-Path $RepoRoot ".venv\Scripts\python.exe"
  if (Test-Path $VenvPython) {
    $PythonExe = $VenvPython
    Write-Host "Using default repo venv interpreter: $PythonExe" -ForegroundColor Gray
  }
}

Write-Banner

$RuntimeDirs = @(
  (Join-Path $BuildDir $Config),
  (Join-Path $BuildDir "Release"),
  (Join-Path $BuildDir "Debug"),
  $VcpkgDebugBin,
  $VcpkgBin,
  $PclRuntimeBin,
  $OpenNI2Bin
) | Where-Object { $_ -and (Test-Path $_) }

if ($RuntimeDirs.Count -gt 0) {
  Write-Host "Runtime PATH prefixes:" -ForegroundColor Gray
  $RuntimeDirs | ForEach-Object { Write-Host "  $_" -ForegroundColor Gray }
}

Write-Stage "Preflight"
Assert-PathExists $RepoRoot "Repo root"
Assert-PathExists $PclDir "pcl_hybrid folder"
Assert-PathExists $PythonExe "Python executable"
Assert-PathExists $VcpkgToolchain "vcpkg toolchain"

Run-Step "CMake available" { cmake --version | Out-Host }
Run-Step "Python available" { & $PythonExe --version | Out-Host }

if (-not $SkipPython) {
  $requiredPyModules = @("pytest", "numpy")
  $missingModules = Get-MissingPythonModules -Interpreter $PythonExe -Modules $requiredPyModules
  if ($missingModules.Count -gt 0) {
    $missingText = ($missingModules -join ", ")
    throw "Python environment is missing required modules for comprehensive tests: $missingText. Install with: `"$PythonExe`" -m pip install $missingText"
  }
  Write-Host "Python precheck OK: required modules present (pytest, numpy)" -ForegroundColor Green
}

if (-not (Test-Path $BuildDir)) {
  New-Item -ItemType Directory -Path $BuildDir | Out-Null
}

if (-not $SkipConfigure) {
  Write-Stage "Configure (CMake)"
  $PclDirArg = @()
  if (Test-Path $PclConfigDir) {
    $PclDirArg = @("-DPCL_DIR=$PclConfigDir")
    Write-Host "Using PCL_DIR: $PclConfigDir" -ForegroundColor Gray
  } else {
    Write-Host "PCL config dir not found, letting CMake resolve PCL automatically: $PclConfigDir" -ForegroundColor DarkYellow
  }

  $VisualizeValue = if ($Visualize) { "ON" } else { "OFF" }
  $VisualizeArg = "-DVISUALIZE=$VisualizeValue"
  Write-Host "VISUALIZE=$VisualizeValue" -ForegroundColor Gray

  Run-Step "cmake configure" {
    cmake -S $PclDir -B $BuildDir `
      -DCMAKE_TOOLCHAIN_FILE="$VcpkgToolchain" `
      -DBUILD_PYTHON=ON `
      $VisualizeArg `
      -DPython3_ROOT_DIR="C:\Program Files\Python39" `
      -DPYTHON_EXECUTABLE="$PythonExe" `
      -DPYTHON_INCLUDE_DIR="C:\Program Files\Python39\include" `
      -DPYTHON_LIBRARY="C:\Program Files\Python39\libs\python39.lib" `
      -DCMAKE_BUILD_TYPE=$Config `
      @PclDirArg | Out-Host
  }
} else {
  Write-Host "Skipping configure" -ForegroundColor DarkYellow
}

if (-not $SkipBuild) {
  Write-Stage "Build and Link"
  Run-Step "build pcl_hybrid" {
    cmake --build $BuildDir --config $Config --target pcl_hybrid | Out-Host
  }
  Run-Step "build pcl_hybrid_py" {
    cmake --build $BuildDir --config $Config --target pcl_hybrid_py | Out-Host
  }
} else {
  Write-Host "Skipping build" -ForegroundColor DarkYellow
}

$ExePath = Join-Path $BuildDir "$Config\pcl_hybrid.exe"
$PydPattern = Join-Path $BuildDir "$Config\pcl_hybrid_py*.pyd"
$PydFound = Get-ChildItem -Path $PydPattern -ErrorAction SilentlyContinue | Select-Object -First 1

$NeedExe = -not $SkipCpp
$NeedPyd = -not $SkipPython

Write-Stage "Artifacts"
if (-not $NeedExe -and -not $NeedPyd) {
  Write-Host "No runtime artifacts required for selected stage combination." -ForegroundColor DarkYellow
} else {
  if ($NeedExe) {
    Assert-PathExists $ExePath "C++ executable"
    Write-Host "Found executable: $ExePath" -ForegroundColor Green
  }
  if ($NeedPyd) {
    if (-not $PydFound) {
      throw "Python extension (.pyd) not found under: $PydPattern"
    }
    Write-Host "Found extension: $($PydFound.FullName)" -ForegroundColor Green
  }
}

if (-not $SkipCpp) {
  Write-Stage "C++ CLI Test"
  # Ensure native runtime dependencies (PCL/Boost/OpenNI2/etc.) are discoverable by the EXE.
  if ($RuntimeDirs.Count -gt 0) {
    $env:PATH = (($RuntimeDirs -join ";") + ";" + $env:PATH)
  }

  Run-Step "run pcl_hybrid.exe" {
    & $ExePath (Join-Path $DataDir "quad.xyz") $CppOut | Out-Host
  }
  Assert-PathExists $CppOut "C++ output JSON"
} else {
  Write-Host "Skipping C++ test" -ForegroundColor DarkYellow
}

if (-not $SkipPython) {
  Write-Stage "Python CLI Tests"

  $env:PCL_HYBRID_BUILD_DIR = (Join-Path $BuildDir $Config)
  $env:PCL_HYBRID_DLL_DIRS = ($RuntimeDirs -join ";")
  $env:PYTHONPATH = (Join-Path $BuildDir $Config) + ";" + ($env:PYTHONPATH)
  if ($RuntimeDirs.Count -gt 0) {
    $env:PATH = (($RuntimeDirs -join ";") + ";" + $env:PATH)
  }

  Run-Step "run main.py" {
    # Probe import using the exact selected build dir before running main.py.
    $probeCode = @"
import os, sys
build_dir = os.environ.get('PCL_HYBRID_BUILD_DIR', '')
if build_dir and os.path.isdir(build_dir):
    if build_dir not in sys.path:
        sys.path.insert(0, build_dir)
try:
    import pcl_hybrid_py
    print('pcl_hybrid_py import probe: OK from', getattr(pcl_hybrid_py, '__file__', '<unknown>'))
except Exception as e:
    print('pcl_hybrid_py import probe: FAIL:', e)
    raise
"@
    & $PythonExe -c $probeCode | Out-Host
    if ($LASTEXITCODE -ne 0) {
      Write-Host "Import probe failed. Running dependency checker on selected module if present..." -ForegroundColor DarkYellow
      $probePyd = Get-ChildItem -Path (Join-Path (Join-Path $BuildDir $Config) "pcl_hybrid_py*.pyd") -ErrorAction SilentlyContinue | Select-Object -First 1
      if ($probePyd) {
        & $PythonExe (Join-Path $PclDir "check_deps.py") $probePyd.FullName | Out-Host
      } else {
        Write-Host "No pyd found for dependency check under: $(Join-Path $BuildDir $Config)" -ForegroundColor DarkYellow
      }
      throw "Python import probe failed before main.py"
    }

    & $PythonExe (Join-Path $PclDir "main.py") (Join-Path $DataDir "quad.xyz") $PyOut | Out-Host
  }
  Assert-PathExists $PyOut "Python output JSON"

  Run-Step "run runtests.py" {
    & $PythonExe (Join-Path $PclDir "runtests.py") | Out-Host
  }

  Run-Step "run pytest buffer test" {
    & $PythonExe -m pytest (Join-Path $PclDir "test_import_buffer.py") -q | Out-Host
  }
} else {
  Write-Host "Skipping Python tests" -ForegroundColor DarkYellow
}

Write-Stage "Rhino Manual Step"
Write-Host "Run this inside Rhino Python editor:" -ForegroundColor White
Write-Host "  import sys" -ForegroundColor Gray
Write-Host "  sys.path.insert(0, r'C:\_LOCAL\GitHub\PC_WIN\tests\pcl_hybrid')" -ForegroundColor Gray
Write-Host "  import Rhino3DPO" -ForegroundColor Gray
Write-Host "  Rhino3DPO.ShowPCLProcessorSettings()" -ForegroundColor Gray
Write-Host "Then click Run, select a point cloud, and verify visualization objects are created." -ForegroundColor White

Write-Stage "Completed"
Write-Host "Automated stages passed. Rhino stage is interactive and must be validated manually." -ForegroundColor Green
