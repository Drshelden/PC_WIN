<#
CI-friendly Windows PowerShell build script for tests/pcl_hybrid

Usage (from repository root or anywhere):
  # set variables as needed
  $vcpkg = 'C:\Users\sheldd\vcpkg'
  .\tests\pcl_hybrid\build_windows_ci.ps1 -VcpkgRoot $vcpkg -Triplet x64-windows -Configuration Debug -Visualize 0

Parameters:
  -VcpkgRoot    : path to vcpkg root (defaults to C:/Users/sheldd/vcpkg if not provided)
  -Triplet      : vcpkg triplet (default: x64-windows)
  -Configuration: CMake build config (Debug/Release) default Debug
  -Visualize    : 0 or 1 to set VISUALIZE (default 0)
  -RunExe       : switch; if present the script will run the produced exe after building

Exit codes:
  0 = success (configure + build succeeded, optional run succeeded)
  non-zero = failed at some step; the script writes errors to stderr
#>

param(
  [string]$VcpkgRoot = 'C:\Users\sheldd\vcpkg',
  [string]$Triplet = 'x64-windows',
  [ValidateSet('Debug','Release','RelWithDebInfo','MinSizeRel')][string]$Configuration = 'Debug',
  [ValidateSet('0','1')][string]$Visualize = '0',
  [switch]$RunExe
)

set -e

$projDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$repoRoot = Resolve-Path (Join-Path $projDir '..')
$buildDir = Join-Path $projDir 'build'
if (-Not (Test-Path $buildDir)) { New-Item -ItemType Directory -Path $buildDir | Out-Null }

Write-Host "Repo root: $repoRoot"
Write-Host "Build dir: $buildDir"
Write-Host "Vcpkg root: $VcpkgRoot"
Write-Host "Triplet: $Triplet"
Write-Host "Configuration: $Configuration"
Write-Host "VISUALIZE: $Visualize"

# Compose toolchain file path
$vcpkgToolchain = Join-Path $VcpkgRoot 'scripts\buildsystems\vcpkg.cmake'
if (-Not (Test-Path $vcpkgToolchain)) {
  Write-Error "vcpkg toolchain file not found at $vcpkgToolchain"
  exit 2
}

# Configure
Push-Location $buildDir
try {
  $cmakeArgs = @(
    "-DCMAKE_TOOLCHAIN_FILE=$vcpkgToolchain",
    "-DVCPKG_TARGET_TRIPLET=$Triplet",
    "-DVISUALIZE=$Visualize",
    "-A", "x64",
    ".."
  )

  Write-Host "Running CMake configure..."
  & cmake @cmakeArgs
  if ($LASTEXITCODE -ne 0) { throw "CMake configure failed with exit $LASTEXITCODE" }

  Write-Host "Building (configuration: $Configuration)..."
  & cmake --build . --config $Configuration
  if ($LASTEXITCODE -ne 0) { throw "Build failed with exit $LASTEXITCODE" }

  if ($RunExe) {
    $exe = Join-Path $buildDir "$Configuration\pcl_hybrid.exe"
    if (-Not (Test-Path $exe)) {
      # try single-config layout
      $exe = Join-Path $buildDir "pcl_hybrid.exe"
    }
    if (-Not (Test-Path $exe)) { throw "Executable not found: $exe" }
    Write-Host "Running executable: $exe"
    & $exe
    if ($LASTEXITCODE -ne 0) { throw "Executable returned exit code $LASTEXITCODE" }
  }

  Write-Host "Build succeeded"
  exit 0
}
catch {
  Write-Error $_
  exit 3
}
finally {
  Pop-Location
}
