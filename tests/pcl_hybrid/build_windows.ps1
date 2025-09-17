param(
    [string]$VcpkgRoot = "$env:USERPROFILE\vcpkg",
    [string]$BuildType = "Release",
    [switch]$Visualize
)

$scriptDir = Split-Path -Parent $MyInvocation.MyCommand.Path
$sourceDir = (Resolve-Path "$scriptDir").Path
$buildDir = Join-Path $sourceDir "build"

if (-not (Test-Path $buildDir)) { New-Item -ItemType Directory -Path $buildDir | Out-Null }

$toolchain = Join-Path $VcpkgRoot "scripts\buildsystems\vcpkg.cmake"
if (-not (Test-Path $toolchain)) { Write-Error "vcpkg toolchain not found at $toolchain. Ensure vcpkg is bootstrapped."; exit 1 }

$visFlag = $false
if ($Visualize.IsPresent) { $visFlag = $true }

Write-Host "Configuring project with vcpkg toolchain: $toolchain"
cmake -S $sourceDir -B $buildDir -A x64 -DCMAKE_TOOLCHAIN_FILE="$toolchain" -DCMAKE_BUILD_TYPE=$BuildType -DVISUALIZE=$([int]$visFlag)

Write-Host "Building ($BuildType)"
cmake --build $buildDir --config $BuildType -- /m

Write-Host "Build finished. Executable located in: $buildDir\$BuildType"
