@echo off
REM CI-friendly Windows batch build script for tests/pcl_hybrid
REM Usage:
REM   build_windows_ci.bat [VcpkgRoot] [Triplet] [Configuration] [Visualize] [RunExe]
REM Examples:
REM   build_windows_ci.bat
REM   build_windows_ci.bat "C:\Users\sheldd\vcpkg" x64-windows Debug 0 1

SETLOCAL

n
:: Defaults
SET "VCPKGROOT=C:\Users\sheldd\vcpkg"
SET "TRIPLET=x64-windows"
SET "CONFIG=Debug"
SET "VISUALIZE=0"
SET "RUNEXE=0"

n
:: If help requested
IF "%1"=="/?" (
  ECHO Usage: %~nx0 [VcpkgRoot] [Triplet] [Configuration] [Visualize] [RunExe]
  ECHO Example: %~nx0 "C:\Users\sheldd\vcpkg" x64-windows Debug 0 1
  EXIT /B 0
)

n
:: Positional args override defaults
IF NOT "%~1"=="" SET "VCPKGROOT=%~1"
IF NOT "%~2"=="" SET "TRIPLET=%~2"
IF NOT "%~3"=="" SET "CONFIG=%~3"
IF NOT "%~4"=="" SET "VISUALIZE=%~4"
IF NOT "%~5"=="" SET "RUNEXE=%~5"

:: Resolve project and build dirs (script is located in tests\pcl_hybrid) 
SET "PROJDIR=%~dp0"
SET "BUILDDIR=%PROJDIR%build"
IF NOT EXIST "%BUILDDIR%" (
  mkdir "%BUILDDIR%"
)

SET "VCPKG_TOOLCHAIN=%VCPKGROOT%\scripts\buildsystems\vcpkg.cmake"
IF NOT EXIST "%VCPKG_TOOLCHAIN%" (
  ECHO Error: vcpkg toolchain file not found at "%VCPKG_TOOLCHAIN%" 1>&2
  EXIT /B 2
)

PUSHD "%BUILDDIR%"
ECHO Configuring with CMake...
cmake "%PROJDIR%" -DCMAKE_TOOLCHAIN_FILE="%VCPKG_TOOLCHAIN%" -DVCPKG_TARGET_TRIPLET="%TRIPLET%" -DVISUALIZE=%VISUALIZE% -A x64
IF ERRORLEVEL 1 (
  ECHO CMake configure failed with exit %ERRORLEVEL% 1>&2
  POPD
  EXIT /B 3
)

ECHO Building (configuration: %CONFIG%)...
cmake --build . --config "%CONFIG%"
IF ERRORLEVEL 1 (
  ECHO Build failed with exit %ERRORLEVEL% 1>&2
  POPD
  EXIT /B 4
)

IF "%RUNEXE%"=="1" (
  SET "EXE=%BUILDDIR%\%CONFIG%\pcl_hybrid.exe"
  IF NOT EXIST "%EXE%" SET "EXE=%BUILDDIR%\pcl_hybrid.exe"
  IF NOT EXIST "%EXE%" (
    ECHO Executable not found: %EXE% 1>&2
    POPD
    EXIT /B 5
  )
  ECHO Running "%EXE%"...
  "%EXE%"
  IF ERRORLEVEL 1 (
    ECHO Executable returned %ERRORLEVEL% 1>&2
    POPD
    EXIT /B 6
  )
)

POPD
ECHO Build succeeded
ENDLOCAL
EXIT /B 0
