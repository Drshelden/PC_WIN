@echo off
REM Batch file to build pcl_hybrid_py for Python 3.9.10 x64 using MSVC, CMake, and vcpkg
REM Edit the following paths as needed for your environment

set "PYTHON_ROOT=C:\Program Files\Python39"
set "VCPKG_TOOLCHAIN=C:\vcpkg\scripts\buildsystems\vcpkg.cmake"
set "DCMAKE_PREFIX_PATH=C:\vcpkg\installed\x64-windows"
set "PROJECT_ROOT=C:\_LOCAL\GitHub\PC_WIN\tests\pcl_hybrid"
cd /d "%PROJECT_ROOT%"

cmake -S . -B build_py39 -DBUILD_PYTHON=ON -DCMAKE_TOOLCHAIN_FILE="%VCPKG_TOOLCHAIN%" -DPython3_ROOT_DIR="%PYTHON_ROOT%" -DPYTHON_EXECUTABLE="%PYTHON_ROOT%\python.exe" -DPYTHON_INCLUDE_DIR="%PYTHON_ROOT%\include" -DPYTHON_LIBRARY="%PYTHON_ROOT%\libs\python39.lib"
REM Step 1: Install dependencies with vcpkg (run manually if already installed)
REM cd /d C:\path\to\vcpkg
REM .\vcpkg install pcl:x64-windows pybind11:x64-windows nlohmann-json:x64-windows

REM Step 2: Configure CMake for Python 3.9.10
cmake -S . -B build_py39 -DBUILD_PYTHON=ON -DCMAKE_TOOLCHAIN_FILE="%VCPKG_TOOLCHAIN%" -DCMAKE_PREFIX_PATH="C:\vcpkg\installed\x64-windows" -DPython3_ROOT_DIR="%PYTHON_ROOT%" -DPYTHON_EXECUTABLE="%PYTHON_ROOT%\python.exe" -DPYTHON_INCLUDE_DIR="%PYTHON_ROOT%\include" -DPYTHON_LIBRARY="%PYTHON_ROOT%\libs\python39.lib"

REM Step 3: Build the Python extension
cmake --build build_py39 --config Release --target pcl_hybrid_py


