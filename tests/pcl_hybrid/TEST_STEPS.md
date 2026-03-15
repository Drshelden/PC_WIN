# pcl_hybrid End-to-End Test Steps (Windows)

This runbook walks through:
1. Configure and build (compile + link) the C++ executable and Python module.
2. Run a C++ command-line test.
3. Run Python command-line tests.
4. Run the Rhino Python workflow.

No repository cleanup actions are included here.

## Recommended Full Test (Default)

Use the automation script as the primary path. By default it runs comprehensively (configure, build, C++ CLI, Python CLI, smoke checks, pytest, then Rhino instructions) unless you pass skip flags.

From repo root:

    .\tests\pcl_hybrid\run_test_flow.ps1 -Config Release

Notes:
- The script auto-prefers repo venv Python if available at .venv\Scripts\python.exe.
- If you want to force a specific interpreter:

      .\tests\pcl_hybrid\run_test_flow.ps1 -Config Release -PythonExe "C:\_LOCAL\GitHub\PC_WIN\.venv\Scripts\python.exe"

## Environment Precedence Checks (Enforced by Script)

When Python tests are enabled (default), the script validates:
1. Python interpreter exists.
2. CMake exists.
3. vcpkg toolchain exists.
4. Required Python modules are present: pytest and numpy.

If requirements are missing, the script stops early with an actionable install command.

## 0. Preflight

Open PowerShell in repo root:

    Set-Location C:\_LOCAL\GitHub\PC_WIN

Confirm required tools:

    cmake --version
    cl
    "C:\Program Files\Python39\python.exe" --version

Confirm vcpkg toolchain exists:

    Test-Path C:\vcpkg\scripts\buildsystems\vcpkg.cmake

If any command fails, fix environment/tool install first.

Confirm Python test dependencies for comprehensive run:

    C:\_LOCAL\GitHub\PC_WIN\.venv\Scripts\python.exe -m pip install pytest numpy

## 1. Configure CMake (Release, Python 3.9)

From repo root:

    cmake -S tests/pcl_hybrid -B tests/pcl_hybrid/build_py39 -DCMAKE_TOOLCHAIN_FILE="C:\vcpkg\scripts\buildsystems\vcpkg.cmake" -DBUILD_PYTHON=ON -DPython3_ROOT_DIR="C:\Program Files\Python39" -DPYTHON_EXECUTABLE="C:\Program Files\Python39\python.exe" -DPYTHON_INCLUDE_DIR="C:\Program Files\Python39\include" -DPYTHON_LIBRARY="C:\Program Files\Python39\libs\python39.lib" -DCMAKE_BUILD_TYPE=Release

Expected:
- Configure completes without fatal errors.
- PCL and pybind11 are found.

## 2. Build and Link Targets

Build C++ executable target:

    cmake --build tests/pcl_hybrid/build_py39 --config Release --target pcl_hybrid

Build Python extension target:

    cmake --build tests/pcl_hybrid/build_py39 --config Release --target pcl_hybrid_py

Expected artifacts (examples):
- tests/pcl_hybrid/build_py39/Release/pcl_hybrid.exe
- tests/pcl_hybrid/build_py39/Release/pcl_hybrid_py.cp39-win_amd64.pyd

## 3. C++ Command-Line Test

Run the native executable on sample data:

    .\tests\pcl_hybrid\build_py39\Release\pcl_hybrid.exe .\tests\pcl_hybrid\data\quad.xyz .\tests\pcl_hybrid\data\output_cpp_cli.json

Expected console signals:
- Loaded <N> points from ...
- Found <K> clusters in total
- Wrote root JSON to ... (if export path provided)

Verify output file exists:

    Test-Path .\tests\pcl_hybrid\data\output_cpp_cli.json

## 4. Python Command-Line Test (wrapper script)

Set runtime paths for module and DLL lookup:

    $env:PYTHONPATH = "C:\_LOCAL\GitHub\PC_WIN\tests\pcl_hybrid\build_py39\Release;" + $env:PYTHONPATH
    $env:PATH = "C:\_LOCAL\GitHub\PC_WIN\tests\pcl_hybrid\build_py39\Release;C:\vcpkg\installed\x64-windows\bin;" + $env:PATH

Run Python wrapper:

    "C:\Program Files\Python39\python.exe" .\tests\pcl_hybrid\main.py .\tests\pcl_hybrid\data\quad.xyz .\tests\pcl_hybrid\data\output_py_cli.json

Expected console signals:
- Loaded <N> points from ...
- Found <K> clusters in total
- Traversing root shape: ...

Verify output file exists:

    Test-Path .\tests\pcl_hybrid\data\output_py_cli.json

## 5. Python Smoke Test Harness

Run the included smoke harness:

    "C:\Program Files\Python39\python.exe" .\tests\pcl_hybrid\runtests.py

Pytest run (required for comprehensive script mode):

    "C:\Program Files\Python39\python.exe" -m pytest .\tests\pcl_hybrid\test_import_buffer.py -q

Expected:
- Module import succeeds.
- Buffer import test runs under pytest and reports pass/skip/fail explicitly.

## 6. Rhino Python Test Path

Prerequisites:
- Rhino installed with Python scripting enabled.
- A point cloud object exists in the Rhino document.
- Built extension is present in tests/pcl_hybrid/build_py39/Release.

### Option A: Run settings UI workflow (recommended)

In Rhino Python editor, execute:

    import sys
    sys.path.insert(0, r"C:\_LOCAL\GitHub\PC_WIN\tests\pcl_hybrid")
    import Rhino3DPO
    Rhino3DPO.ShowPCLProcessorSettings()

Then:
1. In the dialog, click Run.
2. Select a point cloud when prompted.
3. Wait for processing and geometry visualization.

Expected:
- Settings saved to C:\Temp\pcl_processing\settings.json
- Console prints processing stages.
- Plane/cylinder visualization objects are added to Rhino.

### Option B: Direct processor call (no UI)

In Rhino Python editor, execute:

    import sys
    sys.path.insert(0, r"C:\_LOCAL\GitHub\PC_WIN\tests\pcl_hybrid")
    import rhino_client
    rhino_client.run_pcl_processing("BEST")

Expected:
- Prompt to select point cloud.
- Processing completes and adds visualized results.

## 7. Quick Failure Triage

If Python import fails for pcl_hybrid_py:
1. Confirm pyd exists in tests/pcl_hybrid/build_py39/Release.
2. Confirm PATH includes C:\vcpkg\installed\x64-windows\bin.
3. Run dependency checker:

       "C:\Program Files\Python39\python.exe" .\tests\pcl_hybrid\check_deps.py .\tests\pcl_hybrid\build_py39\Release\pcl_hybrid_py.cp39-win_amd64.pyd

If CMake cannot find dependencies:
1. Verify vcpkg package install for x64-windows.
2. Re-run configure with correct toolchain path.

## 8. Pass/Fail Criteria

Pass if all are true:
1. C++ build and Python module build both succeed.
2. Native C++ CLI run succeeds and writes JSON output.
3. Python CLI run succeeds and writes JSON output.
4. pytest test run executes successfully.
5. Rhino run completes and creates output geometry from selected cloud.

Fail if any stage errors or cannot load dependencies.
