# pcl_hybrid — build & run instructions (Windows)

This folder contains the pcl_hybrid demo and Python bindings.

## Prerequisites (Windows)

- Visual Studio 2022 (with Desktop development with C++ workload)
- CMake (3.22+ recommended)
- Python 3.12 (or matching the interpreter you use to build the extension)
- vcpkg (installed and bootstrap-vcpkg completed)

## Recommended vcpkg packages

- pcl:x64-windows
- pybind11:x64-windows
- nlohmann-json:x64-windows

## Quick build and run (PowerShell)

From the repository root:

```powershell
cd .\tests\pcl_hybrid
```

Configure with the vcpkg toolchain (adjust path to your vcpkg):

```powershell
cmake -S . -B build -DBUILD_PYTHON=ON -DCMAKE_TOOLCHAIN_FILE=C:\path\to\vcpkg\scripts\buildsystems\vcpkg.cmake
```

Build the Python extension (Debug config):

```powershell
cmake --build build --config Debug --target pcl_hybrid_py
```

Run the demo using the workspace Python interpreter:

```powershell
python .\main.py .\data\quad.xyz
```

## Notes & troubleshooting

- If the Python import fails (can't load .pyd), ensure that the directory containing the built .pyd (`build\\Debug`) is on your PATH and `sys.path`. The demo script already prepends that directory when present.
- If CMake cannot find PCL or pybind11, make sure you've installed the packages via vcpkg for the matching triplet (`x64-windows`) and passed the vcpkg toolchain file to CMake.
- For MSVC parallel-build PDB collisions you may see errors — the CMake files set `/FS` on relevant targets to mitigate this.

## VS Code debug

- A VS Code debug configuration was added to `.vscode/launch.json` named **"Python: Run pcl_hybrid main.py (quad.xyz)"** which runs the demo with `data/quad.xyz`.
- If your Python extension suggests using `debugpy` instead of `python`, update the `type` field to `debugpy` per the extension's guidance.

## Contact

- If you need help reproducing the environment (vcpkg layout, Python version), tell me your exact Python version and the path to vcpkg and I can tailor the commands further.
