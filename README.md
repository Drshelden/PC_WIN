# PC_WIN — Point Cloud Shape Detection

Windows-native point cloud processing library built on [PCL](https://pointclouds.org/) (1.14+) with Python 3.9 bindings via [pybind11](https://pybind11.readthedocs.io/). Detects planes and cylinders in `.xyz` / `.pcd` / `.e57` point cloud files and exports results as JSON.

---

## Prerequisites

| Requirement | Notes |
|-------------|-------|
| Visual Studio 2022 | Desktop development with C++ workload |
| CMake 3.22+ | Must be on `PATH` |
| Python 3.9 | Installed at `C:\Program Files\Python39` |
| vcpkg | With `pcl:x64-windows`, `pybind11:x64-windows`, `nlohmann-json:x64-windows` |

---

## Repository layout

```
tests/pcl_hybrid/          Core C++ source + Python bindings + test scripts
tests/pcl_hybrid/data/     Sample point cloud files (*.xyz)
tests/pcl_hybrid/bindings/ pybind11 wrapper (py_pcl_hybrid.cpp)
tests/pcl_hybrid/build_py39/  CMake out-of-source build folder
```

---

## Quick start — full build + test (recommended)

Run from the **repository root** in PowerShell:

```powershell
.\RUNME_TEST.ps1
```

This single script:
1. Creates / activates a `.venv` with `numpy`, `pytest`, `pefile`
2. Configures the CMake build (Release, Python 3.9, vcpkg toolchain)
3. Compiles the C++ core library and Python extension (`.pyd`)
4. Runs the C++ smoke test against `data/quad.xyz`
5. Runs the Python import test suite (`runtests.py`)
6. Checks the `.pyd` for missing DLL dependencies

### Switches

```powershell
.\RUNME_TEST.ps1 -Config Debug          # Debug build instead of Release
.\RUNME_TEST.ps1 -SkipConfigure         # Skip CMake configure (already configured)
.\RUNME_TEST.ps1 -SkipBuild             # Skip compile step (already built)
.\RUNME_TEST.ps1 -SkipCpp               # Skip C++ executable test
.\RUNME_TEST.ps1 -SkipPython            # Skip Python test suite
.\RUNME_TEST.ps1 -SkipDepsCheck         # Skip DLL dependency audit
.\RUNME_TEST.ps1 -SkipConfigure -SkipBuild   # Re-run tests only
```

---

## Running shape detection on a point cloud

### Using the Python wrapper (`main.py`)

```powershell
cd tests\pcl_hybrid
python main.py "data\pipe and planes.xyz"
```

Expected output (abbreviated):

```
Loaded 8000 points from data\pipe and planes.xyz
Found 6 clusters in total
Cluster 0: structured array shape (2617,), dtype=[('x','<f4'),('y','<f4'),('z','<f4'),...]
...
Traversing root shape:
  type=root  coefficients=[]
    type=cylinder  coefficients=[0.0, 0.0, 0.249, 0.0, 0.0, 1.0, ...]
    type=cylinder  coefficients=[0.249, 0.0, 0.0, 1.0, 0.0, 0.0, ...]
    type=plane     coefficients=[0.0, 0.0, 1.0, ...]
    ...
```

To save results to a JSON file:

```powershell
python main.py "data\pipe and planes.xyz" output.json
```

The exported JSON contains the full shape tree with coefficients and critical points for each detected plane and cylinder.

### Using the C++ executable (`main`)

After building, the standalone executable is in `build_py39\Release\`:

```powershell
cd tests\pcl_hybrid
.\build_py39\Release\pcl_hybrid.exe "data\pipe and planes.xyz"
.\build_py39\Release\pcl_hybrid.exe "data\pipe and planes.xyz" output.json
```

The executable and `main.py` produce identical results — the Python wrapper calls the same underlying C++ engine through the `.pyd` extension.

### JSON output format

```json
{
  "type": "root",
  "children": [
    {
      "type": "cylinder",
      "coefficients": [cx, cy, cz, ax, ay, az, radius],
      "critical_points": [[x1,y1,z1], [x2,y2,z2]],
      "cluster_id": 0
    },
    {
      "type": "plane",
      "coefficients": [nx, ny, nz, d],
      "critical_points": [[...], [...], [...], [...]],
      "cluster_id": 1
    }
  ]
}
```

Cylinder `coefficients`: `[cx, cy, cz]` = axis point, `[ax, ay, az]` = unit axis direction, `radius` = fitted radius.  
Plane `coefficients`: `[nx, ny, nz]` = unit normal, `d` = offset.

---

## Minimal Python API example (`example.py`)

```python
from pcl_hybrid_py import PCWin_PointCloud, ShapeFinder
import sys, os

# Make sure the built extension is importable
build_dir = r"tests\pcl_hybrid\build_py39\Release"
os.add_dll_directory(os.path.abspath(build_dir))
sys.path.insert(0, os.path.abspath(build_dir))

pc = PCWin_PointCloud()
pc.importPoints(r"tests\pcl_hybrid\data\pipe and planes.xyz")

sf = ShapeFinder()
sf.findShapes(pc)

print("Clusters found:", sf.clusters_size())
print(sf.get_root_json()[:600])
```

Run from the repo root after a successful build:

```powershell
cd tests\pcl_hybrid
python example.py
```

---

## Rhino 3D integration

### `rhino_client.py` — shape detection inside Rhinoceros

Runs **inside Rhinoceros 3D** as a Python script (via the `RunPythonScript` command or a toolbar button). It:

1. Lets you select point cloud objects in the Rhino viewport
2. Exports the selected cloud to a temporary `.xyz` file
3. Calls the local `pcl_hybrid_py` extension to detect planes and cylinders
4. Draws the detected geometry (planes as surfaces, cylinders as solids) and labels back into the Rhino scene

**How to run:**
1. Open Rhinoceros 3D and load a scene containing a point cloud
2. In Rhino, run: `RunPythonScript` → browse to `tests\pcl_hybrid\rhino_client.py`
3. Follow the on-screen prompts to select a point cloud object

The script auto-discovers the built `.pyd` in `build_py39\Release\`.

### `Twinner.py` — settings panel for Rhino

A tabbed Eto.Forms dialog that lets you adjust all PCL processing parameters (cluster tolerance, normal search radius, cylinder/plane RANSAC thresholds, etc.) without editing code. Depends on `rhino_client.py` being importable.

**How to run inside Rhino:**
```
RunPythonScript → tests\pcl_hybrid\Twinner.py
```

---

## Build internals

### Manual CMake build (if not using `RUNME_TEST.ps1`)

```powershell
cd tests\pcl_hybrid

cmake -S . -B build_py39 `
  -DCMAKE_TOOLCHAIN_FILE="C:\vcpkg\scripts\buildsystems\vcpkg.cmake" `
  -DBUILD_PYTHON=ON `
  -DPython3_ROOT_DIR="C:\Program Files\Python39" `
  -DPYTHON_EXECUTABLE="C:\Program Files\Python39\python.exe" `
  -DPYTHON_INCLUDE_DIR="C:\Program Files\Python39\include" `
  -DPYTHON_LIBRARY="C:\Program Files\Python39\libs\python39.lib" `
  -DCMAKE_BUILD_TYPE=Release

cmake --build build_py39 --config Release --target pcl_hybrid_py
```

### Key build targets

| Target | Output | Purpose |
|--------|--------|---------|
| `pcl_hybrid_core` | `build_py39\Release\pcl_hybrid_core.lib` | Static C++ library |
| `pcl_hybrid_py` | `build_py39\Release\pcl_hybrid_py.cp39-win_amd64.pyd` | Python 3.9 extension |
| `pcl_hybrid` | `build_py39\Release\pcl_hybrid.exe` | Standalone C++ executable |

### Runtime DLL requirements

The `.pyd` depends on vcpkg DLLs. Ensure these directories are on `PATH` at runtime:

```
C:\vcpkg\installed\x64-windows\bin
tests\pcl_hybrid\build_py39\Release
```

`main.py` handles this automatically. For standalone use, prepend them to `PATH` or call `os.add_dll_directory()` before importing.

---

## Utility scripts

| Script | Purpose |
|--------|---------|
| `runtests.py` | Import smoke test; run with `python runtests.py` from `tests\pcl_hybrid\` |
| `check_deps.py` | Audit `.pyd` for missing DLL imports: `python check_deps.py build_py39\Release\pcl_hybrid_py.cp39-win_amd64.pyd` |
| `run_test_flow.ps1` | Lower-level pipeline script called by `RUNME_TEST.ps1`; supports all the same switches |

---

## Sample data

| File | Description |
|------|-------------|
| `data\pipe and planes.xyz` | 8 000 points — three axis-aligned cylinders (radius ≈ 0.25, i.e. ½" diameter) intersecting three planes. Standard regression dataset for the cylinder fitting algorithm. |
| `data\cyl1.xyz` – `cyl3.xyz` | Single-cylinder test datasets |
| `data\cyl50.xyz` | Dense 50-point cylinder cross-section |

---

## Algorithm notes

Shape fitting uses a **pairwise surface-normal intersection** algorithm:

1. PCL `NormalEstimation` (k = 50 neighbors) computes a surface normal at every point.
2. For each cluster classified as a cylinder, normals are projected onto the cross-section plane perpendicular to the detected axis.
3. Pairs of projected normal lines are intersected. Near-parallel pairs (`|det| < 0.05`) are discarded.
4. The median intersection is computed; the best 60 % of intersections (closest to the median) are averaged to give the cylinder center.
5. The radius is the trimmed mean of per-point distances to that center (outliers beyond `median + 2.5 × MAD` removed).

This approach is robust to the plane/cylinder boundary leakage that corrupts purely algebraic fits.
