# Repository Cleanup Proposal (No Changes Applied Yet)

## Scope
This proposal focuses on cleanup and reorganization for [tests/pcl_hybrid](tests/pcl_hybrid), which currently mixes source code, build artifacts, diagnostics, scripts, and generated outputs.

No file moves or deletions have been performed yet.

## Goals
1. Reduce repository noise and merge churn.
2. Keep only source-of-truth files in git.
3. Make build/test workflow easier to understand.
4. Preserve reproducibility through explicit scripts and fixture data.

## Current Observations
1. Build output folders are versioned:
	 [tests/pcl_hybrid/build](tests/pcl_hybrid/build), [tests/pcl_hybrid/build_py3](tests/pcl_hybrid/build_py3), [tests/pcl_hybrid/build_py39](tests/pcl_hybrid/build_py39)
2. Cache and generated files appear versioned:
	 [tests/pcl_hybrid/__pycache__](tests/pcl_hybrid/__pycache__), [tests/pcl_hybrid/build.log](tests/pcl_hybrid/build.log), compiled binary [tests/pcl_hybrid/main](tests/pcl_hybrid/main)
3. The folder has multiple one-off/legacy scripts that overlap in purpose.
4. There is no root .gitignore detected in the current workspace scan.

## Proposed Delete Candidates

### Safe to remove from git (generated artifacts)
1. [tests/pcl_hybrid/build](tests/pcl_hybrid/build)
2. [tests/pcl_hybrid/build_py3](tests/pcl_hybrid/build_py3)
3. [tests/pcl_hybrid/build_py39](tests/pcl_hybrid/build_py39)
4. [tests/pcl_hybrid/__pycache__](tests/pcl_hybrid/__pycache__)
5. [tests/pcl_hybrid/build.log](tests/pcl_hybrid/build.log)
6. [tests/pcl_hybrid/main](tests/pcl_hybrid/main)

### Remove or relocate after verification (review-first)
1. [tests/pcl_hybrid/temp.py](tests/pcl_hybrid/temp.py)
2. [tests/pcl_hybrid/temp_compile.py](tests/pcl_hybrid/temp_compile.py)
3. [tests/pcl_hybrid/bindingTest.py](tests/pcl_hybrid/bindingTest.py)
4. [tests/pcl_hybrid/check_compile.bat](tests/pcl_hybrid/check_compile.bat)
5. [tests/pcl_hybrid/check_dlls.bat](tests/pcl_hybrid/check_dlls.bat)
6. [tests/pcl_hybrid/Twinner.rui](tests/pcl_hybrid/Twinner.rui)

### Data outputs to move out of fixture set (if generated)
1. [tests/pcl_hybrid/data/output.json](tests/pcl_hybrid/data/output.json)
2. [tests/pcl_hybrid/data/output_py.json](tests/pcl_hybrid/data/output_py.json)
3. [tests/pcl_hybrid/data/out2.json](tests/pcl_hybrid/data/out2.json)

## Proposed Target Layout
Keep this lightweight and incremental. Phase 1 can be done with almost zero risk.

```text
tests/pcl_hybrid/
	CMakeLists.txt
	README.md
	src/
		main.cpp
		PCWinPointCloud.cpp
		PCWinRegionGrowing.cpp
		ShapeFinder.cpp
		shapes.cpp
		Settings.cpp
		utils.cpp
	include/
		PCWinPointCloud.h
		PCWinRegionGrowing.h
		ShapeFinder.h
		shapes.h
		Settings.h
		utils.h
	bindings/
		py_pcl_hybrid.cpp
	python/
		main.py
		example.py
		rhino_client.py
	scripts/
		build_windows.ps1
		build_windows_ci.ps1
		build_windows_ci.bat
		build_py39.bat
		process_wsl.sh
		check_deps.py
	tests/
		test_import_buffer.py
		test_import_pcl_hybrid.py
		runtests.py
	data/
		samples/
		generated/   (gitignored)
```

## Script Consolidation Proposal
1. Keep one local build entrypoint per platform and one CI entrypoint.
2. Keep only one dependency diagnostic script (prefer [tests/pcl_hybrid/check_deps.py](tests/pcl_hybrid/check_deps.py)).
3. Move scratch/experimental scripts to [tests/pcl_hybrid/misc](tests/pcl_hybrid/misc) or archive branch.

## Recommended .gitignore (Draft)
Create a root .gitignore and include at least:

```gitignore
# Python
__pycache__/
*.py[cod]
*.pyd

# CMake / build outputs
build/
build*/
CMakeCache.txt
CMakeFiles/
cmake_install.cmake
*.sln
*.vcxproj*
*.tlog
*.obj
*.pdb
*.ilk
*.exp
*.lib
*.dll
*.exe

# Logs
*.log

# Test/generated output data
tests/pcl_hybrid/data/generated/
tests/pcl_hybrid/data/output*.json
tests/pcl_hybrid/data/out*.json

# VS Code local
.vscode/*.log
```

Adjust patterns if you intentionally commit specific generated binaries.

## Phased Execution Plan

### Phase 1 (Low risk, high impact)
1. Add .gitignore.
2. Remove generated build trees from git tracking.
3. Remove __pycache__ and logs from git tracking.
4. Keep all source/script files in place.

### Phase 2 (Structure cleanup)
1. Introduce src/include/python/tests/scripts folders.
2. Move files in small batches with build checks after each batch.
3. Update CMake include paths and script references.

### Phase 3 (Script and data curation)
1. Consolidate duplicate scripts.
2. Split fixture data from generated outputs.
3. Archive or delete obsolete files after one release cycle.

## Validation Checklist (when executing later)
1. CMake configure + build succeeds on Windows.
2. Python binding imports successfully from clean checkout.
3. Smoke tests pass:
	 [tests/pcl_hybrid/runtests.py](tests/pcl_hybrid/runtests.py)
4. No required fixture data was removed.
5. CI path remains functional.

## Decision Points Before Execution
1. Should generated JSON outputs be kept as golden regression fixtures?
2. Which Rhino scripts are authoritative: .01/.02/[tests/pcl_hybrid/Rhino3DPO.py](tests/pcl_hybrid/Rhino3DPO.py)?
3. Do you want to keep any prebuilt binaries in git for convenience?

