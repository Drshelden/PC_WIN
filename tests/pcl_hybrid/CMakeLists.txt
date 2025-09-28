cmake_minimum_required(VERSION 3.20)
project(pcl_hybrid LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

option(VISUALIZE "Enable visualization (1=on, 0=off)" ON)
option(BUILD_PYTHON "Build Python bindings (requires pybind11)" OFF)

# Allow user to pass -DCMAKE_TOOLCHAIN_FILE=... when configuring (vcpkg toolchain recommended)
# Example: -DCMAKE_TOOLCHAIN_FILE=C:/Users/you/vcpkg/scripts/buildsystems/vcpkg.cmake

set(PCL_COMPONENTS common io features segmentation)
if(VISUALIZE)
  list(APPEND PCL_COMPONENTS visualization)
endif()

find_package(PCL REQUIRED COMPONENTS ${PCL_COMPONENTS})
find_package(nlohmann_json REQUIRED)

if(NOT PCL_FOUND)
    message(FATAL_ERROR "PCL not found. Make sure you passed -DCMAKE_TOOLCHAIN_FILE to vcpkg's toolchain file.")
endif()

## Use target-scoped include dirs and compile definitions instead of global commands
# (global include_directories / link_directories can cause generator to emit
# plain library names like "common.lib" which don't exist; prefer linking
# the imported targets that PCLConfig provides)

# Apply include directories and compile definitions to the target after creating it

# Source files (adjust if you move files)
set(SRCS
    main.cpp
    utils.cpp
    PCWinRegionGrowing.cpp
    PCWinPointCloud.cpp
    ShapeFinder.cpp
    shapes.cpp
  Settings.cpp
)

# Create a core static library from all sources except main.cpp so we can
# link the same implementation into both the executable and the Python
# extension (avoids duplicate-definition or missing-symbol LNK errors).
set(SRCS_CORE ${SRCS})
list(REMOVE_ITEM SRCS_CORE main.cpp)
add_library(pcl_hybrid_core STATIC ${SRCS_CORE})

add_executable(${PROJECT_NAME} main.cpp)

# Compile-time VISUALIZE macro (numeric)
if(VISUALIZE)
    target_compile_definitions(${PROJECT_NAME} PRIVATE VISUALIZE=1)
else()
    target_compile_definitions(${PROJECT_NAME} PRIVATE VISUALIZE=0)
endif()

# Apply VISUALIZE and common settings to the core library as well
if(VISUALIZE)
  target_compile_definitions(pcl_hybrid_core PRIVATE VISUALIZE=1)
else()
  target_compile_definitions(pcl_hybrid_core PRIVATE VISUALIZE=0)
endif()

# Link PCL and JSON
# Prefer linking the actual import library files discovered by PCLConfig to avoid
# incorrect name-mangling in the generated MSVC project files. Fall back to any
# items listed in ${PCL_LIBRARIES} (these may be imported targets for 3rd-party libs).
## Collect explicit PCL library file paths for the modules we requested
set(PCL_LIB_FILES)
foreach(mod ${PCL_COMPONENTS})
  string(TOUPPER "${mod}" MOD_UPPER)
  # variable that CMake creates in the cache: PCL_<MODULE>_LIBRARY
  set(varName "PCL_${MOD_UPPER}_LIBRARY")
  # If that variable exists, read its value and append if non-empty
  if(DEFINED ${varName})
    set(_libval "${${varName}}")
    if(NOT "${_libval}" STREQUAL "")
      list(APPEND PCL_LIB_FILES "${_libval}")
    endif()
  endif()
endforeach()

## Build a safe list of link items: prefer imported targets (they have IMPORTED_LOCATION),
## otherwise use explicit library file paths discovered in PCL_<MODULE>_LIBRARY.
set(PCL_LINK_ITEMS)
foreach(mod ${PCL_COMPONENTS})
  # prefer imported target name 'pcl_<mod>' if available
  if(TARGET pcl_${mod})
    list(APPEND PCL_LINK_ITEMS pcl_${mod})
  else()
    string(TOUPPER "${mod}" MOD_UPPER)
    set(libvar "PCL_${MOD_UPPER}_LIBRARY")
    if(DEFINED ${libvar} AND NOT "${${libvar}}" STREQUAL "")
      list(APPEND PCL_LINK_ITEMS "${${libvar}}")
    endif()
  endif()
endforeach()

target_link_libraries(${PROJECT_NAME} PRIVATE pcl_hybrid_core)

# Link core implementation to PCL and nlohmann_json
target_link_libraries(pcl_hybrid_core PRIVATE ${PCL_LINK_ITEMS} nlohmann_json::nlohmann_json)

## Apply PCL include dirs and compile definitions to the target (target-scoped)
if(PCL_INCLUDE_DIRS)
  target_include_directories(pcl_hybrid_core PRIVATE ${PCL_INCLUDE_DIRS})
  target_include_directories(${PROJECT_NAME} PRIVATE ${PCL_INCLUDE_DIRS})
endif()
# Ensure project headers (current source dir) are available to targets
target_include_directories(pcl_hybrid_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
if(PCL_DEFINITIONS)
  # PCL_DEFINITIONS may contain items like "-DNAME" or plain "NAME" separated by semicolons.
  # Iterate as a CMake list, strip any leading "-D", and apply each as a proper compile definition
  foreach(_def IN LISTS PCL_DEFINITIONS)
    if(_def)
      # remove an optional leading -D that some PCL configs include
      string(REGEX REPLACE "^-D" "" _def_clean "${_def}")
      # avoid injecting empty or newline-containing strings
      string(REPLACE "\n" "" _def_clean "${_def_clean}")
      if(NOT "${_def_clean}" STREQUAL "")
        target_compile_definitions(pcl_hybrid_core PRIVATE "${_def_clean}")
        target_compile_definitions(${PROJECT_NAME} PRIVATE "${_def_clean}")
      endif()
    endif()
  endforeach()
endif()

# Some helpful output variables
message(STATUS "PCL include dirs: ${PCL_INCLUDE_DIRS}")
message(STATUS "PCL libraries: ${PCL_LIBRARIES}")
message(STATUS "VISUALIZE = ${VISUALIZE}")

# Set runtime output directory for multi-config generators
if(CMAKE_CONFIGURATION_TYPES)
  set_target_properties(${PROJECT_NAME} PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY_${CMAKE_BUILD_TYPE} ${CMAKE_BINARY_DIR}/$<CONFIG>
  )
endif()

# Workaround for MSVC PDB write collisions when parallel cl.exe instances run.
# Adding /FS forces the compiler to serialize PDB writes and avoids C1041 errors.
if(MSVC)
  target_compile_options(${PROJECT_NAME} PRIVATE /FS)
  target_compile_options(pcl_hybrid_core PRIVATE /FS)
endif()

if(BUILD_PYTHON)
  # Try to find pybind11. Prefer CONFIG (vcpkg installs a config file),
  # but fall back to the module mode to be friendlier to other setups.
  find_package(pybind11 CONFIG QUIET)
  if(NOT pybind11_FOUND)
    find_package(pybind11 QUIET)
  endif()

  if(NOT pybind11_FOUND)
    message(WARNING "BUILD_PYTHON=ON but pybind11 was not found. Install pybind11 (via vcpkg) or turn BUILD_PYTHON=OFF.")
  else()
    # Create a python extension that wraps the core PC/Shape API
    pybind11_add_module(pcl_hybrid_py bindings/py_pcl_hybrid.cpp)
    # Ensure the module can include project headers in the source tree
    target_include_directories(pcl_hybrid_py PRIVATE ${CMAKE_CURRENT_SOURCE_DIR} ${PCL_INCLUDE_DIRS})
    # Propagate numeric VISUALIZE compile definition (0 or 1)
    if(VISUALIZE)
      set(PY_VIS_FLAG "VISUALIZE=1")
    else()
      set(PY_VIS_FLAG "VISUALIZE=0")
    endif()
    target_compile_definitions(pcl_hybrid_py PRIVATE ${PY_VIS_FLAG})
  target_link_libraries(pcl_hybrid_py PRIVATE pcl_hybrid_core ${PCL_LINK_ITEMS} nlohmann_json::nlohmann_json)
    if(MSVC)
      target_compile_options(pcl_hybrid_py PRIVATE /FS)
    endif()
  endif()
endif()
