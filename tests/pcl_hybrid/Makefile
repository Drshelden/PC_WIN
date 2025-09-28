TARGET = main
SRCS = /mnt/c/_LOCAL/GitHub/PC_WIN/tests/pcl_hybrid/main.cpp /mnt/c/_LOCAL/GitHub/PC_WIN/tests/pcl_hybrid/utils.cpp /mnt/c/_LOCAL/GitHub/PC_WIN/tests/pcl_hybrid/PCWinRegionGrowing.cpp /mnt/c/_LOCAL/GitHub/PC_WIN/tests/pcl_hybrid/PCWinPointCloud.cpp /mnt/c/_LOCAL/GitHub/PC_WIN/tests/pcl_hybrid/ShapeFinder.cpp /mnt/c/_LOCAL/GitHub/PC_WIN/tests/pcl_hybrid/shapes.cpp /mnt/c/_LOCAL/GitHub/PC_WIN/tests/pcl_hybrid/Settings.cpp
OBJS = $(SRCS:.cpp=.o)
VTK_INCLUDE = /usr/include/vtk-9.1

CXX = g++
# Base flags; optimization or debug flags are appended depending on DEBUG
CXXFLAGS = -std=c++14 -Wall
# Allow toggling visualization from the make command line: `make VISUALIZE=0` or `make VISUALIZE=1`
VISUALIZE ?= 1
ifeq ($(DEBUG),1)
	CXXFLAGS += -g -O0
else
	CXXFLAGS += -O2
endif

# Pass VISUALIZE macro value to the compiler so the code can #if VISUALIZE
CXXFLAGS += -DVISUALIZE=$(VISUALIZE)

# Don't run pkg-config for trivial goals like 'clean' to avoid noisy warnings
ifeq ($(filter clean,$(MAKECMDGOALS)),clean)
	PCL_CFLAGS =
	PCL_LIBS =
	VTK_PKG_LIBS =
	PCL_VERSION =
	PCL_MAJOR =
	PCL_MINOR =
else
	# Query PCL flags from pkg-config (silence pkg-config errors)
	PCL_CFLAGS := $(shell pkg-config --cflags pcl_common 2>/dev/null || true)
	PCL_LIBS   := $(shell pkg-config --libs pcl_io pcl_common pcl_visualization pcl_features pcl_segmentation 2>/dev/null || true)

	# Try to get VTK libs via pkg-config; fallback to common VTK components if pkg-config isn't available
	VTK_PKG_LIBS := $(shell pkg-config --libs vtk 2>/dev/null || true)
	ifeq ($(VTK_PKG_LIBS),)
		VTK_LIBS = -lvtkCommonCore-9.1 -lvtkCommonMath-9.1 -lvtkCommonSystem-9.1 -lvtkCommonTransforms-9.1 -lvtkCommonDataModel-9.1 \
				   -lvtkCommonExecutionModel-9.1 -lvtkCommonMisc-9.1 -lvtkRenderingCore-9.1 -lvtkRenderingOpenGL2-9.1 -lvtkGUISupportQt-9.1
	else
		VTK_LIBS = $(VTK_PKG_LIBS)
	endif

	# Detect PCL version with pkg-config (e.g. "1.14.0")
	PCL_VERSION := $(shell pkg-config --modversion pcl_common 2>/dev/null || true)
	PCL_MAJOR   := $(word 1,$(subst ., ,$(PCL_VERSION)))
	PCL_MINOR   := $(word 2,$(subst ., ,$(PCL_VERSION)))

	# Pass version macros to the compiler
	CXXFLAGS += -DPCL_MAJOR_VERSION=$(PCL_MAJOR) -DPCL_MINOR_VERSION=$(PCL_MINOR)
endif
all: $(TARGET)

$(TARGET): $(SRCS)
	$(CXX) $(CXXFLAGS) -I$(VTK_INCLUDE) $(PCL_CFLAGS) -o $(TARGET) $(SRCS) $(PCL_LIBS) $(VTK_LIBS)

clean:
	rm -f $(TARGET)
