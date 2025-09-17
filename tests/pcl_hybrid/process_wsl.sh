#!/bin/bash

# WSL Ubuntu script to process point cloud data
# Usage: process_wsl.sh <input_xyz_file> [cluster_tolerance] [min_cluster_size] [max_cluster_size]
# Reads XYZ file from Windows temp (converted from Windows path to WSL path)
# Writes to ./data/ws_output.json (Windows reads from here via WSL path)

set -e

# Configuration - WSL reads XYZ file, writes JSON results to local data
INPUT_FILE_WIN_PATH="${1:-/mnt/c/temp/pcl_processing/ws_input.xyz}"
CLUSTER_TOLERANCE="${2:-3.0}"
MIN_CLUSTER_SIZE="${3:-50}"
MAX_CLUSTER_SIZE="${4:-1000000}"
OUTPUT_FILE="./data/ws_output.json"
PROCESSOR_PATH="/mnt/c/_LOCAL/GitHub/PC_WIN/tests/pcl_hybrid/main"

# Convert Windows path to WSL path if needed
INPUT_FILE="$INPUT_FILE_WIN_PATH"
if [[ "$INPUT_FILE" == *"C:\\"* ]]; then
    # Convert Windows path C:\path to /mnt/c/path
    INPUT_FILE=$(echo "$INPUT_FILE" | sed 's|C:\\|/mnt/c/|g' | sed 's|\\|/|g')
fi

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Check if input file exists
if [ ! -f "$INPUT_FILE" ]; then
    print_error "Input file not found: $INPUT_FILE"
    exit 1
fi

# Check if processor exists
if [ ! -f "$PROCESSOR_PATH" ]; then
    print_error "Headless processor not found: $PROCESSOR_PATH"
    exit 1
fi

# Make processor executable if needed
if [ ! -x "$PROCESSOR_PATH" ]; then
    print_warning "Making processor executable..."
    chmod +x "$PROCESSOR_PATH"
fi

# Create data directory if it doesn't exist
mkdir -p data

print_status "Processing point cloud data..."
print_status "Input XYZ file: $INPUT_FILE"
print_status "Output JSON file: $OUTPUT_FILE"
print_status "Parameters: tolerance=$CLUSTER_TOLERANCE, min_size=$MIN_CLUSTER_SIZE, max_size=$MAX_CLUSTER_SIZE"

# Run the headless processor with XYZ input and parameters
print_status "Running headless processor..."
# Only pass solver_type argument (currently set to BEST)
SOLVER_TYPE="BEST"
if "$PROCESSOR_PATH" "$INPUT_FILE" "$OUTPUT_FILE"; then
    print_status "Processing completed successfully!"
    print_status "Results written to: $OUTPUT_FILE"
    # Check if output file was created
    if [ -f "$OUTPUT_FILE" ]; then
        file_size=$(stat -c%s "$OUTPUT_FILE" 2>/dev/null || echo "unknown")
        print_status "Output file size: $file_size bytes"
    else
        print_error "Output file was not created!"
        exit 1
    fi
else
    print_error "Headless processor failed!"
    exit 1
fi

print_status "WSL processing script completed successfully!"
