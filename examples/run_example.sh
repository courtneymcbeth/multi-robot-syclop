#!/bin/bash
# Script to run Multi-Robot SyCLoP examples

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
BUILD_DIR="$SCRIPT_DIR/../build"
EXECUTABLE="$BUILD_DIR/main_mr_syclop"

# Check if executable exists
if [ ! -f "$EXECUTABLE" ]; then
    echo -e "${RED}Error: Executable not found at $EXECUTABLE${NC}"
    echo "Please build the project first with: cmake --build build"
    exit 1
fi

# Create output directory
OUTPUT_DIR="$SCRIPT_DIR/output"
mkdir -p "$OUTPUT_DIR"

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}Multi-Robot SyCLoP Examples${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Example 1: Simple 2-robot swap
echo -e "${GREEN}Running Example 1: 2 Robots Swap${NC}"
echo "Scenario: Two robots need to swap positions"
echo ""
$EXECUTABLE \
    --input "$SCRIPT_DIR/simple_2robots.yaml" \
    --output "$OUTPUT_DIR/simple_2robots_solution.yaml" \
    --stats "$OUTPUT_DIR/simple_2robots_stats.yaml" \
    --cfg "$SCRIPT_DIR/planner_config.yaml" \
    --planner mr-syclop \
    --timelimit 30

echo ""
echo -e "${BLUE}----------------------------------------${NC}"
echo ""

# Example 2: 3 robots in corridor
echo -e "${GREEN}Running Example 2: 3 Robots Corridor${NC}"
echo "Scenario: Three robots must coordinate in tight spaces"
echo ""
$EXECUTABLE \
    --input "$SCRIPT_DIR/3robots_corridor.yaml" \
    --output "$OUTPUT_DIR/3robots_corridor_solution.yaml" \
    --stats "$OUTPUT_DIR/3robots_corridor_stats.yaml" \
    --cfg "$SCRIPT_DIR/planner_config.yaml" \
    --planner mr-syclop \
    --timelimit 60

echo ""
echo -e "${BLUE}========================================${NC}"
echo -e "${GREEN}Examples complete!${NC}"
echo ""
echo "Results saved in: $OUTPUT_DIR"
echo "  - Solution paths: *_solution.yaml"
echo "  - Statistics: *_stats.yaml"
echo ""
