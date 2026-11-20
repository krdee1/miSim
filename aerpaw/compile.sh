#!/bin/bash
set -euo pipefail # stop (and do not report 'Built') if compilation fails
# Build controller_app from codegen + impl sources

AERPAW_DIR="$(cd "$(dirname "$0")" && pwd)"
CODEGEN="$AERPAW_DIR/codegen"
IMPL="$AERPAW_DIR/impl"
BUILD="$AERPAW_DIR/build"
# MATLAB install that generated the code (for tmwtypes.h); override with MATLAB_ROOT=...
MATLAB_ROOT="${MATLAB_ROOT:-/home/kdee/matlab/R2026a}"

mkdir -p "$BUILD"

echo "Compiling controller executable..."

# Compile all codegen sources (handles any new generated files)
g++ -I"$MATLAB_ROOT/extern/include" \
    -I"$CODEGEN" \
    -I"$IMPL" \
    "$IMPL/controller_main.cpp" \
    "$IMPL/controller_impl.cpp" \
    "$CODEGEN"/*.cpp \
    -o "$BUILD/controller_app" \
    -fopenmp \
    -lpthread

echo "Built: $BUILD/controller_app"
