#!/bin/bash
# Build controller_app from codegen + impl sources

AERPAW_DIR="$(cd "$(dirname "$0")" && pwd)"
CODEGEN="$AERPAW_DIR/codegen"
IMPL="$AERPAW_DIR/impl"
BUILD="$AERPAW_DIR/build"

mkdir -p "$BUILD"

# Compile all codegen sources (handles any new generated files)
g++ -I/home/kdee/matlab/R2025a/extern/include \
    -I"$CODEGEN" \
    -I"$IMPL" \
    "$IMPL/controller_main.cpp" \
    "$IMPL/controller_impl.cpp" \
    "$CODEGEN"/*.cpp \
    -o "$BUILD/controller_app" \
    -lpthread

echo "Built: $BUILD/controller_app"
