#!/bin/bash
# Build controller_app from codegen + impl sources

AERPAW_DIR="$(cd "$(dirname "$0")" && pwd)"
CODEGEN="$AERPAW_DIR/codegen"
IMPL="$AERPAW_DIR/impl"
BUILD="$AERPAW_DIR/build"

mkdir -p "$BUILD"

g++ -I/home/kdee/matlab/R2025a/extern/include \
    -I"$CODEGEN" \
    -I"$IMPL" \
    "$IMPL/controller_main.cpp" \
    "$CODEGEN/controller.cpp" \
    "$IMPL/controller_impl.cpp" \
    "$CODEGEN/controller_initialize.cpp" \
    "$CODEGEN/controller_terminate.cpp" \
    -o "$BUILD/controller_app" \
    -lpthread

echo "Built: $BUILD/controller_app"
