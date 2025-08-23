#!/bin/bash

OPENCV_BUILD="$HOME/opencv_build_411"
THIRD_PARTY_DIR="third_party/opencv"

mkdir -p "$THIRD_PARTY_DIR"
ln -sf "$OPENCV_BUILD/install/include/opencv4" "$THIRD_PARTY_DIR/include"
ln -sf "$OPENCV_BUILD/install_debug/include/opencv4" "$THIRD_PARTY_DIR/include_debug"
ln -sf "$OPENCV_BUILD/install/lib" "$THIRD_PARTY_DIR/lib"
ln -sf "$OPENCV_BUILD/install_debug/lib" "$THIRD_PARTY_DIR/lib_debug"
