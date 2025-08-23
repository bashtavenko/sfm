#!/bin/bash

set -e

# Parse command line argument
BUILD_TYPE="Release"
if [ "$1" = "debug" ]; then
    BUILD_TYPE="Debug"
    echo "Building in Debug mode..."
elif [ "$1" != "" ] && [ "$1" != "release" ]; then
    echo "Usage: $0 [debug|release]"
    echo "  debug   - Build debug version with symbols"
    echo "  release - Build optimized release version (default)"
    exit 1
fi

# Configuration
OPENCV_VERSION="4.11.0"
BUILD_DIR="$HOME/opencv_build_411"
if [ "$BUILD_TYPE" = "Debug" ]; then
    INSTALL_DIR="$BUILD_DIR/install_debug"
    BUILD_SUBDIR="build_debug"
else
    INSTALL_DIR="$BUILD_DIR/install"
    BUILD_SUBDIR="build"
fi

echo "Building OpenCV ${OPENCV_VERSION} in $BUILD_TYPE mode..."

# Create and enter build directory
mkdir -p "$BUILD_DIR"
cd "$BUILD_DIR"

# Download sources if they don't exist
if [ ! -d "opencv-${OPENCV_VERSION}" ]; then
    echo "Downloading OpenCV ${OPENCV_VERSION}..."
    curl -L "https://github.com/opencv/opencv/archive/refs/tags/${OPENCV_VERSION}.tar.gz" -o "opencv-${OPENCV_VERSION}.tar.gz"
    tar -xzf "opencv-${OPENCV_VERSION}.tar.gz"
fi


# Create build directory
mkdir -p "$BUILD_SUBDIR"
cd "$BUILD_SUBDIR"

echo "Configuring CMake for $BUILD_TYPE build..."
cmake \
    -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
    -DBUILD_LIST=calib3d,core,features2d,highgui,imgcodecs,imgproc,video,videoio \
    -DWITH_FFMPEG=ON \
    -DWITH_GTK=OFF \
    -DWITH_QT=ON \
    -DBUILD_SHARED_LIBS=ON \
    -DCMAKE_INSTALL_PREFIX="$INSTALL_DIR" \
    -DBUILD_TESTS=OFF \
    -DBUILD_PERF_TESTS=OFF \
    -DBUILD_EXAMPLES=OFF \
     "../opencv-${OPENCV_VERSION}"

echo "Building OpenCV..."
make -j$(sysctl -n hw.logicalcpu)


echo "Installing OpenCV..."
make install

# Create symlinks for versioned libraries
echo "Creating .so version symlinks..."
cd "$INSTALL_DIR/lib"
for lib in libopencv_*.so.4.11.0; do
    if [ -f "$lib" ]; then
        base=$(basename "$lib" .so.4.11.0)
        ln -sf "$lib" "${base}.so.4.11.0"
        echo "Created symlink: ${base}.so.4.11.0 -> $lib"
    fi
done

echo ""
echo "Build completed successfully!"
echo "Build type: $BUILD_TYPE"
echo "Libraries installed in: $INSTALL_DIR/lib"
echo "Headers installed in: $INSTALL_DIR/include"


if [ "$BUILD_TYPE" = "Debug" ]; then
    echo ""
    echo "Debug build complete. Use this path for debugging:"
    echo "  Libraries: $INSTALL_DIR/lib"
    echo "  Headers: $INSTALL_DIR/include"
fi