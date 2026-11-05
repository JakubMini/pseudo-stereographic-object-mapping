#!/usr/bin/env bash
# Builds what the engine needs for Android arm64-v8a into third_party/android/:
# OpenCV (the official prebuilt SDK) and Open CASCADE (static, only the modules STEP export needs).
# Run once; it skips whatever is already there. Takes a while: Open CASCADE is big.
set -euo pipefail

ROOT=$(cd "$(dirname "$0")/../.." && pwd)
OUT="$ROOT/third_party/android"
NDK="${ANDROID_NDK_HOME:-$HOME/Library/Android/sdk/ndk/27.1.12297006}"
ABI=arm64-v8a
API=26
OPENCV=4.10.0
OCCT=7_9_3

mkdir -p "$OUT/src"
cd "$OUT/src"

# OpenCV's config finds its libraries relative to the SDK's own layout, so the tree is kept whole.
if [ ! -f "$OUT/opencv/sdk/native/jni/OpenCVConfig.cmake" ]; then
  echo "== OpenCV $OPENCV for Android"
  curl -fsSL -o opencv.zip "https://github.com/opencv/opencv/releases/download/$OPENCV/opencv-$OPENCV-android-sdk.zip"
  rm -rf OpenCV-android-sdk && unzip -q opencv.zip
  rm -rf "$OUT/opencv" && mv OpenCV-android-sdk "$OUT/opencv"
  rm -f opencv.zip
fi

if [ ! -f "$OUT/occt/lib/cmake/opencascade/OpenCASCADEConfig.cmake" ]; then
  echo "== Open CASCADE $OCCT for Android ($ABI)"
  [ -d "OCCT-$OCCT" ] || { curl -fsSL -o occt.tar.gz "https://github.com/Open-Cascade-SAS/OCCT/archive/refs/tags/V$OCCT.tar.gz"; tar xzf occt.tar.gz; rm occt.tar.gz; }
  cmake -S "OCCT-$OCCT" -B occt-build -G Ninja \
    -DCMAKE_TOOLCHAIN_FILE="$NDK/build/cmake/android.toolchain.cmake" \
    -DANDROID_ABI=$ABI -DANDROID_PLATFORM=android-$API -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_LIBRARY_TYPE=Static -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    -DBUILD_MODULE_FoundationClasses=ON -DBUILD_MODULE_ModelingData=ON -DBUILD_MODULE_ModelingAlgorithms=ON \
    -DBUILD_MODULE_DataExchange=ON -DBUILD_MODULE_ApplicationFramework=OFF -DBUILD_MODULE_Visualization=OFF \
    -DBUILD_MODULE_Draw=OFF -DBUILD_MODULE_DETools=OFF -DBUILD_DOC_Overview=OFF -DBUILD_SAMPLES_QT=OFF \
    -DUSE_FREETYPE=OFF -DUSE_TK=OFF -DUSE_FREEIMAGE=OFF -DUSE_OPENGL=OFF -DUSE_GLES2=OFF -DUSE_OPENVR=OFF \
    -DUSE_RAPIDJSON=OFF -DUSE_DRACO=OFF -DUSE_TBB=OFF -DUSE_VTK=OFF -DUSE_XLIB=OFF -DUSE_FFMPEG=OFF \
    -DINSTALL_DIR="$OUT/occt" -DCMAKE_INSTALL_PREFIX="$OUT/occt"
  cmake --build occt-build
  cmake --install occt-build
fi

echo "== done: $OUT"
