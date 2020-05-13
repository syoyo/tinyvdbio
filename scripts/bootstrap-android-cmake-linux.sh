#!/bin/bash

# Edit path to Android NDK

# Use ANDROID_SDK_HOME environment
ANDROID_NDK_ROOT=$ANDROID_SDK_ROOT/ndk-bundle

# default installation directory of NDK through old? Android Studio
# ANDROID_NDK_ROOT=$HOME/Android/Sdk/ndk-bundle

# Set your own path
#ANDROID_NDK_ROOT=$HOME/local/android-ndk-r16b/

# CMake 3.6 or later required.
CMAKE_BIN=cmake

rm -rf build-android
mkdir build-android
cd build-android

$CMAKE_BIN -G Ninja -DCMAKE_TOOLCHAIN_FILE=$ANDROID_NDK_ROOT/build/cmake/android.toolchain.cmake \
  -DANDROID_ABI=arm64-v8a \
  -DANDROID_NATIVE_API_LEVEL=24 \
  -DANDROID_ARM_MODE=arm \
  -DANDROID_ARM_NEON=TRUE \
  -DANDROID_STL=c++_shared \
  ..

cd ..
