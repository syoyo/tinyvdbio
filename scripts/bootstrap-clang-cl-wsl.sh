builddir=`pwd`/build-clang-cl
rm -rf ${builddir}
mkdir ${builddir}

cd ${builddir}
cmake -G Ninja \
  -DCMAKE_TOOLCHAIN_FILE="cmake/clang-cl-msvc-wsl.cmake" \
  -DCMAKE_BUILD_TYPE=Release \
  -DHOST_ARCH=x64 \
  -DLLVM_NATIVE_TOOLCHAIN="/home/syoyo/local/clang+llvm-10.0.0-x86_64-linux-gnu-ubuntu-18.04/" \
  -DMSVC_BASE:FILEPATH="/mnt/c/Users/syoyo/msvc/MSVC" \
  -DWINSDK_BASE="/mnt/c/Users/syoyo/msvc/sdk" \
  -DWINSDK_VER="10.0.18362.0" \
  ..

