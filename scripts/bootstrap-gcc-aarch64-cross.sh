curdir=`pwd`

builddir=${curdir}/build-aarch64-cross

rm -rf ${builddir}
mkdir ${builddir}

cd ${builddir} && cmake \
  -DCMAKE_TOOLCHAIN_FILE=cmake/aarch64-linux-gnu.toolchain \
  -DCMAKE_VERBOSE_MAKEFILE=1 \
  -DTINYVDBIO_BUILD_TESTS=Off \
  -DTINYVDBIO_BUILD_EXAMPLES=Off \
  ..

