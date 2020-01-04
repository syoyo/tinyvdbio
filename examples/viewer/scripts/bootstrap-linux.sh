rm -rf build
mkdir build

curdir=`pwd`

cd build

cmake \
  -DTVDB_VIEWER_WITH_SYSTEM_ZLIB=On \
  -DTVDB_VIEWER_WITH_BLOSC=On \
  -DBLOSC_INCDIR=${curdir}/../../third_party/c-blosc/blosc \
  -DBLOSC_LIBRARY=${curdir}/../../third_party/c-blosc/build/blosc/libblosc.so \
  ..

cd ..

