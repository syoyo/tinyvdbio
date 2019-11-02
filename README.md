# TinyVDBIO, header-only C++ OpenVDB IO library.

**WARNIG! This is still experimental project and WORK IN PROGRESS project!! not working yet**

TinyVDBIO is header-only C++ OpenVDB IO library. Not all OpenVDB format are supported. Also, TinyVDBIO does not provide non-IO related features(e.g. volume op, iso-surface generation).

TinyVDBIO is good for not only your graphics application, but also HPC visualization tools.

## Features

* [x] Can be compiled with rather old C++03 compiler.
  * [x] C++11 or later is recommended though.
* [x] Big endian support(e.g. Power, SPARC)
* [x] Cross-platform(should be at least compilable on Linux, macOS and Windows)
* [x] Limited support of loading OpenVDB data(version from 220 to 223 are supported)
  * [x] ZIP compression
  * [ ] BLOSC compression
* [ ] Simple saving of OpenVDB data.

### TinyVDB only feature

* [ ] Support big endian machine(e.g. SPARC, POWER).
  * Will be supported soon!

## Limitation on reading OpenVDB file with TinyVDB

File version less than 220(`OPENVDB_FILE_VERSION_SELECTIVE_COMPRESSION`) is not supported.

Currently, only `FloatTree`(`tree::Tree4<float,       5, 4, 3>::Type`) topology is supported.
(At least example VDB files at http://www.openvdb.org/download/ could be read in TinyVDB)

## TODO

* [ ] Support points.
* [ ] Support various topology type.
* [ ] Support Multipass IO version(224)
* [ ] mmap based accesss for larger data set.

## How to use

Simply copy `tinyvdbio.h` and `miniz.c` and `miniz.h` to your project.

## Optional feature

TinyVDBIO use `miniz` as a default zip compression library.
You can define `TINYVDBIO_USE_SYSTEM_ZLIB` for system provided zlib library.

## Example

```
// Uncomment this if you want to use system provided zlib library.
// #define TINYVDBIO_USE_SYSTEM_ZLIB
// #include <zlib.h>

// define this only in *one* .cc file.
#define TINYVDBIO_IMPLEMENTATION
#include "tinyvdbio.h"
```

### Blosc(T.B.W.)

```
$ git submodule update --init
$ cd third_party/c-blosc/
$ rm -rf build
$ mkdir build
$ cd build
$ cmake ..
$ make
```

## Notes

### Terms

`background` is a uniform constant value used when there is no voxel data.

`Node` is composed of Root, Internal and Leaf.
Leaf contains actual voxel data.

Root and Internal node have `Value` or pointer to child node, where `Value` is a constant value for the node(i.e. 1x1x1 voxel data).

There are two bit masks, `child mask` and `value mask`, for each `Node`.


## License

TinyVDBIO is released under the [Mozilla Public License Version 2.0](https://www.mozilla.org/MPL/2.0/), which is a free, open source, and detailed software license developed and maintained by the Mozilla Foundation. It is a hybrid of the modified [BSD license](https://en.wikipedia.org/wiki/BSD_licenses#3-clause) and the [GNU General Public License](https://en.wikipedia.org/wiki/GNU_General_Public_License) (GPL) that seeks to balance the concerns of proprietary and open source developers.

### Notes on patent

TinyVDB uses some code from OpenVDB related to IO, Archive and Tree. According to MPL2.0, Modifing source code may loose patent grant from original contributor(in this case, DreamWorks).

At this point, it looks there is no claimed patent(including application or pending phase) for hierarchical grid representation by DreamWorks.
