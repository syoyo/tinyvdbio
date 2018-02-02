# TinyVDB, header-only C++03 (limited) OpenVDB library.

TinyVDB is header-only C++03 OpenVDB library. Not all features in OpenVDB are implemented.

## Features

* [x] Can be compiled with rather old C++03 compiler.
* [x] Cross-platform(should be at least compilable on Linux, macOS and Windows)
* [x] Limited support of loading OpenVDB data(version 220 or less)
* [x] Simple saving of OpenVDB data.

### TinyVDB only feature

* [x] Support big endian machine(e.g. SPARC, POWER).

## How to use

Simply copy `tinyvdb.h` to your project.

## Optional feature

### Blosc

```
$ git submodule update --init
```

## Notes

Data format


```
+-----------------+
| header          |
+-----------------+
| meta            |
+-----------------+
| grid descriptor |
+-----------------+


+-----------------+   <- grid_pos
| grid meta       |
+-----------------+
| grid data       |
|                 |
+-----------------+


block_pos
end_pos
```

## License

TinyVDB is released under the [Mozilla Public License Version 2.0](https://www.mozilla.org/MPL/2.0/), which is a free, open source, and detailed software license developed and maintained by the Mozilla Foundation. It is a hybrid of the modified [BSD license](https://en.wikipedia.org/wiki/BSD_licenses#3-clause) and the [GNU General Public License](https://en.wikipedia.org/wiki/GNU_General_Public_License) (GPL) that seeks to balance the concerns of proprietary and open source developers.
