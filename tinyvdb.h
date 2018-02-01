///////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2018 Syoyo Fujita
// Copyright (c) 2012-2018 DreamWorks Animation LLC
//
// All rights reserved. This software is distributed under the
// Mozilla Public License 2.0 ( http://www.mozilla.org/MPL/2.0/ )
//
// Redistributions of source code must retain the above copyright
// and license notice and the following restrictions and disclaimer.
//
// *     Neither the name of Syoyo Fujita and DreamWorks Animation nor the names
// of its contributors may be used to endorse or promote products derived from
// this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
// IN NO EVENT SHALL THE COPYRIGHT HOLDERS' AND CONTRIBUTORS' AGGREGATE
// LIABILITY FOR ALL CLAIMS REGARDLESS OF THEIR BASIS EXCEED US$250.00.
//
///////////////////////////////////////////////////////////////////////////
#ifndef TINY_VDB_H_
#define TINY_VDB_H_

//#include <cstdint>

namespace tinyvdb {

///
/// Parse VDB header from a file.
/// Returns true upon success.
/// Returns false when failed to parse VDB header and store error message to
/// `err`.
///
bool ParseVDBHeader(const std::string &filename, std::string *err);

///
/// Parse VDB header from memory.
/// Returns true upon success.
/// Returns false when failed to parse VDB header and store error message to
/// `err`.
///
bool ParseVDBHeader(const unsigned char *data, const size_t len,
                              std::string *err);

}  // namespace tinyvdb

#ifdef TINYVDB_IMPLEMENTATION

#include <iostream>

namespace tinyvdb {

#if __cplusplus > 199711L
// C++11
typedef uint64_t tinvdb_uint64;
typedef int64_t tinyvdb_int64;
#else
// Although `long long` is not a standard type pre C++11, assume it is defined
// as a compiler's extension.
#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wc++11-long-long"
#endif
typedef unsigned long long tinyvdb_uint64;
typedef long long tinyvdb_int64;
#ifdef __clang__
#pragma clang diagnostic pop
#endif
#endif


const int kOPENVDB_MAGIC = 0x56444220;

// HACK
#define MINIZ_LITTLE_ENDIAN (1)

///
/// TinyVDB file version.
///
const unsigned int kTINYVDB_FILE_VERSION = 222;

/// Notable file format version numbers
enum {
    OPENVDB_FILE_VERSION_ROOTNODE_MAP = 213,
    OPENVDB_FILE_VERSION_INTERNALNODE_COMPRESSION = 214,
    OPENVDB_FILE_VERSION_SIMPLIFIED_GRID_TYPENAME = 215,
    OPENVDB_FILE_VERSION_GRID_INSTANCING = 216, 
    OPENVDB_FILE_VERSION_BOOL_LEAF_OPTIMIZATION = 217,
    OPENVDB_FILE_VERSION_BOOST_UUID = 218,
    OPENVDB_FILE_VERSION_NO_GRIDMAP = 219,
    OPENVDB_FILE_VERSION_NEW_TRANSFORM = 219,
    OPENVDB_FILE_VERSION_SELECTIVE_COMPRESSION = 220,
    OPENVDB_FILE_VERSION_FLOAT_FRUSTUM_BBOX = 221,
    OPENVDB_FILE_VERSION_NODE_MASK_COMPRESSION = 222,
    OPENVDB_FILE_VERSION_BLOSC_COMPRESSION = 223,
    OPENVDB_FILE_VERSION_POINT_INDEX_GRID = 223,
    OPENVDB_FILE_VERSION_MULTIPASS_IO = 224
};

static inline void swap2(unsigned short *val) {
#ifdef MINIZ_LITTLE_ENDIAN
  (void)val;
#else
  unsigned short tmp = *val;
  unsigned char *dst = reinterpret_cast<unsigned char *>(val);
  unsigned char *src = reinterpret_cast<unsigned char *>(&tmp);

  dst[0] = src[1];
  dst[1] = src[0];
#endif
}

static inline void swap4(unsigned int *val) {
#ifdef MINIZ_LITTLE_ENDIAN
  (void)val;
#else
  unsigned int tmp = *val;
  unsigned char *dst = reinterpret_cast<unsigned char *>(val);
  unsigned char *src = reinterpret_cast<unsigned char *>(&tmp);

  dst[0] = src[3];
  dst[1] = src[2];
  dst[2] = src[1];
  dst[3] = src[0];
#endif
}

static inline void swap4(int *val) {
#ifdef MINIZ_LITTLE_ENDIAN
  (void)val;
#else
  int tmp = *val;
  unsigned char *dst = reinterpret_cast<unsigned char *>(val);
  unsigned char *src = reinterpret_cast<unsigned char *>(&tmp);

  dst[0] = src[3];
  dst[1] = src[2];
  dst[2] = src[1];
  dst[3] = src[0];
#endif
}

static inline void swap8(tinyvdb::tinyvdb_uint64 *val) {
#ifdef MINIZ_LITTLE_ENDIAN
  (void)val;
#else
  tinyvdb::tinyvdb_uint64 tmp = (*val);
  unsigned char *dst = reinterpret_cast<unsigned char *>(val);
  unsigned char *src = reinterpret_cast<unsigned char *>(&tmp);

  dst[0] = src[7];
  dst[1] = src[6];
  dst[2] = src[5];
  dst[3] = src[4];
  dst[4] = src[3];
  dst[5] = src[2];
  dst[6] = src[1];
  dst[7] = src[0];
#endif
}

static bool ReadMeta(std::ifstream &is)
{
  // Read the number of metadata items.
  int count = 0;
  is.read(reinterpret_cast<char *>(&count), sizeof(int));
  //swap4(&count);
  
  std::cout << "meta = " << count << std::endl;

  return true;

}

bool ParseVDBHeader(const std::string &filename, std::string *err) {

  std::ifstream ifs(filename.c_str(), std::ifstream::binary);
  if (!ifs) {
    if (err) {
      (*err) = "File not found or cannot open file : " + filename;
    }
    return false;
  }

  int64_t magic;

  // [0:7] magic number
  if (!ifs.read(reinterpret_cast<char*>(&magic), 8)) {
    return EXIT_FAILURE;
  }

  if (magic == kOPENVDB_MAGIC) {
    std::cout << "bingo!" << std::endl;
  }

  // [8:11] version 
  unsigned int file_version;
  if (!ifs.read(reinterpret_cast<char*>(&file_version), sizeof(unsigned int))) {
    return EXIT_FAILURE;
  }

  std::cout << "File version: " << file_version << std::endl;

  // Read the library version numbers (not stored prior to file format version 211).
  if (file_version >= 211) {
      unsigned int version;
      ifs.read(reinterpret_cast<char*>(&version), sizeof(unsigned int));
      std::cout << "major version : " << version << std::endl;
      ifs.read(reinterpret_cast<char*>(&version), sizeof(unsigned int));
      std::cout << "minor version : " << version << std::endl;
  }

  // Read the flag indicating whether the stream supports partial reading.
  // (Versions prior to 212 have no flag because they always supported partial reading.)
  if (file_version >= 212) {
      char hasGridOffsets;
      ifs.read(&hasGridOffsets, sizeof(char));
      std::cout << "InputHasGridOffsets = " << (hasGridOffsets ? " yes " : " no ") << std::endl;
  }

  // 5) Read the flag that indicates whether data is compressed.
  //    (From version 222 on, compression information is stored per grid.)
  //mCompression = DEFAULT_COMPRESSION_FLAGS;
  //if (file_version < OPENVDB_FILE_VERSION_BLOSC_COMPRESSION) {
  //    // Prior to the introduction of Blosc, ZLIB was the default compression scheme.
  //    mCompression = (COMPRESS_ZIP | COMPRESS_ACTIVE_MASK);
  //}
  if (file_version >= OPENVDB_FILE_VERSION_SELECTIVE_COMPRESSION &&
      file_version < OPENVDB_FILE_VERSION_NODE_MASK_COMPRESSION)
  {
      char isCompressed;
      ifs.read(&isCompressed, sizeof(char));
      std::cout << "Compression : " << (isCompressed != 0 ? "zip" : "none") << std::endl;
  }

  // 6) Read the 16-byte(128-bit) uuid.
  if (file_version >= OPENVDB_FILE_VERSION_BOOST_UUID) {
    // ASCII UUID = 32 chars + 4 '-''s = 36 bytes.
    char uuid[36];
    ifs.read(uuid, 36);
    // TODO(syoyo): Store UUID somewhere.
    std::cout << "uuid: " << uuid << std::endl;
  } else {
    char uuid[16];
    ifs.read(uuid, 16);
    // TODO(syoyo): Store UUID somewhere.
    std::cout << "uuid: " << uuid << std::endl;
  }

  {
    bool ret = ReadMeta(ifs);
    std::cout << "meta: " << ret << std::endl;
  }

  return true;
}

bool ParseVDBHeader(const unsigned char *data, const size_t len,
                              std::string *err) {
  (void)data;
  (void)len;
  (void)err;

  return false;
}

}  // namespace tinyvdb

#endif

#endif  // TINY_VDB_H_
