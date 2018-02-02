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

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpadded"
#endif

class
GridDescriptor
{
 public:
  GridDescriptor();
  GridDescriptor(const std::string &name, const std::string &grid_type, bool save_float_as_half = false);
  //GridDescriptor(const GridDescriptor &rhs);
  //GridDescriptor& operator=(const GridDescriptor &rhs);
  ~GridDescriptor();

  const std::string &GridName() const {
    return grid_name_;
  }

  tinyvdb_int64 GridPos() const {
    return grid_pos_;
  }

  tinyvdb_int64 BlockPos() const {
    return block_pos_;
  }

  tinyvdb_int64 EndPos() const {
    return end_pos_;
  }


  static std::string AddSuffix(const std::string &name, int n);
  static std::string StripSuffix(const std::string &name);

  ///
  /// Read GridDescriptor from a stream.
  ///
  void Read(std::ifstream &is, const unsigned int n);
 
 private:
  std::string grid_name_;
  std::string unique_name_;
  std::string instance_parent_name_;
  std::string grid_type_;

  bool save_float_as_half_; // use fp16?
  tinyvdb_int64 grid_pos_;
  tinyvdb_int64 block_pos_;
  tinyvdb_int64 end_pos_;
};

#ifdef __clang__
#pragma clang diagnostic pop
#endif

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


///
/// Write VDB data to a file.
///
bool SaveVDB(const std::string &filename, std::string *err);

}  // namespace tinyvdb

#ifdef TINYVDB_IMPLEMENTATION

#include <iostream>
#include <sstream>
#include <vector>
#include <map>

namespace tinyvdb {

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

enum {
  COMPRESS_NONE           = 0,
  COMPRESS_ZIP            = 0x1,
  COMPRESS_ACTIVE_MASK    = 0x2,
  COMPRESS_BLOSC          = 0x4
};

namespace {

// In order not to break backward compatibility with existing VDB files,
// grids stored using 16-bit half floats are flagged by adding the following
// suffix to the grid's type name on output.  The suffix is removed on input
// and the grid's "save float as half" flag set accordingly.
const char* HALF_FLOAT_TYPENAME_SUFFIX = "_HalfFloat";

const char* SEP = "\x1e"; // ASCII "record separator"

}

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

static inline void swap8(tinyvdb::tinyvdb_int64 *val) {
#ifdef MINIZ_LITTLE_ENDIAN
  (void)val;
#else
  tinyvdb::tinyvdb_int64 tmp = (*val);
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

static inline std::string ReadString(std::istream &is) {
  unsigned int size;
  is.read(reinterpret_cast<char *>(&size), sizeof(unsigned int));
  std::string buffer(size, ' ');
  if (size > 0) is.read(&buffer[0], size);
  return buffer;
}

static inline void WriteString(std::ostream &os, const std::string &name) {
  unsigned int size = static_cast<unsigned int>(name.size());
  os.write(reinterpret_cast<char *>(&size), sizeof(unsigned int));
  os.write(&name[0], size);
}

static inline bool ReadBool(std::istream &is) {
  char c = 0;
  unsigned int size;
  is.read(reinterpret_cast<char *>(&size), sizeof(unsigned int));
  if (size == 1) {
    is.read(&c, 1);
  }
  return bool(c);
}

static inline float ReadFloat(std::istream &is) {
  float f = 0.0f;
  unsigned int size;
  is.read(reinterpret_cast<char *>(&size), sizeof(unsigned int));
  if (size == sizeof(float)) {
    is.read(reinterpret_cast<char*>(&f), sizeof(float));
    swap4(reinterpret_cast<unsigned int *>(&f));
  }
  return f;
}

static inline void ReadVec3i(std::istream &is, int v[3]) {
  unsigned int size;
  is.read(reinterpret_cast<char *>(&size), sizeof(unsigned int));
  if (size == 3 * sizeof(int)) {
    is.read(reinterpret_cast<char*>(v), 3 * sizeof(int));
    swap4(&v[0]);
    swap4(&v[1]);
    swap4(&v[2]);
  }
}

static inline tinyvdb_int64 ReadInt64(std::istream &is) {
  tinyvdb_int64 i64 = 0;
  unsigned int size;
  is.read(reinterpret_cast<char *>(&size), sizeof(unsigned int));
  if (size == 8) {
    is.read(reinterpret_cast<char*>(&i64), 8);
    swap8(&i64);
  }
  return i64;
}

// https://stackoverflow.com/questions/874134/find-if-string-ends-with-another-string-in-c
static inline bool EndsWidth(std::string const &value, std::string const &ending)
{
  if (ending.size() > value.size()) return false;
  return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}


GridDescriptor::GridDescriptor():
  save_float_as_half_(false),
  grid_pos_(0),
  block_pos_(0),
  end_pos_(0) {
}

GridDescriptor::GridDescriptor(const std::string &name, const std::string &grid_type, bool save_float_as_half):
  grid_name_(StripSuffix(name)),
  unique_name_(name),
  grid_type_(grid_type),
  save_float_as_half_(save_float_as_half),
  grid_pos_(0),
  block_pos_(0),
  end_pos_(0) {
}

//GridDescriptor::GridDescriptor(const GridDescriptor &rhs) {
//}

GridDescriptor::~GridDescriptor() {}

std::string GridDescriptor::AddSuffix(const std::string &name, int n)
{
  std::ostringstream ss;
  ss << name << SEP << n;
  return ss.str();
}

std::string GridDescriptor::StripSuffix(const std::string& name)
{
    return name.substr(0, name.find(SEP));
}

void GridDescriptor::Read(std::ifstream &is, const unsigned int file_version) {
  unique_name_ = ReadString(is);
  grid_name_ = StripSuffix(unique_name_);

  grid_type_ = ReadString(is);

  if (EndsWidth(grid_type_, HALF_FLOAT_TYPENAME_SUFFIX)) {
    save_float_as_half_ = true;
    // strip suffix
    std::string tmp = grid_type_.substr(0, grid_type_.find(HALF_FLOAT_TYPENAME_SUFFIX));
    grid_type_ = tmp;
  }

  std::cout << "grid_type = " << grid_type_ << std::endl;
  std::cout << "half = " << save_float_as_half_ << std::endl;

  if (file_version >= OPENVDB_FILE_VERSION_GRID_INSTANCING) {
    instance_parent_name_ = ReadString(is);
    std::cout << "instance_parent_name = " << instance_parent_name_ << std::endl;
  }

  // Create the grid of the type if it has been registered.
  //if (!GridBase::isRegistered(mGridType)) {
  //    OPENVDB_THROW(LookupError, "Cannot read grid." <<
  //        " Grid type " << mGridType << " is not registered.");
  //}
  // else
  //GridBase::Ptr grid = GridBase::createGrid(mGridType);
  //if (grid) grid->setSaveFloatAsHalf(mSaveFloatAsHalf);

  // Read in the offsets.
  is.read(reinterpret_cast<char*>(&grid_pos_), sizeof(tinyvdb_int64));
  is.read(reinterpret_cast<char*>(&block_pos_), sizeof(tinyvdb_int64));
  is.read(reinterpret_cast<char*>(&end_pos_), sizeof(tinyvdb_int64));

  std::cout << "grid_pos = " << grid_pos_ << std::endl;
  std::cout << "block_pos = " << block_pos_ << std::endl;
  std::cout << "end_pos = " << end_pos_ << std::endl;

}


static bool ReadMeta(std::ifstream &is) {
  // Read the number of metadata items.
  int count = 0;
  is.read(reinterpret_cast<char *>(&count), sizeof(int));
  // swap4(&count);

  std::cout << "meta = " << count << std::endl;

  for (int i = 0; i < count; i++) {
    std::string name = ReadString(is);

    // read typename string
    std::string type_name = ReadString(is);

    std::cout << "meta[" << i << "] name = " << name
              << ", type_name = " << type_name << std::endl;

    if (type_name.compare("string") == 0) {
      std::string value = ReadString(is);

      std::cout << "  value = " << value << std::endl;

    } else if (type_name.compare("vec3i") == 0) {
      int v[3];
      ReadVec3i(is, v);

      std::cout << "  value = " << v[0] << ", " << v[1] << ", " << v[2] << std::endl;

    } else if (type_name.compare("bool") == 0) {

      bool b = ReadBool(is);

      std::cout << "  value = " << b << std::endl;

    } else if (type_name.compare("float") == 0) {

      float f = ReadFloat(is);

      std::cout << "  value = " << f << std::endl;

    } else if (type_name.compare("int64") == 0) {

      tinyvdb_int64 i64 = ReadInt64(is);

      std::cout << "  value = " << i64 << std::endl;
  
    } else {

      // Unknown metadata
      int num_bytes;  
      is.read(reinterpret_cast<char*>(&num_bytes), sizeof(int));
      swap4(&num_bytes);

      std::cout << "  unknown value. size = " << num_bytes << std::endl;

      std::vector<char> data;
      data.resize(size_t(num_bytes));
      is.read(data.data(), num_bytes);
    }
  }

  return true;
}

static void ReadGridDescriptors(std::ifstream &is, const unsigned int file_version, std::map<std::string, GridDescriptor> *gd_map) {

  // Read the number of metadata items.
  int count = 0;
  is.read(reinterpret_cast<char *>(&count), sizeof(int));

  std::cout << "grid descriptors = " << count << std::endl;

  for (int i = 0; i < count; ++i) {
      // Read the grid descriptor.
      GridDescriptor gd;
      gd.Read(is, file_version);

      (*gd_map)[gd.GridName()] = gd;
#if 0
      // Add the descriptor to the dictionary.
      gridDescriptors().insert(std::make_pair(gd.gridName(), gd));

      // Skip forward to the next descriptor.
      gd.seekToEnd(is);
#endif
  }

}

static void ReadTransform(std::ifstream &is)
{
  // Read the type name.
  std::string type = ReadString(is);
  
  std::cout << "type = " << type << std::endl; 
}


static void ReadGrid(std::ifstream &is, const unsigned int file_version, const GridDescriptor &gd) 
{
  if (file_version >= OPENVDB_FILE_VERSION_NODE_MASK_COMPRESSION) {
    unsigned int c = COMPRESS_NONE;
    is.read(reinterpret_cast<char*>(&c), sizeof(unsigned int));
    std::cout << "compression: " << c << std::endl;
    //io::setDataCompression(is, c);
  }

  ReadMeta(is);

  // read transform
  ReadTransform(is);

  // read topology
  // Save the grid's structure.
  //grid->writeTopology(os);

  is.seekg(gd.GridPos());

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
  if (!ifs.read(reinterpret_cast<char *>(&magic), 8)) {
    return EXIT_FAILURE;
  }

  if (magic == kOPENVDB_MAGIC) {
    std::cout << "bingo!" << std::endl;
  }

  // [8:11] version
  unsigned int file_version;
  if (!ifs.read(reinterpret_cast<char *>(&file_version),
                sizeof(unsigned int))) {
    return EXIT_FAILURE;
  }

  std::cout << "File version: " << file_version << std::endl;

  // Read the library version numbers (not stored prior to file format version
  // 211).
  if (file_version >= 211) {
    unsigned int version;
    ifs.read(reinterpret_cast<char *>(&version), sizeof(unsigned int));
    std::cout << "major version : " << version << std::endl;
    ifs.read(reinterpret_cast<char *>(&version), sizeof(unsigned int));
    std::cout << "minor version : " << version << std::endl;
  }

  // Read the flag indicating whether the stream supports partial reading.
  // (Versions prior to 212 have no flag because they always supported partial
  // reading.)
  char has_grid_offsets = 0;
  if (file_version >= 212) {
    ifs.read(&has_grid_offsets, sizeof(char));
    std::cout << "InputHasGridOffsets = " << (has_grid_offsets ? " yes " : " no ")
              << std::endl;
  }

  // 5) Read the flag that indicates whether data is compressed.
  //    (From version 222 on, compression information is stored per grid.)
  // mCompression = DEFAULT_COMPRESSION_FLAGS;
  // if (file_version < OPENVDB_FILE_VERSION_BLOSC_COMPRESSION) {
  //    // Prior to the introduction of Blosc, ZLIB was the default compression
  //    scheme. mCompression = (COMPRESS_ZIP | COMPRESS_ACTIVE_MASK);
  //}
  char isCompressed = 0;
  if (file_version >= OPENVDB_FILE_VERSION_SELECTIVE_COMPRESSION &&
      file_version < OPENVDB_FILE_VERSION_NODE_MASK_COMPRESSION) {
    ifs.read(&isCompressed, sizeof(char));
    std::cout << "Compression : " << (isCompressed != 0 ? "zip" : "none")
              << std::endl;
  }

  // 6) Read the 16-byte(128-bit) uuid.
  if (file_version >= OPENVDB_FILE_VERSION_BOOST_UUID) {
    // ASCII UUID = 32 chars + 4 '-''s = 36 bytes.
    char uuid[36];
    ifs.read(uuid, 36);
    std::string uuid_string = std::string(uuid, 36);
    // TODO(syoyo): Store UUID somewhere.
    std::cout << "uuid: " << uuid_string << std::endl;
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

  std::map<std::string, GridDescriptor> gd_map;

  if (has_grid_offsets) {
    ReadGridDescriptors(ifs, file_version, &gd_map);
  } else {

  }

  // fixme
  std::map<std::string, GridDescriptor>::iterator it(gd_map.begin());
  //std::map<std::string, GridDescriptor>::iterator itEnd(gd_map.end());

  //for (; it != itEnd; it++) {
  ReadGrid(ifs, file_version, it->second);
  //}

  return true;
}

bool ParseVDBHeader(const unsigned char *data, const size_t len,
                    std::string *err) {
  (void)data;
  (void)len;
  (void)err;

  return false;
}

static bool WriteVDBHeader(std::ostream &os) {

  // [0:7] magic number
  tinyvdb_int64 magic = kOPENVDB_MAGIC;
  os.write(reinterpret_cast<char *>(&magic), 8);

  // [8:11] version
  unsigned int file_version = OPENVDB_FILE_VERSION_NEW_TRANSFORM;
  os.write(reinterpret_cast<char *>(&file_version), sizeof(unsigned int));

#if 0

  std::cout << "File version: " << file_version << std::endl;

  // Read the library version numbers (not stored prior to file format version
  // 211).
  if (file_version >= 211) {
    unsigned int version;
    ifs.read(reinterpret_cast<char *>(&version), sizeof(unsigned int));
    std::cout << "major version : " << version << std::endl;
    ifs.read(reinterpret_cast<char *>(&version), sizeof(unsigned int));
    std::cout << "minor version : " << version << std::endl;
  }

  // Read the flag indicating whether the stream supports partial reading.
  // (Versions prior to 212 have no flag because they always supported partial
  // reading.)
  char has_grid_offsets = 0;
  if (file_version >= 212) {
    ifs.read(&has_grid_offsets, sizeof(char));
    std::cout << "InputHasGridOffsets = " << (has_grid_offsets ? " yes " : " no ")
              << std::endl;
  }

  // 5) Read the flag that indicates whether data is compressed.
  //    (From version 222 on, compression information is stored per grid.)
  // mCompression = DEFAULT_COMPRESSION_FLAGS;
  // if (file_version < OPENVDB_FILE_VERSION_BLOSC_COMPRESSION) {
  //    // Prior to the introduction of Blosc, ZLIB was the default compression
  //    scheme. mCompression = (COMPRESS_ZIP | COMPRESS_ACTIVE_MASK);
  //}
  char isCompressed = 0;
  if (file_version >= OPENVDB_FILE_VERSION_SELECTIVE_COMPRESSION &&
      file_version < OPENVDB_FILE_VERSION_NODE_MASK_COMPRESSION) {
    ifs.read(&isCompressed, sizeof(char));
    std::cout << "Compression : " << (isCompressed != 0 ? "zip" : "none")
              << std::endl;
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

  if (has_grid_offsets) {
    ReadGridDescriptors(ifs);
  } else {
  }
#endif

  return true;
}

bool SaveVDB(const std::string &filename, std::string *err) {
  std::ofstream os(filename.c_str(), std::ifstream::binary);

  if (!os) {
    if (err) {
      (*err) = "Failed to open a file to write: " + filename;
    }
    return false;
  }

  WriteVDBHeader(os);
  //if filemane

  return true;
}

}  // namespace tinyvdb

#endif

#endif  // TINY_VDB_H_
