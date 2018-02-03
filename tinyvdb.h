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

class GridDescriptor {
 public:
  GridDescriptor();
  GridDescriptor(const std::string &name, const std::string &grid_type,
                 bool save_float_as_half = false);
  // GridDescriptor(const GridDescriptor &rhs);
  // GridDescriptor& operator=(const GridDescriptor &rhs);
  ~GridDescriptor();

  const std::string &GridName() const { return grid_name_; }

  bool IsInstance() const { return !instance_parent_name_.empty(); }

  bool SaveFloatAsHalf() const { return save_float_as_half_; }

  tinyvdb_int64 GridPos() const { return grid_pos_; }

  tinyvdb_int64 BlockPos() const { return block_pos_; }

  tinyvdb_int64 EndPos() const { return end_pos_; }

  static std::string AddSuffix(const std::string &name, int n);
  static std::string StripSuffix(const std::string &name);

  ///
  /// Read GridDescriptor from a stream.
  ///
  bool Read(std::ifstream &is, const unsigned int file_version, std::string *err);

 private:
  std::string grid_name_;
  std::string unique_name_;
  std::string instance_parent_name_;
  std::string grid_type_;

  bool save_float_as_half_;  // use fp16?
  tinyvdb_int64 grid_pos_;
  tinyvdb_int64 block_pos_;
  tinyvdb_int64 end_pos_;
};

///
/// Leaf node contains actual voxel data.
///
class LeafNode {
 public:
  LeafNode();
  ~LeafNode();

 private:
};

///
/// Internal node have `InternalNode` or `LeafNode` as children.
///
class InternalNode {
 public:
  InternalNode();
  ~InternalNode();

 private:
};

///
/// Root node may have arbitrary child nodes.
///
template<typename ChildType>
class RootNode {
 public:
  RootNode() {}
  ~RootNode() {}

  bool ReadTopology(std::istream &is, const bool fromHalf);

 private:
  ChildType background_; 
};

///
/// Reader class for Tree data
///
class TreeReader {
 public:
  TreeReader();
  ~TreeReader();

  bool Read(std::istream &is);
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
#include <map>
#include <sstream>
#include <vector>
#include <cassert>

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
  COMPRESS_NONE = 0,
  COMPRESS_ZIP = 0x1,
  COMPRESS_ACTIVE_MASK = 0x2,
  COMPRESS_BLOSC = 0x4
};

namespace {

// In order not to break backward compatibility with existing VDB files,
// grids stored using 16-bit half floats are flagged by adding the following
// suffix to the grid's type name on output.  The suffix is removed on input
// and the grid's "save float as half" flag set accordingly.
const char *HALF_FLOAT_TYPENAME_SUFFIX = "_HalfFloat";

const char *SEP = "\x1e";  // ASCII "record separator"

}  // namespace

// https://gist.github.com/rygorous/2156668
// Reuse MINIZ_LITTLE_ENDIAN flag from miniz.
union FP32 {
  unsigned int u;
  float f;
  struct {
#if MINIZ_LITTLE_ENDIAN
    unsigned int Mantissa : 23;
    unsigned int Exponent : 8;
    unsigned int Sign : 1;
#else
    unsigned int Sign : 1;
    unsigned int Exponent : 8;
    unsigned int Mantissa : 23;
#endif
  } s;
};

#ifdef __clang__
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wpadded"
#endif

union FP16 {
  unsigned short u;
  struct {
#if MINIZ_LITTLE_ENDIAN
    unsigned int Mantissa : 10;
    unsigned int Exponent : 5;
    unsigned int Sign : 1;
#else
    unsigned int Sign : 1;
    unsigned int Exponent : 5;
    unsigned int Mantissa : 10;
#endif
  } s;
};

#ifdef __clang__
#pragma clang diagnostic pop
#endif

static inline FP32 half_to_float(FP16 h) {
  static const FP32 magic = {113 << 23};
  static const unsigned int shifted_exp = 0x7c00
                                          << 13;  // exponent mask after shift
  FP32 o;

  o.u = (h.u & 0x7fffU) << 13U;           // exponent/mantissa bits
  unsigned int exp_ = shifted_exp & o.u;  // just the exponent
  o.u += (127 - 15) << 23;                // exponent adjust

  // handle exponent special cases
  if (exp_ == shifted_exp)    // Inf/NaN?
    o.u += (128 - 16) << 23;  // extra exp adjust
  else if (exp_ == 0)         // Zero/Denormal?
  {
    o.u += 1 << 23;  // extra exp adjust
    o.f -= magic.f;  // renormalize
  }

  o.u |= (h.u & 0x8000U) << 16U;  // sign bit
  return o;
}

static inline FP16 float_to_half_full(FP32 f) {
  FP16 o = {0};

  // Based on ISPC reference code (with minor modifications)
  if (f.s.Exponent == 0)  // Signed zero/denormal (which will underflow)
    o.s.Exponent = 0;
  else if (f.s.Exponent == 255)  // Inf or NaN (all exponent bits set)
  {
    o.s.Exponent = 31;
    o.s.Mantissa = f.s.Mantissa ? 0x200 : 0;  // NaN->qNaN and Inf->Inf
  } else                                      // Normalized number
  {
    // Exponent unbias the single, then bias the halfp
    int newexp = f.s.Exponent - 127 + 15;
    if (newexp >= 31)  // Overflow, return signed infinity
      o.s.Exponent = 31;
    else if (newexp <= 0)  // Underflow
    {
      if ((14 - newexp) <= 24)  // Mantissa might be non-zero
      {
        unsigned int mant = f.s.Mantissa | 0x800000;  // Hidden 1 bit
        o.s.Mantissa = mant >> (14 - newexp);
        if ((mant >> (13 - newexp)) & 1)  // Check for rounding
          o.u++;  // Round, might overflow into exp bit, but this is OK
      }
    } else {
      o.s.Exponent = static_cast<unsigned int>(newexp);
      o.s.Mantissa = f.s.Mantissa >> 13;
      if (f.s.Mantissa & 0x1000)  // Check for rounding
        o.u++;                    // Round, might overflow to inf, this is OK
    }
  }

  o.s.Sign = f.s.Sign;
  return o;
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

static inline bool ReadMetaBool(std::istream &is) {
  char c = 0;
  unsigned int size;
  is.read(reinterpret_cast<char *>(&size), sizeof(unsigned int));
  if (size == 1) {
    is.read(&c, 1);
  }
  return bool(c);
}

static inline float ReadMetaFloat(std::istream &is) {
  float f = 0.0f;
  unsigned int size;
  is.read(reinterpret_cast<char *>(&size), sizeof(unsigned int));
  if (size == sizeof(float)) {
    is.read(reinterpret_cast<char *>(&f), sizeof(float));
    swap4(reinterpret_cast<unsigned int *>(&f));
  }
  return f;
}

static inline void ReadMetaVec3i(std::istream &is, int v[3]) {
  unsigned int size;
  is.read(reinterpret_cast<char *>(&size), sizeof(unsigned int));
  if (size == 3 * sizeof(int)) {
    is.read(reinterpret_cast<char *>(v), 3 * sizeof(int));
    swap4(&v[0]);
    swap4(&v[1]);
    swap4(&v[2]);
  }
}

static inline void ReadVec3d(std::istream &is, double v[3]) {
  is.read(reinterpret_cast<char *>(v), 3 * sizeof(double));
  swap8(reinterpret_cast<tinyvdb_int64*>(&v[0]));
  swap8(reinterpret_cast<tinyvdb_int64*>(&v[1]));
  swap8(reinterpret_cast<tinyvdb_int64*>(&v[2]));
}

static inline tinyvdb_int64 ReadMetaInt64(std::istream &is) {
  tinyvdb_int64 i64 = 0;
  unsigned int size;
  is.read(reinterpret_cast<char *>(&size), sizeof(unsigned int));
  if (size == 8) {
    is.read(reinterpret_cast<char *>(&i64), 8);
    swap8(&i64);
  }
  return i64;
}

// https://stackoverflow.com/questions/874134/find-if-string-ends-with-another-string-in-c
static inline bool EndsWidth(std::string const &value,
                             std::string const &ending) {
  if (ending.size() > value.size()) return false;
  return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

template<typename ValueType>
bool RootNode<ValueType>::ReadTopology(std::istream &is, const bool fromHalf)
{
  {
    int buffer_count;
    is.read(reinterpret_cast<char*>(&buffer_count), sizeof(int));
    if (buffer_count != 1) {
      // OPENVDB_LOG_WARN("multi-buffer trees are no longer supported");
    }
  }

  // Read background value;
  //if (fromHalf) {
  if (0) {
    assert(sizeof(ValueType) == 4);

    // Assume stored value is fp16 and `ValueType` is float
    unsigned short value = 0;
    is.read(reinterpret_cast<char*>(&value), 2);
    swap2(reinterpret_cast<unsigned short*>(&value));

    {
      FP16 fp16;
      fp16.u = value;

      FP32 fp32 = half_to_float(fp16);
      
      background_ = fp32.f;
    }
  } else {
    is.read(reinterpret_cast<char*>(&background_), sizeof(ValueType));

    if (2 == sizeof(ValueType)) {
      swap2(reinterpret_cast<unsigned short*>(&background_));
    } else if (4 == sizeof(ValueType)) {
      swap4(reinterpret_cast<unsigned int*>(&background_));
    } else if (8 == sizeof(ValueType)) {
      swap8(reinterpret_cast<tinyvdb_uint64*>(&background_));
    }
  }

  std::cout << "background : " << background_ << std::endl;

  unsigned int num_tiles = 0;
  unsigned int num_children = 0;
  is.read(reinterpret_cast<char*>(&num_tiles), sizeof(unsigned int));
  swap4(&num_tiles);
  is.read(reinterpret_cast<char*>(&num_children), sizeof(unsigned int));
  swap4(&num_children);

  if ((num_tiles == 0) && (num_children == 0)) {
    return false;
  }

  std::cout << "num_tiles " << num_tiles << std::endl;
  std::cout << "num_children " << num_children << std::endl;

  // Read tiles.
  for (unsigned int n = 0; n < num_tiles; n++) {
    int vec[3];
    ValueType value;
    bool active;

    is.read(reinterpret_cast<char*>(vec), 3 * sizeof(int));
    if (0) { 
      //if (fromHalf) {
      // Assume stored value is fp16 and `ValueType` is float
      unsigned short half_value = 0;
      is.read(reinterpret_cast<char*>(&half_value), 2);
      swap2(reinterpret_cast<unsigned short*>(&half_value));

      {
        FP16 fp16;
        fp16.u = half_value;

        FP32 fp32 = half_to_float(fp16);
        
        value = fp32.f;
      }
    } else {
      is.read(reinterpret_cast<char*>(&value), sizeof(ValueType));
    }
    is.read(reinterpret_cast<char*>(&active), sizeof(bool));
    swap4(&vec[0]);
    swap4(&vec[1]);
    swap4(&vec[2]);

    if (2 == sizeof(ValueType)) {
      swap2(reinterpret_cast<unsigned short*>(&value));
    } else if (4 == sizeof(ValueType)) {
      swap4(reinterpret_cast<unsigned int*>(&value));
    } else if (8 == sizeof(ValueType)) {
      swap8(reinterpret_cast<tinyvdb_uint64*>(&value));
    }
    
    std::cout << "[" << n << "] vec = (" << vec[0] << ", " << vec[1] << ", " << vec[2] << "), value = " << value << ", active = " << active << std::endl;
  }

  return true;


  // Read child nodes.
  for (unsigned int n = 0; n < num_children; n++) {
    (void)fromHalf;
#if 0 // TODO
    int vec[3];
    is.read(reinterpret_cast<char*>(vec), 3 * sizeof(int));

    swap4(&vec[0]);
    swap4(&vec[1]);
    swap4(&vec[2]);

    //ChildT* child = new ChildT(PartialCreate(), origin, mBackground);
    //child->readTopology(is, fromHalf);
    //mTable[Coord(vec)] = NodeStruct(*child);
#endif
  }

  return true;
}

#if 0
template<typename ChildT>
bool TreeReader::Read(std::istream &is)
{
  
}
#endif

GridDescriptor::GridDescriptor()
    : save_float_as_half_(false), grid_pos_(0), block_pos_(0), end_pos_(0) {}

GridDescriptor::GridDescriptor(const std::string &name,
                               const std::string &grid_type,
                               bool save_float_as_half)
    : grid_name_(StripSuffix(name)),
      unique_name_(name),
      grid_type_(grid_type),
      save_float_as_half_(save_float_as_half),
      grid_pos_(0),
      block_pos_(0),
      end_pos_(0) {}

// GridDescriptor::GridDescriptor(const GridDescriptor &rhs) {
//}

GridDescriptor::~GridDescriptor() {}

std::string GridDescriptor::AddSuffix(const std::string &name, int n) {
  std::ostringstream ss;
  ss << name << SEP << n;
  return ss.str();
}

std::string GridDescriptor::StripSuffix(const std::string &name) {
  return name.substr(0, name.find(SEP));
}

bool GridDescriptor::Read(std::ifstream &is, const unsigned int file_version, std::string *err) {
  unique_name_ = ReadString(is);
  grid_name_ = StripSuffix(unique_name_);

  grid_type_ = ReadString(is);

  if (EndsWidth(grid_type_, HALF_FLOAT_TYPENAME_SUFFIX)) {
    save_float_as_half_ = true;
    // strip suffix
    std::string tmp =
        grid_type_.substr(0, grid_type_.find(HALF_FLOAT_TYPENAME_SUFFIX));
    grid_type_ = tmp;
  }

  // FIXME(syoyo): Currently only `Tree_float_5_4_3` is supported.
  if (grid_type_.compare("Tree_float_5_4_3") != 0) {
    if (err) {
      (*err) = "Unsupported grid type: " + grid_type_;
    }
    return false;
  }

  std::cout << "grid_type = " << grid_type_ << std::endl;
  std::cout << "half = " << save_float_as_half_ << std::endl;

  if (file_version >= OPENVDB_FILE_VERSION_GRID_INSTANCING) {
    instance_parent_name_ = ReadString(is);
    std::cout << "instance_parent_name = " << instance_parent_name_
              << std::endl;
  }

  // Create the grid of the type if it has been registered.
  // if (!GridBase::isRegistered(mGridType)) {
  //    OPENVDB_THROW(LookupError, "Cannot read grid." <<
  //        " Grid type " << mGridType << " is not registered.");
  //}
  // else
  // GridBase::Ptr grid = GridBase::createGrid(mGridType);
  // if (grid) grid->setSaveFloatAsHalf(mSaveFloatAsHalf);

  // Read in the offsets.
  is.read(reinterpret_cast<char *>(&grid_pos_), sizeof(tinyvdb_int64));
  is.read(reinterpret_cast<char *>(&block_pos_), sizeof(tinyvdb_int64));
  is.read(reinterpret_cast<char *>(&end_pos_), sizeof(tinyvdb_int64));

  std::cout << "grid_pos = " << grid_pos_ << std::endl;
  std::cout << "block_pos = " << block_pos_ << std::endl;
  std::cout << "end_pos = " << end_pos_ << std::endl;

  return true;
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
      ReadMetaVec3i(is, v);

      std::cout << "  value = " << v[0] << ", " << v[1] << ", " << v[2]
                << std::endl;

    } else if (type_name.compare("bool") == 0) {
      bool b = ReadMetaBool(is);

      std::cout << "  value = " << b << std::endl;

    } else if (type_name.compare("float") == 0) {
      float f = ReadMetaFloat(is);

      std::cout << "  value = " << f << std::endl;

    } else if (type_name.compare("int64") == 0) {
      tinyvdb_int64 i64 = ReadMetaInt64(is);

      std::cout << "  value = " << i64 << std::endl;

    } else {
      // Unknown metadata
      int num_bytes;
      is.read(reinterpret_cast<char *>(&num_bytes), sizeof(int));
      swap4(&num_bytes);

      std::cout << "  unknown value. size = " << num_bytes << std::endl;

      std::vector<char> data;
      data.resize(size_t(num_bytes));
      is.read(data.data(), num_bytes);
    }
  }

  return true;
}

static void ReadGridDescriptors(std::ifstream &is,
                                const unsigned int file_version,
                                std::map<std::string, GridDescriptor> *gd_map) {
  // Read the number of metadata items.
  int count = 0;
  is.read(reinterpret_cast<char *>(&count), sizeof(int));

  std::cout << "grid descriptors = " << count << std::endl;

  for (int i = 0; i < count; ++i) {
    // Read the grid descriptor.
    GridDescriptor gd;
    std::string err;
    bool ret = gd.Read(is, file_version, &err);
    assert(ret);

    (*gd_map)[gd.GridName()] = gd;
#if 0
      // Add the descriptor to the dictionary.
      gridDescriptors().insert(std::make_pair(gd.gridName(), gd));

      // Skip forward to the next descriptor.
      gd.seekToEnd(is);
#endif
  }
}

static void ReadTransform(std::ifstream &is) {
  // Read the type name.
  std::string type = ReadString(is);

  std::cout << "transform type = " << type << std::endl;

  double scale_values[3];
  double voxel_size[3];
  double scale_values_inverse[3];
  double inv_scale_squared[3];
  double inv_twice_scale[3];

  if (type.compare("UniformScaleMap") == 0) {
    scale_values[0] = scale_values[1] = scale_values[2] = 0.0;
    voxel_size[0] = voxel_size[1] = voxel_size[2] = 0.0;
    scale_values_inverse[0] = scale_values_inverse[1] = scale_values_inverse[2] = 0.0;
    inv_scale_squared[0] = inv_scale_squared[1] = inv_scale_squared[2] = 0.0;
    inv_twice_scale[0] = inv_twice_scale[1] = inv_twice_scale[2] = 0.0;

    ReadVec3d(is, scale_values);
    ReadVec3d(is, voxel_size);
    ReadVec3d(is, scale_values_inverse);
    ReadVec3d(is, inv_scale_squared);
    ReadVec3d(is, inv_twice_scale);

    std::cout << "scale_values " << scale_values[0] << ", " << scale_values[1] << ", " << scale_values[2] << std::endl;
    std::cout << "voxel_size " << voxel_size[0] << ", " << voxel_size[1] << ", " << voxel_size[2] << std::endl;
    std::cout << "scale_value_sinverse " << scale_values_inverse[0] << ", " << scale_values_inverse[1] << ", " << scale_values_inverse[2] << std::endl;
    std::cout << "inv_scale_squared " << inv_scale_squared[0] << ", " << inv_scale_squared[1] << ", " << inv_scale_squared[2] << std::endl;
    std::cout << "inv_twice_scale " << inv_twice_scale[0] << ", " << inv_twice_scale[1] << ", " << inv_twice_scale[2] << std::endl;
  } else {
    assert(0);
    // TODO(syoyo): Implement
  }
    

  
  // TODO(syoyo) read transform
}

static void ReadGrid(std::ifstream &is, const unsigned int file_version,
                     const GridDescriptor &gd) {
  if (file_version >= OPENVDB_FILE_VERSION_NODE_MASK_COMPRESSION) {
    unsigned int c = COMPRESS_NONE;
    is.read(reinterpret_cast<char *>(&c), sizeof(unsigned int));
    std::cout << "compression: " << c << std::endl;
    // io::setDataCompression(is, c);
  }

  ReadMeta(is);

  // read transform
  ReadTransform(is);

  // read topology
  if (!gd.IsInstance()) {
    RootNode<float> rootNode;
    rootNode.ReadTopology(is, gd.SaveFloatAsHalf());
  }

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
    std::cout << "InputHasGridOffsets = "
              << (has_grid_offsets ? " yes " : " no ") << std::endl;
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
  // std::map<std::string, GridDescriptor>::iterator itEnd(gd_map.end());

  // for (; it != itEnd; it++) {
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
  // if filemane

  return true;
}

}  // namespace tinyvdb

#endif

#endif  // TINY_VDB_H_
