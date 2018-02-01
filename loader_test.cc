#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
#include <cstdint>

#define TINYVDB_IMPLEMENTATION
#include "tinyvdb.h"

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

int main(int argc, char **argv)
{
  if (argc < 2) {
    std::cerr << "Need input.vdb" << std::endl;
    return EXIT_FAILURE;
  }

  const std::string filename = argv[1];

  std::ifstream ifs(filename, std::ifstream::binary);
  if (!ifs) {
    std::cerr << "File not found or cannot open file : " << filename << std::endl;
    return EXIT_FAILURE;
  }

  // Header

  const int32_t OPENVDB_MAGIC = 0x56444220;
  int64_t magic;

  // [0:7] magic number
  if (!ifs.read(reinterpret_cast<char*>(&magic), sizeof(int64_t))) {
    return EXIT_FAILURE;
  }

  if (magic == OPENVDB_MAGIC) {
    std::cout << "bingo!" << std::endl;
  }

  // [8:11] version 
  uint32_t file_version;
  if (!ifs.read(reinterpret_cast<char*>(&file_version), sizeof(uint32_t))) {
    return EXIT_FAILURE;
  }

  std::cout << "File version: " << file_version << std::endl;

  // Read the library version numbers (not stored prior to file format version 211).
  if (file_version >= 211) {
      uint32_t version;
      ifs.read(reinterpret_cast<char*>(&version), sizeof(uint32_t));
      std::cout << "major version : " << version << std::endl;
      ifs.read(reinterpret_cast<char*>(&version), sizeof(uint32_t));
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

  return EXIT_SUCCESS;
}
