#define TINYVDBIO_IMPLEMENTATION
#include "tinyvdbio.h"

#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
//#include <cstdint>


int main(int argc, char **argv)
{
  if (argc < 2) {
    std::cerr << "Need input.vdb" << std::endl;
    return EXIT_FAILURE;
  }

  // 1. Parse VDB header
  tinyvdb::VDBHeader header;
  std::string warn;
  std::string err;
  tinyvdb::VDBStatus status = tinyvdb::ParseVDBHeader(argv[1], &header, &err);

  if (status != tinyvdb::TINYVDBIO_SUCCESS) {
    if (!err.empty()) {
      std::cerr << err << std::endl;
    }
    return EXIT_FAILURE;
  }

  // 2. Read Grid descriptors
  std::map<std::string, tinyvdb::GridDescriptor> gd_map;

  status = tinyvdb::ReadGridDescriptors(argv[1], header, &gd_map, &err);
  if (status != tinyvdb::TINYVDBIO_SUCCESS) {
    if (!err.empty()) {
      std::cerr << err << std::endl;
    }
    return EXIT_FAILURE;
  }

  // 3. Read Grids
  status = tinyvdb::ReadGrids(argv[1], header, gd_map, &warn, &err);
  if (!warn.empty()) {
    std::cout << warn << std::endl;
  }
  if (status != tinyvdb::TINYVDBIO_SUCCESS) {
    if (!err.empty()) {
      std::cerr << err << std::endl;
    }
    return EXIT_FAILURE;
  }

  std::cout << "Load OK" << std::endl;

  return EXIT_SUCCESS;
}
