#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>

#if defined(TINYVDBIO_USE_SYSTEM_ZLIB)
// or include your own zlib header here.
#include <zlib.h>
#endif

#define TINYVDBIO_IMPLEMENTATION
#include "tinyvdbio.h"

// ../common
#include "clipp.h"

int main(int argc, char **argv)
{
  using namespace clipp;

  std::string infile;
  bool verbose{false};

  auto cli = (
      value("input file", infile),
      option("--verbose").set(verbose).doc("Verbose output")
    );

  if(!parse(argc, argv, cli)) {
    std::cout << make_man_page(cli, argv[0]);
  }

  if (infile.empty()) {
    std::cerr << "Need input.vdb" << std::endl;
    return EXIT_FAILURE;
  }

  // 1. Parse VDB header
  tinyvdb::VDBHeader header;
  std::string warn;
  std::string err;
  tinyvdb::VDBStatus status = tinyvdb::ParseVDBHeader(infile, &header, &err);

  if (status != tinyvdb::TINYVDBIO_SUCCESS) {
    if (!err.empty()) {
      std::cerr << err << std::endl;
    }
    return EXIT_FAILURE;
  }

  // 2. Read Grid descriptors
  std::map<std::string, tinyvdb::GridDescriptor> gd_map;

  status = tinyvdb::ReadGridDescriptors(infile, header, &gd_map, &err);
  if (status != tinyvdb::TINYVDBIO_SUCCESS) {
    if (!err.empty()) {
      std::cerr << err << std::endl;
    }
    return EXIT_FAILURE;
  }

  std::cout << "# of grid descriptors = " << gd_map.size() << std::endl;

  // 3. Read Grids
  status = tinyvdb::ReadGrids(infile, header, gd_map, &warn, &err);
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
