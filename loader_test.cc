#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <fstream>
//#include <cstdint>

#define TINYVDB_IMPLEMENTATION
#include "tinyvdb.h"

int main(int argc, char **argv)
{
  if (argc < 2) {
    std::cerr << "Need input.vdb" << std::endl;
    return EXIT_FAILURE;
  }

  std::string err;
  bool ret = tinyvdb::ParseVDBHeader(argv[1], &err);

  if (!ret) {
    if (!err.empty()) {
      std::cerr << err << std::endl;
    }
  }

  return EXIT_SUCCESS;
}
