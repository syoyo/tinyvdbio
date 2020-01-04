#include <iostream>
#include <string>

#if TINYVDBIO_USE_SYSTEM_ZLIB
#include <zlib.h>
#endif

#define TINYVDBIO_IMPLEMENTATION
#include "tinyvdbio.h"

int main(int argc, char **argv)
{
  if (argc < 2) {
    std::cerr << "Need input.vdb\n";
    return EXIT_FAILURE;
  }

  std::string filename = argv[1];

  std::cout << "Loading : " << filename << "\n";

  return EXIT_SUCCESS;
}
