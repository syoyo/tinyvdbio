#include <cstdio>
#include <cstdlib>

#include "simple_vtk.hpp"

#define TINYVDBIO_IMPLEMENTATION
#include "tinyvdbio.h"

struct Extent
{
  std::array<double, 3> bmin;
  std::array<double, 3> bmax;
};

int main(int argc, char **argv)
{
  std::array<double, 3> origin{};
  std::array<double, 3> spacing{};

  Extent extent;

  SimpleVTK gen;
  gen.enableExtentManagement();
  gen.changeBaseExtent(extent.bmin[0], extent.bmax[0], extent.bmin[1], extent.bmax[1], extent.bmin[2], extent.bmax[2]);
  gen.changeBaseOrigin(origin[0], origin[1], origin[2]);
  gen.changeBaseSpacing(spacing[0], spacing[1], spacing[2]);
  gen.changeRefinementRatio(2.0);

  gen.beginVTK("vtkHierarchicalBoxDataSet");
  gen.beginContent();
  gen.setGridDescription("XYZ");

  gen.endContent();
  gen.endVTK();

  gen.generate("output_amr");

  return EXIT_SUCCESS;

}
