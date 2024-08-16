/*
 * kittivdb.cc
 *
 *  Created on: Aug 11, 2024
 *      Author: amyznikov
 *
 *  Test code based on https://github.com/AcademySoftwareFoundation/openvdb/blob/master/doc/examplecode.txt
 */

#include <core/io/kitti/c_kitti_dataset.h>
#include <openvdb/openvdb.h>
#include <core/debug.h>


//struct VoxelData
//{
//  float x = 0;
//  float y = 0;
//  float z = 0;
//  float r = 0;
//
//  VoxelData()
//  {
//
//  }
//
//  VoxelData(float v) :
//    x(v), y(v), z(v), r(v)
//  {
//  }
//
//};
//
//using VoxelTree = openvdb::tree::Tree4<VoxelData, 5, 4, 3>::Type;
//
//class PointGrid:
//    public openvdb::Grid<VoxelTree>
//{
//public:
//  typedef PointGrid ThisClass;
//  typedef openvdb::Grid<VoxelData> Base;
////  using Ptr                 =
////  using Ptr                 = SharedPtr<ThisClass>;
////  using ConstPtr            = SharedPtr<const ThisClass>;
//
//  //typedef typename Base::Ptr Ptr;
//};

int main(int argc, char * argv[])
{
  // Initialize the OpenVDB library.  This must be called at least
  // once per program and may safely be called multiple times.
  openvdb::initialize();

  // Create an empty floating-point grid with background value 0.
//  openvdb::FloatGrid::Ptr grid = openvdb::FloatGrid::create();
//  PointGrid::Ptr grid = PointGrid::create();

//  std::cout << "Testing random access:" << std::endl;
//
//  // Get an accessor for coordinate-based access to voxels.
//  openvdb::FloatGrid::Accessor accessor = grid->getAccessor();
//
//  // Define a coordinate with large signed indices.
//  openvdb::Coord xyz(1000, -200000000, 30000000);
//
//  // Set the voxel value at (1000, -200000000, 30000000) to 1.
//  accessor.setValue(xyz, 1.0);
//
//  // Verify that the voxel value at (1000, -200000000, 30000000) is 1.
//  std::cout << "Grid" << xyz << " = " << accessor.getValue(xyz) << std::endl;
//
//  // Reset the coordinates to those of a different voxel.
//  xyz.reset(1000, 200000000, -30000000);
//
//  // Verify that the voxel value at (1000, 200000000, -30000000) is
//  // the background value, 0.
//  std::cout << "Grid" << xyz << " = " << accessor.getValue(xyz) << std::endl;
//
//  // Set the voxel value at (1000, 200000000, -30000000) to 2.
//  accessor.setValue(xyz, 2.0);
//
//  // Set the voxels at the two extremes of the available coordinate space.
//  // For 32-bit signed coordinates these are (-2147483648, -2147483648, -2147483648)
//  // and (2147483647, 2147483647, 2147483647).
//  accessor.setValue(openvdb::Coord::min(), 3.0f);
//  accessor.setValue(openvdb::Coord::max(), 4.0f);
//
//  std::cout << "Testing sequential access:" << std::endl;
//
//  // Print all active ("on") voxels by means of an iterator.
//  for( openvdb::FloatGrid::ValueOnCIter iter = grid->cbeginValueOn(); iter; ++iter ) {
//    std::cout << "Grid" << iter.getCoord() << " = " << *iter << std::endl;
//  }

  return 0;
}
