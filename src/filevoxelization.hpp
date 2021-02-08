#ifndef OBJ2VOXEL_FILEVOXELIZATION_HPP
#define OBJ2VOXEL_FILEVOXELIZATION_HPP

#include "voxelization.hpp"

#include <string>

namespace obj2voxel {

struct VoxelizationArgs {
    std::string inFile;
    std::string texture;
    unsigned resolution;
    ColorStrategy colorStrategy;
    Vec3u permutation;
};

using VoxelizationFunction = VoxelMap<WeightedColor> (*)(VoxelizationArgs);

VoxelMap<WeightedColor> voxelizeObj(VoxelizationArgs args);

VoxelMap<WeightedColor> voxelizeStl(VoxelizationArgs args);

}  // namespace obj2voxel

#endif  // FILEVOXELIZATION_HPP
