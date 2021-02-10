#ifndef OBJ2VOXEL_FILEVOXELIZATION_HPP
#define OBJ2VOXEL_FILEVOXELIZATION_HPP

#include "io.hpp"
#include "voxelization.hpp"

#include <string>

namespace obj2voxel {

struct VoxelizationArgs {
    std::string inFile;
    std::string texture;
    unsigned resolution;
    unsigned workerThreads;
    bool downscale;
    ColorStrategy colorStrategy;
    Vec3u permutation;
};

using VoxelizationFunction = VoxelMap<WeightedColor> (*)(VoxelizationArgs);

[[nodiscard]] bool voxelize(VoxelizationArgs args, ITriangleStream &in, VoxelSink &out);

}  // namespace obj2voxel

#endif  // FILEVOXELIZATION_HPP
