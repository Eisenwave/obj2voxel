#ifndef VOXELIZATION_HPP
#define VOXELIZATION_HPP

#include "triangle.hpp"
#include "util.hpp"

#include "voxelio/color.hpp"
#include "voxelio/log.hpp"
#include "voxelio/vec.hpp"

#include <map>

namespace obj2voxel {

using namespace voxelio;

// SIMPLE STRUCTS AND TYPEDEFS =========================================================================================

/// A function called on debug builds which can be used to dump triangles to an STL file and such.
extern void (*globalTriangleDebugCallback)(Triangle);

/// An enum which describes the strategy for coloring in voxels from triangles.
enum class ColorStrategy {
    /// For the maximum strategy, the triangle with the greatest area is chosen as the color.
    MAX,
    /// For the blending strategy, the voxel color is determined by blending the triangle colors using their areas
    /// within the voxel as weights.
    BLEND
};

constexpr const char *nameOf(ColorStrategy strategy)
{
    return strategy == ColorStrategy::MAX ? "MAX" : "BLEND";
}

/// Parses the color strategy. This function is case sensitive.
inline bool parseColorStrategy(const std::string &str, ColorStrategy &out)
{
    if (str == "MAX") {
        out = ColorStrategy::MAX;
        return true;
    }
    if (str == "BLEND") {
        out = ColorStrategy::BLEND;
        return true;
    }
    return false;
}

using InsertionFunction = void (*)(VoxelMap<WeightedColor> &, Vec3u, WeightedColor);

/**
 * @brief Scales down a map of voxels to a lower resolution and combines multiple colors into one using the given
 * strategy.
 * @param voxels the input voxels. This map is passed by value and emptied in the process.
 * @param strategy the color strategy for combining voxel colors
 * @param divisor the divisor of the model size
 * @return the downscaled model
 */
VoxelMap<WeightedColor> downscale(VoxelMap<WeightedColor> voxels, ColorStrategy strategy, unsigned divisor = 2);

/// Throwaway class which manages all necessary data structures for voxelization and simplifies the procedure from the
/// caller's side to just using voxelize(triangle).
///
/// Before the Voxelizer can be used, a transform from model space must be initialized with initTransform(...);
struct Voxelizer {
    static constexpr real_type ANTI_BLEED = 0.5f;

    std::map<std::string, Texture> textures;
    std::vector<TexturedTriangle> buffers[3]{};
    VoxelMap<WeightedUv> uvBuffer;
    VoxelMap<WeightedColor> voxels;
    Vec3 meshMin{}, meshMax{};
    real_type scaleFactor = 1;

    InsertionFunction insertionFunction;
    usize triangleCount = 0;

    Voxelizer(ColorStrategy colorStrategy);

    Voxelizer(const Voxelizer &&) = delete;
    Voxelizer(Voxelizer &&) = delete;

    void initTransform(Vec3 min, Vec3 max, unsigned resolution);

    Vec3 transform(Vec3 v);

    void voxelize(VisualTriangle triangle);
};

}  // namespace obj2voxel

#endif  // VOXELIZATION_HPP
