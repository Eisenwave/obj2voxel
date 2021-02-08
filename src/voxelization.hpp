#ifndef OBJ2VOXEL_VOXELIZATION_HPP
#define OBJ2VOXEL_VOXELIZATION_HPP

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

struct AffineTransform {
    Vec3 matrix[3];
    Vec3 translation;

    constexpr Vec3 apply(Vec3 v) const
    {
        real_type x = dot(matrix[0], v);
        real_type y = dot(matrix[1], v);
        real_type z = dot(matrix[2], v);
        return Vec3{x, y, z} + translation;
    }
};

/// Throwaway class which manages all necessary data structures for voxelization and simplifies the procedure from the
/// caller's side to just using voxelize(triangle).
///
/// Before the Voxelizer can be used, a transform from model space must be initialized with initTransform(...);
class Voxelizer {
    static constexpr real_type ANTI_BLEED = 0.5f;

public:
    static AffineTransform computeTransform(Vec3 min, Vec3 max, unsigned resolution, Vec3u permutation);

private:
    AffineTransform trans;
    std::vector<TexturedTriangle> buffers[3]{};
    VoxelMap<WeightedUv> uvBuffer;
    VoxelMap<WeightedColor> voxels_;
    WeightedCombineFunction<Vec3f> combineFunction;

public:
    Voxelizer(AffineTransform trans, ColorStrategy colorStrategy);

    Voxelizer(const Voxelizer &&) = delete;
    Voxelizer(Voxelizer &&) = default;

    void voxelize(VisualTriangle triangle);

    void mergeResults(VoxelMap<WeightedColor> &out)
    {
        merge(out, voxels_);
    }

    void merge(VoxelMap<WeightedColor> &target, VoxelMap<WeightedColor> &source);

    VoxelMap<WeightedColor> &voxels()
    {
        return voxels_;
    }
};

}  // namespace obj2voxel

#endif  // VOXELIZATION_HPP
