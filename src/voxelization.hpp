#ifndef OBJ2VOXEL_VOXELIZATION_HPP
#define OBJ2VOXEL_VOXELIZATION_HPP

#include "arrayvector.hpp"
#include "triangle.hpp"
#include "util.hpp"

#include "voxelio/color.hpp"
#include "voxelio/log.hpp"
#include "voxelio/vec.hpp"

#include <vector>

namespace obj2voxel {

using namespace voxelio;

// SIMPLE STRUCTS AND TYPEDEFS =========================================================================================

/// A function called on debug builds which can be used to dump triangles to an STL file and such.
extern void (*globalTriangleDebugCallback)(Triangle);

/// An enum which describes the strategy for coloring in voxels from triangles.
enum class ColorStrategy : obj2voxel_enum_t {
    /// For the maximum strategy, the triangle with the greatest area is chosen as the color.
    MAX = OBJ2VOXEL_MAX_STRATEGY,
    /// For the blending strategy, the voxel color is determined by blending the triangle colors using their areas
    /// within the voxel as weights.
    BLEND = OBJ2VOXEL_BLEND_STRATEGY
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

/// Throwaway class which manages all necessary data structures for voxelization and simplifies the procedure from the
/// caller's side to just using voxelize(triangle).
///
/// Before the Voxelizer can be used, a transform from model space must be initialized with initTransform(...);
class Voxelizer {
public:
    using split_buffer_type = ArrayVector<TexturedTriangle, 64>;

private:
    std::vector<TexturedTriangle> subdivisionBuffer;
    split_buffer_type preSplitBuffer;
    split_buffer_type postSplitBuffer;
    VoxelMap<WeightedUv> uvBuffer;
    VoxelMap<WeightedColor> voxels_;
    WeightedCombineFunction<Vec3f> combineFunction;

public:
    Voxelizer(ColorStrategy colorStrategy) noexcept;

    Voxelizer(const Voxelizer &) noexcept = delete;
    Voxelizer(Voxelizer &&) noexcept = default;

    void voxelize(const VisualTriangle &triangle, Vec3u32 min, Vec3u32 max) noexcept;

    void mergeResults(VoxelMap<WeightedColor> &out) noexcept
    {
        merge(out, voxels_);
    }

    void merge(VoxelMap<WeightedColor> &target, VoxelMap<WeightedColor> &source) noexcept;

    /**
     * @brief Scales down the voxels of the voxelizer to half the original resolution.
     */
    void downscale() noexcept;

    VoxelMap<WeightedColor> &voxels() noexcept
    {
        return voxels_;
    }

private:
    /**
     * @brief Voxelizes a triangle.
     *
     * Among the parameters is an array of three buffers.
     * This array must be notnull.
     * The contents of the buffers are cleared by the callee, this parameter is only used so that allocation of new
     * vectors can be avoided for each triangle.
     *
     * @param triangle the input triangle to be voxelized
     * @param buffers three buffers which are used for intermediate operations
     * @param out the output map of voxel locations to weighted colors
     */
    void voxelizeTriangleToUvBuffer(const VisualTriangle &inputTriangle, Vec3u32 min, Vec3u32 max) noexcept;

    void moveUvBufferIntoVoxels(const VisualTriangle &inputTriangle) noexcept;
};

}  // namespace obj2voxel

#endif  // VOXELIZATION_HPP
