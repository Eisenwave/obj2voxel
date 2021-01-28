#ifndef VOXELIZATION_HPP
#define VOXELIZATION_HPP

#include "triangle.hpp"

#include "voxelio/color.hpp"
#include "voxelio/log.hpp"
#include "voxelio/vec.hpp"

namespace obj2voxels {

using namespace voxelio;

// SIMPLE STRUCTS AND TYPEDEFS =========================================================================================

/// A function called on debug builds which can be used to dump triangles to an STL file and such.
extern void (*globalTriangleDebugCallback)(const Triangle &triangle);

/// A color and a weight.
struct WeightedColor {
    float weight;
    Vec3f color;

    constexpr Color32 toColor32() const
    {
        return Color32{color};
    }
};

/// An enum which describes the strategy for coloring in voxels from triangles.
enum class ColorStrategy {
    /// For the maximum strategy, the triangle with the greatest area is chosen as the color.
    MAX,
    /// For the blending strategy, the voxel color is determined by blending the triangle colors using their areas
    /// within the voxel as weights.
    BLEND
};

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

// UTILITY FUNCTIONS ===================================================================================================

/// Mixes two colors based on their weights.
constexpr WeightedColor mix(const WeightedColor &lhs, const WeightedColor &rhs)
{
    float weightSum = lhs.weight + rhs.weight;
    return {weightSum, (lhs.weight * lhs.color + rhs.weight * rhs.color) / weightSum};
}

/// Chooses the color with the greater weight.
constexpr const WeightedColor &max(const WeightedColor &lhs, const WeightedColor &rhs)
{
    return lhs.weight > rhs.weight ? lhs : rhs;
}

/// Emplaces a color in a map at the given location or combines it with the already stored color.
template <ColorStrategy STRATEGY>
inline void insertColor(std::map<Vec3u, WeightedColor> &map, Vec3u pos, WeightedColor color)
{
    auto [location, success] = map.emplace(pos, color);
    if (not success) {
        location->second = STRATEGY == ColorStrategy::MAX ? max(color, location->second) : mix(color, location->second);
    }
}

/**
 * @brief Voxelizes a triangle.
 *
 * Among the parameters is an array of three buffers.
 * This array must be notnull.
 * The contents of the buffers are cleared by the callee, this parameter is only used so that allocation of new vectors
 * can be avoided for each triangle.
 *
 * @param triangle the input triangle to be voxelized
 * @param buffers three buffers which are used for intermediate operations
 * @param out the output map of voxel locations to weighted colors
 */
void voxelize(const VisualTriangle &triangle,
              std::vector<TexturedTriangle> buffers[3],
              std::map<Vec3u, WeightedColor> &out);

}  // namespace obj2voxels

#endif  // VOXELIZATION_HPP
