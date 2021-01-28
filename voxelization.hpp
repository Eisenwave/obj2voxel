#ifndef VOXELIZATION_HPP
#define VOXELIZATION_HPP

#include "triangle.hpp"

#include "voxelio/color.hpp"
#include "voxelio/log.hpp"
#include "voxelio/vec.hpp"

namespace obj2voxels {

using namespace voxelio;

// SIMPLE STRUCTS AND TYPEDEFS =========================================================================================

extern void(*globalTriangleDebugCallback)(const Triangle &triangle);

struct WeightedColor {
    float weight;
    Vec3f color;

    constexpr Color32 toColor32() const
    {
        return Color32{color};
    }
};

/**
 * @brief An enum which describes the strategy for coloring in voxels from triangles.
 */
enum class ColorStrategy {
    /// For the maximum strategy, the triangle with the greatest area is chosen as the color.
    MAX,
    /// For the blending strategy, the voxel color is determined by blending the triangle colors using their areas
    /// within the voxel as weights.
    BLEND
};

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

constexpr WeightedColor mix(const WeightedColor &lhs, const WeightedColor &rhs)
{
    float weightSum = lhs.weight + rhs.weight;
    return {weightSum, (lhs.weight * lhs.color + rhs.weight * rhs.color) / weightSum};
}

constexpr const WeightedColor &max(const WeightedColor &lhs, const WeightedColor &rhs)
{
    return lhs.weight > rhs.weight ? lhs : rhs;
}

template <ColorStrategy STRATEGY>
inline void insertColor(std::map<Vec3u, WeightedColor> &map, Vec3u pos, WeightedColor color)
{
    auto [location, success] = map.emplace(pos, color);
    if (not success) {
        location->second = STRATEGY == ColorStrategy::MAX ? max(color, location->second) : mix(color, location->second);
    }
}

void voxelize(const VisualTriangle &triangle,
              std::vector<TexturedTriangle> buffers[3],
              std::map<Vec3u, WeightedColor> &out);

}  // namespace obj2voxel

#endif  // VOXELIZATION_HPP
