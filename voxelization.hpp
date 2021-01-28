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

using InsertionFunction = void (*)(std::map<Vec3u, WeightedColor> &, Vec3u, WeightedColor);

/// Throwaway class which manages all necessary data structures for voxelization and simplifies the procedure from the
/// caller's side to just using voxelize(triangle).
///
/// Before the Voxelizer can be used, a transform from model space must be initialized with initTransform(...);
struct Voxelizer {
    static constexpr real_type ANTI_BLEED = 0.5f;

    std::map<std::string, Texture> textures;
    std::map<Vec3u, WeightedColor> colorBuffer;
    std::vector<TexturedTriangle> buffers[3]{};
    std::map<Vec3u, WeightedColor> voxels;
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

}  // namespace obj2voxels

#endif  // VOXELIZATION_HPP
