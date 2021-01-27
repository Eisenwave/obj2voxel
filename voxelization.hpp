#ifndef VOXELIZATION_HPP
#define VOXELIZATION_HPP

#include "tinyobj.hpp"

#include "voxelio/vec.hpp"
#include "voxelio/log.hpp"
#include "voxelio/color.hpp"

namespace obj2voxel {

using namespace voxelio;

// SIMPLE STRUCTS AND TYPEDEFS =========================================================================================

using real_type = tinyobj::real_t;

using Vec2 = Vec<real_type, 2>;
using Vec3 = Vec<real_type, 3>;

enum class TriangleType {
    MATERIALLESS,
    UNTEXTURED,
    TEXTURED
};

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

/**
 * @brief Applies a binary function to each component of two vectors and returns the result.
 */
template <typename T, size_t N, const T&(*functor)(const T&, const T&)>
constexpr Vec<T, N> applyForEach(Vec<T, N> a, Vec<T, N> b)
{
    Vec<T, N> result{};
    for (usize i = 0; i < N; ++i) {
        result[i] = functor(a[i], b[i]);
    }
    return result;
}

/// Returns the component-wise min.
template <typename T, size_t N>
constexpr Vec<T, N> min(Vec<T, N> a, Vec<T, N> b)
{
    return applyForEach<T, N, std::min<T>>(a, b);
}

/// Returns the component-wise min.
template <typename T, size_t N>
constexpr Vec<T, N> max(Vec<T, N> a, Vec<T, N> b)
{
    return applyForEach<T, N, std::max<T>>(a, b);
}

template <typename T>
constexpr T min(T a, T b, T c)
{
    return std::min(a, std::min(b, c));
}

template <typename T>
constexpr T max(T a, T b, T c)
{
    return std::max(a, std::max(b, c));
}

template <typename T, size_t N>
T length(Vec<T, N> v)
{
    T result = std::sqrt(dot<T, T, N>(v, v));
    return result;
}

template <typename T, size_t N>
constexpr Vec<T, N> mix(Vec<T, N> a, Vec<T, N> b, T t)
{
    return (1 - t) * a + t * b;
}

constexpr WeightedColor mix(const WeightedColor &lhs, const WeightedColor &rhs)
{
    float weightSum = lhs.weight + rhs.weight;
    return {weightSum, (lhs.weight * lhs.color + rhs.weight * rhs.color) / weightSum};
}

constexpr const WeightedColor &max(const WeightedColor &lhs, const WeightedColor &rhs)
{
    return lhs.weight > rhs.weight ? lhs : rhs;
}

inline void insertColor(std::map<Vec3u, WeightedColor> &map, Vec3u pos, WeightedColor color, ColorStrategy strategy)
{
    auto [location,success] = map.emplace(pos, color);
    if (not success) {
        location->second = strategy == ColorStrategy::MAX ? max(color, location->second)
                                                          : mix(color, location->second);
    }
}

// TRIANGLES ===========================================================================================================

struct Triangle {
    Vec3 v[3];

    constexpr Vec3 vertex(usize index) const {
        VXIO_DEBUG_ASSERT_LT(index, 3);
        return v[index];
    }

    constexpr Vec3 edge(usize index) const {
        VXIO_DEBUG_ASSERT_LT(index, 3);
        return v[(index + 1) % 3] - v[index];
    }

    constexpr real_type min(usize i) const
    {
        return obj2voxel::min(v[0][i], v[1][i], v[2][i]);
    }

    constexpr real_type max(usize i) const
    {
        return obj2voxel::max(v[0][i], v[1][i], v[2][i]);
    }

    constexpr Vec3 min() const {
        return obj2voxel::min(obj2voxel::min(v[0], v[1]), v[2]);
    }

    constexpr Vec3 max() const {
        return obj2voxel::min(obj2voxel::min(v[0], v[1]), v[2]);
    }

    real_type area() const
    {
        return length(cross(edge(0), edge(1))) / 2;
    }

    constexpr Vec3 center() const {
        return (v[0] + v[1] + v[2]) / 3;
    }
};

struct Texture {
    Vec3f get(Vec2 uv) const
    {
        return {};
    }
};

struct TexturedTriangle : public Triangle {
    Vec2 t[3];

    constexpr Vec2 texture(usize index) const {
        VXIO_DEBUG_ASSERT_LT(index, 3);
        return t[index];
    }

    constexpr Vec2 textureCenter() const {
        return (t[0] + t[1] + t[2]) / 3;
    }
};

struct VisualTriangle : public TexturedTriangle {
    TriangleType type;
    union {
        Texture *texture;
        Vec3f color;
    };

    VisualTriangle() : texture{nullptr} {}
    constexpr VisualTriangle(TriangleType type) : type{type}, texture{nullptr} {}

    Vec3f colorAt_f(Vec2 uv) const
    {
        switch (type) {
        case TriangleType::MATERIALLESS: return Vec3f::filledWith(1);
        case TriangleType::UNTEXTURED: return color;
        case TriangleType::TEXTURED: {
            VXIO_ASSERT_NOTNULL(texture);
            return texture->get(uv);
        }
        }
        VXIO_DEBUG_ASSERT_UNREACHABLE();
    }
};

/**
 * @brief Splits a single triangle into multiple triangles on a specified axis plane.
 * The triangles are inserted into two vectors depending on whether the sub-triangles are on the negative or positive
 * side of the axis plane.
 * @param axis the axis of the plane in [0, 3)
 * @param plane the plane offset
 * @param t the triangle
 * @param outLo the output vector for lower triangles
 * @param outHi the output vector for higher triangles
 */
void split(const unsigned axis,
           const unsigned plane,
           const TexturedTriangle t,
           std::vector<TexturedTriangle> &outLo,
           std::vector<TexturedTriangle> &outHi);

/**
 * @brief Splits a buffer of triangles on all axes into pieces which don't intersect any axis plane.
 * The resulting triangles all fit exactly into AABBs or voxels.
 * @param cutBuffer the (notnull) buffer used for currently cut triangles; should be filled with a triangle at the start
 * @param resultBuffer the (notnull) buffer used for storing the resulting triangles
 */
void split(std::vector<TexturedTriangle> *cutBuffer,
           std::vector<TexturedTriangle> *resultBuffer);

void voxelize(const VisualTriangle &triangle,
              std::vector<TexturedTriangle> buffers[2],
              std::map<Vec3u, WeightedColor> &out);

} // namespace obj2voxel

#endif // VOXELIZATION_HPP
