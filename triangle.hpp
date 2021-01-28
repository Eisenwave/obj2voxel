#ifndef TRIANGLE_HPP
#define TRIANGLE_HPP

#include "voxelio/image.hpp"
#include "voxelio/vec.hpp"

#include "3rd_party/tinyobj.hpp"

namespace obj2voxels {

using namespace voxelio;

using real_type = tinyobj::real_t;

using Vec2 = Vec<real_type, 2>;
using Vec3 = Vec<real_type, 3>;

enum class TriangleType { MATERIALLESS, UNTEXTURED, TEXTURED };

// UTILITY FUNCTIONS ===================================================================================================

/**
 * @brief Applies a binary function to each component of two vectors and returns the result.
 */
template <typename T, size_t N, const T &(*functor)(const T &, const T &)>
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

/// Three-parameter min. For Vec types, returns the component-wise minimum.
template <typename T>
constexpr T min(const T &a, const T &b, const T &c)
{
    if constexpr (voxelio::isVec<T>) {
        return obj2voxels::min(a, obj2voxels::min(b, c));
    }
    else {
        return std::min(a, std::min(b, c));
    }
}

/// Three-parameter min. For Vec types, returns the component-wise maximum.
template <typename T>
constexpr T max(const T &a, const T &b, const T &c)
{
    if constexpr (voxelio::isVec<T>) {
        return obj2voxels::max(a, obj2voxels::max(b, c));
    }
    else {
        return std::max(a, std::max(b, c));
    }
}

template <typename T, size_t N>
constexpr Vec<T, N> floor(Vec<T, N> v)
{
    Vec<T, N> result{};
    for (size_t i = 0; i < N; ++i) {
        result[i] = std::floor(v[i]);
    }
    return result;
}

template <typename T, size_t N>
constexpr Vec<T, N> ceil(Vec<T, N> v)
{
    Vec<T, N> result{};
    for (size_t i = 0; i < N; ++i) {
        result[i] = std::ceil(v[i]);
    }
    return result;
}

template <typename T, size_t N>
constexpr Vec<T, N> abs(Vec<T, N> v)
{
    Vec<T, N> result{};
    for (size_t i = 0; i < N; ++i) {
        result[i] = std::abs(v[i]);
    }
    return result;
}

template <typename T, size_t N>
T length(Vec<T, N> v)
{
    T result = std::sqrt(dot<T, T, N>(v, v));
    return result;
}

template <typename T, size_t N>
constexpr Vec<T, N> normalize(Vec<T, N> v)
{
    return v / length(v);
}

template <typename T, size_t N>
constexpr Vec<T, N> mix(Vec<T, N> a, Vec<T, N> b, T t)
{
    return (1 - t) * a + t * b;
}

// TRIANGLES ===========================================================================================================

struct Triangle {
    Vec3 v[3];

    /// Returns a vertex at a given index in [0,3).
    constexpr Vec3 vertex(usize index) const
    {
        VXIO_DEBUG_ASSERT_LT(index, 3);
        return v[index];
    }

    /// Returns an edge between two vertices.
    constexpr Vec3 edge(usize start, usize end) const
    {
        return vertex(end) - vertex(start);
    }

    /// Returns an edge from the vertex at the given index to its successor.
    constexpr Vec3 neighborEdge(usize index) const
    {
        return edge(index, (index + 1) % 3);
    }

    /// Returns the (unnormalized) normal of this triangle.
    constexpr Vec3 normal() const
    {
        return cross(edge(0, 1), edge(0, 2));
    }

    /// Returns the minimum coordinate of this triangle on one axis.
    constexpr real_type min(usize i) const
    {
        VXIO_DEBUG_ASSERT_LT(i, 3);
        return obj2voxels::min(v[0][i], v[1][i], v[2][i]);
    }

    /// Returns the minimum coordinate of this triangle on one axis.
    constexpr real_type max(usize i) const
    {
        VXIO_DEBUG_ASSERT_LT(i, 3);
        return obj2voxels::max(v[0][i], v[1][i], v[2][i]);
    }

    /// Returns an inclusive minimum boundary.
    constexpr Vec3 min() const
    {
        return obj2voxels::min(v[0], v[1], v[2]);
    }

    /// Returns an inclusive maximum boundary.
    constexpr Vec3 max() const
    {
        return obj2voxels::max(v[0], v[1], v[2]);
    }

    /// Returns an inclusive minimum voxel boundary.
    constexpr Vec3u voxelMin() const
    {
        return floor(min()).cast<unsigned>();
    }

    /// Returns an exclusive maximum voxel boundary.
    constexpr Vec3u voxelMax() const
    {
        return floor(max()).cast<unsigned>() + Vec3u::one();
    }

    /// Returns the area of the triangle.
    real_type area() const
    {
        return length(normal()) / 2;
    }

    /// Returns the center of the triangle.
    constexpr Vec3 center() const
    {
        Vec3 sum = v[0] + v[1] + v[2];
        return sum / 3;
    }
};

struct Texture {
    Image image;

    Texture(Image image) : image{std::move(image)}
    {
        image.setWrapMode(WrapMode::CLAMP);
    }

    Vec3f get(Vec2 uv) const
    {
        return image.getPixel(uv).vecf();
    }
};

struct TexturedTriangle : public Triangle {
    Vec2 t[3];

    constexpr Vec2 texture(usize index) const
    {
        VXIO_DEBUG_ASSERT_LT(index, 3);
        return t[index];
    }

    constexpr Vec2 textureCenter() const
    {
        return (t[0] + t[1] + t[2]) / 3;
    }

    constexpr void subdivide4(TexturedTriangle out[4]) const
    {
        Vec3 geo[3]{mix(v[0], v[1], 0.5f), mix(v[1], v[2], 0.5f), mix(v[2], v[0], 0.5f)};
        Vec2 tex[3]{mix(t[0], t[1], 0.5f), mix(t[1], t[2], 0.5f), mix(t[2], t[0], 0.5f)};

        out[0] = {{geo[0], geo[1], geo[2]}, {tex[0], tex[1], tex[2]}};
        out[1] = {{v[0], geo[0], geo[2]}, {t[0], tex[0], tex[2]}};
        out[2] = {{v[1], geo[1], geo[0]}, {t[1], tex[1], tex[0]}};
        out[3] = {{v[2], geo[2], geo[1]}, {t[2], tex[2], tex[1]}};
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
            VXIO_DEBUG_ASSERT_NOTNULL(texture);
            return texture->get({uv.x(), 1 - uv.y()});
        }
        }
        VXIO_DEBUG_ASSERT_UNREACHABLE();
    }
};

}  // namespace obj2voxels

#endif  // TRIANGLE_HPP
