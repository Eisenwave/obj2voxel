#ifndef TRIANGLE_HPP
#define TRIANGLE_HPP

#include "util.hpp"

#include "voxelio/image.hpp"
#include "voxelio/vec.hpp"

namespace obj2voxel {

using namespace voxelio;

using Vec2 = Vec<real_type, 2>;
using Vec3 = Vec<real_type, 3>;

/// The type of a triangle in terms of material.
enum class TriangleType {
    /// For triangles with no material. Such triangles are voxelized as white by default.
    MATERIALLESS,
    /// For triangles with no texture but with a solid color.
    UNTEXTURED,
    /// For triangles with a texture.
    TEXTURED
};

// TRIANGLES ===========================================================================================================

/// A 3D triangle.
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
        return obj2voxel::min(v[0][i], v[1][i], v[2][i]);
    }

    /// Returns the minimum coordinate of this triangle on one axis.
    constexpr real_type max(usize i) const
    {
        VXIO_DEBUG_ASSERT_LT(i, 3);
        return obj2voxel::max(v[0][i], v[1][i], v[2][i]);
    }

    /// Returns an inclusive minimum boundary.
    constexpr Vec3 min() const
    {
        return obj2voxel::min(v[0], v[1], v[2]);
    }

    /// Returns an inclusive maximum boundary.
    constexpr Vec3 max() const
    {
        return obj2voxel::max(v[0], v[1], v[2]);
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

/// Wrapper class for voxelio images.
struct Texture {
    Image image;

    /// Constructs the texture from a voxelio image.
    Texture(Image image) : image{std::move(image)} {}

    /// Returns the color at the given uv coordinates as a vector.
    Vec3f get(Vec2 uv) const
    {
        // TODO move uv transformation on y-axis here instead of doing it in VisualTriangle
        return image.getPixel(uv).vecf();
    }
};

/// A triangle that also has UV coordinates.
struct TexturedTriangle : public Triangle {
    Vec2 t[3];

    /// Returns the texture coordinates at the index in [0,3).
    constexpr Vec2 texture(usize index) const
    {
        VXIO_DEBUG_ASSERT_LT(index, 3);
        return t[index];
    }

    /// Returns the center of the UV coordinates.
    constexpr Vec2 textureCenter() const
    {
        return (t[0] + t[1] + t[2]) / 3;
    }

    /// Subdivides this triangle into four new triangles in a "triforce" or Sierpinski pattern.
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

/// A textured triangle that also has material information.
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

}  // namespace obj2voxel

#endif  // TRIANGLE_HPP
