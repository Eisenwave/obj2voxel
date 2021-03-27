#ifndef OBJ2VOXEL_TRIANGLE_HPP
#define OBJ2VOXEL_TRIANGLE_HPP

#include "obj2voxel.h"
#include "util.hpp"

#include "voxelio/image.hpp"
#include "voxelio/vec.hpp"

#include <optional>

namespace obj2voxel {

using namespace voxelio;

using Vec3 = Vec<real_type, 3>;

using Texture = obj2voxel_texture;

/// The type of a triangle in terms of material.
enum class TriangleType : obj2voxel_enum_t {
    NONE,
    /// For triangles with no material. Such triangles are voxelized as white by default.
    MATERIALLESS,
    /// For triangles with no texture but with a solid color.
    UNTEXTURED,
    /// For triangles with a texture.
    TEXTURED
};

// TRIANGLES ===========================================================================================================

using VisualTriangle = ::obj2voxel_triangle;

/// A 3D triangle.
struct Triangle {
    Vec3 v[3];

    /// Returns a vertex at a given index in [0,3).
    constexpr Vec3 vertex(usize index) const noexcept
    {
        VXIO_DEBUG_ASSERT_LT(index, 3u);
        return v[index];
    }

    /// Returns an edge between two vertices.
    constexpr Vec3 edge(usize start, usize end) const noexcept
    {
        return vertex(end) - vertex(start);
    }

    /// Returns an edge from the vertex at the given index to its successor.
    constexpr Vec3 neighborEdge(usize index) const noexcept
    {
        return edge(index, (index + 1) % 3);
    }

    /// Returns the (unnormalized) normal of this triangle.
    constexpr Vec3 normal() const noexcept
    {
        return cross(edge(0, 1), edge(0, 2));
    }

    /// Returns the minimum coordinate of this triangle on one axis.
    constexpr real_type min(usize i) const noexcept
    {
        VXIO_DEBUG_ASSERT_LT(i, 3u);
        return obj2voxel::min(v[0][i], v[1][i], v[2][i]);
    }

    /// Returns the minimum coordinate of this triangle on one axis.
    constexpr real_type max(usize i) const noexcept
    {
        VXIO_DEBUG_ASSERT_LT(i, 3u);
        return obj2voxel::max(v[0][i], v[1][i], v[2][i]);
    }

    /// Returns an inclusive minimum boundary.
    constexpr Vec3 min() const noexcept
    {
        return obj2voxel::min(v[0], v[1], v[2]);
    }

    /// Returns an inclusive maximum boundary.
    constexpr Vec3 max() const noexcept
    {
        return obj2voxel::max(v[0], v[1], v[2]);
    }

    /// Returns an inclusive minimum voxel boundary.
    constexpr Vec3u32 voxelMin() const noexcept
    {
        return floor(min()).cast<u32>();
    }

    /// Returns an exclusive maximum voxel boundary.
    constexpr Vec3u32 voxelMax() const noexcept
    {
        return floor(max()).cast<u32>() + Vec3u32::one();
    }

    /// Returns the area of the triangle.
    real_type area() const noexcept
    {
        return length(normal()) / 2;
    }

    /// Returns the center of the triangle.
    constexpr Vec3 center() const noexcept
    {
        Vec3 sum = v[0] + v[1] + v[2];
        return sum / 3;
    }
};

/// A triangle that also has UV coordinates.
struct TexturedTriangle : public Triangle {
    Vec2f t[3];

    /// Returns the texture coordinates at the index in [0,3).
    constexpr Vec2f texture(usize index) const noexcept
    {
        VXIO_DEBUG_ASSERT_LT(index, 3u);
        return t[index];
    }

    /// Returns the center of the UV coordinates.
    constexpr Vec2f textureCenter() const noexcept
    {
        return (t[0] + t[1] + t[2]) / 3;
    }

    /// Subdivides this triangle into four new triangles in a "triforce" or Sierpinski pattern.
    constexpr void subdivide4(TexturedTriangle out[4]) const noexcept
    {
        Vec3 geo[3]{mix(v[0], v[1], 0.5f), mix(v[1], v[2], 0.5f), mix(v[2], v[0], 0.5f)};
        Vec2f tex[3]{mix(t[0], t[1], 0.5f), mix(t[1], t[2], 0.5f), mix(t[2], t[0], 0.5f)};

        out[0] = {{geo[0], geo[1], geo[2]}, {tex[0], tex[1], tex[2]}};
        out[1] = {{v[0], geo[0], geo[2]}, {t[0], tex[0], tex[2]}};
        out[2] = {{v[1], geo[1], geo[0]}, {t[1], tex[1], tex[0]}};
        out[3] = {{v[2], geo[2], geo[1]}, {t[2], tex[2], tex[1]}};
    }
};

}  // namespace obj2voxel

// API TYPES ===========================================================================================================

/// Wrapper class for voxelio images.
struct obj2voxel_texture {
    std::optional<voxelio::Image> image;

    /// Leaves the image uninitialized.
    obj2voxel_texture() : image{std::nullopt} {}

    /// Constructs the texture from a voxelio image.
    obj2voxel_texture(voxelio::Image image) : image{std::move(image)} {}

    /// Returns the color at the given uv coordinates as a vector.
    voxelio::Vec3f get(voxelio::Vec2f uv) const
    {
        VXIO_DEBUG_ASSERT(image.has_value());
        // TODO move uv transformation on y-axis here instead of doing it in VisualTriangle
        return image->getPixel(uv).vecf();
    }
};

/// A textured triangle that also has material information.
struct obj2voxel_triangle : public obj2voxel::TexturedTriangle {
    obj2voxel::TriangleType type;
    union {
        const obj2voxel_texture *texture;
        obj2voxel::Vec3f color;
    };

    obj2voxel_triangle() : texture{nullptr} {}

    obj2voxel_triangle(obj2voxel::TriangleType type) : type{type}, texture{nullptr} {}

    obj2voxel::Vec3f colorAt_f(obj2voxel::Vec2f uv) const
    {
        VXIO_DEBUG_ASSERT_NE(type, obj2voxel::TriangleType::NONE);
        switch (type) {
        case obj2voxel::TriangleType::NONE: return obj2voxel::Vec3f{1, 0, 1};
        case obj2voxel::TriangleType::MATERIALLESS: return obj2voxel::Vec3f::filledWith(1);
        case obj2voxel::TriangleType::UNTEXTURED: return color;
        case obj2voxel::TriangleType::TEXTURED: {
            VXIO_DEBUG_ASSERT_NOTNULL(texture);
            return texture->get({uv.x(), 1 - uv.y()});
        }
        }
        VXIO_DEBUG_ASSERT_UNREACHABLE();
    }
};

#endif  // OBJ2VOXEL_TRIANGLE_HPP
