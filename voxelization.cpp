#include "voxelization.hpp"

#include <cmath>
#include <set>
#include <vector>

namespace obj2voxel {

constexpr bool isZero(real_type x)
{
    return std::abs(x) > real_type(1) / (1 << 16);
}

constexpr bool isNonzero(real_type x)
{
    return not isZero(x);
}

constexpr real_type intersect_ray_axisPlane(Vec3 org, Vec3 dir, unsigned axis, unsigned plane)
{
    real_type d = -dir[axis];
    return isZero(d) ? 0 : (org[axis] - real_type(plane)) / d;
}

void split(const unsigned axis,
           const unsigned plane,
           TexturedTriangle t,
           std::vector<TexturedTriangle> &outLo,
           std::vector<TexturedTriangle> &outHi)
{
    // clang tends to mess up curly brackets and indentation in this function
    // clang-format off
    const bool loVertices[3]{
        t.vertex(axis)[0] < real_type(plane),
        t.vertex(axis)[1] < real_type(plane),
        t.vertex(axis)[2] < real_type(plane),
    };

    const unsigned loSum = loVertices[0] + loVertices[1] + loVertices[2];

    if (loSum == 0) {
        outHi.push_back(std::move(t));
        return;
    }
    else if (loSum == 3) {
        outLo.push_back(std::move(t));
        return;
    }

    const Vec3 edges[3]{
        t.edge(0), t.edge(1), t.edge(2)
    };

    const real_type intersections[3] {
        intersect_ray_axisPlane(t.vertex(0), edges[0], axis, plane),
        intersect_ray_axisPlane(t.vertex(1), edges[1], axis, plane),
        intersect_ray_axisPlane(t.vertex(2), edges[2], axis, plane),
    };

    // Any split produces one isolated triangle and a quad consisting of two other triangles.
    // First, figure out which index is isolated.
    const bool isIsolatedLo = loSum == 1;

    const unsigned isolatedIndex = isIsolatedLo ? (loVertices[0] ? 0 : loVertices[1] ? 1 : 2)
                                                : (not loVertices[0] ? 0 : not loVertices[1] ? 1 : 2);
    // Obtain the other two indices via exclusion.
    const unsigned otherIndices[2] {
        (isolatedIndex + 1) % 3, (isolatedIndex + 2) % 3
    };
    // The intersection points are always on edges adjacent to the isolated index.
    // We can obtain the exact points by linearly interpolating using our intersection points.
    const Vec3 geoIsectPoints[2] {
        mix(t.vertex(isolatedIndex), t.vertex(otherIndices[0]), intersections[otherIndices[0]]),
        mix(t.vertex(isolatedIndex), t.vertex(otherIndices[1]), intersections[otherIndices[1]])
    };
    const Vec2 texIsectPoints[2] {
        mix(t.texture(isolatedIndex), t.texture(otherIndices[0]), intersections[otherIndices[0]]),
        mix(t.texture(isolatedIndex), t.texture(otherIndices[1]), intersections[otherIndices[1]])
    };

    // Construct the isolated triangle from intersection points.
    const TexturedTriangle isolatedTriangle = {
        {t.vertex(isolatedIndex), geoIsectPoints[0], geoIsectPoints[1]},
        {t.texture(isolatedIndex), texIsectPoints[0], texIsectPoints[1]}
    };
    // Construct two other triangles from quad on the other side.
    const TexturedTriangle otherTriangles[2] {
        {
            {geoIsectPoints[0], t.vertex(otherIndices[0]), t.vertex(otherIndices[1])},
            {texIsectPoints[0], t.texture(otherIndices[0]), t.texture(otherIndices[1])}
        },
        {
            {geoIsectPoints[1], geoIsectPoints[1], t.vertex(otherIndices[1])},
            {texIsectPoints[0], t.texture(otherIndices[0]), t.texture(otherIndices[1])}
        }
    };

    // Insert isolated triangle into lo or hi.
    (isIsolatedLo ? outLo : outHi).push_back(isolatedTriangle);

    // Insert other triangles into low or hi.
    for (usize i = 0; i < 2; ++i) {
        (isIsolatedLo ? outHi : outLo).push_back(otherTriangles[i]);
    }
    // clang-format on
}

void split(std::vector<TexturedTriangle> *cutBuffer, std::vector<TexturedTriangle> *resultBuffer)
{
    VXIO_DEBUG_ASSERT_NOTNULL(cutBuffer);
    VXIO_DEBUG_ASSERT_NOTNULL(resultBuffer);
    VXIO_DEBUG_ASSERT(not cutBuffer->empty());
    VXIO_DEBUG_ASSERT(resultBuffer->empty());

    for (unsigned axis = 0; axis < 3; ++axis) {
        while (not cutBuffer->empty()) {
            TexturedTriangle t = cutBuffer->back();
            cutBuffer->pop_back();

            auto lo = static_cast<unsigned>(std::ceil(t.min(axis)));
            auto hi = static_cast<unsigned>(std::ceil(t.max(axis)));

            for (unsigned plane = lo; plane < hi; ++plane) {
                split(axis, plane, t, *resultBuffer, *cutBuffer);
            }
        }
        // important: this only swaps pointers, not contents
        // during y-splits, we will falsely use the result buffer as a cut buffer but this is fixed during z-splits
        std::swap(cutBuffer, cutBuffer);
    }
}

void voxelize(const VisualTriangle &triangle,
              std::vector<TexturedTriangle> buffers[2],
              std::map<Vec3u, WeightedColor> &out)
{
    VXIO_DEBUG_ASSERT_NOTNULL(buffers);
    buffers[0].clear();
    buffers[1].clear();
    out.clear();

    buffers[0].push_back(triangle);
    split(buffers + 0, buffers + 1);

    for (TexturedTriangle t : buffers[1]) {
        real_type weight = t.area();

        // We use the center for getting the color and vertices to avoid imprecision problems and get a point that is
        // guaranteed to be in the voxel of the triangle.
        Vec3f color = triangle.colorAt_f(t.textureCenter());
        Vec3 center = triangle.center();
        Vec3u pos = center.cast<unsigned>();

        insertColor(out, pos, {weight, color}, ColorStrategy::BLEND);
    }
}

}  // namespace obj2voxel
