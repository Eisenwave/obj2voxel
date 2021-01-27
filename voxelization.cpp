#include "voxelization.hpp"

#include <cmath>
#include <set>
#include <vector>

namespace obj2voxel {

constexpr real_type EPSILON = real_type(1) / (1 << 16);

constexpr bool isZero(real_type x)
{
    return std::abs(x) < EPSILON;
}

constexpr bool eq(real_type x, unsigned plane)
{
    return isZero(x - real_type(plane));
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
    const bool planarVertices[3]{
        eq(t.vertex(0)[axis], plane),
        eq(t.vertex(1)[axis], plane),
        eq(t.vertex(2)[axis], plane),
    };

    const unsigned planarSum = planarVertices[0] + planarVertices[1] + planarVertices[2];

    // Special case: triangle is parallel tothe splitting plane
    if (planarSum == 3) {
        outHi.push_back(t);
        return;
    }

    const bool loVertices[3]{
        t.vertex(0)[axis] <= real_type(plane),
        t.vertex(1)[axis] <= real_type(plane),
        t.vertex(2)[axis] <= real_type(plane),
    };

    const unsigned loSum = loVertices[0] + loVertices[1] + loVertices[2];

    // Special case: All vertices are on the hi side of the splitting plane
    if (loSum == 0) {
        outHi.push_back(std::move(t));
        return;
    }
    // Special case: All vertices are on the lo side of the splitting plane
    if (loSum == 3) {
        outLo.push_back(std::move(t));
        return;
    }
    // Special case: Two vertices are on the splitting plane, meaning the triangle can't be split
    //               We must still make a decision whether to put it into outLo or outHi
    if (planarSum == 2) {
        const unsigned nonPlanarIndex = not planarVertices[0] ? 0 : not planarVertices[1] ? 1 : 2;
        const bool isNonPlanarLo = t.vertex(nonPlanarIndex)[axis] <= real_type(plane);
        (isNonPlanarLo ? outLo : outHi).push_back(t);
        return;
    }
    // Special case: One vertex lies on the splitting plane
    //               If the remaining vertices are both on one side, we also don't need to split
    //               Otherwise, we only perform one split with the side opposing the planar vertex
    if (planarSum == 1) {
        const unsigned planarIndex = planarVertices[0] ? 0 : planarVertices[1] ? 1 : 2;
        const unsigned nonPlanarIndices[2]{
            (planarIndex + 1) % 3,
            (planarIndex + 2) % 3
        };

        const unsigned nonPlanarLoSum = loVertices[nonPlanarIndices[0]] + loVertices[nonPlanarIndices[1]];
        // Special case: Both non-planar vertices lie on one side of the splitting plane
        if (nonPlanarLoSum != 1) {
            const bool areNonPlanarsLo = nonPlanarLoSum == 2;
            (areNonPlanarsLo ? outLo : outHi).push_back(t);
            return;
        }
        // Special case: The splitting plane goes through one vertex
        //               Instead of producing a triangle and a quad, we only produce two triangles
        //               We only perform one intersection test with the opposing edge
        const Vec3 planarVertex = t.vertex(planarIndex);
        const Vec2 planarTexture = t.texture(planarIndex);
        const Vec3 nonPlanarVertices[2] {
            t.vertex(nonPlanarIndices[0]),
            t.vertex(nonPlanarIndices[1])
        };
        const Vec2 nonPlanarTextures[2] {
            t.texture(nonPlanarIndices[0]),
            t.texture(nonPlanarIndices[1])
        };
        const Vec3 nonPlanarEdge = nonPlanarVertices[1] - nonPlanarVertices[0];

        const real_type intersection = intersect_ray_axisPlane(nonPlanarVertices[0], nonPlanarEdge, axis, plane);
        const Vec3 geoIntersection = mix(nonPlanarVertices[0], nonPlanarVertices[1], intersection);
        const Vec2 texIntersection = mix(nonPlanarTextures[0], nonPlanarTextures[1], intersection);

        const TexturedTriangle triangles[2] {
            {
                {planarVertex, nonPlanarVertices[0], geoIntersection},
                {planarTexture, nonPlanarTextures[0], texIntersection}
            },
            {
                {planarVertex, geoIntersection, nonPlanarVertices[1]},
                {planarTexture, texIntersection, nonPlanarTextures[1]}
            }
        };

        const bool isFirstTriangleLo = loVertices[nonPlanarIndices[0]];

        outLo.push_back(triangles[1 - isFirstTriangleLo]);
        outHi.push_back(triangles[0 + isFirstTriangleLo]);
        return;
    }
    // Regular case: None of the vertices are planar and the triangle is intersected by the splitting plane
    //               We put this into an else-block to make debugging easier by reducing the number of variables
    else {
        VXIO_DEBUG_ASSERT(loSum == 1 || loSum == 2);
        VXIO_DEBUG_ASSERT_EQ(planarSum, 0);

        // Any split produces one isolated triangle and a quad consisting of two other triangles.
        // First, figure out which index is isolated.
        const bool isIsolatedLo = loSum == 1;

        const unsigned isolatedIndex = isIsolatedLo ? (loVertices[0] ? 0 : loVertices[1] ? 1 : 2)
                                                    : (not loVertices[0] ? 0 : not loVertices[1] ? 1 : 2);
        // Obtain the other two indices via exclusion.
        const unsigned otherIndices[2] {
            (isolatedIndex + 1) % 3,
            (isolatedIndex + 2) % 3
        };

        // The intersection points are always on edges adjacent to the isolated index.
        // We can obtain the exact points by linearly interpolating using our intersection points.
        const Vec3 isolatedVertex = t.vertex(isolatedIndex);
        const Vec2 isolatedTexture = t.texture(isolatedIndex);

        const Vec3 otherVertices[2] {
            t.vertex(otherIndices[0]),
            t.vertex(otherIndices[1])
        };
        const Vec2 otherTextures[2] {
            t.texture(otherIndices[0]),
            t.texture(otherIndices[1])
        };
        const Vec3 edgesToOtherVertices[2] {
            t.vertex(otherIndices[0]) - isolatedVertex,
            t.vertex(otherIndices[1]) - isolatedVertex
        };

        const real_type intersections[2] {
            intersect_ray_axisPlane(isolatedVertex, edgesToOtherVertices[0], axis, plane),
            intersect_ray_axisPlane(isolatedVertex, edgesToOtherVertices[1], axis, plane),
        };

        const Vec3 geoIsectPoints[2] {
            mix(isolatedVertex, otherVertices[0], intersections[0]),
            mix(isolatedVertex, otherVertices[1], intersections[1])
        };
        const Vec2 texIsectPoints[2] {
            mix(isolatedTexture, otherTextures[0], intersections[0]),
            mix(isolatedTexture, otherTextures[1], intersections[1])
        };

        // Construct the isolated triangle from intersection points.
        const TexturedTriangle isolatedTriangle = {
            {isolatedVertex, geoIsectPoints[0], geoIsectPoints[1]},
            {isolatedTexture, texIsectPoints[0], texIsectPoints[1]}
        };
        // Construct two other triangles from quad on the other side.
        const TexturedTriangle otherTriangles[2] {
            {
                {geoIsectPoints[0], otherVertices[0], otherVertices[1]},
                {texIsectPoints[0], otherTextures[0], otherTextures[1]}
            },
            {
                {geoIsectPoints[0], geoIsectPoints[1], otherVertices[1]},
                {texIsectPoints[0], texIsectPoints[1], otherTextures[1]}
            }
        };

        // Insert isolated triangle into lo or hi.
        (isIsolatedLo ? outLo : outHi).push_back(isolatedTriangle);

        // Insert other triangles into low or hi.
        for (usize i = 0; i < 2; ++i) {
            (isIsolatedLo ? outHi : outLo).push_back(otherTriangles[i]);
        }
        return;
    }
    VXIO_DEBUG_ASSERT_UNREACHABLE();
    // clang-format on
}

void split(std::vector<TexturedTriangle> *preSplitBuffer,
           std::vector<TexturedTriangle> *postSplitBuffer,
           std::vector<TexturedTriangle> *resultBuffer)
{
    VXIO_DEBUG_ASSERT_NOTNULL(preSplitBuffer);
    VXIO_DEBUG_ASSERT_NOTNULL(postSplitBuffer);
    VXIO_DEBUG_ASSERT_NOTNULL(resultBuffer);

    VXIO_DEBUG_ASSERT(not preSplitBuffer->empty());
    VXIO_DEBUG_ASSERT(postSplitBuffer->empty());
    VXIO_DEBUG_ASSERT(resultBuffer->empty());

    std::vector<TexturedTriangle> *const originalPreSplitBuffer = preSplitBuffer;
    std::vector<TexturedTriangle> *const originalPostSplitBuffer = postSplitBuffer;

    for (unsigned axis = 0; axis < 3; ++axis) {
        do {
            while (not preSplitBuffer->empty()) {
                // no need to pop back here
                TexturedTriangle triangle = preSplitBuffer->back();
                preSplitBuffer->pop_back();

                real_type min = triangle.min(axis);
                auto lo = static_cast<unsigned>(std::ceil(min));
                // auto hi = static_cast<unsigned>(std::ceil(triangle.max(axis)));

                split(axis, lo, triangle, *resultBuffer, *postSplitBuffer);
            }

            // Everything that used to be after the split (hi side) now comes before the next round of splitting
            std::swap(preSplitBuffer, postSplitBuffer);
        } while (not postSplitBuffer->empty());

        VXIO_DEBUG_ASSERT(preSplitBuffer->empty());
        VXIO_DEBUG_ASSERT(postSplitBuffer->empty());

        // We might have swapped our buffers an odd amount of times so we need to ensure that the next swap is valid
        preSplitBuffer = originalPreSplitBuffer;
        postSplitBuffer = originalPostSplitBuffer;

        // important: this only swaps pointers, not contents
        // during y-splits, we will falsely use the result buffer as a cut buffer but this is fixed during z-splits
        std::swap(preSplitBuffer, resultBuffer);
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
