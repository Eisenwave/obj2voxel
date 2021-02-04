#include "voxelization.hpp"

#include <cmath>
#include <set>
#include <vector>

namespace obj2voxel {

void (*globalTriangleDebugCallback)(Triangle) = [](auto) {};

namespace {

// UTILITY & CONSTANTS =================================================================================================

constexpr bool DISABLE_PLANE_DISTANCE_TEST = false;
constexpr real_type EPSILON = real_type(1) / (1 << 16);

inline bool isZero(real_type x)
{
    return std::abs(x) < EPSILON;
}

inline bool eq(real_type x, unsigned plane)
{
    return isZero(x - real_type(plane));
}

inline real_type intersect_ray_axisPlane(Vec3 org, Vec3 dir, unsigned axis, unsigned plane)
{
    real_type d = -dir[axis];
    return isZero(d) ? 0 : (org[axis] - real_type(plane)) / d;
}

/// Returns true if two floating point numbers are exactly equal without warnings (-Wfloat-equal).
template <typename Float>
constexpr bool eqExactly(Float a, Float b)
{
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wfloat-equal"
    return a == b;
#pragma clang diagnostic pop
}

/**
 * @brief Returns the signed distance of a point and a plane.
 * @param p the point
 * @param org any point on the plane
 * @param normal the normalized normal of the plane
 * @return the signed distance of the point and the plane
 */
constexpr real_type distance_point_plane(Vec3 p, Vec3 org, Vec3 normal)
{
    return dot(normal, p - org);
}

/// Emplaces a weighted type in a map at the given location or combines it with the already value.
template <ColorStrategy STRATEGY, typename T>
inline void insertWeighted(VoxelMap<Weighted<T>> &map, Vec3u pos, Weighted<T> color)
{
    auto [location, success] = map.emplace(pos, color);
    if (not success) {
        location->second = STRATEGY == ColorStrategy::MAX ? max(color, location->second) : mix(color, location->second);
    }
}

constexpr InsertionFunction insertionFunctionOf(ColorStrategy colorStrategy)
{
    return colorStrategy == ColorStrategy::BLEND ? insertWeighted<ColorStrategy::BLEND, Vec3>
                                                 : insertWeighted<ColorStrategy::MAX, Vec3>;
}

// TRIANGLE SPLITTING ==================================================================================================

enum class DiscardMode {
    /// No triangles are discarded, they are merely sorted into outLo and outHi. This is the default.
    NONE,
    /// Triangles that would be sorted into lo are discarded instead.
    DISCARD_LO,
    /// Triangles that would be sorted into hi are discarded instead.
    DISCARD_HI
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
template <DiscardMode DISCARD_MODE = DiscardMode::NONE>
void splitTriangle(const unsigned axis,
                   const unsigned plane,
                   TexturedTriangle t,
                   std::vector<TexturedTriangle> &outLo,
                   std::vector<TexturedTriangle> &outHi)
{
    // This local helper function lets us insert into outLo or outHi using a one-liner while still considering the
    // DISCARD_MODE.
    const auto pushLoIfTrueElseHi = [&](TexturedTriangle t, bool lo) -> void {
        if constexpr (DISCARD_MODE == DiscardMode::NONE) {
            (lo ? outLo : outHi).push_back(std::move(t));
        }
        else if constexpr (DISCARD_MODE == DiscardMode::DISCARD_LO) {
            if (not lo) {
                outHi.push_back(std::move(t));
            }
        }
        else if constexpr (DISCARD_MODE == DiscardMode::DISCARD_HI) {
            if (lo) {
                outLo.push_back(std::move(t));
            }
        }
    };

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
        pushLoIfTrueElseHi(std::move(t), true);
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
        pushLoIfTrueElseHi(std::move(t), false);
        return;
    }
    // Special case: All vertices are on the lo side of the splitting plane
    if (loSum == 3) {
        pushLoIfTrueElseHi(std::move(t), true);
        return;
    }
    // Special case: Two vertices are on the splitting plane, meaning the triangle can't be split
    //               We must still make a decision whether to put it into outLo or outHi
    if (planarSum == 2) {
        const unsigned nonPlanarIndex = not planarVertices[0] ? 0 : not planarVertices[1] ? 1 : 2;
        const bool isNonPlanarLo = t.vertex(nonPlanarIndex)[axis] <= real_type(plane);
        pushLoIfTrueElseHi(std::move(t), isNonPlanarLo);
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
            pushLoIfTrueElseHi(std::move(t), areNonPlanarsLo);
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
        const Vec3 geoIntersection = obj2voxel::mix(nonPlanarVertices[0], nonPlanarVertices[1], intersection);
        const Vec2 texIntersection = obj2voxel::mix(nonPlanarTextures[0], nonPlanarTextures[1], intersection);

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

        pushLoIfTrueElseHi(std::move(triangles[0]), isFirstTriangleLo);
        pushLoIfTrueElseHi(std::move(triangles[1]), not isFirstTriangleLo);
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
            obj2voxel::mix(isolatedVertex, otherVertices[0], intersections[0]),
            obj2voxel::mix(isolatedVertex, otherVertices[1], intersections[1])
        };
        const Vec2 texIsectPoints[2] {
            obj2voxel::mix(isolatedTexture, otherTextures[0], intersections[0]),
            obj2voxel::mix(isolatedTexture, otherTextures[1], intersections[1])
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
        pushLoIfTrueElseHi(std::move(isolatedTriangle), isIsolatedLo);

        // Insert other triangles into low or hi.
        for (usize i = 0; i < 2; ++i) {
            pushLoIfTrueElseHi(std::move(otherTriangles[i]), not isIsolatedLo);
        }
        return;
    }
    VXIO_DEBUG_ASSERT_UNREACHABLE();
    // clang-format on
}

/**
 * @brief Subdivides a buffer of triangles.
 * Each triangle in the buffer is split into four smaller triangles if its volume in the voxel grid is too great.
 * @param triangles the buffer of triangles
 */
void subdivideLargeVolumeTriangles(TexturedTriangle inputTriangle, std::vector<TexturedTriangle> &out)
{
    constexpr usize volumeLimit = 512;
    constexpr real_type sqrtThird = real_type(0.5773502691896257645091487805019574556476017512701268760186023264);
    constexpr Vec3 diagonal3 = {sqrtThird, sqrtThird, sqrtThird};

    // If a triangle is parallel to one of the axes, we don't need to subdivide it.
    // Even if the volume is large, very few intersection tests will fail.
    const Vec3 normal = normalize(abs(inputTriangle.normal()));
    const real_type diagonality = dot(normal, diagonal3);
    const real_type diagonality01 = (diagonality - sqrtThird) / (1 - sqrtThird);

    VXIO_DEBUG_ASSERT(out.empty());
    out.push_back(inputTriangle);

    // This corresponds to an angle of 60° or higher from the diagonal vector
    if (diagonality01 < 1 / 2.f) {
        return;
    }

    for (usize i = 0; i < out.size();) {
        const TexturedTriangle triangle = out[i];

        const Vec3u vmin = triangle.voxelMin();
        const Vec3u vmax = triangle.voxelMax();
        VXIO_DEBUG_ASSERT(vmin != vmax);
        const Vec3u size = vmax - vmin;
        const unsigned volume = size[0] * size[1] * size[2];

        if (volume < volumeLimit) {
            ++i;
            continue;
        }

        // The center piece of subdivision replaces the current triangle.
        // The other three corner triangles are appended.
        // We don't increment i here so that we can subdivide the center piece again if necessary.
        TexturedTriangle subTriangles[4];
        triangle.subdivide4(subTriangles);
        out[i] = std::move(subTriangles[0]);
        for (usize j = 1; j < 4; ++j) {
            out.push_back(std::move(subTriangles[j]));
        }
    }
}

// VOXELIZATION ========================================================================================================

[[nodiscard]] WeightedUv voxelizeVoxel(const VisualTriangle &inputTriangle,
                                       Vec3u pos,
                                       std::vector<TexturedTriangle> *preSplitBuffer,
                                       std::vector<TexturedTriangle> *postSplitBuffer)
{
    for (unsigned hi = 0; hi < 2; ++hi) {
        const auto splittingFunction =
            hi ? splitTriangle<DiscardMode::DISCARD_HI> : splitTriangle<DiscardMode::DISCARD_LO>;

        for (unsigned axis = 0; axis < 3; ++axis) {
            const unsigned plane = pos[axis] + hi;

            for (const TexturedTriangle t : *preSplitBuffer) {
                splittingFunction(axis, plane, t, *postSplitBuffer, *postSplitBuffer);
            }

            preSplitBuffer->clear();
            if (postSplitBuffer->empty()) {
                return {0, {}};
            }

            std::swap(preSplitBuffer, postSplitBuffer);
        }
    }

    // Restore original semantics.
    std::swap(preSplitBuffer, postSplitBuffer);

    VXIO_DEBUG_ASSERT(preSplitBuffer->empty());
    VXIO_DEBUG_ASSERTM(not postSplitBuffer->empty(), "Function should have been returned from early otherwise");

    WeightedUv result{};
    for (const TexturedTriangle t : *postSplitBuffer) {
        const float weight = static_cast<float>(inputTriangle.area());
        const Vec2 uv = t.textureCenter();

        result = mix(result, {weight, uv});
    }

    postSplitBuffer->clear();
    return result;
}

void voxelizeSubTriangle(const VisualTriangle &inputTriangle,
                         TexturedTriangle subTriangle,
                         std::vector<TexturedTriangle> *preSplitBuffer,
                         std::vector<TexturedTriangle> *postSplitBuffer,
                         VoxelMap<WeightedUv> &out)
{
    // sqrt(3) = 1.73... with some leeway to account for imprecision
    constexpr real_type distanceLimit = 2;

    const Vec3 planeOrg = subTriangle.vertex(0);
    const Vec3 planeNormal = normalize(subTriangle.normal());

    const Vec3u vmin = subTriangle.voxelMin();
    const Vec3u vmax = subTriangle.voxelMax();

    for (unsigned z = vmin.z(); z < vmax.z(); ++z) {
        for (unsigned y = vmin.y(); y < vmax.y(); ++y) {
            for (unsigned x = vmin.x(); x < vmax.x(); ++x) {
                const Vec3u pos = {x, y, z};

                if constexpr (not DISABLE_PLANE_DISTANCE_TEST) {
                    const Vec3 center = pos + Vec3::filledWith(0.5f);
                    const real_type signedDistance = distance_point_plane(center, planeOrg, planeNormal);

                    if (std::abs(signedDistance) > distanceLimit) {
                        continue;
                    }
                }

                VXIO_DEBUG_ASSERT(preSplitBuffer->empty());
                VXIO_DEBUG_ASSERT(postSplitBuffer->empty());

                preSplitBuffer->push_back(subTriangle);
                WeightedUv uv = voxelizeVoxel(inputTriangle, pos, preSplitBuffer, postSplitBuffer);

                if (not eqExactly(uv.weight, 0.f)) {
                    insertWeighted<ColorStrategy::BLEND>(out, pos, uv);
                }
            }
        }
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
void voxelizeTriangle(VisualTriangle inputTriangle, std::vector<TexturedTriangle> buffers[3], VoxelMap<WeightedUv> &out)
{
    VXIO_DEBUG_ASSERT_NOTNULL(buffers);
    for (size_t i = 0; i < 3; ++i) {
        buffers[i].clear();
    }
    out.clear();

    // 1. Subdivide
    subdivideLargeVolumeTriangles(inputTriangle, buffers[0]);

    if constexpr (voxelio::build::DEBUG) {
        for (const Triangle &t : buffers[0]) {
            globalTriangleDebugCallback(t);
        }
    }

    // 2. Voxelize
    for (TexturedTriangle subTriangle : buffers[0]) {
        voxelizeSubTriangle(inputTriangle, subTriangle, buffers + 1, buffers + 2, out);
    }
}

}  // namespace

// DOWNSCALING =========================================================================================================

VoxelMap<WeightedColor> downscale(VoxelMap<WeightedColor> voxels, ColorStrategy strategy, unsigned divisor)
{
    VXIO_ASSERTM(isPow2(divisor), "Divisor must be power of 2 but is " + stringify(divisor));

    if (divisor == 1) {
        return voxels;
    }

    VoxelMap<WeightedColor> result;

    const InsertionFunction insert = insertionFunctionOf(strategy);

    for (auto iter = voxels.begin(); iter != voxels.end(); iter = voxels.erase(iter)) {
        const Vec3u32 pos = voxels.posOf(iter->first) / divisor;
        insert(result, pos, iter->second);
    }

    return result;
}

// VOXELIZER IMPLEMENTATION ============================================================================================

Voxelizer::Voxelizer(ColorStrategy colorStrategy) : insertionFunction{insertionFunctionOf(colorStrategy)} {}

void Voxelizer::initTransform(Vec3 min, Vec3 max, unsigned resolution, Vec3u permutation)
{
    meshMin = min;
    meshMax = max;

    const Vec3 meshSize = max - min;
    const real_type maxOfAllAxes = obj2voxel::max(meshSize[0], meshSize[1], meshSize[2]);
    const real_type scaleFactor = (real_type(resolution) - ANTI_BLEED) / maxOfAllAxes;
    const Vec3 baseTranslation = (-meshMin * scaleFactor) + Vec3::filledWith(ANTI_BLEED / 2);

    for (usize i = 0; i < 3; ++i) {
        linearTransform[i] = Vec3::zero();
        linearTransform[i][permutation[i]] = scaleFactor;
        translation[i] = baseTranslation[permutation[i]];
        VXIO_LOG(DEBUG, "Mesh transform [" + stringify(i) + "] = " + linearTransform[i].toString());
    }

    VXIO_LOG(DEBUG, "Mesh translation = " + translation.toString());
}

Vec3 Voxelizer::transform(Vec3 v)
{
    real_type x = dot(linearTransform[0], v);
    real_type y = dot(linearTransform[1], v);
    real_type z = dot(linearTransform[2], v);
    return Vec3{x, y, z} + translation;
}

void Voxelizer::voxelize(VisualTriangle triangle)
{
    VXIO_ASSERT(uvBuffer.empty());

    for (usize i = 0; i < 3; ++i) {
        triangle.v[i] = transform(triangle.v[i]);
    }

    ++triangleCount;
    obj2voxel::voxelizeTriangle(triangle, buffers, uvBuffer);
    for (auto &[index, weightedUv] : uvBuffer) {
        Vec3u32 pos = uvBuffer.posOf(index);
        Vec3f color = triangle.colorAt_f(weightedUv.value);
        this->insertionFunction(voxels, pos, {weightedUv.weight, color});
    }
    uvBuffer.clear();
}

}  // namespace obj2voxel
