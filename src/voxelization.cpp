#include "voxelization.hpp"

#include "constants.hpp"

#include <cmath>

namespace obj2voxel {

void (*globalTriangleDebugCallback)(Triangle) = [](auto) {};

namespace {

// UTILITY & CONSTANTS =================================================================================================

constexpr real_type EPSILON = real_type(1) / (1 << 16);

inline bool isZero(real_type x) noexcept
{
    return std::fabs(x) < EPSILON;
}

inline bool eq(real_type x, u32 plane) noexcept
{
    return isZero(x - real_type(plane));
}

inline real_type intersect_ray_axisPlane(Vec3 org, Vec3 dir, u32 axis, u32 plane) noexcept
{
    real_type d = -dir[axis];
    return isZero(d) ? 0 : (org[axis] - real_type(plane)) / d;
}

/**
 * @brief Returns the signed distance of a point and a plane.
 * @param p the point
 * @param org any point on the plane
 * @param normal the normalized normal of the plane
 * @return the signed distance of the point and the plane
 */
constexpr real_type distance_point_plane(Vec3 p, Vec3 org, Vec3 normal) noexcept
{
    return dot(normal, p - org);
}

template <typename T>
constexpr WeightedCombineFunction<T> weightedMaxFunction = obj2voxel::max<T>;

template <typename T>
constexpr WeightedCombineFunction<T> weightedMixFunction = obj2voxel::mix<T>;

template <ColorStrategy STRATEGY, typename T>
constexpr WeightedCombineFunction<T> combineFunction =
    STRATEGY == ColorStrategy::MAX ? weightedMaxFunction<T> : weightedMixFunction<T>;

/// Emplaces a weighted type in a map at the given location or combines it with the already value.
template <ColorStrategy STRATEGY, typename T>
inline void insertWeighted(VoxelMap<Weighted<T>> &map, Vec3u pos, Weighted<T> color) noexcept
{
    auto [location, success] = map.emplace(pos, color);
    if (not success) {
        location->second = combineFunction<STRATEGY, T>(color, location->second);
    }
}

constexpr WeightedCombineFunction<Vec3f> combineFunctionOf(ColorStrategy colorStrategy) noexcept
{
    return colorStrategy == ColorStrategy::BLEND ? combineFunction<ColorStrategy::BLEND, Vec3>
                                                 : combineFunction<ColorStrategy::MAX, Vec3>;
}

// TRIANGLE SPLITTING ==================================================================================================

using split_buffer_type = Voxelizer::split_buffer_type;

enum class DiscardMode {
    /// No triangles are discarded, they are merely sorted into outLo and outHi. This is the default.
    NONE,
    /// Triangles that would be sorted into lo are discarded instead.
    DISCARD_LO,
    /// Triangles that would be sorted into hi are discarded instead.
    DISCARD_HI
};

/// This helper functor lets us insert into outLo or outHi using a one-liner while still considering the DISCARD_MODE.
template <DiscardMode DISCARD_MODE>
struct LoHiPusher {
    split_buffer_type &outLo;
    split_buffer_type &outHi;

    void operator()(TexturedTriangle t, bool lo) const noexcept
    {
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
    }
};

constexpr bool IS_LO_BIASED = false;

struct SplittingValues {
    u32 plane;
    u8 axis;

    u8 loSum = 0;
    u8 planarSum = 0;

    // there are only three vertices, the last bool is padding and remains uninitialized
    bool loVertices[4];
    bool planarVertices[4];

    SplittingValues(u32 plane, u8 axis, const Triangle &triangle) noexcept : plane{plane}, axis{axis}
    {
        for (unsigned i = 0; i < 3; ++i) {
            planarVertices[i] = eq(triangle.vertex(i)[axis], plane);

            const real_type v = triangle.vertex(i)[axis];
            loVertices[i] = IS_LO_BIASED ? (v <= real_type(plane)) : (v < real_type(plane));

            planarSum += planarVertices[i];
            loSum += loVertices[i];
        }
    }

    constexpr u8 firstLo() const noexcept
    {
        return loVertices[0] ? 0 : loVertices[1] ? 1 : 2;
    }

    constexpr u8 firstHi() const noexcept
    {
        return not loVertices[0] ? 0 : not loVertices[1] ? 1 : 2;
    }

    constexpr u8 firstPlanar() const noexcept
    {
        return planarVertices[0] ? 0 : planarVertices[1] ? 1 : 2;
    }

    constexpr u8 firstNonplanar() const noexcept
    {
        return not planarVertices[0] ? 0 : not planarVertices[1] ? 1 : 2;
    }
};

template <DiscardMode DISCARD_MODE>
void splitTriangle_onePlanarCase(const TexturedTriangle &t,
                                 SplittingValues val,
                                 LoHiPusher<DISCARD_MODE> pushLoIfTrueElseHi) noexcept;

template <DiscardMode DISCARD_MODE>
void splitTriangle_regularCase(const TexturedTriangle &t,
                               SplittingValues val,
                               LoHiPusher<DISCARD_MODE> pushLoIfTrueElseHi) noexcept;

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
void splitTriangle(const u32 axis,
                   const u32 plane,
                   const TexturedTriangle &t,
                   split_buffer_type &outLo,
                   split_buffer_type &outHi) noexcept
{
    VXIO_DEBUG_ASSERT_LT(axis, 3u);

    const SplittingValues val{plane, static_cast<u8>(axis), t};
    const LoHiPusher<DISCARD_MODE> pushLoIfTrueElseHi{outLo, outHi};

    // clang tends to mess up curly brackets and indentation in this function
    // clang-format off
#define OBJ2VOXEL_LO_AND_PLANAR(lo, planar) static_cast<u8>(planar << 2u) | lo
    const u8 caseNum = OBJ2VOXEL_LO_AND_PLANAR(val.loSum, val.planarSum);

    switch (caseNum) {
    // Special case: All vertices are on the hi side of the splitting plane.
    case OBJ2VOXEL_LO_AND_PLANAR(0, 0):
    case OBJ2VOXEL_LO_AND_PLANAR(0, 1):
    case OBJ2VOXEL_LO_AND_PLANAR(0, 2):
    case OBJ2VOXEL_LO_AND_PLANAR(0, 3):
        return pushLoIfTrueElseHi(t, false);

    // Special case: All vertices are on the lo side of the splitting plane
    case OBJ2VOXEL_LO_AND_PLANAR(3, 0):
    case OBJ2VOXEL_LO_AND_PLANAR(3, 1):
    case OBJ2VOXEL_LO_AND_PLANAR(3, 2):
    case OBJ2VOXEL_LO_AND_PLANAR(3, 3):
        return pushLoIfTrueElseHi(t, true);

    // Special case: Triangle is parallel to the splitting plane (except if all vertices are hi or lo)
    //               We push based on our bias.
    case OBJ2VOXEL_LO_AND_PLANAR(1, 3):
    case OBJ2VOXEL_LO_AND_PLANAR(2, 3):
        return pushLoIfTrueElseHi(t, IS_LO_BIASED);

    // Special case: Two vertices are on the splitting plane, meaning the triangle can't be split
    //               We must still make a decision whether to put it into outLo or outHi
    case OBJ2VOXEL_LO_AND_PLANAR(1, 2):
    case OBJ2VOXEL_LO_AND_PLANAR(2, 2):
        return pushLoIfTrueElseHi(t, val.loVertices[val.firstNonplanar()]);

    // Special case: One vertex lies on the splitting plane
    //               If the remaining vertices are both on one side, we also don't need to split
    //               Otherwise, we only perform one split with the side opposing the planar vertex
    case OBJ2VOXEL_LO_AND_PLANAR(1, 1):
    case OBJ2VOXEL_LO_AND_PLANAR(2, 1): {
        return splitTriangle_onePlanarCase(t, val, pushLoIfTrueElseHi);
    }

    // Regular case: None of the vertices are planar and the triangle is intersected by the splitting plane
    //               We put this into an else-block to make debugging easier by reducing the number of variables
    case OBJ2VOXEL_LO_AND_PLANAR(1, 0):
    case OBJ2VOXEL_LO_AND_PLANAR(2, 0): {
        return splitTriangle_regularCase(t, val, pushLoIfTrueElseHi);
    }

    } // switch

    VXIO_DEBUG_ASSERT_UNREACHABLE();
    // clang-format on
}

template <DiscardMode DISCARD_MODE>
void splitTriangle_onePlanarCase(const TexturedTriangle &t,
                                 const SplittingValues val,
                                 const LoHiPusher<DISCARD_MODE> pushLoIfTrueElseHi) noexcept
{
    const unsigned planarIndex = val.firstPlanar();
    const unsigned nonPlanarIndices[2]{(planarIndex + 1) % 3, (planarIndex + 2) % 3};

    const unsigned nonPlanarLoSum = val.loVertices[nonPlanarIndices[0]] + val.loVertices[nonPlanarIndices[1]];
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
    const Vec2f planarTexture = t.texture(planarIndex);
    const Vec3 nonPlanarVertices[2]{t.vertex(nonPlanarIndices[0]), t.vertex(nonPlanarIndices[1])};
    const Vec2f nonPlanarTextures[2]{t.texture(nonPlanarIndices[0]), t.texture(nonPlanarIndices[1])};
    const Vec3 nonPlanarEdge = nonPlanarVertices[1] - nonPlanarVertices[0];

    const real_type intersection = intersect_ray_axisPlane(nonPlanarVertices[0], nonPlanarEdge, val.axis, val.plane);
    const Vec3 geoIntersection = obj2voxel::mix(nonPlanarVertices[0], nonPlanarVertices[1], intersection);
    const Vec2f texIntersection = obj2voxel::mix(nonPlanarTextures[0], nonPlanarTextures[1], intersection);

    const TexturedTriangle triangles[2]{
        {{planarVertex, nonPlanarVertices[0], geoIntersection}, {planarTexture, nonPlanarTextures[0], texIntersection}},
        {{planarVertex, geoIntersection, nonPlanarVertices[1]},
         {planarTexture, texIntersection, nonPlanarTextures[1]}}};

    const bool isFirstTriangleLo = val.loVertices[nonPlanarIndices[0]];

    pushLoIfTrueElseHi(std::move(triangles[0]), isFirstTriangleLo);
    pushLoIfTrueElseHi(std::move(triangles[1]), not isFirstTriangleLo);
}

template <DiscardMode DISCARD_MODE>
void splitTriangle_regularCase(const TexturedTriangle &t,
                               const SplittingValues val,
                               const LoHiPusher<DISCARD_MODE> pushLoIfTrueElseHi) noexcept
{
    VXIO_DEBUG_ASSERT(val.loSum == 1 || val.loSum == 2);
    VXIO_DEBUG_ASSERT_EQ(val.planarSum, 0);

    // Any split produces one isolated triangle and a quad consisting of two other triangles.
    // First, figure out which index is isolated.
    const bool isIsolatedLo = val.loSum == 1;

    const unsigned isolatedIndex = isIsolatedLo ? val.firstLo() : val.firstHi();
    // Obtain the other two indices via exclusion.
    const unsigned otherIndices[2]{(isolatedIndex + 1) % 3, (isolatedIndex + 2) % 3};

    // The intersection points are always on edges adjacent to the isolated index.
    // We can obtain the exact points by linearly interpolating using our intersection points.
    const Vec3 isolatedVertex = t.vertex(isolatedIndex);
    const Vec2f isolatedTexture = t.texture(isolatedIndex);

    const Vec3 otherVertices[2]{t.vertex(otherIndices[0]), t.vertex(otherIndices[1])};
    const Vec2f otherTextures[2]{t.texture(otherIndices[0]), t.texture(otherIndices[1])};
    const Vec3 edgesToOtherVertices[2]{t.vertex(otherIndices[0]) - isolatedVertex,
                                       t.vertex(otherIndices[1]) - isolatedVertex};

    const real_type intersections[2]{
        intersect_ray_axisPlane(isolatedVertex, edgesToOtherVertices[0], val.axis, val.plane),
        intersect_ray_axisPlane(isolatedVertex, edgesToOtherVertices[1], val.axis, val.plane),
    };

    const Vec3 geoIsectPoints[2]{obj2voxel::mix(isolatedVertex, otherVertices[0], intersections[0]),
                                 obj2voxel::mix(isolatedVertex, otherVertices[1], intersections[1])};
    const Vec2f texIsectPoints[2]{obj2voxel::mix(isolatedTexture, otherTextures[0], intersections[0]),
                                  obj2voxel::mix(isolatedTexture, otherTextures[1], intersections[1])};

    // Construct the isolated triangle from intersection points.
    const TexturedTriangle isolatedTriangle = {{isolatedVertex, geoIsectPoints[0], geoIsectPoints[1]},
                                               {isolatedTexture, texIsectPoints[0], texIsectPoints[1]}};
    // Construct two other triangles from quad on the other side.
    const TexturedTriangle otherTriangles[2]{{{geoIsectPoints[0], otherVertices[0], otherVertices[1]},
                                              {texIsectPoints[0], otherTextures[0], otherTextures[1]}},
                                             {{geoIsectPoints[0], geoIsectPoints[1], otherVertices[1]},
                                              {texIsectPoints[0], texIsectPoints[1], otherTextures[1]}}};

    // Insert isolated triangle into lo or hi.
    pushLoIfTrueElseHi(std::move(isolatedTriangle), isIsolatedLo);

    // Insert other triangles into low or hi.
    for (usize i = 0; i < 2; ++i) {
        pushLoIfTrueElseHi(std::move(otherTriangles[i]), not isIsolatedLo);
    }
}

// SUBDIVISION =========================================================================================================

constexpr bool isRoughlyAlignedWithAnyAxisPlane(const TexturedTriangle &triangle) noexcept
{
    constexpr real_type sqrtThird = real_type(0.5773502691896257645091487805019574556476017512701268760186023264);
    constexpr Vec3 diagonal3 = {sqrtThird, sqrtThird, sqrtThird};

    // If a triangle is parallel to one of the axes, we don't need to subdivide it.
    // Even if the volume is large, very few intersection tests will fail.
    const Vec3 normal = normalize(abs(triangle.normal()));
    const real_type diagonality = dot(normal, diagonal3);
    const real_type diagonality01 = (diagonality - sqrtThird) / (1 - sqrtThird);

    return diagonality01 < COS_SUBDIVISION_DIAGONALITY_LIMIT;
}

template <typename Action, std::enable_if_t<std::is_invocable_v<Action, TexturedTriangle>, int> = 0>
void forEachSubdividedTriangle(std::vector<TexturedTriangle> &buffer, Action action) noexcept
{
    VXIO_ASSERT_EQ(buffer.size(), 1u);

    do {
        TexturedTriangle &triangle = buffer.back();

        const Vec3u32 vmin = triangle.voxelMin();
        const Vec3u32 vmax = triangle.voxelMax();
        VXIO_DEBUG_ASSERT(vmin != vmax);
        const Vec3u32 size = vmax - vmin;
        const u32 volume = size[0] * size[1] * size[2];

        if (volume < SUBDIVISION_VOLUME_LIMIT) {
            buffer.pop_back();
            action(triangle);
            continue;
        }

        // The center piece of subdivision replaces the current triangle.
        // The other three corner triangles are appended.
        // We don't increment i here so that we can subdivide the center piece again if necessary.
        TexturedTriangle subTriangles[4];
        triangle.subdivide4(subTriangles);
        triangle = std::move(subTriangles[0]);
        for (usize j = 1; j < 4; ++j) {
            buffer.push_back(std::move(subTriangles[j]));
        }
    } while (not buffer.empty());
}

// VOXELIZATION ========================================================================================================

[[nodiscard]] WeightedUv computeTrianglesUvInVoxel(const VisualTriangle &inputTriangle,
                                                   Vec3u32 pos,
                                                   split_buffer_type *preSplitBuffer,
                                                   split_buffer_type *postSplitBuffer) noexcept
{
    for (unsigned hi = 0; hi < 2; ++hi) {
        const auto splittingFunction =
            hi ? splitTriangle<DiscardMode::DISCARD_HI> : splitTriangle<DiscardMode::DISCARD_LO>;

        for (unsigned axis = 0; axis < 3; ++axis) {
            const u32 plane = pos[axis] + hi;

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
    for (const TexturedTriangle &t : *postSplitBuffer) {
        const float weight = static_cast<float>(inputTriangle.area());
        const Vec2f uv = t.textureCenter();

        result = mix(result, {weight, uv});
    }

    postSplitBuffer->clear();
    return result;
}

void voxelizeSubTriangle(const VisualTriangle &inputTriangle,
                         TexturedTriangle subTriangle,
                         Vec3u32 min,
                         Vec3u32 max,
                         split_buffer_type *preSplitBuffer,
                         split_buffer_type *postSplitBuffer,
                         VoxelMap<WeightedUv> &out) noexcept
{
    // sqrt(3) = 1.73... with some leeway to account for imprecision
    constexpr real_type distanceLimit = 2;

    const Vec3 planeOrg = subTriangle.vertex(0);
    const Vec3 planeNormal = normalize(subTriangle.normal());

    Vec3u32 triangleMin = subTriangle.voxelMin();
    triangleMin = obj2voxel::max(min, triangleMin);

    Vec3u32 triangleMax = subTriangle.voxelMax();
    triangleMax = obj2voxel::min(max, triangleMax);

    for (u32 z = triangleMin.z(); z < triangleMax.z(); ++z) {
        for (u32 y = triangleMin.y(); y < triangleMax.y(); ++y) {
            for (u32 x = triangleMin.x(); x < triangleMax.x(); ++x) {
                const Vec3u32 pos = {x, y, z};

                if constexpr (ENABLE_PLANE_DISTANCE_TEST) {
                    const Vec3 center = pos + Vec3::filledWith(0.5f);
                    const real_type signedDistance = distance_point_plane(center, planeOrg, planeNormal);

                    if (std::abs(signedDistance) > distanceLimit) {
                        continue;
                    }
                }

                VXIO_DEBUG_ASSERT(preSplitBuffer->empty());
                VXIO_DEBUG_ASSERT(postSplitBuffer->empty());

                preSplitBuffer->push_back(subTriangle);
                WeightedUv uv = computeTrianglesUvInVoxel(inputTriangle, pos, preSplitBuffer, postSplitBuffer);

                if (not eqExactly(uv.weight, 0.f)) {
                    insertWeighted<ColorStrategy::BLEND>(out, pos, uv);
                }
            }
        }
    }
}

}  // namespace

// VOXELIZER IMPLEMENTATION ============================================================================================

Voxelizer::Voxelizer(ColorStrategy colorStrategy) noexcept : combineFunction{combineFunctionOf(colorStrategy)} {}

void Voxelizer::voxelize(const VisualTriangle &triangle, Vec3u32 min, Vec3u32 max) noexcept
{
    VXIO_ASSERT(uvBuffer.empty());

    voxelizeTriangleToUvBuffer(triangle, min, max);
    moveUvBufferIntoVoxels(triangle);
}

void Voxelizer::voxelizeTriangleToUvBuffer(const VisualTriangle &inputTriangle, Vec3u32 min, Vec3u32 max) noexcept
{
    VXIO_DEBUG_ASSERT(subdivisionBuffer.empty());

    preSplitBuffer.clear();
    postSplitBuffer.clear();
    uvBuffer.clear();

    auto action = [this, &inputTriangle, min, max](const TexturedTriangle &subTriangle) {
        if constexpr (build::DEBUG) {
            globalTriangleDebugCallback(subTriangle);
        }
        voxelizeSubTriangle(inputTriangle, subTriangle, min, max, &preSplitBuffer, &postSplitBuffer, uvBuffer);
    };

    if (isRoughlyAlignedWithAnyAxisPlane(inputTriangle)) {
        action(inputTriangle);
    }
    else {
        subdivisionBuffer.push_back(inputTriangle);
        forEachSubdividedTriangle(subdivisionBuffer, action);
        VXIO_DEBUG_ASSERT(subdivisionBuffer.empty());
    }
}

void Voxelizer::moveUvBufferIntoVoxels(const VisualTriangle &inputTriangle) noexcept
{
    for (auto &[index, weightedUv] : uvBuffer) {
        Vec3u32 pos = uvBuffer.posOf(index);
        Vec3f colorVec = inputTriangle.colorAt_f(weightedUv.value);
        WeightedColor color = {weightedUv.weight, colorVec};

        auto [location, success] = voxels_.emplace(pos, color);
        if (not success) {
            location->second = this->combineFunction(color, location->second);
        }
    }
    uvBuffer.clear();
}

void Voxelizer::merge(VoxelMap<WeightedColor> &target, VoxelMap<WeightedColor> &source) noexcept
{
    for (auto &[index, color] : source) {
        auto [location, success] = target.emplace(index, color);
        if (not success) {
            location->second = this->combineFunction(color, location->second);
        }
    }
}

void Voxelizer::downscale() noexcept
{
    constexpr u32 divisor = 2;

    VoxelMap<WeightedColor> result;

    for (auto iter = voxels_.begin(); iter != voxels_.end(); iter = voxels_.erase(iter)) {
        u64 index = iter->first;

        auto [location, success] = voxels_.emplace(index / divisor, iter->second);
        if (not success) {
            location->second = this->combineFunction(iter->second, location->second);
        }
    }

    voxels_ = std::move(result);
}

}  // namespace obj2voxel
