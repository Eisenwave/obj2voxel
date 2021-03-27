#ifndef OBJ2VOXEL_TESTUTIL_HPP
#define OBJ2VOXEL_TESTUTIL_HPP

#include "obj2voxel.h"

#include "voxelio/assert.hpp"
#include "voxelio/voxelio.hpp"

#include <cstring>
#include <vector>

// CONFIG ==============================================================================================================

//#define LOG_LEVEL_STACK_DISABLED

// UTILITY =============================================================================================================

#ifdef LOG_LEVEL_STACK_DISABLED
void pushLogLevel(obj2voxel_enum_t) {}

void popLogLevel() {}
#else

// evil use of static in header is ok because header is only included once
static std::vector<obj2voxel_enum_t> logLevelStack;

void pushLogLevel(obj2voxel_enum_t level)
{
    logLevelStack.push_back(obj2voxel_get_log_level());
    obj2voxel_set_log_level(level);
}

void popLogLevel()
{
    obj2voxel_set_log_level(logLevelStack.back());
    logLevelStack.pop_back();
}
#endif

// INPUTS ==============================================================================================================

struct TriangleInput {
    const float *vertices;
    size_t vertexCount;

    size_t vertexIndex = 0;

    TriangleInput(const float *vertices, size_t vertexCount) : vertices{vertices}, vertexCount{vertexCount}
    {
        VXIO_ASSERT_DIVISIBLE(vertexCount, 3u);
    }

    bool next(obj2voxel_triangle *triangle)
    {
        constexpr const size_t floatsPerTriangle = 9;

        if (vertexIndex >= vertexCount) {
            return false;
        }

        obj2voxel_set_triangle_basic(triangle, vertices + vertexIndex * floatsPerTriangle);
        vertexIndex += 3;
        return true;
    }
};

template <size_t PRIM_VERTICES, std::enable_if_t<PRIM_VERTICES == 3 || PRIM_VERTICES == 4, int> = 0>
struct IndexedPrimitiveInput {
    const float *vertices;
    const size_t *elements;
    size_t elementCount;
    size_t elementIndex = 0;

    float buffer[9];

    IndexedPrimitiveInput(const float *vertices, const size_t *elements, size_t elementCount)
        : vertices{vertices}, elements{elements}, elementCount{elementCount}
    {
        VXIO_ASSERT_DIVISIBLE(elementCount, PRIM_VERTICES);
    }

    bool next(obj2voxel_triangle *triangle)
    {
        if (elementIndex >= elementCount) {
            return false;
        }

        // To output the triangles of a quad, we switch between cases A and B between each iteration.
        // We can tell the case by the lowest bit of the element index.
        // For PRIM_VERTICES == 3, there is only case A.
        //
        // 0--1     A. output 0, 1, 2 (else-block)
        // |  |     B. output 2, 3, 0 (true-block)
        // 3--2
        if (PRIM_VERTICES == 4 && (elementIndex & 1)) {
            bufferElementVertex(0, -1);
            bufferElementVertex(3, 0);
            bufferElementVertex(6, -3);
            elementIndex += 1;
        }
        else {
            bufferElementVertex(0, 0);
            bufferElementVertex(3, 1);
            bufferElementVertex(6, 2);
            elementIndex += 3;
        }

        obj2voxel_set_triangle_basic(triangle, buffer);
        return true;
    }

    void bufferElementVertex(unsigned bufferOffset, int elementOffset)
    {
        std::memcpy(buffer + bufferOffset, vertices + elements[elementIndex + elementOffset] * 3, sizeof(float) * 3);
    }
};

using IndexedTriangleInput = IndexedPrimitiveInput<3>;
using IndexedQuadInput = IndexedPrimitiveInput<4>;

// OUTPUTS =============================================================================================================

struct CountingOutput {
    size_t voxelCount = 0;

    bool write(uint32_t *, size_t voxelCount)
    {
        this->voxelCount += voxelCount;
        return true;
    }
};

struct HistogramOutput {
    std::unordered_map<uint32_t, uint32_t> histogram;
    size_t voxelCount = 0;

    bool write(uint32_t *voxels, size_t voxelCount)
    {
        this->voxelCount += voxelCount;
        for (size_t i = 0; i < voxelCount; ++i) {
            uint32_t color = voxels[i * 4 + 3];
            ++histogram[color];
        }
        return true;
    }
};

struct VoxelioOutput {
    voxelio::AbstractListWriter &writer;
    size_t voxelCount = 0;

    bool write(uint32_t *rawBuffer, size_t voxelCount)
    {
        this->voxelCount += voxelCount;
        /// Trust me, it's safe :)
        auto *voxelBuffer = reinterpret_cast<voxelio::Voxel32 *>(rawBuffer);
        voxelio::ResultCode result = writer.write(voxelBuffer, voxelCount);
        return voxelio::isGood(result);
    }
};

template <typename T>
bool inputCallback(void *iter, obj2voxel_triangle *triangle)
{
    return reinterpret_cast<T *>(iter)->next(triangle);
}

template <typename T>
bool outputCallback(void *sink, uint32_t *voxelBuffer, size_t voxelCount)
{
    return reinterpret_cast<T *>(sink)->write(voxelBuffer, voxelCount);
}

// TEST METAPROGRAMMING ================================================================================================

using TestFunction = void (*)(void);

struct NamedTest {
    const char *name;
    TestFunction test;
};

/// Stores all tests for execution.
extern std::vector<NamedTest> tests;

#define TEST(name)                       \
    void name();                         \
    static const int name##__ = []() {   \
        tests.push_back({#name, &name}); \
        return 0;                        \
    }();                                 \
    void name()

#endif
