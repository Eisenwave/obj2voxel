#ifndef OBJ2VOXEL_TESTUTIL_HPP
#define OBJ2VOXEL_TESTUTIL_HPP

#include "obj2voxel.h"

#include <vector>

// CONFIG ==============================================================================================================

//#define LOG_LEVEL_STACK_DISABLED

// UTILITY =============================================================================================================

#ifdef LOG_LEVEL_STACK_DISABLED
void pushLogLevel(obj2voxel_enum_t) {}

void popLogLevel() {}
#else

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

struct TriangleIterator {
    const float *vertices;
    size_t triangleCount;

    size_t index = 0;

    TriangleIterator(const float *vertices, size_t triangleCount) : vertices{vertices}, triangleCount{triangleCount} {}

    bool next(obj2voxel_triangle *triangle)
    {
        constexpr const size_t floatsPerTriangle = 9;

        if (index++ >= triangleCount) {
            return false;
        }
        obj2voxel_set_triangle_basic(triangle, vertices + index * floatsPerTriangle);
        return true;
    }
};

namespace callback {

template <typename T>
bool callNext(void *iter, obj2voxel_triangle *triangle)
{
    return reinterpret_cast<T *>(iter)->next(triangle);
}

bool countVoxels(void *counter, uint32_t *, size_t voxelCount)
{
    size_t &typedCounter = *reinterpret_cast<size_t *>(counter);
    typedCounter += voxelCount;
    return true;
}

}  // namespace callback

#endif
