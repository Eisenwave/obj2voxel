#include "obj2voxel.h"
#include "testutil.hpp"

#include "voxelio/format/vl32.hpp"
#include "voxelio/log.hpp"

#include <vector>

std::vector<NamedTest> tests;

namespace {

// TESTS ===============================================================================================================

TEST(errorOnMissingInput)
{
    pushLogLevel(OBJ2VOXEL_LOG_LEVEL_SILENT);

    CountingOutput output;

    obj2voxel_instance *instance = obj2voxel_alloc();
    obj2voxel_set_output_callback(instance, &outputCallback<CountingOutput>, &output);
    obj2voxel_set_resolution(instance, 1);
    obj2voxel_error_t result = obj2voxel_voxelize(instance);
    obj2voxel_free(instance);

    popLogLevel();

    VXIO_ASSERT_EQ(result, OBJ2VOXEL_ERR_NO_INPUT);
}

TEST(errorOnMissingOutput)
{
    constexpr const float vertices[9]{0, 0, 0, 0, 0, 1, 1, 0, 0};

    pushLogLevel(OBJ2VOXEL_LOG_LEVEL_SILENT);

    TriangleInput input{vertices, 3};

    obj2voxel_instance *instance = obj2voxel_alloc();
    obj2voxel_set_input_callback(instance, &inputCallback<TriangleInput>, &input);
    obj2voxel_set_resolution(instance, 1);
    obj2voxel_error_t result = obj2voxel_voxelize(instance);
    obj2voxel_free(instance);

    popLogLevel();

    VXIO_ASSERT_EQ(result, OBJ2VOXEL_ERR_NO_OUTPUT);
}

TEST(errorOnMissingResolution)
{
    constexpr const float vertices[9]{0, 0, 0, 0, 0, 1, 1, 0, 0};

    pushLogLevel(OBJ2VOXEL_LOG_LEVEL_SILENT);

    TriangleInput input{vertices, 3};
    CountingOutput output;

    obj2voxel_instance *instance = obj2voxel_alloc();
    obj2voxel_set_input_callback(instance, &inputCallback<TriangleInput>, &input);
    obj2voxel_set_output_callback(instance, &outputCallback<CountingOutput>, &output);
    obj2voxel_error_t result = obj2voxel_voxelize(instance);
    obj2voxel_free(instance);

    popLogLevel();

    VXIO_ASSERT_EQ(result, OBJ2VOXEL_ERR_NO_RESOLUTION);
}

// clang-format off
constexpr float unitCubeVertices[8 * 3] {
    0, 0, 0,
    0, 0, 1,
    0, 1, 0,
    0, 1, 1,
    1, 0, 0,
    1, 0, 1,
    1, 1, 0,
    1, 1, 1
};

constexpr size_t unitCubeElements[6 * 4] {
    0, 1, 3, 2,
    4, 6, 7, 5,
    0, 4, 5, 1,
    2, 3, 7, 6,
    0, 2, 6, 4,
    1, 5, 7, 3
};
// clang-format on

constexpr size_t expectedUnitCubeVoxels(size_t resolution)
{
    size_t expectedVertexVoxels = 8;
    size_t expectedEdgeVoxels = 12 * (resolution - 2);
    size_t expectedFaceVoxels = 6 * (resolution - 2) * (resolution - 2);
    return expectedVertexVoxels + expectedEdgeVoxels + expectedFaceVoxels;
}

TEST(unitCubeProducesExpectedVoxelCount)
{
    constexpr size_t resolution = 64;
    constexpr size_t expectedVoxels = expectedUnitCubeVoxels(resolution);

    IndexedQuadInput input{unitCubeVertices, unitCubeElements, sizeof(unitCubeElements) / sizeof(size_t)};

    obj2voxel_instance *instance = obj2voxel_alloc();

#ifdef DUMP_OUTPUTS
    std::optional<voxelio::FileOutputStream> outStream =
        voxelio::openForWrite("/tmp/obj2voxel_test.vl32", voxelio::OpenMode::BINARY);
    VXIO_ASSERT(outStream.has_value());
    voxelio::vl32::Writer writer{*outStream};
    VoxelioOutput output{writer};
    obj2voxel_set_output_callback(instance, &outputCallback<VoxelioOutput>, &output);
#else
    CountingOutput output;
    obj2voxel_set_output_callback(instance, &outputCallback<CountingOutput>, &output);
#endif

    obj2voxel_set_input_callback(instance, &inputCallback<IndexedQuadInput>, &input);
    obj2voxel_set_resolution(instance, resolution);
    obj2voxel_error_t result = obj2voxel_voxelize(instance);
    obj2voxel_free(instance);

    VXIO_ASSERT_EQ(result, OBJ2VOXEL_ERR_OK);
    VXIO_ASSERT_EQ(output.voxelCount, expectedVoxels);
}

TEST(unitCubeProducesExpectedByteCount)
{
    constexpr size_t resolution = 64;
    constexpr size_t expectedVoxels = expectedUnitCubeVoxels(resolution);
    constexpr size_t expectedBytes = expectedVoxels * sizeof(uint32_t) * 4;

    IndexedQuadInput input{unitCubeVertices, unitCubeElements, sizeof(unitCubeElements) / sizeof(size_t)};

    obj2voxel_instance *instance = obj2voxel_alloc();
    obj2voxel_set_input_callback(instance, &inputCallback<IndexedQuadInput>, &input);
    obj2voxel_set_output_memory(instance, "vl32");
    obj2voxel_set_resolution(instance, resolution);
    obj2voxel_error_t result = obj2voxel_voxelize(instance);

    VXIO_ASSERT_EQ(result, OBJ2VOXEL_ERR_OK);

    size_t size;
    VXIO_ASSERT_NOTNULL(obj2voxel_get_output_memory(instance, &size));
    obj2voxel_free(instance);

    VXIO_ASSERT_EQ(size, expectedBytes);
}

// MAIN ================================================================================================================

}  // namespace

int main()
{
    obj2voxel_set_log_level(OBJ2VOXEL_LOG_LEVEL_DEBUG);

    VXIO_LOG(INFO, "Running " + voxelio::stringify(tests.size()) + " tests ...");

    for (NamedTest test : tests) {
        VXIO_LOG(INFO, "Running \"" + std::string{test.name} + "\" ...");
        test.test();
    }

#ifndef LOG_LEVEL_STACK_DISABLED
    VXIO_ASSERT_EQ(logLevelStack.size(), 0);
#endif

    VXIO_LOG(INFO, "All tests passed");
    return 0;
}
