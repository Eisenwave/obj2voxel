#include "obj2voxel.h"
#include "testutil.hpp"

#include "voxelio/format/vl32.hpp"
#include "voxelio/log.hpp"

#include <vector>

std::vector<NamedTest> tests;

namespace {

// clang-format off
constexpr std::array<float, 9> triangleVertices{
    0, 0, 0,
    0, 0, 1,
    1, 0, 0
};

constexpr std::array<float, 8 * 3> unitCubeVertices{
    0, 0, 0,
    0, 0, 1,
    0, 1, 0,
    0, 1, 1,
    1, 0, 0,
    1, 0, 1,
    1, 1, 0,
    1, 1, 1
};

constexpr std::array<size_t, 6 * 4> unitCubeElements{
    0, 1, 3, 2,
    4, 6, 7, 5,
    0, 4, 5, 1,
    2, 3, 7, 6,
    0, 2, 6, 4,
    1, 5, 7, 3
};

constexpr std::array<float, 3 * 4 * 3> threePlanesVertices{
    .0, 0, 0,
    .0, 0, 1,
    .0, 1, 1,
    .0, 1, 0,

    .5, 0, 0,
    .5, 0, 1,
    .5, 1, 1,
    .5, 1, 0,

    1., 0, 0,
    1., 0, 1,
    1., 1, 1,
    1., 1, 0,
};

constexpr std::array<size_t, 3 * 4> threePlanesElements{
    0, 1, 2, 3,
    4, 5, 6, 7,
    8, 9, 10, 11
};
// clang-format on

// TESTS ===============================================================================================================

//#define DUMP_OUTPUTS

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
    pushLogLevel(OBJ2VOXEL_LOG_LEVEL_SILENT);

    TriangleInput input{triangleVertices.data(), 3};

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
    pushLogLevel(OBJ2VOXEL_LOG_LEVEL_SILENT);

    TriangleInput input{triangleVertices.data(), 3};
    CountingOutput output;

    obj2voxel_instance *instance = obj2voxel_alloc();
    obj2voxel_set_input_callback(instance, &inputCallback<TriangleInput>, &input);
    obj2voxel_set_output_callback(instance, &outputCallback<CountingOutput>, &output);
    obj2voxel_error_t result = obj2voxel_voxelize(instance);
    obj2voxel_free(instance);

    popLogLevel();

    VXIO_ASSERT_EQ(result, OBJ2VOXEL_ERR_NO_RESOLUTION);
}

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

    IndexedQuadInput input{unitCubeVertices.data(), unitCubeElements.data(), unitCubeElements.size()};

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

    IndexedQuadInput input{unitCubeVertices.data(), unitCubeElements.data(), unitCubeElements.size()};

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

void testVoxelProduction(obj2voxel_instance *instance, size_t expectedVoxels)
{
    CountingOutput output;
    obj2voxel_set_output_callback(instance, &outputCallback<CountingOutput>, &output);

    obj2voxel_error_t result = obj2voxel_voxelize(instance);
    VXIO_ASSERT_EQ(result, OBJ2VOXEL_ERR_OK);

    obj2voxel_free(instance);

    VXIO_ASSERT_EQ(output.voxelCount, expectedVoxels);
}

TEST(unitCubeProducesExpectedVoxelCountForMultipleChunks)
{
    IndexedQuadInput input{unitCubeVertices.data(), unitCubeElements.data(), unitCubeElements.size()};

    obj2voxel_instance *instance = obj2voxel_alloc();
    obj2voxel_set_input_callback(instance, &inputCallback<IndexedQuadInput>, &input);

    const uint32_t resolution = obj2voxel_get_chunk_size(instance) * 2;
    const size_t expectedVoxels = expectedUnitCubeVoxels(resolution);

    obj2voxel_set_resolution(instance, resolution);
    VXIO_ASSERT_EQ(resolution, obj2voxel_get_resolution(instance));

    testVoxelProduction(instance, expectedVoxels);
}

#ifdef DUMP_OUTPUTS
TEST(dumpThreePlanes)
{
    IndexedQuadInput input{threePlanesVertices.data(), threePlanesElements.data(), threePlanesElements.size()};

    obj2voxel_instance *instance = obj2voxel_alloc();
    obj2voxel_set_input_callback(instance, &inputCallback<IndexedQuadInput>, &input);
    obj2voxel_set_output_file(instance, "/tmp/three_planes.vl32", nullptr);
    obj2voxel_set_resolution(instance, obj2voxel_get_chunk_size(instance) * 2);
    obj2voxel_error_t result = obj2voxel_voxelize(instance);
    VXIO_ASSERT_EQ(result, OBJ2VOXEL_ERR_OK);
    obj2voxel_free(instance);
}
#endif

TEST(threePlanesProduceExpectedVoxelCount)
{
    constexpr size_t resolution = 32;
    constexpr size_t expectedVoxels = resolution * resolution * 3;

    IndexedQuadInput input{threePlanesVertices.data(), threePlanesElements.data(), threePlanesElements.size()};

    obj2voxel_instance *instance = obj2voxel_alloc();
    obj2voxel_set_input_callback(instance, &inputCallback<IndexedQuadInput>, &input);
    obj2voxel_set_resolution(instance, resolution);

    testVoxelProduction(instance, expectedVoxels);
}

TEST(threePlanesProduceExpectedVoxelCountForMultipleChunks)
{
    IndexedQuadInput input{threePlanesVertices.data(), threePlanesElements.data(), threePlanesElements.size()};

    obj2voxel_instance *instance = obj2voxel_alloc();
    obj2voxel_set_input_callback(instance, &inputCallback<IndexedQuadInput>, &input);

    const uint32_t resolution = obj2voxel_get_chunk_size(instance) * 2;
    const size_t expectedVoxels = resolution * resolution * 3;

    obj2voxel_set_resolution(instance, resolution);

    testVoxelProduction(instance, expectedVoxels);
}

// MAIN ================================================================================================================

}  // namespace

int main()
{
    voxelio::setLogLevel(voxelio::LogLevel::DEBUG);
    voxelio::enableLoggingSourceLocation(voxelio::build::DEBUG);

    VXIO_LOG(INFO, "Running " + voxelio::stringify(tests.size()) + " tests ...");

    for (NamedTest test : tests) {
        VXIO_LOG(IMPORTANT, "Running \"" + std::string{test.name} + "\" ...");
        test.test();
    }

#ifndef LOG_LEVEL_STACK_DISABLED
    VXIO_ASSERT_EQ(logLevelStack.size(), 0u);
#endif

    VXIO_LOG(IMPORTANT, "All tests passed");
    return 0;
}
