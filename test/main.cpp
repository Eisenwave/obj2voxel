#include "obj2voxel.h"
#include "testutil.hpp"

#include "voxelio/log.hpp"

#include <vector>

// TESTS ===============================================================================================================

#define TEST_HEADER

using TestFunction = void (*)(void);

void test_errorOnMissingInput()
{
    pushLogLevel(OBJ2VOXEL_LOG_LEVEL_SILENT);

    size_t voxelCount = 0;

    obj2voxel_instance *instance = obj2voxel_alloc();
    obj2voxel_set_output_callback(instance, &callback::countVoxels, &voxelCount);
    obj2voxel_set_resolution(instance, 1);
    obj2voxel_error_t result = obj2voxel_voxelize(instance);
    obj2voxel_free(instance);

    popLogLevel();

    VXIO_ASSERT_EQ(result, OBJ2VOXEL_ERR_NO_INPUT);
}

void test_errorOnMissingOutput()
{
    constexpr const float vertices[9]{0, 0, 0, 0, 0, 1, 1, 0, 0};

    pushLogLevel(OBJ2VOXEL_LOG_LEVEL_SILENT);

    TriangleIterator iterator{vertices, 1};

    obj2voxel_instance *instance = obj2voxel_alloc();
    obj2voxel_set_input_callback(instance, &callback::callNext<TriangleIterator>, &iterator);
    obj2voxel_set_resolution(instance, 1);
    obj2voxel_error_t result = obj2voxel_voxelize(instance);
    obj2voxel_free(instance);

    popLogLevel();

    VXIO_ASSERT_EQ(result, OBJ2VOXEL_ERR_NO_OUTPUT);
}

void test_errorOnMissingResolution()
{
    constexpr const float vertices[9]{0, 0, 0, 0, 0, 1, 1, 0, 0};

    pushLogLevel(OBJ2VOXEL_LOG_LEVEL_SILENT);

    TriangleIterator iterator{vertices, 1};
    size_t voxelCount = 0;

    obj2voxel_instance *instance = obj2voxel_alloc();
    obj2voxel_set_input_callback(instance, &callback::callNext<TriangleIterator>, &iterator);
    obj2voxel_set_output_callback(instance, &callback::countVoxels, &voxelCount);
    obj2voxel_error_t result = obj2voxel_voxelize(instance);
    obj2voxel_free(instance);

    popLogLevel();

    VXIO_ASSERT_EQ(result, OBJ2VOXEL_ERR_NO_RESOLUTION);
}

// MAIN ================================================================================================================

struct NamedTest {
    const char *name;
    TestFunction test;
};

int main()
{
#define NAMED_TEST(name) NamedTest{#name, &name}

    obj2voxel_set_log_level(OBJ2VOXEL_LOG_LEVEL_DEBUG);

    std::vector<NamedTest> tests{NAMED_TEST(test_errorOnMissingInput),
                                 NAMED_TEST(test_errorOnMissingOutput),
                                 NAMED_TEST(test_errorOnMissingResolution)};

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
