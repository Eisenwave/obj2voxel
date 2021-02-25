set(INCLUDE_DIR "${obj2voxel_SOURCE_DIR}/include")
set(VXIO_INCLUDE_DIR "${obj2voxel_SOURCE_DIR}/voxelio/include")

add_executable(obj2voxel-test main.cpp testutil.hpp)

target_include_directories(obj2voxel-test PUBLIC "${INCLUDE_DIR}")
target_include_directories(obj2voxel-test PUBLIC "${VXIO_INCLUDE_DIR}")

target_link_libraries(obj2voxel-test PRIVATE obj2voxel PRIVATE voxelio)
