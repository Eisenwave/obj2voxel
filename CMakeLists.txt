cmake_minimum_required(VERSION 3.5)

project(obj2voxels LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

string(TOUPPER "${CMAKE_BUILD_TYPE}" build_type)

if(build_type STREQUAL DEBUG)
    message("Building FLVC DEBUG")
    add_definitions(-DVXIO_DEBUG)
elseif(build_type STREQUAL RELWITHDEBINFO)
    message("Building FLVC RELEASE (with debug info)")
    add_definitions(-DVXIO_RELEASE)
else()
    message("Building FLVC RELEASE")
    add_definitions(-DVXIO_RELEASE)
endif()

add_subdirectory(voxelio)
add_executable(obj2voxels main.cpp svo.hpp 3rd_party/tinyobj.hpp voxelization.cpp voxelization.hpp)
target_link_libraries(obj2voxels voxelio)

target_include_directories(obj2voxels PUBLIC voxelio/include)
