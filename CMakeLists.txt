cmake_minimum_required(VERSION 3.5)

project(obj2voxel LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

string(TOUPPER "${CMAKE_BUILD_TYPE}" build_type)

if(build_type STREQUAL DEBUG)
    message("Building obj2voxel DEBUG")
    add_definitions(-DVXIO_DEBUG)
elseif(build_type STREQUAL RELWITHDEBINFO)
    message("Building obj2voxel RELEASE (with debug info)")
    add_definitions(-DVXIO_RELEASE)
else()
    message("Building obj2voxel RELEASE")
    add_definitions(-DVXIO_RELEASE)
endif()

add_executable(obj2voxel
    main.cpp
    3rd_party/tinyobj.hpp
    3rd_party/args.hpp
    triangle.hpp
    util.hpp
    voxelization.cpp
    voxelization.hpp
    io.cpp
    io.hpp)

add_subdirectory(voxelio)
target_link_libraries(obj2voxel voxelio)

target_include_directories(obj2voxel PUBLIC voxelio/include)
