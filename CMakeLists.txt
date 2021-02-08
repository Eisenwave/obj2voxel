cmake_minimum_required(VERSION 3.5)

project(obj2voxel LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

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
    src/main.cpp
    src/3rd_party/tinyobj.hpp
    src/3rd_party/args.hpp
    src/threading.hpp
    src/triangle.hpp
    src/util.hpp
    src/voxelization.cpp
    src/voxelization.hpp
    src/filevoxelization.cpp
    src/filevoxelization.hpp
    src/io.cpp
    src/io.hpp)

add_subdirectory(voxelio)
target_link_libraries(obj2voxel voxelio ${CMAKE_THREAD_LIBS_INIT})

target_include_directories(obj2voxel PUBLIC voxelio/include)
