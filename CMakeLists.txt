cmake_minimum_required(VERSION 3.5)

project(obj2voxel LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

string(TOUPPER "${CMAKE_BUILD_TYPE}" build_type)

if(build_type STREQUAL DEBUG)
    message("Building obj2voxel DEBUG")
    add_compile_definitions(VXIO_DEBUG)
elseif(build_type STREQUAL RELWITHDEBINFO)
    message("Building obj2voxel RELEASE (with debug info)")
    add_compile_definitions(VXIO_RELEASE)
else()
    message("Building obj2voxel RELEASE")
    add_compile_definitions(VXIO_RELEASE)
endif()

add_subdirectory(voxelio)

#####################
# OBJ2VOXEL LIBRARY #
#####################

add_library(obj2voxel STATIC
    src/3rd_party/tinyobj.hpp
    src/3rd_party/args.hpp
    src/arrayvector.hpp
    src/constants.hpp
    src/ringbuffer.hpp
    src/threading.hpp
    src/triangle.hpp
    src/util.hpp
    src/voxelization.cpp
    src/voxelization.hpp
    src/io.cpp
    src/io.hpp
    src/obj2voxel.cpp
    include/obj2voxel.h)
    
target_link_libraries(obj2voxel PRIVATE voxelio PRIVATE ${CMAKE_THREAD_LIBS_INIT})

target_include_directories(obj2voxel PRIVATE voxelio/include)
target_include_directories(obj2voxel PUBLIC include)

#################
# OBJ2VOXEL CLI #
#################

add_executable(obj2voxel-cli src/main.cpp)
set_target_properties(obj2voxel-cli PROPERTIES OUTPUT_NAME "obj2voxel")

target_link_libraries(obj2voxel-cli PRIVATE obj2voxel PRIVATE voxelio ${CMAKE_THREAD_LIBS_INIT})

target_include_directories(obj2voxel-cli PRIVATE voxelio/include)
target_include_directories(obj2voxel-cli PRIVATE include)

###################
# OBJ2VOXEL TESTS #
###################

add_subdirectory(test)






