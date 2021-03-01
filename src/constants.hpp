#ifndef OBJ2VOXEL_CONSTANTS_HPP
#define OBJ2VOXEL_CONSTANTS_HPP

#include "obj2voxel.h"

#include <cstdint>

namespace obj2voxel {

constexpr uint32_t CHUNK_SIZE = 64;
constexpr uint32_t BATCH_SIZE = 1024;

constexpr bool DEFAULT_SUPERSAMPLE = false;
constexpr obj2voxel_enum_t DEFAULT_COLOR_STRATEGY = OBJ2VOXEL_MAX_STRATEGY;

constexpr obj2voxel_enum_t DEBUG_LOG_LEVEL = OBJ2VOXEL_LOG_LEVEL_DEBUG;
constexpr obj2voxel_enum_t RELEASE_LOG_LEVEL = OBJ2VOXEL_LOG_LEVEL_INFO;

constexpr bool ENABLE_ASYNC_LOGGING = true;
constexpr bool ENABLE_PLANE_DISTANCE_TEST = true;

constexpr const char *CLI_HEADER = "obj2voxel - OBJ and STL voxelizer";
constexpr const char *CLI_FOOTER = "Visit at https://github.com/eisenwave/obj2voxel";
constexpr const char *HELP_DESCR = "Display this help menu.";
constexpr const char *INPUT_DESCR = "First argument. Path to OBJ or STL input file.";
constexpr const char *OUTPUT_DESCR = "Second argument. Path to QEF, PLY or VL32 output file.";
constexpr const char *TEXTURE_DESCR = "Fallback texture path. Used when model has UV coordinates but textures can't "
                                      "be found in the material library. (Default: none)";
constexpr const char *RESOLUTION_DESCR = "Maximum voxel grid resolution on any axis. (Required)";
constexpr const char *STRATEGY_DESCR =
    "Strategy for combining voxels of different triangles. "
    "Blend gives smoother colors at triangle edges but might produce new and unwanted colors. (Default: max)";
constexpr const char *PERMUTATION_ARG = "Permutation of xyz axes in the model. (Default: xyz)";
constexpr const char *SS_DESCR =
    "Enables supersampling. "
    "The model is voxelized at double resolution and then downscaled while combining colors.";
constexpr const char *THREADS_DESCR = "Number of worker threads to be started for voxelization. "
                                      "Set to zero for single-threaded voxelization. "
                                      "(Default: CPU threads)";

}  // namespace obj2voxel

#endif
