#ifndef OBJ2VOXEL_CONSTANTS_HPP
#define OBJ2VOXEL_CONSTANTS_HPP

#include "obj2voxel.h"

#include <cstdint>

namespace obj2voxel {

constexpr uint32_t CHUNK_SIZE = 64;
constexpr uint32_t BATCH_SIZE = 1024;

constexpr size_t SUBDIVISION_VOLUME_LIMIT = 512;
// This corresponds to an angle of 60° or higher from the diagonal vector
constexpr float COS_SUBDIVISION_DIAGONALITY_LIMIT = 0.5f;

constexpr bool DEFAULT_SUPERSAMPLE = false;
constexpr obj2voxel_enum_t DEFAULT_COLOR_STRATEGY = OBJ2VOXEL_MAX_STRATEGY;

constexpr obj2voxel_enum_t DEBUG_LOG_LEVEL = OBJ2VOXEL_LOG_LEVEL_DEBUG;
constexpr obj2voxel_enum_t RELEASE_LOG_LEVEL = OBJ2VOXEL_LOG_LEVEL_INFO;

constexpr bool ENABLE_ASYNC_LOGGING = true;
constexpr bool ENABLE_PLANE_DISTANCE_TEST = true;

constexpr const char *VERSION_HEADER = "===== obj2voxel =====";
constexpr const char *VERSION_STR = "1.3.5-dev";

constexpr const char *CLI_FOOTER = "Visit at https://github.com/eisenwave/obj2voxel";

constexpr const char *HELP_DESCR = "Display this help menu.";
constexpr const char *VERBOSE_DESCR = "Enables verbose logging.";
constexpr const char *VERSION_DESCR = "Displays the version and other information.";
constexpr const char *EIGHTY_DESCR = "Print help menu in 80 column mode.";

constexpr const char *INPUT_DESCR = "First argument. Path to input file.";
constexpr const char *OUTPUT_DESCR = "Second argument. Path to output file.";

constexpr const char *INPUT_FORMAT_DESCR = "Explicit input format. (Optional)";
constexpr const char *OUTPUT_FORMAT_DESCR = "Explicit output format. (Optional)";

constexpr const char *TEXTURE_DESCR = "Fallback texture path. Used when model has UV coordinates but textures can't "
                                      "be found in the material library. (Default: none)";

constexpr const char *RESOLUTION_DESCR = "Maximum voxel grid resolution on any axis. (Required)";

constexpr const char *STRATEGY_DESCR =
    "Strategy for combining voxels of different triangles. "
    "Blend gives smoother colors at triangle edges but might produce new and unwanted colors. (Default: max)";

constexpr const char *PERMUTATION_ARG = "Permutation of xyz axes in the model. "
                                        "Capital letters flip an axis. (e.g. xYz to flip y-axis) "
                                        "(Default: xyz)";

constexpr const char *SS_DESCR =
    "Enables supersampling. "
    "The model is voxelized at double resolution and then downscaled while combining colors.";

constexpr const char *THREADS_DESCR = "Number of worker threads to be started for voxelization. "
                                      "Set to zero for single-threaded voxelization. "
                                      "(Default: CPU threads)";

}  // namespace obj2voxel

#endif
