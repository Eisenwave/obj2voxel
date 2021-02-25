#include "obj2voxel.h"

#include "3rd_party/args.hpp"

#include "voxelio/filetype.hpp"
#include "voxelio/log.hpp"
#include "voxelio/parse.hpp"
#include "voxelio/stream.hpp"
#include "voxelio/stringmanip.hpp"
#include "voxelio/vec.hpp"

#include <iostream>
#include <memory>
#include <set>
#include <thread>
#include <vector>

// MACROS ==============================================================================================================

// comment out when building
//#define OBJ2VOXEL_DUMP_STL
//#define OBJ2VOXEL_MANUAL_TEST

// this macro helps us run the executable with hardcoded parameters without IDE help
#ifdef OBJ2VOXEL_TEST
#define OBJ2VOXEL_TEST_STRING(arg, def) def
#else
#define OBJ2VOXEL_TEST_STRING(arg, def) arg
#endif

#ifdef OBJ2VOXEL_DUMP_STL
#define OBJ2VOXEL_IF_DUMP_STL(code) code
#else
#define OBJ2VOXEL_IF_DUMP_STL(code)
#endif

// IMPLEMENTATION ======================================================================================================

namespace obj2voxel {
namespace {

using namespace voxelio;

constexpr bool DEFAULT_SUPERSAMPLE = false;
constexpr obj2voxel_enum_t DEFAULT_COLOR_STRATEGY = OBJ2VOXEL_MAX_STRATEGY;
constexpr Vec3u DEFAULT_PERMUTATION = {0, 1, 2};

constexpr const char *nameOfColorStrategy(obj2voxel_enum_t strategy)
{
    return strategy == OBJ2VOXEL_BLEND_STRATEGY ? "blend" : "max";
}

int mainImpl(std::string inFile,
             std::string outFile,
             unsigned resolution,
             unsigned threads = 0,
             std::string textureFile = "",
             bool supersample = DEFAULT_SUPERSAMPLE,
             obj2voxel_enum_t colorStrategy = DEFAULT_COLOR_STRATEGY,
             Vec3u permutation = DEFAULT_PERMUTATION)
{
    VXIO_LOG(INFO,
             "Converting \"" + inFile + "\" to \"" + outFile + "\" at resolution " + stringifyLargeInt(resolution) +
                 " with strategy " + std::string(nameOfColorStrategy(colorStrategy)));

    if (inFile.empty()) {
        VXIO_LOG(ERROR, "Input file path must not be empty");
        return 1;
    }

    std::optional<FileType> inType = detectFileType(inFile);
    if (not inType.has_value()) {
        VXIO_LOG(WARNING, "Can't detect file type of \"" + inFile + "\", assuming Wavefront OBJ");
        inType = FileType::WAVEFRONT_OBJ;
    }

    if (*inType != FileType::WAVEFRONT_OBJ && *inType != FileType::STEREOLITHOGRAPHY) {
        VXIO_LOG(ERROR, "Detected input file type (" + std::string(nameOf(*inType)) + ") is not supported");
        return 1;
    }

    std::optional<FileType> outType = detectFileType(outFile);
    if (not outType.has_value()) {
        VXIO_LOG(ERROR, "Can't detect file type of \"" + outFile + "\"");
        return 1;
    }

    static const std::set<FileType> supportedOut{
        FileType::QUBICLE_EXCHANGE, FileType::VL32, FileType::STANFORD_TRIANGLE, FileType::XYZRGB};
    if (supportedOut.find(*outType) == supportedOut.end()) {
        VXIO_LOG(ERROR, "Detected output file type (" + std::string(nameOf(*outType)) + ") is not supported");
        return 1;
    }

    std::optional<FileOutputStream> outStream = FileOutputStream::open(outFile);
    if (not outStream.has_value()) {
        VXIO_LOG(ERROR, "Failed to open \"" + outFile + "\" for write");
        return 1;
    }

    if (resolution >= 1024 * 1024) {
        VXIO_LOG(WARNING, "Very high resolution (" + stringifyLargeInt(resolution) + "), intentional?")
    }
    if (threads == 1) {
        VXIO_LOG(WARNING, "Running with one worker thread is usually pointless; better use -j 0");
    }

    OBJ2VOXEL_IF_DUMP_STL(globalTriangleDebugCallback = writeTriangleAsBinaryToDebugStl);

    obj2voxel_instance *instance = obj2voxel_alloc();

    std::vector<std::thread> workers;
    workers.reserve(threads);

    VXIO_LOG(DEBUG, "Starting up worker threads ...");

    for (usize i = 0; i < threads; ++i) {
        auto &worker = workers.emplace_back(&obj2voxel_run_worker, instance);
        VXIO_ASSERT(worker.joinable());
    }

    obj2voxel_set_parallel(instance, threads != 0);
    obj2voxel_set_input_file(instance, inFile.c_str(), nullptr);
    obj2voxel_set_output_file(instance, outFile.c_str(), nullptr);

    obj2voxel_texture *texture = nullptr;
    if (not textureFile.empty()) {
        texture = obj2voxel_texture_alloc();
        bool loadSuccess = obj2voxel_texture_load_from_file(texture, textureFile.c_str(), nullptr);
        if (loadSuccess) {
            obj2voxel_set_texture(instance, texture);
            VXIO_LOG(INFO, "Loaded fallback texture \"" + textureFile + '"');
        }
        else {
            VXIO_LOG(WARNING, "Continuing without fallback texture because it could not be loaded");
        }
    }

    int unitTransform[9]{};
    for (usize i = 0; i < 3; ++i) {
        unitTransform[i * 3 + permutation[i]] = 1;
    }
    obj2voxel_set_unit_transform(instance, unitTransform);

    obj2voxel_set_resolution(instance, resolution);
    obj2voxel_set_supersampling(instance, 1 + supersample);
    obj2voxel_set_color_strategy(instance, static_cast<obj2voxel_enum_t>(colorStrategy));

    obj2voxel_error_t resultCode = obj2voxel_voxelize(instance);

    OBJ2VOXEL_IF_DUMP_STL(dumpDebugStl("/tmp/obj2voxel_debug.stl"));

    obj2voxel_stop_workers(instance);
    for (auto &worker : workers) {
        worker.join();
    }

    if (texture != nullptr) {
        obj2voxel_texture_free(texture);
    }

    obj2voxel_free(instance);
    return resultCode;
}

void initLogging()
{
    constexpr bool asyncLogging = true;

    if constexpr (voxelio::build::DEBUG) {
        setLogLevel(LogLevel::DEBUG);
        VXIO_LOG(DEBUG, "Running debug build");
    }

    setLogBackend(nullptr, asyncLogging);
}

// CLI ARGUMENT PARSING ================================================================================================

}  // namespace
}  // namespace obj2voxel

#ifndef OBJ2VOXEL_MANUAL_TEST
static bool parsePermutation(std::string str, voxelio::Vec3u &out)
{
    using namespace voxelio;

    if (str.size() != 3) {
        return false;
    }
    toLowerCase(str);

    bool found[3]{};

    for (usize i = 0; i < 3; ++i) {
        unsigned axis = static_cast<unsigned>(str[i] - 'x');
        if (axis > 2) {
            return false;
        }
        out[i] = axis;
        found[axis] = true;
    }
    return found[0] + found[1] + found[2] == 3;
}

constexpr const char *HEADER = "obj2voxel - OBJ and STL voxelizer";
constexpr const char *FOOTER = "Visit at https://github.com/eisenwave/obj2voxel";
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

int main(int argc, char **argv)
#else
int main()
#endif
{
    using namespace obj2voxel;

    initLogging();

#ifdef OBJ2VOXEL_MANUAL_TEST
    return mainImpl("//home/user/assets/mesh/sword/sword.obj", "/home/user/assets/mesh/sword/sword_128.vl32", 128);
#else
    const std::unordered_map<std::string, obj2voxel_enum_t> strategyMap{{"max", OBJ2VOXEL_MAX_STRATEGY},
                                                                        {"blend", OBJ2VOXEL_BLEND_STRATEGY}};
    const unsigned threadCount = std::thread::hardware_concurrency();

    args::ArgumentParser parser(HEADER, FOOTER);

    auto ggroup = args::Group(parser, "General Options:");
    auto helpArg = args::HelpFlag(ggroup, "help", HELP_DESCR, {'h', "help"});

    auto fgroup = args::Group(parser, "File Options:");
    auto inFileArg = args::Positional<std::string>(fgroup, "INPUT_FILE", INPUT_DESCR);
    auto outFileArg = args::Positional<std::string>(fgroup, "OUTPUT_FILE", OUTPUT_DESCR);
    auto textureArg = args::ValueFlag<std::string>(fgroup, "texture", TEXTURE_DESCR, {'t'}, "");

    auto vgroup = args::Group(parser, "Voxelization Options:");
    auto resolutionArg = args::ValueFlag<unsigned>(vgroup, "resolution", RESOLUTION_DESCR, {'r'});
    auto strategyArg = args::MapFlag<std::string, obj2voxel_enum_t>(
        vgroup, "max|blend", STRATEGY_DESCR, {'s'}, strategyMap, DEFAULT_COLOR_STRATEGY);
    auto permutationArg = args::ValueFlag<std::string>(vgroup, "permutation", PERMUTATION_ARG, {'p'}, "xyz");
    auto ssArg = args::Flag(vgroup, "supersample", SS_DESCR, {'u'});
    auto threadsArg = args::ValueFlag<unsigned>(vgroup, "threads", THREADS_DESCR, {'j'}, threadCount);

    bool complete = parser.ParseCLI(argc, argv);
    complete &= parser.Matched();
    complete &= inFileArg.Matched();
    complete &= outFileArg.Matched();
    complete &= resolutionArg.Matched();

    if (helpArg.Matched() || not complete) {
        parser.Help(std::cout);
        return 1;
    }

    Vec3u permutation;
    if (not parsePermutation(permutationArg.Get(), permutation)) {
        VXIO_LOG(ERROR, "\"" + permutationArg.Get() + "\" is not a valid permutation");
        return 1;
    }
    VXIO_LOG(DEBUG, "Parsed permutation: " + permutationArg.Get() + " -> " + permutation.toString());

    mainImpl(std::move(inFileArg.Get()),
             std::move(outFileArg.Get()),
             resolutionArg.Get(),
             threadsArg.Get(),
             std::move(textureArg.Get()),
             ssArg.Get(),
             strategyArg.Get(),
             permutation);
#endif
}
