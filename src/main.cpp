#include "obj2voxel.h"

#include "3rd_party/args.hpp"
#include "constants.hpp"

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

constexpr Vec3u IDENTITY_PERMUTATION = {0, 1, 2};

constexpr const char *nameOfColorStrategy(obj2voxel_enum_t strategy)
{
    return strategy == OBJ2VOXEL_BLEND_STRATEGY ? "blend" : "max";
}

enum class FilePurpose { INPUT, OUTPUT };

template <FilePurpose PURPOSE>
constexpr bool isSupportedFormat(FileType type);

template <>
constexpr bool isSupportedFormat<FilePurpose::INPUT>(FileType type)
{
    switch (type) {
    case FileType::WAVEFRONT_OBJ:
    case FileType::STEREOLITHOGRAPHY: return true;
    default: return false;
    }
}

template <>
constexpr bool isSupportedFormat<FilePurpose::OUTPUT>(FileType type)
{
    switch (type) {
    case FileType::MAGICA_VOX:
    case FileType::QUBICLE_EXCHANGE:
    case FileType::STANFORD_TRIANGLE:
    case FileType::VL32:
    case FileType::XYZRGB: return true;
    default: return false;
    }
}

template <FilePurpose PURPOSE>
FileType getAndValidateFileType(const std::string &file)
{
    constexpr bool input = PURPOSE == FilePurpose::INPUT;

    std::optional<FileType> type = detectFileType(file);
    if (not type.has_value()) {
        VXIO_LOG(WARNING, "Can't detect file type of \"" + file + '"' + (input ? ", assuming Wavefront OBJ" : ""));
        return FileType::WAVEFRONT_OBJ;
    }

    if (not isSupportedFormat<PURPOSE>(*type)) {
        VXIO_LOG(ERROR, "Detected input file type (" + std::string(nameOf(*type)) + ") is not supported");
        std::exit(1);
    }
    return *type;
}

int mainImpl(std::string inFile,
             std::string outFile,
             unsigned resolution,
             unsigned threads = 0,
             std::string textureFile = "",
             bool supersample = DEFAULT_SUPERSAMPLE,
             obj2voxel_enum_t colorStrategy = DEFAULT_COLOR_STRATEGY,
             Vec3u permutation = IDENTITY_PERMUTATION)
{
    VXIO_LOG(INFO,
             "Converting \"" + inFile + "\" to \"" + outFile + "\" at resolution " + stringifyLargeInt(resolution) +
                 " with strategy " + std::string(nameOfColorStrategy(colorStrategy)));

    if (inFile.empty()) {
        VXIO_LOG(ERROR, "Input file path must not be empty");
        return 1;
    }

    FileType inType = getAndValidateFileType<FilePurpose::INPUT>(inFile);
    FileType outType = getAndValidateFileType<FilePurpose::OUTPUT>(inFile);

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
    obj2voxel_set_input_file(instance, inFile.c_str(), extensionOf(inType));
    obj2voxel_set_output_file(instance, outFile.c_str(), extensionOf(outType));

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
    if constexpr (voxelio::build::DEBUG) {
        obj2voxel_set_log_level(DEBUG_LOG_LEVEL);
        VXIO_LOG(DEBUG, "Running debug build");
    }
    else {
        obj2voxel_set_log_level(RELEASE_LOG_LEVEL);
    }

    setLogBackend(nullptr, ENABLE_ASYNC_LOGGING);
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

    args::ArgumentParser parser(CLI_HEADER, CLI_FOOTER);

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
