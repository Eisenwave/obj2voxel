#include "obj2voxel.h"

#include "3rd_party/args.hpp"
#include "constants.hpp"

#include "voxelio/filetype.hpp"
#include "voxelio/log.hpp"
#include "voxelio/parse.hpp"
#include "voxelio/stream.hpp"
#include "voxelio/stringmanip.hpp"
#include "voxelio/vec.hpp"

#include <chrono>
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
FileType getAndValidateFileType(const std::string &file, const std::string &format)
{
    constexpr bool isInput = PURPOSE == FilePurpose::INPUT;

    std::optional<FileType> type;
    if (format.empty()) {
        type = detectFileType(file);
        if (not type.has_value()) {
            if constexpr (PURPOSE == FilePurpose::INPUT) {
                VXIO_LOG(WARNING, "Can't detect file type of \"" + file + "\", assuming Wavefront OBJ");
                return FileType::WAVEFRONT_OBJ;
            }
            else {
                VXIO_LOG(FAILURE, "Can't detect file type of \"" + file + "\"");
                std::exit(1);
            }
        }
    }
    else {
        type = fileTypeOfExtension(format);
        if (not type.has_value()) {
            VXIO_LOG(FAILURE, '"' + format + "\" is not a valid format");
            std::exit(1);
        }
    }

    if (not isSupportedFormat<PURPOSE>(*type)) {
        VXIO_LOG(FAILURE,
                 "Detected " + std::string(isInput ? "input" : "output") + " file type (" + std::string(nameOf(*type)) +
                     ") is not supported");
        std::exit(1);
    }
    return *type;
}

int mainImpl(std::string inFile,
             std::string outFile,
             std::string inFormat,
             std::string outFormat,
             unsigned resolution,
             unsigned threads,
             std::string textureFile,
             bool supersample,
             obj2voxel_enum_t colorStrategy,
             const int unitTransform[9])
{
    VXIO_LOG(INFO,
             "Converting \"" + inFile + "\" to \"" + outFile + "\" at resolution " + stringifyLargeInt(resolution) +
                 " with strategy " + std::string(nameOfColorStrategy(colorStrategy)));

    if (inFile.empty()) {
        VXIO_LOG(ERROR, "Input file path must not be empty");
        return 1;
    }

    const FileType inType = getAndValidateFileType<FilePurpose::INPUT>(inFile, inFormat);
    const FileType outType = getAndValidateFileType<FilePurpose::OUTPUT>(outFile, outFormat);

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

    if (threads != 0) {
        VXIO_LOG(DEBUG, "Starting up " + stringify(threads) + " worker threads ...");

        for (usize i = 0; i < threads; ++i) {
            auto &worker = workers.emplace_back(&obj2voxel_run_worker, instance);
            VXIO_ASSERT(worker.joinable());
        }
    }
    else {
        VXIO_LOG(DEBUG, "Running single-threaded (no worker threads started)");
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
        voxelio::setLogLevel(voxelio::LogLevel::DEBUG);
        VXIO_LOG(DEBUG, "Running debug build");
    }
    else {
        voxelio::setLogLevel(voxelio::LogLevel::INFO);
        voxelio::enableLoggingTimestamp(false);
        voxelio::enableLoggingSourceLocation(false);
    }

    setLogBackend(nullptr, ENABLE_ASYNC_LOGGING);
}

// CLI ARGUMENT PARSING ================================================================================================

}  // namespace
}  // namespace obj2voxel

[[maybe_unused]] static void parsePermutation(std::string str, int outUnitTransform[9])
{
    using namespace voxelio;

    if (str.size() != 3) {
        VXIO_LOG(FAILURE, "Invalid permutation length (" + stringify(str.size()) + ")");
        std::exit(1);
    }

    bool found[3]{};

    for (usize i = 0; i < 3; ++i) {
        int *const outRow = outUnitTransform + i * 3;

        char c = str[i];
        int twoIfNegative = 0;
        if (std::isupper(c)) {
            c = static_cast<char>(std::tolower(c));
            twoIfNegative = 2;
        }

        usize axis = static_cast<unsigned>(c - 'x');
        if (axis > 2) {
            VXIO_LOG(FAILURE, "Invalid permutation char: '" + std::string{c} + "'")
            std::exit(1);
        }

        found[axis] = true;

        outRow[axis] = 1 - twoIfNegative;
        outRow[++axis % 3] = 0;
        outRow[++axis % 3] = 0;
    }

    if (found[0] + found[1] + found[2] != 3) {
        VXIO_LOG(FAILURE, "Invalid combination of permutation chars \"" + str + "\"");
        std::exit(1);
    }
}

int main([[maybe_unused]] int argc, [[maybe_unused]] char **argv)
{
    using namespace obj2voxel;

    using clock_type = std::chrono::high_resolution_clock;
    const auto startTime = clock_type::now();
    const unsigned threadCount = std::thread::hardware_concurrency();

    initLogging();

#ifdef OBJ2VOXEL_MANUAL_TEST
    constexpr int identityUnitTransform[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};
    return mainImpl("/home/user/assets/mesh/sword/sword.obj",
                    "/home/user/assets/mesh/sword/sword_2048.vl32",
                    "",
                    "",
                    1024,
                    threadCount,
                    "",
                    DEFAULT_SUPERSAMPLE,
                    OBJ2VOXEL_MAX_STRATEGY,
                    identityUnitTransform);
#endif

    const std::unordered_map<std::string, obj2voxel_enum_t> strategyMap{{"max", OBJ2VOXEL_MAX_STRATEGY},
                                                                        {"blend", OBJ2VOXEL_BLEND_STRATEGY}};

    args::ArgumentParser parser("", CLI_FOOTER);

    auto ggroup = args::Group(parser, "General Options:");
    auto helpArg = args::HelpFlag(ggroup, "help", HELP_DESCR, {'h', "help"});
    auto eightyArg = args::Flag(ggroup, "eighty", EIGHTY_DESCR, {"80"});
    auto verboseArg = args::Flag(ggroup, "verbose", VERBOSE_DESCR, {'v', "verbose"});
    auto versionArg = args::Flag(ggroup, "version", VERBOSE_DESCR, {'V', "version"});

    auto fgroup = args::Group(parser, "File Options:");
    auto inFileArg = args::Positional<std::string>(fgroup, "INPUT_FILE", INPUT_DESCR);
    auto outFileArg = args::Positional<std::string>(fgroup, "OUTPUT_FILE", OUTPUT_DESCR);
    auto inFormatArg = args::ValueFlag<std::string>(fgroup, "obj|stl", INPUT_FORMAT_DESCR, {'i'}, "");
    auto outFormatArg = args::ValueFlag<std::string>(fgroup, "ply|qef|vl32|vox|xyzrgb", OUTPUT_FORMAT_DESCR, {'o'}, "");
    auto textureArg = args::ValueFlag<std::string>(fgroup, "texture", TEXTURE_DESCR, {'t'}, "");

    auto vgroup = args::Group(parser, "Voxelization Options:");
    auto resolutionArg = args::ValueFlag<unsigned>(vgroup, "resolution", RESOLUTION_DESCR, {'r', "res"});
    auto strategyArg = args::MapFlag<std::string, obj2voxel_enum_t>(
        vgroup, "max|blend", STRATEGY_DESCR, {'s', "strat"}, strategyMap, DEFAULT_COLOR_STRATEGY);
    auto permutationArg = args::ValueFlag<std::string>(vgroup, "permutation", PERMUTATION_ARG, {'p', "perm"}, "xyz");
    auto ssArg = args::Flag(vgroup, "supersample", SS_DESCR, {'u', "super"});
    auto threadsArg = args::ValueFlag<unsigned>(vgroup, "threads", THREADS_DESCR, {'j', "threads"}, threadCount);

    bool complete = parser.ParseCLI(argc, argv);
    complete &= parser.Matched();
    complete &= inFileArg.Matched();
    complete &= outFileArg.Matched();
    complete &= resolutionArg.Matched();

    if (versionArg.Matched() && not helpArg.Matched()) {
        std::cout << VERSION_HEADER << '\n';
        std::cout << "Version:  " << VERSION_STR << '\n';
        std::cout << "Builtins: ";
#ifdef VXIO_HAS_BUILTIN_IS_CONSTANT_EVALUATED
        std::cout << "ceval;";
#endif
#ifdef VXIO_HAS_BUILTIN_CLZ
        std::cout << "clz;";
#endif
#ifdef VXIO_HAS_BUILTIN_MSB
        std::cout << "msb;";
#endif
#ifdef VXIO_HAS_BUILTIN_BSWAP
        std::cout << "bswap;";
#endif
#ifdef VXIO_HAS_BUILTIN_PDEP
        std::cout << "pdep;";
#endif
        std::cout << '\n';
        return 0;
    }

    if (helpArg.Matched() || not complete) {
        parser.helpParams.width = eightyArg.Matched() ? 80 : 120;
        parser.helpParams.usageString = "Usage: ";
        parser.helpParams.flagindent = 2;
        parser.helpParams.progindent = 0;
        parser.helpParams.optionsString = "";
        parser.helpParams.longSeparator = "";
        parser.helpParams.gutter = 4;
        parser.helpParams.programName = "obj2voxel";
        parser.helpParams.addNewlineBeforeDescription = false;
        parser.Help(std::cout);
        return not complete;
    }

    if (verboseArg.Matched()) {
        voxelio::enableLoggingSourceLocation(true);
        voxelio::enableLoggingTimestamp(true);
        voxelio::setLogLevel(LogLevel::DEBUG);
    }

    int unitTransform[9];
    parsePermutation(permutationArg.Get(), unitTransform);

    mainImpl(std::move(inFileArg.Get()),
             std::move(outFileArg.Get()),
             std::move(inFormatArg.Get()),
             std::move(outFormatArg.Get()),
             resolutionArg.Get(),
             threadsArg.Get(),
             std::move(textureArg.Get()),
             ssArg.Get(),
             strategyArg.Get(),
             unitTransform);

    i64 nanos = std::chrono::duration_cast<std::chrono::nanoseconds>(clock_type::now() - startTime).count();

    VXIO_LOG(IMPORTANT, "Done! (" + stringifyTime(static_cast<u64>(nanos), 2) + ')');
}
