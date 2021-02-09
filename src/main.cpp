#include "filevoxelization.hpp"
#include "io.hpp"

#include "3rd_party/args.hpp"

#include "voxelio/filetype.hpp"
#include "voxelio/parse.hpp"
#include "voxelio/stream.hpp"
#include "voxelio/stringmanip.hpp"

#include <iostream>
#include <memory>
#include <set>
#include <vector>

// MACROS ==============================================================================================================

// comment out when building
//#define OBJ2VOXEL_DUMP_STL
//#define OBJ2VOXEL_MANUAL_TEST

// tinobj implementation must be included last because it is alos included by voxelization.hpp (but no implementation)
#define TINYOBJLOADER_IMPLEMENTATION 1
#include "3rd_party/tinyobj.hpp"

// this macro helps us run the executable with hardcoded parameters without IDE help
#ifdef OBJ2VOXEL_TEST
#define OBJ2VOXEL_TEST_STRING(arg, def) def
#else
#define OBJ2VOXEL_TEST_STRING(arg, def) arg
#endif

// IMPLEMENTATION ======================================================================================================

namespace obj2voxel {
namespace {

using namespace voxelio;

constexpr bool DEFAULT_SUPERSAMPLE = false;
constexpr ColorStrategy DEFAULT_COLOR_STRATEGY = ColorStrategy::MAX;
constexpr Vec3u DEFAULT_PERMUTATION = {0, 1, 2};

int mainImpl(std::string inFile,
             std::string outFile,
             unsigned resolution,
             std::string textureFile = "",
             bool supersample = DEFAULT_SUPERSAMPLE,
             ColorStrategy colorStrategy = DEFAULT_COLOR_STRATEGY,
             Vec3u permutation = DEFAULT_PERMUTATION)
{
    VXIO_LOG(INFO,
             "Converting \"" + inFile + "\" to \"" + outFile + "\" at resolution " + stringifyLargeInt(resolution) +
                 " with strategy " + std::string(nameOf(colorStrategy)));

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

#ifdef OBJ2VOXEL_DUMP_STL
    globalTriangleDebugCallback = writeTriangleAsBinaryToDebugStl;
#endif

    VoxelizationArgs args;
    args.inFile = std::move(inFile);
    args.texture = std::move(textureFile);
    args.resolution = resolution * (1 + supersample);
    args.downscale = supersample;
    args.colorStrategy = colorStrategy;
    args.permutation = permutation;

    std::unique_ptr<ITriangleStream> triangles =
        *inType == FileType::WAVEFRONT_OBJ ? loadObj(args.inFile, args.texture) : loadStl(args.inFile);
    VoxelSink sink{*outStream, *outType, resolution};

    int resultCode = not voxelize(args, *triangles, sink);

#ifdef OBJ2VOXEL_DUMP_STL
    dumpDebugStl("/tmp/obj2voxel_debug.stl");
#endif

    return resultCode;
}

void initLogging()
{
    constexpr bool asyncLogging = true;

    if constexpr (voxelio::build::DEBUG) {
        voxelio::logLevel = LogLevel::DEBUG;
        VXIO_LOG(DEBUG, "Running debug build");
    }

    setLogCallback(nullptr, asyncLogging);
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
    const std::unordered_map<std::string, ColorStrategy> strategyMap{{"max", ColorStrategy::MAX},
                                                                     {"blend", ColorStrategy::BLEND}};

    args::ArgumentParser parser(HEADER, FOOTER);

    args::Group ggroup(parser, "General Options:");
    args::HelpFlag helpArg(ggroup, "help", HELP_DESCR, {'h', "help"});

    args::Group fgroup(parser, "File Options:");
    args::Positional<std::string> inFileArg(fgroup, "INPUT_FILE", INPUT_DESCR);
    args::Positional<std::string> outFileArg(fgroup, "OUTPUT_FILE", OUTPUT_DESCR);
    args::ValueFlag<std::string> textureArg(fgroup, "texture", TEXTURE_DESCR, {'t'}, "");

    args::Group vgroup(parser, "Voxelization Options:");
    args::ValueFlag<unsigned> resolutionArg(vgroup, "resolution", RESOLUTION_DESCR, {'r'});
    args::MapFlag<std::string, ColorStrategy> strategyArg(
        vgroup, "max|blend", STRATEGY_DESCR, {'s'}, strategyMap, DEFAULT_COLOR_STRATEGY);
    args::ValueFlag<std::string> permutationArg(vgroup, "permutation", PERMUTATION_ARG, {'p'}, "xyz");
    args::Flag ssArg(vgroup, "supersample", SS_DESCR, {'u'});

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
             std::move(textureArg.Get()),
             ssArg.Get(),
             strategyArg.Get(),
             permutation);
#endif
}
