#include "io.hpp"
#include "voxelization.hpp"

#include "3rd_party/args.hpp"

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

struct VoxelizationArgs {
    std::string inFile;
    std::string texture;
    unsigned resolution;
    ColorStrategy colorStrategy;
    Vec3u permutation;
};

VoxelMap<WeightedColor> voxelizeObj(VoxelizationArgs args)
{
    // Load obj model
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    if (not loadObj(args.inFile, attrib, shapes, materials)) {
        std::exit(1);
    }
    if (attrib.vertices.empty()) {
        VXIO_LOG(WARNING, "Model has no vertices, aborting and writing empty voxel model");
        return {};
    }
    VXIO_LOG(INFO, "Loaded OBJ model with " + stringifyLargeInt(attrib.vertices.size() / 3) + " vertices");

    // Determine mesh to voxel space transform

    Vec3 meshMin, meshMax;
    findBoundaries(attrib.vertices, meshMin, meshMax);

    Voxelizer voxelizer{args.colorStrategy};
    voxelizer.initTransform(meshMin, meshMax, args.resolution, args.permutation);

    // Load textures
    Texture *defaultTexture = nullptr;
    if (not args.texture.empty()) {
        std::optional<Texture> loadedDefault = loadTexture(args.texture, "-t");
        if (loadedDefault.has_value()) {
            auto [location, success] = voxelizer.textures.emplace("", std::move(*loadedDefault));
            VXIO_ASSERTM(success, "Multiple default textures?!");
            defaultTexture = &location->second;
        }
    }
    for (tinyobj::material_t &material : materials) {
        std::string name = material.diffuse_texname;
        if (name.empty()) {
            continue;
        }
        std::optional<Texture> tex = loadTexture(name, material.name);
        if (tex.has_value()) {
            voxelizer.textures.emplace(std::move(name), std::move(*tex));
        }
    }
    VXIO_LOG(INFO, "Loaded " + stringifyLargeInt(voxelizer.textures.size()) + " textures");

    // Loop over shapes
    for (usize s = 0; s < shapes.size(); s++) {
        tinyobj::shape_t &shape = shapes[s];

        // Loop over faces(polygon)
        usize index_offset = 0;
        for (usize f = 0; f < shape.mesh.num_face_vertices.size(); f++) {
            usize vertexCount = shape.mesh.num_face_vertices[f];
            VXIO_DEBUG_ASSERT_EQ(vertexCount, 3);

            VisualTriangle triangle;
            bool hasTexCoords = true;

            // Loop over vertices in the face.
            for (usize v = 0; v < vertexCount; v++) {
                // access to vertex
                tinyobj::index_t idx = shape.mesh.indices[index_offset + v];
                if (idx.vertex_index < 0) {
                    VXIO_LOG(ERROR, "Vertex without vertex coordinates found");
                    exit(1);
                }
                Vec3 &vertex = triangle.v[v];
                vertex = Vec3{attrib.vertices.data() + 3 * idx.vertex_index};

                if (idx.texcoord_index >= 0) {
                    triangle.t[v] = Vec2{attrib.texcoords.data() + 2 * idx.texcoord_index};
                }
                else {
                    // Even if this value will never be used, it's not good practice to leave it unitialized.
                    // This could lead to accidental denormalized float operations which are expensive.
                    // Remove this line if we ever create a special case for meshes with no UV coordinates.
                    triangle.t[v] = {};
                    hasTexCoords = false;
                }
            }
            index_offset += vertexCount;

            int materialIndex = shape.mesh.material_ids[f];
            if (materialIndex < 0) {
                if (hasTexCoords && defaultTexture != nullptr) {
                    triangle.type = TriangleType::TEXTURED;
                    triangle.texture = defaultTexture;
                }
                else {
                    triangle.type = TriangleType::MATERIALLESS;
                }
            }
            else if (hasTexCoords) {
                const tinyobj::material_t &material = materials[static_cast<usize>(materialIndex)];
                const std::string &textureName = material.diffuse_texname;
                if (textureName.empty()) {
                    goto untextured;
                }
                auto location = voxelizer.textures.find(textureName);
                if (location == voxelizer.textures.end()) {
                    VXIO_LOG(ERROR,
                             "Face with material \"" + material.name + "\" has unloaded texture name \"" + textureName +
                                 '"');
                    std::exit(1);
                }
                triangle.texture = &location->second;
                triangle.type = TriangleType::TEXTURED;
            }
            else {
            untextured:
                Vec3 color{materials[static_cast<usize>(materialIndex)].diffuse};
                triangle.color = color.cast<float>();
                triangle.type = TriangleType::UNTEXTURED;
            }

            voxelizer.voxelize(triangle);
        }
    }
    VXIO_LOG(INFO, "Voxelized " + stringifyLargeInt(voxelizer.triangleCount) + " triangles");

    return std::move(voxelizer.voxels);
}

VoxelMap<WeightedColor> voxelizeStl(VoxelizationArgs args)
{
    std::vector<f32> stl = loadStl(args.inFile);

    VXIO_LOG(INFO, "Loaded STL model with " + stringifyLargeInt(stl.size() / 3) + " vertices");

    Voxelizer voxelizer{args.colorStrategy};

    {
        Vec3 meshMin, meshMax;
        findBoundaries(stl, meshMin, meshMax);
        voxelizer.initTransform(meshMin, meshMax, args.resolution, args.permutation);
    }

    f32 *data = stl.data();

    VXIO_ASSERT(stl.size() % 9 == 0);
    for (usize i = 0; i < stl.size(); i += 9) {
        VisualTriangle triangle;

        for (usize i = 0; i < 3; ++i) {
            triangle.v[i] = Vec3f{data}.cast<real_type>();
            data += 3;
            triangle.t[i] = {};
        }

        triangle.type = TriangleType::MATERIALLESS;

        voxelizer.voxelize(std::move(triangle));
    }

    VXIO_LOG(INFO, "Voxelized " + stringifyLargeInt(voxelizer.triangleCount) + " triangles");

    return std::move(voxelizer.voxels);
}

int mainImpl(std::string inFile,
             std::string outFile,
             std::string textureFile,
             unsigned resolution,
             bool supersample,
             ColorStrategy colorStrategy,
             Vec3u permutation)
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
    args.colorStrategy = colorStrategy;
    args.permutation = permutation;

    VoxelMap<WeightedColor> voxels = *inType == FileType::WAVEFRONT_OBJ ? voxelizeObj(args) : voxelizeStl(args);

#ifdef OBJ2VOXEL_DUMP_STL
    dumpDebugStl("/tmp/obj2voxel_debug.stl");
#endif

    if (supersample) {
        voxels = downscale(voxels, colorStrategy);
    }

    VXIO_LOG(INFO, "Model was voxelized, writing voxels to disk ...");
    bool success = writeMapWithVoxelio(voxels, resolution, *outType, *outStream);

    return not success;
}

bool parsePermutation(std::string str, Vec3u &out)
{
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

}  // namespace
}  // namespace obj2voxel

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
{
    using namespace obj2voxel;

    if constexpr (voxelio::build::DEBUG) {
        voxelio::logLevel = voxelio::LogLevel::DEBUG;
        VXIO_LOG(DEBUG, "Running debug build");
    }

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
        vgroup, "max|blend", STRATEGY_DESCR, {'s'}, strategyMap, ColorStrategy::MAX);
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
             std::move(textureArg.Get()),
             resolutionArg.Get(),
             ssArg.Get(),
             strategyArg.Get(),
             permutation);
}
