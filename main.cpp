#include "svo.hpp"
#include "voxelization.hpp"

#include "voxelio/filetype.hpp"
#include "voxelio/format/qef.hpp"
#include "voxelio/format/vl32.hpp"
#include "voxelio/parse.hpp"
#include "voxelio/stream.hpp"
#include "voxelio/stringmanip.hpp"
#include "voxelio/voxelio.hpp"

#include <map>
#include <memory>
#include <set>
#include <vector>

// tinobj implementation must be included last because it is alos included by voxelization.hpp (but no implementation)
#define TINYOBJLOADER_IMPLEMENTATION 1
#include "3rd_party/tinyobj.hpp"

// MACROS ==============================================================================================================

// comment out when building
#define OBJ2VOXEL_TEST

#ifndef OBJ2VOXEL_TEST
#define OBJ2VOXEL_MAIN_PARAMS int argc, char **argv
#else
#define OBJ2VOXEL_MAIN_PARAMS
#endif

// this macro helps us run the executable with hardcoded parameters without IDE help
#ifdef OBJ2VOXEL_TEST
#define OBJ2VOXEL_TEST_STRING(arg, def) def
#else
#define OBJ2VOXEL_TEST_STRING(arg, def) arg
#endif

// IMPLEMENTATION ======================================================================================================

namespace obj2voxel {

using namespace voxelio;
using namespace mve;

using svo_type = SparseVoxelOctree<Color32>;
using svo_node_type = svo_type::node_type;
using svo_branch_type = svo_type::branch_type;
using svo_leaf_type = svo_type::leaf_type;

AbstractListWriter *makeWriter(OutputStream &stream, FileType type)
{
    switch (type) {
    case FileType::QUBICLE_EXCHANGE: return new qef::Writer{stream};
    case FileType::VL32: return new vl32::Writer{stream};
    default: VXIO_ASSERT_UNREACHABLE();
    }
}

void findBoundaries(const std::vector<real_type> &points, Vec3 &outMin, Vec3 &outMax)
{
    Vec3 min = {points[0], points[1], points[2]};
    Vec3 max = min;

    for (size_t i = 0; i < points.size(); i += 3) {
        Vec3 p{points.data() + i};
        min = obj2voxel::min(min, p);
        max = obj2voxel::max(max, p);
    }

    outMin = min;
    outMax = max;
}

bool loadObj(const std::string &inFile,
             tinyobj::attrib_t &attrib,
             std::vector<tinyobj::shape_t> &shapes,
             std::vector<tinyobj::material_t> &materials)
{
    std::string warn;
    std::string err;

    bool tinyobjSuccess = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, inFile.c_str());

    if (not warn.empty()) {
        VXIO_LOG(WARNING, warn);
    }
    if (not err.empty()) {
        VXIO_LOG(ERROR, err);
    }

    return tinyobjSuccess;
}

std::map<Vec3u, WeightedColor> voxelize_obj(const std::string &inFile, usize resolution, ColorStrategy colorStrategy)
{
    std::map<Vec3u, WeightedColor> voxels;

    // Load obj model
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;

    if (not loadObj(inFile, attrib, shapes, materials)) {
        std::exit(1);
    }
    if (attrib.vertices.empty()) {
        VXIO_LOG(INFO, "Model has no vertices");
        return voxels;
    }
    VXIO_LOG(INFO, "Loaded OBJ model with " + stringify(attrib.vertices.size() / 3) + " vertices");

    // Determine mesh to voxel space transform
    Vec3 meshMin, meshMax;
    findBoundaries(attrib.vertices, meshMin, meshMax);
    const Vec3 meshSize = meshMax - meshMin;
    const real_type scaleFactor = (real_type(resolution) - 0.5f) / max(meshSize[0], meshSize[1], meshSize[2]);

    const auto transform = [meshMin, scaleFactor](Vec3 v) -> Vec3 {
        return (v + meshMin) * scaleFactor;
    };

    // Voxelize mesh
    std::map<std::string, Texture> textures;
    std::vector<TexturedTriangle> buffers[2];
    std::map<Vec3u, WeightedColor> colorBuffer;

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
                    VXIO_LOG(ERROR, "vertex without vertex coordinates found");
                    exit(1);
                }
                Vec3 &vertex = triangle.v[v];
                vertex = Vec3{attrib.vertices.data() + 3 * idx.vertex_index};
                vertex = transform(triangle.v[v]);

                if (idx.texcoord_index >= 0) {
                    triangle.t[v] = Vec2{attrib.texcoords.data() + 2 * idx.texcoord_index};
                }
                else {
                    hasTexCoords = false;
                }
            }
            index_offset += vertexCount;

            int materialIndex = shape.mesh.material_ids[f];
            if (materialIndex < 0) {
                triangle.type = TriangleType::MATERIALLESS;
            }
            else if (hasTexCoords) {
                triangle.texture = &textures[materials[static_cast<usize>(materialIndex)].diffuse_texname];
                triangle.type = TriangleType::TEXTURED;
            }
            else {
                Vec3 color{materials[static_cast<usize>(materialIndex)].diffuse};
                triangle.color = color.cast<float>();
                triangle.type = TriangleType::UNTEXTURED;
            }

            voxelize(triangle, buffers, colorBuffer);
            for (auto &[pos, weightedColor] : colorBuffer) {
                insertColor(voxels, pos, weightedColor, colorStrategy);
            }
            colorBuffer.clear();
        }
    }

    return voxels;
}

constexpr usize VOXEL_BUFFER_64_SIZE = 8 * 1024;
constexpr usize VOXEL_BUFFER_32_SIZE = VOXEL_BUFFER_64_SIZE * 2;

static Voxel32 VOXEL_BUFFER_32[VOXEL_BUFFER_32_SIZE];

[[nodiscard]] int convert_svo_voxelio(svo_type &svo, FileType outFormat, FileOutputStream &out)
{
    std::unique_ptr<AbstractListWriter> writer{makeWriter(out, outFormat)};

    usize voxelCount = 0;
    usize voxelIndex = 0;

    const auto flushBuffer = [&writer, &voxelIndex]() -> bool {
        voxelio::ResultCode writeResult = writer->write(VOXEL_BUFFER_32, voxelIndex);
        if (not voxelio::isGood(writeResult)) {
            VXIO_LOG(ERROR, "Flush/Write error: " + informativeNameOf(writeResult));
            return false;
        }
        voxelIndex = 0;
        return true;
    };

    Vec3i32 svoMin = svo.minIncl();
    auto end = svo.depthFirstNodeRange().end();

    for (auto iter = svo.depthFirstNodeRange().begin(); iter != end; ++iter) {
        if (iter.isAtBranch()) {
            continue;
        }
        usize baseIndex = iter.index();
        auto *leaf = downcast<svo_leaf_type *>(iter.node());

        for (usize i = 0; i < 8; ++i) {
            Vec3u32 uPos;
            voxelio::dileave3(baseIndex + i, uPos.data());
            Vec3i32 pos = uPos.cast<i32>() + svoMin;
            Color32 color = leaf->at(i);

            ++voxelCount;

            VOXEL_BUFFER_32[voxelIndex] = {pos, {color}};
            if (++voxelIndex == VOXEL_BUFFER_32_SIZE) {
                if (not flushBuffer()) {
                    return 1;
                }
            }
        }
    };

    VXIO_LOG(INFO, "Flushing remaining " + stringify(voxelIndex) + " voxels ...");
    VXIO_LOG(INFO, "All voxels written! (" + stringifyLargeInt(voxelCount) + " voxels)");

    int resultCode = not flushBuffer();

    VXIO_LOG(INFO, "Done!");
    return resultCode;
}

[[nodiscard]] int convert_map_voxelio(std::map<Vec3u, WeightedColor> &map,
                                      usize resolution,
                                      FileType outFormat,
                                      FileOutputStream &out)
{
    std::unique_ptr<AbstractListWriter> writer{makeWriter(out, outFormat)};
    writer->setCanvasDimensions(Vec<usize, 3>::filledWith(resolution).cast<u32>());

    usize voxelCount = 0;
    usize voxelIndex = 0;

    const auto flushBuffer = [&writer, &voxelIndex]() -> bool {
        voxelio::ResultCode writeResult = writer->write(VOXEL_BUFFER_32, voxelIndex);
        if (not voxelio::isGood(writeResult)) {
            VXIO_LOG(ERROR, "Flush/Write error: " + informativeNameOf(writeResult));
            return false;
        }
        voxelIndex = 0;
        return true;
    };

    for (auto [pos, weightedColor] : map) {
        Vec3i32 pos32 = pos.cast<i32>();
        Color32 color32 = weightedColor.toColor32();
        ++voxelCount;

        VOXEL_BUFFER_32[voxelIndex] = {pos32, {color32}};
        if (++voxelIndex == VOXEL_BUFFER_32_SIZE) {
            if (not flushBuffer()) {
                return 1;
            }
        }
    };

    VXIO_LOG(INFO, "Flushing remaining " + stringify(voxelIndex) + " voxels ...");
    VXIO_LOG(INFO, "All voxels written! (" + stringifyLargeInt(voxelCount) + " voxels)");

    int resultCode = not flushBuffer();

    VXIO_LOG(INFO, "Done!");
    return resultCode;
}

int main_impl(std::string inFile, std::string outFile, std::string resolutionStr, std::string colorStratStr)
{
    if (inFile.empty()) {
        VXIO_LOG(ERROR, "Input file path must not be empty");
        return 1;
    }

    std::optional<FileType> outType = detectFileType(outFile);
    if (not outType.has_value()) {
        VXIO_LOG(ERROR, "Can't detect file type of \"" + inFile + "\"");
        return 1;
    }

    std::optional<FileOutputStream> outStream = FileOutputStream::open(outFile);
    if (not outStream.has_value()) {
        VXIO_LOG(ERROR, "Failed to open \"" + outFile + "\" for write");
        return 1;
    }

    usize resolution;
    if (not voxelio::parseDec(resolutionStr, resolution)) {
        VXIO_LOG(ERROR, resolutionStr + " is not a valid resolution");
        return 1;
    }

    ColorStrategy colorStrategy;
    toUpperCase(colorStratStr);
    if (not parseColorStrategy(colorStratStr, colorStrategy)) {
        VXIO_LOG(ERROR, "Invalid color strategy \"" + colorStratStr + "\"");
        return 1;
    }

    // all the actual work is done in these two lines
    std::map<Vec3u, WeightedColor> weightedVoxels = voxelize_obj(inFile, resolution, colorStrategy);
    bool success = convert_map_voxelio(weightedVoxels, resolution, *outType, *outStream);

    return not success;
}

}  // namespace obj2voxel

int main(OBJ2VOXEL_MAIN_PARAMS)
{
#ifndef OBJ2VOXEL_TEST
    if (argc < 4) {
        VXIO_LOG(ERROR, "Usage: <in_file:path> <out_file:path> <resolution:uint> [color_strat:(max|blend)=max]");
        return 1;
    }
#endif

    std::string inFile = OBJ2VOXEL_TEST_STRING(argv[1], "/tmp/obj2voxel/in.obj");
    std::string outFile = OBJ2VOXEL_TEST_STRING(argv[2], "/tmp/obj2voxel/out.qef");
    std::string resolutionStr = OBJ2VOXEL_TEST_STRING(argv[3], "16");
    std::string colorStratStr = OBJ2VOXEL_TEST_STRING(argc >= 5 ? argv[4] : "max", "max");

    obj2voxel::main_impl(std::move(inFile), std::move(outFile), std::move(resolutionStr), std::move(colorStratStr));
}
