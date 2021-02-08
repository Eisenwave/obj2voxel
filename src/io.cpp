#include "io.hpp"

// TODO consider not including all of voxelization because this is currently happening just for the triangle callback
#include "voxelization.hpp"

#include "voxelio/format/ply.hpp"
#include "voxelio/format/png.hpp"
#include "voxelio/format/qef.hpp"
#include "voxelio/format/vl32.hpp"
#include "voxelio/format/xyzrgb.hpp"

#include "voxelio/filetype.hpp"
#include "voxelio/log.hpp"
#include "voxelio/stream.hpp"
#include "voxelio/stringmanip.hpp"
#include "voxelio/voxelio.hpp"

#include <algorithm>

namespace obj2voxel {

// INPUT ===============================================================================================================

static ByteArrayOutputStream globalDebugStl;

void writeTriangleAsBinaryToDebugStl(Triangle triangle)
{
    Vec3 normal = triangle.normal();
    normal /= length(normal);

    globalDebugStl.writeLittle<3, f32>(normal.data());
    globalDebugStl.writeLittle<3, f32>(triangle.vertex(0).data());
    globalDebugStl.writeLittle<3, f32>(triangle.vertex(1).data());
    globalDebugStl.writeLittle<3, f32>(triangle.vertex(2).data());
    globalDebugStl.writeLittle<u16>(0);
}

void dumpDebugStl(const std::string &path)
{
    u8 buffer[80]{};
    std::optional<FileOutputStream> stlDump = FileOutputStream::open(path);
    VXIO_ASSERT(stlDump.has_value());
    stlDump->write(buffer, sizeof(buffer));
    VXIO_ASSERT_EQ(globalDebugStl.size() % 50, 0);
    stlDump->writeLittle<u32>(static_cast<u32>(globalDebugStl.size() / 50));

    ByteArrayInputStream inStream{globalDebugStl};
    do {
        inStream.read(buffer, 50);
        if (inStream.eof()) break;
        stlDump->write(buffer, 50);
    } while (true);
}

// TRIANGLE STREAMS ====================================================================================================

namespace {

struct ObjTriangleStream : public TriangleStream {
    tinyobj::attrib_t attrib;
    std::vector<tinyobj::shape_t> shapes;
    std::vector<tinyobj::material_t> materials;
    std::map<std::string, Texture> textures;
    Texture *defaultTexture = nullptr;

    usize shapesIndex = 0;
    usize faceIndex = 0;
    usize indexOffset = 0;

    bool hasNext() final
    {
        return shapesIndex < shapes.size() && faceIndex < shapes[shapesIndex].mesh.num_face_vertices.size();
    }

    VisualTriangle next() final;

    usize vertexCount() final
    {
        return attrib.vertices.size() / 3;
    }

    f32 *vertexBegin() final
    {
        return attrib.vertices.data();
    }
};

struct StlTriangleStream : public TriangleStream {
    std::vector<f32> vertices;
    usize index = 0;

    bool hasNext() final
    {
        return index < vertices.size();
    }

    VisualTriangle next() final;

    usize vertexCount() final
    {
        return vertices.size() / 3;
    }

    f32 *vertexBegin() final
    {
        return vertices.data();
    }
};

VisualTriangle ObjTriangleStream::next()
{
    tinyobj::shape_t &shape = shapes[shapesIndex];

    usize vertexCount = shape.mesh.num_face_vertices[faceIndex];
    VXIO_DEBUG_ASSERT_EQ(vertexCount, 3);

    VisualTriangle triangle;
    bool hasTexCoords = true;

    // Loop over vertices in the face.
    for (usize v = 0; v < vertexCount; v++) {
        // access to vertex
        tinyobj::index_t idx = shape.mesh.indices[indexOffset + v];
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

    int materialIndex = shape.mesh.material_ids[faceIndex];
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
        // Even though the voxelizer is detached, this is safe because the voxelizer never modifies its own
        // textures.
        auto location = textures.find(textureName);
        if (location == textures.end()) {
            VXIO_LOG(ERROR,
                     "Face with material \"" + material.name + "\" has unloaded texture name \"" + textureName + '"');
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

    indexOffset += vertexCount;
    ++faceIndex;
    return triangle;
}

VisualTriangle StlTriangleStream::next()
{
    VisualTriangle result;

    for (usize i = 0; i < 3; ++i) {
        result.v[i] = Vec3f{vertices.data() + index}.cast<real_type>();
        result.t[i] = {};
    }
    index += 9;

    result.type = TriangleType::MATERIALLESS;
    return result;
}

}  // namespace

// FILE LOADING ========================================================================================================

std::unique_ptr<TriangleStream> loadObj(const std::string &inFile, const std::string &textureFile)
{
    std::string warn;
    std::string err;

    ObjTriangleStream stream;

    if (not textureFile.empty()) {
        std::optional<Texture> loadedDefault = loadTexture(textureFile, "-t");
        if (loadedDefault.has_value()) {
            auto [location, success] = stream.textures.emplace("", std::move(*loadedDefault));
            VXIO_ASSERTM(success, "Multiple default textures?!");
            stream.defaultTexture = &location->second;
        }
    }

    bool tinyobjSuccess =
        tinyobj::LoadObj(&stream.attrib, &stream.shapes, &stream.materials, &warn, &err, inFile.c_str());
    trim(warn);
    trim(err);

    if (not warn.empty()) {
        std::vector<std::string> warnings = splitAtDelimiter(warn, '\n');
        for (const std::string &warning : warnings) {
            VXIO_LOG(WARNING, "TinyOBJ: " + warning);
        }
    }
    if (not err.empty()) {
        VXIO_LOG(ERROR, "TinyOBJ: " + err);
    }
    if (not tinyobjSuccess) {
        return nullptr;
    }

    for (tinyobj::material_t &material : stream.materials) {
        std::string name = material.diffuse_texname;
        if (name.empty()) {
            continue;
        }
        std::optional<Texture> tex = loadTexture(name, material.name);
        if (tex.has_value()) {
            stream.textures.emplace(std::move(name), std::move(*tex));
        }
    }
    VXIO_LOG(INFO, "Loaded " + stringifyLargeInt(stream.textures.size()) + " textures");

    return std::make_unique<ObjTriangleStream>(std::move(stream));
}

std::unique_ptr<TriangleStream> loadStl(const std::string &inFile)
{
    std::optional<FileInputStream> stream = FileInputStream::open(inFile);
    if (not stream.has_value()) {
        VXIO_LOG(ERROR, "Failed to open STL file: \"" + inFile + "\"");
        std::exit(1);
    }

    char header[80];
    usize headerSize = stream->read(reinterpret_cast<u8 *>(header), sizeof(header));
    if (headerSize != 80) {
        VXIO_LOG(ERROR, "Binary STL file must start with a header of 80 characters");
        std::exit(1);
    }
    if (std::string{header, 5} == "solid") {
        VXIO_LOG(ERROR, "The given file is an ASCII STL file which is not supported");
        std::exit(1);
    }

    u32 triangleCount = stream->readLittle<u32>();
    if (not stream->good()) {
        VXIO_LOG(ERROR, "Couldn't read STL triangle count");
        std::exit(1);
    }

    StlTriangleStream result;
    for (u32 i = 0; i < triangleCount; ++i) {
        f32 triangleData[12];

        stream->readLittle<12, f32>(triangleData);
        stream->readLittle<u16>();
        if (not stream->good()) {
            VXIO_LOG(ERROR, "Unexpected EOF or error when reading triangle");
            std::exit(1);
        }

        result.vertices.insert(result.vertices.end(), triangleData + 3, triangleData + 12);
    }

    return std::make_unique<StlTriangleStream>(std::move(result));
}

std::optional<Texture> loadTexture(const std::string &name, const std::string &material)
{
    std::string sanitizedName = name;
    std::replace(sanitizedName.begin(), sanitizedName.end(), '\\', '/');
    std::optional<FileInputStream> stream = FileInputStream::open(sanitizedName, OpenMode::BINARY);
    if (not stream.has_value()) {
        VXIO_LOG(WARNING, "Failed to open texture file \"" + name + "\" of material \"" + material + '"');
        return std::nullopt;
    }

    std::string err;
    std::optional<Image> image = voxelio::png::decode(*stream, 4, err);
    if (not image.has_value()) {
        VXIO_LOG(WARNING, "Could open, but failed to decode texture \"" + name + "\" of material \"" + material + '"');
        VXIO_LOG(WARNING, "Caused by STBI error: " + err);
        return std::nullopt;
    }
    VXIO_ASSERT(err.empty());
    image->setWrapMode(WrapMode::REPEAT);

    VXIO_LOG(INFO, "Loaded texture \"" + name + "\"");
    return Texture{std::move(*image)};
}

// OUTPUT ==============================================================================================================

constexpr usize VOXEL_BUFFER_BYTE_SIZE = 8192;
constexpr usize VOXEL_BUFFER_32_SIZE = VOXEL_BUFFER_BYTE_SIZE / sizeof(Voxel32);

static Voxel32 VOXEL_BUFFER_32[VOXEL_BUFFER_32_SIZE];

AbstractListWriter *makeWriter(OutputStream &stream, FileType type)
{
    switch (type) {
    case FileType::QUBICLE_EXCHANGE: return new qef::Writer{stream};
    case FileType::VL32: return new vl32::Writer{stream};
    case FileType::STANFORD_TRIANGLE: return new ply::Writer{stream};
    case FileType::XYZRGB: return new xyzrgb::Writer{stream};
    default: VXIO_ASSERT_UNREACHABLE();
    }
}

[[nodiscard]] int writeMapWithVoxelio(VoxelMap<WeightedColor> &map,
                                      usize resolution,
                                      FileType outFormat,
                                      FileOutputStream &out)
{
    std::unique_ptr<AbstractListWriter> writer{makeWriter(out, outFormat)};
    writer->setCanvasDimensions(Vec<usize, 3>::filledWith(resolution).cast<u32>());

    const bool usePalette = requiresPalette(outFormat);
    VXIO_LOG(DEBUG, "Writing " + std::string(nameOf(outFormat)) + (usePalette ? " with" : " without") + " palette");

    if (usePalette) {
        Palette32 &palette = writer->palette();
        for (auto [pos, color] : map) {
            palette.insert(Color32{color.value});
        }
    }

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

    for (auto [index, weightedColor] : map) {
        Vec3u32 pos = map.posOf(index);
        Color32 color = Color32{weightedColor.value};
        VOXEL_BUFFER_32[voxelIndex].pos = pos.cast<i32>();
        if (usePalette) {
            VOXEL_BUFFER_32[voxelIndex].index = writer->palette().indexOf(color);
        }
        else {
            VOXEL_BUFFER_32[voxelIndex].argb = color;
        }

        ++voxelCount;
        if (++voxelIndex == VOXEL_BUFFER_32_SIZE) {
            if (not flushBuffer()) {
                return 1;
            }
        }
    };

    VXIO_LOG(DEBUG, "Flushing remaining " + stringify(voxelIndex) + " voxels ...");
    VXIO_LOG(INFO, "All voxels written! (" + stringifyLargeInt(voxelCount) + " voxels)");

    bool finalSuccess = flushBuffer();
    if (not finalSuccess) {
        return 1;
    }

    writer.reset();

    VXIO_LOG(INFO, "Done!");
    return 0;
}

}  // namespace obj2voxel
