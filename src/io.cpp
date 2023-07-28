#include "io.hpp"

// TODO consider not including all of voxelization because this is currently happening just for the triangle callback
#include "voxelization.hpp"

#include "voxelio/format/ply.hpp"
#include "voxelio/format/png.hpp"
#include "voxelio/format/qef.hpp"
#include "voxelio/format/vl32.hpp"
#include "voxelio/format/vox.hpp"
#include "voxelio/format/xyzrgb.hpp"

#include "voxelio/fstream.hpp"
#include "voxelio/filetype.hpp"
#include "voxelio/log.hpp"
#include "voxelio/stream.hpp"
#include "voxelio/stringmanip.hpp"
#include "voxelio/voxelio.hpp"

#include <algorithm>

#define TINYOBJLOADER_IMPLEMENTATION
#include "3rd_party/tinyobj.hpp"

static_assert(std::is_same_v<obj2voxel::real_type, tinyobj::real_t>, "obj2voxel::real_type does not match tinyobj");

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
    std::optional<FileOutputStream> stlDump = FileOutputStream(path, OpenMode::WRITE);
    VXIO_ASSERT(stlDump->good());
    stlDump->write(buffer, sizeof(buffer));
    VXIO_ASSERT_DIVISIBLE(globalDebugStl.size(), 50u);
    stlDump->writeLittle<u32>(static_cast<u32>(globalDebugStl.size() / 50));

    ByteArrayInputStream inStream{globalDebugStl};
    do {
        inStream.read(buffer, 50);
        if (inStream.eof()) break;
        stlDump->write(buffer, 50);
    } while (true);
}

// TRIANGLE STREAMS ====================================================================================================

ITriangleStream::~ITriangleStream() noexcept = default;

namespace {

struct CallbackTriangleStream final : public ITriangleStream {
    obj2voxel_triangle_callback *callback;
    void *callbackData;

    CallbackTriangleStream(obj2voxel_triangle_callback *callback, void *callbackData = nullptr)
        : callback{callback}, callbackData{callbackData}
    {
    }

    bool next(VisualTriangle &out) noexcept final
    {
        return callback(callbackData, &out);
    }
};

template <usize PRIM_VERTICES, std::enable_if_t<PRIM_VERTICES == 3 || PRIM_VERTICES == 4, int> = 0>
struct SimpleMeshTriangleStream final : public ITriangleStream {
    static constexpr usize floatsPerVertex = 3;

    const float *vertices;
    usize vertexCount;

    usize vertexIndex = 0;

    SimpleMeshTriangleStream(const float vertices[], usize vertexCount) noexcept
        : vertices{vertices}, vertexCount{vertexCount}
    {
        VXIO_ASSERT_DIVISIBLE(vertexCount, PRIM_VERTICES);
    }

    bool next(VisualTriangle &out) noexcept final
    {
        if (vertexIndex >= vertexCount) {
            return false;
        }

        if (PRIM_VERTICES == 4 && (vertexIndex & 1)) {
            out.v[0] = Vec3{vertices + (vertexIndex - 1) * floatsPerVertex};
            out.v[1] = Vec3{vertices + (vertexIndex + 0) * floatsPerVertex};
            out.v[2] = Vec3{vertices + (vertexIndex + 1) * floatsPerVertex};
            ++vertexIndex;
        }
        else {
            out.v[0] = Vec3{vertices + vertexIndex++ * floatsPerVertex};
            out.v[1] = Vec3{vertices + vertexIndex++ * floatsPerVertex};
            out.v[2] = Vec3{vertices + vertexIndex++ * floatsPerVertex};
        }
        return true;
    }
};

template <usize PRIM_VERTICES, std::enable_if_t<PRIM_VERTICES == 3 || PRIM_VERTICES == 4, int> = 0>
struct IndexedMeshTriangleStream final : public ITriangleStream {
    static constexpr usize floatsPerVertex = 3;

    const float *vertices;
    const usize *elements;

    usize elementCount;

    usize elementIndex = 0;

    IndexedMeshTriangleStream(const float vertices[], const usize elements[], usize elementCount) noexcept
        : vertices{vertices}, elements{elements}, elementCount{elementCount}
    {
        VXIO_ASSERT_DIVISIBLE(elementCount, PRIM_VERTICES);
    }

    bool next(VisualTriangle &out) noexcept final
    {
        if (elementIndex >= elementCount) {
            return false;
        }

        if (PRIM_VERTICES == 4 && (elementIndex & 1)) {
            setVertex(out.v[0], -1);
            setVertex(out.v[1], 0);
            setVertex(out.v[2], 1);
            elementIndex += 1;
        }
        else {
            setVertex(out.v[0], 0);
            setVertex(out.v[1], 1);
            setVertex(out.v[2], 2);
            elementIndex += 3;
        }
        return true;
    }

    void setVertex(Vec3 &dest, int elementOffset) const
    {
        const float *vertex = vertices + elements[elementIndex + static_cast<usize>(elementOffset)] * floatsPerVertex;
        std::memcpy(dest.data(), vertex, sizeof(float) * floatsPerVertex);
    }
};

struct StlTriangleStream final : public ITriangleStream {
private:
    std::vector<f32> vertices;
    usize index = 0;

public:
    StlTriangleStream(std::vector<f32> vertices) noexcept : vertices{std::move(vertices)} {}

    bool next(VisualTriangle &triangle) noexcept final
    {
        if (not hasNext()) {
            return false;
        }

        for (usize i = 0; i < 3; ++i) {
            index += 3;
            triangle.v[i] = Vec3f{vertices.data() + index}.cast<real_type>();
            triangle.t[i] = {};
        }

        triangle.type = TriangleType::MATERIALLESS;
        return true;
    }

private:
    bool hasNext() const noexcept
    {
        return index < vertices.size();
    }
};

struct ObjTriangleStream final : public ITriangleStream {
public:
    using attrib_type = tinyobj::attrib_t;
    using shapes_type = std::vector<tinyobj::shape_t>;
    using materials_type = std::vector<tinyobj::material_t>;
    using textures_type = std::map<std::string, Texture>;

private:
    attrib_type attrib;
    shapes_type shapes;
    materials_type materials;
    textures_type textures;
    const Texture *defaultTexture;

    usize shapesIndex = 0;
    usize faceIndex = 0;
    usize faceCountOfCurrentShape;

    usize indexOffset = 0;

public:
    ObjTriangleStream(attrib_type attrib,
                      shapes_type shapes,
                      materials_type materials,
                      textures_type textures,
                      const Texture *defaultTexture) noexcept
        : attrib{std::move(attrib)}
        , shapes{std::move(shapes)}
        , materials{std::move(materials)}
        , textures{std::move(textures)}
        , defaultTexture{defaultTexture}
        , faceCountOfCurrentShape{faceCountOfShapeOrZero(0)}
    {
    }

    bool next(VisualTriangle &out) noexcept final;

private:
    bool hasNext() const noexcept
    {
        return shapesIndex < shapes.size() && faceIndex < faceCountOfCurrentShape;
    }

    /// Returns the face count of the shape at the given index or zero if the index is out of bounds.
    usize faceCountOfShapeOrZero(usize index) const noexcept
    {
        return index >= shapes.size() ? 0 : shapes[index].mesh.num_face_vertices.size();
    }
};

bool ObjTriangleStream::next(VisualTriangle &triangle) noexcept
{
    if (not hasNext()) {
        return false;
    }

    tinyobj::shape_t &shape = shapes[shapesIndex];

    usize vertexCount = shape.mesh.num_face_vertices[faceIndex];
    VXIO_DEBUG_ASSERT_EQ(vertexCount, 3u);

    bool hasTexCoords = true;

    // Loop over vertices in the face.
    for (usize v = 0; v < vertexCount; v++) {
        // access to vertex
        tinyobj::index_t idx = shape.mesh.indices[indexOffset + v];
        VXIO_ASSERTM(idx.vertex_index >= 0, "Vertex without vertex coordinates found");
        Vec3 &vertex = triangle.v[v];
        vertex = Vec3{attrib.vertices.data() + 3 * idx.vertex_index};

        if (idx.texcoord_index >= 0) {
            triangle.t[v] = Vec2f{attrib.texcoords.data() + 2 * idx.texcoord_index};
        }
        else {
            // Even if this value will never be used by untexture materials, we initialize it.
            // This could lead to accidental denormalized float operations which are expensive.
            triangle.t[v] = {};
            hasTexCoords = false;
        }
    }

    const int materialIndex = shape.mesh.material_ids[faceIndex];
    const tinyobj::material_t *material = materialIndex < 0 ? nullptr : &materials[static_cast<usize>(materialIndex)];

    if (material == nullptr) {
        if (hasTexCoords && defaultTexture != nullptr) {
            triangle.type = TriangleType::TEXTURED;
            triangle.texture = defaultTexture;
        }
        else {
            triangle.type = TriangleType::MATERIALLESS;
        }
    }
    else if (hasTexCoords) {
        const std::string &textureName = material->diffuse_texname;
        if (textureName.empty()) {
            goto untextured;
        }
        auto location = textures.find(textureName);
        VXIO_ASSERTM(location != textures.end(),
                     "Face with material \"" + material->name + "\" has unloaded texture name \"" + textureName + '"');
        triangle.texture = &location->second;
        triangle.type = TriangleType::TEXTURED;
    }
    else {
    untextured:
        triangle.color = Vec3{material->diffuse}.cast<float>();
        triangle.type = TriangleType::UNTEXTURED;
    }

    indexOffset += vertexCount;
    if (++faceIndex >= faceCountOfCurrentShape) {
        faceIndex = 0;
        indexOffset = 0;
        faceCountOfCurrentShape = faceCountOfShapeOrZero(++shapesIndex);
    }
    return true;
}

}  // namespace

std::unique_ptr<ITriangleStream> ITriangleStream::fromSimpleMesh(MeshType type,
                                                                 const float vertices[],
                                                                 usize vertexCount) noexcept
{
    switch (type) {
    case MeshType::TRIANGLE:
        return std::unique_ptr<ITriangleStream>{new SimpleMeshTriangleStream<3>{vertices, vertexCount}};
    case MeshType::QUAD:
        return std::unique_ptr<ITriangleStream>{new SimpleMeshTriangleStream<4>{vertices, vertexCount}};
    }
    VXIO_ASSERT_UNREACHABLE();
}

std::unique_ptr<ITriangleStream> ITriangleStream::fromIndexedMesh(MeshType type,
                                                                  const float vertices[],
                                                                  const usize elements[],
                                                                  usize elementCount) noexcept
{
    switch (type) {
    case MeshType::TRIANGLE:
        return std::unique_ptr<ITriangleStream>{new IndexedMeshTriangleStream<3>{vertices, elements, elementCount}};
    case MeshType::QUAD:
        return std::unique_ptr<ITriangleStream>{new IndexedMeshTriangleStream<4>{vertices, elements, elementCount}};
    }
    VXIO_ASSERT_UNREACHABLE();
}

std::unique_ptr<ITriangleStream> ITriangleStream::fromCallback(obj2voxel_triangle_callback callback,
                                                               void *callbackData) noexcept
{
    return std::unique_ptr<ITriangleStream>{new CallbackTriangleStream{callback, callbackData}};
}

// FILE LOADING ========================================================================================================

std::unique_ptr<ITriangleStream> ITriangleStream::fromObjFile(const std::string &inFile,
                                                              const Texture *defaultTexture) noexcept
{
    std::string warn;
    std::string err;

    ObjTriangleStream::attrib_type attrib;
    ObjTriangleStream::shapes_type shapes;
    ObjTriangleStream::materials_type materials;
    ObjTriangleStream::textures_type textures;

    bool tinyobjSuccess = tinyobj::LoadObj(&attrib, &shapes, &materials, &warn, &err, inFile.c_str());
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

    for (tinyobj::material_t &material : materials) {
        std::string name = material.diffuse_texname;
        if (name.empty()) {
            continue;
        }
        std::optional<Texture> tex = loadTexture(name, material.name);
        if (tex.has_value()) {
            textures.emplace(std::move(name), std::move(*tex));
        }
    }
    VXIO_LOG(INFO, "Loaded " + stringifyLargeInt(textures.size()) + " material textures");

    return std::unique_ptr<ObjTriangleStream>(new ObjTriangleStream{
        std::move(attrib), std::move(shapes), std::move(materials), std::move(textures), defaultTexture});
}

std::unique_ptr<ITriangleStream> ITriangleStream::fromStlFile(const std::string &inFile) noexcept
{
    std::optional<FileInputStream> stream = FileInputStream(inFile, OpenMode::READ);
    if (not stream->good()) {
        VXIO_LOG(ERROR, "Failed to open STL file: \"" + inFile + "\"");
        return nullptr;
    }

    char header[80];
    usize headerSize = stream->read(reinterpret_cast<u8 *>(header), sizeof(header));
    if (headerSize != 80) {
        VXIO_LOG(ERROR, "Binary STL file must start with a header of 80 characters");
        return nullptr;
    }
    if (std::string{header, 5} == "solid") {
        VXIO_LOG(ERROR, "The given file is an ASCII STL file which is not supported");
        return nullptr;
    }

    u32 triangleCount = stream->readLittle<u32>();
    if (not stream->good()) {
        VXIO_LOG(ERROR, "Couldn't read STL triangle count");
        return nullptr;
    }

    std::vector<float> vertices;
    for (u32 i = 0; i < triangleCount; ++i) {
        f32 triangleData[12];

        stream->readLittle<12, f32>(triangleData);
        stream->readLittle<u16>();
        if (not stream->good()) {
            VXIO_LOG(ERROR, "Unexpected EOF or error when reading triangle");
            return nullptr;
        }

        vertices.insert(vertices.end(), triangleData + 3, triangleData + 12);
    }

    return std::unique_ptr<StlTriangleStream>(new StlTriangleStream{std::move(vertices)});
}

std::optional<Texture> loadTexture(const std::string &name, const std::string &material)
{
    std::string sanitizedName = name;
    std::replace(sanitizedName.begin(), sanitizedName.end(), '\\', '/');
    std::optional<FileInputStream> stream = FileInputStream(sanitizedName, OpenMode::BINARY);
    if (not stream->good()) {
        VXIO_LOG(WARNING, "Failed to open texture file \"" + sanitizedName + "\" of material \"" + material + '"');
        return std::nullopt;
    }

    std::string err;
    std::optional<Image> image = voxelio::png::decode(*stream, 4, err);
    if (not image.has_value()) {
        VXIO_LOG(WARNING,
                 "Could open, but failed to decode texture \"" + sanitizedName + "\" of material \"" + material + '"');
        VXIO_LOG(WARNING, "Caused by STBI error: " + err);
        return std::nullopt;
    }
    VXIO_ASSERT(err.empty());
    image->setWrapMode(WrapMode::REPEAT);

    VXIO_LOG(INFO, "Loaded texture \"" + sanitizedName + "\"");
    return Texture{std::move(*image)};
}

// OUTPUT ==============================================================================================================

IVoxelSink::~IVoxelSink() noexcept = default;

namespace {

AbstractListWriter *makeWriter(OutputStream &stream, FileType type)
{
    switch (type) {
    case FileType::MAGICA_VOX: return new vox::Writer{stream};
    case FileType::QUBICLE_EXCHANGE: return new qef::Writer{stream};
    case FileType::VL32: return new vl32::Writer{stream};
    case FileType::STANFORD_TRIANGLE: return new ply::Writer{stream};
    case FileType::XYZRGB: return new xyzrgb::Writer{stream};
    default: VXIO_ASSERT_UNREACHABLE();
    }
}

struct CallbackVoxelSink final : public IVoxelSink {
    obj2voxel_voxel_callback *callback;
    void *callbackData;

    usize voxelCount = 0;
    bool good = true;

    CallbackVoxelSink(obj2voxel_voxel_callback *callback, void *callbackData = nullptr)
        : callback{callback}, callbackData{callbackData}
    {
    }

    const OutputStream *streamOrNull() const final
    {
        return nullptr;
    }

    bool canWrite() const noexcept final
    {
        return good;
    }

    usize voxelsWritten() const noexcept final
    {
        return voxelCount;
    }

    void write(Voxel32 voxels[], usize size) noexcept final;

    void finalize() noexcept final
    {
        VXIO_LOG(DEBUG, "Flushing callback sink (no-op)");
    }
};

/**
 * @brief A Java-style iterator/stream which can be used to stream through the triangles of a mesh regardless of
 * internal format.
 * This stream abstracts from all voxelio specifics such as palettes.
 *
 * Only formats that don't use palettes such as VL32 and XYZRGB can be streamed directly to disk.
 * For other formats like QEF, the full palette must be built before anything can be written.
 * This results in ALL voxels being dumped into a std::vector until flush() is called.
 */
struct VoxelioVoxelSink final : public IVoxelSink {
private:
    static constexpr usize BUFFER_SIZE = 8192;

    std::unique_ptr<OutputStream> stream;
    std::unique_ptr<AbstractListWriter> writer;
    std::vector<Voxel32> buffer;

    usize voxelCount = 0;
    ResultCode err = ResultCode::OK;
    const bool usePalette;
    bool finalized = false;

public:
    VoxelioVoxelSink(std::unique_ptr<OutputStream> out, FileType outFormat, usize resolution);

    /// Destroys the sink. This calls flush(), which can fail.
    /// To avoid errors in the destructor, flush() should be called manually before destruction.
    ~VoxelioVoxelSink() final
    {
        if (isGood(err)) {
            finalize();
            VXIO_ASSERT(isGood(err));
        }
    }

    const OutputStream *streamOrNull() const final
    {
        return stream.get();
    }

    bool canWrite() const noexcept final
    {
        return isGood(err);
    }

    usize voxelsWritten() const noexcept final
    {
        return voxelCount;
    }

    void write(Voxel32 voxels[], usize size) noexcept final;

    void finalize() noexcept final;
};

VoxelioVoxelSink::VoxelioVoxelSink(std::unique_ptr<OutputStream> out, FileType outFormat, usize resolution)
    : stream{std::move(out)}, writer{makeWriter(*stream, outFormat)}, usePalette{requiresPalette(outFormat)}
{
    ResultCode sizeResult = writer->setGlobalVolumeSize(Vec<usize, 3>::filledWith(resolution).cast<u32>());
    VXIO_ASSERT(isGood(sizeResult));

    const bool usePalette = requiresPalette(outFormat);
    VXIO_LOG(DEBUG, "Writing " + std::string(nameOf(outFormat)) + (usePalette ? " with" : " without") + " palette");

    buffer.reserve(BUFFER_SIZE);
}

void VoxelioVoxelSink::write(Voxel32 voxels[], usize size) noexcept
{
    VXIO_ASSERTM(not finalized, "Writing to finalized voxel sink");
    VXIO_ASSERTM(canWrite(), "Writing to a failed voxel sink");

    voxelCount += size;

    if (usePalette) {
        Palette32 &palette = writer->palette();
        for (usize i = 0; i < size; ++i) {
            Voxel32 &voxel = voxels[i];
            voxel.index = palette.insert(voxel.argb);
        }
        this->buffer.insert(buffer.end(), voxels, voxels + size);
    }
    else {
        voxelio::ResultCode writeResult = writer->write(voxels, size);
        if (not voxelio::isGood(writeResult)) {
            VXIO_LOG(ERROR, "Flush/Write error: " + informativeNameOf(writeResult));
            err = writeResult;
        }
    }
}

void VoxelioVoxelSink::finalize() noexcept
{
    if (finalized) {
        return;
    }
    finalized = true;
    if (not isGood(err)) {
        VXIO_LOG(DEBUG, "Skipping voxel sink finalization because writer is failed");
        return;
    }
    VXIO_ASSERT_CONSEQUENCE(not usePalette, buffer.size() == 0);
    VXIO_ASSERT_CONSEQUENCE(usePalette, buffer.size() == voxelCount);

    if (usePalette) {
        VXIO_LOG(DEBUG, "Flushing " + stringify(buffer.size()) + " voxels to paletted writer ...");
        // not strictly necessary but allows us to keep apart init errors and write errors

        if (ResultCode initResult = writer->init(); not isGood(initResult)) {
            err = initResult;
            return;
        }

        if (ResultCode writeResult = writer->write(buffer.data(), buffer.size()); not isGood(writeResult)) {
            err = writeResult;
            return;
        }
        buffer.clear();
    }

    err = writer->finalize();
}

void CallbackVoxelSink::write(Voxel32 voxels[], usize count) noexcept
{
    VXIO_ASSERTM(canWrite(), "Writing to a failed voxel sink");

    // This cast is not standard compliant despite all these assertions.
    // However, because a Voxel32 is essentially a struct of four ints, reinterpreting it as four ints will be safe
    // for virtually all compilers and architectures.
    static_assert(sizeof(Voxel32) == sizeof(uint32_t[4]));
    static_assert(alignof(Voxel32) == alignof(uint32_t[4]));
    static_assert(offsetof(Voxel32, pos) == 0);
    static_assert(offsetof(Voxel32, argb) == 12);

    this->voxelCount += count;
    uint32_t *voxelData = reinterpret_cast<uint32_t *>(voxels);
    good &= callback(callbackData, voxelData, count);
}

}  // namespace

std::unique_ptr<IVoxelSink> IVoxelSink::fromCallback(obj2voxel_voxel_callback *callback, void *callbackData) noexcept
{
    return std::unique_ptr<IVoxelSink>{new CallbackVoxelSink{callback, callbackData}};
}

std::unique_ptr<IVoxelSink> IVoxelSink::fromVoxelio(std::unique_ptr<OutputStream> out,
                                                    FileType outFormat,
                                                    usize resolution) noexcept
{
    return std::unique_ptr<IVoxelSink>{new VoxelioVoxelSink{std::move(out), outFormat, resolution}};
}

}  // namespace obj2voxel
