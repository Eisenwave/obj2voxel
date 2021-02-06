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

bool loadObj(const std::string &inFile,
             tinyobj::attrib_t &attrib,
             std::vector<tinyobj::shape_t> &shapes,
             std::vector<tinyobj::material_t> &materials)
{
    std::string warn;
    std::string err;

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

    return tinyobjSuccess;
}

std::vector<f32> loadStl(const std::string &inFile)
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

    std::vector<f32> result;
    for (u32 i = 0; i < triangleCount; ++i) {
        f32 triangleData[12];

        stream->readLittle<12, f32>(triangleData);
        stream->readLittle<u16>();
        if (not stream->good()) {
            VXIO_LOG(ERROR, "Unexpected EOF or error when reading triangle");
            std::exit(1);
        }

        result.insert(result.end(), triangleData + 3, triangleData + 12);
    }

    return result;
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
