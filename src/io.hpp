#ifndef OBJ2VOXEL_IO_HPP
#define OBJ2VOXEL_IO_HPP

#include "triangle.hpp"

#include "voxelio/filetype.hpp"
#include "voxelio/log.hpp"
#include "voxelio/streamfwd.hpp"
#include "voxelio/voxelio.hpp"

#include <optional>
#include <vector>

namespace obj2voxel {

/// Writes a triangle to the debug STL file.
/// This file exists only in memory and dumpDebugStl(...) must be called to write it to disk.
void writeTriangleAsBinaryToDebugStl(Triangle triangle);

/// Dumps the debug STL file at the given file path.
void dumpDebugStl(const std::string &path);

/**
 * @brief A Java-style iterator/stream which can be used to stream through the triangles of a mesh regardless of
 * internal format.
 */
struct ITriangleStream {
    /// virtual destructor
    virtual ~ITriangleStream() = default;

    /// Returns the next triangle.
    /// The behavior of this method is undefined if hasNext() is not true.
    virtual VisualTriangle next() = 0;

    /// Returns true if the stream has at least one more triangle.
    virtual bool hasNext() = 0;

    /// Returns the number of vertices in the inderlying mesh.
    virtual usize vertexCount() const = 0;

    /// Returns the begin of the flattened vertex data.
    virtual const f32 *vertexBegin() const = 0;
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
struct VoxelSink {
private:
    static constexpr usize BUFFER_SIZE = 8192;

    std::unique_ptr<AbstractListWriter> writer;
    const bool usePalette;
    std::vector<Voxel32> buffer;
    ResultCode err = ResultCode::OK;
    usize voxelCount = 0;

public:
    VoxelSink(OutputStream &out, FileType outFormat, usize resolution);

    /// Destroys the sink. This calls flush(), which can fail.
    /// To avoid errors in the destructor, flush() should be called manually before destruction.
    ~VoxelSink()
    {
        flush();
    }

    /// Returns true if the writer has not encountered any errors yet and the sink can take more voxels.
    bool canWrite()
    {
        return err == ResultCode::OK;
    }

    /// Writes a buffer of voxels to the sink.
    void write(Voxel32 voxels[], usize size);

    /// Flushes the sink.
    void flush();
};

/**
 * @brief Loads an OBJ file from disk.
 * @param inFile the input file
 * @param textureFile the default texture file, to be used for vertices with no material but UV coordinates
 * @return the OBJ triangle stream or nullptr if the file couldn't be opened
 */
std::unique_ptr<ITriangleStream> loadObj(const std::string &inFile, const std::string &textureFile);

/**
 * @brief Loads an STL file from disk.
 * The result is a vector of triangle vertices.
 * Each nine coordinates in the vector are one triangle.
 * No normals are stored in the vector.
 * @param inFile the input file path
 * @return the STL triangle stream or nullptr if the file couldn't be opened
 */
std::unique_ptr<ITriangleStream> loadStl(const std::string &inFile);

/// Loads a texture with the given file name.
std::optional<Texture> loadTexture(const std::string &name, const std::string &material);

}  // namespace obj2voxel

#endif  // OBJ2VOXEL_IO_HPP
