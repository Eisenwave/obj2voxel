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

enum class MeshType : obj2voxel_enum_t { TRIANGLE, QUAD };

/**
 * @brief A Java-style iterator/stream which can be used to stream through the triangles of a mesh regardless of
 * internal format.
 */
struct ITriangleStream {
    static std::unique_ptr<ITriangleStream> fromSimpleMesh(MeshType type,
                                                           const float vertices[],
                                                           const usize vertexCount) noexcept;
    static std::unique_ptr<ITriangleStream> fromIndexedMesh(MeshType type,
                                                            const float vertices[],
                                                            const usize elements[],
                                                            usize elementCount) noexcept;

    static std::unique_ptr<ITriangleStream> fromCallback(obj2voxel_triangle_callback callback,
                                                         void *callbackData) noexcept;

    /**
     * @brief Loads an OBJ file from disk.
     * @param inFile the input file
     * @param textureFile the default texture file, to be used for vertices with no material but UV coordinates
     * @return the OBJ triangle stream or nullptr if the file couldn't be opened
     */
    static std::unique_ptr<ITriangleStream> fromObjFile(const std::string &inFile,
                                                        const Texture *defaultTexture) noexcept;

    /**
     * @brief Loads an STL file from disk.
     * The result is a vector of triangle vertices.
     * Each nine coordinates in the vector are one triangle.
     * No normals are stored in the vector.
     * @param inFile the input file path
     * @return the STL triangle stream or nullptr if the file couldn't be opened
     */
    static std::unique_ptr<ITriangleStream> fromStlFile(const std::string &inFile) noexcept;

    /// Virtual destructor.
    virtual ~ITriangleStream() noexcept;

    /// Assigns the next triangle.
    /// Returns true if another triangle could be obtained from the stream, else false.
    /// If false gets returned, this signals the end of the stream and no more triangles should be read.
    virtual bool next(VisualTriangle &out) noexcept = 0;
};

struct IVoxelSink {
    static std::unique_ptr<IVoxelSink> fromCallback(obj2voxel_voxel_callback callback, void *callbackData) noexcept;
    static std::unique_ptr<IVoxelSink> fromVoxelio(std::unique_ptr<OutputStream> out,
                                                   FileType outFormat,
                                                   usize resolution) noexcept;

    /// Virtual destructor, flushes the sink.
    virtual ~IVoxelSink() noexcept;

    /// Returns the underlying output stream or nullptr if the sink is streamless.
    virtual const OutputStream *streamOrNull() const = 0;

    /// Returns true if the writer has not encountered any errors yet and the sink can take more voxels.
    virtual bool canWrite() const noexcept = 0;

    /// Returns the total number of voxels written to the sink.
    virtual usize voxelsWritten() const noexcept = 0;

    /// Writes a buffer of voxels to the sink.
    virtual void write(Voxel32 voxels[], usize size) noexcept = 0;

    /// Flushes the sink.
    virtual void finalize() noexcept = 0;
};

/// Loads a texture with the given file name.
std::optional<Texture> loadTexture(const std::string &name, const std::string &material);

}  // namespace obj2voxel

#endif  // OBJ2VOXEL_IO_HPP
