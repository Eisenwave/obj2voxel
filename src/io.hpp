#ifndef OBJ2VOXEL_IO_HPP
#define OBJ2VOXEL_IO_HPP

#include "triangle.hpp"

#include "voxelio/filetype.hpp"
#include "voxelio/streamfwd.hpp"

#include <optional>
#include <vector>

namespace obj2voxel {

/// Writes a triangle to the debug STL file.
/// This file exists only in memory and dumpDebugStl(...) must be called to write it to disk.
void writeTriangleAsBinaryToDebugStl(Triangle triangle);

/// Dumps the debug STL file at the given file path.
void dumpDebugStl(const std::string &path);

struct TriangleStream {
    virtual ~TriangleStream() = default;
    virtual VisualTriangle next() = 0;
    virtual bool hasNext() = 0;
    virtual usize vertexCount() = 0;

    virtual f32 *vertexBegin() = 0;
};

std::unique_ptr<TriangleStream> loadObj(const std::string &inFile, const std::string &textureFile);

/**
 * @brief Loads an STL file from disk.
 * The result is a vector of triangle vertices.
 * Each nine coordinates in the vector are one triangle.
 * No normals are stored in the vector.
 * @param inFile the input file path
 * @return a list of all vertex coordinates
 */
std::unique_ptr<TriangleStream> loadStl(const std::string &inFile);

/// Loads a texture with the given file name.
std::optional<Texture> loadTexture(const std::string &name, const std::string &material);

/// Writes a voxel map to disk using the voxelio format of choice.
[[nodiscard]] int writeMapWithVoxelio(VoxelMap<WeightedColor> &map,
                                      usize resolution,
                                      FileType outFormat,
                                      FileOutputStream &out);

}  // namespace obj2voxel

#endif  // OBJ2VOXEL_IO_HPP
