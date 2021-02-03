#ifndef IO_HPP
#define IO_HPP

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

template <typename Float>
void findBoundaries(const std::vector<Float> &points, Vec<Float, 3> &outMin, Vec<Float, 3> &outMax)
{
    Vec<Float, 3> min = {points[0], points[1], points[2]};
    Vec<Float, 3> max = min;

    for (size_t i = 0; i < points.size(); i += 3) {
        Vec<Float, 3> p{points.data() + i};
        min = obj2voxel::min(min, p);
        max = obj2voxel::max(max, p);
    }

    outMin = min;
    outMax = max;
}

bool loadObj(const std::string &inFile,
             tinyobj::attrib_t &attrib,
             std::vector<tinyobj::shape_t> &shapes,
             std::vector<tinyobj::material_t> &materials);

/**
 * @brief Loads an STL file from disk.
 * The result is a vector of triangle vertices.
 * Each nine coordinates in the vector are one triangle.
 * No normals are stored in the vector.
 * @param inFile the input file path
 * @return a list of all vertex coordinates
 */
std::vector<f32> loadStl(const std::string &inFile);

/// Loads a texture with the given file name.
Texture loadTexture(const std::string &name, const std::string &material);

/// Writes a voxel map to disk using the voxelio format of choice.
[[nodiscard]] int writeMapWithVoxelio(VoxelMap<WeightedColor> &map,
                                      usize resolution,
                                      FileType outFormat,
                                      FileOutputStream &out);

}  // namespace obj2voxel

#endif  // IO_HPP
