# obj2voxel

![voxelized sword](img/sword_voxelized.png)

**obj2voxel** is a command-line voxelizer for Wavefront OBJ files.
It uses [tinyobj](https://github.com/tinyobjloader/tinyobjloader) for loading OBJ files and [voxel-io](https://github.com/Eisenwave/voxel-io) for writing voxel models.

## Supported Formats

- **Wavefront OBJ** (Read)
- **STL (Stereolithography)** (Read)
- **QEF** (Write)
- **VL32** (Write)
- **PLY** (Write)

**Note:** VL32 is a format used only by voxel-io.
It's simply an array of `(x,y,z,argb)` 32-bit big-endian integer quadruples.
VL32 is bit-identical to the PLY files exported by obj2voxel when the first **322** header bytes are removed.

The exported PLY files are point clouds consisting of vertices with integer coordinates:
```ply
ply
format binary_big_endian 1.0
element vertex ...
property int x
property int y
property int z
property uchar alpha
property uchar red
property uchar green
property uchar blue
end_header
```

## Installation

Linux:
```sh
mkdir build
cd build
cmake ..
make         # optionally with -j <number of threads> option for multithreaded compile
```

## Usage

```sh
# Usage
obj2voxel <in_file:path> <out_file:path> <resolution:uint> [color_strat:(max|blend)=max]

# Example
obj2voxel in.obj out.qef 128 max
```

**Explanation:** obj2voxel takes only positonal arguments:

- `in_file` is the relative or absolute path to the input file.
  Depending on the extension `.stl` or `.obj` a different input format is chosen.
  If the file type can't be detected, the default is Wavefront OBJ.
- `out_file` is the relative or absolue path to the output file.
  Depending on the extension `.qef` or `.vl32` a different output format is chosen.
  There is no default so obj2voxel exits if it can't be chosen.
- `resolution` is the maximum voxel grid resolution on any axis.
- `color_strat` is a coloring strategy for when multiple triangles occupy one voxel.
  `max` means that the greatest triangle section is chosen for coloring.
  `blend` means that the different triangle sections will be blended together using their areas in the voxel as weights.
  
A usual run of obj2voxel looks like this:
![screenshot](img/terminal_screenshot.png)

## Performance

obj2voxel can produce up to one million voxels per second in optimal circumstances.
Any resolution lower than 1000 should be voxelized almost instantly.

The maximum memory consumption is about 64 bytes per voxel.

## Approach

In case you're curious how obj2voxel voxelizes models:
1. Triangles are first transformed from model space to voxel grid space.
2. Triangles are then subdivided into smaller triangles if their bounding boxes are large.
   This reduces the number of wasted iterations in the next step.
3. For every voxel in the bounding box of the triangle, the triangle is cut at the six bounding planes of the voxel.
   If some portion of the subtriangle remains inside the voxel after all six cuts, the triangle is converted into a pair of weight and color.
   Otherwise, the triangle does not interesect the voxel.
   The weight is the area of the triangle and the color is the material color at the center of the triangle.
4. Colors from multiple triangles are blended together using either `max` or `blend` modes.
