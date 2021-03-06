# obj2voxel

![voxelized sword](img/sword_voxelized.png)

**obj2voxel** is a command-line voxelizer for Wavefront OBJ files.
It uses [tinyobj](https://github.com/tinyobjloader/tinyobjloader) for loading OBJ files and [voxel-io](https://github.com/Eisenwave/voxel-io) for writing voxel models.

## Installation

Download the [latest release](https://github.com/eisenwave/obj2voxel/releases) executable.
There are prebuilt binaries for Windows and Linux.
Or, if you want to build from source, proceed as follows (on Linux):

```sh
mkdir build
cd build
cmake ..
make         # optionally with -j <number of threads> option for multithreaded compile
```
After installing, the executable will be in your `build` directory.

## Usage

```sh
# Usage
./obj2voxel input_file output_file -r <resolution> # ...

# Example
./obj2voxel in.obj out.qef -t texture.png -r 128 -s max
```

##### `input_file`
is the relative or absolute path to the input file.
Depending on the extension `.stl` or `.obj` a different input format is chosen.
If the file type can't be detected, the default is Wavefront OBJ.
  
##### `output_file`
is the relative or absolue path to the output file.
Depending on the extension `.ply`, `.qef`, etc. a different output format is chosen.
Check the list of supported formats.
There is no default so obj2voxel fails if the file type can't be identified by its extension.

##### `-r <resolution>`
is the voxel grid resolution.
This is a maximum for all axes, meaning that a non-cubical model will still fit into this block.
The output model will be at most r³ voxels large.

##### `-s (max|blend)`
is a coloring strategy for when multiple triangles occupy one voxel.
See below for more details on how this option impacts the voxels.
The default is `max`.

##### `-t <texture>`
is the optional path to a texture file.
This texture is used for triangles with UV coordinates but no materials.
There are some models which don't have material libraries at all.
This option is very useful for those types of models.

##### `-p <permutation>`
is the axis permutation.
The default is `xyz`; another order such as `xzy` may be specified to reorder axes.
This is useful for importing models from software where a different axis is being used for "up".

##### `-u`
enables 2x supersampling.
The model is voxelized at double resolution and then downscaled.
See below for more details.

##### `-j <threads>`
is the number of worker threads to be started.
obj2voxel supports parallelism and if `threads` is not zero, worker threads will be started that voxelize many triangles simultaneously.
This option is set to the number of hardware threads by default.
You can also set it exactly to `0`, which disables paralellism completely.
Setting it to `1` is usually pointless and ends up being slower than just using `-j 0`.

### Usage Example

A usual run of obj2voxel looks like this:
![screenshot](img/terminal_screenshot.png)

### Max vs Blend Strategies

There are two strategies for combining colors in a voxelized model.
When voxelizing, weighted colors are produced where the weights are the areas of the triangle sections inside of a
voxel.

- `max` means that the greatest triangle section is chosen for the color of a voxel.
  Max produces sharper colors and doesn't introduce any colors that weren't in the original mesh.
  However, at low resolutions, it can look noisy and small details from the mesh might disappear.
- `blend` means that triangle sections will be blended together using their weights.
  Blend produces smoother colors and reproduces smaller details at least somewhat.
  However, it introduces new colors and can make the model look blurry.
  For example, blend would produce a magenta edge between a red and blue triangle which might be unwanted.

**Example 1:** "Spot" model. `max` is left, `blend` is right.<br>
![blend vs max using Spot model](img/blend_vs_max_spot.png)

**Example 2:** "Sword" model. `max` is bottom, `blend` is top.<br>
![blend vs max using Sword model](img/blend_vs_max_sword.png)

Supersampling can also improve color accuracy by voxelizing at a higher resolution and blending multiple voxels.
In this comparison, the right cow is supersampled:<br>
![regular vs 2x supersampling](img/supersampling_spot.png)

Supersampling will usually produce slightly more voxels.
  
## Supported Formats

| Name                    | Extension | Purpose    |
| ----------------------- | --------- | ---------- |
| Wavefront OBJ           | `obj`     | Input      |
| Stereolithography       | `stl`     | Input      |
| Stanford Triangle       | `ply`     | Output     |
| Qubicle Exchange Format | `qef`     | Output     |
| Magica Voxel            | `vox`     | Output\*   |
| VL32                    | `vl32`    | Output     |
| XYZRGB                  | `xyzrgb`  | Output\*\* |

**\*Warning:** VOX support is still experimental; writing the file in the end can take a long time because building a 255-color palette is somewhat inefficient.
Use of streamable formats like VL32 is highly recommended, only use VOX for lower resolutions.

**\*\*Note**: XYZRGB's official extension is `xyzrgb` but the software [*FileToVox*](https://github.com/Zarbuz/FileToVox) uses the extension `xyz` instead. Rename the files before importing into *FileToVox*.

### PLY

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
voxel-io works with signed positions which is why `int` is used instead of `uint`, but the positions exported are always
positive.

### VL32

VL32 is a format used only by voxel-io.
It's simply an array of `(x,y,z,argb)` 32-bit big-endian integer quadruples.
VL32 is bit-identical to the PLY files exported by obj2voxel when the first **300** header bytes are removed.
It is always exactly 300 bytes, the voxel-io library makes sure of that.

To read a VL32 file, implement the following pseudo-code:
```cpp
while (not end_of_file_reached()) {
    int32_t x = read_big_endian_int32();
    int32_t y = read_big_endian_int32();
    int32_t z = read_big_endian_int32();
    uint8_t a = read_byte();
    uint8_t r = read_byte();
    uint8_t g = read_byte();
    uint8_t b = read_byte();
}
```

## Performance

On high-end hardware and with some models, obj2voxel can produce 10 million voxels per second.
Voxelization is a highly parallel task and scales very well with high thread counts.
Any resolution lower than 1024 should be voxelized almost instantly, even with a single thread.

The memory consumption of obj2voxel depends on the size of the input model because the model is loaded into memory entirely.
Certain output formats like PLY, VL32 and XYZRGB can be streamed, meaning that obj2voxel will consume very little memory when producing them.
Other formats like QEF require a palette to be constructed, so all voxels must be buffered in memory before they can be written.
This usally requires around 16 bytes per voxel.

The aforementioned streamable formats don't require this and because obj2voxel has a chunk-based approach to voxelization, the memory consumption will be very low.
Even voxelizing at 8192 resolution might require only 100MB.

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
