#ifndef OBJ2VOXEL_HEADER
#define OBJ2VOXEL_HEADER

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

// PRIMITIVE/FORWARD DEFINITIONS =======================================================================================

/// Type used for enums.
typedef unsigned char obj2voxel_enum_t;
/// Type used for byte arrays.
typedef unsigned char obj2voxel_byte_t;
/// Type used for error codes.
typedef unsigned char obj2voxel_error_t;

/// The obj2voxel instance. Implementation-defined.
typedef struct obj2voxel_instance obj2voxel_instance;
/// An obj2voxel texture. Implementation-defined.
typedef struct obj2voxel_texture obj2voxel_texture;
/// A triangle with geometry, texture coordinates and other data necessary to voxelized it.
/// Triangles can be basic, in which case no other data is used, they can be textured in which case they need a
/// texture pointer, or they can have a flat color.
typedef struct obj2voxel_triangle obj2voxel_triangle;

/// A callback which iterates over a sequence of triangles.
/// Returns true if loading a triangle succeeded.
typedef bool(obj2voxel_triangle_callback)(void *callback_data, obj2voxel_triangle *out_triangle);
/// A callback which writes voxels to an output.
/// Returns true if writing voxels succeeded.
typedef bool(obj2voxel_voxel_callback)(void *callback_data, uint32_t *voxel_data, size_t voxel_count);
/// A callback which handles log messages.
/// Returns true if the message was handled or false if it should be default-logged.
typedef bool(obj2voxel_log_callback)(void *callback_data, const char *msg, obj2voxel_enum_t level);

// ENUMS ===============================================================================================================

/// Voxel color is determined solely by triangle piece with the gratest area.
static const obj2voxel_enum_t OBJ2VOXEL_MAX_STRATEGY = 0;
/// Voxel color is a weighted average of triangle piece colors, weighted by area.
static const obj2voxel_enum_t OBJ2VOXEL_BLEND_STRATEGY = 1;

/// UV coordinates are clamped to range [0,1].
static const obj2voxel_enum_t OBJ2VOXEL_UV_CLAMP = 0;
/// UV coordinates are wrapped around range [0,1] (for tiling textures).
static const obj2voxel_enum_t OBJ2VOXEL_UV_WRAP = 1;

/// Nothing gets logged.
static const obj2voxel_enum_t OBJ2VOXEL_LOG_LEVEL_SILENT = 0;
/// Errors get logged.
static const obj2voxel_enum_t OBJ2VOXEL_LOG_LEVEL_ERROR = 1;
/// Errors and warnings get logged.
static const obj2voxel_enum_t OBJ2VOXEL_LOG_LEVEL_WARNING = 2;
/// Errors, warnings, and info messages get logged.
static const obj2voxel_enum_t OBJ2VOXEL_LOG_LEVEL_INFO = 3;
/// All messages, including debug messages get logged.
static const obj2voxel_enum_t OBJ2VOXEL_LOG_LEVEL_DEBUG = 4;

/// No error.
static const obj2voxel_error_t OBJ2VOXEL_ERR_OK = 0;
/// No input was provided.
static const obj2voxel_error_t OBJ2VOXEL_ERR_NO_INPUT = 1;
/// No output was provided.
static const obj2voxel_error_t OBJ2VOXEL_ERR_NO_OUTPUT = 2;
/// No resolution was specified.
static const obj2voxel_error_t OBJ2VOXEL_ERR_NO_RESOLUTION = 3;
/// An I/O error occured when attempting to open the input file.
static const obj2voxel_error_t OBJ2VOXEL_ERR_IO_ERROR_ON_OPEN_INPUT_FILE = 4;
/// An I/O error occured when attempting to open the output file.
static const obj2voxel_error_t OBJ2VOXEL_ERR_IO_ERROR_ON_OPEN_OUTPUT_FILE = 5;
/// An I/O error occured when attempting write voxels.
static const obj2voxel_error_t OBJ2VOXEL_ERR_IO_ERROR_DURING_VOXEL_WRITE = 6;
/// Voxelization was attempted after it was already completed once.
/// Instances are throwaway objects, only to be used once!
static const obj2voxel_error_t OBJ2VOXEL_ERR_DOUBLE_VOXELIZATION = 7;

// INSTANCE ============================================================================================================

/**
 * @brief Allocates a new instance.
 * The allocation method is implementation-defined.
 * An instance must be freed using obj2voxel_free().
 * @return the instance
 */
obj2voxel_instance *obj2voxel_alloc(void);

/**
 * @brief Frees an instance created by obj2voxel_alloc().
 * @param instance the instance
 */
void obj2voxel_free(obj2voxel_instance *instance);

// ERROR HANDLING ======================================================================================================

/**
 * @brief Sets the maximum granularity of messages to be logged.
 * For example, if the level is set to OBJ2VOXEL_LOG_LEVEL_WARNING, only warnings, but no info and debug messages are
 * logged.
 * @param level the log level
 */
void obj2voxel_set_log_level(obj2voxel_enum_t level);

/**
 * @brief Sets a custom callback for log messages.
 * By default, log messages simply get printed to stdout.
 * To avoid this and handle potential warnings and errors, a custom callback can be specified.
 * @param callback the callback or nullptr if the behavior should be reset to printing to stdout
 * @param callback_data the data passed to the callback each invocation
 */
void obj2voxel_set_log_callback(obj2voxel_log_callback *callback, void *callback_data);

/**
 * @brief Returns the current log level.
 * @return the current log level
 */
obj2voxel_enum_t obj2voxel_get_log_level(void);

// SETTINGS ============================================================================================================

/**
 * @brief Sets the voxelization resolution on all axes.
 * Setting this to 128 means that the model is voxelized in a 128x128x128 cube.
 * @param instance the instance
 * @param resolution the resolution; must not be zero
 */
void obj2voxel_set_resolution(obj2voxel_instance *instance, uint32_t resolution);

/**
 * @brief Sets the level of supersampling.
 * This is effectively a multiplier of the voxel resolution.
 * @param instance the instance
 * @param level the level; must be 1 or 2
 */
void obj2voxel_set_supersampling(obj2voxel_instance *instance, uint32_t level);

/**
 * @brief Sets the color strategy.
 * See the repo documentation for more on this.
 * @param instance the instance
 * @param strategy OBJ2VOXEL_MAX_STRATEGY or OBJ2VOXEL_BLEND_STRATEGY
 */
void obj2voxel_set_color_strategy(obj2voxel_instance *instance, obj2voxel_enum_t strategy);

/**
 * @brief Adds a fallback texture to the instance.
 * The fallback texture is used for voxelizing input files when a triangle has UV coordinates but no material.
 *
 * The instance does not take ownership of the texture, so it must be kept alive during voxelization and freed
 * afterwards.
 * @param instance the instance
 * @param texture the texture
 */
void obj2voxel_set_texture(obj2voxel_instance *instance, obj2voxel_texture *texture);

/**
 * @brief Sets the input to a file path with an optional type.
 * If the type is not specified, the file type is detected based on the file path and/or contents.
 * The file is not loaded immediately, but when voxelization starts.
 * @param instance the instance
 * @param file the file
 * @param type the file type as an extension without a dot or null for auto-detection (e.g. "vox")
 */
void obj2voxel_set_input_file(obj2voxel_instance *instance, const char *file, const char *type);

/**
 * @brief Sets the input to a callback that iterates over a sequence of triangles.
 * The callback returns a bool which indicates whether another triangle could be loaded.
 * If the callback returns true, it is expected that it modified the triangle it received.
 * @param instance the instance
 * @param callback the callback
 * @param callback_data data passed to the callback each invocation
 */
void obj2voxel_set_input_callback(obj2voxel_instance *instance,
                                  obj2voxel_triangle_callback *callback,
                                  void *callback_data);

/**
 * @brief Sets the output to a file path with an optional type.
 * If the type is not specified, the file type is detected based on the file path and/or contents.
 * The file is not written immediately, but when voxelization starts.
 * @param instance the instance
 * @param file the file
 * @param type the file type as an extension without a dot or null for auto-detection (e.g. "vox")
 */
void obj2voxel_set_output_file(obj2voxel_instance *instance, const char *file, const char *type);

/**
 * @brief Sets the output to be memory with a file type.
 * Voxels will be stored in memory instead of being written to a file.
 * The output bytes can be obtained using obj2voxel_get_output_memory() after voxelization.
 * @param instance the instance
 * @param type the file type as an extension without a dot or null for auto-detection (e.g. "vox")
 */
void obj2voxel_set_output_memory(obj2voxel_instance *instance, const char *type);

/**
 * @brief Sets the output to a callback that consumes voxel data.
 * The data passed to the callback is laid out in VL32 format, meaning (x,y,z,argb).
 * @param instance the instance
 * @param callback the callback
 * @param callback_data data passed to the callback each invocation
 */
void obj2voxel_set_output_callback(obj2voxel_instance *instance,
                                   obj2voxel_voxel_callback *callback,
                                   void *callback_data);

/**
 * @brief Toggles parallelism.
 * Parallelism is disabled by default.
 * When parallelism is enabled, it is the responsibility of the caller to start worker threads using
 * obj2voxel_run_worker() before starting voxelization.
 * @param instance the instance
 * @param enabled true if parallelism should be enabled
 */
void obj2voxel_set_parallel(obj2voxel_instance *instance, bool enabled);

/**
 * @brief Sets a unit cube transformation for the model.
 * The transformation is a linear transformation matrix in row-major format.
 * It should not change the size of the model but merely swap axes or flip the model.
 * @param instance the instance
 * @param transform the transformation matrix
 */
void obj2voxel_set_unit_transform(obj2voxel_instance *instance, const int transform[9]);

/**
 * @brief Sets the mesh boundaries manually.
 * Normally, this information is obtained from the file or callback.
 * However, sometimes boundaries are already known and can be provided here.
 * @param instance the instance
 * @param bounds the mesh boundaries: min_x, min_y, min_z, max_x, max_y, max_z
 */
void obj2voxel_set_mesh_boundaries(obj2voxel_instance *instance, const float bounds[6]);

/**
 * @brief Returns the resolution configured by obj2voxel_set_resolution().
 * Returns zero if no resolution has been set yet.
 * @param instance the instance
 * @return the resolution of the instance or zero
 */
uint32_t obj2voxel_get_resolution(obj2voxel_instance *instance);

/**
 * @brief Returns the chunk size of the instance.
 * Voxelization happens in C*C*C blocks where C is the chunk size.
 * Chunks are laid out in a regular grid with the origin chunk containing the coordinate range [0, C) on all axes.
 * @param instance the instance
 * @return the chunk size of the instance
 */
uint32_t obj2voxel_get_chunk_size(obj2voxel_instance *instance);

/**
 * @brief After voxelization, returns a pointer to the memory written by voxelization.
 * If the output was not set using obj2voxel_set_output_memory, nullptr is returned and out_size remains unchanged.
 * @param instance the instance
 * @param out_size the memory size output parameter
 * @return the pointer to the output memory or nullptr if the output is not a memory output
 */
const obj2voxel_byte_t *obj2voxel_get_output_memory(obj2voxel_instance *instance, size_t *out_size);

// TRIANGLES ===========================================================================================================

/**
 * @brief Sets a triangle to be a basic triangle with three vertices.
 * @param triangle the triangle
 * @param vertices the x, y, z, coordinates of each vertex
 */
void obj2voxel_set_triangle_basic(obj2voxel_triangle *triangle, const float vertices[9]);

/**
 * @brief Sets a triangle to be a single-colored triangle with three vertices and an rgb color.
 * @param triangle the triangle
 * @param vertices the x, y, z, coordinates of each vertex
 * @param color the r, g, b color
 */
void obj2voxel_set_triangle_colored(obj2voxel_triangle *triangle, const float vertices[9], const float color[3]);

/**
 * @brief Sets a triangle to be a textured triangle with three vertices, three UV coordinates and a texture pointer.
 * @param triangle the triangle
 * @param vertices the x, y, z, coordinates of each vertex
 * @param textures the u, v, coordinates of each vertex
 * @param texture the texture pointer (not null)
 */
void obj2voxel_set_triangle_textured(obj2voxel_triangle *triangle,
                                     const float vertices[9],
                                     const float textures[6],
                                     obj2voxel_texture *texture);

// TEXTURES ============================================================================================================

/**
 * @brief Allocates a new texture.
 * @return the allocated texture
 */
obj2voxel_texture *obj2voxel_texture_alloc(void);

/**
 * @brief Frees a texture.
 * @param texture the texture
 */
void obj2voxel_texture_free(obj2voxel_texture *texture);

/**
 * @brief Loads a texture from an image file.
 * @param texture the texture
 * @param file the image file
 * @param type the type as an extension without a dot, or null for auto-detection
 * @return true if the texture could be loaded
 */
bool obj2voxel_texture_load_from_file(obj2voxel_texture *texture, const char *file, const char *type);

/**
 * @brief Loads a texture from memory.
 * @param texture the texture
 * @param data the image bytes
 * @param size the size of the byte data
 * @param type the type as an extension without a dot, or null for auto-detection
 * @return true if the texture could be loaded
 */
bool obj2voxel_texture_load_from_memory(obj2voxel_texture *texture,
                                        const obj2voxel_byte_t *data,
                                        size_t size,
                                        const char *type);

/**
 * @brief Loads a texture from pixel data.
 * The pixel data always has a bit-depth of 8.
 * The number of channels must be 3 or 4 where 3 stands for RGB and 4 stands for ARGB.
 * @param texture the texture
 * @param pixels the pixel data
 * @param width the width
 * @param height the height
 * @param channels the number of channels
 * @return true if the texture could be loaded
 */
bool obj2voxel_texture_load_pixels(
    obj2voxel_texture *texture, const obj2voxel_byte_t *pixels, size_t width, size_t height, size_t channels);

/**
 * @brief Sets the UV mode of the texture.
 * The default is wrap.
 * @param texture the texture
 * @param mode OBJ2VOXEL_UV_CLAMP or OBJ2VOXEL_UV_WRAP
 */
void obj2voxel_teture_set_uv_mode(obj2voxel_texture *texture, obj2voxel_enum_t mode);

/**
 * @brief Gets the metadata of a texture.
 * @param texture the texture
 * @param out_width the output width
 * @param out_height the output height
 * @param out_channels the output channel count which is 3 or 4
 */
void obj2voxel_texture_get_meta(obj2voxel_texture *texture,
                                size_t *out_width,
                                size_t *out_height,
                                size_t *out_channels);

/**
 * @brief Copies the pixels of the texture into an output buffer.
 * The output buffer must be (width * height * channels) bytes large.
 * @param texture the texture
 * @param out_pixels the pixels
 */
void obj2voxel_texture_get_pixels(obj2voxel_texture *texture, obj2voxel_byte_t *out_pixels);

// THREADING ===========================================================================================================

/**
 * @brief Runs a worker thread.
 * At least one worker thread must be running if parallelism is enabled.
 * This method will return eventually after obj2voxel_stop_workers() is called.
 * @param instance the instance
 */
void obj2voxel_run_worker(obj2voxel_instance *instance);

/**
 * @brief Stops all worker threads executing obj2voxel_run_worker().
 * This is done by issuing the command to stop execution to every worker thread.
 *
 * It does not mean that the threads will be joined immediately, it just gets worker threads into a state where it is
 * safe to join them.
 * @param instance the instance
 */
void obj2voxel_stop_workers(obj2voxel_instance *instance);

/**
 * @brief Returns the number of worker threads that are known to the instance.
 * @param instance the instance
 */
uint32_t obj2voxel_get_worker_count(obj2voxel_instance *instance);

// VOXELIZATION ========================================================================================================

/**
 * @brief Voxelizes using all settings, the input, and the output specified.
 * After voxelization, an instance is no longer usable and should be freed.
 * @param instance the instance
 * @return the error code (OBJ2VOXEL_ERR_OK means no error)
 */
obj2voxel_error_t obj2voxel_voxelize(obj2voxel_instance *instance);

#ifdef __cplusplus
}
#endif

#endif
