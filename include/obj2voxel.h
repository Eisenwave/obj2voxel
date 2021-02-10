#ifndef OBJ2VOXEL_HEADER
#define OBJ2VOXEL_HEADER

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// PRIMITIVE/FORWARD DEFINITIONS =======================================================================================

/// Type used for enums.
typedef unsigned char obj2voxel_enum_t;
/// Type used for byte arrays.
typedef unsigned char obj2voxel_byte_t;

/// The obj2voxel instance. Implementation-defined.
typedef struct obj2voxel obj2voxel;
/// An obj2voxel texture. Implementation-defined.
typedef struct obj2voxel_texture obj2voxel_texture;
/// A triangle with geometry, texture coordinates and other data necessary to voxelized it.
/// Triangles can be basic, in which case no other data is used, they can be textured in which case they need a
/// texture pointer, or they can have a flat color.
typedef struct obj2voxel_visual_triangle obj2voxel_visual_triangle;

// ENUMS ===============================================================================================================

#define OBJ2VOXEL_ENUM static const obj2voxel_enum_t

OBJ2VOXEL_ENUM OBJ2VOXEL_MAX_STRATEGY = 0;
OBJ2VOXEL_ENUM OBJ2VOXEL_BLEND_STRATEGY = 1;

OBJ2VOXEL_ENUM OBJ2VOXEL_BASIC_TRIANGLE = 0;
OBJ2VOXEL_ENUM OBJ2VOXEL_TEXTURED_TRIANGLE = 1;
OBJ2VOXEL_ENUM OBJ2VOXEL_COLOR_TRIANGLE = 2;

OBJ2VOXEL_ENUM OBJ2VOXEL_UV_CLAMP = 0;
OBJ2VOXEL_ENUM OBJ2VOXEL_UV_WRAP = 0;

// HELPER TYPES ========================================================================================================

struct obj2voxel_visual_triangle {
    /// The vertex data. (x,y,z)*3
    float vertices[9];
    /// The texture coordinates. (u,v)*3
    float textures[6];
    /// OBJ2VOXEL_BASIC_TRIANGLE, OBJ2VOXEL_TEXTURED_TRIANGLE, or OBJ2VOXEL_COLOR_TRIANGLE
    obj2voxel_enum_t type;

    union {
        /// The texture pointer. Used for OBJ2VOXEL_TEXTURED_TRIANGLE.
        obj2voxel_texture *texture;
        /// The color. Used for OBJ2VOXEL_COLOR_TRIANGLE.
        float color[3];
    };

};

/// A callback which iterates over a sequence of triangles.
typedef bool(*obj2voxel_triangle_callback)(void *callback_data, obj2voxel_visual_triangle *out_triangle);
///
typedef bool(*obj2voxel_voxel_callback)(void *callback_data, uint32_t *voxel_data, size_t voxel_count);

// INSTANCE ============================================================================================================

/**
 * @brief Allocates a new instance.
 * The allocation method is implementation defined.
 * An instance must be freed using obj2voxel_free().
 * @return the instance
 */
obj2voxel *obj2voxel_alloc(void);

/**
 * @brief Frees an instance created by obj2voxel_alloc().
 * @param instance the instance
 */
void obj2voxel_free(obj2voxel *instance);

// SETTINGS ============================================================================================================

/**
 * @brief Sets the voxelization resolution on all axes.
 * Setting this to 128 means that the model is voxelized in a 128x128x128 cube.
 * @param instance the instance
 * @param resolution the resolution; must not be zero
 */
void obj2voxel_set_resolution(obj2voxel *instance, uint32_t resolution);

/**
 * @brief Sets the level of supersampling.
 * This is effectively a multiplier of the voxel resolution.
 * @param instance the instance
 * @param level the level; must be 1 or 2
 */
void obj2voxel_set_supersampling(obj2voxel *instance, uint32_t level);

/**
 * @brief Sets the color strategy.
 * See the repo documentation for more on this.
 * @param instance the instance
 * @param strategy OBJ2VOXEL_MAX_STRATEGY or OBJ2VOXEL_BLEND_STRATEGY
 */
void obj2voxel_set_color_strategy(obj2voxel *instance, obj2voxel_enum_t strategy);

/**
 * @brief Adds a named texture to the instance.
 * This follows move semantics and the original texture is no longer usable, but must still be freed.
 * @param instance the instance
 * @param name the texture name or "" for the fallback texture
 * @param texture the texture
 */
void obj2voxel_move_texture(obj2voxel *instance, const char* name, obj2voxel_texture *texture);

/**
 * @brief Sets the input to a file path with an optional type.
 * If the type is not specified, the file type is detected based on the file path and/or contents.
 * The file is not loaded immediately, but when voxelization starts.
 * @param instance the instance
 * @param file the file
 * @param type the file type as an extension without a dot or null for auto-detection
 */
void obj2voxel_set_input_file(obj2voxel *instance, const char *file, const char *type);

/**
 * @brief Sets the input to a callback that iterates over a sequence of triangles.
 * The callback returns a bool which indicates whether another triangle could be loaded.
 * If the callback returns true, it is expected that it modified the triangle it received.
 * @param instance the instance
 * @param callback the callback
 * @param callback_data data passed to the callback each invocation
 */
void obj2voxel_set_input_callback(obj2voxel *instance, obj2voxel_triangle_callback callback, void *callback_data);

/**
 * @brief Sets the output to a file path with an optional type.
 * If the type is not specified, the file type is detected based on the file path and/or contents.
 * The file is not written immediately, but when voxelization starts.
 * @param instance the instance
 * @param file the file
 * @param type the file type as an extension without a dot or null for auto-detection
 */
void obj2voxel_set_output_file(obj2voxel *instance, const char *file, const char *type);

/**
 * @brief Sets the output to a callback that consumes voxel data.
 * The data passed to the callback is laid out in VL32 format, meaning (x,y,z,argb).
 * @param instance the instance
 * @param callback the callback
 * @param callback_data data passed to the callback each invocation
 */
void obj2voxel_set_output_callback(obj2voxel *instance, obj2voxel_voxel_callback callback, void *callback_data);

/**
 * @brief Toggles parallelism.
 * Parallelism is disabled by default.
 * When parallelism is enabled, it is the responsibility of the caller to start worker threads using
 * obj2voxel_run_worker() before starting voxelization.
 * @param enabled true if parallelism should be enabled
 */
void obj2voxel_set_parallel(bool enabled);

/**
 * @brief Sets a unit cube transformation for the model.
 * The transformation is a linear transformation matrix in row-major format.
 * It should not change the size of the model but merely swap axes or flip the model.
 * @param transform the transformation matrix
 */
void obj2voxel_set_unit_transform(float transform[9]);

/**
 * @brief Sets the mesh boundaries manually.
 * Normally, this information is obtained from the file or callback.
 * However, sometimes boundaries are already known and can be provided here.
 * @param bounds the mesh boundaries: min_x, min_y, min_z, max_x, max_y, max_z
 */
void obj2voxel_set_mesh_boundaries(float min[6]);

// TEXTURES

/**
 * @brief Allocates a new texture.
 * @return the allocated texture
 */
obj2voxel_texture* obj2voxel_texture_alloc(void);

/**
 * @brief Frees a texture.
 * @param texture the texture
 */
void obj2voxel_texture_free(obj2voxel_texture *texture);

/**
 * @brief Loads a texture from an image file.
 * @param texture the texture
 * @param file the image file
 * @param type the type as an extension without a dot or null for auto-detection
 * @return true if the texture could be loaded
 */
bool obj2voxel_texture_load_from_file(obj2voxel_texture *texture, const char *file, const char *type);

/**
 * @brief Loads a texture from memory.
 * @param texture the texture
 * @param data the image bytes
 * @param size the size of the byte data
 * @param type the type as an extension without a dot or null for auto-detection
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
bool obj2voxel_texture_load_pixels(obj2voxel_texture *texture,
                                   const obj2voxel_byte_t *pixels,
                                   uint32_t width, uint32_t height, uint32_t channels);

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
                                uint32_t *out_width,
                                uint32_t *out_height,
                                uint32_t *out_channels);


/**
 * @brief Copies the pixels of the texture into an output buffer.
 * The output buffer must be (width * height * channels) bytes large.
 * @param texture the texture
 * @param out_pixels the pixels
 */
void obj2voxel_texture_get_pixels(obj2voxel_texture *texture, obj2voxel_byte_t *out_pixels);

// VOXELIZATION

/**
 * @brief Voxelizes using all settings, the input, and the output specified.
 * @param instance the instance
 */
void obj2voxel_voxelize(obj2voxel *instance);

/**
 * @brief Runs a worker thread.
 * At least one worker thread must be running if parallelism is enabled.
 * @param instance the instance
 */
void obj2voxel_run_worker(obj2voxel *instance);

#ifdef __cplusplus
}
#endif

#endif
