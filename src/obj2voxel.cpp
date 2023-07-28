#include "obj2voxel.h"

#include "constants.hpp"
#include "io.hpp"
#include "threading.hpp"
#include "voxelization.hpp"

#include "voxelio/format/png.hpp"
#include "voxelio/fstream.hpp"
#include "voxelio/stringify.hpp"

#include <atomic>
#include <ostream>  // we only use this to stringify std::thread::id in a debug log message
#include <thread>

namespace obj2voxel {
namespace {

// WORKER COMMAND ======================================================================================================

enum class CommandType {
    /// Computes boundaries of a triangle batch and combines with currently known mesh bounds.
    FIND_MESH_BOUNDS,
    /// Transforms a triangle and stores its voxel minimum and maximum coordinates.
    TRANSFORM_TRIANGLES,
    /// Instructs a worker thread sort a triangle into chunks.
    SORT_TRIANGLE_INTO_CHUNKS,
    /// Instructs a worker thread to voxelize a chunk.
    VOXELIZE_CHUNK,
    /// Instructs a worker to exit.
    EXIT
};

struct WorkerCommand {
    CommandType type;
    u32 index;
};

// COMMAND QUEUE =======================================================================================================

/// Thread-safe queue of worker commands.
class CommandQueue {
private:
    async::RingBuffer<WorkerCommand, 128> buffer;
    async::Counter<uintmax_t> commandCounter;

public:
    /// Signals completion of one command. To be called by workers.
    void complete()
    {
        --commandCounter;
    }

    /// Receives one command. To be called by workers.
    WorkerCommand receive()
    {
        return buffer.pop();
    }

    /// Issues a command to workers. To be called by main thread.
    void issue(WorkerCommand command)
    {
        ++commandCounter;
        buffer.push(std::move(command));
    }

    /// Waits until all commands have been completed.
    void waitForCompletion()
    {
        commandCounter.waitUntilZero();
    }
};

// INSTANCE DEFINITION =================================================================================================

enum class IoType {
    /// No input or output was specified.
    MISSING,
    /// A file opened for reading/writing.
    FILE,
    /// A file in memory, backed by ByteArrayXXStream.
    MEMORY_FILE,
    /// A callback for reading all triangles or for writing all voxels.
    CALLBACK
};

struct TypedFile {
    const char *path;
    voxelio::FileType type;

    constexpr IoType ioType() const
    {
        return path == nullptr ? IoType::MEMORY_FILE : IoType::FILE;
    }
};

template <typename Callback>
struct CallbackWithData {
    Callback *callback;
    void *data;
};

template <typename Callback>
struct FileOrCallback {
    IoType type;
    union {
        TypedFile file;
        CallbackWithData<Callback> callbackWithData;
    };

    FileOrCallback() : type{IoType::MISSING}, file{nullptr, voxelio::FileType{0}} {}

    FileOrCallback(TypedFile file) : type{file.ioType()}, file{file} {}

    FileOrCallback(CallbackWithData<Callback> callback) : type{IoType::CALLBACK}, callbackWithData{callback} {}

    bool isPresent() const
    {
        return type != IoType::MISSING;
    }
};

struct CachedTriangle : public VisualTriangle {
    Vec3u32 chunkMin;
    Vec3u32 chunkMax;

    void transform(const AffineTransform &transform)
    {
        for (usize i = 0; i < 3; ++i) {
            v[i] = transform * v[i];
        }
    }
};

}  // namespace
}  // namespace obj2voxel

using namespace obj2voxel;

/**
 * @brief A struct that stores all shared state between the main thread and its workers.
 */
struct obj2voxel_instance {
    // configurable
    FileOrCallback<obj2voxel_triangle_callback> input;
    FileOrCallback<obj2voxel_voxel_callback> output;
    Texture *defaultTexture = nullptr;
    Vec3f meshMin = Vec3f::filledWith(std::numeric_limits<float>::infinity());
    Vec3f meshMax = -meshMin;
    ColorStrategy colorStrategy = ColorStrategy::MAX;
    uint32_t outputResolution = 0;
    uint32_t sampleResolution = 0;
    uint32_t supersampling = 1;
    bool parallel = false;
    bool boundsKnown = false;
    int unitTransform[9]{1, 0, 0, 0, 1, 0, 0, 0, 1};

    // initialized during voxelization
    std::unique_ptr<IVoxelSink> voxelSink = nullptr;
    std::vector<CachedTriangle> triangles;
    std::unordered_map<uint64_t, std::vector<uint32_t>> chunks;
    uint32_t chunkCount = 0;
    AffineTransform meshTransform;

    // threading
    CommandQueue queue;
    std::mutex sinkMutex;
    std::mutex workerMutex;
    std::mutex boundsMutex;
    uint32_t workerCount = 0;
    bool workersStopped = false;
    bool sinkWritable = true;
    bool done = false;
};

// ALGORITHM ===========================================================================================================

namespace obj2voxel {
namespace {

void findMeshBounds(obj2voxel_instance &instance, u32 batchStartIndex)
{
    VXIO_DEBUG_ASSERT_LT(batchStartIndex, instance.triangles.size());

    const usize end = std::min(instance.triangles.size(), batchStartIndex + usize{BATCH_SIZE});

    auto min = Vec3::filledWith(std::numeric_limits<real_type>::infinity());
    auto max = -min;

    for (usize i = batchStartIndex; i < end; ++i) {
        const CachedTriangle &triangle = instance.triangles[i];
        min = obj2voxel::min(triangle.min(), min);
        max = obj2voxel::max(triangle.max(), max);
    }

    {
        std::lock_guard<std::mutex> lock{instance.boundsMutex};
        instance.meshMin = obj2voxel::min(instance.meshMin, min);
        instance.meshMax = obj2voxel::max(instance.meshMax, max);
    }
}

void applyMeshTransform(obj2voxel_instance &instance, u32 batchStartIndex)
{
    VXIO_DEBUG_ASSERT_LT(batchStartIndex, instance.triangles.size());

    const usize end = std::min(instance.triangles.size(), batchStartIndex + usize{BATCH_SIZE});
    for (usize i = batchStartIndex; i < end; ++i) {
        CachedTriangle &triangle = instance.triangles[i];
        triangle.transform(instance.meshTransform);

        const Vec3u32 voxelMin = triangle.voxelMin();
        const Vec3u32 voxelMax = triangle.voxelMax();

        // TODO does this min really work when a triangle is exactly on a chunk boundary?
        // TODO investigate a model with a plane that halves it, see if plane is voxelized if it lies between chunks

        // voxelMax() returns an exclusive bound which we need to make inclusive again
        triangle.chunkMin = voxelMin / CHUNK_SIZE;
        triangle.chunkMax = (voxelMax - Vec3u32::one()) / CHUNK_SIZE;

        VXIO_IF_DEBUG(Vec3u32 chunkMaxInVoxelSpace = triangle.chunkMax * CHUNK_SIZE + Vec3u32::filledWith(CHUNK_SIZE));
        VXIO_DEBUG_ASSERTM(obj2voxel::min(voxelMax, chunkMaxInVoxelSpace) == voxelMax, "Potentially lost voxels");
    }
}

void sortTriangleIntoChunks(obj2voxel_instance &instance, u32 triangleIndex)
{
    VXIO_DEBUG_ASSERT_LT(triangleIndex, instance.triangles.size());

    const CachedTriangle &triangle = instance.triangles[triangleIndex];
    const Vec3u32 min = triangle.chunkMin;
    const Vec3u32 max = triangle.chunkMax;

    for (u32 z = min.z(); z <= max.z(); ++z) {
        for (u32 y = min.y(); y <= max.y(); ++y) {
            for (u32 x = min.x(); x <= max.x(); ++x) {
                u64 morton = ileave3(x, y, z);
                VXIO_DEBUG_ASSERT_LT(morton, instance.chunkCount);
                instance.chunks[morton].push_back(triangleIndex);
            }
        }
    }
}

void computeChunkBounds(u64 morton, Vec3u32 &outMin, Vec3u32 &outMax)
{
    Vec3u32 chunkPos;
    dileave3(morton, chunkPos.data());

    outMin = chunkPos * CHUNK_SIZE;
    outMax = outMin + Vec3u32::filledWith(CHUNK_SIZE);
}

void voxelizeChunk(obj2voxel_instance &instance, Voxelizer &voxelizer, u32 chunkIndex)
{
    VXIO_ASSERT(voxelizer.voxels().empty());

    // it's okay that we don't use the mutex here, this is just an optional pre-emptive check
    if (not instance.sinkWritable) {
        VXIO_LOG(DEBUG, "Aborting chunk voxelization because sink is not writable");
        return;
    }

    const std::vector<u32> &chunk = instance.chunks.at(chunkIndex);

    Vec3u32 chunkMin, chunkMax;
    computeChunkBounds(chunkIndex, chunkMin, chunkMax);
    VXIO_ASSERT(chunkMin != chunkMax);

    for (u32 triangle : chunk) {
        voxelizer.voxelize(instance.triangles[triangle], chunkMin, chunkMax);
    }

    if (instance.supersampling > 1) {
        VXIO_ASSERT_LT(instance.supersampling, 3u);
        voxelizer.downscale();
    }

    const usize voxelCount = voxelizer.voxels().size();
    // TODO consider making this a member of worker thread instead
    const auto buffer = std::make_unique<Voxel32[]>(voxelCount);

    u32 i = 0;
    for (auto [index, color] : voxelizer.voxels()) {
        Vec3u32 pos32 = VoxelMap<WeightedColor>::posOf(index);
        if constexpr (build::DEBUG) {
            VXIO_DEBUG_ASSERT_EQ(index / (CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE), chunkIndex);
            for (usize i = 0; i < 3; ++i) {
                VXIO_DEBUG_ASSERT_GE(pos32[i], chunkMin[i]);
                VXIO_DEBUG_ASSERT_LT(pos32[i], chunkMax[i]);
            }
        }

        Color32 color32{color.value};
        buffer[i++] = {pos32.cast<i32>(), {color32.argb()}};
    }
    VXIO_ASSERT_EQ(i, voxelCount);
    {
        std::lock_guard<std::mutex> lock{instance.sinkMutex};
        if (instance.sinkWritable &= instance.voxelSink->canWrite()) {
            instance.voxelSink->write(buffer.get(), voxelCount);
        }
    }
    if (instance.sinkWritable) {
        VXIO_LOG(SPAM,
                 "Voxelized chunk " + stringifyOct(chunkIndex) + " t:" + stringifyDec(chunk.size()) + " -> " +
                     stringifyDec(voxelCount));
    }
    else {
        VXIO_LOG(ERROR, "Can't write voxels because sink has failed (IO error?)");
    }

    voxelizer.voxels().clear();
}

// MAIN THREAD UTILITY =================================================================================================

voxelio::FileType detectFileType(const char *file, const char *type)
{
    std::optional<voxelio::FileType> fileType;
    if (type != nullptr) {
        std::string typeStr{type};
        fileType = voxelio::fileTypeOfExtension(typeStr);
        VXIO_ASSERTM(fileType.has_value(), typeStr + " is not a recognized file extension");
    }
    else if (file != nullptr) {
        std::string fileStr{file};
        fileType = voxelio::detectFileType(file);
        VXIO_ASSERTM(fileType.has_value(), fileStr + " does not have a recognizable extension");
    }
    else {
        VXIO_ASSERTM(false, "Neither file path nor type were given");
    }

    return *fileType;
}

constexpr ColorFormat colorFormatOfChannelCount(size_t channels)
{
    switch (channels) {
    case 3: return ColorFormat::RGB24;
    case 4: return ColorFormat::ARGB32;
    }
    VXIO_ASSERT_UNREACHABLE();
}

LogLevel voxelioLogLevelOf(obj2voxel_enum_t level)
{
    switch (level) {
    case OBJ2VOXEL_LOG_LEVEL_SILENT: return LogLevel::NONE;
    case OBJ2VOXEL_LOG_LEVEL_ERROR: return LogLevel::ERROR;
    case OBJ2VOXEL_LOG_LEVEL_WARNING: return LogLevel::WARNING;
    case OBJ2VOXEL_LOG_LEVEL_INFO: return LogLevel::INFO;
    case OBJ2VOXEL_LOG_LEVEL_DEBUG: return LogLevel::DEBUG;
    }
    VXIO_ASSERT_UNREACHABLE();
}

obj2voxel_enum_t obj2voxelLogLevelOf(LogLevel level)
{
    switch (level) {
    case LogLevel::NONE: return OBJ2VOXEL_LOG_LEVEL_SILENT;
    case LogLevel::ERROR: return OBJ2VOXEL_LOG_LEVEL_ERROR;
    case LogLevel::WARNING: return OBJ2VOXEL_LOG_LEVEL_WARNING;
    case LogLevel::INFO: return OBJ2VOXEL_LOG_LEVEL_INFO;
    default: return OBJ2VOXEL_LOG_LEVEL_DEBUG;
    }
}

AffineTransform computeMeshTransform(obj2voxel_instance &instance)
{
    constexpr float ANTI_BLEED = 0.5f;

    const Vec3 meshSize = instance.meshMax - instance.meshMin;
    const real_type maxOfAllAxes = obj2voxel::max(meshSize[0], meshSize[1], meshSize[2]);
    const real_type sampleScale = real_type(instance.sampleResolution) - ANTI_BLEED;

    VXIO_DEBUG_ASSERT_NE(instance.sampleResolution, 0u);

    // translate to positive octant [0, t]
    AffineTransform result{1, -instance.meshMin};
    // scale and translate to unit cube [-1, 1]
    result = AffineTransform{real_type{2} / maxOfAllAxes, -Vec3::one()} * result;
    // unit transform and offset back to [0, 2]
    result = AffineTransform::fromUnitTransform(instance.unitTransform, Vec3::one()) * result;
    // range transform to voxel grid [a/2, t-a/2]
    result = AffineTransform{sampleScale / 2, Vec3::filledWith(ANTI_BLEED / 2)} * result;

    if (result.isUniformScale()) {
        VXIO_LOG(DEBUG,
                 "Mesh transform = uniform scale (" + stringify(result.matrix[0][0]) + ") + translation " +
                     result.translation.toString());
    }
    else {
        for (usize i = 0; i < 3; ++i) {
            VXIO_LOG(DEBUG, "Mesh transform [" + stringify(i) + "] = " + result.matrix[i].toString());
        }
        VXIO_LOG(DEBUG, "Mesh translation = " + result.translation.toString());
    }

    return result;
}

// VOXELIZATION MAIN FUNCTIONALITY =====================================================================================

/// A specialized class that implements stages of the voxelization pipeline in either parallel or single-threaded way.
template <bool PARALLEL>
struct VoxelizationHelper {
    void voxelizeChunk(u32 chunkIndex);
    void findMeshBounds(u32 batchStartIndex);
    void transformTriangles(u32 batchStartIndex);
    void waitForCompletion();
};

template <>
struct VoxelizationHelper<true> {
    obj2voxel_instance &instance;

    void voxelizeChunk(u32 chunkIndex)
    {
        if (instance.chunks.find(chunkIndex) != instance.chunks.end()) {
            instance.queue.issue({CommandType::VOXELIZE_CHUNK, chunkIndex});
        }
    }

    void findMeshBounds(u32 batchStartIndex)
    {
        instance.queue.issue({CommandType::FIND_MESH_BOUNDS, batchStartIndex});
    }

    void transformTriangles(u32 batchStartIndex)
    {
        instance.queue.issue({CommandType::TRANSFORM_TRIANGLES, batchStartIndex});
    }

    void waitForCompletion()
    {
        instance.queue.waitForCompletion();
    }
};

template <>
struct VoxelizationHelper<false> {
    obj2voxel_instance &instance;
    Voxelizer voxelizer{instance.colorStrategy};

    void voxelizeChunk(u32 chunkIndex)
    {
        if (instance.chunks.find(chunkIndex) != instance.chunks.end()) {
            obj2voxel::voxelizeChunk(instance, voxelizer, chunkIndex);
        }
    }

    void findMeshBounds(u32 batchStartIndex)
    {
        obj2voxel::findMeshBounds(instance, batchStartIndex);
    }

    void transformTriangles(u32 batchStartIndex)
    {
        obj2voxel::applyMeshTransform(instance, batchStartIndex);
    }

    void waitForCompletion() {}
};

template <bool PARALLEL>
[[nodiscard]] obj2voxel_error_t voxelize_specialized(obj2voxel_instance &instance)
{
    VoxelizationHelper<PARALLEL> helper{instance};

    VXIO_LOG(DEBUG, "Sorting triangles into chunks ...");
    const usize triangleCount = instance.triangles.size();

    if (not instance.boundsKnown) {
        for (u32 i = 0; i < triangleCount; i += BATCH_SIZE) {
            helper.findMeshBounds(i);
        }
        helper.waitForCompletion();
    }

    instance.meshTransform = computeMeshTransform(instance);

    for (u32 i = 0; i < triangleCount; i += BATCH_SIZE) {
        helper.transformTriangles(i);
    }
    helper.waitForCompletion();

    for (u32 i = 0; i < triangleCount; ++i) {
        sortTriangleIntoChunks(instance, i);
    }

    if (instance.supersampling > 1) {
        VXIO_LOG(INFO,
                 "Chunks will be downscaled from " + stringifyLargeInt(instance.sampleResolution) +
                     " to output resolution " + stringifyLargeInt(instance.outputResolution) + " ...");
    }

    helper.waitForCompletion();

    VXIO_LOG(DEBUG, "Voxelizing ...");
    // TODO consider simply iterating over pairs instead
    for (u32 i = 0; i < instance.chunkCount; ++i) {
        helper.voxelizeChunk(i);
    }

    helper.waitForCompletion();

    if (not instance.voxelSink->canWrite()) {
        VXIO_LOG(ERROR, "Voxelization failed because of IO error");
        return OBJ2VOXEL_ERR_IO_ERROR_DURING_VOXEL_WRITE;
    }

    VXIO_LOG(INFO, "Voxelized " + stringifyLargeInt(triangleCount) + " triangles, writing any buffered voxels ...");

    instance.voxelSink->finalize();

    VXIO_LOG(INFO, "All " + stringifyLargeInt(instance.voxelSink->voxelsWritten()) + " voxels written");
    return OBJ2VOXEL_ERR_OK;
}

std::unique_ptr<ITriangleStream> openInput(obj2voxel_instance &instance)
{
    FileOrCallback<obj2voxel_triangle_callback> &input = instance.input;

    VXIO_ASSERT(input.isPresent());

    switch (input.type) {
    case IoType::CALLBACK: {
        return ITriangleStream::fromCallback(input.callbackWithData.callback, input.callbackWithData.data);
    }
    case IoType::FILE: {
        switch (input.file.type) {
        case FileType::WAVEFRONT_OBJ: return ITriangleStream::fromObjFile(input.file.path, instance.defaultTexture);
        case FileType::STEREOLITHOGRAPHY: return ITriangleStream::fromStlFile(input.file.path);
        default: return nullptr;
        }
    }
    default: VXIO_ASSERT_UNREACHABLE();
    }
}

std::unique_ptr<IVoxelSink> openOutput(obj2voxel_instance &instance)
{
    FileOrCallback<obj2voxel_voxel_callback> &output = instance.output;

    VXIO_ASSERT(output.isPresent());

    switch (output.type) {
    case IoType::MISSING: {
        VXIO_ASSERT_UNREACHABLE();
    }

    case IoType::CALLBACK: {
        return IVoxelSink::fromCallback(output.callbackWithData.callback, output.callbackWithData.data);
    }

    case IoType::FILE: {
        std::optional<FileOutputStream> stream = FileOutputStream(output.file.path, OpenMode::BINARY);
        if (not stream->good()) {
            return nullptr;
        }

        std::unique_ptr<OutputStream> streamPtr{new FileOutputStream{std::move(*stream)}};

        return IVoxelSink::fromVoxelio(std::move(streamPtr), output.file.type, instance.outputResolution);
    }

    case IoType::MEMORY_FILE: {
        std::unique_ptr<ByteArrayOutputStream> streamPtr{new ByteArrayOutputStream};

        return IVoxelSink::fromVoxelio(std::move(streamPtr), output.file.type, instance.outputResolution);
    }
    }
    VXIO_ASSERT_UNREACHABLE();
}

[[nodiscard]] obj2voxel_error_t voxelize(obj2voxel_instance &instance, ITriangleStream &stream)
{
    u32 chunkCountCbrt = divCeil(instance.sampleResolution, CHUNK_SIZE);
    instance.chunkCount = chunkCountCbrt * chunkCountCbrt * chunkCountCbrt;

    VXIO_LOG(DEBUG, "Caching triangles ...");

    CachedTriangle triangle{};
    while (stream.next(triangle)) {
        instance.triangles.push_back(triangle);
    }

    if (instance.triangles.empty()) {
        VXIO_LOG(WARNING, "Model has no triangles, aborting and writing empty voxel model");
        instance.voxelSink->finalize();
        return instance.voxelSink->canWrite() ? OBJ2VOXEL_ERR_OK : OBJ2VOXEL_ERR_IO_ERROR_DURING_VOXEL_WRITE;
    }
    else {
        VXIO_LOG(INFO, "Cached model with " + stringifyLargeInt(instance.triangles.size()) + " triangles");

        return instance.parallel ? voxelize_specialized<true>(instance) : voxelize_specialized<false>(instance);
    }
}

[[nodiscard]] obj2voxel_error_t voxelize(obj2voxel_instance &instance)
{
    if (instance.done) {
        return OBJ2VOXEL_ERR_DOUBLE_VOXELIZATION;
    }
    if (not instance.input.isPresent()) {
        VXIO_LOG(ERROR, "No input was specified");
        return OBJ2VOXEL_ERR_NO_INPUT;
    }
    if (not instance.output.isPresent()) {
        VXIO_LOG(ERROR, "No output was specified");
        return OBJ2VOXEL_ERR_NO_OUTPUT;
    }
    if (instance.outputResolution == 0) {
        VXIO_LOG(ERROR, "No resolution was specified");
        return OBJ2VOXEL_ERR_NO_RESOLUTION;
    }

    std::unique_ptr<ITriangleStream> input = openInput(instance);
    if (input == nullptr) {
        return OBJ2VOXEL_ERR_IO_ERROR_ON_OPEN_INPUT_FILE;
    }

    instance.voxelSink = openOutput(instance);
    if (instance.voxelSink == nullptr) {
        return OBJ2VOXEL_ERR_IO_ERROR_ON_OPEN_OUTPUT_FILE;
    }

    obj2voxel_error_t result = voxelize(instance, *input);
    if (instance.output.type != IoType::MEMORY_FILE) {
        instance.voxelSink.reset();
    }

    instance.done = true;
    return result;
}

static obj2voxel_log_callback *logCallback = nullptr;
static void *logCallbackData = nullptr;

}  // namespace
}  // namespace obj2voxel

// API IMPLEMENTATION ==================================================================================================

obj2voxel_instance *obj2voxel_alloc(void)
{
    return new obj2voxel_instance;
}

void obj2voxel_free(obj2voxel_instance *instance)
{
    VXIO_ASSERT_NOTNULL(instance);
    delete instance;
}

void obj2voxel_set_log_level(obj2voxel_enum_t level)
{
    voxelio::setLogLevel(voxelioLogLevelOf(level));
}

obj2voxel_enum_t obj2voxel_get_log_level()
{
    return obj2voxelLogLevelOf(voxelio::getLogLevel());
}

void obj2voxel_set_log_callback(obj2voxel_log_callback *callback, void *callback_data)
{
    if (callback == nullptr) {
        voxelio::setLogFormatter(nullptr);
    }

    logCallback = callback;
    logCallbackData = callback_data;

    voxelio::setLogFormatter([](const char *msg, LogLevel level, SourceLocation location) -> void {
        if (not logCallback(logCallbackData, msg, obj2voxelLogLevelOf(level))) {
            defaultFormat(msg, level, location);
        }
    });
}

void obj2voxel_set_resolution(obj2voxel_instance *instance, uint32_t resolution)
{
    VXIO_ASSERT_NOTNULL(instance);
    VXIO_ASSERT_NE(resolution, 0u);
    instance->outputResolution = resolution;
    instance->sampleResolution = resolution * instance->supersampling;
}

void obj2voxel_set_supersampling(obj2voxel_instance *instance, uint32_t level)
{
    VXIO_ASSERT_NOTNULL(instance);
    VXIO_ASSERT_NE(level, 0u);
    instance->supersampling = level;
    instance->sampleResolution = instance->outputResolution * instance->supersampling;
}

void obj2voxel_set_color_strategy(obj2voxel_instance *instance, obj2voxel_enum_t strategy)
{
    VXIO_ASSERT_NOTNULL(instance);
    VXIO_ASSERT_LT(strategy, 2);
    instance->colorStrategy = strategy == OBJ2VOXEL_MAX_STRATEGY ? ColorStrategy::MAX : ColorStrategy::BLEND;
}

void obj2voxel_set_texture(obj2voxel_instance *instance, obj2voxel_texture *texture)
{
    VXIO_ASSERT_NOTNULL(instance);
    VXIO_ASSERT_NOTNULL(texture);
    instance->defaultTexture = texture;
}

void obj2voxel_set_input_file(obj2voxel_instance *instance, const char *file, const char *type)
{
    VXIO_ASSERT_NOTNULL(instance);
    VXIO_ASSERT_NOTNULL(file);

    instance->input = TypedFile{file, detectFileType(file, type)};
}

void obj2voxel_set_input_callback(obj2voxel_instance *instance,
                                  obj2voxel_triangle_callback *callback,
                                  void *callback_data)
{
    VXIO_ASSERT_NOTNULL(instance);
    VXIO_ASSERT_NOTNULL(callback);

    instance->input = CallbackWithData<obj2voxel_triangle_callback>{callback, callback_data};
}

void obj2voxel_set_output_file(obj2voxel_instance *instance, const char *file, const char *type)
{
    VXIO_ASSERT_NOTNULL(instance);
    VXIO_ASSERT_NOTNULL(file);

    instance->output = TypedFile{file, detectFileType(file, type)};
}

void obj2voxel_set_output_memory(obj2voxel_instance *instance, const char *type)
{
    VXIO_ASSERT_NOTNULL(instance);
    VXIO_ASSERT_NOTNULL(type);

    instance->output = TypedFile{nullptr, detectFileType(nullptr, type)};
}

uint32_t obj2voxel_get_resolution(obj2voxel_instance *instance)
{
    VXIO_ASSERT_NOTNULL(instance);

    return instance->outputResolution;
}

uint32_t obj2voxel_get_chunk_size(obj2voxel_instance *instance)
{
    VXIO_ASSERT_NOTNULL(instance);

    return CHUNK_SIZE;
}

const obj2voxel_byte_t *obj2voxel_get_output_memory(obj2voxel_instance *instance, size_t *out_size)
{
    static_assert(std::is_same_v<obj2voxel_byte_t, u8>);

    VXIO_ASSERT_NOTNULL(instance);
    VXIO_ASSERTM(instance->voxelSink != nullptr, "Accessing output memory before voxelization");

    if (instance->output.type != IoType::MEMORY_FILE) {
        return nullptr;
    }
    const OutputStream *stream = instance->voxelSink->streamOrNull();
    VXIO_ASSERT_NOTNULL(stream);

    // this downcast is safe because memory files always use ByteArrayOutputStream
    const ByteArrayOutputStream *byteStream = downcast<const ByteArrayOutputStream *>(stream);
    *out_size = byteStream->size();
    return byteStream->data();
}

void obj2voxel_set_output_callback(obj2voxel_instance *instance,
                                   obj2voxel_voxel_callback *callback,
                                   void *callback_data)
{
    VXIO_ASSERT_NOTNULL(instance);
    VXIO_ASSERT_NOTNULL(callback);

    instance->output = CallbackWithData<obj2voxel_voxel_callback>{callback, callback_data};
}

void obj2voxel_set_parallel(obj2voxel_instance *instance, bool enabled)
{
    VXIO_ASSERT_NOTNULL(instance);
    instance->parallel = enabled;
}

void obj2voxel_set_unit_transform(obj2voxel_instance *instance, const int transform[9])
{
    VXIO_ASSERT_NOTNULL(instance);
    VXIO_ASSERT_NOTNULL(transform);
    std::memcpy(instance->unitTransform, transform, sizeof(instance->unitTransform));
}

void obj2voxel_set_mesh_boundaries(obj2voxel_instance *instance, const float bounds[6])
{
    VXIO_ASSERT_NOTNULL(instance);
    VXIO_ASSERT_NOTNULL(bounds);
    for (usize i = 0; i < 6; ++i) {
        VXIO_ASSERTM(std::isfinite(bounds[i]), "Infinite mesh boundaries provided (" + stringify(bounds[i]) + ')');
    }
    instance->meshMin = Vec3f{bounds};
    instance->meshMax = Vec3f{bounds + 3};
    instance->boundsKnown = true;
    VXIO_ASSERTM(obj2voxel::min(instance->meshMin, instance->meshMax) == instance->meshMin,
                 "Lower mesh bound must be <= the maximum on each axis");
}

void obj2voxel_set_triangle_basic(obj2voxel_triangle *triangle, const float vertices[9])
{
    VXIO_DEBUG_ASSERT_NOTNULL(triangle);
    VXIO_DEBUG_ASSERT_NOTNULL(vertices);
    triangle->type = obj2voxel::TriangleType::MATERIALLESS;
    triangle->v[0] = obj2voxel::Vec3{vertices + 0};
    triangle->v[1] = obj2voxel::Vec3{vertices + 3};
    triangle->v[2] = obj2voxel::Vec3{vertices + 6};
}

void obj2voxel_set_triangle_colored(obj2voxel_triangle *triangle, const float vertices[9], const float color[3])
{
    VXIO_DEBUG_ASSERT_NOTNULL(triangle);
    VXIO_DEBUG_ASSERT_NOTNULL(vertices);
    triangle->type = obj2voxel::TriangleType::MATERIALLESS;
    triangle->v[0] = obj2voxel::Vec3{vertices + 0};
    triangle->v[1] = obj2voxel::Vec3{vertices + 3};
    triangle->v[2] = obj2voxel::Vec3{vertices + 6};
    triangle->color = obj2voxel::Vec3f{color};
}

void obj2voxel_set_triangle_textured(obj2voxel_triangle *triangle,
                                     const float vertices[9],
                                     const float textures[6],
                                     obj2voxel_texture *texture)
{
    VXIO_DEBUG_ASSERT_NOTNULL(triangle);
    VXIO_DEBUG_ASSERT_NOTNULL(vertices);
    triangle->type = obj2voxel::TriangleType::TEXTURED;
    triangle->v[0] = obj2voxel::Vec3{vertices + 0};
    triangle->v[1] = obj2voxel::Vec3{vertices + 3};
    triangle->v[2] = obj2voxel::Vec3{vertices + 6};
    triangle->t[0] = obj2voxel::Vec2f{textures + 0};
    triangle->t[1] = obj2voxel::Vec2f{textures + 2};
    triangle->t[2] = obj2voxel::Vec2f{textures + 4};
    triangle->texture = texture;
}

obj2voxel_texture *obj2voxel_texture_alloc(void)
{
    return new obj2voxel_texture;
}

void obj2voxel_texture_free(obj2voxel_texture *texture)
{
    VXIO_ASSERT_NOTNULL(texture);
    delete texture;
}

bool obj2voxel_texture_load_from_file(obj2voxel_texture *texture, const char *file, const char *type)
{
    VXIO_ASSERT_NOTNULL(texture);
    VXIO_ASSERT_NOTNULL(file);
    FileType fileType = detectFileType(file, type);
    if (categoryOf(fileType) != FileTypeCategory::IMAGE) {
        return false;
    }

    std::optional<FileInputStream> fileStream = FileInputStream(file, OpenMode::BINARY);
    if (not fileStream->good()) {
        return false;
    }
    std::string error;
    std::optional<Image> image = voxelio::png::decode(*fileStream, 4, error);
    if (not image.has_value()) {
        return false;
    }
    texture->image = std::move(image);

    return true;
}

bool obj2voxel_texture_load_from_memory(obj2voxel_texture *texture,
                                        const obj2voxel_byte_t *data,
                                        size_t size,
                                        const char *type)
{
    VXIO_ASSERT_NOTNULL(texture);
    VXIO_ASSERT_NOTNULL(data);
    FileType fileType = detectFileType(nullptr, type);
    if (categoryOf(fileType) != FileTypeCategory::IMAGE) {
        return false;
    }

    std::string error;
    std::optional<Image> image = voxelio::png::decode(data, size, 4, error);
    if (not image.has_value()) {
        return false;
    }
    texture->image = std::move(image);

    return true;
}

bool obj2voxel_texture_load_pixels(
    obj2voxel_texture *texture, const obj2voxel_byte_t *pixels, size_t width, size_t height, size_t channels)
{
    VXIO_ASSERT_NOTNULL(texture);
    VXIO_ASSERT_NOTNULL(pixels);
    VXIO_ASSERT_LT(channels, 5u);
    VXIO_ASSERT_GT(channels, 0u);

    size_t size = size_t{width} * height * channels;
    auto data = std::make_unique<uint8_t[]>(size);
    std::memcpy(data.get(), pixels, size);
    texture->image = voxelio::Image{width, height, colorFormatOfChannelCount(channels), std::move(data)};
    return true;
}

void obj2voxel_teture_set_uv_mode(obj2voxel_texture *texture, obj2voxel_enum_t mode)
{
    VXIO_ASSERTM(texture->image.has_value(), "Can't set UV mode of empty texture");
    auto wrapMode = mode == OBJ2VOXEL_UV_CLAMP ? voxelio::WrapMode::CLAMP : voxelio::WrapMode::REPEAT;
    texture->image->setWrapMode(wrapMode);
}

void obj2voxel_texture_get_meta(obj2voxel_texture *texture, size_t *out_width, size_t *out_height, size_t *out_channels)
{
    VXIO_ASSERT_NOTNULL(texture);
    VXIO_ASSERTM(texture->image.has_value(), "Can't get metadata of empty image");
    *out_width = texture->image->width();
    *out_height = texture->image->height();
    *out_channels = voxelio::channelCountOf(texture->image->format());
}

void obj2voxel_texture_get_pixels(obj2voxel_texture *texture, obj2voxel_byte_t *out_pixels)
{
    VXIO_ASSERT_NOTNULL(texture);
    VXIO_ASSERT_NOTNULL(out_pixels);
    VXIO_ASSERTM(texture->image.has_value(), "Can't get pixels of empty image");
    std::memcpy(out_pixels, texture->image->data(), texture->image->dataSize());
}

obj2voxel_error_t obj2voxel_voxelize(obj2voxel_instance *instance)
{
    VXIO_ASSERT_NOTNULL(instance);
    return obj2voxel::voxelize(*instance);
}

void obj2voxel_run_worker(obj2voxel_instance *instance)
{
    VXIO_ASSERT_NOTNULL(instance);
    using namespace obj2voxel;

    {
        std::lock_guard<std::mutex> lock{instance->workerMutex};
        if (instance->workersStopped) {
            return;
        }
        ++instance->workerCount;
    }

    Voxelizer voxelizer{instance->colorStrategy};

    VXIO_LOG(DEBUG, "VoxelizerThread " + voxelio::stringify(std::this_thread::get_id()) + " started");
    bool looping = true;
    do {
        WorkerCommand command = instance->queue.receive();
        switch (command.type) {
        case CommandType::FIND_MESH_BOUNDS: findMeshBounds(*instance, command.index); break;
        case CommandType::TRANSFORM_TRIANGLES: applyMeshTransform(*instance, command.index); break;
        case CommandType::VOXELIZE_CHUNK: voxelizeChunk(*instance, voxelizer, command.index); break;
        case CommandType::SORT_TRIANGLE_INTO_CHUNKS: sortTriangleIntoChunks(*instance, command.index); break;
        case CommandType::EXIT: looping = false; break;
        }
        instance->queue.complete();
    } while (looping);
}

void obj2voxel_stop_workers(obj2voxel_instance *instance)
{
    VXIO_ASSERT_NOTNULL(instance);
    std::lock_guard<std::mutex> lock{instance->workerMutex};
    instance->workersStopped = true;

    for (; instance->workerCount != 0; --instance->workerCount) {
        instance->queue.issue({CommandType::EXIT, 0});
    }
}

uint32_t obj2voxel_get_worker_count(obj2voxel_instance *instance)
{
    VXIO_ASSERT_NOTNULL(instance);
    std::lock_guard<std::mutex> lock{instance->workerMutex};
    return instance->workerCount;
}
