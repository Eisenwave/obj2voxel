#include "filevoxelization.hpp"

#include "io.hpp"
#include "threading.hpp"

#include "voxelio/stringify.hpp"

#include <ostream>
#include <thread>

namespace obj2voxel {

namespace {

//#define OBJ2VOXEL_DISABLE_PARALLELISM

// WORKER COMMAND ======================================================================================================

enum class CommandType {
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

#ifndef OBJ2VOXEL_DISABLE_PARALLELISM
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
#endif
};

// ALGORITHM ===========================================================================================================

/**
 * @brief A struct that stores all shared state between the main thread and its workers.
 */
struct Instance {
    // voxelize() function inputss
    ColorStrategy colorStrategy;
    VoxelSink *voxelSink = nullptr;
    bool downscale = false;
    unsigned resolution = 0;

    // late-initialized inputs
    AffineTransform meshTransform;
    u32 chunkCount = 0;

    // data structures
    CommandQueue queue;
    std::vector<VisualTriangle> triangles;
    std::unordered_map<u64, std::vector<u32>> chunks;
    std::mutex sinkMutex;
};

constexpr u32 CHUNK_SIZE = 64;

void sortTriangleIntoChunks(Instance &instance, u32 triangleIndex)
{
    VXIO_DEBUG_ASSERT_LT(triangleIndex, instance.triangles.size());

    const AffineTransform &transform = instance.meshTransform;
    VisualTriangle &triangle = instance.triangles[triangleIndex];

    for (usize i = 0; i < 3; ++i) {
        triangle.v[i] = transform.apply(triangle.v[i]);
    }

    const Vec3u32 voxelMin = triangle.voxelMin();
    const Vec3u32 voxelMax = triangle.voxelMax();

    // TODO does this min really work when a triangle is exactly on a chunk boundary?
    // TODO investigate using a model with a plane that halves it, see if plane is voxelized if it lies between chunks
    // voxelMax() returns an exclusive bound which we need to make inclusive again
    Vec3u32 min = voxelMin / CHUNK_SIZE;
    Vec3u32 max = (voxelMax - Vec3u32::one()) / CHUNK_SIZE;

    Vec3u32 chunkMax = max * CHUNK_SIZE + Vec3u32::filledWith(CHUNK_SIZE);
    VXIO_DEBUG_ASSERT(obj2voxel::min(voxelMax, chunkMax) == voxelMax);

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

void voxelizeChunk(Instance &instance, Voxelizer &voxelizer, u32 chunkIndex)
{
    VXIO_ASSERT(voxelizer.voxels().empty());

    const std::vector<u32> &chunk = instance.chunks.at(chunkIndex);

    Vec3u32 chunkMin, chunkMax;
    computeChunkBounds(chunkIndex, chunkMin, chunkMax);
    VXIO_ASSERT(chunkMin != chunkMax);

    for (u32 triangle : chunk) {
        voxelizer.voxelize(instance.triangles[triangle], chunkMin, chunkMax);
    }

    if (instance.downscale) {
        voxelizer.downscale();
    }

    const usize voxelCount = voxelizer.voxels().size();
    // TODO consider making this a member of worker thread instead
    const auto buffer = std::make_unique<Voxel32[]>(voxelCount);

    u32 i = 0;
    for (auto [index, color] : voxelizer.voxels()) {
        Vec3u32 pos32 = VoxelMap<WeightedColor>::posOf(index);
        if constexpr (build::DEBUG) {
            VXIO_ASSERT_EQ(index / (CHUNK_SIZE * CHUNK_SIZE * CHUNK_SIZE), chunkIndex);
            for (usize i = 0; i < 3; ++i) {
                VXIO_ASSERT_GE(pos32[i], chunkMin[i]);
                VXIO_ASSERT_LT(pos32[i], chunkMax[i]);
            }
        }

        Color32 color32{color.value};
        buffer[i++] = {pos32.cast<i32>(), {color32.argb()}};
    }
    VXIO_ASSERT_EQ(i, voxelCount);
    {
        std::lock_guard<std::mutex> lock{instance.sinkMutex};
        instance.voxelSink->write(buffer.get(), voxelCount);
    }
    VXIO_LOG(SPAM,
             "Voxelized chunk " + stringifyOct(chunkIndex) + " t:" + stringifyDec(chunk.size()) + " -> " +
                 stringifyDec(voxelCount));

    voxelizer.voxels().clear();
}

// WORKER THREAD =======================================================================================================

#ifndef OBJ2VOXEL_DISABLE_PARALLELISM
void runWorker(Instance &instance)
{
    // voxelio::logLevel = LogLevel::SPAM;
    Voxelizer voxelizer{instance.colorStrategy};

    VXIO_LOG(DEBUG, "VoxelizerThread " + stringify(std::this_thread::get_id()) + " started");
    bool looping = true;
    do {
        WorkerCommand command = instance.queue.receive();
        switch (command.type) {
        case CommandType::VOXELIZE_CHUNK: voxelizeChunk(instance, voxelizer, command.index); break;

        case CommandType::SORT_TRIANGLE_INTO_CHUNKS: sortTriangleIntoChunks(instance, command.index); break;

        case CommandType::EXIT: looping = false; break;
        }
        instance.queue.complete();
    } while (looping);
}

void joinWorkers(std::vector<std::thread> &threads, CommandQueue &queue)
{
    for (usize i = 0; i < threads.size(); ++i) {
        queue.issue({CommandType::EXIT, 0});
    }
    for (auto &worker : threads) {
        worker.join();
    }
}
#endif

// MAIN THREAD IMPLEMENTATIONS =========================================================================================

template <typename Float>
void findBoundaries(const Float data[], usize vertexCount, Vec<Float, 3> &outMin, Vec<Float, 3> &outMax)
{
    auto min = Vec<Float, 3>::filledWith(std::numeric_limits<Float>::infinity());
    auto max = -min;

    usize limit = vertexCount * 3;
    for (size_t i = 0; i < limit; i += 3) {
        Vec<Float, 3> p{data + i};
        min = obj2voxel::min(min, p);
        max = obj2voxel::max(max, p);
    }

    outMin = min;
    outMax = max;
}

AffineTransform computeTransform(const ITriangleStream &stream, unsigned resolution, Vec3u permutation)
{
    Vec3 meshMin, meshMax;
    findBoundaries(stream.vertexBegin(), stream.vertexCount(), meshMin, meshMax);
    AffineTransform meshTransform = Voxelizer::computeTransform(meshMin, meshMax, resolution, permutation);

    return meshTransform;
}

// VOXELIZATION MAIN FUNCTIONALITY =====================================================================================

bool voxelize_parallel(Instance &instance, ITriangleStream &stream, unsigned threadCount)
{
    VXIO_ASSERT_NE(threadCount, 0);

    std::vector<std::thread> workers;
    workers.reserve(threadCount);

    VXIO_LOG(DEBUG, "Starting up worker threads ...");

    for (usize i = 0; i < threadCount; ++i) {
        auto &worker = workers.emplace_back(&runWorker, std::ref(instance));
        VXIO_ASSERT(worker.joinable());
    }

    VXIO_LOG(DEBUG, "Sorting triangles into chunks ...");

    while (stream.hasNext()) {
        VisualTriangle triangle = stream.next();
        auto index = static_cast<u32>(instance.triangles.size());

        // We can't push the triangle yet because this may cause a std::vector reallocation.
        // It also doesn't make sense to sort ourselves or to copy the triangle into the command because the worker
        // also applies the mesh transform in-place.
        // TODO investigate if this is still a performance benefit or if we should do everything on the main thread.
        instance.queue.waitForCompletion();

        instance.triangles.push_back(triangle);
        instance.queue.issue({CommandType::SORT_TRIANGLE_INTO_CHUNKS, index});
    }

    if (instance.downscale) {
        VXIO_LOG(INFO,
                 "Chunks will be downscaled from " + stringifyLargeInt(instance.resolution) + " to output resolution " +
                     stringifyLargeInt(instance.resolution / 2) + " ...");
    }

    instance.queue.waitForCompletion();

    VXIO_LOG(DEBUG, "Voxelizing ...");
    for (u32 i = 0; i < instance.chunkCount; ++i) {
        if (instance.chunks.find(i) != instance.chunks.end()) {
            instance.queue.issue({CommandType::VOXELIZE_CHUNK, i});
        }
    }

    instance.queue.waitForCompletion();

    VXIO_LOG(INFO, "Voxelized " + stringifyLargeInt(instance.triangles.size()) + " triangles");

    joinWorkers(workers, instance.queue);

    return true;
}

bool voxelize_single(Instance &instance, ITriangleStream &stream)
{
    // logLevel = LogLevel::SPAM;

    while (stream.hasNext()) {
        auto index = static_cast<u32>(instance.triangles.size());
        instance.triangles.push_back(stream.next());
        sortTriangleIntoChunks(instance, index);
    }

    if (instance.downscale) {
        VXIO_LOG(INFO,
                 "Chunks will be downscaled from " + stringifyLargeInt(instance.resolution) + " to output resolution " +
                     stringifyLargeInt(instance.resolution / 2) + " ...");
    }

    Voxelizer voxelizer{instance.colorStrategy};

    VXIO_LOG(DEBUG, "Voxelizing ...");
    for (u32 i = 0; i < instance.chunkCount; ++i) {
        if (instance.chunks.find(i) != instance.chunks.end()) {
            voxelizeChunk(instance, voxelizer, i);
        }
    }

    VXIO_LOG(INFO, "Voxelized " + stringifyLargeInt(instance.triangles.size()) + " triangles");

    return true;
}

}  // namespace

bool voxelize(VoxelizationArgs args, ITriangleStream &stream, VoxelSink &sink)
{
    if (stream.vertexCount() == 0) {
        VXIO_LOG(WARNING, "Model has no vertices, aborting and writing empty voxel model");
        sink.flush();
        return true;
    }
    VXIO_LOG(INFO, "Loaded model with " + stringifyLargeInt(stream.vertexCount()) + " vertices");

    Instance instance;
    instance.colorStrategy = args.colorStrategy;
    instance.voxelSink = &sink;
    instance.downscale = args.downscale;
    instance.resolution = args.resolution;

    u32 chunkCountCbrt = divCeil(args.resolution, CHUNK_SIZE);
    instance.chunkCount = chunkCountCbrt * chunkCountCbrt * chunkCountCbrt;

    instance.meshTransform = computeTransform(stream, args.resolution, args.permutation);

    return args.workerThreads == 0 ? voxelize_single(instance, stream)
                                   : voxelize_parallel(instance, stream, args.workerThreads);
}

}  // namespace obj2voxel
