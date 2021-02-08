#include "filevoxelization.hpp"

#include "io.hpp"
#include "threading.hpp"

#include "voxelio/stringify.hpp"

#include <ostream>
#include <thread>

namespace obj2voxel {

namespace {

enum class CommandType { VOXELIZE_TRIANGLE, MERGE_MAPS, EXIT };

struct WorkerCommand {
    static WorkerCommand exit()
    {
        return {};
    }

    struct MergeMaps {
        VoxelMap<WeightedColor> *target;
        VoxelMap<WeightedColor> *source;
    };

    CommandType type;

    union {
        std::nullptr_t nothing;
        VisualTriangle triangle;
        MergeMaps mergeMaps;
    };

    constexpr explicit WorkerCommand(VisualTriangle triangle) : type{CommandType::VOXELIZE_TRIANGLE}, triangle{triangle}
    {
    }

    constexpr WorkerCommand(VoxelMap<WeightedColor> *target, VoxelMap<WeightedColor> *source)
        : type{CommandType::MERGE_MAPS}, mergeMaps{target, source}
    {
        VXIO_ASSERT_NE(target, source);
    }

    constexpr WorkerCommand() : type{CommandType::EXIT}, nothing{nullptr} {}
};

using ring_buffer_type = async::RingBuffer<WorkerCommand, 128>;
using counter_type = async::Counter<uintmax_t>;
using texture_map_type = std::map<std::string, Texture>;

template <typename Float>
void findBoundaries(const float data[], usize vertexCount, Vec<Float, 3> &outMin, Vec<Float, 3> &outMax)
{
    Vec<Float, 3> min = {data[0], data[1], data[2]};
    Vec<Float, 3> max = min;

    usize limit = vertexCount * 3;
    for (size_t i = 0; i < limit; i += 3) {
        Vec<Float, 3> p{data + i};
        min = obj2voxel::min(min, p);
        max = obj2voxel::max(max, p);
    }

    outMin = min;
    outMax = max;
}

class WorkerThread final
    : public Voxelizer
    , public std::thread {
public:
    WorkerThread(AffineTransform meshTransform,
                 ColorStrategy colorStrat,
                 ring_buffer_type &commandBuffer,
                 counter_type &commandCounter)
        : Voxelizer{meshTransform, colorStrat}, std::thread{&WorkerThread::run, this, &commandBuffer, &commandCounter}
    {
    }

    WorkerThread(const WorkerThread &) = delete;
    WorkerThread(WorkerThread &&) = default;

private:
    void run(ring_buffer_type *commandBuffer, counter_type *commandCounter);
};

void WorkerThread::run(ring_buffer_type *commandBuffer, counter_type *commandCounter)
{
    VXIO_DEBUG_ASSERT_NOTNULL(commandBuffer);
    VXIO_DEBUG_ASSERT_NOTNULL(commandCounter);

    VXIO_LOG(DEBUG, "VoxelizerThread " + stringify(get_id()) + " started");
    bool looping = true;
    do {
        WorkerCommand command = commandBuffer->pop();
        switch (command.type) {
        case CommandType::VOXELIZE_TRIANGLE: {
            this->voxelize(std::move(command.triangle));
            break;
        }

        case CommandType::MERGE_MAPS: {
            VXIO_DEBUG_ASSERT_NOTNULL(command.mergeMaps.target);
            VXIO_DEBUG_ASSERT_NOTNULL(command.mergeMaps.source);
            this->merge(*command.mergeMaps.target, *command.mergeMaps.source);
            command.mergeMaps.source->clear();
            break;
        }

        case CommandType::EXIT: {
            looping = false;
            break;
        }
        }
        commandCounter->operator--();
    } while (looping);
}

VoxelMap<WeightedColor> voxelizeTriangles(VoxelizationArgs args, TriangleStream &stream)
{
    if (stream.vertexCount() == 0) {
        VXIO_LOG(WARNING, "Model has no vertices, aborting and writing empty voxel model");
        return {};
    }
    VXIO_LOG(INFO, "Loaded model with " + stringifyLargeInt(stream.vertexCount()) + " vertices");

    // Determine mesh to voxel space transform

    ring_buffer_type commandBuffer;
    counter_type commandCounter;

    usize totalTriangleCount = 0;
    usize threadCount = std::thread::hardware_concurrency();

    Vec3 meshMin, meshMax;
    findBoundaries(stream.vertexBegin(), stream.vertexCount(), meshMin, meshMax);
    AffineTransform meshTransform = Voxelizer::computeTransform(meshMin, meshMax, args.resolution, args.permutation);

    std::vector<WorkerThread> voxelizers;
    voxelizers.reserve(threadCount);
    for (usize i = 0; i < threadCount; ++i) {
        auto &voxelizer = voxelizers.emplace_back(meshTransform, args.colorStrategy, commandBuffer, commandCounter);
        VXIO_ASSERT(voxelizer.joinable());
    }

    // Loop over shapes
    while (stream.hasNext()) {
        ++commandCounter;
        ++totalTriangleCount;
        commandBuffer.push(WorkerCommand{stream.next()});
    }
    VXIO_LOG(DEBUG, "Pushed all triangles, waiting until buffer is empty");

    // We first wait until all remaining triangles have been voxelized.
    commandCounter.waitUntilZero();

    VXIO_LOG(INFO, "Voxelized " + stringifyLargeInt(totalTriangleCount) + " triangles, merging results ...");

    VoxelMap<WeightedColor> result;

    while (true) {
        VoxelMap<WeightedColor> *mergeTarget = nullptr, *mergeSource = nullptr;
        usize commandsIssuedInLoop = 0;

        for (auto &voxelizer : voxelizers) {
            if (voxelizer.voxels().empty()) {
                continue;
            }

            if (mergeTarget == nullptr) {
                mergeTarget = &voxelizer.voxels();
                continue;
            }
            else {
                mergeSource = &voxelizer.voxels();

                if (mergeTarget->size() < mergeSource->size()) {
                    std::swap(mergeTarget, mergeSource);
                }
                ++commandsIssuedInLoop;
                ++commandCounter;
                commandBuffer.push(WorkerCommand{mergeTarget, mergeSource});
                mergeTarget = mergeSource = nullptr;
            }
        }

        if (commandsIssuedInLoop == 0) {
            if (mergeTarget != nullptr) {
                result = std::move(*mergeTarget);
            }
            break;
        }
        commandCounter.waitUntilZero();
    }

    for (usize i = 0; i < voxelizers.size(); ++i) {
        commandBuffer.push(WorkerCommand::exit());
    }
    for (auto &worker : voxelizers) {
        worker.join();
    }

    return result;
}

}  // namespace

// FILE SPECIFICS ======================================================================================================

VoxelMap<WeightedColor> voxelizeObj(VoxelizationArgs args)
{
    std::unique_ptr<TriangleStream> stream = loadObj(args.inFile, args.texture);
    if (stream == nullptr) {
        std::exit(1);
    }

    return voxelizeTriangles(std::move(args), *stream);
}

VoxelMap<WeightedColor> voxelizeStl(VoxelizationArgs args)
{
    std::unique_ptr<TriangleStream> stream = loadStl(args.inFile);
    if (stream == nullptr) {
        std::exit(1);
    }

    return voxelizeTriangles(std::move(args), *stream);
}

}  // namespace obj2voxel
