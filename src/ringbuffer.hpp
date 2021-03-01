#ifndef OBJ2VOXEL_RINGBUFFER_HPP
#define OBJ2VOXEL_RINGBUFFER_HPP

#include "voxelio/assert.hpp"
#include "voxelio/primitives.hpp"

namespace obj2voxel {

using namespace voxelio;

/**
 * @brief A simple ring buffer implementation for trivially destructible types.
 * A ring buffer is a FIFO container with a constant capacity.
 */
template <typename T, usize N, std::enable_if_t<std::is_trivially_destructible_v<T>, int> = 0>
class RingBuffer {
    T content[N];
    usize r = 0, w = 0, avail = 0;

public:
    constexpr RingBuffer() = default;

    /// Pops one element from the ring buffer.
    /// The result is always the least recently pushed element.
    /// This method fails if the buffer is empty.
    constexpr T pop()
    {
        VXIO_DEBUG_ASSERT_NE(avail, 0);
        --avail;
        r = (r + 1) % N;
        return std::move(content[r]);
    }

    /// Pushes one element to the ring buffer.
    /// This method fails if the buffer is full.
    constexpr void push(T value)
    {
        VXIO_DEBUG_ASSERT_NE(avail, N);
        ++avail;
        w = (w + 1) % N;
        content[w] = std::move(value);
    }

    /// Clears the ring buffer.
    constexpr void clear()
    {
        r = w = avail = 0;
    }

    /// Returns the least recently pushed element without popping it.
    /// This method fails if the buffer is empty.
    constexpr const T &peek() const
    {
        VXIO_DEBUG_ASSERT_NE(avail, 0);
        return content[r];
    }

    /// Returns the current size of the ring buffer which can be at most N.
    constexpr usize size() const
    {
        return avail;
    }

    /// Returns true if the buffer is empty.
    constexpr bool empty() const
    {
        return avail == 0;
    }

    /// Returns true if the buffer is full.
    constexpr bool full() const
    {
        return avail == N;
    }
};

}  // namespace obj2voxel

#endif  // OBJ2VOXEL_RINGBUFFER_HPP
