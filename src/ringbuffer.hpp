#ifndef OBJ2VOXEL_RINGBUFFER_HPP
#define OBJ2VOXEL_RINGBUFFER_HPP

#include "voxelio/assert.hpp"

#include <cstddef>
#include <type_traits>

namespace obj2voxel {

/**
 * @brief A simple ring buffer implementation for trivially destructible types.
 * A ring buffer is a FIFO container with a constant capacity.
 */
template <typename T, std::size_t N, std::enable_if_t<std::is_trivial_v<T>, int> = 0>
class RingBuffer {
public:
    static constexpr std::size_t capacity = N;

private:
    T content[N];
    std::size_t r = 0, w = 0, avail = 0;

public:
    constexpr RingBuffer() noexcept = default;

    /// Pops one element from the ring buffer.
    /// The result is always the least recently pushed element.
    /// This method fails if the buffer is empty.
    constexpr T pop() noexcept
    {
        VXIO_DEBUG_ASSERT_NE(avail, 0u);
        --avail;
        r = (r + 1) % N;
        return std::move(content[r]);
    }

    /// Pushes one element to the ring buffer.
    /// This method fails if the buffer is full.
    constexpr void push(T value) noexcept
    {
        VXIO_DEBUG_ASSERT_NE(avail, N);
        ++avail;
        w = (w + 1) % N;
        content[w] = std::move(value);
    }

    /// Clears the ring buffer.
    constexpr void clear() noexcept
    {
        r = w = avail = 0;
    }

    /// Returns the least recently pushed element without popping it.
    /// This method fails if the buffer is empty.
    constexpr const T &peek() const noexcept
    {
        VXIO_DEBUG_ASSERT_NE(avail, 0u);
        return content[r];
    }

    /// Returns the current size of the ring buffer which can be at most N.
    constexpr std::size_t size() const noexcept
    {
        return avail;
    }

    /// Returns true if the buffer is empty.
    constexpr bool empty() const noexcept
    {
        return avail == 0;
    }

    /// Returns true if the buffer is full.
    constexpr bool full() const noexcept
    {
        return avail == N;
    }
};

}  // namespace obj2voxel

#endif  // OBJ2VOXEL_RINGBUFFER_HPP
