#ifndef OBJ2VOXEL_ARRAYVECTOR_HPP
#define OBJ2VOXEL_ARRAYVECTOR_HPP

#include "voxelio/assert.hpp"

#include <cstddef>
#include <type_traits>

namespace obj2voxel {

/**
 * @brief A simple class that implements std::vector-like interface, but backed by a constant-sized array.
 */
template <typename T, std::size_t N, std::enable_if_t<std::is_trivial_v<T>, int> = 0>
struct ArrayVector {
public:
    using iterator = T *;
    using const_iterator = const T *;

    static constexpr std::size_t capacity = N;

private:
    T data_[N];
    std::size_t size_ = 0;

public:
    constexpr ArrayVector() = default;

    /// Returns true if the ArrayVector is empty.
    constexpr bool empty() const
    {
        return size_ == 0;
    }

    /// Returns ture if the ArrayVector can store no more elements.
    constexpr bool full() const
    {
        return size_ >= N;
    }

    /// Returns the amount of stored elements.
    constexpr std::size_t size() const
    {
        return size_;
    }

    /// Returns a pointer to the underlying data.
    constexpr T *data()
    {
        return data_;
    }

    /// Returns a const pointer to the underlying data.
    constexpr const T *data() const
    {
        return data_;
    }

    /// Removes all elements.
    constexpr void clear()
    {
        size_ = 0;
    }

    /// Appends an element to the end of the vector.
    /// This method fails if the ArrayVector can store no more elements, i.e. it is full().
    constexpr void push_back(T t)
    {
        VXIO_DEBUG_ASSERT_LT(size_, N);
        data_[size_++] = std::move(t);
    }

    /// Accesses an element at the given index.
    constexpr const T &operator[](std::size_t i) const
    {
        VXIO_DEBUG_ASSERT_LT(i, size());
        return data_[i];
    }

    /// Accesses an element at the given index.
    constexpr T &operator[](std::size_t i)
    {
        VXIO_DEBUG_ASSERT_LT(i, size());
        return data_[i];
    }

    /// Returns an iterator at the start of the ArrayVector.
    constexpr const T *begin() const
    {
        return data_;
    }

    /// Returns an iterator one past the end of the ArrayVector.
    constexpr const T *end() const
    {
        return data_ + size_;
    }

    /// Returns an iterator at the start of the ArrayVector.
    constexpr T *begin()
    {
        return data_;
    }

    /// Returns an iterator one past the end of the ArrayVector.
    constexpr T *end()
    {
        return data_ + size_;
    }
};

}  // namespace obj2voxel

#endif  // OBJ2VOXEL_ARRAYVECTOR_HPP
