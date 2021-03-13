#ifndef OBJ2VOXEL_ARRAYVECTOR_HPP
#define OBJ2VOXEL_ARRAYVECTOR_HPP

#include "voxelio/assert.hpp"

#include <cstddef>
#include <type_traits>

namespace obj2voxel {

template <typename T, size_t N, std::enable_if_t<std::is_trivial_v<T>, int> = 0>
struct ArrayVector {
private:
    T data_[N];
    size_t size_ = 0;

public:
    constexpr ArrayVector() = default;

    bool empty() const
    {
        return size_ == 0;
    }

    constexpr size_t size() const
    {
        return size_;
    }

    constexpr T *data()
    {
        return data_;
    }

    constexpr const T *data() const
    {
        return data_;
    }

    constexpr void clear()
    {
        size_ = 0;
    }

    constexpr void push_back(T t)
    {
        VXIO_DEBUG_ASSERT_LT(size_, N);
        data_[size_++] = std::move(t);
    }

    constexpr const T &operator[](size_t i) const
    {
        VXIO_DEBUG_ASSERT_LT(i, size());
        return data_[i];
    }

    constexpr T &operator[](size_t i)
    {
        VXIO_DEBUG_ASSERT_LT(i, size());
        return data_[i];
    }

    constexpr const T *begin() const
    {
        return data_;
    }

    constexpr const T *end() const
    {
        return data_ + size_;
    }

    constexpr T *begin()
    {
        return data_;
    }

    constexpr T *end()
    {
        return data_ + size_;
    }
};

}  // namespace obj2voxel

#endif  // OBJ2VOXEL_ARRAYVECTOR_HPP
