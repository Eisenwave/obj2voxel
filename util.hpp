#ifndef WEIGHTED_HPP
#define WEIGHTED_HPP

#include "3rd_party/tinyobj.hpp"

#include "voxelio/color.hpp"
#include "voxelio/types.hpp"
#include "voxelio/vec.hpp"

#include <map>

namespace obj2voxel {

using namespace voxelio;
using real_type = tinyobj::real_t;

// VEC UTILITY FUNCTIONS ===============================================================================================

namespace detail {

/**
 * @brief Applies a binary function to each component of two vectors and returns the result.
 */
template <typename T, size_t N, typename BinaryFunction>
constexpr Vec<T, N> applyBinary(Vec<T, N> a, Vec<T, N> b, BinaryFunction function)
{
    Vec<T, N> result{};
    for (usize i = 0; i < N; ++i) {
        result[i] = function(a[i], b[i]);
    }
    return result;
}

/**
 * @brief Applies a unary function to each component of two vectors and returns the result.
 */
template <typename T, size_t N, typename UnaryFunction>
constexpr Vec<T, N> applyUnary(Vec<T, N> a, UnaryFunction function)
{
    Vec<T, N> result{};
    for (usize i = 0; i < N; ++i) {
        result[i] = function(a[i]);
    }
    return result;
}

}  // namespace detail

/// Returns the component-wise min.
template <typename T, size_t N>
constexpr Vec<T, N> min(Vec<T, N> a, Vec<T, N> b)
{
    return detail::applyBinary<T, N>(a, b, std::min<T>);
}

/// Returns the component-wise min.
template <typename T, size_t N>
constexpr Vec<T, N> max(Vec<T, N> a, Vec<T, N> b)
{
    return detail::applyBinary<T, N>(a, b, std::max<T>);
}

/// Three-parameter min. For Vec types, returns the component-wise minimum.
template <typename T>
constexpr T min(const T &a, const T &b, const T &c)
{
    if constexpr (voxelio::isVec<T>) {
        return obj2voxel::min(a, obj2voxel::min(b, c));
    }
    else {
        return std::min(a, std::min(b, c));
    }
}

/// Three-parameter min. For Vec types, returns the component-wise maximum.
template <typename T>
constexpr T max(const T &a, const T &b, const T &c)
{
    if constexpr (voxelio::isVec<T>) {
        return obj2voxel::max(a, obj2voxel::max(b, c));
    }
    else {
        return std::max(a, std::max(b, c));
    }
}

/// Computes the component-wise floor of the vector.
template <typename T, size_t N>
constexpr Vec<T, N> floor(Vec<T, N> v)
{
    T (*function)(T) = std::floor;
    return detail::applyUnary<T, N>(v, function);
}

/// Computes the component-wise ceil of the vector.
template <typename T, size_t N>
constexpr Vec<T, N> ceil(Vec<T, N> v)
{
    T (*function)(T) = std::ceil;
    return detail::applyUnary<T, N>(v, function);
}

/// Computes the component-wise abs of the vector.
template <typename T, size_t N>
constexpr Vec<T, N> abs(Vec<T, N> v)
{
    T (*function)(T) = std::abs;
    return detail::applyUnary<T, N>(v, function);
}

/// Computes the length or magnitude of the vector.
template <typename T, size_t N>
T length(Vec<T, N> v)
{
    T result = std::sqrt(dot<T, T, N>(v, v));
    return result;
}

/// Divides a vector by its length.
template <typename T, size_t N>
constexpr Vec<T, N> normalize(Vec<T, N> v)
{
    return v / length(v);
}

/// Mixes or linearly interpolates two vectors.
template <typename T, size_t N>
constexpr Vec<T, N> mix(Vec<T, N> a, Vec<T, N> b, T t)
{
    return (1 - t) * a + t * b;
}

// WEIGHTED ============================================================================================================

template <typename T>
struct Weighted {
    float weight;
    T value;
};

using WeightedColor = Weighted<Vec<real_type, 3>>;
using WeightedUv = Weighted<Vec<real_type, 2>>;

/// Mixes two colors based on their weights.
template <typename T>
constexpr Weighted<T> mix(const Weighted<T> &lhs, const Weighted<T> &rhs)
{
    float weightSum = lhs.weight + rhs.weight;
    return {weightSum, (lhs.weight * lhs.value + rhs.weight * rhs.value) / weightSum};
}

/// Chooses the color with the greater weight.
template <typename T>
constexpr Weighted<T> max(const Weighted<T> &lhs, const Weighted<T> &rhs)
{
    return lhs.weight > rhs.weight ? lhs : rhs;
}

}  // namespace obj2voxel

#endif  // WEIGHTED_HPP
