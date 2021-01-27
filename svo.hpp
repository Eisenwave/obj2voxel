#ifndef SVO_HPP
#define SVO_HPP

#include "voxelio/assert.hpp"
#include "voxelio/bits.hpp"
#include "voxelio/ileave.hpp"
#include "voxelio/stringify.hpp"
#include "voxelio/vec.hpp"

#include <array>
#include <bitset>
#include <cstddef>
#include <memory>
#include <stack>
#include <vector>

namespace mve {

/// The type of a SparseVoxelOctree node.
enum class SvoNodeType {
    /// A branch. This type of node has other nodes as children.
    BRANCH,
    /// A leaf. This type of node has voxels as children.
    LEAF
};

constexpr const char *nameOf(SvoNodeType type)
{
    return type == SvoNodeType::BRANCH ? "BRANCH" : "LEAF";
}

namespace detail {

template <typename T>
struct Voidable {
    T value;
};

template <>
struct Voidable<void> {
};

}  // namespace detail

template <size_t N, typename BT = void>
class SvoNode {
private:
    detail::Voidable<BT> val;

protected:
    std::bitset<N> mask_{};

    virtual void doClear() = 0;

public:
    SvoNode() = default;
    virtual ~SvoNode() = default;

    SvoNode(SvoNode &&) = default;
    SvoNode(const SvoNode &) = default;

    SvoNode &operator=(SvoNode &&) = default;
    SvoNode &operator=(const SvoNode &) = default;

    const std::bitset<N> &mask() const
    {
        return mask_;
    }

    /**
     * Returns the number of children for branches or the number of colors for leafs.
     * This is not a recursive test.
     * @return the number of children
     */
    size_t count() const
    {
        return mask_.count();
    }

    void clear()
    {
        doClear();
        mask_.reset();
    }

    /**
     * Returns whether this node has no children.
     * This is not a recursive test, meaning that true will be returned, even if all children are empty.
     * @return true if this node has no children
     */
    bool empty() const
    {
        return mask_.none();
    }

    /**
     * Returns whether all children of this node exist.
     * This is not a recursive test.
     * @return true if this node has all possible children
     */
    bool full() const
    {
        return mask_.all();
    }

    /**
     * Returns true if the node has a child at the given index.
     * @param index the index
     * @return true if the node has a child at index
     */
    bool has(size_t index) const
    {
        VXIO_DEBUG_ASSERT_LT(index, N);
        return mask_.test(index);
    }

    /**
     * @brief Returns the index of the first child which exists or N if the node is empty.
     * @return the index of the first child
     */
    size_t firstIndex() const
    {
        if constexpr (N == 8) {
            return voxelio::countTrailingZeros(static_cast<uint8_t>(mask_.to_ullong()));
        }
        else if constexpr (N == 64) {
            return voxelio::countTrailingZeros(static_cast<uint64_t>(mask_.to_ullong()));
        }
        else {
            if (mask_.none()) {
                return N;
            }
            size_t i = 0;
            for (; i < N; ++i) {
                if (mask_.test(i)) {
                    break;
                }
            }
            return i;
        }
    }

    template <typename V = BT, std::enable_if_t<not std::is_void_v<V>, int> = 0>
    auto &value()
    {
        return val.value;
    }

    template <typename V = BT, std::enable_if_t<not std::is_void_v<V>, int> = 0>
    const auto &value() const
    {
        return val.value;
    }

    /**
     * Clones this node virtually. The result is an unmanaged pointer on the heap.
     * @return a pointer to the cloned object
     */
    virtual SvoNode *clone() const = 0;

    virtual SvoNodeType type() const = 0;
};

template <size_t N, typename BT>
std::array<std::unique_ptr<SvoNode<N, BT>>, N> deepCopy(const std::array<std::unique_ptr<SvoNode<N, BT>>, N> &copyOf)
{
    std::array<std::unique_ptr<SvoNode<N, BT>>, N> result;
    for (size_t i = 0; i < N; ++i) {
        result[i] = std::unique_ptr<SvoNode<N, BT>>(copyOf[i]->clone());
    }
    return result;
}

template <size_t N, typename BT = void>
class SvoBranch : public SvoNode<N, BT> {
public:
    using self_type = SvoBranch<N, BT>;
    using child_type = SvoNode<N, BT>;

private:
    template <typename T>
    using uptr = std::unique_ptr<T>;

    std::array<uptr<child_type>, N> children{};

protected:
    void doClear() final;

public:
    SvoBranch() = default;
    SvoBranch(const SvoBranch &copyOf) : SvoNode<N, BT>{copyOf}, children{deepCopy(copyOf.children)} {}
    SvoBranch(SvoBranch &&moveOf) = default;
    ~SvoBranch() final = default;

    SvoBranch *clone() const final
    {
        return new SvoBranch{*this};
    }

    SvoNodeType type() const final
    {
        return SvoNodeType::BRANCH;
    }

    child_type *child(size_t index)
    {
        VXIO_DEBUG_ASSERT_LT(index, N);
        return children[index].get();
    }

    const child_type *child(size_t index) const
    {
        VXIO_DEBUG_ASSERT_LT(index, N);
        return children[index].get();
    }

    uptr<child_type> extract(size_t index)
    {
        VXIO_DEBUG_ASSERT_LT(index, N);
        VXIO_DEBUG_ASSERT(this->has(index));
        this->mask_.reset(index);
        return std::move(children[index]);
    }

    void insert(size_t index, uptr<child_type> node)
    {
        VXIO_DEBUG_ASSERT_LT(index, N);
        children[index] = std::move(node);
        this->mask_.set(index);
    }

    SvoBranch &operator=(const SvoBranch &copyOf)
    {
        children = deepCopy(copyOf.children);
        this->mask_ = copyOf.mask_;
        return *this;
    }

    SvoBranch &operator=(SvoBranch &&moveOf) = default;
};

template <typename T, size_t N, typename BT = void>
class SvoLeaf : public SvoNode<N, BT> {
private:
    std::array<T, N> data{};

protected:
    void doClear() final;

public:
    SvoLeaf() = default;
    ~SvoLeaf() final = default;

    SvoLeaf(SvoLeaf &&) = default;
    SvoLeaf(const SvoLeaf &) = default;

    SvoLeaf &operator=(SvoLeaf &&) = default;
    SvoLeaf &operator=(const SvoLeaf &) = default;

    SvoLeaf *clone() const final
    {
        return new SvoLeaf{*this};
    }

    SvoNodeType type() const final
    {
        return SvoNodeType::LEAF;
    }

    // custom methods
    const T &at(size_t index) const
    {
        VXIO_DEBUG_ASSERT_LT(index, N);
        VXIO_DEBUG_ASSERT(this->has(index));
        return data[index];
    }

    T &at(size_t index)
    {
        VXIO_DEBUG_ASSERT_LT(index, N);
        VXIO_DEBUG_ASSERT(this->has(index));
        return data[index];
    }

    const T *get_if(size_t index) const
    {
        return this->has(index) ? &data[index] : nullptr;
    }

    T *get_if(size_t index)
    {
        return this->has(index) ? &data[index] : nullptr;
    }

    T &operator[](size_t index)
    {
        this->mask_.set(index);
        return data[index];
    }
};

template <size_t N, typename BT>
void SvoBranch<N, BT>::doClear()
{
    for (auto &child : children) {
        child.release();
    }
}

template <typename T, size_t N, typename BT>
void SvoLeaf<T, N, BT>::doClear()
{
    // TODO destroy elements
}

namespace detail {

/**
 * @brief A two's complement-oriented abs function.
 * This decrements negative values after computing the absolute.
 *
 * The motivation for this is that an SVO can hold one more coordinate in negative direciton than positive, so when
 * deciding whether the SVO needs to be grown for insertion, this is taken into account.
 *
 * Examples: abs_svo(3) = 3, abs_svo(-3) = 2, abs_svo(-1) = 0
 *
 * @param x the input
 * @return x if x is positive, else abs(x) - 1
 */
constexpr uint32_t abs_svo(int32_t x)
{
    // It is crucial that we first add 1, then negate.
    // This prevents signed integer underflow which is ub in C++17-.
    return static_cast<uint32_t>(x < 0 ? -(x + 1) : x);
}

/// Interleaves three single-bit numbers.
constexpr uint64_t ileave3_one(uint32_t x, uint32_t y, uint32_t z)
{
    return (x << 2) | (y << 1) | (z << 0);
}

/// Deinterleaves three single-bit numbers.
constexpr voxelio::Vec3u32 dileave3_one(uint64_t n)
{
    return voxelio::Vec3u64{(n >> 2) & 1, (n >> 1) & 1, (n >> 0) & 1}.cast<uint32_t>();
}

/// Interleaves three two-bit numbers.
constexpr uint64_t ileave3_two(uint32_t x, uint32_t y, uint32_t z)
{
    uint64_t hi = ((x & 2) << 2) | ((y & 2) << 1) | ((z & 2) << 0);
    uint64_t lo = ((x & 1) << 2) | ((y & 1) << 1) | ((z & 1) << 0);
    // normally we would need to shift hi by 3 bits but all bits are already shifted because of their
    // position within each coordinate
    return (hi << 2) | lo;
}

/// Deinterleaves three two-bit numbers.
constexpr voxelio::Vec3u32 dileave3_two(uint64_t n)
{
    uint32_t hi = static_cast<uint32_t>(n >> 3) & 0b111;
    uint32_t lo = static_cast<uint32_t>(n >> 0) & 0b111;
    uint32_t x = ((hi & 4) >> 1) | ((lo & 4) >> 2);
    uint32_t y = ((hi & 2) >> 0) | ((lo & 2) >> 1);
    uint32_t z = ((hi & 1) << 1) | ((lo & 1) >> 0);
    return {x, y, z};
}

}  // namespace detail

template <typename TSparseVoxelOctree>
class depth_first_range_impl;

template <typename T, size_t SQUASH = 0, typename BT = void>
class SparseVoxelOctree {
private:
    using i32 = int32_t;
    using u32 = uint32_t;
    using u64 = uint64_t;

    using Vec3i32 = voxelio::Vec3i32;
    using Vec3u32 = voxelio::Vec3u32;

    template <typename Pointer>
    using uptr = std::unique_ptr<Pointer>;

    /**
     * Returns the unilateral capacity for a given depth.
     * Due to the fact that the SVO extends one coordinate further into negative space, this will be an inclusive
     * limit for negative coordinates and an exclusive limit for positive coordinates.
     *
     * In mathematical terms, we support the insertion of coordinates n, when:
     * -unilateralCapacity() <= n < unilateralCapacity()
     *
     * @param depth the depth
     * @return the unilateral capacity
     */
    static constexpr u32 unilateralCapacity(size_t depth)
    {
        // this effectively calculates:
        // pow(2, d * (squashes + 1) + squashes)
        //
        // for no squash, this calculates:
        // pow(2, d)
        //
        // for one squash, this calculates:
        // pow(4, d) * 2
        return 1 << (depth * (SQUASHES + 1) + SQUASHES);
    }

public:
    /// An exclusive upper bound for coordinates.
    static constexpr i32 COORDINATE_UPPER_LIMIT = 1 << 21;
    /// An inclusive lower bound for coordinates.
    static constexpr i32 COORDINATE_LOWER_LIMIT = -COORDINATE_UPPER_LIMIT;
    /// The amount of bits that one octree node index digit has.
    static constexpr size_t INDEX_DIGIT_BITS = 3 * (SQUASH + 1);
    /// A mask for a single octree node index digit.
    static constexpr size_t INDEX_DIGIT_MASK = (1 << INDEX_DIGIT_BITS) - 1;
    /// The size of a node in all directions. Also equal to cbrt(BRANCHING_FACTOR).
    static constexpr size_t UNILATERAL_NODE_SIZE = 1 << (SQUASH + 1);
    /// The number of SVO layer squashes. Zero for regular SVO with 8 branches.
    static constexpr size_t SQUASHES = SQUASH;
    /// The branching factor. 8 for an unsquashed SVO, 64 for one squash, etc.
    static constexpr size_t BRANCHING_FACTOR = 1 << INDEX_DIGIT_BITS;

    using self_type = SparseVoxelOctree;

    using value_type = T;
    using branch_value_type = BT;

    using node_type = SvoNode<BRANCHING_FACTOR, branch_value_type>;
    using branch_type = SvoBranch<BRANCHING_FACTOR, branch_value_type>;
    using leaf_type = SvoLeaf<T, BRANCHING_FACTOR, branch_value_type>;

    using depth_first_range = depth_first_range_impl<SparseVoxelOctree>;
    using const_depth_first_range = depth_first_range_impl<const SparseVoxelOctree>;

    depth_first_range depthFirstNodeRange()
    {
        return {this};
    }

    const_depth_first_range depthFirstNodeRange() const
    {
        return {this};
    }

private:
    template <typename Consumer>
    static constexpr bool isNodeConsumer = std::is_invocable_r_v<void, Consumer, node_type *, SvoNodeType>;

private:
    uptr<branch_type> root = std::make_unique<branch_type>();
    size_t depth = 1;

public:
    SparseVoxelOctree() = default;
    SparseVoxelOctree(const SparseVoxelOctree &copyOf) : root{copyOf.root->clone()} {}
    SparseVoxelOctree(SparseVoxelOctree &&) = default;

    value_type *getIfExists(const Vec3i32 &pos);
    const value_type *getIfExists(const Vec3i32 &pos) const;

    value_type &at(const Vec3i32 &pos);
    const value_type &at(const Vec3i32 &pos) const;

    /**
     * Removes all nodes from the octree. This is not guaranteed to free all memory associated with the octree.
     */
    void clear();

    bool empty() const
    {
        return root->empty();
    }

    /**
     * Returns true exactly when the octree contains a voxel at the given position.
     * @param pos the position
     * @return true exactly when the octree contains a voxel at the given position
     */
    bool contains(const Vec3i32 &pos) const;

    /**
     * Returns the absolute depth of the SVO.
     * The absolute depth is the number of SVO layers of nodes.
     * Individual voxels do not count as nodes.
     *
     * @return the signed depth
     */
    size_t depthAbsolute() const
    {
        return depth + 1;
    }

    /**
     * Returns the signed depth of the SVO.
     * The signed depth is the absolute depth - 1, because it extends into both positive and negative direction.
     * Thus, one SVO layer is effectively skipped.
     *
     * For example, a 4x4x4 SVO has an absolute depth of 2 and a signed depth of 1.
     * It requires two SVO layers until individual voxels are reached.
     * @return the signed depth
     */
    size_t depthSigned() const
    {
        return depth;
    }

    /**
     * @brief Applies a consumer function to all nodes.
     * The order of traversal is from top to bottom.
     * It is guaranteed that a parent node will be consumed before its child nodes.
     *
     * This function makes many optimizations that are not available to the iterator of depth_first_range, so it should
     * be preferred.
     *
     * @tparam consumer the consumer functor void(node_type*, SvoNodeType)
     */
    template <typename Consumer,
              std::enable_if_t<std::is_invocable_r_v<void, Consumer, node_type *, SvoNodeType>, int> = 0>
    void forEachNodeTopDown(const Consumer &consumer)
    {
        forEachNodeTopDown_impl(consumer);
    }

    /**
     * @brief Applies a consumer function to all nodes.
     * The order of traversal is from bottom to top.
     * It is guaranteed that all child nodes will be consumed before their parent node.
     *
     * This function makes many optimizations that are not available to the iterator of depth_first_range, so it should
     * be preferred.
     *
     * @tparam consumer the consumer functor void(node_type*, SvoNodeType)
     */
    template <typename Consumer,
              std::enable_if_t<std::is_invocable_r_v<void, Consumer, node_type *, SvoNodeType>, int> = 0>
    void forEachNodeBottomUp(const Consumer &consumer)
    {
        forEachNodeBottomUp_impl(consumer);
    }

    /**
     * @brief Converts a position to an index inside the SVO.
     * This index can be used for faster random access in other methods.
     * The index becomes invalid when the SVO is resized.
     * @param pos the position
     * @return the index of the position
     */
    u64 indexOf(Vec3i32 pos) const
    {
        VXIO_DEBUG_ASSERT_EQ(boundsTest(pos), 0);
        Vec3u32 uPos = Vec3i32{pos - minIncl()}.cast<u32>();
        u64 result = voxelio::ileave3(uPos[0], uPos[1], uPos[2]);
        return result;
    }

    /**
     * @brief Converts an index obtained using indexOf(Vec3i32) back to the input position.
     * @param pos the position
     * @return the index of the position
     */
    Vec3i32 indexToPos(u64 index) const
    {
        Vec3u32 uPos;
        voxelio::dileave3(index, uPos.data());
        return uPos.cast<i32>() + minIncl();
    }

    /**
     * @brief Inserts a value into the SVO.
     * If necessary, the SVO will be resized to fit the position.
     * @param pos the position at which to insert
     * @param value the value to insert
     */
    void insert(Vec3i32 pos, value_type value)
    {
        reserve(pos);
        insert_unsafe(indexOf(pos), std::move(value));
    }

    /// Returns the minimum coordinates that the SVO can currently contain.
    Vec3i32 minIncl() const
    {
        return Vec3i32::filledWith(-unilateralCapacity());
    }

    /// Returns the maximum coordinates that the SVO can currently contain.
    Vec3i32 maxIncl() const
    {
        return Vec3i32::filledWith(unilateralCapacity() - 1);
    }

    /// Returns coordinates that are lower in every dimension than any point the SVO can currently contain.
    /// Inserting a voxel into the SVO at this location will resize the SVO exactly once.
    Vec3i32 minExcl() const
    {
        return Vec3i32::filledWith(-unilateralCapacity() - 1);
    }

    /// Returns coordinates that are greater in every dimension than any point the SVO can currently contain.
    /// Inserting a voxel into the SVO at this location will resize the SVO exactly once.
    Vec3i32 maxExcl() const
    {
        return Vec3i32::filledWith(unilateralCapacity());
    }

    /**
     * @brief Ensures that the SVO has enough space to contain a voxel at the position which is to be inserted.
     * @param pos the position
     */
    void reserve(const Vec3i32 &pos)
    {
        if (u32 lim = boundsTest(pos); lim != 0) {
            reserve(lim);
        }
    }

    /**
     * @brief Ensures that the SVO has enough space to contain a coordinate which is to be inserted.
     * @param max the coordinate (inclusive)
     */
    void reserve(u32 max);

    branch_type &rootNode()
    {
        return *root;
    }

    const branch_type &rootNode() const
    {
        return *root;
    }

    SparseVoxelOctree &operator=(const SparseVoxelOctree &copyOf)
    {
        root.reset(copyOf.root->clone());
        depth = copyOf.depth;
        return *this;
    }

    SparseVoxelOctree &operator=(SparseVoxelOctree &&) = default;

    value_type &operator[](const Vec3i32 &pos)
    {
        reserve(pos);
        return findValueOrCreate_unsafe(indexOf(pos));
    }

private:
    /**
     * @brief Returns the unilateral capacity of the SVO.
     * Due to the fact that the SVO extends one coordinate further into negative space, this will be an inclusive
     * limit for negative coordinates and an exclusive limit for positive coordinates.
     *
     * In mathematical terms, we support the insertion of coordinates n, when:
     * -unilateralCapacity() <= n < unilateralCapacity()
     *
     * @return the unilateral capacity
     */
    u32 unilateralCapacity() const
    {
        return unilateralCapacity(depth);
    }

    /**
     * @brief Tests whether the input vector lies within this octree. The result is an unsigned integer which indicates
     * how much the octree has to be enlarged to fit the vector. The dimensions of the octree in all directions need to
     * be > this integer.
     * @param v the input position
     * @return 0 if the test passes or a maximum-like coordinate if the test fails
     */
    uint32_t boundsTest(const Vec3i32 &v) const
    {
        u32 max = detail::abs_svo(v[0]) | detail::abs_svo(v[1]) | detail::abs_svo(v[2]);
        VXIO_DEBUG_ASSERT_LT(max, COORDINATE_UPPER_LIMIT);
        return max >= unilateralCapacity() ? max : 0;
    }

    value_type &findValueOrCreate_unsafe(u64 octreeNodeIndex);
    node_type *findNode_unsafe_impl(u64 octreeNodeIndex) const;
    value_type *findIfExists_unsafe_impl(u64 octreeNodeIndex) const;

    node_type *findNode_unsafe(u64 octreeNodeIndex)
    {
        return findNode_unsafe_impl(octreeNodeIndex);
    }

    const node_type *findNode_unsafe(u64 octreeNodeIndex) const
    {
        return findNode_unsafe_impl(octreeNodeIndex);
    }

    value_type *findValue_unsafe(u64 octreeNodeIndex)
    {
        return findIfExists_unsafe_impl(octreeNodeIndex);
    }

    const value_type *findValue_unsafe(u64 octreeNodeIndex) const
    {
        return findIfExists_unsafe_impl(octreeNodeIndex);
    }

    /**
     * @brief Grows the SVO unilaterally once.
     * This effectively doubles each SVO dimension and increases the depth by one.
     */
    void growOnce();

    /**
     * @brief Inserts a color at the given index. This assumes that space has already been allocated.
     * @param octreeNodeIndex the index at which to insert
     * @param color the color to be inserted
     */
    void insert_unsafe(u64 octreeNodeIndex, value_type color)
    {
        findValueOrCreate_unsafe(octreeNodeIndex) = std::move(color);
    }

    template <typename Consumer>
    void forEachNodeTopDown_impl(const Consumer &consumer);

    template <typename Consumer>
    void forEachNodeBottomUp_impl(const Consumer &consumer);
};

// VOXEL INSERTION/ACCESS/SEARCH =======================================================================================

template <typename T, size_t SQUASH, typename BT>
T &SparseVoxelOctree<T, SQUASH, BT>::findValueOrCreate_unsafe(u64 octreeNodeIndex)
{
    node_type *node = root.get();
    for (size_t s = depth * INDEX_DIGIT_BITS; s != size_t(-INDEX_DIGIT_BITS); s -= INDEX_DIGIT_BITS) {
        u32 octDigit = (octreeNodeIndex >> s) & INDEX_DIGIT_MASK;
        if (s != 0) {
            auto *branch = voxelio::downcast<branch_type *>(node);
            if (branch->has(octDigit)) {
                node = branch->child(octDigit);
            }
            else {
                auto *child = s == INDEX_DIGIT_BITS ? static_cast<node_type *>(new leaf_type)
                                                    : static_cast<node_type *>(new branch_type);
                node = child;
                branch->insert(octDigit, uptr<node_type>{child});
            }
        }
        else {
            auto *leaf = voxelio::downcast<leaf_type *>(node);
            return (*leaf)[octDigit];
        }
    }
    VXIO_DEBUG_ASSERT_UNREACHABLE();
}

template <typename T, size_t SQUASH, typename BranchT>
typename SparseVoxelOctree<T, SQUASH, BranchT>::node_type *SparseVoxelOctree<T, SQUASH, BranchT>::findNode_unsafe_impl(
    u64 octreeNodeIndex) const
{
    node_type *node = root.get();
    for (size_t s = depth * INDEX_DIGIT_BITS;; s -= INDEX_DIGIT_BITS) {
        if (s == 0) {
            return node;
        }

        u32 octDigit = (octreeNodeIndex >> s) & INDEX_DIGIT_MASK;
        auto *branch = voxelio::downcast<branch_type *>(node);
        if (branch->has(octDigit)) {
            node = branch->child(octDigit);
        }
        else {
            return nullptr;
        }
    }
    VXIO_DEBUG_ASSERT_UNREACHABLE();
}

template <typename T, size_t SQUASH, typename BranchT>
T *SparseVoxelOctree<T, SQUASH, BranchT>::findIfExists_unsafe_impl(u64 octreeNodeIndex) const
{
    node_type *node = root.get();
    for (size_t s = depth * INDEX_DIGIT_BITS; s != size_t(-INDEX_DIGIT_BITS); s -= INDEX_DIGIT_BITS) {
        u32 octDigit = (octreeNodeIndex >> s) & INDEX_DIGIT_MASK;
        if (s != 0) {
            auto *branch = voxelio::downcast<branch_type *>(node);
            if (not branch->has(octDigit)) {
                return nullptr;
            }
            else {
                node = branch->child(octDigit);
            }
        }
        else {
            auto *leaf = voxelio::downcast<leaf_type *>(node);
            return leaf->get_if(octDigit);
        }
    }
    VXIO_DEBUG_ASSERT_UNREACHABLE();
}

template <typename T, size_t SQUASH, typename BranchT>
T *SparseVoxelOctree<T, SQUASH, BranchT>::getIfExists(const Vec3i32 &pos)
{
    if (u32 lim = boundsTest(pos); lim != 0) {
        return nullptr;
    }
    return findValue_unsafe(indexOf(pos));
}

template <typename T, size_t SQUASH, typename BranchT>
const T *SparseVoxelOctree<T, SQUASH, BranchT>::getIfExists(const Vec3i32 &pos) const
{
    if (u32 lim = boundsTest(pos); lim != 0) {
        return nullptr;
    }
    return findValue_unsafe(indexOf(pos));
}

template <typename T, size_t SQUASH, typename BranchT>
T &SparseVoxelOctree<T, SQUASH, BranchT>::at(const Vec3i32 &pos)
{
    auto *result = getIfExists(pos);
    VXIO_ASSERT_NOTNULL(result);
    return *result;
}

template <typename T, size_t SQUASH, typename BranchT>
const T &SparseVoxelOctree<T, SQUASH, BranchT>::at(const Vec3i32 &pos) const
{
    auto *result = getIfExists(pos);
    VXIO_ASSERT_NOTNULL(result);
    return *result;
}

template <typename T, size_t SQUASH, typename BranchT>
bool SparseVoxelOctree<T, SQUASH, BranchT>::contains(const Vec3i32 &pos) const
{
    if (boundsTest(pos) != 0) {
        return false;
    }
    return findValue_unsafe(indexOf(pos)) != nullptr;
}

// STRUCTURE MANAGEMENT ================================================================================================

template <typename T, size_t SQUASH, typename BranchT>
void SparseVoxelOctree<T, SQUASH, BranchT>::clear()
{
    root->clear();
}

template <typename T, size_t SQUASH, typename BranchT>
void SparseVoxelOctree<T, SQUASH, BranchT>::reserve(u32 lim)
{
    while (lim >= unilateralCapacity()) {
        growOnce();
    }
}

template <typename T, size_t SQUASH, typename BranchT>
void SparseVoxelOctree<T, SQUASH, BranchT>::growOnce()
{
    VXIO_ASSERT_LT(unilateralCapacity(depth + 1), COORDINATE_UPPER_LIMIT);
    // Special case for unsquashed octrees.
    // Unsquashed growth is much simpler because each node receives exactly one new parent.
    // Within a parent, nodes move into the opposite corner.
    if constexpr (SQUASH == 0) {
        for (size_t i = 0; i < 8; ++i) {
            if (not root->has(i)) {
                continue;
            }
            auto parent = std::make_unique<branch_type>();
            parent->insert(~i & INDEX_DIGIT_MASK, root->extract(i));
            root->insert(i, std::move(parent));
        }
    }
    // Regular case for squashed octrees.
    // tl;dr: we use the more significant digit i to get the position in root, j becomes position in new parent
    else {
        constexpr size_t subBranchingFactor = BRANCHING_FACTOR >> 3;
        constexpr size_t subIndexDigitBits = INDEX_DIGIT_BITS - 3;
        constexpr size_t subIndexDigitMask = (1 << subIndexDigitBits) - 1;
        constexpr uint32_t unilateralMax = UNILATERAL_NODE_SIZE - 1;
        constexpr uint32_t unilateralCenter = unilateralMax / 2;

        constexpr auto calcParentIndexInRoot = [](size_t i) -> size_t {
            // We deinterleave the most significant digit and add the center onto it.
            // This gives us a 2x2x2 volume around the center, where all parents are created.
            const u32 x = ((i >> 2) & 1) + unilateralCenter;
            const u32 y = ((i >> 1) & 1) + unilateralCenter;
            const u32 z = ((i >> 0) & 1) + unilateralCenter;
            if constexpr (SQUASHES == 1) {
                return detail::ileave3_two(x, y, z);
            }
            else {
                return voxelio::ileave3(x, y, z);
            }
        };

        // Yes, using 8 instead of the branching factor is intentional.
        // There can only be 8 new parents, even when squashing layers.
        for (size_t i = 0; i < 8; ++i) {
            // i becomes the most significant digit of the octree node index.
            const size_t not_i = ~i & 0b111;
            const size_t childIndexInRoot_hi = i << subIndexDigitBits;
            const size_t childIndexInParent_hi = not_i << subIndexDigitBits;
            const size_t parentIndexInRoot = calcParentIndexInRoot(i);

            // Multiple branches can go into the same parent.
            // Namely, all branches in the current i-iteration go into this parent.
            uptr<branch_type> parent = nullptr;

            for (size_t j = 0; j < subBranchingFactor; ++j) {
                // j becomes the concatenation of all less significant digits.
                const size_t childIndexInRoot = childIndexInRoot_hi | j;
                if (not root->has(childIndexInRoot)) {
                    continue;
                }

                const size_t childIndexInParent = childIndexInParent_hi | j;
                if (parent != nullptr) {
                    parent->insert(childIndexInParent, root->extract(childIndexInRoot));
                    continue;
                }
                parent = std::make_unique<branch_type>();
                parent->insert(childIndexInParent, root->extract(childIndexInRoot));

                // Because we are editing root's children in-place, we run danger of replacing an existing value.
                // Fortunately, all such children are guaranteed to belong into our new parent anyways.
                // So we just need to add it to our parent as well in such a case.
                if (root->has(parentIndexInRoot)) {
                    // TODO investigate if this really ever happens
                    size_t lo = parentIndexInRoot & subIndexDigitMask;
                    size_t obstructingChildIndexInParent = childIndexInParent_hi | lo;
                    VXIO_DEBUG_ASSERT(not parent->has(obstructingChildIndexInParent));
                    parent->insert(obstructingChildIndexInParent, root->extract(parentIndexInRoot));
                }

            }  // for j

            if (parent != nullptr) {
                // Very important:
                // We can not do this inside of the j-loop right after creating the parent.
                // Otherwise we may run into the same value again during the j-loop.
                root->insert(parentIndexInRoot, std::move(parent));
            }

        }  // for i
    }      // else constexpr
    depth += 1;
}

// ITERATORS ===========================================================================================================

template <typename TSparseVoxelOctree>
class depth_first_range_impl {
public:
    struct iterator;

private:
    TSparseVoxelOctree *parent;

public:
    depth_first_range_impl(TSparseVoxelOctree *parent) noexcept : parent{parent}
    {
        VXIO_DEBUG_ASSERT_NOTNULL(parent);
    }
    depth_first_range_impl(depth_first_range_impl &&) = default;
    depth_first_range_impl(const depth_first_range_impl &) = default;

    depth_first_range_impl &operator=(depth_first_range_impl &&) = default;
    depth_first_range_impl &operator=(const depth_first_range_impl &) = default;

    iterator begin() const noexcept
    {
        return {parent};
    }

    iterator end() const noexcept
    {
        return {nullptr};
    }
};

template <typename TSparseVoxelOctree>
struct depth_first_range_impl<TSparseVoxelOctree>::iterator {
private:
    static constexpr size_t INDEX_DIGIT_BITS = TSparseVoxelOctree::INDEX_DIGIT_BITS;
    static constexpr size_t BRANCHING_FACTOR = TSparseVoxelOctree::BRANCHING_FACTOR;

    using node_type = typename TSparseVoxelOctree::node_type;
    using branch_type = typename TSparseVoxelOctree::branch_type;
    using leaf_type = typename TSparseVoxelOctree::leaf_type;

    struct Pos {
        node_type *node;
        size_t index;
    };

    TSparseVoxelOctree *parent;
    std::vector<Pos> positionStack;
    uint64_t morton = 0;

public:
    static constexpr bool is_const = std::is_const_v<TSparseVoxelOctree>;

    using self_type = iterator;
    using difference_type = uint64_t;
    using value_type = std::conditional_t<is_const, node_type, node_type> *;
    using reference = value_type &;
    using pointer = value_type *;
    using iterator_category = std::forward_iterator_tag;

    iterator(TSparseVoxelOctree *parent) noexcept : parent{parent}
    {
        if (parent != nullptr) {
            positionStack.push_back({&parent->rootNode(), 0});
        }
    }

    iterator(iterator &&) = default;
    iterator(const iterator &) = default;

    iterator &operator=(iterator &&) = default;
    iterator &operator=(const iterator &) = default;

    /**
     * @brief Returns the current base index of the iterator.
     * This result is only geometrically meaningful for leaf nodes.
     * For branch nodes, it can be used to test for equality.
     *
     * Base index means the minimum index of all voxels inside of the leaf node.
     *
     * Calling this for an iterator which has reached the end is undefined behavior.
     * @return the current base index
     */
    uint64_t index() const noexcept
    {
        VXIO_DEBUG_ASSERTM(not positionStack.empty(), "Can't call index() for an iterator which is empty");
        return morton << INDEX_DIGIT_BITS;
    }

    /**
     * @brief Returns the current node at which the iterator resides.
     * Calling this for an iterator which has reached the end is undefined behavior.
     * @return the current node
     */
    value_type node() const noexcept
    {
        VXIO_DEBUG_ASSERTM(not positionStack.empty(), "Can't call node() for an iterator which is empty");
        return positionStack.back().node;
    }

    /**
     * @brief Returns the type of node at which the iterator resides.
     * Calling this for an iterator which has reached the end is undefined behavior.
     * @return the current node type
     */
    SvoNodeType nodeType() const noexcept
    {
        VXIO_DEBUG_ASSERTM(not positionStack.empty(), "Can't call nodeType() for an iterator which is empty");
        // We make a decision based only on the stack size.
        // This saves a virtual call to get the node type from the top of the stack.
        SvoNodeType result = positionStack.size() == parent->depthAbsolute() ? SvoNodeType::LEAF : SvoNodeType::BRANCH;
        VXIO_DEBUG_ASSERT_EQ(result, positionStack.back().node->type());
        return result;
    }

    /// Returns true exactly when the iterator is at a leaf node.
    /// Calling this for an iterator which has reached the end is undefined behavior.
    bool isAtLeaf() noexcept
    {
        return nodeType() == SvoNodeType::LEAF;
    }

    /// Returns true exactly when the iterator is at a branch node.
    /// Calling this for an iterator which has reached the end is undefined behavior.
    bool isAtBranch() noexcept
    {
        return nodeType() == SvoNodeType::BRANCH;
    }

    /**
     * @brief Returns the current base position of the node of the iterator in 3D-space.
     * This result is only meaningful for leaf nodes.
     *
     * Base position means the minimum position of all voxels inside of the leaf node.
     *
     * Calling this for an iterator which has reached the end is undefined behavior.
     * @return the current base position in 3D-space
     */
    voxelio::Vec3i32 pos() const noexcept
    {
        return parent->indexToPos(index());
    }

    /**
     * @brief Tells the iterator to go deeper into the SVO.
     * This is the behavior of operator++().
     *
     * If it is not possible to go any deeper, the next subbranch of the parent is visited.
     * This decision is made recurively.
     *
     * Calling this for an iterator which has reached the end is undefined behavior.
     */
    void goDeeper() noexcept
    {
        traverse(false);
    }

    /**
     * @brief Tells the iterator to abandon the current branch and visit the next branch of the parent.
     *
     * If it is not possible to go any deeper starting from the parent, the next subbranch of its parent is visited.
     * This decision is made recurively.
     *
     * Going sideways is equivalent to going up one level in the tree and then going deeper.
     *
     * Calling this for an iterator which has reached the end is undefined behavior.
     */
    void goSideways() noexcept
    {
        traverse(true);
    }

    self_type &operator++() noexcept
    {
        goDeeper();
        return *this;
    }

    bool operator==(const self_type &other) const noexcept
    {
        return this->positionStack.size() == other.positionStack.size() && this->morton == other.morton;
    }

    bool operator!=(const self_type &other) const noexcept
    {
        return not operator==(other);
    }

    value_type operator*() noexcept
    {
        return node();
    }

private:
    void traverse(bool forceOnePop) noexcept;

    bool handleReachedEndOfNodeAndSignalRootPop() noexcept;
};

template <typename TSparseVoxelOctree>
void depth_first_range_impl<TSparseVoxelOctree>::iterator::traverse(bool forceOnePop) noexcept
{
    VXIO_DEBUG_ASSERTM(not positionStack.empty(), "Called traverse() on empty iterator");
    VXIO_DEBUG_ASSERT_NOTNULL(positionStack.back().node);

    if (forceOnePop || isAtLeaf()) {
        if (handleReachedEndOfNodeAndSignalRootPop()) {
            return;
        }
    }
    VXIO_DEBUG_ASSERT_NOTNULL(positionStack.back().node);

    do {
        Pos &pos = positionStack.back();
        auto *branch = voxelio::downcast<branch_type *>(pos.node);
        for (; pos.index < BRANCHING_FACTOR; ++pos.index) {
            node_type *child = branch->child(pos.index);
            if (child != nullptr) {
                morton <<= INDEX_DIGIT_BITS;
                morton |= pos.index;
                positionStack.push_back({child, 0});
                return;
            }
        }
        if (handleReachedEndOfNodeAndSignalRootPop()) {
            return;
        }
    } while (true);
}

template <typename TSparseVoxelOctree>
bool depth_first_range_impl<TSparseVoxelOctree>::iterator::handleReachedEndOfNodeAndSignalRootPop() noexcept
{
    VXIO_DEBUG_ASSERT(not positionStack.empty());
    do {
        positionStack.pop_back();
        morton >>= INDEX_DIGIT_BITS;
        if (positionStack.empty()) {
            return true;
        }
        if (++positionStack.back().index < BRANCHING_FACTOR) {
            return false;
        }
    } while (true);
}

// FUNCTIONAL ==========================================================================================================

template <typename T, size_t SQUASH, typename BT>
template <typename Consumer>
void SparseVoxelOctree<T, SQUASH, BT>::forEachNodeTopDown_impl(const Consumer &consumer)
{
    static_assert(isNodeConsumer<Consumer>);

    struct Pos {
        branch_type *branch;
        size_t index = 0;
    };

    std::stack<Pos> stack;
    stack.push({root.get(), 0});
    consumer(root.get(), SvoNodeType::BRANCH);

    while (not stack.empty()) {
        Pos &pos = stack.top();

        if (stack.size() == depth) {  // one layer above leafs
            for (; pos.index < BRANCHING_FACTOR; ++pos.index) {
                auto *child = voxelio::downcast<leaf_type *>(pos.branch->child(pos.index));
                if (child != nullptr) {
                    consumer(child, SvoNodeType::LEAF);
                }
            }
        }
        else {
            for (; pos.index < BRANCHING_FACTOR; ++pos.index) {
                auto *child = voxelio::downcast<branch_type *>(pos.branch->child(pos.index));
                if (child != nullptr) {
                    stack.push({child, 0});
                    consumer(child, SvoNodeType::BRANCH);
                    goto end_of_while;
                }
            }
        }

        do {
            stack.pop();
            if (stack.empty()) {
                return;
            }
            Pos &pos = stack.top();
            if (++pos.index < BRANCHING_FACTOR) {
                break;
            }
        } while (true);

    end_of_while:;
    }
}

template <typename T, size_t SQUASH, typename BT>
template <typename Consumer>
void SparseVoxelOctree<T, SQUASH, BT>::forEachNodeBottomUp_impl(const Consumer &consumer)
{
    static_assert(isNodeConsumer<Consumer>);

    struct Pos {
        node_type *node;
        size_t index = 0;
    };

    std::stack<Pos> stack;
    stack.push({root.get(), 0});

    while (not stack.empty()) {
        Pos &topPos = stack.top();
        node_type *top = topPos.node;
        SvoNodeType type = stack.size() > depth ? SvoNodeType::LEAF : SvoNodeType::BRANCH;

        if (type == SvoNodeType::BRANCH) {
            for (; topPos.index < BRANCHING_FACTOR; ++topPos.index) {
                node_type *child = voxelio::downcast<branch_type *>(topPos.node)->child(topPos.index);
                if (child != nullptr) {
                    stack.push({child, 0});
                    goto end_of_while;
                }
            }
        }

        do {
            consumer(top, type);

            stack.pop();
            if (stack.empty()) {
                return;
            }
            Pos &pos = stack.top();
            if (++pos.index < BRANCHING_FACTOR) {
                break;
            }
            top = pos.node;
            type = SvoNodeType::BRANCH;
        } while (true);

    end_of_while:;
    }
}

// FUNCTIONAL EXTERNAL FUNCTIONS =======================================================================================

/**
 * @brief Invokes a void(leaf_type*) consumer function for every leaf node in the SVO.
 */
template <
    typename Consumer,
    typename T,
    size_t SQUASH = 0,
    typename BT = void,
    std::enable_if_t<std::is_invocable_r_v<void, Consumer, typename SparseVoxelOctree<T, SQUASH, BT>::leaf_type *>,
                     int> = 0>
void forEachLeaf(SparseVoxelOctree<T, SQUASH, BT> &svo, const Consumer &consumer)
{
    using svo_type = std::remove_reference_t<decltype(svo)>;

    svo.forEachNodeTopDown([&consumer](typename svo_type::node_type *node, SvoNodeType type) {
        if (type == SvoNodeType::LEAF) {
            consumer(voxelio::downcast<typename svo_type::leaf_type *>(node));
        }
    });
}

/**
 * @brief Invokes a void(branch_type*) consumer function for every branch node in the SVO.
 */
template <
    typename Consumer,
    typename T,
    size_t SQUASH = 0,
    typename BT = void,
    std::enable_if_t<std::is_invocable_r_v<void, Consumer, typename SparseVoxelOctree<T, SQUASH, BT>::branch_type *>,
                     int> = 0>
void forEachBranch(SparseVoxelOctree<T, SQUASH, BT> &svo, const Consumer &consumer)
{
    using svo_type = std::remove_reference_t<decltype(svo)>;

    svo.forEachNodeTopDown([&consumer](typename svo_type::node_type *node, SvoNodeType type) {
        if (type == SvoNodeType::BRANCH) {
            consumer(voxelio::downcast<typename svo_type::branch_type *>(node));
        }
    });
}

/**
 * @brief Invokes a void(value_type&) consumer function for every voxel in the svo.
 */
template <typename Consumer,
          typename T,
          size_t SQUASH,
          typename BT,
          std::enable_if_t<std::is_invocable_r_v<void, Consumer, T &>, int> = 0>
void forEachValue(SparseVoxelOctree<T, SQUASH, BT> &svo, const Consumer &consumer)
{
    using svo_type = std::remove_reference_t<decltype(svo)>;
    using leaf_type = typename svo_type::leaf_type;
    using value_type = typename svo_type::value_type;

    forEachLeaf([&consumer](leaf_type *leaf) -> void {
        for (size_t i = 0; i < svo_type::BRANCHING_FACTOR; ++i) {
            value_type *value = leaf->at(i);
            if (value != nullptr) {
                consumer(*value);
            }
        }
    });
}

/**
 * @brief Invokes a void(Vec3i32, value_type&) consumer function for every voxel in the svo.
 */
template <typename Consumer,
          typename T,
          size_t SQUASH,
          typename BT,
          std::enable_if_t<std::is_invocable_r_v<void, Consumer, voxelio::Vec3i32, T &>, int> = 0>
void forEachVoxel(SparseVoxelOctree<T, SQUASH, BT> &svo, const Consumer &consumer)
{
    using svo_type = std::remove_reference_t<decltype(svo)>;
    using leaf_type = typename svo_type::leaf_type;

    constexpr size_t loopLimit = svo_type::UNILATERAL_NODE_SIZE;

    constexpr auto indexOf_singleDigit = [](uint32_t x, uint32_t y, uint32_t z) -> uint64_t {
        if constexpr (SQUASH == 0) {
            return detail::ileave3_one(x, y, z);
        }
        else if constexpr (SQUASH == 1) {
            return detail::ileave3_two(x, y, z);
        }
        else {
            return voxelio::ileave3(x, y, z);
        }
    };

    // We need to use iterators instead of forEachLeaf to easily obtain the base 3D position of a leaf.
    auto range = svo.depthFirstNodeRange();
    auto end = range.end();
    for (auto iter = range.begin(); iter != end; ++iter) {
        if (iter.isAtBranch()) {
            continue;
        }
        auto *leaf = voxelio::downcast<leaf_type *>(iter.node());
        voxelio::Vec3i32 basePos = iter.pos();
        for (uint32_t x = 0; x < loopLimit; ++x) {
            for (uint32_t y = 0; y < loopLimit; ++y) {
                for (uint32_t z = 0; z < loopLimit; ++z) {
                    uint32_t index = indexOf_singleDigit(x, y, z);
                    if (leaf->has(index)) {
                        voxelio::Vec3u32 upos = {x, y, z};
                        consumer(basePos + upos.cast<int32_t>(), leaf->at(index));
                    }
                }
            }
        }
    }
}

namespace detail {

template <bool EXPAND,
          typename Reduction,
          typename T,
          size_t SQUASH,
          typename BT,
          std::enable_if_t<std::is_invocable_r_v<void,
                                                 Reduction,
                                                 typename SparseVoxelOctree<T, SQUASH, BT>::node_type *[],
                                                 size_t,
                                                 typename SparseVoxelOctree<T, SQUASH, BT>::branch_type &>,
                           int> = 0>
void reduceOrExpandNodes(SparseVoxelOctree<T, SQUASH, BT> &svo, const Reduction &reduction)
{
    using svo_type = std::remove_reference_t<decltype(svo)>;
    using node_type = typename svo_type::node_type;
    using branch_type = typename svo_type::branch_type;

    node_type *buffer[svo_type::BRANCHING_FACTOR];

    const auto consumer = [&buffer, &reduction](node_type *node, SvoNodeType type) -> void {
        if (type == SvoNodeType::LEAF) {
            return;
        }
        branch_type *oldBranch = voxelio::downcast<branch_type *>(node);
        size_t oldCount = 0;
        for (size_t i = 0; i < svo_type::BRANCHING_FACTOR; ++i) {
            if (oldBranch->has(i)) {
                buffer[oldCount++] = oldBranch->child(i);
            }
        }

        reduction(buffer, oldCount, *oldBranch);
    };
    if constexpr (EXPAND) {
        svo.forEachNodeTopDown(consumer);
    }
    else {
        svo.forEachNodeBottomUp(consumer);
    }
}

}  // namespace detail

template <typename Reduction,
          typename T,
          size_t SQUASH,
          typename BT,
          std::enable_if_t<std::is_invocable_r_v<void,
                                                 Reduction,
                                                 typename SparseVoxelOctree<T, SQUASH, BT>::node_type *[],
                                                 size_t,
                                                 typename SparseVoxelOctree<T, SQUASH, BT>::branch_type &>,
                           int> = 0>
void reduceNodes(SparseVoxelOctree<T, SQUASH, BT> &svo, const Reduction &reduction)
{
    return detail::reduceOrExpandNodes<false>(svo, reduction);
}

template <typename Reduction,
          typename T,
          size_t SQUASH,
          typename BT,
          std::enable_if_t<std::is_invocable_r_v<void,
                                                 Reduction,
                                                 typename SparseVoxelOctree<T, SQUASH, BT>::node_type *[],
                                                 size_t,
                                                 typename SparseVoxelOctree<T, SQUASH, BT>::branch_type &>,
                           int> = 0>
void expandNodes(SparseVoxelOctree<T, SQUASH, BT> &svo, const Reduction &reduction)
{
    return detail::reduceOrExpandNodes<true>(svo, reduction);
}

template <typename Reduction,
          typename T,
          size_t SQUASH,
          typename BT,
          std::enable_if_t<std::is_invocable_r_v<typename SparseVoxelOctree<T, SQUASH, BT>::branch_value_type,
                                                 Reduction,
                                                 typename SparseVoxelOctree<T, SQUASH, BT>::branch_value_type[],
                                                 size_t>,
                           int> = 0>
void reduceNodeValues(SparseVoxelOctree<T, SQUASH, BT> &svo, const Reduction &reduction)
{
    using svo_type = SparseVoxelOctree<T, SQUASH, BT>;
    using node_type = typename svo_type::node_type;
    using branch_type = typename svo_type::branch_type;
    using branch_value_type = typename svo_type::branch_value_type;

    branch_value_type buffer[svo_type::BRANCHING_FACTOR];

    reduceNodes(svo, [&buffer, &reduction](node_type *children[], size_t count, branch_type &parent) {
        for (size_t i = 0; i < count; ++i) {
            buffer[i] = children[i]->value();
            branch_value_type result = reduction(buffer, count);
            parent.value() = std::move(result);
        }
    });
}

}  // namespace mve

#endif  // SVO_HPP
