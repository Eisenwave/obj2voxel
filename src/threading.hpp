#ifndef OBJ2VOXEL_THREADING_HPP
#define OBJ2VOXEL_THREADING_HPP

#include "ringbuffer.hpp"

#include <condition_variable>
#include <mutex>

namespace obj2voxel::async {

/**
 * @brief An asynchronous implementation of a ring buffer.
 * This allows multiple threads to push and pop elements from/to the ring buffer.
 *
 * In the cases where a pop() or push() would fail for a regular ring buffer, this implementation blocks the executing
 * thread until the operation can be completed.
 */
template <typename T, std::size_t N>
class RingBuffer {
    obj2voxel::RingBuffer<T, N> buffer{};

    mutable std::mutex mutex{};
    std::condition_variable readCon{}, writeCon{};

public:
    /// Pops one element from the ring buffer.
    /// If the buffer is empty, the thread will be blocked until an element is available.
    T pop() noexcept
    {
        std::unique_lock<std::mutex> lock{mutex};
        readCon.wait(lock, [this] {
            return not buffer.empty();
        });
        T result = buffer.pop();
        writeCon.notify_one();
        return result;
    }

    /// Pushes one element to the ring buffer.
    /// If the buffer is full, the thread will be blocked until space is available.
    void push(T value) noexcept
    {
        std::unique_lock<std::mutex> lock{mutex};
        writeCon.wait(lock, [this] {
            return not buffer.full();
        });
        buffer.push(std::move(value));
        readCon.notify_one();
    }

    /// Tries to pop one element from the ring buffer.
    /// If the buffer is empty, false will be returned without waiting for a pushed element.
    /// Otherwise, the result will written to out and true is returned.
    bool tryPop(T &out) noexcept
    {
        std::lock_guard<std::mutex> lock{mutex};
        if (buffer.empty()) {
            return false;
        }
        out = buffer.pop();
        writeCon.notify_one();
        return true;
    }

    /// Clears the ring buffer.
    void clear() noexcept
    {
        std::lock_guard<std::mutex> lock{mutex};
        buffer.clear();
        writeCon.notify_all();
    }

    /// Returns the least recently pushed element without popping it.
    /// If the buffer is empty, the thread will be blocked until an element is available.
    const T &peek() const noexcept
    {
        std::lock_guard<std::mutex> lock{mutex};
        readCon.wait(lock, [this] {
            return not buffer.empty();
        });
        return buffer.peek();
    }

    /// Returns the current size thread-safely.
    std::size_t size() const noexcept
    {
        std::lock_guard<std::mutex> lock{mutex};
        return buffer.size();
    }

    /// Returns true if the buffer is empty thread-safely.
    bool empty() const noexcept
    {
        std::lock_guard<std::mutex> lock{mutex};
        return buffer.empty();
    }

    /// Returns true if the buffer is full thread-safely.
    bool full() const noexcept
    {
        std::lock_guard<std::mutex> lock{mutex};
        return buffer.full();
    }
};

/// A simple extension of conidition variables that allows waiting until a certain event is triggered.
/// A thread that should wait for the event must call wait().
/// Any other thread can then trigger the event with trigger(), which unblocks all waiting threads.
/// Unlike with condition variables, triggering is permanent, so an event that has once been triggered must be reset().
class Event {
    bool flag = false;
    mutable std::mutex mutex;
    mutable std::condition_variable condition;

public:
    explicit Event(bool triggered = false) noexcept : flag{triggered} {}

    bool wait() noexcept
    {
        std::unique_lock<std::mutex> lock{mutex};
        if (flag) {
            return false;
        }
        condition.wait(lock);
        return true;
    }

    void trigger() noexcept
    {
        std::lock_guard<std::mutex> lock{mutex};
        flag = true;
        condition.notify_all();
    }

    void reset() noexcept
    {
        std::lock_guard<std::mutex> lock{mutex};
        flag = false;
    }
};

/// An atomic counter which allows waiting until a certain value has been reached.
template <typename T = std::uintmax_t>
class Counter {
public:
    using type = T;

private:
    T count;
    mutable std::mutex mutex;
    mutable std::condition_variable condition;

public:
    Counter(T count = {}) noexcept : count{count} {}

    /// Increments the counter atomically.
    Counter &operator++() noexcept
    {
        std::lock_guard<std::mutex> lock{mutex};
        ++count;
        condition.notify_all();
        return *this;
    }

    /// Decrements the counter atomically.
    Counter &operator--() noexcept
    {
        std::lock_guard<std::mutex> lock{mutex};
        --count;
        condition.notify_all();
        return *this;
    }

    template <typename Predicate, std::enable_if_t<std::is_invocable_r_v<bool, Predicate, type>, int> = 0>
    void wait(Predicate predicate) const noexcept
    {
        std::unique_lock<std::mutex> lock{mutex};
        if (predicate(count)) {
            return;
        }
        condition.wait(lock, [this, &predicate]() -> bool {
            return predicate(count);
        });
    }

    void waitUntilZero() const noexcept
    {
        wait([](type t) {
            return t == 0;
        });
    }

    const type &operator*() const noexcept
    {
        std::lock_guard<std::mutex> lock{mutex};
        return count;
    }
};

}  // namespace obj2voxel::async

#endif  // OBJ2VOXEL_THREADING_HPP
