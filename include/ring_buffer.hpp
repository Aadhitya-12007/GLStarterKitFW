#pragma once

#include <array>
#include <cstddef>

template <typename T, std::size_t Capacity>
class RingBuffer {
public:
    void clear() noexcept {
        head_  = 0U;
        count_ = 0U;
    }

    // Overwrites the oldest item once full. O(1) time, zero heap.
    void push(const T& item) noexcept {
        data_[head_] = item;
        head_ = (head_ + 1U) % Capacity;
        if (count_ < Capacity) {
            ++count_;
        }
    }

    // Copies samples into `out` in chronological order (oldest -> newest).
    // Index 0..99 = 2.0 s pre-trigger, index 100..149 = 1.0 s post-trigger.
    void copy_chronological(std::array<T, Capacity>& out) const noexcept {
        out.fill(T{});
        const std::size_t start = (count_ < Capacity) ? 0U : head_;
        for (std::size_t i = 0U; i < count_; ++i) {
            out[i] = data_[(start + i) % Capacity];
        }
    }

    std::size_t size() const noexcept { return count_; }

private:
    std::array<T, Capacity> data_{};
    std::size_t head_{0U};   // Next slot to write
    std::size_t count_{0U};  // Number of valid samples stored (0..Capacity)
};