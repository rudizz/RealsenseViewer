#pragma once

#include <condition_variable>
#include <cstddef>
#include <deque>
#include <mutex>
#include <stdexcept>
#include <utility>

namespace rsv {

template <typename T>
class ConcurrentQueue {
public:
    explicit ConcurrentQueue(std::size_t capacity)
        : capacity_(capacity)
    {
        if (capacity_ == 0) {
            throw std::invalid_argument("ConcurrentQueue capacity must be greater than zero");
        }
    }

    ~ConcurrentQueue() = default;
    ConcurrentQueue(const ConcurrentQueue&) = delete;
    ConcurrentQueue& operator=(const ConcurrentQueue&) = delete;
    ConcurrentQueue(ConcurrentQueue&&) = delete;
    ConcurrentQueue& operator=(ConcurrentQueue&&) = delete;


    bool tryPush(T item)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (closed_ || queue_.size() >= capacity_) {
                ++droppedCount_;
                return false;
            }

            queue_.push_back(std::move(item));
        }

        available_.notify_one();
        return true;
    }

    bool tryPop(T& output)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return popUnlocked(output);
    }

    bool waitPop(T& output)
    {
        {
        std::unique_lock<std::mutex> lock(mutex_);
        available_.wait(lock, [this] {
            return closed_ || !queue_.empty();
        });
        }
        
        return popUnlocked(output);
    }

    void close() noexcept
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            closed_ = true;
        }

        available_.notify_all();
    }

    void reset()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        queue_.clear();
        droppedCount_ = 0;
        closed_ = false;
    }

    [[nodiscard]] std::size_t droppedCount() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return droppedCount_;
    }

private:
    const std::size_t capacity_;
    mutable std::mutex mutex_;
    std::condition_variable available_;
    std::deque<T> queue_;
    std::size_t droppedCount_ = 0;
    bool closed_ = false;

    bool popUnlocked(T& output)
    {
        if (queue_.empty()) {
            return false;
        }

        output = std::move(queue_.front());
        queue_.pop_front();
        return true;
    }

};

} // namespace rsv
