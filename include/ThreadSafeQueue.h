#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>
#include <optional>
#include <chrono>

/**
 * @brief Thread-safe queue implementation using mutex and condition variable
 * @tparam T Type of elements stored in the queue
 */
template<typename T>
class ThreadSafeQueue {
public:
    ThreadSafeQueue() : shutdown_(false) {}
    
    ~ThreadSafeQueue() {
        shutdown();
    }
    
    /**
     * @brief Push an item to the queue (non-blocking)
     */
    void push(T item) {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (shutdown_) return;  // Don't accept new items after shutdown
            queue_.push(std::move(item));
        }
        cv_.notify_one();  // Wake up one waiting thread
    }
    
    /**
     * @brief Pop an item from the queue (blocking until item available or shutdown)
     * @return std::optional<T> The item, or std::nullopt if queue is shutdown
     */
    std::optional<T> pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        
        // Wait until queue has items or shutdown is called
        cv_.wait(lock, [this]() { 
            return !queue_.empty() || shutdown_; 
        });
        
        // If shutdown and queue is empty, return nullopt
        if (shutdown_ && queue_.empty()) {
            return std::nullopt;
        }
        
        // Get item from queue
        T item = std::move(queue_.front());
        queue_.pop();
        return item;
    }
    
    /**
     * @brief Try to pop an item with timeout
     * @param timeout_ms Timeout in milliseconds
     * @return std::optional<T> The item, or std::nullopt if timeout or shutdown
     */
    std::optional<T> pop_timeout(int timeout_ms) {
        std::unique_lock<std::mutex> lock(mutex_);
        
        // Wait with timeout
        bool has_item = cv_.wait_for(lock, std::chrono::milliseconds(timeout_ms), 
            [this]() { return !queue_.empty() || shutdown_; });
        
        if (!has_item || (shutdown_ && queue_.empty())) {
            return std::nullopt;
        }
        
        T item = std::move(queue_.front());
        queue_.pop();
        return item;
    }
    
    /**
     * @brief Get current queue size
     */
    size_t size() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.size();
    }
    
    /**
     * @brief Check if queue is empty
     */
    bool empty() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return queue_.empty();
    }
    
    /**
     * @brief Shutdown the queue (unblock all waiting threads)
     */
    void shutdown() {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            shutdown_ = true;
        }
        cv_.notify_all();  // Wake up all waiting threads
    }
    
    /**
     * @brief Clear all items in the queue
     */
    void clear() {
        std::lock_guard<std::mutex> lock(mutex_);
        while (!queue_.empty()) {
            queue_.pop();
        }
    }

private:
    std::queue<T> queue_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    bool shutdown_;
};
