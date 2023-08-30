#ifndef LIFO_QUEUE_H
#define LIFO_QUEUE_H

class LifoQueue {
public:
    LifoQueue(size_t limit) : limit_(limit) {}

    void Push(const cv::Vec3f& value) {
        std::unique_lock<std::mutex> lock(mutex_);
        // Wait until the queue size is less than the limit
        cv_full_.wait(lock, [this]() { return queue_.size() < limit_; });
        queue_.push(value);
        cv_empty_.notify_one();
    }

    cv::Vec3f Pop() {
        std::unique_lock<std::mutex> lock(mutex_);
        // Wait until the queue is not empty
        cv_empty_.wait(lock, [this]() { return !queue_.empty(); });
        cv::Vec3f value = queue_.front();
        queue_.pop();
        cv_full_.notify_one();
        return value;
    }

private:
    size_t limit_;
    std::queue<cv::Vec3f> queue_;
    std::mutex mutex_;
    std::condition_variable cv_empty_;
    std::condition_variable cv_full_;
};

#endif // LIFO_QUEUE_H
