#pragma once

#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>
#include <queue>

using namespace std;
using namespace boost;

template <typename T>
class LockFreeQueue {
private:
    queue<T> queue_;
    mutable mutex mutex_;
    condition_variable cv_;

public:
    void Produce(T& data) {
        unique_lock<mutex> lock(mutex_);
        queue_.push(data);
        lock.unlock();
        cv_.notify_one();
    }

    bool Consume(T& result)
    {
        unique_lock<mutex> lock(mutex_);
        while (queue_.empty()) cv_.wait(lock);
        result = queue_.front();
        queue_.pop();
    }
};