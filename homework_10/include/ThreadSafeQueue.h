#pragma once
// ============================================================
// ThreadSafeQueue<T> — потокобезпечна черга (ДЗ10).
//
// Один писач (MissionProcessor), один читач (DronePhysics).
// Захист — std::mutex; очікування непорожньої черги —
// std::condition_variable. Підтримує "розблокування" через
// shutdown(), щоб читач не завис на pop під час зупинки.
// ============================================================
#include <mutex>
#include <condition_variable>
#include <queue>
#include <optional>
#include <utility>

template <typename T>
class ThreadSafeQueue {
public:
    // Покласти елемент і розбудити одного читача.
    void push(T value) {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            q_.push(std::move(value));
        }
        cv_.notify_one();
    }

    // Неблокуюче зняття: nullopt якщо черга порожня.
    std::optional<T> tryPop() {
        std::lock_guard<std::mutex> lk(mtx_);
        if (q_.empty()) return std::nullopt;
        T v = std::move(q_.front());
        q_.pop();
        return v;
    }

    // Блокуюче зняття: чекає, доки з'явиться елемент або shutdown().
    // Повертає nullopt лише якщо чергу зупинено й вона порожня.
    std::optional<T> waitPop() {
        std::unique_lock<std::mutex> lk(mtx_);
        cv_.wait(lk, [this] { return !q_.empty() || shutdown_; });
        if (q_.empty()) return std::nullopt;   // shutdown
        T v = std::move(q_.front());
        q_.pop();
        return v;
    }

    // Залишити лише найсвіжіший елемент, повернути його (якщо є).
    // Зручно, коли важлива тільки остання команда.
    std::optional<T> drainLatest() {
        std::lock_guard<std::mutex> lk(mtx_);
        std::optional<T> last;
        while (!q_.empty()) { last = std::move(q_.front()); q_.pop(); }
        return last;
    }

    bool empty() const {
        std::lock_guard<std::mutex> lk(mtx_);
        return q_.empty();
    }

    // Розблокувати всіх, хто чекає на waitPop().
    void shutdown() {
        {
            std::lock_guard<std::mutex> lk(mtx_);
            shutdown_ = true;
        }
        cv_.notify_all();
    }

private:
    mutable std::mutex      mtx_;
    std::condition_variable cv_;
    std::queue<T>           q_;
    bool                    shutdown_ = false;
};
