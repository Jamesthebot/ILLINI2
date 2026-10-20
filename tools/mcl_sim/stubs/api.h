#pragma once
#include <climits>
#include <cstdint>
#include <cstdio>
#include <functional>
#include <mutex>

#define PROS_ERR (INT32_MAX)

namespace pros {
    uint32_t millis();
    uint64_t micros();

    namespace detail {
        void startTask(std::function<void()> fn);
    }

    class Mutex {
        private:
            std::mutex m_;
        public:
            void lock() { m_.lock(); }
            void unlock() { m_.unlock(); }
            bool try_lock() { return m_.try_lock(); }
    };

    class Task {
        public:
            template <typename F>
            Task(F fn, const char* name = "") { detail::startTask(std::function<void()>(fn)); }
            static void delay_until(uint32_t* prev, uint32_t delta);
    };

    class Distance {
        private:
            int port_;
        public:
            Distance(int port) : port_(port) {}
            int32_t get();
            int32_t get_confidence();
            bool is_installed() { return true; }
            int get_port() const { return port_; }
    };

    class Imu {
        private:
            int port_;
        public:
            Imu(int port) : port_(port) {}
            double get_rotation();
    };
}
