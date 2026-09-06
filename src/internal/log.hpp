#pragma once

#include <iostream>
#include <mutex>

namespace mmd::log {

template <typename... Args> void debug(Args&&... args) {
    static std::mutex mutex;
    std::scoped_lock lock(mutex);
    std::cout << "[DEBUG] ";
    (std::cout << ... << args);
    std::cout << '\n';
    std::cout.flush();
}

template <typename... Args> void info(Args&&... args) {
    static std::mutex mutex;
    std::scoped_lock lock(mutex);
    std::cout << "[INFO] ";
    (std::cout << ... << args);
    std::cout << '\n';
    std::cout.flush();
}

template <typename... Args> void warn(Args&&... args) {
    static std::mutex mutex;
    std::scoped_lock lock(mutex);
    std::cerr << "[WARN] ";
    (std::cerr << ... << args);
    std::cerr << '\n';
    std::cerr.flush();
}

} // namespace mmd::log
