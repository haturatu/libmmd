#include "atomic_file_output.hpp"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <system_error>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <windows.h>
#endif

namespace mmd::internal {
namespace {
std::filesystem::path reserveDirectory(const std::filesystem::path &parent) {
    static std::atomic<std::uint64_t> sequence{};
    for (int attempt = 0; attempt < 64; ++attempt) {
        const auto stamp = std::chrono::steady_clock::now().time_since_epoch().count();
        const auto name = ".libmmd-save-" + std::to_string(stamp) + "-" +
                          std::to_string(sequence.fetch_add(1, std::memory_order_relaxed));
        const auto candidate = parent / name;
        std::error_code error;
        if (std::filesystem::create_directory(candidate, error))
            return candidate;
        if (error && error != std::errc::file_exists)
            throw std::filesystem::filesystem_error("cannot stage PMX output", candidate, error);
    }
    throw std::runtime_error("cannot reserve PMX staging directory");
}
} // namespace

AtomicFileOutput::AtomicFileOutput(const std::filesystem::path &target) : target_(std::filesystem::absolute(target)) {
    try {
        directory_ = reserveDirectory(target_.parent_path());
#ifndef _WIN32
        // No model data is written until the directory is private.
        std::filesystem::permissions(directory_, std::filesystem::perms::owner_all);
#endif
        staged_ = directory_ / "model.pmx";
        output_.open(staged_, std::ios::binary | std::ios::trunc);
        if (!output_)
            throw std::runtime_error("cannot open staged PMX output: " + target_.string());
    } catch (...) {
        if (output_.is_open())
            output_.close();
        cleanup();
        throw;
    }
}

AtomicFileOutput::~AtomicFileOutput() {
    output_.exceptions(std::ios::goodbit);
    if (output_.is_open())
        output_.close();
    cleanup();
}

void AtomicFileOutput::cleanup() noexcept {
    std::error_code ignored;
    if (!staged_.empty())
        std::filesystem::remove(staged_, ignored);
    if (!directory_.empty())
        std::filesystem::remove(directory_, ignored);
}

void AtomicFileOutput::commit() {
    output_.flush();
    if (!output_)
        throw std::runtime_error("failed while flushing PMX output: " + target_.string());
    output_.close();
    if (!output_)
        throw std::runtime_error("failed while closing PMX output: " + target_.string());
#ifndef _WIN32
    // Replacing an existing file should retain its POSIX mode bits. Errors
    // here also leave the destination untouched; a missing target is normal.
    std::error_code error;
    const auto status = std::filesystem::status(target_, error);
    if (error && error != std::errc::no_such_file_or_directory)
        throw std::filesystem::filesystem_error("cannot inspect PMX destination", target_, error);
    if (std::filesystem::is_regular_file(status))
        std::filesystem::permissions(staged_, status.permissions());
    std::filesystem::rename(staged_, target_);
#else
    // std::filesystem::rename cannot overwrite an existing file on Windows.
    // Both paths are on the same volume; never fall back to delete-and-copy.
    if (!::MoveFileExW(staged_.c_str(), target_.c_str(), MOVEFILE_REPLACE_EXISTING | MOVEFILE_WRITE_THROUGH))
        throw std::filesystem::filesystem_error(
            "cannot replace PMX destination", staged_, target_,
            std::error_code(static_cast<int>(::GetLastError()), std::system_category()));
#endif
}

} // namespace mmd::internal
