#pragma once

#include <filesystem>
#include <fstream>

namespace mmd::internal {

// Writes on the destination filesystem, then publishes a closed, complete file.
// The private staging directory reserves the name exclusively in C++20.
class AtomicFileOutput {
  public:
    explicit AtomicFileOutput(const std::filesystem::path &target);
    ~AtomicFileOutput();
    AtomicFileOutput(const AtomicFileOutput &) = delete;
    AtomicFileOutput &operator=(const AtomicFileOutput &) = delete;
    [[nodiscard]] std::ostream &stream() noexcept {
        return output_;
    }
    void commit();

  private:
    void cleanup() noexcept;
    std::filesystem::path target_;
    std::filesystem::path directory_;
    std::filesystem::path staged_;
    std::ofstream output_;
};

} // namespace mmd::internal
