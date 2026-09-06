#include <mmd/pmx.hpp>

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>

extern "C" int LLVMFuzzerTestOneInput(const std::uint8_t *data, std::size_t size) {
    const auto path = std::filesystem::temp_directory_path() / "libmmd-pmx-fuzz-input.pmx";
    {
        std::ofstream output(path, std::ios::binary | std::ios::trunc);
        output.write(reinterpret_cast<const char *>(data), static_cast<std::streamsize>(size));
    }

    try {
        static_cast<void>(mmd::pmx::load(path));
    } catch (...) {
    }
    std::filesystem::remove(path);
    return 0;
}
