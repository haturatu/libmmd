#include <mmd/pmx.hpp>

#include <array>
#include <cstdint>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string_view>

namespace {

template <typename T> void append(std::ofstream &output, T value) {
    output.write(reinterpret_cast<const char *>(&value), sizeof(value));
}

void appendBytes(std::ofstream &output, const void *data, std::size_t size) {
    output.write(reinterpret_cast<const char *>(data), static_cast<std::streamsize>(size));
}

void appendUtf8Text(std::ofstream &output, std::string_view text) {
    append(output, static_cast<std::int32_t>(text.size()));
    appendBytes(output, text.data(), text.size());
}

void appendUtf16AsciiText(std::ofstream &output, std::size_t byteCount) {
    append(output, static_cast<std::int32_t>(byteCount));
    for (std::size_t i = 0; i < byteCount; i += 2)
        append(output, static_cast<std::uint16_t>('a'));
}

void appendPmxHeader(std::ofstream &output, std::int32_t vertexCount, std::uint8_t textEncoding = 1) {
    appendBytes(output, "PMX ", 4);
    append(output, 2.0F);
    append(output, static_cast<std::uint8_t>(8));
    const std::array<std::uint8_t, 8> settings{textEncoding, 0, 1, 1, 1, 1, 1, 1};
    appendBytes(output, settings.data(), settings.size());
    appendUtf8Text(output, {});
    appendUtf8Text(output, {});
    appendUtf8Text(output, {});
    appendUtf8Text(output, {});
    append(output, vertexCount);
}

bool check(bool condition, std::string_view message) {
    if (!condition)
        std::fprintf(stderr, "FAIL: %.*s\n", static_cast<int>(message.size()), message.data());
    return condition;
}

bool rejectsPmx(const std::filesystem::path &path) {
    try {
        static_cast<void>(mmd::pmx::load(path));
    } catch (...) {
        return true;
    }
    return false;
}

void writeHugeVertexCount(const std::filesystem::path &path) {
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    appendPmxHeader(output, 100'000'000);
}

void writeMinimalBones(const std::filesystem::path &path) {
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    appendPmxHeader(output, 0);
    append(output, static_cast<std::int32_t>(0)); // index count
    append(output, static_cast<std::int32_t>(0)); // texture count
    append(output, static_cast<std::int32_t>(0)); // material count
    append(output, static_cast<std::int32_t>(2)); // bone count
    for (int i = 0; i < 2; ++i) {
        appendUtf8Text(output, {});
        appendUtf8Text(output, {});
        const std::array<float, 3> position{};
        appendBytes(output, position.data(), sizeof(position));
        append(output, static_cast<std::int8_t>(-1));       // parent
        append(output, static_cast<std::int32_t>(0));       // deform layer
        append(output, static_cast<std::uint16_t>(0x0001)); // tail is a bone index
        append(output, static_cast<std::int8_t>(-1));       // tail bone
    }
    append(output, static_cast<std::int32_t>(0)); // morph count
    append(output, static_cast<std::int32_t>(0)); // display frame count
    append(output, static_cast<std::int32_t>(0)); // rigid body count
    append(output, static_cast<std::int32_t>(0)); // joint count
}

void writeCumulativeTextBudgetCase(const std::filesystem::path &path) {
    constexpr std::int32_t textureCount = 4;
    constexpr std::int32_t textBytes = 16 * 1024;
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    appendPmxHeader(output, 0);
    append(output, static_cast<std::int32_t>(0)); // index count
    append(output, textureCount);
    for (std::int32_t i = 0; i < textureCount; ++i) {
        append(output, textBytes);
        output.seekp(textBytes, std::ios::cur);
    }
    append(output, static_cast<std::int32_t>(0)); // material count
    append(output, static_cast<std::int32_t>(0)); // bone count
    append(output, static_cast<std::int32_t>(0)); // morph count
    append(output, static_cast<std::int32_t>(0)); // display frame count
    append(output, static_cast<std::int32_t>(0)); // rigid body count
    append(output, static_cast<std::int32_t>(0)); // joint count
}

void writeUtf16TextBudgetCase(const std::filesystem::path &path) {
    constexpr std::int32_t textureCount = 2;
    constexpr std::size_t textBytes = 16 * 1024;
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    appendPmxHeader(output, 0, 0);
    append(output, static_cast<std::int32_t>(0)); // index count
    append(output, textureCount);
    for (std::int32_t i = 0; i < textureCount; ++i)
        appendUtf16AsciiText(output, textBytes);
    append(output, static_cast<std::int32_t>(0)); // material count
    append(output, static_cast<std::int32_t>(0)); // bone count
    append(output, static_cast<std::int32_t>(0)); // morph count
    append(output, static_cast<std::int32_t>(0)); // display frame count
    append(output, static_cast<std::int32_t>(0)); // rigid body count
    append(output, static_cast<std::int32_t>(0)); // joint count
}

} // namespace

int main() {
    bool ok = true;
    const auto hugeVertexPath = std::filesystem::temp_directory_path() / "libmmd-pmx-huge-vertex-count.pmx";
    writeHugeVertexCount(hugeVertexPath);
    ok &= check(rejectsPmx(hugeVertexPath), "rejects implausible vertex count");
    std::filesystem::remove(hugeVertexPath);

    const auto minimalBonePath = std::filesystem::temp_directory_path() / "libmmd-pmx-minimal-bones.pmx";
    writeMinimalBones(minimalBonePath);
    const auto minimalBones = mmd::pmx::load(minimalBonePath);
    ok &= check(minimalBones.bones.size() == 2, "accepts legal compact bones");
    ok &=
        check(!minimalBones.bones.empty() && minimalBones.bones[0].tailBone == -1, "preserves compact bone tail index");
    std::filesystem::remove(minimalBonePath);

    const auto textBudgetPath = std::filesystem::temp_directory_path() / "libmmd-pmx-text-budget.pmx";
    writeCumulativeTextBudgetCase(textBudgetPath);
    ok &= check(rejectsPmx(textBudgetPath), "rejects cumulative PMX UTF-8 text budget");
    std::filesystem::remove(textBudgetPath);

    const auto utf16TextBudgetPath = std::filesystem::temp_directory_path() / "libmmd-pmx-utf16-text-budget.pmx";
    writeUtf16TextBudgetCase(utf16TextBudgetPath);
    ok &= check(rejectsPmx(utf16TextBudgetPath), "rejects cumulative PMX UTF-16 text budget");
    std::filesystem::remove(utf16TextBudgetPath);
    return ok ? 0 : 1;
}
