#include "atomic_file_output.hpp"
#include <mmd/document.hpp>

#include <array>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <iterator>
#include <stdexcept>
#include <string>

#ifdef __linux__
#include <csignal>
#include <sys/resource.h>
#include <sys/wait.h>
#include <unistd.h>
#endif

namespace {
void require(bool condition, const char *message) {
    if (!condition)
        throw std::runtime_error(message);
}

struct TestDirectory {
    std::filesystem::path path;
    TestDirectory() {
        for (int attempt = 0; attempt < 64; ++attempt) {
            path = std::filesystem::temp_directory_path() /
                   ("libmmd-serialization-test-" +
                    std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
            if (std::filesystem::create_directory(path))
                return;
        }
        throw std::runtime_error("cannot create test directory");
    }
    ~TestDirectory() {
        std::error_code ignored;
        std::filesystem::remove_all(path, ignored);
    }
};

std::string bytes(const std::filesystem::path &path) {
    std::ifstream input(path, std::ios::binary);
    require(static_cast<bool>(input), "cannot read saved file");
    return {std::istreambuf_iterator<char>(input), std::istreambuf_iterator<char>()};
}

mmd::PmxModel fixture() {
    mmd::PmxModel model;
    model.metadata.version = 2.1F;
    model.metadata.additionalUvCount = 4;
    model.metadata.modelName = "roundtrip \xE3\x83\x86\xE3\x82\xB9\xE3\x83\x88 \xF0\x9F\x98\x80";
    model.bones.resize(1);
    model.vertices.resize(5);
    for (std::size_t index = 0; index < model.vertices.size(); ++index) {
        auto &vertex = model.vertices[index];
        vertex.weightType = static_cast<mmd::PmxWeightType>(index);
        vertex.bones = {0, 0, 0, 0};
        vertex.weights = {0.4F, 0.3F, 0.2F, 0.1F};
        vertex.position = {static_cast<float>(index), 1, 0};
        vertex.additionalUv[3] = {1, 2, 3, 4};
    }
    model.indices = {0, 1, 2};
    model.textures.resize(1);
    model.textures[0].storedPath = "texture.png";
    model.materials.resize(1);
    model.materials[0].textureIndex = 0;
    model.materials[0].sphereTextureIndex = 0;
    model.materials[0].toonTextureIndex = 0;
    model.materials[0].indexCount = 3;
    model.rigidBodies.resize(1);
    model.rigidBodies[0].bone = 0;
    model.rigidBodies[0].size = {1, 1, 1};
    model.rigidBodies[0].mass = 1;
    model.rigidBodies[0].mode = 2;
    model.joints.resize(1);
    model.joints[0].bodyA = 0;
    model.joints[0].bodyB = 0;
    model.morphs.resize(2);
    model.morphs[0].type = 0;
    model.morphs[0].offsets.resize(1);
    model.morphs[0].offsets[0].index = 1;
    model.morphs[0].offsets[0].scalar = 0.5F;
    model.morphs[1].type = 1;
    model.morphs[1].offsets.resize(1);
    model.morphs[1].offsets[0].index = 0;
    model.softBodies.resize(1);
    model.softBodies[0].material = 0;
    model.softBodies[0].anchors.push_back({0, 0, false});
    model.softBodies[0].pinnedVertices.push_back(1);
    return model;
}

constexpr std::array<std::uint8_t mmd::PmxFormat::*, 6> widthFields{
    &mmd::PmxFormat::vertexIndexSize, &mmd::PmxFormat::textureIndexSize, &mmd::PmxFormat::materialIndexSize,
    &mmd::PmxFormat::boneIndexSize,   &mmd::PmxFormat::morphIndexSize,   &mmd::PmxFormat::rigidBodyIndexSize};

void formatRoundtrips(const std::filesystem::path &path) {
    std::size_t roundtrips = 0;
    for (float version : {2.0F, 2.1F}) {
        auto model = fixture();
        model.metadata.version = version;
        if (version < 2.1F) {
            model.vertices[4].weightType = mmd::PmxWeightType::bdef4;
            model.softBodies.clear();
        }
        for (auto encoding : {mmd::PmxTextEncoding::utf16le, mmd::PmxTextEncoding::utf8}) {
            model.format.textEncoding = encoding;
            for (std::size_t combination = 0; combination < 729; ++combination) {
                constexpr std::array<std::uint8_t, 3> widths{1, 2, 4};
                auto digits = combination;
                for (auto field : widthFields) {
                    model.format.*field = widths[digits % 3];
                    digits /= 3;
                }
                static_cast<void>(mmd::pmx::save(path, model));
                const auto loaded = mmd::pmx::load(path);
                require(loaded.format == model.format, "format roundtrip changed valid header settings");
                require(mmd::pmx::semanticEqual(model, loaded, mmd::PmxComparisonProfile::logical),
                        "format roundtrip changed PMX payload");
                ++roundtrips;
            }
        }
    }
    require(roundtrips == 2916, "format combination coverage is incomplete");
}

void rejectedSave(const std::filesystem::path &path, const mmd::PmxModel &invalid, const std::string &before) {
    require(!mmd::pmx::validate(invalid).valid(), "invalid serialization input passed validation");
    for (auto policy : {mmd::PmxIndexWidthPolicy::preserveAndWiden, mmd::PmxIndexWidthPolicy::minimal,
                        mmd::PmxIndexWidthPolicy::force32}) {
        mmd::PmxSaveOptions options;
        options.indexWidths = policy;
        for (auto mode : {mmd::PmxSaveMode::preserve, mmd::PmxSaveMode::canonical}) {
            options.mode = mode;
            bool rejected = false;
            try {
                static_cast<void>(mmd::pmx::save(path, invalid, options));
            } catch (const std::runtime_error &) {
                rejected = true;
            }
            require(rejected, "invalid serialization input was saved");
            require(bytes(path) == before, "rejected save altered destination");
        }
    }
}

void invalidFormats(const std::filesystem::path &path) {
    mmd::PmxModel model;
    model.metadata.version = 2.1F;
    static_cast<void>(mmd::pmx::save(path, model));
    const auto before = bytes(path);
    for (auto field : widthFields)
        for (std::uint8_t bad : {std::uint8_t{0}, std::uint8_t{3}, std::uint8_t{255}}) {
            auto invalid = model;
            invalid.format.*field = bad;
            rejectedSave(path, invalid, before);
            mmd::PmxDocument document(invalid);
            require(!document.transaction().commit().committed, "invalid format transaction committed");
        }
    for (std::uint8_t bad : {std::uint8_t{2}, std::uint8_t{255}}) {
        auto invalid = model;
        invalid.format.textEncoding = static_cast<mmd::PmxTextEncoding>(bad);
        rejectedSave(path, invalid, before);
    }
}

void invalidWeights(const std::filesystem::path &path) {
    const auto model = fixture();
    static_cast<void>(mmd::pmx::save(path, model));
    const auto before = bytes(path);
    for (std::uint8_t bad : {std::uint8_t{5}, std::uint8_t{255}}) {
        auto invalid = model;
        invalid.vertices[0].weightType = static_cast<mmd::PmxWeightType>(bad);
        rejectedSave(path, invalid, before);
        mmd::PmxDocument document(model);
        const auto vertex = document.vertexHandle(0);
        require(!document.replaceVertex(vertex, invalid.vertices[0]).committed, "invalid weight property committed");
        for (int setter = 0; setter < 3; ++setter) {
            auto transaction = document.transaction();
            if (setter == 0)
                require(transaction.setVertex(vertex, invalid.vertices[0]), "vertex setter failed");
            if (setter == 1)
                require(transaction.setVertexSkin(vertex, invalid.vertices[0]), "vertex skin setter failed");
            if (setter == 2) {
                mmd::PmxVertexSkin skin;
                skin.type = invalid.vertices[0].weightType;
                skin.bones.fill(document.boneHandle(0));
                require(transaction.setVertexSkin(vertex, skin), "handle skin setter failed");
            }
            require(!transaction.commit().committed, "invalid weight transaction committed");
            require(document.model().vertices == model.vertices, "failed weight edit changed document");
        }
    }
}

void noStagingFiles(const std::filesystem::path &parent) {
    for (const auto &entry : std::filesystem::directory_iterator(parent))
        require(!entry.path().filename().native().starts_with(std::filesystem::path(".libmmd-save-").native()),
                "save leaked staging files");
}

void atomicFailures(const std::filesystem::path &path) {
    const auto model = fixture();
    static_cast<void>(mmd::pmx::save(path, model));
    const auto before = bytes(path);
    {
        mmd::internal::AtomicFileOutput abandoned(path);
        abandoned.stream() << "partial data";
    }
    require(bytes(path) == before, "abandoned output changed destination");
    bool failed = false;
    try {
        mmd::internal::AtomicFileOutput output(path);
        output.stream() << "partial data";
        output.stream().setstate(std::ios::badbit);
        output.commit();
    } catch (const std::runtime_error &) {
        failed = true;
    }
    require(failed && bytes(path) == before, "failed flush changed destination");
    failed = false;
    try {
        mmd::internal::AtomicFileOutput output(path);
        output.stream().exceptions(std::ios::failbit | std::ios::badbit);
        output.stream() << "partial data";
        output.stream().setstate(std::ios::badbit);
    } catch (const std::ios_base::failure &) {
        failed = true;
    }
    require(failed && bytes(path) == before, "I/O exception cleanup changed destination");
    noStagingFiles(path.parent_path());
    const auto directoryTarget = path.parent_path() / "directory-target";
    std::filesystem::create_directory(directoryTarget);
    const auto marker = directoryTarget / "keep.txt";
    {
        std::ofstream output(marker);
        output << "keep";
    }
    failed = false;
    try {
        static_cast<void>(mmd::pmx::save(directoryTarget, model));
    } catch (const std::exception &) {
        failed = true;
    }
    require(failed && bytes(marker) == "keep", "failed replacement damaged destination");
    noStagingFiles(path.parent_path());
#ifndef _WIN32
    const auto mode = std::filesystem::perms::owner_read | std::filesystem::perms::owner_write;
    std::filesystem::permissions(path, mode);
    static_cast<void>(mmd::pmx::save(path, model));
    require(std::filesystem::status(path).permissions() == mode, "replacement changed POSIX file mode");
#endif
#ifdef __linux__
    // Exercise an actual PMX write failure after validation, without changing
    // the parent's process limits. EFBIG represents the same short-write path
    // as ENOSPC; the partially written staged file must never be published.
    const auto child = ::fork();
    require(child >= 0, "cannot fork I/O failure test");
    if (child == 0) {
        const rlimit limit{1024, 1024};
        if (::setrlimit(RLIMIT_FSIZE, &limit) != 0 || std::signal(SIGXFSZ, SIG_IGN) == SIG_ERR)
            ::_exit(2);
        auto large = model;
        large.metadata.comment.assign(8192, 'x');
        try {
            static_cast<void>(mmd::pmx::save(path, large));
        } catch (const std::exception &) {
            ::_exit(bytes(path) == before ? 0 : 3);
        }
        ::_exit(4);
    }
    int status{};
    require(::waitpid(child, &status, 0) == child && WIFEXITED(status) && WEXITSTATUS(status) == 0,
            "write failure did not preserve PMX destination");
    require(bytes(path) == before, "write failure changed original PMX");
    noStagingFiles(path.parent_path());
#endif
    require(mmd::pmx::semanticEqual(model, mmd::pmx::load(path)), "original PMX became unreadable after failed saves");
}
} // namespace

int main() {
    try {
        TestDirectory directory;
        const auto path = directory.path / std::filesystem::path(std::u8string(u8"\u4fdd\u5b58\u30e2\u30c7\u30eb.pmx"));
        formatRoundtrips(path);
        invalidFormats(path);
        invalidWeights(path);
        atomicFailures(path);
        noStagingFiles(directory.path);
        std::puts("serialization hardening tests passed (2916 format/version roundtrips)");
    } catch (const std::exception &error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
}
