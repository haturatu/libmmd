#include <mmd/pmx.hpp>

#include <cassert>
#include <filesystem>

int main() {
    mmd::PmxModel model;
    model.metadata.version = 2.1F;
    model.metadata.modelName = "roundtrip";
    model.vertices = {
        {.position = {0.0F, 0.0F, 0.0F}, .normal = {0.0F, 1.0F, 0.0F}, .uv = {0.0F, 0.0F}, .bones = {0, -1, -1, -1}},
        {.position = {1.0F, 0.0F, 0.0F}, .normal = {0.0F, 1.0F, 0.0F}, .uv = {1.0F, 0.0F}, .bones = {0, -1, -1, -1}},
        {.position = {0.0F, 1.0F, 0.0F}, .normal = {0.0F, 1.0F, 0.0F}, .uv = {0.0F, 1.0F}, .bones = {0, -1, -1, -1}},
    };
    model.indices = {0, 1, 2};
    model.materials = {{.name = "material", .indexCount = 3}};
    model.bones = {{.name = "root", .tailOffset = {0.0F, 1.0F, 0.0F}}};
    const auto path = std::filesystem::temp_directory_path() / "libmmd-pmx-roundtrip.pmx";
    mmd::pmx::save(path, model);
    const auto reloaded = mmd::pmx::load(path);
    assert(reloaded.metadata.modelName == model.metadata.modelName);
    assert(reloaded.vertices.size() == model.vertices.size());
    assert(reloaded.indices == model.indices);
    assert(reloaded.materials.size() == 1);
    assert(mmd::pmx::validate(reloaded).valid());
    std::filesystem::remove(path);
}
