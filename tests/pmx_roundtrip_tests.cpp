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

    auto formatted = model;
    formatted.format.vertexIndexSize = 1;
    formatted.format.textureIndexSize = 1;
    formatted.format.materialIndexSize = 1;
    formatted.format.boneIndexSize = 1;
    formatted.format.morphIndexSize = 1;
    formatted.format.rigidBodyIndexSize = 1;
    const auto preservedPath = std::filesystem::temp_directory_path() / "libmmd-pmx-preserved-widths.pmx";
    const auto preserved = mmd::pmx::save(preservedPath, formatted);
    assert(!preserved.bone.widened());
    const auto preservedModel = mmd::pmx::load(preservedPath);
    assert(preservedModel.format.vertexIndexSize == 1);
    assert(preservedModel.format.boneIndexSize == 1);

    formatted.bones.resize(129);
    const auto widened = mmd::pmx::save(preservedPath, formatted);
    assert(widened.bone.oldWidth == 1);
    assert(widened.bone.newWidth == 2);
    assert(widened.bone.widened());
    const auto widenedModel = mmd::pmx::load(preservedPath);
    assert(widenedModel.format.boneIndexSize == 2);
    assert(mmd::pmx::semanticEqual(formatted, widenedModel));

    auto semanticallyEquivalent = model;
    semanticallyEquivalent.format.boneIndexSize = 1;
    semanticallyEquivalent.sourcePath = "other.pmx";
    semanticallyEquivalent.vertices[0].bones[3] = 99;
    assert(mmd::pmx::semanticEqual(model, semanticallyEquivalent));
    semanticallyEquivalent.vertices[0].bones[0] = -1;
    const auto diff = mmd::pmx::semanticCompare(model, semanticallyEquivalent);
    assert(!diff.equal());
    assert(!diff.differences.empty());

    std::filesystem::remove(path);
    std::filesystem::remove(preservedPath);
}
