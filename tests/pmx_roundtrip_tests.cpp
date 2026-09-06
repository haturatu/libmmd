#include <mmd/document.hpp>
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
    static_cast<void>(mmd::pmx::save(path, model));
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
    formatted.format.boneIndexSize = 2;
    formatted.bones.resize(1);
    assert(mmd::pmx::chooseIndexWidths(formatted).bone == 2);
    assert(mmd::pmx::chooseIndexWidths(formatted, {.indexWidths = mmd::PmxIndexWidthPolicy::minimal}).bone == 1);

    auto semanticallyEquivalent = model;
    semanticallyEquivalent.format.boneIndexSize = 1;
    semanticallyEquivalent.sourcePath = "other.pmx";
    semanticallyEquivalent.vertices[0].bones[3] = 99;
    assert(mmd::pmx::semanticEqual(model, semanticallyEquivalent));
    semanticallyEquivalent.vertices[0].bones[0] = -1;
    const auto diff = mmd::pmx::semanticCompare(model, semanticallyEquivalent);
    assert(!diff.equal());
    assert(!diff.differences.empty());
    model.textures = {{"tex\\body.png"}};
    semanticallyEquivalent = model;
    semanticallyEquivalent.textures[0].storedPath = "tex/body.png";
    assert(mmd::pmx::semanticEqual(model, semanticallyEquivalent));
    assert(!mmd::pmx::semanticEqual(model, semanticallyEquivalent, mmd::PmxComparisonProfile::preservation));

    mmd::PmxDocument document(model);
    const auto root = document.boneHandle(0);
    assert(document.resolve(root) != nullptr);
    assert(document.referencesTo(root).size() == 3);
    assert(document.faces().size() == 1);
    assert(document.referencesTo(document.vertexHandle(0)).size() == 1);
    {
        auto withAllMaterialMorph = model;
        withAllMaterialMorph.morphs = {{.name = "all", .type = 8, .offsets = {{.index = -1}}}};
        mmd::PmxDocument allMaterialDocument(std::move(withAllMaterialMorph));
        assert(allMaterialDocument.allMaterialReferences().size() == 1);
        assert(allMaterialDocument.allMaterialReferences()[0].targetKind == mmd::ReferenceTargetKind::all);
    }
    {
        auto transaction = document.transaction();
        const auto impact = transaction.analyzeErase(root);
        assert(impact.vertexWeights == 3);
        assert(!transaction.eraseBone(root));
        const auto result = transaction.commit();
        assert(!result.committed);
        assert(!result.errors.empty());
    }
    {
        auto transaction = document.transaction();
        const auto extra = transaction.addBone({.name = "extra"});
        assert(extra);
        assert(transaction.moveBone(root, 1));
        const auto result = transaction.commit();
        assert(result.committed);
        assert(document.resolve(root)->name == "root");
        assert(document.model().vertices[0].bones[0] == 1);
    }

    std::filesystem::remove(path);
    std::filesystem::remove(preservedPath);
}
