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
    {
        auto relationModel = model;
        relationModel.bones.push_back({.name = "child"});
        mmd::PmxDocument editable(relationModel);
        const auto root = editable.boneHandle(0);
        const auto child = editable.boneHandle(1);
        const auto checkEdit = [&](auto edit) {
            const auto before = editable.model();
            auto transaction = editable.transaction();
            assert(edit(transaction));
            const auto result = transaction.commit();
            assert(result.committed);
            assert(result.changes.bones == std::vector<mmd::BoneHandle>{child});
            const auto after = editable.model();
            assert(editable.applyPatch(result.patch, false));
            assert(mmd::pmx::semanticEqual(editable.model(), before));
            assert(editable.applyPatch(result.patch, true));
            assert(mmd::pmx::semanticEqual(editable.model(), after));
        };
        checkEdit([&](auto &tx) { return tx.setBoneParent(child, root); });
        checkEdit([&](auto &tx) { return tx.setBoneParent(child, std::nullopt); });
        checkEdit([&](auto &tx) { return tx.setBoneTailBone(child, root); });
        checkEdit([&](auto &tx) { return tx.setBoneTailOffset(child, {0.0F, 1.0F, 0.0F}); });
        checkEdit([&](auto &tx) { return tx.setBoneInherit(child, root, 0.5F, true, false); });
        checkEdit([&](auto &tx) { return tx.setBoneInherit(child, std::nullopt, 0.0F, false, false); });
        checkEdit([&](auto &tx) { return tx.setBoneIkTarget(child, root); });
        checkEdit([&](auto &tx) { return tx.addBoneIkLink(child, mmd::BoneIkLinkDraft{.bone = root}); });
        checkEdit([&](auto &tx) { return tx.eraseBoneIkLink(child, 0); });
        checkEdit([&](auto &tx) { return tx.setBoneIkTarget(child, std::nullopt); });
    }
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

    {
        const auto before = model;
        mmd::PmxDocument noOp(model);
        assert(noOp.transaction().commit().committed);
        assert(mmd::pmx::semanticEqual(before, noOp.model(), mmd::PmxComparisonProfile::preservation));
    }
    mmd::PmxDocument document(model);
    const auto root = document.boneHandle(0);
    assert(document.resolve(root) != nullptr);
    assert(document.referencesTo(root).size() == 3);
    auto staleRoot = root;
    ++staleRoot.generation;
    assert(document.referencesTo(staleRoot).empty());
    {
        mmd::PmxDocument foreignDocument(model);
        mmd::PmxDocument localDocument(model);
        const auto foreignRoot = foreignDocument.boneHandle(0);
        const auto localRoot = localDocument.boneHandle(0);
        assert(foreignRoot.domain != localRoot.domain);
        assert(localDocument.referencesTo(foreignRoot).empty());
        auto transaction = localDocument.transaction();
        assert(!transaction.setBoneParent(localRoot, foreignRoot));
        assert(!transaction.setBoneTailBone(localRoot, foreignRoot));
        assert(!transaction.setBoneInherit(localRoot, foreignRoot, 1.0F, true, false));
        assert(!transaction.setBoneIkTarget(localRoot, foreignRoot));
        assert(!transaction.addBoneIkLink(localRoot, {.bone = foreignRoot}));
        assert(transaction.commit().committed);
        mmd::BoneDraft foreignDraft;
        foreignDraft.value.name = "foreign parent";
        foreignDraft.parent = foreignRoot;
        auto draftTransaction = localDocument.transaction();
        assert(!draftTransaction.addBone(std::move(foreignDraft)));
        assert(!draftTransaction.commit().committed);
    }
    assert(document.faces().size() == 1);
    assert(document.referencesTo(document.materialHandle(0)).size() == 1);
    assert(document.referencesTo(document.vertexHandle(0)).size() == 1);
    {
        auto withAllMaterialMorph = model;
        withAllMaterialMorph.morphs = {{.name = "all", .type = 8, .offsets = {{.index = -1}}}};
        mmd::PmxDocument allMaterialDocument(std::move(withAllMaterialMorph));
        assert(allMaterialDocument.allMaterialReferences().size() == 1);
        assert(allMaterialDocument.allMaterialReferences()[0].targetKind == mmd::ReferenceTargetKind::all);
    }
    {
        const auto before = document.model();
        auto transaction = document.transaction();
        const auto impact = transaction.analyzeErase(root);
        assert(impact.vertexWeights == 3);
        assert(!transaction.eraseBone(root));
        const auto result = transaction.commit();
        assert(!result.committed);
        assert(!result.errors.empty());
        assert(mmd::pmx::semanticEqual(before, document.model(), mmd::PmxComparisonProfile::preservation));
        assert(document.resolve(root) != nullptr);
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
    {
        mmd::PmxDocument editable(model);
        const auto firstFace = editable.faceHandle(0);
        const auto firstVertex = editable.vertexHandle(0);
        const auto secondVertex = editable.vertexHandle(1);
        const auto thirdVertex = editable.vertexHandle(2);
        auto transaction = editable.transaction();
        assert(transaction.analyzeErase(firstVertex).faces == 1);
        const auto secondMaterial = transaction.addMaterial({.name = "second"});
        assert(secondMaterial);
        assert(transaction.setFaceMaterial(firstFace, secondMaterial));
        assert(transaction.addFace(firstVertex, secondVertex, thirdVertex, secondMaterial));
        const auto unreferencedVertex = transaction.addVertex({.bones = {0, -1, -1, -1}});
        assert(transaction.moveVertex(unreferencedVertex, 0));
        const auto firstMorph = transaction.addMorph({.name = "a", .type = 0});
        const auto secondMorph = transaction.addMorph({.name = "b", .type = 0});
        assert(transaction.moveMorph(secondMorph, 0));
        const auto texture = transaction.addTexture({"new.png"});
        assert(texture);
        const auto rigidBody = transaction.addRigidBody({.name = "body", .bone = 0});
        assert(rigidBody);
        assert(transaction.addJoint({.name = "joint", .bodyA = 0, .bodyB = 0}));
        const auto frame = transaction.addDisplayFrame({.name = "frame"});
        const auto softBody = transaction.addSoftBody({.name = "soft", .material = -1});
        assert(frame && softBody);
        const auto result = transaction.commit();
        assert(result.committed);
        assert(editable.model().materials[0].indexCount == 0);
        assert(editable.model().materials[1].indexCount == 6);
        assert(editable.model().indices.size() == 6);
        assert(editable.resolve(unreferencedVertex) != nullptr);
        assert(editable.resolve(firstFace) != nullptr);
        assert(editable.model().morphs[0].name == "b");
        assert(editable.resolve(frame) != nullptr);
        assert(editable.resolve(softBody) != nullptr);
    }
    {
        auto shifted = model;
        shifted.textures = {{"unused.png"}, {"body.png"}};
        shifted.materials[0].textureIndex = 1;
        mmd::PmxDocument editable(std::move(shifted));
        auto transaction = editable.transaction();
        assert(transaction.eraseTexture(editable.textureHandle(0)));
        assert(transaction.commit().committed);
        assert(editable.model().textures[0].storedPath == "body.png");
        assert(editable.model().materials[0].textureIndex == 0);
    }
    {
        auto shifted = model;
        shifted.vertices.insert(shifted.vertices.begin(), {.bones = {0, -1, -1, -1}});
        shifted.indices = {1, 2, 3};
        shifted.morphs = {{.name = "vertex", .type = 1, .offsets = {{.index = 1}}}};
        mmd::PmxDocument editable(std::move(shifted));
        auto transaction = editable.transaction();
        assert(transaction.eraseVertex(editable.vertexHandle(0)));
        assert(transaction.commit().committed);
        assert(editable.model().morphs[0].offsets[0].index == 0);
    }
    {
        auto shifted = model;
        shifted.morphs = {{.name = "unused", .type = 0}, {.name = "shown", .type = 0}};
        shifted.displayFrames = {{.items = {{.bone = false, .index = 1}}}};
        mmd::PmxDocument editable(std::move(shifted));
        auto transaction = editable.transaction();
        assert(transaction.eraseMorph(editable.morphHandle(0)));
        assert(transaction.commit().committed);
        assert(editable.model().displayFrames[0].items[0].index == 0);
    }
    {
        auto shifted = model;
        shifted.rigidBodies = {{.name = "unused", .bone = 0}, {.name = "used", .bone = 0}};
        shifted.joints = {{.name = "joint", .bodyA = 1, .bodyB = 1}};
        mmd::PmxDocument editable(std::move(shifted));
        auto transaction = editable.transaction();
        assert(transaction.eraseRigidBody(editable.rigidBodyHandle(0)));
        assert(transaction.commit().committed);
        assert(editable.model().joints[0].bodyA == 0);
        assert(editable.model().joints[0].bodyB == 0);
    }
    {
        auto shifted = model;
        shifted.materials.push_back({.name = "replacement"});
        shifted.morphs = {{.name = "all", .type = 8, .offsets = {{.index = -1}}}};
        mmd::PmxDocument editable(std::move(shifted));
        auto transaction = editable.transaction();
        assert(transaction.eraseMaterial(editable.materialHandle(0), editable.materialHandle(1)));
        assert(transaction.commit().committed);
        assert(editable.model().materials.size() == 1);
        assert(editable.model().materials[0].indexCount == 3);
        assert(editable.model().morphs[0].offsets[0].index == -1);
    }
    {
        auto shifted = model;
        shifted.materials.insert(shifted.materials.begin(), {.name = "unused"});
        shifted.morphs = {{.name = "material", .type = 8, .offsets = {{.index = 1}}}};
        mmd::PmxDocument editable(std::move(shifted));
        auto transaction = editable.transaction();
        assert(transaction.eraseMaterial(editable.materialHandle(0)));
        assert(transaction.commit().committed);
        assert(editable.model().morphs[0].offsets[0].index == 0);
    }
    {
        mmd::PmxDocument editable(model);
        auto transaction = editable.transaction();
        const auto inserted = transaction.insertBone(0, {.name = "inserted", .parent = 0});
        assert(inserted);
        assert(transaction.commit().committed);
        assert(editable.resolve(inserted)->parent == 1);
        assert(editable.model().vertices[0].bones[0] == 1);
    }
    {
        mmd::PmxDocument editable(model);
        const auto rootBone = editable.boneHandle(0);
        const auto material = editable.materialHandle(0);
        auto transaction = editable.transaction();
        const auto bone = transaction.addBone({.value = {.name = "draft"}, .parent = rootBone});
        const auto body = transaction.addRigidBody({.value = {.name = "draft body"}, .bone = rootBone});
        assert(bone && body);
        assert(transaction.addJoint({.value = {.name = "draft joint"}, .bodyA = body, .bodyB = body}));
        assert(transaction.addSoftBody({.value = {.name = "draft soft"}, .material = material}));
        assert(transaction.commit().committed);
    }
    {
        mmd::PmxDocument editable(model);
        const auto rootBone = editable.boneHandle(0);
        mmd::BoneDraft draft;
        draft.value.name = "handle relations";
        draft.value.flags = static_cast<std::uint16_t>(0x0001U | 0x0020U | 0x0100U);
        draft.value.parent = 99;
        draft.value.tailBone = 99;
        draft.value.inheritParent = 99;
        draft.value.ikTarget = 99;
        draft.value.ikLinks = {{.bone = 99}};
        draft.value.inheritRatio = 0.5F;
        draft.parent = rootBone;
        draft.tailBone = rootBone;
        draft.inheritParent = rootBone;
        draft.ikTarget = rootBone;
        draft.ikLinks = {{.bone = rootBone, .limited = true, .minimum = {-1.0F, -2.0F, -3.0F}}};
        auto transaction = editable.transaction();
        const auto relationBone = transaction.addBone(std::move(draft));
        assert(relationBone);
        const auto result = transaction.commit();
        assert(result.committed);
        const auto *stored = editable.resolve(relationBone);
        assert(stored != nullptr);
        assert(stored->parent == 0);
        assert(stored->tailBone == 0);
        assert(stored->inheritParent == 0);
        assert(stored->ikTarget == 0);
        assert(stored->ikLinks.size() == 1);
        assert(stored->ikLinks[0].bone == 0);
        assert(stored->ikLinks[0].limited);
        assert(stored->ikLinks[0].minimum[0] == -1.0F);
    }
    {
        mmd::PmxDocument editable(model);
        const auto rootBone = editable.boneHandle(0);
        auto transaction = editable.transaction();
        const auto relationBone = transaction.addBone({.value = {.name = "editable relations"}});
        assert(relationBone);
        assert(transaction.setBoneParent(relationBone, rootBone));
        assert(transaction.setBoneParent(relationBone, std::nullopt));
        assert(transaction.setBoneTailBone(relationBone, rootBone));
        assert(transaction.setBoneTailOffset(relationBone, {1.0F, 2.0F, 3.0F}));
        assert(transaction.setBoneInherit(relationBone, rootBone, 0.25F, true, false));
        assert(transaction.setBoneInherit(relationBone, std::nullopt, 0.0F, false, false));
        assert(transaction.setBoneIkTarget(relationBone, rootBone));
        assert(transaction.addBoneIkLink(relationBone, {.bone = rootBone}));
        assert(transaction.eraseBoneIkLink(relationBone, 0));
        assert(transaction.setBoneIkTarget(relationBone, std::nullopt));
        assert(transaction.commit().committed);
        const auto *stored = editable.resolve(relationBone);
        assert(stored != nullptr);
        assert(stored->parent == -1);
        assert((stored->flags & 0x0001U) == 0);
        assert(stored->tailBone == -1);
        assert(stored->tailOffset == (mmd::Float3{1.0F, 2.0F, 3.0F}));
        assert((stored->flags & 0x0300U) == 0);
        assert(stored->inheritParent == -1);
        assert((stored->flags & 0x0020U) == 0);
        assert(stored->ikTarget == -1);
        assert(stored->ikLinks.empty());
    }
    {
        auto staleIk = model;
        staleIk.bones.push_back({.name = "stale IK", .ikLinks = {{.bone = 0}}});
        mmd::PmxDocument editable(std::move(staleIk));
        auto transaction = editable.transaction();
        assert(transaction.setBoneIkTarget(editable.boneHandle(1), editable.boneHandle(0)));
        assert(transaction.commit().committed);
        assert((editable.model().bones[1].flags & 0x0020U) != 0);
        assert(editable.model().bones[1].ikTarget == 0);
        assert(editable.model().bones[1].ikLinks.empty());
    }
    {
        mmd::PmxDocument editable(model);
        auto transaction = editable.transaction();
        mmd::BoneDraft invalid;
        invalid.value.name = "mismatched tail";
        invalid.value.flags = 0x0001U;
        assert(!transaction.addBone(std::move(invalid)));
        const auto result = transaction.commit();
        assert(!result.committed);
        assert(!result.errors.empty());
    }
    {
        auto twoBones = model;
        twoBones.bones.push_back({.name = "second"});
        mmd::PmxDocument editable(std::move(twoBones));
        const auto first = editable.boneHandle(0);
        const auto second = editable.boneHandle(1);
        auto transaction = editable.transaction();
        assert(transaction.setBoneParent(first, second));
        assert(!transaction.setBoneInherit(second, first, 1.0F, true, false));
        assert(transaction.commit().committed);
    }
    {
        auto twoBones = model;
        twoBones.bones.push_back({.name = "second"});
        mmd::PmxDocument editable(std::move(twoBones));
        const auto first = editable.boneHandle(0);
        const auto second = editable.boneHandle(1);
        auto transaction = editable.transaction();
        assert(transaction.setBoneInherit(first, second, 1.0F, true, false));
        assert(!transaction.setBoneParent(second, first));
        assert(transaction.commit().committed);
    }
    {
        auto mixedCycle = model;
        mixedCycle.bones.push_back({.name = "second"});
        mixedCycle.bones[0].parent = 1;
        mixedCycle.bones[1].flags = 0x0100U;
        mixedCycle.bones[1].inheritParent = 0;
        assert(!mmd::pmx::validate(mixedCycle).valid());
    }

    std::filesystem::remove(path);
    std::filesystem::remove(preservedPath);
}
