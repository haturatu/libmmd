#include <mmd/document.hpp>

#include <atomic>

namespace mmd {
namespace {

std::atomic<std::uint64_t> nextDocumentDomain{1};

template <typename Tag>
void addReference(std::unordered_map<std::uint64_t, std::vector<ReferenceSite>> &map,
                  const PmxDocument::Table<Tag> &table, std::int32_t target, ReferenceObjectKind ownerKind,
                  std::uint64_t ownerId, std::uint32_t ownerGeneration, ReferenceField field,
                  std::uint32_t subIndex = 0) {
    if (target < 0)
        return;
    const auto targetHandle = table.at(static_cast<std::size_t>(target));
    if (targetHandle)
        map[targetHandle.id].push_back({ownerKind, ownerId, ownerGeneration, field, subIndex});
}

bool containsBoneReference(const PmxModel &model, std::size_t target) {
    const auto match = [target](std::int32_t value) { return value == static_cast<std::int32_t>(target); };
    for (const auto &vertex : model.vertices) {
        const auto count = vertex.weightType == PmxWeightType::bdef1                                               ? 1U
                           : vertex.weightType == PmxWeightType::bdef2 || vertex.weightType == PmxWeightType::sdef ? 2U
                                                                                                                   : 4U;
        for (std::size_t i = 0; i < count; ++i)
            if (match(vertex.bones[i]))
                return true;
    }
    for (const auto &bone : model.bones) {
        if (match(bone.parent) || ((bone.flags & 1U) != 0 && match(bone.tailBone)) ||
            ((bone.flags & 0x0300U) != 0 && match(bone.inheritParent)) ||
            ((bone.flags & 0x0020U) != 0 &&
             (match(bone.ikTarget) || std::any_of(bone.ikLinks.begin(), bone.ikLinks.end(),
                                                  [&](const auto &link) { return match(link.bone); }))))
            return true;
    }
    for (const auto &morph : model.morphs)
        if (morph.type == 2)
            for (const auto &offset : morph.offsets)
                if (match(offset.index))
                    return true;
    for (const auto &frame : model.displayFrames)
        for (const auto &item : frame.items)
            if (item.bone && match(item.index))
                return true;
    return std::any_of(model.rigidBodies.begin(), model.rigidBodies.end(),
                       [&](const auto &body) { return match(body.bone); });
}

constexpr std::uint16_t boneTailBoneFlag = 0x0001U;
constexpr std::uint16_t boneIkFlag = 0x0020U;
constexpr std::uint16_t boneInheritFlags = 0x0300U;
constexpr std::uint16_t boneInheritRotationFlag = 0x0100U;
constexpr std::uint16_t boneInheritTranslationFlag = 0x0200U;

bool hasBoneDependencyCycle(const std::vector<PmxBone> &bones) {
    std::vector<std::uint8_t> state(bones.size());
    const auto visit = [&](auto &&self, std::size_t index) -> bool {
        if (state[index] == 1)
            return true;
        if (state[index] == 2)
            return false;
        state[index] = 1;
        const auto visitEdge = [&](std::int32_t target) {
            if (target < 0 || static_cast<std::size_t>(target) >= bones.size())
                return false;
            return self(self, static_cast<std::size_t>(target));
        };
        if (visitEdge(bones[index].parent) ||
            ((bones[index].flags & boneInheritFlags) != 0 && visitEdge(bones[index].inheritParent)))
            return true;
        state[index] = 2;
        return false;
    };
    for (std::size_t i = 0; i < bones.size(); ++i)
        if (visit(visit, i))
            return true;
    return false;
}

} // namespace

std::uint64_t PmxDocument::allocateDomain() noexcept {
    auto domain = nextDocumentDomain.fetch_add(1, std::memory_order_relaxed);
    while (domain == 0)
        domain = nextDocumentDomain.fetch_add(1, std::memory_order_relaxed);
    return domain;
}

PmxDocument::PmxDocument() : domain_(allocateDomain()) {
    rebuildIndexes();
}
PmxDocument::PmxDocument(PmxModel model) : model_(std::move(model)), domain_(allocateDomain()) {
    rebuildIndexes();
}
PmxDocument::PmxDocument(const PmxDocument &other) : model_(other.model_), domain_(allocateDomain()) {
    rebuildIndexes();
}
PmxDocument &PmxDocument::operator=(const PmxDocument &other) {
    if (this != &other) {
        model_ = other.model_;
        domain_ = allocateDomain();
        dirty_ = false;
        rebuildIndexes();
    }
    return *this;
}

void PmxDocument::rebuildIndexes() {
    vertices_.domain = domain_;
    textures_.domain = domain_;
    materials_.domain = domain_;
    bones_.domain = domain_;
    morphs_.domain = domain_;
    displayFrames_.domain = domain_;
    rigidBodies_.domain = domain_;
    joints_.domain = domain_;
    softBodies_.domain = domain_;
    facesTable_.domain = domain_;
    vertices_.reset(model_.vertices.size());
    textures_.reset(model_.textures.size());
    materials_.reset(model_.materials.size());
    bones_.reset(model_.bones.size());
    morphs_.reset(model_.morphs.size());
    displayFrames_.reset(model_.displayFrames.size());
    rigidBodies_.reset(model_.rigidBodies.size());
    joints_.reset(model_.joints.size());
    softBodies_.reset(model_.softBodies.size());
    faces_.clear();
    std::size_t offset{};
    for (std::size_t material = 0; material < model_.materials.size(); ++material) {
        const auto end = std::min(model_.indices.size(), offset + model_.materials[material].indexCount);
        for (; offset + 2 < end; offset += 3) {
            PmxFace face;
            for (std::size_t vertex = 0; vertex < 3; ++vertex)
                face.vertices[vertex] = vertices_.at(model_.indices[offset + vertex]);
            face.material = materials_.at(material);
            faces_.push_back(face);
        }
        offset = end;
    }
    facesTable_.reset(faces_.size());
    dirty_ = false;
    rebuildReferences();
}

void PmxDocument::rebuildReferences() {
    refs_ = {};
    const auto owner = [](const auto &table, std::size_t i) { return table.at(i); };
    for (std::size_t i = 0; i < model_.vertices.size(); ++i) {
        const auto h = owner(vertices_, i);
        const auto &v = model_.vertices[i];
        const auto count = v.weightType == PmxWeightType::bdef1                                          ? 1U
                           : v.weightType == PmxWeightType::bdef2 || v.weightType == PmxWeightType::sdef ? 2U
                                                                                                         : 4U;
        for (std::size_t n = 0; n < count; ++n)
            addReference(refs_.bones, bones_, v.bones[n], ReferenceObjectKind::vertex, h.id, h.generation,
                         ReferenceField::vertexBone, static_cast<std::uint32_t>(n));
    }
    for (std::size_t i = 0; i < model_.materials.size(); ++i) {
        const auto h = owner(materials_, i);
        const auto &m = model_.materials[i];
        addReference(refs_.textures, textures_, m.textureIndex, ReferenceObjectKind::material, h.id, h.generation,
                     ReferenceField::materialTexture);
        addReference(refs_.textures, textures_, m.sphereTextureIndex, ReferenceObjectKind::material, h.id, h.generation,
                     ReferenceField::materialSphereTexture);
        if (m.toonMode == 0)
            addReference(refs_.textures, textures_, m.toonTextureIndex, ReferenceObjectKind::material, h.id,
                         h.generation, ReferenceField::materialToonTexture);
    }
    for (std::size_t i = 0; i < model_.bones.size(); ++i) {
        const auto h = owner(bones_, i);
        const auto &b = model_.bones[i];
        addReference(refs_.bones, bones_, b.parent, ReferenceObjectKind::bone, h.id, h.generation,
                     ReferenceField::boneParent);
        if (b.flags & 1U)
            addReference(refs_.bones, bones_, b.tailBone, ReferenceObjectKind::bone, h.id, h.generation,
                         ReferenceField::boneTail);
        if (b.flags & 0x0300U)
            addReference(refs_.bones, bones_, b.inheritParent, ReferenceObjectKind::bone, h.id, h.generation,
                         ReferenceField::boneInheritParent);
        if (b.flags & 0x0020U) {
            addReference(refs_.bones, bones_, b.ikTarget, ReferenceObjectKind::bone, h.id, h.generation,
                         ReferenceField::boneIkTarget);
            for (std::size_t n = 0; n < b.ikLinks.size(); ++n)
                addReference(refs_.bones, bones_, b.ikLinks[n].bone, ReferenceObjectKind::bone, h.id, h.generation,
                             ReferenceField::boneIkLink, static_cast<std::uint32_t>(n));
        }
    }
    for (std::size_t i = 0; i < model_.morphs.size(); ++i) {
        const auto h = owner(morphs_, i);
        const auto &m = model_.morphs[i];
        for (std::size_t n = 0; n < m.offsets.size(); ++n) {
            const auto x = m.offsets[n].index;
            if (m.type == 0 || m.type == 9)
                addReference(refs_.morphs, morphs_, x, ReferenceObjectKind::morph, h.id, h.generation,
                             ReferenceField::morphOffset, static_cast<std::uint32_t>(n));
            else if (m.type == 2)
                addReference(refs_.bones, bones_, x, ReferenceObjectKind::morph, h.id, h.generation,
                             ReferenceField::morphOffset, static_cast<std::uint32_t>(n));
            else if (m.type == 8) {
                if (x == -1)
                    refs_.allMaterials.push_back({ReferenceObjectKind::morph, h.id, h.generation,
                                                  ReferenceField::morphOffset, static_cast<std::uint32_t>(n),
                                                  ReferenceTargetKind::all});
                else
                    addReference(refs_.materials, materials_, x, ReferenceObjectKind::morph, h.id, h.generation,
                                 ReferenceField::morphOffset, static_cast<std::uint32_t>(n));
            } else if (m.type == 10)
                addReference(refs_.rigidBodies, rigidBodies_, x, ReferenceObjectKind::morph, h.id, h.generation,
                             ReferenceField::morphOffset, static_cast<std::uint32_t>(n));
        }
    }
    for (std::size_t i = 0; i < model_.displayFrames.size(); ++i) {
        const auto h = owner(displayFrames_, i);
        for (std::size_t n = 0; n < model_.displayFrames[i].items.size(); ++n) {
            const auto &item = model_.displayFrames[i].items[n];
            if (item.bone)
                addReference(refs_.bones, bones_, item.index, ReferenceObjectKind::displayFrame, h.id, h.generation,
                             ReferenceField::displayItem, static_cast<std::uint32_t>(n));
            else
                addReference(refs_.morphs, morphs_, item.index, ReferenceObjectKind::displayFrame, h.id, h.generation,
                             ReferenceField::displayItem, static_cast<std::uint32_t>(n));
        }
    }
    for (std::size_t i = 0; i < model_.rigidBodies.size(); ++i) {
        const auto h = owner(rigidBodies_, i);
        addReference(refs_.bones, bones_, model_.rigidBodies[i].bone, ReferenceObjectKind::rigidBody, h.id,
                     h.generation, ReferenceField::rigidBodyBone);
    }
    for (std::size_t i = 0; i < model_.joints.size(); ++i) {
        const auto h = owner(joints_, i);
        addReference(refs_.rigidBodies, rigidBodies_, model_.joints[i].bodyA, ReferenceObjectKind::joint, h.id,
                     h.generation, ReferenceField::jointBodyA);
        addReference(refs_.rigidBodies, rigidBodies_, model_.joints[i].bodyB, ReferenceObjectKind::joint, h.id,
                     h.generation, ReferenceField::jointBodyB);
    }
    for (std::size_t i = 0; i < model_.softBodies.size(); ++i) {
        const auto h = owner(softBodies_, i);
        const auto &b = model_.softBodies[i];
        addReference(refs_.materials, materials_, b.material, ReferenceObjectKind::softBody, h.id, h.generation,
                     ReferenceField::softBodyMaterial);
        for (std::size_t n = 0; n < b.anchors.size(); ++n)
            addReference(refs_.rigidBodies, rigidBodies_, b.anchors[n].rigidBody, ReferenceObjectKind::softBody, h.id,
                         h.generation, ReferenceField::softBodyAnchorRigidBody, static_cast<std::uint32_t>(n));
    }
    for (std::size_t i = 0; i < faces_.size(); ++i) {
        const auto faceOwner = facesTable_.at(i);
        const auto material = faces_[i].material;
        if (material)
            refs_.materials[material.id].push_back(
                {ReferenceObjectKind::face, faceOwner.id, faceOwner.generation, ReferenceField::faceMaterial, 0});
    }
}

std::vector<ReferenceSite> PmxDocument::referencesTo(VertexHandle handle) const {
    ensure();
    if (!vertices_.index(handle))
        return {};
    std::vector<ReferenceSite> result;
    for (std::size_t i = 0; i < faces_.size(); ++i)
        for (std::size_t n = 0; n < 3; ++n)
            if (faces_[i].vertices[n] == handle) {
                const auto owner = facesTable_.at(i);
                result.push_back({ReferenceObjectKind::face, owner.id, owner.generation, ReferenceField::faceVertex,
                                  static_cast<std::uint32_t>(n)});
            }
    for (std::size_t i = 0; i < model_.morphs.size(); ++i) {
        const auto &morph = model_.morphs[i];
        if (morph.type != 1 && (morph.type < 3 || morph.type > 7))
            continue;
        const auto owner = morphs_.at(i);
        for (std::size_t n = 0; n < morph.offsets.size(); ++n)
            if (morph.offsets[n].index >= 0 && vertices_.at(static_cast<std::size_t>(morph.offsets[n].index)) == handle)
                result.push_back({ReferenceObjectKind::morph, owner.id, owner.generation, ReferenceField::morphOffset,
                                  static_cast<std::uint32_t>(n)});
    }
    for (std::size_t i = 0; i < model_.softBodies.size(); ++i) {
        const auto owner = softBodies_.at(i);
        const auto &body = model_.softBodies[i];
        for (std::size_t n = 0; n < body.anchors.size(); ++n)
            if (body.anchors[n].vertex >= 0 && vertices_.at(static_cast<std::size_t>(body.anchors[n].vertex)) == handle)
                result.push_back({ReferenceObjectKind::softBody, owner.id, owner.generation,
                                  ReferenceField::softBodyAnchorVertex, static_cast<std::uint32_t>(n)});
        for (std::size_t n = 0; n < body.pinnedVertices.size(); ++n)
            if (body.pinnedVertices[n] >= 0 && vertices_.at(static_cast<std::size_t>(body.pinnedVertices[n])) == handle)
                result.push_back({ReferenceObjectKind::softBody, owner.id, owner.generation,
                                  ReferenceField::softBodyPinnedVertex, static_cast<std::uint32_t>(n)});
    }
    return result;
}

void PmxDocument::remapBoneReferences(PmxModel &model, const std::vector<std::int32_t> &map) {
    const auto remap = [&](std::int32_t &x) {
        if (x >= 0 && static_cast<std::size_t>(x) < map.size())
            x = map[static_cast<std::size_t>(x)];
    };
    for (auto &v : model.vertices) {
        const auto count = v.weightType == PmxWeightType::bdef1                                          ? 1U
                           : v.weightType == PmxWeightType::bdef2 || v.weightType == PmxWeightType::sdef ? 2U
                                                                                                         : 4U;
        for (std::size_t i = 0; i < count; ++i)
            remap(v.bones[i]);
    }
    for (auto &b : model.bones) {
        remap(b.parent);
        if (b.flags & 1U)
            remap(b.tailBone);
        if (b.flags & 0x0300U)
            remap(b.inheritParent);
        if (b.flags & 0x0020U) {
            remap(b.ikTarget);
            for (auto &x : b.ikLinks)
                remap(x.bone);
        }
    }
    for (auto &m : model.morphs)
        if (m.type == 2)
            for (auto &x : m.offsets)
                remap(x.index);
    for (auto &f : model.displayFrames)
        for (auto &x : f.items)
            if (x.bone)
                remap(x.index);
    for (auto &b : model.rigidBodies)
        remap(b.bone);
}

namespace {
void remapMaterialReferences(PmxModel &model, const std::vector<std::int32_t> &map) {
    const auto remap = [&](std::int32_t &x) {
        if (x >= 0 && static_cast<std::size_t>(x) < map.size())
            x = map[static_cast<std::size_t>(x)];
    };
    for (auto &morph : model.morphs)
        if (morph.type == 8)
            for (auto &offset : morph.offsets)
                remap(offset.index);
    for (auto &body : model.softBodies)
        remap(body.material);
}
void remapVertexReferences(PmxModel &model, const std::vector<std::int32_t> &map) {
    const auto remap = [&](std::int32_t &x) {
        if (x >= 0 && static_cast<std::size_t>(x) < map.size())
            x = map[static_cast<std::size_t>(x)];
    };
    for (auto &morph : model.morphs)
        if (morph.type == 1 || (morph.type >= 3 && morph.type <= 7))
            for (auto &offset : morph.offsets)
                remap(offset.index);
    for (auto &body : model.softBodies) {
        for (auto &anchor : body.anchors)
            remap(anchor.vertex);
        for (auto &vertex : body.pinnedVertices)
            remap(vertex);
    }
}
void remapMorphReferences(PmxModel &model, const std::vector<std::int32_t> &map) {
    const auto remap = [&](std::int32_t &x) {
        if (x >= 0 && static_cast<std::size_t>(x) < map.size())
            x = map[static_cast<std::size_t>(x)];
    };
    for (auto &morph : model.morphs)
        if (morph.type == 0 || morph.type == 9)
            for (auto &offset : morph.offsets)
                remap(offset.index);
    for (auto &frame : model.displayFrames)
        for (auto &item : frame.items)
            if (!item.bone)
                remap(item.index);
}
void remapTextureReferences(PmxModel &model, const std::vector<std::int32_t> &map) {
    const auto remap = [&](std::int32_t &x) {
        if (x >= 0 && static_cast<std::size_t>(x) < map.size())
            x = map[static_cast<std::size_t>(x)];
    };
    for (auto &material : model.materials) {
        remap(material.textureIndex);
        remap(material.sphereTextureIndex);
        if (material.toonMode == 0)
            remap(material.toonTextureIndex);
    }
}
void remapRigidBodyReferences(PmxModel &model, const std::vector<std::int32_t> &map) {
    const auto remap = [&](std::int32_t &x) {
        if (x >= 0 && static_cast<std::size_t>(x) < map.size())
            x = map[static_cast<std::size_t>(x)];
    };
    for (auto &morph : model.morphs)
        if (morph.type == 10)
            for (auto &offset : morph.offsets)
                remap(offset.index);
    for (auto &joint : model.joints) {
        remap(joint.bodyA);
        remap(joint.bodyB);
    }
    for (auto &body : model.softBodies)
        for (auto &anchor : body.anchors)
            remap(anchor.rigidBody);
}
} // namespace

EraseImpact PmxDocument::Transaction::analyzeErase(BoneHandle handle) const {
    EraseImpact impact;
    const auto target = bones_.index(handle);
    if (!target)
        return impact;
    const auto match = [&](std::int32_t value) { return value == static_cast<std::int32_t>(*target); };
    for (const auto &vertex : model_.vertices) {
        const auto count = vertex.weightType == PmxWeightType::bdef1                                               ? 1U
                           : vertex.weightType == PmxWeightType::bdef2 || vertex.weightType == PmxWeightType::sdef ? 2U
                                                                                                                   : 4U;
        for (std::size_t n = 0; n < count; ++n)
            if (match(vertex.bones[n]))
                ++impact.vertexWeights;
    }
    for (const auto &bone : model_.bones) {
        if (match(bone.parent) || ((bone.flags & 1U) && match(bone.tailBone)) ||
            ((bone.flags & 0x0300U) && match(bone.inheritParent)))
            ++impact.childBones;
        if ((bone.flags & 0x0020U) && match(bone.ikTarget))
            ++impact.ikLinks;
        for (const auto &link : bone.ikLinks)
            if (match(link.bone))
                ++impact.ikLinks;
    }
    for (const auto &morph : model_.morphs)
        if (morph.type == 2)
            for (const auto &offset : morph.offsets)
                if (match(offset.index))
                    ++impact.morphOffsets;
    for (const auto &frame : model_.displayFrames)
        for (const auto &item : frame.items)
            if (item.bone && match(item.index))
                ++impact.displayEntries;
    for (const auto &body : model_.rigidBodies)
        if (match(body.bone))
            ++impact.rigidBodies;
    return impact;
}
bool PmxDocument::Transaction::boneReferenced(std::size_t i) const {
    return containsBoneReference(model_, i);
}
bool PmxDocument::Transaction::materialReferenced(std::size_t i) const {
    for (const auto &m : model_.morphs)
        if (m.type == 8)
            for (const auto &x : m.offsets)
                if (x.index == static_cast<std::int32_t>(i))
                    return true;
    return std::any_of(model_.softBodies.begin(), model_.softBodies.end(),
                       [&](const auto &b) { return b.material == static_cast<std::int32_t>(i); });
}
BoneHandle PmxDocument::Transaction::addBone(BoneDraft draft) {
    if (done_)
        return {};

    const auto flags = draft.value.flags;
    const auto tailBoneEnabled = (flags & boneTailBoneFlag) != 0;
    const auto inheritEnabled = (flags & boneInheritFlags) != 0;
    const auto ikEnabled = (flags & boneIkFlag) != 0;
    if (tailBoneEnabled != draft.tailBone.has_value()) {
        errors_.push_back("bone draft tail reference does not match flags");
        return {};
    }
    if (inheritEnabled != draft.inheritParent.has_value()) {
        errors_.push_back("bone draft inherit reference does not match flags");
        return {};
    }
    if (ikEnabled != draft.ikTarget.has_value() || (!ikEnabled && !draft.ikLinks.empty())) {
        errors_.push_back("bone draft IK references do not match flags");
        return {};
    }

    const auto valid = [&](const auto &handle) { return !handle || bones_.index(*handle).has_value(); };
    if (!valid(draft.parent) || !valid(draft.tailBone) || !valid(draft.inheritParent) || !valid(draft.ikTarget)) {
        errors_.push_back("bone draft contains an invalid handle");
        return {};
    }
    for (const auto &link : draft.ikLinks) {
        if (!bones_.index(link.bone)) {
            errors_.push_back("bone draft contains an invalid IK link");
            return {};
        }
    }

    const auto inheritRatio = draft.value.inheritRatio;
    const auto inheritRotation = (flags & boneInheritRotationFlag) != 0;
    const auto inheritTranslation = (flags & boneInheritTranslationFlag) != 0;
    auto bone = std::move(draft.value);
    // Serialized indices are never accepted from the editor-facing draft.
    bone.parent = -1;
    bone.tailBone = -1;
    bone.inheritParent = -1;
    bone.ikTarget = -1;
    bone.ikLinks.clear();

    const auto handle = addBone(std::move(bone));
    if (!handle)
        return {};
    const auto fail = [&](const char *message) {
        errors_.push_back(message);
        return BoneHandle{};
    };
    if (draft.parent && !setBoneParent(handle, *draft.parent))
        return fail("invalid bone draft parent");
    if (draft.tailBone && !setBoneTailBone(handle, *draft.tailBone))
        return fail("invalid bone draft tail reference");
    if (draft.inheritParent &&
        !setBoneInherit(handle, *draft.inheritParent, inheritRatio, inheritRotation, inheritTranslation))
        return fail("invalid bone draft inherit reference");
    if (draft.ikTarget && !setBoneIkTarget(handle, *draft.ikTarget))
        return fail("invalid bone draft IK target");
    for (const auto &link : draft.ikLinks)
        if (!addBoneIkLink(handle, link))
            return fail("invalid bone draft IK link");
    return handle;
}
bool PmxDocument::Transaction::setBoneParent(BoneHandle child, std::optional<BoneHandle> parent) {
    const auto c = bones_.index(child);
    if (!c)
        return false;
    if (!parent) {
        model_.bones[*c].parent = -1;
        return true;
    }
    const auto p = bones_.index(*parent);
    if (!p || *c == *p)
        return false;
    auto &current = model_.bones[*c];
    const auto previous = current.parent;
    current.parent = static_cast<std::int32_t>(*p);
    if (hasBoneDependencyCycle(model_.bones)) {
        current.parent = previous;
        return false;
    }
    return true;
}
bool PmxDocument::Transaction::setBoneTailBone(BoneHandle bone, BoneHandle target) {
    const auto b = bones_.index(bone), t = bones_.index(target);
    if (!b || !t || *b == *t)
        return false;
    model_.bones[*b].flags |= boneTailBoneFlag;
    model_.bones[*b].tailBone = static_cast<std::int32_t>(*t);
    return true;
}
bool PmxDocument::Transaction::setBoneTailOffset(BoneHandle bone, Float3 offset) {
    const auto b = bones_.index(bone);
    if (!b)
        return false;
    model_.bones[*b].flags &= static_cast<std::uint16_t>(~boneTailBoneFlag);
    model_.bones[*b].tailBone = -1;
    model_.bones[*b].tailOffset = offset;
    return true;
}
bool PmxDocument::Transaction::setBoneInherit(BoneHandle bone, std::optional<BoneHandle> parent, float ratio,
                                              bool rotation, bool translation) {
    const auto b = bones_.index(bone);
    if (!b)
        return false;
    if (!parent) {
        if (rotation || translation)
            return false;
        model_.bones[*b].flags &= static_cast<std::uint16_t>(~boneInheritFlags);
        model_.bones[*b].inheritParent = -1;
        model_.bones[*b].inheritRatio = 0.0F;
        return true;
    }
    const auto p = bones_.index(*parent);
    if (!p || *b == *p || (!rotation && !translation))
        return false;
    auto &current = model_.bones[*b];
    const auto previousFlags = current.flags;
    const auto previousParent = current.inheritParent;
    const auto previousRatio = current.inheritRatio;
    current.flags &= static_cast<std::uint16_t>(~boneInheritFlags);
    if (rotation)
        current.flags |= boneInheritRotationFlag;
    if (translation)
        current.flags |= boneInheritTranslationFlag;
    current.inheritParent = static_cast<std::int32_t>(*p);
    current.inheritRatio = ratio;
    if (hasBoneDependencyCycle(model_.bones)) {
        current.flags = previousFlags;
        current.inheritParent = previousParent;
        current.inheritRatio = previousRatio;
        return false;
    }
    return true;
}
bool PmxDocument::Transaction::setBoneIkTarget(BoneHandle bone, std::optional<BoneHandle> target) {
    const auto b = bones_.index(bone);
    if (!b)
        return false;
    auto &current = model_.bones[*b];
    if (!target) {
        current.flags &= static_cast<std::uint16_t>(~boneIkFlag);
        current.ikTarget = -1;
        current.ikLinks.clear();
        return true;
    }
    const auto t = bones_.index(*target);
    if (!t || *b == *t)
        return false;
    if ((current.flags & boneIkFlag) == 0)
        current.ikLinks.clear();
    current.flags |= boneIkFlag;
    current.ikTarget = static_cast<std::int32_t>(*t);
    return true;
}
bool PmxDocument::Transaction::addBoneIkLink(BoneHandle bone, BoneIkLinkDraft link) {
    const auto b = bones_.index(bone), target = bones_.index(link.bone);
    if (!b || !target || (model_.bones[*b].flags & boneIkFlag) == 0 || model_.bones[*b].ikTarget < 0)
        return false;
    model_.bones[*b].ikLinks.push_back({static_cast<std::int32_t>(*target), link.limited, link.minimum, link.maximum});
    return true;
}
bool PmxDocument::Transaction::eraseBoneIkLink(BoneHandle bone, std::size_t index) {
    const auto b = bones_.index(bone);
    if (!b || index >= model_.bones[*b].ikLinks.size())
        return false;
    model_.bones[*b].ikLinks.erase(model_.bones[*b].ikLinks.begin() + static_cast<std::ptrdiff_t>(index));
    return true;
}
BoneHandle PmxDocument::Transaction::insertBone(std::size_t destination, PmxBone bone) {
    if (done_ || destination > model_.bones.size())
        return {};
    std::vector<std::int32_t> map(model_.bones.size());
    for (std::size_t i = 0; i < map.size(); ++i)
        map[i] = i >= destination ? static_cast<std::int32_t>(i + 1) : static_cast<std::int32_t>(i);
    remapBoneReferences(model_, map);
    const auto remap = [&](std::int32_t &x) {
        if (x >= 0 && static_cast<std::size_t>(x) < map.size())
            x = map[static_cast<std::size_t>(x)];
    };
    remap(bone.parent);
    if (bone.flags & 1U)
        remap(bone.tailBone);
    if (bone.flags & 0x0300U)
        remap(bone.inheritParent);
    if (bone.flags & 0x0020U) {
        remap(bone.ikTarget);
        for (auto &link : bone.ikLinks)
            remap(link.bone);
    }
    model_.bones.insert(model_.bones.begin() + static_cast<std::ptrdiff_t>(destination), std::move(bone));
    return bones_.insert(destination);
}
bool PmxDocument::Transaction::eraseBone(BoneHandle h, ErasePolicy policy) {
    const auto target = bones_.index(h);
    if (!target)
        return false;
    if (policy == ErasePolicy::rejectIfReferenced && boneReferenced(*target)) {
        errors_.push_back("cannot erase referenced bone");
        return false;
    }
    std::vector<std::int32_t> map(model_.bones.size());
    for (std::size_t i = 0; i < map.size(); ++i)
        map[i] = i < *target ? static_cast<std::int32_t>(i) : i == *target ? -1 : static_cast<std::int32_t>(i - 1);
    model_.bones.erase(model_.bones.begin() + static_cast<std::ptrdiff_t>(*target));
    bones_.slots.erase(bones_.slots.begin() + static_cast<std::ptrdiff_t>(*target));
    remapBoneReferences(model_, map);
    return true;
}
bool PmxDocument::Transaction::moveBone(BoneHandle h, std::size_t destination) {
    const auto source = bones_.index(h);
    if (!source || destination >= model_.bones.size())
        return false;
    if (*source == destination)
        return true;
    std::vector<std::int32_t> map(model_.bones.size());
    for (std::size_t i = 0; i < map.size(); ++i)
        map[i] = i == *source                                               ? static_cast<std::int32_t>(destination)
                 : *source < destination && i > *source && i <= destination ? static_cast<std::int32_t>(i - 1)
                 : destination < *source && i >= destination && i < *source ? static_cast<std::int32_t>(i + 1)
                                                                            : static_cast<std::int32_t>(i);
    auto bone = std::move(model_.bones[*source]);
    model_.bones.erase(model_.bones.begin() + static_cast<std::ptrdiff_t>(*source));
    model_.bones.insert(model_.bones.begin() + static_cast<std::ptrdiff_t>(destination), std::move(bone));
    auto slot = bones_.slots[*source];
    bones_.slots.erase(bones_.slots.begin() + static_cast<std::ptrdiff_t>(*source));
    bones_.slots.insert(bones_.slots.begin() + static_cast<std::ptrdiff_t>(destination), slot);
    remapBoneReferences(model_, map);
    return true;
}
bool PmxDocument::Transaction::eraseMaterial(MaterialHandle h) {
    return eraseMaterial(h, {});
}
MaterialHandle PmxDocument::Transaction::addMaterial(PmxMaterial material) {
    if (done_)
        return {};
    material.indexCount = 0;
    model_.materials.push_back(std::move(material));
    return materials_.append();
}
bool PmxDocument::Transaction::moveMaterial(MaterialHandle h, std::size_t destination) {
    const auto source = materials_.index(h);
    if (!source || destination >= model_.materials.size())
        return false;
    if (*source == destination)
        return true;
    std::vector<std::int32_t> map(model_.materials.size());
    for (std::size_t i = 0; i < map.size(); ++i)
        map[i] = i == *source                                               ? static_cast<std::int32_t>(destination)
                 : *source < destination && i > *source && i <= destination ? static_cast<std::int32_t>(i - 1)
                 : destination < *source && i >= destination && i < *source ? static_cast<std::int32_t>(i + 1)
                                                                            : static_cast<std::int32_t>(i);
    auto value = std::move(model_.materials[*source]);
    model_.materials.erase(model_.materials.begin() + static_cast<std::ptrdiff_t>(*source));
    model_.materials.insert(model_.materials.begin() + static_cast<std::ptrdiff_t>(destination), std::move(value));
    auto slot = materials_.slots[*source];
    materials_.slots.erase(materials_.slots.begin() + static_cast<std::ptrdiff_t>(*source));
    materials_.slots.insert(materials_.slots.begin() + static_cast<std::ptrdiff_t>(destination), slot);
    remapMaterialReferences(model_, map);
    return true;
}
bool PmxDocument::Transaction::eraseMaterial(MaterialHandle h, std::optional<MaterialHandle> replacement) {
    const auto target = materials_.index(h);
    if (!target)
        return false;
    for (const auto &face : faces_)
        if (face.material == h && !replacement) {
            errors_.push_back("cannot erase material with faces");
            return false;
        }
    if (materialReferenced(*target)) {
        errors_.push_back("cannot erase referenced material");
        return false;
    }
    if (replacement && (!materials_.index(*replacement) || *replacement == h))
        return false;
    for (auto &face : faces_)
        if (face.material == h)
            face.material = *replacement;
    std::vector<std::int32_t> map(model_.materials.size());
    for (std::size_t i = 0; i < map.size(); ++i)
        map[i] = i < *target ? static_cast<std::int32_t>(i) : i == *target ? -1 : static_cast<std::int32_t>(i - 1);
    model_.materials.erase(model_.materials.begin() + static_cast<std::ptrdiff_t>(*target));
    materials_.slots.erase(materials_.slots.begin() + static_cast<std::ptrdiff_t>(*target));
    remapMaterialReferences(model_, map);
    return true;
}
VertexHandle PmxDocument::Transaction::addVertex(PmxVertex vertex) {
    if (done_)
        return {};
    model_.vertices.push_back(std::move(vertex));
    return vertices_.append();
}
VertexEraseImpact PmxDocument::Transaction::analyzeErase(VertexHandle handle) const {
    VertexEraseImpact impact;
    const auto target = vertices_.index(handle);
    if (!target)
        return impact;
    for (const auto &face : faces_)
        for (const auto &vertex : face.vertices)
            if (vertex == handle)
                ++impact.faces;
    for (const auto &morph : model_.morphs)
        if (morph.type == 1 || (morph.type >= 3 && morph.type <= 7))
            for (const auto &offset : morph.offsets)
                if (offset.index == static_cast<std::int32_t>(*target))
                    ++impact.morphOffsets;
    for (const auto &body : model_.softBodies) {
        for (const auto &anchor : body.anchors)
            if (anchor.vertex == static_cast<std::int32_t>(*target))
                ++impact.softBodyAnchors;
        for (const auto &vertex : body.pinnedVertices)
            if (vertex == static_cast<std::int32_t>(*target))
                ++impact.pinnedVertices;
    }
    return impact;
}
bool PmxDocument::Transaction::vertexReferenced(std::size_t index) const {
    const auto h = vertices_.at(index);
    if (!h)
        return true;
    for (const auto &face : faces_)
        for (const auto &vertex : face.vertices)
            if (vertex == h)
                return true;
    for (const auto &morph : model_.morphs)
        if (morph.type == 1 || (morph.type >= 3 && morph.type <= 7))
            for (const auto &offset : morph.offsets)
                if (offset.index == static_cast<std::int32_t>(index))
                    return true;
    for (const auto &body : model_.softBodies) {
        for (const auto &anchor : body.anchors)
            if (anchor.vertex == static_cast<std::int32_t>(index))
                return true;
        for (const auto &vertex : body.pinnedVertices)
            if (vertex == static_cast<std::int32_t>(index))
                return true;
    }
    return false;
}
bool PmxDocument::Transaction::moveVertex(VertexHandle h, std::size_t destination) {
    const auto source = vertices_.index(h);
    if (!source || destination >= model_.vertices.size())
        return false;
    if (*source == destination)
        return true;
    std::vector<std::int32_t> map(model_.vertices.size());
    for (std::size_t i = 0; i < map.size(); ++i)
        map[i] = i == *source                                               ? static_cast<std::int32_t>(destination)
                 : *source < destination && i > *source && i <= destination ? static_cast<std::int32_t>(i - 1)
                 : destination < *source && i >= destination && i < *source ? static_cast<std::int32_t>(i + 1)
                                                                            : static_cast<std::int32_t>(i);
    auto value = std::move(model_.vertices[*source]);
    model_.vertices.erase(model_.vertices.begin() + static_cast<std::ptrdiff_t>(*source));
    model_.vertices.insert(model_.vertices.begin() + static_cast<std::ptrdiff_t>(destination), std::move(value));
    auto slot = vertices_.slots[*source];
    vertices_.slots.erase(vertices_.slots.begin() + static_cast<std::ptrdiff_t>(*source));
    vertices_.slots.insert(vertices_.slots.begin() + static_cast<std::ptrdiff_t>(destination), slot);
    remapVertexReferences(model_, map);
    return true;
}
bool PmxDocument::Transaction::eraseVertex(VertexHandle h) {
    const auto target = vertices_.index(h);
    if (!target || vertexReferenced(*target)) {
        errors_.push_back("cannot erase referenced vertex");
        return false;
    }
    std::vector<std::int32_t> map(model_.vertices.size());
    for (std::size_t i = 0; i < map.size(); ++i)
        map[i] = i < *target ? static_cast<std::int32_t>(i) : i == *target ? -1 : static_cast<std::int32_t>(i - 1);
    model_.vertices.erase(model_.vertices.begin() + static_cast<std::ptrdiff_t>(*target));
    vertices_.slots.erase(vertices_.slots.begin() + static_cast<std::ptrdiff_t>(*target));
    remapVertexReferences(model_, map);
    return true;
}
FaceHandle PmxDocument::Transaction::addFace(VertexHandle a, VertexHandle b, VertexHandle c, MaterialHandle material) {
    if (done_ || !vertices_.index(a) || !vertices_.index(b) || !vertices_.index(c) || !materials_.index(material))
        return {};
    faces_.push_back({{a, b, c}, material});
    return facesTable_.append();
}
bool PmxDocument::Transaction::eraseFace(FaceHandle h) {
    const auto index = facesTable_.index(h);
    if (!index)
        return false;
    faces_.erase(faces_.begin() + static_cast<std::ptrdiff_t>(*index));
    facesTable_.slots.erase(facesTable_.slots.begin() + static_cast<std::ptrdiff_t>(*index));
    return true;
}
bool PmxDocument::Transaction::setFaceMaterial(FaceHandle h, MaterialHandle material) {
    const auto index = facesTable_.index(h);
    if (!index || !materials_.index(material))
        return false;
    faces_[*index].material = material;
    return true;
}
MorphHandle PmxDocument::Transaction::addMorph(PmxMorph morph) {
    if (done_)
        return {};
    model_.morphs.push_back(std::move(morph));
    return morphs_.append();
}
bool PmxDocument::Transaction::morphReferenced(std::size_t index) const {
    for (const auto &morph : model_.morphs)
        if (morph.type == 0 || morph.type == 9)
            for (const auto &offset : morph.offsets)
                if (offset.index == static_cast<std::int32_t>(index))
                    return true;
    for (const auto &frame : model_.displayFrames)
        for (const auto &item : frame.items)
            if (!item.bone && item.index == static_cast<std::int32_t>(index))
                return true;
    return false;
}
bool PmxDocument::Transaction::moveMorph(MorphHandle h, std::size_t destination) {
    const auto source = morphs_.index(h);
    if (!source || destination >= model_.morphs.size())
        return false;
    if (*source == destination)
        return true;
    std::vector<std::int32_t> map(model_.morphs.size());
    for (std::size_t i = 0; i < map.size(); ++i)
        map[i] = i == *source                                               ? static_cast<std::int32_t>(destination)
                 : *source < destination && i > *source && i <= destination ? static_cast<std::int32_t>(i - 1)
                 : destination < *source && i >= destination && i < *source ? static_cast<std::int32_t>(i + 1)
                                                                            : static_cast<std::int32_t>(i);
    auto value = std::move(model_.morphs[*source]);
    model_.morphs.erase(model_.morphs.begin() + static_cast<std::ptrdiff_t>(*source));
    model_.morphs.insert(model_.morphs.begin() + static_cast<std::ptrdiff_t>(destination), std::move(value));
    auto slot = morphs_.slots[*source];
    morphs_.slots.erase(morphs_.slots.begin() + static_cast<std::ptrdiff_t>(*source));
    morphs_.slots.insert(morphs_.slots.begin() + static_cast<std::ptrdiff_t>(destination), slot);
    remapMorphReferences(model_, map);
    return true;
}
bool PmxDocument::Transaction::eraseMorph(MorphHandle h) {
    const auto target = morphs_.index(h);
    if (!target || morphReferenced(*target)) {
        errors_.push_back("cannot erase referenced morph");
        return false;
    }
    std::vector<std::int32_t> map(model_.morphs.size());
    for (std::size_t i = 0; i < map.size(); ++i)
        map[i] = i < *target ? static_cast<std::int32_t>(i) : i == *target ? -1 : static_cast<std::int32_t>(i - 1);
    model_.morphs.erase(model_.morphs.begin() + static_cast<std::ptrdiff_t>(*target));
    morphs_.slots.erase(morphs_.slots.begin() + static_cast<std::ptrdiff_t>(*target));
    remapMorphReferences(model_, map);
    return true;
}
bool PmxDocument::Transaction::textureReferenced(std::size_t i) const {
    for (const auto &material : model_.materials)
        if (material.textureIndex == static_cast<std::int32_t>(i) ||
            material.sphereTextureIndex == static_cast<std::int32_t>(i) ||
            (material.toonMode == 0 && material.toonTextureIndex == static_cast<std::int32_t>(i)))
            return true;
    return false;
}
TextureHandle PmxDocument::Transaction::addTexture(PmxTexture texture) {
    if (done_)
        return {};
    model_.textures.push_back(std::move(texture));
    return textures_.append();
}
bool PmxDocument::Transaction::moveTexture(TextureHandle h, std::size_t destination) {
    const auto source = textures_.index(h);
    if (!source || destination >= model_.textures.size())
        return false;
    if (*source == destination)
        return true;
    std::vector<std::int32_t> map(model_.textures.size());
    for (std::size_t i = 0; i < map.size(); ++i)
        map[i] = i == *source                                               ? static_cast<std::int32_t>(destination)
                 : *source < destination && i > *source && i <= destination ? static_cast<std::int32_t>(i - 1)
                 : destination < *source && i >= destination && i < *source ? static_cast<std::int32_t>(i + 1)
                                                                            : static_cast<std::int32_t>(i);
    auto value = std::move(model_.textures[*source]);
    model_.textures.erase(model_.textures.begin() + static_cast<std::ptrdiff_t>(*source));
    model_.textures.insert(model_.textures.begin() + static_cast<std::ptrdiff_t>(destination), std::move(value));
    auto slot = textures_.slots[*source];
    textures_.slots.erase(textures_.slots.begin() + static_cast<std::ptrdiff_t>(*source));
    textures_.slots.insert(textures_.slots.begin() + static_cast<std::ptrdiff_t>(destination), slot);
    remapTextureReferences(model_, map);
    return true;
}
bool PmxDocument::Transaction::eraseTexture(TextureHandle h) {
    const auto target = textures_.index(h);
    if (!target || textureReferenced(*target)) {
        errors_.push_back("cannot erase referenced texture");
        return false;
    }
    std::vector<std::int32_t> map(model_.textures.size());
    for (std::size_t i = 0; i < map.size(); ++i)
        map[i] = i < *target ? static_cast<std::int32_t>(i) : i == *target ? -1 : static_cast<std::int32_t>(i - 1);
    model_.textures.erase(model_.textures.begin() + static_cast<std::ptrdiff_t>(*target));
    textures_.slots.erase(textures_.slots.begin() + static_cast<std::ptrdiff_t>(*target));
    remapTextureReferences(model_, map);
    return true;
}
bool PmxDocument::Transaction::rigidBodyReferenced(std::size_t i) const {
    for (const auto &morph : model_.morphs)
        if (morph.type == 10)
            for (const auto &offset : morph.offsets)
                if (offset.index == static_cast<std::int32_t>(i))
                    return true;
    for (const auto &joint : model_.joints)
        if (joint.bodyA == static_cast<std::int32_t>(i) || joint.bodyB == static_cast<std::int32_t>(i))
            return true;
    for (const auto &body : model_.softBodies)
        for (const auto &anchor : body.anchors)
            if (anchor.rigidBody == static_cast<std::int32_t>(i))
                return true;
    return false;
}
RigidBodyHandle PmxDocument::Transaction::addRigidBody(PmxRigidBody body) {
    if (done_)
        return {};
    model_.rigidBodies.push_back(std::move(body));
    return rigidBodies_.append();
}
bool PmxDocument::Transaction::moveRigidBody(RigidBodyHandle h, std::size_t destination) {
    const auto source = rigidBodies_.index(h);
    if (!source || destination >= model_.rigidBodies.size())
        return false;
    if (*source == destination)
        return true;
    std::vector<std::int32_t> map(model_.rigidBodies.size());
    for (std::size_t i = 0; i < map.size(); ++i)
        map[i] = i == *source                                               ? static_cast<std::int32_t>(destination)
                 : *source < destination && i > *source && i <= destination ? static_cast<std::int32_t>(i - 1)
                 : destination < *source && i >= destination && i < *source ? static_cast<std::int32_t>(i + 1)
                                                                            : static_cast<std::int32_t>(i);
    auto value = std::move(model_.rigidBodies[*source]);
    model_.rigidBodies.erase(model_.rigidBodies.begin() + static_cast<std::ptrdiff_t>(*source));
    model_.rigidBodies.insert(model_.rigidBodies.begin() + static_cast<std::ptrdiff_t>(destination), std::move(value));
    auto slot = rigidBodies_.slots[*source];
    rigidBodies_.slots.erase(rigidBodies_.slots.begin() + static_cast<std::ptrdiff_t>(*source));
    rigidBodies_.slots.insert(rigidBodies_.slots.begin() + static_cast<std::ptrdiff_t>(destination), slot);
    remapRigidBodyReferences(model_, map);
    return true;
}
bool PmxDocument::Transaction::eraseRigidBody(RigidBodyHandle h) {
    const auto target = rigidBodies_.index(h);
    if (!target || rigidBodyReferenced(*target)) {
        errors_.push_back("cannot erase referenced rigid body");
        return false;
    }
    std::vector<std::int32_t> map(model_.rigidBodies.size());
    for (std::size_t i = 0; i < map.size(); ++i)
        map[i] = i < *target ? static_cast<std::int32_t>(i) : i == *target ? -1 : static_cast<std::int32_t>(i - 1);
    model_.rigidBodies.erase(model_.rigidBodies.begin() + static_cast<std::ptrdiff_t>(*target));
    rigidBodies_.slots.erase(rigidBodies_.slots.begin() + static_cast<std::ptrdiff_t>(*target));
    remapRigidBodyReferences(model_, map);
    return true;
}
JointHandle PmxDocument::Transaction::addJoint(PmxJoint joint) {
    if (done_)
        return {};
    model_.joints.push_back(std::move(joint));
    return joints_.append();
}
bool PmxDocument::Transaction::eraseJoint(JointHandle h) {
    const auto target = joints_.index(h);
    if (!target)
        return false;
    model_.joints.erase(model_.joints.begin() + static_cast<std::ptrdiff_t>(*target));
    joints_.slots.erase(joints_.slots.begin() + static_cast<std::ptrdiff_t>(*target));
    return true;
}
DisplayFrameHandle PmxDocument::Transaction::addDisplayFrame(PmxDisplayFrame frame) {
    if (done_)
        return {};
    model_.displayFrames.push_back(std::move(frame));
    return displayFrames_.append();
}
bool PmxDocument::Transaction::eraseDisplayFrame(DisplayFrameHandle h) {
    const auto target = displayFrames_.index(h);
    if (!target)
        return false;
    model_.displayFrames.erase(model_.displayFrames.begin() + static_cast<std::ptrdiff_t>(*target));
    displayFrames_.slots.erase(displayFrames_.slots.begin() + static_cast<std::ptrdiff_t>(*target));
    return true;
}
SoftBodyHandle PmxDocument::Transaction::addSoftBody(PmxSoftBody body) {
    if (done_)
        return {};
    if (model_.metadata.version < 2.1F) {
        errors_.push_back("soft bodies require PMX 2.1");
        return {};
    }
    model_.softBodies.push_back(std::move(body));
    return softBodies_.append();
}
bool PmxDocument::Transaction::eraseSoftBody(SoftBodyHandle h) {
    const auto target = softBodies_.index(h);
    if (!target)
        return false;
    model_.softBodies.erase(model_.softBodies.begin() + static_cast<std::ptrdiff_t>(*target));
    softBodies_.slots.erase(softBodies_.slots.begin() + static_cast<std::ptrdiff_t>(*target));
    return true;
}
PmxTransactionResult PmxDocument::Transaction::commit() {
    if (done_)
        return {false, {}, {"transaction has already finished"}};
    done_ = true;
    std::vector<std::vector<const PmxFace *>> facesByMaterial(model_.materials.size());
    for (const auto &face : faces_) {
        const auto material = materials_.index(face.material);
        if (!material) {
            errors_.push_back("face has an invalid material");
            continue;
        }
        facesByMaterial[*material].push_back(&face);
    }
    model_.indices.clear();
    for (std::size_t material = 0; material < model_.materials.size(); ++material) {
        auto &current = model_.materials[material];
        current.indexCount = 0;
        for (const auto *face : facesByMaterial[material]) {
            for (const auto vertex : face->vertices) {
                const auto index = vertices_.index(vertex);
                if (!index) {
                    errors_.push_back("face has an invalid vertex");
                    continue;
                }
                model_.indices.push_back(static_cast<std::uint32_t>(*index));
            }
            current.indexCount += 3;
        }
    }
    auto validation = pmx::validate(model_);
    if (!errors_.empty() || !validation.valid())
        return {false, std::move(validation), std::move(errors_)};
    document_.model_ = std::move(model_);
    document_.vertices_ = std::move(vertices_);
    document_.textures_ = std::move(textures_);
    document_.bones_ = std::move(bones_);
    document_.materials_ = std::move(materials_);
    document_.morphs_ = std::move(morphs_);
    document_.displayFrames_ = std::move(displayFrames_);
    document_.rigidBodies_ = std::move(rigidBodies_);
    document_.joints_ = std::move(joints_);
    document_.softBodies_ = std::move(softBodies_);
    document_.faces_ = std::move(faces_);
    document_.facesTable_ = std::move(facesTable_);
    document_.dirty_ = false;
    document_.rebuildReferences();
    return {true, std::move(validation), {}};
}

} // namespace mmd
