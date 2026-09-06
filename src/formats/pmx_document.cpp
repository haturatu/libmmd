#include <mmd/document.hpp>


namespace mmd {
namespace {

template <typename Tag, typename Value, typename Callback>
bool updateValue(PmxDocument::Table<Tag> &table, std::vector<Value> &values, PmxHandle<Tag> handle, Callback &&callback) {
    const auto index = table.index(handle);
    if (!index)
        return false;
    callback(values[*index]);
    return true;
}

template <typename Handle>
void recordHandle(std::vector<Handle> &handles, Handle handle) {
    if (handle && std::find(handles.begin(), handles.end(), handle) == handles.end())
        handles.push_back(handle);
}

std::int32_t boneIndex(const PmxDocument::Table<BoneTag> &table, std::optional<BoneHandle> value) {
    if (!value)
        return -1;
    const auto index = table.index(*value);
    return index ? static_cast<std::int32_t>(*index) : -2;
}

std::int32_t textureIndex(const PmxDocument::Table<TextureTag> &table, std::optional<TextureHandle> value) {
    if (!value)
        return -1;
    const auto index = table.index(*value);
    return index ? static_cast<std::int32_t>(*index) : -2;
}

std::int32_t materialIndex(const PmxDocument::Table<MaterialTag> &table, std::optional<MaterialHandle> value) {
    if (!value)
        return -1;
    const auto index = table.index(*value);
    return index ? static_cast<std::int32_t>(*index) : -2;
}

std::int32_t rigidBodyIndex(const PmxDocument::Table<RigidBodyTag> &table, RigidBodyHandle value) {
    const auto index = table.index(value);
    return index ? static_cast<std::int32_t>(*index) : -2;
}

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

} // namespace

void PmxDocument::rebuildIndexes() {
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

ValidationResult PmxDocument::validate() const {
    ensure();
    auto result = pmx::validate(model_);
    for (auto &issue : result.issues) {
        const auto assign = [&](ReferenceObjectKind kind, const char *name, auto handle, const char *field) {
            issue.object = name;
            issue.location.kind = kind;
            issue.location.id = handle.id;
            issue.location.generation = handle.generation;
            issue.location.field = field;
        };
        if ((issue.message.find("vertex") != std::string::npos || issue.message.find("QDEF") != std::string::npos ||
             issue.message.find("weight") != std::string::npos || issue.message.find("non-finite") != std::string::npos) &&
            !vertices_.slots.empty())
            assign(ReferenceObjectKind::vertex, "vertex", vertices_.at(0), "vertex");
        else if (issue.message.find("material") != std::string::npos && !materials_.slots.empty())
            assign(ReferenceObjectKind::material, "material", materials_.at(0), "material");
        else if ((issue.message.find("bone") != std::string::npos || issue.message.find("IK") != std::string::npos) &&
                 !bones_.slots.empty())
            assign(ReferenceObjectKind::bone, "bone", bones_.at(0), "bone");
        else if (issue.message.find("morph") != std::string::npos && !morphs_.slots.empty())
            assign(ReferenceObjectKind::morph, "morph", morphs_.at(0), "morph");
        else if (issue.message.find("joint") != std::string::npos && !joints_.slots.empty())
            assign(ReferenceObjectKind::joint, "joint", joints_.at(0), "joint");
        else if (issue.message.find("rigid") != std::string::npos && !rigidBodies_.slots.empty())
            assign(ReferenceObjectKind::rigidBody, "rigidBody", rigidBodies_.at(0), "rigidBody");
        else if (issue.message.find("display") != std::string::npos && !displayFrames_.slots.empty())
            assign(ReferenceObjectKind::displayFrame, "displayFrame", displayFrames_.at(0), "displayFrame");
        else {
            issue.object = "model";
            issue.location.kind = ReferenceObjectKind::model;
        }
    }
    return result;
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

bool PmxDocument::Transaction::setMetadata(PmxMetadata metadata) {
    if (metadata.version < 2.0F || metadata.version > 2.1F || metadata.additionalUvCount > 4)
        return false;
    model_.metadata = std::move(metadata);
    changes_.topologyChanged = true;
    return true;
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
    const auto handle = bones_.insert(destination);
    recordHandle(changes_.bones, handle);
    changes_.topologyChanged = true;
    return handle;
}
bool PmxDocument::Transaction::setBone(BoneHandle handle, const PmxBone &bone) {
    const auto index = bones_.index(handle);
    if (!index)
        return false;
    model_.bones[*index] = bone;
    recordHandle(changes_.bones, handle);
    return true;
}
bool PmxDocument::Transaction::setBoneName(BoneHandle handle, std::string value) {
    return updateValue(bones_, model_.bones, handle, [&](auto &bone) { bone.name = std::move(value); }) &&
           (recordHandle(changes_.bones, handle), true);
}
bool PmxDocument::Transaction::setBoneEnglishName(BoneHandle handle, std::string value) {
    return updateValue(bones_, model_.bones, handle, [&](auto &bone) { bone.englishName = std::move(value); }) &&
           (recordHandle(changes_.bones, handle), true);
}
bool PmxDocument::Transaction::setBonePosition(BoneHandle handle, Float3 value) {
    return updateValue(bones_, model_.bones, handle, [&](auto &bone) { bone.position = value; }) &&
           (recordHandle(changes_.bones, handle), true);
}
bool PmxDocument::Transaction::setBoneTailBone(BoneHandle handle, std::optional<BoneHandle> value) {
    const auto index = boneIndex(bones_, value);
    if (index == -2)
        return false;
    return updateValue(bones_, model_.bones, handle, [&](auto &bone) { bone.tailBone = index; }) &&
           (recordHandle(changes_.bones, handle), true);
}
bool PmxDocument::Transaction::setBoneTailOffset(BoneHandle handle, Float3 value) {
    return updateValue(bones_, model_.bones, handle, [&](auto &bone) { bone.tailOffset = value; }) &&
           (recordHandle(changes_.bones, handle), true);
}
bool PmxDocument::Transaction::setBoneDeformLayer(BoneHandle handle, std::int32_t value) {
    return updateValue(bones_, model_.bones, handle, [&](auto &bone) { bone.deformLayer = value; }) &&
           (recordHandle(changes_.bones, handle), true);
}
bool PmxDocument::Transaction::setBoneFlags(BoneHandle handle, std::uint16_t value) {
    return updateValue(bones_, model_.bones, handle, [&](auto &bone) { bone.flags = value; }) &&
           (recordHandle(changes_.bones, handle), true);
}
bool PmxDocument::Transaction::setBoneInheritParent(BoneHandle handle, std::optional<BoneHandle> value) {
    const auto index = boneIndex(bones_, value);
    if (index == -2)
        return false;
    return updateValue(bones_, model_.bones, handle, [&](auto &bone) { bone.inheritParent = index; }) &&
           (recordHandle(changes_.bones, handle), true);
}
bool PmxDocument::Transaction::setBoneInheritRatio(BoneHandle handle, float value) {
    return updateValue(bones_, model_.bones, handle, [&](auto &bone) { bone.inheritRatio = value; }) &&
           (recordHandle(changes_.bones, handle), true);
}
bool PmxDocument::Transaction::setBoneFixedAxis(BoneHandle handle, Float3 value) {
    return updateValue(bones_, model_.bones, handle, [&](auto &bone) { bone.fixedAxis = value; }) &&
           (recordHandle(changes_.bones, handle), true);
}
bool PmxDocument::Transaction::setBoneLocalAxes(BoneHandle handle, Float3 x, Float3 z) {
    return updateValue(bones_, model_.bones, handle, [&](auto &bone) {
               bone.localAxisX = x;
               bone.localAxisZ = z;
           }) &&
           (recordHandle(changes_.bones, handle), true);
}
bool PmxDocument::Transaction::setBoneExternalParentKey(BoneHandle handle, std::int32_t value) {
    return updateValue(bones_, model_.bones, handle, [&](auto &bone) { bone.externalParentKey = value; }) &&
           (recordHandle(changes_.bones, handle), true);
}
bool PmxDocument::Transaction::setBoneIkTarget(BoneHandle handle, std::optional<BoneHandle> value) {
    const auto index = boneIndex(bones_, value);
    if (index == -2)
        return false;
    return updateValue(bones_, model_.bones, handle, [&](auto &bone) { bone.ikTarget = index; }) &&
           (recordHandle(changes_.bones, handle), true);
}
bool PmxDocument::Transaction::setBoneIkLimits(BoneHandle handle, std::int32_t loops, float angle) {
    return updateValue(bones_, model_.bones, handle, [&](auto &bone) {
               bone.ikLoopCount = loops;
               bone.ikLimitAngle = angle;
           }) &&
           (recordHandle(changes_.bones, handle), true);
}
bool PmxDocument::Transaction::setBoneIkLink(BoneHandle handle, std::size_t index, PmxIkLink value) {
    const auto bone = bones_.index(handle);
    if (!bone || index >= model_.bones[*bone].ikLinks.size())
        return false;
    model_.bones[*bone].ikLinks[index] = value;
    recordHandle(changes_.bones, handle);
    return true;
}
bool PmxDocument::Transaction::addBoneIkLink(BoneHandle handle, PmxIkLink value) {
    return updateValue(bones_, model_.bones, handle, [&](auto &bone) { bone.ikLinks.push_back(value); }) &&
           (recordHandle(changes_.bones, handle), true);
}
bool PmxDocument::Transaction::eraseBoneIkLink(BoneHandle handle, std::size_t index) {
    const auto bone = bones_.index(handle);
    if (!bone || index >= model_.bones[*bone].ikLinks.size())
        return false;
    model_.bones[*bone].ikLinks.erase(model_.bones[*bone].ikLinks.begin() + static_cast<std::ptrdiff_t>(index));
    recordHandle(changes_.bones, handle);
    return true;
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
    recordHandle(changes_.bones, h);
    changes_.topologyChanged = true;
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
    recordHandle(changes_.bones, h);
    changes_.topologyChanged = true;
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
    const auto handle = materials_.append();
    recordHandle(changes_.materials, handle);
    changes_.topologyChanged = true;
    return handle;
}
bool PmxDocument::Transaction::setMaterial(MaterialHandle handle, const PmxMaterial &material) {
    const auto index = materials_.index(handle);
    if (!index)
        return false;
    if (material.indexCount != model_.materials[*index].indexCount) {
        errors_.push_back("material index count is owned by the face graph");
        return false;
    }
    model_.materials[*index] = material;
    recordHandle(changes_.materials, handle);
    return true;
}
bool PmxDocument::Transaction::setMaterialName(MaterialHandle handle, std::string value) {
    return updateValue(materials_, model_.materials, handle, [&](auto &material) { material.name = std::move(value); }) &&
           (recordHandle(changes_.materials, handle), true);
}
bool PmxDocument::Transaction::setMaterialEnglishName(MaterialHandle handle, std::string value) {
    return updateValue(materials_, model_.materials, handle,
                       [&](auto &material) { material.englishName = std::move(value); }) &&
           (recordHandle(changes_.materials, handle), true);
}
bool PmxDocument::Transaction::setMaterialDiffuse(MaterialHandle handle, Float4 value) {
    return updateValue(materials_, model_.materials, handle, [&](auto &material) { material.diffuse = value; }) &&
           (recordHandle(changes_.materials, handle), true);
}
bool PmxDocument::Transaction::setMaterialSpecular(MaterialHandle handle, Float3 value) {
    return updateValue(materials_, model_.materials, handle, [&](auto &material) { material.specular = value; }) &&
           (recordHandle(changes_.materials, handle), true);
}
bool PmxDocument::Transaction::setMaterialShininess(MaterialHandle handle, float value) {
    return updateValue(materials_, model_.materials, handle, [&](auto &material) { material.shininess = value; }) &&
           (recordHandle(changes_.materials, handle), true);
}
bool PmxDocument::Transaction::setMaterialAmbient(MaterialHandle handle, Float3 value) {
    return updateValue(materials_, model_.materials, handle, [&](auto &material) { material.ambient = value; }) &&
           (recordHandle(changes_.materials, handle), true);
}
bool PmxDocument::Transaction::setMaterialDrawFlags(MaterialHandle handle, std::uint8_t value) {
    return updateValue(materials_, model_.materials, handle, [&](auto &material) { material.drawFlags = value; }) &&
           (recordHandle(changes_.materials, handle), true);
}
bool PmxDocument::Transaction::setMaterialEdge(MaterialHandle handle, Float4 color, float size) {
    return updateValue(materials_, model_.materials, handle, [&](auto &material) {
               material.edgeColor = color;
               material.edgeSize = size;
           }) &&
           (recordHandle(changes_.materials, handle), true);
}
bool PmxDocument::Transaction::setMaterialTexture(MaterialHandle handle, std::optional<TextureHandle> value) {
    const auto index = textureIndex(textures_, value);
    if (index == -2)
        return false;
    return updateValue(materials_, model_.materials, handle, [&](auto &material) { material.textureIndex = index; }) &&
           (recordHandle(changes_.materials, handle), changes_.texturesChanged = true, true);
}
bool PmxDocument::Transaction::setMaterialSphereTexture(MaterialHandle handle, std::optional<TextureHandle> value) {
    const auto index = textureIndex(textures_, value);
    if (index == -2)
        return false;
    return updateValue(materials_, model_.materials, handle,
                       [&](auto &material) { material.sphereTextureIndex = index; }) &&
           (recordHandle(changes_.materials, handle), changes_.texturesChanged = true, true);
}
bool PmxDocument::Transaction::setMaterialToonTexture(MaterialHandle handle, std::optional<TextureHandle> value) {
    const auto index = textureIndex(textures_, value);
    if (index == -2)
        return false;
    return updateValue(materials_, model_.materials, handle,
                       [&](auto &material) { material.toonTextureIndex = index; }) &&
           (recordHandle(changes_.materials, handle), changes_.texturesChanged = true, true);
}
bool PmxDocument::Transaction::setMaterialSphereMode(MaterialHandle handle, std::uint8_t value) {
    return updateValue(materials_, model_.materials, handle, [&](auto &material) { material.sphereMode = value; }) &&
           (recordHandle(changes_.materials, handle), true);
}
bool PmxDocument::Transaction::setMaterialToonMode(MaterialHandle handle, std::uint8_t value) {
    if (value > 1)
        return false;
    return updateValue(materials_, model_.materials, handle, [&](auto &material) { material.toonMode = value; }) &&
           (recordHandle(changes_.materials, handle), true);
}
bool PmxDocument::Transaction::setMaterialMemo(MaterialHandle handle, std::string value) {
    return updateValue(materials_, model_.materials, handle, [&](auto &material) { material.memo = std::move(value); }) &&
           (recordHandle(changes_.materials, handle), true);
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
    recordHandle(changes_.materials, h);
    changes_.topologyChanged = true;
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
    recordHandle(changes_.materials, h);
    changes_.topologyChanged = true;
    return true;
}
VertexHandle PmxDocument::Transaction::addVertex(PmxVertex vertex) {
    if (done_)
        return {};
    model_.vertices.push_back(std::move(vertex));
    const auto handle = vertices_.append();
    recordHandle(changes_.vertices, handle);
    changes_.topologyChanged = true;
    return handle;
}
bool PmxDocument::Transaction::setVertex(VertexHandle handle, const PmxVertex &vertex) {
    const auto index = vertices_.index(handle);
    if (!index)
        return false;
    model_.vertices[*index] = vertex;
    recordHandle(changes_.vertices, handle);
    return true;
}
bool PmxDocument::Transaction::setVertexPosition(VertexHandle handle, Float3 value) {
    return updateValue(vertices_, model_.vertices, handle, [&](auto &vertex) { vertex.position = value; }) &&
           (recordHandle(changes_.vertices, handle), true);
}
bool PmxDocument::Transaction::setVertexNormal(VertexHandle handle, Float3 value) {
    return updateValue(vertices_, model_.vertices, handle, [&](auto &vertex) { vertex.normal = value; }) &&
           (recordHandle(changes_.vertices, handle), true);
}
bool PmxDocument::Transaction::setVertexUv(VertexHandle handle, Float2 value) {
    return updateValue(vertices_, model_.vertices, handle, [&](auto &vertex) { vertex.uv = value; }) &&
           (recordHandle(changes_.vertices, handle), true);
}
bool PmxDocument::Transaction::setVertexAdditionalUv(VertexHandle handle, std::uint32_t channel, Float4 value) {
    if (channel >= 4)
        return false;
    return updateValue(vertices_, model_.vertices, handle,
                       [&](auto &vertex) { vertex.additionalUv[channel] = value; }) &&
           (recordHandle(changes_.vertices, handle), true);
}
bool PmxDocument::Transaction::setVertexEdgeScale(VertexHandle handle, float value) {
    return updateValue(vertices_, model_.vertices, handle, [&](auto &vertex) { vertex.edgeScale = value; }) &&
           (recordHandle(changes_.vertices, handle), true);
}
bool PmxDocument::Transaction::setVertexSkin(VertexHandle handle, const PmxVertex &value) {
    return setVertex(handle, value);
}
bool PmxDocument::Transaction::setVertexSkin(VertexHandle handle, const PmxVertexSkin &value) {
    PmxVertex vertex;
    vertex.weightType = value.type;
    vertex.weights = value.weights;
    vertex.sdefC = value.sdefC;
    vertex.sdefR0 = value.sdefR0;
    vertex.sdefR1 = value.sdefR1;
    const auto count = value.type == PmxWeightType::bdef1
                           ? std::size_t{1}
                           : (value.type == PmxWeightType::bdef2 || value.type == PmxWeightType::sdef ? std::size_t{2}
                                                                                                      : std::size_t{4});
    for (std::size_t i = 0; i < count; ++i) {
        const auto index = bones_.index(value.bones[i]);
        if (!index)
            return false;
        vertex.bones[i] = static_cast<std::int32_t>(*index);
    }
    const auto current = vertices_.index(handle);
    if (!current)
        return false;
    vertex.position = model_.vertices[*current].position;
    vertex.normal = model_.vertices[*current].normal;
    vertex.uv = model_.vertices[*current].uv;
    vertex.additionalUv = model_.vertices[*current].additionalUv;
    vertex.edgeScale = model_.vertices[*current].edgeScale;
    return setVertex(handle, vertex);
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
    recordHandle(changes_.vertices, h);
    changes_.topologyChanged = true;
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
    recordHandle(changes_.vertices, h);
    changes_.topologyChanged = true;
    return true;
}
FaceHandle PmxDocument::Transaction::addFace(VertexHandle a, VertexHandle b, VertexHandle c, MaterialHandle material) {
    if (done_ || !vertices_.index(a) || !vertices_.index(b) || !vertices_.index(c) || !materials_.index(material))
        return {};
    faces_.push_back({{a, b, c}, material});
    const auto handle = facesTable_.append();
    changes_.topologyChanged = true;
    return handle;
}
bool PmxDocument::Transaction::eraseFace(FaceHandle h) {
    const auto index = facesTable_.index(h);
    if (!index)
        return false;
    faces_.erase(faces_.begin() + static_cast<std::ptrdiff_t>(*index));
    facesTable_.slots.erase(facesTable_.slots.begin() + static_cast<std::ptrdiff_t>(*index));
    changes_.topologyChanged = true;
    return true;
}
bool PmxDocument::Transaction::setFaceMaterial(FaceHandle h, MaterialHandle material) {
    const auto index = facesTable_.index(h);
    if (!index || !materials_.index(material))
        return false;
    faces_[*index].material = material;
    recordHandle(changes_.materials, material);
    changes_.topologyChanged = true;
    return true;
}
MorphHandle PmxDocument::Transaction::addMorph(PmxMorph morph) {
    if (done_)
        return {};
    model_.morphs.push_back(std::move(morph));
    const auto handle = morphs_.append();
    recordHandle(changes_.morphs, handle);
    changes_.topologyChanged = true;
    return handle;
}
bool PmxDocument::Transaction::setMorph(MorphHandle handle, const PmxMorph &morph) {
    const auto index = morphs_.index(handle);
    if (!index)
        return false;
    model_.morphs[*index] = morph;
    recordHandle(changes_.morphs, handle);
    return true;
}
bool PmxDocument::Transaction::setMorphName(MorphHandle handle, std::string value) {
    return updateValue(morphs_, model_.morphs, handle, [&](auto &morph) { morph.name = std::move(value); }) &&
           (recordHandle(changes_.morphs, handle), true);
}
bool PmxDocument::Transaction::setMorphEnglishName(MorphHandle handle, std::string value) {
    return updateValue(morphs_, model_.morphs, handle,
                       [&](auto &morph) { morph.englishName = std::move(value); }) &&
           (recordHandle(changes_.morphs, handle), true);
}
bool PmxDocument::Transaction::setMorphPanel(MorphHandle handle, std::uint8_t value) {
    return updateValue(morphs_, model_.morphs, handle, [&](auto &morph) { morph.panel = value; }) &&
           (recordHandle(changes_.morphs, handle), true);
}
bool PmxDocument::Transaction::setMorphType(MorphHandle handle, std::uint8_t value) {
    return updateValue(morphs_, model_.morphs, handle, [&](auto &morph) { morph.type = value; }) &&
           (recordHandle(changes_.morphs, handle), true);
}
bool PmxDocument::Transaction::setMorphOffset(MorphHandle handle, std::size_t index, PmxMorphOffset value) {
    const auto morph = morphs_.index(handle);
    if (!morph || index >= model_.morphs[*morph].offsets.size())
        return false;
    model_.morphs[*morph].offsets[index] = value;
    recordHandle(changes_.morphs, handle);
    return true;
}
bool PmxDocument::Transaction::setVertexMorphOffset(MorphHandle handle, std::size_t offsetIndex, VertexHandle vertex,
                                                      Float3 value) {
    const auto morph = morphs_.index(handle);
    const auto index = vertices_.index(vertex);
    if (!morph || !index || model_.morphs[*morph].type != 1)
        return false;
    PmxMorphOffset valueData;
    valueData.index = static_cast<std::int32_t>(*index);
    valueData.vector3 = value;
    return setMorphOffset(handle, offsetIndex, valueData);
}
bool PmxDocument::Transaction::setBoneMorphOffset(MorphHandle handle, std::size_t offsetIndex, BoneHandle bone,
                                                    Float3 translation, Float4 rotation) {
    const auto morph = morphs_.index(handle);
    const auto index = bones_.index(bone);
    if (!morph || !index || model_.morphs[*morph].type != 2)
        return false;
    PmxMorphOffset valueData;
    valueData.index = static_cast<std::int32_t>(*index);
    valueData.vector3 = translation;
    valueData.vector4 = rotation;
    return setMorphOffset(handle, offsetIndex, valueData);
}
bool PmxDocument::Transaction::setGroupMorphOffset(MorphHandle handle, std::size_t offsetIndex, MorphHandle target,
                                                     float weight) {
    const auto morph = morphs_.index(handle);
    const auto index = morphs_.index(target);
    if (!morph || !index || model_.morphs[*morph].type != 0)
        return false;
    PmxMorphOffset valueData;
    valueData.index = static_cast<std::int32_t>(*index);
    valueData.scalar = weight;
    return setMorphOffset(handle, offsetIndex, valueData);
}
bool PmxDocument::Transaction::setUvMorphOffset(MorphHandle handle, std::size_t offsetIndex, VertexHandle vertex,
                                                  std::uint32_t channel, Float4 value) {
    const auto morph = morphs_.index(handle);
    const auto index = vertices_.index(vertex);
    if (!morph || !index || channel > 4 || model_.morphs[*morph].type != channel + 3U)
        return false;
    PmxMorphOffset valueData;
    valueData.index = static_cast<std::int32_t>(*index);
    valueData.vector4 = value;
    return setMorphOffset(handle, offsetIndex, valueData);
}
bool PmxDocument::Transaction::setMaterialMorphOffset(MorphHandle handle, std::size_t offsetIndex,
                                                        std::optional<MaterialHandle> material, std::uint8_t operation,
                                                        std::array<Float4, 8> values) {
    const auto morph = morphs_.index(handle);
    const auto index = materialIndex(materials_, material);
    if (!morph || index == -2 || operation > 1 || model_.morphs[*morph].type != 8)
        return false;
    PmxMorphOffset valueData;
    valueData.index = index;
    valueData.operation = operation;
    valueData.materialVectors = values;
    return setMorphOffset(handle, offsetIndex, valueData);
}
bool PmxDocument::Transaction::setFlipMorphOffset(MorphHandle handle, std::size_t offsetIndex, MorphHandle target,
                                                   float weight) {
    const auto morph = morphs_.index(handle);
    const auto index = morphs_.index(target);
    if (!morph || !index || model_.morphs[*morph].type != 9)
        return false;
    PmxMorphOffset valueData;
    valueData.index = static_cast<std::int32_t>(*index);
    valueData.scalar = weight;
    return setMorphOffset(handle, offsetIndex, valueData);
}
bool PmxDocument::Transaction::setImpulseMorphOffset(MorphHandle handle, std::size_t offsetIndex,
                                                      RigidBodyHandle body, Float3 velocity, Float3 torque, bool local) {
    const auto morph = morphs_.index(handle);
    const auto index = rigidBodies_.index(body);
    if (!morph || !index || model_.morphs[*morph].type != 10)
        return false;
    PmxMorphOffset valueData;
    valueData.index = static_cast<std::int32_t>(*index);
    valueData.vector3 = velocity;
    valueData.tertiaryVector3 = torque;
    valueData.local = local;
    return setMorphOffset(handle, offsetIndex, valueData);
}
bool PmxDocument::Transaction::addMorphOffset(MorphHandle handle, PmxMorphOffset value) {
    return updateValue(morphs_, model_.morphs, handle, [&](auto &morph) { morph.offsets.push_back(value); }) &&
           (recordHandle(changes_.morphs, handle), true);
}
bool PmxDocument::Transaction::addVertexMorphOffset(MorphHandle handle, VertexHandle vertex, Float3 value) {
    const auto morph = morphs_.index(handle);
    const auto index = vertices_.index(vertex);
    if (!morph || !index || model_.morphs[*morph].type != 1)
        return false;
    PmxMorphOffset offset;
    offset.index = static_cast<std::int32_t>(*index);
    offset.vector3 = value;
    return addMorphOffset(handle, offset);
}
bool PmxDocument::Transaction::addBoneMorphOffset(MorphHandle handle, BoneHandle bone, Float3 translation,
                                                   Float4 rotation) {
    const auto morph = morphs_.index(handle);
    const auto index = bones_.index(bone);
    if (!morph || !index || model_.morphs[*morph].type != 2)
        return false;
    PmxMorphOffset offset;
    offset.index = static_cast<std::int32_t>(*index);
    offset.vector3 = translation;
    offset.vector4 = rotation;
    return addMorphOffset(handle, offset);
}
bool PmxDocument::Transaction::addGroupMorphOffset(MorphHandle handle, MorphHandle target, float weight) {
    const auto morph = morphs_.index(handle);
    const auto index = morphs_.index(target);
    if (!morph || !index || model_.morphs[*morph].type != 0)
        return false;
    PmxMorphOffset offset;
    offset.index = static_cast<std::int32_t>(*index);
    offset.scalar = weight;
    return addMorphOffset(handle, offset);
}
bool PmxDocument::Transaction::addUvMorphOffset(MorphHandle handle, VertexHandle vertex, std::uint32_t channel,
                                                 Float4 value) {
    const auto morph = morphs_.index(handle);
    const auto index = vertices_.index(vertex);
    if (!morph || !index || channel > 4 || model_.morphs[*morph].type != channel + 3U)
        return false;
    PmxMorphOffset offset;
    offset.index = static_cast<std::int32_t>(*index);
    offset.vector4 = value;
    return addMorphOffset(handle, offset);
}
bool PmxDocument::Transaction::addMaterialMorphOffset(MorphHandle handle, std::optional<MaterialHandle> material,
                                                       std::uint8_t operation, std::array<Float4, 8> values) {
    const auto morph = morphs_.index(handle);
    const auto index = materialIndex(materials_, material);
    if (!morph || index == -2 || operation > 1 || model_.morphs[*morph].type != 8)
        return false;
    PmxMorphOffset offset;
    offset.index = index;
    offset.operation = operation;
    offset.materialVectors = values;
    return addMorphOffset(handle, offset);
}
bool PmxDocument::Transaction::addFlipMorphOffset(MorphHandle handle, MorphHandle target, float weight) {
    const auto morph = morphs_.index(handle);
    const auto index = morphs_.index(target);
    if (!morph || !index || model_.morphs[*morph].type != 9)
        return false;
    PmxMorphOffset offset;
    offset.index = static_cast<std::int32_t>(*index);
    offset.scalar = weight;
    return addMorphOffset(handle, offset);
}
bool PmxDocument::Transaction::addImpulseMorphOffset(MorphHandle handle, RigidBodyHandle body, Float3 velocity,
                                                      Float3 torque, bool local) {
    const auto morph = morphs_.index(handle);
    const auto index = rigidBodies_.index(body);
    if (!morph || !index || model_.morphs[*morph].type != 10)
        return false;
    PmxMorphOffset offset;
    offset.index = static_cast<std::int32_t>(*index);
    offset.vector3 = velocity;
    offset.tertiaryVector3 = torque;
    offset.local = local;
    return addMorphOffset(handle, offset);
}
bool PmxDocument::Transaction::eraseMorphOffset(MorphHandle handle, std::size_t index) {
    const auto morph = morphs_.index(handle);
    if (!morph || index >= model_.morphs[*morph].offsets.size())
        return false;
    model_.morphs[*morph].offsets.erase(model_.morphs[*morph].offsets.begin() + static_cast<std::ptrdiff_t>(index));
    recordHandle(changes_.morphs, handle);
    return true;
}
bool PmxDocument::Transaction::moveMorphOffset(MorphHandle handle, std::size_t from, std::size_t to) {
    const auto morph = morphs_.index(handle);
    if (!morph || from >= model_.morphs[*morph].offsets.size() || to >= model_.morphs[*morph].offsets.size())
        return false;
    if (from == to)
        return true;
    auto &offsets = model_.morphs[*morph].offsets;
    auto value = std::move(offsets[from]);
    offsets.erase(offsets.begin() + static_cast<std::ptrdiff_t>(from));
    offsets.insert(offsets.begin() + static_cast<std::ptrdiff_t>(to), std::move(value));
    recordHandle(changes_.morphs, handle);
    return true;
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
    recordHandle(changes_.morphs, h);
    changes_.topologyChanged = true;
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
    recordHandle(changes_.morphs, h);
    changes_.topologyChanged = true;
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
    const auto handle = textures_.append();
    recordHandle(changes_.textures, handle);
    changes_.texturesChanged = true;
    changes_.topologyChanged = true;
    return handle;
}
bool PmxDocument::Transaction::setTexture(TextureHandle handle, const PmxTexture &texture) {
    const auto index = textures_.index(handle);
    if (!index)
        return false;
    model_.textures[*index] = texture;
    recordHandle(changes_.textures, handle);
    changes_.texturesChanged = true;
    return true;
}
bool PmxDocument::Transaction::setTexturePath(TextureHandle handle, std::string value) {
    return updateValue(textures_, model_.textures, handle, [&](auto &texture) { texture.storedPath = std::move(value); }) &&
           (recordHandle(changes_.textures, handle), changes_.texturesChanged = true, true);
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
    recordHandle(changes_.textures, h);
    changes_.texturesChanged = true;
    changes_.topologyChanged = true;
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
    recordHandle(changes_.textures, h);
    changes_.texturesChanged = true;
    changes_.topologyChanged = true;
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
    const auto handle = rigidBodies_.append();
    recordHandle(changes_.rigidBodies, handle);
    changes_.physicsChanged = true;
    changes_.topologyChanged = true;
    return handle;
}
bool PmxDocument::Transaction::setRigidBody(RigidBodyHandle handle, const PmxRigidBody &body) {
    const auto index = rigidBodies_.index(handle);
    if (!index)
        return false;
    model_.rigidBodies[*index] = body;
    recordHandle(changes_.rigidBodies, handle);
    changes_.physicsChanged = true;
    return true;
}
bool PmxDocument::Transaction::setRigidBodyBone(RigidBodyHandle handle, std::optional<BoneHandle> value) {
    const auto index = boneIndex(bones_, value);
    if (index == -2)
        return false;
    return updateValue(rigidBodies_, model_.rigidBodies, handle, [&](auto &body) { body.bone = index; }) &&
           (recordHandle(changes_.rigidBodies, handle), changes_.physicsChanged = true, true);
}
bool PmxDocument::Transaction::setRigidBodyShape(RigidBodyHandle handle, std::uint8_t shape, Float3 size) {
    return updateValue(rigidBodies_, model_.rigidBodies, handle, [&](auto &body) {
               body.shape = shape;
               body.size = size;
           }) &&
           (recordHandle(changes_.rigidBodies, handle), changes_.physicsChanged = true, true);
}
bool PmxDocument::Transaction::setRigidBodyTransform(RigidBodyHandle handle, Float3 position, Float3 rotation) {
    return updateValue(rigidBodies_, model_.rigidBodies, handle, [&](auto &body) {
               body.position = position;
               body.rotation = rotation;
           }) &&
           (recordHandle(changes_.rigidBodies, handle), changes_.physicsChanged = true, true);
}
bool PmxDocument::Transaction::setRigidBodyPhysical(RigidBodyHandle handle, float mass, float linearDamping,
                                                     float angularDamping, float restitution, float friction) {
    return updateValue(rigidBodies_, model_.rigidBodies, handle, [&](auto &body) {
               body.mass = mass;
               body.linearDamping = linearDamping;
               body.angularDamping = angularDamping;
               body.restitution = restitution;
               body.friction = friction;
           }) &&
           (recordHandle(changes_.rigidBodies, handle), changes_.physicsChanged = true, true);
}
bool PmxDocument::Transaction::setRigidBodyCollision(RigidBodyHandle handle, std::uint8_t group, std::uint16_t mask) {
    return updateValue(rigidBodies_, model_.rigidBodies, handle, [&](auto &body) {
               body.group = group;
               body.collisionMask = mask;
           }) &&
           (recordHandle(changes_.rigidBodies, handle), changes_.physicsChanged = true, true);
}
bool PmxDocument::Transaction::setRigidBodyMode(RigidBodyHandle handle, std::uint8_t mode) {
    return updateValue(rigidBodies_, model_.rigidBodies, handle, [&](auto &body) { body.mode = mode; }) &&
           (recordHandle(changes_.rigidBodies, handle), changes_.physicsChanged = true, true);
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
    recordHandle(changes_.rigidBodies, h);
    changes_.physicsChanged = true;
    changes_.topologyChanged = true;
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
    recordHandle(changes_.rigidBodies, h);
    changes_.physicsChanged = true;
    changes_.topologyChanged = true;
    return true;
}
JointHandle PmxDocument::Transaction::addJoint(PmxJoint joint) {
    if (done_)
        return {};
    model_.joints.push_back(std::move(joint));
    const auto handle = joints_.append();
    recordHandle(changes_.joints, handle);
    changes_.physicsChanged = true;
    changes_.topologyChanged = true;
    return handle;
}
bool PmxDocument::Transaction::setJoint(JointHandle handle, const PmxJoint &joint) {
    const auto index = joints_.index(handle);
    if (!index)
        return false;
    model_.joints[*index] = joint;
    recordHandle(changes_.joints, handle);
    changes_.physicsChanged = true;
    return true;
}
bool PmxDocument::Transaction::setJointBodies(JointHandle handle, RigidBodyHandle bodyA, RigidBodyHandle bodyB) {
    const auto a = rigidBodyIndex(rigidBodies_, bodyA);
    const auto b = rigidBodyIndex(rigidBodies_, bodyB);
    if (a == -2 || b == -2)
        return false;
    return updateValue(joints_, model_.joints, handle, [&](auto &joint) {
               joint.bodyA = a;
               joint.bodyB = b;
           }) &&
           (recordHandle(changes_.joints, handle), changes_.physicsChanged = true, true);
}
bool PmxDocument::Transaction::setJointTransform(JointHandle handle, Float3 position, Float3 rotation) {
    return updateValue(joints_, model_.joints, handle, [&](auto &joint) {
               joint.position = position;
               joint.rotation = rotation;
           }) &&
           (recordHandle(changes_.joints, handle), changes_.physicsChanged = true, true);
}
bool PmxDocument::Transaction::setJointLimits(JointHandle handle, Float3 translationMinimum, Float3 translationMaximum,
                                              Float3 rotationMinimum, Float3 rotationMaximum) {
    return updateValue(joints_, model_.joints, handle, [&](auto &joint) {
               joint.translationMinimum = translationMinimum;
               joint.translationMaximum = translationMaximum;
               joint.rotationMinimum = rotationMinimum;
               joint.rotationMaximum = rotationMaximum;
           }) &&
           (recordHandle(changes_.joints, handle), changes_.physicsChanged = true, true);
}
bool PmxDocument::Transaction::setJointSprings(JointHandle handle, Float3 translation, Float3 rotation) {
    return updateValue(joints_, model_.joints, handle, [&](auto &joint) {
               joint.translationSpring = translation;
               joint.rotationSpring = rotation;
           }) &&
           (recordHandle(changes_.joints, handle), changes_.physicsChanged = true, true);
}
bool PmxDocument::Transaction::eraseJoint(JointHandle h) {
    const auto target = joints_.index(h);
    if (!target)
        return false;
    model_.joints.erase(model_.joints.begin() + static_cast<std::ptrdiff_t>(*target));
    joints_.slots.erase(joints_.slots.begin() + static_cast<std::ptrdiff_t>(*target));
    recordHandle(changes_.joints, h);
    changes_.physicsChanged = true;
    changes_.topologyChanged = true;
    return true;
}
DisplayFrameHandle PmxDocument::Transaction::addDisplayFrame(PmxDisplayFrame frame) {
    if (done_)
        return {};
    model_.displayFrames.push_back(std::move(frame));
    const auto handle = displayFrames_.append();
    recordHandle(changes_.displayFrames, handle);
    changes_.topologyChanged = true;
    return handle;
}
bool PmxDocument::Transaction::setDisplayFrame(DisplayFrameHandle handle, const PmxDisplayFrame &frame) {
    const auto index = displayFrames_.index(handle);
    if (!index)
        return false;
    model_.displayFrames[*index] = frame;
    recordHandle(changes_.displayFrames, handle);
    return true;
}
bool PmxDocument::Transaction::setDisplayFrameName(DisplayFrameHandle handle, std::string value) {
    return updateValue(displayFrames_, model_.displayFrames, handle,
                       [&](auto &frame) { frame.name = std::move(value); }) &&
           (recordHandle(changes_.displayFrames, handle), true);
}
bool PmxDocument::Transaction::setDisplayFrameEnglishName(DisplayFrameHandle handle, std::string value) {
    return updateValue(displayFrames_, model_.displayFrames, handle,
                       [&](auto &frame) { frame.englishName = std::move(value); }) &&
           (recordHandle(changes_.displayFrames, handle), true);
}
bool PmxDocument::Transaction::setDisplayFrameItem(DisplayFrameHandle handle, std::size_t index, PmxDisplayItem value) {
    const auto frame = displayFrames_.index(handle);
    if (!frame || index >= model_.displayFrames[*frame].items.size())
        return false;
    model_.displayFrames[*frame].items[index] = value;
    recordHandle(changes_.displayFrames, handle);
    return true;
}
bool PmxDocument::Transaction::setDisplayFrameItem(DisplayFrameHandle handle, std::size_t index, BoneHandle bone) {
    const auto frame = displayFrames_.index(handle);
    const auto target = bones_.index(bone);
    if (!frame || !target || index >= model_.displayFrames[*frame].items.size())
        return false;
    return setDisplayFrameItem(handle, index, PmxDisplayItem{true, static_cast<std::int32_t>(*target)});
}
bool PmxDocument::Transaction::setDisplayFrameItem(DisplayFrameHandle handle, std::size_t index, MorphHandle morph) {
    const auto frame = displayFrames_.index(handle);
    const auto target = morphs_.index(morph);
    if (!frame || !target || index >= model_.displayFrames[*frame].items.size())
        return false;
    return setDisplayFrameItem(handle, index, PmxDisplayItem{false, static_cast<std::int32_t>(*target)});
}
bool PmxDocument::Transaction::addDisplayFrameItem(DisplayFrameHandle handle, PmxDisplayItem value) {
    return updateValue(displayFrames_, model_.displayFrames, handle, [&](auto &frame) { frame.items.push_back(value); }) &&
           (recordHandle(changes_.displayFrames, handle), true);
}
bool PmxDocument::Transaction::addDisplayFrameItem(DisplayFrameHandle handle, BoneHandle bone) {
    const auto index = bones_.index(bone);
    return index && addDisplayFrameItem(handle, PmxDisplayItem{true, static_cast<std::int32_t>(*index)});
}
bool PmxDocument::Transaction::addDisplayFrameItem(DisplayFrameHandle handle, MorphHandle morph) {
    const auto index = morphs_.index(morph);
    return index && addDisplayFrameItem(handle, PmxDisplayItem{false, static_cast<std::int32_t>(*index)});
}
bool PmxDocument::Transaction::eraseDisplayFrameItem(DisplayFrameHandle handle, std::size_t index) {
    const auto frame = displayFrames_.index(handle);
    if (!frame || index >= model_.displayFrames[*frame].items.size())
        return false;
    model_.displayFrames[*frame].items.erase(model_.displayFrames[*frame].items.begin() + static_cast<std::ptrdiff_t>(index));
    recordHandle(changes_.displayFrames, handle);
    return true;
}
bool PmxDocument::Transaction::moveDisplayFrameItem(DisplayFrameHandle handle, std::size_t from, std::size_t to) {
    const auto frame = displayFrames_.index(handle);
    if (!frame || from >= model_.displayFrames[*frame].items.size() || to >= model_.displayFrames[*frame].items.size())
        return false;
    auto &items = model_.displayFrames[*frame].items;
    auto value = items[from];
    items.erase(items.begin() + static_cast<std::ptrdiff_t>(from));
    items.insert(items.begin() + static_cast<std::ptrdiff_t>(to), value);
    recordHandle(changes_.displayFrames, handle);
    return true;
}
bool PmxDocument::Transaction::moveDisplayFrame(DisplayFrameHandle handle, std::size_t destination) {
    const auto source = displayFrames_.index(handle);
    if (!source || destination >= model_.displayFrames.size())
        return false;
    if (*source == destination)
        return true;
    auto value = std::move(model_.displayFrames[*source]);
    model_.displayFrames.erase(model_.displayFrames.begin() + static_cast<std::ptrdiff_t>(*source));
    model_.displayFrames.insert(model_.displayFrames.begin() + static_cast<std::ptrdiff_t>(destination), std::move(value));
    auto slot = displayFrames_.slots[*source];
    displayFrames_.slots.erase(displayFrames_.slots.begin() + static_cast<std::ptrdiff_t>(*source));
    displayFrames_.slots.insert(displayFrames_.slots.begin() + static_cast<std::ptrdiff_t>(destination), slot);
    recordHandle(changes_.displayFrames, handle);
    changes_.topologyChanged = true;
    return true;
}
bool PmxDocument::Transaction::eraseDisplayFrame(DisplayFrameHandle h) {
    const auto target = displayFrames_.index(h);
    if (!target)
        return false;
    model_.displayFrames.erase(model_.displayFrames.begin() + static_cast<std::ptrdiff_t>(*target));
    displayFrames_.slots.erase(displayFrames_.slots.begin() + static_cast<std::ptrdiff_t>(*target));
    recordHandle(changes_.displayFrames, h);
    changes_.topologyChanged = true;
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
    const auto handle = softBodies_.append();
    recordHandle(changes_.softBodies, handle);
    changes_.physicsChanged = true;
    changes_.topologyChanged = true;
    return handle;
}
bool PmxDocument::Transaction::setSoftBody(SoftBodyHandle handle, const PmxSoftBody &body) {
    const auto index = softBodies_.index(handle);
    if (!index)
        return false;
    model_.softBodies[*index] = body;
    recordHandle(changes_.softBodies, handle);
    changes_.physicsChanged = true;
    return true;
}
bool PmxDocument::Transaction::setSoftBodyMaterial(SoftBodyHandle handle, std::optional<MaterialHandle> value) {
    const auto index = materialIndex(materials_, value);
    if (index == -2)
        return false;
    return updateValue(softBodies_, model_.softBodies, handle, [&](auto &body) { body.material = index; }) &&
           (recordHandle(changes_.softBodies, handle), changes_.physicsChanged = true, true);
}
bool PmxDocument::Transaction::setSoftBodyAnchors(SoftBodyHandle handle, std::vector<PmxSoftBodyAnchor> value) {
    return updateValue(softBodies_, model_.softBodies, handle,
                       [&](auto &body) { body.anchors = std::move(value); }) &&
           (recordHandle(changes_.softBodies, handle), changes_.physicsChanged = true, true);
}
bool PmxDocument::Transaction::setSoftBodyPinnedVertices(SoftBodyHandle handle, std::vector<std::int32_t> value) {
    return updateValue(softBodies_, model_.softBodies, handle,
                       [&](auto &body) { body.pinnedVertices = std::move(value); }) &&
           (recordHandle(changes_.softBodies, handle), changes_.physicsChanged = true, true);
}
bool PmxDocument::Transaction::eraseSoftBody(SoftBodyHandle h) {
    const auto target = softBodies_.index(h);
    if (!target)
        return false;
    model_.softBodies.erase(model_.softBodies.begin() + static_cast<std::ptrdiff_t>(*target));
    softBodies_.slots.erase(softBodies_.slots.begin() + static_cast<std::ptrdiff_t>(*target));
    recordHandle(changes_.softBodies, h);
    changes_.physicsChanged = true;
    changes_.topologyChanged = true;
    return true;
}
PmxTransactionResult PmxDocument::Transaction::commit() {
    if (done_)
        return {false, {}, {"transaction has already finished"}, {}};
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
    return {true, std::move(validation), {}, std::move(changes_)};
}

} // namespace mmd
