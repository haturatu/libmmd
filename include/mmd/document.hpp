#pragma once

#include <mmd/pmx.hpp>

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace mmd {

template <typename Tag> struct PmxHandle {
    std::uint64_t id{};
    std::uint32_t generation{};
    [[nodiscard]] explicit constexpr operator bool() const noexcept {
        return id != 0;
    }
    auto operator<=>(const PmxHandle &) const = default;
};
struct VertexTag {};
struct TextureTag {};
struct MaterialTag {};
struct BoneTag {};
struct MorphTag {};
struct DisplayFrameTag {};
struct RigidBodyTag {};
struct JointTag {};
struct SoftBodyTag {};
struct FaceTag {};
using VertexHandle = PmxHandle<VertexTag>;
using TextureHandle = PmxHandle<TextureTag>;
using MaterialHandle = PmxHandle<MaterialTag>;
using BoneHandle = PmxHandle<BoneTag>;
using MorphHandle = PmxHandle<MorphTag>;
using DisplayFrameHandle = PmxHandle<DisplayFrameTag>;
using RigidBodyHandle = PmxHandle<RigidBodyTag>;
using JointHandle = PmxHandle<JointTag>;
using SoftBodyHandle = PmxHandle<SoftBodyTag>;
using FaceHandle = PmxHandle<FaceTag>;

enum class ReferenceObjectKind : std::uint8_t {
    vertex,
    material,
    bone,
    morph,
    displayFrame,
    rigidBody,
    joint,
    softBody,
    face
};
// ReferenceIndex stores actual target edges. A missing entry represents an
// optional PMX reference with no target; `all` is the material-morph selector.
enum class ReferenceTargetKind : std::uint8_t { object, all };
enum class ReferenceField : std::uint8_t {
    vertexBone,
    materialTexture,
    materialSphereTexture,
    materialToonTexture,
    boneParent,
    boneTail,
    boneInheritParent,
    boneIkTarget,
    boneIkLink,
    morphOffset,
    displayItem,
    rigidBodyBone,
    jointBodyA,
    jointBodyB,
    softBodyMaterial,
    softBodyAnchorRigidBody,
    softBodyAnchorVertex,
    softBodyPinnedVertex,
    faceVertex,
    faceMaterial,
};
// Identifies a graph edge without storing a vector-invalidated pointer.
struct ReferenceSite {
    ReferenceObjectKind ownerKind{};
    std::uint64_t ownerId{};
    std::uint32_t ownerGeneration{};
    ReferenceField field{};
    std::uint32_t subIndex{};
    ReferenceTargetKind targetKind{ReferenceTargetKind::object};
};
struct PmxFace {
    VertexHandle vertices[3]{};
    MaterialHandle material{};
};
enum class ErasePolicy : std::uint8_t { rejectIfReferenced };
struct EraseImpact {
    std::size_t vertexWeights{}, childBones{}, ikLinks{}, morphOffsets{}, displayEntries{}, rigidBodies{};
    [[nodiscard]] std::size_t total() const noexcept {
        return vertexWeights + childBones + ikLinks + morphOffsets + displayEntries + rigidBodies;
    }
};
struct VertexEraseImpact {
    std::size_t faces{}, morphOffsets{}, softBodyAnchors{}, pinnedVertices{};
    [[nodiscard]] std::size_t total() const noexcept {
        return faces + morphOffsets + softBodyAnchors + pinnedVertices;
    }
};
struct BoneIkLinkDraft {
    BoneHandle bone;
    bool limited{};
    Float3 minimum{};
    Float3 maximum{};
};
struct BoneDraft {
    PmxBone value;
    std::optional<BoneHandle> parent;
    std::optional<BoneHandle> tailBone;
    std::optional<BoneHandle> inheritParent;
    std::optional<BoneHandle> ikTarget;
    std::vector<BoneIkLinkDraft> ikLinks;
    // All bone-index fields in value are ignored and rebuilt from these
    // handles. The reference-related bits in value.flags must agree with the
    // optional handles and links.
};
struct RigidBodyDraft {
    PmxRigidBody value;
    std::optional<BoneHandle> bone;
};
struct JointDraft {
    PmxJoint value;
    RigidBodyHandle bodyA;
    RigidBodyHandle bodyB;
};
struct SoftBodyDraft {
    PmxSoftBody value;
    std::optional<MaterialHandle> material;
};
struct PmxTransactionResult {
    bool committed{};
    ValidationResult validation;
    std::vector<std::string> errors;
};

class PmxDocument {
  public:
    template <typename Tag> struct Slot {
        std::uint64_t id{};
        std::uint32_t generation{1};
    };
    template <typename Tag> struct Table {
        std::uint64_t nextId{1};
        std::vector<Slot<Tag>> slots;
        mutable std::unordered_map<std::uint64_t, std::size_t> indexById;
        void rebuildIndex() const {
            indexById.clear();
            indexById.reserve(slots.size());
            for (std::size_t i = 0; i < slots.size(); ++i)
                indexById.emplace(slots[i].id, i);
        }
        void reset(std::size_t count) {
            slots.clear();
            slots.reserve(count);
            for (std::size_t i = 0; i < count; ++i)
                slots.push_back({nextId++, 1});
            rebuildIndex();
        }
        [[nodiscard]] PmxHandle<Tag> at(std::size_t i) const {
            return i < slots.size() ? PmxHandle<Tag>{slots[i].id, slots[i].generation} : PmxHandle<Tag>{};
        }
        [[nodiscard]] std::optional<std::size_t> index(PmxHandle<Tag> h) const {
            auto found = indexById.find(h.id);
            if (found != indexById.end() && found->second < slots.size() && slots[found->second].id == h.id &&
                slots[found->second].generation == h.generation)
                return found->second;
            if (found != indexById.end()) {
                rebuildIndex();
                found = indexById.find(h.id);
            }
            if (found == indexById.end() || slots[found->second].generation != h.generation)
                return std::nullopt;
            return found->second;
        }
        [[nodiscard]] PmxHandle<Tag> append() {
            slots.push_back({nextId++, 1});
            indexById.emplace(slots.back().id, slots.size() - 1);
            return at(slots.size() - 1);
        }
        [[nodiscard]] PmxHandle<Tag> insert(std::size_t i) {
            if (i > slots.size())
                return {};
            slots.insert(slots.begin() + static_cast<std::ptrdiff_t>(i), {nextId++, 1});
            rebuildIndex();
            return at(i);
        }
        void erase(std::size_t i) {
            slots.erase(slots.begin() + static_cast<std::ptrdiff_t>(i));
            rebuildIndex();
        }
        void move(std::size_t from, std::size_t to) {
            auto value = slots[from];
            slots.erase(slots.begin() + static_cast<std::ptrdiff_t>(from));
            slots.insert(slots.begin() + static_cast<std::ptrdiff_t>(to), value);
            rebuildIndex();
        }
    };

  private:
    struct ReferenceIndex {
        std::unordered_map<std::uint64_t, std::vector<ReferenceSite>> vertices, textures, materials, bones, morphs,
            rigidBodies;
        std::vector<ReferenceSite> allMaterials;
    };

  public:
    class Transaction;
    PmxDocument() {
        rebuildIndexes();
    }
    explicit PmxDocument(PmxModel model) : model_(std::move(model)) {
        rebuildIndexes();
    }
    [[nodiscard]] const PmxModel &model() const noexcept {
        return model_;
    }
    [[deprecated("Use transaction() for structural edits; unsafeModel invalidates handles")]] [[nodiscard]] PmxModel &
    unsafeModel() noexcept {
        dirty_ = true;
        return model_;
    }
    [[nodiscard]] ValidationResult validate() const {
        return pmx::validate(model_);
    }
    [[nodiscard]] VertexHandle vertexHandle(std::size_t i) const {
        ensure();
        return vertices_.at(i);
    }
    [[nodiscard]] TextureHandle textureHandle(std::size_t i) const {
        ensure();
        return textures_.at(i);
    }
    [[nodiscard]] MaterialHandle materialHandle(std::size_t i) const {
        ensure();
        return materials_.at(i);
    }
    [[nodiscard]] BoneHandle boneHandle(std::size_t i) const {
        ensure();
        return bones_.at(i);
    }
    [[nodiscard]] MorphHandle morphHandle(std::size_t i) const {
        ensure();
        return morphs_.at(i);
    }
    [[nodiscard]] DisplayFrameHandle displayFrameHandle(std::size_t i) const {
        ensure();
        return displayFrames_.at(i);
    }
    [[nodiscard]] RigidBodyHandle rigidBodyHandle(std::size_t i) const {
        ensure();
        return rigidBodies_.at(i);
    }
    [[nodiscard]] JointHandle jointHandle(std::size_t i) const {
        ensure();
        return joints_.at(i);
    }
    [[nodiscard]] SoftBodyHandle softBodyHandle(std::size_t i) const {
        ensure();
        return softBodies_.at(i);
    }
    [[nodiscard]] FaceHandle faceHandle(std::size_t i) const {
        ensure();
        return facesTable_.at(i);
    }
    [[nodiscard]] const std::vector<PmxFace> &faces() const {
        ensure();
        return faces_;
    }
    [[nodiscard]] const PmxVertex *resolve(VertexHandle h) const {
        ensure();
        return resolve(model_.vertices, vertices_, h);
    }
    [[nodiscard]] const PmxTexture *resolve(TextureHandle h) const {
        ensure();
        return resolve(model_.textures, textures_, h);
    }
    [[nodiscard]] const PmxMaterial *resolve(MaterialHandle h) const {
        ensure();
        return resolve(model_.materials, materials_, h);
    }
    [[nodiscard]] const PmxBone *resolve(BoneHandle h) const {
        ensure();
        return resolve(model_.bones, bones_, h);
    }
    [[nodiscard]] const PmxMorph *resolve(MorphHandle h) const {
        ensure();
        return resolve(model_.morphs, morphs_, h);
    }
    [[nodiscard]] const PmxDisplayFrame *resolve(DisplayFrameHandle h) const {
        ensure();
        return resolve(model_.displayFrames, displayFrames_, h);
    }
    [[nodiscard]] const PmxRigidBody *resolve(RigidBodyHandle h) const {
        ensure();
        return resolve(model_.rigidBodies, rigidBodies_, h);
    }
    [[nodiscard]] const PmxJoint *resolve(JointHandle h) const {
        ensure();
        return resolve(model_.joints, joints_, h);
    }
    [[nodiscard]] const PmxSoftBody *resolve(SoftBodyHandle h) const {
        ensure();
        return resolve(model_.softBodies, softBodies_, h);
    }
    [[nodiscard]] const PmxFace *resolve(FaceHandle h) const {
        ensure();
        return resolve(faces_, facesTable_, h);
    }
    [[nodiscard]] std::vector<ReferenceSite> referencesTo(VertexHandle h) const;
    [[nodiscard]] std::vector<ReferenceSite> referencesTo(TextureHandle h) const {
        return lookup(h, textures_, refs_.textures);
    }
    [[nodiscard]] std::vector<ReferenceSite> referencesTo(MaterialHandle h) const {
        return lookup(h, materials_, refs_.materials);
    }
    [[nodiscard]] std::vector<ReferenceSite> referencesTo(BoneHandle h) const {
        return lookup(h, bones_, refs_.bones);
    }
    [[nodiscard]] std::vector<ReferenceSite> referencesTo(MorphHandle h) const {
        return lookup(h, morphs_, refs_.morphs);
    }
    [[nodiscard]] std::vector<ReferenceSite> referencesTo(RigidBodyHandle h) const {
        return lookup(h, rigidBodies_, refs_.rigidBodies);
    }
    [[nodiscard]] std::vector<ReferenceSite> allMaterialReferences() const {
        ensure();
        return refs_.allMaterials;
    }
    [[nodiscard]] Transaction transaction();

  private:
    PmxModel model_;
    mutable bool dirty_{};
    mutable Table<VertexTag> vertices_;
    mutable Table<TextureTag> textures_;
    mutable Table<MaterialTag> materials_;
    mutable Table<BoneTag> bones_;
    mutable Table<MorphTag> morphs_;
    mutable Table<DisplayFrameTag> displayFrames_;
    mutable Table<RigidBodyTag> rigidBodies_;
    mutable Table<JointTag> joints_;
    mutable Table<SoftBodyTag> softBodies_;
    mutable Table<FaceTag> facesTable_;
    mutable std::vector<PmxFace> faces_;
    mutable ReferenceIndex refs_;
    template <typename T, typename Tag>
    static const T *resolve(const std::vector<T> &values, const Table<Tag> &table, PmxHandle<Tag> h) {
        const auto i = table.index(h);
        return i ? &values[*i] : nullptr;
    }
    template <typename Tag>
    std::vector<ReferenceSite> lookup(PmxHandle<Tag> h, const Table<Tag> &table,
                                      const std::unordered_map<std::uint64_t, std::vector<ReferenceSite>> &map) const {
        ensure();
        if (!table.index(h))
            return {};
        const auto i = map.find(h.id);
        return i == map.end() ? std::vector<ReferenceSite>{} : i->second;
    }
    void ensure() const {
        if (dirty_)
            const_cast<PmxDocument *>(this)->rebuildIndexes();
    }
    void rebuildIndexes();
    void rebuildReferences();
    static void remapBoneReferences(PmxModel &model, const std::vector<std::int32_t> &map);
    friend class Transaction;
};

class PmxDocument::Transaction {
  public:
    explicit Transaction(PmxDocument &d) : document_(d) {
        document_.ensure();
        model_ = d.model_;
        vertices_ = d.vertices_;
        textures_ = d.textures_;
        materials_ = d.materials_;
        bones_ = d.bones_;
        morphs_ = d.morphs_;
        displayFrames_ = d.displayFrames_;
        rigidBodies_ = d.rigidBodies_;
        joints_ = d.joints_;
        softBodies_ = d.softBodies_;
        facesTable_ = d.facesTable_;
        faces_ = d.faces_;
    }
    [[nodiscard]] BoneHandle addBone(PmxBone bone) {
        return insertBone(model_.bones.size(), std::move(bone));
    }
    [[nodiscard]] BoneHandle addBone(BoneDraft draft);
    // Low-level DTO insertion. References in bone are interpreted as
    // pre-insertion PMX indices; use BoneDraft for editor-facing code.
    [[nodiscard]] BoneHandle insertBone(std::size_t destination, PmxBone bone);
    [[nodiscard]] bool renameBone(BoneHandle h, std::string name) {
        const auto i = bones_.index(h);
        if (!i)
            return false;
        model_.bones[*i].name = std::move(name);
        return true;
    }
    [[nodiscard]] bool setBoneParent(BoneHandle child, std::optional<BoneHandle> parent);
    [[nodiscard]] bool setBoneTailBone(BoneHandle bone, BoneHandle target);
    [[nodiscard]] bool setBoneTailOffset(BoneHandle bone, Float3 offset);
    [[nodiscard]] bool setBoneInherit(BoneHandle bone, std::optional<BoneHandle> parent, float ratio, bool rotation,
                                      bool translation);
    [[nodiscard]] bool setBoneIkTarget(BoneHandle bone, BoneHandle target);
    [[nodiscard]] bool addBoneIkLink(BoneHandle bone, BoneIkLinkDraft link);
    [[nodiscard]] bool eraseBoneIkLink(BoneHandle bone, std::size_t index);
    [[nodiscard]] EraseImpact analyzeErase(BoneHandle h) const;
    [[nodiscard]] bool eraseBone(BoneHandle h, ErasePolicy policy = ErasePolicy::rejectIfReferenced);
    [[nodiscard]] bool moveBone(BoneHandle h, std::size_t destination);
    [[nodiscard]] bool eraseMaterial(MaterialHandle h);
    // FaceGraph owns indexCount; the supplied PmxMaterial::indexCount is ignored.
    [[nodiscard]] MaterialHandle addMaterial(PmxMaterial material);
    [[nodiscard]] bool moveMaterial(MaterialHandle h, std::size_t destination);
    [[nodiscard]] bool eraseMaterial(MaterialHandle h, std::optional<MaterialHandle> replacement);
    [[nodiscard]] VertexHandle addVertex(PmxVertex vertex);
    [[nodiscard]] VertexEraseImpact analyzeErase(VertexHandle h) const;
    [[nodiscard]] bool moveVertex(VertexHandle h, std::size_t destination);
    [[nodiscard]] bool eraseVertex(VertexHandle h);
    [[nodiscard]] FaceHandle addFace(VertexHandle a, VertexHandle b, VertexHandle c, MaterialHandle material);
    [[nodiscard]] bool eraseFace(FaceHandle h);
    [[nodiscard]] bool setFaceMaterial(FaceHandle h, MaterialHandle material);
    [[nodiscard]] MorphHandle addMorph(PmxMorph morph);
    [[nodiscard]] bool moveMorph(MorphHandle h, std::size_t destination);
    [[nodiscard]] bool eraseMorph(MorphHandle h);
    [[nodiscard]] TextureHandle addTexture(PmxTexture texture);
    [[nodiscard]] bool moveTexture(TextureHandle h, std::size_t destination);
    [[nodiscard]] bool eraseTexture(TextureHandle h);
    [[nodiscard]] RigidBodyHandle addRigidBody(PmxRigidBody body);
    [[nodiscard]] RigidBodyHandle addRigidBody(RigidBodyDraft draft) {
        draft.value.bone = -1;
        const auto handle = addRigidBody(std::move(draft.value));
        const auto body = rigidBodies_.index(handle);
        if (!handle || !body)
            return {};
        if (draft.bone) {
            const auto bone = bones_.index(*draft.bone);
            if (!bone) {
                errors_.push_back("invalid rigid body draft bone");
                return {};
            }
            model_.rigidBodies[*body].bone = static_cast<std::int32_t>(*bone);
        }
        return handle;
    }
    [[nodiscard]] bool moveRigidBody(RigidBodyHandle h, std::size_t destination);
    [[nodiscard]] bool eraseRigidBody(RigidBodyHandle h);
    [[nodiscard]] JointHandle addJoint(PmxJoint joint);
    [[nodiscard]] JointHandle addJoint(JointDraft draft) {
        const auto a = rigidBodies_.index(draft.bodyA), b = rigidBodies_.index(draft.bodyB);
        if (!a || !b) {
            errors_.push_back("invalid joint draft body");
            return {};
        }
        draft.value.bodyA = static_cast<std::int32_t>(*a);
        draft.value.bodyB = static_cast<std::int32_t>(*b);
        return addJoint(std::move(draft.value));
    }
    [[nodiscard]] bool eraseJoint(JointHandle h);
    [[nodiscard]] DisplayFrameHandle addDisplayFrame(PmxDisplayFrame frame);
    [[nodiscard]] bool eraseDisplayFrame(DisplayFrameHandle h);
    [[nodiscard]] SoftBodyHandle addSoftBody(PmxSoftBody body);
    [[nodiscard]] SoftBodyHandle addSoftBody(SoftBodyDraft draft) {
        draft.value.material = -1;
        if (draft.material) {
            const auto material = materials_.index(*draft.material);
            if (!material) {
                errors_.push_back("invalid soft body draft material");
                return {};
            }
            draft.value.material = static_cast<std::int32_t>(*material);
        }
        return addSoftBody(std::move(draft.value));
    }
    [[nodiscard]] bool eraseSoftBody(SoftBodyHandle h);
    [[nodiscard]] PmxTransactionResult commit();

  private:
    PmxDocument &document_;
    PmxModel model_;
    Table<VertexTag> vertices_;
    Table<TextureTag> textures_;
    Table<MaterialTag> materials_;
    Table<BoneTag> bones_;
    Table<MorphTag> morphs_;
    Table<DisplayFrameTag> displayFrames_;
    Table<RigidBodyTag> rigidBodies_;
    Table<JointTag> joints_;
    Table<SoftBodyTag> softBodies_;
    Table<FaceTag> facesTable_;
    std::vector<PmxFace> faces_;
    std::vector<std::string> errors_;
    bool done_{};
    [[nodiscard]] bool boneReferenced(std::size_t i) const;
    [[nodiscard]] bool materialReferenced(std::size_t i) const;
    [[nodiscard]] bool vertexReferenced(std::size_t i) const;
    [[nodiscard]] bool morphReferenced(std::size_t i) const;
    [[nodiscard]] bool textureReferenced(std::size_t i) const;
    [[nodiscard]] bool rigidBodyReferenced(std::size_t i) const;
};

inline PmxDocument::Transaction PmxDocument::transaction() {
    ensure();
    return Transaction(*this);
}

} // namespace mmd
