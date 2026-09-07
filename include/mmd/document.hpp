#pragma once

#include <mmd/pmx.hpp>

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <memory>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace mmd {

template <typename Tag> struct PmxHandle {
    std::uint64_t domain{};
    std::uint64_t id{};
    std::uint32_t generation{};
    [[nodiscard]] explicit constexpr operator bool() const noexcept {
        return domain != 0 && id != 0;
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
    bool operator==(const PmxFace &) const = default;
};
struct PmxVertexSkin {
    PmxWeightType type{PmxWeightType::bdef1};
    std::array<BoneHandle, 4> bones{};
    Float4 weights{1.0F, 0.0F, 0.0F, 0.0F};
    Float3 sdefC{};
    Float3 sdefR0{};
    Float3 sdefR1{};
};
struct PmxChangeSet {
    bool topologyChanged{};
    std::vector<VertexHandle> vertices;
    std::vector<TextureHandle> textures;
    std::vector<MaterialHandle> materials;
    std::vector<BoneHandle> bones;
    std::vector<MorphHandle> morphs;
    std::vector<DisplayFrameHandle> displayFrames;
    std::vector<RigidBodyHandle> rigidBodies;
    std::vector<JointHandle> joints;
    std::vector<SoftBodyHandle> softBodies;
    bool texturesChanged{};
    bool physicsChanged{};
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
struct BoneDraft {
    PmxBone value;
    std::optional<BoneHandle> parent;
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
class PmxPatch {
  public:
    struct Data;
    [[nodiscard]] bool empty() const noexcept {
        return !data_;
    }

  private:
    std::shared_ptr<const Data> data_;
    friend class PmxDocument;
};
struct PmxTransactionResult {
    bool committed{};
    ValidationResult validation;
    std::vector<std::string> errors;
    PmxChangeSet changes;
    PmxPatch patch;
};

class PmxDocument {
  public:
    template <typename Tag> struct Slot {
        std::uint64_t id{};
        std::uint32_t generation{1};
    };
    template <typename Tag> struct Table {
        std::uint64_t domain{};
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
            return i < slots.size() ? PmxHandle<Tag>{domain, slots[i].id, slots[i].generation} : PmxHandle<Tag>{};
        }
        [[nodiscard]] std::optional<std::size_t> index(PmxHandle<Tag> h) const {
            if (h.domain != domain)
                return std::nullopt;
            auto found = indexById.find(h.id);
            if (found != indexById.end() && found->second < slots.size() && slots[found->second].id == h.id &&
                slots[found->second].generation == h.generation)
                return found->second;
            if (found != indexById.end()) {
                rebuildIndex();
                found = indexById.find(h.id);
            }
            if (found == indexById.end() || slots[found->second].id != h.id ||
                slots[found->second].generation != h.generation)
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
    PmxDocument();
    explicit PmxDocument(PmxModel model);
    PmxDocument(const PmxDocument &other);
    PmxDocument &operator=(const PmxDocument &other);
    PmxDocument(PmxDocument &&) noexcept = default;
    PmxDocument &operator=(PmxDocument &&) noexcept = default;
    [[nodiscard]] std::uint64_t domain() const noexcept {
        return domain_;
    }
    void restoreSnapshot(const PmxDocument &snapshot, std::uint64_t targetDomain);
    [[nodiscard]] bool applyPatch(const PmxPatch &patch, bool forward);
    [[nodiscard]] const PmxModel &model() const noexcept {
        return model_;
    }
    void setSourcePath(std::filesystem::path path) {
        model_.sourcePath = std::move(path);
    }
    [[deprecated("Use transaction() for structural edits; unsafeModel invalidates handles")]] [[nodiscard]] PmxModel &
    unsafeModel() noexcept {
        dirty_ = true;
        return model_;
    }
    [[nodiscard]] ValidationResult validate() const;
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
    [[nodiscard]] PmxTransactionResult replaceMetadata(const PmxMetadata &metadata);
    [[nodiscard]] PmxTransactionResult replaceVertex(VertexHandle h, const PmxVertex &vertex);
    [[nodiscard]] PmxTransactionResult replaceTexture(TextureHandle h, const PmxTexture &texture);
    [[nodiscard]] PmxTransactionResult replaceMaterial(MaterialHandle h, const PmxMaterial &material);
    [[nodiscard]] PmxTransactionResult replaceBone(BoneHandle h, const PmxBone &bone);
    [[nodiscard]] PmxTransactionResult replaceMorph(MorphHandle h, const PmxMorph &morph);
    [[nodiscard]] PmxTransactionResult replaceDisplayFrame(DisplayFrameHandle h, const PmxDisplayFrame &frame);
    [[nodiscard]] PmxTransactionResult replaceRigidBody(RigidBodyHandle h, const PmxRigidBody &body);
    [[nodiscard]] PmxTransactionResult replaceJoint(JointHandle h, const PmxJoint &joint);
    [[nodiscard]] PmxTransactionResult replaceSoftBody(SoftBodyHandle h, const PmxSoftBody &body);
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
    std::uint64_t domain_{};
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
    static std::uint64_t allocateDomain() noexcept;
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
    void rebuildReferencesFor(ReferenceObjectKind kind, std::size_t index);
    [[nodiscard]] ValidationResult validateProperty(ReferenceObjectKind kind, std::size_t index) const;
    static void remapBoneReferences(PmxModel &model, const std::vector<std::int32_t> &map);
    [[nodiscard]] PmxTransactionResult finishPropertyEdit(PmxChangeSet changes,
                                                          ReferenceObjectKind kind = ReferenceObjectKind::model,
                                                          std::size_t index = 0);
    [[nodiscard]] PmxPatch makePatch(const Transaction &transaction) const;
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
    [[nodiscard]] BoneHandle addBone(BoneDraft draft) {
        draft.value.parent = -1;
        const auto handle = addBone(std::move(draft.value));
        if (handle && draft.parent && !setBoneParent(handle, *draft.parent)) {
            errors_.push_back("invalid bone draft parent");
            return {};
        }
        return handle;
    }
    // References in bone are interpreted as pre-insertion PMX indices. Prefer
    // addBone() followed by handle-based setters for new editor code.
    [[nodiscard]] BoneHandle insertBone(std::size_t destination, PmxBone bone);
    [[nodiscard]] bool renameBone(BoneHandle h, std::string name) {
        const auto i = bones_.index(h);
        if (!i)
            return false;
        model_.bones[*i].name = std::move(name);
        if (std::find(changes_.bones.begin(), changes_.bones.end(), h) == changes_.bones.end())
            changes_.bones.push_back(h);
        return true;
    }
    [[nodiscard]] bool setBoneParent(BoneHandle child, BoneHandle parent) {
        return setBoneParent(child, std::optional<BoneHandle>{parent});
    }
    [[nodiscard]] bool setBoneParent(BoneHandle child, std::optional<BoneHandle> parent) {
        const auto c = bones_.index(child);
        const auto p = parent ? bones_.index(*parent) : std::optional<std::size_t>{};
        if (!c || (parent && (!p || *c == *p)))
            return false;
        if (parent) {
            std::vector<bool> visited(model_.bones.size());
            for (auto cursor = *p;;) {
                if (cursor >= model_.bones.size() || visited[cursor] || cursor == *c)
                    return false;
                visited[cursor] = true;
                const auto next = model_.bones[cursor].parent;
                if (next < 0)
                    break;
                cursor = static_cast<std::size_t>(next);
            }
        }
        model_.bones[*c].parent = p ? static_cast<std::int32_t>(*p) : -1;
        if (std::find(changes_.bones.begin(), changes_.bones.end(), child) == changes_.bones.end())
            changes_.bones.push_back(child);
        return true;
    }
    [[nodiscard]] bool setMetadata(PmxMetadata metadata);
    [[nodiscard]] bool setBone(BoneHandle h, const PmxBone &bone);
    [[nodiscard]] bool setBoneName(BoneHandle h, std::string value);
    [[nodiscard]] bool setBoneEnglishName(BoneHandle h, std::string value);
    [[nodiscard]] bool setBonePosition(BoneHandle h, Float3 value);
    [[nodiscard]] bool setBoneTailBone(BoneHandle h, std::optional<BoneHandle> value);
    [[nodiscard]] bool setBoneTailOffset(BoneHandle h, Float3 value);
    [[nodiscard]] bool setBoneDeformLayer(BoneHandle h, std::int32_t value);
    [[nodiscard]] bool setBoneFlags(BoneHandle h, std::uint16_t value);
    [[nodiscard]] bool setBoneInheritParent(BoneHandle h, std::optional<BoneHandle> value);
    [[nodiscard]] bool setBoneInheritRatio(BoneHandle h, float value);
    [[nodiscard]] bool setBoneFixedAxis(BoneHandle h, Float3 value);
    [[nodiscard]] bool setBoneLocalAxes(BoneHandle h, Float3 x, Float3 z);
    [[nodiscard]] bool setBoneExternalParentKey(BoneHandle h, std::int32_t value);
    [[nodiscard]] bool setBoneIkTarget(BoneHandle h, std::optional<BoneHandle> value);
    [[nodiscard]] bool setBoneIkLimits(BoneHandle h, std::int32_t loops, float angle);
    [[nodiscard]] bool setBoneIkLink(BoneHandle h, std::size_t index, PmxIkLink value);
    [[nodiscard]] bool addBoneIkLink(BoneHandle h, PmxIkLink value);
    [[nodiscard]] bool eraseBoneIkLink(BoneHandle h, std::size_t index);
    [[nodiscard]] EraseImpact analyzeErase(BoneHandle h) const;
    [[nodiscard]] bool eraseBone(BoneHandle h, ErasePolicy policy = ErasePolicy::rejectIfReferenced);
    [[nodiscard]] bool moveBone(BoneHandle h, std::size_t destination);
    [[nodiscard]] bool eraseMaterial(MaterialHandle h);
    [[nodiscard]] bool setMaterial(MaterialHandle h, const PmxMaterial &material);
    [[nodiscard]] bool setMaterialName(MaterialHandle h, std::string value);
    [[nodiscard]] bool setMaterialEnglishName(MaterialHandle h, std::string value);
    [[nodiscard]] bool setMaterialDiffuse(MaterialHandle h, Float4 value);
    [[nodiscard]] bool setMaterialSpecular(MaterialHandle h, Float3 value);
    [[nodiscard]] bool setMaterialShininess(MaterialHandle h, float value);
    [[nodiscard]] bool setMaterialAmbient(MaterialHandle h, Float3 value);
    [[nodiscard]] bool setMaterialDrawFlags(MaterialHandle h, std::uint8_t value);
    [[nodiscard]] bool setMaterialEdge(MaterialHandle h, Float4 color, float size);
    [[nodiscard]] bool setMaterialTexture(MaterialHandle h, std::optional<TextureHandle> value);
    [[nodiscard]] bool setMaterialSphereTexture(MaterialHandle h, std::optional<TextureHandle> value);
    [[nodiscard]] bool setMaterialToonTexture(MaterialHandle h, std::optional<TextureHandle> value);
    [[nodiscard]] bool setMaterialSphereMode(MaterialHandle h, std::uint8_t value);
    [[nodiscard]] bool setMaterialToonMode(MaterialHandle h, std::uint8_t value);
    [[nodiscard]] bool setMaterialMemo(MaterialHandle h, std::string value);
    // FaceGraph owns indexCount; the supplied PmxMaterial::indexCount is ignored.
    [[nodiscard]] MaterialHandle addMaterial(PmxMaterial material);
    [[nodiscard]] bool moveMaterial(MaterialHandle h, std::size_t destination);
    [[nodiscard]] bool eraseMaterial(MaterialHandle h, std::optional<MaterialHandle> replacement);
    [[nodiscard]] VertexHandle addVertex(PmxVertex vertex);
    [[nodiscard]] bool setVertex(VertexHandle h, const PmxVertex &vertex);
    [[nodiscard]] bool setVertexPosition(VertexHandle h, Float3 value);
    [[nodiscard]] bool setVertexNormal(VertexHandle h, Float3 value);
    [[nodiscard]] bool setVertexUv(VertexHandle h, Float2 value);
    [[nodiscard]] bool setVertexAdditionalUv(VertexHandle h, std::uint32_t channel, Float4 value);
    [[nodiscard]] bool setVertexEdgeScale(VertexHandle h, float value);
    [[nodiscard]] bool setVertexSkin(VertexHandle h, const PmxVertex &value);
    [[nodiscard]] bool setVertexSkin(VertexHandle h, const PmxVertexSkin &value);
    [[nodiscard]] VertexEraseImpact analyzeErase(VertexHandle h) const;
    [[nodiscard]] bool moveVertex(VertexHandle h, std::size_t destination);
    [[nodiscard]] bool eraseVertex(VertexHandle h);
    [[nodiscard]] FaceHandle addFace(VertexHandle a, VertexHandle b, VertexHandle c, MaterialHandle material);
    [[nodiscard]] bool eraseFace(FaceHandle h);
    [[nodiscard]] bool setFaceMaterial(FaceHandle h, MaterialHandle material);
    [[nodiscard]] MorphHandle addMorph(PmxMorph morph);
    [[nodiscard]] bool setMorph(MorphHandle h, const PmxMorph &morph);
    [[nodiscard]] bool setMorphName(MorphHandle h, std::string value);
    [[nodiscard]] bool setMorphEnglishName(MorphHandle h, std::string value);
    [[nodiscard]] bool setMorphPanel(MorphHandle h, std::uint8_t value);
    [[nodiscard]] bool setMorphType(MorphHandle h, std::uint8_t value);
    [[nodiscard]] bool setMorphOffset(MorphHandle h, std::size_t index, PmxMorphOffset value);
    [[nodiscard]] bool setVertexMorphOffset(MorphHandle h, std::size_t offset, VertexHandle vertex, Float3 value);
    [[nodiscard]] bool setBoneMorphOffset(MorphHandle h, std::size_t offset, BoneHandle bone, Float3 translation,
                                           Float4 rotation);
    [[nodiscard]] bool setGroupMorphOffset(MorphHandle h, std::size_t offset, MorphHandle target, float weight);
    [[nodiscard]] bool setUvMorphOffset(MorphHandle h, std::size_t offset, VertexHandle vertex, std::uint32_t channel,
                                         Float4 value);
    [[nodiscard]] bool setMaterialMorphOffset(MorphHandle h, std::size_t offset, std::optional<MaterialHandle> material,
                                               std::uint8_t operation, std::array<Float4, 8> values);
    [[nodiscard]] bool setFlipMorphOffset(MorphHandle h, std::size_t offset, MorphHandle target, float weight);
    [[nodiscard]] bool setImpulseMorphOffset(MorphHandle h, std::size_t offset, RigidBodyHandle body, Float3 velocity,
                                              Float3 torque, bool local);
    [[nodiscard]] bool addMorphOffset(MorphHandle h, PmxMorphOffset value);
    [[nodiscard]] bool addVertexMorphOffset(MorphHandle h, VertexHandle vertex, Float3 value);
    [[nodiscard]] bool addBoneMorphOffset(MorphHandle h, BoneHandle bone, Float3 translation, Float4 rotation);
    [[nodiscard]] bool addGroupMorphOffset(MorphHandle h, MorphHandle target, float weight);
    [[nodiscard]] bool addUvMorphOffset(MorphHandle h, VertexHandle vertex, std::uint32_t channel, Float4 value);
    [[nodiscard]] bool addMaterialMorphOffset(MorphHandle h, std::optional<MaterialHandle> material, std::uint8_t operation,
                                               std::array<Float4, 8> values);
    [[nodiscard]] bool addFlipMorphOffset(MorphHandle h, MorphHandle target, float weight);
    [[nodiscard]] bool addImpulseMorphOffset(MorphHandle h, RigidBodyHandle body, Float3 velocity, Float3 torque,
                                              bool local);
    [[nodiscard]] bool eraseMorphOffset(MorphHandle h, std::size_t index);
    [[nodiscard]] bool moveMorphOffset(MorphHandle h, std::size_t from, std::size_t to);
    [[nodiscard]] bool moveMorph(MorphHandle h, std::size_t destination);
    [[nodiscard]] bool eraseMorph(MorphHandle h);
    [[nodiscard]] TextureHandle addTexture(PmxTexture texture);
    [[nodiscard]] bool setTexture(TextureHandle h, const PmxTexture &texture);
    [[nodiscard]] bool setTexturePath(TextureHandle h, std::string value);
    [[nodiscard]] bool moveTexture(TextureHandle h, std::size_t destination);
    [[nodiscard]] bool eraseTexture(TextureHandle h);
    [[nodiscard]] RigidBodyHandle addRigidBody(PmxRigidBody body);
    [[nodiscard]] bool setRigidBody(RigidBodyHandle h, const PmxRigidBody &body);
    [[nodiscard]] bool setRigidBodyName(RigidBodyHandle h, std::string value);
    [[nodiscard]] bool setRigidBodyEnglishName(RigidBodyHandle h, std::string value);
    [[nodiscard]] bool setRigidBodyBone(RigidBodyHandle h, std::optional<BoneHandle> value);
    [[nodiscard]] bool setRigidBodyShape(RigidBodyHandle h, std::uint8_t shape, Float3 size);
    [[nodiscard]] bool setRigidBodyTransform(RigidBodyHandle h, Float3 position, Float3 rotation);
    [[nodiscard]] bool setRigidBodyPhysical(RigidBodyHandle h, float mass, float linearDamping,
                                             float angularDamping, float restitution, float friction);
    [[nodiscard]] bool setRigidBodyCollision(RigidBodyHandle h, std::uint8_t group, std::uint16_t mask);
    [[nodiscard]] bool setRigidBodyMode(RigidBodyHandle h, std::uint8_t mode);
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
    [[nodiscard]] bool setJoint(JointHandle h, const PmxJoint &joint);
    [[nodiscard]] bool setJointName(JointHandle h, std::string value);
    [[nodiscard]] bool setJointEnglishName(JointHandle h, std::string value);
    [[nodiscard]] bool setJointType(JointHandle h, std::uint8_t value);
    [[nodiscard]] bool setJointBodies(JointHandle h, RigidBodyHandle bodyA, RigidBodyHandle bodyB);
    [[nodiscard]] bool setJointTransform(JointHandle h, Float3 position, Float3 rotation);
    [[nodiscard]] bool setJointLimits(JointHandle h, Float3 translationMinimum, Float3 translationMaximum,
                                      Float3 rotationMinimum, Float3 rotationMaximum);
    [[nodiscard]] bool setJointSprings(JointHandle h, Float3 translation, Float3 rotation);
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
    [[nodiscard]] bool setDisplayFrame(DisplayFrameHandle h, const PmxDisplayFrame &frame);
    [[nodiscard]] bool setDisplayFrameName(DisplayFrameHandle h, std::string value);
    [[nodiscard]] bool setDisplayFrameEnglishName(DisplayFrameHandle h, std::string value);
    [[nodiscard]] bool setDisplayFrameItem(DisplayFrameHandle h, std::size_t index, PmxDisplayItem value);
    [[nodiscard]] bool setDisplayFrameItem(DisplayFrameHandle h, std::size_t index, BoneHandle bone);
    [[nodiscard]] bool setDisplayFrameItem(DisplayFrameHandle h, std::size_t index, MorphHandle morph);
    [[nodiscard]] bool addDisplayFrameItem(DisplayFrameHandle h, PmxDisplayItem value);
    [[nodiscard]] bool addDisplayFrameItem(DisplayFrameHandle h, BoneHandle bone);
    [[nodiscard]] bool addDisplayFrameItem(DisplayFrameHandle h, MorphHandle morph);
    [[nodiscard]] bool eraseDisplayFrameItem(DisplayFrameHandle h, std::size_t index);
    [[nodiscard]] bool moveDisplayFrameItem(DisplayFrameHandle h, std::size_t from, std::size_t to);
    [[nodiscard]] bool moveDisplayFrame(DisplayFrameHandle h, std::size_t destination);
    [[nodiscard]] bool eraseDisplayFrame(DisplayFrameHandle h);
    [[nodiscard]] SoftBodyHandle addSoftBody(PmxSoftBody body);
    [[nodiscard]] bool setSoftBody(SoftBodyHandle h, const PmxSoftBody &body);
    [[nodiscard]] bool setSoftBodyName(SoftBodyHandle h, std::string value);
    [[nodiscard]] bool setSoftBodyEnglishName(SoftBodyHandle h, std::string value);
    [[nodiscard]] bool setSoftBodyMaterial(SoftBodyHandle h, std::optional<MaterialHandle> value);
    [[nodiscard]] bool setSoftBodyAnchors(SoftBodyHandle h, std::vector<PmxSoftBodyAnchor> value);
    [[nodiscard]] bool setSoftBodyPinnedVertices(SoftBodyHandle h, std::vector<std::int32_t> value);
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
    friend class PmxDocument;
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
    PmxChangeSet changes_;
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
