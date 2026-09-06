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
    [[nodiscard]] explicit constexpr operator bool() const noexcept { return id != 0; }
    auto operator<=>(const PmxHandle&) const = default;
};
struct VertexTag {}; struct TextureTag {}; struct MaterialTag {}; struct BoneTag {};
struct MorphTag {}; struct DisplayFrameTag {}; struct RigidBodyTag {}; struct JointTag {}; struct SoftBodyTag {};
using VertexHandle = PmxHandle<VertexTag>; using TextureHandle = PmxHandle<TextureTag>;
using MaterialHandle = PmxHandle<MaterialTag>; using BoneHandle = PmxHandle<BoneTag>;
using MorphHandle = PmxHandle<MorphTag>; using DisplayFrameHandle = PmxHandle<DisplayFrameTag>;
using RigidBodyHandle = PmxHandle<RigidBodyTag>; using JointHandle = PmxHandle<JointTag>;
using SoftBodyHandle = PmxHandle<SoftBodyTag>;

enum class ReferenceObjectKind : std::uint8_t { vertex, material, bone, morph, displayFrame, rigidBody, joint, softBody };
enum class ReferenceField : std::uint8_t {
    vertexBone, materialTexture, materialSphereTexture, materialToonTexture, boneParent, boneTail,
    boneInheritParent, boneIkTarget, boneIkLink, morphOffset, displayItem, rigidBodyBone,
    jointBodyA, jointBodyB, softBodyMaterial, softBodyAnchorRigidBody, softBodyAnchorVertex, softBodyPinnedVertex,
};
// Identifies a graph edge without storing a vector-invalidated pointer.
struct ReferenceSite { ReferenceObjectKind ownerKind{}; std::uint64_t ownerId{}; std::uint32_t ownerGeneration{}; ReferenceField field{}; std::uint32_t subIndex{}; };
struct PmxTransactionResult { bool committed{}; ValidationResult validation; std::vector<std::string> errors; };

class PmxDocument {
public:
    template <typename Tag> struct Slot { std::uint64_t id{}; std::uint32_t generation{1}; };
    template <typename Tag> struct Table {
        std::uint64_t nextId{1}; std::vector<Slot<Tag>> slots;
        void reset(std::size_t count) { slots.clear(); slots.reserve(count); for (std::size_t i=0;i<count;++i) slots.push_back({nextId++,1}); }
        [[nodiscard]] PmxHandle<Tag> at(std::size_t i) const { return i < slots.size() ? PmxHandle<Tag>{slots[i].id,slots[i].generation} : PmxHandle<Tag>{}; }
        [[nodiscard]] std::optional<std::size_t> index(PmxHandle<Tag> h) const { for(std::size_t i=0;i<slots.size();++i) if(slots[i].id==h.id && slots[i].generation==h.generation) return i; return std::nullopt; }
        [[nodiscard]] PmxHandle<Tag> append() { slots.push_back({nextId++,1}); return at(slots.size()-1); }
    };
private:
    struct ReferenceIndex {
        std::unordered_map<std::uint64_t,std::vector<ReferenceSite>> vertices, textures, materials, bones, morphs, rigidBodies;
    };
public:
    class Transaction;
    PmxDocument() { rebuildIndexes(); }
    explicit PmxDocument(PmxModel model) : model_(std::move(model)) { rebuildIndexes(); }
    [[nodiscard]] const PmxModel& model() const noexcept { return model_; }
    [[deprecated("Use transaction() for structural edits; unsafeModel invalidates handles")]]
    [[nodiscard]] PmxModel& unsafeModel() noexcept { dirty_=true; return model_; }
    [[nodiscard]] ValidationResult validate() const { return pmx::validate(model_); }
    [[nodiscard]] VertexHandle vertexHandle(std::size_t i) const { ensure(); return vertices_.at(i); }
    [[nodiscard]] TextureHandle textureHandle(std::size_t i) const { ensure(); return textures_.at(i); }
    [[nodiscard]] MaterialHandle materialHandle(std::size_t i) const { ensure(); return materials_.at(i); }
    [[nodiscard]] BoneHandle boneHandle(std::size_t i) const { ensure(); return bones_.at(i); }
    [[nodiscard]] MorphHandle morphHandle(std::size_t i) const { ensure(); return morphs_.at(i); }
    [[nodiscard]] RigidBodyHandle rigidBodyHandle(std::size_t i) const { ensure(); return rigidBodies_.at(i); }
    [[nodiscard]] const PmxVertex* resolve(VertexHandle h) const { ensure(); return resolve(model_.vertices,vertices_,h); }
    [[nodiscard]] const PmxTexture* resolve(TextureHandle h) const { ensure(); return resolve(model_.textures,textures_,h); }
    [[nodiscard]] const PmxMaterial* resolve(MaterialHandle h) const { ensure(); return resolve(model_.materials,materials_,h); }
    [[nodiscard]] const PmxBone* resolve(BoneHandle h) const { ensure(); return resolve(model_.bones,bones_,h); }
    [[nodiscard]] const PmxMorph* resolve(MorphHandle h) const { ensure(); return resolve(model_.morphs,morphs_,h); }
    [[nodiscard]] const PmxRigidBody* resolve(RigidBodyHandle h) const { ensure(); return resolve(model_.rigidBodies,rigidBodies_,h); }
    [[nodiscard]] std::vector<ReferenceSite> referencesTo(VertexHandle h) const { return lookup(h,refs_.vertices); }
    [[nodiscard]] std::vector<ReferenceSite> referencesTo(TextureHandle h) const { return lookup(h,refs_.textures); }
    [[nodiscard]] std::vector<ReferenceSite> referencesTo(MaterialHandle h) const { return lookup(h,refs_.materials); }
    [[nodiscard]] std::vector<ReferenceSite> referencesTo(BoneHandle h) const { return lookup(h,refs_.bones); }
    [[nodiscard]] std::vector<ReferenceSite> referencesTo(MorphHandle h) const { return lookup(h,refs_.morphs); }
    [[nodiscard]] std::vector<ReferenceSite> referencesTo(RigidBodyHandle h) const { return lookup(h,refs_.rigidBodies); }
    [[nodiscard]] Transaction transaction();
private:
    PmxModel model_; mutable bool dirty_{};
    mutable Table<VertexTag> vertices_; mutable Table<TextureTag> textures_; mutable Table<MaterialTag> materials_; mutable Table<BoneTag> bones_; mutable Table<MorphTag> morphs_; mutable Table<DisplayFrameTag> displayFrames_; mutable Table<RigidBodyTag> rigidBodies_; mutable Table<JointTag> joints_; mutable Table<SoftBodyTag> softBodies_; mutable ReferenceIndex refs_;
    template <typename T,typename Tag> static const T* resolve(const std::vector<T>& values,const Table<Tag>& table,PmxHandle<Tag> h) { const auto i=table.index(h); return i ? &values[*i] : nullptr; }
    template <typename Tag> std::vector<ReferenceSite> lookup(PmxHandle<Tag> h,const std::unordered_map<std::uint64_t,std::vector<ReferenceSite>>& map) const { ensure(); const auto i=map.find(h.id); return i==map.end()?std::vector<ReferenceSite>{}:i->second; }
    void ensure() const { if(dirty_) const_cast<PmxDocument*>(this)->rebuildIndexes(); }
    void rebuildIndexes(); void rebuildReferences();
    static void remapBoneReferences(PmxModel& model,const std::vector<std::int32_t>& map);
    friend class Transaction;
};

class PmxDocument::Transaction {
public:
    explicit Transaction(PmxDocument& d) : document_(d) { document_.ensure(); model_=d.model_; bones_=d.bones_; materials_=d.materials_; }
    [[nodiscard]] BoneHandle addBone(PmxBone bone) { if(done_) return {}; model_.bones.push_back(std::move(bone)); return bones_.append(); }
    [[nodiscard]] bool renameBone(BoneHandle h,std::string name) { const auto i=bones_.index(h); if(!i) return false; model_.bones[*i].name=std::move(name); return true; }
    [[nodiscard]] bool setBoneParent(BoneHandle child,BoneHandle parent) { const auto c=bones_.index(child),p=bones_.index(parent); if(!c||!p) return false; model_.bones[*c].parent=static_cast<std::int32_t>(*p); return true; }
    [[nodiscard]] bool eraseBone(BoneHandle h);
    [[nodiscard]] bool moveBone(BoneHandle h,std::size_t destination);
    [[nodiscard]] bool eraseMaterial(MaterialHandle h);
    [[nodiscard]] PmxTransactionResult commit();
private:
    PmxDocument& document_; PmxModel model_; Table<BoneTag> bones_; Table<MaterialTag> materials_; std::vector<std::string> errors_; bool done_{};
    [[nodiscard]] bool boneReferenced(std::size_t i) const;
    [[nodiscard]] bool materialReferenced(std::size_t i) const;
};

inline PmxDocument::Transaction PmxDocument::transaction() { ensure(); return Transaction(*this); }

} // namespace mmd
