#include <mmd/document.hpp>

namespace mmd {
namespace {

template <typename Tag>
void addReference(std::unordered_map<std::uint64_t, std::vector<ReferenceSite>>& map,
                  const PmxDocument::Table<Tag>& table, std::int32_t target,
                  ReferenceObjectKind ownerKind, std::uint64_t ownerId, std::uint32_t ownerGeneration,
                  ReferenceField field, std::uint32_t subIndex = 0) {
    if (target < 0) return;
    const auto targetHandle = table.at(static_cast<std::size_t>(target));
    if (targetHandle)
        map[targetHandle.id].push_back({ownerKind, ownerId, ownerGeneration, field, subIndex});
}

bool containsBoneReference(const PmxModel& model, std::size_t target) {
    const auto match = [target](std::int32_t value) { return value == static_cast<std::int32_t>(target); };
    for (const auto& vertex : model.vertices) {
        const auto count = vertex.weightType == PmxWeightType::bdef1 ? 1U : vertex.weightType == PmxWeightType::bdef2 || vertex.weightType == PmxWeightType::sdef ? 2U : 4U;
        for (std::size_t i=0;i<count;++i) if (match(vertex.bones[i])) return true;
    }
    for (const auto& bone : model.bones) {
        if (match(bone.parent) || ((bone.flags&1U)!=0 && match(bone.tailBone)) || ((bone.flags&0x0300U)!=0 && match(bone.inheritParent)) || ((bone.flags&0x0020U)!=0 && (match(bone.ikTarget) || std::any_of(bone.ikLinks.begin(),bone.ikLinks.end(),[&](const auto& link){return match(link.bone);})))) return true;
    }
    for (const auto& morph : model.morphs) if (morph.type==2) for(const auto& offset:morph.offsets) if(match(offset.index)) return true;
    for (const auto& frame : model.displayFrames) for(const auto& item:frame.items) if(item.bone && match(item.index)) return true;
    return std::any_of(model.rigidBodies.begin(),model.rigidBodies.end(),[&](const auto& body){return match(body.bone);});
}

} // namespace

void PmxDocument::rebuildIndexes() {
    vertices_.reset(model_.vertices.size()); textures_.reset(model_.textures.size()); materials_.reset(model_.materials.size());
    bones_.reset(model_.bones.size()); morphs_.reset(model_.morphs.size()); displayFrames_.reset(model_.displayFrames.size());
    rigidBodies_.reset(model_.rigidBodies.size()); joints_.reset(model_.joints.size()); softBodies_.reset(model_.softBodies.size());
    dirty_ = false; rebuildReferences();
}

void PmxDocument::rebuildReferences() {
    refs_ = {};
    const auto owner = [](const auto& table, std::size_t i) { return table.at(i); };
    for (std::size_t i=0;i<model_.vertices.size();++i) {
        const auto h=owner(vertices_,i); const auto& v=model_.vertices[i]; const auto count=v.weightType==PmxWeightType::bdef1?1U:v.weightType==PmxWeightType::bdef2||v.weightType==PmxWeightType::sdef?2U:4U;
        for(std::size_t n=0;n<count;++n) addReference(refs_.bones,bones_,v.bones[n],ReferenceObjectKind::vertex,h.id,h.generation,ReferenceField::vertexBone,static_cast<std::uint32_t>(n));
    }
    for (std::size_t i=0;i<model_.materials.size();++i) {
        const auto h=owner(materials_,i); const auto& m=model_.materials[i];
        addReference(refs_.textures,textures_,m.textureIndex,ReferenceObjectKind::material,h.id,h.generation,ReferenceField::materialTexture);
        addReference(refs_.textures,textures_,m.sphereTextureIndex,ReferenceObjectKind::material,h.id,h.generation,ReferenceField::materialSphereTexture);
        if(m.toonMode==0) addReference(refs_.textures,textures_,m.toonTextureIndex,ReferenceObjectKind::material,h.id,h.generation,ReferenceField::materialToonTexture);
    }
    for (std::size_t i=0;i<model_.bones.size();++i) {
        const auto h=owner(bones_,i); const auto& b=model_.bones[i];
        addReference(refs_.bones,bones_,b.parent,ReferenceObjectKind::bone,h.id,h.generation,ReferenceField::boneParent);
        if(b.flags&1U) addReference(refs_.bones,bones_,b.tailBone,ReferenceObjectKind::bone,h.id,h.generation,ReferenceField::boneTail);
        if(b.flags&0x0300U) addReference(refs_.bones,bones_,b.inheritParent,ReferenceObjectKind::bone,h.id,h.generation,ReferenceField::boneInheritParent);
        if(b.flags&0x0020U) { addReference(refs_.bones,bones_,b.ikTarget,ReferenceObjectKind::bone,h.id,h.generation,ReferenceField::boneIkTarget); for(std::size_t n=0;n<b.ikLinks.size();++n) addReference(refs_.bones,bones_,b.ikLinks[n].bone,ReferenceObjectKind::bone,h.id,h.generation,ReferenceField::boneIkLink,static_cast<std::uint32_t>(n)); }
    }
    for (std::size_t i=0;i<model_.morphs.size();++i) {
        const auto h=owner(morphs_,i); const auto& m=model_.morphs[i];
        for(std::size_t n=0;n<m.offsets.size();++n) { const auto x=m.offsets[n].index; if(m.type==0||m.type==9) addReference(refs_.morphs,morphs_,x,ReferenceObjectKind::morph,h.id,h.generation,ReferenceField::morphOffset,static_cast<std::uint32_t>(n)); else if(m.type==1||(m.type>=3&&m.type<=7)) addReference(refs_.vertices,vertices_,x,ReferenceObjectKind::morph,h.id,h.generation,ReferenceField::morphOffset,static_cast<std::uint32_t>(n)); else if(m.type==2) addReference(refs_.bones,bones_,x,ReferenceObjectKind::morph,h.id,h.generation,ReferenceField::morphOffset,static_cast<std::uint32_t>(n)); else if(m.type==8) addReference(refs_.materials,materials_,x,ReferenceObjectKind::morph,h.id,h.generation,ReferenceField::morphOffset,static_cast<std::uint32_t>(n)); else if(m.type==10) addReference(refs_.rigidBodies,rigidBodies_,x,ReferenceObjectKind::morph,h.id,h.generation,ReferenceField::morphOffset,static_cast<std::uint32_t>(n)); }
    }
    for(std::size_t i=0;i<model_.displayFrames.size();++i) { const auto h=owner(displayFrames_,i); for(std::size_t n=0;n<model_.displayFrames[i].items.size();++n) { const auto& item=model_.displayFrames[i].items[n]; if(item.bone) addReference(refs_.bones,bones_,item.index,ReferenceObjectKind::displayFrame,h.id,h.generation,ReferenceField::displayItem,static_cast<std::uint32_t>(n)); else addReference(refs_.morphs,morphs_,item.index,ReferenceObjectKind::displayFrame,h.id,h.generation,ReferenceField::displayItem,static_cast<std::uint32_t>(n)); } }
    for(std::size_t i=0;i<model_.rigidBodies.size();++i) { const auto h=owner(rigidBodies_,i); addReference(refs_.bones,bones_,model_.rigidBodies[i].bone,ReferenceObjectKind::rigidBody,h.id,h.generation,ReferenceField::rigidBodyBone); }
    for(std::size_t i=0;i<model_.joints.size();++i) { const auto h=owner(joints_,i); addReference(refs_.rigidBodies,rigidBodies_,model_.joints[i].bodyA,ReferenceObjectKind::joint,h.id,h.generation,ReferenceField::jointBodyA); addReference(refs_.rigidBodies,rigidBodies_,model_.joints[i].bodyB,ReferenceObjectKind::joint,h.id,h.generation,ReferenceField::jointBodyB); }
    for(std::size_t i=0;i<model_.softBodies.size();++i) { const auto h=owner(softBodies_,i); const auto& b=model_.softBodies[i]; addReference(refs_.materials,materials_,b.material,ReferenceObjectKind::softBody,h.id,h.generation,ReferenceField::softBodyMaterial); for(std::size_t n=0;n<b.anchors.size();++n) { addReference(refs_.rigidBodies,rigidBodies_,b.anchors[n].rigidBody,ReferenceObjectKind::softBody,h.id,h.generation,ReferenceField::softBodyAnchorRigidBody,static_cast<std::uint32_t>(n)); addReference(refs_.vertices,vertices_,b.anchors[n].vertex,ReferenceObjectKind::softBody,h.id,h.generation,ReferenceField::softBodyAnchorVertex,static_cast<std::uint32_t>(n)); } for(std::size_t n=0;n<b.pinnedVertices.size();++n) addReference(refs_.vertices,vertices_,b.pinnedVertices[n],ReferenceObjectKind::softBody,h.id,h.generation,ReferenceField::softBodyPinnedVertex,static_cast<std::uint32_t>(n)); }
}

void PmxDocument::remapBoneReferences(PmxModel& model,const std::vector<std::int32_t>& map) {
    const auto remap=[&](std::int32_t& x){if(x>=0)x=map[static_cast<std::size_t>(x)];};
    for(auto& v:model.vertices) { const auto count=v.weightType==PmxWeightType::bdef1?1U:v.weightType==PmxWeightType::bdef2||v.weightType==PmxWeightType::sdef?2U:4U; for(std::size_t i=0;i<count;++i) remap(v.bones[i]); }
    for(auto& b:model.bones) { remap(b.parent); if(b.flags&1U)remap(b.tailBone); if(b.flags&0x0300U)remap(b.inheritParent); if(b.flags&0x0020U){remap(b.ikTarget);for(auto& x:b.ikLinks)remap(x.bone);} }
    for(auto& m:model.morphs)if(m.type==2)for(auto& x:m.offsets)remap(x.index);
    for(auto& f:model.displayFrames)for(auto& x:f.items)if(x.bone)remap(x.index);
    for(auto& b:model.rigidBodies)remap(b.bone);
}

bool PmxDocument::Transaction::boneReferenced(std::size_t i) const { return containsBoneReference(model_,i); }
bool PmxDocument::Transaction::materialReferenced(std::size_t i) const { for(const auto& m:model_.morphs)if(m.type==8)for(const auto& x:m.offsets)if(x.index==static_cast<std::int32_t>(i))return true; return std::any_of(model_.softBodies.begin(),model_.softBodies.end(),[&](const auto& b){return b.material==static_cast<std::int32_t>(i);}); }
bool PmxDocument::Transaction::eraseBone(BoneHandle h) { const auto target=bones_.index(h); if(!target)return false; if(boneReferenced(*target)){errors_.push_back("cannot erase referenced bone");return false;} std::vector<std::int32_t> map(model_.bones.size());for(std::size_t i=0;i<map.size();++i)map[i]=i<*target?static_cast<std::int32_t>(i):static_cast<std::int32_t>(i-1);model_.bones.erase(model_.bones.begin()+static_cast<std::ptrdiff_t>(*target));bones_.slots.erase(bones_.slots.begin()+static_cast<std::ptrdiff_t>(*target));remapBoneReferences(model_,map);return true; }
bool PmxDocument::Transaction::moveBone(BoneHandle h,std::size_t destination) { const auto source=bones_.index(h);if(!source||destination>=model_.bones.size())return false;if(*source==destination)return true;std::vector<std::int32_t> map(model_.bones.size());for(std::size_t i=0;i<map.size();++i)map[i]=i==*source?static_cast<std::int32_t>(destination):*source<destination&&i>*source&&i<=destination?static_cast<std::int32_t>(i-1):destination<*source&&i>=destination&&i<*source?static_cast<std::int32_t>(i+1):static_cast<std::int32_t>(i);auto bone=std::move(model_.bones[*source]);model_.bones.erase(model_.bones.begin()+static_cast<std::ptrdiff_t>(*source));model_.bones.insert(model_.bones.begin()+static_cast<std::ptrdiff_t>(destination),std::move(bone));auto slot=bones_.slots[*source];bones_.slots.erase(bones_.slots.begin()+static_cast<std::ptrdiff_t>(*source));bones_.slots.insert(bones_.slots.begin()+static_cast<std::ptrdiff_t>(destination),slot);remapBoneReferences(model_,map);return true; }
bool PmxDocument::Transaction::eraseMaterial(MaterialHandle h) { const auto target=materials_.index(h);if(!target)return false;if(materialReferenced(*target)){errors_.push_back("cannot erase referenced material");return false;}model_.materials.erase(model_.materials.begin()+static_cast<std::ptrdiff_t>(*target));materials_.slots.erase(materials_.slots.begin()+static_cast<std::ptrdiff_t>(*target));return true; }
PmxTransactionResult PmxDocument::Transaction::commit() { if(done_)return {false,{}, {"transaction has already finished"}};done_=true;auto validation=pmx::validate(model_);if(!errors_.empty()||!validation.valid())return {false,std::move(validation),std::move(errors_)};document_.model_=std::move(model_);document_.bones_=std::move(bones_);document_.materials_=std::move(materials_);document_.dirty_=false;document_.rebuildReferences();return {true,std::move(validation),{}}; }

} // namespace mmd
