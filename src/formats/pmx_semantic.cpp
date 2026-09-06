#include <mmd/pmx.hpp>

#include <filesystem>
#include <string>

namespace mmd {
namespace {

struct Comparator {
    SemanticCompareResult result;

    template <typename T> void field(const T& lhs, const T& rhs, const std::string& path) {
        if (!(lhs == rhs))
            result.differences.push_back({path, "values differ"});
    }

    template <typename T> bool count(const std::vector<T>& lhs, const std::vector<T>& rhs,
                                     const std::string& path) {
        field(lhs.size(), rhs.size(), path + ".size");
        return lhs.size() == rhs.size();
    }
};

void vertex(Comparator& c, const PmxVertex& lhs, const PmxVertex& rhs, std::uint8_t additionalUvs,
            const std::string& path) {
    c.field(lhs.position, rhs.position, path + ".position");
    c.field(lhs.normal, rhs.normal, path + ".normal");
    c.field(lhs.uv, rhs.uv, path + ".uv");
    for (std::uint8_t i = 0; i < additionalUvs; ++i)
        c.field(lhs.additionalUv[i], rhs.additionalUv[i], path + ".additionalUv[" + std::to_string(i) + "]");
    c.field(lhs.weightType, rhs.weightType, path + ".weightType");
    if (lhs.weightType != rhs.weightType) return;
    const auto bone = [&](std::size_t i) { c.field(lhs.bones[i], rhs.bones[i], path + ".bones[" + std::to_string(i) + "]"); };
    const auto weight = [&](std::size_t i) { c.field(lhs.weights[i], rhs.weights[i], path + ".weights[" + std::to_string(i) + "]"); };
    switch (lhs.weightType) {
    case PmxWeightType::bdef1: bone(0); break;
    case PmxWeightType::bdef2: bone(0); bone(1); weight(0); break;
    case PmxWeightType::sdef:
        bone(0); bone(1); weight(0);
        c.field(lhs.sdefC, rhs.sdefC, path + ".sdefC");
        c.field(lhs.sdefR0, rhs.sdefR0, path + ".sdefR0");
        c.field(lhs.sdefR1, rhs.sdefR1, path + ".sdefR1");
        break;
    case PmxWeightType::bdef4:
    case PmxWeightType::qdef:
        for (std::size_t i = 0; i < 4; ++i) { bone(i); weight(i); }
        break;
    }
    c.field(lhs.edgeScale, rhs.edgeScale, path + ".edgeScale");
}

void bone(Comparator& c, const PmxBone& lhs, const PmxBone& rhs, const std::string& path) {
    c.field(lhs.name, rhs.name, path + ".name"); c.field(lhs.englishName, rhs.englishName, path + ".englishName");
    c.field(lhs.position, rhs.position, path + ".position"); c.field(lhs.parent, rhs.parent, path + ".parent");
    c.field(lhs.deformLayer, rhs.deformLayer, path + ".deformLayer"); c.field(lhs.flags, rhs.flags, path + ".flags");
    if (lhs.flags != rhs.flags) return;
    if ((lhs.flags & 0x0001U) != 0) c.field(lhs.tailBone, rhs.tailBone, path + ".tailBone");
    else c.field(lhs.tailOffset, rhs.tailOffset, path + ".tailOffset");
    if ((lhs.flags & 0x0300U) != 0) { c.field(lhs.inheritParent, rhs.inheritParent, path + ".inheritParent"); c.field(lhs.inheritRatio, rhs.inheritRatio, path + ".inheritRatio"); }
    if ((lhs.flags & 0x0400U) != 0) c.field(lhs.fixedAxis, rhs.fixedAxis, path + ".fixedAxis");
    if ((lhs.flags & 0x0800U) != 0) { c.field(lhs.localAxisX, rhs.localAxisX, path + ".localAxisX"); c.field(lhs.localAxisZ, rhs.localAxisZ, path + ".localAxisZ"); }
    if ((lhs.flags & 0x2000U) != 0) c.field(lhs.externalParentKey, rhs.externalParentKey, path + ".externalParentKey");
    if ((lhs.flags & 0x0020U) == 0) return;
    c.field(lhs.ikTarget, rhs.ikTarget, path + ".ikTarget"); c.field(lhs.ikLoopCount, rhs.ikLoopCount, path + ".ikLoopCount"); c.field(lhs.ikLimitAngle, rhs.ikLimitAngle, path + ".ikLimitAngle");
    if (!c.count(lhs.ikLinks, rhs.ikLinks, path + ".ikLinks")) return;
    for (std::size_t i = 0; i < lhs.ikLinks.size(); ++i) {
        const auto& a = lhs.ikLinks[i]; const auto& b = rhs.ikLinks[i]; const auto p = path + ".ikLinks[" + std::to_string(i) + "]";
        c.field(a.bone, b.bone, p + ".bone"); c.field(a.limited, b.limited, p + ".limited");
        if (a.limited && b.limited) { c.field(a.minimum, b.minimum, p + ".minimum"); c.field(a.maximum, b.maximum, p + ".maximum"); }
    }
}

void morphOffset(Comparator& c, const PmxMorphOffset& lhs, const PmxMorphOffset& rhs,
                 std::uint8_t type, const std::string& path) {
    c.field(lhs.index, rhs.index, path + ".index");
    if (type == 0 || type == 9) c.field(lhs.scalar, rhs.scalar, path + ".scalar");
    else if (type == 1) c.field(lhs.vector3, rhs.vector3, path + ".vector3");
    else if (type == 2) { c.field(lhs.vector3, rhs.vector3, path + ".translation"); c.field(lhs.vector4, rhs.vector4, path + ".rotation"); }
    else if (type >= 3 && type <= 7) c.field(lhs.vector4, rhs.vector4, path + ".vector4");
    else if (type == 8) { c.field(lhs.operation, rhs.operation, path + ".operation"); for (std::size_t i = 0; i < 7; ++i) c.field(lhs.materialVectors[i], rhs.materialVectors[i], path + ".materialVectors[" + std::to_string(i) + "]"); }
    else if (type == 10) { c.field(lhs.local, rhs.local, path + ".local"); c.field(lhs.vector3, rhs.vector3, path + ".velocity"); c.field(lhs.tertiaryVector3, rhs.tertiaryVector3, path + ".torque"); }
}

template <typename T, typename Equal>
void collection(Comparator& c, const std::vector<T>& lhs, const std::vector<T>& rhs,
                const std::string& path, Equal equal) {
    if (!c.count(lhs, rhs, path)) return;
    for (std::size_t i = 0; i < lhs.size(); ++i)
        equal(lhs[i], rhs[i], path + "[" + std::to_string(i) + "]");
}

std::string logicalTexturePath(std::string path) {
    std::replace(path.begin(), path.end(), '\\', '/');
    return std::filesystem::path(path).lexically_normal().generic_string();
}

} // namespace

SemanticCompareResult pmx::semanticCompare(const PmxModel& lhs, const PmxModel& rhs,
                                           PmxComparisonProfile profile) {
    Comparator c;
    c.field(lhs.metadata.version, rhs.metadata.version, "metadata.version");
    c.field(lhs.metadata.modelName, rhs.metadata.modelName, "metadata.modelName");
    c.field(lhs.metadata.englishName, rhs.metadata.englishName, "metadata.englishName");
    c.field(lhs.metadata.comment, rhs.metadata.comment, "metadata.comment");
    c.field(lhs.metadata.englishComment, rhs.metadata.englishComment, "metadata.englishComment");
    c.field(lhs.metadata.additionalUvCount, rhs.metadata.additionalUvCount, "metadata.additionalUvCount");
    if (lhs.metadata.additionalUvCount != rhs.metadata.additionalUvCount) return std::move(c.result);
    collection(c, lhs.vertices, rhs.vertices, "vertices", [&](const auto& a, const auto& b, const auto& p) { vertex(c, a, b, lhs.metadata.additionalUvCount, p); });
    c.field(lhs.indices, rhs.indices, "indices");
    collection(c, lhs.textures, rhs.textures, "textures", [&](const auto& a, const auto& b, const auto& p) {
        if (profile == PmxComparisonProfile::preservation) c.field(a.storedPath, b.storedPath, p + ".storedPath");
        else c.field(logicalTexturePath(a.storedPath), logicalTexturePath(b.storedPath), p + ".logicalPath");
    });
    collection(c, lhs.materials, rhs.materials, "materials", [&](const auto& a, const auto& b, const auto& p) {
        c.field(a.name,b.name,p+".name"); c.field(a.englishName,b.englishName,p+".englishName"); c.field(a.diffuse,b.diffuse,p+".diffuse"); c.field(a.specular,b.specular,p+".specular"); c.field(a.shininess,b.shininess,p+".shininess"); c.field(a.ambient,b.ambient,p+".ambient"); c.field(a.drawFlags,b.drawFlags,p+".drawFlags"); c.field(a.edgeColor,b.edgeColor,p+".edgeColor"); c.field(a.edgeSize,b.edgeSize,p+".edgeSize"); c.field(a.textureIndex,b.textureIndex,p+".textureIndex"); c.field(a.sphereTextureIndex,b.sphereTextureIndex,p+".sphereTextureIndex"); c.field(a.sphereMode,b.sphereMode,p+".sphereMode"); c.field(a.toonMode,b.toonMode,p+".toonMode"); c.field(a.toonTextureIndex,b.toonTextureIndex,p+".toonTextureIndex"); c.field(a.memo,b.memo,p+".memo"); c.field(a.indexCount,b.indexCount,p+".indexCount");
    });
    collection(c, lhs.bones, rhs.bones, "bones", [&](const auto& a, const auto& b, const auto& p) { bone(c, a, b, p); });
    collection(c, lhs.morphs, rhs.morphs, "morphs", [&](const auto& a, const auto& b, const auto& p) {
        c.field(a.name,b.name,p+".name"); c.field(a.englishName,b.englishName,p+".englishName"); c.field(a.panel,b.panel,p+".panel"); c.field(a.type,b.type,p+".type");
        if (a.type != b.type || !c.count(a.offsets,b.offsets,p+".offsets")) return;
        for (std::size_t i = 0; i < a.offsets.size(); ++i) morphOffset(c,a.offsets[i],b.offsets[i],a.type,p+".offsets["+std::to_string(i)+"]");
    });
    collection(c, lhs.displayFrames, rhs.displayFrames, "displayFrames", [&](const auto& a, const auto& b, const auto& p) {
        c.field(a.name,b.name,p+".name"); c.field(a.englishName,b.englishName,p+".englishName"); c.field(a.special,b.special,p+".special"); if (!c.count(a.items,b.items,p+".items")) return;
        for (std::size_t i=0;i<a.items.size();++i) { c.field(a.items[i].bone,b.items[i].bone,p+".items["+std::to_string(i)+"].bone"); c.field(a.items[i].index,b.items[i].index,p+".items["+std::to_string(i)+"].index"); }
    });
    collection(c, lhs.rigidBodies, rhs.rigidBodies, "rigidBodies", [&](const auto& a, const auto& b, const auto& p) {
        c.field(a.name,b.name,p+".name"); c.field(a.englishName,b.englishName,p+".englishName"); c.field(a.bone,b.bone,p+".bone"); c.field(a.group,b.group,p+".group"); c.field(a.collisionMask,b.collisionMask,p+".collisionMask"); c.field(a.shape,b.shape,p+".shape"); c.field(a.size,b.size,p+".size"); c.field(a.position,b.position,p+".position"); c.field(a.rotation,b.rotation,p+".rotation"); c.field(a.mass,b.mass,p+".mass"); c.field(a.linearDamping,b.linearDamping,p+".linearDamping"); c.field(a.angularDamping,b.angularDamping,p+".angularDamping"); c.field(a.restitution,b.restitution,p+".restitution"); c.field(a.friction,b.friction,p+".friction"); c.field(a.mode,b.mode,p+".mode");
    });
    collection(c, lhs.joints, rhs.joints, "joints", [&](const auto& a, const auto& b, const auto& p) {
        c.field(a.name,b.name,p+".name"); c.field(a.englishName,b.englishName,p+".englishName"); c.field(a.type,b.type,p+".type"); c.field(a.bodyA,b.bodyA,p+".bodyA"); c.field(a.bodyB,b.bodyB,p+".bodyB"); c.field(a.position,b.position,p+".position"); c.field(a.rotation,b.rotation,p+".rotation"); c.field(a.translationMinimum,b.translationMinimum,p+".translationMinimum"); c.field(a.translationMaximum,b.translationMaximum,p+".translationMaximum"); c.field(a.rotationMinimum,b.rotationMinimum,p+".rotationMinimum"); c.field(a.rotationMaximum,b.rotationMaximum,p+".rotationMaximum"); c.field(a.translationSpring,b.translationSpring,p+".translationSpring"); c.field(a.rotationSpring,b.rotationSpring,p+".rotationSpring");
    });
    collection(c, lhs.softBodies, rhs.softBodies, "softBodies", [&](const auto& a, const auto& b, const auto& p) {
        c.field(a.name,b.name,p+".name"); c.field(a.englishName,b.englishName,p+".englishName"); c.field(a.shape,b.shape,p+".shape"); c.field(a.material,b.material,p+".material"); c.field(a.group,b.group,p+".group"); c.field(a.collisionMask,b.collisionMask,p+".collisionMask"); c.field(a.flags,b.flags,p+".flags"); c.field(a.bendingLinkDistance,b.bendingLinkDistance,p+".bendingLinkDistance"); c.field(a.clusterCount,b.clusterCount,p+".clusterCount"); c.field(a.totalMass,b.totalMass,p+".totalMass"); c.field(a.collisionMargin,b.collisionMargin,p+".collisionMargin"); c.field(a.aeroModel,b.aeroModel,p+".aeroModel"); c.field(a.config,b.config,p+".config"); c.field(a.cluster,b.cluster,p+".cluster"); c.field(a.iteration,b.iteration,p+".iteration"); c.field(a.materialConfig,b.materialConfig,p+".materialConfig");
        if (!c.count(a.anchors,b.anchors,p+".anchors")) return;
        for (std::size_t i=0;i<a.anchors.size();++i) { c.field(a.anchors[i].rigidBody,b.anchors[i].rigidBody,p+".anchors["+std::to_string(i)+"].rigidBody"); c.field(a.anchors[i].vertex,b.anchors[i].vertex,p+".anchors["+std::to_string(i)+"].vertex"); c.field(a.anchors[i].nearMode,b.anchors[i].nearMode,p+".anchors["+std::to_string(i)+"].nearMode"); }
        c.field(a.pinnedVertices,b.pinnedVertices,p+".pinnedVertices");
    });
    if (profile == PmxComparisonProfile::preservation) {
        c.field(lhs.format.textEncoding, rhs.format.textEncoding, "format.textEncoding");
        c.field(lhs.format.vertexIndexSize, rhs.format.vertexIndexSize, "format.vertexIndexSize");
        c.field(lhs.format.textureIndexSize, rhs.format.textureIndexSize, "format.textureIndexSize");
        c.field(lhs.format.materialIndexSize, rhs.format.materialIndexSize, "format.materialIndexSize");
        c.field(lhs.format.boneIndexSize, rhs.format.boneIndexSize, "format.boneIndexSize");
        c.field(lhs.format.morphIndexSize, rhs.format.morphIndexSize, "format.morphIndexSize");
        c.field(lhs.format.rigidBodyIndexSize, rhs.format.rigidBodyIndexSize, "format.rigidBodyIndexSize");
    }
    return std::move(c.result);
}

bool pmx::semanticEqual(const PmxModel& lhs, const PmxModel& rhs, PmxComparisonProfile profile) {
    return semanticCompare(lhs, rhs, profile).equal();
}

} // namespace mmd
