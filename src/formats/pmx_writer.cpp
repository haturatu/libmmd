#include <mmd/pmx.hpp>

#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <string_view>
#include <type_traits>

namespace mmd {
namespace {

template <typename T> void write(std::ostream& output, const T& value, std::string_view field) {
    static_assert(std::is_trivially_copyable_v<T>);
    output.write(reinterpret_cast<const char*>(&value), sizeof(value));
    if (!output)
        throw std::runtime_error("failed while writing PMX " + std::string(field));
}

template <std::size_t N> void writeArray(std::ostream& output, const std::array<float, N>& value, std::string_view field) {
    output.write(reinterpret_cast<const char*>(value.data()), static_cast<std::streamsize>(sizeof(value)));
    if (!output)
        throw std::runtime_error("failed while writing PMX " + std::string(field));
}

void writeText(std::ostream& output, std::string_view value, std::string_view field) {
    if (value.size() > static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max()))
        throw std::runtime_error("PMX text is too long: " + std::string(field));
    write(output, static_cast<std::int32_t>(value.size()), field);
    output.write(value.data(), static_cast<std::streamsize>(value.size()));
    if (!output)
        throw std::runtime_error("failed while writing PMX " + std::string(field));
}

void writeIndex(std::ostream& output, std::int32_t value, std::string_view field) {
    write(output, value, field);
}
void writeVertexIndex(std::ostream& output, std::uint32_t value, std::string_view field) {
    write(output, value, field);
}
void writeCount(std::ostream& output, std::size_t value, std::string_view field) {
    if (value > static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max()))
        throw std::runtime_error("PMX count is too large: " + std::string(field));
    write(output, static_cast<std::int32_t>(value), field);
}

bool inRange(std::int32_t value, std::size_t count, bool allowNone = true) {
    return (allowNone && value == -1) || (value >= 0 && static_cast<std::size_t>(value) < count);
}
bool finite(float value) { return std::isfinite(value); }
template <std::size_t N> bool finite(const std::array<float, N>& values) {
    for (const auto value : values) {
        if (!finite(value))
            return false;
    }
    return true;
}

void addError(ValidationResult& result, bool condition, std::string message) {
    if (!condition)
        result.errors.push_back(std::move(message));
}

} // namespace

ValidationResult pmx::validate(const PmxModel& model) {
    ValidationResult result;
    addError(result, model.metadata.version >= 2.0F && model.metadata.version <= 2.1F, "unsupported PMX version");
    addError(result, model.metadata.additionalUvCount <= 4, "additional UV count exceeds four");
    addError(result, model.indices.size() % 3 == 0, "index count is not divisible by three");
    for (const auto index : model.indices)
        addError(result, index < model.vertices.size(), "vertex index is out of range");
    std::uint64_t materialIndices{};
    for (const auto& material : model.materials) {
        materialIndices += material.indexCount;
        addError(result, inRange(material.textureIndex, model.textures.size()), "material texture index is out of range");
        addError(result, inRange(material.sphereTextureIndex, model.textures.size()), "material sphere texture index is out of range");
        if (material.toonMode == 0)
            addError(result, inRange(material.toonTextureIndex, model.textures.size()), "material toon texture index is out of range");
        else
            addError(result, material.toonMode == 1 && material.toonTextureIndex >= 0 && material.toonTextureIndex <= 9,
                     "shared toon index is invalid");
    }
    addError(result, materialIndices == model.indices.size(), "material ranges do not cover indices");
    for (const auto& vertex : model.vertices) {
        addError(result, finite(vertex.position) && finite(vertex.normal) && finite(vertex.uv) && finite(vertex.weights),
                 "vertex contains non-finite values");
        const auto boneCount = vertex.weightType == PmxWeightType::bdef1 ? 1U
                           : vertex.weightType == PmxWeightType::bdef2 || vertex.weightType == PmxWeightType::sdef ? 2U
                                                                                                                    : 4U;
        for (std::size_t i = 0; i < boneCount; ++i)
            addError(result, inRange(vertex.bones[i], model.bones.size()), "vertex bone index is out of range");
    }
    for (const auto& bone : model.bones) {
        addError(result, inRange(bone.parent, model.bones.size()), "bone parent index is out of range");
        if ((bone.flags & 0x0001U) != 0)
            addError(result, inRange(bone.tailBone, model.bones.size()), "bone tail index is out of range");
        if ((bone.flags & 0x0300U) != 0)
            addError(result, inRange(bone.inheritParent, model.bones.size()), "bone inherit index is out of range");
        if ((bone.flags & 0x0020U) != 0) {
            addError(result, inRange(bone.ikTarget, model.bones.size()), "IK target index is out of range");
            for (const auto& link : bone.ikLinks)
                addError(result, inRange(link.bone, model.bones.size()), "IK link index is out of range");
        }
    }
    for (const auto& morph : model.morphs) {
        addError(result, morph.type <= 10, "unknown morph type");
        for (const auto& offset : morph.offsets) {
            const auto count = morph.type == 1 || (morph.type >= 3 && morph.type <= 7) ? model.vertices.size()
                             : morph.type == 2 ? model.bones.size()
                             : morph.type == 8 ? model.materials.size()
                             : morph.type == 10 ? model.rigidBodies.size()
                                                : model.morphs.size();
            addError(result, inRange(offset.index, count, morph.type != 1 && !(morph.type >= 3 && morph.type <= 7)),
                     "morph reference index is out of range");
        }
    }
    for (const auto& frame : model.displayFrames)
        for (const auto& item : frame.items)
            addError(result, inRange(item.index, item.bone ? model.bones.size() : model.morphs.size(), false),
                     "display frame index is out of range");
    for (const auto& body : model.rigidBodies)
        addError(result, inRange(body.bone, model.bones.size()), "rigid body bone index is out of range");
    for (const auto& joint : model.joints) {
        addError(result, inRange(joint.bodyA, model.rigidBodies.size()), "joint A body index is out of range");
        addError(result, inRange(joint.bodyB, model.rigidBodies.size()), "joint B body index is out of range");
    }
    if (!model.softBodies.empty())
        addError(result, model.metadata.version >= 2.1F, "soft bodies require PMX 2.1");
    return result;
}

void pmx::save(const std::filesystem::path& path, const PmxModel& model) {
    const auto validation = validate(model);
    if (!validation.valid())
        throw std::runtime_error("cannot save invalid PMX: " + validation.errors.front());
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output)
        throw std::runtime_error("cannot open PMX for writing: " + path.string());

    output.write("PMX ", 4);
    write(output, model.metadata.version, "version");
    write(output, std::uint8_t{8}, "header size");
    const std::array<std::uint8_t, 8> settings{1, model.metadata.additionalUvCount, 4, 4, 4, 4, 4, 4};
    output.write(reinterpret_cast<const char*>(settings.data()), static_cast<std::streamsize>(settings.size()));
    writeText(output, model.metadata.modelName, "model name");
    writeText(output, model.metadata.englishName, "English model name");
    writeText(output, model.metadata.comment, "comment");
    writeText(output, model.metadata.englishComment, "English comment");
    writeCount(output, model.vertices.size(), "vertex count");
    for (const auto& vertex : model.vertices) {
        writeArray(output, vertex.position, "vertex position");
        writeArray(output, vertex.normal, "vertex normal");
        writeArray(output, vertex.uv, "vertex UV");
        for (std::uint8_t i = 0; i < model.metadata.additionalUvCount; ++i)
            writeArray(output, vertex.additionalUv[i], "additional UV");
        write(output, static_cast<std::uint8_t>(vertex.weightType), "weight type");
        const auto writeBone = [&](std::size_t i) { writeIndex(output, vertex.bones[i], "vertex bone"); };
        switch (vertex.weightType) {
        case PmxWeightType::bdef1:
            writeBone(0);
            break;
        case PmxWeightType::bdef2:
            writeBone(0); writeBone(1); write(output, vertex.weights[0], "BDEF2 weight");
            break;
        case PmxWeightType::bdef4:
        case PmxWeightType::qdef:
            for (std::size_t i = 0; i < 4; ++i) writeBone(i);
            writeArray(output, vertex.weights, "BDEF4 weight");
            break;
        case PmxWeightType::sdef:
            writeBone(0); writeBone(1); write(output, vertex.weights[0], "SDEF weight");
            writeArray(output, vertex.sdefC, "SDEF C"); writeArray(output, vertex.sdefR0, "SDEF R0"); writeArray(output, vertex.sdefR1, "SDEF R1");
            break;
        }
        write(output, vertex.edgeScale, "edge scale");
    }
    writeCount(output, model.indices.size(), "index count");
    for (const auto index : model.indices) writeVertexIndex(output, index, "vertex index");
    writeCount(output, model.textures.size(), "texture count");
    for (const auto& texture : model.textures) writeText(output, texture.generic_string(), "texture");
    writeCount(output, model.materials.size(), "material count");
    for (const auto& material : model.materials) {
        writeText(output, material.name, "material name"); writeText(output, material.englishName, "material English name");
        writeArray(output, material.diffuse, "material diffuse"); writeArray(output, material.specular, "material specular"); write(output, material.shininess, "material shininess"); writeArray(output, material.ambient, "material ambient");
        write(output, material.drawFlags, "material flags"); writeArray(output, material.edgeColor, "edge color"); write(output, material.edgeSize, "edge size");
        writeIndex(output, material.textureIndex, "texture index"); writeIndex(output, material.sphereTextureIndex, "sphere texture index"); write(output, material.sphereMode, "sphere mode"); write(output, material.toonMode, "toon mode");
        if (material.toonMode == 0) writeIndex(output, material.toonTextureIndex, "toon texture index"); else write(output, static_cast<std::uint8_t>(material.toonTextureIndex), "shared toon index");
        writeText(output, material.memo, "material memo"); writeCount(output, material.indexCount, "material index count");
    }
    writeCount(output, model.bones.size(), "bone count");
    for (const auto& bone : model.bones) {
        writeText(output, bone.name, "bone name"); writeText(output, bone.englishName, "bone English name"); writeArray(output, bone.position, "bone position"); writeIndex(output, bone.parent, "bone parent"); write(output, bone.deformLayer, "bone layer"); write(output, bone.flags, "bone flags");
        if ((bone.flags & 0x0001U) != 0) writeIndex(output, bone.tailBone, "bone tail"); else writeArray(output, bone.tailOffset, "bone tail offset");
        if ((bone.flags & 0x0300U) != 0) { writeIndex(output, bone.inheritParent, "bone inherit parent"); write(output, bone.inheritRatio, "bone inherit ratio"); }
        if ((bone.flags & 0x0400U) != 0) writeArray(output, bone.fixedAxis, "bone fixed axis");
        if ((bone.flags & 0x0800U) != 0) { writeArray(output, bone.localAxisX, "bone local X"); writeArray(output, bone.localAxisZ, "bone local Z"); }
        if ((bone.flags & 0x2000U) != 0) write(output, bone.externalParentKey, "external parent key");
        if ((bone.flags & 0x0020U) != 0) { writeIndex(output, bone.ikTarget, "IK target"); write(output, bone.ikLoopCount, "IK loop count"); write(output, bone.ikLimitAngle, "IK angle"); writeCount(output, bone.ikLinks.size(), "IK link count"); for (const auto& link : bone.ikLinks) { writeIndex(output, link.bone, "IK link bone"); write(output, static_cast<std::uint8_t>(link.limited), "IK limited"); if (link.limited) { writeArray(output, link.minimum, "IK minimum"); writeArray(output, link.maximum, "IK maximum"); } } }
    }
    writeCount(output, model.morphs.size(), "morph count");
    for (const auto& morph : model.morphs) {
        writeText(output, morph.name, "morph name"); writeText(output, morph.englishName, "morph English name"); write(output, morph.panel, "morph panel"); write(output, morph.type, "morph type"); writeCount(output, morph.offsets.size(), "morph offset count");
        for (const auto& offset : morph.offsets) {
            if (morph.type == 0 || morph.type == 9) { writeIndex(output, offset.index, "group morph index"); write(output, offset.scalar, "group morph weight"); }
            else if (morph.type == 1) { writeVertexIndex(output, static_cast<std::uint32_t>(offset.index), "vertex morph index"); writeArray(output, offset.vector3, "vertex morph offset"); }
            else if (morph.type == 2) { writeIndex(output, offset.index, "bone morph index"); writeArray(output, offset.vector3, "bone morph translation"); writeArray(output, offset.vector4, "bone morph rotation"); }
            else if (morph.type >= 3 && morph.type <= 7) { writeVertexIndex(output, static_cast<std::uint32_t>(offset.index), "UV morph index"); writeArray(output, offset.vector4, "UV morph offset"); }
            else if (morph.type == 8) { writeIndex(output, offset.index, "material morph index"); write(output, offset.operation, "material morph operation"); writeArray(output, offset.materialVectors[0], "morph diffuse"); const Float3 specular{offset.materialVectors[1][0], offset.materialVectors[1][1], offset.materialVectors[1][2]}; writeArray(output, specular, "morph specular"); write(output, offset.materialVectors[1][3], "morph power"); const Float3 ambient{offset.materialVectors[2][0], offset.materialVectors[2][1], offset.materialVectors[2][2]}; writeArray(output, ambient, "morph ambient"); writeArray(output, offset.materialVectors[3], "morph edge color"); write(output, offset.materialVectors[2][3], "morph edge size"); for (std::size_t i = 4; i <= 6; ++i) writeArray(output, offset.materialVectors[i], "morph texture data"); }
            else { writeIndex(output, offset.index, "impulse rigid body"); write(output, static_cast<std::uint8_t>(offset.local), "impulse local"); writeArray(output, offset.vector3, "impulse velocity"); writeArray(output, offset.tertiaryVector3, "impulse torque"); }
        }
    }
    writeCount(output, model.displayFrames.size(), "display frame count");
    for (const auto& frame : model.displayFrames) { writeText(output, frame.name, "display frame name"); writeText(output, frame.englishName, "display frame English name"); write(output, static_cast<std::uint8_t>(frame.special), "display special"); writeCount(output, frame.items.size(), "display item count"); for (const auto& item : frame.items) { write(output, static_cast<std::uint8_t>(!item.bone), "display item type"); writeIndex(output, item.index, "display item index"); } }
    writeCount(output, model.rigidBodies.size(), "rigid body count");
    for (const auto& body : model.rigidBodies) { writeText(output, body.name, "body name"); writeText(output, body.englishName, "body English name"); writeIndex(output, body.bone, "body bone"); write(output, body.group, "body group"); write(output, body.collisionMask, "body collision mask"); write(output, body.shape, "body shape"); writeArray(output, body.size, "body size"); writeArray(output, body.position, "body position"); writeArray(output, body.rotation, "body rotation"); write(output, body.mass, "body mass"); write(output, body.linearDamping, "body linear damping"); write(output, body.angularDamping, "body angular damping"); write(output, body.restitution, "body restitution"); write(output, body.friction, "body friction"); write(output, body.mode, "body mode"); }
    writeCount(output, model.joints.size(), "joint count");
    for (const auto& joint : model.joints) { writeText(output, joint.name, "joint name"); writeText(output, joint.englishName, "joint English name"); write(output, joint.type, "joint type"); writeIndex(output, joint.bodyA, "joint A"); writeIndex(output, joint.bodyB, "joint B"); writeArray(output, joint.position, "joint position"); writeArray(output, joint.rotation, "joint rotation"); writeArray(output, joint.translationMinimum, "joint translation minimum"); writeArray(output, joint.translationMaximum, "joint translation maximum"); writeArray(output, joint.rotationMinimum, "joint rotation minimum"); writeArray(output, joint.rotationMaximum, "joint rotation maximum"); writeArray(output, joint.translationSpring, "joint translation spring"); writeArray(output, joint.rotationSpring, "joint rotation spring"); }
    if (model.metadata.version >= 2.1F) {
        writeCount(output, model.softBodies.size(), "soft body count");
        for (const auto& body : model.softBodies) { writeText(output, body.name, "soft body name"); writeText(output, body.englishName, "soft body English name"); write(output, body.shape, "soft body shape"); writeIndex(output, body.material, "soft body material"); write(output, body.group, "soft body group"); write(output, body.collisionMask, "soft body collision mask"); write(output, body.flags, "soft body flags"); write(output, body.bendingLinkDistance, "soft body bending distance"); write(output, body.clusterCount, "soft body cluster count"); write(output, body.totalMass, "soft body mass"); write(output, body.collisionMargin, "soft body margin"); write(output, body.aeroModel, "soft body aero model"); writeArray(output, body.config, "soft body config"); writeArray(output, body.cluster, "soft body cluster"); for (const auto value : body.iteration) write(output, value, "soft body iteration"); writeArray(output, body.materialConfig, "soft body material config"); writeCount(output, body.anchors.size(), "soft body anchor count"); for (const auto& anchor : body.anchors) { writeIndex(output, anchor.rigidBody, "soft body anchor body"); writeVertexIndex(output, static_cast<std::uint32_t>(anchor.vertex), "soft body anchor vertex"); write(output, static_cast<std::uint8_t>(anchor.nearMode), "soft body anchor mode"); } writeCount(output, body.pinnedVertices.size(), "soft body pin count"); for (const auto vertex : body.pinnedVertices) writeVertexIndex(output, static_cast<std::uint32_t>(vertex), "soft body pin vertex"); }
    }
}

} // namespace mmd
