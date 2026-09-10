#include <mmd/pmx.hpp>

#include <algorithm>
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

thread_local PmxTextEncoding outputEncoding = PmxTextEncoding::utf8;

std::string utf8ToUtf16Le(std::string_view value) {
    std::string output;
    for (std::size_t index = 0; index < value.size();) {
        const auto lead = static_cast<unsigned char>(value[index++]);
        std::uint32_t codepoint = lead;
        std::size_t trailing = 0;
        if ((lead & 0xe0U) == 0xc0U) {
            codepoint = lead & 0x1fU;
            trailing = 1;
        } else if ((lead & 0xf0U) == 0xe0U) {
            codepoint = lead & 0x0fU;
            trailing = 2;
        } else if ((lead & 0xf8U) == 0xf0U) {
            codepoint = lead & 0x07U;
            trailing = 3;
        }
        for (std::size_t i = 0; i < trailing && index < value.size(); ++i)
            codepoint = (codepoint << 6U) | (static_cast<unsigned char>(value[index++]) & 0x3fU);
        const auto append = [&output](std::uint16_t unit) {
            output.push_back(static_cast<char>(unit));
            output.push_back(static_cast<char>(unit >> 8U));
        };
        if (codepoint <= 0xffffU)
            append(static_cast<std::uint16_t>(codepoint));
        else {
            codepoint -= 0x10000U;
            append(static_cast<std::uint16_t>(0xd800U + (codepoint >> 10U)));
            append(static_cast<std::uint16_t>(0xdc00U + (codepoint & 0x3ffU)));
        }
    }
    return output;
}

template <typename T> void write(std::ostream &output, const T &value, std::string_view field) {
    static_assert(std::is_trivially_copyable_v<T>);
    output.write(reinterpret_cast<const char *>(&value), sizeof(value));
    if (!output)
        throw std::runtime_error("failed while writing PMX " + std::string(field));
}

template <std::size_t N>
void writeArray(std::ostream &output, const std::array<float, N> &value, std::string_view field) {
    output.write(reinterpret_cast<const char *>(value.data()), static_cast<std::streamsize>(sizeof(value)));
    if (!output)
        throw std::runtime_error("failed while writing PMX " + std::string(field));
}

void writeText(std::ostream &output, std::string_view value, std::string_view field) {
    const auto encoded = outputEncoding == PmxTextEncoding::utf16le ? utf8ToUtf16Le(value) : std::string(value);
    if (encoded.size() > static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max()))
        throw std::runtime_error("PMX text is too long: " + std::string(field));
    write(output, static_cast<std::int32_t>(encoded.size()), field);
    output.write(encoded.data(), static_cast<std::streamsize>(encoded.size()));
    if (!output)
        throw std::runtime_error("failed while writing PMX " + std::string(field));
}

void writeIndex(std::ostream &output, std::int32_t value, std::uint8_t width, std::string_view field) {
    switch (width) {
    case 1:
        if (value < std::numeric_limits<std::int8_t>::min() || value > std::numeric_limits<std::int8_t>::max())
            throw std::runtime_error("PMX signed index does not fit in one byte: " + std::string(field));
        write(output, static_cast<std::int8_t>(value), field);
        return;
    case 2:
        if (value < std::numeric_limits<std::int16_t>::min() || value > std::numeric_limits<std::int16_t>::max())
            throw std::runtime_error("PMX signed index does not fit in two bytes: " + std::string(field));
        write(output, static_cast<std::int16_t>(value), field);
        return;
    case 4:
        write(output, value, field);
        return;
    default:
        throw std::runtime_error("invalid PMX signed index width");
    }
}
void writeVertexIndex(std::ostream &output, std::uint32_t value, std::uint8_t width, std::string_view field) {
    switch (width) {
    case 1:
        if (value > std::numeric_limits<std::uint8_t>::max())
            throw std::runtime_error("PMX vertex index does not fit in one byte: " + std::string(field));
        write(output, static_cast<std::uint8_t>(value), field);
        return;
    case 2:
        if (value > std::numeric_limits<std::uint16_t>::max())
            throw std::runtime_error("PMX vertex index does not fit in two bytes: " + std::string(field));
        write(output, static_cast<std::uint16_t>(value), field);
        return;
    case 4:
        write(output, value, field);
        return;
    default:
        throw std::runtime_error("invalid PMX vertex index width");
    }
}
void writeCount(std::ostream &output, std::size_t value, std::string_view field) {
    if (value > static_cast<std::size_t>(std::numeric_limits<std::int32_t>::max()))
        throw std::runtime_error("PMX count is too large: " + std::string(field));
    write(output, static_cast<std::int32_t>(value), field);
}

bool inRange(std::int32_t value, std::size_t count, bool allowNone = true) {
    return (allowNone && value == -1) || (value >= 0 && static_cast<std::size_t>(value) < count);
}
bool finite(float value) {
    return std::isfinite(value);
}
template <std::size_t N> bool finite(const std::array<float, N> &values) {
    for (const auto value : values) {
        if (!finite(value))
            return false;
    }
    return true;
}

void addError(ValidationResult &result, bool condition, std::string message) {
    if (!condition)
        result.issues.push_back({ValidationSeverity::error, ValidationCode::generic, {}, std::move(message)});
}

void addIndexedError(ValidationResult &result, bool condition, std::string message, ReferenceObjectKind kind,
                     std::size_t index) {
    if (!condition) {
        ValidationIssue issue{ValidationSeverity::error, ValidationCode::generic, {}, std::move(message)};
        issue.location.kind = kind;
        issue.location.subIndex = static_cast<std::uint32_t>(index);
        result.issues.push_back(std::move(issue));
    }
}

} // namespace

std::uint8_t pmx::requiredVertexIndexWidth(std::size_t count) noexcept {
    if (count <= 256)
        return 1;
    if (count <= 65'536)
        return 2;
    return 4;
}

std::uint8_t pmx::requiredSignedIndexWidth(std::size_t count) noexcept {
    if (count <= 128)
        return 1;
    if (count <= 32'768)
        return 2;
    return 4;
}

PmxIndexWidths pmx::chooseIndexWidths(const PmxModel &model, PmxSaveOptions options) noexcept {
    const PmxIndexWidths required{
        requiredVertexIndexWidth(model.vertices.size()),  requiredSignedIndexWidth(model.textures.size()),
        requiredSignedIndexWidth(model.materials.size()), requiredSignedIndexWidth(model.bones.size()),
        requiredSignedIndexWidth(model.morphs.size()),    requiredSignedIndexWidth(model.rigidBodies.size()),
    };
    if (options.indexWidths == PmxIndexWidthPolicy::force32)
        return {4, 4, 4, 4, 4, 4};
    if (options.indexWidths == PmxIndexWidthPolicy::minimal)
        return required;
    return {
        std::max(model.format.vertexIndexSize, required.vertex),
        std::max(model.format.textureIndexSize, required.texture),
        std::max(model.format.materialIndexSize, required.material),
        std::max(model.format.boneIndexSize, required.bone),
        std::max(model.format.morphIndexSize, required.morph),
        std::max(model.format.rigidBodyIndexSize, required.rigidBody),
    };
}

ValidationResult pmx::validate(const PmxModel &model) {
    ValidationResult result;
    addError(result, model.metadata.version >= 2.0F && model.metadata.version <= 2.1F, "unsupported PMX version");
    addError(result, model.metadata.additionalUvCount <= 4, "additional UV count exceeds four");
    addError(result, model.indices.size() % 3 == 0, "index count is not divisible by three");
    for (const auto index : model.indices)
        addError(result, index < model.vertices.size(), "vertex index is out of range");
    std::uint64_t materialIndices{};
    for (std::size_t materialIndex = 0; materialIndex < model.materials.size(); ++materialIndex) {
        const auto &material = model.materials[materialIndex];
        materialIndices += material.indexCount;
        addIndexedError(result, inRange(material.textureIndex, model.textures.size()),
                        "material texture index is out of range", ReferenceObjectKind::material, materialIndex);
        addIndexedError(result, inRange(material.sphereTextureIndex, model.textures.size()),
                        "material sphere texture index is out of range", ReferenceObjectKind::material, materialIndex);
        if (material.toonMode == 0)
            addIndexedError(result, inRange(material.toonTextureIndex, model.textures.size()),
                            "material toon texture index is out of range", ReferenceObjectKind::material,
                            materialIndex);
        else
            addError(result, material.toonMode == 1 && material.toonTextureIndex >= 0 && material.toonTextureIndex <= 9,
                     "shared toon index is invalid");
    }
    addError(result, materialIndices == model.indices.size(), "material ranges do not cover indices");
    for (std::size_t vertexIndex = 0; vertexIndex < model.vertices.size(); ++vertexIndex) {
        const auto &vertex = model.vertices[vertexIndex];
        bool vertexFinite = finite(vertex.position) && finite(vertex.normal) && finite(vertex.uv) && finite(vertex.weights) && finite(vertex.edgeScale);
        for (std::size_t i = 0; i < model.metadata.additionalUvCount; ++i)
            vertexFinite = vertexFinite && finite(vertex.additionalUv[i]);
        addIndexedError(result, vertexFinite,
                        "vertex contains non-finite values", ReferenceObjectKind::vertex, vertexIndex);
        if (vertex.weightType == PmxWeightType::sdef)
            addIndexedError(result, finite(vertex.sdefC) && finite(vertex.sdefR0) && finite(vertex.sdefR1),
                            "SDEF parameters contain non-finite values", ReferenceObjectKind::vertex, vertexIndex);
        addIndexedError(result, model.metadata.version >= 2.1F || vertex.weightType != PmxWeightType::qdef,
                        "QDEF requires PMX 2.1", ReferenceObjectKind::vertex, vertexIndex);
        const auto boneCount = vertex.weightType == PmxWeightType::bdef1 ? 1U
                               : vertex.weightType == PmxWeightType::bdef2 || vertex.weightType == PmxWeightType::sdef
                                   ? 2U
                                   : 4U;
        for (std::size_t i = 0; i < boneCount; ++i)
            addIndexedError(result, inRange(vertex.bones[i], model.bones.size()), "vertex bone index is out of range",
                            ReferenceObjectKind::vertex, vertexIndex);
    }
    for (std::size_t boneIndex = 0; boneIndex < model.bones.size(); ++boneIndex) {
        const auto &bone = model.bones[boneIndex];
        addIndexedError(result, finite(bone.position), "bone position contains non-finite values", ReferenceObjectKind::bone, boneIndex);
        if ((bone.flags & 0x0001U) == 0)
            addIndexedError(result, finite(bone.tailOffset), "bone tail offset contains non-finite values", ReferenceObjectKind::bone, boneIndex);
        if ((bone.flags & 0x0300U) != 0)
            addIndexedError(result, finite(bone.inheritRatio), "bone inherit ratio is non-finite", ReferenceObjectKind::bone, boneIndex);
        if ((bone.flags & 0x0400U) != 0)
            addIndexedError(result, finite(bone.fixedAxis), "bone fixed axis contains non-finite values", ReferenceObjectKind::bone, boneIndex);
        if ((bone.flags & 0x0800U) != 0)
            addIndexedError(result, finite(bone.localAxisX) && finite(bone.localAxisZ), "bone local axis contains non-finite values", ReferenceObjectKind::bone, boneIndex);
        addIndexedError(result, inRange(bone.parent, model.bones.size()), "bone parent index is out of range",
                        ReferenceObjectKind::bone, boneIndex);
        if ((bone.flags & 0x0001U) != 0)
            addIndexedError(result, inRange(bone.tailBone, model.bones.size()), "bone tail index is out of range",
                            ReferenceObjectKind::bone, boneIndex);
        if ((bone.flags & 0x0300U) != 0)
            addIndexedError(result, inRange(bone.inheritParent, model.bones.size()),
                            "bone inherit index is out of range", ReferenceObjectKind::bone, boneIndex);
        if ((bone.flags & 0x0020U) != 0) {
            addIndexedError(result, finite(bone.ikLimitAngle), "IK limit angle is non-finite", ReferenceObjectKind::bone, boneIndex);
            addIndexedError(result, inRange(bone.ikTarget, model.bones.size()), "IK target index is out of range",
                            ReferenceObjectKind::bone, boneIndex);
            for (const auto &link : bone.ikLinks)
                addIndexedError(result, inRange(link.bone, model.bones.size()), "IK link index is out of range",
                                ReferenceObjectKind::bone, boneIndex);
            for (const auto &link : bone.ikLinks)
                if (link.limited)
                    addIndexedError(result, finite(link.minimum) && finite(link.maximum), "IK link limit contains non-finite values", ReferenceObjectKind::bone, boneIndex);
        }
    }
    std::vector<std::vector<std::size_t>> dependents(model.bones.size());
    std::vector<std::size_t> indegree(model.bones.size());
    for (std::size_t child = 0; child < model.bones.size(); ++child) {
        const auto addDependency = [&](std::int32_t parent) {
            if (parent >= 0 && static_cast<std::size_t>(parent) < model.bones.size()) {
                dependents[static_cast<std::size_t>(parent)].push_back(child);
                ++indegree[child];
            }
        };
        addDependency(model.bones[child].parent);
        if ((model.bones[child].flags & 0x0300U) != 0)
            addDependency(model.bones[child].inheritParent);
    }
    std::vector<std::size_t> pending;
    for (std::size_t index = 0; index < indegree.size(); ++index)
        if (indegree[index] == 0)
            pending.push_back(index);
    std::size_t visited{};
    while (!pending.empty()) {
        const auto index = pending.back();
        pending.pop_back();
        ++visited;
        for (const auto child : dependents[index])
            if (--indegree[child] == 0)
                pending.push_back(child);
    }
    if (visited != model.bones.size())
        result.issues.push_back(
            {ValidationSeverity::error, ValidationCode::bone_cycle, {}, "bone dependency graph contains a cycle"});
    for (std::size_t morphIndex = 0; morphIndex < model.morphs.size(); ++morphIndex) {
        const auto &morph = model.morphs[morphIndex];
        addError(result, morph.type <= 10, "unknown morph type");
        addIndexedError(result, model.metadata.version >= 2.1F || (morph.type != 9 && morph.type != 10),
                        "flip and impulse morphs require PMX 2.1", ReferenceObjectKind::morph, morphIndex);
        for (const auto &offset : morph.offsets) {
            const auto count = morph.type == 1 || (morph.type >= 3 && morph.type <= 7) ? model.vertices.size()
                               : morph.type == 2                                       ? model.bones.size()
                               : morph.type == 8                                       ? model.materials.size()
                               : morph.type == 10                                      ? model.rigidBodies.size()
                                                                                       : model.morphs.size();
            addIndexedError(result,
                            inRange(offset.index, count, morph.type != 1 && !(morph.type >= 3 && morph.type <= 7)),
                            "morph reference index is out of range", ReferenceObjectKind::morph, morphIndex);
            const bool valuesFinite = (morph.type == 0 || morph.type == 9) ? finite(offset.scalar) : morph.type == 1 ? finite(offset.vector3) : morph.type == 2 ? finite(offset.vector3) && finite(offset.vector4) : (morph.type >= 3 && morph.type <= 7) ? finite(offset.vector4) : morph.type == 8 ? finite(offset.materialVectors[0]) && finite(offset.materialVectors[1]) && finite(offset.materialVectors[2]) && finite(offset.materialVectors[3]) && finite(offset.materialVectors[4]) && finite(offset.materialVectors[5]) && finite(offset.materialVectors[6]) : morph.type == 10 ? finite(offset.vector3) && finite(offset.tertiaryVector3) : true;
            addIndexedError(result, valuesFinite, "morph offset contains non-finite values", ReferenceObjectKind::morph, morphIndex);
            if (morph.type == 8)
                addIndexedError(result, offset.operation <= 1, "material morph operation is invalid", ReferenceObjectKind::morph, morphIndex);
        }
    }
    for (std::size_t frameIndex = 0; frameIndex < model.displayFrames.size(); ++frameIndex)
        for (const auto &item : model.displayFrames[frameIndex].items)
            addIndexedError(result, inRange(item.index, item.bone ? model.bones.size() : model.morphs.size(), false),
                            "display frame index is out of range", ReferenceObjectKind::displayFrame, frameIndex);
    for (std::size_t bodyIndex = 0; bodyIndex < model.rigidBodies.size(); ++bodyIndex)
        addIndexedError(result, inRange(model.rigidBodies[bodyIndex].bone, model.bones.size()),
                        "rigid body bone index is out of range", ReferenceObjectKind::rigidBody, bodyIndex);
    for (std::size_t jointIndex = 0; jointIndex < model.joints.size(); ++jointIndex) {
        const auto &joint = model.joints[jointIndex];
        addIndexedError(result, inRange(joint.bodyA, model.rigidBodies.size()), "joint A body index is out of range",
                        ReferenceObjectKind::joint, jointIndex);
        addIndexedError(result, inRange(joint.bodyB, model.rigidBodies.size()), "joint B body index is out of range",
                        ReferenceObjectKind::joint, jointIndex);
    }
    if (!model.softBodies.empty())
        addError(result, model.metadata.version >= 2.1F, "soft bodies require PMX 2.1");
    return result;
}

PmxSaveReport pmx::save(const std::filesystem::path &path, const PmxModel &model, PmxSaveOptions options) {
    const auto validation = validate(model);
    if (!validation.valid())
        throw std::runtime_error("cannot save invalid PMX: " + validation.issues.front().message);
    std::ofstream output(path, std::ios::binary | std::ios::trunc);
    if (!output)
        throw std::runtime_error("cannot open PMX for writing: " + path.string());

    output.write("PMX ", 4);
    const auto version = model.metadata.version;
    write(output, version, "version");
    write(output, std::uint8_t{8}, "header size");
    const auto encoding =
        options.mode == PmxSaveMode::preserve ? static_cast<std::uint8_t>(model.format.textEncoding) : std::uint8_t{1};
    outputEncoding = static_cast<PmxTextEncoding>(encoding);
    const auto widths = chooseIndexWidths(model, options);
    const std::array<std::uint8_t, 8> settings{encoding,        model.metadata.additionalUvCount,
                                               widths.vertex,   widths.texture,
                                               widths.material, widths.bone,
                                               widths.morph,    widths.rigidBody};
    output.write(reinterpret_cast<const char *>(settings.data()), static_cast<std::streamsize>(settings.size()));
    writeText(output, model.metadata.modelName, "model name");
    writeText(output, model.metadata.englishName, "English model name");
    writeText(output, model.metadata.comment, "comment");
    writeText(output, model.metadata.englishComment, "English comment");
    writeCount(output, model.vertices.size(), "vertex count");
    for (const auto &vertex : model.vertices) {
        writeArray(output, vertex.position, "vertex position");
        writeArray(output, vertex.normal, "vertex normal");
        writeArray(output, vertex.uv, "vertex UV");
        for (std::uint8_t i = 0; i < model.metadata.additionalUvCount; ++i)
            writeArray(output, vertex.additionalUv[i], "additional UV");
        write(output, static_cast<std::uint8_t>(vertex.weightType), "weight type");
        const auto writeBone = [&](std::size_t i) { writeIndex(output, vertex.bones[i], widths.bone, "vertex bone"); };
        switch (vertex.weightType) {
        case PmxWeightType::bdef1:
            writeBone(0);
            break;
        case PmxWeightType::bdef2:
            writeBone(0);
            writeBone(1);
            write(output, vertex.weights[0], "BDEF2 weight");
            break;
        case PmxWeightType::bdef4:
        case PmxWeightType::qdef:
            for (std::size_t i = 0; i < 4; ++i)
                writeBone(i);
            writeArray(output, vertex.weights, "BDEF4 weight");
            break;
        case PmxWeightType::sdef:
            writeBone(0);
            writeBone(1);
            write(output, vertex.weights[0], "SDEF weight");
            writeArray(output, vertex.sdefC, "SDEF C");
            writeArray(output, vertex.sdefR0, "SDEF R0");
            writeArray(output, vertex.sdefR1, "SDEF R1");
            break;
        }
        write(output, vertex.edgeScale, "edge scale");
    }
    writeCount(output, model.indices.size(), "index count");
    for (const auto index : model.indices)
        writeVertexIndex(output, index, widths.vertex, "vertex index");
    writeCount(output, model.textures.size(), "texture count");
    for (const auto &texture : model.textures)
        writeText(output, texture.storedPath, "texture");
    writeCount(output, model.materials.size(), "material count");
    for (const auto &material : model.materials) {
        writeText(output, material.name, "material name");
        writeText(output, material.englishName, "material English name");
        writeArray(output, material.diffuse, "material diffuse");
        writeArray(output, material.specular, "material specular");
        write(output, material.shininess, "material shininess");
        writeArray(output, material.ambient, "material ambient");
        write(output, material.drawFlags, "material flags");
        writeArray(output, material.edgeColor, "edge color");
        write(output, material.edgeSize, "edge size");
        writeIndex(output, material.textureIndex, widths.texture, "texture index");
        writeIndex(output, material.sphereTextureIndex, widths.texture, "sphere texture index");
        write(output, material.sphereMode, "sphere mode");
        write(output, material.toonMode, "toon mode");
        if (material.toonMode == 0)
            writeIndex(output, material.toonTextureIndex, widths.texture, "toon texture index");
        else
            write(output, static_cast<std::uint8_t>(material.toonTextureIndex), "shared toon index");
        writeText(output, material.memo, "material memo");
        writeCount(output, material.indexCount, "material index count");
    }
    writeCount(output, model.bones.size(), "bone count");
    for (const auto &bone : model.bones) {
        writeText(output, bone.name, "bone name");
        writeText(output, bone.englishName, "bone English name");
        writeArray(output, bone.position, "bone position");
        writeIndex(output, bone.parent, widths.bone, "bone parent");
        write(output, bone.deformLayer, "bone layer");
        write(output, bone.flags, "bone flags");
        if ((bone.flags & 0x0001U) != 0)
            writeIndex(output, bone.tailBone, widths.bone, "bone tail");
        else
            writeArray(output, bone.tailOffset, "bone tail offset");
        if ((bone.flags & 0x0300U) != 0) {
            writeIndex(output, bone.inheritParent, widths.bone, "bone inherit parent");
            write(output, bone.inheritRatio, "bone inherit ratio");
        }
        if ((bone.flags & 0x0400U) != 0)
            writeArray(output, bone.fixedAxis, "bone fixed axis");
        if ((bone.flags & 0x0800U) != 0) {
            writeArray(output, bone.localAxisX, "bone local X");
            writeArray(output, bone.localAxisZ, "bone local Z");
        }
        if ((bone.flags & 0x2000U) != 0)
            write(output, bone.externalParentKey, "external parent key");
        if ((bone.flags & 0x0020U) != 0) {
            writeIndex(output, bone.ikTarget, widths.bone, "IK target");
            write(output, bone.ikLoopCount, "IK loop count");
            write(output, bone.ikLimitAngle, "IK angle");
            writeCount(output, bone.ikLinks.size(), "IK link count");
            for (const auto &link : bone.ikLinks) {
                writeIndex(output, link.bone, widths.bone, "IK link bone");
                write(output, static_cast<std::uint8_t>(link.limited), "IK limited");
                if (link.limited) {
                    writeArray(output, link.minimum, "IK minimum");
                    writeArray(output, link.maximum, "IK maximum");
                }
            }
        }
    }
    writeCount(output, model.morphs.size(), "morph count");
    for (const auto &morph : model.morphs) {
        writeText(output, morph.name, "morph name");
        writeText(output, morph.englishName, "morph English name");
        write(output, morph.panel, "morph panel");
        write(output, morph.type, "morph type");
        writeCount(output, morph.offsets.size(), "morph offset count");
        for (const auto &offset : morph.offsets) {
            if (morph.type == 0 || morph.type == 9) {
                writeIndex(output, offset.index, widths.morph, "group morph index");
                write(output, offset.scalar, "group morph weight");
            } else if (morph.type == 1) {
                writeVertexIndex(output, static_cast<std::uint32_t>(offset.index), widths.vertex, "vertex morph index");
                writeArray(output, offset.vector3, "vertex morph offset");
            } else if (morph.type == 2) {
                writeIndex(output, offset.index, widths.bone, "bone morph index");
                writeArray(output, offset.vector3, "bone morph translation");
                writeArray(output, offset.vector4, "bone morph rotation");
            } else if (morph.type >= 3 && morph.type <= 7) {
                writeVertexIndex(output, static_cast<std::uint32_t>(offset.index), widths.vertex, "UV morph index");
                writeArray(output, offset.vector4, "UV morph offset");
            } else if (morph.type == 8) {
                writeIndex(output, offset.index, widths.material, "material morph index");
                write(output, offset.operation, "material morph operation");
                writeArray(output, offset.materialVectors[0], "morph diffuse");
                const Float3 specular{offset.materialVectors[1][0], offset.materialVectors[1][1],
                                      offset.materialVectors[1][2]};
                writeArray(output, specular, "morph specular");
                write(output, offset.materialVectors[1][3], "morph power");
                const Float3 ambient{offset.materialVectors[2][0], offset.materialVectors[2][1],
                                     offset.materialVectors[2][2]};
                writeArray(output, ambient, "morph ambient");
                writeArray(output, offset.materialVectors[3], "morph edge color");
                write(output, offset.materialVectors[2][3], "morph edge size");
                for (std::size_t i = 4; i <= 6; ++i)
                    writeArray(output, offset.materialVectors[i], "morph texture data");
            } else {
                writeIndex(output, offset.index, widths.rigidBody, "impulse rigid body");
                write(output, static_cast<std::uint8_t>(offset.local), "impulse local");
                writeArray(output, offset.vector3, "impulse velocity");
                writeArray(output, offset.tertiaryVector3, "impulse torque");
            }
        }
    }
    writeCount(output, model.displayFrames.size(), "display frame count");
    for (const auto &frame : model.displayFrames) {
        writeText(output, frame.name, "display frame name");
        writeText(output, frame.englishName, "display frame English name");
        write(output, static_cast<std::uint8_t>(frame.special), "display special");
        writeCount(output, frame.items.size(), "display item count");
        for (const auto &item : frame.items) {
            write(output, static_cast<std::uint8_t>(!item.bone), "display item type");
            writeIndex(output, item.index, item.bone ? widths.bone : widths.morph, "display item index");
        }
    }
    writeCount(output, model.rigidBodies.size(), "rigid body count");
    for (const auto &body : model.rigidBodies) {
        writeText(output, body.name, "body name");
        writeText(output, body.englishName, "body English name");
        writeIndex(output, body.bone, widths.bone, "body bone");
        write(output, body.group, "body group");
        write(output, body.collisionMask, "body collision mask");
        write(output, body.shape, "body shape");
        writeArray(output, body.size, "body size");
        writeArray(output, body.position, "body position");
        writeArray(output, body.rotation, "body rotation");
        write(output, body.mass, "body mass");
        write(output, body.linearDamping, "body linear damping");
        write(output, body.angularDamping, "body angular damping");
        write(output, body.restitution, "body restitution");
        write(output, body.friction, "body friction");
        write(output, body.mode, "body mode");
    }
    writeCount(output, model.joints.size(), "joint count");
    for (const auto &joint : model.joints) {
        writeText(output, joint.name, "joint name");
        writeText(output, joint.englishName, "joint English name");
        write(output, joint.type, "joint type");
        writeIndex(output, joint.bodyA, widths.rigidBody, "joint A");
        writeIndex(output, joint.bodyB, widths.rigidBody, "joint B");
        writeArray(output, joint.position, "joint position");
        writeArray(output, joint.rotation, "joint rotation");
        writeArray(output, joint.translationMinimum, "joint translation minimum");
        writeArray(output, joint.translationMaximum, "joint translation maximum");
        writeArray(output, joint.rotationMinimum, "joint rotation minimum");
        writeArray(output, joint.rotationMaximum, "joint rotation maximum");
        writeArray(output, joint.translationSpring, "joint translation spring");
        writeArray(output, joint.rotationSpring, "joint rotation spring");
    }
    if (model.metadata.version >= 2.1F) {
        writeCount(output, model.softBodies.size(), "soft body count");
        for (const auto &body : model.softBodies) {
            writeText(output, body.name, "soft body name");
            writeText(output, body.englishName, "soft body English name");
            write(output, body.shape, "soft body shape");
            writeIndex(output, body.material, widths.material, "soft body material");
            write(output, body.group, "soft body group");
            write(output, body.collisionMask, "soft body collision mask");
            write(output, body.flags, "soft body flags");
            write(output, body.bendingLinkDistance, "soft body bending distance");
            write(output, body.clusterCount, "soft body cluster count");
            write(output, body.totalMass, "soft body mass");
            write(output, body.collisionMargin, "soft body margin");
            write(output, body.aeroModel, "soft body aero model");
            writeArray(output, body.config, "soft body config");
            writeArray(output, body.cluster, "soft body cluster");
            for (const auto value : body.iteration)
                write(output, value, "soft body iteration");
            writeArray(output, body.materialConfig, "soft body material config");
            writeCount(output, body.anchors.size(), "soft body anchor count");
            for (const auto &anchor : body.anchors) {
                writeIndex(output, anchor.rigidBody, widths.rigidBody, "soft body anchor body");
                writeVertexIndex(output, static_cast<std::uint32_t>(anchor.vertex), widths.vertex,
                                 "soft body anchor vertex");
                write(output, static_cast<std::uint8_t>(anchor.nearMode), "soft body anchor mode");
            }
            writeCount(output, body.pinnedVertices.size(), "soft body pin count");
            for (const auto vertex : body.pinnedVertices)
                writeVertexIndex(output, static_cast<std::uint32_t>(vertex), widths.vertex, "soft body pin vertex");
        }
    }
    outputEncoding = PmxTextEncoding::utf8;
    return {.changedEncoding =
                options.mode == PmxSaveMode::canonical && model.format.textEncoding != PmxTextEncoding::utf8,
            .vertex = {model.format.vertexIndexSize, widths.vertex},
            .texture = {model.format.textureIndexSize, widths.texture},
            .material = {model.format.materialIndexSize, widths.material},
            .bone = {model.format.boneIndexSize, widths.bone},
            .morph = {model.format.morphIndexSize, widths.morph},
            .rigidBody = {model.format.rigidBodyIndexSize, widths.rigidBody}};
}

std::filesystem::path pmx::resolveTexturePath(const PmxModel &model, std::size_t textureIndex) {
    if (textureIndex >= model.textures.size())
        throw std::out_of_range("PMX texture index is out of range");
    auto stored = model.textures[textureIndex].storedPath;
    std::replace(stored.begin(), stored.end(), '\\', '/');
    return (model.sourcePath.parent_path() / std::filesystem::path(stored)).lexically_normal();
}

} // namespace mmd
