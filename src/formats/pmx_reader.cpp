#include "log.hpp"
#include "mapped_file.hpp"
#include <mmd/pmx.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <fstream>
#include <limits>
#include <numeric>
#include <stdexcept>
#include <string_view>
#include <type_traits>

namespace mmd {
namespace {

constexpr std::int32_t maxElements = 300'000'000;
constexpr std::size_t maxDecodedBytes = 512U * 1024U * 1024U;

std::size_t remainingBytes(std::istream &input) {
    const auto position = input.tellg();
    input.seekg(0, std::ios::end);
    const auto end = input.tellg();
    input.seekg(position);
    if (position < 0 || end < position)
        throw std::runtime_error("cannot determine remaining PMX bytes");
    return static_cast<std::size_t>(end - position);
}

struct DecodedBudget {
    std::size_t remaining{maxDecodedBytes};
};
thread_local DecodedBudget *activeBudget{};

class ScopedDecodedBudget final {
  public:
    explicit ScopedDecodedBudget(DecodedBudget &budget) : previous_(activeBudget) {
        activeBudget = &budget;
    }
    ~ScopedDecodedBudget() {
        activeBudget = previous_;
    }
    ScopedDecodedBudget(const ScopedDecodedBudget &) = delete;
    ScopedDecodedBudget &operator=(const ScopedDecodedBudget &) = delete;

  private:
    DecodedBudget *previous_{};
};

template <typename T>
void checkedResize(std::vector<T> &destination, std::size_t count, std::istream &input, std::size_t minimumEncodedBytes,
                   std::string_view field, DecodedBudget &budget) {
    if (minimumEncodedBytes == 0 || count > remainingBytes(input) / minimumEncodedBytes ||
        count > budget.remaining / sizeof(T))
        throw std::runtime_error("implausible PMX " + std::string(field));
    budget.remaining -= count * sizeof(T);
    destination.resize(count);
}

template <typename T>
void checkedResize(std::vector<T> &destination, std::size_t count, std::istream &input, std::size_t minimumEncodedBytes,
                   std::string_view field) {
    if (activeBudget == nullptr)
        throw std::runtime_error("missing PMX decoded allocation budget");
    checkedResize(destination, count, input, minimumEncodedBytes, field, *activeBudget);
}

template <typename T> T read(std::istream &input, std::string_view field) {
    static_assert(std::is_trivially_copyable_v<T>);
    T value{};
    input.read(reinterpret_cast<char *>(&value), sizeof(value));
    if (!input)
        throw std::runtime_error("truncated PMX while reading " + std::string(field));
    return value;
}

template <std::size_t N> std::array<float, N> readFloatArray(std::istream &input, std::string_view field) {
    std::array<float, N> value{};
    input.read(reinterpret_cast<char *>(value.data()), static_cast<std::streamsize>(sizeof(value)));
    if (!input)
        throw std::runtime_error("truncated PMX while reading " + std::string(field));
    return value;
}

std::int32_t readCount(std::istream &input, std::string_view field, std::int32_t maximum = maxElements) {
    const auto count = read<std::int32_t>(input, field);
    if (count < 0 || count > maximum)
        throw std::runtime_error("invalid PMX " + std::string(field));
    return count;
}

std::string utf16LeToUtf8(std::string_view bytes) {
    std::string output;
    output.reserve(bytes.size());
    for (std::size_t i = 0; i + 1 < bytes.size(); i += 2) {
        const auto lo = static_cast<unsigned char>(bytes[i]);
        const auto hi = static_cast<unsigned char>(bytes[i + 1]);
        std::uint32_t codepoint = static_cast<std::uint32_t>(lo) | (static_cast<std::uint32_t>(hi) << 8U);
        if (codepoint >= 0xD800U && codepoint <= 0xDBFFU && i + 3 < bytes.size()) {
            const auto lo2 = static_cast<unsigned char>(bytes[i + 2]);
            const auto hi2 = static_cast<unsigned char>(bytes[i + 3]);
            const auto trail = static_cast<std::uint32_t>(lo2) | (static_cast<std::uint32_t>(hi2) << 8U);
            if (trail >= 0xDC00U && trail <= 0xDFFFU) {
                codepoint = 0x10000U + ((codepoint - 0xD800U) << 10U) + trail - 0xDC00U;
                i += 2;
            }
        }
        if (codepoint <= 0x7FU)
            output.push_back(static_cast<char>(codepoint));
        else if (codepoint <= 0x7FFU) {
            output.push_back(static_cast<char>(0xC0U | (codepoint >> 6U)));
            output.push_back(static_cast<char>(0x80U | (codepoint & 0x3FU)));
        } else if (codepoint <= 0xFFFFU) {
            output.push_back(static_cast<char>(0xE0U | (codepoint >> 12U)));
            output.push_back(static_cast<char>(0x80U | ((codepoint >> 6U) & 0x3FU)));
            output.push_back(static_cast<char>(0x80U | (codepoint & 0x3FU)));
        } else {
            output.push_back(static_cast<char>(0xF0U | (codepoint >> 18U)));
            output.push_back(static_cast<char>(0x80U | ((codepoint >> 12U) & 0x3FU)));
            output.push_back(static_cast<char>(0x80U | ((codepoint >> 6U) & 0x3FU)));
            output.push_back(static_cast<char>(0x80U | (codepoint & 0x3FU)));
        }
    }
    return output;
}

std::string readText(std::istream &input, std::uint8_t encoding) {
    const auto size = readCount(input, "text length", 16 * 1024 * 1024);
    std::string bytes(static_cast<std::size_t>(size), '\0');
    input.read(bytes.data(), size);
    if (!input)
        throw std::runtime_error("truncated PMX text");
    return encoding == 0 ? utf16LeToUtf8(bytes) : bytes;
}

std::int32_t readSignedIndex(std::istream &input, std::uint8_t size, std::string_view field) {
    switch (size) {
    case 1:
        return read<std::int8_t>(input, field);
    case 2:
        return read<std::int16_t>(input, field);
    case 4:
        return read<std::int32_t>(input, field);
    default:
        throw std::runtime_error("invalid PMX index size");
    }
}

std::uint32_t readVertexIndex(std::istream &input, std::uint8_t size) {
    switch (size) {
    case 1:
        return read<std::uint8_t>(input, "vertex index");
    case 2:
        return read<std::uint16_t>(input, "vertex index");
    case 4:
        return read<std::uint32_t>(input, "vertex index");
    default:
        throw std::runtime_error("invalid PMX vertex index size");
    }
}

struct Header {
    PmxMetadata metadata;
    PmxFormat format;
    std::array<std::uint8_t, 8> settings{};
};

Header readHeader(std::istream &input, const std::filesystem::path &path) {
    std::array<char, 4> magic{};
    input.read(magic.data(), static_cast<std::streamsize>(magic.size()));
    if (!input || std::string_view(magic.data(), magic.size()) != "PMX ") {
        throw std::runtime_error("not a PMX file: " + path.string());
    }
    Header result;
    result.metadata.version = read<float>(input, "version");
    if (!(result.metadata.version >= 2.0F && result.metadata.version <= 2.1F)) {
        throw std::runtime_error("unsupported PMX version");
    }
    const auto headerSize = read<std::uint8_t>(input, "header size");
    if (headerSize != result.settings.size())
        throw std::runtime_error("unsupported PMX global header size");
    input.read(reinterpret_cast<char *>(result.settings.data()), static_cast<std::streamsize>(result.settings.size()));
    if (!input)
        throw std::runtime_error("truncated PMX header");
    result.metadata.textEncoding = result.settings[0];
    result.metadata.additionalUvCount = result.settings[1];
    if (result.metadata.textEncoding > 1 || result.metadata.additionalUvCount > 4) {
        throw std::runtime_error("invalid PMX global settings");
    }
    for (std::size_t i = 2; i < result.settings.size(); ++i) {
        if (result.settings[i] != 1 && result.settings[i] != 2 && result.settings[i] != 4) {
            throw std::runtime_error("invalid PMX index size");
        }
    }
    result.metadata.modelName = readText(input, result.metadata.textEncoding);
    result.metadata.englishName = readText(input, result.metadata.textEncoding);
    result.metadata.comment = readText(input, result.metadata.textEncoding);
    result.metadata.englishComment = readText(input, result.metadata.textEncoding);
    result.metadata.vertexCount = readCount(input, "vertex count", 100'000'000);
    result.format.textEncoding = static_cast<PmxTextEncoding>(result.settings[0]);
    result.format.vertexIndexSize = result.settings[2];
    result.format.textureIndexSize = result.settings[3];
    result.format.materialIndexSize = result.settings[4];
    result.format.boneIndexSize = result.settings[5];
    result.format.morphIndexSize = result.settings[6];
    result.format.rigidBodyIndexSize = result.settings[7];
    return result;
}

void readVertices(std::istream &input, const Header &header, PmxModel &model, DecodedBudget &budget) {
    const auto minimumEncodedBytes = 12U + 12U + 8U +
                                     static_cast<std::size_t>(header.metadata.additionalUvCount) * 16U + 1U +
                                     static_cast<std::size_t>(header.settings[5]) + 4U;
    checkedResize(model.vertices, static_cast<std::size_t>(header.metadata.vertexCount), input, minimumEncodedBytes,
                  "vertex count", budget);
    const auto boneSize = header.settings[5];
    for (auto &vertex : model.vertices) {
        vertex.position = readFloatArray<3>(input, "position");
        vertex.normal = readFloatArray<3>(input, "normal");
        vertex.uv = readFloatArray<2>(input, "uv");
        for (std::uint8_t i = 0; i < header.metadata.additionalUvCount; ++i) {
            vertex.additionalUv[i] = readFloatArray<4>(input, "additional uv");
        }
        const auto type = read<std::uint8_t>(input, "weight type");
        if (type > 4 || (type == 4 && header.metadata.version < 2.1F)) {
            throw std::runtime_error("invalid PMX weight type");
        }
        vertex.weightType = static_cast<PmxWeightType>(type);
        const auto bone = [&] { return readSignedIndex(input, boneSize, "bone index"); };
        switch (vertex.weightType) {
        case PmxWeightType::bdef1:
            vertex.bones[0] = bone();
            break;
        case PmxWeightType::bdef2:
            vertex.bones[0] = bone();
            vertex.bones[1] = bone();
            vertex.weights[0] = read<float>(input, "BDEF2 weight");
            vertex.weights[1] = 1.0F - vertex.weights[0];
            break;
        case PmxWeightType::bdef4:
        case PmxWeightType::qdef:
            for (auto &index : vertex.bones)
                index = bone();
            vertex.weights = readFloatArray<4>(input, "BDEF4/QDEF weights");
            break;
        case PmxWeightType::sdef:
            vertex.bones[0] = bone();
            vertex.bones[1] = bone();
            vertex.weights[0] = read<float>(input, "SDEF weight");
            vertex.weights[1] = 1.0F - vertex.weights[0];
            vertex.sdefC = readFloatArray<3>(input, "SDEF C");
            vertex.sdefR0 = readFloatArray<3>(input, "SDEF R0");
            vertex.sdefR1 = readFloatArray<3>(input, "SDEF R1");
            break;
        }
        vertex.edgeScale = read<float>(input, "edge scale");
    }
}

void readMaterials(std::istream &input, const Header &header, PmxModel &model, DecodedBudget &budget) {
    const auto textureCount = readCount(input, "texture count", 1'000'000);
    model.textures.reserve(static_cast<std::size_t>(textureCount));
    for (std::int32_t i = 0; i < textureCount; ++i) {
        model.textures.push_back({readText(input, header.metadata.textEncoding)});
    }
    const auto count = readCount(input, "material count", 1'000'000);
    checkedResize(model.materials, static_cast<std::size_t>(count), input, 20, "material count", budget);
    std::uint64_t coveredIndices = 0;
    for (auto &material : model.materials) {
        material.name = readText(input, header.metadata.textEncoding);
        material.englishName = readText(input, header.metadata.textEncoding);
        material.diffuse = readFloatArray<4>(input, "material diffuse");
        material.specular = readFloatArray<3>(input, "material specular");
        material.shininess = read<float>(input, "material shininess");
        material.ambient = readFloatArray<3>(input, "material ambient");
        material.drawFlags = read<std::uint8_t>(input, "material draw flags");
        material.edgeColor = readFloatArray<4>(input, "material edge color");
        material.edgeSize = read<float>(input, "material edge size");
        material.textureIndex = readSignedIndex(input, header.settings[3], "texture index");
        material.sphereTextureIndex = readSignedIndex(input, header.settings[3], "sphere texture index");
        material.sphereMode = read<std::uint8_t>(input, "sphere mode");
        material.toonMode = read<std::uint8_t>(input, "toon mode");
        material.toonTextureIndex = material.toonMode == 0
                                        ? readSignedIndex(input, header.settings[3], "toon texture index")
                                        : static_cast<std::int32_t>(read<std::uint8_t>(input, "shared toon index"));
        material.memo = readText(input, header.metadata.textEncoding);
        const auto indexCount = readCount(input, "material index count");
        material.indexCount = static_cast<std::uint32_t>(indexCount);
        coveredIndices += material.indexCount;
    }
    if (coveredIndices != model.indices.size())
        throw std::runtime_error("PMX material ranges do not cover indices");
}

void readBones(std::istream &input, const Header &header, PmxModel &model, DecodedBudget &budget) {
    const auto count = readCount(input, "bone count", 10'000'000);
    checkedResize(model.bones, static_cast<std::size_t>(count), input, 24, "bone count", budget);
    for (auto &bone : model.bones) {
        bone.name = readText(input, header.metadata.textEncoding);
        bone.englishName = readText(input, header.metadata.textEncoding);
        bone.position = readFloatArray<3>(input, "bone position");
        bone.parent = readSignedIndex(input, header.settings[5], "parent bone");
        bone.deformLayer = read<std::int32_t>(input, "bone layer");
        bone.flags = read<std::uint16_t>(input, "bone flags");
        if ((bone.flags & 0x0001U) != 0)
            bone.tailBone = readSignedIndex(input, header.settings[5], "tail bone");
        else
            bone.tailOffset = readFloatArray<3>(input, "tail offset");
        if ((bone.flags & 0x0300U) != 0) {
            bone.inheritParent = readSignedIndex(input, header.settings[5], "inherit bone");
            bone.inheritRatio = read<float>(input, "inherit ratio");
        }
        if ((bone.flags & 0x0400U) != 0)
            bone.fixedAxis = readFloatArray<3>(input, "fixed axis");
        if ((bone.flags & 0x0800U) != 0) {
            bone.localAxisX = readFloatArray<3>(input, "local axis x");
            bone.localAxisZ = readFloatArray<3>(input, "local axis z");
        }
        if ((bone.flags & 0x2000U) != 0)
            bone.externalParentKey = read<std::int32_t>(input, "external parent key");
        if ((bone.flags & 0x0020U) != 0) {
            bone.ikTarget = readSignedIndex(input, header.settings[5], "IK target");
            bone.ikLoopCount = read<std::int32_t>(input, "IK loop count");
            bone.ikLimitAngle = read<float>(input, "IK angle");
            const auto linkCount = readCount(input, "IK link count", 1'000'000);
            checkedResize(bone.ikLinks, static_cast<std::size_t>(linkCount), input,
                          static_cast<std::size_t>(header.settings[5]) + 1U, "IK link count", budget);
            for (auto &link : bone.ikLinks) {
                link.bone = readSignedIndex(input, header.settings[5], "IK link bone");
                link.limited = read<std::uint8_t>(input, "IK link limit") != 0;
                if (link.limited) {
                    link.minimum = readFloatArray<3>(input, "IK minimum");
                    link.maximum = readFloatArray<3>(input, "IK maximum");
                }
            }
        }
    }
}

void readMorphs(std::istream &input, const Header &header, PmxModel &model, DecodedBudget &budget) {
    const auto count = readCount(input, "morph count", 10'000'000);
    checkedResize(model.morphs, static_cast<std::size_t>(count), input, 10, "morph count", budget);
    for (auto &morph : model.morphs) {
        morph.name = readText(input, header.metadata.textEncoding);
        morph.englishName = readText(input, header.metadata.textEncoding);
        morph.panel = read<std::uint8_t>(input, "morph panel");
        morph.type = read<std::uint8_t>(input, "morph type");
        if (morph.type > 10 || (morph.type >= 9 && header.metadata.version < 2.1F)) {
            throw std::runtime_error("invalid PMX morph type");
        }
        const auto offsetCount = readCount(input, "morph offset count", 100'000'000);
        checkedResize(morph.offsets, static_cast<std::size_t>(offsetCount), input, 5, "morph offset count", budget);
        for (auto &offset : morph.offsets) {
            if (morph.type == 0 || morph.type == 9) {
                offset.index = readSignedIndex(input, header.settings[6], "group morph index");
                offset.scalar = read<float>(input, "group morph weight");
            } else if (morph.type == 1) {
                offset.index = static_cast<std::int32_t>(readVertexIndex(input, header.settings[2]));
                offset.vector3 = readFloatArray<3>(input, "vertex morph offset");
            } else if (morph.type == 2) {
                offset.index = readSignedIndex(input, header.settings[5], "bone morph index");
                offset.vector3 = readFloatArray<3>(input, "bone morph translation");
                offset.vector4 = readFloatArray<4>(input, "bone morph rotation");
            } else if (morph.type >= 3 && morph.type <= 7) {
                offset.index = static_cast<std::int32_t>(readVertexIndex(input, header.settings[2]));
                offset.vector4 = readFloatArray<4>(input, "UV morph offset");
            } else if (morph.type == 8) {
                offset.index = readSignedIndex(input, header.settings[4], "material morph index");
                offset.operation = read<std::uint8_t>(input, "material morph operation");
                offset.materialVectors[0] = readFloatArray<4>(input, "morph diffuse");
                const auto specular = readFloatArray<3>(input, "morph specular");
                offset.materialVectors[1] = {specular[0], specular[1], specular[2], read<float>(input, "morph power")};
                const auto ambient = readFloatArray<3>(input, "morph ambient");
                offset.materialVectors[2] = {ambient[0], ambient[1], ambient[2], 0.0F};
                offset.materialVectors[3] = readFloatArray<4>(input, "morph edge color");
                offset.materialVectors[2][3] = read<float>(input, "morph edge size");
                offset.materialVectors[4] = readFloatArray<4>(input, "morph texture");
                offset.materialVectors[5] = readFloatArray<4>(input, "morph sphere");
                offset.materialVectors[6] = readFloatArray<4>(input, "morph toon");
            } else {
                offset.index = readSignedIndex(input, header.settings[7], "impulse rigid body");
                offset.local = read<std::uint8_t>(input, "impulse local flag") != 0;
                offset.vector3 = readFloatArray<3>(input, "impulse velocity");
                offset.tertiaryVector3 = readFloatArray<3>(input, "impulse torque");
            }
        }
    }
}

void readDisplayFrames(std::istream &input, const Header &header, PmxModel &model) {
    const auto count = readCount(input, "display frame count", 1'000'000);
    checkedResize(model.displayFrames, static_cast<std::size_t>(count), input, 10, "display frame count");
    for (auto &frame : model.displayFrames) {
        frame.name = readText(input, header.metadata.textEncoding);
        frame.englishName = readText(input, header.metadata.textEncoding);
        frame.special = read<std::uint8_t>(input, "display frame special") != 0;
        const auto itemCount = readCount(input, "display item count", 10'000'000);
        checkedResize(frame.items, static_cast<std::size_t>(itemCount), input, 2, "display item count");
        for (auto &item : frame.items) {
            item.bone = read<std::uint8_t>(input, "display item type") == 0;
            item.index =
                readSignedIndex(input, item.bone ? header.settings[5] : header.settings[6], "display item index");
        }
    }
}

void readPhysics(std::istream &input, const Header &header, PmxModel &model) {
    const auto bodyCount = readCount(input, "rigid body count", 1'000'000);
    checkedResize(model.rigidBodies, static_cast<std::size_t>(bodyCount), input, 40, "rigid body count");
    for (std::size_t bodyIndex = 0; bodyIndex < model.rigidBodies.size(); ++bodyIndex) {
        auto &body = model.rigidBodies[bodyIndex];
        body.name = readText(input, header.metadata.textEncoding);
        body.englishName = readText(input, header.metadata.textEncoding);
        body.bone = readSignedIndex(input, header.settings[5], "rigid body bone");
        body.group = read<std::uint8_t>(input, "rigid body group");
        body.collisionMask = read<std::uint16_t>(input, "rigid body mask");
        body.shape = read<std::uint8_t>(input, "rigid body shape");
        body.size = readFloatArray<3>(input, "rigid body size");
        body.position = readFloatArray<3>(input, "rigid body position");
        body.rotation = readFloatArray<3>(input, "rigid body rotation");
        body.mass = read<float>(input, "rigid body mass");
        body.linearDamping = read<float>(input, "rigid body linear damping");
        body.angularDamping = read<float>(input, "rigid body angular damping");
        body.restitution = read<float>(input, "rigid body restitution");
        body.friction = read<float>(input, "rigid body friction");
        body.mode = read<std::uint8_t>(input, "rigid body mode");
        std::size_t repaired = 0;
        const auto sanitize = [&repaired](auto &values, const float fallback = 0.0F) {
            for (auto &value : values) {
                if (!std::isfinite(value)) {
                    value = fallback;
                    ++repaired;
                }
            }
        };
        const auto sanitizeScalar = [&repaired](float &value, const float fallback = 0.0F) {
            if (!std::isfinite(value)) {
                value = fallback;
                ++repaired;
            }
        };
        sanitize(body.position);
        sanitize(body.rotation);
        sanitizeScalar(body.linearDamping);
        sanitizeScalar(body.angularDamping);
        sanitizeScalar(body.restitution);
        sanitizeScalar(body.friction);
        const auto repairedMass = !std::isfinite(body.mass);
        sanitizeScalar(body.mass);
        const auto invalidShape =
            !std::ranges::all_of(body.size, [](const float value) { return std::isfinite(value); }) || body.shape > 2 ||
            body.mode > 2;
        const auto invalidDynamicMass = body.mode != 0 && repairedMass;
        if (invalidShape || invalidDynamicMass) {
            body.physicsEnabled = false;
            sanitize(body.size);
        }
        if (repaired != 0 || invalidShape)
            log::warn("PMX rigid body #", bodyIndex, " \"", body.name,
                      "\": repaired invalid physics values; body physics ",
                      (body.physicsEnabled ? "enabled" : "disabled"));
    }
    const auto jointCount = readCount(input, "joint count", 1'000'000);
    checkedResize(model.joints, static_cast<std::size_t>(jointCount), input, 50, "joint count");
    for (std::size_t jointIndex = 0; jointIndex < model.joints.size(); ++jointIndex) {
        auto &joint = model.joints[jointIndex];
        joint.name = readText(input, header.metadata.textEncoding);
        joint.englishName = readText(input, header.metadata.textEncoding);
        joint.type = read<std::uint8_t>(input, "joint type");
        joint.bodyA = readSignedIndex(input, header.settings[7], "joint body A");
        joint.bodyB = readSignedIndex(input, header.settings[7], "joint body B");
        joint.position = readFloatArray<3>(input, "joint position");
        joint.rotation = readFloatArray<3>(input, "joint rotation");
        joint.translationMinimum = readFloatArray<3>(input, "joint translation minimum");
        joint.translationMaximum = readFloatArray<3>(input, "joint translation maximum");
        joint.rotationMinimum = readFloatArray<3>(input, "joint rotation minimum");
        joint.rotationMaximum = readFloatArray<3>(input, "joint rotation maximum");
        joint.translationSpring = readFloatArray<3>(input, "joint translation spring");
        joint.rotationSpring = readFloatArray<3>(input, "joint rotation spring");
        const auto finite = [](const auto &values) {
            return std::ranges::all_of(values, [](const float value) { return std::isfinite(value); });
        };
        if (!finite(joint.position) || !finite(joint.rotation) || !finite(joint.translationMinimum) ||
            !finite(joint.translationMaximum) || !finite(joint.rotationMinimum) || !finite(joint.rotationMaximum) ||
            !finite(joint.translationSpring) || !finite(joint.rotationSpring)) {
            joint.physicsEnabled = false;
            log::warn("PMX joint #", jointIndex, " \"", joint.name,
                      "\": disabled because it contains non-finite physics values");
        }
    }
}

void readSoftBodies(std::istream &input, const Header &header, PmxModel &model) {
    if (header.metadata.version < 2.1F || input.peek() == std::char_traits<char>::eof())
        return;
    const auto count = readCount(input, "soft body count", 1'000'000);
    checkedResize(model.softBodies, static_cast<std::size_t>(count), input, 50, "soft body count");
    for (auto &body : model.softBodies) {
        body.name = readText(input, header.metadata.textEncoding);
        body.englishName = readText(input, header.metadata.textEncoding);
        body.shape = read<std::uint8_t>(input, "soft body shape");
        body.material = readSignedIndex(input, header.settings[4], "soft body material");
        body.group = read<std::uint8_t>(input, "soft body group");
        body.collisionMask = read<std::uint16_t>(input, "soft body mask");
        body.flags = read<std::uint8_t>(input, "soft body flags");
        body.bendingLinkDistance = read<std::int32_t>(input, "soft body bending distance");
        body.clusterCount = read<std::int32_t>(input, "soft body cluster count");
        body.totalMass = read<float>(input, "soft body mass");
        body.collisionMargin = read<float>(input, "soft body margin");
        body.aeroModel = read<std::int32_t>(input, "soft body aero model");
        body.config = readFloatArray<12>(input, "soft body config");
        body.cluster = readFloatArray<6>(input, "soft body cluster");
        for (auto &value : body.iteration)
            value = read<std::int32_t>(input, "soft body iteration");
        body.materialConfig = readFloatArray<3>(input, "soft body material config");
        const auto anchorCount = readCount(input, "soft body anchor count", 10'000'000);
        const auto anchorEncodedBytes =
            static_cast<std::size_t>(header.settings[7]) + static_cast<std::size_t>(header.settings[2]) + 1U;
        checkedResize(body.anchors, static_cast<std::size_t>(anchorCount), input, anchorEncodedBytes,
                      "soft body anchor count");
        for (auto &anchor : body.anchors) {
            anchor.rigidBody = readSignedIndex(input, header.settings[7], "soft body anchor rigid body");
            anchor.vertex = static_cast<std::int32_t>(readVertexIndex(input, header.settings[2]));
            anchor.nearMode = read<std::uint8_t>(input, "soft body anchor near mode") != 0;
        }
        const auto pinCount = readCount(input, "soft body pin count", 100'000'000);
        checkedResize(body.pinnedVertices, static_cast<std::size_t>(pinCount), input, header.settings[2],
                      "soft body pin count");
        for (auto &vertex : body.pinnedVertices)
            vertex = static_cast<std::int32_t>(readVertexIndex(input, header.settings[2]));
    }
}

} // namespace

PmxMetadata pmx::probe(const std::filesystem::path &path) {
    MappedFileStream input(path);
    return readHeader(input, path).metadata;
}

PmxModel pmx::load(const std::filesystem::path &path) {
    MappedFileStream input(path);
    const auto header = readHeader(input, path);
    PmxModel model;
    DecodedBudget budget;
    const ScopedDecodedBudget scopedBudget(budget);
    model.metadata = header.metadata;
    model.format = header.format;
    model.sourcePath = path;
    readVertices(input, header, model, budget);
    const auto indexCount = readCount(input, "index count");
    if (indexCount % 3 != 0)
        throw std::runtime_error("PMX index count is not divisible by three");
    checkedResize(model.indices, static_cast<std::size_t>(indexCount), input, header.settings[2], "index count",
                  budget);
    for (auto &index : model.indices) {
        index = readVertexIndex(input, header.settings[2]);
        if (index >= model.vertices.size())
            throw std::runtime_error("PMX vertex index out of range");
    }
    readMaterials(input, header, model, budget);
    readBones(input, header, model, budget);
    readMorphs(input, header, model, budget);
    readDisplayFrames(input, header, model);
    readPhysics(input, header, model);
    readSoftBodies(input, header, model);
    return model;
}

PmxMesh pmx::loadMesh(const std::filesystem::path &path) {
    auto model = load(path);
    PmxMesh mesh{.metadata = std::move(model.metadata),
                 .vertices = std::move(model.vertices),
                 .indices = std::move(model.indices)};
    Float3 minimum{std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity(),
                   std::numeric_limits<float>::infinity()};
    Float3 maximum{-std::numeric_limits<float>::infinity(), -std::numeric_limits<float>::infinity(),
                   -std::numeric_limits<float>::infinity()};
    bool found = false;
    for (const auto &vertex : mesh.vertices) {
        if (!std::ranges::all_of(vertex.position, [](float value) { return std::isfinite(value); }))
            continue;
        found = true;
        for (std::size_t axis = 0; axis < 3; ++axis) {
            minimum[axis] = std::min(minimum[axis], vertex.position[axis]);
            maximum[axis] = std::max(maximum[axis], vertex.position[axis]);
        }
    }
    if (!found)
        return mesh;
    const Float3 center{std::midpoint(minimum[0], maximum[0]), std::midpoint(minimum[1], maximum[1]),
                        std::midpoint(minimum[2], maximum[2])};
    const auto extent = std::max({static_cast<double>(maximum[0]) - static_cast<double>(minimum[0]),
                                  static_cast<double>(maximum[1]) - static_cast<double>(minimum[1]),
                                  static_cast<double>(maximum[2]) - static_cast<double>(minimum[2]), 0.001});
    const float scale = std::isfinite(extent) && extent > 0.0 ? static_cast<float>(1.8 / extent) : 1.0F;
    for (auto &vertex : mesh.vertices) {
        for (std::size_t axis = 0; axis < 3; ++axis)
            vertex.position[axis] = (vertex.position[axis] - center[axis]) * scale;
    }
    return mesh;
}

} // namespace mmd
