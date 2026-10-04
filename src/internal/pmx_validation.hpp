#pragma once

#include <mmd/pmx.hpp>

#include <algorithm>
#include <cmath>

namespace mmd::internal {

template <std::size_t N> bool finitePmxComponents(const std::array<float, N> &values) noexcept {
    return std::all_of(values.begin(), values.end(), [](const float value) { return std::isfinite(value); });
}

inline bool validPmxIndexWidth(std::uint8_t width) noexcept {
    return width == 1 || width == 2 || width == 4;
}

inline bool validVertexWeightType(PmxWeightType type) noexcept {
    return static_cast<std::uint8_t>(type) <= static_cast<std::uint8_t>(PmxWeightType::qdef);
}

// Shared nonrecursive dependency check for document edits and global validation.
// Parent and active inheritance edges both participate; duplicate edges are
// counted independently so a parent also used for append remains valid.
inline bool hasBoneDependencyCycle(const std::vector<PmxBone> &bones) {
    std::vector<std::vector<std::size_t>> dependents(bones.size());
    std::vector<std::size_t> indegree(bones.size());
    for (std::size_t child = 0; child < bones.size(); ++child) {
        const auto addDependency = [&](std::int32_t parent) {
            if (parent >= 0 && static_cast<std::size_t>(parent) < bones.size()) {
                dependents[static_cast<std::size_t>(parent)].push_back(child);
                ++indegree[child];
            }
        };
        addDependency(bones[child].parent);
        if ((bones[child].flags & 0x0300U) != 0)
            addDependency(bones[child].inheritParent);
    }
    std::vector<std::size_t> pending;
    pending.reserve(bones.size());
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
    return visited != bones.size();
}

inline bool validPmxReference(std::int32_t value, std::size_t count, bool allowNone = false) noexcept {
    return (allowNone && value == -1) || (value >= 0 && static_cast<std::size_t>(value) < count);
}

// Both property inspection and transaction/save validation use these contracts.
// Report carries field and element identity so editor diagnostics stay useful.
template <typename Report> void validateRigidBody(const PmxModel &model, const PmxRigidBody &body, Report report) {
    report(validPmxReference(body.bone, model.bones.size(), true), ValidationCode::invalid_reference, "bone",
           "rigid body bone index is out of range", 0U);
    report(body.shape <= 2, ValidationCode::invalid_format, "shape", "rigid body shape is invalid", 0U);
    report(body.mode <= 2, ValidationCode::invalid_format, "mode", "rigid body mode is invalid", 0U);
    report(body.group <= 15, ValidationCode::invalid_format, "group", "rigid body collision group is invalid", 0U);
    const auto numeric = [&](bool valid, const char *field) {
        report(valid, ValidationCode::generic, field, "rigid body contains non-finite numeric values", 0U);
    };
    numeric(finitePmxComponents(body.size), "size");
    numeric(finitePmxComponents(body.position), "position");
    numeric(finitePmxComponents(body.rotation), "rotation");
    numeric(std::isfinite(body.mass), "mass");
    numeric(std::isfinite(body.linearDamping), "linearDamping");
    numeric(std::isfinite(body.angularDamping), "angularDamping");
    numeric(std::isfinite(body.restitution), "restitution");
    numeric(std::isfinite(body.friction), "friction");
}

template <typename Report> void validateJoint(const PmxModel &model, const PmxJoint &joint, Report report) {
    report(joint.type <= 5 && (model.metadata.version >= 2.1F || joint.type == 0), ValidationCode::invalid_format,
           "type", "joint type is invalid for the PMX version", 0U);
    report(validPmxReference(joint.bodyA, model.rigidBodies.size(), true), ValidationCode::invalid_reference, "bodyA",
           "joint A body index is out of range", 0U);
    report(validPmxReference(joint.bodyB, model.rigidBodies.size(), true), ValidationCode::invalid_reference, "bodyB",
           "joint B body index is out of range", 0U);
    const auto numeric = [&](const Float3 &values, const char *field) {
        report(finitePmxComponents(values), ValidationCode::generic, field, "joint contains non-finite numeric values",
               0U);
    };
    numeric(joint.position, "position");
    numeric(joint.rotation, "rotation");
    numeric(joint.translationMinimum, "translationMinimum");
    numeric(joint.translationMaximum, "translationMaximum");
    numeric(joint.rotationMinimum, "rotationMinimum");
    numeric(joint.rotationMaximum, "rotationMaximum");
    numeric(joint.translationSpring, "translationSpring");
    numeric(joint.rotationSpring, "rotationSpring");
}

template <typename Report> void validateSoftBody(const PmxModel &model, const PmxSoftBody &body, Report report) {
    report(model.metadata.version >= 2.1F, ValidationCode::invalid_format, "version", "soft bodies require PMX 2.1",
           0U);
    report(body.shape <= 1, ValidationCode::invalid_format, "shape", "soft body shape is invalid", 0U);
    report(body.group <= 15, ValidationCode::invalid_format, "group", "soft body collision group is invalid", 0U);
    report(body.aeroModel >= 0 && body.aeroModel <= 4, ValidationCode::invalid_format, "aeroModel",
           "soft body aero model is invalid", 0U);
    const auto numeric = [&](bool valid, const char *field) {
        report(valid, ValidationCode::generic, field, "soft body contains non-finite numeric values", 0U);
    };
    numeric(std::isfinite(body.totalMass), "totalMass");
    numeric(std::isfinite(body.collisionMargin), "collisionMargin");
    numeric(finitePmxComponents(body.config), "config");
    numeric(finitePmxComponents(body.cluster), "cluster");
    numeric(finitePmxComponents(body.materialConfig), "materialConfig");
    report(validPmxReference(body.material, model.materials.size(), true), ValidationCode::invalid_reference,
           "material", "soft body material index is out of range", 0U);
    for (std::size_t index = 0; index < body.anchors.size(); ++index) {
        const auto &anchor = body.anchors[index];
        report(validPmxReference(anchor.rigidBody, model.rigidBodies.size(), true), ValidationCode::invalid_reference,
               "anchors.rigidBody", "soft body rigid body index is out of range", static_cast<std::uint32_t>(index));
        report(validPmxReference(anchor.vertex, model.vertices.size()), ValidationCode::invalid_reference,
               "anchors.vertex", "soft body vertex index is out of range", static_cast<std::uint32_t>(index));
    }
    for (std::size_t index = 0; index < body.pinnedVertices.size(); ++index)
        report(validPmxReference(body.pinnedVertices[index], model.vertices.size()), ValidationCode::invalid_reference,
               "pinnedVertices", "soft body pinned vertex index is out of range", static_cast<std::uint32_t>(index));
}

// Morph offsets are retained on invalid documents so editors can repair them.
// Keep the same type-specific numeric contract in validation and animation so
// malformed values never reach the preview evaluator.
inline bool finiteMorphOffset(std::uint8_t type, const PmxMorphOffset &offset) noexcept {
    switch (type) {
    case 0:
    case 9:
        return std::isfinite(offset.scalar);
    case 1:
        return finitePmxComponents(offset.vector3);
    case 2:
        return finitePmxComponents(offset.vector3) && finitePmxComponents(offset.vector4);
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
        return finitePmxComponents(offset.vector4);
    case 8:
        return std::all_of(offset.materialVectors.begin(), offset.materialVectors.begin() + 7,
                           [](const Float4 &value) { return finitePmxComponents(value); });
    case 10:
        return finitePmxComponents(offset.vector3) && finitePmxComponents(offset.tertiaryVector3);
    default:
        return false;
    }
}

inline bool validMorphOffsetOperation(std::uint8_t type, const PmxMorphOffset &offset) noexcept {
    return type != 8U || offset.operation <= 1U;
}

inline bool validMorphOffset(std::uint8_t type, const PmxMorphOffset &offset) noexcept {
    return finiteMorphOffset(type, offset) && validMorphOffsetOperation(type, offset);
}

} // namespace mmd::internal
