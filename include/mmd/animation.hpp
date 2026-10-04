#pragma once

#include <mmd/pmx.hpp>
#include <mmd/vmd.hpp>

#include <cstddef>
#include <cstdint>
#include <limits>
#include <memory>
#include <span>
#include <unordered_map>
#include <vector>

namespace mmd {

class MmdPhysics;

struct AnimatedModelFrame {
    struct BoneTransform {
        Float4 rotation{0.0F, 0.0F, 0.0F, 1.0F};
        Float3 translation{};
        // Final solved local pose (including morph, append, IK and physics).
        Float3 localTranslation{};
        Float4 localRotation{0.0F, 0.0F, 0.0F, 1.0F};
        // Final bone origin in PMX model space; independent of vertex skinning.
        Float3 worldPosition{};
        // Sampled VMD/VPD input before runtime deformation. Editors register
        // these values to avoid baking morph/IK/append/physics twice.
        Float3 inputTranslation{};
        Float4 inputRotation{0.0F, 0.0F, 0.0F, 1.0F};
    };
    std::vector<PmxVertex> vertices;
    // Effective vertex-morph weights are populated for GPU skinning. The
    // renderer combines these weights with its immutable sparse morph table.
    std::vector<float> morphWeights;
    std::vector<BoneTransform> bones;
    struct Material {
        Float4 diffuse{};
        Float3 specular{};
        float shininess{};
        Float3 ambient{};
        Float4 edgeColor{};
        float edgeSize{};
        Float4 textureMultiply{1.0F, 1.0F, 1.0F, 1.0F};
        Float4 textureAdd{};
        Float4 sphereMultiply{1.0F, 1.0F, 1.0F, 1.0F};
        Float4 sphereAdd{};
        Float4 toonMultiply{1.0F, 1.0F, 1.0F, 1.0F};
        Float4 toonAdd{};
    };
    std::vector<Material> materials;
    bool visible{true};
};

struct PreviewNormalization {
    Float3 center{};
    float scale{1.0F};
};

struct MorphExpansionLimits {
    std::uint64_t maxSteps{1'000'000};
};

struct MorphExpansionResult {
    std::vector<float> effectiveWeights;
    bool cycleDetected{};
    bool budgetExceeded{};
};

// Expands group and flip morph roots without recursion. Invalid references and
// non-finite weights are ignored; expansion stops deterministically at the
// configured work budget.
[[nodiscard]] MorphExpansionResult expandMorphWeights(const PmxModel &model, std::span<const float> rootWeights,
                                                      MorphExpansionLimits limits = {});

// IK evaluation is bounded per frame, including models with both pre- and
// post-physics IK phases. Limits are configurable for applications that need
// to accommodate unusually dense rigs while retaining deterministic defaults.
struct IkEvaluationLimits {
    std::uint64_t maxLinkSteps{25'000};
    std::uint64_t maxBoneUpdates{100'000};
};

struct IkEvaluationStats {
    std::uint64_t linkSteps{};
    std::uint64_t dirtyBoneUpdates{};
    std::uint64_t localPoseRebuilds{};
    std::uint64_t globalPoseRebuilds{};
    std::uint64_t localPoseVisits{};
    std::uint64_t globalPoseVisits{};
    bool budgetExceeded{};
};

struct MotionCompatibility {
    std::size_t pmxBoneCount{};
    std::size_t vmdBoneKeyCount{};
    std::size_t vmdBoneTrackCount{};
    std::size_t matchedBoneKeyCount{};
    std::size_t matchedBoneTrackCount{};
};

// Replaces the sampled VMD weight for one model morph. Group/flip morphs are
// still expanded by the regular evaluator, so bone, material, and vertex
// morphs all follow the same animation and skinning path. A temporary override
// uses `index == MorphOverride::temporary` and supplies transient vertex or
// bone offsets without changing the model.
struct MorphOverride {
    // A temporary override has no model morph index. Its offsets are applied
    // by the same evaluator before IK and skinning, which lets pose editing
    // use the normal bone deformation path without mutating the document.
    static constexpr std::size_t temporary = std::numeric_limits<std::size_t>::max();
    std::size_t index{};
    float weight{};
    std::uint8_t type{};
    std::span<const PmxMorphOffset> offsets{};
};

using MorphOverrides = std::span<const MorphOverride>;

// Absolute, transient VMD-local input. Applied before bone morphs, append,
// IK and skinning; never mutates the motion/model. Invalid indices/nonfinite
// poses are ignored. Last valid override for a bone wins.
struct BoneOverride {
    std::size_t index{};
    Float3 translation{};
    Float4 rotation{0.0F, 0.0F, 0.0F, 1.0F};
    bool physics{true};
};
using BoneOverrides = std::span<const BoneOverride>;

// Affine model-space parent transform applied after local IK/physics solving
// and before skinning. Descendants inherit it; local authoring inputs remain
// unchanged. Repeated entries for one bone use the last valid transform.
struct ExternalParentTransform {
    std::size_t index{};
    Float3 translation{};
    Float4 rotation{0.0F, 0.0F, 0.0F, 1.0F};
};
using ExternalParentTransforms = std::span<const ExternalParentTransform>;

class MmdAnimator {
  public:
    explicit MmdAnimator(const PmxModel &model);
    ~MmdAnimator();

    void setMotion(const VmdMotion *motion);
    // VPD replaces the VMD local translation/rotation of matching bones.
    // Unspecified bones and morph tracks retain their motion; nullptr removes
    // the override. Replacing/removing the pose resynchronizes physics.
    void setPose(const VpdPose *pose);
    void setPhysics(MmdPhysics *physics);
    void setIkEnabled(bool enabled) noexcept;
    void setIkEvaluationLimits(IkEvaluationLimits limits) noexcept;
    [[nodiscard]] IkEvaluationStats ikEvaluationStats() const noexcept;
    [[nodiscard]] MotionCompatibility motionCompatibility() const;
    [[nodiscard]] AnimatedModelFrame evaluate(float frame, float deltaSeconds = 0.0F, bool gpuSkinning = false,
                                              MorphOverrides overrides = {}, BoneOverrides boneOverrides = {},
                                              ExternalParentTransforms externalParents = {});

  private:
    struct Impl;
    const PmxModel &model_;
    const VmdMotion *motion_{};
    const VpdPose *pose_{};
    MmdPhysics *physics_{};
    bool ikEnabled_{true};
    float previousFrame_{-1.0F};
    std::unique_ptr<Impl> impl_;
};

// Applies one stable model-space transform so animated vertices remain framed.
void normalizeForPreview(std::vector<PmxVertex> &vertices, const PreviewNormalization &normalization);
[[nodiscard]] PreviewNormalization previewNormalization(const PmxModel &model);

namespace animation {
using Animator = MmdAnimator;
using ModelFrame = AnimatedModelFrame;
} // namespace animation

} // namespace mmd
