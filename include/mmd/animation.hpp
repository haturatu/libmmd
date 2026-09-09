#pragma once

#include <mmd/pmx.hpp>
#include <mmd/vmd.hpp>

#include <cstdint>
#include <cstddef>
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

class MmdAnimator {
  public:
    explicit MmdAnimator(const PmxModel &model);
    ~MmdAnimator();

    void setMotion(const VmdMotion *motion);
    void setPose(const VpdPose *pose);
    void setPhysics(MmdPhysics *physics);
    void setIkEnabled(bool enabled) noexcept;
    [[nodiscard]] MotionCompatibility motionCompatibility() const;
    [[nodiscard]] AnimatedModelFrame evaluate(float frame, float deltaSeconds = 0.0F,
                                               bool gpuSkinning = false,
                                               MorphOverrides overrides = {});

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
