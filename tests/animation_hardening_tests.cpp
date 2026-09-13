#include <mmd/animation.hpp>
#include <mmd/pmx.hpp>

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <limits>
#include <span>

int main() {
    mmd::PmxModel model;
    model.morphs = {
        {.name = "root", .type = 0, .offsets = {{.index = 1, .scalar = 0.5F}, {.index = 2, .scalar = 2.0F}}},
        {.name = "vertex", .type = 1},
        {.name = "flip", .type = 9, .offsets = {{.index = 1, .scalar = 0.5F}}},
    };
    const std::array<float, 3> rootWeights{1.0F, 0.0F, 0.0F};
    const auto expanded = mmd::expandMorphWeights(model, rootWeights);
    assert(expanded.effectiveWeights.size() == 3);
    assert(expanded.effectiveWeights[1] == 1.5F);
    assert(!expanded.budgetExceeded);

    model.morphs = {
        {.name = "self", .type = 0, .offsets = {{.index = 0, .scalar = 1.0F}, {.index = 1, .scalar = 2.0F}}},
        {.name = "vertex", .type = 1}};
    const std::array<float, 2> selfRoot{1.0F, 0.0F};
    const auto selfCycle = mmd::expandMorphWeights(model, selfRoot, {.maxSteps = 2});
    assert(selfCycle.cycleDetected);
    assert(!selfCycle.budgetExceeded);
    assert(selfCycle.effectiveWeights[1] == 2.0F);

    model.morphs = {{.name = "a", .type = 0, .offsets = {{.index = 1, .scalar = 1.0F}, {.index = 2, .scalar = 3.0F}}},
                    {.name = "b", .type = 9, .offsets = {{.index = 0, .scalar = 1.0F}}},
                    {.name = "vertex", .type = 1}};
    const std::array<float, 3> cycleRoot{1.0F, 0.0F, 0.0F};
    const auto twoNodeCycle = mmd::expandMorphWeights(model, cycleRoot, {.maxSteps = 2});
    assert(twoNodeCycle.cycleDetected);
    assert(!twoNodeCycle.budgetExceeded);
    assert(twoNodeCycle.effectiveWeights[2] == 3.0F);

    model.morphs = {{.name = "active", .type = 0, .offsets = {{.index = 2, .scalar = 1.0F}}},
                    {.name = "inactive", .type = 0, .offsets = {{.index = 2, .scalar = 10.0F}}},
                    {.name = "vertex", .type = 1}};
    const std::array<float, 3> inactiveRoot{1.0F, 0.0F, 0.0F};
    const auto inactiveGroup = mmd::expandMorphWeights(model, inactiveRoot, {.maxSteps = 1});
    assert(!inactiveGroup.budgetExceeded);
    assert(inactiveGroup.effectiveWeights[2] == 1.0F);

    const std::array<float, 3> invalidWeights{std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F};
    const auto nonFinite = mmd::expandMorphWeights(model, invalidWeights);
    assert(nonFinite.effectiveWeights[1] == 0.0F);

    const auto evaluateOverride = [](mmd::PmxModel model, std::size_t morphIndex, bool gpuSkinning = false) {
        mmd::MmdAnimator animator(model);
        mmd::MorphOverride overrideValue;
        overrideValue.index = morphIndex;
        overrideValue.weight = 1.0F;
        overrideValue.type = model.morphs[morphIndex].type;
        overrideValue.offsets = model.morphs[morphIndex].offsets;
        const std::array<mmd::MorphOverride, 1> overrides{overrideValue};
        return animator.evaluate(0.0F, 0.0F, gpuSkinning, overrides);
    };

    {
        mmd::PmxModel invalid;
        invalid.metadata.version = 2.1F;
        invalid.vertices.resize(1);
        invalid.morphs.push_back({.name = "vertex", .type = 1});
        invalid.morphs[0].offsets.push_back(
            {.index = 0, .vector3 = {std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F}});
        assert(!mmd::pmx::validate(invalid).valid());
        const auto frame = evaluateOverride(invalid, 0);
        assert(std::all_of(frame.vertices[0].position.begin(), frame.vertices[0].position.end(),
                           [](const float value) { return std::isfinite(value); }));
    }

    {
        mmd::PmxModel invalid;
        invalid.metadata.version = 2.1F;
        invalid.materials.emplace_back();
        invalid.morphs.push_back({.name = "material operation", .type = 8});
        invalid.morphs[0].offsets.push_back({.index = 0, .operation = 2});
        const auto validation = mmd::pmx::validate(invalid);
        assert(!validation.valid());
        assert(std::any_of(validation.issues.begin(), validation.issues.end(),
                           [](const auto &issue) { return issue.message == "material morph operation is invalid"; }));
        assert(std::none_of(validation.issues.begin(), validation.issues.end(), [](const auto &issue) {
            return issue.message == "morph offset contains invalid numeric values";
        }));
    }

    {
        mmd::PmxModel invalid;
        invalid.metadata.version = 2.1F;
        invalid.bones.push_back({.name = "bone"});
        invalid.morphs.push_back({.name = "bone", .type = 2});
        invalid.morphs[0].offsets.push_back(
            {.index = 0, .vector3 = {std::numeric_limits<float>::infinity(), 0.0F, 0.0F}});
        assert(!mmd::pmx::validate(invalid).valid());
        const auto frame = evaluateOverride(invalid, 0);
        assert(frame.bones.size() == 1);
        assert(std::all_of(frame.bones[0].translation.begin(), frame.bones[0].translation.end(),
                           [](const float value) { return std::isfinite(value); }));
    }

    {
        mmd::PmxModel invalid;
        invalid.metadata.version = 2.1F;
        invalid.vertices.resize(1);
        invalid.morphs.push_back({.name = "uv", .type = 3});
        invalid.morphs[0].offsets.push_back(
            {.index = 0, .vector4 = {0.0F, 0.0F, std::numeric_limits<float>::infinity(), 0.0F}});
        assert(!mmd::pmx::validate(invalid).valid());
        const auto frame = evaluateOverride(invalid, 0);
        assert(std::all_of(frame.vertices[0].uv.begin(), frame.vertices[0].uv.end(),
                           [](const float value) { return std::isfinite(value); }));
    }

    {
        mmd::PmxModel invalid;
        invalid.metadata.version = 2.1F;
        invalid.vertices.resize(2);
        invalid.morphs.push_back({.name = "mixed vertex", .type = 1});
        invalid.morphs[0].offsets = {
            {.index = 0, .vector3 = {1.0F, 0.0F, 0.0F}},
            {.index = 1, .vector3 = {std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F}},
        };
        assert(!mmd::pmx::validate(invalid).valid());
        const auto cpuFrame = evaluateOverride(invalid, 0);
        const auto gpuFrame = evaluateOverride(invalid, 0, true);
        // A malformed sparse morph is rejected consistently by both preview
        // paths rather than applying a different subset of its offsets.
        assert(cpuFrame.vertices[0].position == mmd::Float3{});
        assert(gpuFrame.morphWeights[0] == 0.0F);
    }

    {
        mmd::PmxModel invalid;
        invalid.metadata.version = 2.1F;
        mmd::PmxMaterial material;
        material.diffuse = {1.0F, 1.0F, 1.0F, 1.0F};
        invalid.materials.push_back(material);
        invalid.morphs.push_back({.name = "material", .type = 8});
        invalid.morphs[0].offsets.push_back({.index = 0});
        invalid.morphs[0].offsets[0].materialVectors[0][0] = std::numeric_limits<float>::quiet_NaN();
        assert(!mmd::pmx::validate(invalid).valid());
        const auto frame = evaluateOverride(invalid, 0);
        assert(frame.materials.size() == 1);
        assert(std::all_of(frame.materials[0].diffuse.begin(), frame.materials[0].diffuse.end(),
                           [](const float value) { return std::isfinite(value); }));
    }

    {
        mmd::PmxModel invalid;
        invalid.metadata.version = 2.1F;
        invalid.rigidBodies.emplace_back();
        invalid.morphs.push_back({.name = "impulse", .type = 10});
        invalid.morphs[0].offsets.push_back(
            {.index = 0, .vector3 = {0.0F, std::numeric_limits<float>::infinity(), 0.0F}});
        assert(!mmd::pmx::validate(invalid).valid());
        static_cast<void>(evaluateOverride(invalid, 0));
    }

    // A large number of unrelated root bones must not turn one IK link's
    // dirty closure into a full-model work item. This also exercises the
    // iterative hierarchy path with a realistic stress-sized model.
    {
        constexpr std::size_t boneCount = 100'000;
        mmd::PmxModel large;
        large.bones.resize(boneCount);
        for (std::size_t index = 0; index < boneCount; ++index) {
            large.bones[index].position = {static_cast<float>(index), 0.0F, 0.0F};
            large.bones[index].parent = -1;
        }
        large.bones[0].position = {0.0F, 0.0F, 0.0F};
        large.bones[1].position = {0.0F, 1.0F, 0.0F};
        large.bones[2].position = {1.0F, 1.0F, 0.0F};
        large.bones[2].flags = 0x0020U;
        large.bones[2].ikTarget = 1;
        large.bones[2].ikLoopCount = 1;
        large.bones[2].ikLimitAngle = 1.0F;
        large.bones[2].ikLinks.push_back({.bone = 0});
        mmd::MmdAnimator animator(large);
        animator.setIkEvaluationLimits({.maxLinkSteps = 2, .maxBoneUpdates = 3});
        const auto frame = animator.evaluate(0.0F);
        const auto stats = animator.ikEvaluationStats();
        assert(frame.bones.size() == boneCount);
        assert(std::all_of(frame.bones[0].translation.begin(), frame.bones[0].translation.end(),
                           [](const float value) { return std::isfinite(value); }));
        assert(stats.linkSteps == 1);
        assert(stats.dirtyBoneUpdates == 1);
        assert(stats.localPoseRebuilds == 1);
        assert(stats.globalPoseRebuilds == 1);
        assert(stats.localPoseVisits == 1);
        assert(stats.globalPoseVisits == 1);
        assert(!stats.budgetExceeded);
    }

    // The link budget is shared by both IK phases in one evaluate call. A
    // post-physics IK link cannot reset the limit after the pre-physics phase.
    {
        mmd::PmxModel phased;
        phased.bones.resize(4);
        phased.bones[0].position = {0.0F, 0.0F, 0.0F};
        phased.bones[1].position = {0.0F, 1.0F, 0.0F};
        phased.bones[2].position = {1.0F, 1.0F, 0.0F};
        phased.bones[3].position = {-1.0F, 1.0F, 0.0F};
        for (std::size_t index : {2U, 3U}) {
            phased.bones[index].flags = 0x0020U;
            phased.bones[index].ikTarget = 1;
            phased.bones[index].ikLoopCount = 1;
            phased.bones[index].ikLimitAngle = 1.0F;
            phased.bones[index].ikLinks.push_back({.bone = 0});
        }
        phased.bones[3].flags |= 0x1000U;
        mmd::MmdAnimator animator(phased);
        animator.setIkEvaluationLimits({.maxLinkSteps = 1, .maxBoneUpdates = 100});
        static_cast<void>(animator.evaluate(0.0F));
        const auto stats = animator.ikEvaluationStats();
        assert(stats.linkSteps == 1);
        assert(stats.budgetExceeded);
    }

    // A zero bone-update budget rolls back the tentative link rotation and
    // leaves the frame finite instead of publishing a half-rebuilt pose.
    {
        mmd::PmxModel limited;
        limited.bones.resize(3);
        limited.bones[0].position = {0.0F, 0.0F, 0.0F};
        limited.bones[1].position = {0.0F, 1.0F, 0.0F};
        limited.bones[2].position = {1.0F, 1.0F, 0.0F};
        limited.bones[2].flags = 0x0020U;
        limited.bones[2].ikTarget = 1;
        limited.bones[2].ikLoopCount = 1;
        limited.bones[2].ikLimitAngle = 1.0F;
        limited.bones[2].ikLinks.push_back({.bone = 0});
        mmd::MmdAnimator animator(limited);
        animator.setIkEvaluationLimits({.maxLinkSteps = 10, .maxBoneUpdates = 0});
        const auto frame = animator.evaluate(0.0F);
        const auto stats = animator.ikEvaluationStats();
        assert(stats.linkSteps == 1);
        assert(stats.budgetExceeded);
        assert(std::all_of(frame.bones[0].rotation.begin(), frame.bones[0].rotation.end(),
                           [](const float value) { return std::isfinite(value); }));
    }
    return 0;
}
