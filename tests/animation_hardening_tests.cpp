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

    const auto evaluateOverride = [](mmd::PmxModel model, std::size_t morphIndex) {
        mmd::MmdAnimator animator(model);
        mmd::MorphOverride overrideValue;
        overrideValue.index = morphIndex;
        overrideValue.weight = 1.0F;
        overrideValue.type = model.morphs[morphIndex].type;
        overrideValue.offsets = model.morphs[morphIndex].offsets;
        const std::array<mmd::MorphOverride, 1> overrides{overrideValue};
        return animator.evaluate(0.0F, 0.0F, false, overrides);
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
        const auto frame = mmd::MmdAnimator(large).evaluate(0.0F);
        assert(frame.bones.size() == boneCount);
        assert(std::all_of(frame.bones[0].translation.begin(), frame.bones[0].translation.end(),
                           [](const float value) { return std::isfinite(value); }));
    }
    return 0;
}
