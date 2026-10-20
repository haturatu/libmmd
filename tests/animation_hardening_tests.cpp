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
    // Track/pose bindings survive replacement, clearing, missing names and
    // duplicate names without retaining stale sampling cursors or output.
    {
        mmd::PmxModel source;
        source.bones.resize(3);
        source.bones[0].name = "bone";
        source.bones[1].name = "bone";
        source.bones[2].name = "missing";
        source.vertices.resize(1);
        source.vertices[0].bones[0] = 0;
        source.materials.resize(1);
        source.materials[0].diffuse = {1, 1, 1, 1};
        source.morphs = {{.name = "morph", .type = 1, .offsets = {{.index = 0, .vector3 = {1, 0, 0}}}},
                         {.name = "missing", .type = 1}};
        mmd::VmdMotion first;
        first.bones.resize(2);
        first.bones[0].name = "bone";
        first.bones[1].name = "bone";
        first.bones[1].frame = 29;
        first.bones[1].translation = {2, 0, 0};
        first.morphs = {{"morph", 0, 1}};
        first.ik = {{.frame = 0, .visible = false}};
        mmd::MmdAnimator animator(source);
        animator.setMotion(&first);
        mmd::AnimatedModelFrame reused;
        animator.evaluateInto(reused, 30, 0, true);
        assert(reused.bones[0].translation[0] == 2 && reused.bones[1].translation[0] == 2);
        assert(reused.bones[2].translation[0] == 0 && reused.morphWeights[0] == 1 && reused.morphWeights[1] == 0);
        assert(!reused.visible);
        const auto *bones = reused.bones.data();
        const auto *materials = reused.materials.data();
        const auto retained = animator.evaluate(30, 0, true);
        mmd::VpdPose pose;
        pose.bones = {{.name = "bone", .translation = {1, 0, 0}},
                      {.name = "bone", .translation = {3, 0, 0}},
                      {.name = "absent", .translation = {9, 0, 0}}};
        animator.setPose(&pose);
        animator.evaluateInto(reused, 30, 0, true, {}, mmd::VertexOutput::omitUnchangedGpu);
        assert(reused.vertices.empty() && reused.bones[0].translation[0] == 3 && reused.bones[1].translation[0] == 2);
        assert(reused.bones.data() == bones && reused.materials.data() == materials);
        assert(retained.vertices.size() == 1 && retained.bones[0].translation[0] == 2);
        animator.setPose(nullptr);
        mmd::VmdMotion second;
        second.bones.resize(1);
        second.bones[0].name = "bone";
        second.bones[0].translation = {-4, 0, 0};
        animator.setMotion(&second);
        animator.evaluateInto(reused, 0, 0, true);
        assert(reused.bones[0].translation[0] == -4 && reused.morphWeights[0] == 0 && reused.visible);
        animator.setMotion(nullptr);
        animator.evaluateInto(reused, 0, 0, false);
        assert(reused.vertices.size() == 1 && reused.morphWeights.empty() && reused.bones[0].translation[0] == 0);
        assert(reused.visible && reused.materials[0].diffuse == source.materials[0].diffuse);
        animator.setMotion(&first);
        animator.evaluateInto(reused, 30, 0, true);
        assert(reused.morphWeights[0] == 1 && reused.bones[0].translation[0] == 2 && !reused.visible);
    }
    // Cached topology preserves the expansion order and cycle/budget policy,
    // while every frame resets propagation, flags and effective weights.
    {
        mmd::PmxModel graph;
        graph.morphs = {
            {.name = "root", .type = 0, .offsets = {{.index = 1, .scalar = 0.5F}, {.index = 2, .scalar = 2.0F}}},
            {.name = "leaf", .type = 1},
            {.name = "flip", .type = 9, .offsets = {{.index = 1, .scalar = 0.5F}}},
        };
        mmd::MorphWeightExpander cache(graph);
        mmd::MorphExpansionResult result;
        for (unsigned iteration = 0; iteration < 10; ++iteration) {
            const std::array<float, 3> roots{iteration % 2 == 0 ? 1.0F : 0.0F, 0.0F, 0.0F};
            for (std::uint64_t budget : {0U, 1U, 2U, 3U, 100U}) {
                const auto expected = mmd::expandMorphWeights(graph, roots, {budget});
                cache.expandInto(result, roots, {budget});
                assert(result.effectiveWeights == expected.effectiveWeights);
                assert(result.budgetExceeded == expected.budgetExceeded);
                assert(result.cycleDetected == expected.cycleDetected);
            }
        }
        const auto *storage = result.effectiveWeights.data();
        cache.expandInto(result, rootWeights);
        assert(result.effectiveWeights.data() == storage && result.effectiveWeights[1] == 1.5F);
        cache.expandInto(result, result.effectiveWeights);
        assert(result.effectiveWeights[1] == 1.5F); // aliased root input is read before output reset
        const std::array<float, 3> nanRoots{std::numeric_limits<float>::quiet_NaN(), 0, 0};
        cache.expandInto(result, nanRoots);
        assert(result.effectiveWeights[1] == 0 && !result.budgetExceeded);

        mmd::PmxModel cyclic;
        cyclic.morphs = {
            {.name = "cycle", .type = 0, .offsets = {{.index = 0, .scalar = 1}, {.index = 1, .scalar = 2}}},
            {.name = "leaf", .type = 1}};
        mmd::MorphWeightExpander cycleCache(cyclic);
        cycleCache.expandInto(result, selfRoot, {.maxSteps = 1});
        assert(result.cycleDetected && result.budgetExceeded && result.effectiveWeights[1] == 0);
        cycleCache.expandInto(result, selfRoot, {.maxSteps = 2});
        assert(result.cycleDetected && !result.budgetExceeded && result.effectiveWeights[1] == 2);
        cycleCache.expandInto(result, {});
        assert(result.cycleDetected && !result.budgetExceeded && result.effectiveWeights[1] == 0);
    }
    // Opting out is allowed only for unchanged GPU bind attributes. Public
    // callers retain the original full-output default and CPU skinning path.
    {
        mmd::PmxModel source;
        source.bones.resize(1);
        source.vertices.resize(1);
        source.vertices[0].bones[0] = 0;
        source.vertices[0].position = {1.0F, 2.0F, 3.0F};
        source.materials.resize(1);
        source.materials[0].diffuse = {0.2F, 0.3F, 0.4F, 1.0F};
        source.morphs = {{.name = "vertex", .type = 1, .offsets = {{.index = 0, .vector3 = {0.5F, 0.0F, 0.0F}}}}};
        const std::array<mmd::MorphOverride, 1> overrides{{{.index = 0, .weight = 1.0F}}};
        mmd::MmdAnimator animator(source);
        const auto full = animator.evaluate(0, 0, true, overrides);
        const auto omitted = animator.evaluate(0, 0, true, overrides, mmd::VertexOutput::omitUnchangedGpu);
        assert(full.vertices.size() == 1 && omitted.vertices.empty());
        assert(full.morphWeights == omitted.morphWeights && omitted.morphWeights[0] == 1.0F);
        assert(full.bones[0].translation == omitted.bones[0].translation);
        assert(full.materials[0].diffuse == omitted.materials[0].diffuse);
        const auto cpu = animator.evaluate(0, 0, false, overrides, mmd::VertexOutput::omitUnchangedGpu);
        assert(cpu.vertices.size() == 1 && cpu.vertices[0].position[0] == 1.5F);
        const std::array<mmd::MorphOverride, 1> temporary{
            {{.index = mmd::MorphOverride::temporary, .weight = 1.0F, .type = 1, .offsets = source.morphs[0].offsets}}};
        assert(animator.evaluate(0, 0, true, temporary, mmd::VertexOutput::omitUnchangedGpu).vertices[0].position[0] ==
               1.5F);
        for (std::uint8_t type = 3; type <= 7; ++type) {
            auto uvSource = source;
            uvSource.morphs[0].type = type;
            uvSource.morphs[0].offsets[0].vector4 = {0.25F, 0.5F, 0.75F, 1.0F};
            mmd::MmdAnimator uvAnimator(uvSource);
            const auto uvFull = uvAnimator.evaluate(0, 0, true, overrides);
            const auto uvOptimized = uvAnimator.evaluate(0, 0, true, overrides, mmd::VertexOutput::omitUnchangedGpu);
            assert(uvOptimized.vertices.size() == 1);
            assert(uvFull.vertices[0].uv == uvOptimized.vertices[0].uv);
            assert(uvFull.vertices[0].additionalUv == uvOptimized.vertices[0].additionalUv);
            if (type == 3)
                assert(uvOptimized.vertices[0].uv[0] == 0.25F);
            else
                assert(uvOptimized.vertices[0].additionalUv[type - 4][0] == 0.25F);
        }
        source.softBodies.resize(1);
        mmd::MmdAnimator softAnimator(source);
        assert(softAnimator.evaluate(0, 0, true, {}, mmd::VertexOutput::omitUnchangedGpu).vertices.size() == 1);
    }
    return 0;
}
