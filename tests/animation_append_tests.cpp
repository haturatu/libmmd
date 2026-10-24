#include <mmd/animation.hpp>

#include <array>
#include <cmath>
#include <iostream>
#include <numbers>
#include <stdexcept>

namespace {
void require(bool condition, const char *message) {
    if (!condition)
        throw std::runtime_error(message);
}

bool near(const mmd::Float3 &a, const mmd::Float3 &b) {
    for (std::size_t i = 0; i < a.size(); ++i)
        if (std::abs(a[i] - b[i]) > 1e-4F)
            return false;
    return true;
}

bool sameRotation(const mmd::Float4 &a, const mmd::Float4 &b) {
    float dot{};
    for (std::size_t i = 0; i < a.size(); ++i)
        dot += a[i] * b[i];
    return std::abs(std::abs(dot) - 1.0F) < 1e-5F;
}

void checkIkAppend(bool appendSource, bool afterPhysics, bool localAppend) {
    mmd::PmxModel model;
    model.bones.resize(6);
    model.bones[0].name = "link";
    model.bones[1].name = "effector";
    model.bones[1].parent = 0;
    model.bones[1].position = {0.0F, 1.0F, 0.0F};
    model.bones[2].name = "IK";
    model.bones[2].position = {1.0F, 0.0F, 0.0F};
    model.bones[2].flags = 0x0020U;
    model.bones[2].ikTarget = 1;
    model.bones[2].ikLoopCount = 8;
    model.bones[2].ikLimitAngle = std::numbers::pi_v<float>;
    model.bones[2].ikLinks.push_back({.bone = 0});
    model.bones[3].name = "linkD";
    model.bones[3].flags =
        static_cast<std::uint16_t>(0x0100U | (afterPhysics ? 0x1000U : 0U) | (localAppend ? 0x0080U : 0U));
    model.bones[3].inheritParent = 0;
    model.bones[3].inheritRatio = 1.0F;
    model.bones[3].deformLayer = 1;
    model.bones[4].name = "effectorD";
    model.bones[4].parent = 3;
    model.bones[4].position = model.bones[1].position;
    model.bones[4].flags = afterPhysics ? 0x1000U : 0U;
    model.bones[4].deformLayer = 1;
    model.bones[5].name = "grant";
    model.bones[5].deformLayer = -1;
    if (appendSource) {
        model.bones[0].flags = 0x0100U;
        model.bones[0].inheritParent = 5;
        model.bones[0].inheritRatio = 1.0F;
    }
    model.vertices.resize(2);
    for (std::size_t i = 0; i < model.vertices.size(); ++i) {
        model.vertices[i].position = model.bones[1].position;
        model.vertices[i].bones[0] = i == 0 ? 1 : 4;
    }

    mmd::VmdMotion motion;
    motion.lastFrame = 30;
    for (const auto frame : {0U, 30U}) {
        mmd::VmdBoneKey key;
        key.name = appendSource ? "grant" : "link";
        key.frame = frame;
        const float halfAngle = frame == 0 ? std::numbers::pi_v<float> / 4.0F : std::numbers::pi_v<float> / 6.0F;
        key.rotation = {std::sin(halfAngle), 0.0F, 0.0F, std::cos(halfAngle)};
        motion.bones.push_back(key);
    }

    mmd::MmdAnimator animator(model);
    animator.setMotion(&motion);
    for (const float frame : {0.0F, 30.0F, 0.0F}) {
        const auto result = animator.evaluate(frame);
        require(!animator.ikEvaluationStats().budgetExceeded, "IK unexpectedly exhausted its budget");
        require(near(result.vertices[0].position, model.bones[2].position), "IK did not reach its target");
        // An identity-keyed, ratio-one copy must drive the same geometry as
        // the IK link, even when its sampled rotation and IK do not commute.
        require(sameRotation(result.bones[0].rotation, result.bones[3].rotation),
                "append copy lost the IK link's rotation");
        require(near(result.vertices[0].position, result.vertices[1].position),
                "append copy deformed the mesh away from the IK effector");
    }
}
} // namespace

int main() {
    try {
        // Same-phase copies exercise dirty rebuilds during IK. Post-physics
        // copies exercise the full rebuild after the pre-physics IK pass.
        for (const bool appendSource : {false, true})
            for (const bool afterPhysics : {false, true})
                for (const bool localAppend : {false, true})
                    checkIkAppend(appendSource, afterPhysics, localAppend);
    } catch (const std::exception &error) {
        std::cerr << "[ERROR] " << error.what() << '\n';
        return 1;
    }
}
