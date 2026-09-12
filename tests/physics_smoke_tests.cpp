#include <mmd/animation.hpp>
#include <mmd/physics.hpp>

#include <cassert>
#include <cmath>
#include <cstdio>

// Regression test for the DAYO_HAS_BULLET / LIBMMD_HAS_BULLET guard
// mismatch: the Bullet backend must actually be active when the library
// reports LIBMMD_HAS_BULLET=1, otherwise MmdPhysics silently stays a no-op.
int main() {
    mmd::PmxModel model;
    mmd::PmxRigidBody body;
    body.name = "smoke";
    body.group = 0;
    body.collisionMask = 0xffff;
    body.shape = 0;
    body.size = {1.0F, 0.0F, 0.0F};
    body.position = {0.0F, 10.0F, 0.0F};
    body.rotation = {0.0F, 0.0F, 0.0F};
    body.mass = 1.0F;
    body.linearDamping = 0.0F;
    body.angularDamping = 0.0F;
    body.restitution = 0.0F;
    body.friction = 0.5F;
    body.mode = 1;
    body.physicsEnabled = true;
    model.rigidBodies.push_back(body);

    mmd::MmdPhysics physics(model);
#if LIBMMD_HAS_BULLET
    if (!physics.available()) {
        std::printf("FAIL: physics unavailable despite LIBMMD_HAS_BULLET=1\n");
        return 1;
    }
    if (physics.bodyCount() != 1) {
        std::printf("FAIL: expected 1 body, got %zu\n", physics.bodyCount());
        return 1;
    }
    const auto before = physics.bodyTransform(0);
    for (int i = 0; i < 60; ++i)
        physics.step(1.0F / 60.0F);
    const auto after = physics.bodyTransform(0);
    if (!std::isfinite(after.position[0]) || !std::isfinite(after.position[1]) || !std::isfinite(after.position[2])) {
        std::printf("FAIL: body transform is non-finite after gravity simulation\n");
        return 1;
    }
    if (after.position[1] >= before.position[1]) {
        std::printf("FAIL: body did not fall under gravity\n");
        return 1;
    }

    mmd::PmxModel syncModel;
    syncModel.bones.push_back({.name = "animated"});

    mmd::PmxRigidBody animatedBody;
    animatedBody.name = "animated body";
    animatedBody.bone = 0;
    animatedBody.group = 0;
    animatedBody.collisionMask = 0;
    animatedBody.shape = 0;
    animatedBody.size = {0.5F, 0.0F, 0.0F};
    animatedBody.position = {0.0F, 0.0F, 0.0F};
    animatedBody.mass = 1.0F;
    animatedBody.mode = 2;
    syncModel.rigidBodies.push_back(animatedBody);

    mmd::PmxRigidBody unboundBody = body;
    unboundBody.name = "unbound body";
    unboundBody.bone = -1;
    unboundBody.position = {0.0F, 10.0F, 0.0F};
    syncModel.rigidBodies.push_back(unboundBody);

    mmd::VmdMotion motion;
    motion.bones.push_back({.name = "animated", .frame = 0, .translation = {1.0F, 0.0F, 0.0F}});
    mmd::MmdPhysics syncPhysics(syncModel);
    mmd::MmdAnimator animator(syncModel);
    animator.setMotion(&motion);
    animator.setPhysics(&syncPhysics);
    static_cast<void>(animator.evaluate(0.0F, 0.0F));
    if (std::abs(syncPhysics.bodyTransform(0).position[0] - 1.0F) > 1e-4F) {
        std::printf("FAIL: first animated pose did not synchronize the bound body\n");
        return 1;
    }

    const auto unboundInitial = syncPhysics.bodyTransform(1);
    static_cast<void>(animator.evaluate(1.0F, 1.0F / 30.0F));
    const auto unboundAdvanced = syncPhysics.bodyTransform(1);
    if (unboundAdvanced.position[1] >= unboundInitial.position[1]) {
        std::printf("FAIL: unbound body did not advance before pose replacement\n");
        return 1;
    }

    mmd::VpdPose pose;
    pose.bones.push_back({.name = "animated", .translation = {2.0F, 0.0F, 0.0F}});
    animator.setPose(&pose);
    static_cast<void>(animator.evaluate(1.0F, 0.0F));
    const auto unboundReset = syncPhysics.bodyTransform(1);
    if (std::abs(unboundReset.position[1] - unboundInitial.position[1]) > 1e-4F) {
        std::printf("FAIL: pose replacement did not reset unbound physics state\n");
        return 1;
    }
#else
    if (physics.available() || physics.bodyCount() != 0) {
        std::printf("FAIL: physics unexpectedly available without Bullet\n");
        return 1;
    }
#endif
    std::printf("physics smoke test passed\n");
    return 0;
}
