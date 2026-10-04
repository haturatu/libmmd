#include <mmd/animation.hpp>
#include <mmd/physics.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <limits>
#include <stdexcept>
#include <string>

#if LIBMMD_HAS_BULLET
#include "bullet_compatibility.hpp"
#endif

namespace {
void require(bool condition, const char *message) {
    if (!condition)
        throw std::runtime_error(message);
}

void settingsValidation() {
    mmd::PmxModel model;
    const auto invalid = [&](mmd::PhysicsSettings settings) {
        bool rejected = false;
        try {
            mmd::MmdPhysics physics(model, settings);
        } catch (const std::invalid_argument &) {
            rejected = true;
        }
        require(rejected, "invalid settings accepted");
    };
    mmd::PhysicsSettings settings;
    settings.solverIterations = 0;
    invalid(settings);
    settings = {};
    settings.fixedTimeStep = std::numeric_limits<float>::quiet_NaN();
    invalid(settings);
    settings.fixedTimeStep = 0;
    invalid(settings);
    settings = {};
    settings.compatibility.constraintForceMixing = -1;
    invalid(settings);
    settings = {};
    settings.compatibility.stopErp = 2;
    invalid(settings);
    settings.compatibility = mmd::PhysicsCompatibilityProfile::mmd275Compatible();
    settings.compatibility.useFrameOffset = true;
    invalid(settings);
}

#if LIBMMD_HAS_BULLET
mmd::Float3 rotate(const mmd::Float4 &q, const mmd::Float3 &v) {
    const mmd::Float3 t{2 * (q[1] * v[2] - q[2] * v[1]), 2 * (q[2] * v[0] - q[0] * v[2]),
                        2 * (q[0] * v[1] - q[1] * v[0])};
    return {v[0] + q[3] * t[0] + q[1] * t[2] - q[2] * t[1], v[1] + q[3] * t[1] + q[2] * t[0] - q[0] * t[2],
            v[2] + q[3] * t[2] + q[0] * t[1] - q[1] * t[0]};
}

void near(const mmd::Float3 &a, const mmd::Float3 &b, const char *message) {
    for (std::size_t axis = 0; axis < 3; ++axis)
        require(std::isfinite(a[axis]) && std::abs(a[axis] - b[axis]) < 0.002F, message);
}

mmd::PmxRigidBody body(int bone, std::uint8_t mode, mmd::Float3 position) {
    mmd::PmxRigidBody result;
    result.bone = bone;
    result.mode = mode;
    result.position = position;
    result.size = {0.5F, 0, 0};
    result.mass = 1;
    result.collisionMask = 0;
    return result;
}

mmd::PmxBone bone(const char *name, mmd::Float3 position = {}, int parent = -1) {
    mmd::PmxBone result;
    result.name = name;
    result.position = position;
    result.parent = parent;
    return result;
}

mmd::PmxJoint link(int a, int b) {
    mmd::PmxJoint joint;
    joint.bodyA = a;
    joint.bodyB = b;
    joint.translationMinimum = {1, 1, 1};
    joint.translationMaximum = {-1, -1, -1};
    joint.rotationMinimum = {1, 1, 1};
    joint.rotationMaximum = {-1, -1, -1};
    return joint;
}

void jointNormalization() {
    for (int scenario = 0; scenario < 9; ++scenario) {
        mmd::PmxModel model;
        model.bones = {bone("parent"), bone("child", {}, 0), bone("unrelated")};
        model.rigidBodies = {body(0, 1, {}), body(1, 2, {}), body(2, 1, {})};
        if (scenario != 0)
            model.joints.push_back(link(0, 1));
        if (scenario == 1)
            model.joints[0].physicsEnabled = false;
        if (scenario == 2)
            model.joints[0].bodyA = 2;
        if (scenario == 3)
            model.joints[0].type = 1;
        if (scenario == 4)
            model.joints[0].position[0] = std::numeric_limits<float>::quiet_NaN();
        if (scenario == 5)
            model.joints[0].bodyA = 999;
        if (scenario == 6)
            model.rigidBodies[0].physicsEnabled = false;
        if (scenario == 8)
            std::swap(model.joints[0].bodyA, model.joints[0].bodyB);
        mmd::MmdPhysics physics(model);
        require(physics.bodyMode(1) == (scenario >= 7 ? 1 : 2), "mode 2 normalization ignored joint validity");
    }
}

void nestedModes() {
    for (const std::uint8_t parentMode : {std::uint8_t{1}, std::uint8_t{2}}) {
        for (bool reversed : {false, true}) {
            mmd::PmxModel model;
            model.bones.push_back(bone("root"));
            model.bones.push_back(bone("child", {0, -2, 0}, 0));
            model.bones.push_back(bone("grandchild", {0, -4, 0}, 1));
            for (int index = 0; index < 3; ++index) {
                auto position = model.bones[static_cast<std::size_t>(index)].position;
                position[1] += 0.5F; // Nonzero body-to-bone bind offset.
                model.rigidBodies.push_back(body(index, index == 0 ? parentMode : std::uint8_t{2}, position));
                mmd::PmxVertex vertex;
                vertex.position = position;
                vertex.bones[0] = index;
                vertex.weights[0] = 1;
                model.vertices.push_back(vertex);
            }
            if (reversed)
                std::reverse(model.rigidBodies.begin(), model.rigidBodies.end());
            model.joints = {link(0, 1), link(1, 2)};
            mmd::MmdPhysics physics(model);
            physics.setGravity({0, 0, 0});
            mmd::MmdAnimator animator(model);
            animator.setPhysics(&physics);
            mmd::VmdMotion motion;
            motion.bones.push_back({.name = "root", .frame = 0, .translation = {1.5F, 0, 0}});
            animator.setMotion(&motion);
            static_cast<void>(animator.evaluate(0, 0));
            for (std::size_t index = 0; index < 3; ++index) {
                const auto bone = model.rigidBodies[index].bone;
                require(physics.bodyMode(index) == (bone == 0 ? parentMode : 1), "incorrect nested runtime mode");
                require(model.rigidBodies[index].mode == (bone == 0 ? parentMode : 2), "PMX mode was mutated");
                const float halfAngle = 0.2F * static_cast<float>(bone + 1);
                physics.teleportBody(index, {{static_cast<float>(bone + 2), static_cast<float>(bone), 0},
                                             {0, 0, std::sin(halfAngle), std::cos(halfAngle)}});
            }
            const auto frame = animator.evaluate(0, 0);
            for (std::size_t index = 0; index < 3; ++index) {
                const auto bone = static_cast<std::size_t>(model.rigidBodies[index].bone);
                if (bone == 0 && parentMode == 2) {
                    const auto rotated = rotate(frame.bones[0].rotation, model.bones[0].position);
                    mmd::Float3 position = frame.bones[0].translation;
                    for (std::size_t axis = 0; axis < 3; ++axis)
                        position[axis] += rotated[axis];
                    near(position, {1.5F, 0, 0}, "standalone mode 2 lost animated position");
                } else {
                    near(frame.vertices[bone].position, physics.bodyTransform(index).position,
                         "nested mesh did not follow displaced/rotated body");
                }
            }
        }
    }
    // An invalid/disabled body on a parent bone is effectively kinematic.
    for (int scenario = 0; scenario < 4; ++scenario) {
        mmd::PmxModel model;
        model.bones.push_back(bone("root"));
        model.bones.push_back(bone("child", {}, 0));
        auto parent = body(0, 2, {});
        if (scenario == 0)
            parent.physicsEnabled = false;
        if (scenario == 1)
            parent.mass = 0;
        if (scenario == 2)
            parent.size = {};
        if (scenario == 3)
            parent.mode = 0;
        model.rigidBodies = {body(1, 2, {}), parent, body(-1, 2, {}), body(999, 2, {})};
        model.joints = {link(0, 1)};
        mmd::MmdPhysics physics(model);
        require(physics.bodyMode(0) == 2, "unusable parent normalized child");
        require(physics.bodyMode(2) == 2 && physics.bodyMode(3) == 2, "unbound mode changed");
    }
}

void collisionMovesMesh(mmd::PhysicsCompatibilityProfile profile) {
    mmd::PmxModel model;
    model.bones.push_back(bone("skirt top", {0, 4, 0}));
    model.bones.push_back(bone("skirt child", {0.4F, 0, 0}, 0));
    auto top = body(0, 2, model.bones[0].position);
    auto child = body(1, 2, model.bones[1].position);
    child.group = 6;
    child.collisionMask = 0xffbf;
    auto thigh = body(-1, 0, {});
    thigh.size[0] = 1;
    thigh.collisionMask = 0xffff;
    model.rigidBodies = {child, thigh, top};
    model.joints = {link(2, 0)};
    mmd::PmxVertex vertex;
    vertex.position = child.position;
    vertex.bones[0] = 1;
    vertex.weights[0] = 1;
    model.vertices.push_back(vertex);
    mmd::PhysicsSettings settings;
    settings.compatibility = profile;
    settings.solverIterations = 25;
    settings.fixedTimeStep = 1.0F / 240.0F;
    mmd::MmdPhysics physics(model, settings);
    physics.setGravity({0, 0, 0});
    mmd::MmdAnimator animator(model);
    animator.setPhysics(&physics);
    static_cast<void>(animator.evaluate(0, 0));
    mmd::AnimatedModelFrame frame;
    for (int index = 1; index <= 60; ++index)
        frame = animator.evaluate(static_cast<float>(index) / 2, 1.0F / 60.0F);
    const auto center = physics.bodyTransform(0).position;
    require(center[0] > 1.4F, "skirt body did not escape thigh collision");
    near(frame.vertices[0].position, center, "collision displaced body but left mesh behind");
}

void profileDrivesSolver() {
    mmd::PmxModel model;
    model.rigidBodies = {body(-1, 1, {}), body(-1, 1, {2, 0, 0})};
    mmd::PmxJoint joint;
    joint.bodyA = 0;
    joint.bodyB = 1;
    joint.position = {0, 1, 0};
    joint.rotationMinimum = {-1, -1, -1};
    joint.rotationMaximum = {1, 1, 1};
    joint.translationSpring = {5, 5, 5};
    model.joints.push_back(joint);
    mmd::PhysicsSettings settings;
    // Isolate the lever-arm correction from the frame-offset setting.
    settings.compatibility.useFrameOffset = false;
    mmd::MmdPhysics native(model, settings);
    settings.compatibility.useBullet275Constraint = true;
    mmd::MmdPhysics compatible(model, settings);
    for (auto *physics : {&native, &compatible}) {
        require(physics->jointCount() == 1, "profile constraint not created");
        physics->setGravity({0, 0, 0});
        physics->teleportBody(1, {{4, 0, 0}, {0, 0, 0, 1}});
        physics->step(1.0F / 60.0F);
    }
    const auto a = native.bodyTransform(0);
    const auto b = compatible.bodyTransform(0);
    require(std::abs(a.rotation[2] - b.rotation[2]) > 0.001F,
            "public compatibility profile did not affect off-center constraint torque");
    for (const auto *physics : {&native, &compatible})
        for (std::size_t index = 0; index < 2; ++index)
            for (float component : physics->bodyTransform(index).position)
                require(std::isfinite(component), "profile constraint simulation became nonfinite");
}

void constraintJacobian() {
    btEmptyShape shape;
    btRigidBody::btRigidBodyConstructionInfo construction(1, nullptr, &shape, {1, 1, 1});
    btRigidBody a(construction), b(construction);
    a.setWorldTransform(btTransform::getIdentity());
    b.setWorldTransform(btTransform(btQuaternion::getIdentity(), {2, 0, 0}));
    for (bool spring : {false, true}) {
        const btTransform frameA(btQuaternion({0, 0, 1}, 0.3F), {0, 1, 0});
        const btTransform frameB(btQuaternion::getIdentity(), {0, 2, 0});
        mmd::internal::Bullet275SpringConstraint compatible(a, b, frameA, frameB, true);
        btGeneric6DofSpringConstraint native(a, b, frameA, frameB, true);
        for (auto *constraint : {static_cast<btGeneric6DofSpringConstraint *>(&compatible), &native}) {
            constraint->setUseFrameOffset(false);
            constraint->setLinearLowerLimit({0, 1, 0});
            constraint->setLinearUpperLimit({0, -1, 0}); // Middle axis free, exercises row indexing.
            constraint->setAngularLowerLimit({1, 1, 1});
            constraint->setAngularUpperLimit({-1, -1, -1});
            if (spring) {
                constraint->enableSpring(1, true);
                constraint->setStiffness(1, 5);
            }
            btTypedConstraint::btConstraintInfo1 count{};
            constraint->getInfo1(&count);
            btScalar j1linear[18]{}, j2linear[18]{}, j1angular[18]{}, j2angular[18]{};
            btScalar error[18]{}, cfm[18]{}, lower[18]{}, upper[18]{};
            btTypedConstraint::btConstraintInfo2 info{};
            info.fps = 120;
            info.erp = 0.475F;
            info.rowskip = 3;
            info.m_J1linearAxis = j1linear;
            info.m_J2linearAxis = j2linear;
            info.m_J1angularAxis = j1angular;
            info.m_J2angularAxis = j2angular;
            info.m_constraintError = error;
            info.cfm = cfm;
            info.m_lowerLimit = lower;
            info.m_upperLimit = upper;
            info.m_numIterations = 10;
            constraint->getInfo2(&info);
            int row = 0;
            for (int axis = 0; axis < 3; ++axis) {
                if (!constraint->getTranslationalLimitMotor()->needApplyForce(axis))
                    continue;
                const auto anchor = constraint == &compatible ? compatible.getCalculatedTransformA().getOrigin()
                                                              : native.getCalculatedTransformB().getOrigin();
                const auto expected = (anchor - a.getCenterOfMassPosition()).cross(frameA.getBasis().getColumn(axis));
                for (int component = 0; component < 3; ++component)
                    require(std::abs(j1angular[row * 3 + component] - expected[component]) < 1e-5F,
                            "incorrect frame-A constraint lever arm");
                ++row;
            }
        }
    }
}

void checkModel(const char *path) {
    const auto model = mmd::pmx::load(path);
    for (bool compatible : {false, true}) {
        mmd::PhysicsSettings settings;
        if (compatible)
            settings.compatibility = mmd::PhysicsCompatibilityProfile::mmd275Compatible();
        mmd::MmdPhysics physics(model, settings);
        std::size_t skirt = 0, normalized = 0, tops = 0;
        for (std::size_t index = 0; index < model.rigidBodies.size(); ++index) {
            const auto &source = model.rigidBodies[index];
            if (source.mode != 2 || source.group != 6 || source.shape != 1 || !source.name.starts_with("裙_"))
                continue;
            ++skirt;
            if (physics.bodyMode(index) == 1)
                ++normalized;
            if (physics.bodyMode(index) == 2)
                ++tops;
        }
        std::printf("Vivian topology: %zu skirt boxes, %zu normalized, %zu roots\n", skirt, normalized, tops);
        require(skirt == 150 && normalized == 135 && tops == 15, "unexpected Vivian skirt topology");
        mmd::MmdAnimator animator(model);
        animator.setPhysics(&physics);
        static_cast<void>(animator.evaluate(0, 0));
        for (int step = 1; step <= 120; ++step) {
            const auto frame = animator.evaluate(static_cast<float>(step) / 2, 1.0F / 60.0F);
            for (std::size_t index = 0; index < model.rigidBodies.size(); ++index) {
                const auto &source = model.rigidBodies[index];
                if (!source.name.starts_with("裙_") || source.mode != 2 || physics.bodyMode(index) != 1 ||
                    source.bone < 0 || static_cast<std::size_t>(source.bone) >= frame.bones.size())
                    continue;
                const auto bone = static_cast<std::size_t>(source.bone);
                // Skinning translation + rotated body bind center reconstructs
                // the simulated center, including nonzero rigid-body bind rotations.
                auto center = rotate(frame.bones[bone].rotation, source.position);
                for (std::size_t axis = 0; axis < 3; ++axis)
                    center[axis] += frame.bones[bone].translation[axis];
                const auto actual = physics.bodyTransform(index).position;
                for (std::size_t axis = 0; axis < 3; ++axis) {
                    if (std::abs(center[axis] - actual[axis]) >= 0.002F)
                        std::fprintf(stderr,
                                     "Mismatch: step=%d body=%s bone=%s flags=%x center=(%f,%f,%f) body=(%f,%f,%f)\n",
                                     step, source.name.c_str(), model.bones[bone].name.c_str(), model.bones[bone].flags,
                                     center[0], center[1], center[2], actual[0], actual[1], actual[2]);
                }
                near(center, actual, "Vivian mesh/body center diverged");
            }
            for (const auto &vertex : frame.vertices)
                for (float component : vertex.position)
                    require(std::isfinite(component), "Vivian simulation produced nonfinite mesh");
        }
        std::printf(
            "Vivian (%s): %zu skirt boxes, %zu mode 2 roots, %zu effective mode 1 children; 120 frames finite\n",
            compatible ? "mmd-2.75-compatible" : "default", skirt, tops, normalized);
    }
}
#endif
} // namespace

int main(int argc, char **argv) {
    try {
        settingsValidation();
#if LIBMMD_HAS_BULLET
        jointNormalization();
        nestedModes();
        collisionMovesMesh({});
        collisionMovesMesh(mmd::PhysicsCompatibilityProfile::mmd275Compatible());
        constraintJacobian();
        profileDrivesSolver();
        if (argc > 1)
            checkModel(argv[1]);
#else
        static_cast<void>(argc);
        static_cast<void>(argv);
#endif
        std::puts("physics compatibility tests passed");
    } catch (const std::exception &error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
}
