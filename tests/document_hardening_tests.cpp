#include <mmd/animation.hpp>
#include <mmd/document.hpp>

#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <limits>
#include <stdexcept>

namespace {
void require(bool condition, const char *message) {
    if (!condition)
        throw std::runtime_error(message);
}

void deepBoneEdits() {
    mmd::PmxModel model;
    model.metadata.version = 2.1F;
    constexpr std::size_t count = 100'000;
    model.bones.resize(count);
    // Parent indices run forward so visiting bone zero used to recurse 100k
    // levels, rather than hitting the already visited predecessor each time.
    for (std::size_t index = 0; index + 1 < count; ++index)
        model.bones[index].parent = static_cast<std::int32_t>(index + 1);
    require(mmd::pmx::validate(model).valid(), "deep chain rejected by global validation");
    mmd::PmxDocument document(std::move(model));
    auto transaction = document.transaction();
    require(transaction.setBoneParent(document.boneHandle(0), document.boneHandle(2)), "deep parent edit failed");
    require(transaction.setBoneInherit(document.boneHandle(0), document.boneHandle(count - 1), 1, true, false),
            "deep inheritance edit failed");
    require(!transaction.setBoneParent(document.boneHandle(count - 1), document.boneHandle(0)),
            "deep parent cycle accepted");
    require(!transaction.setBoneInherit(document.boneHandle(count - 1), document.boneHandle(0), 1, false, true),
            "deep mixed dependency cycle accepted");
    require(transaction.commit().committed, "valid deep edits failed to commit after cycle rollback");
    require(document.model().bones[count - 1].parent == -1, "rejected edge was not rolled back");
}

void propertyDependencyCycle() {
    mmd::PmxModel model;
    model.metadata.version = 2.1F;
    model.bones.resize(2);
    model.bones[0].parent = 1;
    mmd::PmxDocument document(model);
    auto invalid = model.bones[1];
    invalid.flags = 0x0100U;
    invalid.inheritParent = 0;
    const auto result = document.replaceBone(document.boneHandle(1), invalid);
    require(!result.committed && !result.validation.valid(), "property edit accepted mixed dependency cycle");
    require(document.model().bones[1] == model.bones[1], "cycle property edit was not rolled back");
}

mmd::PmxModel physicsModel() {
    mmd::PmxModel model;
    model.metadata.version = 2.1F;
    model.bones.resize(1);
    model.vertices.resize(1);
    model.vertices[0].bones[0] = 0;
    model.materials.resize(1);
    model.rigidBodies.resize(1);
    model.rigidBodies[0].bone = 0;
    model.rigidBodies[0].size = {1, 1, 1};
    model.rigidBodies[0].mass = 1;
    model.rigidBodies[0].mode = 2;
    model.softBodies.resize(1);
    model.softBodies[0].material = 0;
    model.softBodies[0].anchors.push_back({0, 0, false});
    model.softBodies[0].pinnedVertices.push_back(0);
    return model;
}

void sameDiagnostics(const mmd::ValidationResult &global, const mmd::ValidationResult &property) {
    require(!global.valid() && !property.valid(), "invalid physics data accepted by a validator");
    require(global.issues.size() == property.issues.size(), "physics validators disagree on issues");
    for (std::size_t index = 0; index < global.issues.size(); ++index) {
        require(global.issues[index].code == property.issues[index].code &&
                    global.issues[index].message == property.issues[index].message &&
                    global.issues[index].location.field == property.issues[index].location.field,
                "physics validators disagree on diagnostics");
        require(property.issues[index].location.id != 0, "property issue lost handle identity");
    }
}

void saveRejects(const mmd::PmxModel &model) {
    const auto path = std::filesystem::temp_directory_path() / "libmmd-invalid-physics.pmx";
    bool rejected = false;
    try {
        static_cast<void>(mmd::pmx::save(path, model));
    } catch (const std::runtime_error &) {
        rejected = true;
    }
    if (!rejected)
        std::filesystem::remove(path);
    require(rejected, "save accepted invalid physics data");
}

void rigidBodyValidation() {
    const auto original = physicsModel();
    require(mmd::pmx::validate(original).valid(), "valid physics fixture rejected");
    for (int scenario = 0; scenario < 19; ++scenario) {
        auto invalid = original;
        auto &body = invalid.rigidBodies[0];
        const float nan = std::numeric_limits<float>::quiet_NaN();
        if (scenario == 0)
            body.shape = 255;
        if (scenario == 1)
            body.mode = 255;
        if (scenario == 2)
            body.group = 16;
        if (scenario >= 3 && scenario <= 5)
            body.size[static_cast<std::size_t>(scenario - 3)] = nan;
        if (scenario >= 6 && scenario <= 8)
            body.position[static_cast<std::size_t>(scenario - 6)] = nan;
        if (scenario >= 9 && scenario <= 11)
            body.rotation[static_cast<std::size_t>(scenario - 9)] = nan;
        if (scenario == 12)
            body.mass = nan;
        if (scenario == 13)
            body.linearDamping = nan;
        if (scenario == 14)
            body.angularDamping = nan;
        if (scenario == 15)
            body.restitution = nan;
        if (scenario == 16)
            body.friction = std::numeric_limits<float>::infinity();
        if (scenario == 17)
            body.bone = 1;
        if (scenario == 18) {
            body.physicsEnabled = false;
            body.shape = 255;
        }
        mmd::PmxDocument document(original);
        const auto property = document.replaceRigidBody(document.rigidBodyHandle(0), body);
        require(!property.committed, "invalid rigid body property committed");
        sameDiagnostics(mmd::pmx::validate(invalid), property.validation);
        require(document.model().rigidBodies == original.rigidBodies, "rejected property changed document");
        auto transaction = document.transaction();
        const auto handle = document.rigidBodyHandle(0);
        if (scenario == 0)
            require(transaction.setRigidBodyShape(handle, body.shape, body.size), "shape setter failed");
        else if (scenario == 1)
            require(transaction.setRigidBodyMode(handle, body.mode), "mode setter failed");
        else if (scenario == 2)
            require(transaction.setRigidBodyCollision(handle, body.group, body.collisionMask), "group setter failed");
        else
            require(transaction.setRigidBody(handle, body), "rigid body setter failed");
        require(!transaction.commit().committed, "invalid rigid body transaction committed");
        require(document.model().rigidBodies == original.rigidBodies, "failed transaction changed document");
        saveRejects(invalid);
    }
}

void softBodyValidation() {
    const auto original = physicsModel();
    for (int scenario = 0; scenario < 8; ++scenario) {
        auto invalid = original;
        auto &body = invalid.softBodies[0];
        if (scenario == 0)
            body.material = 1;
        if (scenario == 1)
            body.material = -2;
        if (scenario == 2)
            body.anchors[0].rigidBody = 1;
        if (scenario == 3)
            body.anchors[0].rigidBody = -2;
        if (scenario == 4)
            body.anchors[0].vertex = 1;
        if (scenario == 5)
            body.anchors[0].vertex = -1;
        if (scenario == 6)
            body.pinnedVertices[0] = 1;
        if (scenario == 7)
            body.pinnedVertices[0] = -1;
        mmd::PmxDocument document(original);
        const auto property = document.replaceSoftBody(document.softBodyHandle(0), body);
        require(!property.committed, "invalid soft body property committed");
        sameDiagnostics(mmd::pmx::validate(invalid), property.validation);
        auto transaction = document.transaction();
        require(transaction.setSoftBody(document.softBodyHandle(0), body), "soft body setter failed");
        require(!transaction.commit().committed, "invalid soft body transaction committed");
        require(document.model().softBodies == original.softBodies, "failed transaction changed document");
        saveRejects(invalid);
    }
    auto detached = original;
    detached.softBodies[0].material = -1;
    detached.softBodies[0].anchors[0].rigidBody = -1;
    require(mmd::pmx::validate(detached).valid(), "editor's optional none references became invalid");
}

void jointValidation() {
    auto original = physicsModel();
    original.joints.resize(1);
    original.joints[0].bodyA = 0;
    original.joints[0].bodyB = 0;
    const auto check = [&](const mmd::PmxJoint &joint) {
        auto invalid = original;
        invalid.joints[0] = joint;
        mmd::PmxDocument document(original);
        const auto property = document.replaceJoint(document.jointHandle(0), joint);
        require(!property.committed, "invalid joint property committed");
        sameDiagnostics(mmd::pmx::validate(invalid), property.validation);
        auto transaction = document.transaction();
        require(transaction.setJoint(document.jointHandle(0), joint), "joint setter failed");
        require(!transaction.commit().committed, "invalid joint transaction committed");
        require(document.model().joints == original.joints, "failed joint edit changed document");
        saveRejects(invalid);
    };
    constexpr std::array<mmd::Float3 mmd::PmxJoint::*, 8> fields{
        &mmd::PmxJoint::position,           &mmd::PmxJoint::rotation,        &mmd::PmxJoint::translationMinimum,
        &mmd::PmxJoint::translationMaximum, &mmd::PmxJoint::rotationMinimum, &mmd::PmxJoint::rotationMaximum,
        &mmd::PmxJoint::translationSpring,  &mmd::PmxJoint::rotationSpring};
    for (auto field : fields)
        for (std::size_t component = 0; component < 3; ++component)
            for (float bad : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
                auto joint = original.joints[0];
                (joint.*field)[component] = bad;
                check(joint);
            }
    for (std::uint8_t type : {std::uint8_t{6}, std::uint8_t{255}}) {
        auto joint = original.joints[0];
        joint.type = type;
        check(joint);
    }
    auto disabled = original.joints[0];
    disabled.physicsEnabled = false;
    disabled.position[0] = std::numeric_limits<float>::quiet_NaN();
    check(disabled);
    for (int type = 0; type <= 5; ++type) {
        auto model = original;
        model.joints[0].type = static_cast<std::uint8_t>(type);
        require(mmd::pmx::validate(model).valid(), "valid PMX 2.1 joint type rejected");
        model.metadata.version = 2.0F;
        model.softBodies.clear();
        require(mmd::pmx::validate(model).valid() == (type == 0), "PMX 2.0 joint type contract violated");
        if (type != 0) {
            mmd::PmxDocument document(model);
            require(!document.replaceJoint(document.jointHandle(0), model.joints[0]).committed,
                    "PMX 2.0 joint property accepted 2.1 type");
            require(!document.transaction().commit().committed, "PMX 2.0 joint transaction accepted 2.1 type");
            saveRejects(model);
        }
    }
}

void softBodyValues() {
    const auto original = physicsModel();
    const auto check = [&](const mmd::PmxSoftBody &body) {
        auto invalid = original;
        invalid.softBodies[0] = body;
        mmd::PmxDocument document(original);
        const auto property = document.replaceSoftBody(document.softBodyHandle(0), body);
        require(!property.committed, "invalid soft body value property committed");
        sameDiagnostics(mmd::pmx::validate(invalid), property.validation);
        auto transaction = document.transaction();
        require(transaction.setSoftBody(document.softBodyHandle(0), body), "soft body setter failed");
        require(!transaction.commit().committed, "invalid soft body value transaction committed");
        require(document.model().softBodies == original.softBodies, "failed soft body edit changed document");
        saveRejects(invalid);
    };
    for (int scenario = 0; scenario < 3; ++scenario) {
        auto body = original.softBodies[0];
        if (scenario == 0)
            body.shape = 255;
        if (scenario == 1)
            body.group = 255;
        if (scenario == 2)
            body.aeroModel = -1;
        check(body);
    }
    auto invalidAero = original.softBodies[0];
    invalidAero.aeroModel = 999;
    check(invalidAero);
    for (float bad : {std::numeric_limits<float>::quiet_NaN(), std::numeric_limits<float>::infinity()}) {
        for (auto field : {&mmd::PmxSoftBody::totalMass, &mmd::PmxSoftBody::collisionMargin}) {
            auto body = original.softBodies[0];
            body.*field = bad;
            check(body);
        }
        const auto arrays = [&](auto field) {
            for (std::size_t index = 0; index < (original.softBodies[0].*field).size(); ++index) {
                auto body = original.softBodies[0];
                (body.*field)[index] = bad;
                check(body);
            }
        };
        arrays(&mmd::PmxSoftBody::config);
        arrays(&mmd::PmxSoftBody::cluster);
        arrays(&mmd::PmxSoftBody::materialConfig);
    }
    for (std::uint8_t shape : {std::uint8_t{0}, std::uint8_t{1}})
        for (int aero = 0; aero <= 4; ++aero) {
            auto model = original;
            model.softBodies[0].shape = shape;
            model.softBodies[0].aeroModel = aero;
            model.softBodies[0].group = 15;
            require(mmd::pmx::validate(model).valid(), "valid soft body enum boundary rejected");
        }
}

void poseOverridesMotion() {
    mmd::PmxModel model;
    model.bones.resize(2);
    model.bones[0].name = "posed";
    model.bones[1].name = "motion only";
    mmd::VmdMotion motion;
    mmd::VmdBoneKey key;
    key.name = "posed";
    key.translation = {1, 0, 0};
    key.rotation = {0, 0, 0.5F, std::sqrt(0.75F)};
    motion.bones.push_back(key);
    key.name = "motion only";
    key.translation = {4, 0, 0};
    motion.bones.push_back(key);
    mmd::VpdPose pose;
    mmd::VpdBonePose value;
    value.name = "posed";
    value.translation = {2, 0, 0};
    value.rotation = {0, 0, 0, 1};
    pose.bones.push_back(value);
    mmd::MmdAnimator animator(model);
    animator.setMotion(&motion);
    animator.setPose(&pose);
    const auto posed = animator.evaluate(0);
    require(std::abs(posed.bones[0].translation[0] - 2) < 1e-5F, "VPD translation added to motion");
    require(std::abs(posed.bones[0].rotation[2]) < 1e-5F, "VPD rotation composed with motion");
    require(std::abs(posed.bones[1].translation[0] - 4) < 1e-5F, "unspecified VPD bone lost motion");
    animator.setPose(nullptr);
    const auto restored = animator.evaluate(0);
    require(std::abs(restored.bones[0].translation[0] - 1) < 1e-5F, "clearing VPD did not restore motion");
    require(std::abs(restored.bones[0].rotation[2] - 0.5F) < 1e-5F, "clearing VPD did not restore rotation");
}
} // namespace

int main() {
    try {
        deepBoneEdits();
        propertyDependencyCycle();
        rigidBodyValidation();
        softBodyValidation();
        jointValidation();
        softBodyValues();
        poseOverridesMotion();
        std::puts("document and pose hardening tests passed");
    } catch (const std::exception &error) {
        std::fprintf(stderr, "FAIL: %s\n", error.what());
        return 1;
    }
}
