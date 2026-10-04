#pragma once

#include <btBulletDynamicsCommon.h>

namespace mmd::internal {

// Apply the 2.75-style frame-A lever arm locally rather than patching a
// process-wide Bullet installation. This is equivalent to babylon-mmd's
// constraint-fix.patch for the legacy (useFrameOffset=false) 6DoF path.
class Bullet275SpringConstraint final : public btGeneric6DofSpringConstraint {
  public:
    using btGeneric6DofSpringConstraint::btGeneric6DofSpringConstraint;

    void getInfo2(btConstraintInfo2 *info) override {
        btGeneric6DofSpringConstraint::getInfo2(info);
        if (getUseFrameOffset())
            return;
        const auto arm = getCalculatedTransformA().getOrigin() - getRigidBodyA().getCenterOfMassPosition();
        int row = 0;
        for (int axis = 0; axis < 3; ++axis) {
            if (!getTranslationalLimitMotor()->needApplyForce(axis))
                continue;
            const auto torque = arm.cross(getCalculatedTransformA().getBasis().getColumn(axis));
            for (int component = 0; component < 3; ++component)
                info->m_J1angularAxis[row * info->rowskip + component] = torque[component];
            ++row;
        }
    }
};

} // namespace mmd::internal
