#include <mmd/animation.hpp>

#if LIBMMD_CONSUMER_WITH_PHYSICS
#include <mmd/physics.hpp>
#endif

int main() {
    mmd::PmxModel model;
#if LIBMMD_CONSUMER_WITH_PHYSICS
    mmd::MmdPhysics physics(model);
    static_cast<void>(physics);
#endif
    mmd::MmdAnimator animator(model);
    static_cast<void>(animator);
    return 0;
}
