#include <mmd/animation.hpp>

int main() {
    mmd::PmxModel model;
    mmd::MmdAnimator animator(model);
    static_cast<void>(animator);
    return 0;
}
