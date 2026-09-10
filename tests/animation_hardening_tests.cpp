#include <mmd/animation.hpp>

#include <array>
#include <cassert>
#include <limits>

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

    model.morphs[0].offsets = {{.index = 0, .scalar = 1.0F}};
    const auto bounded = mmd::expandMorphWeights(model, rootWeights, {.maxSteps = 8});
    assert(bounded.budgetExceeded);
    assert(bounded.cycleDetected);

    const std::array<float, 3> invalidWeights{std::numeric_limits<float>::quiet_NaN(), 0.0F, 0.0F};
    const auto nonFinite = mmd::expandMorphWeights(model, invalidWeights);
    assert(nonFinite.effectiveWeights[1] == 0.0F);
    return 0;
}
