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
    return 0;
}
