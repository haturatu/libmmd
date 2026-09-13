#pragma once

#include <mmd/pmx.hpp>

#include <algorithm>
#include <cmath>

namespace mmd::internal {

template <std::size_t N> bool finiteMorphComponents(const std::array<float, N> &values) noexcept {
    return std::all_of(values.begin(), values.end(), [](const float value) { return std::isfinite(value); });
}

// Morph offsets are retained on invalid documents so editors can repair them.
// Keep the same type-specific numeric contract in validation and animation so
// malformed values never reach the preview evaluator.
inline bool finiteMorphOffset(std::uint8_t type, const PmxMorphOffset &offset) noexcept {
    switch (type) {
    case 0:
    case 9:
        return std::isfinite(offset.scalar);
    case 1:
        return finiteMorphComponents(offset.vector3);
    case 2:
        return finiteMorphComponents(offset.vector3) && finiteMorphComponents(offset.vector4);
    case 3:
    case 4:
    case 5:
    case 6:
    case 7:
        return finiteMorphComponents(offset.vector4);
    case 8:
        return std::all_of(offset.materialVectors.begin(), offset.materialVectors.begin() + 7,
                           [](const Float4 &value) { return finiteMorphComponents(value); });
    case 10:
        return finiteMorphComponents(offset.vector3) && finiteMorphComponents(offset.tertiaryVector3);
    default:
        return false;
    }
}

inline bool validMorphOffsetOperation(std::uint8_t type, const PmxMorphOffset &offset) noexcept {
    return type != 8U || offset.operation <= 1U;
}

inline bool validMorphOffset(std::uint8_t type, const PmxMorphOffset &offset) noexcept {
    return finiteMorphOffset(type, offset) && validMorphOffsetOperation(type, offset);
}

} // namespace mmd::internal
