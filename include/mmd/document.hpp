#pragma once

#include <mmd/pmx.hpp>

#include <utility>

namespace mmd {

// Editor-facing ownership boundary. Structural mutation APIs will be added in
// a later release once every PMX cross-reference can be updated atomically.
class PmxDocument {
  public:
    PmxDocument() = default;
    explicit PmxDocument(PmxModel model) : model_(std::move(model)) {}

    [[nodiscard]] PmxModel& model() noexcept { return model_; }
    [[nodiscard]] const PmxModel& model() const noexcept { return model_; }
    [[nodiscard]] ValidationResult validate() const { return pmx::validate(model_); }

  private:
    PmxModel model_;
};

} // namespace mmd
