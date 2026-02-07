#pragma once

#include "seahowl/env/env_model.h"

#include <vector>

namespace seahowl {
namespace fluid {

class ComponentFluid {
  public:
    /** @brief Discretization fractions (normalized abscissa) in the range [0, 1]. */
    std::vector<double> discretization_fractions{};

    virtual ~ComponentFluid() = default;
    virtual void build() = 0;
    virtual void compute_env_loads(const env::EnvModel& env_model, double time) = 0;
    virtual void setup_environment(const env::EnvModel& env_model){};
    virtual void initialize(double time, double dt){};
};

}  // namespace fluid
}  // namespace seahowl
