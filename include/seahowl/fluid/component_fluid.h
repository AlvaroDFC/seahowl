#pragma once

#include "seahowl/env/env_model.h"

#include <vector>

namespace seahowl {
/** @brief Fluid dynamics module (aerodynamics and hydrodynamics). */
namespace fluid {

class ComponentFluid {
  public:
    /** @brief Discretization fractions (normalized abscissa) in the range [0, 1]. */
    std::vector<double> discretization_fractions{};

    virtual ~ComponentFluid() = default;

    /**
     * @brief Builds the fluid component.
     */
    virtual void build() = 0;

    /**
     * @brief Computes environmental loads on fluid component.
     *
     * @param[in] env_model Environmental model providing fluid conditions.
     * @param[in] time Time of simulation [s]
     */
    virtual void compute_env_loads(const env::EnvModel& env_model, double time) = 0;

    /**
     * @brief Sets up the environment for the fluid component.
     *
     * @param[in] env_model Environmental model to set up.
     */
    virtual void setup_environment(const env::EnvModel& env_model){};

    /**
     * @brief Initializes the fluid component.
     *
     * @param[in] time Time of the simulation (usually 0 at init) [s]
     * @param[in] dt Time step length [s]
     */
    virtual void initialize(double time, double dt){};
};

}  // namespace fluid
}  // namespace seahowl
