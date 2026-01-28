#pragma once

#include "seahowl/fluid/hydro/foundation_fluid.h"

#include <memory>

// forward declarations
namespace seahowl {
namespace fluid {
namespace hydro {
class MooringSystemHydro;
}  // namespace hydro
}  // namespace fluid
namespace hydro = fluid::hydro;
namespace env {
class FluidModel;
}  // namespace env
}  // namespace seahowl

namespace seahowl {
namespace fluid {

/**@brief Hydrodynamic module */
namespace hydro {

/**
 * @brief Floater of wind turbine as an hydrodynamic component.
 */
class FloaterHydro : public FoundationFluid {
  public:
    /** @brief Mooring system of the floater. */
    std::shared_ptr<MooringSystemHydro> mooring_system;

    /**
     * @brief Constructor.
     */
    FloaterHydro();

    void build() override;

    void compute_env_loads(const env::EnvModel& fluid_model, double time) override;
};

}  // namespace hydro
}  // namespace fluid
}  // namespace seahowl
