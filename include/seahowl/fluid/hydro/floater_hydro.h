#pragma once

#include "seahowl/commons/component_fluid.h"
#include "seahowl/fluid/hydro/foundation_fluid.h"

#include <memory>

// forward declarations
namespace seahowl {
namespace hydro {
class MooringSystemHydro;
}  // namespace hydro
namespace env {
class FluidModel;
}  // namespace env
}  // namespace seahowl

namespace seahowl {

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

    Vector3d get_force_hydro();

    Vector3d get_torque_hydro();

    Eigen::Matrix<double, 6, 6> get_added_mass_matrix();

  protected:
    /** @brief External force acting on floater. */
    Vector3d force_hydro = {0.0, 0.0, 0.0};

    /** @brief External torque acting on floater. */
    Vector3d torque_hydro = {0.0, 0.0, 0.0};

    /** @brief Added mass matrix of floater. */
    Eigen::Matrix<double, 6, 6> added_mass_matrix = Eigen::Matrix<double, 6, 6>::Zero();
};

}  // namespace hydro
}  // namespace seahowl
