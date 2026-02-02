#pragma once

// SEAHOWL headers
#include "seahowl/fluid/aero/tower_aero.h"
#include "seahowl/fluid/hydro/foundation_fluid.h"

namespace seahowl {
/**@brief Fluid model module */
namespace fluid {

/**@brief HydroDynamic module */
namespace hydro {

/**
 * @brief Monopile of wind turbine as an hydrodynamic component.
 */
class MonopileHydro : public seahowl::aero::TowerAero, public virtual FoundationFluid {};

}  // namespace hydro
}  // namespace fluid
namespace hydro = fluid::hydro;
}  // namespace seahowl
