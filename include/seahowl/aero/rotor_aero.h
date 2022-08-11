#pragma once

#include <seahowl/aero/blade_aero.h>
#include <seahowl/aero/wind_models.h>

#include <chrono/core/ChVector.h>

namespace seahowl {
namespace aero {

/**@brief Aerodynamic model for rotor */
class RotorAero {
  public:
    std::vector<std::shared_ptr<seahowl::aero::BladeAero>> blades; ///< Blades 
    chrono::ChVector<double> hub_position{0.0, 0.0, 0.0};
    chrono::ChQuaternion<double> hub_rotation;
    double radius = 0.0; ///< Total radius of the rotor (hub + blade)
    double hub_radius = 0.0; ///< Hub Radius @todo include a class Hub

    RotorAero(){};
    ~RotorAero(){};

    void build(std::vector<std::shared_ptr<seahowl::aero::BladeAero>> blades);
    void compute_chords_solidity();
    void compute_distances_from_hub();
    void compute_distances_from_tip();
    void compute_radii();
    void compute_wind_loads_bemt(WindModel& wind_model, double time);
};

}  // namespace aero
}  // namespace seahowl