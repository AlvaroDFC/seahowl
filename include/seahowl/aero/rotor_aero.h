#pragma once

#include <seahowl/aero/blade_aero.h>
#include <seahowl/aero/wind_models.h>

#include "chrono/core/ChVector.h"

using namespace chrono; /// TODO remove from header

/**@brief Aerodynamic model for rotor */
class RotorAero {
  public:
    std::vector<std::shared_ptr<BladeAero>> blades;
    ChVector<double> hub_position;
    ChQuaternion<double> hub_rotation;
    double radius;
    double hub_radius;

    RotorAero(){};
    ~RotorAero(){};

    void build(std::vector<std::shared_ptr<BladeAero>> blades);
    void compute_chords_solidity();
    void compute_distances_from_hub();
    void compute_distances_from_tip();
    void compute_radii();
    void compute_wind_loads_bemt(WindModel& wind_model, double time);
};
