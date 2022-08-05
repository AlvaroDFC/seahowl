#pragma once

#include <seahowl/aero/reference_point_aero.h>
#include <seahowl/aero/wind_models.h>
#include <seahowl/utils.h>

#include "chrono/core/ChVector.h"

using namespace chrono; /// TODO remove from header

/**@brief Blade aerodynamic element */
struct BladeElementAero {
    BladeReferencePointAero properties;
    double length = 0.0;
    double pitch = 0.0;
    double swept_annulus = 1.0;
    double chord_solidity = 1.0;
    double induction_factor_axial = 0.0;
    double induction_factor_tangential = 0.0;
    double radius = 0.0;
    double distance_from_hub = 99999.9;  // far by default for no effect
    double distance_from_tip = 99999.9;  // far by default for no effect

    BladeElementAero(BladeReferencePointAero& point1, BladeReferencePointAero& point2);
    ~BladeElementAero() {}

    // ChVector2<double> get_induced_velocity_element(ChVector2<double>& local_velocity0);
    ChVector2<double> get_induced_velocity_rotor(ChVector2<double>& local_velocity_rotor0,
                                                 size_t nblades,
                                                 bool tip_loss = true,
                                                 bool hub_loss = true);
};

/**@brief Aerodynamic model for blade */
class BladeAero {
  public:
    std::vector<double> discretization_fractions;
    std::vector<BladeReferencePointAero> reference_points;
    std::vector<BladeReferencePointAero> discretized_points;
    std::vector<BladeElementAero> elements;
    std::vector<ChVector<double>> loads;

    BladeAero() {}
    ~BladeAero() {}

    void build();
    void compute_distances_from_tip();
    void compute_distances_from_hub(ChVector<double> hub_apex_position, double hub_radius);
    void compute_radii(ChVector<double> hub_apex_position);
    // void compute_wind_loads_bemt(WindModel& wind_model, double time);
};

