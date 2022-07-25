#ifndef BLADE_AERO_H_
#define BLADE_AERO_H_

#include "reference_point_aero.h"
#include "wind_models.h"
#include "../utils.h"

#include "chrono/core/ChVector.h"

using namespace chrono;

struct BladeElementAero {
    BladeReferencePointAero properties;
    double length;
    double pitch = 0.0;
    double swept_annulus = 1.0;
    double chord_solidity = 1.0;
    double induction_factor_axial = 0.0;
    double induction_factor_tangential = 0.0;

    BladeElementAero(BladeReferencePointAero& point1, BladeReferencePointAero& point2);
    ~BladeElementAero() {}

    ChVector2<double> get_induced_velocity(ChVector2<double>& local_velocity0);
    ChVector2<double> get_induced_velocity_rotor(ChVector2<double>& local_velocity_rotor0);
};

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
    void compute_wind_loads_bemt(WindModel& wind_model, double time);
};

#endif  // BLADE_AERO_H_
