#ifndef ROTOR_AERO_H_
#define ROTOR_AERO_H_

#include "blade_aero.h"
#include "wind_models.h"

#include "chrono/core/ChVector.h"

using namespace chrono;

class RotorAero {
  public:
    std::vector<std::shared_ptr<BladeAero>> blades;
    ChVector<double> hub_position;
    ChQuaternion<double> hub_rotation;
    double radius;

    RotorAero(){};
    ~RotorAero(){};

    void build(std::vector<std::shared_ptr<BladeAero>> blades);
    void compute_chords_solidity();
    void compute_wind_loads_bemt(WindModel& wind_model, double time);
};

#endif  // ROTOR_AERO_H_
