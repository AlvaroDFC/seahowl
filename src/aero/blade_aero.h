#ifndef BLADE_AERO_H_
#define BLADE_AERO_H_

#include "reference_point_aero.h"

#include "chrono/core/ChVector.h"

class BladeAero {
  public:
    std::vector<double> discretization_fractions;
    std::vector<BladeReferencePointAero> reference_points;
    std::vector<BladeReferencePointAero> discretized_points;
    std::vector<ChVector<double>> loads;

    BladeAero() {}

    void build() { discretized_points = get_discretized_points(discretization_fractions, reference_points); }

    void compute_loads() {
        // TODO: actually compute loads
        for (int ii = 0; ii < loads.size(); ii++) {
            loads[ii] = ChVector<double>(0.0, 0.0, 0.0);
        }
    }
};

#endif  // BLADE_AERO_H_
