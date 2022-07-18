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
    double pitch_beta;

    BladeElementAero(BladeReferencePointAero& point1, BladeReferencePointAero& point2) {
        properties = (point1 + point2) * 0.5;
        length = (point1.coordinates - point2.coordinates).Length();
    }

    ~BladeElementAero() {}
};

class BladeAero {
  public:
    std::vector<double> discretization_fractions;
    std::vector<BladeReferencePointAero> reference_points;
    std::vector<BladeReferencePointAero> discretized_points;
    std::vector<BladeElementAero> elements;
    std::vector<ChVector<double>> loads;

    BladeAero() {}

    void build();
    void compute_loads(double time, WindModel& wind_model);
};

#endif  // BLADE_AERO_H_
