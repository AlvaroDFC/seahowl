#include "seahowl/aero/tower_aero.h"

using seahowl::aero::TowerElementAero;
using seahowl::aero::TowerAero;

TowerElementAero::TowerElementAero(TowerReferencePointAero& point1, TowerReferencePointAero& point2) {
    properties = (point1 + point2) * 0.5;
    length = (point1.coordinates - point2.coordinates).Length();
}

TowerElementAero::~TowerElementAero() {}

TowerAero::TowerAero() {}

TowerAero::~TowerAero() {}

void TowerAero::build() {
    // check that enough reference points were defined to create elements (at least 2)
    if (reference_points.size() < 2) {
        throw std::runtime_error("Not enough aero reference points defined for tower.");
    }

    // check that discretization_fractions was defined, otherwise take reference point fractions
    if (discretization_fractions.size() == 0) {
        for (int ii = 0; ii < reference_points.size(); ii++) {
            discretization_fractions.push_back(reference_points[ii].fraction);
        }
    }

    // build
    discretized_points = seahowl::core::get_discretized_points(discretization_fractions, reference_points);
    for (int ii = 0; ii < discretized_points.size() - 1; ii++) {
        // make element
        auto element = TowerElementAero(discretized_points[ii], discretized_points[ii + 1]);
        elements.push_back(element);
        // push empty load
        loads.push_back(chrono::ChVector<double>(0.0, 0.0, 0.0));
    }
}
