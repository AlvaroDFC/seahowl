#ifndef UTILS_H_
#define UTILS_H_

#include "chrono/physics/ChLoadContainer.h"

using namespace chrono;

template <typename T>
std::vector<T> get_discretized_points(std::vector<double>& discretization_fractions, std::vector<T>& reference_points) {
    if (discretization_fractions.size() == 0) {
        // discretize at centers of reference directly
        return reference_points;
    } else {
        std::vector<T> discretized_points;
        std::vector<double> reference_fractions;
        for (int ii = 0; ii < reference_points.size(); ii++) {
            reference_fractions.push_back(reference_points[ii].fraction);
        }
        // check bounds
        if (reference_fractions[0] != 0.0 || reference_fractions[reference_fractions.size() - 1] != 1.0) {
            throw std::runtime_error("Reference fractions must start with 0 and end with 1 but got " +
                                     std::to_string(reference_fractions[0]) + " and " +
                                     std::to_string(reference_fractions[reference_fractions.size() - 1]) + ".");
        }
        for (int ii = 0; ii < discretization_fractions.size(); ii++) {
            // find position of node
            // interpolate to node position from centers of reference at key fractions
            double fraction = discretization_fractions[ii];
            // check bounds
            if (fraction < 0.0 || fraction > 1.0) {
                throw std::runtime_error("Discretization fraction must be between 0 and 1 but was " +
                                         std::to_string(fraction) + ".");
            }
            int idx = std::upper_bound(reference_fractions.begin(), reference_fractions.end(), fraction) -
                      reference_fractions.begin();
            // decrease index for convenience
            idx -= 1;
            double fraction_lower = reference_fractions[idx];
            double fraction_upper = reference_fractions[idx + 1];
            double fraction_range = fraction_upper - fraction_lower;
            // check if fraction is same as lower or upper bound to avoid division by zero
            if (fraction_lower == fraction) {
                discretized_points.push_back(reference_points[idx]);
            } else if (fraction_upper == fraction) {
                discretized_points.push_back(reference_points[idx + 1]);
            } else {
                // get weighted point
                auto discretized_point =
                    (reference_points[idx] * (1.0 - (fraction - fraction_lower) / fraction_range) +
                     reference_points[idx + 1] * (1.0 - (fraction_upper - fraction) / fraction_range));
                discretized_points.push_back(discretized_point);
            }
        }
        return discretized_points;
    }
}

class ChLoaderWeighted : public ChLoaderUdistributed {
  public:
    std::vector<ChVector<double>> loads;
    std::vector<double> positions;
    int integration_points;

    ChLoaderWeighted(std::shared_ptr<ChLoadableU> mloadable) : ChLoaderUdistributed(mloadable) {
        integration_points = 10;
    };

    void set_positions(std::vector<double> positions) {
        this->positions.clear();
        this->positions.assign(positions.begin(), positions.end());
    }

    void set_loads(std::vector<ChVector<double>> loads) {
        // check that number of loads is the same as number of positions
        if (loads.size() != this->positions.size()) {
            throw std::runtime_error("Number of loads (" + std::to_string(loads.size()) +
                                     ") for element is different from number of positions (" +
                                     std::to_string(this->positions.size()) + ") along element.");
        }
        this->loads.clear();
        this->loads.assign(loads.begin(), loads.end());
    }

    // Compute F=F(u)
    virtual void ComputeF(const double U,              // parametric coordinate along element
                          ChVectorDynamic<>& F,        // resulting loads go here
                          ChVectorDynamic<>* state_x,  // if !=0 update pos
                          ChVectorDynamic<>* state_w   // if !=0 update speed
    ) {
        // initialize load
        auto load = ChVector<double>(0.0, 0.0, 0.0);
        // find between which load positions is U and interpolate
        for (int ii = 0; ii < std::max(0, (int)positions.size() - 1); ii++) {
            if (positions[ii] <= U && U <= positions[ii + 1]) {
                double range = positions[ii + 1] - positions[ii];
                load = (1 - (U - positions[ii]) / range) * loads[ii] +
                       (1 - (positions[ii + 1] - U) / range) * loads[ii + 1];
                break;
            }
        }
        // apply load
        // forces
        F(0) = load.x();
        F(1) = load.y();
        F(2) = load.z();
        // moments
        F(3) = 0.0;
        F(4) = 0.0;
        F(5) = 0.0;
    }

    virtual int GetIntegrationPointsU() { return integration_points; }
};

#endif  // UTILS_H_
