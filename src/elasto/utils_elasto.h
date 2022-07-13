#ifndef UTILS_ELASTO_H_
#define UTILS_ELASTO_H_

#include "chrono/physics/ChLoadContainer.h"

using namespace chrono;

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

#endif  // UTILS_ELASTO_H_
