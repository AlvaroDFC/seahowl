#ifndef TURBINE_H_
#define TURBINE_H_

#include "blade_core.h"
#include "../core/rotor.h"
#include "../elasto/tower_elasto.h"

class Turbine {
  public:
    std::vector<std::shared_ptr<Blade>> blades;
    Rotor rotor;
    Tower tower;

    Turbine();

    ~Turbine() {}

    void build(ChSystemSMC& system, std::shared_ptr<ChMesh> mesh);
    void prestep(double time, WindModel& wind_model);
    void translate(ChVector<double> translation_vector);
    void rotate(double angle, ChVector<double> axis);
};

#endif  // TURBINE_H_
