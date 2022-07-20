#ifndef TURBINE_ELASTO_H_
#define TURBINE_ELASTO_H_

#include "blade_core.h"
#include "../elasto/rotor_elasto.h"
#include "../elasto/tower_elasto.h"

class Turbine {
  public:
    std::vector<std::shared_ptr<Blade>> blades;
    RotorElasto rotor;
    TowerElasto tower;

    Turbine();

    ~Turbine() {}

    void build(ChSystemSMC& system, std::shared_ptr<ChMesh> mesh);
    void prestep(double time, WindModel& wind_model);
    void translate(ChVector<double> translation_vector);
    void rotate(double angle, ChVector<double> axis);
};

#endif  // TURBINE_ELASTO_H_
