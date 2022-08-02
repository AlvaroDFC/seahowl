#ifndef TURBINE_H_
#define TURBINE_H_

#include "blade.h"
#include "rotor.h"

#include "../elasto/tower_elasto.h"

class Turbine {
  public:
    std::vector<std::shared_ptr<Blade>> blades;
    Rotor rotor;
    TowerElasto tower;

    Turbine();

    ~Turbine() {}

    void build(ChSystemSMC& system, std::shared_ptr<ChMesh> mesh);
    void prestep(double time);
    void poststep(double time);
    void translate(ChVector<double> translation_vector);
    void rotate(double angle, ChVector<double> axis);
    void compute_wind_loads(WindModel& wind_model, double time);
};

#endif  // TURBINE_H_
