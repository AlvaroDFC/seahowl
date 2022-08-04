#ifndef ROTOR_H_
#define ROTOR_H_

#include "blade.h"
#include "../elasto/rotor_elasto.h"
#include "../aero/rotor_aero.h"

/**@brief Wind turbine rotor: Hub + blades */
class Rotor {
  public:
    RotorElasto elasto;
    RotorAero aero;
    std::vector<std::shared_ptr<Blade>> blades;

    Rotor();
    ~Rotor() {}

    void update_positions_aero();
    void build(ChSystemSMC& system, std::vector<std::shared_ptr<Blade>> blades);
    void prestep(double time);
    void poststep(double time);
};

#endif  // ROTOR_H_
