#pragma once

#include <seahowl/core/blade.h>
#include <seahowl/elasto/rotor_elasto.h>
#include <seahowl/aero/rotor_aero.h>


/**@brief Wind turbine rotor: Hub + blades */
class Rotor {
  public:
    seahowl::elasto::RotorElasto elasto;
    RotorAero aero;
    std::vector<std::shared_ptr<seahowl::core::Blade>> blades;

    Rotor();
    ~Rotor() {}

    void update_positions_aero();
    void build(chrono::ChSystemSMC& system, std::vector<std::shared_ptr<seahowl::core::Blade>> blades);
    void prestep(double time);
    void poststep(double time);
};

