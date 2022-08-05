#pragma once

#include <seahowl/core/blade.h>
#include <seahowl/elasto/rotor_elasto.h>
#include <seahowl/aero/rotor_aero.h>

namespace seahowl {
namespace core {

/**@brief Wind turbine rotor: Hub + blades 

@todo Rotor should be composed of blades  + hub
*/
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

}  // namespace core
}  // namespace seahowl
