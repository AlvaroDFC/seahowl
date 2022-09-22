#pragma once

#include <seahowl/core/blade.h>
#include <seahowl/core/rotor.h>
#include <seahowl/core/tower.h>
#include <seahowl/servo/controller.h>

#include <vector>

/**@brief Seahowl base namespace */
namespace seahowl {
namespace servo {
class Controller;
}

/**@brief Seahowl core module */
namespace core {

/**@brief The turbine (without support and foundations)


@todo Turbine is par Turbine class part TurbineElasto class (translate , rotate, ...)
*/
class Turbine {
  public:
    std::vector<std::shared_ptr<Blade>> blades;              ///< Blades => To be moved in Rotor
    Rotor rotor;                                             ///< Rotor.  @todo Should be Hub + Blades
    Tower tower;                                             ///< Tower
    std::shared_ptr<seahowl::servo::Controller> controller;  ///< Controller

    double generator_efficiency = 1.0;  ///< Efficiency of generator
    double gearbox_ratio = 1.0;         ///< Gearbox ratio
    double gearbox_efficiency = 1.0;    ///< Efficiency of gearbox

    Turbine();
    ~Turbine();

    void assemble(chrono::ChSystemSMC& system, std::shared_ptr<chrono::fea::ChMesh> mesh);
    void build();
    void init(double time, double dt);
    void prestep(double time, double dt);
    void poststep(double time, double dt);
    void translate(chrono::ChVector<double> translation_vector);
    void rotate(double angle, chrono::ChVector<double> axis);
    void compute_wind_loads(seahowl::aero::WindModel& wind_model, double time);
    double get_generated_power();
};

}  // namespace core
}  // namespace seahowl
