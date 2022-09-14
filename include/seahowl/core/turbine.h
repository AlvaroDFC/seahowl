#pragma once

#include <seahowl/core/blade.h>
#include <seahowl/core/rotor.h>
#include <seahowl/core/tower.h>
#include <seahowl/servo/controller.h>

#include <vector>

/**@brief Seahowl base namespace */
namespace seahowl {

/**@brief Seahowl core module */
namespace core {

/**@brief The turbine (without support and foundations)


@todo Turbine is par Turbine class part TurbineElasto class (translate , rotate, ...)
*/
class Turbine {
  public:
    std::vector<std::shared_ptr<Blade>> blades;  ///< Blades => To be moved in Rotor
    Rotor rotor;                                 ///< Rotor.  @todo Should be Hub + Blades
    Tower tower;                                 ///< Tower

    Turbine();
    ~Turbine();

    void build(chrono::ChSystemSMC& system, std::shared_ptr<chrono::fea::ChMesh> mesh);
    void prestep(double time);
    void poststep(double time);
    void translate(chrono::ChVector<double> translation_vector);
    void rotate(double angle, chrono::ChVector<double> axis);
    void compute_wind_loads(seahowl::aero::WindModel& wind_model, double time);
};

}  // namespace core
}  // namespace seahowl
