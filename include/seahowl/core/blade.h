#pragma once

//#include <seahowl/elasto/blade_elasto.h>
//#include <seahowl/aero/blade_aero.h>
//#include <seahowl/utils.h>

#include <memory>
#include <vector>

class BladeElasto;
class BladeAero;
struct BladeReferencePoint;
struct DiscretizationPoint;

namespace chrono {
class ChSystemSMC;
    namespace fea {
class ChMesh;
}
}


namespace seahowl {
namespace core {

/**@brief Wind turbine blade base class

Pattern <mediator> for elasto and aero
*/
class Blade {
  public:
    std::shared_ptr<BladeElasto> m_elasto;
    std::shared_ptr<BladeAero> m_aero;
    std::vector<BladeReferencePoint> m_reference_points;  ///TODO  Refactor: Only used for construction to pass to elasto and aero. Use a Builder
    std::vector<DiscretizationPoint> m_mapping_aero2elasto;
    std::vector<DiscretizationPoint> m_mapping_elasto2aero;

    Blade();
    ~Blade() {}

    void build(chrono::ChSystemSMC& system, std::shared_ptr<chrono::fea::ChMesh> mesh);
    void set_discretization_elasto(std::vector<double> fractions);
    void set_discretization_aero(std::vector<double> fractions);
    void compute_mapping_aero2elasto();
    void compute_mapping_elasto2aero();
    void prestep(double time);
    void poststep(double time);

    /**@brief Compute aerodynamic loadings */
    void update_positions_aero();

    /**@brief Compute elastodynamic loadings */
    void update_loads_elasto();
};

}  // namespace core
}  // namespace seahowl