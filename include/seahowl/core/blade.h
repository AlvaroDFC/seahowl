#pragma once

#include <memory>
#include <vector>

namespace seahowl {
namespace core {
struct DiscretizationPoint;
struct BladeReferencePoint;
}
namespace elasto {
class BladeElasto;
}
namespace aero {
class BladeAero;
}
}  // namespace seahowl



namespace chrono {
class ChSystemSMC;
namespace fea {
class ChMesh;
}
}  // namespace chrono

namespace seahowl {
namespace core {

/**@brief Wind turbine blade base class

Pattern <mediator> for elasto and aero
*/
class Blade {
  public:
    std::shared_ptr<seahowl::elasto::BladeElasto> m_elasto;  ///< Elastodynamic element mesh
    std::shared_ptr<seahowl::aero::BladeAero> m_aero;        ///< Aerodynamic element mesh
    std::vector<BladeReferencePoint>
        m_reference_points;  ///<@todo  Refactor: Only used for construction to pass to elasto and aero. Use a Builder
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