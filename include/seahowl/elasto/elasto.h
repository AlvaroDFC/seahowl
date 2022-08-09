#pragma once

#include <vector>
#include <memory>

#include <chrono/core/ChVector.h>

namespace chrono {
namespace fea {
class ChNodeFEAxyzrot;
class ChElementBeamTaperedTimoshenko;
}  // namespace fea
}  // namespace chrono

#include <seahowl/elasto/reference_point_elasto.h>
#include <seahowl/elasto/utils_elasto.h>


namespace seahowl {
///@brief Elastodynamic model module
namespace elasto {

struct BladeReferencePointElasto;

/**@biref Component <<interface>>


*/
class ElastoComponent {
  public:
    ///@{
    virtual void rotate(double angle, chrono::ChVector<double> axis) const = 0; ///< Rotate the system
    virtual void translate(chrono::ChVector<double> translation_vector) const = 0; ///< Translate the system
    virtual double get_mass() const = 0; ///< Get total mass
    std::vector<double> discretization_fractions; ///< Fraction ?
    std::vector<BladeReferencePointElasto> reference_points; ///< Original points
    std::vector<BladeReferencePointElasto> discretized_points; ///< Discretized point
    ///@}
};

/**@brief Finite Element Elastodynamic component */
class ElastoFEAComponent : public ElastoComponent {
  public:
    std::vector<std::shared_ptr<chrono::fea::ChNodeFEAxyzrot>> nodes; ///< Finite Element Nodes
    std::vector<std::shared_ptr<chrono::fea::ChElementBeamTaperedTimoshenko>> elements; ///< Beam elements

    ///@{
    virtual void rotate(double angle, chrono::ChVector<double> axis) const override;     ///< Rotate the system
    virtual void translate(chrono::ChVector<double> translation_vector) const override;  ///< Translate the system
    virtual double get_mass() const override;                                            ///< Get total mass
    virtual void set_damping_coefficients(double axial, double edge, double flap, double torsion) = 0;
    ///@}
};

}  // namespace elasto
}  // namespace seahowl