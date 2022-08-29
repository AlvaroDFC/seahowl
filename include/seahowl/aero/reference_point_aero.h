#pragma once

#include <seahowl/core/reference_point.h>

namespace seahowl {
namespace aero {

/**@brief Reference aerodynamic (DOF) point for blade */
struct BladeReferencePointAero {
    double fraction;                                    ///< Fraction (normalized abscissa)
    chrono::ChVector<double> m_coordinates;             ///< Point coordinates
    chrono::ChQuaternion<double> rotation;              ///< Rotation
    chrono::ChVector<double> velocity;                  ///< Velocity
    double chord;                                       ///< Chord length
    double structural_twist;                            ///< Twist
    std::vector<AirfoilProperties> airfoil_properties;  ///< Airfoil properties for each elements

    BladeReferencePointAero();
    BladeReferencePointAero(seahowl::core::BladeReferencePoint& point);
    ~BladeReferencePointAero();

    BladeReferencePointAero operator*(const double factor) const;
    BladeReferencePointAero operator+(const BladeReferencePointAero& other) const;
};

}  // namespace aero
}  // namespace seahowl
