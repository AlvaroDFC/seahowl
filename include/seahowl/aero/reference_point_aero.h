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

    BladeReferencePointAero() {}

    BladeReferencePointAero(seahowl::core::BladeReferencePoint& point) {
        fraction = point.fraction;
        m_coordinates = point.m_coordinates;
        velocity = chrono::ChVector<double>(0.0, 0.0, 0.0);
        chord = point.chord;
        structural_twist = point.structural_twist;
        airfoil_properties = point.airfoil_properties;
    }

    BladeReferencePointAero operator*(const double factor) const {
        BladeReferencePointAero new_point = *this;
        new_point.fraction *= factor;
        new_point.m_coordinates *= factor;
        new_point.velocity *= factor;
        new_point.chord *= factor;
        new_point.structural_twist *= factor;
        for (int ii = 0; ii < airfoil_properties.size(); ii++) {
            new_point.airfoil_properties[ii] = airfoil_properties[ii] * factor;
        }
        return new_point;
    };
    BladeReferencePointAero operator+(const BladeReferencePointAero& other) const {
        BladeReferencePointAero new_point = *this;
        new_point.fraction += other.fraction;
        new_point.m_coordinates += other.m_coordinates;
        new_point.velocity += other.velocity;
        new_point.chord += other.chord;
        new_point.structural_twist += other.structural_twist;
        for (int ii = 0; ii < airfoil_properties.size(); ii++) {
            if (airfoil_properties[ii].reynolds_number != other.airfoil_properties[ii].reynolds_number) {
                throw std::runtime_error("Trying to add airfoil properties with different Reynolds number.");
            }
            new_point.airfoil_properties[ii] = airfoil_properties[ii] + other.airfoil_properties[ii];
        }
        return new_point;
    };
};

}  // namespace aero
}  // namespace seahowl
