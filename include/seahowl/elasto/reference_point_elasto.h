#pragma once

#include <seahowl/core/reference_point.h>

#include <chrono/fea/ChElementBeamTaperedTimoshenko.h>

namespace seahowl {
namespace elasto {

/**@brief Elastodynamic model DOF reference point */
struct BladeReferencePointElasto {
    chrono::ChVector<double> m_coordinates{0.0, 0.0, 0.0};  ///< Coordinates of reference point
    chrono::ChVector2<double> m_offset_elastic{0.0, 0.0};   ///< Offset of center of elasticity
    chrono::ChVector2<double> m_offset_gravity{0.0, 0.0};   ///< Offset of center of gravity
    chrono::ChMatrixNM<double, 6, 6> stiffness_matrix;      ///< Stiffness matrix
    chrono::ChMatrixNM<double, 6, 6> mass_matrix;           ///< Mass matrix
    double fraction = 0.0;                                  ///< Fraction (normalized abscissa along blade)
    double structural_twist = 0.0;                          ///< Twist angle (radians)
    chrono::fea::DampingCoefficients damping_coefficients;  ///< Damping coefficients

    BladeReferencePointElasto();
    BladeReferencePointElasto(seahowl::core::BladeReferencePoint point);
    ~BladeReferencePointElasto();

    BladeReferencePointElasto operator*(const double factor) const;
    BladeReferencePointElasto operator+(const BladeReferencePointElasto& other) const;
};

/**@brief Tower (DOF) reference point */
struct TowerReferencePoint {
    chrono::ChVector<double> coordinates;                   ///< Coordinates of reference point
    double fraction = 0.0;                                  ///< Fraction (normalized abscissa along tower)
    double density = 0.0;                                   ///< Density
    double stiffness_axial = 0.0;                           ///< Axial stiffness
    double stiffness_foreaft = 0.0;                         ///< Fore-aft stiffness
    double stiffness_sideside = 0.0;                        ///< Side-side stiffness
    double stiffness_torsion = 0.0;                         ///< Torsional stiffness
    chrono::fea::DampingCoefficients damping_coefficients;  ///< Damping coefficients

    TowerReferencePoint operator*(const double factor) const;
    TowerReferencePoint operator+(const TowerReferencePoint& other) const;
};

}  // namespace elasto
}  // namespace seahowl
