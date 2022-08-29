#pragma once

#include <seahowl/core/reference_point.h>

#include <chrono/fea/ChElementBeamTaperedTimoshenko.h>

namespace seahowl {
namespace elasto {

/**@brief Elastodynamic model DOF reference point */
struct BladeReferencePointElasto {
    chrono::ChVector<double> m_coordinates{0.0, 0.0, 0.0};
    chrono::ChVector2<double> m_offset_elastic{0.0, 0.0};
    chrono::ChVector2<double> m_offset_gravity{0.0, 0.0};
    chrono::ChMatrixNM<double, 6, 6> stiffness_matrix;
    chrono::ChMatrixNM<double, 6, 6> mass_matrix;
    double fraction = 0.0;
    double structural_twist = 0.0;
    chrono::fea::DampingCoefficients damping_coefficients;

    BladeReferencePointElasto();
    BladeReferencePointElasto(seahowl::core::BladeReferencePoint point);
    ~BladeReferencePointElasto();

    BladeReferencePointElasto operator*(const double factor) const;
    BladeReferencePointElasto operator+(const BladeReferencePointElasto& other) const;
};

/**@brief Tower (DOF) reference point */
struct TowerReferencePoint {
    chrono::ChVector<double> coordinates;
    double fraction = 1.0;
    double density = 0.0;
    double stiffness_axial = 0.0;
    double stiffness_foreaft = 0.0;
    double stiffness_sideside = 0.0;
    double stiffness_torsion = 0.0;
    chrono::fea::DampingCoefficients damping_coefficients;

    TowerReferencePoint operator*(const double factor) const;
    TowerReferencePoint operator+(const TowerReferencePoint& other) const;
};

}  // namespace elasto
}  // namespace seahowl
