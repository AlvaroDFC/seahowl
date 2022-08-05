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

    BladeReferencePointElasto() {
        stiffness_matrix.setZero();
        mass_matrix.setZero();
        damping_coefficients.bx = 0.03;
        damping_coefficients.by = 0.03;
        damping_coefficients.bz = 0.03;
        damping_coefficients.bt = 0.06;
        damping_coefficients.alpha = 0.0;
    }

    BladeReferencePointElasto(seahowl::core::BladeReferencePoint point) {
        m_coordinates = point.m_coordinates;
        m_offset_elastic = point.m_offset_elastic;
        m_offset_gravity = point.m_offset_gravity;
        stiffness_matrix = point.stiffness_matrix;
        mass_matrix = point.mass_matrix;
        fraction = point.fraction;
        structural_twist = point.structural_twist;
        damping_coefficients = point.damping_coefficients;
    }

    ~BladeReferencePointElasto() {}

    BladeReferencePointElasto operator*(const double factor) const {
        BladeReferencePointElasto new_point = *this;
        new_point.m_coordinates *= factor;
        new_point.m_offset_elastic *= factor;
        new_point.m_offset_gravity *= factor;
        new_point.fraction *= factor;
        new_point.structural_twist *= factor;
        new_point.mass_matrix *= factor;
        new_point.stiffness_matrix *= factor;
        new_point.damping_coefficients.bx *= factor;
        new_point.damping_coefficients.by *= factor;
        new_point.damping_coefficients.bz *= factor;
        new_point.damping_coefficients.bt *= factor;
        new_point.damping_coefficients.alpha *= factor;
        // TODO include airfoil properties
        return new_point;
    };
    BladeReferencePointElasto operator+(const BladeReferencePointElasto& other) const {
        BladeReferencePointElasto new_point = *this;
        new_point.m_coordinates += other.m_coordinates;
        new_point.m_offset_elastic += other.m_offset_elastic;
        new_point.m_offset_gravity += other.m_offset_gravity;
        new_point.fraction += other.fraction;
        new_point.structural_twist += other.structural_twist;
        new_point.stiffness_matrix += other.stiffness_matrix;
        new_point.mass_matrix += other.mass_matrix;
        new_point.damping_coefficients.bx += other.damping_coefficients.bx;
        new_point.damping_coefficients.by += other.damping_coefficients.by;
        new_point.damping_coefficients.bz += other.damping_coefficients.bz;
        new_point.damping_coefficients.bt += other.damping_coefficients.bt;
        new_point.damping_coefficients.alpha += other.damping_coefficients.alpha;
        // TODO include airfoil properties
        return new_point;
    };
};

}  // namespace elasto
}  // namespace seahowl