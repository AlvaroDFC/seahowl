#pragma once

#include <chrono/fea/ChElementBeamTaperedTimoshenko.h>
#include <seahowl/aero/airfoil.h>

#include <vector>

//using namespace chrono;
//using namespace chrono::fea;

/**@brief Reference point (mesh) for Blade */
struct BladeReferencePoint {
    chrono::ChVector<double> m_coordinates{0.0, 0.0, 0.0};
    chrono::ChVector2<double> m_offset_elastic{0.0, 0.0};
    chrono::ChVector2<double> m_offset_gravity{0.0, 0.0};
    chrono::ChMatrixNM<double, 6, 6> stiffness_matrix;
    chrono::ChMatrixNM<double, 6, 6> mass_matrix;
    double fraction = 0.0;
    double structural_twist = 0.0;
    double chord = 0.0;
    chrono::fea::DampingCoefficients damping_coefficients; ///< Damping coefficients 
    std::vector<seahowl::aero::AirfoilProperties> airfoil_properties; ///< Airfoil properties for each elemnts

    BladeReferencePoint() {
        stiffness_matrix.setZero();
        mass_matrix.setZero();
        damping_coefficients.bx = 0.03;
        damping_coefficients.by = 0.03;
        damping_coefficients.bz = 0.03;
        damping_coefficients.bt = 0.06;
        damping_coefficients.alpha = 0.0;
    }

    ~BladeReferencePoint() {}

    BladeReferencePoint operator*(const double factor) const {
        BladeReferencePoint new_point = *this;
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
        for (int ii = 0; ii < airfoil_properties.size(); ii++) {
            new_point.airfoil_properties[ii] = airfoil_properties[ii] * factor;
        }
        return new_point;
    };
    BladeReferencePoint operator+(const BladeReferencePoint& other) const {
        BladeReferencePoint new_point = *this;
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
        for (int ii = 0; ii < airfoil_properties.size(); ii++) {
            if (airfoil_properties[ii].reynolds_number != other.airfoil_properties[ii].reynolds_number) {
                throw std::runtime_error("Trying to add airfoil properties with different Reynolds number.");
            }
            new_point.airfoil_properties[ii] = airfoil_properties[ii] + other.airfoil_properties[ii];
        }
        return new_point;
    };
};

