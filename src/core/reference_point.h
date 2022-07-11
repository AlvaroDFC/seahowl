#ifndef REFERENCE_POINT_H_
#define REFERENCE_POINT_H_

#include "chrono/fea/ChElementBeamTaperedTimoshenko.h"

using namespace chrono;
using namespace chrono::fea;

struct AirfoilProperties {
    double reynolds_number = 0.0;
    std::vector<double> alpha;
    std::vector<double> lift_coeff;
    std::vector<double> drag_coeff;
    std::vector<double> am_coeff;

    AirfoilProperties() {}

    ~AirfoilProperties() {}

    AirfoilProperties operator*(const double factor) const {
        AirfoilProperties new_point = *this;
        /* new_point.reynolds_number *= factor; */
        for (int ii = 0; ii < alpha.size(); ii++) {
            new_point.alpha[ii] *= factor;
            new_point.lift_coeff[ii] *= factor;
            new_point.drag_coeff[ii] *= factor;
            new_point.am_coeff[ii] *= factor;
        }
        return new_point;
    };
    AirfoilProperties operator+(const AirfoilProperties& other) const {
        AirfoilProperties new_point = *this;
        /* new_point.reynolds_number += other.reynolds_number; */
        for (int ii = 0; ii < alpha.size(); ii++) {
            new_point.alpha[ii] += other.alpha[ii];
            new_point.lift_coeff[ii] += other.lift_coeff[ii];
            new_point.drag_coeff[ii] *= other.drag_coeff[ii];
            new_point.am_coeff[ii] *= other.am_coeff[ii];
        }
        return new_point;
    };
};

struct BladeReferencePoint {
    ChVector<double> coordinates = ChVector<double>(0.0, 0.0, 0.0);
    ChVector2<double> offset_elastic = ChVector2<double>(0.0, 0.0);
    ChVector2<double> offset_gravity = ChVector2<double>(0.0, 0.0);
    ChMatrixNM<double, 6, 6> stiffness_matrix;
    ChMatrixNM<double, 6, 6> mass_matrix;
    double fraction = 0.0;
    double structural_twist = 0.0;
    DampingCoefficients damping_coefficients;
    std::vector<AirfoilProperties> airfoil_properties;

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
        new_point.coordinates *= factor;
        new_point.offset_elastic *= factor;
        new_point.offset_gravity *= factor;
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
        new_point.coordinates += other.coordinates;
        new_point.offset_elastic += other.offset_elastic;
        new_point.offset_gravity += other.offset_gravity;
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

#endif  // REFERENCE_POINT_H_
