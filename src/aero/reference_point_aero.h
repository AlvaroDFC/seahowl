#ifndef REFERENCE_POINT_AERO_H_
#define REFERENCE_POINT_AERO_H_

#include "../core/reference_point.h"

struct BladeReferencePointAero {
    double fraction;
    chrono::ChVector<double> coordinates;
    chrono::ChQuaternion<double> rotation;
    chrono::ChVector<double> velocity;
    double chord;
    double structural_twist;
    std::vector<AirfoilProperties> airfoil_properties;

    BladeReferencePointAero() {}

    BladeReferencePointAero(BladeReferencePoint& point) {
        fraction = point.fraction;
        coordinates = point.coordinates;
        velocity = chrono::ChVector<double>(0.0, 0.0, 0.0);
        chord = point.chord;
        structural_twist = point.structural_twist;
        airfoil_properties = point.airfoil_properties;
    }

    BladeReferencePointAero operator*(const double factor) const {
        BladeReferencePointAero new_point = *this;
        new_point.fraction *= factor;
        new_point.coordinates *= factor;
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
        new_point.coordinates += other.coordinates;
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

#endif  // REFERENCE_POINT_AERO_H_
