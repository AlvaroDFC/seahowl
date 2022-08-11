#pragma once

#include <vector>
#include <stdexcept>

namespace seahowl {
namespace aero {

/**@brief AriFoil coefficients */
struct AirfoilCoefficients {
    double alpha = 0.0; ///< Angle of attack
    double lift = 0.0;  ///< Lift coefficient Cl
    double drag = 0.0;  ///< Drag coefficient Cd
    double added_mass = 0.0; ///< Added mass coefficient

    AirfoilCoefficients() {}

    ~AirfoilCoefficients() {}

    AirfoilCoefficients operator*(const double factor) const {
        AirfoilCoefficients new_point = *this;
        new_point.alpha *= factor;
        new_point.lift *= factor;
        new_point.drag *= factor;
        new_point.added_mass *= factor;
        return new_point;
    }
    AirfoilCoefficients operator+(const AirfoilCoefficients& other) const {
        AirfoilCoefficients new_point = *this;
        /* new_point.reynolds_number += other.reynolds_number; */
        new_point.alpha += other.alpha;
        new_point.lift += other.lift;
        new_point.drag *= other.drag;
        new_point.added_mass *= other.added_mass;
        return new_point;
    };
};

/**@brief AriFoil properties

@todo depends also of air density
*/
struct AirfoilProperties {
    double reynolds_number = 0.0; ///< Reynlods number
    std::vector<AirfoilCoefficients> coefficients_list; ///< (For each elements ?) the tabulated Lift, drag coefficient according to angle of attack 

    AirfoilProperties() {}

    ~AirfoilProperties() {}

    AirfoilProperties operator*(const double factor) const {
        AirfoilProperties new_point = *this;
        /* new_point.reynolds_number *= factor; */
        for (int ii = 0; ii < coefficients_list.size(); ii++) {
            new_point.coefficients_list[ii] = coefficients_list[ii] * factor;
        }
        return new_point;
    };
    AirfoilProperties operator+(const AirfoilProperties& other) const {
        AirfoilProperties new_point = *this;
        /* new_point.reynolds_number += other.reynolds_number; */
        for (int ii = 0; ii < coefficients_list.size(); ii++) {
            new_point.coefficients_list[ii] = coefficients_list[ii] + other.coefficients_list[ii];
        }
        return new_point;
    };

    AirfoilCoefficients find_coefficients(double alpha) {
        for (auto ii = 0; ii < coefficients_list.size() - 1; ii++) {
            double alpha1 = coefficients_list[ii].alpha;
            double alpha2 = coefficients_list[ii + 1].alpha;
            if (alpha1 <= alpha && alpha <= alpha2) {
                double alpha_range = alpha2 - alpha1;
                double weight1 = 1.0 - (alpha - alpha1) / alpha_range;
                double weight2 = 1.0 - (alpha2 - alpha) / alpha_range;
                AirfoilCoefficients coefficients;
                coefficients.alpha = coefficients_list[ii].alpha * weight1 + coefficients_list[ii + 1].alpha * weight2;
                coefficients.lift = coefficients_list[ii].lift * weight1 + coefficients_list[ii + 1].lift * weight2;
                coefficients.drag = coefficients_list[ii].drag * weight1 + coefficients_list[ii + 1].drag * weight2;
                coefficients.added_mass =
                    coefficients_list[ii].added_mass * weight1 + coefficients_list[ii + 1].added_mass * weight2;
                return coefficients;
            }
        }
        throw std::runtime_error("Could not find alpha value (" + std::to_string(alpha) + ") for airfoil.");
    }
};

}  // namespace aero
}  // namespace seahowl