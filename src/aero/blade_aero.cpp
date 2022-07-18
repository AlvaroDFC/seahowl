#include "blade_aero.h"

void BladeAero::build() {
    // check that enough reference points were defined to create elements (at least 2)
    if (reference_points.size() < 2) {
        std::runtime_error("Not enough aero reference points defined for blade.");
    }

    // check that discretization_fractions was defined, otherwise take reference point fractions
    if (discretization_fractions.size() == 0) {
        for (int ii = 0; ii < reference_points.size(); ii++) {
            discretization_fractions.push_back(reference_points[ii].fraction);
        }
    }

    // build
    discretized_points = get_discretized_points(discretization_fractions, reference_points);
    for (int ii = 0; ii < discretized_points.size() - 1; ii++) {
        // make element
        auto element = BladeElementAero(discretized_points[ii], discretized_points[ii + 1]);
        elements.push_back(element);
        // push empty load
        loads.push_back(ChVector<double>(0.0, 0.0, 0.0));
    }
}

void BladeAero::compute_loads(double time, WindModel& wind_model) {
    // TODO: include induction factors
    for (int ii = 0; ii < elements.size(); ii++) {
        auto& element = elements[ii];
        double length = element.length;
        auto& properties = element.properties;
        double chord = properties.chord;

        auto wind_speed = wind_model.get_wind_speed(properties.coordinates, 0.0);
        double density = 1.225;
        // get fluid relative speed locally
        auto relative_speed = wind_speed - properties.velocity;
        auto local_speed = properties.rotation.RotateBack(relative_speed);
        double local_speed_x = -local_speed[1];
        double local_speed_y = local_speed[2];

        // get coefficients from angle of attack
        double alpha = atan2(local_speed_y, local_speed_x) * 180 / CH_C_PI;
        auto coefficients = properties.airfoil_properties[0].find_coefficients(alpha);

        // calculate W values
        double W = sqrt(local_speed_x * local_speed_x + local_speed_y * local_speed_y);
        double lift = 0.5 * density * W * W * chord * coefficients.lift * length;
        double drag = 0.5 * density * W * W * chord * coefficients.drag * length;
        // double phi = properties.structural_twist + alpha;
        // double force_lift = lift * sin(phi) - drag * cos(phi);
        // double force_drag = lift * cos(phi) + drag * sin(phi);

        // transform from local to global load
        auto load_local = ChVector<double>(0.0, drag, lift);
        auto load_global = properties.rotation.Rotate(load_local);
        loads[ii] = load_global;
    }
}
