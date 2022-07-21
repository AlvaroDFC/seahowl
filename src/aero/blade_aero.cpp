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
    double density = 1.225;
    for (int ii = 0; ii < elements.size(); ii++) {
        auto& element = elements[ii];
        auto& properties = element.properties;

        // get fluid relative velocity
        auto wind_velocity = wind_model.get_wind_velocity(properties.coordinates, 0.0);
        auto global_velocity = wind_velocity - properties.velocity;
        // project locally
        auto local_velocity0_3d = properties.rotation.RotateBack(global_velocity);
        auto local_velocity0 = ChVector2<double>(-local_velocity0_3d[1], local_velocity0_3d[2]);

        // update induction factors and return local velocity
        auto local_velocity = element.get_induced_velocity(local_velocity0);

        // get coefficients from angle of attack
        double alpha = atan2(local_velocity.y(), local_velocity.x()) * 180 / CH_C_PI;
        auto coefficients = properties.airfoil_properties[0].find_coefficients(alpha);

        // calculate drag and lift force values
        double vel = local_velocity.Length();
        double chord = properties.chord;
        double length = element.length;
        double lift = 0.5 * density * vel * vel * chord * coefficients.lift * length;
        double drag = 0.5 * density * vel * vel * chord * coefficients.drag * length;

        // transform from local to global load
        auto load_local = ChVector<double>(0.0, drag, lift);
        auto load_global = properties.rotation.Rotate(load_local);
        loads[ii] = load_global;
    }
}
