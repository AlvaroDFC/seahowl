#include "seahowl/aero/blade_aero.h"
#include <seahowl/aero/bemt.h>

using seahowl::aero::BladeElementAero;
using seahowl::aero::BladeAero;
using seahowl::aero::get_induced_velocity;

BladeElementAero::BladeElementAero(BladeReferencePointAero& point1, BladeReferencePointAero& point2) {
    properties = (point1 + point2) * 0.5;
    length = (point1.coordinates - point2.coordinates).Length();
}

BladeElementAero::~BladeElementAero() {}

chrono::ChVector2<double> BladeElementAero::get_induced_velocity_rotor(chrono::ChVector2<double>& local_velocity_rotor0,
                                                                       size_t nblades,
                                                                       bool tip_loss,
                                                                       bool hub_loss) {
    return get_induced_velocity(*this, local_velocity_rotor0, nblades, tip_loss, hub_loss);
}

BladeAero::BladeAero() {}

BladeAero::~BladeAero() {}

void BladeAero::build() {
    // check that enough reference points were defined to create elements (at least 2)
    if (reference_points.size() < 2) {
        throw std::runtime_error("Not enough aero reference points defined for blade.");
    }

    // check that discretization_fractions was defined, otherwise take reference point fractions
    if (discretization_fractions.size() == 0) {
        for (int ii = 0; ii < reference_points.size(); ii++) {
            discretization_fractions.push_back(reference_points[ii].fraction);
        }
    }

    // build
    discretized_points = seahowl::core::get_discretized_points(discretization_fractions, reference_points);
    for (int ii = 0; ii < discretized_points.size() - 1; ii++) {
        // make element
        auto element = BladeElementAero(discretized_points[ii], discretized_points[ii + 1]);
        elements.push_back(element);
        // push empty load
        loads.push_back(chrono::ChVector<double>(0.0, 0.0, 0.0));
        relative_velocities_induced.push_back(chrono::ChVector<double>(0.0, 0.0, 0.0));
        wind_velocities.push_back(chrono::ChVector<double>(0.0, 0.0, 0.0));
        wind_velocities_shadowed.push_back(chrono::ChVector<double>(0.0, 0.0, 0.0));
    }

    // get distance from tip
    compute_distances_from_tip();
}

void BladeAero::compute_distances_from_tip() {
    // this is the position of the element at the tip
    auto& element_tip_position = elements.back().properties.coordinates;
    // need to add 0.5*length of the element to get actual distance from tip
    double offset = 0.5 * elements.back().length;
    for (int ii = 0; ii < elements.size(); ii++) {
        auto& element = elements[ii];
        element.distance_from_tip = (element.properties.coordinates - element_tip_position).Length() + offset;
    }
}

void BladeAero::compute_distances_from_hub(chrono::ChVector<double> hub_apex_position, double hub_radius) {
    for (auto& element : elements) {
        element.distance_from_hub = (element.properties.coordinates - hub_apex_position).Length() - hub_radius;
    }
}

void BladeAero::compute_radii(chrono::ChVector<double> hub_apex_position) {
    for (auto& element : elements) {
        element.radius = (element.properties.coordinates - hub_apex_position).Length();
    }
}

// // DO NOT USE: use compute_wind_loads_bemt from RotorAero class instead
// // method is kept here for testing purposes
// void BladeAero::compute_wind_loads_bemt(WindModel& wind_model, double time) {
//     double density = wind_model.get_density();
//     for (int ii = 0; ii < elements.size(); ii++) {
//         auto& element = elements[ii];
//         auto& properties = element.properties;

//         // get fluid relative velocity
//         auto wind_velocity = wind_model.get_wind_velocity(properties.coordinates, time);
//         auto global_velocity = wind_velocity - properties.velocity;
//         // project locally
//         auto local_velocity0_3d = properties.rotation.RotateBack(global_velocity);
//         // project using BEMT convention: x along chord, y along thickness up
//         auto local_velocity0 = ChVector2<double>(local_velocity0_3d[1], -local_velocity0_3d[2]);

//         // update induction factors and return local velocity
//         auto local_velocity = element.get_induced_velocity_element(local_velocity0);

//         // get coefficients from angle of attack
//         double alpha = atan2(local_velocity.y(), -local_velocity.x());
//         auto coefficients = properties.airfoil_properties[0].find_coefficients(alpha * 180 / CH_C_PI);

//         // calculate drag and lift force
//         double vel = local_velocity.Length();
//         double chord = properties.chord;
//         double length = element.length;
//         double lift = 0.5 * density * vel * vel * chord * coefficients.lift * length;
//         double drag = 0.5 * density * vel * vel * chord * coefficients.drag * length;

//         // project to element local frame
//         double lift_local = lift * cos(alpha) + drag * sin(alpha);
//         double drag_local = lift * sin(alpha) - drag * cos(alpha);

//         // transform from local to global load
//         // use chrono convention
//         auto load_local = ChVector<double>(0.0, lift_local, -drag_local);
//         auto load_global = properties.rotation.Rotate(load_local);
//         loads[ii] = load_global;
//     }
// }
