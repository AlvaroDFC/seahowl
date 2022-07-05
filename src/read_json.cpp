#include <fstream>
#include <iostream>
#include "read_json.h"

using json = nlohmann::json;

std::vector<BladeReferencePoint> get_blade_reference_points_from_json(std::string filepath) {
    std::ifstream json_file(filepath);

    // populate json object
    json json_obj;
    json_file >> json_obj;

    // EXTRACT INFO
    //
    std::vector<double> blade_fractions = json_obj["fractions"];
    std::vector<std::vector<double>> centers_reference_vec = json_obj["centers_reference"];
    std::vector<std::vector<double>> offsets_elastic_vec;
    if (json_obj.contains("offsets_elastic")) {
        offsets_elastic_vec = json_obj["offsets_elastic"];
    } else {
        for (int ii = 0; ii < centers_reference_vec.size(); ii++) {
            std::vector<double> offset_elastic{0.0, 0.0};
            offsets_elastic_vec.push_back(offset_elastic);
        }
    }
    std::vector<std::vector<double>> offsets_gravity_vec;
    if (json_obj.contains("offsets_gravity")) {
        offsets_gravity_vec = json_obj["offsets_gravity"];
    } else {
        for (int ii = 0; ii < centers_reference_vec.size(); ii++) {
            std::vector<double> offset_gravity{0.0, 0.0};
            offsets_gravity_vec.push_back(offset_gravity);
        }
    }
    std::vector<double> stiffness_edge = json_obj["stiffness_edge"];
    std::vector<double> stiffness_flap = json_obj["stiffness_flap"];
    std::vector<double> densities = json_obj["densities"];
    std::vector<double> structural_twist = json_obj["structural_twist"];
    std::vector<double> damping_coefficients = json_obj["damping_coefficients"];

    // MAKE BLADE REFERENCE POINTS
    //
    std::vector<BladeReferencePoint> reference_points;
    for (int ii = 0; ii < blade_fractions.size(); ii++) {
        BladeReferencePoint reference_point;
        reference_point.fraction = blade_fractions[ii];
        auto coords = centers_reference_vec[ii];
        reference_point.coordinates = ChVector<double>(coords[0], coords[1], coords[2]);
        auto oe = offsets_elastic_vec[ii];
        reference_point.offset_elastic = ChVector2<double>(oe[0], oe[1]);
        auto og = offsets_gravity_vec[ii];
        reference_point.offset_gravity = ChVector2<double>(og[0], og[1]);
        reference_point.density = densities[ii];
        reference_point.structural_twist = structural_twist[ii] * CH_C_PI / 180.0;
        ;
        reference_point.stiffness_edge = stiffness_edge[ii];
        reference_point.stiffness_flap = stiffness_flap[ii];
        reference_point.damping_coefficients.bx = damping_coefficients[0];
        reference_point.damping_coefficients.by = damping_coefficients[1];
        reference_point.damping_coefficients.bz = damping_coefficients[2];
        reference_point.damping_coefficients.bt = damping_coefficients[3];
        // TODO: change to actual values
        reference_point.stiffness_axial = 210e9;
        reference_point.stiffness_torsion = 1e11;

        reference_points.push_back(reference_point);
    }

    return reference_points;
}

Blade get_blade_from_json(std::string filepath) {
    std::ifstream json_file(filepath);

    // populate json object
    json json_obj;
    json_file >> json_obj;

    std::vector<double> discretization_fractions = json_obj["discretization_fractions"];

    Blade blade = Blade();
    blade.reference_points = get_blade_reference_points_from_json(filepath);
    blade.discretization_fractions = discretization_fractions;

    return blade;
}

std::vector<TowerReferencePoint> get_tower_reference_points_from_json(std::string filepath) {
    std::ifstream json_file(filepath);

    // populate json object
    json json_obj;
    json_file >> json_obj;

    // EXTRACT INFO
    //
    std::vector<double> tower_fractions = json_obj["fractions"];
    double height = json_obj["height"];
    double base_height = json_obj["base_height"];
    std::vector<double> stiffness_foreaft = json_obj["stiffness_foreaft"];
    std::vector<double> stiffness_sideside = json_obj["stiffness_sideside"];
    std::vector<double> densities = json_obj["densities"];
    std::vector<double> damping_coefficients = json_obj["damping_coefficients"];

    // MAKE TOWER REFERENCE POINTS
    //
    std::vector<TowerReferencePoint> reference_points;
    for (int ii = 0; ii < tower_fractions.size(); ii++) {
        TowerReferencePoint reference_point;
        reference_point.fraction = tower_fractions[ii];
        reference_point.coordinates = ChVector<double>(0.0, 0.0, (height - base_height) * tower_fractions[ii]);
        reference_point.density = densities[ii];
        reference_point.stiffness_sideside = stiffness_sideside[ii];
        reference_point.stiffness_foreaft = stiffness_foreaft[ii];
        reference_point.damping_coefficients.bx = damping_coefficients[0];
        reference_point.damping_coefficients.by = damping_coefficients[1];
        reference_point.damping_coefficients.bz = damping_coefficients[2];
        reference_point.damping_coefficients.bt = damping_coefficients[3];
        // TODO: change to actual values
        reference_point.stiffness_axial = 210e9;
        reference_point.stiffness_torsion = 1e11;

        reference_points.push_back(reference_point);
    }

    return reference_points;
}

Tower get_tower_from_json(std::string filepath) {
    std::ifstream json_file(filepath);

    // populate json object
    json json_obj;
    json_file >> json_obj;

    std::vector<double> discretization_fractions = json_obj["discretization_fractions"];
    double height = json_obj["height"];
    double base_height = json_obj["base_height"];

    Tower tower = Tower();
    tower.height = height;
    tower.base_height = base_height;
    tower.reference_points = get_tower_reference_points_from_json(filepath);
    tower.discretization_fractions = discretization_fractions;

    return tower;
}

Rotor get_rotor_from_json(std::string filepath) {
    std::ifstream json_file(filepath);

    // populate json object
    json json_obj;
    json_file >> json_obj;

    // EXTRACT INFO
    //
    auto rotor = Rotor();
    // blades
    rotor.blade_precones = (std::vector<double>)json_obj["precones"];
    for (int ii = 0; ii < rotor.blade_precones.size(); ii++) {
        // convert to radians
        rotor.blade_precones[ii] *= CH_C_PI / 180.0;
    }
    // hub
    rotor.hub.center_of_mass = json_obj["hub"]["CM"];
    rotor.hub.mass = json_obj["hub"]["mass"];
    rotor.hub.inertia = json_obj["hub"]["inertia"];
    rotor.hub.overhang = json_obj["hub"]["overhang"];
    rotor.hub.radius = json_obj["hub"]["radius"];
    // nacelle
    std::vector<double> cm = json_obj["nacelle"]["CM"];
    rotor.nacelle.center_of_mass = ChVector<double>(cm[0], cm[1], cm[2]);
    rotor.nacelle.mass = json_obj["nacelle"]["mass"];
    rotor.nacelle.inertia = json_obj["nacelle"]["inertia"];
    rotor.nacelle.yaw_bearing_mass = json_obj["nacelle"]["yaw_bearing_mass"];
    // shaft
    rotor.shaft.distance_from_towertop = json_obj["shaft"]["distance_from_towertop"];
    rotor.shaft.tilt = (double)json_obj["shaft"]["tilt"] * CH_C_PI / 180.0;

    return rotor;
}
