#include <nlohmann/json.hpp>
#include <fstream>
#include <iostream>
#include "blade.h"

#define LEN(arr) ((int)(sizeof(arr) / sizeof(arr)[0]))

using json = nlohmann::json;

Blade blade_from_json(std::string filepath) {
    std::ifstream json_file(filepath);

    // populate json object
    json json_obj;
    json_file >> json_obj;

    // extract blade information
    std::vector<double> blade_fraction = json_obj["fraction"];
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
    // std::vector<double> pitch_angles = json_obj["pitch_angles"];

    // populate blade
    auto blade = Blade();
    std::vector<ChVector<double>> centers_reference;
    std::vector<ChVector2<double>> offsets_elastic;
    std::vector<ChVector2<double>> offsets_gravity;
    for (int ii = 0; ii < centers_reference_vec.size(); ii++) {
        auto cr = centers_reference_vec[ii];
        centers_reference.push_back(ChVector<double>(cr[0], cr[1], cr[2]));
        auto oe = offsets_elastic_vec[ii];
        offsets_elastic.push_back(ChVector2<double>(oe[0], oe[1]));
        auto og = offsets_gravity_vec[ii];
        offsets_gravity.push_back(ChVector2<double>(og[0], og[1]));
    }
    blade.centers_reference = centers_reference;
    blade.offsets_elastic = offsets_elastic;
    blade.offsets_gravity = offsets_gravity;
    blade.element_densities = densities;
    blade.structural_twist = structural_twist;
    blade.stiffness_flap = stiffness_flap;
    blade.stiffness_edge = stiffness_edge;
    blade.set_damping_coefficients(damping_coefficients[0], damping_coefficients[1], damping_coefficients[2],
                                   damping_coefficients[3]);
    // TODO: change to actual values
    std::vector<double> stiffness_axial(50, 210e9);
    blade.stiffness_axial = stiffness_axial;
    std::vector<double> stiffness_torsion(50, 1e11);
    blade.stiffness_torsion = stiffness_torsion;

    return blade;
}
