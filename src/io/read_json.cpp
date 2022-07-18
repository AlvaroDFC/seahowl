#include <fstream>
#include <iostream>
#include "read_json.h"
#include "../utils.h"
#include <filesystem>
namespace fs = std::filesystem;

using json = nlohmann::json;

std::vector<BladeReferencePoint> get_blade_reference_points_from_json(std::string filepath) {
    std::ifstream json_file(filepath);

    // populate json object
    json json_obj;
    json_file >> json_obj;

    // EXTRACT INFO
    //
    std::vector<BladeReferencePoint> reference_points;
    auto points = json_obj["reference_points"];
    std::vector<double> damping_coefficients = json_obj["damping_coefficients"];
    double blade_length = points[points.size() - 1]["coordinates"][2];
    for (int ii = 0; ii < points.size(); ii++) {
        auto point = points[ii];
        auto reference_point = BladeReferencePoint();

        std::vector<double> coords = point["coordinates"];
        reference_point.fraction = coords[2] / blade_length;
        reference_point.coordinates = ChVector<double>(coords[0], coords[1], coords[2]);
        if (point.contains("offset_gravity")) {
            std::vector<double> og = point["offsets_gravity"];
            reference_point.offset_gravity = ChVector2<double>(og[0], og[1]);
        }
        if (point.contains("offset_elastic")) {
            std::vector<double> oe = point["offset_elastic"];
            reference_point.offset_elastic = ChVector2<double>(oe[0], oe[1]);
        }

        std::vector<std::vector<double>> sm = point["stiffness_matrix"];
        std::vector<std::vector<double>> mm = point["mass_matrix"];
        int jjo, kko;
        // apply offsets to indices to switch from IEC standard to Chrono standard
        for (int jj = 0; jj < 6; jj++) {
            if (jj == 2 || jj == 5) {
                jjo = -2;
            }
            if (jj == 1 || jj == 4) {
                jjo = +0;
            }
            if (jj == 0 || jj == 3) {
                jjo = +2;
            }
            for (int kk = 0; kk < 6; kk++) {
                if (kk == 2 || kk == 5) {
                    kko = -2;
                }
                if (kk == 1 || kk == 4) {
                    kko = +0;
                }
                if (kk == 0 || kk == 3) {
                    kko = +2;
                }
                reference_point.stiffness_matrix(jj + jjo, kk + kko) = sm[jj][kk];
                reference_point.mass_matrix(jj + jjo, kk + kko) = mm[jj][kk];
            }
        }
        double twist = point["twist"];
        reference_point.structural_twist = twist * CH_C_PI / 180.0;
        reference_point.damping_coefficients.bx = damping_coefficients[0];
        reference_point.damping_coefficients.by = damping_coefficients[1];
        reference_point.damping_coefficients.bz = damping_coefficients[2];
        reference_point.damping_coefficients.bt = damping_coefficients[3];

        if (point.contains("chord")) {
            reference_point.chord = point["chord"];
        }

        // populate json object
        if (point.contains("airfoil_file")) {
            auto main_directory = fs::path(filepath).parent_path();
            std::string airfoil_filename = point["airfoil_file"];
            auto airfoil_filepath = main_directory / airfoil_filename;
            std::ifstream airfoil_file(airfoil_filepath.u8string());
            json json_airfoil;
            airfoil_file >> json_airfoil;

            int nreynolds = json_airfoil.size();
            for (int jj = 0; jj < nreynolds; jj++) {
                auto airfoil_properties = json_airfoil[jj];
                std::vector<std::vector<double>> coeffs = airfoil_properties["coefficients"];
                double reynolds_number = airfoil_properties["reynolds_number"];

                std::vector<AirfoilCoefficients> coefficients_list;

                for (int kk = 0; kk < coeffs.size(); kk++) {
                    AirfoilCoefficients coefficients;
                    coefficients.alpha = coeffs[kk][0];
                    coefficients.lift = coeffs[kk][1];
                    coefficients.drag = coeffs[kk][2];
                    coefficients.added_mass = coeffs[kk][3];
                    coefficients_list.push_back(coefficients);
                }
                AirfoilProperties airfoil;
                airfoil.reynolds_number = reynolds_number;
                airfoil.coefficients_list = coefficients_list;
                reference_point.airfoil_properties.push_back(airfoil);
            }
        }

        reference_points.push_back(reference_point);
    }

    return reference_points;
}

Blade get_blade_from_json(std::string filepath) {
    std::ifstream json_file(filepath);

    // populate json object
    json json_obj;
    json_file >> json_obj;

    bool fpm_mode = json_obj["fpm_mode"];

    Blade blade = Blade();
    blade.elasto->fpm_mode = fpm_mode;
    blade.reference_points = get_blade_reference_points_from_json(filepath);
    if (json_obj.contains("discretization_elasto")) {
        std::vector<double> discretization_elasto = json_obj["discretization_elasto"];
        blade.set_discretization_elasto(discretization_elasto);
    }
    if (json_obj.contains("discretization_aero")) {
        std::vector<double> discretization_aero = json_obj["discretization_aero"];
        blade.set_discretization_aero(discretization_aero);
    }

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
    std::vector<double> precones = json_obj["precones"];
    rotor.blade_precones = precones;
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
