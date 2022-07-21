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
    std::vector<BladeReferencePoint> reference_points;
    auto points = json_obj.at("reference_points").get<json>();
    auto damping_coefficients = json_obj.at("damping_coefficients").get<std::vector<double>>();
    if (damping_coefficients.size() != 4) {
        throw std::runtime_error("Damping coefficients has to be vector of length 4.");
    }
    double blade_length = points[points.size() - 1]["coordinates"][2];
    for (int ii = 0; ii < points.size(); ii++) {
        auto point = points[ii];
        auto reference_point = BladeReferencePoint();

        auto coords = point.at("coordinates").get<std::vector<double>>();
        if (coords.size() != 3) {
            throw std::runtime_error("Coordinates has to be vector of length 3.");
        }
        reference_point.fraction = coords[2] / blade_length;
        reference_point.coordinates = ChVector<double>(coords[0], coords[1], coords[2]);
        if (point.contains("offset_gravity")) {
            auto og = point.at("offsets_gravity").get<std::vector<double>>();
            reference_point.offset_gravity = ChVector2<double>(og[0], og[1]);
        }
        if (point.contains("offset_elastic")) {
            auto oe = point.at("offsets_elastic").get<std::vector<double>>();
            reference_point.offset_elastic = ChVector2<double>(oe[0], oe[1]);
        }

        auto sm = point.at("stiffness_matrix").get<std::vector<std::vector<double>>>();
        auto mm = point.at("mass_matrix").get<std::vector<std::vector<double>>>();
        if (sm.size() != 6 || mm.size() != 6) {
            throw std::runtime_error("Mass and stiffness matrices hqve to be defined as 6x6 matrices.");
        }
        int jjo, kko;
        // apply offsets to indices to switch from IEC standard to Chrono standard
        for (int jj = 0; jj < 6; jj++) {
            if (sm[jj].size() != 6 || mm[jj].size() != 6) {
                throw std::runtime_error("Mass and stiffness matrices hqve to be defined as 6x6 matrices.");
            }
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
        reference_point.structural_twist = point.at("twist").get<double>() * CH_C_PI / 180.0;
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
            auto airfoil_filename = point.at("airfoil_file").get<std::string>();
            auto airfoil_filepath = main_directory / airfoil_filename;
            std::ifstream airfoil_file(airfoil_filepath.u8string());
            json json_airfoil;
            airfoil_file >> json_airfoil;

            int nreynolds = json_airfoil.size();
            for (int jj = 0; jj < nreynolds; jj++) {
                auto airfoil_properties = json_airfoil[jj];
                auto coeffs = airfoil_properties.at("coefficients").get<std::vector<std::vector<double>>>();

                std::vector<AirfoilCoefficients> coefficients_list;

                for (int kk = 0; kk < coeffs.size(); kk++) {
                    if (coeffs[kk].size() != 4) {
                        throw std::runtime_error("Airfoil coefficients has to be vectors of length 4.");
                    }
                    AirfoilCoefficients coefficients;
                    coefficients.alpha = coeffs[kk][0];
                    coefficients.lift = coeffs[kk][1];
                    coefficients.drag = coeffs[kk][2];
                    coefficients.added_mass = coeffs[kk][3];
                    coefficients_list.push_back(coefficients);
                }
                AirfoilProperties airfoil;
                airfoil_properties.at("reynolds_number").get_to(airfoil.reynolds_number);
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

    Blade blade = Blade();
    blade.elasto->fpm_mode = json_obj.at("fpm_mode").get<bool>();
    blade.reference_points = get_blade_reference_points_from_json(filepath);
    if (json_obj.contains("discretization_elasto")) {
        auto discretization_elasto = json_obj.at("discretization_elasto").get<std::vector<double>>();
        blade.set_discretization_elasto(discretization_elasto);
    }
    if (json_obj.contains("discretization_aero")) {
        auto discretization_aero = json_obj.at("discretization_aero").get<std::vector<double>>();
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
    double height = json_obj.at("height").get<double>();
    double base_height = json_obj.at("base_height").get<double>();
    auto damping_coefficients = json_obj.at("damping_coefficients").get<std::vector<double>>();

    // MAKE TOWER REFERENCE POINTS
    std::vector<TowerReferencePoint> reference_points;
    auto points = json_obj.at("reference_points").get<json>();
    for (int ii = 0; ii < points.size(); ii++) {
        auto point = points[ii];
        auto reference_point = TowerReferencePoint();
        point.at("fraction").get_to(reference_point.fraction);
        reference_point.coordinates = ChVector<double>(0.0, 0.0, (height - base_height) * reference_point.fraction);
        point.at("stiffness_sideside").get_to(reference_point.stiffness_sideside);
        point.at("stiffness_foreaft").get_to(reference_point.stiffness_foreaft);
        point.at("density").get_to(reference_point.density);
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

TowerElasto get_tower_from_json(std::string filepath) {
    std::ifstream json_file(filepath);

    // populate json object
    json json_obj;
    json_file >> json_obj;

    TowerElasto tower = TowerElasto();
    tower.height = json_obj.at("height").get<double>();
    tower.base_height = json_obj.at("base_height").get<double>();
    tower.reference_points = get_tower_reference_points_from_json(filepath);
    tower.discretization_fractions = json_obj.at("discretization_fractions").get<std::vector<double>>();

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
    json_obj.at("precones").get_to(rotor.elasto.blade_precones);
    for (int ii = 0; ii < rotor.elasto.blade_precones.size(); ii++) {
        // convert to radians
        rotor.elasto.blade_precones[ii] *= CH_C_PI / 180.0;
    }
    // hub
    auto hub = json_obj.at("hub");
    hub.at("CM").get_to(rotor.elasto.hub.center_of_mass);
    hub.at("mass").get_to(rotor.elasto.hub.mass);
    hub.at("inertia").get_to(rotor.elasto.hub.inertia);
    hub.at("overhang").get_to(rotor.elasto.hub.overhang);
    hub.at("radius").get_to(rotor.elasto.hub.radius);
    // nacelle
    auto nacelle = json_obj.at("nacelle");
    auto cm = nacelle.at("CM").get<std::vector<double>>();
    if (cm.size() != 3) {
        throw std::runtime_error("Center of mass of nacelle has to be vector of length 3.");
    }
    rotor.elasto.nacelle.center_of_mass = ChVector<double>(cm[0], cm[1], cm[2]);
    nacelle.at("mass").get_to(rotor.elasto.nacelle.mass);
    nacelle.at("inertia").get_to(rotor.elasto.nacelle.inertia);
    nacelle.at("yaw_bearing_mass").get_to(rotor.elasto.nacelle.yaw_bearing_mass);
    // shaft
    auto shaft = json_obj.at("shaft");
    shaft.at("distance_from_towertop").get_to(rotor.elasto.shaft.distance_from_towertop);
    shaft.at("tilt").get_to(rotor.elasto.shaft.tilt);
    // convert to radians
    rotor.elasto.shaft.tilt *= CH_C_PI / 180.0;

    return rotor;
}

Turbine get_turbine_from_json(std::vector<std::string> filepaths_blades,
                              std::string filepath_rotor,
                              std::string filepath_tower) {
    std::vector<std::shared_ptr<Blade>> blades;
    for (int ii = 0; ii < filepaths_blades.size(); ii++) {
        blades.push_back(std::make_shared<Blade>(get_blade_from_json(filepaths_blades[ii])));
    }

    auto rotor = get_rotor_from_json(filepath_rotor);

    auto tower = get_tower_from_json(filepath_tower);

    auto turbine = Turbine();
    turbine.rotor = rotor;
    turbine.tower = tower;
    turbine.blades = blades;

    return turbine;
}
