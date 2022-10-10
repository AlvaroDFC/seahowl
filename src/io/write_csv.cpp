#include <seahowl/io/write_csv.h>

#include <iostream>
#include <fstream>
#include <string>

void write_turbine_info_to_csv(std::string filename, seahowl::core::System ssystem, double time) {
    std::ofstream myfile;
    if (time == 0.0) {
        myfile.open(filename);
        myfile << "time (s),wind (m/s),rpm,power (W),pitch (rad),torque elec (Nm),axial thrust (N),axial torque (Nm),rotor azimuth (deg),\n";
    } else {
        myfile.open(filename, std::ios_base::app);
    }
    myfile << std::to_string(time);
    myfile << ",";
    myfile << std::to_string(
        ssystem.wind_model.get_wind_velocity(ssystem.turbine.rotor.elasto.body_hub->GetPos(), time).Length());
    myfile << ",";
    myfile << std::to_string(ssystem.turbine.rotor.elasto.get_rpm());
    myfile << ",";
    myfile << std::to_string(ssystem.turbine.get_generated_power());
    myfile << ",";
    myfile << std::to_string(ssystem.turbine.rotor.elasto.pitch_collective);
    myfile << ",";
    myfile << std::to_string(ssystem.turbine.controller->get_torque_elec());
    myfile << ",";
    myfile << std::to_string(ssystem.turbine.rotor.elasto.get_axial_thrust());
    myfile << ",";
    myfile << std::to_string(ssystem.turbine.rotor.elasto.get_axial_torque());
    myfile << ",";
    myfile << std::to_string(ssystem.turbine.rotor.elasto.get_azimuth());
    myfile << ",\n";
    myfile.close();
}