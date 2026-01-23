// Local test headers
#include "fixture_components.h"

// SEAHOWL headers
#include <seahowl/core.h>
#include <seahowl/elasto.h>
#include <seahowl/env.h>
#include <seahowl/io.h>
#include <seahowl/servo.h>

// Third-party libraries
#include <gtest/gtest.h>
#include <spdlog/spdlog.h>

// Standard library
#include <filesystem>

using std::filesystem::path;

using namespace seahowl;
using namespace seahowl::elasto;

// The fixture for testing
class TestController : public FixtureComponents {
  protected:
    TestController() : FixtureComponents() {
        ref_dir /= "test_controller/ref";
        test_dir /= "test_controller/test";
    }

    // Create simulation with standard output settings disabled
    seahowl::core::Simulation create_simulation(double dt, double duration) {
        seahowl::core::Simulation simulation;
        simulation.dt = dt;
        simulation.duration = duration;
        simulation.outputs->dt_output = 9999.9;
        simulation.outputs->has_csv = false;
        simulation.outputs->has_gui = false;
        simulation.outputs->has_vtk = false;
        return simulation;
    }

    // Setup constant wind model with shear and reference height from turbine
    std::shared_ptr<seahowl::env::ConstantWind> setup_wind(seahowl::core::System& system_core,
                                                           seahowl::core::Turbine& turbine,
                                                           const Vector3d& velocity,
                                                           double shear_coefficient = 0.12) {
        auto wind_model = std::make_shared<seahowl::env::ConstantWind>();
        wind_model->shear_coefficient = shear_coefficient;
        wind_model->reference_height = turbine.elasto.rna->rotor->body_hub->get_position().z();
        wind_model->set_wind_velocity(velocity);
        system_core.env_model->add_model(wind_model);
        return wind_model;
    }

    // Run simulation loop writing to dataset each step
    void run_simulation_loop(seahowl::core::Simulation& simulation, TestFrameworkDataset& dataset) {
        auto& system_core = *simulation.system_core;
        dataset.test_csv.write_row();  // output at time=0
        while (system_core.get_time() < simulation.duration) {
            simulation.step();
            dataset.test_csv.write_row();
        }
    }
};

TEST_F(TestController, collective_pitch_control) {
    auto simulation = create_simulation(0.05, 200.0);
    auto& system_core = *simulation.system_core;
    auto& system_elasto = system_core.elasto;

    seahowl::io::add_turbine_to_system_from_file((DATADIR / "IEA15MW/onshore/turbine.json").generic_string(),
                                                 system_core);
    auto& turbine = *system_core.turbines[0];
    for (auto& blade : turbine.rna.rotor.blades) {
        blade->elasto.actuator_pitch->set_fixed_actuator(false);
        blade->elasto.apply_pitch_increment(0.2);
    }
    system_elasto.do_statics(true, 10);

    setup_wind(system_core, turbine, seahowl::Vector3d(12.0, 0.0, 0.0));
    simulation.initialize();

    TestFrameworkDataset test_dataset({false, (ref_dir / "test_collective_pitch_control.csv").generic_string(),
                                       (test_dir / "test_collective_pitch_control.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time (s)", [&system_elasto]() { return system_elasto.get_time(); });
    test_dataset.test_csv.add_function("blade pitch (rad)",
                                       [&turbine]() { return turbine.rna.rotor.blades[0]->elasto.get_pitch(); });
    test_dataset.test_csv.add_function(
        "blade root moment (Nm)", [&turbine]() { return turbine.rna.rotor.blades[0]->elasto.get_blade_root_moment(); });

    run_simulation_loop(simulation, test_dataset);
    EvaluateTest(test_dataset);
};

TEST_F(TestController, collective_pitch_control_snap) {
    auto simulation = create_simulation(0.05, 200.0);
    auto& system_core = *simulation.system_core;
    auto& system_elasto = system_core.elasto;

    seahowl::io::add_turbine_to_system_from_file((DATADIR / "IEA15MW/onshore/turbine.json").generic_string(),
                                                 system_core);
    auto& turbine = *system_core.turbines[0];
    for (auto& blade : turbine.rna.rotor.blades) {
        blade->elasto.actuator_pitch->set_fixed_actuator(true);
        blade->elasto.apply_pitch_increment(0.2);
    }
    system_elasto.do_statics(true, 10);

    setup_wind(system_core, turbine, seahowl::Vector3d(12.0, 0.0, 0.0));
    simulation.initialize();

    TestFrameworkDataset test_dataset({false, (ref_dir / "test_collective_pitch_control_snap.csv").generic_string(),
                                       (test_dir / "test_collective_pitch_control_snap.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time (s)", [&system_elasto]() { return system_elasto.get_time(); });
    test_dataset.test_csv.add_function("blade pitch (rad)",
                                       [&turbine]() { return turbine.rna.rotor.blades[0]->elasto.get_pitch(); });
    test_dataset.test_csv.add_function(
        "blade root moment (Nm)", [&turbine]() { return turbine.rna.rotor.blades[0]->elasto.get_blade_root_moment(); });

    run_simulation_loop(simulation, test_dataset);
    EvaluateTest(test_dataset);
};

TEST_F(TestController, individual_pitch_control) {
    auto simulation = create_simulation(0.05, 200.0);
    auto& system_core = *simulation.system_core;
    auto& system_elasto = system_core.elasto;

    seahowl::io::add_turbine_to_system_from_file((DATADIR / "IEA15MW/onshore/turbine_ipc.json").generic_string(),
                                                 system_core);
    auto& turbine = *system_core.turbines[0];
    for (auto& blade : turbine.rna.rotor.blades) {
        blade->elasto.apply_pitch_increment(0.2);
    }
    system_elasto.do_statics(true, 10);

    setup_wind(system_core, turbine, seahowl::Vector3d(12.0, 0.0, 0.0));
    simulation.initialize();

    TestFrameworkDataset test_dataset({false, (ref_dir / "test_individual_pitch_control.csv").generic_string(),
                                       (test_dir / "test_individual_pitch_control.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time (s)", [&system_elasto]() { return system_elasto.get_time(); });
    test_dataset.test_csv.add_function("blade pitch (rad)",
                                       [&turbine]() { return turbine.rna.rotor.blades[0]->elasto.get_pitch(); });
    test_dataset.test_csv.add_function(
        "blade root moment (Nm)", [&turbine]() { return turbine.rna.rotor.blades[0]->elasto.get_blade_root_moment(); });

    run_simulation_loop(simulation, test_dataset);
    EvaluateTest(test_dataset);
};

TEST_F(TestController, yaw_control) {
    auto simulation = create_simulation(0.05, 50.0);
    auto& system_core = *simulation.system_core;
    auto& system_elasto = system_core.elasto;

    seahowl::io::add_turbine_to_system_from_file((DATADIR / "IEA15MW/onshore/turbine_rigid.json").generic_string(),
                                                 system_core);
    auto& turbine = *system_core.turbines[0];

    auto controller = std::make_shared<seahowl::servo::ControllerDISCON>(
        (DATADIR / "IEA15MW/base/controller/DISCON_yaw.IN").generic_string(),
        (DATADIR / "IEA15MW/base/controller/libdiscon.so").generic_string());
    turbine.controller = controller;

    turbine.rna.elasto.actuator_yaw->set_fixed_actuator(false);
    turbine.rna.elasto.apply_yaw_increment(-0.1);
    system_elasto.do_statics(true, 10);

    setup_wind(system_core, turbine, seahowl::Vector3d(12.0, 1.0, 0.0));
    simulation.initialize();

    TestFrameworkDataset test_dataset({false, (ref_dir / "test_yaw_control.csv").generic_string(),
                                       (test_dir / "test_yaw_control.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time (s)", [&system_elasto]() { return system_elasto.get_time(); });
    test_dataset.test_csv.add_function("yaw (rad)", [&turbine]() { return turbine.rna.elasto.get_yaw(); });
    test_dataset.test_csv.add_function("towerbase moment (N)",
                                       [&turbine]() { return turbine.tower.elasto.get_tower_base_moment(); });

    run_simulation_loop(simulation, test_dataset);
    EvaluateTest(test_dataset);
};

TEST_F(TestController, yaw_control_snap) {
    auto simulation = create_simulation(0.05, 50.0);
    auto& system_core = *simulation.system_core;
    auto& system_elasto = system_core.elasto;

    seahowl::io::add_turbine_to_system_from_file((DATADIR / "IEA15MW/onshore/turbine_rigid.json").generic_string(),
                                                 system_core);
    auto& turbine = *system_core.turbines[0];

    auto controller = std::make_shared<seahowl::servo::ControllerDISCON>(
        (DATADIR / "IEA15MW/base/controller/DISCON_yaw.IN").generic_string(),
        (DATADIR / "IEA15MW/base/controller/libdiscon.so").generic_string());
    turbine.controller = controller;

    turbine.rna.elasto.actuator_yaw->set_fixed_actuator(true);
    turbine.rna.elasto.apply_yaw_increment(-0.1);
    system_elasto.do_statics(true, 10);

    setup_wind(system_core, turbine, seahowl::Vector3d(12.0, 1.0, 0.0));
    simulation.initialize();

    TestFrameworkDataset test_dataset({false, (ref_dir / "test_yaw_control_snap.csv").generic_string(),
                                       (test_dir / "test_yaw_control_snap.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time (s)", [&system_elasto]() { return system_elasto.get_time(); });
    test_dataset.test_csv.add_function("yaw (rad)", [&turbine]() { return turbine.rna.elasto.get_yaw(); });
    test_dataset.test_csv.add_function("towerbase moment (N)",
                                       [&turbine]() { return turbine.tower.elasto.get_tower_base_moment(); });

    run_simulation_loop(simulation, test_dataset);
    EvaluateTest(test_dataset);
};

TEST_F(TestController, IEA15) {
    std::vector<std::pair<double, double>> time_speed_vector = {{0.0, 8.0},    {150.0, 10.0}, {200.0, 12.0},
                                                                {250.0, 15.0}, {300.0, 20.0}, {350.0, 25.0}};

    auto simulation = create_simulation(0.05, 400.0);
    auto& system_core = *simulation.system_core;
    auto& system_elasto = system_core.elasto;

    seahowl::io::add_turbine_to_system_from_file((DATADIR / "IEA15MW/onshore/turbine.json").generic_string(),
                                                 system_core);
    auto& turbine = *system_core.turbines[0];
    system_elasto.do_statics(true, 10);

    auto wind_model = setup_wind(system_core, turbine, seahowl::Vector3d(time_speed_vector[0].second, 0.0, 0.0));
    simulation.initialize();

    TestFrameworkDataset test_dataset({false, (ref_dir / "test_controller.csv").generic_string(),
                                       (test_dir / "test_controller.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time (s)", [&system_elasto]() { return system_elasto.get_time(); });
    test_dataset.test_csv.add_function("power (W)", [&turbine]() { return turbine.get_generated_power(); });
    for (size_t idx_blade = 0; idx_blade < turbine.rna.elasto.rotor->blades.size(); idx_blade++) {
        test_dataset.test_csv.add_function(
            "pitch blade" + std::to_string(idx_blade + 1) + " (rad)",
            [&turbine, idx_blade]() { return turbine.rna.elasto.rotor->blades[idx_blade]->get_pitch(); });
    }

    test_dataset.test_csv.write_row();

    size_t istep = 0;
    while (system_core.get_time() < simulation.duration) {
        for (auto& time_speed_pair : time_speed_vector) {
            if (istep == (int)round(time_speed_pair.first / simulation.dt)) {
                auto wind_speed = time_speed_pair.second;
                spdlog::info("Setting wind speed to {}m/s after {:.2f}s simulation time.", wind_speed,
                             round(system_core.get_time()));
                wind_model->set_wind_velocity(seahowl::Vector3d(wind_speed, 0.0, 0.0));
            }
        }
        simulation.step();
        test_dataset.test_csv.write_row();
        istep += 1;
    }

    EvaluateTest(test_dataset);
}

TEST_F(TestController, discon_50turbines) {
    auto simulation = create_simulation(0.05, 0.1);
    auto& system_core = *simulation.system_core;
    auto& system_elasto = system_core.elasto;

    for (int ii = 0; ii < 50; ii++) {
        seahowl::io::add_turbine_to_system_from_file((DATADIR / "IEA15MW/onshore/turbine_rigid.json").generic_string(),
                                                     system_core);
        system_core.turbines.back()->elasto.translate(seahowl::Vector3d(200.0 * ii, 200.0 * ii, 0.0));
    }
    system_elasto.do_statics(true, 10);

    setup_wind(system_core, *system_core.turbines[0], seahowl::Vector3d(12.0, 0.0, 0.0));
    simulation.initialize();

    while (system_core.get_time() < simulation.duration) {
        simulation.step();
    }
    // this test will throw an error if discon libraries cannot be loaded.
}

TEST_F(TestController, actuator_disk) {
    auto simulation = create_simulation(0.05, 350.0);
    auto& system_core = *simulation.system_core;
    auto& system_elasto = system_core.elasto;

    seahowl::io::add_turbine_to_system_from_file((DATADIR / "IEA15MW/onshore/turbine_disk.json").generic_string(),
                                                 system_core);
    auto& turbine = *system_core.turbines[0];
    system_elasto.do_statics(true, 10);

    auto wind_model = std::make_shared<seahowl::env::WindRamp>();
    wind_model->set_wind_ramp(Vector3d(5.0, 0.0, 0.0), 0.0, Vector3d(15.0, 0.0, 0.0), 250.0);
    wind_model->shear_coefficient = 0.12;
    system_core.env_model->add_model(wind_model);

    simulation.initialize();

    TestFrameworkDataset test_dataset({false, (ref_dir / "test_controller_actuator_disk.csv").generic_string(),
                                       (test_dir / "test_controller_actuator_disk.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time (s)", [&system_elasto]() { return system_elasto.get_time(); });
    test_dataset.test_csv.add_function("rpm (-)", [&turbine]() { return turbine.rna.elasto.get_rpm(); });
    test_dataset.test_csv.add_function("power (W)", [&turbine]() { return turbine.get_generated_power(); });
    test_dataset.test_csv.add_function("pitch (rad)",
                                       [&turbine]() { return turbine.rna.elasto.rotor->pitch_collective; });

    run_simulation_loop(simulation, test_dataset);
    EvaluateTest(test_dataset);
}
