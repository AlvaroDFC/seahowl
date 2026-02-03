// Local test headers
#include "fixture_components.h"

// SEAHOWL headers
#include <seahowl/core.h>
#include <seahowl/elasto.h>
#include <seahowl/env.h>
#include <seahowl/fluid.h>
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
class TestTurbine : public FixtureComponents {
  protected:
    TestTurbine() : FixtureComponents() {
        ref_dir /= "test_turbine/ref";
        test_dir /= "test_turbine/test";
    }

    // Shared state
    std::shared_ptr<seahowl::env::ConstantWind> wind_model;
    seahowl::env::EnvModel env_model;
    SystemElastoChrono system_elasto;
    double time = 0.0;
    double dt = 0.1;

    // Setup constant wind environment
    void setup_wind(const Vector3d& velocity, double shear_coefficient = 0.12) {
        wind_model = std::make_shared<seahowl::env::ConstantWind>();
        wind_model->set_wind_velocity(velocity);
        wind_model->shear_coefficient = shear_coefficient;
        env_model.add_model(wind_model);
    }

    // Load turbine from file and optionally replace controller with empty one
    seahowl::core::Turbine load_turbine(const std::string& turbine_file) {
        auto turbine = seahowl::io::get_turbine_from_file((DATADIR / turbine_file).generic_string());
        // remove controller
        turbine.controller = std::make_shared<seahowl::servo::Controller>();
        return turbine;
    }

    // Build and initialize turbine
    void init_turbine(seahowl::core::Turbine& turbine) {
        turbine.build();
        turbine.elasto.assemble(system_elasto);
        turbine.initialize(time, dt);
    }

    // Run statics prestep with standard constraint pattern
    void do_statics(seahowl::core::Turbine& turbine) {
        turbine.rna.elasto.link_shaft_hub->set_constraints(true, true, true, true, true, true);
        system_elasto.do_statics(true, 10);
        turbine.rna.elasto.link_shaft_hub->set_constraints(true, true, true, false, true, true);
        turbine.poststep(0.0, dt);
    }

    // Add common turbine metrics to test dataset
    void add_common_metrics(TestFrameworkDataset& dataset, seahowl::core::Turbine& turbine) {
        dataset.test_csv.add_function("time (s)", [this]() { return system_elasto.get_time(); });
        dataset.test_csv.add_function("rpm (-)", [&turbine]() { return turbine.rna.elasto.get_rpm(); });
        dataset.test_csv.add_function("axial torque (Nm)",
                                      [&turbine]() { return turbine.rna.elasto.get_axial_torque(); });
        dataset.test_csv.add_function("blade1 root moment (Nm)", [&turbine]() {
            return turbine.rna.elasto.rotor->blades[0]->get_blade_root_moment();
        });
    }

    // Run simulation loop
    void run_simulation(seahowl::core::Turbine& turbine, TestFrameworkDataset& dataset, double duration) {
        while (time < duration) {
            turbine.apply_control(time, dt);
            turbine.fluid.compute_env_loads(env_model, time);
            turbine.prestep(time, dt);
            system_elasto.step(dt);
            time += dt;
            turbine.poststep(time, dt);
            dataset.test_csv.write_row();
        }
    }
};

TEST_F(TestTurbine, IEA15MW_fixed_pitch_fea) {
    setup_wind(Vector3d(8.0, 0.0, 0.0));
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    auto turbine = load_turbine("IEA15MW/onshore/turbine.json");
    // force FEA mode on blades
    for (auto& blade : turbine.elasto.rna->rotor->blades) {
        dynamic_cast<seahowl::elasto::BladeElastoFEA&>(*blade).fpm_mode = false;
    }
    init_turbine(turbine);

    turbine.rna.elasto.rotor->apply_collective_pitch_increment(seahowl::PI / 8.0);
    do_statics(turbine);

    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA15MW_fixed_pitch_fea.csv").generic_string(),
                                       (test_dir / "test_IEA15MW_fixed_pitch_fea.test.csv").generic_string()});
    add_common_metrics(test_dataset, turbine);

    run_simulation(turbine, test_dataset, 50.0);
    EvaluateTest(test_dataset);
}

TEST_F(TestTurbine, IEA15MW_fixed_pitch_fpm) {
    setup_wind(Vector3d(8.0, 0.0, 0.0));
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    auto turbine = load_turbine("IEA15MW/onshore/turbine.json");
    // force FPM mode on blades
    for (auto& blade : turbine.elasto.rna->rotor->blades) {
        dynamic_cast<seahowl::elasto::BladeElastoFEA&>(*blade).fpm_mode = true;
    }
    init_turbine(turbine);

    turbine.rna.elasto.rotor->apply_collective_pitch_increment(seahowl::PI / 8.0);
    do_statics(turbine);

    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA15MW_fixed_pitch_fpm.csv").generic_string(),
                                       (test_dir / "test_IEA15MW_fixed_pitch_fpm.test.csv").generic_string()});
    add_common_metrics(test_dataset, turbine);

    run_simulation(turbine, test_dataset, 50.0);
    EvaluateTest(test_dataset);
}

TEST_F(TestTurbine, IEA15MW_fixed_pitch_rigid) {
    setup_wind(Vector3d(8.0, 0.0, 0.0));
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    auto turbine = load_turbine("IEA15MW/onshore/turbine_rigid.json");
    init_turbine(turbine);

    turbine.rna.elasto.rotor->apply_collective_pitch_increment(seahowl::PI / 8.0);
    do_statics(turbine);

    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA15MW_fixed_pitch_rigid.csv").generic_string(),
                                       (test_dir / "test_IEA15MW_fixed_pitch_rigid.test.csv").generic_string()});
    add_common_metrics(test_dataset, turbine);

    run_simulation(turbine, test_dataset, 50.0);
    EvaluateTest(test_dataset);
}

TEST_F(TestTurbine, IEA15MW_target_rpm) {
    dt = 0.05;
    setup_wind(Vector3d(8.0, 0.0, 0.0));
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    auto turbine = load_turbine("IEA15MW/onshore/turbine_rigid.json");
    auto controller = std::make_shared<seahowl::servo::ControllerVariableTorque>();
    controller->target_rpm = 2.0;
    turbine.controller = controller;
    init_turbine(turbine);

    turbine.rna.elasto.rotor->apply_collective_pitch_increment(seahowl::PI / 8.0);
    do_statics(turbine);

    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA15MW_target_rpm.csv").generic_string(),
                                       (test_dir / "test_IEA15MW_target_rpm.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time (s)", [this]() { return system_elasto.get_time(); });
    test_dataset.test_csv.add_function("rpm (-)", [&turbine]() { return turbine.rna.elasto.get_rpm(); });

    run_simulation(turbine, test_dataset, 100.0);
    EvaluateTest(test_dataset);
}

TEST_F(TestTurbine, IEA15MW_actuator_disk) {
    setup_wind(Vector3d(11.0, 0.0, 0.0));
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    auto turbine = load_turbine("IEA15MW/onshore/turbine_disk.json");
    auto controller = std::make_shared<seahowl::servo::ControllerVariableTorque>();
    controller->target_rpm = 7.56;
    turbine.controller = controller;
    init_turbine(turbine);

    double initial_pitch = 0.0;
    turbine.rna.elasto.rotor->apply_collective_pitch_increment(initial_pitch);
    do_statics(turbine);

    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA15MW_actuator_disk.csv").generic_string(),
                                       (test_dir / "test_IEA15MW_actuator_disk.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time (s)", [this]() { return system_elasto.get_time(); });
    test_dataset.test_csv.add_function("rpm (-)", [&turbine]() { return turbine.rna.elasto.get_rpm(); });
    test_dataset.test_csv.add_function("power (W)", [&turbine]() { return turbine.get_generated_power(); });

    // Phase 1: wind 11 m/s
    int count = 0;
    while (time < 100.0) {
        turbine.apply_control(time, dt);
        turbine.fluid.compute_env_loads(env_model, time);
        // prestep (accumulates loads from aero to elasto)
        turbine.prestep(time, dt);
        system_elasto.step(dt);
        time += dt;
        turbine.poststep(time, dt);
        if (count == 5) {
            test_dataset.test_csv.write_row();
            count = 0;
        }
        count += 1;
    }

    // Phase 2: wind 15 m/s with pitch adjustment
    wind_model->set_wind_velocity(Vector3d(15.0, 0.0, 0.0));
    initial_pitch = 11.0 * seahowl::PI / 180.0;
    turbine.rna.elasto.rotor->apply_collective_pitch_increment(initial_pitch);
    count = 0;
    while (time < 200) {
        turbine.apply_control(time, dt);
        turbine.fluid.compute_env_loads(env_model, time);
        // prestep (accumulates loads from aero to elasto)
        turbine.prestep(time, dt);
        system_elasto.step(dt);
        time += dt;
        turbine.poststep(time, dt);
        if (count == 5) {
            test_dataset.test_csv.write_row();
            count = 0;
        }
        count += 1;
    }

    EvaluateTest(test_dataset);
}

TEST_F(TestTurbine, IEA15MW_multiturbines) {
    setup_wind(Vector3d(8.0, 0.0, 0.0));

    auto system_elasto_ptr = std::make_shared<SystemElastoChrono>();
    system_elasto_ptr->set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    auto system_core = seahowl::core::System(system_elasto_ptr, std::make_shared<seahowl::fluid::SystemFluid>());
    system_core.env_model->add_model(wind_model);

    constexpr int nturbines = 3;
    for (int ii = 0; ii < nturbines; ii++) {
        auto turbine = load_turbine("IEA15MW/onshore/turbine_rigid.json");
        turbine.build();
        turbine.elasto.translate(Vector3d(ii * 150.0, ii * (-150.0), 0.0));
        system_core.turbines.push_back(std::make_shared<seahowl::core::Turbine>(turbine));
        system_core.elasto.turbines.push_back(std::make_shared<seahowl::elasto::TurbineElasto>(turbine.elasto));
        system_core.fluid.turbines.push_back(std::make_shared<seahowl::fluid::TurbineFluid>(turbine.fluid));
    }

    system_core.initialize(time, dt);

    for (auto& turbine : system_core.turbines) {
        turbine->rna.elasto.rotor->apply_collective_pitch_increment(seahowl::PI / 8.0);
    }

    system_elasto_ptr->do_statics(true, 10);
    system_core.poststep(0.0, dt);

    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA15MW_multiturbines.csv").generic_string(),
                                       (test_dir / "test_IEA15MW_multiturbines.test.csv").generic_string()});
    test_dataset.test_csv.add_function("time (s)", [&system_elasto_ptr]() { return system_elasto_ptr->get_time(); });
    for (size_t idx = 0; idx < system_core.turbines.size(); idx++) {
        test_dataset.test_csv.add_function("rpm turbine " + std::to_string(idx + 1) + " (-)", [&system_core, idx]() {
            return system_core.turbines[idx]->rna.elasto.get_rpm();
        });
    }

    while (time < 50.0) {
        system_core.prestep(time, dt);
        system_core.step(dt);
        time += dt;
        system_core.poststep(time, dt);
        test_dataset.test_csv.write_row();
    }

    EvaluateTest(test_dataset);
}

TEST_F(TestTurbine, IEA34MW_fixed_pitch) {
    setup_wind(Vector3d(8.0, 0.0, 0.0));
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    auto turbine = load_turbine("IEA34MW/turbine.json");
    // force FPM mode on blades
    for (auto& blade : turbine.elasto.rna->rotor->blades) {
        dynamic_cast<seahowl::elasto::BladeElastoFEA&>(*blade).fpm_mode = true;
    }
    init_turbine(turbine);

    turbine.rna.elasto.rotor->apply_collective_pitch_increment(seahowl::PI / 8.0);
    do_statics(turbine);

    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA34MW_fixed_pitch.csv").generic_string(),
                                       (test_dir / "test_IEA34MW_fixed_pitch.test.csv").generic_string()});
    add_common_metrics(test_dataset, turbine);

    run_simulation(turbine, test_dataset, 50.0);
    EvaluateTest(test_dataset);
}

TEST_F(TestTurbine, IEA10MW_fixed_pitch) {
    setup_wind(Vector3d(8.0, 0.0, 0.0));
    system_elasto.set_gravitational_acceleration(Vector3d(0.0, 0.0, -9.81));

    auto turbine = load_turbine("IEA10MW/turbine.json");
    // force FPM mode on blades
    for (auto& blade : turbine.elasto.rna->rotor->blades) {
        dynamic_cast<seahowl::elasto::BladeElastoFEA&>(*blade).fpm_mode = true;
    }
    init_turbine(turbine);

    turbine.rna.elasto.rotor->apply_collective_pitch_increment(seahowl::PI / 8.0);
    do_statics(turbine);

    TestFrameworkDataset test_dataset({false, (ref_dir / "test_IEA10MW_fixed_pitch.csv").generic_string(),
                                       (test_dir / "test_IEA10MW_fixed_pitch.test.csv").generic_string()});
    add_common_metrics(test_dataset, turbine);

    run_simulation(turbine, test_dataset, 50.0);
    EvaluateTest(test_dataset);
}
