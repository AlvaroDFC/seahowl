#include <gtest/gtest.h>
#include <cmath>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "../../src/elasto/blade_elasto.h"
#include "../../src/elasto/rotor.h"
#include "../../src/elasto/tower.h"
#include "../../src/io/read_json.h"

using namespace chrono;

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST(test_blade, mass_deflection) {
    // system
    ChSystemSMC system;
    system.Set_G_acc(ChVector<double>(0.0, -9.81, 0.0));
    auto solver = chrono_types::make_shared<ChSolverSparseLU>();
    system.SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);

    // mesh for blade
    auto blades_mesh = chrono_types::make_shared<ChMesh>();
    system.AddMesh(blades_mesh);
    // blade
    auto blade_core = get_blade_from_json("../../data/IEA15MW_blade.json");
    std::vector<double> fractions;
    fractions.clear();
    blade_core.set_discretization_elasto(fractions);
    blade_core.build(system, blades_mesh);
    auto blade = blade_core.elasto;
    blade->nodes[0]->SetFixed(true);

    system.Setup();
    system.DoStaticLinear();

    // check mass
    double blade_mass = 67058.294688;
    ASSERT_NEAR(blade_mass, blade->get_mass(), 1.0);

    // check deflection from gravity (edge)
    double deflection_edge = -1.4705;
    system.DoStaticLinear();
    ASSERT_NEAR(deflection_edge, blade->nodes.back()->GetPos().y(), 0.001);

    // check deflection from gravity (flap)
    double deflection_flap = 1.6295;
    blade->rotate(-CH_C_PI / 2.0, VECT_Z);
    system.DoStaticLinear();
    ASSERT_NEAR(deflection_flap, blade->nodes.back()->GetPos().y(), 0.001);
}

TEST(test_rotor, mass) {
    // system
    ChSystemSMC system;
    system.Set_G_acc(ChVector<double>(0.0, -9.81, 0.0));
    auto solver = chrono_types::make_shared<ChSolverSparseLU>();
    system.SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);

    // check mass with blades
    auto blades_mesh = chrono_types::make_shared<ChMesh>();
    system.AddMesh(blades_mesh);
    std::vector<std::shared_ptr<BladeElasto>> blades;
    for (int ii = 0; ii < 3; ii++) {
        auto blade_core = get_blade_from_json("../../data/IEA15MW_blade.json");
        std::vector<double> fractions;
        fractions.clear();
        blade_core.set_discretization_elasto(fractions);
        blade_core.build(system, blades_mesh);
        auto blade = blade_core.elasto;
        blades.push_back(blade);
    }

    auto rotor = get_rotor_from_json("../../data/IEA15MW_RNA.json");
    rotor.build(system, blades);
    rotor.body_yaw_bearing->SetBodyFixed(true);

    system.Setup();
    system.DoStaticLinear();
    // check mass
    double rotor_total_mass = 945710.88406;
    ASSERT_NEAR(rotor_total_mass, rotor.get_mass(), 1.0);
}

TEST(test_tower, mass) {
    // system
    ChSystemSMC system;
    system.Set_G_acc(ChVector<double>(0.0, -9.81, 0.0));
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    system.SetSolver(solver);
    solver->EnableDiagonalPreconditioner(true);
    solver->EnableWarmStart(true);
    solver->SetMaxIterations(40000);
    solver->SetTolerance(1e-12);

    // mesh for tower
    auto tower_mesh = chrono_types::make_shared<ChMesh>();
    system.AddMesh(tower_mesh);
    // tower
    auto tower = get_tower_from_json("../../data/IEA15MW_tower.json");
    tower.build(tower_mesh);

    system.Setup();
    system.DoStaticLinear();

    // check mass
    double tower_mass = 1467613.86;
    ASSERT_NEAR(tower_mass, tower.get_mass(), 1.0);
}

TEST(test_blade, natural_period_dynamic_edge) {
    // system
    ChSystemSMC system;
    system.Set_G_acc(ChVector<double>(0.0, -9.81, 0.0));
    auto solver = chrono_types::make_shared<ChSolverSparseLU>();
    system.SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);

    // mesh for blade
    auto blades_mesh = chrono_types::make_shared<ChMesh>();
    system.AddMesh(blades_mesh);
    // blade
    auto blade_core = get_blade_from_json("../../data/IEA15MW_blade.json");
    std::vector<double> fractions;
    fractions.clear();
    blade_core.set_discretization_elasto(fractions);
    blade_core.build(system, blades_mesh);
    auto blade = blade_core.elasto;
    blade->nodes[0]->SetFixed(true);

    system.Setup();
    system.DoStaticLinear();

    // static position of blade tip
    double pos0 = blade->nodes.back()->GetPos().y();

    // check zero-crossings (static position of blade tip)
    int step = 0;
    double pos_y = 0.0;
    double dt = 0.02;
    int npeaks = 0;
    double natural_period = 0.0;
    double time = 0.0;
    double end_time = 10.0;
    double start_time = 0.0;
    blade->nodes.back()->SetForce(ChVector<double>(0.0, 1000.0, 0.0));
    while (time < end_time) {
        if (time > 0.5) {
            blade->nodes.back()->SetForce(ChVector<double>(0.0, 0.0, 0.0));
            if (blade->nodes.back()->GetPos().y() < pos0 && pos_y > pos0) {
                if (start_time == 0.0) {
                    start_time = time;
                } else {
                    npeaks += 1;
                    natural_period = (time - start_time) / npeaks;
                }
            }
        }
        pos_y = blade->nodes.back()->GetPos().y();
        system.DoStepDynamics(dt);
        time += dt;
        step += 1;
    }

    // literature edgewise natural frequency for IEA15MW: 0.642Hz (1.558s)
    double natural_period_ref = 1.35;
    ASSERT_NEAR(natural_period_ref, natural_period, 0.01);
}

TEST(test_blade, natural_period_dynamic_flap) {
    // system
    ChSystemSMC system;
    system.Set_G_acc(ChVector<double>(0.0, -9.81, 0.0));
    auto solver = chrono_types::make_shared<ChSolverSparseLU>();
    system.SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);

    // mesh for blade
    auto blades_mesh = chrono_types::make_shared<ChMesh>();
    system.AddMesh(blades_mesh);
    // blade
    auto blade_core = get_blade_from_json("../../data/IEA15MW_blade.json");
    std::vector<double> fractions;
    fractions.clear();
    blade_core.set_discretization_elasto(fractions);
    blade_core.build(system, blades_mesh);
    auto blade = blade_core.elasto;
    blade->nodes[0]->SetFixed(true);

    // rotate blade for flap
    blade->rotate(-CH_C_PI / 2.0, VECT_Z);

    system.Setup();
    system.DoStaticLinear();

    // static position of blade tip
    double pos0 = blade->nodes.back()->GetPos().y();

    // check zero-crossings (static position of blade tip)
    int step = 0;
    double pos_y = 0.0;
    double dt = 0.02;
    int npeaks = 0;
    double natural_period = 0.0;
    double time = 0.0;
    double end_time = 10.0;
    double start_time = 0.0;
    blade->nodes.back()->SetForce(ChVector<double>(0.0, 1000.0, 0.0));
    while (time < end_time) {
        if (time > 0.5) {
            blade->nodes.back()->SetForce(ChVector<double>(0.0, 0.0, 0.0));
            if (blade->nodes.back()->GetPos().y() < pos0 && pos_y > pos0) {
                if (start_time == 0.0) {
                    start_time = time;
                } else {
                    npeaks += 1;
                    natural_period = (time - start_time) / npeaks;
                }
            }
        }
        pos_y = blade->nodes.back()->GetPos().y();
        system.DoStepDynamics(dt);
        time += dt;
        step += 1;
    }

    // literature flapwise natural frequency for IEA15MW: 0.555Hz (1.802s)
    double natural_period_ref = 1.92;
    ASSERT_NEAR(natural_period_ref, natural_period, 0.01);
}
