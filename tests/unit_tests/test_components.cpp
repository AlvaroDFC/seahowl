#include <gtest/gtest.h>
#include <cmath>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"

#include "../../src/blade.h"
#include "../../src/rotor.h"
#include "../../src/tower.h"
#include "../../src/read_json.h"

using namespace chrono;

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST(test_blade, mass_deflection) {
    // system
    ChSystemSMC system;
    system.Set_G_acc(ChVector<double>(0.0, -9.81, 0.0));
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    system.SetSolver(solver);
    solver->EnableDiagonalPreconditioner(true);
    solver->EnableWarmStart(true);
    solver->SetMaxIterations(40000);
    solver->SetTolerance(1e-12);

    // mesh for blade
    auto blades_mesh = chrono_types::make_shared<ChMesh>();
    system.AddMesh(blades_mesh);
    // blade
    auto blade = get_blade_from_json("../../data/IEA15MW_blade.json");
    blade.discretization_fractions.clear();
    blade.build(system, blades_mesh);
    blade.nodes[0]->SetFixed(true);

    system.Setup();
    system.DoStaticLinear();

    // check mass
    double blade_mass = 68537.9;
    ASSERT_NEAR(blade_mass, blade.get_mass(), 1.0);

    // check deflection from gravity (edge)
    double deflection_edge = -2.17637;
    system.DoStaticLinear();
    ASSERT_NEAR(deflection_edge, blade.nodes[blade.nodes.size() - 1]->GetPos().y(), 0.001);

    // check deflection from gravity (flap)
    double deflection_flap = -4.96310;
    blade.rotate(CH_C_PI / 2.0, VECT_Z);
    system.DoStaticLinear();
    ASSERT_NEAR(deflection_flap, blade.nodes[blade.nodes.size() - 1]->GetPos().y(), 0.001);
}

TEST(test_rotor, mass) {
    // system
    ChSystemSMC system;
    system.Set_G_acc(ChVector<double>(0.0, -9.81, 0.0));
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    system.SetSolver(solver);
    solver->EnableDiagonalPreconditioner(true);
    solver->EnableWarmStart(true);
    solver->SetMaxIterations(40000);
    solver->SetTolerance(1e-12);

    // mesh for blade
    auto rotor = get_rotor_from_json("../../data/IEA15MW_RNA.json");
    std::vector<std::shared_ptr<Blade>> blades;
    rotor.build(system, blades);

    // check mass
    double rotor_mass = 744536;
    ASSERT_NEAR(rotor_mass, rotor.get_mass(), 1.0);

    // check mass with blades
    auto blades_mesh = chrono_types::make_shared<ChMesh>();
    system.AddMesh(blades_mesh);
    for (int ii = 0; ii < 3; ii++) {
        auto blade = std::make_shared<Blade>(get_blade_from_json("../../data/IEA15MW_blade.json"));
        blade->discretization_fractions.clear();
        blade->build(system, blades_mesh);
        rotor.blades.push_back(blade);
    }
    system.Setup();
    system.DoStaticLinear();
    // check mass
    double rotor_total_mass = 950149.6;
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
