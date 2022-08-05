#include "chrono/fea/ChVisualizationFEAmesh.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include <cmath>

#include <seahowl/io/read_json.h>
#include <seahowl/servo/controller.h>
#include <seahowl/elasto/blade_elasto.h>

using namespace chrono;
using namespace chrono::irrlicht;
using namespace irr;

/**@brief Driver main function */
int main(int argc, char* argv[]) {
    // SETUP

    // general options
    bool visualization_on = true;
    bool statics_prestep = true;
    // solver
    auto solver_type = ChSolver::Type::SPARSE_LU;
    auto verbose = false;
    // timestepping
    auto timestepper_type = ChTimestepper::Type::HHT;
    double dt = 0.1;
    // wind
    auto wind_model = ConstantWind();
    wind_model.set_wind_velocity(ChVector<double>(8.0, 0.0, 0.0));
    // turbine
    double initial_pitch = CH_C_PI / 8.0;
    // target RPM for simple generator control
    // set to 0.0 for no control
    auto controller = ControllerVariableTorque();
    controller.target_rpm = 0.0;

    // system
    ChSystemSMC system;
    system.Set_G_acc(ChVector<double>(0.0, -9.81, 0.0));
    system.SetNumThreads(ChOMP::GetNumProcs(), 0, 1);

    switch (solver_type) {
        case ChSolver::Type::SPARSE_QR: {
            std::cout << "Using SparseQR solver" << std::endl;
            auto solver = chrono_types::make_shared<ChSolverSparseQR>();
            system.SetSolver(solver);
            solver->UseSparsityPatternLearner(true);
            solver->LockSparsityPattern(true);
            solver->SetVerbose(verbose);
            break;
        }
        case ChSolver::Type::SPARSE_LU: {
            std::cout << "Using SparseLU solver" << std::endl;
            auto solver = chrono_types::make_shared<ChSolverSparseLU>();
            system.SetSolver(solver);
            solver->UseSparsityPatternLearner(true);
            solver->LockSparsityPattern(true);
            solver->SetVerbose(verbose);
            break;
        }
        case ChSolver::Type::MINRES: {
            std::cout << "Using MINRES solver" << std::endl;
            auto solver = chrono_types::make_shared<ChSolverMINRES>();
            system.SetSolver(solver);
            solver->SetMaxIterations(40000);
            solver->SetTolerance(1e-5);
            solver->EnableDiagonalPreconditioner(true);
            solver->EnableWarmStart(true);  // IMPORTANT for convergence when using EULER_IMPLICIT_LINEARIZED
            solver->SetVerbose(verbose);
            break;
        }
    }
    system.SetTimestepperType(timestepper_type);
    if (auto mystepper = std::dynamic_pointer_cast<ChTimestepperHHT>(system.GetTimestepper())) {
        mystepper->SetStepControl(false);
        mystepper->SetModifiedNewton(false);
        // mystepper->SetAlpha(-0.5);
        // GetLog() << mystepper->GetAlpha() << "\n";
    }

    // mesh for blade
    auto blades_mesh = chrono_types::make_shared<chrono::fea::ChMesh>();
    system.AddMesh(blades_mesh);

    std::vector<std::string> blades_files = {"../data/IEA15MW_blade.json", "../data/IEA15MW_blade.json",
                                             "../data/IEA15MW_blade.json"};
    auto rotor_file = "../data/IEA15MW_RNA.json";
    auto tower_file = "../data/IEA15MW_tower.json";

    std::vector<std::shared_ptr<seahowl::core::Turbine>> turbines;
    int nturbines = 1;
    for (int ii = 0; ii < nturbines; ii++) {
        auto turbine = std::make_shared<seahowl::core::Turbine>(
            get_turbine_from_json(blades_files, rotor_file, tower_file));
        // clear discretization defined in file
        for (auto& blade: turbine->m_blades) {
            blade->m_elasto->discretization_fractions.clear();
            blade->m_aero->discretization_fractions.clear();
        }
        turbine->build(system, blades_mesh);
        turbine->m_tower.nodes[0]->SetFixed(true);
        turbines.push_back(turbine);

        // increase elements for visualization
        for (auto& bladei: turbine->m_blades) {
            auto blade = bladei->m_elasto;
            for (auto& elm: blade->elements) {
                elm->GetTaperedSection()->GetSectionA()->SetDrawThickness(2.0, 0.5);
                elm->GetTaperedSection()->GetSectionB()->SetDrawThickness(2.0, 0.5);
            }
        }
        // increase elements for visualization
        auto& tower = turbine->m_tower;
        for (auto& elm:  tower.elements) {
            elm->GetTaperedSection()->GetSectionA()->SetDrawThickness(3.0, 3.0);
            elm->GetTaperedSection()->GetSectionB()->SetDrawThickness(3.0, 3.0);
        }

        turbine->translate(ChVector<double>(150.0 * ii, 150.0 * ii * pow(-1.0, ii), -150.0));
        turbine->rotate(-CH_C_PI / 2.0, VECT_X);
    }

    // VISUALIZATION

    ChIrrApp application(&system, L"Blade", core::dimension2d<u32>(800, 600), VerticalDir::Y, false, true);
    if (visualization_on) {
        // make visualization app
        application.AddTypicalLights();
        // application.AddTypicalSky();
        application.AddTypicalCamera(core::vector3df(-300, 150, -50));

        auto visualize_beam = chrono_types::make_shared<chrono::fea::ChVisualizationFEAmesh>(*(blades_mesh.get()));
        visualize_beam->SetFEMdataType(chrono::fea::ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_MZ);
        visualize_beam->SetColorscaleMinMax(-0.4, 0.4);
        blades_mesh->AddAsset(visualize_beam);

        // visualize nodes
        auto visualize_nodes = chrono_types::make_shared<chrono::fea::ChVisualizationFEAmesh>(*(blades_mesh.get()));
        visualize_nodes->SetFEMglyphType(chrono::fea::ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
        visualize_nodes->SetFEMdataType(chrono::fea::ChVisualizationFEAmesh::E_PLOT_NODE_DISP_Y);
        visualize_nodes->SetSymbolsThickness(1.0);
        visualize_nodes->SetSymbolsScale(1.0);
        visualize_nodes->SetZbufferHide(false);
        blades_mesh->AddAsset(visualize_nodes);

        // visualize node coordinate systems
        auto visualize_nodes_coordsys = chrono_types::make_shared<chrono::fea::ChVisualizationFEAmesh>(*(blades_mesh.get()));
        visualize_nodes_coordsys->SetFEMglyphType(chrono::fea::ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
        visualize_nodes_coordsys->SetFEMdataType(chrono::fea::ChVisualizationFEAmesh::E_PLOT_NONE);
        visualize_nodes_coordsys->SetSymbolsThickness(10.0);
        visualize_nodes_coordsys->SetSymbolsScale(1.0);
        visualize_nodes_coordsys->SetZbufferHide(false);
        blades_mesh->AddAsset(visualize_nodes_coordsys);

        // needed for visulization after setting everything up
        application.AssetBindAll();
        application.AssetUpdateAll();
        application.AddShadowAll();
    }

    // SIMULATION LOOP

    if (visualization_on) {
        application.SetTimestep(dt);
        application.SetVideoframeSave(false);
        application.SetVideoframeSaveInterval(20);
    }

    // simulation loop
    double time = 0.0;
    int step = 0;
    // initialization

    for (auto& turbine: turbines) {
        turbine->m_rotor.elasto.apply_collective_pitch_increment(initial_pitch);
        turbine->prestep(time);
        turbine->poststep(time);
    }

    double torque_aero = 0.0;
    double average_torque_aero = 0.0;
    double torque_elec = 0.0;
    double average_rpm = 0.0;
    // statics
    if (statics_prestep) {
        system.DoStaticLinear();
        system.DoStaticNonlinear(10, true);
    }
    // while (application.GetDevice()->run()) {
    while (true) {
        // prestep
        for (auto& turbine: turbines) {
            
            // compute forces
            turbine->m_rotor.aero.compute_wind_loads_bemt(wind_model, time);
            // prestep (accumulates loads from aero to elasto)
            turbine->prestep(time);
        }

        // step
        if (visualization_on) {
            // this should be in while(...) loop, but it is here to allow no visualization at all
            application.GetDevice()->run();

            application.BeginScene();
            application.DrawAll();
            application.DoStep();
            application.EndScene();
        } else {
            system.DoStepDynamics(dt);
        }
        time += system.GetStep();
        step += 1;
        GetLog() << "time " << time << " step: " << step << " rpm: " << turbines[0]->m_rotor.elasto.get_rpm()
                 << " average rpm: " << average_rpm << "\n";

        // poststep
        for (auto& turbine:  turbines) {

            turbine->poststep(time);

            if (controller.target_rpm > 0.0) {
                // get torque elec from controller
                double rpm = turbine->m_rotor.elasto.get_rpm();
                double torque_total = turbine->m_rotor.elasto.get_torque();
                double torque_elec = controller.get_torque_elec(torque_total, rpm);
                // apply torque elec to hub rigid body
                turbine->m_rotor.elasto.body_hub->Empty_forces_accumulators();
                // torque elec is apply on Z axis of hub body (locally)
                turbine->m_rotor.elasto.body_hub->Accumulate_torque(ChVector<double>(0.0, 0.0, torque_elec), true);
            }
        }
    }

    return 0;
}
