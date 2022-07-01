#include "chrono/fea/ChVisualizationFEAmesh.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include <cmath>

#include "blade.h"
#include "rotor.h"
#include "read_json.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace irr;

int main(int argc, char* argv[]) {
    // SETUP

    // system
    ChSystemSMC system;
    system.Set_G_acc(ChVector<double>(0.0, 0.0, -9.81));
    system.SetNumThreads(ChOMP::GetNumProcs(), 0, 1);

    // solver
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    system.SetSolver(solver);
    solver->SetMaxIterations(4000);
    solver->SetVerbose(true);
    solver->SetTolerance(1e-12);
    solver->EnableDiagonalPreconditioner(true);
    solver->EnableWarmStart(true);
    system.SetSolverForceTolerance(1e-10);

    // mesh for blade
    auto blades_mesh = chrono_types::make_shared<ChMesh>();
    system.AddMesh(blades_mesh);

    // blade
    GetLog() << "Building blades\n";
    auto blade1 = get_blade_from_json("../data/IEA15MW_blade.json");
    auto blade2 = get_blade_from_json("../data/IEA15MW_blade.json");
    auto blade3 = get_blade_from_json("../data/IEA15MW_blade.json");

    std::vector<Blade*> blades{&blade1, &blade2, &blade3};

    for (int ii = 0; ii < blades.size(); ii++) {
        auto blade = blades[ii];
        blade->make_blade(blades_mesh);
        for (int jj = 0; jj < blade->elements.size(); jj++) {
            blade->elements[jj]->GetTaperedSection()->GetSectionA()->SetDrawThickness(0.5, 0.5);
            blade->elements[jj]->GetTaperedSection()->GetSectionB()->SetDrawThickness(0.5, 0.5);
        }
    }

    // rotor
    GetLog() << "Building rotor\n";
    auto rotor = get_rotor_from_json("../data/IEA15MW_RNA.json");
    rotor.build(system, blades);
    // fix body shaft hub
    rotor.body_shaft_hub->SetBodyFixed(true);

    auto link_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    link_motor->Initialize(rotor.body_shaft_hub, rotor.body_hub_apex, rotor.body_shaft_hub->GetAssetsFrame());
    system.AddLink(link_motor);
    auto my_speed_function = chrono_types::make_shared<ChFunction_Ramp>(0.0, CH_C_PI / 100.);
    link_motor->SetSpeedFunction(my_speed_function);
    link_motor->SetDisabled(false);

    // tower
    GetLog() << "Building tower\n";
    auto tower = get_tower_from_json("../data/IEA15MW_tower.json");
    tower.build(blades_mesh);
    for (int jj = 0; jj < tower.elements.size(); jj++) {
        tower.elements[jj]->GetTaperedSection()->GetSectionA()->SetDrawThickness(0.5, 0.5);
        tower.elements[jj]->GetTaperedSection()->GetSectionB()->SetDrawThickness(0.5, 0.5);
    }
    // translate tower to make it match the current turbine configuration
    tower.translate(ChVector<double>(0.0, 0.0, -tower.height - rotor.shaft.distance_from_towertop));
    // fix bottom of tower
    tower.nodes[0]->SetFixed(true);

    GetLog() << "Finished building system\n";

    // VISUALIZATION

    // make visualization app
    ChIrrApp application(&system, L"Blade", core::dimension2d<u32>(800, 600), VerticalDir::Y, false, true);
    application.AddTypicalLights();
    application.AddTypicalSky();
    application.AddTypicalCamera(core::vector3df(-100, -50, 3));

    auto visualize_beam = chrono_types::make_shared<ChVisualizationFEAmesh>(*(blades_mesh.get()));
    visualize_beam->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_MZ);
    visualize_beam->SetColorscaleMinMax(-0.4, 0.4);
    blades_mesh->AddAsset(visualize_beam);

    // visualize nodes
    auto visualize_nodes = chrono_types::make_shared<ChVisualizationFEAmesh>(*(blades_mesh.get()));
    visualize_nodes->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    visualize_nodes->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_DISP_Y);
    visualize_nodes->SetSymbolsThickness(1.0);
    visualize_nodes->SetSymbolsScale(1.0);
    visualize_nodes->SetZbufferHide(false);
    blades_mesh->AddAsset(visualize_nodes);

    // visualize node coordinate systems
    auto visualize_nodes_coordsys = chrono_types::make_shared<ChVisualizationFEAmesh>(*(blades_mesh.get()));
    visualize_nodes_coordsys->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
    visualize_nodes_coordsys->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    visualize_nodes_coordsys->SetSymbolsThickness(10.0);
    visualize_nodes_coordsys->SetSymbolsScale(1.0);
    visualize_nodes_coordsys->SetZbufferHide(false);
    blades_mesh->AddAsset(visualize_nodes_coordsys);

    // needed for visulization after setting everything up
    application.AssetBindAll();
    application.AssetUpdateAll();
    application.AddShadowAll();

    // SIMULATION LOOP

    application.SetTimestep(0.01);
    application.SetVideoframeSave(false);
    application.SetVideoframeSaveInterval(20);
    double time = 0.0;
    int step = 0;
    application.DoStep();
    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        application.DoStep();
        application.EndScene();
        time += system.GetStep();
        step += 1;
        GetLog() << "time " << time << " step: " << step
                 << " pos: " << blade1.nodes[blade1.nodes.size() - 1]->GetPos().y() << "\n";
    }

    return 0;
}
