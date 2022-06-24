#include "chrono/fea/ChVisualizationFEAmesh.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include <cmath>

#include "blade.h"
#include "read_json.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace irr;

int main(int argc, char* argv[]) {
    // SETUP

    // system
    ChSystemSMC system;
    system.SetNumThreads(ChOMP::GetNumProcs(), 0, 1);

    // solver
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    system.SetSolver(solver);
    solver->SetMaxIterations(4000);
    solver->SetTolerance(1e-12);
    solver->EnableDiagonalPreconditioner(true);
    solver->EnableWarmStart(true);
    system.SetSolverForceTolerance(1e-10);

    // mesh for blade
    auto blades_mesh = chrono_types::make_shared<ChMesh>();
    system.AddMesh(blades_mesh);

    // blade
    auto reference_points = blade_reference_points_from_json("../data/IEA15MW_blade.json");
    auto blade = Blade();
    blade.reference_points = reference_points;

    blade.make_blade(blades_mesh);

    // add point to fix root node of blade
    auto rotor_center = chrono_types::make_shared<ChBodyEasyBox>(0.2, 0.2, 0.2, 2700, true, true);
    system.Add(rotor_center);
    rotor_center->SetPos(ChVector<>(0, 0, -1));
    rotor_center->SetBodyFixed(true);
    // link between rotor and root node
    auto link_pos = chrono_types::make_shared<ChLinkMateFix>();
    link_pos->Initialize(blade.nodes[0], rotor_center);
    system.Add(link_pos);

    // VISUALIZATION

    // make visualization app
    ChIrrApp application(&system, L"Blade", core::dimension2d<u32>(800, 600), VerticalDir::Y, false, true);
    application.AddTypicalLights();
    application.AddTypicalSky();
    application.AddTypicalCamera(core::vector3df(10, 3, -10));

    // visualize beams
    // increase size of visualization for beams
    for (int ii = 0; ii < blade.elements.size(); ii++) {
        blade.elements[ii]->GetTaperedSection()->GetSectionA()->SetDrawThickness(0.1, 0.1);
        blade.elements[ii]->GetTaperedSection()->GetSectionB()->SetDrawThickness(0.1, 0.1);
    }
    auto visualize_beam = chrono_types::make_shared<ChVisualizationFEAmesh>(*(blades_mesh.get()));
    visualize_beam->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ELEM_BEAM_MZ);
    visualize_beam->SetColorscaleMinMax(-0.4, 0.4);
    blades_mesh->AddAsset(visualize_beam);

    // visualize nodes
    auto visualize_nodes = chrono_types::make_shared<ChVisualizationFEAmesh>(*(blades_mesh.get()));
    visualize_nodes->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);
    visualize_nodes->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NODE_DISP_Y);
    visualize_nodes->SetSymbolsThickness(0.2);
    visualize_nodes->SetSymbolsScale(1.0);
    visualize_nodes->SetZbufferHide(false);
    blades_mesh->AddAsset(visualize_nodes);

    // visualize node coordinate systems
    auto visualize_nodes_coordsys = chrono_types::make_shared<ChVisualizationFEAmesh>(*(blades_mesh.get()));
    visualize_nodes_coordsys->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS);
    visualize_nodes_coordsys->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    visualize_nodes_coordsys->SetSymbolsThickness(1.0);
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
                 << " pos: " << blade.nodes[blade.nodes.size() - 1]->GetPos().y() << "\n";
    }

    return 0;
}
