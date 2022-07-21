#pragma warning(push, 0)

#include <chrono/fea/ChVisualizationFEAmesh.h>
#include <chrono/physics/ChBodyEasy.h>
#include <chrono/physics/ChLinkMate.h>
#include <chrono/physics/ChSystemSMC.h>
#include <chrono/solver/ChIterativeSolverLS.h>

#include <chrono/physics/ChLinkMotorRotationSpeed.h>
#include <chrono/solver/ChDirectSolverLS.h>


#pragma warning(pop)

#include <cmath>

#include "elasto/blade_elasto.h"
#include "core/blade_core.h"
#include "elasto/rotor.h"
#include "io/read_json.h"

using namespace chrono;

#ifdef HAVE_IRRLICHT
    #include <chrono_irrlicht/ChIrrApp.h>
using namespace chrono::irrlicht;
using namespace irr;
#endif


int main(int argc, char* argv[]) {
    // SETUP

    // system
    ChSystemSMC system;
    system.Set_G_acc(ChVector<double>(0.0, -9.81, 0.0));
    system.SetNumThreads(ChOMP::GetNumProcs(), 0, 1);

    auto solver = chrono_types::make_shared<ChSolverSparseLU>();
    system.SetSolver(solver);
    solver->UseSparsityPatternLearner(true);
    solver->LockSparsityPattern(true);
    solver->SetVerbose(false);

    // mesh for blade
    auto blades_mesh = chrono_types::make_shared<ChMesh>();
    system.AddMesh(blades_mesh);

    std::vector<std::string> blades_files = {"../data/IEA15MW_blade.json", "../data/IEA15MW_blade.json",
                                             "../data/IEA15MW_blade.json"};
    auto rotor_file = "../data/IEA15MW_RNA.json";
    auto tower_file = "../data/IEA15MW_tower.json";

    std::vector<std::shared_ptr<Turbine>> turbines;
    int nturbines = 1;
    for (int ii = 0; ii < nturbines; ii++) {
        auto turbine = std::make_shared<Turbine>(get_turbine_from_json(blades_files, rotor_file, tower_file));
        // clear discretization defined in file
        for (int jj = 0; jj < turbine->blades.size(); jj++) {
            turbine->blades[jj]->elasto->discretization_fractions.clear();
            turbine->blades[jj]->aero->discretization_fractions.clear();
        }
        turbine->build(system, blades_mesh);
        turbine->tower.nodes[0]->SetFixed(true);
        turbines.push_back(turbine);

        // increase elements for visualization
        for (int jj = 0; jj < turbine->blades.size(); jj++) {
            auto& blade = turbine->blades[jj]->elasto;
            for (int kk = 0; kk < blade->elements.size(); kk++) {
                blade->elements[kk]->GetTaperedSection()->GetSectionA()->SetDrawThickness(2.0, 0.5);
                blade->elements[kk]->GetTaperedSection()->GetSectionB()->SetDrawThickness(2.0, 0.5);
            }
        }
        // increase elements for visualization
        auto& tower = turbine->tower;
        for (int kk = 0; kk < tower.elements.size(); kk++) {
            tower.elements[kk]->GetTaperedSection()->GetSectionA()->SetDrawThickness(3.0, 3.0);
            tower.elements[kk]->GetTaperedSection()->GetSectionB()->SetDrawThickness(3.0, 3.0);
        }

        turbine->translate(ChVector<double>(150.0 * ii, 150.0 * ii * pow(-1.0, ii), -150.0));
        turbine->rotate(-CH_C_PI / 2.0, VECT_X);
    }

    // VISUALIZATION

 #ifdef HAVE_IRRLICHT
    // make visualization app
    ChIrrApp application(&system, L"Blade", core::dimension2d<u32>(800, 600), VerticalDir::Y, false, true);
    application.AddTypicalLights();
    application.AddTypicalSky();
    application.AddTypicalCamera(core::vector3df(-300, 150, -50));

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
#endif
    // SIMULATION LOOP

    double dt = 0.1;
    //application.SetTimestep(dt);
    //application.SetVideoframeSave(false);
    //application.SetVideoframeSaveInterval(20);

    system.Setup();
    double time = 0.0;
    int step = 0;
    // system.DoStaticLinear();
    // system.DoStaticNonlinear(10, true);
    // application.DoStep();
    auto wind_model = ConstantWind();
    wind_model.set_wind_speed(ChVector<double>(10.59, 0.0, 0.0));

#ifdef HAVE_IRRLICHT
    while ( application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        application.DoStep();
        application.EndScene();
        
#else
    while (true) {
        system.DoStepDynamics(dt);
#endif
        time += system.GetStep();
        step += 1;
        GetLog() << "time " << time << " step: " << step << " rpm: " << turbines[0]->rotor.get_rpm() << "\n";

        // apply force
        for (int ii = 0; ii < turbines.size(); ii++) {
            turbines[ii]->prestep(time, wind_model);
        }
    }

    return 0;
}
