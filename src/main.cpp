#include "chrono/fea/ChVisualizationFEAmesh.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono_irrlicht/ChIrrApp.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
#include "chrono/solver/ChDirectSolverLS.h"
#include <cmath>

#include "elasto/blade_elasto.h"
#include "core/blade_core.h"
#include "elasto/rotor.h"
#include "io/read_json.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace irr;

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

    std::vector<std::shared_ptr<BladeElasto>> all_blades_elasto;
    for (int kk = 0; kk < 3; kk++) {
        // blade
        GetLog() << "Building blades\n";
        std::vector<std::shared_ptr<BladeElasto>> blades_elasto;
        for (int ii = 0; ii < 3; ii++) {
            auto blade = std::make_shared<Blade>(get_blade_from_json("../data/IEA15MW_blade.json"));
            blade->elasto->discretization_fractions.clear();
            blades_elasto.push_back(blade->elasto);
            all_blades_elasto.push_back(blade->elasto);
            blade->build(system, blades_mesh);
            for (int jj = 0; jj < blade->elasto->elements.size(); jj++) {
                blade->elasto->elements[jj]->GetTaperedSection()->GetSectionA()->SetDrawThickness(2.0, 0.5);
                blade->elasto->elements[jj]->GetTaperedSection()->GetSectionB()->SetDrawThickness(2.0, 0.5);
            }
        }

        // rotor
        GetLog() << "Building rotor\n";
        auto rotor = get_rotor_from_json("../data/IEA15MW_RNA.json");
        rotor.build(system, blades_elasto);

        // auto link_motor = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
        // link_motor->Initialize(rotor.body_shaft, rotor.body_hub, rotor.body_shaft->GetAssetsFrame());
        // system.AddLink(link_motor);
        // auto my_speed_function = chrono_types::make_shared<ChFunction_Ramp>(0.0, CH_C_PI / 100.);
        // link_motor->SetSpeedFunction(my_speed_function);
        // link_motor->SetDisabled(false);

        // tower
        GetLog() << "Building tower\n";
        auto tower = get_tower_from_json("../data/IEA15MW_tower.json");
        tower.build(blades_mesh);
        for (int jj = 0; jj < tower.elements.size(); jj++) {
            tower.elements[jj]->GetTaperedSection()->GetSectionA()->SetDrawThickness(2.0, 2.0);
            tower.elements[jj]->GetTaperedSection()->GetSectionB()->SetDrawThickness(2.0, 2.0);
        }
        // translate tower to make it match the current turbine configuration
        tower.translate(ChVector<double>(0.0, 0.0, -tower.height - rotor.shaft.distance_from_towertop));
        // fix bottom of tower
        tower.nodes[0]->SetFixed(true);

        // link rotor to tower
        rotor.link_tower(tower, system);

        tower.translate(ChVector<double>(150.0 * kk, 150.0 * kk * pow(-1.0, kk), 0.0));
        rotor.translate(ChVector<double>(150.0 * kk, 150.0 * kk * pow(-1.0, kk), 0.0));
        tower.rotate(-CH_C_PI / 2.0, VECT_X);
        rotor.rotate(-CH_C_PI / 2.0, VECT_X);

        GetLog() << "Finished building system\n";
        double mass_blades = 0.0;
        for (int ii = 0; ii < blades_elasto.size(); ii++) {
            GetLog() << "Blade" << ii << " mass: " << blades_elasto[ii]->get_mass() << "\n";
            mass_blades += blades_elasto[ii]->get_mass();
        }
        GetLog() << "RNA mass: " << rotor.get_mass() << "\n";
        GetLog() << "RNA mass (without blades): " << rotor.get_mass() - mass_blades << "\n";
        GetLog() << "Tower mass: " << tower.get_mass() << "\n";
        GetLog() << "Total mass: " << rotor.get_mass() + tower.get_mass() << "\n";
    }

    // VISUALIZATION

    // make visualization app
    ChIrrApp application(&system, L"Blade", core::dimension2d<u32>(800, 600), VerticalDir::Y, false, true);
    application.AddTypicalLights();
    application.AddTypicalSky();
    application.AddTypicalCamera(core::vector3df(-300, 3, -50));

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

    double dt = 0.1;
    application.SetTimestep(dt);
    application.SetVideoframeSave(false);
    application.SetVideoframeSaveInterval(20);
    double time = 0.0;
    int step = 0;
    system.DoStaticLinear();
    // system.DoStaticNonlinear(10, true);
    // application.DoStep();
    while (application.GetDevice()->run()) {
        application.BeginScene();
        application.DrawAll();
        application.DoStep();
        application.EndScene();
        // system.DoStepDynamics(dt);
        time += system.GetStep();
        step += 1;
        GetLog() << "time " << time << " step: " << step << "\n";

        // apply force
        for (int jj = 0; jj < all_blades_elasto.size(); ++jj) {
            auto blade = all_blades_elasto[jj];
            for (int kk = 0; kk < blade->elements.size(); ++kk) {
                auto node = blade->nodes[kk];
                blade->loaders_aero[kk]->loader.positions = {-1.0, 1.0};
                auto force = node->TransformDirectionLocalToParent(ChVector<>(0.0, 1000.0, 0.0)) *
                             std::min(100.0, time) / 100.0 / double(jj + 1);
                blade->loaders_aero[kk]->loader.loads = {force, force};
            }
        }
    }

    return 0;
}
