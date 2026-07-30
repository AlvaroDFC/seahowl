#include "seahowl/io/write_vtk.h"

// SEAHOWL headers
#include "seahowl/commons/numerics.h"
#include "seahowl/core/blade.h"
#include "seahowl/core/system.h"
#include "seahowl/elasto/blade_elasto.h"
#include "seahowl/elasto/floater_elasto.h"

// Third-party libraries
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkUnstructuredGrid.h>
#include <vtkXMLUnstructuredGridWriter.h>

// Standard library
#include <cstdio>
#include <filesystem>
#include <map>
#include <string>
#include <vector>

namespace fs = std::filesystem;

using seahowl::Vector3d;
using namespace seahowl::io;

OutputMeshVTK::OutputMeshVTK(seahowl::elasto::ComponentElastoFEA& component) : component(component) {
    mesh = vtkUnstructuredGrid::New();
}

OutputMeshVTK::OutputMeshVTK(const OutputMeshVTK& rhs) : component(rhs.component) {
    mesh = vtkUnstructuredGrid::New();
    mesh->ShallowCopy(rhs.mesh);
    base = rhs.base;
    reference_root_position = rhs.reference_root_position;
    reference_root_rotation = rhs.reference_root_rotation;
    local_offsets = rhs.local_offsets;
}
OutputMeshVTK::OutputMeshVTK(OutputMeshVTK&& source) noexcept : component(source.component) {
    mesh = source.mesh;
    source.mesh = nullptr;
    base = source.base;
    reference_root_position = source.reference_root_position;
    reference_root_rotation = source.reference_root_rotation;
    local_offsets = std::move(source.local_offsets);
}

OutputMeshVTK::~OutputMeshVTK() {
    if (mesh != nullptr)
        mesh->Delete();
}

void OutputMeshVTK::initialize(const char* base_name) {
    base = base_name;

    auto coords = component.get_nodes_positions();
    auto rotations = component.get_nodes_rotations();

    // capture the undeformed reference configuration (root node) so write() can later remove
    // rigid-body motion from "Displacement", leaving only the elastic/flexible residual.
    reference_root_position = coords[0];
    reference_root_rotation = rotations[0];
    local_offsets.clear();
    local_offsets.reserve(coords.size());
    for (const auto& coord : coords) {
        local_offsets.push_back(reference_root_rotation.inverse() * (coord - reference_root_position));
    }

    auto points = vtkSmartPointer<vtkPoints>::New();
    points->SetDataTypeToDouble();
    const vtkIdType nPoints = coords.size();
    points->SetNumberOfPoints(nPoints);
    double* pDst = static_cast<double*>(points->GetVoidPointer(0));
    memcpy(pDst, &coords[0], sizeof(double) * nPoints * 3);
    mesh->SetPoints(points);

    const vtkIdType nCells = nPoints - 1;
    vtkIdType ptIds[2] = {0, 1};

    for (vtkIdType iCell = 0; iCell < nCells; ++iCell) {
        ptIds[0] = iCell;
        ptIds[1] = iCell + 1;
        mesh->InsertNextCell(VTK_LINE, 2, ptIds);
    }

    std::vector<std::string> myKeys = {"Displacement", "Forces", "Velocity", "Acceleration", "Direction", "Rotation"};

    // initialize arrays properties
    for (auto const& key : myKeys) {
        auto val = vtkSmartPointer<vtkDoubleArray>::New();

        val->SetName(key.c_str());

        if (key == "Rotation") {
            val->SetNumberOfComponents(4);
        } else {
            val->SetNumberOfComponents(3);
        }
        val->SetNumberOfTuples(nPoints);
        val->Fill(0.0);
        mesh->GetPointData()->AddArray(val);
    }
}

void OutputMeshVTK::write(double time, int time_step) const {
    std::map<std::string, std::vector<Vector3d>> arrays_values;

    // positions
    auto points = mesh->GetPoints();
    double* pDst0 = static_cast<double*>(points->GetVoidPointer(0));
    auto values0 = component.get_nodes_positions();
    memcpy(pDst0, &values0[0], sizeof(double) * values0.size() * 3);
    mesh->SetPoints(points);

    // rotations (needed for the "Rotation" array below, and to predict rigid-body motion)
    auto rotations = component.get_nodes_rotations();

    // displacement = current position - rigid-body-predicted position (current root pose applied
    // to the undeformed body-fixed offset), i.e. the elastic/flexible residual only, expressed in
    // the global frame. Node 0 (root) is the rigid-body reference for the whole component.
    const auto& current_root_position = values0[0];
    const auto& current_root_rotation = rotations[0];
    std::vector<Vector3d> displacements(values0.size());
    for (std::size_t idx = 0; idx < values0.size(); ++idx) {
        auto rigid_predicted = current_root_position + current_root_rotation * local_offsets[idx];
        displacements[idx] = values0[idx] - rigid_predicted;
    }

    // vectors
    arrays_values.insert({"Displacement", displacements});
    arrays_values.insert({"Forces", component.get_nodes_loads()});
    arrays_values.insert({"Velocity", component.get_nodes_velocities()});
    arrays_values.insert({"Acceleration", component.get_nodes_accelerations()});
    arrays_values.insert({"Direction", component.get_nodes_directions()});

    for (auto const& keyval : arrays_values) {
        auto& key = keyval.first;
        auto& val = keyval.second;
        auto arr = mesh->GetPointData()->GetArray(key.c_str());

        double* pDst = static_cast<double*>(arr->GetVoidPointer(0));

        memcpy(pDst, &val[0], sizeof(double) * val.size() * 3);
    }

    // quaternions
    auto arr = mesh->GetPointData()->GetArray("Rotation");
    double* pDst = static_cast<double*>(arr->GetVoidPointer(0));
    memcpy(pDst, &rotations[0], sizeof(double) * rotations.size() * 4);

    {
        char fname[2048];
        std::sprintf(fname, "%s_%03d.vtu", base.c_str(), time_step);
        auto writer = vtkSmartPointer<vtkXMLUnstructuredGridWriter>::New();
        writer->SetFileName(fname);
        writer->SetInputData(mesh);
        writer->Write();
    }
}

OutputSystemVTK::OutputSystemVTK(seahowl::core::System& system_core, const std::string& output_folder)
    : system_core(system_core), output_folder(output_folder) {}

void OutputSystemVTK::initialize() {
    vtk_meshes.clear();

    // iterate turbines
    for (auto [turbine_ptr, idx_turbine] = std::tuple{system_core.turbines.begin(), 0};
         turbine_ptr != system_core.turbines.end(); turbine_ptr++, idx_turbine++) {
        auto& turbine = *turbine_ptr;
        fs::create_directory(output_folder);

        // blades
        for (auto [blade_ptr, idx_blade] = std::tuple{turbine->rna.rotor.blades.begin(), 0};
             blade_ptr != turbine->rna.rotor.blades.end(); blade_ptr++, idx_blade++) {
            auto& blade = *blade_ptr;
            try {
                auto& post_blade =
                    vtk_meshes.emplace_back(dynamic_cast<seahowl::elasto::BladeElastoFEA&>(blade->elasto));
                post_blade.initialize(
                    (output_folder + "/turbine" + std::to_string(idx_turbine) + "_blade" + std::to_string(idx_blade))
                        .c_str());
            } catch (const std::exception& e) {
                // do nothing if node blade EFA
            }
        }

        // tower
        auto& post_tower = vtk_meshes.emplace_back(turbine->tower.elasto);
        post_tower.initialize((output_folder + "/turbine" + std::to_string(idx_turbine) + "_tower").c_str());

        // moorings
        try {
            if (turbine->foundation) {
                auto& floater_core = dynamic_cast<seahowl::core::Floater&>(*turbine->foundation);
                auto& floater = dynamic_cast<seahowl::elasto::FloaterElasto&>(floater_core.elasto);
                for (auto [mooring_ptr, idx_mooring] = std::tuple{floater.mooring_system->moorings.begin(), 0};
                     mooring_ptr != floater.mooring_system->moorings.end(); mooring_ptr++, idx_mooring++) {
                    auto& mooring = *mooring_ptr;
                    try {
                        auto& post_mooring =
                            vtk_meshes.emplace_back(dynamic_cast<seahowl::elasto::ComponentElastoFEA&>(*mooring));
                        post_mooring.initialize((output_folder + "/mooring" + std::to_string(idx_mooring)).c_str());
                    } catch (const std::exception& e) {
                        // do nothing if node component elasto EFA
                    }
                }
            }
        } catch (const std::exception& e) {
            // do nothing if not floater
        }
    }

    // iterate elasto components
    for (auto [component_ptr, idx_component] = std::tuple{system_core.elasto.components.begin(), 0};
         component_ptr != system_core.elasto.components.end(); component_ptr++, idx_component++) {
        auto& component = *component_ptr;
        try {
            auto& post_component =
                vtk_meshes.emplace_back(dynamic_cast<seahowl::elasto::ComponentElastoFEA&>(*component));
            post_component.initialize((output_folder + "/component" + std::to_string(idx_component)).c_str());
        } catch (const std::exception& e) {
            // do nothing if node component elasto EFA
        }
    }
}

void OutputSystemVTK::write(int step) {
    for (auto const& vtk_output : vtk_meshes) {
        vtk_output.write(system_core.elasto.get_time(), step);
    }
}
