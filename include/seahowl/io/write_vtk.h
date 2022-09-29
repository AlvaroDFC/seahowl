#include <seahowl/elasto/elasto.h>

#include <vtkSmartPointer.h>
#include <vtkUnstructuredGrid.h>
#include <vtkXMLUnstructuredGridWriter.h>
#include <vtkPoints.h>
#include <vtkPointData.h>
#include <vtkDoubleArray.h>


struct OutputMeshVTK {
    vtkSmartPointer<vtkUnstructuredGrid> mesh;
    vtkSmartPointer<vtkXMLUnstructuredGridWriter> writer;
    std::map<std::string, vtkSmartPointer<vtkDoubleArray>> arrays;

    double time;
    double dt;
    std::string base = "";

    OutputMeshVTK();
    ~OutputMeshVTK();

    void init(seahowl::elasto::ElastoFEAComponent& component, const char* base_name);
    void write(seahowl::elasto::ElastoFEAComponent& component, double time, int time_step);
};