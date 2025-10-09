#include "seahowl/env/seastate_adapter.h"

#include <stdexcept>
#include <vector>
#include <string>
#include <fstream>
#include <spdlog/spdlog.h>
#include <filesystem>

using namespace seahowl::env;
namespace fs = std::filesystem;

/**
 * @brief SeaState module in OpenFAST
 * Long term this should be moved to a header file in OpenFAST
 */
extern "C" {
// 
#define CHANNEL_NAME_SIZE 20        // maximum number of characters in a channel name (without c_null termination)
#define PASSED_STRING_LENGTH 1025   // file name length for fixed length filenames (includes c_null termination)
#define ERROR_MSG_LEN 8197          // total character count in error message (includes c_null termination)
#define MAX_OUT_CHANS 20000         // maximum number of channels in any OF c-bind interface

void SeaSt_C_PreInit(float* Gravity_C,                         // in  - gravity (use negative to point downwards) (m/s^2)
                     float* WtrDens_C,                         // in  - water density (kg/m^3)
                     float* WtrDpth_C,                         // in  - water depth (MSL to sea-floor) (m)
                     float* MSL2SWL_C,                         // in  - Mean sea level to still water level (m)
                     int* DebugLevel_In,                       // in  - for debugging interface (0: none, 4: everything including meshes)
                     char OutVTKDir_C[PASSED_STRING_LENGTH],   // in  - Directory to put all vtk output.  Length InfStrLen
                     int* WrVTK_in,                            // in  - Write VTK outputs [0: none, 1: init only, 2: animation]
                     double* WrVTK_inDT,                       // in  - Timestep between VTK writes
                     int* ErrStat_C,                           // out - Error status (0: none, 1: Info, 2: warn, 3: severe, 4: fatal)
                     char ErrMsg_C[ERROR_MSG_LEN]);            // out - Message returned about error (empty if none)

void SeaSt_C_Init(char InputFile_C[PASSED_STRING_LENGTH],      // in  - SeaState input file (absolute or relative path)
                  char OutRootName_C[PASSED_STRING_LENGTH],    // in  - rootname for output summary, output, or echo files (absolute or relative path)
                  int* NSteps_C,                               // in  - total number of timesteps to simulate (-)
                  double* TimeInterval_C,                      // in  - timestep (s)
                  int* NumChannels_C,                          // out - number of output channels
                  char* OutputChannelNames_C,                  // out - channel names - each channel name is CHANNEL_NAME_SIZE in length
                  char* OutputChannelUnits_C,                  // out - channel units - each channel unit is CHANNEL_NAME_SIZE in length
                  int* ErrStat_C,                              // out - Error status (0: none, 1: Info, 2: warn, 3: severe, 4: fatal)
                  char ErrMsg_C[ERROR_MSG_LEN]);               // out - Message returned about error (empty if none)

void SeaSt_C_CalcOutput(double* Time_C,                        // in  - current time (s)
                        float* OutputChannelValues_C,          // out - output channel values
                        int* ErrStat_C,                        // out - Error status (0: none, 1: Info, 2: warn, 3: severe, 4: fatal)
                        char ErrMsg_C[ERROR_MSG_LEN]);         // out - Message returned about error (empty if none)

void SeaSt_C_End(int* ErrStat_C,                               // out - Error status (0: none, 1: Info, 2: warn, 3: severe, 4: fatal)
                 char ErrMsg_C[ERROR_MSG_LEN]);                // out - Message returned about error (empty if none)

void SeaSt_C_GetWaveFieldPointer(int* WaveFieldPointer_C,      // out - pointer to wavefield data (fotran pointer converted to C pointer.  Store as int* locally)
                                 int* ErrStat_C,               // out - Error status (0: none, 1: Info, 2: warn, 3: severe, 4: fatal)
                                 char ErrMsg_C[ERROR_MSG_LEN]);// out - Message returned about error (empty if none)

void SeaSt_C_SetWaveFieldPointer(int* WaveFieldPointer_C,      // in  - pointer to wavefield data - retrieved from a GetWaveFieldPointer call. Stored as int* locally.
                                 int* ErrStat_C,               // out - Error status (0: none, 1: Info, 2: warn, 3: severe, 4: fatal)
                                 char ErrMsg_C[ERROR_MSG_LEN]);// out - Message returned about error (empty if none)

// Get the fluid velocity, acceleration, and node-in-water status at time+position coordinate
// NOTE: if wave stretching is turned off, the SWL is used as the cutoff for the nodeInWater and for Vel / Acc values 
void SeaSt_C_GetFluidVelAcc(double* Time_C,                    // in  - current time (s)
                            float* Pos_c[3],                   // in  - position in 3D (m). Relative to SWL   
                            float* Vel_c[3],                   // out - velocity at requested point.  (m/s)
                            float* Acc_c[3],                   // out - acceleration at requested point.  (m/s^2)
                            int* NodeInWater_C,                // out - node is in or out of water (0: out of water, 1: in water)
                            int* ErrStat_C,                    // out - Error status (0: none, 1: Info, 2: warn, 3: severe, 4: fatal)
                            char ErrMsg_C[ERROR_MSG_LEN]);     // out - Message returned about error (empty if none)

void SeaSt_C_GetSurfElev(double* Time_C,                       // in  - current time (s)
                         float* Pos_c[2],                      // in  - position in 2D (m).
                         float* Elev_C,                        // out - wave elevation relative to SWL (m) 
                         int* ErrStat_C,                       // out - Error status (0: none, 1: Info, 2: warn, 3: severe, 4: fatal)
                         char ErrMsg_C[ERROR_MSG_LEN]);        // out - Message returned about error (empty if none)

void SeaSt_C_GetSurfNorm(double* Time_C,                       // in  - current time (s)
                         float* Pos_c[2],                      // in  - position in 2D (m)
                         float* NormVec_C[3],                  // out - unit vector normal to surface (-)
                         int* ErrStat_C,                       // out - Error status (0: none, 1: Info, 2: warn, 3: severe, 4: fatal)
                         char ErrMsg_C[ERROR_MSG_LEN]);        // out - Message returned about error (empty if none)
}

/**
 * @brief SeaState wrapping inferface
 */
struct seahowl::env::SeaStateLib {
    ~SeaStateLib();
};

SeaStateLib::~SeaStateLib() {}

SeaStateAdapter::SeaStateAdapter(std::string seastate_infile) {
    this->seastate_infile = seastate_infile;
}

SeaStateAdapter::~SeaStateAdapter() {}

double SeaStateAdapter::get_water_level(const Vector3d& position, double time) const {
    // make water level -inf
    return -99999999.9;
};

double SeaStateAdapter::get_density_this(const seahowl::Vector3d& position, double time) const {
    return 0.0;
}

seahowl::Vector3d SeaStateAdapter::get_velocity_this(const seahowl::Vector3d& position, double time) const {
    return seahowl::Vector3d(0.0, 0.0, 0.0);
}

seahowl::Vector3d SeaStateAdapter::get_acceleration_this(const seahowl::Vector3d& position, double time) const {
    return seahowl::Vector3d(0.0, 0.0, 0.0);
}
