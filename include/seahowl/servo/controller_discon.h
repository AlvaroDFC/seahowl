#pragma once

#include <iostream>
#include <cstring>

#include <seahowl/servo/controller.h>

/// <summary>
/// Fortran Fonction definition of DISCO (ROSCO) controller
/// </summary>
/// @todo shoudl be private
/// <param name="avrSWAP"></param>
/// <param name="aviFAIL"></param>
/// <param name="accINFILE"></param>
/// <param name="avcOUTNAME"></param>
/// <param name="avcMSG"></param>
extern "C" void DISCON(float* avrSWAP, int* aviFAIL, char* accINFILE, char* avcOUTNAME, char* avcMSG);

namespace seahowl {
namespace servo {

/**@brief ROSCO Discon wrapping (adapter) interface
 *
 * @todo Set as Pimpl private implementation of seahowl::servo::Controller class
 */
struct DisconController {
    float& m_time = avrSWAP[1];     ///<@brief Time
    float& m_dt = avrSWAP[2];       ///<@brief Time step
    float& m_pitch = avrSWAP[41];   ///<@brief Pitch return controller states
    float& m_torque = avrSWAP[46];  ///<@brief Torque return controller states

    /// <summary>
    /// Reset Controller state as initial
    ///
    /// </summary>
    void ResetFirst() {
        avrSWAP[0] = 0;  // Initial step iStatus
    }

    /// <summary>
    /// Initialize the controller parameters
    /// </summary>
    /// <param name="dt">timestep</param>
    /// <param name="infile">DISCON.IN input file path</param>
    /// <param name="outname">a name (not a path)</param>
    void Init(double dt, double pitch, std::string infile = u8"DISCON.IN", std::string outname = u8"simDEBUG.RO.dbg");

    /// <summary>
    /// Call the DISCON controller
    /// </summary>
    void Call();

    /// <summary>
    /// Helper to set the guess pitch
    /// </summary>
    /// <param name="pitch_angle"></param>
    void SetPitch(double pitch_angle);

    /// <summary>
    /// Helper to set Inflow wind speed
    /// </summary>
    /// <param name="ws">The inflow wind speed. m.s^{-1}</param>
    void SetWindSpeed(double ws);

    /// <summary>
    /// Set Value in avrSWAP array of DISCON
    /// </summary>
    /// <param name="index">Index Fortran. (eg +1 compared to C)</param>
    /// <param name="value">The value to set</param>
    /// <param name="log">If true print the value on standard output</param>
    void SetAvrSWAP(size_t index, float value, bool log = false);

    // Helper to force the cast of value to float
    void SetAvrSWAP(size_t index, size_t value, bool log = false);
    // Helper to force the cast of value to float
    void SetAvrSWAP(size_t index, double value, bool log = false);

    /// <summary>
    /// Get Value from avrSWAP array of DISCON
    /// </summary>
    /// <param name="index">Index Fortran. (eg +1 compared to C)</param>
    /// <param name="log">If true print the value on standard output</param>
    float GetAvrSWAP(size_t index, bool log = false) const;

    /// <summary>
    /// Set input filename with path relative to working directory.
    /// Path is used for other files
    /// ex! control/DISCON.in find other files in control directory
    /// </summary>
    /// <param name="name">DISCON.IN input file path</param>
    void SetINFILE(std::string name = u8"DISCON.IN");

    /// <summary>
    /// Set output base name (relative to working directory).
    /// </summary>
    /// <param name="name">a name (not a path)</param>
    void SetOUTNAME(std::string name = u8"simDEBUG.RO.dbg");

    /// <summary>
    /// Print all output
    /// </summary>
    void PrintAllOut(std::ostream& ssout = std::cout) const;

  private:
    static constexpr size_t MAX_SWAP = 500;

    float avrSWAP[MAX_SWAP];
    int aviFAIL;
    char accINFILE[4096];
    char avcOUTNAME[1024];
    char avcMSG[4096];
};

/**@brief DISCON controler */
class ControllerDISCON : public seahowl::servo::Controller {
  private:
    double torque_elec_previous = 0.0;

  public:
    seahowl::servo::DisconController pImpl;
    double target_rpm = 0.0;

    ControllerDISCON(double dt, double pitch, std::string infile = u8"DISCON.IN", std::string outname = u8"simDEBUG.RO.dbg");
    ~ControllerDISCON(){};

    double get_torque_elec(double time, double dt, double Omega, double pitch);
};

}  // namespace servo
}  // namespace seahowl
