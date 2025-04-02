#include <seahowl/fluid/hydro/hydrodyn_adapter.h>
#include "seahowl/elasto/floater_elasto.h"  // TODO: create main body for floater hydro

#include <spdlog/spdlog.h>

using namespace seahowl::hydro;

extern "C" {

void HydroDyn_C_Init(char* OutRootName,
                     const char** SeaSt_InputFileString,
                     int& SeaSt_InputFileStringLength,
                     const char** HD_InputFileString,
                     int& HD_InputFileStringLength,
                     float& Gravity,
                     float& defWtrDens,
                     float& defWtrDpth,
                     float& defMSL2SWL,
                     float& PtfmRefPtPositionX,
                     float& PtfmRefPtPositionY,
                     int& NumNodePts,
                     float* InitNodePositions,
                     // int& NumWaveElev,
                     // float* WaveElevXY, //Placeholder for later
                     int& InterpOrder,
                     double& T_initial,
                     double& DT,
                     double& TMax,
                     int& NumChannels,
                     char* OutputChannelNames,
                     char* OutputChannelUnits,
                     int& ErrStat,
                     char* ErrMsg);

void HydroDyn_C_CalcOutput(double& Time,
                           int& NumNodePts,
                           float* NodePos,
                           float* NodeVel,
                           float* NodeAcc,
                           float* NodeFrc,
                           float* OutputChannelValues,
                           int& ErrStat,
                           char* ErrMsg);

void HydroDyn_C_CalcOutput_and_AddedMass(double& Time,
                                         int& NumNodePts,
                                         float* NodePos,
                                         float* NodeVel,
                                         float* NodeAcc,
                                         float* NodeFrc,
                                         float* NodeAdm,
                                         float* OutputChannelValues,
                                         int& ErrStat,
                                         char* ErrMsg);

void HydroDyn_C_UpdateStates(double& Time,
                             double& TimeNext,
                             int& NumNodePts,
                             float* NodePos,
                             float* NodeVel,
                             float* NodeAcc,
                             float* OutputChannelValues,
                             int& ErrStat,
                             char* ErrMsg);

void HydroDyn_C_End(int& ErrStat, char* ErrMsg);
}

/**
 * @brief Interface to HydroDyn library.
 * Additional notes and information on the interfacing is included
 * there. Many notes taken from HydroDyn Python-C interface library.
 *
 *   Note on angles:
 *       All angles passed in are assumed to be given in radians as an Euler
 *       angle sequence R(z)*R(y)*R(x) (notice the order as this important when
 *       passing angles in). Written in matrix form as (doxygen formatted):
 *
 *          \f{eqnarray*}{
 *          M & = & R(\theta_z) R(\theta_y) R(\theta_x) \\
 *            & = & \begin{bmatrix}  \cos(\theta_z) & \sin(\theta_z) & 0 \\
 *                                  -\sin(\theta_z) & \cos(\theta_z) & 0 \\
 *                                    0      &  0      & 1 \end{bmatrix}
 *                  \begin{bmatrix}  \cos(\theta_y) & 0 & -\sin(\theta_y) \\
 *                                         0 & 1 & 0        \\
 *                                   \sin(\theta_y) & 0 & \cos(\theta_y)  \end{bmatrix}
 *                  \begin{bmatrix}   1 &  0       & 0       \\
 *                                    0 &  \cos(\theta_x) & \sin(\theta_x) \\
 *                                    0 & -\sin(\theta_x) & \cos(\theta_x) \end{bmatrix} \\
 *            & = & \begin{bmatrix}
 *             \cos(\theta_y)\cos(\theta_z) &   \cos(\theta_x)\sin(\theta_z)+\sin(\theta_x)\sin(\theta_y)\cos(\theta_z)
 * &
 *                                              \sin(\theta_x)\sin(\theta_z)-\cos(\theta_x)\sin(\theta_y)\cos(\theta_z)
 * \\
 *             -\cos(\theta_y)\sin(\theta_z)  & \cos(\theta_x)\cos(\theta_z)-\sin(\theta_x)\sin(\theta_y)\sin(\theta_z)
 * &
 *                                              \sin(\theta_x)\cos(\theta_z)+\cos(\theta_x)\sin(\theta_y)\sin(\theta_z)
 * \\
 *             \sin(\theta_y)                & -\sin(\theta_x)\cos(\theta_y) & \cos(\theta_x)\cos(\theta_y) \\
 *                  \end{bmatrix}
 *          \f}
 *
 *       When passed into the Fortran library, this Euler angle set is converted
 *       into a DCM (direction cosine matrix) and stored on the input mesh.  All
 *       calculations internally in HD are then performed using the DCM form of
 *       the input angles.
 *
 *       It should be noted that a small angle assumption when returning the
 *       outputs for the platform roll, pitch, and yaw.  These angles are
 *       assumed to be small enough that treating them as independent angles
 *       does not introduce significant error in the output channels.  This may
 *       yield a small discrepency between the values passed in and the returned
 *       output channel values.  These output channels should not be directly
 *       used -- they are only for reporting purposes.
 */

struct seahowl::hydro::HydroDynLib {
    ~HydroDynLib();

    void set_hydrodyn_infile(const std::string& name);
    void set_seastate_infile(const std::string& name);
    void set_outfile_name(const std::string& name);

    void initialize_arrays(int NumNodePts);
    void set_time(double time);

    void CheckError();
    void Init();
    void Calcul();
    void Update();
    void End();

    /*  OutRootName
     *  If HD writes a file (echo, summary, or other),
     *  use this for the root of the file name.
     */
    char OutRootName[1024];

    // Input file string
    std::string HDinputFileString;
    std::string SSinputFileString;

    // Input file string length
    int HDinputFileStringLength;
    int SSinputFileStringLength;

    // Initial environmental conditions
    float gravity = 9.80665;   // Gravitational acceleration (m/s^2)
    float defWtrDens = 1025.;  // Water density (kg/m^3)
    float defWtrDpth = 200.;   // Water depth (m)
    float defMSL2SWL = 0.;     // Offset between still-water level and mean sea level (m) [positive upward]

    /* Number of bodies and initial reference point
     * The initial position is only set as (X,Y).  The Z value and
     * orientation is set by HD and will be returned along with the full
     * set of numBodies where it is expecting loads inputs.
     */
    float PtfmRefPtPositionX = 0.;
    float PtfmRefPtPositionY = 0.;

    /* Nodes
     * The number of nodes must be constant throughout simulation.  The
     * initial position is given in the initNodePos array (resize as
     * needed, should be Nx6).
     * Rotations are given in radians assuming small angles.  See note at
     * top of this file.
     */
    int NumNodePts = 1;  // Single ptfm attachment point for floating rigid
    float* NodePos;
    float* NodeVel;
    float* NodeAcc;
    float* NodeFrc;
    float* NodeAdm;

    int InterpOrder = 1;   // default of linear interpolation
    double Time = 0.;      // current time
    double TimeNext = 0.;  // next time
    double DT = 0.1;       // typical default for HD
    double TMax = 600.0;   // typical default for HD waves FFT

    int NumChannels = 0;  // Number of channels returned
    char OutputChannelNames[20 * 8000];
    char OutputChannelUnits[20 * 8000];
    float OutputChannelValues[20];
    int ErrStat = 0;
    char ErrMsg[1024];
};

HydroDynLib::~HydroDynLib() {
    delete[] NodePos, NodeVel, NodeAcc, NodeFrc, NodeAdm;
}

void HydroDynLib::initialize_arrays(int NumNodePts) {
    this->NumNodePts = NumNodePts;

    NodePos = new float[6 * NumNodePts]{0.0};
    NodeVel = new float[6 * NumNodePts]{0.0};
    NodeAcc = new float[6 * NumNodePts]{0.0};
    NodeFrc = new float[6 * NumNodePts]{0.0};
    NodeAdm = new float[6 * NumNodePts]{0.0};
}

void HydroDynLib::set_hydrodyn_infile(const std::string& name) {
    spdlog::debug("Set HydroDyn INFILE: {}.", name);
    HDinputFileString = name;
    HDinputFileStringLength = HDinputFileString.length();
}

void HydroDynLib::set_seastate_infile(const std::string& name) {
    spdlog::debug("Set SeaState INFILE: {}.", name);
    SSinputFileString = name;
    SSinputFileStringLength = SSinputFileString.length();
}

void HydroDynLib::set_outfile_name(const std::string& name) {
    spdlog::debug("Set Hydrodyn output file: {}.", name);
    strcpy(OutRootName, name.c_str());
}

void HydroDynLib::set_time(double time) {
    Time = time;
    TimeNext = Time + DT;
}

void HydroDynLib::CheckError() {
    if (ErrStat == 0) {
        return;
    } else if (ErrStat == 1) {
        spdlog::info("HydroDyn INFO: {}.", ErrMsg);
    } else if (ErrStat == 2) {
        spdlog::warn("HydroDyn WARNING: {}", ErrMsg);
    } else {
        throw std::runtime_error("HydroDyn ERROR: " + std::string(ErrMsg));
    }
}

void HydroDynLib::Init() {
    // input files
    const char* HDinputFile = HDinputFileString.c_str();
    const char* SSinputFile = SSinputFileString.c_str();

    HydroDyn_C_Init(OutRootName, &SSinputFile, SSinputFileStringLength, &HDinputFile, HDinputFileStringLength, gravity,
                    defWtrDens, defWtrDpth, defMSL2SWL, PtfmRefPtPositionX, PtfmRefPtPositionY, NumNodePts, NodePos,
                    InterpOrder, Time, DT, TMax, NumChannels, OutputChannelNames, OutputChannelUnits, ErrStat, ErrMsg);
    CheckError();
}

void HydroDynLib::Calcul() {
    HydroDyn_C_CalcOutput(Time, NumNodePts, NodePos, NodeVel, NodeAcc, NodeFrc, OutputChannelValues, ErrStat, ErrMsg);
    // HydroDyn_C_CalcOutput_and_AddedMass(Time, NumNodePts, NodePos, NodeVel, NodeAcc, NodeFrc, NodeAdm
    //                       OutputChannelValues, ErrStat, ErrMsg);
    CheckError();
}

void HydroDynLib::Update() {
    HydroDyn_C_UpdateStates(Time, TimeNext, NumNodePts, NodePos, NodeVel, NodeAcc, OutputChannelValues, ErrStat,
                            ErrMsg);
    CheckError();
}

void HydroDynLib::End() {
    HydroDyn_C_End(ErrStat, ErrMsg);
    CheckError();
}

HydroDynAdapter::HydroDynAdapter() {
    spdlog::debug("Initialising HydroDyn Adapter");
    interface_hydrodyn = std::make_unique<HydroDynLib>();
    interface_hydrodyn->set_outfile_name("Floater");
}

HydroDynAdapter::~HydroDynAdapter() {}

void HydroDynAdapter::set_infiles(const std::string& HydroDynInfile, const std::string& SeaStateInfile) {
    interface_hydrodyn->set_hydrodyn_infile(HydroDynInfile);
    interface_hydrodyn->set_seastate_infile(SeaStateInfile);
}

void HydroDynAdapter::initialize(double time, double dt, seahowl::elasto::FloaterElasto& floater) {
    interface_hydrodyn->DT = dt;
    interface_hydrodyn->set_time(time);

    // get the number of body
    auto& floater_bodies = *floater.body_main;
    int NumNodePts = 1;

    interface_hydrodyn->initialize_arrays(NumNodePts);

    // resize vector of hydrodyn loads and moments
    forces_hydrodyn.resize(NumNodePts);
    moments_hydrodyn.resize(NumNodePts);

    // update turbine variables
    update_floater_body_motion(floater);

    interface_hydrodyn->Init();
}

void HydroDynAdapter::update_floater_body_motion(seahowl::elasto::FloaterElasto& floater) {
    // Get the information about floater body
    auto& floater_bodies = *floater.body_main;
    int NumNodePts = 1;

    for (int i = 0; i < NumNodePts; i++) {
        // auto& floater_body = floater_bodies[i]; //TODO: to adapt the multiple body case
        auto& floater_body = floater_bodies;
        auto floater_body_pos = floater_body.get_position();
        auto floater_body_rot = floater_body.get_rpy_angles();
        auto floater_body_vel = floater_body.get_velocity();
        auto floater_body_rotvel = floater_body.get_rotational_velocity();
        auto floater_body_acc = floater_body.get_acceleration();
        auto floater_body_rotacc = floater_body.get_rotational_acceleration();

        for (int j = 0; j < 3; j++) {
            int ii = i * 6 + j;
            interface_hydrodyn->NodePos[ii] = floater_body_pos[j];
            interface_hydrodyn->NodePos[ii + 3] = floater_body_rot[j];
            interface_hydrodyn->NodeVel[ii] = floater_body_vel[j];
            interface_hydrodyn->NodeVel[ii + 3] = floater_body_rotvel[j];
            interface_hydrodyn->NodeAcc[ii] = floater_body_acc[j];
            interface_hydrodyn->NodeAcc[ii + 3] = floater_body_rotacc[j];
        }
    }
}

void HydroDynAdapter::compute_loads(double time, seahowl::elasto::FloaterElasto& floater) {
    interface_hydrodyn->set_time(time);
    update_floater_body_motion(floater);
    interface_hydrodyn->Update();
    interface_hydrodyn->Calcul();

    // get loads from HydroDyn
    for (int ii = 0; ii < interface_hydrodyn->NumNodePts; ii++) {
        forces_hydrodyn[ii][0] = interface_hydrodyn->NodeFrc[ii * 6 + 0];
        forces_hydrodyn[ii][1] = interface_hydrodyn->NodeFrc[ii * 6 + 1];
        forces_hydrodyn[ii][2] = interface_hydrodyn->NodeFrc[ii * 6 + 2];
        moments_hydrodyn[ii][0] = interface_hydrodyn->NodeFrc[ii * 6 + 3];
        moments_hydrodyn[ii][1] = interface_hydrodyn->NodeFrc[ii * 6 + 4];
        moments_hydrodyn[ii][2] = interface_hydrodyn->NodeFrc[ii * 6 + 5];
    }
}

void HydroDynAdapter::end() {
    interface_hydrodyn->End();
}
