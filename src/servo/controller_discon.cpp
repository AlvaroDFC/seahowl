#include "seahowl/servo/controller_discon.h"

    /*
CHARACTER(KIND=C_CHAR),         INTENT(IN   )   :: accINFILE(NINT(avrSWAP(50)))     ! The name of the parameter
input file CHARACTER(KIND=C_CHAR),         INTENT(IN   )   :: avcOUTNAME(NINT(avrSWAP(51)))    ! OUTNAME (Simulation
RootName) CHARACTER(KIND=C_CHAR),         INTENT(INOUT)   :: avcMSG(NINT(avrSWAP(49)))        ! MESSAGE (Message from
DLL to simulation code [ErrMsg])  The message which will be displayed by the calling program if aviFAIL <> 0.
CHARACTER(SIZE(avcOUTNAME)-1)                   :: RootName                         ! a Fortran version of the input C
string (not considered an array here)    [subtract 1 for the C null-character] CHARACTER(SIZE(avcMSG)-1) :: ErrMsg
*/

/**
/// FORTRAN so +1

    ! Load variables from calling program (See Appendix A of Bladed User's Guide):
    LocalVar%iStatus            = NINT(avrSWAP(1))
    LocalVar%Time               = avrSWAP(2)
    LocalVar%DT                 = avrSWAP(3)
    LocalVar%VS_MechGenPwr      = avrSWAP(14)
    LocalVar%VS_GenPwr          = avrSWAP(15)
    LocalVar%GenSpeed           = avrSWAP(20)
    LocalVar%RotSpeed           = avrSWAP(21)
    LocalVar%GenTqMeas          = avrSWAP(23)
    LocalVar%Y_M                = avrSWAP(24)
    LocalVar%HorWindV           = avrSWAP(27)
    LocalVar%rootMOOP(1)        = avrSWAP(30)
    LocalVar%rootMOOP(2)        = avrSWAP(31)
    LocalVar%rootMOOP(3)        = avrSWAP(32)
    LocalVar%FA_Acc             = avrSWAP(53)
    LocalVar%NacIMU_FA_Acc      = avrSWAP(83)
    LocalVar%Azimuth            = avrSWAP(60)
    LocalVar%NumBl              = NINT(avrSWAP(61))

    ! --- NJA: usually feedback back the previous pitch command helps for numerical stability, sometimes it does
not... IF (LocalVar%iStatus == 0) THEN LocalVar%BlPitch(1) = avrSWAP(4) LocalVar%BlPitch(2) = avrSWAP(33)
        LocalVar%BlPitch(3) = avrSWAP(34)
    ELSE
        LocalVar%BlPitch(1) = LocalVar%PitCom(1)
        LocalVar%BlPitch(2) = LocalVar%PitCom(2)
        LocalVar%BlPitch(3) = LocalVar%PitCom(3)
    ENDIF


    ! Set unused outputs to zero (See Appendix A of Bladed User's Guide):
    avrSWAP(35) = 1.0 ! Generator contactor status: 1=main (high speed) variable-speed generator
    avrSWAP(36) = 0.0 ! Shaft brake status: 0=off
    avrSWAP(41) = 0.0 ! Demanded yaw actuator torque
    avrSWAP(46) = 0.0 ! Demanded pitch rate (Collective pitch)
    avrSWAP(55) = 0.0 ! Pitch override: 0=yes
    avrSWAP(56) = 0.0 ! Torque override: 0=yes
    avrSWAP(65) = 0.0 ! Number of variables returned for logging
    avrSWAP(72) = 0.0 ! Generator start-up resistance
    avrSWAP(79) = 0.0 ! Request for loads: 0=none
    avrSWAP(80) = 0.0 ! Variable slip current status
    avrSWAP(81) = 0.0 ! Variable slip current demand


    */

void seahowl::servo::DisconController::Init() {

    for (auto& v : avrSWAP) {
        v = 0.0;
    }

    avrSWAP[0] = 0;  // This the first call
    avrSWAP[0] = 0;
    //1;                  // iStatus
    //avrSWAP[1] = 0.1;   // time
    avrSWAP[2] = 0.1;   // dT
    avrSWAP[60] = 3;    // n. blades
    avrSWAP[19] = 1.0;  // Hard code initial gen speed
    avrSWAP[20] = 1.0;  // Hard code initial rotation speed
    avrSWAP[82] = 0;    // #HARD CODE initial nacIMU = 0
    avrSWAP[26] = 10;   // Initial wind speed m /s
    avrSWAP[3] = 0.0;   // Initial blade pitch
    avrSWAP[32] = 0.0;  //
    avrSWAP[33] = 0.0;  // Troque initial
    avrSWAP[22] = 0;    //
    
    avrSWAP[58] = 500;  // Buffer chaar size
    avrSWAP[49] = 20;
    // 9;                  // len(self.param_name)
    avrSWAP[50] = 500;  // self.char_buffer
    avrSWAP[51] = 500;  // self.char_buffer

    aviFAIL = 1;  // c_int32();

    // accINFILE = ''; /// param_name
    strcpy(accINFILE, u8"controller/DISCON.IN");
    strcpy(avcOUTNAME, "simDEBUG.RO.dbg");

    // avcOUTNAME


    // Add states to avr
    avrSWAP[1] = 0.1; //time
    avrSWAP[2] = 0.1; //dt
    avrSWAP[3] = 2.0; //pitch
    avrSWAP[32] = 2.0; //pitch
    avrSWAP[33] = 2.0; //pitch
    //avrSWAP[14] = 0.0; /// genspeed*torque*geneff
    avrSWAP[22] = 3.0;//torque
    //avrSWAP[19] = genspeed
    //avrSWAP[20] = rotspeed
    avrSWAP[26] = 10.0; //ws Wind speed initial
    //avrSWAP[82] = NacIMU_FA_Acc


    // First step
    ResetFirst();
}

void seahowl::servo::DisconController::SetPitch(double pitch_angle) {
    avrSWAP[3] = pitch_angle;  // pitch
    avrSWAP[32] = pitch_angle;  // pitch
    avrSWAP[33] = pitch_angle;  // pitch
}

void seahowl::servo::DisconController::SetWindSpeed(double ws) {
    avrSWAP[26] = ws;  // ws Wind speed initial
}

void seahowl::servo::DisconController::Call() {
    DISCON(avrSWAP, &aviFAIL, accINFILE, avcOUTNAME, avcMSG);
    avrSWAP[0] = 1;  // iStatus : standard  step (not the first)
    
}