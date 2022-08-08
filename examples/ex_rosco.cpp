#include <iostream>
#include <cstring>

extern "C" void DISCON(float* avrSWAP, int* aviFAIL, char* accINFILE, char* avcOUTNAME, char* avcMSG);

/**@brief ROSCO Discon wrapping interface */
struct DisconController {
    float avrSWAP[500];
    int aviFAIL;
    char accINFILE[4096];
    char avcOUTNAME[1024];
    char avcMSG[4096];

    /*
    CHARACTER(KIND=C_CHAR),         INTENT(IN   )   :: accINFILE(NINT(avrSWAP(50)))     ! The name of the parameter input file
CHARACTER(KIND=C_CHAR),         INTENT(IN   )   :: avcOUTNAME(NINT(avrSWAP(51)))    ! OUTNAME (Simulation RootName)
CHARACTER(KIND=C_CHAR),         INTENT(INOUT)   :: avcMSG(NINT(avrSWAP(49)))        ! MESSAGE (Message from DLL to simulation code [ErrMsg])  The message which will be displayed by the calling program if aviFAIL <> 0.
CHARACTER(SIZE(avcOUTNAME)-1)                   :: RootName                         ! a Fortran version of the input C string (not considered an array here)    [subtract 1 for the C null-character]
CHARACTER(SIZE(avcMSG)-1)                       :: ErrMsg      
    */

    void init() {
        for (auto& v :avrSWAP) {
            v = 0.0;
        }

        avrSWAP[2] = 0.1;  // dT
        avrSWAP[60] = 3; // n. blades
        avrSWAP[19] = 1.0;  // Hard code initial gen speed
        avrSWAP[20] = 1.0;  // Hard code initial rotation speed
        avrSWAP[82] = 0;  // #HARD CODE initial nacIMU = 0
        avrSWAP[26] = 10; // Initial wind speed m /s
        avrSWAP[3] = 0.0; // Initial blade pitch
        avrSWAP[32] = 0.0; //
        avrSWAP[33] = 0.0; // Troque initial
        avrSWAP[22] = 0;  //
        avrSWAP[0] = 0; // This the first call
        avrSWAP[58] = 500; // Buffer chaar size
        avrSWAP[49] = 20;
         //9;                  // len(self.param_name)
        avrSWAP[50] = 500; //self.char_buffer
        avrSWAP[51] = 500;  // self.char_buffer



        aviFAIL = 0;   // c_int32();

        //accINFILE = ''; /// param_name
        strcpy(accINFILE, "controller\\DISCON.IN");
        strcpy(avcOUTNAME, "simDEBUG.RO.dbg");

        //avcOUTNAME 

    }
};

      
      int main(int argc, char* argv[]) {
    std::cout << "Hello ROSCO\n";
    DisconController discon_params;
        
    discon_params.init();

    DISCON(discon_params.avrSWAP, &discon_params.aviFAIL, discon_params.accINFILE, discon_params.avcOUTNAME, discon_params.avcMSG);

    std::cout << "MSG " << discon_params.avcMSG << std::endl;
    std::cout << "End controll\n";
    return 0;
}