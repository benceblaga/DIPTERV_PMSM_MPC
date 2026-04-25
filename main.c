    //===================================================\\
   //                                                     \\
  //   <<< FIELD ORIENTED CONTROL OF TWO PMSM DRIVES >>>   \\
 //              CPU_1 :: CONTROL IMPLEMENTATION            \\
//===========================================================\\


  //
 // Included Files
//
#include "F28x_Project.h"
#include "math.h"
#include "CLAmath.h"
#include <PMSM_T_S_MPC_CPU1_header.h>
#include "F2837xD_Ipc_drivers.h"
#include "MPC.h"



/*----------------------------------------------------------------------------/
/                              Defines                                        /
/----------------------------------------------------------------------------*/
    // Programming :: please ignore
#define WAITSTEP    asm(" RPT #255||NOP")
    // Constants
#define TWOPI         6.283185307179586
#define OnePerSqrt3   0.577350269189626
#define Sqrt3Per2     0.866025403784439
    // Parameters
#define PWMMAX 10000.0 //PWM
#define Ts     0.00010 //Sampling Time
    // Controller
#define M_MAX_1  1.17
#define M_MAX_2  0.70
#define V_MAX    0.577350269
//#define M_CONST1 7.8632478632
//#define M_CONST2 9.1428571429
#define M_CONST1 10.4297
#define M_CONST2 12.713
#define SAMPLE_SIZE 8192

/*----------------------------------------------------------------------------/
/                              Interrupt Prototypes                           /
/----------------------------------------------------------------------------*/
__interrupt void cla1Isr1();
__interrupt void cla1Isr2();
__interrupt void cla1Isr3();
__interrupt void cla1Isr4();
__interrupt void cla1Isr5();
__interrupt void cla1Isr6();
__interrupt void cla1Isr7();
__interrupt void cla1Isr8();
__interrupt void adca1_isr(void);
__interrupt void Record_IPM(void);


/*----------------------------------------------------------------------------/
/                              IPC :: RAM data arrays for CPU1                /
/----------------------------------------------------------------------------*/
float c1_r_array[64];   // mapped to GS0 of shared RAM owned by CPU02
float c1_r_w_array[64]; // mapped to GS1 of shared RAM owned by CPU01
#pragma DATA_SECTION(c1_r_array,"SHARERAMGS0");
#pragma DATA_SECTION(c1_r_w_array,"SHARERAMGS1");
  //
 // IPC :: Externals
//
extern uint16_t isrfuncLoadStart;
extern uint16_t isrfuncLoadEnd;
extern uint16_t isrfuncRunStart;
extern uint16_t isrfuncLoadSize;


/*----------------------------------------------------------------------------/
/                              Function Prototypes                            /
/----------------------------------------------------------------------------*/
void initDAC(void);
void QEPInit(void);
void ConfigureADC(void);
void CLA_initCpu1Cla1(void);
void CLA_configClaMemory(void);
void EPWM_Config(Uint16 period);
void OutputDACA(float value, float gain);
void OutputDACB(float value, float gain);
void swap(float*a,float*b);
void sortfive(float *array);
void infkiller(float *value);


void PI_controller(float reference_value, float actual_value, float *error, float *q_prev, float *u, float P, float I, float UMAX, float T);

  //
 // Structures
//
    //EPWM pointer collection for easy setup
volatile struct EPWM_REGS *ePWM[6] = { &EPwm1Regs, &EPwm2Regs, &EPwm3Regs, &EPwm4Regs, &EPwm5Regs, &EPwm6Regs};


/*----------------------------------------------------------------------------/
/                              Global Variables                               /
/----------------------------------------------------------------------------*/
    // Int
Uint16 duty_1a, duty_1b, duty_1c, duty_2a, duty_2b, duty_2c; // motor 1&2 duty cycles

    // Float
float v_1a_ref = 0.50; // motor 1 a-phase reference
float v_1b_ref = 0.50; // motor 1 b-phase reference
float v_1c_ref = 0.50; // motor 1 c-phase reference
float v_2a_ref = 0.50; // motor 2 a-phase reference
float v_2b_ref = 0.50; // motor 2 b-phase reference
float v_2c_ref = 0.50; // motor 2 c-phase reference

float U_1d_comp;
float U_1q_comp;
float U_1d_CLA;
float U_1q_CLA;

float w_1 = 0.0; // speed of motor 1
float w_2 = 0.0; // speed of motor 2
float w_1_acc = 0.0; // motor 1 speed auxiliary accumulator
float w_2_acc = 0.0; // motor 1 speed auxiliary accumulator

float w_1_filter = 0.0;
float w_1_filtered = 0.0;

float i_1a_off = 0.0, i_1b_off = 0.0, i_2a_off = 0.0, i_2b_off = 0.0; // current offset
float i_1a, i_1b, i_2a, i_2b;
float I_scaler; // current adc to Amps coefficient

float w_ref_act;
float w_ref_delta = 0.01;
int w_ref_increment = 0;

float alpha1, alpha2; // mechanical angles of motors 1 & 2

float CP_speed, CI_speed; // PI controller parameters (speed loop)
float CP_1current, CI_1current; // PI controller parameters (current loop) M1
float CP_2current, CI_2current; // PI controller parameters (current loop) M2

float E_1s = 0.0, E_1d = 0.0, E_1q = 0.0, E_2s = 0.0, E_2d = 0.0, E_2q = 0.0; // Previous Errors
float Q_1s = 0.0, Q_1d = 0.0, Q_1q = 0.0, Q_2s = 0.0, Q_2d = 0.0, Q_2q = 0.0; // Integrators
float U_1s = 0.0, U_1d = 0.0, U_1q = 0.0, U_2s = 0.0, U_2d = 0.0, U_2q = 0.0; // Actuation

float w_ref; // Speed reference
//float w_1_ref = 0.0, w_2_ref = 0.0; // Speed  references
float M_1_ref = 0.0, M_2_ref = 0.0; // Torque references
float i_1d_ref_ext = 0.0, i_1q_ref_ext = 0.0;
float i_2d_ref_ext = 0.0, i_2q_ref_ext = 0.0;
float i_2d_ref_dummy = 0.0, i_2q_ref_dummy = 0.0;
int control_select = 0;


/*----------------------------------------------------------------------------/
/                              Debug Please Ignore                            /
/----------------------------------------------------------------------------*/
float sin1, cos1, sin2, cos2;
float i_1al, i_1be, i_2al, i_2be;
float i_1d, i_1q, i_2d, i_2q;
float i_1d_ref, i_1q_ref, i_2d_ref, i_2q_ref;
float v_1al, v_1be, v_2al, v_2be;
float v_1a, v_1b, v_1c, v_2a, v_2b, v_2c;
float v_1mid;
float v_2mid;

float i_1d_median;
float i_1q_median;
float i_2d_median;
float i_2q_median;

unsigned int Pos_1prev;
unsigned int Pos_2prev;

unsigned int DAC_select = 0;

unsigned int Sample_counter = 0;

unsigned int Measurement_counter = 0;
unsigned int Measurement_ack = 0;
unsigned int Measurement_start = 0;
unsigned int Measurement_flag = 0;
float Measurement_acc_1 = 0.0;
float Measurement_acc_2 = 0.0;
unsigned int D_cnt = 0;
unsigned int Q_cnt = 0;

float M1_Rs = 0.2309;
float M1_Ld = 0.0005565;
float M1_Lq = 0.0007213;
float M1_Ps = 0.0159682;

float M2_Rs = 0.3596;
float M2_Ld = 0.0008264;
float M2_Lq = 0.0011268;
float M2_Ps = 0.0131485;

float i_1d_pred = 0.0;
float i_1q_pred = 0.0;

float i_2d_pred = 0.0;
float i_2q_pred = 0.0;

float alpha_corr = 0.0;

float i_1d_mean;
float i_1q_mean;
float i_2d_mean;
float i_2q_mean;
float i_1d_variance;
float i_1q_variance;
float i_2d_variance;
float i_2q_variance;
float i_1d_mean_latch;
float i_1q_mean_latch;
float i_2d_mean_latch;
float i_2q_mean_latch;
float i_1d_variance_latch;
float i_1q_variance_latch;
float i_2d_variance_latch;
float i_2q_variance_latch;

float U_1d_opt_latch = 0.0;
float U_1q_opt_latch = 0.0;
float U_2d_opt_latch = 0.0;
float U_2q_opt_latch = 0.0;

int Predictive_enable = 0;

float M1_TPPI = 0.0;
float M2_TPPI = 0.0;
float w_TPPI = 0.0;

float u_1d_mean;
float u_1q_mean;
float u_2d_mean;
float u_2q_mean;
float u_1d_variance;
float u_1q_variance;
float u_2d_variance;
float u_2q_variance;
float u_1d_mean_latch;
float u_1q_mean_latch;
float u_2d_mean_latch;
float u_2q_mean_latch;
float u_1d_variance_latch;
float u_1q_variance_latch;
float u_2d_variance_latch;
float u_2q_variance_latch;

float Current_Kalman_gain = 0.7;

float Analog_speed1;
float Analog_speed2;

float Ratio;

int Torque_Control_enable = 0;
float M_1ref_ext = 0.0;
float M_2ref_ext = 0.0;
volatile int IPM_enable = 0;
volatile int IPM_approve = 0;

float IPM_i_1d;
float IPM_i_1q;
float IPM_i_2d;
float IPM_i_2q;
float IPM_M1;
float IPM_M2;

float SpeedScaler;

    // Int
unsigned int startup_state_counter  = 0; // startup state-machine counter
unsigned int startup_state_variable = 0; // startup state-machine state parameter
unsigned int QEPcounter1; // motor 1 speed event counter
unsigned int QEPcounter2; // motor 2 speed event counter
unsigned int Speed_scaler = 314159; // speed scaler
unsigned int Pos1; // Position of motor 1
unsigned int Pos2; // Position of motor 2
unsigned int Cntr;


/*----------------------------------------------------------------------------/
/                              Global Arrays                                  /
/----------------------------------------------------------------------------*/
float U_d_meas[17][17];
float U_q_meas[17][17];
float I_d_meas[17][17];
float I_q_meas[17][17];

unsigned int Median_enable = 0;
float i_1d_buffer[5];
float i_1q_buffer[5];
float i_2d_buffer[5];
float i_2q_buffer[5];
float i_1d_sorted[5];
float i_1q_sorted[5];
float i_2d_sorted[5];
float i_2q_sorted[5];

int Decoupling_enable = 0;
    // float

// TPPI

//float i_d_test;
//float i_q_test;
//float v_d_test;
//float v_q_test;
//float alpha_test;
//float w_test;
//float i_d_ref_test;
//float i_q_ref_test;


/*----------------------------------------------------------------------------/
/                              CLA Communication                              /
/----------------------------------------------------------------------------*/
#ifdef __cplusplus
#pragma DATA_SECTION("CpuToCla1MsgRAM");
float CPU_2_CLA[15];
#pragma DATA_SECTION("Cla1ToCpuMsgRAM");
float CLA_2_CPU[16];
#else
#pragma DATA_SECTION(CPU_2_CLA,"CpuToCla1MsgRAM");
float CPU_2_CLA[15];
#pragma DATA_SECTION(CLA_2_CPU,"Cla1ToCpuMsgRAM");
float CLA_2_CPU[16];
#endif

//typedef struct { float *i_1a; float *i_1b; float *i_2a; float *i_2b; float *i_1d; float *i_1q; float *i_2d; float *i_2q; float *i_1d_ref; float *i_1q_ref; float *i_2d_ref; float *i_2q_ref; float *v_1al; float *v_1be; float *v_2al; float *v_2be; float *w; float *w_ref; float *sin1; float *cos1; } DAC_OUT_VALUES;
//typedef struct { float *i_1a; float *i_1b; float *i_2a; float *i_2b; float *i_1d; float *i_1q; float *i_2d; float *i_2q; float *i_1d_ref; float *i_1q_ref; float *i_2d_ref; float *i_2q_ref; float *v_1al; float *v_1be; float *v_2al; float *v_2be; float *w; float *w_ref; float *sin1; float *cos1; } DAC_OUT_SCALER;

//float *DAC_VAL[24];
//float DAC_SCA[24];
float *DAC_VAL[28] = { &i_1a,  &i_1b,  &i_2a,  &i_2b,  &i_1d,  &i_1q,  &i_2d,  &i_2q, &i_1d, &i_1d_ref,  &i_1q, &i_1q_ref,  &i_2d, &i_2d_ref,  &i_2q, &i_2q_ref, &v_1al, &v_1be, &v_2al, &v_2be, &w_1, &w_ref_act, &sin1, &cos1, &i_1d_ref, &i_1q_ref, &i_2d_ref, &i_2q_ref
};
float  DAC_SCA[28] = { 256.0f, 256.0f, 256.0f, 256.0f, 256.0f, 256.0f, 256.0f, 256.0f, 256.0f,    256.0f, 256.0f,    256.0f, 256.0f,    256.0f, 256.0f,    256.0f, 2000.0f, 2000.0f, 2000.0f, 2000.0f,  5.0f,    5.0f, 2000.0f, 2000.0f, 256.0f,256.0f,256.0f,256.0f,
};

    //\\
   //  \\
  //    \\
 //\\  //\\
//  \\//  \\



/*----------------------------------------------------------------------------/
/                              MPC Init                                       /
/----------------------------------------------------------------------------*/
volatile unsigned int MPC_ON = 0u;
// put the MPC function in the RAM
#pragma CODE_SECTION(MPC_step, "TI.ramfunc");

void main(void)
{
//
// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2837xD_SysCtrl.c file.
//
    InitSysCtrl();
    EALLOW;
    DevCfgRegs.CPUSEL5.bit.SCI_A = 1; // Authorize CPU2 for SCI_A
    EDIS;

//
// Step 2. Boot CPU2
//
#ifdef _STANDALONE
#ifdef _FLASH
    //
    //  Send boot command to allow the CPU02 application to begin execution
    //
    IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_FLASH);
#else
    //
    //  Send boot command to allow the CPU02 application to begin execution
    //
    IPCBootCPU2(C1C2_BROM_BOOTMODE_BOOT_FROM_RAM);
#endif
#endif

//
// Step 2. Initialize GPIO:
// This example function is found in the F2837xD_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
//
    InitGpio(); // Skipped for this example

    //EPWM Clock Gating
    CpuSysRegs.PCLKCR2.bit.EPWM1=1;
    CpuSysRegs.PCLKCR2.bit.EPWM2=1;
    CpuSysRegs.PCLKCR2.bit.EPWM3=1;
    CpuSysRegs.PCLKCR2.bit.EPWM4=1;
    CpuSysRegs.PCLKCR2.bit.EPWM5=1;
    CpuSysRegs.PCLKCR2.bit.EPWM6=1;

    EALLOW;

      //
     // EPWM MUX
    //

    //EPWM 1
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO0 (EPWM1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up on GPIO1 (EPWM1B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure as EPWM1B
    //EPWM 2
    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 1;    // Disable pull-up on GPIO2 (EPWM2A)
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 1;    // Disable pull-up on GPIO3 (EPWM2B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure as EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure as EPWM2B
    //EPWM 3
    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 1;    // Disable pull-up on GPIO4 (EPWM3A)
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 1;    // Disable pull-up on GPIO5 (EPWM3B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure as EPWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure as EPWM3B
    //EPWM 4
    GpioCtrlRegs.GPAPUD.bit.GPIO6 = 1;    // Disable pull-up on GPIO6 (EPWM4A)
    GpioCtrlRegs.GPAPUD.bit.GPIO7 = 1;    // Disable pull-up on GPIO7 (EPWM4B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 1;   // Configure as EPWM4A
    GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 1;   // Configure as EPWM4B
    //EPWM 5
    GpioCtrlRegs.GPAPUD.bit.GPIO8 = 1;    // Disable pull-up on GPIO8 (EPWM5A)
    GpioCtrlRegs.GPAPUD.bit.GPIO9 = 1;    // Disable pull-up on GPIO9 (EPWM5B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 1;   // Configure as EPWM5A
    GpioCtrlRegs.GPAMUX1.bit.GPIO9 = 1;   // Configure as EPWM5B
    //EPWM 6
    GpioCtrlRegs.GPAPUD.bit.GPIO10 = 1;   // Disable pull-up on GPI10 (EPWM6A)
    GpioCtrlRegs.GPAPUD.bit.GPIO11 = 1;   // Disable pull-up on GPI11 (EPWM6B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;  // Configure as EPWM6A
    GpioCtrlRegs.GPAMUX1.bit.GPIO11 = 1;  // Configure as EPWM6B


    // EQEP1
    GpioCtrlRegs.GPAPUD.bit.GPIO20 = 1;   // Disable pull-up on GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO21 = 1;   // Disable pull-up on GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAPUD.bit.GPIO22 = 1;   // Disable pull-up on GPIO22 (EQEP1S)
    GpioCtrlRegs.GPAPUD.bit.GPIO23 = 1;   // Disable pull-up on GPIO23 (EQEP1I)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO20 = 0; // Sync to SYSCLKOUT GPIO20 (EQEP1A)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO21 = 0; // Sync to SYSCLKOUT GPIO21 (EQEP1B)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO22 = 0; // Sync to SYSCLKOUT GPIO22 (EQEP1S)
    GpioCtrlRegs.GPAQSEL2.bit.GPIO23 = 0; // Sync to SYSCLKOUT GPIO23 (EQEP1I)
    GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 1;   // Configure GPIO20 as EQEP1A
    GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 1;   // Configure GPIO21 as EQEP1B
    GpioCtrlRegs.GPAMUX2.bit.GPIO22 = 1;   // Configure GPIO22 as EQEP1S
    GpioCtrlRegs.GPAMUX2.bit.GPIO23 = 1;   // Configure GPIO23 as EQEP1I
    // EQEP2
    GpioCtrlRegs.GPBPUD.bit.GPIO54 = 1;   // Disable pull-up on GPIO24 (EQEP2A)
    GpioCtrlRegs.GPBPUD.bit.GPIO55 = 1;   // Disable pull-up on GPIO25 (EQEP2B)
    GpioCtrlRegs.GPBPUD.bit.GPIO56 = 1;   // Disable pull-up on GPIO26 (EQEP2S)
    GpioCtrlRegs.GPBPUD.bit.GPIO57 = 1;   // Disable pull-up on GPIO27 (EQEP2I)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO54 = 0; // Sync to SYSCLKOUT GPIO24 (EQEP2A)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO55 = 0; // Sync to SYSCLKOUT GPIO25 (EQEP2B)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO56 = 0; // Sync to SYSCLKOUT GPIO26 (EQEP2S)
    GpioCtrlRegs.GPBQSEL2.bit.GPIO57 = 0; // Sync to SYSCLKOUT GPIO27 (EQEP2I)

    //There is no curse in Elvish, Entish, or the tongues of Men for this treachery (Also this is the code for muxing eQEP2)
    GpioCtrlRegs.GPBGMUX2.bit.GPIO54 = 1;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO55 = 1;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO56 = 1;
    GpioCtrlRegs.GPBGMUX2.bit.GPIO57 = 1;
    GpioCtrlRegs.GPBMUX2.bit.GPIO54 = 1;   // Configure GPIO24 as EQEP2A
    GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 1;   // Configure GPIO25 as EQEP2B
    GpioCtrlRegs.GPBMUX2.bit.GPIO56 = 1;   // Configure GPIO26 as EQEP2S
    GpioCtrlRegs.GPBMUX2.bit.GPIO57 = 1;   // Configure GPIO27 as EQEP2I



    //TESTPIN GPIO32 (J1 p2)
    GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;
    GpioCtrlRegs.GPBDIR.bit.GPIO32 =  1;
    GpioDataRegs.GPBCLEAR.bit.GPIO32 =  1;

    // ENABLE PIN FOR INVERTER 1 (J6 p13)
    GpioCtrlRegs.GPDMUX2.bit.GPIO124 = 0;
    GpioCtrlRegs.GPDDIR.bit.GPIO124 =  1;
    GpioDataRegs.GPDSET.bit.GPIO124 =  1;

    // ENABLE PIN FOR INVERTER 2 (J6 p53)
    GpioCtrlRegs.GPAMUX2.bit.GPIO26 = 0;
    GpioCtrlRegs.GPADIR.bit.GPIO26 =  1;
    GpioDataRegs.GPASET.bit.GPIO26 =  1;


    EDIS;

//
// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
//
    DINT;

//
// Initialize the PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the F2837xD_PieCtrl.c file.
//
    InitPieCtrl();

//
// Disable CPU interrupts and clear all CPU interrupt flags:
//
    IER = 0x0000;
    IFR = 0x0000;

//
// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2837xD_DefaultIsr.c.
// This function is found in F2837xD_PieVect.c.
//
    InitPieVectTable();
    CLA_configClaMemory();
    CLA_initCpu1Cla1();

//
// Map ISR functions
//
    EALLOW;
    PieVectTable.ADCA1_INT = &adca1_isr; //function for ADCA interrupt 1
    PieVectTable.IPC1_INT = &Record_IPM;
    EDIS;

//
// Configure Timer0
//

//
// Configure the ADC and QEP and DAC
//
    ConfigureADC();
    QEPInit();
    initDAC();


    //Configure the PWM
    EPWM_Config(((unsigned int) PWMMAX));

//
// Configure the ePWM
//
//
// Enable global Interrupts and higher priority real-time debug events:
//
    IER |= M_INT1; //Enable group 1 interrupts
    IER |= M_INT13; //Enable group 13 interrupt
    IER |= M_INT14; //Enable group 13 interrupt
    EINT;  // Enable Global interrupt INTM
    ERTM;  // Enable Global realtime interrupt DBGM

//
// enable PIE interrupt
//
    PieCtrlRegs.PIEIER1.all = 0;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    //PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx13 = 1;   // CPU1 to CPU2 INT1
    PieCtrlRegs.PIEIER1.bit.INTx14 = 1;   // CPU2 to CPU1 INT1



//
// sync ePWM
//
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;
    ClkCfgRegs.PERCLKDIVSEL.bit.EPWMCLKDIV = 0x0; //division by 1
    EDIS;

//
// Step 11. Give Memory Access to GS0/ GS14 SARAM to CPU02
//
    while( !(MemCfgRegs.GSxMSEL.bit.MSEL_GS0 &
             MemCfgRegs.GSxMSEL.bit.MSEL_GS14))
    {
        EALLOW;

        MemCfgRegs.GSxMSEL.bit.MSEL_GS0 = 1;
        MemCfgRegs.GSxMSEL.bit.MSEL_GS14 = 1;

        EDIS;
    }
//
// Step 12. Copy ISR routine to a specified RAM location to determine the size
//
    memcpy(&isrfuncRunStart, &isrfuncLoadStart, (uint32_t)&isrfuncLoadSize);



//
// start Timer0
//

    DELAY_US(2000);
    //CpuTimer1Regs.TCR.all = 0x4000;
    //CpuTimer2Regs.TCR.all = 0x4000;

    GpioDataRegs.GPDCLEAR.bit.GPIO124 =  1;          //enable inverter 1
    GpioDataRegs.GPACLEAR.bit.GPIO26  =  1;          //enable inverter 2

    // Initial values
    I_scaler = 33.0/4096.0;

    CP_speed    = 0.005;
    CP_1current = 0.010;
    CP_2current = 0.010;
    CI_speed    = 0.100;
    CI_1current = 1.000;
    CI_2current = 1.000;

    w_ref = 0.0;
    //w_1_ref = 0.0;
    //w_2_ref = 0.0;

    Cla1ForceTask8andWait(); // Initialization CLA Task
    //typedef struct { float *i_1a; float *i_1b;
                     //float *i_2a; float *i_2b;
                     //float *i_1d; float *i_1q;
                     //float *i_2d; float *i_2q;
                     //float *i_1d_ref; float *i_1q_ref;
                     //float *i_2d_ref; float *i_2q_ref;
                     //float *v_1al; float *v_1be;
                     //float *v_2al; float *v_2be;
                     //float *w; float *w_ref;
                     //float *sin1; float *cos1; } DAC_OUT_VALUES;


    //IPM PARAMETER SEND

    //float M_1ref = c2_r_array[0];
    //float M_2ref = c2_r_array[1];

    //float w_ref = c2_r_array[2];

    c1_r_w_array[3] = M1_Rs;
    c1_r_w_array[4] = M2_Rs;
    c1_r_w_array[5] = M1_Ld;
    c1_r_w_array[6] = M2_Ld;
    c1_r_w_array[7] = M1_Lq;
    c1_r_w_array[8] = M2_Lq;
    c1_r_w_array[9] = M1_Ps;
    c1_r_w_array[10]= M2_Ps;



    //float P = 4.0;

    c1_r_w_array[11] = 12.0;
    c1_r_w_array[12] = 10.0;
    c1_r_w_array[13] = 9.0;
    c1_r_w_array[14] = 6.0;

    CPU_2_CLA[14] = 24.0;

/*----------------------------------------------------------------------------/
/                              MPC Init                                       /
/----------------------------------------------------------------------------*/
    MPC_initialize();

    while(1)
    {
    }
}

    //=============//
   //             //
  // End of Main //
 //             //
//=============//

//
// ConfigureADC - Write ADC configurations and power up the ADC for both
//                ADC A and ADC B
//
void ConfigureADC(void)
{
    EALLOW;
        AdcaRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
        AdcaRegs.ADCCTL2.bit.RESOLUTION = ADC_RESOLUTION_12BIT;
        AdcaRegs.ADCCTL2.bit.SIGNALMODE = ADC_SIGNALMODE_SINGLE;
        AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1; // pulse position to late
        AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1; // power up

        AdcbRegs.ADCCTL2.bit.PRESCALE = 6; //set ADCCLK divider to /4
        AdcbRegs.ADCCTL2.bit.RESOLUTION = ADC_RESOLUTION_12BIT;
        AdcbRegs.ADCCTL2.bit.SIGNALMODE = ADC_SIGNALMODE_SINGLE;
        AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;
        AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;

        DELAY_US(1000);
    EDIS;

    EALLOW;
        AdcaRegs.ADCSOC0CTL.bit.CHSEL = 2; // J3/29 pin current for i_1a
        AdcbRegs.ADCSOC0CTL.bit.CHSEL = 2; // J3/28 pin current for i_1b
                                           // WARNING SWAP!!!
        AdcaRegs.ADCSOC1CTL.bit.CHSEL = 3; // J3/25 pin current for i_2a
        AdcbRegs.ADCSOC1CTL.bit.CHSEL = 3; // J3/26 pin current for i_2b


        AdcaRegs.ADCSOC0CTL.bit.ACQPS = 120; //sample window
        AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 0x05; //epwm1a
        AdcbRegs.ADCSOC0CTL.bit.ACQPS = 120; //sample window
        AdcbRegs.ADCSOC0CTL.bit.TRIGSEL = 0x05; //epwm1a
        AdcaRegs.ADCSOC1CTL.bit.ACQPS = 120; //sample window
        AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 0x05; //epwm1a
        AdcbRegs.ADCSOC1CTL.bit.ACQPS = 120; //sample window
        AdcbRegs.ADCSOC1CTL.bit.TRIGSEL = 0x05; //epwm1a


        //SET INTERRUPT TO SOC0
        AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //end of SOC0 will set INT1 flag
        AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   //enable INT1 flag
        AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //make sure INT1 flag is cleared

    EDIS;
}


void  QEPInit(void)
{
    //EQEP 1
    EQep1Regs.QUPRD=200000;         // Unit Timer for 1000Hz at 200 MHz SYSCLKOUT
    EQep1Regs.QDECCTL.bit.QSRC=00;      // QEP quadrature count mode
    EQep1Regs.QEPCTL.bit.FREE_SOFT=2;
    EQep1Regs.QEPCTL.bit.PCRM=0b01;       // PCRM=00 mode - QPOSCNT reset on index event, PCRM=01 reset on max
    EQep1Regs.QEPCTL.bit.UTE=1;         // Unit Timeout Enable
    EQep1Regs.QEPCTL.bit.QCLM=0;        // Latch on CPU
    EQep1Regs.QPOSMAX=2047;
    //EQep1Regs.QEPCTL.bit.QPEN=1;        // QEP enable
    EQep1Regs.QCAPCTL.bit.UPPS=2;       //
    EQep1Regs.QCAPCTL.bit.CCPS=3;       // 1/8 for CAP clock
    EQep1Regs.QCAPCTL.bit.CEN=1;        // QEP Capture Enable

    /*
    //EQEP 2
    EQep2Regs.QUPRD=200000;         // Unit Timer for 1000Hz at 200 MHz SYSCLKOUT
    EQep2Regs.QDECCTL.bit.QSRC=00;      // QEP quadrature count mode
    EQep2Regs.QEPCTL.bit.FREE_SOFT=2;
    EQep2Regs.QEPCTL.bit.PCRM=0b01;       // PCRM=00 mode - QPOSCNT reset on index event, PCRM=01 reset on max
    EQep2Regs.QEPCTL.bit.UTE=1;         // Unit Timeout Enable
    EQep2Regs.QEPCTL.bit.QCLM=0;        // Latch on CPU read
    EQep2Regs.QPOSMAX=999;
    //EQep1Regs.QEPCTL.bit.QPEN=1;        // QEP enable
    EQep2Regs.QCAPCTL.bit.UPPS=2;       // 1/32 for unit position
    EQep2Regs.QCAPCTL.bit.CCPS=4;       // 1/64 for CAP clock
    EQep2Regs.QCAPCTL.bit.CEN=1;        // QEP Capture Enable
    */
}


  //
 // MAIN DRIVER CODE
//
interrupt void adca1_isr(void)
{
    GpioDataRegs.GPBSET.bit.GPIO32 = 1;
    //IpcRegs.IPCSET.bit.IPC0 = 1;                    // Wake CPU2 with interrupt

    Cntr++;

    // Step 0: do nothing for 100 cycles (startup_state_counter < 10000)
    // Step 1: measure zero current (startup_state_counter < 100)
    // Step 2: rotate motor 1 to starting position and then release ( 3 seconds startup_state_counter < 30000 * (10000/PWMMAX) )
    // Step 3: do nothing for 1000 cycles (startup_state_counter  < 1000)
    // Step 4: rotate motor 2 to starting position and then release ( 3 seconds startup_state_counter < 30000 * (10000/PWMMAX) )
    // Step 5: start normal operation

    //unsigned int startup_state_counter  = 0;
    //unsigned int startup_state_variable = 0;

    //check if startup is true
    if(startup_state_variable<5)
    {
        if(startup_state_variable==0)  // do nothing phase
        {
            v_1a_ref = 0.50;
            v_1b_ref = 0.50;
            v_1c_ref = 0.50;
            v_2a_ref = 0.50;
            v_2b_ref = 0.50;
            v_2c_ref = 0.50;
            duty_1a=(Uint16)(v_1a_ref*PWMMAX);
            duty_1b=(Uint16)(v_1b_ref*PWMMAX);
            duty_1c=(Uint16)(v_1c_ref*PWMMAX);
            duty_2a=(Uint16)(v_2a_ref*PWMMAX);
            duty_2b=(Uint16)(v_2b_ref*PWMMAX);
            duty_2c=(Uint16)(v_2c_ref*PWMMAX);
            //set reference comparator
            EPwm1Regs.CMPA.bit.CMPA = duty_1a;
            EPwm2Regs.CMPA.bit.CMPA = duty_1b;
            EPwm3Regs.CMPA.bit.CMPA = duty_1c;
            EPwm4Regs.CMPA.bit.CMPA = duty_2a;
            EPwm5Regs.CMPA.bit.CMPA = duty_2b;
            EPwm6Regs.CMPA.bit.CMPA = duty_2c;
            if(++startup_state_counter == 10000)
            {
                startup_state_variable = 1; //proceed to next step
                startup_state_counter  = 0; // reset counter
            }
        }
        if(startup_state_variable==1) // measure zero current
        {
            // read adc results
            i_1a_off = i_1a_off + (float)AdcaResultRegs.ADCRESULT0;
            i_1b_off = i_1b_off + (float)AdcbResultRegs.ADCRESULT0;
            i_2a_off = i_2a_off + (float)AdcaResultRegs.ADCRESULT1;
            i_2b_off = i_2b_off + (float)AdcbResultRegs.ADCRESULT1;
            if(++startup_state_counter == 100)
            {
                // offset calculation
                i_1a_off = i_1a_off / 100.0;
                i_1b_off = i_1b_off / 100.0;
                i_2a_off = i_2a_off / 100.0;
                i_2b_off = i_2b_off / 100.0;
                startup_state_variable = 2; //proceed to next step
                startup_state_counter  = 0; // reset counter
            }
        }

        unsigned int motor_positioning_timer = (unsigned int) (30000.0 * (10000.0/((float)PWMMAX))); // limit of the motor positioning for steps 2 & 3

        if(startup_state_variable==2) // rotate the motor to the initial position, enable eQEP1 and release
        {
            //set a-phase direction
            v_1a_ref = 0.600;
            v_1b_ref = 0.450;
            v_1c_ref = 0.450;

            if(++startup_state_counter == motor_positioning_timer)
            {
                //enable position sensor on motor 1
                EQep1Regs.QEPCTL.bit.QPEN=1;
                //reset to zero
                v_1a_ref = 0.50;
                v_1b_ref = 0.50;
                v_1c_ref = 0.50;

                startup_state_variable = 3;
                startup_state_counter  = 0;
            }
            //duty cycles
            duty_1a=(Uint16)(v_1a_ref*PWMMAX);
            duty_1b=(Uint16)(v_1b_ref*PWMMAX);
            duty_1c=(Uint16)(v_1c_ref*PWMMAX);
            //set reference comparator
            EPwm1Regs.CMPA.bit.CMPA = duty_1a;
            EPwm2Regs.CMPA.bit.CMPA = duty_1b;
            EPwm3Regs.CMPA.bit.CMPA = duty_1c;
        }

        if(startup_state_variable==3)  // do nothing phase
        {
            if(++startup_state_counter == 1000)
            {
                startup_state_variable = 4; //proceed to next step
                startup_state_counter  = 0; // reset counter
                Q_1s = 0.0;
                Q_2s = 0.0;
            }
        }

        if(startup_state_variable==4) // rotate the motor to the initial position, enable eQEP1 and release
        {
            //set a-phase direction
            v_2a_ref = 0.600;
            v_2b_ref = 0.450;
            v_2c_ref = 0.450;

            if(++startup_state_counter == motor_positioning_timer)
            {
                // enable position sensor on motor 2
                //EQep2Regs.QEPCTL.bit.QPEN=1;
                //reset to zero
                v_2a_ref = 0.50;
                v_2b_ref = 0.50;
                v_2c_ref = 0.50;

                startup_state_variable = 5;
                startup_state_counter  = 0;
            }
            //duty cycles
            duty_2a=(Uint16)(v_2a_ref*PWMMAX);
            duty_2b=(Uint16)(v_2b_ref*PWMMAX);
            duty_2c=(Uint16)(v_2c_ref*PWMMAX);
            //set reference comparator
            EPwm4Regs.CMPA.bit.CMPA = duty_2a;
            EPwm5Regs.CMPA.bit.CMPA = duty_2b;
            EPwm6Regs.CMPA.bit.CMPA = duty_2c;
        }
    } // end of startup sequence bracket
    else // <<Normal operation>>
    {
        // startup_state_counter variable now becomes the counter for the speed loop

        // speed acquisition

        // new algorithm
        if((EQep1Regs.QEPSTS.bit.UPEVNT == 1) && (EQep1Regs.QCPRDLAT != 0))
        {
            QEPcounter1++;
            Uint32 pos_delta = EQep1Regs.QCPRDLAT;
            float w_1_aux;
            w_1_aux = __divf32(1.0,(float) pos_delta);
            //w_1_aux*= 314159.265358979;  // HURST
            //w_1_aux*= 38349.519697;
            //w_1_aux*= 19174.75984857;  // CCPS=5
            w_1_aux*= 76699.03939428;  // CCPS=3
            w_1_aux = (EQep1Regs.QEPSTS.bit.COEF==0)*w_1_aux;
            w_1_aux = (EQep1Regs.QEPSTS.bit.QDF==1) ? w_1_aux : -w_1_aux; // speed direction;
            w_1_acc+= w_1_aux;
            EQep1Regs.QEPSTS.all = 0x88; // Clear errors and events
        }
        /*
        if(EQep2Regs.QEPSTS.bit.UPEVNT == 1)
        {
            QEPcounter2++;
            Uint32 pos_delta = EQep2Regs.QCPRDLAT;
            float w_2_aux;
            w_2_aux = __divf32(1.0,(float) pos_delta);
            w_2_aux*= 314159.265358979;
            w_2_aux = (EQep2Regs.QEPSTS.bit.COEF==0)*w_2_aux;
            w_2_aux = (EQep2Regs.QEPSTS.bit.QDF==1) ? w_2_aux : -w_2_aux; // speed direction;
            w_2_acc+= w_2_aux;
            EQep2Regs.QEPSTS.all = 0x88; // Clear errors and events
        }
        */
        if(++startup_state_counter == 5)
        {
            w_1 = QEPcounter1 == 0 ? w_1 : __divf32(w_1_acc, (float) QEPcounter1);
            //w_1 = __divf32(w_1_acc, (float) QEPcounter1);
            infkiller(&w_1);

            w_1_filtered = w_1_filtered*w_1_filter + (1.0f - w_1_filter)*w_1;
            w_1 = w_1_filtered;
            //w_2 = __divf32(w_2_acc, (float) QEPcounter2);
            w_1_acc = 0.0;
            //w_2_acc = 0.0;
            QEPcounter1 = 0;
            //QEPcounter2 = 0;
            startup_state_counter = 0;

             //
            // PI speed controller stuff
           //
          //PI_controller(float reference_value, float actual_value, float *error, float *q_prev, float *u, float P, float I, float UMAX, float T)

          M_1ref_ext = M_1ref_ext > M_MAX_1 ? M_MAX_1 : M_1ref_ext;
          M_1ref_ext = M_1ref_ext <-M_MAX_1 ?-M_MAX_1 : M_1ref_ext;

          M_2ref_ext = M_2ref_ext > M_MAX_2 ? M_MAX_2 : M_2ref_ext;
          M_2ref_ext = M_2ref_ext <-M_MAX_2 ?-M_MAX_2 : M_2ref_ext;

          if(w_ref_increment == 1)
          {
              if(w_ref_act < w_ref)
              {
                  if(w_ref-w_ref_act < w_ref_delta)
                  {
                      w_ref_act = w_ref;
                  }
                  else
                  {
                      w_ref_act += w_ref_delta;
                  }
              }
              else
              {
                  if(w_ref_act-w_ref < w_ref_delta)
                  {
                      w_ref_act = w_ref;
                  }
                  else
                  {
                      w_ref_act -= w_ref_delta;
                  }
              }

          }
          else
          {
              w_ref_act = w_ref;
          }


          if(control_select == 0)
          {
              PI_controller(  w_ref_act,  w_1, &E_1s, &Q_1s, &U_1s, CP_speed, CI_speed, M_MAX_1, Ts*20.0 );
          }
          else
          {
              PI_controller( -w_ref_act, -w_1, &E_2s, &Q_2s, &U_2s, CP_speed, CI_speed, M_MAX_2, Ts*20.0 );
          }
          infkiller(&Q_1s);
          infkiller(&Q_2s);
          infkiller(&U_1s);
          infkiller(&U_2s);

          M_1_ref = U_1s;
          M_2_ref = U_2s;
        }

        /*
        unsigned int Pos_1 = EQep1Regs.QPOSCNT;
        Analog_speed1  = (float) Pos_1 - Pos_1prev;
        if (Analog_speed1 < -300.0) { Analog_speed1 += 1000.0; }
        if (Analog_speed1 >  300.0) { Analog_speed1 -= 1000.0; }
        Analog_speed1 *= TWOPI*0.001;
        Pos_1prev = Pos_1;
        Analog_speed1 *= 500.0;

        Ratio = Analog_speed1 / w_1;
        */

        // Current measurement aquisition
        unsigned int sensorsample_11 = AdcaResultRegs.ADCRESULT0;
        unsigned int sensorsample_12 = AdcbResultRegs.ADCRESULT0;
        unsigned int sensorsample_21 = AdcaResultRegs.ADCRESULT1;
        unsigned int sensorsample_22 = AdcbResultRegs.ADCRESULT1;
        // Conversion to Amps
        i_1a = -((float)sensorsample_11 - i_1a_off) * I_scaler;
        i_1b = -((float)sensorsample_12 - i_1b_off) * I_scaler;
        i_2a = -((float)sensorsample_21 - i_2a_off) * I_scaler;
        i_2b = -((float)sensorsample_22 - i_2b_off) * I_scaler;

        Pos1=(unsigned int)EQep1Regs.QPOSCNT;
        //Pos2=(unsigned int)EQep2Regs.QPOSCNT;

        //unsigned int Pos_Mod1 = Pos1 - (Pos1/200)*200; HURST MOTOR
        //unsigned int Pos_Mod2 = Pos2 - (Pos2/200)*200;
        //alpha1 = ((float) Pos_Mod1)*0.005*TWOPI;
        //alpha2 = ((float) Pos_Mod2)*0.005*TWOPI;

        alpha1 = ((float)Pos1)*0.0030679615758; // 1/2048*2*pi
        //alpha1+= alpha_corr;
        //alpha2 = alpha1;
        //alpha2 = ((float)Pos2)*0.0030679615758;

        //alpha1 += w_1*4.0f*Ts*0.5f;

        //float sin1, cos1, sin2, cos2;
        sin1 = sinf(alpha1);
        cos1 = cosf(alpha1);
        //sin2 = sinf(alpha2);
        //cos2 = cosf(alpha2);
        sin2 = -sin1;
        cos2 =  cos1;

        // Clarke
        //float i_1al, i_1be, i_2al, i_2be;
        i_1al = i_1a;
        i_1be = OnePerSqrt3*(2.0*i_1b+i_1a);
        i_2al = i_2a;
        i_2be = OnePerSqrt3*(2.0*i_2b+i_2a);

        // Park
        //float i_1d, i_1q, i_2d, i_2q;
        i_1d = cos1*i_1al + sin1*i_1be;
        i_1q = cos1*i_1be - sin1*i_1al;
        i_2d = cos2*i_2al + sin2*i_2be;
        i_2q = cos2*i_2be - sin2*i_2al;


        //Median filter of D-Q currents
        register int m = 0;
        for(m=0;m<4;m++)
        {
            i_1d_buffer[m] = i_1d_buffer[m+1];
            i_1d_sorted[m] = i_1d_buffer[m];
            i_1q_buffer[m] = i_1q_buffer[m+1];
            i_1q_sorted[m] = i_1q_buffer[m];
            i_2d_buffer[m] = i_2d_buffer[m+1];
            i_2d_sorted[m] = i_2d_buffer[m];
            i_2q_buffer[m] = i_2q_buffer[m+1];
            i_2q_sorted[m] = i_2q_buffer[m];

        }
        i_1d_buffer[4] = i_1d;
        i_1d_sorted[4] = i_1d;
        i_1q_buffer[4] = i_1q;
        i_1q_sorted[4] = i_1q;
        i_2d_buffer[4] = i_2d;
        i_2d_sorted[4] = i_2d;
        i_2q_buffer[4] = i_2q;
        i_2q_sorted[4] = i_2q;
        sortfive(i_1d_sorted);
        sortfive(i_1q_sorted);
        sortfive(i_2d_sorted);
        sortfive(i_2q_sorted);

        i_1d_median = i_1d_sorted[2];
        i_1q_median = i_1q_sorted[2];
        i_2d_median = i_2d_sorted[2];
        i_2q_median = i_2q_sorted[2];
        if(Median_enable==1)
        {
            i_1d = i_1d_median;
            i_1q = i_1q_median;
            i_2d = i_2d_median;
            i_2q = i_2q_median;
        }

        // References
        //float i_1d_ref, i_1q_ref, i_2d_ref, i_2q_ref;

        i_1d_ref = 0.0;
        i_2d_ref = 0.0;
        i_1q_ref = M_CONST1*M_1_ref;
        i_2q_ref = M_CONST2*M_2_ref;

        // PI magic
        if(control_select == 0)
        {
            i_1d_ref = 0.0;
            i_1q_ref = M_CONST1*M_1_ref;
            i_2d_ref = i_2d_ref_ext;
            i_2q_ref = i_2q_ref_ext;
        }
        else
        {
            i_1d_ref = i_1d_ref_ext;
            i_1q_ref = i_1q_ref_ext;
            i_2d_ref = 0.0;
            i_2q_ref = M_CONST2*M_2_ref;
        }

        if((IPM_enable == 1) && !IpcRegs.IPCSET.bit.IPC0)
        {
            float M1_setpoint = 6.0*M1_Ps*i_1q_ref*(1.0+(M1_Ld-M1_Lq)/M1_Ps*i_1d_ref);
            float M2_setpoint = 6.0*M2_Ps*i_2q_ref*(1.0+(M2_Ld-M2_Lq)/M2_Ps*i_2d_ref);

            c1_r_w_array[0] = M1_setpoint;
            c1_r_w_array[1] = M2_setpoint;
            c1_r_w_array[2] = w_1;

            //c1_r_w_array[0] = M1_TPPI;
            //c1_r_w_array[1] = M2_TPPI;
            //c1_r_w_array[2] = w_TPPI;

            IpcRegs.IPCSET.bit.IPC0 = 1;
            //GpioDataRegs.GPBSET.bit.GPIO32 = 1;
            if(IPM_approve == 1)
            {
                i_1d_ref = c1_r_array[0];
                i_1q_ref = c1_r_array[1];
                i_2d_ref = c1_r_array[2];
                i_2q_ref = c1_r_array[3];
            }

        }


        if(++Sample_counter == SAMPLE_SIZE)
        {
            // Latch batch data
            i_1d_mean_latch = i_1d_mean;
            i_1q_mean_latch = i_1q_mean;
            i_2d_mean_latch = i_2d_mean;
            i_2q_mean_latch = i_2q_mean;
            i_1d_mean = 0.0;
            i_1q_mean = 0.0;
            i_2d_mean = 0.0;
            i_2q_mean = 0.0;

            i_1d_variance_latch = sqrt(i_1d_variance);
            i_1q_variance_latch = sqrt(i_1q_variance);
            i_2d_variance_latch = sqrt(i_2d_variance);
            i_2q_variance_latch = sqrt(i_2q_variance);
            i_1d_variance = 0.0;
            i_1q_variance = 0.0;
            i_2d_variance = 0.0;
            i_2q_variance = 0.0;

            u_1d_mean_latch = u_1d_mean;
            u_1q_mean_latch = u_1q_mean;
            u_2d_mean_latch = u_2d_mean;
            u_2q_mean_latch = u_2q_mean;
            u_1d_mean = 0.0;
            u_1q_mean = 0.0;
            u_2d_mean = 0.0;
            u_2q_mean = 0.0;

            u_1d_variance_latch = sqrt(u_1d_variance);
            u_1q_variance_latch = sqrt(u_1q_variance);
            u_2d_variance_latch = sqrt(u_2d_variance);
            u_2q_variance_latch = sqrt(u_2q_variance);
            u_1d_variance = 0.0;
            u_1q_variance = 0.0;
            u_2d_variance = 0.0;
            u_2q_variance = 0.0;

            Sample_counter  = 0;

            if(Measurement_start)
            {
                if(control_select==0)
                {
                    i_1d_ref = 0.0;
                    i_1q_ref = M_CONST1*M_1_ref;
                    i_2d_ref_ext = -6.0 + ((float) D_cnt)*0.75;
                    i_2q_ref_ext = -6.0 + ((float) Q_cnt)*0.75;
                }
                else
                {
                    i_1d_ref_ext = -4.0 + ((float) D_cnt)*0.5;
                    i_1q_ref_ext = -4.0 + ((float) Q_cnt)*0.5;
                    i_2d_ref = 0.0;
                    i_2q_ref = M_CONST2*M_1_ref;
                }

                if(Measurement_flag==3)
                {
                    if(control_select==0)
                    {
                        U_d_meas[D_cnt][Q_cnt] = u_2d_mean_latch;
                        U_q_meas[D_cnt][Q_cnt] = u_2q_mean_latch;
                        I_d_meas[D_cnt][Q_cnt] = i_2d_mean_latch;
                        I_q_meas[D_cnt][Q_cnt] = i_2q_mean_latch;
                    }
                    else
                    {
                        U_d_meas[D_cnt][Q_cnt] = u_1d_mean_latch;
                        U_q_meas[D_cnt][Q_cnt] = u_1q_mean_latch;
                        I_d_meas[D_cnt][Q_cnt] = i_1d_mean_latch;
                        I_q_meas[D_cnt][Q_cnt] = i_1q_mean_latch;
                    }

                    if(Q_cnt % 2 == 0)
                    {
                        if(++D_cnt==17)
                        {
                            D_cnt = 16;
                            if(++Q_cnt==17)
                            {
                                Q_cnt = 0;
                                Measurement_start = 0;
                                i_1d_ref_ext = 0.0;
                                i_1q_ref_ext = 0.0;
                                i_2d_ref_ext = 0.0;
                                i_2q_ref_ext = 0.0;
                            }
                        }
                    }
                    else
                    {
                        if(--D_cnt==65535)
                        {
                            D_cnt = 0;
                            if(++Q_cnt==17)
                            {
                                Q_cnt = 0;
                                Measurement_start = 0;
                                i_1d_ref_ext = 0.0;
                                i_1q_ref_ext = 0.0;
                                i_2d_ref_ext = 0.0;
                                i_2q_ref_ext = 0.0;
                            }
                        }
                    }

                    Measurement_flag = 0;
                }
                Measurement_flag++;
            }
        }

        /*----------------------------------------------------------------------------/
        /                              MPC function calling                           /
        /----------------------------------------------------------------------------*/

        if (MPC_ON == 1u){

            // Inputs
            MPC_U.Te_ref = M_1_ref;
            MPC_U.wm = w_1;
            MPC_U.Id_meas = i_1d;
            MPC_U.Iq_meas = i_1q;

            // step the MPC controller
            MPC_step();

            // Outputs
            U_1d = MPC_Y.Ud_out;
            U_1q = MPC_Y.Uq_out;

        }
        else{

            PI_controller( i_1d_ref, i_1d, &E_1d, &Q_1d, &U_1d, CP_1current, CI_1current, V_MAX, Ts );
            PI_controller( i_1q_ref, i_1q, &E_1q, &Q_1q, &U_1q, CP_1current, CI_1current, V_MAX, Ts );
        }

        PI_controller( i_2d_ref, i_2d, &E_2d, &Q_2d, &U_2d, CP_2current, CI_2current, V_MAX, Ts );
        PI_controller( i_2q_ref, i_2q, &E_2q, &Q_2q, &U_2q, CP_2current, CI_2current, V_MAX, Ts );

        infkiller(&Q_1d);
        infkiller(&Q_2d);
        infkiller(&U_1d);
        infkiller(&U_2d);

        infkiller(&Q_1q);
        infkiller(&Q_2q);
        infkiller(&U_1q);
        infkiller(&U_2q);

        // Recursive mean and variance^2 calculation
        float divisor = __divf32(1.0,(float) Sample_counter + 1.0);
        float i_1d_tmp = i_1d_mean;
        float i_1q_tmp = i_1q_mean;
        float i_2d_tmp = i_2d_mean;
        float i_2q_tmp = i_2q_mean;
        i_1d_mean = i_1d_mean + (i_1d - i_1d_mean) * divisor;
        i_1q_mean = i_1q_mean + (i_1q - i_1q_mean) * divisor;
        i_2d_mean = i_2d_mean + (i_2d - i_2d_mean) * divisor;
        i_2q_mean = i_2q_mean + (i_2q - i_2q_mean) * divisor;


        i_1d_variance = i_1d_variance + i_1d_tmp*i_1d_tmp - i_1d_mean*i_1d_mean + (i_1d*i_1d-i_1d_variance*i_1d_variance-i_1d_tmp*i_1d_tmp)*divisor;
        i_1q_variance = i_1q_variance + i_1q_tmp*i_1q_tmp - i_1q_mean*i_1q_mean + (i_1q*i_1q-i_1q_variance*i_1q_variance-i_1q_tmp*i_1q_tmp)*divisor;
        i_2d_variance = i_2d_variance + i_2d_tmp*i_2d_tmp - i_2d_mean*i_2d_mean + (i_2d*i_2d-i_2d_variance*i_2d_variance-i_2d_tmp*i_2d_tmp)*divisor;
        i_2q_variance = i_2q_variance + i_2q_tmp*i_2q_tmp - i_2q_mean*i_2q_mean + (i_2q*i_2q-i_2q_variance*i_2q_variance-i_2q_tmp*i_2q_tmp)*divisor;

        float u_1d_tmp = u_1d_mean;
        float u_1q_tmp = u_1q_mean;
        float u_2d_tmp = u_2d_mean;
        float u_2q_tmp = u_2q_mean;
        u_1d_mean = u_1d_mean + (U_1d - u_1d_mean) * divisor;
        u_1q_mean = u_1q_mean + (U_1q - u_1q_mean) * divisor;
        u_2d_mean = u_2d_mean + (U_2d - u_2d_mean) * divisor;
        u_2q_mean = u_2q_mean + (U_2q - u_2q_mean) * divisor;

        u_1d_variance = u_1d_variance + u_1d_tmp*u_1d_tmp - u_1d_mean*u_1d_mean + (U_1d*U_1d-u_1d_variance*u_1d_variance-u_1d_tmp*u_1d_tmp)*divisor;
        u_1q_variance = u_1q_variance + u_1q_tmp*u_1q_tmp - u_1q_mean*u_1q_mean + (U_1q*U_1q-u_1q_variance*u_1q_variance-u_1q_tmp*u_1q_tmp)*divisor;
        u_2d_variance = u_2d_variance + u_2d_tmp*u_2d_tmp - u_2d_mean*u_2d_mean + (U_2d*U_2d-u_2d_variance*u_2d_variance-u_2d_tmp*u_2d_tmp)*divisor;
        u_2q_variance = u_2q_variance + u_2q_tmp*u_2q_tmp - u_2q_mean*u_2q_mean + (U_2q*U_2q-u_2q_variance*u_2q_variance-u_2q_tmp*u_2q_tmp)*divisor;


        // PREDICTIVE EVALUATION
        CPU_2_CLA[0]  = i_1d;
        CPU_2_CLA[1]  = i_1q;
        if(Predictive_enable & 0x1)
        {
            Q_1d = 0.0;
            Q_1q = 0.0;
            float u_1d = cos1*U_1d_opt_latch + sin1*U_1q_opt_latch;
            float u_1q = cos1*U_1q_opt_latch - sin1*U_1d_opt_latch;
            CPU_2_CLA[2]  = u_1d;
            CPU_2_CLA[3]  = u_1q;
        }
        else
        {
            CPU_2_CLA[2]  = U_1d;
            CPU_2_CLA[3]  = U_1q;
        }

        CPU_2_CLA[4]  = M1_Rs;
        CPU_2_CLA[5]  = M1_Ld;
        CPU_2_CLA[6]  = M1_Lq;
        CPU_2_CLA[7]  = M1_Ps;

        CPU_2_CLA[8]  = alpha1;
        CPU_2_CLA[9]  = 4.0*w_1;
        CPU_2_CLA[10] = Ts;

        CPU_2_CLA[11] = i_1d_ref;
        CPU_2_CLA[12] = i_1q_ref;
        CPU_2_CLA[13] = Current_Kalman_gain;

        Cla1ForceTask1andWait();

        i_1d_pred = CLA_2_CPU[0];
        i_1q_pred = CLA_2_CPU[1];

        U_1d_CLA = cos1*CLA_2_CPU[2] + sin1*CLA_2_CPU[3];
        U_1q_CLA = cos1*CLA_2_CPU[3] - sin1*CLA_2_CPU[2];

        if(Predictive_enable & 0x1)
        {
            v_1al = CLA_2_CPU[2];
            v_1be = CLA_2_CPU[3];
            U_1d_opt_latch = CLA_2_CPU[2];
            U_1q_opt_latch = CLA_2_CPU[3];
        }
        else
        {
            float U_1d_ref = U_1d;
            float U_1q_ref = U_1q;
            U_1d_comp = -w_1*M1_Lq*i_1q*4.0;
            U_1d_comp*= 0.04166666666666666666666666666667;
            U_1q_comp = w_1*M1_Ld*i_1d*4.0 + w_1*4.0*M1_Ps;
            U_1q_comp*= 0.04166666666666666666666666666667;


            if(Decoupling_enable == 1)
            {
                U_1d_ref += U_1d_comp;
                U_1q_ref += U_1q_comp;
            }
            v_1al = cos1*U_1d_ref - sin1*U_1q_ref;
            v_1be = cos1*U_1q_ref + sin1*U_1d_ref;
            U_1d_opt_latch = U_1d_ref;
            U_1q_opt_latch = U_1q_ref;
        }



        CPU_2_CLA[0]  = i_2d;
        CPU_2_CLA[1]  = i_2q;
        if(Predictive_enable & 0x2)
        {
            Q_2d = 0.0;
            Q_2q = 0.0;
            float u_2d = cos2*U_2d_opt_latch + sin2*U_2q_opt_latch;
            float u_2q = cos2*U_2q_opt_latch - sin2*U_2d_opt_latch;
            CPU_2_CLA[2]  = u_2d;
            CPU_2_CLA[3]  = u_2q;
        }
        else
        {
            CPU_2_CLA[2]  = U_2d;
            CPU_2_CLA[3]  = U_2q;
        }

        CPU_2_CLA[4]  = M2_Rs;
        CPU_2_CLA[5]  = M2_Ld;
        CPU_2_CLA[6]  = M2_Lq;
        CPU_2_CLA[7]  = M2_Ps;

        CPU_2_CLA[8]  = -1.0*alpha1;
        CPU_2_CLA[9]  = -4.0*w_1;
        CPU_2_CLA[10] = Ts;

        CPU_2_CLA[11] = i_2d_ref;
        CPU_2_CLA[12] = i_2q_ref;
        CPU_2_CLA[13] = Current_Kalman_gain;

        Cla1ForceTask1andWait();

        i_2d_pred = CLA_2_CPU[0];
        i_2q_pred = CLA_2_CPU[1];


        if(Predictive_enable & 0x2)
        {
            v_2al = CLA_2_CPU[2];
            v_2be = CLA_2_CPU[3];
            U_2d_opt_latch = CLA_2_CPU[2];
            U_2q_opt_latch = CLA_2_CPU[3];
        }
        else
        {
            float U_2d_ref = U_2d;
            float U_2q_ref = U_2q;
            if(Decoupling_enable == 1)
            {
                U_2d_ref -= w_1*M2_Lq*i_2q*0.16666667;
                U_2q_ref += w_1*M2_Ld*i_2d*0.16666667;
            }
            v_2al = cos2*U_2d_ref - sin2*U_2q_ref;
            v_2be = cos2*U_2q_ref + sin2*U_2d_ref;
            U_2d_opt_latch = U_2d_ref;
            U_2q_opt_latch = U_2q_ref;
        }



        // Inverse Clarke
        //float v_1a, v_1b, v_1c, v_2a, v_2b, v_2c;
        v_1a =  v_1al;
        v_1b =  v_1be*Sqrt3Per2 - v_1al*0.5;
        v_1c =  v_1b - 2.0*v_1be*Sqrt3Per2;
        v_2a =  v_2al;
        v_2b =  v_2be*Sqrt3Per2 - v_2al*0.5;
        v_2c =  v_2b - 2.0*v_2be*Sqrt3Per2;


        /*----------------------------------------------------------------------------/
        /                              SVPWM                                          /
        /----------------------------------------------------------------------------*/
        //float v_1mid;
        //float v_2mid;

        register int ab, ac, bc;
        ab = (v_1a > v_1b);
        ac = (v_1a > v_1c);
        bc = (v_1b > v_1c);
        ac = ab ? !ac : ac;
        bc = ab ? !bc : bc;
        v_1mid  = ( ac &&  bc)*v_1a;
        v_1mid += (!ac && !bc)*v_1b;
        v_1mid += (!ac &&  bc)*v_1c;
        v_1mid *= 0.5;
        v_1mid += 0.5;
        // Reference vectors
        v_1a_ref = v_1a + v_1mid;
        v_1b_ref = v_1b + v_1mid;
        v_1c_ref = v_1c + v_1mid;


        ab = (v_2a > v_2b);
        ac = (v_2a > v_2c);
        bc = (v_2b > v_2c);
        ac = ab ? !ac : ac;
        bc = ab ? !bc : bc;
        v_2mid  = ( ac &&  bc)*v_2a;
        v_2mid += (!ac && !bc)*v_2b;
        v_2mid += (!ac &&  bc)*v_2c;
        v_2mid *= 0.5;
        v_2mid += 0.5;
        // Reference vectors
        v_2a_ref = v_2a + v_2mid;
        v_2b_ref = v_2b + v_2mid;
        v_2c_ref = v_2c + v_2mid;

          //
         // Actuation
        //

        //remove overflow
        v_1a_ref = v_1a_ref < 0 ? 0 : v_1a_ref;
        v_1b_ref = v_1b_ref < 0 ? 0 : v_1b_ref;
        v_1c_ref = v_1c_ref < 0 ? 0 : v_1c_ref;
        v_2a_ref = v_2a_ref < 0 ? 0 : v_2a_ref;
        v_2b_ref = v_2b_ref < 0 ? 0 : v_2b_ref;
        v_2c_ref = v_2c_ref < 0 ? 0 : v_2c_ref;


        //duty cycles
        duty_1a=(Uint16)(v_1a_ref*PWMMAX);
        duty_1b=(Uint16)(v_1b_ref*PWMMAX);
        duty_1c=(Uint16)(v_1c_ref*PWMMAX);
        duty_2a=(Uint16)(v_2a_ref*PWMMAX);
        duty_2b=(Uint16)(v_2b_ref*PWMMAX);
        duty_2c=(Uint16)(v_2c_ref*PWMMAX);


        //set reference comparator
        EPwm1Regs.CMPA.bit.CMPA = duty_1a;
        EPwm2Regs.CMPA.bit.CMPA = duty_1b;
        EPwm3Regs.CMPA.bit.CMPA = duty_1c;
        EPwm4Regs.CMPA.bit.CMPA = duty_2a;
        EPwm5Regs.CMPA.bit.CMPA = duty_2b;
        EPwm6Regs.CMPA.bit.CMPA = duty_2c;

        // DACOUTS

        //OutputDACA(v_1a_ref,2048.0);
        //OutputDACB(v_1b_ref,2048.0);

        float DACA_value, DACB_value;
        float DACA_gain,  DACB_gain;

        int DAC_SEL;

        DAC_SEL = DAC_select;
        DAC_SEL <<= 1;

        DACA_value = *DAC_VAL[DAC_SEL];
        DACA_gain  =  DAC_SCA[DAC_SEL];
        DAC_SEL++;
        DACB_value = *DAC_VAL[DAC_SEL];
        DACB_gain  =  DAC_SCA[DAC_SEL];


        /* LEGACY DAC
        if(DAC_select == 0)
        {
            DACA_value = i_2q_ref; DACA_gain  = 300.0;
            DACB_value = i_2q;     DACB_gain  = 300.0;
        }
        if(DAC_select == 1)
        {
            DACA_value = U_2d;     DACA_gain  = 3000.0;
            DACB_value = U_2q;     DACB_gain  = 3000.0;
        }
        if(DAC_select == 2)
        {
            DACA_value = w_ref; DACA_gain  = 5.0;
            DACB_value = w_1;     DACB_gain  = 5.0;
        }
        if(DAC_select == 3)
        {
            DACA_value = v_1a_ref; DACA_gain  = 1848.0;
            DACB_value = 0.0;      DACB_gain  =    0.0;
        }
        if(DAC_select == 4)
        {
            DACA_value = sin1;      DACA_gain  = 2000.0;
            DACB_value = cos1;      DACB_gain  = 2000.0;
        }
        if(DAC_select == 5)
        {
            DACA_value = i_1a;      DACA_gain  = 200.0;
            DACB_value = i_1b;      DACB_gain  = 200.0;
        }
        if(DAC_select == 6)
        {
            DACA_value = i_2a;      DACA_gain  = 200.0;
            DACB_value = i_2b;      DACB_gain  = 200.0;
        }
        if(DAC_select == 7)
        {
            DACA_value = i_2d;           DACA_gain  = 200.0;
            DACB_value = i_2d_pred;      DACB_gain  = 200.0;
        }
        if(DAC_select == 8)
        {
            DACA_value = i_2q;           DACA_gain  = 200.0;
            DACB_value = i_2q_pred;      DACB_gain  = 200.0;
        }
        if(DAC_select == 9)
        {
            DACA_value = v_2al;         DACA_gain  = 2000.0;
            DACB_value = U_2d_opt_latch;      DACB_gain  = 2000.0;
        }
        if(DAC_select == 10)
        {
            DACA_value = v_2be;         DACA_gain  = 2000.0;
            DACB_value = U_2q_opt_latch;      DACB_gain  = 2000.0;
        }
        */

        OutputDACA(DACA_value, DACA_gain);
        OutputDACB(DACB_value, DACB_gain);



    } // End of Control Code and State-Machine



    GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;
    //Acknowledge the Interrupt
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear INT1 flag
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

  //
 // DAC ENABLE (pins 30 (DACA) & 70 (DACB)
//
void initDAC()
{
    EALLOW;
    DacaRegs.DACCTL.bit.DACREFSEL=1;
    DacaRegs.DACOUTEN.bit.DACOUTEN=1;
    DacaRegs.DACVALS.all = 0;

    DacbRegs.DACCTL.bit.DACREFSEL=1;
    DacbRegs.DACOUTEN.bit.DACOUTEN=1;
    DacbRegs.DACVALS.all = 0;

    DELAY_US(10);
    EDIS;
}
// DAC OUTPUT
void OutputDACA(float value, float gain)
{
    int temp;
    temp = (int)(value*gain);
    DacaRegs.DACVALS.bit.DACVALS = temp + 2047;
}

void OutputDACB(float value, float gain)
{
    int temp;
    temp = (int)(value*gain);
    DacbRegs.DACVALS.bit.DACVALS = temp + 2047;
}

  //
 // EPWM Configuration
//
void EPWM_Config(Uint16 period)
{
Uint16 j;
   for (j=0;j<6;j++)
   {
    (*ePWM[j]).TBPRD = period-1;                        // PWM frequency = 1 / period
    (*ePWM[j]).CMPA.bit.CMPA = period/2;             // set duty 50% initially
    (*ePWM[j]).CMPA.bit.CMPAHR = 0;             // initialize HRPWM extension
    (*ePWM[j]).CMPB.bit.CMPB = period/2;                       // set duty 50% initially
    (*ePWM[j]).TBPHS.all = 0;
    //(*ePWM[j]).TBCTR = 0;
    (*ePWM[j]).TBCTR = period/2;
    (*ePWM[j]).TBCTL.bit.PRDLD = TB_IMMEDIATE;
    (*ePWM[j]).TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN;
    if (j==0)
    {

        (*ePWM[j]).TBCTL.bit.PHSEN = TB_DISABLE; //Disable hardware synch for EPWM1
        (*ePWM[j]).TBCTL.bit.SYNCOSEL = TB_CTR_ZERO ; //TB_CTR_ZEROTB_SYNC_DISABLE
    }
    else
    {
        (*ePWM[j]).TBCTL.bit.PHSEN = TB_ENABLE;
        (*ePWM[j]).TBCTL.bit.SYNCOSEL = TB_SYNC_IN;
    }

    (*ePWM[j]).TBCTL.bit.HSPCLKDIV = TB_DIV1;
    (*ePWM[j]).TBPHS.all = 0x0000;
    (*ePWM[j]).TBCTL.bit.CLKDIV = TB_DIV1;
    (*ePWM[j]).TBCTL.bit.FREE_SOFT = 11;

    (*ePWM[j]).CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; //CC_CTR_ZERO
    (*ePWM[j]).CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    (*ePWM[j]).CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    (*ePWM[j]).CMPCTL.bit.SHDWBMODE = CC_SHADOW;

    (*ePWM[j]).AQCTLA.bit.CAD = AQ_SET;               // PWM toggle high/low
    (*ePWM[j]).AQCTLA.bit.CAU = AQ_CLEAR;

    (*ePWM[j]).AQSFRC.bit.RLDCSF = 0b10;
    (*ePWM[j]).DBCTL.bit.OUT_MODE = DB_FULL_ENABLE; // Enable Dead-band module
    (*ePWM[j]).DBCTL.bit.POLSEL = DB_ACTV_HIC;      // Active High Complementary (AHC)
    (*ePWM[j]).DBRED.bit.DBRED = 40;                // RED = 100 TBCLKs = 0.2us
    (*ePWM[j]).DBFED.bit.DBFED = 40;

   }

   EPwm1Regs.ETSEL.bit.SOCAEN  = 1;         // Enable SOC on A group
   EPwm1Regs.ETSEL.bit.SOCASEL = 1;         // sampling at zero
   EPwm1Regs.ETPS.bit.SOCAPRD  = 1;         // Generate pulse on 1st event

   //There is no curse in Elvish, Entish, or the tongues of Men for this treachery (Default setting for SOCAEN is 1 WTF)
   EPwm2Regs.ETSEL.bit.SOCAEN  = 0;
   EPwm3Regs.ETSEL.bit.SOCAEN  = 0;
   EPwm4Regs.ETSEL.bit.SOCAEN  = 0;
   EPwm5Regs.ETSEL.bit.SOCAEN  = 0;
   EPwm6Regs.ETSEL.bit.SOCAEN  = 0;
}

void PI_controller(float reference_value, float actual_value, float *error, float *q_prev, float *u, float P, float I, float UMAX, float T)
{
    float error_prev = *error;
    *error = reference_value - actual_value;
    float q;
    q = *q_prev + P*(*error - error_prev) + T*(I*error_prev + *u - *q_prev);
    *q_prev = q;

    float actuation;
    actuation = q;
    actuation = actuation >  UMAX ?  UMAX : actuation;
    actuation = actuation < -UMAX ? -UMAX : actuation;
    *u = actuation;
}

void swap(float*a,float*b)
{
    if(*a>*b)
    {
        float temp = *a;
        *a = *b;
        *b = temp;
    }
}

void sortfive(float *array)
{
    swap(&array[0],&array[3]);
    swap(&array[1],&array[4]);

    swap(&array[0],&array[2]);
    swap(&array[1],&array[3]);

    swap(&array[0],&array[1]);
    swap(&array[2],&array[4]);

    swap(&array[1],&array[2]);
    swap(&array[3],&array[4]);

    swap(&array[2],&array[3]);
}

void infkiller(float *value)
{
    if(*value == INFINITY)
    {
        *value = 0.0;
    }
    if(*value ==-INFINITY)
    {
        *value = 0.0;
    }

}


__interrupt void Record_IPM(void)
{
    IPM_i_1d = c1_r_array[0];
    IPM_i_1q = c1_r_array[1];
    IPM_i_2d = c1_r_array[2];
    IPM_i_2q = c1_r_array[3];


    IPM_M1 = 1.5*4.0*IPM_i_1q*(M1_Ps+(M1_Ld-M1_Lq)*IPM_i_1d);
    IPM_M2 = 1.5*4.0*IPM_i_2q*(M2_Ps+(M2_Ld-M2_Lq)*IPM_i_2d);
    //IPM_enable = 0;
    IpcRegs.IPCACK.bit.IPC1 = 1;                // Clear the interrupt
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
    //GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;
}

   //====================================\\
  //           CLA DEFINITIONS            \\
 // DO NOT TOUCH ANYTHING BELOW THIS POINT \\
//==========================================\\

void CLA_configClaMemory(void)
{
    //IpcSync(5);

    extern uint32_t Cla1funcsRunStart, Cla1funcsLoadStart, Cla1funcsLoadSize;
    EALLOW;

#ifdef _FLASH
    //
    // Copy over code from FLASH to RAM
    //
    memcpy((uint32_t *)&Cla1funcsRunStart, (uint32_t *)&Cla1funcsLoadStart,
            (uint32_t)&Cla1funcsLoadSize);
#endif //_FLASH

    //
    // Initialize and wait for CLA1ToCPUMsgRAM
    //
    MemCfgRegs.MSGxINIT.bit.INIT_CLA1TOCPU = 1;
    while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CLA1TOCPU != 1){};

    //
    // Initialize and wait for CPUToCLA1MsgRAM
    //
    MemCfgRegs.MSGxINIT.bit.INIT_CPUTOCLA1 = 1;
    while(MemCfgRegs.MSGxINITDONE.bit.INITDONE_CPUTOCLA1 != 1){};

    //
    // Select LS4RAM and LS5RAM to be the programming space for the CLA
    // First configure the CLA to be the master for LS4 and LS5 and then
    // set the space to be a program block
    //

    MemCfgRegs.LSxMSEL.bit.MSEL_LS4 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS4 = 1;
    MemCfgRegs.LSxMSEL.bit.MSEL_LS5 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS5 = 1;

    //
    // Next configure LS0RAM and LS1RAM as data spaces for the CLA
    // First configure the CLA to be the master for LS0(1) and then
    // set the spaces to be code blocks
    //
    MemCfgRegs.LSxMSEL.bit.MSEL_LS0 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS0 = 0;

    MemCfgRegs.LSxMSEL.bit.MSEL_LS1 = 1;
    MemCfgRegs.LSxCLAPGM.bit.CLAPGM_LS1 = 0;

    EDIS;
} //CLA_configClaMemory


void CLA_initCpu1Cla1(void)
{
    //
    // Compute all CLA task vectors
    // On Type-1 CLAs the MVECT registers accept full 16-bit task addresses as
    // opposed to offsets used on older Type-0 CLAs
    //
    EALLOW;
    Cla1Regs.MVECT1 = (uint16_t)(&Cla1Task1);
    Cla1Regs.MVECT2 = (uint16_t)(&Cla1Task2);
    Cla1Regs.MVECT3 = (uint16_t)(&Cla1Task3);
    Cla1Regs.MVECT4 = (uint16_t)(&Cla1Task4);
    Cla1Regs.MVECT5 = (uint16_t)(&Cla1Task5);
    Cla1Regs.MVECT6 = (uint16_t)(&Cla1Task6);
    Cla1Regs.MVECT7 = (uint16_t)(&Cla1Task7);
    Cla1Regs.MVECT8 = (uint16_t)(&Cla1Task8);

    //
    // Enable the IACK instruction to start a task on CLA in software
    // for all  8 CLA tasks. Also, globally enable all 8 tasks (or a
    // subset of tasks) by writing to their respective bits in the
    // MIER register
    //
    Cla1Regs.MCTL.bit.IACKE = 1;
    Cla1Regs.MIER.all = 0x00FF;

    //
    // Configure the vectors for the end-of-task interrupt for all
    // 8 tasks
    //
    PieVectTable.CLA1_1_INT = &cla1Isr1;
    PieVectTable.CLA1_2_INT = &cla1Isr2;
    PieVectTable.CLA1_3_INT = &cla1Isr3;
    PieVectTable.CLA1_4_INT = &cla1Isr4;
    PieVectTable.CLA1_5_INT = &cla1Isr5;
    PieVectTable.CLA1_6_INT = &cla1Isr6;
    PieVectTable.CLA1_7_INT = &cla1Isr7;
    PieVectTable.CLA1_8_INT = &cla1Isr8;

    //
    // Enable CLA interrupts at the group and subgroup levels
    //
    PieCtrlRegs.PIEIER11.all = 0xFFFF;
    IER |= (M_INT11 );
    EDIS;

} //CLA_initCpu1Cla1


  //
 // CLA interrups
//
__interrupt void cla1Isr1 () { PieCtrlRegs.PIEACK.all = M_INT11; }
__interrupt void cla1Isr2 () { PieCtrlRegs.PIEACK.all = M_INT11; }
__interrupt void cla1Isr3 () { PieCtrlRegs.PIEACK.all = M_INT11; }
__interrupt void cla1Isr4 () { PieCtrlRegs.PIEACK.all = M_INT11; }
__interrupt void cla1Isr5 () { PieCtrlRegs.PIEACK.all = M_INT11; }
__interrupt void cla1Isr6 () { PieCtrlRegs.PIEACK.all = M_INT11; }
__interrupt void cla1Isr7 () { PieCtrlRegs.PIEACK.all = M_INT11; }
__interrupt void cla1Isr8 () { PieCtrlRegs.PIEACK.all = M_INT11; }

    //=============//
   //             //
  // End of File //
 //             //
//=============//
