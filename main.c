#include "Settings.h"
#include "DSP2803x_Device.h"
#include "DPlib.h"
#include "IQmathLib.h"

#include "sineTable_50Hz.h"
#include "DSP2803x_EPwm_defines.h"
#include "ups6kva.h"
#include "SineAnalyzer_AF.h"
#define FLASH               // Uncomment for FLASH config

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// FUNCTION PROTOTYPES
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// Add protoypes of functions being used in the project here
void DeviceInit(void);
void PieCntlInit(void);
void PieVectTableInit(void);

void InitFlash(void);
void MemCopy();
extern void init_ADC();
extern void ePWM_prefare();

// Used for running BackGround in flash, and ISR in RAM.
extern Uint16 *RamfuncsLoadStart, *RamfuncsLoadEnd, *RamfuncsRunStart;

void init_T0(void);
void init_T1(void);
void init_T2(void);
void Coef_fill();
void Net_connect();

#pragma CODE_SECTION(int_EPWM6, "ramfuncs");
#pragma CODE_SECTION(SECONDARY_ISR, "ramfuncs");

//-------------------------------- DPLIB --------------------------------------------
void ADC_SOC_CNF(int ChSel[], int Trigsel[], int ACQPS[], int IntChSel, int mode);
void PWM_1ch_UpDwnCnt_CNF(int16 n, int16 period, int16 mode, int16 phase, int16 DB);
void PWM_1ch_UpDwnCntCompl_CNF(int16 n, int16 period, int16 mode, int16 phase, int16 DB);
//-----------------------------------------------------------------------------------

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
// VARIABLE DECLARATIONS - GENERAL
//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

/////////////////////////////////ADCDRV_1ch Variables/////////////////////////////////////
volatile int32  ADCout_V = 0;
volatile int32  ADCout_HALF = 0;
volatile int32  ADCout_I = 0;
//////////////////////////////////2p2z Variables//////////////////////////////////////////
volatile int32 V_Ref = 0;
volatile int32 V_Fdb = 0;
volatile int32 V_Out = 0;

volatile int32 I_Ref = 0;
volatile int32 I_Fdb = 0;
volatile int32 I_Fdb_rect = 0;
volatile int32 I_Out = 0;
//////////////////////////////////Internal Variables//////////////////////////////////////
volatile int32  Zero_Duty = _IQ24(0.5);
volatile int32  Duty = 0;

//////////////////////////////////AC_DC DP_LIB Variables//////////////////////////////////
volatile long Ipfc=0;//K_dcm_corr=0;
volatile long Vbus=0;
volatile long Vbus_H=0;
volatile long Vbus_L=0;
volatile long VL_fb=0;
volatile long VN_fb=0;
volatile long I_SW_O=0;
volatile long Vref_Iout=0;

volatile long DutyA;
volatile long Vrect=0, VinvSqr=0, VrectAvg=0, VbusAvg_H=0, VbusAvg_L=0, VbusAvg=0, VrectRMS=0, Freq_Vin=0;
volatile long IoutAvg=0;
volatile long IswAvg=0;
volatile long IinAvg=0;
volatile long VbusVcmd,PFCIcmd;             //PFCIcmd_avg;

volatile long   KoutTargetSlewed;
volatile long   SinTableSlewed;

volatile long   VbusTargetSlewed;           // Slewed set point for the voltage loop
volatile long   VbusSlewRate = 1500;        // Voltage loop Slew rate adjustment (Q24)
volatile long   pfc_slew_temp;              // Temp variable: used only if implementing
                                            // slew rate control in the slower state machine
long    temp_zero;

//int   init_boost = 10240;                 // Small boost command when PFC is enabled the first time
int     init_boost = 4000;                  // Small boost command when PFC is enabled the first time
int16   start_flag, run_flag, OV_flag=0;

SineAnalyzer sine_mainsV = SineAnalyzer_DEFAULTS;
// Used for ADC Configuration
int ChSel[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int TrigSel[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int ACQPS[16] = {8,8,8,8,8,8,8,8,8,8,8,8,8,8,8,8};
//////////////////////////////////////////////////

#pragma DATA_SECTION(CNTL_2P2Z_CoefStruct1, "CNTL_2P2Z_Coef");
struct  CNTL_2P2Z_CoefStruct CNTL_2P2Z_CoefStruct1;    //объявляем экземпляр нового типа данных (для контура тока корректора)
struct  CNTL_2P2Z_CoefStruct CNTL_2P2Z_CoefStruct2;    //объявляем экземпляр нового типа данных (для контура напряжения корректора)

volatile signed int sampleCount;                // счетчик табличной нарезки

volatile unsigned int PosAngle = 0;             //признак отсчета угла по "+" волне
volatile unsigned int NegAngle = 0;             //признак отсчета угла по "-" волне
volatile unsigned int SyncPosFlag =0;
volatile unsigned int SyncNegFlag =0;
//volatile int fire_angle = 240;                    // заносим значение выраженное в ШИМ-периодах (чуть < 10 мс)
volatile int fire_angle = 270;                     // заносим значение выраженное в ШИМ-периодах (чуть > 10 мс)
volatile int fire_angle_count = 0;
volatile int fire_angle_min = 0;                   // признак того, что угол отпирания минимален- нарезка закончена

volatile int ab_run_flag = 0;


volatile int16 VTimer0[2];                         // софт таймер, два экземпляра для отсчета длит. имп. вкл тиристора
volatile int16 VTimer1;                            // софт таймер для отсчета угла вкл верхнего тиристора
volatile int16 VTimer2;                            // софт таймер для отсчета угла вкл нижнего тиристора
volatile unsigned int VS_H_f = 0;                  // флаг импульса управления верхним тиристором
volatile unsigned int VS_L_f = 0;                  // флаг импульса управления нижним тиристором



// -------------------------------- FRAMEWORK --------------------------------------
// State Machine function prototypes
//----------------------------------------------------------------------------------
// Alpha states
void A0(void);  //state A0
//void B0(void);    //state B0
void C0(void);  //state C0

// A branch states
void A1(void);  //state A1
void A2(void);  //state A2
//void A3(void);    //state A3

// B branch states
//void B1(void);    //state B1
//void B2(void);    //state B2
//void B3(void);    //state B3

// C branch states
void C1(void);  //state C1
void C2(void);  //state C1
void C3(void);  //state C1


// Variable declarations
void (*Alpha_State_Ptr)(void);  // Base States pointer
void (*A_Task_Ptr)(void);       // State pointer A branch
void (*B_Task_Ptr)(void);       // State pointer B branch
void (*C_Task_Ptr)(void);       // State pointer C branch

void loop(void);
#pragma CODE_SECTION(loop, "ramfuncs");



volatile struct EPWM_REGS *ePWM[] =
                  { &EPwm1Regs,         //intentional: (ePWM[0] not used)
                    &EPwm1Regs,
                    &EPwm2Regs,
                    &EPwm3Regs,
                    &EPwm4Regs,
                    &EPwm5Regs,
                    &EPwm6Regs,
                    &EPwm7Regs,
                  };

// ---------------------------------- USER -----------------------------------------
#ifndef FLASH
#pragma DATA_SECTION(V_OUT_INT_array,"ServiceData");
#pragma DATA_SECTION(V_IN_array,"ServiceData");
long    V_OUT_INT_array[512];                  //массив значений x(n)
long    V_IN_array[512];                       //массив значений x(n)
unsigned int x_i;
#endif
volatile unsigned int SoftStartThyristors=0;             // флаг плавного запуска тиристоров
volatile unsigned int VbusavgH_f=0;
volatile unsigned int VbusavgL_f=0;
volatile unsigned int Vbusavg_f=0;
volatile unsigned int Fault=0;

volatile unsigned long Iout_fault = 0;
volatile unsigned long Isw_fault = 0;
volatile unsigned long Iin_fault=0;
void main(void)
{
//=================================================================================
//  INITIALISATION - General
//=================================================================================

    DeviceInit(); // Device Life support & GPIO.

#ifdef FLASH
    // Copy time critical code and Flash setup code to RAM.
    // The  RamfuncsLoadStart, RamfuncsLoadEnd, and RamfuncsRunStart
    // symbols are created by the linker. Refer to the linker files.
    MemCopy(&RamfuncsLoadStart, &RamfuncsLoadEnd, &RamfuncsRunStart);
    // Call Flash Initialization to setup flash waitstates.
    // This function must reside in RAM.
    InitFlash(); // Call the flash wrapper init function.
    //SCIA_Init(15000000, 57600); // 15000000 is the LSPCLK or the Clock used for the SCI Module
                                // 57600 is the Baudrate desired of the SCI module
#endif //(FLASH)

    init_T0();
    init_T1();
    init_T2();
    init_ADC();


    //sine analyzer initialization
    sine_mainsV.Vin=0;
   // sine_mainsV.SampleFreq=_IQ15(5120.0);
    sine_mainsV.SampleFreq=_IQ15(5128.0);   // 1/195us=5128Hz
    sine_mainsV.Threshold=_IQ15(0.1);       //(0.02);
    sine_mainsV.antifreezeperiod = 512;     //512*195us=0.1ms=10Hz
    // End sine analyzer initialization

    // Configure ADC to be triggered from EPWM1 Period event
    //Map channel to ADC Pin
    ChSel[0]=0xB;       //Map channel 0 to pin ADC-B3   V_OUT_INT
    ChSel[1]=0x3;       //Map channel 0 to pin ADC-A3   HALF_REF
    ChSel[2]=0x8;       //Map channel 0 to pin ADC-B0   I_OUT


    ChSel[3]=0x9;       //Ipfc      I_IN
    ChSel[4]=0xA;       //Vbus      Vbus_H
    ChSel[5]=0x1;       //VL_fb     V_IN
    ChSel[6]=0xC;       //VbusL     Vbus_L
    ChSel[7]=0x2;       //I_SW_O

    // Select Trigger Event for ADC conversion
    TrigSel[0]= ADCTRIG_EPWM1_SOCA;
    TrigSel[1]= ADCTRIG_EPWM1_SOCA;
    TrigSel[2]= ADCTRIG_EPWM1_SOCA;

    TrigSel[3]= ADCTRIG_EPWM1_SOCA;         //1 было
    TrigSel[4]= ADCTRIG_EPWM1_SOCA;         //1
    TrigSel[5]= ADCTRIG_EPWM1_SOCA;         //1
    TrigSel[6]= ADCTRIG_EPWM1_SOCA;         //1
    TrigSel[7]= ADCTRIG_EPWM1_SOCA;         //1
    // associate the appropriate peripheral trigger to the ADC channel

    // Configure the ADC with auto interrupt clear mode
    // ADC interrupt after EOC of channel 0
    ADC_SOC_CNF(ChSel,TrigSel,ACQPS,1,0);       //SOC1 is linked with ADCINT1

    // Configure the EPWM1,2,3 to issue the SOC (start-of-conversion)
    EPwm3Regs.ETSEL.bit.SOCAEN  = 1;
    EPwm3Regs.ETSEL.bit.SOCASEL = ET_CTR_PRD;   // Use PRD event as trigger for ADC SOC
    EPwm3Regs.ETPS.bit.SOCAPRD  = ET_1ST;        // Generate pulse on every event

    ///////////////////////////////////////////////////PFC/////////////////////////////////////////////////////////
    EPwm1Regs.ETSEL.bit.SOCAEN  = 1;
    EPwm1Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;  // Use PRD event as trigger for ADC SOC
    EPwm1Regs.ETPS.bit.SOCAPRD  = ET_1ST;        // Generate pulse on every event

    EPwm1Regs.ETSEL.bit.SOCBEN  = 1;
    EPwm1Regs.ETSEL.bit.SOCBSEL = ET_CTRU_CMPB; //  Use CMPB event when up count as trigger for DPL_-_PFC at positive wave
    EPwm1Regs.CMPB = 120;
    EPwm1Regs.ETPS.bit.SOCBPRD  = ET_1ST;        // Generate pulse on every event

    EPwm2Regs.ETSEL.bit.SOCAEN  = 1;
    EPwm2Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;  // Use PRD event as trigger for ADC SOC
    EPwm2Regs.ETPS.bit.SOCAPRD  = ET_1ST;        // Generate pulse on every event

    EPwm2Regs.ETSEL.bit.SOCBEN  = 1;
    EPwm2Regs.ETSEL.bit.SOCBSEL = ET_CTRU_CMPB;  // Use CMPB event when up count as trigger for DPL_ISR_PFC at negative wave
    EPwm2Regs.CMPB = 120;
    EPwm2Regs.ETPS.bit.SOCBPRD  = ET_1ST;        // Generate pulse on every event

    // Configure PWM1 for switching frequency 25.6 Khz, @60Mhz CPU clock =>
    //period = (60Mhz/25600 hz) = 2340
    PWM_1ch_UpDwnCnt_CNF(6,2340,1,0,150);       //ePWM6 управление верхним ключом INV
    PWM_1ch_UpDwnCntCompl_CNF(3,2340,1,0,150);  //ePWM3 управление нижним ключом INV

    PWM_1ch_UpDwnCnt_CNF(2,2340,1,0,120);       //ePWM2 управление верхним ключом PFC
    PWM_1ch_UpDwnCnt_CNF(1,2340,1,0,120);       //ePWM1 управление нижним ключом PFC

    // Digital Power CLA(DP) library initialisation
    DPL_Init();             // initialize DP library
    Coef_fill();
    Net_connect();

    ePWM_prefare();


    CpuTimer0Regs.PRD.all =  mSec1;         // A tasks
    CpuTimer1Regs.PRD.all =  fire_angle;    // задействую аппаратный таймер для измерения угла тиристора
    CpuTimer2Regs.PRD.all =  mSec10;        // C tasks

    // Tasks State-machine init
    Alpha_State_Ptr = &A0;
    A_Task_Ptr = &A1;
    C_Task_Ptr = &C1;

    // Enable global Interrupts and higher priority real-time debug events:
    EINT;   // Enable Global interrupt INTM
    ERTM;   // Enable Global real-time interrupt DBGM
    //END OF INTERRUPTS SETUP


    loop();



}//end main

void loop (void)
{
  for(;;)
  {
      (*Alpha_State_Ptr)(); // jump to an Alpha state (A0,B0,...)
        //===========================================================
  }  //end for
}  //end loop

void A0(void)
{
    // loop rate synchronizer for A-tasks
    if(CpuTimer0Regs.TCR.bit.TIF == 1)
    {
        CpuTimer0Regs.TCR.bit.TIF = 1;  // clear flag

        //-----------------------------------------------------------
        (*A_Task_Ptr)();            // jump to an A Task (A1,A2,A3,...)
        //-----------------------------------------------------------
    }
    Alpha_State_Ptr = &C0;      // Comment out to allow only A tasks
}

void C0(void)
{
    // loop rate synchronizer for C-tasks
    if(CpuTimer2Regs.TCR.bit.TIF == 1)
    {
        CpuTimer2Regs.TCR.bit.TIF = 1;              // clear flag

        //-----------------------------------------------------------
        (*C_Task_Ptr)();        // jump to a C Task (C1,C2,C3,...)
        //-----------------------------------------------------------
    }

    Alpha_State_Ptr = &A0;  // Back to State A0
}

void A1(void)
//--------------------------------------------------------
{
    if(CpuTimer1Regs.TCR.bit.TIF == 1)
    {
        CpuTimer1Regs.TCR.bit.TIF = 1;  // clear flag

       // data_out();
       // SendData();
    }

}

void C1(void)  // soft start thyristors
//------------------------------------------------------
{
    if (VbusTargetSlewed == 0)                  // start
        {
         temp_zero = _IQ24(0);
         CNTL_2P2Z_Ref2 = &temp_zero;            // Slewed Voltage Command
        }

    if ((VrectRMS >= V_RECT_RMS_THRSHLD)&&(VbusavgH_f==0)&&(Fault==0))       // 180 Vnet_rms V_RECT_RMS_THRSHLD
    {
        SoftStartThyristors = 1;

        if (start_flag == 0 && VbusAvg > VBUS_MIN)//Use this to start PFC in stand alone mode.
            //Comment this line and uncomment the line above to start PFC from CCS watch window using start_flag
        {
            if(!fire_angle_min)
            {
                GpioDataRegs.GPBSET.bit.GPIO32 = 1;         //open the upper thyristor
                GpioDataRegs.GPASET.bit.GPIO12 = 1;         //open the lower thyristor
                fire_angle_min = 1;                         //firing angle is minimal - stop increasing the angle
            }

            VbusTargetSlewed = Vbus+init_boost;        // Start slewing the boost command from a value slightly greater than the PFC output voltage
            CNTL_2P2Z_Ref2 = &VbusTargetSlewed;         // Slewed Voltage Command

            start_flag = 1;                             // This flag makes sure above code is executed only once when..
                                                        // the VbusTarget command goes from zero to a value > 150V
            CNTL_2P2Z_CoefStruct2.max  = _IQ24(0.0002);
            CNTL_2P2Z_CoefStruct1.max  = _IQ24(0.001);

        //-----------------
        //the next time CpuTimer2 'counter' reaches Period value go to C2
            C_Task_Ptr = &C2;
#ifdef RECT_debug
            C_Task_Ptr = &C3;
#endif
        //-----------------
        }
        else        //
        {
            if(!fire_angle_min)
            {
                fire_angle_count = (fire_angle_count + 1) % 10;
                if(fire_angle_count ==0 )     //урежение на порядок до 200 мс
                {
                    if(fire_angle > 110)
                        fire_angle--;          //постепенно уменьшаем угол раз в 100 мс, пока не дошли до амплитудной точки
                }
            }
            C_Task_Ptr = &C3;

        }
    }//(VrectRMS >=_IQ24(0.2))
    else
    {
        SoftStartThyristors = 0;
        GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;         //close the upper thyristor
        GpioDataRegs.GPACLEAR.bit.GPIO12 = 1;         //close the lower thyristor

        fire_angle = 270;                            // start level
        fire_angle_min = 0;
        start_flag = 0;

        VbusTargetSlewed=0;                     // раскоментить если необходимо выключать PFC вместе с RECT при V_INrms<180
        CNTL_2P2Z_CoefStruct2.max  = _IQ24(0.0002);
        CNTL_2P2Z_CoefStruct1.max  = _IQ24(0.001);

        run_flag = 0;
        KoutTargetSlewed = _IQ24(0.0);
        Coef2P2Z_V.max = _IQ24(0.0);
        Coef2P2Z_V.min = _IQ24(0.0);

        VTimer1++;                                // обнуляем таймер отсчета угла вкл. тиристора по "+" полуволне
        VTimer2++;                                // обнуляем таймер отсчета угла вкл. тиристора по "-" полуволне

        VTimer0[0]++;                             // обнуляем счетчик импульса вкл. тиристора "+" волны
        VTimer0[1]++;                             // обнуляем счетчик импульса вкл. тиристора "-" волны
        VS_H_f = 0;
        VS_L_f = 0;

        C_Task_Ptr = &C3;
    }
}
//----------------------------------------
void C2(void) //Slew Rate ("Soft Start")
//----------------------------------------
{

//pfcSlewRate has to be a positive value
//pfc_slew_temp = VbusTarget - VbusTargetSlewed;
pfc_slew_temp = VBUS_TARGET - VbusTargetSlewed;     //0.897(IQ24)- 0.682(IQ24)= + error

if (pfc_slew_temp >= VbusSlewRate) // Positive Command. Slewed Vbus is less than VBUS_TARGET, so increase it. This is
                                    //implement soft-start for Vbus. VbusSlewRate is initialized at the begining of this file.
{
    VbusTargetSlewed = VbusTargetSlewed + VbusSlewRate;
    C_Task_Ptr = &C2;
}
else
{
//Soft-start is complete. So set the flag for RUN mode and go to Task C3 for RUN time adjustment of Vbus
//  if ((-1)*(pfc_slew_temp) >= VbusSlewRate) // Negative Command
//  {
//      VbusTargetSlewed = VbusTargetSlewed - VbusSlewRate;
        VbusTargetSlewed = VBUS_TARGET;
        //Gui_Vbus_set = VBUS_RATED_VOLTS*64;//Q15, Set Gui Vbus set point to initial value VBUS_RATED_VOLTS
        //start_flag = 0;

//  }
}


if ((CNTL_2P2Z_CoefStruct2.max  < _IQ24(0.95))||(CNTL_2P2Z_CoefStruct1.max <  _IQ24(0.95)))
{
    if(CNTL_2P2Z_CoefStruct2.max < _IQ24(0.93))
   {
        CNTL_2P2Z_CoefStruct2.max += _IQ24(0.001);
   } else
   {
        CNTL_2P2Z_CoefStruct2.max = _IQ24(0.95);
                  //PFC_status.bit.Mode=3;
   }

   if(CNTL_2P2Z_CoefStruct1.max < _IQ24(0.93))
   {
        CNTL_2P2Z_CoefStruct1.max += _IQ24(0.001);
   }
   else
   {
        CNTL_2P2Z_CoefStruct1.max = _IQ24(0.95);
                  //PFC_status.bit.Mode=3;
   }
   C_Task_Ptr = &C2;

}

if ((CNTL_2P2Z_CoefStruct1.max  == _IQ24(0.95))&&(CNTL_2P2Z_CoefStruct2.max ==  _IQ24(0.95))&&(( VbusTargetSlewed == VBUS_TARGET)))
{
#ifndef PFC_debug
         run_flag = 1;
#endif
        C_Task_Ptr = &C3;

    //-----------------
    //the next time CpuTimer2 'counter' reaches Period value go to C3
    //C_Task_Ptr = &C1;
    //-----------------
}
}


//-----------------------------------------
void C3(void) //
//-----------------------------------------
{

#ifndef INV

    if (run_flag == 1) //If soft-start is over and PFC running normally
    {

        // pfcSlewRate has to be a positive value
        pfc_slew_temp = VBUS_TARGET - VbusTargetSlewed;

        if (pfc_slew_temp >= VbusSlewRate) // Positive Command. Increase Vbus
        {
            VbusTargetSlewed = VbusTargetSlewed + VbusSlewRate;
        }
        else
        {
            if ((-1)*(pfc_slew_temp) >= VbusSlewRate) // Negative Command. Reduce Vbus
            {
                VbusTargetSlewed = VbusTargetSlewed - VbusSlewRate;
                //      VbusTargetSlewed = VBUS_TARGET;
                //      start_flag = 0;
            }
        }

#endif
//        if(KoutTargetSlewed <_IQ24(1.0))
//            {KoutTargetSlewed += _IQ24(0.001);}
//        else{
            KoutTargetSlewed = _IQ24(1.0);
    //         }
//        Coef2P2Z_V.max = _IQ24(C2P2ZCoeff_V_MAX);
//        Coef2P2Z_V.min = _IQ24(C2P2ZCoeff_V_MIN);

        // если PFC вышел на режим и VBUS достигло 400 В, плавно поднимаем насыщение  V регулятора инвертора

        if(Coef2P2Z_V.max < _IQ24(C2P2ZCoeff_V_MAX))
            Coef2P2Z_V.max += _IQ24(0.0005);
        else
            Coef2P2Z_V.max = _IQ24(C2P2ZCoeff_V_MAX);

        if(Coef2P2Z_V.min > _IQ24(C2P2ZCoeff_V_MIN))
             Coef2P2Z_V.min -= _IQ24(0.0005);
        else
             Coef2P2Z_V.min = _IQ24(C2P2ZCoeff_V_MIN);


#ifndef INV

        if ((VbusAvg < VBUS_UNDERVP_THRSHLD)||(VbusAvg_H < VBUS_UNDERVP_THRSHLD)||(VbusAvg_H < VBUS_UNDERVP_THRSHLD))  //Check for Vbus UV Condition
        {
            EALLOW;
            EPwm3Regs.TZFRC.bit.OST = 1;//Turn off PWM for UV condition
            EPwm6Regs.TZFRC.bit.OST = 1;//Turn off PWM for UV condition
            GpioDataRegs.GPASET.bit.GPIO6 = 1;         //LED1
            EDIS;
            VbusavgL_f=1;
        }

    }
#endif
    //-----------------
    //the next time CpuTimer2 'counter' reaches Period value go to C1
    C_Task_Ptr = &C1;
    //-----------------

}

interrupt void int_EPWM6(void)  //SOC0_SOC1 EPWM3SOCB trigger pulse окончание измерения Vout и zero_level, запуск за несколько тактов до момента окончания импульса
{
    EALLOW;
    EPwm6Regs.ETCLR.bit.INT = 1;                        // clear interrupt flag of PWMINT6
    PieCtrlRegs.PIEACK.bit.ACK3 = 1;                    // clear the bit and enables the PIE block interrupts
    EDIS;

    //V_Ref=_IQ24mpy(SinTableSlewed,I_Out);
#ifndef FLASH
    *(V_OUT_INT_array+x_i) =  ADCout_V;
//    *(V_IN_array+x_i) =  Duty;
//    *(V_Ref_array+x_i) = VbusAvg;
//    *(V_Ref_array+x_i) = Vrect;
//    *(V_Fdb_array+x_i) = Vbus;
    x_i++;
    x_i&=0x1FF;
#endif

    if(!fire_angle_min && SoftStartThyristors)                           // режем угол, пока не достигнем мин. значения
    {

        VTimer1++;                                // инкрементируем таймер отсчета угла вкл. тиристора по "+" полуволне
        VTimer2++;                                // инкрементируем таймер отсчета угла вкл. тиристора по "-" полуволне

        VTimer0[0]++;                             // инкрементируем счетчик импульса вкл. тиристора "+" волны
        VTimer0[1]++;                             // инкрементируем счетчик импульса вкл. тиристора "-" волны

        if((PosAngle)&&(VTimer1 > fire_angle))
        {
            PosAngle = 0;                         // сбрасываем признак отсчета угла
            //GpioDataRegs.GPBSET.bit.GPIO33 = 1;   // включаем верхний тиристор
            GpioDataRegs.GPBSET.bit.GPIO32 = 1;   // включаем верхний тиристор
            VS_H_f  = 1;                          // устанавливаем признак импульса управления верхним тиристором
            VTimer0[0] = 0;                       // обнуляем софт-таймер

        }

        if((NegAngle)&&(VTimer2 > fire_angle))
        {
            NegAngle = 0;                         // сбрасываем признак отсчета угла
            //GpioDataRegs.GPBSET.bit.GPIO32 = 1;   // включаем нижний тиристор
            GpioDataRegs.GPASET.bit.GPIO12 = 1;   // включаем нижний тиристор
            VS_L_f  = 1;                          // устанавливаем признак импульса управления нижним тиристором
            VTimer0[1] = 0;                       // обнуляем софт-таймер

        }
        //}

        if(VS_H_f && VTimer0[0] > 25)
        {
            VS_H_f = 0;
            //GpioDataRegs.GPBCLEAR.bit.GPIO33 = 1;      // снимаем импульс управления, так как тиристор уже включен
            GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;      // снимаем импульс управления, так как тиристор уже включен
        }

        if(VS_L_f && VTimer0[1] > 25)
        {
            VS_L_f = 0;
            //GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;      // снимаем импульс управления, так как тиристор уже включен
            GpioDataRegs.GPACLEAR.bit.GPIO12 = 1;      // снимаем импульс управления, так как тиристор уже включен
        }
    }
}

interrupt void SECONDARY_ISR(void)
{
    EALLOW;
    EPwm5Regs.ETCLR.bit.INT = 1;                        // clear interrupt flag of PWMINT6
    PieCtrlRegs.PIEACK.bit.ACK3 = 1;                    // clear the bit and enables the PIE block interrupts
    EDIS;
     static long a= I_IN_THRSHLD_35;
    if ((VbusAvg > VBUS_OVP_THRSHLD)||(VbusAvg_H > VBUS_OVP_THRSHLD)||(VbusAvg_H > VBUS_OVP_THRSHLD)||(IoutAvg>=I_OUT_THRSHLD_30)||(IinAvg>=a))//Check for Vbus OV Condition
     {
         OV_flag = 1;
         EALLOW;
         GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;         //close the upper thyristor
         GpioDataRegs.GPACLEAR.bit.GPIO12 = 1;         //close the lower thyristor
         EPwm1Regs.TZFRC.bit.OST = 1;//Turn off PWM for OV condition
         EPwm2Regs.TZFRC.bit.OST = 1;//Turn off PWM for OV condition
         EPwm3Regs.TZFRC.bit.OST = 1;//Turn off PWM for UV condition
         EPwm6Regs.TZFRC.bit.OST = 1;//Turn off PWM for UV condition
         GpioDataRegs.GPASET.bit.GPIO8 = 1;         //LED2
         GpioDataRegs.GPASET.bit.GPIO6 = 1;         //LED1
         EDIS;

         VbusTargetSlewed = 0;

         VbusavgH_f=1;
         //Gui_Vbus_set = 0;
     }



     //Calculate RMS input voltage and input frequency

     sine_mainsV.Vin = Vrect >> 9;          // input in IQ15 format
     SineAnalyzer_MACRO (sine_mainsV);
     VrectRMS = (sine_mainsV.Vrms)<< 9;     // Convert sine_mainsV.Vrms from Q15 to Q24 and save as VrectRMS
     Freq_Vin = sine_mainsV.SigFreq;        // Q15
     if(sine_mainsV.antifreezecounter != 0)
     {
         sine_mainsV.antifreezecounter--;
     }
     else
     {
         VrectRMS = 0;
         Freq_Vin = 0;
     }


}

void Net_connect()
{
///////////////////////////////////////DC_AC///////////////////////////////////////////////
    CNTL_2P2Z_RefV =   &V_Ref;
    CNTL_2P2Z_OutV =   &V_Out;
    CNTL_2P2Z_FdbkV =  &V_Fdb;
    CNTL_2P2Z_CoefV =  &Coef2P2Z_V.b2;

    //CNTL_2P2Z_RefI = &I_Ref;
    CNTL_2P2Z_RefI = &Vref_Iout;
    CNTL_2P2Z_OutI = &I_Out;
    CNTL_2P2Z_FdbkI = &IoutAvg;
    CNTL_2P2Z_CoefI = &Coef2P2Z_I.b2;

    ADCDRV_1ch_Rlt0  =&ADCout_V;
    ADCout_V = _IQ24(0.0);
    ADCDRV_1ch_Rlt1  =&ADCout_HALF;
    ADCout_HALF = _IQ24(0.0);
    ADCDRV_1ch_Rlt2  =&ADCout_I;
    ADCout_I = _IQ24(0.0);

    PWMDRV_1ch_UpDwnCnt_Duty6 = &Duty;
    PWMDRV_1ch_UpDwnCnt_Duty3 = &Duty;
    Duty=_IQ24(0.0);


///////////////////////////////////////////AC_DC////////////////////////////////////////////

    ADCDRV_1ch_Rlt3 = &Ipfc;
    ADCDRV_1ch_Rlt4 = &Vbus_H;
    ADCDRV_1ch_Rlt5 = &VL_fb;
    ADCDRV_1ch_Rlt6 = &Vbus_L;
    ADCDRV_1ch_Rlt7 = &I_SW_O;
    //connect the 2P2Z connections, for the inner Current Loop, Loop1
    CNTL_2P2Z_Ref1 = &PFCIcmd;
    CNTL_2P2Z_Out1 = &DutyA; //Comment to open the curr loop.Then specify open loop duty to boost  bus volt.
    CNTL_2P2Z_Fdbk1= &Ipfc;
    CNTL_2P2Z_Coef1 = &CNTL_2P2Z_CoefStruct1.b2;
    //connect the 2P2Z connections, for the outer Voltage Loop, Loop2
    CNTL_2P2Z_Ref2 = &VbusTargetSlewed;
    CNTL_2P2Z_Out2 = &VbusVcmd;
    CNTL_2P2Z_Fdbk2= &Vbus;
    CNTL_2P2Z_Coef2 = &CNTL_2P2Z_CoefStruct2.b2;
    // Math_avg block connections - Instance 2
    //MATH_EMAVG_In1=&Vbus;
    //MATH_EMAVG_Out1=&VbusAvg;
    //MATH_EMAVG_Multiplier1=_IQ30(0.00390625);   //
    // INV_RMS_SQR block connections
    VrectRMS = (sine_mainsV.Vrms)<< 9;//Q15 --> Q24, (sine_mainsV.Vrms) is in Q15
    PFC_InvRmsSqr_In1=&VrectRMS;
    PFC_InvRmsSqr_Out1=&VinvSqr;
    //PFC_InvRmsSqr_VminOverVmax1=_IQ30(0.696);     // 160V/230V
    PFC_InvRmsSqr_VminOverVmax1=_IQ30(0.9999);      // 230V/230V
    //PFC_InvRmsSqr_Vmin1=_IQ24(0.41);              // 160V/390V
    PFC_InvRmsSqr_Vmin1=_IQ24(0.589);               // 230V/390V
    // PFC_BL_ICMD block connections
    PFC_ICMD_Vcmd1 = &VbusVcmd;
    PFC_ICMD_VinvSqr1=&VinvSqr;
    PFC_ICMD_VacRect1=&Vrect;
    PFC_ICMD_Out1=&PFCIcmd;
    //PFC_ICMD_Vpfc1 = &Vbus;
    //PFC_ICMD_Duty1 = &DutyA;
    PFC_ICMD_VmaxOverVmin1 = _IQ24(1.71875);    //_IQ24(275/160)=;//VoutMax=521.4V, VinMax=409.8V(peak)= 289.8Vrms

    PWMDRV_1ch_UpDwnCnt_Duty1 = &DutyA;
    PWMDRV_1ch_UpDwnCnt_Duty2 = &DutyA;
    //PWMDRV_1ch_UpDwnCnt_Duty4 = &PFCIcmd;//Use DPWM4A and ext RC filter to verify PFC reference current

    // Math_avg block connections - Instance 1
    MATH_EMAVG_In1=&Vbus;
    MATH_EMAVG_Out1=&VbusAvg;
    MATH_EMAVG_Multiplier1=_IQ30(0.00002454);

    MATH_EMAVG_In2=&Vbus_H;
    MATH_EMAVG_Out2=&VbusAvg_H;
    MATH_EMAVG_Multiplier2=_IQ30(0.00002454);

    MATH_EMAVG_In3=&Vbus_L;
    MATH_EMAVG_Out3=&VbusAvg_L;
    MATH_EMAVG_Multiplier3=_IQ30(0.00002454);        //1% pulse

    MATH_EMAVG_In4=&I_Fdb_rect;
    MATH_EMAVG_Out4=&IoutAvg;
    MATH_EMAVG_Multiplier4=_IQ30(0.002454);         //10% pulse

    MATH_EMAVG_In5=&Ipfc;
    MATH_EMAVG_Out5=&IinAvg;
    MATH_EMAVG_Multiplier5=_IQ30(0.00002454);


    // Initialize the net variables
    DutyA =_IQ24(0.0);
    //Duty4A =_IQ24(0.0);
    VrectAvg = _IQ24(0.0);
    VrectRMS = _IQ24(0.0);
    VbusAvg = _IQ24(0.0);
    VbusAvg_H = _IQ24(0.0);
    VbusAvg_L = _IQ24(0.0);
    VinvSqr = _IQ24(0.0);
    Vrect = _IQ24(0.0);
    PFCIcmd = _IQ24(0.0);

    VbusVcmd = _IQ24(0.5);          //Variable initialized for open Volt loop $ closed current loop test with light load

    KoutTargetSlewed = _IQ24(0.001);
    VbusTargetSlewed = _IQ24(0.0);
    pfc_slew_temp = 0;
    start_flag = 0;
    run_flag = 0;
    temp_zero = 0;
    Iout_fault= I_OUT_THRSHLD_54;
    Isw_fault= I_SW_O_THRSHLD_54;
    Iin_fault= I_IN_THRSHLD_65;
    Vref_Iout=VREF_Iout;
}

void Coef_fill()
{

/////////////////////////////////////////////////DC_AC//////////////////////////////////////////
    Coef2P2Z_V.b2 = _IQ26(C2P2ZCoeff_V_B2);
    Coef2P2Z_V.b1 = _IQ26(C2P2ZCoeff_V_B1);
    Coef2P2Z_V.b0 = _IQ26(C2P2ZCoeff_V_B0);
    Coef2P2Z_V.a2 = _IQ26(C2P2ZCoeff_V_A2);
    Coef2P2Z_V.a1 = _IQ26(C2P2ZCoeff_V_A1);
    //Coef2P2Z_V.max = _IQ24(C2P2ZCoeff_V_MAX);
    Coef2P2Z_V.max = _IQ24(0.0);
    Coef2P2Z_V.min = _IQ24(0.0);
    Coef2P2Z_V.i_min = _IQ24(C2P2ZCoeff_V_MIN);

    Coef2P2Z_I.b2 = _IQ26(C2P2ZCoeff_I_B2);
    Coef2P2Z_I.b1 = _IQ26(C2P2ZCoeff_I_B1);
    Coef2P2Z_I.b0 = _IQ26(C2P2ZCoeff_I_B0);
    Coef2P2Z_I.a2 = _IQ26(C2P2ZCoeff_I_A2);
    Coef2P2Z_I.a1 = _IQ26(C2P2ZCoeff_I_A1);
    Coef2P2Z_I.max = _IQ24(C2P2ZCoeff_I_MAX);
    Coef2P2Z_I.min = _IQ24(C2P2ZCoeff_I_MIN);
    Coef2P2Z_I.i_min = _IQ24(C2P2ZCoeff_I_MIN);
/////////////////////////////////////////////////////AC_DC/////////////////////////////////////

    // Coefficients for Inner Current Loop
    // PID coefficients & Clamp values - Current loop (Q26), 100kHz Cloop sampling

    //Dmax_I  = _IQ24(0.984375);
   // Pgain_I = _IQ26(0.04);
    //Pgain_I = _IQ26(0.0);
    //Igain_I = _IQ26(0.006875);
 //   Igain_I = _IQ26(0.07);
  //  Dgain_I = _IQ26(0.0);

    //Dmax_V  = _IQ24(0.984375);
 //   Pgain_V = _IQ26(0.4);
    //Pgain_I = _IQ26(0.0);
    //Igain_I = _IQ26(0.006875);
 //   Igain_V = _IQ26(0.08);
 //   Dgain_V = _IQ26(0.0);

    // Coefficient init --- Coeeficient values in Q26
    // Use IQ Maths to generate floating point values for the CLA
    CNTL_2P2Z_CoefStruct1.b2   = _IQ26(C2P2ZCoeff_1_B2);            // B2
    CNTL_2P2Z_CoefStruct1.b1   = _IQ26(C2P2ZCoeff_1_B1);            // B1
    CNTL_2P2Z_CoefStruct1.b0   = _IQ26(C2P2ZCoeff_1_B0);            // B0
    CNTL_2P2Z_CoefStruct1.a2   = _IQ26(C2P2ZCoeff_1_A2);            // A2 = 0
    CNTL_2P2Z_CoefStruct1.a1   = _IQ26(C2P2ZCoeff_1_A1);            // A1 = 1
    CNTL_2P2Z_CoefStruct1.max  = _IQ24(0);                          //Clamp Hi
    CNTL_2P2Z_CoefStruct1.min  = _IQ24(0.0);                        //Clamp Min
    CNTL_2P2Z_CoefStruct1.i_min  = _IQ24(0.0);                      //Clamp Min

    //    CNTL_2P2Z_CoefStruct2.b2   =  Dgain_V;                          // B2
    //    CNTL_2P2Z_CoefStruct2.b1   = (Igain_V-Pgain_V-Dgain_V-Dgain_V); // B1
    //    CNTL_2P2Z_CoefStruct2.b0   = (Pgain_V + Igain_V + Dgain_V);     // B0
    //    CNTL_2P2Z_CoefStruct2.a2   =  0.0;                              // A2 = 0
    //    CNTL_2P2Z_CoefStruct2.a1   = _IQ26(1.0);                        // A1 = 1
    //    CNTL_2P2Z_CoefStruct2.max  = _IQ24(0.9);                        // Clamp Hi
    //    CNTL_2P2Z_CoefStruct2.min  = _IQ24(0.0);                        // Clamp Min
    //    CNTL_2P2Z_CoefStruct2.i_min  = _IQ24(0.0);                      // Clamp Min

    CNTL_2P2Z_CoefStruct2.b2   = _IQ26(C2P2ZCoeff_2_B2);            // B2
    CNTL_2P2Z_CoefStruct2.b1   = _IQ26(C2P2ZCoeff_2_B1);            // B1
    CNTL_2P2Z_CoefStruct2.b0   = _IQ26(C2P2ZCoeff_2_B0);            // B0
    CNTL_2P2Z_CoefStruct2.a2   = _IQ26(C2P2ZCoeff_2_A2);            // A2 = 0
    CNTL_2P2Z_CoefStruct2.a1   = _IQ26(C2P2ZCoeff_2_A1);            // A1 = 1
    CNTL_2P2Z_CoefStruct2.max  = _IQ24(0);                          // Clamp Hi
    CNTL_2P2Z_CoefStruct2.min  = _IQ24(C2P2ZCoeff_2_MIN);           // Clamp Min
    CNTL_2P2Z_CoefStruct2.i_min  = _IQ24(C2P2ZCoeff_2_MIN);         // Clamp Min

}
