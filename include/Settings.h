// TI File $Revision: /main/4 $
// Checkin $Date: August 2, 2006   16:58:50 $
//###########################################################################
//
// FILE:   ProjectSettings.h
//
// TITLE:  DSP280x Project Settings.
//
// DECRIPTION: 	This file contains the definitions for this project, and is 
//				linked to both X-Main.c and X-ISR.asm (where X is the project
//				name).  When editing this file please "Rebuild All".

//###########################################################################
// $TI Release: Project Settings
// $Release Date: February 29, 2008 $
//###########################################################################
#ifndef _PROJSETTINGS_H
#define _PROJSETTINGS_H

//***************************************************************
//  NOTE: WHEN CHANGING THIS FILE PLEASE REBUILD ALL
//***************************************************************

//===============================================================
// Incremental Build options for System check-out
//---------------------------------------------------------------
#define INCR_BUILD 3		//1 -
							//2 - 
							//3 - closed voltage and current loop
							//4 - 
							//5 - 
#define VoltCurrLoopExecRatio   1
//==================================================================================
// System Settings
//----------------------------------------------------------------------------------
//==================================================================================
// Interrupt Framework options
//==================================================================================

//#define EPWMn_DPL_ISR	1	// for EPWM triggered ISR set as 1
//#define ADC_DPL_ISR	    0	// for ADC INT 1 triggered ISR set as 1
#define EPWMn_DPL_ISR   0   // for EPWM triggered ISR set as 1                  Отладка АЦП
#define ADC_DPL_ISR     1   // for ADC INT 1 triggered ISR set as 1             Отладка АЦП
#define CLAn_DPL_ISR	0	// for CLA Task n Triggered ISR set as 1

//----------------------------------------------------------------------------------
// If EPWMn_DPL_ISR = 1, then choose which module
//----------------------------------------------------------------------------------
//#define EPWM1			1	// EPWM1 provides ISR trigger                       Отладка АЦП
#define EPWM1           0   // EPWM1 provides ISR trigger
#define EPWM2			0 	// EPWM2 provides ISR trigger
#define EPWM3			0	// EPWM3 provides ISR trigger
#define EPWM4			0	// EPWM4 provides ISR trigger
#define EPWM5			0	// EPWM5 provides ISR trigger
#define EPWM6			0	// EPWM6 provides ISR trigger

#define ADC_REF             (float32)(2.5)
#define DAC_REF             (float32)(3.3)

//VIN
#define VIN_FULL_RANGE      (float32)(418.6) //full range of ADC for Vin (each of line and neutral sense)
#define VIN_THRSHLD         (float32)(254.55) // 180 rms
#define Kdiv_VIN            (float32)(1.0L/61.0L)        //0.0163934426229              // коэффициент делителя на входе ОУ
#define Ku_VIN              (float32)(3.65L/10.0L)       //0.365                        // коэффициент усиления ОУ
#define VIN_GAIN            (float32)(Kdiv_VIN*Ku_VIN)

#define V_IN_OpAmpOut       (float32)((VIN_THRSHLD*VIN_GAIN+ADC_REF)/2.0)     //input ADC
#define V_IN_ADC            (float32)(V_IN_OpAmpOut/ADC_REF)                //format ADC
#define V_RECT_AMP          (float32)(V_IN_ADC-0.5)                         // Vrect_max
#define ISQRT2              (float32)(0.7071067811865475)
#define V_RECT_RMS_THRSHLD  _IQ24(V_RECT_AMP*ISQRT2)                  // VrectRMS
//
//I_OUT
#define IOUT_FULL_RANGE     (float32)(100.0)    //full range of ADC for I_OUT (amplitude)
#define IOUT_MAX_30         (float32)(42.426)   //30*sqrt(2)
#define Kdt                 (float32)(1.0/2500.0)*62.0   //=0.0248 Turns ratio of current transformer*Ri(Ohm)
#define IOUT_OpAmpOut       (float32)((IOUT_MAX_30*Kdt+ADC_REF)/2.0)   //=1.776 input ADC
#define IOUT_ADC            (float32)(IOUT_OpAmpOut/ADC_REF)             //=0.71  format ADC
#define IoutRect_max        (float32)(IOUT_ADC-0.5)                      //=0.21  IoutRect_max
#define I_OUT_THRSHLD_30    _IQ24(IoutRect_max*2.0/3.14159)              //=0.13369 IoutRectAvg=IoutRect_max*2/pi

#define IOUT_MAX_54         (float32)(76.368)   //54*sqrt(2)
#define IOUT_OpAmpOut_54    (float32)((IOUT_MAX_54*Kdt+ADC_REF)/2.0)   //=2.1969 input ADC
#define IOUT_ADC_54         (float32)(IOUT_OpAmpOut_54/ADC_REF)        //=0.8787  format ADC
#define IoutRect_54         (float32)(IOUT_ADC_54-0.5)                    //=0.3787  IoutRect_max
#define I_OUT_THRSHLD_54    _IQ24(IoutRect_54)                         //

//#define VBUS_FULL_RANGE 	(410.0)//(519.0)(522.5)//(503.0) //full range of ADC for VBUS
#define VBUS_FULL_RANGE     (440.0)//(519.0)(522.5)//(503.0) //full range of ADC for VBUS
//#define VBUS_RATED_VOLTS	(350.0)//(395.0) //395.0V
#define VBUS_RATED_VOLTS    (395.0)//395.0V
//#define VBUS_RATED_VOLTS    (390.0)//(395.0) //395.0V
//#define VBUS_MIN ((int32)((315.0/VBUS_FULL_RANGE)*4096*4096)) //Min bus volt with AC in and PFC off



//#define VBUS_OVP_THRSHLD ((int32)((405.0/VBUS_FULL_RANGE)*4096*4096)) //435V
#define VBUS_OVP_THRSHLD ((int32)((425.0/VBUS_FULL_RANGE)*4096*4096)) //425V

#define VBUS_AB_START  ((int32)((360.0/VBUS_FULL_RANGE)*4096*4096)) //напряжение на шине ниже которого переключаемся на работу от АБ

//#define VBUS_SWITCH_THRSHLD ((int32)((375.0/VBUS_FULL_RANGE)*4096*4096)) //напряжение на шине ниже которого переключаемся на работу от АБ
#define VBUS_SWITCH_THRSHLD ((int32)((360.0/VBUS_FULL_RANGE)*4096*4096)) //напряжение на шине ниже которого переключаемся на работу от АБ
#define VBUS_UNDERVP_THRSHLD ((int32)((340.0/VBUS_FULL_RANGE)*4096*4096)) //435V

#define VBUS_DPWM_OFF_LEVEL ((int32)((425.0/VBUS_FULL_RANGE)*4096*4096)) //425V
#define VBUS_DPWM_ON_LEVEL  ((int32)((395.0/VBUS_FULL_RANGE)*4096*4096)) //395V
#define VBUS_TARGET			((int32)((VBUS_RATED_VOLTS/VBUS_FULL_RANGE)*4096*4096)) //395V
#define VBUS_ERROR_NL_CNTRL_THRSHLD ((int32)((10.0/VBUS_FULL_RANGE)*4096*4096)) //Vbus error threshold to activate NL Vloop control

#define VBUS_FULL_RANGE_IQ20 ( (int32) 440*4096*256) //предел измерения напряжения на шине,
                                                                                    //выраженный в формате IQ23

//#define CURR_GAIN       (float64)(0.01L*8.0L*12.1L/10.05L)
#define CURR_GAIN       (float64)((1.0L/2500L)*(59.0L/2.0L)*(25.5L/10.0L)*0.5L)
#define COMP1_POS_V     (float64)(ADC_REF/2.0L + CURR_LIM_1*CURR_GAIN)
//#define COMP3_NEG_V     (float64)(ADC_REF/2.0L - CURR_LIM_1*CURR_GAIN)
#define COMP3_NEG_V     (float64)(ADC_REF/2.0L + CURR_LIM_1*CURR_GAIN)

#define COMP1_REF_TRG   (Uint16)(COMP1_POS_V/DAC_REF*1023.0L)
#define COMP3_REF_TRG   (Uint16)(COMP3_NEG_V/DAC_REF*1023.0L)
// Pulse-by-pulse Current Limit
#define CURR_LIM    60.0L
#define DELT_LIM    6.0L
//#define CURR_LIM_0  (float64)(CURR_LIM - DELT_LIM)
#define CURR_LIM_1  (float64)(CURR_LIM + DELT_LIM)



//===============================================================
// System Settings
//---------------------------------------------------------------
//#define RECT_debug    // установка настроек для проверки плавного запуска тиристоров без PFC

//#define  PFC_debug    // установка настроек для проверки ключей PFC
#ifdef PFC_debug
    #define VBUS_MIN ((int32)((300.0/VBUS_FULL_RANGE)*4096*4096)) //Min bus volt with AC in and PFC off
#else
    #define VBUS_MIN ((int32)((300.0/VBUS_FULL_RANGE)*4096*4096)) //Min bus volt with AC in and PFC off
#endif
//#define  INV   // установка настроек для запуска инвертора без PFC

#endif //_PROJSETTINGS_H
//===========================================================================
// End of file.
//===========================================================================
