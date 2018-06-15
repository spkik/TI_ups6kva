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

#define EPWMn_DPL_ISR	1	// for EPWM triggered ISR set as 1
#define ADC_DPL_ISR	    0	// for ADC INT 1 triggered ISR set as 1
#define CLAn_DPL_ISR	0	// for CLA Task n Triggered ISR set as 1

//----------------------------------------------------------------------------------
// If EPWMn_DPL_ISR = 1, then choose which module
//----------------------------------------------------------------------------------
#define EPWM1			1	// EPWM1 provides ISR trigger
#define EPWM2			0 	// EPWM2 provides ISR trigger
#define EPWM3			0	// EPWM3 provides ISR trigger
#define EPWM4			0	// EPWM4 provides ISR trigger
#define EPWM5			0	// EPWM5 provides ISR trigger
#define EPWM6			0	// EPWM6 provides ISR trigger

#define VIN_FULL_RANGE (400.0) //full range of ADC for Vin (each of line and neutral sense)
#define VAB_NOM_THRSHLD ((int32)((180.0/VIN_FULL_RANGE)*4096*4096)) //180V

//#define VBUS_FULL_RANGE 	(410.0)//(519.0)(522.5)//(503.0) //full range of ADC for VBUS
#define VBUS_FULL_RANGE     (440.0)//(519.0)(522.5)//(503.0) //full range of ADC for VBUS
//#define VBUS_RATED_VOLTS	(350.0)//(395.0) //395.0V
#define VBUS_RATED_VOLTS    (395.0)//395.0V
//#define VBUS_RATED_VOLTS    (390.0)//(395.0) //395.0V
//#define VBUS_MIN ((int32)((315.0/VBUS_FULL_RANGE)*4096*4096)) //Min bus volt with AC in and PFC off

//#define  PFC_debug   // установка настроек для проверки ключей PFC при выключенных тиристорах
#ifdef PFC_debug
    #define VBUS_MIN ((int32)((0.0/VBUS_FULL_RANGE)*4096*4096)) //Min bus volt with AC in and PFC off
#else
    #define VBUS_MIN ((int32)((300.0/VBUS_FULL_RANGE)*4096*4096)) //Min bus volt with AC in and PFC off
#endif

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
int Vflag;
//===============================================================
// System Settings
//---------------------------------------------------------------


//#define  INV   // установка настроек для запуска инвертора без PFC

#endif //_PROJSETTINGS_H
//===========================================================================
// End of file.
//===========================================================================
