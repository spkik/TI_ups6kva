#include "Settings.h"
#include "DSP2803x_EPwm_defines.h"
#include "DSP2803x_Device.h"
#include "DPlib.h"

// Functions that will be run from RAM need to be assigned to
// a different section.  This section will then be mapped to a load and
// run address using the linker cmd file.
#pragma CODE_SECTION(InitFlash, "ramfuncs");
#define Device_cal (void   (*)(void))0x3D7C80

//Slope of temperature sensor (deg. C / ADC code, fixed pt Q15 format)
#define getTempSlope() (*(int (*)(void))0x3D7E80)()
//ADC code corresponding to temperature sensor output at 0-degreesC
#define getTempOffset() (*(int (*)(void))0x3D7E83)()





void DeviceInit(void);
void PieCntlInit(void);
void PieVectTableInit(void);
void WDogDisable(void);
void PLLset(Uint16);
void ISR_ILLEGAL(void);

void ePWM_prefare(void);
void init_ADC(void);
void init_T0(void);
void init_T1(void);
void init_T2(void);

interrupt void int_EPWM6(void);
interrupt void SECONDARY_ISR(void);
//interrupt void my_ADC_ISR();

//--------------------------------------------------------------------
//  Configure Device for target Application Here
//--------------------------------------------------------------------
void DeviceInit(void)
{
	WDogDisable(); 	// Disable the watchdog initially.
	DINT;			// Global Disable all Interrupts.
	IER = 0x0000;	// Disable CPU interrupts.
	IFR = 0x0000;	// Clear all CPU interrupt flags.


    // The Device_cal function, which copies the ADC & oscillator calibration values
    // from TI reserved OTP into the appropriate trim registers, occurs automatically
    // in the Boot ROM. If the boot ROM code is bypassed during the debug process, the
    // following function MUST be called for the ADC and oscillators to function according
    // to specification.
	EALLOW;
	SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1; // Enable ADC peripheral clock.
	(*Device_cal)();					  // Auto-calibrate from TI OTP.
	SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 0; // Return ADC clock to original state.
	EDIS;


    // Switch to Internal Oscillator 1 and turn off all other clock
    // sources to minimize power consumption.
	EALLOW;


    SysCtrlRegs.CLKCTL.bit.XTALOSCOFF = 0;     // Turn on XTALOSC
    SysCtrlRegs.CLKCTL.bit.XCLKINOFF = 1;      // Turn off XCLKIN
    SysCtrlRegs.CLKCTL.bit.OSCCLKSRC2SEL = 0;  // Switch to external clock
    SysCtrlRegs.CLKCTL.bit.OSCCLKSRCSEL = 1;   // Switch from INTOSC1 to INTOSC2/ext clk
    SysCtrlRegs.CLKCTL.bit.WDCLKSRCSEL = 1;    // Switch Watchdog Clk Src to external clock
    SysCtrlRegs.CLKCTL.bit.INTOSC2OFF = 1;     // Turn off INTOSC2
    SysCtrlRegs.CLKCTL.bit.INTOSC1OFF = 1;     // Turn off INTOSC1

    EDIS;


// SYSTEM CLOCK speed based on internal oscillator = 10 MHz !!!!! WE HAVE 12 MHz
// 0xC =  60	MHz		(12)
// 0xB =  55	MHz		(11)
// 0xA =  50	MHz		(10)
// 0x9 =  45	MHz		(9)
// 0x8 =  40	MHz		(8)
// 0x7 =  35	MHz		(7)
// 0x6 =  30	MHz		(6)
// 0x5 =  25	MHz		(5)
// 0x4 =  20	MHz		(4)
// 0x3 =  15	MHz		(3)
// 0x2 =  10	MHz		(2)

	PLLset(0xA); // Choose from options above. =60MHz

// Initialise interrupt controller and Vector Table
// to defaults for now. Application ISR mapping done later.
	PieCntlInit();
    PieVectTableInit();

   EALLOW; // Below registers are "protected", allow access.

   // LOW SPEED CLOCKS prescale register settings.
   SysCtrlRegs.LOSPCP.all = 0x0002; // Sysclk / 4 (15 MHz)
   SysCtrlRegs.XCLK.bit.XCLKOUTDIV = 2;
      	
// PERIPHERAL CLOCK ENABLES 
//--------------------------------------------------------------
// If you are not using a peripheral you may want to switch
// the clock off to save power, i.e. set to =0 
// 
// Note: not all peripherals are available on all 280x derivates.
// Refer to the datasheet for your particular device.
//-------------------------------------------------------------- 

   SysCtrlRegs.PCLKCR0.bit.ADCENCLK   = 1; // ADC.
   //------------------------------------------------
   SysCtrlRegs.PCLKCR3.bit.COMP1ENCLK = 1; // COMP1.
   SysCtrlRegs.PCLKCR3.bit.COMP2ENCLK = 1; // COMP2.
   SysCtrlRegs.PCLKCR3.bit.COMP3ENCLK = 1; // COMP3.
   //------------------------------------------------
   SysCtrlRegs.PCLKCR0.bit.I2CAENCLK  = 1; // I2C.
   //------------------------------------------------
   SysCtrlRegs.PCLKCR0.bit.SPIAENCLK  = 1; // SPI-A.
   //------------------------------------------------
   SysCtrlRegs.PCLKCR0.bit.SCIAENCLK  = 1; // SCI-A.
   //------------------------------------------------
   SysCtrlRegs.PCLKCR1.bit.ECAP1ENCLK = 1; // eCAP1.
   //------------------------------------------------
   SysCtrlRegs.PCLKCR1.bit.EPWM1ENCLK = 1; // ePWM1.
   SysCtrlRegs.PCLKCR1.bit.EPWM2ENCLK = 1; // ePWM2.
   SysCtrlRegs.PCLKCR1.bit.EPWM3ENCLK = 1; // ePWM3.
   SysCtrlRegs.PCLKCR1.bit.EPWM4ENCLK = 0; // ePWM4.
   SysCtrlRegs.PCLKCR1.bit.EPWM5ENCLK = 1; // ePWM5.
   SysCtrlRegs.PCLKCR1.bit.EPWM6ENCLK = 1; // ePWM6.

   //------------------------------------------------
   SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC  = 0; // Disable TBCLK.
   //------------------------------------------------
   SysCtrlRegs.PCLKCR0.bit.ECANAENCLK=	0;    // eCAN-A
                  
                                     
//--------------------------------------------------------------------------------------
// GPIO (GENERAL PURPOSE I/O) CONFIG
//--------------------------------------------------------------------------------------
//-----------------------
// QUICK NOTES on USAGE:
//-----------------------
// If GpioCtrlRegs.GP?MUX?bit.GPIO?= 1, 2 or 3 (i.e. Non GPIO func), then leave
//	rest of lines commented
// If GpioCtrlRegs.GP?MUX?bit.GPIO?= 0 (i.e. GPIO func), then:
//	1) uncomment GpioCtrlRegs.GP?DIR.bit.GPIO? = ? and choose pin to be IN or OUT
//	2) If IN, can leave next to lines commented
//	3) If OUT, uncomment line with ..GPACLEAR.. to force pin LOW or
//			   uncomment line with ..GPASET.. to force pin HIGH or
//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------
//  GPIO-00 - PIN FUNCTION = --Spare--
	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 0;		// 0=GPIO,  1=EPWM1A,  2=Resv,  3=Resv
	GpioCtrlRegs.GPADIR.bit.GPIO0 = 1;		// 1=OUTput,  0=INput 
//	GpioDataRegs.GPACLEAR.bit.GPIO0 = 1;	// uncomment if --> Set Low initially
	GpioDataRegs.GPASET.bit.GPIO0 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-01 - PIN FUNCTION = --Spare--
	GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 0;		// 0=GPIO,  1=EPWM1B,  2=EMU0,  3=COMP1OUT
	GpioCtrlRegs.GPADIR.bit.GPIO1 = 1;		// 1=OUTput,  0=INput 
	//GpioIntRegs.GPIOXINT1SEL.all = 1;
//	GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO1 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-02 - PIN FUNCTION = --Spare--
		GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 0;		// 0=GPIO,  1=EPWM2A,  2=Resv,  3=Resv
	GpioCtrlRegs.GPADIR.bit.GPIO2 = 1;		// 1=OUTput,  0=INput 
//	GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;	// uncomment if --> Set Low initially
	GpioDataRegs.GPASET.bit.GPIO2 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-03 - PIN FUNCTION = --Spare--
	GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 0;		// 0=GPIO,  1=EPWM2B,  2=Resv,  3=COMP2OUT
	GpioCtrlRegs.GPADIR.bit.GPIO3 = 1;		// 1=OUTput,  0=INput 
	GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO3 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-04 - PIN FUNCTION = --Spare--
		GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;		// 0=GPIO,  1=EPWM3A, 2=Resv, 	3=Resv
	GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;		// 1=OUTput,  0=INput 
//	GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;	// uncomment if --> Set Low initially
	    GpioDataRegs.GPASET.bit.GPIO4 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-05 - PIN FUNCTION = --Spare--
	GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 0;		// 0=GPIO,  1=EPWM3B,  2=Resv,  3=ECAP1
	GpioCtrlRegs.GPADIR.bit.GPIO5 = 1;		// 1=OUTput,  0=INput 
//	GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO5 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-06 - PIN FUNCTION = --Spare--			FAN_PWM
	GpioCtrlRegs.GPAMUX1.bit.GPIO6 = 0;		// 0=GPIO,  1=EPWM4A,  2=SYNCI,  3=SYNCO
	GpioCtrlRegs.GPADIR.bit.GPIO6 = 1;		// 1=OUTput,  0=INput
	GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO6 = 1;		// IND B SET
//--------------------------------------------------------------------------------------
//  GPIO-07 - PIN FUNCTION = --Spare--
	GpioCtrlRegs.GPAMUX1.bit.GPIO7 = 2;		// 0=GPIO,  1=EPWM4B,  2=SCIRX-A,  3=Resv
	GpioCtrlRegs.GPADIR.bit.GPIO7 = 0;		// 1=OUTput,  0=INput
//	GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO7 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-8 - PIN FUNCTION = --Spare--//сигнал на включение выходного реле, высокий - активный
	GpioCtrlRegs.GPAMUX1.bit.GPIO8 = 0;	    // 0=GPIO,  1=SPISIMO-A,  2=Resv,  3=TZ2
	GpioCtrlRegs.GPADIR.bit.GPIO8 = 1;		// 1=OUTput,  0=INput
	GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO8 = 1;		// IND A SET
//--------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------
//  GPIO-10 - PIN FUNCTION = --Spare--
		GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;	// 0=GPIO,  1=EPWM6A
	GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;		// 1=OUTput,  0=INput
//	GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;	// uncomment if --> Set Low initially
	    GpioDataRegs.GPASET.bit.GPIO10 = 1;		// IND A SET
//--------------------------------------------------------------------------------------
//  GPIO-12 - PIN FUNCTION = --Spare--
	GpioCtrlRegs.GPAMUX1.bit.GPIO12 = 0;	// 0=GPIO,  1=TZ1,  2=SCITX-A,  3=Resv
	GpioCtrlRegs.GPADIR.bit.GPIO12 = 1;		// 1=OUTput,  0=INput
	GpioDataRegs.GPACLEAR.bit.GPIO12 = 1;	// сперва выключаем тиристор (нижний)
//	GpioDataRegs.GPASET.bit.GPIO12 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-13 - GPIO-15 Do Not Exist
//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------
//  GPIO-16 - PIN FUNCTION = --Spare--
	GpioCtrlRegs.GPAMUX2.bit.GPIO16 = 0;	// 0=GPIO,  1=SPISIMO-A,  2=Resv,  3=TZ2
	GpioCtrlRegs.GPADIR.bit.GPIO16 = 1;		// 1=OUTput,  0=INput 
//	GpioDataRegs.GPACLEAR.bit.GPIO16 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO16 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-17 - PIN FUNCTION = --Spare--
	GpioCtrlRegs.GPAMUX2.bit.GPIO17 = 0;	// 0=GPIO,  1=SPISOMI-A,  2=Resv,  3=TZ3
	GpioCtrlRegs.GPADIR.bit.GPIO17 = 1;		// 1=OUTput,  0=INput 
//	GpioDataRegs.GPACLEAR.bit.GPIO17 = 1;	// uncomment if --> Set Low initially
	GpioDataRegs.GPASET.bit.GPIO17 = 1;		// IND C SET
//--------------------------------------------------------------------------------------
//  GPIO-18 - PIN FUNCTION = --Spare--
	GpioCtrlRegs.GPAMUX2.bit.GPIO18 = 0;	// 0=GPIO,  1=SPICLK-A,  2=SCITX-A,  3=XCLKOUT
	GpioCtrlRegs.GPADIR.bit.GPIO18 = 1;		// 1=OUTput,  0=INput 
//	GpioDataRegs.GPACLEAR.bit.GPIO18 = 1;	// uncomment if --> Set Low initially
	GpioDataRegs.GPASET.bit.GPIO18 = 1;		// IND D SET
//--------------------------------------------------------------------------------------
//  GPIO-19 - PIN FUNCTION = --Spare--
	GpioCtrlRegs.GPAMUX2.bit.GPIO19 = 0;	// 0=GPIO,  1=SPISTE-A,  2=SCIRX-A,  3=ECAP1
	GpioCtrlRegs.GPADIR.bit.GPIO19 = 0;		// 1=OUTput,  0=INput 
//	GpioDataRegs.GPACLEAR.bit.GPIO19 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO19 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-20 - PIN FUNCTION = --Spare--
	GpioCtrlRegs.GPAMUX2.bit.GPIO20 = 0;	// 0=GPIO,  1=SPISTE-A,  2=SCIRX-A,  3=ECAP1
	GpioCtrlRegs.GPADIR.bit.GPIO20 = 1;		// 1=OUTput,  0=INput
	GpioDataRegs.GPACLEAR.bit.GPIO20 = 1;	// uncomment if --> Set Low initially
	//GpioDataRegs.GPASET.bit.GPIO20 = 1;		// uncomment if --> Set High initially
	//------------------------------------------------
//  GPIO-21 - PIN FUNCTION = --Spare--
	GpioCtrlRegs.GPAMUX2.bit.GPIO21 = 0;	// 0=GPIO,  1=I2C-SCL,  2=SYNCO,  3=ADCSOCB
	GpioCtrlRegs.GPADIR.bit.GPIO21 = 1;		// 1=OUTput,  0=INput
//	GpioDataRegs.GPACLEAR.bit.GPIO21 = 1;	// uncomment if --> Set Low initially
	GpioDataRegs.GPASET.bit.GPIO21 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-24 - PIN FUNCTION = --Spare--
	GpioCtrlRegs.GPAMUX2.bit.GPIO24	= 0;	// 0=GPIO,  1=COMP2OUT,  2=EMU1,  3=Resv
	GpioCtrlRegs.GPADIR.bit.GPIO24 = 0;		// 1=OUTput,  0=INput
//	GpioDataRegs.GPBCLEAR.bit.GPIO24 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPBSET.bit.GPIO24 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//--------------------------------------------------------------------------------------
//  GPIO-20 - GPIO-27 Do Not Exist
//--------------------------------------------------------------------------------------
//  GPIO-28 - PIN FUNCTION = --Spare--
	GpioCtrlRegs.GPAMUX2.bit.GPIO28 = 1;	// 0=GPIO,  1=SCIRX-A,  2=I2C-SDA,  3=TZ2
//	GpioCtrlRegs.GPADIR.bit.GPIO28 = 1;		// 1=OUTput,  0=INput
//	GpioDataRegs.GPACLEAR.bit.GPIO28 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO28 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-29 - PIN FUNCTION = --Spare--
	GpioCtrlRegs.GPAMUX2.bit.GPIO29 = 1;	// 0=GPIO,  1=SCITXD-A,  2=I2C-SCL,  3=TZ3
//	GpioCtrlRegs.GPADIR.bit.GPIO29 = 1;		// 1=OUTput,  0=INput
//	GpioDataRegs.GPACLEAR.bit.GPIO29 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPASET.bit.GPIO29 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-30 - PIN FUNCTION = --Spare--
	GpioCtrlRegs.GPAMUX2.bit.GPIO30 = 0;	// 0=GPIO,  1=I2C-SDA,  2=SYNCI,  3=ADCSOCA
	GpioCtrlRegs.GPADIR.bit.GPIO30 = 1;		// 1=OUTput,  0=INput 
//	GpioDataRegs.GPBCLEAR.bit.GPIO30 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPBSET.bit.GPIO30 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-31 - PIN FUNCTION = --Spare--
	GpioCtrlRegs.GPAMUX2.bit.GPIO31 = 0;	// 0=GPIO,  1=I2C-SDA,  2=SYNCI,  3=ADCSOCA
	GpioCtrlRegs.GPADIR.bit.GPIO31 = 1;		// 1=OUTput,  0=INput 
//	GpioDataRegs.GPBCLEAR.bit.GPIO31 = 1;	// uncomment if --> Set Low initially
//	GpioDataRegs.GPBSET.bit.GPIO31 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-32 - PIN FUNCTION = --Spare--
	GpioCtrlRegs.GPBMUX1.bit.GPIO32 = 0;	// 0=GPIO,  1=I2C-SDA,  2=SYNCI,  3=ADCSOCA
	GpioCtrlRegs.GPBDIR.bit.GPIO32 = 1;		// 1=OUTput,  0=INput 
	GpioDataRegs.GPBCLEAR.bit.GPIO32 = 1;	// сперва выключаем тиристор (верхний)
//	GpioDataRegs.GPBSET.bit.GPIO32 = 1;		// uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
//  GPIO-32 - PIN FUNCTION = --Spare--
	GpioCtrlRegs.GPBMUX1.bit.GPIO33 = 0;    // 0=GPIO,  1=I2C-SDA,  2=SYNCI,  3=ADCSOCA
	GpioCtrlRegs.GPBDIR.bit.GPIO33 = 1;     // 1=OUTput,  0=INput
    GpioDataRegs.GPBCLEAR.bit.GPIO33 = 1;   // сперва выключаем тиристор (тиристор АБ)
//  GpioDataRegs.GPBSET.bit.GPIO33 = 1;     // uncomment if --> Set High initially
//--------------------------------------------------------------------------------------
	EDIS;	// Disable register access
}

//============================================================================
// NOTE:
// IN MOST APPLICATIONS THE FUNCTIONS AFTER THIS POINT CAN BE LEFT UNCHANGED
// THE USER NEED NOT REALLY UNDERSTAND THE BELOW CODE TO SUCCESSFULLY RUN THIS
// APPLICATION.
//============================================================================

void WDogDisable(void)
{
    EALLOW;
    SysCtrlRegs.WDCR= 0x0068;
    EDIS;
}

// This function initializes the PLLCR register.
// void InitPll(Uint16 val, Uint16 clkindiv)
void PLLset(Uint16 val)
{
   volatile Uint16 iVol;

   // Make sure the PLL is not running in limp mode.
   if (SysCtrlRegs.PLLSTS.bit.MCLKSTS != 0)
   {
	  EALLOW;
      // OSCCLKSRC1 failure detected. PLL running in limp mode.
      // Re-enable missing clock logic.
      SysCtrlRegs.PLLSTS.bit.MCLKCLR = 1;
      EDIS;
      // Replace this line with a call to an appropriate
      // SystemShutdown(); function.
      asm("        ESTOP0"); // Uncomment for debugging purposes.
   }

   // DIVSEL MUST be 0 before PLLCR can be changed from
   // 0x0000. It is set to 0 by an external reset XRSn
   // This puts us in 1/4.
   if (SysCtrlRegs.PLLSTS.bit.DIVSEL != 0)
   {
       EALLOW;
       SysCtrlRegs.PLLSTS.bit.DIVSEL = 0;
       EDIS;
   }

   // Change the PLLCR.
   if (SysCtrlRegs.PLLCR.bit.DIV != val)
   {

      EALLOW;
      // Before setting PLLCR turn off missing clock detect logic.
      SysCtrlRegs.PLLSTS.bit.MCLKOFF = 1;
      SysCtrlRegs.PLLCR.bit.DIV = val;
      EDIS;

      // Optional: Wait for PLL to lock.
      // During this time the CPU will switch to OSCCLK/2 until
      // the PLL is stable.  Once the PLL is stable the CPU will
      // switch to the new PLL value.
      //
      // This time-to-lock is monitored by a PLL lock counter.
      //
      // Code is not required to sit and wait for the PLL to lock.
      // However, if the code does anything that is timing critical,
      // and requires the correct clock be locked, then it is best to
      // wait until this switching has completed.

      // Wait for the PLL lock bit to be set.
      // The watchdog should be disabled before this loop, or fed within
      // the loop via ServiceDog().

	  // Uncomment to disable the watchdog
      WDogDisable();

	  while(SysCtrlRegs.PLLSTS.bit.PLLLOCKS != 1) {}

      EALLOW;
      SysCtrlRegs.PLLSTS.bit.MCLKOFF = 0;
	  EDIS;
	}

	  // Divide down SysClk by 2 to increase stability.
	EALLOW;
	SysCtrlRegs.PLLSTS.bit.DIVSEL = 2; 
	EDIS;
}

// This function initializes the PIE control registers to a known state.
void PieCntlInit(void)
{
    // Disable Interrupts at the CPU level:
    DINT;

    // Disable the PIE.
    PieCtrlRegs.PIECTRL.bit.ENPIE = 0;  
	// Clear all PIEIER registers:
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;	// enable ADCINT1 interrupt                             отладка АЦП
   // PieCtrlRegs.PIEIER1.bit.INTx1 = 0;	// disable ADCINT1 interrupt на период отладки PFC

    PieCtrlRegs.PIEIER1.bit.INTx2 = 0;	// disable ADCINT2 interrupt

    PieCtrlRegs.PIEIER1.bit.INTx6 = 0;	// disable ADCINT9 interrupt

	PieCtrlRegs.PIEIER2.bit.INTx1 = 0;	// disable EPWM1_TZINT interrupt
	PieCtrlRegs.PIEIER2.bit.INTx2 = 0;	// disable EPWM2_TZINT interrupt
	
	//PieCtrlRegs.PIEIER3.bit.INTx1 = 1;  // enable EPWM1_INT interrupt
	PieCtrlRegs.PIEIER3.bit.INTx1 = 0;	// enable EPWM1_INT interrupt                           отладка АЦП
	PieCtrlRegs.PIEIER3.bit.INTx2 = 0;	// disable EPWM1_INT	interrupt
	PieCtrlRegs.PIEIER3.bit.INTx5 = 1;	// enable EPWM5_INT	interrupt for secondary slow ISR
	PieCtrlRegs.PIEIER3.bit.INTx6 = 1;	// enable EPWM6_INT	interrupt
		
	PieCtrlRegs.PIEIER4.bit.INTx1 = 0;  // disable eCAP1 interrupt
	PieCtrlRegs.PIEIER5.all = 0;
	PieCtrlRegs.PIEIER6.bit.INTx1 = 0;  // enable SPI receive interrupt
	PieCtrlRegs.PIEIER7.all = 0;
	PieCtrlRegs.PIEIER8.all = 0;
	PieCtrlRegs.PIEIER9.all = 0;
	//PieCtrlRegs.PIEIER10.bit.INTx1 = 0; // enable ADCINT1 interrupt
	//PieCtrlRegs.PIEIER10.bit.INTx2 = 0; // enable ADCINT2 interrupt
	//PieCtrlRegs.PIEIER10.bit.INTx3 = 0; // enable ADCINT3 interrupt
	PieCtrlRegs.PIEIER11.all = 0;
	PieCtrlRegs.PIEIER12.all = 0;

	// Clear all PIEIFR registers:
	PieCtrlRegs.PIEIFR1.all = 0;
	PieCtrlRegs.PIEIFR2.all = 0;
	PieCtrlRegs.PIEIFR3.all = 0;	
	PieCtrlRegs.PIEIFR4.all = 0;
	PieCtrlRegs.PIEIFR5.all = 0;
	PieCtrlRegs.PIEIFR6.all = 0;
	PieCtrlRegs.PIEIFR7.all = 0;
	PieCtrlRegs.PIEIFR8.all = 0;
	PieCtrlRegs.PIEIFR9.all = 0;
	PieCtrlRegs.PIEIFR10.all = 0;
	PieCtrlRegs.PIEIFR11.all = 0;
	PieCtrlRegs.PIEIFR12.all = 0;
	XIntruptRegs.XINT1CR.bit.ENABLE = 0;
	XIntruptRegs.XINT1CR.bit.POLARITY = 2;
	IER |= 0x20F;  //enable group 10, group 4,  group 3, group 2 and group 1  interrupts
	IER |= M_INT6; //enable Timer1 interrupt
	IER |= M_INT13;//enable Timer1 interrupt
	IER |= M_INT14;//enable Timer2 interrupt 
//	DBGIER |= 0x200;
}	

void PieVectTableInit(void)
{
		
	EALLOW;	

	PieVectTable.EPWM6_INT = &int_EPWM6;
	PieVectTable.EPWM5_INT = &SECONDARY_ISR;
	//PieVectTable.EPWM1_INT = &DPL_ISR;
	PieVectTable.ADCINT1 = &DPL_ISR;
		
	EDIS;
	
	// Enable the PIE Vector Table
	PieCtrlRegs.PIECTRL.bit.ENPIE = 1;	//enable PIE 
}

interrupt void ISR_ILLEGAL(void) // Illegal operation TRAP.
{
  // Insert ISR Code here.
;
  // Next two lines for debug only to halt the processor here
  // Remove after inserting ISR Code.
  asm("          ESTOP0");
  for(;;);

}

void ePWM_prefare()
{
	EALLOW;
	/////////////////////PWM5 for Secondary_ISR/////////////////////////////////////////////////////////////////////////////////////
	EPwm5Regs.TBPRD = SECONDARY_ISR_PERIOD;		    // Period =5*1170 TBCLK counts (5120 Hz @ 60MHz clock) for Up-down-count mode
	//EPwm2Regs.TBPHS.half.TBPHS = 511; 			// Set Phase register to 511 discretes
	EPwm5Regs.TBPHS.half.TBPHS = 0; 				// Set Phase register to 0 discretes
	EPwm5Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; 	// Up-count mode
	EPwm5Regs.TBCTL.bit.PHSEN = 0; 	    			// Phase loading disabled
	EPwm5Regs.TBCTL.bit.PRDLD = 0;
	//EPwm4Regs.TBCTL.bit.SYNCOSEL = 1;
	EPwm5Regs.CMPCTL.bit.SHDWAMODE = 0; 			// Shadow mode. Operates as a double buffer. All writes via the CPU access the shadow register
	EPwm5Regs.CMPCTL.bit.LOADAMODE = 0; 			// load on CTR=Zero. This bit has no effect in immediate mode (CMPCTL[SHDWAMODE] = 1)
	EPwm5Regs.CMPA.half.CMPA = 600;     	    	// duty cycle of PWM5
	//EPwm3Regs.CMPA.half.CMPA = 400;
	EPwm5Regs.CMPB = 0;							    // момент запуска измерения первичного тока2
	EPwm5Regs.TBCTR = 0x0000;						// clear counter 22.04.15

	//EPwm5Regs.AQCTLA.bit.ZRO = 2; 					// Clear: force EPWM3A output low
	//EPwm5Regs.AQCTLA.bit.CAU = 1; 					// Set: force EPWM2A output high
	//EPwm4Regs.AQCTLA.bit.CAD = 2; 					// Clear: force EPWM2A output low
	EPwm5Regs.ETSEL.bit.INTEN = 1;               	//
	EPwm5Regs.ETSEL.bit.INTSEL = ET_CTR_PRD;   		// Select Int from counter = PRD
	EPwm5Regs.ETPS.bit.INTPRD = ET_1ST;        		// Generate pulse on 1st event

	/////////////////////trip zone settings for cycle by cycle event//////////////////////////////////////////////
//	EPwm3Regs.TZSEL.bit.DCAEVT1 = TZ_ENABLE;      // Enable DCAEVT2 as CBC trip source for ePWM2 module.
//	EPwm3Regs.DCTRIPSEL.bit.DCAHCOMPSEL = DC_COMP1OUT;        // COMP1OUT is input of Event Qualification A
//	EPwm3Regs.TZDCSEL.bit.DCAEVT1 = TZ_DCAH_HI;     // DCAEVT2 = DCAH high(will become active)

//	EPwm3Regs.DCFCTL.bit.PULSESEL = DC_PULSESEL_ZERO;// Time-base counter equal to zero (TBCTR = 0x0000) - start blank timer
//	EPwm3Regs.DCFCTL.bit.BLANKINV = DC_BLANK_NOTINV;// Blanking window is not inverted
//	EPwm3Regs.DCFCTL.bit.BLANKE = DC_BLANK_ENABLE;    // Blanking window is enabled
//	EPwm3Regs.DCFCTL.bit.SRCSEL   = DC_SRC_DCAEVT1;   // Source Is DCAEVT2 Signal
//	EPwm3Regs.DCFOFFSET = 0;                      // no offset
//	EPwm3Regs.DCFWINDOW = 60;                     // Blanking window is  0.5 us
//	EPwm3Regs.DCACTL.bit.EVT1SRCSEL = DC_EVT_FLT; // Source Is DCEVTFILT Signal
	//EPwm3Regs.DCACTL.bit.EVT2SRCSEL = DC_EVT2;        // Filter is not used
//	EPwm3Regs.DCACTL.bit.EVT1FRCSYNCSEL = DC_EVT_ASYNC; // Take async path
	//EPwm3Regs.TZEINT.bit.CBC = 1;                 // enable interrupt for CBC 01.11.12

//	EPwm6Regs.TZSEL.bit.DCAEVT1 = TZ_ENABLE;      // Enable DCAEVT2 as CBC trip source for ePWM3 module.
//	EPwm6Regs.DCTRIPSEL.bit.DCAHCOMPSEL = DC_COMP1OUT;      // COMP2OUT is input of Event Qualification A
//	EPwm6Regs.TZDCSEL.bit.DCAEVT1 = TZ_DCAH_HI;     // DCAEVT2 = DCAH high(will become active)

//	EPwm6Regs.DCFCTL.bit.PULSESEL = DC_PULSESEL_ZERO;// Time-base counter equal to zero (TBCTR = 0x0000) - start blank timer
//	EPwm6Regs.DCFCTL.bit.BLANKINV = DC_BLANK_NOTINV; // Blanking window is not inverted
//	EPwm6Regs.DCFCTL.bit.BLANKE = DC_BLANK_ENABLE;   // Blanking window is enabled
//	EPwm6Regs.DCFCTL.bit.SRCSEL   = DC_SRC_DCAEVT1;   // Source Is DCAEVT2 Signal
//	EPwm6Regs.DCFOFFSET = 0;                      // no offset
//	EPwm6Regs.DCFWINDOW = 60;                     // Blanking window is  0.5 us
//	EPwm6Regs.DCACTL.bit.EVT1SRCSEL = DC_EVT_FLT;;    // Source Is DCEVTFILT Signal
	//EPwm6Regs.DCACTL.bit.EVT2SRCSEL = DC_EVT2;        // Filter is not used
//	EPwm6Regs.DCACTL.bit.EVT1FRCSYNCSEL = DC_EVT_ASYNC; // Take async path
	//EPwm6Regs.TZEINT.bit.CBC = 1;                 // enable interrupt for CBC 01.11.12

    EPwm3Regs.TZSEL.bit.OSHT6 = 1;                  // Enable TZ6 as a one-shot trip source for  ePWM2  module
    EPwm3Regs.TZCTL.bit.TZA = TZ_FORCE_LO;          // Force EPWM1A to a low state

    EPwm6Regs.TZSEL.bit.OSHT6 = 1;                  // Enable TZ6 as a one-shot trip source for  ePWM2  module
    EPwm6Regs.TZCTL.bit.TZA = TZ_FORCE_LO;          // Force EPWM1A to a low state

	EPwm2Regs.TZSEL.bit.OSHT6 = 1;      			// Enable TZ6 as a one-shot trip source for  ePWM2  module
	EPwm2Regs.TZCTL.bit.TZA = TZ_FORCE_LO;			// Force EPWM2A to a low state

	EPwm1Regs.TZSEL.bit.OSHT6 = 1;      			// Enable TZ6 as a one-shot trip source for  ePWM2  module
	EPwm1Regs.TZCTL.bit.TZA = TZ_FORCE_LO;			// Force EPWM1A to a low state

	// UNTERRUPT
    EPwm6Regs.ETSEL.bit.INTEN = ET_ENABLE;                  // Enable EPWM2_INT generation
    EPwm6Regs.ETSEL.bit.INTSEL = ET_CTR_ZERO;                   // Enable event time-base counter equal to zero
	EPwm6Regs.ETPS.bit.INTPRD = 1;                  // Generate an interrupt on the first event INTCNT = 01 (first event)

	EPwm1Regs.ETSEL.bit.INTEN = ET_ENABLE;          // ENable EPWM1_INT generation
	EPwm1Regs.ETSEL.bit.INTSEL = ET_CTRU_CMPB;      // Enable event: time-base counter equal to CMPB when the timer is incrementing
	EPwm1Regs.ETPS.bit.INTPRD = ET_1ST;             // Generate an interrupt on the first event INTCNT = 01 (first event)

	/*3*/EPwm1Regs.TBCTL.bit.HSPCLKDIV = 0;
	/*3*/EPwm1Regs.TBCTL.bit.CLKDIV = 0;
	EPwm2Regs.TBCTL.bit.HSPCLKDIV = 0;
	EPwm2Regs.TBCTL.bit.CLKDIV = 0;
	/*1*/EPwm3Regs.TBCTL.bit.HSPCLKDIV = 0;
	/*1*/EPwm3Regs.TBCTL.bit.CLKDIV = 0;
	EPwm4Regs.TBCTL.bit.HSPCLKDIV = 0;
	EPwm4Regs.TBCTL.bit.CLKDIV = 0;

	EPwm5Regs.TBCTL.bit.HSPCLKDIV = 0;
	EPwm5Regs.TBCTL.bit.CLKDIV = 0;

	EPwm6Regs.TBCTL.bit.HSPCLKDIV = 0;
	EPwm6Regs.TBCTL.bit.CLKDIV = 0;
	SysCtrlRegs.PCLKCR0.bit.TBCLKSYNC  = 1;         // Enable TBCLK


	GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;			// 0=GPIO,  1=EPWM1A,  2=Resv,  3=Resv
	//GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;		// 1=OUTput,  0=INput

	GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;			// 0=GPIO,  1=EPWM1A,  2=Resv,  3=Resv
	//GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;		// 1=OUTput,  0=INput

////////////////////выключение инвертора на время отладки корректора//////////////////////////////

//	GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 0;		// 0=GPIO,  1=EPWM6A,  2=Resv,  3=Resv
//	GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;			// 1=OUTput,  0=INput
//	GpioDataRegs.GPASET.bit.GPIO10 = 1;
//
//	GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 0;			// 0=GPIO,  1=EPWM3A,  2=Resv,  3=Resv
//	GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;			// 1=OUTput,  0=INput
//	GpioDataRegs.GPASET.bit.GPIO4 = 1;

////////////////////включение инвертора после отладки корректора//////////////////////////////

	GpioCtrlRegs.GPAMUX1.bit.GPIO10 = 1;        // 0=GPIO,  1=EPWM6A,  2=Resv,  3=Resv
	//GpioCtrlRegs.GPADIR.bit.GPIO10 = 1;         // 1=OUTput,  0=INput
	//GpioDataRegs.GPACLEAR.bit.GPIO10 = 1;
	//GpioDataRegs.GPASET.bit.GPIO10 = 1;

	GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;         // 0=GPIO,  1=EPWM3A,  2=Resv,  3=Resv
	//GpioCtrlRegs.GPADIR.bit.GPIO4 = 1;          // 1=OUTput,  0=INput
	//GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;
	//GpioDataRegs.GPASET.bit.GPIO4 = 1;

	EDIS;
}

void init_ADC()
{
												// Configure the ADC to sample the temperature sensor
EALLOW;
//AdcRegs.ADCCTL1.bit.TEMPCONV = 1; 				// Connect A5 - temp sensor
//AdcRegs.ADCCTL1.bit.INTPULSEPOS = 1;			// INT pulse generation occurs 1 cycle prior to ADC result latching into its result register
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//AdcRegs.ADCSOC0CTL.bit.CHSEL = 0x3; 			// Set SOC0 to sample ADCINA1 - V_out_int (output_voltage)
//AdcRegs.ADCSOC1CTL.bit.CHSEL = 0x3; 			// Set SOC1 to sample ADCINB1 - I_in
//AdcRegs.ADCSAMPLEMODE.bit.SIMULEN0 = 1;			// Simultaneous sampling enable for SOC0/SOC1
//
//AdcRegs.ADCSOC2CTL.bit.CHSEL = 0x8; 			// Set SOC2 to sample ADCINB0 -  ouput_current
//
//AdcRegs.ADCSOC3CTL.bit.CHSEL = 0x6; 			// Set SOC2 to sample ADCINA6 - v_bus
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//AdcRegs.ADCSOC0CTL.bit.ACQPS = 6; 				// Set SOC0 ACQPS to 7 ADCCLK
//AdcRegs.ADCSOC1CTL.bit.ACQPS = 6; 				// Set SOC1 ACQPS to 7 ADCCLK
//
//AdcRegs.ADCSOC2CTL.bit.ACQPS = 6; 				// Set SOC2 ACQPS to 7 ADCCLK
//AdcRegs.ADCSOC3CTL.bit.ACQPS = 6; 				// Set SOC2 ACQPS to 7 ADCCLK
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//AdcRegs.ADCSOC0CTL.bit.TRIGSEL = 0x0A;			// ePWM3, ADCSOCB starts a conversion	SOC0 (ADCINT1)  V_out mesuring
//AdcRegs.ADCSOC1CTL.bit.TRIGSEL = 0x0A;			// ePWM3, ADCSOCB starts a conversion	SOC1 (ADCINT1)  zero_level
//
//AdcRegs.ADCSOC2CTL.bit.TRIGSEL = 0x9;			// ePWM3, ADCSOCA starts a conversion	SOC2 (ADCINT2)	I_avg
//AdcRegs.ADCSOC3CTL.bit.TRIGSEL = 0xA;			// ePWM6, ADCSOCB starts a conversion	SOC3 (ADCINT3)	I_max
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//AdcRegs.INTSEL1N2.bit.INT1SEL = 0; 				// Connect ADCINT1 to EOC0
//AdcRegs.INTSEL1N2.bit.INT1E = 1; 				// Enable ADCINT1
//
//
//AdcRegs.INTSEL1N2.bit.INT2SEL = 2; 				// Connect ADCINT2 to EOC2
//AdcRegs.INTSEL1N2.bit.INT2E = 1; 				// Enable ADCINT2
//
//AdcRegs.INTSEL9N10.bit.INT9SEL = 3; 			// Connect ADCINT9 to EOC3
//AdcRegs.INTSEL9N10.bit.INT9E = 1; 				// Enable ADCINT3
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//AdcRegs.ADCCTL1.bit.ADCREFSEL = 1;				// External reference is select
//AdcRegs.ADCCTL1.bit.ADCPWDN = 1; 				// The analog circuitry inside the core is powered up
//AdcRegs.ADCCTL1.bit.ADCBGPWD = 1;				// Bandgap buffer's curcuitry inside core is powered up
//AdcRegs.ADCCTL1.bit.ADCREFPWD = 1;				// Reference buffer's curcuitry inside core is powered down
//AdcRegs.ADCCTL1.bit.ADCENABLE = 1;  			// ADC is enable
//
//Comp1Regs.COMPCTL.bit.COMPDACEN = 1;			// Comparator_DAC logic is powered up
//Comp1Regs.COMPCTL.bit.SYNCSEL = 0;  			// Synchronous version of comparator output is passed
//Comp1Regs.COMPCTL.bit.QUALSEL = 4;  			// Input to the block must be consistent for 5 consecutive
//Comp1Regs.COMPCTL.bit.CMPINV = 0;

//Comp1Regs.COMPCTL.bit.COMPDACEN = 1;			// Comparator_DAC logic is powered up
//Comp1Regs.COMPCTL.bit.SYNCSEL = 0;  			// Synchronous version of comparator output is passed
//Comp1Regs.COMPCTL.bit.QUALSEL = 4;  			// Input to the block must be consistent for 5 consecutive
//Comp1Regs.COMPCTL.bit.CMPINV = 0;				// срабатывание по отрицательной полуволне 25.04.15


//Comp1Regs.DACVAL.all = COMP1_REF_TRG;			// ограничение положительной полуволны
//Comp3Regs.DACVAL.all = COMP3_REF_TRG;			// ограничение положительной полуволны

//AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 0x3;		// SOC0-SOC2 are high priority, SOC3-SOC15 are in round robin mode
//AdcRegs.SOCPRICTL.bit.SOCPRIORITY = 0x8;		// приоритет устанвливается в файле cnf
EDIS;


}

void init_T0(void)
{
CpuTimer0Regs.TCR.bit.TIF = 1; 		  			// clear interrupt flag
CpuTimer0Regs.TCR.bit.TIE = 0;		  			// disable timer interrupt
}

void init_T1(void)
{
CpuTimer1Regs.TCR.bit.TIF = 1; 		  			// clear interrupt flag
CpuTimer1Regs.TCR.bit.TIE = 0;		  			// enable timer1 interrupt
}

void init_T2(void)
{
CpuTimer2Regs.TCR.bit.TIF = 1;                  // clear interrupt flag
CpuTimer2Regs.TCR.bit.TIE = 0;                  // disable timer1 interrupt
}


// This function initializes the Flash Control registers.

//                   CAUTION
// This function MUST be executed out of RAM. Executing it
// out of OTP/Flash will yield unpredictable results.

void InitFlash(void)
{
   EALLOW;
   // Enable Flash Pipeline mode to improve performance
   // of code executed from Flash.
   FlashRegs.FOPT.bit.ENPIPE = 1;

   //                    CAUTION
   // Minimum waitstates required for the flash operating
   // at a given CPU rate must be characterized by TI.
   // Refer to the datasheet for the latest information.

   // Set the Paged Waitstate for the Flash.
   FlashRegs.FBANKWAIT.bit.PAGEWAIT = 3;

   // Set the Random Waitstate for the Flash.
   FlashRegs.FBANKWAIT.bit.RANDWAIT = 3;

   // Set the Waitstate for the OTP.
   FlashRegs.FOTPWAIT.bit.OTPWAIT = 5;

   //                         CAUTION
   // ONLY THE DEFAULT VALUE FOR THESE 2 REGISTERS SHOULD BE USED.
   FlashRegs.FSTDBYWAIT.bit.STDBYWAIT = 0x01FF;
   FlashRegs.FACTIVEWAIT.bit.ACTIVEWAIT = 0x01FF;
   EDIS;

   // Force a pipeline flush to ensure that the write to
   // the last register configured occurs before returning.

   asm(" RPT #7 || NOP");
}


// This function will copy the specified memory contents from
// one location to another.
//
// Uint16 *SourceAddr        Pointer to the first word to be moved
//                            SourceAddr < SourceEndAddr
// Uint16* SourceEndAddr     Pointer to the last word to be moved
// Uint16* DestAddr          Pointer to the first destination word
//
// No checks are made for invalid memory locations or that the
// end address is > then the first start address.

void MemCopy(Uint16 *SourceAddr, Uint16* SourceEndAddr, Uint16* DestAddr)
{
    while(SourceAddr < SourceEndAddr)
    {
       *DestAddr++ = *SourceAddr++;
    }
    return;
}

//===========================================================================
// End of file.
//===========================================================================









