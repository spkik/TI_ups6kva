;----------------------------------------------------------------------------------
;	FILE:			{ProjectName}-DPL-ISR.asm
;
;	Description:	{ProjectName}-DPL-ISR.asm contains the ISR for the system
;					It also contains the initailization routine for all the macros 
;					being used in the system both for CLA and C28x macros
;   
;   Revision/ Version: See {ProjectName}-Main.c
;----------------------------------------------------------------------------------

		;Gives peripheral addresses visibility in assembly
	    .cdecls   C,LIST,"PeripheralHeaderIncludes.h"

		;include C header file - sets INCR_BUILD etc.(used in conditional builds)
		.cdecls C,NOLIST, "Settings.h"

		;Include files for the Power Library Maco's being used by the system
		.include "ADCDRV_1ch.asm"
		.include "CNTL_2P2Z.asm"
		.include "PWMDRV_1ch_UpDwnCnt.asm"
		.include "PWMDRV_1ch_UpDwnCntCompl.asm"

		.include "PFC_ICMD.asm"
		.include "MATH_EMAVG.asm"
		.include "CNTL_2P2Z.asm"
		.include "PFC_INVSQR.asm"
		.include "PFC_InvRmsSqr.asm"
			
;=============================================================================
; Digital Power library - Initailization Routine 
;=============================================================================

		; label to DP initialisation function
		.def 	_DPL_Init
		.data
		.def	 IntCount

		.ref	_SyncPosFlag
		.ref	_SyncNegFlag

		.ref	_PosAngle
		.ref	_NegAngle
		.ref	_VTimer1
		.ref	_VTimer2

		.ref 	_sampleCount
		.ref	_ADCout_V
		.ref	_ADCout_HALF
		.ref    _ADCout_I
		.ref	_V_Fdb
		.ref	_V_Ref
		.ref	_V_Out
		.ref	_I_Fdb
		.ref	_I_Out
		.ref	_Zero_Duty
		.ref	_Duty
		.ref	_sineTable_50Hz

		.ref 	_DutyA
		.ref	_ab_run_flag

		; dummy variable for pointer initialisation
ZeroNet	 		.usect "ZeroNet_Section",2,1,1	; output terminal 1
VloopCtr 		.usect "ISRVariables",2,1,1
;sineTable_50Hz 	.usect "SineTableSection",256,1 ;
IntCount		.usect "ISR_Section",1,1,1 ;
;SyncPosFlag		.usect "ISR_Section",1,1,1 ;
;SyncNegFlag		.usect "ISR_Section",1,1,1 ;

		.text
_DPL_Init:
		ZAPA
		MOVL	XAR0, #ZeroNet
		MOVL	*XAR0, ACC

		; Initialize all the DP library macro used here 
		;---------------------------------------------------------
		.if(INCR_BUILD = 10)
			ADCDRV_1ch_INIT 0
			ADCDRV_1ch_INIT 1
			ADCDRV_1ch_INIT 2

			CNTL_2P2Z_INIT V	; контур напряжения
			CNTL_2P2Z_INIT I	; контур тока

			PWMDRV_1ch_UpDwnCnt_INIT 6
			PWMDRV_1ch_UpDwnCnt_INIT 3

		.endif


		.if(INCR_BUILD = 1)

			ADCDRV_1ch_INIT 3
			ADCDRV_1ch_INIT 4
			ADCDRV_1ch_INIT 5
			ADCDRV_1ch_INIT 6
			ADCDRV_1ch_INIT 7

			PWMDRV_1ch_UpDwnCnt_INIT 1
			PWMDRV_1ch_UpDwnCnt_INIT 2

			MOVW	DP, #IntCount
			MOV	    @IntCount, #0

			MOVW	DP, #_sampleCount
			MOV	    @_sampleCount, #0

		.endif
		;---------------------------------------------------------

		.if(INCR_BUILD = 2)
		;//////////////////////////////////////////////////DC_AC////////////////////////////////////////////
			ADCDRV_1ch_INIT 0
			ADCDRV_1ch_INIT 1
			ADCDRV_1ch_INIT 2

			CNTL_2P2Z_INIT V	; контур напряжения
			CNTL_2P2Z_INIT I	; контур тока

			PWMDRV_1ch_UpDwnCnt_INIT 6
			PWMDRV_1ch_UpDwnCnt_INIT 3

		;//////////////////////////////////////////////////AC_DC////////////////////////////////////////////

			ADCDRV_1ch_INIT 3
			ADCDRV_1ch_INIT 4
			ADCDRV_1ch_INIT 5
			ADCDRV_1ch_INIT 6

			CNTL_2P2Z_INIT 1

			MATH_EMAVG_INIT 2
			MATH_EMAVG_INIT 1

			PFC_InvRmsSqr_INIT 1

			PFC_ICMD_INIT 1				;Bridgeless PFC current command init

			PWMDRV_1ch_UpDwnCnt_INIT 2	; PWM2A
			PWMDRV_1ch_UpDwnCnt_INIT 1	; PWM1A
			;PWMDRV_1ch_UpDwnCnt_INIT 4	; PWM4A

		.endif

		.if(INCR_BUILD = 3)

		;/////////////////////////////////////////////////////DC_AC////////////////////////////////////////
			ADCDRV_1ch_INIT 0
			ADCDRV_1ch_INIT 1
			ADCDRV_1ch_INIT 2

			CNTL_2P2Z_INIT V	; контур напряжения
			CNTL_2P2Z_INIT I	; контур тока

			PWMDRV_1ch_UpDwnCnt_INIT 6
			PWMDRV_1ch_UpDwnCnt_INIT 3

		;//////////////////////////////////////////////////////AC_DC//////////////////////////////////////

			ADCDRV_1ch_INIT 3
			ADCDRV_1ch_INIT 4
			ADCDRV_1ch_INIT 5
			ADCDRV_1ch_INIT 6

			CNTL_2P2Z_INIT 2		;voltage compensator
			CNTL_2P2Z_INIT 1		;current compensator

			MATH_EMAVG_INIT 1

			PFC_InvRmsSqr_INIT 1

			PFC_ICMD_INIT 1				;Bridgeless PFC current command init

			PWMDRV_1ch_UpDwnCnt_INIT 2	; PWM2A
			PWMDRV_1ch_UpDwnCnt_INIT 1	; PWM1A

			MOVW	DP, #IntCount
			MOV	    @IntCount, #0

			MOVW	DP, #_sampleCount
			MOV	    @_sampleCount, #0

			;MOVW	DP, #SyncPosFlag
			;MOV	    @SyncPosFlag, #0

			;MOVW	DP, #SyncNegFlag
			;MOV	    @SyncNegFlag, #0

		.endif

		LRETR

;-----------------------------------------------------------------------------------------

; Digital Power library - Interrupt Service Routine

		.sect "ramfuncs"
		; label to DP ISR Run function
		.def	_DPL_ISR

_DPL_ISR:
		; full context save - push any unprotected registers onto stack
		PUSH  	AR1H:AR0H
		PUSH  	XAR2
		PUSH  	XAR3
		PUSH  	XAR4
		PUSH  	XAR5
		PUSH  	XAR6
		PUSH  	XAR7
		PUSH  	XT
		SPM   	0          				; set C28 mode
		CLRC  	AMODE       
		CLRC  	PAGE0,OVM 
;		CLRC	INTM					; clear interrupt mask - comment if ISR non-nestable
;-----------------------------------------------------------------------------------------

; call DP library modules


		.if(INCR_BUILD = 10)
		;///////////////////////////////////////////DC_AC DPL_ISR SECTION////////////////////////////////////////////////////////////////////

			ADCDRV_1ch 0				; V_OUT_INT load adc result
			ADCDRV_1ch 1				; HALF_REF load adc result

			MOVW	DP,#_ADCout_V
			MOVL	ACC,@_ADCout_V		; ACC = Vout
			MOVW	DP,#_ADCout_HALF
			SUBL	ACC,@_ADCout_HALF	; ACC = Vout - half_ref
			MOVW	DP,#_V_Fdb
			MOVL	@_V_Fdb, ACC

			MOVW	DP,#IntCount
			INC		@IntCount
			TBIT 	@IntCount, #2
			SB		COMPV,NTC
			AND		@IntCount,#(1<<3)


			MOVW	DP,#_sampleCount
			MOVL 	XAR4,#_sineTable_50Hz   ;в XAR4 адрес 0-го элемента sinTable_50Hz
			MOV 	ACC,@_sampleCount << #1		;в ACC индекс (смещение)

			ADDL	@XAR4,ACC				;в XAR4 = адрес 0-го элемента sinTable_50Hz + смещение
			MOVL	ACC,*XAR4				;загружаем в ACC содержимое ячейки sinTable_50Hz, на кот. указывает XAR4
			MOVW	DP,#_V_Ref
			MOVL 	@_V_Ref,ACC				;выгружаем аккумулятор в переменную

			MOVW	DP,#_sampleCount
			INC 	@_sampleCount
			AND		@_sampleCount,#0x7F


COMPV:
			CNTL_2P2Z V

			ADCDRV_1ch 2
			MOVW	DP,#_ADCout_I
			MOVL	ACC,@_ADCout_I		; ACC = Vout
			MOVW	DP,#_ADCout_HALF
			SUBL	ACC,@_ADCout_HALF	; ACC = Vout - half_ref
			MOVW	DP,#_ADCout_I
			MOVL	@_I_Fdb, ACC

			CNTL_2P2Z I

			MOVW	DP,#_I_Out
			MOVL	ACC,@_I_Out
			MOVW	DP,#_Zero_Duty
			ADDL	ACC,@_Zero_Duty
			MOVW	DP,#_Duty
			MOVL	@_Duty,ACC

			PWMDRV_1ch_UpDwnCnt 6 		; Run PWMDRV_1ch_UpDwnCntCompl ePWM6
			PWMDRV_1ch_UpDwnCnt 3 		; Run PWMDRV_1ch_UpDwnCntCompl ePWM3			*/

		.endif

		;///////////////////////////////////////////END DC_AC DPL_ISR SECTION//////////////////////////////////////////////////////////////

		.if(INCR_BUILD = 1)

			ADCDRV_1ch 3				; Ipfc load adc result
			ADCDRV_1ch 4				; Vbus load adc result
			ADCDRV_1ch 5				; VL_fb load adc result
			ADCDRV_1ch 6				; VN_fb load adc result

			PWMDRV_1ch_UpDwnCnt 2 		; Run PWMDRV_1ch_UpDwnCntCompl ePWM2			*/
			;PWMDRV_1ch_UpDwnCnt 1 		; Run PWMDRV_1ch_UpDwnCntCompl ePWM1

		.endif
;----------------------------------------------------------------------------------------
;-----------------------------------------------------------------------------------------
		.if(INCR_BUILD = 2)

			ADCDRV_1ch 3				; Ipfc load adc result
			ADCDRV_1ch 4				; Vbus load adc result
			ADCDRV_1ch 5				; VL_fb load adc result
			ADCDRV_1ch 6				; VN_fb load adc result

			PFC_ICMD 1					;Bridgeless PFC current command

			CNTL_2P2Z		1

			PWMDRV_1ch_UpDwnCnt 2		; PWM2A
			PWMDRV_1ch_UpDwnCnt 1		; PWM1A

		    PFC_InvRmsSqr  	1
		    MATH_EMAVG		1
			MATH_EMAVG		2


			;PWMDRV_1ch_UpDwnCnt 4		; PWM4A

		.endif

;------------------------------------------------------------------------------------------------------------
		.if(INCR_BUILD = 3)

		;////////////////////////////////////////////////////////////////DC_AC DPL_ISR SECTION///////////////////////////////////////////////////////////////////////////

			ADCDRV_1ch 0				; V_OUT_INT load adc result
			ADCDRV_1ch 1				; HALF_REF load adc result

			MOVW	DP,#_ADCout_V
			MOVL	ACC,@_ADCout_V		; ACC = Vout
			MOVW	DP,#_ADCout_HALF
			SUBL	ACC,@_ADCout_HALF	; ACC = Vout - half_ref
			MOVW	DP,#_V_Fdb
			MOVL	@_V_Fdb, ACC
		;///////////////////////////////////////выборка из таблицы нового значения, либо использование старого взависимости/////////////////////////////////////////////
			MOVW	DP,#IntCount
			INC		@IntCount
			TBIT 	@IntCount, #2
			SB		COMPV,NTC				;если номер ШИМ-периода не достиг 4 (начиная с 0), то используем предыдущее значение задания регулятора
			AND		@IntCount,#(1<<3)		;если же достиг, сбрасываем это бит, т.е обнуляем номер ШИМ- периода и переходим к процедуре выборки нового значения из таблицы

			MOVW	DP,#_sampleCount
			MOVL 	XAR4,#_sineTable_50Hz   ;в XAR4 адрес 0-го элемента sinTable_50Hz
			MOV 	ACC,@_sampleCount << #1		;в ACC индекс (смещение)

			ADDL	@XAR4,ACC				;в XAR4 = адрес 0-го элемента sinTable_50Hz + смещение
			MOVL	ACC,*XAR4				;загружаем в ACC содержимое ячейки sinTable_50Hz, на кот. указывает XAR4
			MOVW	DP,#_V_Ref
			MOVL 	@_V_Ref,ACC				;выгружаем аккумулятор в переменную

			MOVW	DP,#_sampleCount
			INC 	@_sampleCount
			AND		@_sampleCount,#0x7F
		;///////////////////////////////////////////от номера ШИМ-периода и загрузка его в поле задания регулятора////////////////////////////////////////////////////////

COMPV:
			CNTL_2P2Z V

			ADCDRV_1ch 2
			MOVW	DP,#_ADCout_I
			MOVL	ACC,@_ADCout_I		; ACC = Vout
			MOVW	DP,#_ADCout_HALF
			SUBL	ACC,@_ADCout_HALF	; ACC = Vout - half_ref
			MOVW	DP,#_ADCout_I
			MOVL	@_I_Fdb, ACC

			CNTL_2P2Z I

			;MOVW	DP,#_I_Out
			;MOVL	ACC,@_I_Out
					MOVW	DP,#_V_Out
					MOVL	ACC,@_V_Out
			MOVW	DP,#_Zero_Duty
			ADDL	ACC,@_Zero_Duty
			MOVW	DP,#_Duty
			MOVL	@_Duty,ACC

			PWMDRV_1ch_UpDwnCnt 6 		; Run PWMDRV_1ch_UpDwnCntCompl ePWM6
			PWMDRV_1ch_UpDwnCnt 3 		; Run PWMDRV_1ch_UpDwnCntCompl ePWM3			*/

		;///////////////////////////////////////////////////////////////////PFC DPL_ISR SECTION//////////////////////////////////////////

			ADCDRV_1ch 		3			; Ipfc load adc result
			ADCDRV_1ch 		4			; Vbus load adc result
			ADCDRV_1ch 		5			; VL_fb load adc result
			ADCDRV_1ch 		6			; VN_fb load adc result

			PFC_ICMD 		1			; Bridgeless PFC current command

			CNTL_2P2Z		1			; PFC current loop compensator

			PWMDRV_1ch_UpDwnCnt 1		; PWM1A
			PWMDRV_1ch_UpDwnCnt 2		; PWM2A

		    PFC_InvRmsSqr  	1
			MATH_EMAVG		1


		;Execute Vloop every VoltCurrLoopExecRatio times, defined in BridgelessPFC-Settings.h file
			MOVW	DP,#(VloopCtr)
			INC		@VloopCtr
			CMP		@VloopCtr,#VoltCurrLoopExecRatio
			B		SKIP_VLOOP_CALC,LT

			MOV		@VloopCtr,#0
			CNTL_2P2Z		2			; PFC Volt loop compensator

SKIP_VLOOP_CALC:

		.endif
;----------------------------------------------------
			MOVW	DP,#_ab_run_flag
			TBIT 	@_ab_run_flag, #0
			SB		CalculateVrect,NTC  				; если флаг работы от АБ не установлен, работаем от сети, определяем полярность Vin

	        MOVW 	DP, #_AdcResult                     ; load Data Page to read ADC results
        	MOV		ACC, @_AdcResult.ADCRESULT0<<12		; ACC = V_OUT_INT
        	SUB		ACC, @_AdcResult.ADCRESULT1<<12		; ACC = V_OUT_INT - half_ref

        	B		NegativeCycle_INV, LEQ				; Branch to Negative Half Cycle
			B		PositiveCycle_INV, UNC

CalculateVrect:
	        MOVW 	DP, #_AdcResult                     ; load Data Page to read ADC results
        	MOV		ACC, @_AdcResult.ADCRESULT5<<12		; ACC = Line
        	SUB		ACC, @_AdcResult.ADCRESULT6<<12		; ACC = Line - Neutral
        	B		NegativeCycle, LEQ					; Branch to Negative Half Cycle
PositiveCycle:
			; Save Vrect
			.ref	_Vrect
        	MOVW	DP, #_Vrect
			MOVL	@_Vrect, ACC

PositiveCycle_INV:
			; ePWM1 & ADC configuration
	        MOVW 	DP, #_AdcRegs.ADCSOC4CTL            ; load Data Page to read ADC results
	        EALLOW
			;MOV		@_AdcRegs.ADCSOC4CTL.bit.CHSEL, #10	; Switch ADC to mesure VbusH voltage
			MOV		@_AdcRegs.ADCSOC4CTL,#0x3A86		; Switch ADC
			EDIS
	        MOVW 	DP, #_EPwm2Regs.AQCTLA                     ; load Data Page to read ePWM registers
			;MOV		@_EPwm2Regs.AQCTLA.bit.CAU, #1		; CLEAR ePWM1 on CompA-Up (enable switching)
			;MOV		@_EPwm2Regs.AQCTLA.bit.CAD, #2		; SET ePWM1 on CompA-Down (enable switching)
			MOV		@_EPwm2Regs.AQCTLA, #144			; SET ePWM1 on CompA-Down, CLEAR CompA-Up (enable switching)
	        MOVW 	DP, #_EPwm1Regs.AQCTLA                     ; load Data Page to read ePWM registers
;			MOV		@_EPwm2Regs.AQCTLA.bit.CAU, #1		; CLEAR ePWM2 on CompA-Up
;			MOV		@_EPwm2Regs.AQCTLA.bit.CAD, #1		; CLEAR ePWM2 on CompA-Down (force low)
			MOV		@_EPwm1Regs.AQCTLA, #160				; CLEAR ePWM2 on CompA-Up/Down (force low)
			; Check if near Zero crossing before forcing ePWM2 High

			MOVW	DP,#_SyncPosFlag
			TBIT 	@_SyncPosFlag, #0
			SB		SkipPWM1Force,TC							;jump if test bit set

			SUB		ACC, #500<<12
			;SUB		ACC, #300<<12
			;SUB		ACC, #100<<12
			;SUB		ACC, #25<<12
			B		SkipPWM1Force, GT        			; пропускаем если далеко от нуля

			;MOVW 	DP, #_EPwm1Regs.AQCTLA                     ; load Data Page to read ePWM registers
;			MOV		@_EPwm2Regs.AQCTLA.bit.CAU, #2		; SET ePWM2 on CompA-Up (force high)
;			MOV		@_EPwm2Regs.AQCTLA.bit.CAD, #2		; SET ePWM2 on CompA-Down
			;MOV		@_EPwm1Regs.AQCTLA, #160			; SET ePWM2 on CompA-Up/Down (force high)


			MOVW	DP,#_SyncPosFlag
			OR		@_SyncPosFlag,#(1<<0)				;устанавливаем флаг синхронизации по "+" полуволне
			MOVW	DP,#_SyncNegFlag
			AND		@_SyncNegFlag,#~(1<<0)				;сбрасываем флаг синхронизации по "-" полуволне

			MOVW	DP,#_ab_run_flag
			TBIT 	@_ab_run_flag, #0
			SB		SkipPWM1Force,TC  					; если флаг работы от АБ установлен, пропускаем след. фрагмент

			MOVW	DP,#_sampleCount					; начало "+" полуволны для инвертора
			MOV 	@_sampleCount,#0
			MOVW	DP,#IntCount						; начало "+" полуволны для инвертора
			MOV 	@IntCount,#0

			MOVW 	DP, #_VTimer1
			MOVW 	@_VTimer1, #0						;очистка софт-таймера1 для отсчета "+" угла

			MOVW	DP,#_PosAngle
			OR		@_PosAngle,#(1<<0)					;устанавливаем признак отсчета угла по "+" волне
			MOVW	DP,#_NegAngle
			AND		@_NegAngle,#~(1<<0)					;сбрасываем признак отсчета угла по "-" волне

;			MOV		@_EPwm2Regs.AQCTLA.bit.CAU, #2		; SET ePWM2 on CompA-Up (force high)
;			MOV		@_EPwm2Regs.AQCTLA.bit.CAD, #2		; SET ePWM2 on CompA-Down
;			MOV		@_EPwm1Regs.AQCTLA, #160			; SET ePWM2 on CompA-Up/Down (force high)
SkipPWM1Force:
;	        MOVW 	DP, #_GpioDataRegs.GPADAT                 	; load Data Page to read GPIO registers
;			MOV 	@_GpioDataRegs.GPASET, #128 ; Set GPIO7, Used for debug purposes


        	B		ControlLoopEnd, UNC
NegativeCycle:
	; Save Vrect
        	MOV		ACC, @_AdcResult.ADCRESULT6<<12		; ACC = Neutral
        	SUB		ACC, @_AdcResult.ADCRESULT5<<12		; ACC = Neutral - Line
        	MOVW	DP, #_Vrect
			MOVL	@_Vrect, ACC

NegativeCycle_INV:
	; ePWM2 & ADC configuration
	        MOVW 	DP, #_AdcRegs.ADCSOC4CTL            ; load Data Page to read ADC results
;			MOV		@_AdcRegs.ADCSOC1CTL.bit.CHSEL, #4	; Switch ADC to mesure VbusH voltage
			EALLOW
			MOV		@_AdcRegs.ADCSOC4CTL, #0x2B06		; Switch ADC
	        EDIS
	        MOVW 	DP, #_EPwm1Regs.AQCTLA                   ; load Data Page to read ePWM registers
;			MOV		@_EPwm1Regs.AQCTLA.bit.CAU, #1		; CLEAR ePWM2 on CompA-Up (enable switching)
;			MOV		@_EPwm1Regs.AQCTLA.bit.CAD, #2		; SET ePWM2 on CompA-Down (enable switching)
			MOV		@_EPwm1Regs.AQCTLA, #144			; SET ePWM2 on CompA-Down, CLEAR CompA-Up (enable switching)
	        MOVW 	DP, #_EPwm2Regs.AQCTLA                     ; load Data Page to read ePWM registers
;			MOV		@_EPwm2Regs.AQCTLA.bit.CAU, #1		; CLEAR ePWM1 on CompA-Up
;			MOV		@_EPwm2Regs.AQCTLA.bit.CAD, #1		; CLEAR ePWM1 on CompA-Down (force low)
			MOV		@_EPwm2Regs.AQCTLA, #160				; CLEAR ePWM1 on CompA-Up/Down (force low)
	; Check if near Zero crossing before forcing ePWM1 High

			MOVW	DP,#_SyncNegFlag
			TBIT 	@_SyncNegFlag, #0
			SB		SkipPWM2Force,TC					;jump if test bit set

			SUB		ACC, #500<<12
			;SUB		ACC, #300<<12
			;SUB		ACC, #100<<12
			;SUB		ACC, #25<<12
			B		SkipPWM2Force, GT					; пропускаем если далеко от нуля

			;MOVW 	DP, #_EPwm2Regs.AQCTLA              ; load Data Page to read ePWM registers
;			MOV		@_EPwm1Regs.AQCTLA.bit.CAU, #2		; SET ePWM2 on CompA-Up (force high)
;			MOV		@_EPwm1Regs.AQCTLA.bit.CAD, #2		; SET ePWM2 on CompA-Down
			;MOV		@_EPwm2Regs.AQCTLA, #160			; SET ePWM2 on CompA-Up/Down (force high)

			MOVW	DP,#_SyncNegFlag
			OR		@_SyncNegFlag,#(1<<0)				;устанавливаем флаг синхронизации по "-" полуволне
			MOVW	DP,#_SyncPosFlag
			AND		@_SyncPosFlag,#~(1<<0)				;сбрасываем флаг синхронизации по "+" полуволне

			MOVW	DP,#_ab_run_flag
			TBIT 	@_ab_run_flag, #0
			SB		SkipPWM2Force,TC  					; если флаг работы от АБ установлен, пропускаем след. фрагмент

			MOVW	DP,#_sampleCount					;начало "+" полуволны для инвертора
			MOV 	@_sampleCount,#64
			MOVW	DP,#IntCount						; начало "+" полуволны для инвертора
			MOV 	@IntCount,#0

			MOVW 	DP, #_VTimer2
			MOVW 	@_VTimer2, #0						;очистка софт-таймера2 для отсчета "-" угла

			MOVW	DP,#_NegAngle
			OR		@_NegAngle,#(1<<0)					;устанавливаем признак отсчета угла по "-" волне
			MOVW	DP,#_PosAngle
			AND		@_PosAngle,#~(1<<0)					;сбрасываем признак отсчета угла по "+" волне

;			MOV		@_EPwm2Regs.AQCTLA.bit.CAU, #2		; SET ePWM1 on CompA-Up (force high)
;			MOV		@_EPwm2Regs.AQCTLA.bit.CAD, #2		; SET ePWM1 on CompA-Down
;			MOV		@_EPwm2Regs.AQCTLA, #160			; SET ePWM1 on CompA-Up/Down (force high)
SkipPWM2Force:
;	        MOVW 	DP, #_GpioDataRegs.GPADAT            ; load Data Page to read GPIO registers
;			MOV	@_GpioDataRegs.GPACLEAR, #128 			; Clear GPIO7, Used for debug purposes

			;MOVW	DP,#_sampleCount					; начало "-" полуволны для инвертора
			;MOV 	@_sampleCount,#64


ControlLoopEnd:


			;.endif
		;----------------------------------------------------------
;			.ref 	_Duty4A
;			MOVW	DP, #_Vrect
;			MOVL	ACC, @_Vrect
;			MOVW	DP, #_Duty4A
;			MOVL    @_Duty4A, ACC	;Write 9 bit value to Duty4A

;-----------------------------------------------------------------------------------------



; Interrupt management before exit

	.if(EPWMn_DPL_ISR=1)

	.if(EPWM1)
		MOVW 	DP,#_EPwm1Regs.ETCLR
		MOV 	@_EPwm1Regs.ETCLR,#0x01			; Clear EPWM1 Int flag
	.endif ; EPWM1

	.if(EPWM2)
		MOVW 	DP,#_EPwm2Regs.ETCLR
		MOV 	@_EPwm2Regs.ETCLR,#0x01			; Clear EPWM2 Int flag
	.endif ; EPWM2

	.if(EPWM3)
		MOVW 	DP,#_EPwm3Regs.ETCLR
		MOV 	@_EPwm3Regs.ETCLR,#0x01			; Clear EPWM3 Int flag
	.endif ; EPWM3

	.if(EPWM4)
		MOVW 	DP,#_EPwm4Regs.ETCLR
		MOV 	@_EPwm4Regs.ETCLR,#0x01			; Clear EPWM4 Int flag
	.endif ; EPWM4

	.if(EPWM5)
		MOVW 	DP,#_EPwm5Regs.ETCLR
		MOV 	@_EPwm5Regs.ETCLR,#0x01			; Clear EPWM5 Int flag
	.endif ; EPWM5

	.if(EPWM6)
		MOVW 	DP,#_EPwm6Regs.ETCLR
		MOV 	@_EPwm6Regs.ETCLR,#0x01			; Clear EPWM6 Int flag
	.endif ; EPWM6

		MOVW 	DP,#_PieCtrlRegs.PIEACK			; Acknowledge PIE interrupt Group 3
		MOV 	@_PieCtrlRegs.PIEACK, #0x4
	.endif ; EPWMn_ISR

	.if(CLAn_DPL_ISR=1)

		MOVW 	DP,#_PieCtrlRegs.PIEACK			; Acknowledge PIE interrupt Group 11
		MOV 	@_PieCtrlRegs.PIEACK, #0x0400	
		
	.endif 
	
	.if(ADC_DPL_ISR=1)
	; Case where ISR is triggered by ADC 
		MOVW 	DP,#_AdcRegs.ADCINTFLGCLR
		MOV 	@_AdcRegs.ADCINTFLGCLR,#0x01		; Clear ADCINT1 Flag

		MOVW 	DP,#_PieCtrlRegs.PIEACK			; Acknowledge PIE interrupt Group 1
		MOV 	@_PieCtrlRegs.PIEACK,#0x1
	.endif 

;-----------------------------------------------------------------------------------------
; full context restore
;		SETC	INTM					; set INTM to protect context restore
		POP   	XT
		POP   	XAR7
		POP   	XAR6
		POP   	XAR5
		POP   	XAR4
		POP   	XAR3
		POP   	XAR2
		POP   	AR1H:AR0H
		IRET							; return from interrupt
		.end

; end of file

