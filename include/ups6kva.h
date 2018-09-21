
#ifndef ups6kva
#define ups6kva

#define C2P2ZCoeff_V_B0   1.7590612
#define C2P2ZCoeff_V_B1   -3.4321850
#define C2P2ZCoeff_V_B2   1.7399285
#define C2P2ZCoeff_V_A1   0.7892720
#define C2P2ZCoeff_V_A2   0.2107280

#define C2P2ZCoeff_V_MAX 0.500000
#define C2P2ZCoeff_V_MIN -0.500000

#define C2P2ZCoeff_I_B2 0.000000
#define C2P2ZCoeff_I_B1 0.000000
#define C2P2ZCoeff_I_B0 1.300000
#define C2P2ZCoeff_I_A2 0.000000
#define C2P2ZCoeff_I_A1 0.000000

#define C2P2ZCoeff_I_MAX 0.500000
#define C2P2ZCoeff_I_MIN -0.500000

//////////////////////////////////////////////////PFC////////////////////////////////////////////////////////////////////

//#define C2P2ZCoeff_2_B2 0.000000
//#define C2P2ZCoeff_2_B1 -2.2299178
//#define C2P2ZCoeff_2_B0 2.23000490616406

//#define C2P2ZCoeff_2_B1 -2.2299178
//#define C2P2ZCoeff_2_B0 2.27347088203125
////////////////////////////////////////////////����������//////////////////////////////////
#define C2P2ZCoeff_2_B2 0.000000
#define C2P2ZCoeff_2_B1 -2.2299178
#define C2P2ZCoeff_2_B0 2.23078886164063

//#define C2P2ZCoeff_2_B1 -4.4598356
//#define C2P2ZCoeff_2_B0 4.46157772328125

#define C2P2ZCoeff_2_A2 0.000000
#define C2P2ZCoeff_2_A1 1.000000

#define C2P2ZCoeff_2_MAX 0.98
#define C2P2ZCoeff_2_MIN -0.300000

/////////////////////////////////////////���////////////////////////////////////////////////
#define C2P2ZCoeff_1_B2 0.000000                            // 0.0002304
//#define C2P2ZCoeff_1_B2 0.0
#define C2P2ZCoeff_1_B1 -2.64000                            // -0.9004608
//#define C2P2ZCoeff_1_B1 -0.9
#define C2P2ZCoeff_1_B0 2.992000                            // 1.0174179
//#define C2P2ZCoeff_1_B0 0.9

#define C2P2ZCoeff_1_A2 0.000000
#define C2P2ZCoeff_1_A1 1.000000

#define C2P2ZCoeff_1_MAX 0.98
#define C2P2ZCoeff_1_MIN 0.000000
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//#if (INCR_BUILD == 10)
//----------- DIFGITAL CONTROLLER --------------------
	#pragma DATA_SECTION(Coef2P2Z_V, "CNTL_2P2Z_Coef");
	#pragma DATA_SECTION(Coef2P2Z_I, "CNTL_2P2Z_Coef");

	struct	CNTL_2P2Z_CoefStruct Coef2P2Z_V;    //��������� ��������� ������ ���� ������ (��� ������� ����������)
	struct	CNTL_2P2Z_CoefStruct Coef2P2Z_I;	//��������� ��������� ������ ���� ������ (��� ������� ����)

///////////////////////////////////////////DPLIB Net Terminals///////////////////////////////////////
////////////////////////////////////////ADCDRV_1ch Net Terminals//////////////////////////////////////
	extern volatile long *ADCDRV_1ch_Rlt0;
	extern volatile long *ADCDRV_1ch_Rlt1;
	extern volatile long *ADCDRV_1ch_Rlt2;
	///////////////////////////////////////////2p2z Net Terminals//////////////////////////////////////
	extern volatile int32 *CNTL_2P2Z_RefV;	// ��������� �� ���������� V_Ref
	extern volatile int32 *CNTL_2P2Z_FdbkV;	// ��������� �� ���������� V_Fdb
	extern volatile int32 *CNTL_2P2Z_OutV;	// ��������� �� ���������� V_Out
	extern volatile int32 *CNTL_2P2Z_CoefV;	// ��������� �� ������ �������������

	extern volatile int32 *CNTL_2P2Z_RefI;	// ��������� �� ���������� I_Ref
	extern volatile int32 *CNTL_2P2Z_FdbkI;	// ��������� �� ���������� I_Fdb
	extern volatile int32 *CNTL_2P2Z_OutI;	// ��������� �� ���������� I_Out
	extern volatile int32 *CNTL_2P2Z_CoefI;	// ��������� �� ������ �������������
//#endif
/////////////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////PWMDRV_1ch_UpDwnCnt Variables//////////////////////////////////////////
extern volatile long *PWMDRV_1ch_UpDwnCnt_Duty6;	// instance #6
extern volatile long PWMDRV_1ch_UpDwnCnt_Period6;

extern volatile long *PWMDRV_1ch_UpDwnCnt_Duty3;	// instance #3
extern volatile long PWMDRV_1ch_UpDwnCnt_Period3;
/////////////////////////////////////////////////////////////////////PFC/////////////////////////
/////////////////////////////////////////////PFC///////////////////////////////////////////


extern volatile long *ADCDRV_1ch_Rlt3;	// instance #1, IpfcA
extern volatile long *ADCDRV_1ch_Rlt4;	// instance #2, Vbus
extern volatile long *ADCDRV_1ch_Rlt5;	// instance #4, VL_fb
extern volatile long *ADCDRV_1ch_Rlt6;	// instance #5, VN_fb

extern volatile long *PWMDRV_1ch_UpDwnCnt_Duty2;	// instance #2
extern volatile long PWMDRV_1ch_UpDwnCnt_Period2;

extern volatile long *PWMDRV_1ch_UpDwnCnt_Duty1;	// instance #1
extern volatile long PWMDRV_1ch_UpDwnCnt_Period1;

extern volatile int32 *CNTL_2P2Z_Ref1;	// ��������� �� ���������� I_Ref
extern volatile int32 *CNTL_2P2Z_Fdbk1;	// ��������� �� ���������� I_Fdb
extern volatile int32 *CNTL_2P2Z_Out1;	// ��������� �� ���������� I_Out
extern volatile int32 *CNTL_2P2Z_Coef1;	// ��������� �� ������ �������������

extern volatile int32 *CNTL_2P2Z_Ref2;  // ��������� �� ���������� Vbus_set
extern volatile int32 *CNTL_2P2Z_Fdbk2; // ��������� �� ���������� Vbus
extern volatile int32 *CNTL_2P2Z_Out2;  // ��������� �� ���������� VbusVcmd
extern volatile int32 *CNTL_2P2Z_Coef2; // ��������� �� ������ �������������

extern volatile int32  *MATH_EMAVG_In1;
extern volatile int32  *MATH_EMAVG_Out1;
extern volatile long MATH_EMAVG_Multiplier1;

extern volatile int32  *MATH_EMAVG_In2;
extern volatile int32  *MATH_EMAVG_Out2;
extern volatile long MATH_EMAVG_Multiplier2;

extern volatile int32  *MATH_EMAVG_In3;
extern volatile int32  *MATH_EMAVG_Out3;
extern volatile long MATH_EMAVG_Multiplier3;

extern volatile int32  *MATH_EMAVG_In4;
extern volatile int32  *MATH_EMAVG_Out4;
extern volatile long MATH_EMAVG_Multiplier4;

extern volatile long *PFC_ICMD_Vcmd1;
extern volatile long *PFC_ICMD_VinvSqr1;
extern volatile long *PFC_ICMD_VacRect1;
extern volatile long *PFC_ICMD_Out1;
extern volatile long PFC_ICMD_VmaxOverVmin1;
//extern volatile long *PFC_ICMD_Vpfc1;
//extern volatile long *PFC_ICMD_Duty1;
//extern volatile long PFC_ICMD_VmaxOverVmin;

//PFC_InvRmsSqr - instance #1
extern volatile long *PFC_InvRmsSqr_In1;
extern volatile long *PFC_InvRmsSqr_Out1;
extern volatile long PFC_InvRmsSqr_VminOverVmax1;
extern volatile long PFC_InvRmsSqr_Vmin1;

////////////////////////////////////////////SFRA/////////////////////////////////////////////////



////////////////////////////////////////////////////////////////////////////////////////////////

#endif /* SINETABLE_50HZ_H_ */
