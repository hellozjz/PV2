/* ****************************************************************************************************
 * File name: main.c		//	for DC/DC converter: Battery and PV converter
 * Written by: Nguyen Xuan Bac
 * Edited by: Zhang Zhe
 * Last updated: 04 Jan 2017													  				  *
 *******************************************************************************************************/
#include "DSP28x_Project.h"
#include "IQmathlib.h"
#include "stdio.h"
#include "string.h"
#include "user_define.h"
#include "CanBus.h"

// Prototype statements for functions found within this file.
interrupt void cpu_timer0_isr(void);
interrupt void  adc_isr(void);
void InitEPwm_Interleaved(void);
void Scan_button(void);
void Turn_off_Converter(void);
void BAT_Send_Data_Canbus(int16 d0, int16 d1, int16 d2, int16 group_data_index);
void PV_Send_Data_Canbus(int16 d0_PV, int16 d1_PV, int16 d2_PV, int16 group_data_index);
void BAT_Send_To_BBB(void);
void PV_Send_To_BBB(void);
void Receive_Data_Canbus(void);
void Protection(void);
void Soft_Transition(void);
void BAT_Transition_Mode(void);
void PV1_Transition_Mode(void);
void PV2_Transition_Mode(void);
void PV3_Transition_Mode(void);
void ADC_Calculation(void);
void MPPT_PO(void);
void PI_Controller(void);
void PWM_Modulation(void);
float LowPassFilter(float32 PreOut, float32 Input, float32 CutFre);

//===========================================================================================================
// define constant
#define cycle_time 	100				// interrupt timer 0  100us
#define Ts 			0.0001			// Descretized period
#define ADC_MODCLK 	0x0003 			// HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 150/(2*3)   = 25.0 MHz
#define ADC_CKPS   	0x1   			// ADC module clock = HSPCLK/2*ADC_CKPS   = 25.0MHz/(1*2) = 12.5MHz
#define ADC_SHCLK  	0x6   			// S/H width in ADC module periods                        = 16 ADC clocks
#define deadtime	150
#define	period		1875//375				//	375 <=>100kHz, 750 <=> 50kHz
#define phase_b		1250//250
#define phase_c		1250//250
//====================== ADC Calibration ==================================================================
#define offset_a0 	2063	//ADC_A1	ADCINA1		I3  Iw
#define offset_a1 	2056	//ADC_A2	ADCINA2		V4	vDC
#define offset_a2 	2047	//ADC_A3	ADCINA3		V3	Vw
#define offset_a3 	2057	//ADC_A4	ADCINA4		I2	Iv
#define offset_a4 	2068	//ADC_A5	ADCINA5		I1	Iu
#define offset_a5 	2054	//ADC_A7	ADCINA7		V1	Vu
#define offset_b6 	2041	//ADC_A6	ADCINA6		V2	Vv
#define offset_b7 	2066	//ADC_B7	ADCINB7		I4	iDC

#define gain_a0		0.0553	//ADC_A1	ADCINA1		I3	Iw
#define gain_a1	    0.5634	//ADC_A2	ADCINA2		V4	vDC
#define gain_a2	    0.5582	//ADC_A3	ADCINA3		V3	Vw
#define gain_a3	    0.0974	//ADC_A4	ADCINA4		I2	Iv
#define gain_a4	    0.0968	//ADC_A5	ADCINA5		I1	Iu
#define gain_a5	    0.5630	//ADC_A7	ADCINA7		V1	Vu
#define gain_b6	    0.5597	//ADC_A6	ADCINA6		V2	Vv
#define gain_b7	    0.0974	//ADC_B7	ADCINB7		I4	iDC


// LEVEL5-PV
#define offset_a0_pv 	2063//2042//2043//	2062		//ADC_A1	ADCINA1		I3, Iw		2043
#define offset_a1_pv 	2056//2037//2042//	2063		//ADC_A2	ADCINA2		V4	vDC		2072
#define offset_a2_pv 	2047//2036//2038//	2042		//ADC_A3	ADCINA3		V3	Vw		2037
#define offset_a3_pv 	2057//2047//2044//	2061		//ADC_A4	ADCINA4		I2	Iv		2044
#define offset_a4_pv 	2068//2047//2044//	2062		//ADC_A5	ADCINA5		I1	Iu		2044
#define offset_a5_pv 	2054//2031//2033//	2055		//ADC_A7	ADCINA7		V1	Vu		2058
#define offset_b6_pv 	2041//2037//2035//	2059		//ADC_A6	ADCINA6		V2	Vv		2035
#define offset_b7_pv 	2066//2046//2043//	2065		//ADC_B7	ADCINB7		I4	iDC		2043

#define gain_a0_pv		0.0553//-0.1896	//ADC_A1	ADCINA1		I3
#define gain_a1_pv	    0.5634//0.3421	//ADC_A2	ADCINA2		V4
#define gain_a2_pv	    0.5582//0.3379	//ADC_A3	ADCINA3		V3
#define gain_a3_pv	    0.0974//-0.0971	//ADC_A4	ADCINA4		I2
#define gain_a4_pv	    0.0968//0.0938//-0.1859	//ADC_A5	ADCINA5		I1
#define gain_a5_pv	    0.5630//0.3457	//ADC_A7	ADCINA7		V1
#define gain_b6_pv	    0.5597//0.3502	//ADC_A6	ADCINA6		V2
#define gain_b7_pv	    0.0974//0.0523//-0.051	//ADC_B7	ADCINB7		I4


#define	epsilon			0.2					//delta_vref
#define	deltaP			0.2					//threshold of PV power MPPT mode
#define MPPT_ADC_COUNT	300

#define vPV_min 		200
#define vPV_max 		300
#define PRECHARGING_DELAY  20000


#define	BAT_CONVERTER 	1
#define	PV_CONVERTER 	2

#define VRM				0
#define	PRM				1
#define	MPPT			2
#define	IDLE			3
#define	ISOLATED		4


// Test
#define	IMMEDIATE_STOP_DC_VOL		450
#define OFF_THRESHOLD_OVER_DC_VOL	420			//BAT shutdown
#define PV_THRESHOLD_OVER_DC_VOL	400			//PV shutdown
#define BAT_THRESHOLD_OVER_DC_VOL	395			//BAT auto transition
#define	BAT_THRESHOLD_LOW_DC_VOL	3//65			//BAT auto transition
#define	OFF_THRESHOLD_LOW_DC_VOL	3//40			//PV and BAT shutdown

#define i_sat 						30				// maximum input current of inner controller
#define MAX_DUTY					1875
#define MIN_DUTY					0
#define MAX_DUTY_PV_A				1400
#define MIN_DUTY_PV_A				500

#define	PROTECT_COUNT		10
#define	TRIP_COUNT			5

#define LED_BLINKING_COUNT		5000

//======================================= Variables ===========================================================================

//Test
float MIN_INPUT_VOL = 1;//00;	//				150
float MAX_INPUT_VOL = 330;	//				330
float MAX_INPUT_CUR = 40;	//				12
float MAX_OUTPUT_CUR = 30;	//				15
Uint16	converter = BAT_CONVERTER;	// BAT converter
float 	kpv_A = 1;
float 	kiv_A = 100;
float 	kpi_A = 1;
float 	kii_A = 100;

float 	kpv_A1 = 1;
float 	kiv_A1 = 100;
float 	kpi_A1 = 1;
float 	kii_A1 = 100;

float 	kpv_B = 1;
float 	kiv_B = 100;
float 	kpi_B = 1;
float 	kii_B = 100;

float 	kpv_C = 1;
float 	kiv_C = 100;
float 	kpi_C = 1;
float 	kii_C = 100;

float 	duty_max = period, duty_max_A = period, duty_max_B = period, duty_max_C = period; 						//normal mode
float 	duty_min = 0, duty_min_A = 0, duty_min_B = 0, duty_min_C = 0;
float 	duty_max_reset = 0, duty_max_reset_A = 0, duty_max_reset_B = 0, duty_max_reset_C = 0;
float 	duty_min_set = 0, duty_min_set_A = 0, duty_min_set_B = 0, duty_min_set_C = 0;
float	duty_step_plus = 0.005;
float	duty_step_minus	= 0.005;
float	duty_step_pv    = 0.02;

float 	vref_A = 0, vref_B = 0, vref_C = 0;									//reference output voltage
//float 	vref_A_pre = 0, vref_B_pre = 0, vref_C_pre = 0;
float 	i_set = 0, i_set_A = 0, i_set_B = 0, i_set_C = 0;

//float 	ei = 0, ei_pre = 0;
float 	ei_A = 0, ei_B = 0, ei_C = 0, ei_A_pre = 0, ei_B_pre = 0, ei_C_pre = 0;
//float 	ev = 0, ev_pre = 0;
float 	ev_A = 0, ev_B = 0, ev_C = 0, ev_A_pre = 0, ev_B_pre = 0, ev_C_pre = 0;

//float 	iref = 0, iref_pre = 0;					// inner controller variables
float 	iref_A = 0, iref_B = 0, iref_C = 0;
float 	iref_A_pre = 0, iref_B_pre = 0, iref_C_pre = 0;						// inner controller variables

//float 	duty = 0, duty_pre = 0;
float 	duty_A = 0, duty_B = 0, duty_C = 0, duty_A_pre = 0, duty_B_pre = 0, duty_C_pre = 0;

float 	duty_HA = 0, duty_LA = period, duty_HB = 0, duty_LB = period, duty_HC = 0, duty_LC = period;

// ADC variables
long 	adc_a0 = 0, adc_a1 = 0, adc_a2 = 0, adc_a3 = 0, adc_a4 = 0, adc_a5 = 0, adc_b6 = 0, adc_b7 = 0;

float 	iiA = 0, iiA_sum = 0, iiA_ave = 0;
float 	iiB = 0, iiB_sum = 0, iiB_ave = 0;
float 	iiC = 0, iiC_sum = 0, iiC_ave = 0;

float 	viA = 0, viA_sum = 0, viA_ave = 0;
float 	viB = 0, viB_sum = 0, viB_ave = 0;
float 	viC = 0, viC_sum = 0, viC_ave = 0;

float 	vo = 0, vo_sum  = 0, vo_ave  = 0;
float 	io = 0, io_sum  = 0, io_ave  = 0;
float 	i_in_ave = 0;

float	iA_BBB=0;		// ---->for low pass filter send to GUI
float	iB_BBB=0;
float	iC_BBB=0;


Uint16  ConversionCount = 0;
float 	ConversionCount1;
float	 	PowerPV_A = 0, PowerPV_B = 0, PowerPV_C = 0;
float	 	PowerPV_A_pre = 0, PowerPV_B_pre = 0, PowerPV_C_pre = 0;
float		viA_ave_pre = 0, viB_ave_pre = 0, viC_ave_pre = 0;
float		vref_A_pre = 0, vref_B_pre = 0, vref_C_pre = 0;
float		ave_viA = 0, ave_viB = 0, ave_viC = 0;
float		ave_iiA = 0, ave_iiB = 0, ave_iiC = 0;
float		sum_viA = 0, sum_viB = 0, sum_viC = 0;
float		sum_iiA = 0, sum_iiB = 0, sum_iiC = 0;
float		ave_viA_pre = 0, ave_viB_pre = 0, ave_viC_pre = 0;
float		ave_iiA_pre = 0, ave_iiB_pre = 0, ave_iiC_pre = 0;
float		epsilon_A = 0.4, epsilon_B = 0.4, epsilon_C = 0.4;
Uint16		k;
Uint16		update_pv = 0;
int		mode = ISOLATED, mode_A = ISOLATED, mode_B = ISOLATED, mode_C = ISOLATED;
int		new_mode = 0, new_mode_A = 0, new_mode_B = 0, new_mode_C = 0;
Uint16	mode_change = 0, mode_change_counter = 0;
float	reference_vol = 380, reference_vol_A = 380, reference_vol_B = 380, reference_vol_C = 380;
float	reference_vol_new = 380, reference_vol_new_A = 380, reference_vol_new_B = 380, reference_vol_new_C = 380;
float	droop_coef = 0, droop_coef_A = 0, droop_coef_B = 0, droop_coef_C = 0;
float	reference_power = 0, reference_power_A = 0, reference_power_B = 0, reference_power_C = 0;
float	reference_power_new = 0, reference_power_new_A = 0, reference_power_new_B = 0, reference_power_new_C = 0;
Uint16	ecan_count = 0;
Uint16 	ecan_receive_done = 0, ecan_request = 0, ecan_send_done = 1, ecan_error = 0;
Uint16  ecan_send_done_PV = 0;
int16	d_0, d_1, d_2;
int16	d_0_PV, d_1_PV, d_2_PV;
Uint16	message_7 = 0, message_16 = 0;

Uint16 	CONVERTER_STATUS = 0;
Uint16	CH1_ONOFF_STATUS = 0, CH2_ONOFF_STATUS = 0, CH3_ONOFF_STATUS = 0;
Uint16	INPUT_RELAY_STATUS = 0, OUTPUT_RELAY_STATUS = 0;
Uint16	PV1_PRIOR = 1, PV2_PRIOR = 1, PV3_PRIOR = 1;

Uint16 	fault_message1 = 0, fault_message2 = 0;

float 	heatsink_temp = 0;
Uint16	heatsink_temp_int;
int16	iiA_int, iiB_int, iiC_int, io_int;
int16	viA_int, viB_int, viC_int, vo_int;
int16	bat_vol_int, dc_bus_vol_int;
Uint16	ref_vol_confirm_int, ref_vol_confirm_A_int, ref_vol_confirm_B_int, ref_vol_confirm_C_int;
Uint16	droop_coef_confirm_int, droop_coef_confirm_A_int, droop_coef_confirm_B_int, droop_coef_confirm_C_int;
int16	ref_power_confirm_int, ref_power_confirm_A_int, ref_power_confirm_B_int, ref_power_confirm_C_int;
Uint16	reserve = 0;
Uint16 	START_BUTTON;
Uint16 	STOP_BUTTON;
Uint16 	stop = 0, stop_A = 0, stop_B = 0, stop_C = 0;
Uint16 	st = 0;
Uint16  delay_set = 0;
Uint32  delay_counter = 0;

Uint16 	short_circuit = 0;
Uint16 	short_sw = 0;

Uint16	over_vol_DC = 0, over_vol_DC_counter = 0;
Uint16	PV_over_vol_DC = 0, PV_over_vol_DC_counter = 0;
Uint16	BAT_over_vol_DC = 0, BAT_over_vol_DC_counter = 0;
Uint16	BAT_low_vol_DC	= 0, BAT_low_vol_DC_counter  = 0;
Uint16	low_vol_DC	= 0, low_vol_DC_counter	= 0;

Uint16	over_vol_A = 0, over_vol_A_counter = 0;
Uint16	over_vol_B = 0, over_vol_B_counter = 0;
Uint16	over_vol_C = 0, over_vol_C_counter = 0;

Uint16	low_vol_A = 0, low_vol_A_counter = 0;
Uint16	low_vol_B = 0, low_vol_B_counter = 0;
Uint16	low_vol_C = 0, low_vol_C_counter = 0;

int16	over_cur_A	= 0, over_cur_A_counter = 0;
int16	over_cur_B	= 0, over_cur_B_counter = 0;
int16	over_cur_C	= 0, over_cur_C_counter = 0;
int16	over_cur_DC = 0, over_cur_DC_counter = 0;


// Testing
Uint32 	time2change = 0;
Uint16	test = 0, test_duty = 0, test1 = 0;
float32	testing = 0;

Uint32  led_count = 0; 					//For LED Blinking
Uint32  ecan_guard_count = 0;
Uint32	ecan_error_count = 0;

extern Uint32 device_id;
extern int variable_index;

// flash setup
// These are defined by the linker (see F28335.cmd)
extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;
extern Uint16 RamfuncsLoadSize;

//********************************************* MAIN Function ****************************************************
main() {

	InitSysCtrl();// Initialize System Control: PLL, WatchDog, enable Peripheral Clocks

// Specific clock setting for ADC:
	EALLOW;
	SysCtrlRegs.HISPCP.all = ADC_MODCLK;	// HSPCLK = SYSCLKOUT/ADC_MODCLK
	EDIS;

//flash setup
	memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (Uint32) &RamfuncsLoadSize);
	InitFlash();
	InitGpio();								// Initialize I/O port

//Clear all interrupts and initialize PIE vector table
	DINT;
	// Disable CPU interrupts
	InitPieCtrl();// Initialize the PIE control registers to their default state
	IER = 0x0000;							// Disable CPU interrupts
	IFR = 0x0000;							// Clear all CPU interrupt flags
	InitPieVectTable();	// Initialize the PIE vector table with pointers to the shell Interrupt Service Routines (ISR).

	EALLOW;
	PieVectTable.TINT0 = &cpu_timer0_isr;
	EDIS;

	InitCpuTimers();						// Initialize the CPU timer
	InitEPwm_Interleaved();

    EALLOW;  // This is needed to write to EALLOW protected register
    PieVectTable.ADCINT = &adc_isr;																								//===========
    EDIS;    // This is needed to disable write to EALLOW protected registers

    InitAdc();								// Initialize ADC module
	ConfigCpuTimer(&CpuTimer0, 150, cycle_time);

// Start timer 0
	CpuTimer0Regs.TCR.all = 0x4001; // Use write-only instruction to set TSS bit = 0
	IER |= M_INT1;									// Enable interrupt timer 0
	PieCtrlRegs.PIEIER1.bit.INTx6 = 1; 				// Enable ADCINT in PIE														//============
	PieCtrlRegs.PIEIER1.bit.INTx7 = 1;// Enable TINT0 in the PIE: Group 1 interrupt 7
	EINT;
	// Enable Global interrupt INTM
	ERTM;

	configureEcanB();

	OFF_ALL_RELAY();
	Turn_off_Converter();
	stop = 1;
//  testing
    converter = 2;

// One time message to BBB
	if (converter == BAT_CONVERTER)
	{
	    BAT_Send_Data_Canbus(MAX_INPUT_CUR, MAX_OUTPUT_CUR, BAT_THRESHOLD_OVER_DC_VOL, BAT_MESSAGE_101_INDEX);
	    BAT_Send_Data_Canbus(MIN_INPUT_VOL, OFF_THRESHOLD_OVER_DC_VOL, OFF_THRESHOLD_LOW_DC_VOL, BAT_MESSAGE_102_INDEX);
	}
	else
	{
	    ;
	}



// forever loop
for (;;) {

//	Keep sending message to BBB 10ms once

	if (led_count > LED_BLINKING_COUNT)
	{
		GpioDataRegs.GPATOGGLE.bit.GPIO15 = 1 ;
		led_count = 0;
	}

		if (ecan_count >= 10000)
		{
			ecan_count 	= 0;
			if (converter == BAT_CONVERTER)
			{
				BAT_Send_To_BBB();
			}
			else // converter == PV_CONVERTER
			{
				PV_Send_To_BBB();
			}
		}
}
}
// Interrupt Timer 0*******************************************************************************
interrupt void cpu_timer0_isr(void) {
	time2change++;
	ecan_count++;								// period to send ecan: 100 x 100us = 10ms
	led_count++;
	ecan_guard_count++;

//	if (mode_change == 1)		mode_change_counter++;
	if (delay_set == 1) delay_counter++;
	ADC_Calculation();
	Receive_Data_Canbus();

	testing = LowPassFilter(testing, reference_power, 0.5);

	Scan_button();
	Protection();

	if (converter == BAT_CONVERTER)
	{
	    if ((stop == 1) || (new_mode == ISOLATED))  Turn_off_Converter();
	}
	else
	{
	    if ((stop_A == 1) && (stop_B == 1) && (stop_C == 1))
	    {
	    	stop = 1;
	    	Turn_off_Converter();
	    }
	    if ((new_mode_A == ISOLATED) || (new_mode_B == ISOLATED) || (new_mode_C == ISOLATED))  Turn_off_Converter();
	}

	Soft_Transition();
	if (converter == PV_CONVERTER)	MPPT_PO();
	PI_Controller();
	PWM_Modulation();

//	duty_HA=900;
//	duty_LA=900;
//	duty_HB=900;
//	duty_LB=900;
//	duty_HC=900;
//	duty_LC=900;

	/* Update duty cycle */
	 	EPwm1Regs.CMPA.half.CMPA 	= duty_HA; 			// adjust duty for output EPWM1A; // max = 375 <=> pwm = 1 (switch on)
	 	EPwm1Regs.CMPB 				= duty_LA;			// min = 375 <=> pwm = 0 (switch off)

	 	EPwm2Regs.CMPA.half.CMPA 	= duty_HB;
	 	EPwm2Regs.CMPB 				= duty_LB;
	 	EPwm3Regs.CMPA.half.CMPA 	= duty_HC;
	 	EPwm3Regs.CMPB 				= duty_LC;

	 	PieCtrlRegs.PIEACK.all 		= PIEACK_GROUP1;
}
void InitEPwm_Interleaved() {
	//=====================================================================
	// Config
	// Initialization Time
	//========================
	// EPWM Module 1 config
	EPwm1Regs.TBPRD = period; // Period = 1500 TBCLK counts = 1500*(1/150Mhz) = 10us => 100kHz
	EPwm1Regs.TBPHS.half.TBPHS = 0; // Set Phase register to zero
	EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
	EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE; // Master module
	EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;
	EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_CTR_ZERO; // Sync down-stream module
	EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
	EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
	EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
	EPwm1Regs.AQCTLA.bit.CAU = AQ_SET; // set actions for EPWM1A
	EPwm1Regs.AQCTLA.bit.CAD = AQ_CLEAR;
	EPwm1Regs.AQCTLB.bit.CBU = AQ_CLEAR;	// set actions for EPWM1B
	EPwm1Regs.AQCTLB.bit.CBD = AQ_SET;
		// EPWM Module 2 config
		EPwm2Regs.TBPRD = period; // Period = 1500 TBCLK counts
		EPwm2Regs.TBPHS.half.TBPHS = phase_b; // Phase = 500/1500 * 360 = 120 deg
		EPwm2Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
		EPwm2Regs.TBCTL.bit.PHSEN = TB_ENABLE; // Slave module
		EPwm2Regs.TBCTL.bit.PHSDIR = TB_DOWN; // Count DOWN on sync (=120 deg)
		EPwm2Regs.TBCTL.bit.PRDLD = TB_SHADOW;
		EPwm2Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through
		EPwm2Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
		EPwm2Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
		EPwm2Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
		EPwm2Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
		EPwm2Regs.AQCTLA.bit.CAU = AQ_SET; // set actions for EPWM2A
		EPwm2Regs.AQCTLA.bit.CAD = AQ_CLEAR;

		EPwm2Regs.AQCTLB.bit.CBU = AQ_CLEAR;	// set actions for EPWM1B
		EPwm2Regs.AQCTLB.bit.CBD = AQ_SET;
		// EPWM Module 3 config
		EPwm3Regs.TBPRD = period; // Period = 900 TBCLK counts
		EPwm3Regs.TBPHS.half.TBPHS = phase_c; // Phase = 500/1500 * 360 = 120 deg
		EPwm3Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
		EPwm3Regs.TBCTL.bit.PHSEN = TB_ENABLE; // Slave module
		EPwm3Regs.TBCTL.bit.PHSDIR = TB_UP; // Count UP on sync (=240 deg)
		EPwm3Regs.TBCTL.bit.PRDLD = TB_SHADOW;
		EPwm3Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // sync flow-through
		EPwm3Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
		EPwm3Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
		EPwm3Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
		EPwm3Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
		EPwm3Regs.AQCTLA.bit.CAU = AQ_SET; // set actions for EPWM3Ai
		EPwm3Regs.AQCTLA.bit.CAD = AQ_CLEAR;

		EPwm3Regs.AQCTLB.bit.CBU = AQ_CLEAR;	// set actions for EPWM1B
		EPwm3Regs.AQCTLB.bit.CBD = AQ_SET;

		// EPWM Module 4 config---ADC interrupt timer
			EPwm4Regs.TBPRD = period*0.2; // Period = 1500 TBCLK counts = 1500*(1/150Mhz) = 10us => 100kHz
			EPwm4Regs.TBPHS.half.TBPHS = 0; // Set Phase register to zero
			EPwm4Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; // Symmetrical mode
			EPwm4Regs.TBCTL.bit.PHSEN = TB_ENABLE; // Master module
			EPwm4Regs.TBCTL.bit.PRDLD = TB_SHADOW;
			EPwm4Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_IN; // Sync down-stream module
			EPwm4Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
			EPwm4Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;
			EPwm4Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO; // load on CTR=Zero
			EPwm4Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO; // load on CTR=Zero
			EPwm4Regs.AQCTLA.bit.CAU = AQ_SET; // set actions for EPWM1A
			EPwm4Regs.AQCTLA.bit.CAD = AQ_CLEAR;
			EPwm4Regs.AQCTLB.bit.CBU = AQ_CLEAR;	// set actions for EPWM1B
			EPwm4Regs.AQCTLB.bit.CBD = AQ_SET;
}
void Scan_button() {
	START_BUTTON = GpioDataRegs.GPBDAT.bit.GPIO61;// Scan Start button, press => START_BUTTON = 0 (nhan start) => Precharge
	STOP_BUTTON = GpioDataRegs.GPBDAT.bit.GPIO60;	// Scan Stop button

		if ((START_BUTTON == 0) && (STOP_BUTTON == 1))
		{
			stop = 0;
			stop_A = 0;
			stop_B = 0;
			stop_C = 0;
			fault_message1 = 0;
			fault_message2 = 0;

		}
		else if (STOP_BUTTON == 0)
			{
			stop = 1;
			stop_A = 1;
			stop_B = 1;
			stop_C = 1;

			//CTO
//			GpioDataRegs.GPBCLEAR.bit.GPIO55 = 1;
//			GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;
//			GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;
			}
}
//===================================================================================
void Turn_off_Converter() {
	DISABLE_ALL_PWM();
	OFF_ALL_RELAY();
	OPEN_RELAY_DC();
	OPEN_RELAY_BAT();
	OPEN_RELAY_INPUT();
	OPEN_RELAY_OUTPUT();
//	GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;
//	GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;

	CONVERTER_STATUS = 0;

	INPUT_RELAY_STATUS = 0;
	OUTPUT_RELAY_STATUS = 0;

	LED_RUN_OFF();
	LED_STOP_ON();

	CH1_ONOFF_STATUS = 0;
	CH2_ONOFF_STATUS = 0;
	CH3_ONOFF_STATUS = 0;

	duty_max_reset = 0;
	duty_min_set = 0;
	duty_max = MAX_DUTY;
	duty_min = MIN_DUTY;
	mode = ISOLATED;


	short_sw = 0;
	over_vol_DC_counter = 0;
	PV_over_vol_DC_counter = 0;
	BAT_over_vol_DC_counter = 0;
	BAT_low_vol_DC_counter  = 0;
	low_vol_DC_counter	= 0;

	over_vol_A_counter = 0;
	over_vol_B_counter = 0;
	over_vol_C_counter = 0;

	low_vol_A_counter = 0;
	low_vol_B_counter = 0;
	low_vol_C_counter = 0;

	over_cur_A_counter = 0;
	over_cur_B_counter = 0;
	over_cur_C_counter = 0;
	over_cur_DC_counter = 0;
}

void BAT_Send_Data_Canbus(int16 d0, int16 d1, int16 d2, int16 group_data_index)
{
	struct CAN_DATA can_data_to_send_1;
	ecan_send_done = 0;
	can_data_to_send_1.data0 = d0;
	can_data_to_send_1.data1 = d1;
	can_data_to_send_1.data2 = d2;

	can_data_to_send_1.index = group_data_index;

	send_data(BAT_ID_INDEX, can_data_to_send_1);

	ecan_send_done = 1;
}
void PV_Send_Data_Canbus(int16 d0_PV, int16 d1_PV, int16 d2_PV, int16 group_data_index)
{
	struct CAN_DATA can_data_to_send_1;
	ecan_send_done_PV = 0;
	can_data_to_send_1.data0 = d0_PV;
	can_data_to_send_1.data1 = d1_PV;
	can_data_to_send_1.data2 = d2_PV;

	can_data_to_send_1.index = group_data_index;

	send_data(PV_ID_INDEX, can_data_to_send_1);

	ecan_send_done_PV = 1;
}
void BAT_Send_To_BBB(){
			iiA_int		= (int16) (iiA_ave*10);
			iiB_int		= (int16) (iiB_ave*10);
			iiC_int		= (int16) (iiC_ave*10);
			BAT_Send_Data_Canbus(iiA_int, iiB_int, iiC_int, BAT_MESSAGE_1_INDEX);
			io_int					= (int16) (io_ave*10);
			ref_vol_confirm_int		= (Uint16) (reference_vol*10);
			droop_coef_confirm_int	= (Uint16) (droop_coef*10);
			BAT_Send_Data_Canbus(io_int, ref_vol_confirm_int, droop_coef_confirm_int, BAT_MESSAGE_2_INDEX);
			if (CH1_ONOFF_STATUS != 0)		message_7 |= 0x0001;
			else 							message_7 &= 0xFFFE;
			if (CH2_ONOFF_STATUS != 0)		message_7 |= 0x0002;
			else 							message_7 &= 0xFFFD;
			if (CH3_ONOFF_STATUS != 0)		message_7 |= 0x0004;
			else 							message_7 &= 0xFFFB;
			if (INPUT_RELAY_STATUS != 0)	message_7 |= 0x0008;
			else 							message_7 &= 0xFFF7;
			if (OUTPUT_RELAY_STATUS != 0)	message_7 |= 0x0010;
			else 							message_7 &= 0xFFEF;

			switch (mode)
			{
			case VRM:
			{
				message_7 &= 0b1111111000111111;
				break;
			}
			case PRM:
			{
				message_7 |= 0b0000000001000000;
				message_7 &= 0b1111111001111111;
				break;
			}
			case MPPT:
			{
				message_7 |= 0b0000000010000000;
				message_7 &= 0b1111111010111111;
				break;
			}
			case IDLE:
			{
				message_7 |= 0b0000000011000000;
                message_7 &= 0b1111111011111111;
				break;
			}
			case ISOLATED:
			{
                message_7 |= 0b0000000100000000;
                message_7 &= 0b1111111100111111;

                break;
			}
			}

			heatsink_temp_int = (Uint16) (heatsink_temp*10);

			BAT_Send_Data_Canbus(message_7, reserve, heatsink_temp_int, BAT_MESSAGE_3_INDEX);

			ref_power_confirm_int		= (int16) (reference_power*10);
			bat_vol_int					= (int16) (viA_ave*10);
			dc_bus_vol_int				= (int16) (vo_ave*10);
			BAT_Send_Data_Canbus(ref_power_confirm_int, bat_vol_int, dc_bus_vol_int, BAT_MESSAGE_4_INDEX);
			BAT_Send_Data_Canbus(fault_message1, fault_message2, reserve, BAT_MESSAGE_5_INDEX);
}
void PV_Send_To_BBB(){
			iiA_int		= (int16) (iA_BBB*10);
			iiB_int		= (int16) (iB_BBB*10);
			iiC_int		= (int16) (iC_BBB*10);
			PV_Send_Data_Canbus(iiA_int, iiB_int, iiC_int, PV_MESSAGE_1_INDEX);
			viA_int		= (int16) (viA_ave*10);
			viB_int		= (int16) (viB_ave*10);
			viC_int		= (int16) (viC_ave*10);


			PV_Send_Data_Canbus(viA_int, viB_int, viC_int, PV_MESSAGE_2_INDEX);
			ref_vol_confirm_A_int		= (int16) (reference_vol_A*10);
			ref_power_confirm_A_int		= (int16) (reference_power_A*10);
			droop_coef_confirm_A_int	= (Uint16) (droop_coef_A*10);

			if (mode_A == VRM)			ref_power_confirm_A_int = droop_coef_confirm_A_int;
			PV_Send_Data_Canbus(mode_A, ref_vol_confirm_A_int, ref_power_confirm_A_int, PV_MESSAGE_3_INDEX);
			ref_vol_confirm_B_int		= (int16) (reference_vol_B*10);
			ref_power_confirm_B_int		= (int16) (reference_power_B*10);
			droop_coef_confirm_B_int	= (Uint16) (droop_coef_B*10);
			if (mode_B == VRM)			ref_power_confirm_B_int = droop_coef_confirm_B_int;
			PV_Send_Data_Canbus(mode_B, ref_vol_confirm_B_int, ref_power_confirm_B_int, PV_MESSAGE_4_INDEX);
			ref_vol_confirm_C_int		= (int16) (reference_vol_C*10);
			ref_power_confirm_C_int		= (int16) (reference_power_C*10);
			droop_coef_confirm_C_int	= (Uint16) (droop_coef_C*10);

			if (mode_C == VRM)			ref_power_confirm_C_int = droop_coef_confirm_C_int;
			PV_Send_Data_Canbus(mode_C, ref_vol_confirm_C_int, ref_power_confirm_C_int, PV_MESSAGE_5_INDEX);
			if (CH1_ONOFF_STATUS != 0)		message_16 |= 0b0000000000000001;
			else 							message_16 &= 0b1111111111111110;
			if (CH2_ONOFF_STATUS != 0)		message_16 |= 0b0000000000000010;
			else 							message_16 &= 0b1111111111111101;
			if (CH3_ONOFF_STATUS != 0)		message_16 |= 0b0000000000000100;
			else 							message_16 &= 0b1111111111111011;
			if (INPUT_RELAY_STATUS != 0)	message_16 |= 0b0000000000001000;
			else 							message_16 &= 0b1111111111110111;
			if (OUTPUT_RELAY_STATUS != 0)	message_16 |= 0b0000000000010000;
			else 							message_16 &= 0b1111111111101111;

		switch (PV1_PRIOR)
		{
			case 1:
			{
				message_16 |= 0b0000000000100000;
				message_16 &= 0b1111111110111111;
				break;
			}
			case 2:
			{
				message_16 |= 0b0000000001000000;
				message_16 &= 0b1111111111011111;
				break;
			}
			case 3:
			{
				message_16 |= 0b0000000001100000;
				break;
			}
		}

		switch (PV2_PRIOR)
		{
			case 1:
			{
				message_16 |= 0b0000000010000000;
				message_16 &= 0b1111111011111111;
				break;
			}
			case 2:
			{
				message_16 |= 0b0000000100000000;
				message_16 &= 0b1111111101111111;
				break;
			}
			case 3:
			{
				message_16 |= 0b0000000110000000;
				break;
			}
		}

		switch (PV3_PRIOR)
		{
			case 1:
			{
				message_16 |= 0b0000001000000000;
				message_16 &= 0b1111101111111111;
				break;
			}
			case 2:
			{
				message_16 |= 0b0000010000000000;
				message_16 &= 0b1111110111111111;
				break;
			}
			case 3:
			{
				message_16 |= 0b0000011000000000;
				break;
			}
		}

			io_int = (int16) (io_ave*10);
			vo_int = (int16) (vo_ave*10);
			PV_Send_Data_Canbus(message_16, io_int, vo_int, PV_MESSAGE_6_INDEX);

			heatsink_temp_int				= (int16) (heatsink_temp*10);
			PV_Send_Data_Canbus(heatsink_temp_int, reserve, reserve, PV_MESSAGE_7_INDEX);
			PV_Send_Data_Canbus(fault_message1, fault_message2, reserve, PV_MESSAGE_201_INDEX);
}
void Receive_Data_Canbus()
{
	if (new_data)  					// check if new data come, receive new_mode
	{
		ecan_receive_done = 0;
		if (can_data.id == BAT_ID)		//device_id
		{
			switch (can_data.index)										//variable_index
			{

			case BBB_MESSAGE_1_INDEX:									//droop control
			{
                new_mode = can_data.data0;

                reference_vol_new = (float) (can_data.data1);
                reference_vol_new = reference_vol_new * 0.1;

                if (new_mode == VRM)
                {
                    droop_coef = (float) (can_data.data2);
                    droop_coef = droop_coef * 0.1;
                }
                else if (new_mode == PRM)
                {
                    reference_power_new = (float)(can_data.data2);
                    reference_power_new = reference_power_new*0.1;
                }
                else
                {
                    reference_power_new = 0;
                }

                break;
			}

			case BBB_MESSAGE_2_INDEX:
			{
                if (can_data.data0 == 1)
                {
                    new_mode = ISOLATED;
                }
                else if (can_data.data0 == 0)
                {
                    new_mode = IDLE;
                    stop = 0;
                }
					break;
			}

			}
		}	//End if
		if (can_data.id == PV_ID)		//device_id
		{
			switch (can_data.index)
			{
			    case BBB_MESSAGE_1_INDEX:
			    {
					new_mode_A = can_data.data0;

					reference_vol_new_A = (float) (can_data.data1);
					reference_vol_new_A = reference_vol_new_A * 0.1;

					if (new_mode_A == VRM)
					{
						droop_coef_A = (float) (can_data.data2);
						droop_coef_A = droop_coef_A * 0.1;
					}
					else if (new_mode_A == PRM)
					{
						reference_power_new_A = (float)(can_data.data2);
						reference_power_new_A = reference_power_new_A*0.1;
					}
	                else
	                {
	                    reference_power_new_A = 0;
	                }
					break;
			    }

			    case BBB_MESSAGE_2_INDEX:
			    {
                    new_mode_B = can_data.data0;

                    reference_vol_new_B = (float) (can_data.data1);
                    reference_vol_new_B = reference_vol_new_B * 0.1;

                    if (new_mode_B == VRM)
                    {
                        droop_coef_B = (float) (can_data.data2);
                        droop_coef_B = droop_coef_B * 0.1;
                    }
                    else if (new_mode_B == PRM)
                    {
                        reference_power_new_B = (float)(can_data.data2);
                        reference_power_new_B = reference_power_new_B*0.1;
                    }
                    else
                    {
                        reference_power_new_B = 0;
                    }
                    break;
				}

			    case BBB_MESSAGE_3_INDEX:
			    {
                    new_mode_C = can_data.data0;

                    reference_vol_new_C = (float) (can_data.data1);
                    reference_vol_new_C = reference_vol_new_C * 0.1;

                    if (new_mode_C == VRM)
                    {
                        droop_coef_C = (float) (can_data.data2);
                        droop_coef_C = droop_coef_C * 0.1;
                    }
                    else if (new_mode_C == PRM)
                    {
                        reference_power_new_C = (float)(can_data.data2);
                        reference_power_new_C = reference_power_new_C*0.1;
                    }
                    else
                    {
                        reference_power_new_C = 0;
                    }
                    break;
				}

			    case BBB_MESSAGE_4_INDEX:
			    {
			        if (can_data.data0 == 1)
			        {
			            new_mode_A = ISOLATED;
			            new_mode_B = ISOLATED;
			            new_mode_C = ISOLATED;
			        }
			        else if (can_data.data0 == 0)
                    {
                        new_mode_A = IDLE;
                        new_mode_B = IDLE;
                        new_mode_C = IDLE;
                        stop = 0;
                        stop_A = 0;
                        stop_B = 0;
                        stop_C = 0;
                    }

			        break;
			    }

			    case BBB_MESSAGE_102_INDEX:
			    {
			        break;
			    }

                case BBB_MESSAGE_103_INDEX:
                {
                    break;
                }

			}
		}	//End if


//===============================================================================
		new_data = FALSE;
	}	// End if

	ecan_receive_done = 1;
}
void Protection() {

if (stop == 0)
{
	if (vo_ave > IMMEDIATE_STOP_DC_VOL)
	{
		stop = 1;
		Turn_off_Converter();
		fault_message2 |= 0b0000000000001000;	// DC bus EXTREMELY HIGH voltage
		over_vol_DC = 1;
	}
	if (vo_ave > OFF_THRESHOLD_OVER_DC_VOL)
	{
		over_vol_DC_counter++;
		if(over_vol_DC_counter >= TRIP_COUNT)
		{
			stop = 1;
			Turn_off_Converter();
			fault_message2 |= 0b0000000000001000;	// DC bus EXTREMELY HIGH voltage
			over_vol_DC = 1;
			// Send_data_CanBus();
		}
	}
	else
	{
		over_vol_DC_counter = 0;
		over_vol_DC = 0;
	}
if (converter == PV_CONVERTER)
{
	if (vo_ave > PV_THRESHOLD_OVER_DC_VOL)
	{
		PV_over_vol_DC_counter++;
		if(PV_over_vol_DC_counter >= PROTECT_COUNT)		//count
		{
			PV_over_vol_DC = 1;
			if (mode_A == PRM)	new_mode_A = VRM;
			if (mode_B == PRM)	new_mode_B = VRM;
			if (mode_C == PRM)	new_mode_C = VRM;
		}
	}
	else
	{
		PV_over_vol_DC_counter = 0;
		PV_over_vol_DC = 0;
	}
}
if (converter == BAT_CONVERTER)
{
	if (vo_ave > BAT_THRESHOLD_OVER_DC_VOL)
	{
		BAT_over_vol_DC_counter++;
		if(BAT_over_vol_DC_counter >= PROTECT_COUNT)
		{
			BAT_over_vol_DC = 1;
			if (mode == PRM)	new_mode = VRM;
		}
	}
	else
	{
		BAT_over_vol_DC_counter = 0;
		BAT_over_vol_DC = 0;
	}
}
	if (vo_ave < BAT_THRESHOLD_LOW_DC_VOL) //check all
	{
		BAT_low_vol_DC_counter++;
		if(BAT_low_vol_DC_counter >= PROTECT_COUNT)
		{
			BAT_low_vol_DC = 1;
			if (mode == PRM)	new_mode = VRM;
		}
	}

	else
		{
		BAT_low_vol_DC_counter = 0;
		BAT_low_vol_DC = 0;
		}
	if (vo_ave < OFF_THRESHOLD_LOW_DC_VOL)
	{
		low_vol_DC_counter++;
		if(low_vol_DC_counter >= TRIP_COUNT)
		{
			stop = 1;
			Turn_off_Converter();
			fault_message2 |= 0b0000000000010000;
			low_vol_DC = 1;
		}
		else
		{
			low_vol_DC_counter = 0;
			low_vol_DC	= 0;
		}

	}
	if (viA_ave > MAX_INPUT_VOL)
	{
		over_vol_A_counter++;
		if (over_vol_A_counter >= PROTECT_COUNT)
		{
			stop_A = 1;
			fault_message2 |= 0b0000000000000100;
			over_vol_A = 1;
		}
		else
		{
			over_vol_A = 0;
			over_vol_A_counter = 0;
		}
	}
	if (viB_ave > MAX_INPUT_VOL)
	{
		over_vol_B_counter++;
		if (over_vol_B_counter >= PROTECT_COUNT)
		{
			stop_B = 1;
			fault_message2 |= 0b0000000000000100;
			over_vol_B = 1;
		}
		else
		{
			over_vol_B = 0;
			over_vol_B_counter = 0;
		}
	}

	if (viC_ave > MAX_INPUT_VOL)
	{
		over_vol_C_counter++;
		if (over_vol_C_counter >= PROTECT_COUNT)
		{
			stop_C = 1;
			fault_message2 |= 0b0000000000000100;
			over_vol_C = 1;
		}
		else
		{
			over_vol_C = 0;
			over_vol_C_counter = 0;
		}
	}
	if (viA_ave < MIN_INPUT_VOL)
	{
		low_vol_A_counter++;
		if (low_vol_A_counter >= PROTECT_COUNT)
		{
			stop_A = 1;
			fault_message2 |= 0b0000000000000100;
			low_vol_A = 1;
		}
		else
		{
			low_vol_A = 0;
			low_vol_A_counter = 0;
		}
	}
	if (viB_ave < MIN_INPUT_VOL)
	{
		low_vol_B_counter++;
		if (low_vol_B_counter >= PROTECT_COUNT)
		{
			stop_B = 1;
			fault_message2 |= 0b0000000000000100;
			low_vol_B = 1;
		}
		else
		{
			low_vol_B = 0;
			low_vol_B_counter = 0;
		}
	}
	if (viC_ave < MIN_INPUT_VOL)
	{
		low_vol_C_counter++;
		if (low_vol_C_counter >= PROTECT_COUNT)
		{
			stop_C = 1;
			fault_message2 |= 0b0000000000000100;
			low_vol_C = 1;
		}
		else
		{
			low_vol_C = 0;
			low_vol_C_counter = 0;
		}
	}
	if (iiA_ave > MAX_INPUT_CUR)
	{
		over_cur_A_counter ++;
		if (over_cur_A_counter >= PROTECT_COUNT)			// over range for 10s (50000x200us)
		{
			stop_A = 1;
			fault_message1 |= 0b0000000000000001;
			over_cur_A = 1;
		}
		else
		{
			over_cur_A = 0;
			over_cur_A_counter = 0;
		}
	}
	if (iiB_ave > MAX_INPUT_CUR)
	{
		over_cur_B_counter ++;
		if (over_cur_B_counter >= PROTECT_COUNT)			// over range for 10s (50000x200us)
		{
			stop_B = 1;
			fault_message1 |= 0b0000000000000010;
			over_cur_B = 1;
			// Send_data_CanBus();
		}
		else
		{
			over_cur_B = 0;
			over_cur_B_counter = 0;
		}
	}
	if (iiC_ave > MAX_INPUT_CUR)
	{
		over_cur_C_counter ++;
		if (over_cur_C_counter >= PROTECT_COUNT)			// over range for 10s (50000x200us)
		{
			stop_C = 1;
			fault_message1 |= 0b0000000000000100;
			over_cur_C = 1;
		}
		else
		{
			over_cur_C = 0;
			over_cur_C_counter = 0;
		}
	}
	if (io_ave > MAX_OUTPUT_CUR)
	{
		over_cur_DC_counter ++;
		if (over_cur_DC_counter >= PROTECT_COUNT)			// over range for 10s (50000x200us)
		{
			stop = 1;
			Turn_off_Converter();
			fault_message1 |= 0b0000000000000100;
			over_cur_DC = 1;
			// Send_data_CanBus();
		}
		else
		{
			over_cur_DC = 0;
			over_cur_DC_counter = 0;
		}
	}
}
//================================================================================

}

void Soft_Transition() {
// =====================================
// SOFT TRANSITION MODE OF OPERATION  ==
// Check for mode_change => soft transition mode

if (converter == BAT_CONVERTER)
{
	BAT_Transition_Mode();
}

//=======================================================
else	//PV converter
{
	PV1_Transition_Mode();
	PV2_Transition_Mode();
	PV3_Transition_Mode();
}
switch (converter)
{
case BAT_CONVERTER:
{
	if ((duty_max_reset_A == 1) && (mode != IDLE) && (mode != ISOLATED))
	{
//			test_duty++;
		duty_max_A = duty_max_A + duty_step_plus;
		if (duty_max_A >= MAX_DUTY)
		{
//				test_duty++;
			duty_max_A = MAX_DUTY;
			duty_max_reset_A = 0;
		}
	}
	if ((duty_min_set_A == 1) && (mode != IDLE) && (mode != ISOLATED))
	{
		duty_min_A = duty_min_A - duty_step_minus;
		if (duty_min_A <= MIN_DUTY)
		{
			duty_min_A = MIN_DUTY;
			duty_min_set_A = 0;
		}
	}

//******************************************************
	if ((duty_max_reset_B == 1) && (mode != IDLE) && (mode != ISOLATED))
	{
//			test_duty++;
		duty_max_B = duty_max_B + duty_step_plus;
		if (duty_max_B >= MAX_DUTY)
		{
//				test_duty++;
			duty_max_B = MAX_DUTY;
			duty_max_reset_B = 0;
		}
	}
	if ((duty_min_set_B == 1) && (mode != IDLE) && (mode != ISOLATED))
	{
		duty_min_B = duty_min_B - duty_step_minus;
		if (duty_min_B <= MIN_DUTY)
		{
			duty_min_B = MIN_DUTY;
			duty_min_set_B = 0;
		}
	}

//******************************************************
	if ((duty_max_reset_C == 1) && (mode != IDLE) && (mode != ISOLATED))
	{
//			test_duty++;
		duty_max_C = duty_max_C + duty_step_plus;
		if (duty_max_C >= MAX_DUTY)
		{
//				test_duty++;
			duty_max_C = MAX_DUTY;
			duty_max_reset_C = 0;
		}
	}
	if ((duty_min_set_C == 1) && (mode != IDLE) && (mode != ISOLATED))
	{
		duty_min_C = duty_min_C - duty_step_minus;
		if (duty_min_C <= MIN_DUTY)
		{
			duty_min_C = MIN_DUTY;
			duty_min_set_C = 0;
		}
	}
	break;
}

case PV_CONVERTER:
{
	if ((duty_max_reset_A == 1) && (mode_A != IDLE) && (mode_A != ISOLATED))
	{
//			test_duty++;
		duty_max_A = duty_max_A + duty_step_pv;
		if (duty_max_A >= MAX_DUTY_PV_A)
		{
//				test_duty++;
			duty_max_A = MAX_DUTY_PV_A;
			duty_max_reset_A = 0;
		}
	}
	if ((duty_min_set_A == 1) && (mode_A != IDLE) && (mode_A != ISOLATED))
	{
		duty_min_A = duty_min_A - duty_step_pv;
		if (duty_min_A <= MIN_DUTY_PV_A)
		{
			duty_min_A = MIN_DUTY_PV_A;
			duty_min_set_A = 0;
		}
	}
	if ((duty_max_reset_B == 1) && (mode_B != IDLE) && (mode_B != ISOLATED))
	{
//			test_duty++;
		duty_max_B = duty_max_B + 0.02;
		if (duty_max_B >= MAX_DUTY)
		{
//				test_duty++;
			duty_max_B = MAX_DUTY;
			duty_max_reset_B = 0;
		}
	}
	if ((duty_min_set_B == 1) && (mode_B != IDLE) && (mode_B != ISOLATED))
	{
		duty_min_B = duty_min_B - 0.002;
		if (duty_min_B <= MIN_DUTY)
		{
			duty_min_B = MIN_DUTY;
			duty_min_set_B = 0;
		}
	}
	if ((duty_max_reset_C == 1) && (mode_C != IDLE) && (mode_C != ISOLATED))
	{
//			test_duty++;
		duty_max_C = duty_max_C + 0.02;
		if (duty_max_C >= MAX_DUTY)
		{
//				test_duty++;
			duty_max_C = MAX_DUTY;
			duty_max_reset_C = 0;
		}
	}
	if ((duty_min_set_C == 1) && (mode_C != IDLE) && (mode_C != ISOLATED))
	{
		duty_min_C = duty_min_C - 0.002;
		if (duty_min_C <= MIN_DUTY)
		{
			duty_min_C = MIN_DUTY;
			duty_min_set_C = 0;
		}
	}
	break;
}
}
}
void BAT_Transition_Mode() {
if (stop == 0)
{
		if ((mode == ISOLATED) && (new_mode != mode))
		{

			ENABLE_ALL_PWM();
			ENABLE_ALL_RELAY();
			CLOSE_RELAY_INPUT();
			CLOSE_RELAY_OUTPUT();
			delay_set = 1;
			if (delay_counter >= PRECHARGING_DELAY)
			{
				CLOSE_RELAY_DC();
				CLOSE_RELAY_BAT();
				INPUT_RELAY_STATUS = 1;
				OUTPUT_RELAY_STATUS = 1;
			}
			if (delay_counter >= (PRECHARGING_DELAY*1.5))
			{
				delay_set = 0;
				delay_counter = 0;
				OPEN_RELAY_INPUT();
				OPEN_RELAY_OUTPUT();
				mode = IDLE;
				duty_min_set_A = 0;
				duty_max_reset_A = 0;

				duty_min_set_B = 0;
				duty_max_reset_B = 0;

				duty_min_set_C = 0;
				duty_max_reset_C = 0;

				LED_RUN_OFF();
				LED_STOP_ON();
				CONVERTER_STATUS = 0;
				CH1_ONOFF_STATUS = 0;
				CH2_ONOFF_STATUS = 0;
				CH3_ONOFF_STATUS = 0;

			}

		}

if (delay_set == 0){
switch (new_mode)
{
	case VRM:
	{
		if (mode == IDLE)
		{
			duty_max_A = 0;
			duty_min_A = 0;
			duty_max_reset_A = 1;

			duty_max_B = 0;
			duty_min_B = 0;
			duty_max_reset_B = 1;

			duty_max_C = 0;
			duty_min_C = 0;
			duty_max_reset_C = 1;

			mode = new_mode;
			reference_vol = reference_vol_new;
			CONVERTER_STATUS = 1;
			CH1_ONOFF_STATUS = 1;
			CH2_ONOFF_STATUS = 1;
			CH3_ONOFF_STATUS = 1;
			LED_RUN_ON();
			LED_STOP_OFF();
		}

		else if ((mode == PRM) || ((mode == VRM) && (reference_vol != reference_vol_new)))
		{
			mode = new_mode;
			reference_vol = reference_vol_new;
		}

		break;
	}

	case PRM:
	{
		if ((mode == VRM) || ((mode == PRM) && (reference_power_new != reference_power)))
		{
			mode = new_mode;
			reference_power = reference_power_new;
		}
		else if (mode == IDLE)
		{
			if (reference_power_new < 0)
			{
				duty_max_A = period;
				duty_min_A = period;
				duty_min_set_A = 1;

				duty_max_B = period;
				duty_min_B = period;
				duty_min_set_B = 1;

				duty_max_C = period;
				duty_min_C = period;
				duty_min_set_C = 1;
			}
			else
			{
				duty_min_A = 0;
				duty_max_A = 0;
				duty_max_reset_A = 1;

				duty_min_B = 0;
				duty_max_B = 0;
				duty_max_reset_B = 1;

				duty_min_C = 0;
				duty_max_C = 0;
				duty_max_reset_C = 1;
			}
			mode = new_mode;
			reference_power = reference_power_new;

			CONVERTER_STATUS = 1;
			CH1_ONOFF_STATUS = 1;
			CH2_ONOFF_STATUS = 1;
			CH3_ONOFF_STATUS = 1;
			LED_RUN_ON();
			LED_STOP_OFF();
		}
			break;
	}

	case IDLE:
	{
		mode = new_mode;
		CONVERTER_STATUS = 0;
		CH1_ONOFF_STATUS = 0;
		CH2_ONOFF_STATUS = 0;
		CH3_ONOFF_STATUS = 0;
		LED_RUN_OFF();
		LED_STOP_ON();
		break;
	}

	case ISOLATED:
	{
		mode = new_mode;
		Turn_off_Converter();
		break;
	}

}
}

}
}
void PV1_Transition_Mode() {
if (stop == 0)
{
	if ((mode_A == ISOLATED) && (new_mode_A != mode_A))
	{
		ENABLE_ALL_PWM();
		ENABLE_ALL_RELAY();
		CLOSE_RELAY_INPUT();
		CLOSE_RELAY_OUTPUT();

		delay_set = 1;
		if (delay_counter >= PRECHARGING_DELAY)
		{
			CLOSE_RELAY_DC();
			CLOSE_RELAY_BAT();
			INPUT_RELAY_STATUS = 1;
			OUTPUT_RELAY_STATUS = 1;
		}

//			DELAY_US(200000);
		if (delay_counter >= (PRECHARGING_DELAY*1.5))
		{
			delay_set = 0;
			delay_counter = 0;
			OPEN_RELAY_INPUT();
			OPEN_RELAY_OUTPUT();
			mode_A = IDLE;
			duty_min_set_A = 0;
			duty_max_reset_A = 0;

			LED_RUN_OFF();
			LED_STOP_ON();
			CONVERTER_STATUS = 0;
			CH1_ONOFF_STATUS = 0;
			CH2_ONOFF_STATUS = 0;
			CH3_ONOFF_STATUS = 0;
		}
		if ((mode_B == IDLE) || (mode_C == IDLE))
		{
			mode_A = IDLE;
			duty_min_set_A = 0;
			duty_max_reset_A = 0;
		}

	}

if (delay_set == 0) {
switch (new_mode_A)
{
case VRM:
{
	if ((mode_A == IDLE) || (mode_A == MPPT) || (mode_A == PRM))
	{
		duty_max_A = 0;
		duty_min_A = 0;
		duty_max_reset_A = 1;
		mode_A = new_mode_A;
		reference_vol_A = 380;
		CONVERTER_STATUS = 1;
		CH1_ONOFF_STATUS = 1;
		LED_RUN_ON();
		LED_STOP_OFF();
	}
	break;
}

case PRM:
{
	if ((mode_A == MPPT) || (mode_A == VRM) || (mode_A == IDLE))
	{
		duty_min_A = 0;
		duty_max_A = 0;
		duty_max_reset_A = 1;
		mode_A = new_mode_A;
		reference_power_A = reference_power_new_A;
		CONVERTER_STATUS = 1;
		CH1_ONOFF_STATUS = 1;
		LED_RUN_ON();
		LED_STOP_OFF();
	}
	else if ((mode_A == PRM) && (reference_power_new_A != reference_power_A))
	{
		reference_power_A = reference_power_new_A;
	}
		break;
}

case MPPT:
{
	if(mode_A != MPPT)
	{
		vref_A = vPV_max;
		duty_max_A = period;
		duty_min_A = period;
		duty_min_set_A = 1;
		mode_A = MPPT;
		CONVERTER_STATUS = 1;
		CH1_ONOFF_STATUS = 1;
		LED_RUN_ON();
		LED_STOP_OFF();

		sum_viA = 0;
		sum_iiA = 0;
		ave_viA = viA_ave;
		ave_iiA = iiA_ave;
		k = 0;
	}
	break;
}

case IDLE:
{
	mode_A = IDLE;
	CONVERTER_STATUS = 0;
	CH1_ONOFF_STATUS = 0;
	LED_RUN_OFF();
	LED_STOP_ON();
	break;
}

case ISOLATED:
{
	mode_B = ISOLATED;
	mode_A = ISOLATED;
	mode_C = ISOLATED;
	Turn_off_Converter();
	break;
}

}
}
}
}
void PV2_Transition_Mode() {
	if (stop == 0)
	{
			if ((mode_B == ISOLATED) && (new_mode_B != mode_B))
			{
//				mode_B = IDLE;
//				duty_min_set_B = 0;
//				duty_max_reset_B = 0;
//
//				ENABLE_ALL_PWM();
//				CLOSE_RELAY_DC();
//				CLOSE_RELAY_BAT();
//				INPUT_RELAY_STATUS = 1;
//				OUTPUT_RELAY_STATUS = 1;
//				LED_RUN_OFF();
//				LED_STOP_ON();
//				CONVERTER_STATUS = 0;
//				CH2_ONOFF_STATUS = 0;


//				ENABLE_ALL_PWM();
//				ENABLE_ALL_RELAY();
//				CLOSE_RELAY_INPUT();
//				CLOSE_RELAY_OUTPUT();
//
//				delay_set = 1;
//				if (delay_counter >= 20000)
//				{
//					CLOSE_RELAY_DC();
//					CLOSE_RELAY_BAT();
//					INPUT_RELAY_STATUS = 1;
//					OUTPUT_RELAY_STATUS = 1;
//				}

		//			DELAY_US(200000);

//				if (delay_counter >= 30000)
//				{
//					delay_set = 0;
//					delay_counter = 0;
//					OPEN_RELAY_INPUT();
//					OPEN_RELAY_OUTPUT();
//					mode_B = IDLE;
//					duty_min_set_B = 0;
//					duty_max_reset_B = 0;
//
//					LED_RUN_OFF();
//					LED_STOP_ON();
//					CONVERTER_STATUS = 0;
//					CH1_ONOFF_STATUS = 0;
//					CH2_ONOFF_STATUS = 0;
//					CH3_ONOFF_STATUS = 0;
//				}

//				if ((mode_A == 0) || (mode_C == 0))
//					{
//					mode_B = IDLE;
//					duty_min_set_B = 0;
//					duty_max_reset_B = 0;
//					}

			}
if (delay_set == 0) {
	switch (new_mode_B)
	{
		case VRM:
		{
			if (mode_B == IDLE)
			{
				duty_max_B = 0;
				duty_min_B = 0;
				duty_max_reset_B = 1;

				mode_B = new_mode_B;
				reference_vol_B = reference_vol_new_B;
				CONVERTER_STATUS = 1;
				CH2_ONOFF_STATUS = 1;
				LED_RUN_ON();
				LED_STOP_OFF();
			}

			else if ((mode_B == PRM) || ((mode_B == VRM) && (reference_vol_B != reference_vol_new_B)))
			{
				mode_B = new_mode_B;
				reference_vol_B = reference_vol_new_B;
			}

			break;
		}

		case PRM:
		{
			if ((mode_B == VRM) || ((mode_B == PRM) && (reference_power_new_B != reference_power_B)))
			{
				mode_B = new_mode_B;
				reference_power_B = reference_power_new_B;
			}
			else if (mode_B == IDLE)
			{
				duty_min_B = 0;
				duty_max_B = 0;
				duty_max_reset_B = 1;
				mode_B = new_mode_B;
				reference_power_B = reference_power_new_B;

				CONVERTER_STATUS = 1;
				CH2_ONOFF_STATUS = 1;
				LED_RUN_ON();
				LED_STOP_OFF();
			}
				break;
		}
		case MPPT:
				{
					if(mode_B != MPPT)
					{
						vref_B = vPV_max;
						duty_max_B = period;
						duty_min_B = period;
						duty_min_set_B = 1;
						mode_B = MPPT;
						CONVERTER_STATUS = 1;
						CH2_ONOFF_STATUS = 1;
						LED_RUN_ON();
						LED_STOP_OFF();

						sum_viB = 0;
						sum_iiB = 0;
						ave_viB = viB_ave;
						ave_iiB = iiB_ave;
						k = 0;
					}
					break;
				}

		case IDLE:
		{
			mode_B = new_mode_B;
			CONVERTER_STATUS = 0;
			CH2_ONOFF_STATUS = 0;
			LED_RUN_OFF();
			LED_STOP_ON();
			break;
		}

		case ISOLATED:
		{
			mode_B = new_mode_B;
			Turn_off_Converter();
			break;
		}

	}
}
	}
}

void PV3_Transition_Mode() {
	if (stop == 0)
	{
//			if ((mode_C == ISOLATED) && (new_mode_C != mode_C))
//			{
//				mode_C = IDLE;
//				duty_min_set_C = 0;
//				duty_max_reset_C = 0;

//				ENABLE_ALL_PWM();
//				CLOSE_RELAY_DC();
//				CLOSE_RELAY_BAT();
//				INPUT_RELAY_STATUS = 1;
//				OUTPUT_RELAY_STATUS = 1;
//				LED_RUN_OFF();
//				LED_STOP_ON();
//				CONVERTER_STATUS = 0;
//				CH3_ONOFF_STATUS = 0;

//				ENABLE_ALL_PWM();
//				ENABLE_ALL_RELAY();
//				CLOSE_RELAY_INPUT();
//				CLOSE_RELAY_OUTPUT();

//				delay_set = 1;
//				if (delay_counter >= 20000)
//				{
//					CLOSE_RELAY_DC();
//					CLOSE_RELAY_BAT();
//					INPUT_RELAY_STATUS = 1;
//					OUTPUT_RELAY_STATUS = 1;
//				}

		//			DELAY_US(200000);
//				if (delay_counter >= 300000)
//				{
//					delay_set = 0;
//					delay_counter = 0;
//					OPEN_RELAY_INPUT();
//					OPEN_RELAY_OUTPUT();
//					mode_C = IDLE;
//					duty_min_set_C = 0;
//					duty_max_reset_C = 0;
//
//					LED_RUN_OFF();
//					LED_STOP_ON();
//					CONVERTER_STATUS = 0;
//					CH1_ONOFF_STATUS = 0;
//					CH2_ONOFF_STATUS = 0;
//					CH3_ONOFF_STATUS = 0;
//				}

//				if ((mode_A == 0) || (mode_B == 0))
//					{
//					mode_C = IDLE;
//					duty_min_set_C = 0;
//					duty_max_reset_C = 0;
//					}
//
//
//			}
if (delay_set == 0) {
	switch (new_mode_C)
	{
		case VRM:
		{
			if (mode_C == IDLE)
			{
				duty_max_C = 0;
				duty_min_C = 0;
				duty_max_reset_C = 1;

				mode_C = new_mode_C;
				reference_vol_C = reference_vol_new_C;
				CONVERTER_STATUS = 1;
				CH3_ONOFF_STATUS = 1;
				LED_RUN_ON();
				LED_STOP_OFF();
			}

			else if ((mode_C == PRM) || ((mode_C == VRM) && (reference_vol_C != reference_vol_new_C)))
			{
				mode_C = new_mode_C;
				reference_vol_C = reference_vol_new_C;
			}

			break;
		}

		case PRM:
		{
			if ((mode_C == VRM) || ((mode_C == PRM) && (reference_power_new_C != reference_power_C)))
			{
				mode_C = new_mode_C;
				reference_power_C = reference_power_new_C;
			}
			else if (mode_C == IDLE)
			{
				duty_min_C = 0;
				duty_max_C = 0;
				duty_max_reset_C = 1;
				mode_C = new_mode_C;
				reference_power_C = reference_power_new_C;

				CONVERTER_STATUS = 1;
				CH3_ONOFF_STATUS = 1;
				LED_RUN_ON();
				LED_STOP_OFF();
			}
				break;
		}
		case MPPT:
		{
			if(mode_C != MPPT)
			{
				vref_C = vPV_max;
				duty_max_C = period;
				duty_min_C = period;
				duty_min_set_C = 1;
				mode_C = MPPT;
				CONVERTER_STATUS = 1;
				CH3_ONOFF_STATUS = 1;
				LED_RUN_ON();
				LED_STOP_OFF();

				sum_viC = 0;
				sum_iiC = 0;
				ave_viC = viA_ave;
				ave_iiC = iiA_ave;
				k = 0;
			}
			break;
		}

		case IDLE:
		{
			mode_C = new_mode_C;
			CONVERTER_STATUS = 0;
			CH3_ONOFF_STATUS = 0;
			LED_RUN_OFF();
			LED_STOP_ON();
			break;
		}

		case ISOLATED:
		{
			mode_C = new_mode_C;
			Turn_off_Converter();
			break;
		}

	}
}
	}
}
interrupt void  adc_isr(void)
{
	/*adc_a0 = AdcRegs.ADCRESULT0 >> 4;  //Iout     offset  2055
	adc_a1 = AdcRegs.ADCRESULT1 >> 4;  //IL3              2055
	adc_a2 = AdcRegs.ADCRESULT2 >> 4;  //VESS1            2057
	adc_a3 = AdcRegs.ADCRESULT3 >> 4;  //VDC+             2058
	adc_a4 = AdcRegs.ADCRESULT4 >> 4;  //IL2              2054
	adc_a5 = AdcRegs.ADCRESULT5 >> 4;  //IL1              2058
	adc_b6 = AdcRegs.ADCRESULT6 >> 4;  //VESS3
	adc_b7 = AdcRegs.ADCRESULT7 >> 4;  //VESS2*/


		adc_a0 = AdcRegs.ADCRESULT7>>4;  //	Iw
		adc_a1 = AdcRegs.ADCRESULT5>>4;  // vDC
		adc_a2 = AdcRegs.ADCRESULT1>>4;  // Vw
		adc_a3 = AdcRegs.ADCRESULT0>>4;  //	Iv
		adc_a4 = AdcRegs.ADCRESULT3>>4;  // Iu
		adc_a5 = AdcRegs.ADCRESULT6>>4;  //	Vu
		adc_b6 = AdcRegs.ADCRESULT2>>4;  //	Vv
		adc_b7 = AdcRegs.ADCRESULT4>>4;  // iDC


	if (converter == BAT_CONVERTER)
	{
		iiA = (float) ((adc_a4-offset_a4)*gain_a4);
		iiB  = (float) ((adc_a3-offset_a3)*gain_a3);
		iiC = (float) ((adc_a0-offset_a0)*gain_a0);

		viA = (float) ((adc_a5-offset_a5)*gain_a5);
		viB = (float) ((adc_b6-offset_b6)*gain_b6);
		viC = (float) ((adc_a2-offset_a2)*gain_a2);		// ADCresult = 4096*analog/3   3.0/4096.0

		vo = (float) ((adc_a1-offset_a1)*gain_a1);
		io  = (float) ((adc_b7-offset_b7)*gain_b7);
	}
	else
	{
		iiA = (float) ((adc_a4-offset_a4_pv)*gain_a4_pv);
		iiB  = (float) ((adc_a3-offset_a3_pv)*gain_a3_pv);
		iiC = (float) ((adc_a0-offset_a0_pv)*gain_a0_pv);

		viA = (float) ((adc_a5-offset_a5_pv)*gain_a5_pv);
		viB = (float) ((adc_b6-offset_b6_pv)*gain_b6_pv);
		viC = (float) ((adc_a2-offset_a2_pv)*gain_a2_pv);		// ADCresult = 4096*analog/3   3.0/4096.0

		vo = (float) ((adc_a1-offset_a1_pv)*gain_a1_pv);
		io  = (float) ((adc_b7-offset_b7_pv)*gain_b7_pv);
	}
	ConversionCount++;

	viA_sum = viA_sum + viA;
	viB_sum = viB_sum + viB;
	viC_sum = viC_sum + viC;
	vo_sum  = vo_sum  + vo;

	iiA_sum = iiA_sum + iiA;
	iiB_sum = iiB_sum + iiB;
	iiC_sum = iiC_sum + iiC;
	io_sum  = io_sum  + io;
	AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;         // Reset SEQ1
	AdcRegs.ADCST.bit.INT_SEQ1_CLR = 1;       // Clear INT SEQ1 bit
	PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;   // Acknowledge interrupt to PIE
}
void ADC_Calculation(){
	if (ConversionCount != 0)	ConversionCount1 = (float) (1.0 / ConversionCount);

	viA_ave = viA_sum*ConversionCount1;
	viB_ave = viB_sum*ConversionCount1;
	viC_ave = viC_sum*ConversionCount1;
	vo_ave  = vo_sum*ConversionCount1;

	iiA_ave = iiA_sum*ConversionCount1;
	iiB_ave = iiB_sum*ConversionCount1;
	iiC_ave = iiC_sum*ConversionCount1;
	io_ave  = io_sum*ConversionCount1;

	i_in_ave = (iiA_ave + iiB_ave + iiC_ave)*0.333;

	iA_BBB = LowPassFilter(iA_BBB, iiA_ave, 0.5);
	iB_BBB = LowPassFilter(iB_BBB, iiB_ave, 0.5);
	iC_BBB = LowPassFilter(iC_BBB, iiC_ave, 0.5);

	viA_sum = 0;
	viB_sum = 0;
	viC_sum = 0;
	vo_sum  = 0;
	iiA_sum = 0;
	iiB_sum = 0;
	iiC_sum = 0;
	io_sum  = 0;

	ConversionCount = 0;
}
void PI_Controller() {

if (converter == BAT_CONVERTER)
{
	if ((mode == VRM) && (ecan_receive_done == 1))
		{
		vref_A = reference_vol - droop_coef *io_ave;
		vref_B = vref_A;
		vref_C = vref_A;
		ev_A = vref_A - vo_ave;
		ev_B = ev_A;
		ev_C = ev_A;

		reference_power = viA_ave*i_in_ave*0.003;
		}

		iref_A = iref_A_pre + (kpv_A + kiv_A * Ts / 2) * ev_A + (-kpv_A + kiv_A * Ts / 2) * ev_A_pre;
		iref_B = iref_B_pre + (kpv_B + kiv_B * Ts / 2) * ev_B + (-kpv_B + kiv_B * Ts / 2) * ev_B_pre;
		iref_C = iref_C_pre + (kpv_C + kiv_C * Ts / 2) * ev_C + (-kpv_C + kiv_C * Ts / 2) * ev_C_pre;
		if (iref_A < -i_sat)	iref_A = -i_sat;
		if (iref_B < -i_sat)	iref_B = -i_sat;
		if (iref_C < -i_sat)	iref_C = -i_sat;
		if (iref_A > i_sat)	iref_A = i_sat;
		if (iref_B > i_sat)	iref_B = i_sat;
		if (iref_C > i_sat)	iref_C = i_sat;

	// Power Dispatch
		if (mode == PRM)
		{
			i_set = testing*333.3/viA_ave;

			if (i_set >= 0.7*MAX_INPUT_CUR)	i_set =  0.7*MAX_INPUT_CUR;
			if (i_set <= -0.7*MAX_INPUT_CUR)	i_set = -0.7*MAX_INPUT_CUR;
			iref_A = i_set;
			iref_B = i_set;
			iref_C = i_set;
		}
		ei_A = iref_A - iiA_ave;
		duty_A = duty_A_pre + (kpi_A + kii_A * Ts / 2) * ei_A + (-kpi_A + kii_A * Ts / 2) * ei_A_pre;

		ei_B = iref_B - iiB_ave;
		duty_B = duty_B_pre + (kpi_B + kii_B * Ts / 2) * ei_B + (-kpi_B + kii_B * Ts / 2) * ei_B_pre;

		ei_C = iref_C - iiC_ave;
		duty_C = duty_C_pre + (kpi_C + kii_C * Ts / 2) * ei_C + (-kpi_C + kii_C * Ts / 2) * ei_C_pre;
		if (duty_A >= duty_max_A) 		duty_A = duty_max_A;
		if (duty_A <= duty_min_A)		duty_A = duty_min_A;

		if (duty_B >= duty_max_B) 		duty_B = duty_max_B;
		if (duty_B <= duty_min_B)		duty_B = duty_min_B;

		if (duty_C >= duty_max_C) 		duty_C = duty_max_C;
		if (duty_C <= duty_min_C)		duty_C = duty_min_C;
		viA_ave_pre = viA_ave;
		vref_A_pre = vref_A;
		ev_A_pre = ev_A;
		ei_A_pre = ei_A;
		iref_A_pre = iref_A;
		duty_A_pre = duty_A;

		viB_ave_pre = viB_ave;
		vref_B_pre = vref_B;
		ev_B_pre = ev_B;
		ei_B_pre = ei_B;
		iref_B_pre = iref_B;
		duty_B_pre = duty_B;

		viC_ave_pre = viC_ave;
		vref_C_pre = vref_C;
		ev_C_pre = ev_C;
		ei_C_pre = ei_C;
		iref_C_pre = iref_C;
		duty_C_pre = duty_C;
}
else
{
	if ((mode_A == VRM) && (ecan_receive_done == 1))
	{
		vref_A = reference_vol_A - droop_coef_A *io_ave;
		ev_A = vref_A - vo_ave;
	}

	if (mode_A == MPPT)
	{
		ev_A = vref_A - viA_ave;
	}
	iref_A = iref_A_pre + (kpv_A1 + kiv_A1 * Ts / 2) * ev_A + (-kpv_A1 + kiv_A1 * Ts / 2) * ev_A_pre;
	if (iref_A < -i_sat)	iref_A = -i_sat;
	if (iref_A > i_sat)		iref_A = i_sat;

	// Power Dispatch
		if (mode_A == PRM)
			{
			i_set_A = reference_power_A*1000/viA_ave;      // 3 channel
			if (i_set_A >=  0.4*MAX_INPUT_CUR)	i_set_A =  0.4*MAX_INPUT_CUR;
			if (i_set_A <= -0.4*MAX_INPUT_CUR)	i_set_A = -0.4*MAX_INPUT_CUR;
			iref_A = i_set_A;
			}
		ei_A = iref_A - iiA_ave;
		duty_A = duty_A_pre + (kpi_A1 + kii_A1 * Ts / 2) * ei_A + (-kpi_A1 + kii_A1 * Ts / 2) * ei_A_pre;
		if (mode_A == 2){
		if (duty_min_A <= MIN_DUTY_PV_A)	duty_min_A = MIN_DUTY_PV_A;
		}
		if (duty_A >= duty_max_A) 		duty_A = duty_max_A;
		if (duty_A <= duty_min_A)		duty_A = duty_min_A;
		viA_ave_pre = viA_ave;
		vref_A_pre = vref_A;
		ev_A_pre = ev_A;
		ei_A_pre = ei_A;
		iref_A_pre = iref_A;
		duty_A_pre = duty_A;

		PowerPV_A_pre = PowerPV_A;
		ave_viA_pre	  = ave_viA;
		ave_iiA_pre	  = ave_iiA;
		if ((mode_B == VRM) && (ecan_receive_done == 1))
		{
		vref_B = reference_vol_B - droop_coef_B *io_ave;
		ev_B = vref_B - vo_ave;
		}

		if (mode_B == MPPT)
		{
		ev_B = vref_B - viB_ave;
		}

		iref_B = iref_B_pre + (kpv_B + kiv_B * Ts / 2) * ev_B + (-kpv_B + kiv_B * Ts / 2) * ev_B_pre;
		if (iref_B < -i_sat)	iref_B = -i_sat;
		if (iref_B > i_sat)		iref_B = i_sat;

	// Power Dispatch
		if (mode_B == PRM)
			{
			i_set_B = reference_power_B*1000/viB_ave;      //3 channel
			if (i_set_B >=  0.4*MAX_INPUT_CUR)	i_set_B =  0.4*MAX_INPUT_CUR;
			if (i_set_B <= -0.4*MAX_INPUT_CUR)	i_set_B = -0.4*MAX_INPUT_CUR;
			iref_B = i_set_B;
			}
		ei_B = iref_B - iiB_ave;
		duty_B = duty_B_pre + (kpi_B + kii_B * Ts / 2) * ei_B + (-kpi_B + kii_B * Ts / 2) * ei_B_pre;
		if (duty_B >= duty_max_B) 		duty_B = duty_max_B;
		if (duty_B <= duty_min_B)		duty_B = duty_min_B;
		viB_ave_pre = viB_ave;
		vref_B_pre = vref_B;
		ev_B_pre = ev_B;
		ei_B_pre = ei_B;
		iref_B_pre = iref_B;
		duty_B_pre = duty_B;
		if ((mode_C == VRM) && (ecan_receive_done == 1))
		{
		vref_C = reference_vol_C - droop_coef_C *io_ave;
		ev_C = vref_C - vo_ave;
		}

		if (mode_C == MPPT)
		{
		ev_C = vref_C - viC_ave;
		}

		iref_C = iref_C_pre + (kpv_C + kiv_C * Ts / 2) * ev_C + (-kpv_C + kiv_C * Ts / 2) * ev_C_pre;
		if (iref_C < -i_sat)	iref_C = -i_sat;
		if (iref_C > i_sat)		iref_C = i_sat;

		// Power Dispatch
		if (mode_C == PRM)
			{
			i_set_C = reference_power_C*1000/viC_ave;      //3 channel
			if (i_set_C >=  0.4*MAX_INPUT_CUR)	i_set_C =  0.4*MAX_INPUT_CUR;
			if (i_set_C <= -0.4*MAX_INPUT_CUR)	i_set_C = -0.4*MAX_INPUT_CUR;
			iref_C = i_set_C;
			}
			ei_C = iref_C - iiC_ave;
			duty_C = duty_C_pre + (kpi_C + kii_C * Ts / 2) * ei_C + (-kpi_C + kii_C * Ts / 2) * ei_C_pre;
		if (duty_C >= duty_max_C) 		duty_C = duty_max_C;
		if (duty_C <= duty_min_C)		duty_C = duty_min_C;
		viC_ave_pre = viC_ave;
		vref_C_pre = vref_C;
		ev_C_pre = ev_C;
		ei_C_pre = ei_C;
		iref_C_pre = iref_C;
		duty_C_pre = duty_C;
}

}
void PWM_Modulation () {

	duty_HA = period - duty_A - deadtime;				// 0 = off; 375 = ON max

	if (duty_HA <= 0) duty_HA = 0;

	if (mode_A == MPPT)
	{
		duty_LA = duty_A;
	}
	else
	{
		duty_LA = period - duty_A;
	}

	duty_HB = period - duty_B - deadtime;

	if (duty_HB <= 0) duty_HB = 0;

	if (mode_B == MPPT)
	{
		duty_LB = duty_B;
	}
	else
	{
		duty_LB = period - duty_B;
	}

//	duty_LB = period - duty_B;


	duty_HC = period - duty_C - deadtime;

	if (duty_HC <= 0) duty_HC = 0;

	if (mode_C == MPPT)
	{
		duty_LC = duty_C;
	}
	else
	{
		duty_LC = period - duty_C;
	}

//	duty_LC = period - duty_C;


	if (converter == BAT_CONVERTER)
	{
		if (duty_max_reset_A == 1)	duty_HA = 0;								// 0 = off; 375 = ON max
		if (duty_max_reset_B == 1)	duty_HB = 0;								// 0 = off; 375 = ON max
		if (duty_max_reset_C == 1)	duty_HC = 0;								// 0 = off; 375 = ON max


		if (duty_min_set_A == 1)	duty_LA = period;								// 0 = ON max; 375 = off
		if (duty_min_set_B == 1)	duty_LB = period;								// 0 = ON max; 375 = off
		if (duty_min_set_C == 1)	duty_LC = period;								// 0 = ON max; 375 = off
	}

switch (converter)
{
case BAT_CONVERTER:
{
if ((stop == 1) || (mode == IDLE) || (mode == ISOLATED))
{
	duty_HA = 0;								// 375 = off; 0 = ON max
	duty_HB = 0;
	duty_HC = 0;
	duty_LA = period;								// 375 = off; 0 = ON max
	duty_LB = period;
	duty_LC = period;
	CH1_ONOFF_STATUS = 0;
	CH2_ONOFF_STATUS = 0;
	CH3_ONOFF_STATUS = 0;
	CONVERTER_STATUS = 0;
}
break;
}
case PV_CONVERTER:
{
if ((stop_A == 1) || (mode_A == IDLE) || (mode_A == ISOLATED))
{
	duty_HA = 0;
	duty_LA = period;
	CH1_ONOFF_STATUS = 0;
}

if ((stop_B == 1) || (mode_B == IDLE) || (mode_B == ISOLATED))
{
	duty_HB= 0;
	duty_LB = period;
	CH2_ONOFF_STATUS = 0;
}

if ((stop_C == 1) || (mode_C == IDLE) || (mode_C == ISOLATED))
{
	duty_HC = 0;
	duty_LC = period;
	CH3_ONOFF_STATUS = 0;
}

if ((CH1_ONOFF_STATUS == 0) && (CH2_ONOFF_STATUS == 0) && (CH3_ONOFF_STATUS == 0))

	{	CONVERTER_STATUS = 0;
		duty_HA = 0;
		duty_HB = 0;
		duty_HC = 0;
		duty_LA = period;
		duty_LB = period;
		duty_LC = period;
	}
break;
}
}


//test
//
//	 duty_HA = 0;
//	 duty_LA = 1875;
////////
//	 duty_HB = 0;
//	 duty_LB = 1875;
//
//	 duty_HC = 0;
//	 duty_LC = 1875;
 //

}
void MPPT_PO(){

	// Channel A
		if (mode_A == MPPT)
		{
			sum_viA = sum_viA + viA_ave;
			sum_iiA = sum_iiA + iiA_ave;
			k++;
			if (k >= (MPPT_ADC_COUNT - 1))
			{
				k = 0;
				ave_viA = sum_viA / MPPT_ADC_COUNT;
				ave_iiA = sum_iiA / MPPT_ADC_COUNT;
				sum_viA = 0;
				sum_iiA = 0;

				PowerPV_A = ave_viA * ave_iiA;
				if ((PowerPV_A - PowerPV_A_pre) < -deltaP)
				{
					epsilon_A = -epsilon_A;
					vref_A= vref_A_pre + epsilon_A;
				}
				else if ((PowerPV_A - PowerPV_A_pre) > deltaP)
				{
					vref_A= vref_A_pre + epsilon_A;
				}
				else
				{
					vref_A= vref_A_pre;
				}
				if (vref_A <= vPV_min)			vref_A = vPV_min;   //PV reference voltage saturation
				if (vref_A >= vPV_max)			vref_A = vPV_max;

			}

		}
// Channel B
	/*if (mode_B == MPPT) //MPPT
	{
			PowerPV_B = viB_ave * iiB_ave;

			if ((PowerPV_B - PowerPV_B_pre) <= deltaP) //Caculate to change the reference of the  Voltage Close Loop
			{
				if (viB_ave > viB_ave_pre)	vref_B = vref_B_pre + epsilon;
				else						vref_B = vref_B_pre - epsilon;
			}
			else if ((PowerPV_B - PowerPV_B_pre) >= -deltaP)
			{
				if (viB_ave > viB_ave_pre)	vref_B = vref_B_pre - epsilon;
				else						vref_B = vref_B_pre + epsilon;
			}
			else
			{
				vref_B = vref_B_pre;
			}
			if (vref_B <= vPV_min)			vref_B = vPV_min;   //PV reference voltage saturation
			if (vref_B >= vPV_max)			vref_B = vPV_max;
	}*/

//Channel B
		if (mode_B == MPPT)
			{
					sum_viB = sum_viB + viB_ave;
					sum_iiB = sum_iiB + iiB_ave;
					k++;
					if (k >= (MPPT_ADC_COUNT - 1))
					{
						k = 0;
						ave_viB = sum_viB / MPPT_ADC_COUNT;
						ave_iiB = sum_iiB / MPPT_ADC_COUNT - 0.6;
						sum_viB = 0;
						sum_iiB = 0;

						PowerPV_B = ave_viB * ave_iiB;
						if ((PowerPV_B - PowerPV_B_pre) < -deltaP)
						{
							epsilon_B = -epsilon_B;
							vref_B= vref_B_pre + epsilon_B;
						}
						else if ((PowerPV_B - PowerPV_B_pre) > deltaP)
						{
							vref_B= vref_B_pre + epsilon_B;
						}
						else
						{
							vref_B= vref_B_pre;
						}
						if (vref_B <= vPV_min)			vref_B = vPV_min;   //PV reference voltage saturation
						if (vref_B >= vPV_max)			vref_B = vPV_max;

					}
			}

// Channel C
	 //MPPT
//	{
//			PowerPV_C = viC_ave * iiC_ave;
//
//			if ((PowerPV_C - PowerPV_C_pre) <= deltaP) //Caculate to change the reference of the  Voltage Close Loop
//			{
//				if (viC_ave > viC_ave_pre)	vref_C = vref_C_pre + epsilon;
//				else						vref_C = vref_C_pre - epsilon;
//			}
//			else if ((PowerPV_C - PowerPV_C_pre) >= -deltaP)
//			{
//				if (viC_ave > viC_ave_pre)	vref_C = vref_C_pre - epsilon;
//				else						vref_C = vref_C_pre + epsilon;
//			}
//			else
//			{
//				vref_C = vref_C_pre;
//			}
//			if (vref_C <= vPV_min)			vref_C = vPV_min;   //PV reference voltage saturation
//			if (vref_C >= vPV_max)			vref_C = vPV_max;
//	}
//	channel c
	
	if (mode_C == MPPT)
	{
			sum_viC = sum_viC + viC_ave;
			sum_iiC = sum_iiC + iiC_ave;
			k++;
			if (k >= (MPPT_ADC_COUNT - 1))
			{
				k = 0;
				ave_viC = sum_viC / MPPT_ADC_COUNT;
				ave_iiC = sum_iiC / MPPT_ADC_COUNT - 0.6;
				sum_viC = 0;
				sum_iiC = 0;

				PowerPV_C = ave_viC * ave_iiC;
				if ((PowerPV_C - PowerPV_C_pre) < -deltaP)
				{
					epsilon_C = -epsilon_C;
					vref_C= vref_C_pre + epsilon_C;
				}
				else if ((PowerPV_C - PowerPV_C_pre) > deltaP)
				{
					vref_C= vref_C_pre + epsilon_C;
				}
				else
				{
					vref_C= vref_C_pre;
				}
				if (vref_C <= vPV_min)			vref_C = vPV_min;   //PV reference voltage saturation
				if (vref_C >= vPV_max)			vref_C = vPV_max;

			}

	PowerPV_A_pre = PowerPV_A;
	PowerPV_B_pre = PowerPV_B;
	PowerPV_C_pre = PowerPV_C;
}
}

float LowPassFilter(float32 PreOut, float32 Input, float32 CutFre)
{
  float OutputFilterInc;
  float OutputFilter;

  OutputFilterInc = 2 * 3.14 * CutFre * Ts;
  OutputFilter = (Input * OutputFilterInc + PreOut) / (1.0 + OutputFilterInc);

  return OutputFilter;
}




