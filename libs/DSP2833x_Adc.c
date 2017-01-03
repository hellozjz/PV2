// TI File $Revision: /main/5 $
// Checkin $Date: October 23, 2007   13:34:09 $
//###########################################################################
//
// FILE:	DSP2833x_Adc.c
//
// TITLE:	DSP2833x ADC Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2833x/DSP2823x C/C++ Header Files V1.31 $
// $Release Date: August 4, 2009 $
//###########################################################################

//#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
//#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

#include "DSP28x_Project.h"
#include "IQmathlib.h"

#define ADC_usDELAY  5000L

#define ADC_MODCLK 	0x0003 			// HSPCLK = SYSCLKOUT/2*ADC_MODCLK2 = 150/(2*3)   = 25.0 MHz
#define ADC_CKPS   	0x1   			// ADC module clock = HSPCLK/2*ADC_CKPS   = 25.0MHz/(1*2) = 12.5MHz
#define ADC_SHCLK  	0x6   			// S/H width in ADC module periods                        = 16 ADC clocks
//---------------------------------------------------------------------------
// InitAdc:
//---------------------------------------------------------------------------
// This function initializes ADC to a known state.
//
void InitAdc(void)
{
    extern void DSP28x_usDelay(Uint32 Count);


    // *IMPORTANT*
	// The ADC_cal function, which  copies the ADC calibration values from TI reserved
	// OTP into the ADCREFSEL and ADCOFFTRIM registers, occurs automatically in the
	// Boot ROM. If the boot ROM code is bypassed during the debug process, the
	// following function MUST be called for the ADC to function according
	// to specification. The clocks to the ADC MUST be enabled before calling this
	// function.
	// See the device data manual and/or the ADC Reference
	// Manual for more information.

	    EALLOW;
		SysCtrlRegs.PCLKCR0.bit.ADCENCLK = 1;
		ADC_cal();
		EDIS;




    // To powerup the ADC the ADCENCLK bit should be set first to enable
    // clocks, followed by powering up the bandgap, reference circuitry, and ADC core.
    // Before the first conversion is performed a 5ms delay must be observed
	// after power up to give all analog circuits time to power up and settle

    // Please note that for the delay function below to operate correctly the
	// CPU_RATE define statement in the DSP2833x_Examples.h file must
	// contain the correct CPU clock period in nanoseconds.

    AdcRegs.ADCTRL3.all = 0x00E0;  // Power up bandgap/reference/ADC circuits
    DELAY_US(ADC_usDELAY);         // Delay before converting ADC channels

    // Specific ADC setup for this example:
       AdcRegs.ADCTRL1.bit.ACQ_PS   	= ADC_SHCLK;
       AdcRegs.ADCTRL3.bit.ADCCLKPS 	= ADC_CKPS;
       AdcRegs.ADCTRL1.bit.SEQ_CASC 	= 0;        // 0: saparated (8 channels), 1: Cascaded mode (16 channels)
       AdcRegs.ADCTRL1.bit.CONT_RUN 	= 0;       	// 0: Setup non-continuous run, 1: continuous conversion mode
       AdcRegs.ADCTRL1.bit.SEQ_OVRD 	= 0;       	// Disable Sequencer override feature
       AdcRegs.ADCTRL2.bit.RST_SEQ1 = 1;        //Reset Sequencer1 to state CONV00
       AdcRegs.ADCTRL3.bit.SMODE_SEL = 0;

       AdcRegs.ADCMAXCONV.bit.MAX_CONV1 = 0x0007;	// Setup 8 conv's on SEQ1

//EEE
//       AdcRegs.ADCCHSELSEQ1.bit.CONV00 	= 0x0;		// Setup ADCINA0 as 1st SEQ1
//       AdcRegs.ADCCHSELSEQ1.bit.CONV01 	= 0x1;		// Setup ADCINA1 as 2nd SEQ1
//       AdcRegs.ADCCHSELSEQ1.bit.CONV02 	= 0x2;		// Setup ADCINA2 as 3rd SEQ1
//       AdcRegs.ADCCHSELSEQ1.bit.CONV03 	= 0x3;		// Setup ADCINA3 as 4th SEQ1
//       AdcRegs.ADCCHSELSEQ2.bit.CONV04 	= 0x4;		// Setup ADCINA4 as 5th SEQ1
//       AdcRegs.ADCCHSELSEQ2.bit.CONV05 	= 0x5;		// Setup ADCINA5 as 6th SEQ1
//       AdcRegs.ADCCHSELSEQ2.bit.CONV06 	= 0xE;		// Setup ADCINB6 as 7th SEQ1
//       AdcRegs.ADCCHSELSEQ2.bit.CONV07 	= 0xF;		// Setup ADCINB7 as 8th SEQ1

//CTO
       AdcRegs.ADCCHSELSEQ1.bit.CONV00 	= 0x1;		// ADCINA1 (A1) as 1nd SEQ1		I3
       AdcRegs.ADCCHSELSEQ1.bit.CONV01 	= 0x2;		// ADCINA2 (A2) as 2rd SEQ1		V4
       AdcRegs.ADCCHSELSEQ1.bit.CONV02 	= 0x3;		// ADCINA3 (A3) as 3th SEQ1		V3
       AdcRegs.ADCCHSELSEQ1.bit.CONV03 	= 0x4;		// ADCINA4 (A4) as 4th SEQ1		I2
       AdcRegs.ADCCHSELSEQ2.bit.CONV04 	= 0x5;		// ADCINA5 (A5) as 5th SEQ1		I1
       AdcRegs.ADCCHSELSEQ2.bit.CONV05 	= 0x7;		// ADCINA7 (A7) as 6st SEQ1		V1
       AdcRegs.ADCCHSELSEQ2.bit.CONV06 	= 0x6;		// ADCINA6 (A6) as 7th SEQ1		V2
       AdcRegs.ADCCHSELSEQ2.bit.CONV07 	= 0xF;		// ADCINB7 (B7) as 8th SEQ1		I4


       AdcRegs.ADCTRL2.bit.INT_ENA_SEQ1 = 1;  		// Enable SEQ1 interrupt (every EOS)													//==============
       AdcRegs.ADCTRL2.bit.EPWM_SOCA_SEQ1 = 1; 		// Enable SOCA from ePWM to start SEQ1													//==============
       EPwm4Regs.ETSEL.bit.SOCAEN = 1;                // 1 Enable the ADC Start of Conversion A (EPWMxSOCA) Pulse, Enable EPWMxSOCA pulse.	//==============
       EPwm4Regs.ETSEL.bit.SOCASEL = ET_CTR_ZERO;     //ZERO ´¥·¢ADC_A																		//==============
       EPwm4Regs.ETPS.bit.SOCAPRD = ET_1ST;




}

//===========================================================================
// End of file.
//===========================================================================
