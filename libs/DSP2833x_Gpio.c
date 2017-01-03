// TI File $Revision: /main/1 $
// Checkin $Date: August 18, 2006   13:46:25 $
//###########################################################################
//
// FILE:	DSP2833x_Gpio.c
//
// TITLE:	DSP2833x General Purpose I/O Initialization & Support Functions.
//
//###########################################################################
// $TI Release: DSP2833x/DSP2823x C/C++ Header Files V1.31 $
// $Release Date: August 4, 2009 $
//###########################################################################

#include "DSP2833x_Device.h"     // DSP2833x Headerfile Include File
#include "DSP2833x_Examples.h"   // DSP2833x Examples Include File

//---------------------------------------------------------------------------
// InitGpio: 
//---------------------------------------------------------------------------
// This function initializes the Gpio to a known (default) state.
//
// For more details on configuring GPIO's as peripheral functions,
// refer to the individual peripheral examples and/or GPIO setup example. 
void InitGpio(void)
{
   EALLOW;
   
   // Each GPIO pin can be: 
   // a) a GPIO input/output
   // b) peripheral function 1
   // c) peripheral function 2
   // d) peripheral function 3
   // By default, all are GPIO Inputs 
   GpioCtrlRegs.GPAMUX1.all = 0x0000;     // GPIO functionality GPIO0-GPIO15
   GpioCtrlRegs.GPAMUX2.all = 0x0000;     // GPIO functionality GPIO16-GPIO31
//   GpioCtrlRegs.GPBMUX1.all = 0x0000;     // GPIO functionality GPIO32-GPIO39
//   GpioCtrlRegs.GPBMUX2.all = 0x0000;     // GPIO functionality GPIO48-GPIO63
//   GpioCtrlRegs.GPCMUX1.all = 0x0000;     // GPIO functionality GPIO64-GPIO79
//   GpioCtrlRegs.GPCMUX2.all = 0x0000;     // GPIO functionality GPIO80-GPIO95

//   GpioCtrlRegs.GPADIR.all = 0x0000;      // GPIO0-GPIO31 are inputs
//   GpioCtrlRegs.GPBDIR.all = 0x0000;      // GPIO32-GPIO63 are inputs   
//   GpioCtrlRegs.GPCDIR.all = 0x0000;      // GPI064-GPIO95 are inputs

   // Each input can have different qualification
   // a) input synchronized to SYSCLKOUT
   // b) input qualified by a sampling window
   // c) input sent asynchronously (valid for peripheral inputs only)
//   GpioCtrlRegs.GPAQSEL1.all = 0x0000;    // GPIO0-GPIO15 Synch to SYSCLKOUT 
//   GpioCtrlRegs.GPAQSEL2.all = 0x0000;    // GPIO16-GPIO31 Synch to SYSCLKOUT
//   GpioCtrlRegs.GPBQSEL1.all = 0x0000;    // GPIO32-GPIO39 Synch to SYSCLKOUT 
//   GpioCtrlRegs.GPBQSEL2.all = 0x0000;    // GPIO48-GPIO63 Synch to SYSCLKOUT 

   // Pull-ups can be enabled or disabled. 
   GpioCtrlRegs.GPAPUD.all = 0x0000;      // Pullup's enabled GPIO0-GPIO31
   GpioCtrlRegs.GPBPUD.all = 0x0000;      // Pullup's enabled GPIO32-GPIO63
//   GpioCtrlRegs.GPCPUD.all = 0x0000;      // Pullup's enabled GPIO64-GPIO79

   //GpioCtrlRegs.GPAPUD.all = 0xFFFF;    // Pullup's disabled GPIO0-GPIO31
   //GpioCtrlRegs.GPBPUD.all = 0xFFFF;    // Pullup's disabled GPIO32-GPIO34
   //GpioCtrlRegs.GPCPUD.all = 0xFFFF     // Pullup's disabled GPIO64-GPIO79


	GpioCtrlRegs.GPADIR.out.output = 0x1FFF;  // GPIO0 - GPIO12 as output

	GpioCtrlRegs.GPADIR.bit.GPIO14		= 1;  		////LED RUN

	GpioCtrlRegs.GPADIR.bit.GPIO15 		= 1;  		//LED STOP

	GpioCtrlRegs.GPAMUX2.bit.GPIO26 	= 0;  		//LED BLINK
	GpioCtrlRegs.GPADIR.bit.GPIO26 		= 1;  		//Output
	GpioCtrlRegs.GPAPUD.bit.GPIO26 		= 0; 		// Enable pull-up

//	GpioCtrlRegs.GPAMUX2.bit.GPIO24 	= 0;  		//check total time for T0 interrupt
//	GpioCtrlRegs.GPADIR.bit.GPIO24 		= 1;  		//Output
//	GpioCtrlRegs.GPAPUD.bit.GPIO24 		= 0; 		// Enable pull-up

	GpioCtrlRegs.GPAMUX2.bit.GPIO16 	= 0;  		//FAULT
	GpioCtrlRegs.GPADIR.bit.GPIO16 		= 0;  		//Input
	GpioCtrlRegs.GPAPUD.bit.GPIO16 		= 0; 		// Enable pull-up

	GpioCtrlRegs.GPBMUX2.bit.GPIO61  	= 0;	  // GPIO61 = input: Start button
	GpioCtrlRegs.GPBDIR.bit.GPIO61   	= 0;
	GpioCtrlRegs.GPBPUD.bit.GPIO61   	= 0;

	GpioCtrlRegs.GPBMUX2.bit.GPIO60  	= 0;	// GPIO60 = input: Stop button
	GpioCtrlRegs.GPBDIR.bit.GPIO60   	= 0;
	GpioCtrlRegs.GPBPUD.bit.GPIO60   	= 0;

	GpioCtrlRegs.GPADIR.bit.GPIO10 		= 1;	// relay in/DC
	GpioCtrlRegs.GPADIR.bit.GPIO11 		= 1;    // relay out/Bat
	GpioCtrlRegs.GPADIR.bit.GPIO12 		= 1;	// enable/disable all PWM and relay

    GpioCtrlRegs.GPAPUD.bit.GPIO0 		= 0;    // Enable pull-up on GPIO0 (EPWM1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO1 		= 0;    // Enable pull-up on GPIO1 (EPWM1B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 		= 1;   	// Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 		= 1;   	// Configure GPIO1 as EPWM1B

    GpioCtrlRegs.GPAPUD.bit.GPIO2 = 0;    // Enable pull-up on GPIO2 (EPWM2A)
    GpioCtrlRegs.GPAPUD.bit.GPIO3 = 0;    // Enable pull-up on GPIO3 (EPWM3B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO2 = 1;   // Configure GPIO2 as EPWM2A
    GpioCtrlRegs.GPAMUX1.bit.GPIO3 = 1;   // Configure GPIO3 as EPWM2B

    GpioCtrlRegs.GPAPUD.bit.GPIO4 = 0;    // Enable pull-up on GPIO4 (EPWM3A)
    GpioCtrlRegs.GPAPUD.bit.GPIO5 = 0;    // Enable pull-up on GPIO5 (EPWM3B)
    GpioCtrlRegs.GPAMUX1.bit.GPIO4 = 1;   // Configure GPIO4 as EPWM3A
    GpioCtrlRegs.GPAMUX1.bit.GPIO5 = 1;   // Configure GPIO5 as EPWM3B

//    GpioCtrlRegs.GPAPUD.bit.GPIO6 		= 0;    // Enable pull-up on GPIO2 (EPWM4A)
//    GpioCtrlRegs.GPAPUD.bit.GPIO7 		= 0;    // Enable pull-up on GPIO3 (EPWM4B)
//    GpioCtrlRegs.GPAMUX1.bit.GPIO6 		= 1;   	// Configure GPIO2 as EPWM4A
//    GpioCtrlRegs.GPAMUX1.bit.GPIO7 		= 1;   	// Configure GPIO3 as EPWM4B
//
//    GpioCtrlRegs.GPAPUD.bit.GPIO8 		= 0;    // Enable pull-up on GPIO8 (EPWM5A)
//    GpioCtrlRegs.GPAPUD.bit.GPIO9 		= 0;    // Enable pull-up on GPIO9 (EPWM5B)
//    GpioCtrlRegs.GPAMUX1.bit.GPIO8 		= 1;   	// Configure GPIO8 as EPWM5A
//    GpioCtrlRegs.GPAMUX1.bit.GPIO9 		= 1;   	// Configure GPIO9 as EPWM5B

// CTO ==============

GpioCtrlRegs.GPBPUD.bit.GPIO55 = 0;   // Enable pullup on GPIO55		// precharging relay output (DC side)
//GpioDataRegs.GPBSET.bit.GPIO55 = 1;   // Load output latch
GpioCtrlRegs.GPBMUX2.bit.GPIO55 = 0;  // GPIO55 = GPIO55
GpioCtrlRegs.GPBDIR.bit.GPIO55 = 1;   // GPIO55 = output

GpioCtrlRegs.GPCPUD.bit.GPIO84 = 0;   // Enable pullup on GPIO55		// precharging relay input
//GpioDataRegs.GPCSET.bit.GPIO84 = 1;   // Load output latch
GpioCtrlRegs.GPCMUX2.bit.GPIO84 = 0;  // GPIO55 = GPIO55
GpioCtrlRegs.GPCDIR.bit.GPIO84 = 1;   // GPIO55 = output

GpioCtrlRegs.GPAPUD.bit.GPIO25 = 0;   // Enable pullup on GPIO55		// Output Contactor
//GpioDataRegs.GPASET.bit.GPIO25 = 1;   // Load output latch
GpioCtrlRegs.GPAMUX2.bit.GPIO25 = 0;  // GPIO55 = GPIO55
GpioCtrlRegs.GPADIR.bit.GPIO25 = 1;   // GPIO55 = output

GpioCtrlRegs.GPAPUD.bit.GPIO24 = 0;   // Enable pullup on GPIO55		// Input Contactor
//GpioDataRegs.GPASET.bit.GPIO24 = 1;   // Load output latch
GpioCtrlRegs.GPAMUX2.bit.GPIO24 = 0;  // GPIO55 = GPIO55
GpioCtrlRegs.GPADIR.bit.GPIO24 = 1;   // GPIO55 = output

GpioCtrlRegs.GPAPUD.bit.GPIO13 = 0;   // Enable pullup on GPIO55		// reset all relays (1 = off all relays)
//GpioDataRegs.GPASET.bit.GPIO13 = 1;   // Load output latch
GpioCtrlRegs.GPAMUX1.bit.GPIO13 = 0;  // GPIO55 = GPIO55
GpioCtrlRegs.GPADIR.bit.GPIO13 = 1;   // GPIO55 = output


GpioDataRegs.GPBCLEAR.bit.GPIO55 = 1;
GpioDataRegs.GPCCLEAR.bit.GPIO84 = 1;
GpioDataRegs.GPACLEAR.bit.GPIO25 = 1;		//RELAY2
GpioDataRegs.GPACLEAR.bit.GPIO24 = 1;		//RELAY1
GpioDataRegs.GPACLEAR.bit.GPIO13 = 1;		//RESET
//===================

	GpioDataRegs.GPADAT.out.output 		= 0;
   EDIS;

}


//===========================================================================
// End of file.
//===========================================================================
