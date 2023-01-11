/*
 * init.c
 *
 *  Created on: 2021¦~6¤ë11¤é
 *      Author: Johnson
 */

#include "F28x_Project.h"

#include <stdio.h>

// GPIO initialization

#define LED4    23
#define LED5    34

void InitGPIO(void)
{
    //
    // Initialize GPIO and configure the GPIO pin as a push-pull output
    //
    InitGpio();
    EALLOW;
    // Disable internal pull-up for the selected output pins for reduced power consumption
    // Pull-ups can be enabled or disabled by the user.
    GpioCtrlRegs.GPAPUD.bit.GPIO0 = 1;    // Disable pull-up on GPIO0 (EPWM1A)
    GpioCtrlRegs.GPAPUD.bit.GPIO1 = 1;    // Disable pull-up on GPIO1 (EPWM1B)

    // Configure EPWM-1 pins using GPIO regs This specifies which of the possible GPIO pins will be EPWM1 functional pins.
    GpioCtrlRegs.GPAMUX1.bit.GPIO0 = 1;   // Configure GPIO0 as EPWM1A
    GpioCtrlRegs.GPAMUX1.bit.GPIO1 = 1;   // Configure GPIO1 as EPWM1B

    GPIO_SetupPinMux(LED4, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(LED4, GPIO_OUTPUT, GPIO_PUSHPULL);
    GPIO_SetupPinMux(LED5, GPIO_MUX_CPU1, 0);
    GPIO_SetupPinOptions(LED5, GPIO_OUTPUT, GPIO_PUSHPULL);

    EDIS;

}

// PIE initialization

void InitPIE(void)
{
    //
    // Initialize PIE and clear PIE registers. Disables CPU interrupts.
    //
    DINT;
    InitPieCtrl();
    IER = 0x0000;
    IFR = 0x0000;

    //
    // Initialize the PIE vector table with pointers to the shell Interrupt
    // Service Routines (ISR).
    //
    InitPieVectTable();
    EINT;

}

// initADC - Function to configure and power up ADCA.

void InitADC(void)
{
    // Setup VREF as internal
    // In order to set ADCC REF=3.3V, we need to setup both ADCA,ADCB,ADCC to 3v3.
    SetVREF(ADC_ADCA, ADC_INTERNAL, ADC_VREF3P3);


    EALLOW;

    // Set ADCCLK divider to /4
    AdcaRegs.ADCCTL2.bit.PRESCALE = 6;

    // Set pulse positions to late
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;

    // Power up the ADC
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;

    EDIS;

    DELAY_US(1000);
}

void InitADCSOC(void)
{
    // Select the channels to convert and the end of conversion flag
    EALLOW;

    // ADCASOC0
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 0;     // SOC0 will convert pin A1
                                           // 0:A0  1:A1  2:A2  3:A3
                                           // 4:A4   5:A5   6:A6   7:A7
                                           // 8:A8   9:A9   A:A10  B:A11
                                           // C:A12  D:A13  E:A14  F:A15
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 9;     // Sample window is 10 SYSCLK cycles
    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 5;   // Trigger on CPU Timer0 TINT0n

    // ADCASOC1
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 1;     // SOC1 will convert pin A1
                                           // 0:A0  1:A1  2:A2  3:A3
                                           // 4:A4   5:A5   6:A6   7:A7
                                           // 8:A8   9:A9   A:A10  B:A11
                                           // C:A12  D:A13  E:A14  F:A15
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 9;     // Sample window is 10 SYSCLK cycles
    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 5;   // Trigger on CPU Timer0 TINT0n

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; // End of SOC0 will set INT1 flag
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1;   // Enable INT1 flag
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; // Make sure INT1 flag is cleared

    EDIS;

}

void InitCPUTimer(void)
{
    DINT;
    //
    // Initialize the Device Peripheral. For this example, only initialize the
    // Cpu Timers.
    //
    InitCpuTimers();
    //
    // Configure CPU-Timer 0, 1, and 2 to interrupt every second:
    // 100MHz CPU Freq, 1 second Period (in uSeconds)
    //
    ConfigCpuTimer(&CpuTimer0, 100, 50000);

    //
    // To ensure precise timing, use write-only instructions to write to the
    // entire register. Therefore, if any of the configuration bits are changed
    // in ConfigCpuTimer and InitCpuTimers, the below settings must also be
    // be updated.
    //
    CpuTimer0Regs.TCR.all = 0x4000;

    //
    // Enable CPU int1 which is connected to CPU-Timer 0, CPU int13
    // which is connected to CPU-Timer 1, and CPU int 14, which is connected
    // to CPU-Timer 2
    //
    IER |= M_INT1;

    //
    // Enable TINT0 in the PIE: Group 1 interrupt 7
    //
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    PieCtrlRegs.PIEIER1.bit.INTx1 = 1;
    EINT;

}

void InitEPWM(unsigned int period)
{

    EALLOW;

    EPwm1Regs.TBCTL.bit.PRDLD = TB_SHADOW;  // set Immediate load
    EPwm1Regs.TBPRD = period;           // PWM frequency = (1 / period) * 100MHz
    EPwm1Regs.CMPA.bit.CMPA = period >> 1;   // set duty 50% initially
    EPwm1Regs.CMPB.bit.CMPB = period >> 1;   // set duty 50% initially
    EPwm1Regs.CMPB.all |= (1 << 8);         // initialize HRPWM extension
    EPwm1Regs.TBPHS.all = 0;
    EPwm1Regs.TBCTR = 0;

    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UP;

    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm1Regs.TBCTL.bit.SYNCOSEL = TB_SYNC_DISABLE;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = TB_DIV1;
    EPwm1Regs.TBCTL.bit.CLKDIV = TB_DIV1;
    EPwm1Regs.TBCTL.bit.FREE_SOFT = 11;

    EPwm1Regs.CMPCTL.bit.LOADAMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.LOADBMODE = CC_CTR_ZERO;
    EPwm1Regs.CMPCTL.bit.SHDWAMODE = CC_SHADOW;
    EPwm1Regs.CMPCTL.bit.SHDWBMODE = CC_SHADOW;

    EPwm1Regs.AQCTLA.bit.ZRO = AQ_SET;      // PWM toggle high/low
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;
    EPwm1Regs.AQCTLB.bit.ZRO = AQ_CLEAR;
    EPwm1Regs.AQCTLB.bit.CBU = AQ_SET;

    EPwm1Regs.DBCTL.bit.OUT_MODE = 3;       // Set DeadBend register
    EPwm1Regs.DBCTL.bit.POLSEL = 2;
    EPwm1Regs.DBRED.bit.DBRED = 250;        // raise dead time = 0.25uS
    EPwm1Regs.DBFED.bit.DBFED = 250;        // fall dead time = 0.25uS

    EPwm1Regs.ETSEL.bit.SOCAEN = 0;     // Disable SOC on A group
    EPwm1Regs.ETSEL.bit.SOCASEL = 4;    // Select SOC on up-count
    EPwm1Regs.ETPS.bit.SOCAPRD = 1;     // Generate pulse on 1st event

    EPwm1Regs.TBCTL.bit.CTRMODE = 3;    // Freeze counter

    EDIS;

}
