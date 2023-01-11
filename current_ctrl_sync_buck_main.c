/**
 * This file implement the sync buck converter
 * Input: 24V
 * Output: 4.8-5.8V 0.2-2A
 * Load: GW Q9LR32.PM-M4N3-XX55-1-180-R18 x9
 * The current sense circuit using LM358 as amplifier, Rsense = 0.03 Ohm, Av = 10
 * ADC0: Vout * 2/3 (V)
 * ADC1: Iout * 0.03 * 30 (V)
 * ePWM1A: Upper MOSFET
 * ePWM1B: Lower MOAFET
 * Deadtime: 0.5uS
 * Duty range:
 * MOSFET: 030N10N N-MOSFET
 * Inductor: 560uH/2A
 *
 * Author: Chun-Lin Chen johnson35762@gmail.com
 * Date: 2021-07-05
 * License: GPL-3.0
 *
 */

#include "F28x_Project.h"
#include "init.h"
#include <stdio.h>
#include "pid.h"

#define __PWM_FREQ    5000            // PWM period = 50uS

// -----------------------------------------------------------------------------------------------
// __Flash_RAM = 1 , the code will storage into flash memory
// __Flash_RAM = 0 , the code will storage into RAM
// Set project property to corresponding cmd file.

#define __FLASH_RAM 1
#if __Flash_RAM

extern Uint16 RamfuncsLoadStart;
extern Uint16 RamfuncsLoadEnd;
extern Uint16 RamfuncsRunStart;

#endif

// Variables -------------------------------------------------------------------------------------
PID_STRUCT *psPID;

unsigned int voltBuff = 0;  // voltage buffer
unsigned int curBuff = 0;   // current buffer
unsigned char direction = 0;

// Function Prototypes ----------------------------------------------------------------------------
__interrupt void cpuTimer0ISR(void);
__interrupt void adcA1ISR(void);

// Main --------------------------------------------------------------------------------------------
void main(void)
{
    psPID = Init_pid();       // Initialize PID controller parameter

    // Decide where the code will store into, RAM or Flash memory
#if __FLASH_RAM

    memcpy(&RamfuncsRunStart, &RamfuncsLoadStart, (size_t) &RamfuncsLoadSize);

#endif

    // Initialize device clock and peripherals
    InitSysCtrl();
    InitGPIO();
    InitPIE();
    InitCPUTimer();
    InitEPWM(__PWM_FREQ);
    InitADC();
    InitADCSOC();

    // Map ISR functions

    EALLOW;
    PieVectTable.TIMER0_INT = &cpuTimer0ISR;
    PieVectTable.ADCA1_INT = &adcA1ISR;     // Function for ADCA interrupt 1
    EDIS;

    EPwm1Regs.CMPA.bit.CMPA = 2500;

    // Enable Global Interrupt (INTM) and realtime interrupt (DBGM)
    EINT;
    ERTM;

    // Sync ePWM
    EALLOW;
    CpuSysRegs.PCLKCR0.bit.TBCLKSYNC = 1;

    EPwm1Regs.ETSEL.bit.SOCAEN = 1;    // Enable SOCA
    EPwm1Regs.TBCTL.bit.CTRMODE = 0;   // Unfreeze, and enter up count mode

    EDIS;

    // Loop Forever
    while (1)
    {
        if (psPID->RefSetPoint > 2400)  // Output voltage upper limit
            direction = 1;
        if (psPID->RefSetPoint < 1900)  // Output voltage lower limit
            direction = 0;
        if (direction)
            psPID->RefSetPoint -= 10;   //Swap the output voltage
        else
            psPID->RefSetPoint += 10;
        DELAY_US(50000);

    }
}

__interrupt void cpuTimer0ISR(void)
{

    // Toggle LED
    GpioDataRegs.GPATOGGLE.bit.GPIO23 = 1;

    // Acknowledge this interrupt to receive more interrupts from group 1
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}

// adcA1ISR - ADC A Interrupt 1 ISR
__interrupt void adcA1ISR(void)
{
    GpioDataRegs.GPBTOGGLE.bit.GPIO34 = 1;
    curBuff = AdcaResultRegs.ADCRESULT1;

    psPID->Feedback_AdcPoint = AdcaResultRegs.ADCRESULT0; // Get feedback voltage
    // curBuff = AdcaResultRegs.ADCRESULT1;     // Get feedback current

    pid_process(psPID);     // Process the PD controller
    EPwm1Regs.CMPA.bit.CMPA = psPID->Output; // Set the duty cycle

    // Clear the interrupt flag and issue ACK
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1;
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;

}

