// Filename:            DevInit.c
//
// Description:	        Initialization code for Hwi 
//
// Version:             1.0
//
// Target:              TMS320F28379D
//
// Author:              David Romalo
//
// Date:                19Oct2021

// Edited by: William Lau (WL)
// Date: Dec. 01, 2021

#include <Headers/F2837xD_device.h>

extern void DelayUs(Uint16);

void DeviceInit(void)
{
EALLOW;
    //---------------------------------------------------------------
    // INITIALIZE A-D
    //---------------------------------------------------------------
    CpuSysRegs.PCLKCR13.bit.ADC_A = 1; //enable A-D clock for ADC-A
    CpuSysRegs.PCLKCR13.bit.ADC_B = 1; //enable A-D clock for ADC-B	// WL
    AdcaRegs.ADCCTL2.bit.PRESCALE = 0xf; // inputclk/8.5
    AdcbRegs.ADCCTL2.bit.PRESCALE = 0xf; // inputclk/8.5	// WL
    AdcaRegs.ADCCTL1.bit.ADCPWDNZ = 1;  //turn on ADC
    AdcbRegs.ADCCTL1.bit.ADCPWDNZ = 1;  //turn on ADC	// WL

    //generate INT pulse on end of conversion:
    AdcaRegs.ADCCTL1.bit.INTPULSEPOS = 1;
    AdcbRegs.ADCCTL1.bit.INTPULSEPOS = 1;	// WL

    //wait 1 ms after power-up before using the ADC:
    DelayUs(1000);

    AdcaRegs.ADCSOC0CTL.bit.TRIGSEL = 2; //trigger source = CPU1 Timer 1
    AdcaRegs.ADCSOC0CTL.bit.CHSEL = 14; //set SOC0 to sample ADC14	// WL
    AdcaRegs.ADCSOC0CTL.bit.ACQPS = 49; //set SOC0 window to 139 SYSCLK cycles	// WL

    AdcaRegs.ADCSOC1CTL.bit.TRIGSEL = 2; //trigger source = CPU1 Timer 1	// WL
    AdcaRegs.ADCSOC1CTL.bit.CHSEL = 0; //set SOC1 to sample ADC0	// WL
    AdcaRegs.ADCSOC1CTL.bit.ACQPS = 49; //set SOC1 window to 139 SYSCLK cycles	// WL

    AdcaRegs.ADCSOC2CTL.bit.TRIGSEL = 2; //trigger source = CPU1 Timer 1	// WL
    AdcaRegs.ADCSOC2CTL.bit.CHSEL = 2; //set SOC2 to sample ADC2	// WL
    AdcaRegs.ADCSOC2CTL.bit.ACQPS = 49; //set SOC2 window to 139 SYSCLK cycles	// WL

    AdcbRegs.ADCSOC2CTL.bit.TRIGSEL = 2; //trigger source = CPU1 Timer 1	// WL
    AdcbRegs.ADCSOC2CTL.bit.CHSEL = 2; //set SOC2 to sample ADC2	// WL
    AdcbRegs.ADCSOC2CTL.bit.ACQPS = 49; //set SOC2 window to 139 SYSCLK cycles	// WL

    AdcaRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //connect interrupt ADCINT1 to EOC0
    AdcbRegs.ADCINTSEL1N2.bit.INT1SEL = 0; //connect interrupt ADCINT1 to EOC0	// WL
    AdcaRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable interrupt ADCINT1
    AdcbRegs.ADCINTSEL1N2.bit.INT1E = 1; //enable interrupt ADCINT1	// WL
EDIS;
}
