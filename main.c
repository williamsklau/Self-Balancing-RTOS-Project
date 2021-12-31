// Filename:            main.c
//
// Description:         Contains main, HWI, SWI, TSK, and IDLE threads.
//
// Target:              TMS320F28379D
//
// Author:              WL, MG
//
// Date:                Dec. 01, 2021

//defines:
// WL
#define xdc__strict //suppress typedef warnings
#define VREFHI 3.0

//includes:
// WL
#include <xdc/std.h>
#include <xdc/runtime/System.h>
#include <ti/sysbios/BIOS.h>
#include <ti/sysbios/knl/Swi.h>
#include <ti/sysbios/knl/Task.h>
#include <ti/sysbios/knl/Semaphore.h>

// MG
#include "driverlib.h"
#include "device.h"
#include "board.h"

// WL
#include <Headers/F2837xD_device.h>
extern const Task_Handle Tsk0;
extern const Semaphore_Handle semPID;

//function prototypes:
// WL
extern void DeviceInit(void);
extern const Swi_Handle Swi0; //Swi handle defined in .cfg file:

// MG
EPWM_SignalParams pwmSignal =
            {20000, 0.9f, 0.1f, true, DEVICE_SYSCLK_FREQ, SYSCTL_EPWMCLK_DIV_2,
            EPWM_COUNTER_MODE_UP_DOWN, EPWM_CLOCK_DIVIDER_1,
            EPWM_HSCLOCK_DIVIDER_1};

// WL
// global variables:
static int buffer_size = 100;	// size of accel averaging
static int buffer_size_pot = 300;	// size of pot averaging
float accel_buffer [100];	// 
float kp_buffer [300];
float ki_buffer [300];
float kd_buffer [300];
int index_accel_buffer = 0;
int index_pot_buffer = 0;

float accel_voltage;
float accel_filtered_reading;
float kp_reading;
float ki_reading;
float kd_reading;
float kp_filtered_reading;
float ki_filtered_reading;
float kd_filtered_reading;
float kp_maxgain = 1000.0;
float ki_maxgain = 20.0;
float kd_maxgain = 10.0;

float kp_output;
float ki_output = 0;
float kd_output;
float output;
float error;
float errorPrevious = 0;
float negDuty, posDuty;

// experimentally found. will fluctuate with time/temperature. 
// set value before execution
float accel_pos1G = 1.82;
float accel_neg1G = 1.22;
float accel_zeroG = 1.55;
float stiction = 10;

/* ======== main ======== */
// Author: WL, MG
Int main()
{ 
    //initialization:
     DeviceInit(); //initialize ADC peripherals  // WL

    
    // Initializes device clock and peripherals
    Device_init(); // MG
    // Initializes the GPIO pins for ePWM1 and control outputs.
    Board_init(); // MG
    // Disable sync(Freeze clock to PWM as well)
    SysCtl_disablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC); // MG
    // Configuring ePWM module for desired frequency and duty
    EPWM_configureSignal(myEPWM1_BASE, &pwmSignal); // MG
    // Enable sync and clock to PWM
    SysCtl_enablePeripheral(SYSCTL_PERIPH_CLK_TBCLKSYNC); // MG
    // Enable global Interrupts and higher priority real-time debug events:
    EINT;  // Enable Global interrupt INTM // MG
    ERTM;  // Enable Global realtime interrupt DBGM // MG


    //jump to RTOS (does not return):
    BIOS_start(); // WL
    return(0); // WL
}

/* ======== IdlePotFilter ======== */
// Author: WL
// Calculates average for the Proportional, Integral, and Derivative potentiometer
Void IdlePotFilter(Void){
   //CPU utilization measurement
   GpioDataRegs.GPASET.bit.GPIO9 = 1;

	// insert PID pot value if buffer is not full
   if(index_pot_buffer < buffer_size_pot) {
       kp_buffer[index_pot_buffer] = kp_reading;
       ki_buffer[index_pot_buffer] = ki_reading;
       kd_buffer[index_pot_buffer] = kd_reading;
       index_pot_buffer++;
   }
   // if buffer is full, find average of PID pot values 
   else{
       float sum_kp = 0.0;
       float sum_ki = 0.0;
       float sum_kd = 0.0;
       int count;

       for(count = 0; count < buffer_size_pot; count++){
           sum_kp += kp_buffer[count];
           sum_ki += ki_buffer[count];
           sum_kd += kd_buffer[count];
       }
       kp_filtered_reading = sum_kp / buffer_size_pot;
       ki_filtered_reading = sum_ki / buffer_size_pot;
       kd_filtered_reading = sum_kd / buffer_size_pot;
       index_pot_buffer = 0;
   }

	//CPU utilization measurement
	GpioDataRegs.GPACLEAR.bit.GPIO9 = 1;
}

/* ======== TskPID ======== */
// Author: WL, MG
// Calculates PID output to motor
Void TskPID(Void){
    while(TRUE){
        Semaphore_pend(semPID, BIOS_WAIT_FOREVER);	// WL
        //CPU utilization measurement
        GpioDataRegs.GPASET.bit.GPIO8 = 1;	// WL
		
		// find error from setpoint
        error = accel_zeroG - accel_voltage;		// WL
        // Proportional calculation
        kp_output = kp_filtered_reading * error;	// WL
        // Integral calculation
        ki_output += ki_filtered_reading * error;	// WL
        // Anti-windup limit
        if(ki_output > 50)	// WL
            ki_output = 50;	// WL
        if(ki_output < -50)	// WL
            ki_output = -50;	// WL
        // Derivative calculation
        kd_output = kd_filtered_reading * (error - errorPrevious);	// WL
        errorPrevious = error;	// WL

		// Summate PID
        output = (int)(kp_output + ki_output + kd_output);	// WL

        // Limit motor output to -100% <-> +100%
        if(output > 100.0)	// WL
            output = 100.0;	// WL
        if(output < -100.0)	// WL
            output = -100.0;	// WL

		// Adjust output for stiction/motor stalling
        if(output < stiction && output > 0)	// WL
            output = stiction;	// WL
        if(output > -(stiction) && output < 0)	// WL
            output = -stiction;	// WL

        if(output >= 0){	// WL
            posDuty = output / 100;	// WL
            negDuty = (100 - output) /100;	// WL

            GpioDataRegs.GPASET.bit.GPIO2 = 1;	// MG
            GpioDataRegs.GPACLEAR.bit.GPIO3 = 1;	// MG
            GpioDataRegs.GPACLEAR.bit.GPIO4 = 1;	// MG
            GpioDataRegs.GPASET.bit.GPIO5 = 1;	// MG
        }else if(output < 0){	// WL
            posDuty = (-1 * output) / 100;	// WL
            negDuty = (100 + output) / 100;	// WL

            GpioDataRegs.GPACLEAR.bit.GPIO2 = 1;	// MG
            GpioDataRegs.GPASET.bit.GPIO3 = 1;	// MG
            GpioDataRegs.GPASET.bit.GPIO4 = 1;	// MG
            GpioDataRegs.GPACLEAR.bit.GPIO5 = 1;	// MG
        }

        // set motor speed
        pwmSignal.dutyValA = posDuty;	// MG, WL
        pwmSignal.dutyValB = negDuty;	// MG, WL
        EPWM_configureSignal(myEPWM1_BASE, &pwmSignal);	// MG

        GpioDataRegs.GPACLEAR.bit.GPIO8 = 1;	// WL
    }
}

/* ======== SwiAccelFilter ======== */
// Author: WL
// Averages acceleration reading
Void SwiAccelFilter(Void)
{
    //CPU utilization measurement
    GpioDataRegs.GPASET.bit.GPIO7 = 1;

    // is buffer is full?
    if(index_accel_buffer < buffer_size){
        accel_buffer[index_accel_buffer] = accel_voltage;
        index_accel_buffer++;
    }
    // perform averaging filter on data if buffer full
    else{
        float sum = 0;
        int count;
        for(count = 0; count < buffer_size; count++)
            sum += accel_buffer[count];
        accel_filtered_reading = sum / buffer_size;

        Semaphore_post(semPID);
        index_accel_buffer = 0;
    }

	//CPU utilization measurement
    GpioDataRegs.GPACLEAR.bit.GPIO7 = 1;
}

/* ======== HwiADC ======== */
// Author: WL
// Retrieves ADC reading
Void HwiADC(Void)
{
    //CPU utilization measurement
    GpioDataRegs.GPASET.bit.GPIO6 = 1;

    //read acceleration ADC value and convert to voltage
    accel_voltage = (VREFHI / 0x0FFF) * AdcaResultRegs.ADCRESULT0;

    //read potentiometer and convert to gain value
    kp_reading = ((kp_maxgain / 0x0FFF) * AdcaResultRegs.ADCRESULT1);
    ki_reading = ((ki_maxgain / 0x0FFF) * AdcaResultRegs.ADCRESULT2);
    kd_reading = ((kd_maxgain / 0x0FFF) * AdcbResultRegs.ADCRESULT2);

    Swi_post(Swi0); // Post SwiAccelFilter to filter accel value
    AdcaRegs.ADCINTFLGCLR.bit.ADCINT1 = 1; //clear interrupt flag

	//CPU utilization measurement
    GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;
}
