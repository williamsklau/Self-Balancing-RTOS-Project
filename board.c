/*
 * Copyright (c) 2020 Texas Instruments Incorporated - http://www.ti.com
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
// Edited by: Maxwell Gibson (Look for MG)
// Date: November 28, 2021


#include "board.h"

void Board_init()
{
	EALLOW;

	PinMux_init();
	EPWM_init();

	EDIS;
}

void PinMux_init()
{
	// Configure the ePWM output on J4/40 and J4/39 (MG)
	GPIO_setPinConfig(GPIO_0_EPWM1A); // J4/40
	GPIO_setPinConfig(GPIO_1_EPWM1B); // J4/39

	// Configure pins J4/38, J4/37, J4/36, and J4/35 for GPIO operation (MG)
	GPIO_setPinConfig(GPIO_2_GPIO2); // J4/38
	GPIO_setPinConfig(GPIO_3_GPIO3); // J4/37
	GPIO_setPinConfig(GPIO_4_GPIO4); // J4/36
	GPIO_setPinConfig(GPIO_5_GPIO5); // J4/35

	// Configure direction on pins J4/38, J4/37, J4/36, and J4/35 as outputs (MG)
	GPIO_setDirectionMode(2, GPIO_DIR_MODE_OUT);
	GPIO_setDirectionMode(3, GPIO_DIR_MODE_OUT);
	GPIO_setDirectionMode(4, GPIO_DIR_MODE_OUT);
	GPIO_setDirectionMode(5, GPIO_DIR_MODE_OUT);

	// Configure for CPU utilization
    GPIO_setDirectionMode(6, GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(7, GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(8, GPIO_DIR_MODE_OUT);
    GPIO_setDirectionMode(9, GPIO_DIR_MODE_OUT);
}

void EPWM_init(){
	//myEPWM1 initialization
	//myEPWM2 initialization
	//myEPWM3 initialization
}
