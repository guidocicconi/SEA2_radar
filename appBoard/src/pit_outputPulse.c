/*
###############################################################################
#
# Copyright 2023, Guido Cicconi
# All rights reserved
#
# This file is part of EmbeddedFirmware.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
#    contributors may be used to endorse or promote products derived from this
#    software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#                                                                             */

/*==================[inclusions]=============================================*/
#include "pit_outputPulse.h"
#include "fsl_pit.h"
#include "task.h"
#include "efHal_gpio.h"
/*==================[macros and typedef]=====================================*/

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/
efHal_gpio_id_t _pin;
/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

extern void pit_OP_config(void)
{
	/* Structure of initialize PIT */
	pit_config_t pitConfig;

	/*
	* pitConfig.enableRunInDebug = false;
	*/
	PIT_GetDefaultConfig(&pitConfig);

	PIT_Init(PIT, &pitConfig);
	PIT_EnableInterrupts(PIT, kPIT_Chnl_0, kPIT_TimerInterruptEnable);
	EnableIRQ(PIT_IRQn);
}

extern void pit_OP_launch(efHal_gpio_id_t pin, uint32_t timeUs){
	PIT_StopTimer(PIT, kPIT_Chnl_0);
	PIT_SetTimerPeriod(PIT, kPIT_Chnl_0, USEC_TO_COUNT(timeUs, CLOCK_GetFreq(kCLOCK_BusClk)));
	_pin = pin;

	taskENTER_CRITICAL();
	efHal_gpio_setPin(pin, true);
	PIT_StartTimer(PIT, kPIT_Chnl_0);
	taskEXIT_CRITICAL();
}

void PIT_IRQHandler(void)
{
	efHal_gpio_setPin(_pin, false);
	PIT_StopTimer(PIT, kPIT_Chnl_0);
    PIT_ClearStatusFlags(PIT, kPIT_Chnl_0, kPIT_TimerFlag);
}

/*==================[end of file]============================================*/

