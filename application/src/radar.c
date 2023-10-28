/*
###############################################################################
#
# Copyright 2021, Gustavo Muro
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
#include "../../../pwm_test/appBoard/inc/appBoard.h"
#include "efHal_gpio.h"
#include "efHal_pwm.h"
#include "FreeRTOS.h"
#include "task.h"

/*==================[macros and typedef]=====================================*/

#define SERVO_PWM EF_HAL_PWM0
#define SERVO_PWM_FREQ 50

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

static TimerHandle_t timerHandle;

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
static void main_task(void *pvParameters)
{
    efHal_pwm_setPeriod(SERVO_PWM, (uint32_t)((float)1e9/(float)SERVO_PWM_FREQ));
    xTimerStart(timerHandle, portMAX_DELAY);
    for (;;)
    {
    	portYIELD();
    }
}

static void timerCallback (TimerHandle_t xTimer){
	static uint8_t servoPos = 0;
	float servoDutyNs = 0;
	servoDutyNs = 1e6 + (float)servoPos/180.0 * 1e6;
	if(servoPos >= 180) servoPos -= 5;
	else if(servoPos <= 0) servoPos += 5;
	efHal_pwm_setDuty(SERVO_PWM, servoDutyNs, EF_HAL_PWM_DUTY_NS);
}

/*==================[external functions definition]==========================*/
int main(void)
{
    appBoard_init();

    xTaskCreate(main_task, "main_task", 100, NULL, 0, NULL);

    timerHandle = xTimerCreate("timer", 100/portTICK_PERIOD_MS, pdTRUE, 0, timerCallback);

    vTaskStartScheduler();
    for (;;);
}

extern void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    while (1);
}

/*==================[end of file]============================================*/
