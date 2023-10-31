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
#include "appBoard.h"
#include "efHal_gpio.h"
#include "efHal_pwm.h"
#include "FreeRTOS.h"
#include "task.h"
#include "softTimers.h"
#include "servo.h"
#include "sensor_sr04.h"

/*==================[macros and typedef]=====================================*/

#define SERVO_PWM EF_HAL_PWM0
#define SENSOR_SR04_TRIG EF_HAL_D4
#define SENSOR_SR04_ECHO EF_HAL_D5

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
static void main_task(void *pvParameters)
{
	uint8_t servoPosDegre = 0;
	int8_t deltaPosDegree = 5;
	uint16_t distanceCm = 0;

    servo_init(SERVO_PWM);
    sensor_sr04_init(SENSOR_SR04_TRIG, SENSOR_SR04_ECHO);

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
    	distanceCm = sensor_sr04_measure(SENSOR_DISTANCE_CM);

    	servo_setPos(servoPosDegre);

    	servoPosDegre += deltaPosDegree;
    	if(servoPosDegre >= 150) deltaPosDegree = -5;
    	else if(servoPosDegre <= 30) deltaPosDegree = 5;

    	vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(100));
    }
}

static void blinky_task(void *pvParameters)
{
    for (;;)
    {
    	vTaskDelay(pdMS_TO_TICKS(1000));
    	efHal_gpio_togglePin(EF_HAL_GPIO_LED_GREEN);
    }
}


/*==================[external functions definition]==========================*/
int main(void)
{
    appBoard_init();

    xTaskCreate(main_task, "main_task", 100, NULL, 1, NULL);
    xTaskCreate(blinky_task, "blinky_task", 100, NULL, 0, NULL);

    vTaskStartScheduler();
    for (;;);
}

extern void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    while (1);
}

void vApplicationTickHook(void)
{
	softTimers_rollOver();
}
/*==================[end of file]============================================*/
