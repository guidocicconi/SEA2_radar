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
#include <stdio.h>
#include <string.h>
#include "appBoard.h"
#include "efHal_gpio.h"
#include "efHal_pwm.h"
#include "efHal_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "softTimers.h"
#include "servo.h"
#include "sensor_sr04.h"
#include "efLeds.h"

/*==================[macros and typedef]=====================================*/
#define SERVO_MIN_ANGLE 30
#define SERVO_MAX_ANGLE 150
#define SERVO_DELTA_ANGLE 1
#define SERVO_PERIOD_MS 30

#define SERVO_PWM EF_HAL_PWM0
#define SENSOR_SR04_TRIG EF_HAL_D4
#define SENSOR_SR04_ECHO EF_HAL_D7

#define RADAR_UART efHal_dh_UART0
#define RADAR_UART_BAUDRATE 115200

typedef enum{
	GREEN_LED = 0,
	RED_LED,
	TOTAL_LEDS
}leds_t;

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

static TaskHandle_t uartTaskHandler = NULL, sensorTaskHandler = NULL;

static efLeds_conf_t leds_conf[2] = {
		{EF_HAL_GPIO_LED_GREEN, false},
		{EF_HAL_GPIO_LED_RED, false},
};

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
static void servo_task(void *pvParameters)
{
	uint8_t servoPosDegree = SERVO_MIN_ANGLE;
	int8_t deltaPosDegree = SERVO_DELTA_ANGLE;

	efLeds_init(leds_conf, TOTAL_LEDS);
	efLeds_msg(GREEN_LED, EF_LEDS_MSG_HEARTBEAT);

    servo_init(SERVO_PWM, SERVO_MIN_ANGLE);

    TickType_t xLastWakeTime = xTaskGetTickCount();

    for (;;)
    {
    	servo_setPos(servoPosDegree);

    	vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SERVO_PERIOD_MS));

    	xTaskNotify(sensorTaskHandler, servoPosDegree, eSetValueWithOverwrite);

    	servoPosDegree += deltaPosDegree;
    	if(servoPosDegree >= SERVO_MAX_ANGLE) deltaPosDegree = -SERVO_DELTA_ANGLE;
    	else if(servoPosDegree <= SERVO_MIN_ANGLE) deltaPosDegree = SERVO_DELTA_ANGLE;
    }
}

static void sensor_task(void *pvParameters)
{
	uint16_t distanceCm = 0;
	uint32_t servoPosDegree = 0, uartNotifyValue = 0;

    sensor_sr04_init(SENSOR_SR04_TRIG, SENSOR_SR04_ECHO);

    for (;;)
    {
    	xTaskNotifyWait(0, 0, &servoPosDegree, pdMS_TO_TICKS(300));

    	distanceCm = sensor_sr04_measure(SENSOR_UNIT_CM);

    	uartNotifyValue = ((servoPosDegree << 16) & 0xFFFF0000) | (distanceCm & 0xFFFF);
    	xTaskNotify(uartTaskHandler, uartNotifyValue, eSetValueWithOverwrite);
    }
}

static void uart_task(void *pvParameters)
{
	uint32_t uartNotifyValue = 0;
	uint16_t posValue = 0, distValue = 0;
	char uartDataToSend[8] = {0};

	efHal_uart_conf_t uart_conf = {
			.baudrate = RADAR_UART_BAUDRATE,
			.dataBits = EF_HAL_UART_DATA_BITS_8,
			.parity = EF_HAL_UART_PARITY_NONE,
			.stopBits = EF_HAL_UART_STOP_BITS_1
	};

	efHal_uart_conf(RADAR_UART, &uart_conf);

    for (;;)
    {
    	xTaskNotifyWait(0, 0, &uartNotifyValue, pdMS_TO_TICKS(300));

    	posValue = (uartNotifyValue >> 16) & 0xFFFF;
    	distValue = uartNotifyValue & 0xFFFF;

    	sprintf(uartDataToSend, "%d,%d\n", distValue, posValue);

    	efHal_uart_send(RADAR_UART, uartDataToSend, strlen(uartDataToSend), pdMS_TO_TICKS(100));
    }
}

/*==================[external functions definition]==========================*/
int main(void)
{
    appBoard_init();

    xTaskCreate(servo_task, "servo_task", 100, NULL, 1, NULL);
    xTaskCreate(sensor_task, "sensor_task", 100, NULL, 2, &sensorTaskHandler);
    xTaskCreate(uart_task, "uart_task", 200, NULL, 0, &uartTaskHandler);

    vTaskStartScheduler();
    for (;;);
}

extern void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    while (1);
}

void vApplicationTickHook(void)
{
	softTimers_rollOver();
}
/*==================[end of file]============================================*/
