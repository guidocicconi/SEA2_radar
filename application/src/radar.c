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
#include "FreeRTOS.h"
#include "task.h"
#include "appBoard.h"
#include "efHal_gpio.h"
#include "efHal_uart.h"
#include "efHal_spi.h"
#include "softTimers.h"
#include "efLeds.h"
#include "servo.h"
#include "sensor_sr04.h"
#include "oled.h"

/*==================[macros and typedef]=====================================*/
#define SERVO_MIN_ANGLE 30
#define SERVO_MAX_ANGLE 150
#define SERVO_DELTA_ANGLE 1
#define SERVO_PERIOD_MS 50

#define SERVO_PWM EF_HAL_PWM0
#define SENSOR_SR04_TRIG EF_HAL_D4
#define SENSOR_SR04_ECHO EF_HAL_D5

#define DISPLAY_SPI efHal_dh_SPI0
#define DISPLAY_CMD_PIN EF_HAL_D6
#define DISPLAY_RST_PIN EF_HAL_D7

#define RADAR_UART efHal_dh_UART0
#define RADAR_UART_BAUDRATE 115200

typedef enum{
	GREEN_LED = 0,
	RED_LED,
	TOTAL_LEDS
}leds_t;

/*==================[internal functions declaration]=========================*/

/*==================[internal data definition]===============================*/

static TaskHandle_t uartTaskHandler = NULL, displayTaskHandler = NULL;

static efLeds_conf_t leds_conf[2] = {
		{EF_HAL_GPIO_LED_GREEN, false},
		{EF_HAL_GPIO_LED_RED, false},
};

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
static void servo_sensor_task(void *pvParameters)
{
	uint8_t servoPosDegree = SERVO_MIN_ANGLE;
	int8_t deltaPosDegree = SERVO_DELTA_ANGLE;
	uint16_t distanceCm = 0;
	uint32_t notifyValue = 0;

    sensor_sr04_init(SENSOR_SR04_TRIG, SENSOR_SR04_ECHO);
    servo_init(SERVO_PWM, SERVO_MIN_ANGLE);

    for (;;)
    {
    	servo_setPos(servoPosDegree);

    	vTaskDelay(pdMS_TO_TICKS(SERVO_PERIOD_MS));

    	distanceCm = sensor_sr04_measure(SENSOR_UNIT_CM);

    	notifyValue = ((servoPosDegree << 16) & 0xFFFF0000) | (distanceCm & 0xFFFF);

    	xTaskNotify(uartTaskHandler, notifyValue, eSetValueWithOverwrite);
    	//xTaskNotify(displayTaskHandler, notifyValue, eSetValueWithOverwrite);

    	servoPosDegree += deltaPosDegree;
    	if(servoPosDegree >= SERVO_MAX_ANGLE) deltaPosDegree = -SERVO_DELTA_ANGLE;
    	else if(servoPosDegree <= SERVO_MIN_ANGLE) deltaPosDegree = SERVO_DELTA_ANGLE;
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
    	xTaskNotifyWait(0, 0, &uartNotifyValue, portMAX_DELAY);

    	posValue = (uartNotifyValue >> 16) & 0xFFFF;
    	distValue = uartNotifyValue & 0xFFFF;

    	sprintf(uartDataToSend, "%d,%d\n", distValue, posValue);

    	efHal_uart_send(RADAR_UART, uartDataToSend, strlen(uartDataToSend), portMAX_DELAY);
    }
}

static void display_task(void *pvParameters)
{
	uint32_t dislpayNotifyValue = 0;
	uint16_t posValue = 0, distValue = 0;

	char displayPos[13] = {0};
	char displayDist[17] = {0};

	oled_init(DISPLAY_SPI, DISPLAY_CMD_PIN, DISPLAY_RST_PIN);

    for (;;)
    {
    	xTaskNotifyWait(0, 0, &dislpayNotifyValue, portMAX_DELAY);

    	posValue = (dislpayNotifyValue >> 16) & 0xFFFF;
    	distValue = dislpayNotifyValue & 0xFFFF;

    	sprintf(displayPos, "Ángulo: %03d°", posValue);
    	if(distValue != 0) sprintf(displayDist, "Distancia: %03dcm", distValue);
    	else sprintf(displayDist, "Distancia: ERROR");

    	oled_putString(5, 5, (uint8_t*)displayPos, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    	oled_putString(5, 25, (uint8_t*)displayDist, OLED_COLOR_WHITE, OLED_COLOR_BLACK);
    }
}

/*==================[external functions definition]==========================*/
int main(void)
{
    xTaskCreate(servo_sensor_task, "servo_sensor_task", 100, NULL, 2, NULL);
    xTaskCreate(uart_task, "uart_task", 200, NULL, 1, &uartTaskHandler);
    //xTaskCreate(display_task, "display_task", 200, NULL, 0, &displayTaskHandler);

    vTaskStartScheduler();
    for (;;);
}

extern void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    while (1);
}

void vApplicationDaemonTaskStartupHook(){
    appBoard_init();
	efLeds_init(leds_conf, TOTAL_LEDS);
	efLeds_msg(GREEN_LED, EF_LEDS_MSG_HEARTBEAT);
}

void vApplicationTickHook(void)
{
	softTimers_rollOver();
}
/*==================[end of file]============================================*/
