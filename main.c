/*
    FreeRTOS V9.0.0 - Copyright (C) 2016 Real Time Engineers Ltd.
    All rights reserved

    VISIT http://www.FreeRTOS.org TO ENSURE YOU ARE USING THE LATEST VERSION.

    This file is part of the FreeRTOS distribution.

    FreeRTOS is free software; you can redistribute it and/or modify it under
    the terms of the GNU General Public License (version 2) as published by the
    Free Software Foundation >>>> AND MODIFIED BY <<<< the FreeRTOS exception.

    ***************************************************************************
    >>!   NOTE: The modification to the GPL is included to allow you to     !<<
    >>!   distribute a combined work that includes FreeRTOS without being   !<<
    >>!   obliged to provide the source code for proprietary components     !<<
    >>!   outside of the FreeRTOS kernel.                                   !<<
    ***************************************************************************

    FreeRTOS is distributed in the hope that it will be useful, but WITHOUT ANY
    WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
    FOR A PARTICULAR PURPOSE.  Full license text is available on the following
    link: http://www.freertos.org/a00114.html

    ***************************************************************************
     *                                                                       *
     *    FreeRTOS provides completely free yet professionally developed,    *
     *    robust, strictly quality controlled, supported, and cross          *
     *    platform software that is more than just the market leader, it     *
     *    is the industry's de facto standard.                               *
     *                                                                       *
     *    Help yourself get started quickly while simultaneously helping     *
     *    to support the FreeRTOS project by purchasing a FreeRTOS           *
     *    tutorial book, reference manual, or both:                          *
     *    http://www.FreeRTOS.org/Documentation                              *
     *                                                                       *
    ***************************************************************************

    http://www.FreeRTOS.org/FAQHelp.html - Having a problem?  Start by reading
    the FAQ page "My application does not run, what could be wwrong?".  Have you
    defined configASSERT()?

    http://www.FreeRTOS.org/support - In return for receiving this top quality
    embedded software for free we request you assist our global community by
    participating in the support forum.

    http://www.FreeRTOS.org/training - Investing in training allows your team to
    be as productive as possible as early as possible.  Now you can receive
    FreeRTOS training directly from Richard Barry, CEO of Real Time Engineers
    Ltd, and the world's leading authority on the world's leading RTOS.

    http://www.FreeRTOS.org/plus - A selection of FreeRTOS ecosystem products,
    including FreeRTOS+Trace - an indispensable productivity tool, a DOS
    compatible FAT file system, and our tiny thread aware UDP/IP stack.

    http://www.FreeRTOS.org/labs - Where new FreeRTOS products go to incubate.
    Come and try FreeRTOS+TCP, our new open source TCP/IP stack for FreeRTOS.

    http://www.OpenRTOS.com - Real Time Engineers ltd. license FreeRTOS to High
    Integrity Systems ltd. to sell under the OpenRTOS brand.  Low cost OpenRTOS
    licenses offer ticketed support, indemnification and commercial middleware.

    http://www.SafeRTOS.com - High Integrity Systems also provide a safety
    engineered and independently SIL3 certified version for use in safety and
    mission critical applications that require provable dependability.

    1 tab == 4 spaces!
*/

/*
FreeRTOS is a market leading RTOS from Real Time Engineers Ltd. that supports
31 architectures and receives 77500 downloads a year. It is professionally
developed, strictly quality controlled, robust, supported, and free to use in
commercial products without any requirement to expose your proprietary source
code.

This simple FreeRTOS demo does not make use of any IO ports, so will execute on
any Cortex-M3 of Cortex-M4 hardware.  Look for TODO markers in the code for
locations that may require tailoring to, for example, include a manufacturer
specific header file.

This is a starter project, so only a subset of the RTOS features are
demonstrated.  Ample source comments are provided, along with web links to
relevant pages on the http://www.FreeRTOS.org site.

Here is a description of the project's functionality:

The main() Function:
main() creates the tasks and software timers described in this section, before
starting the scheduler.

The Queue Send Task:
The queue send task is implemented by the prvQueueSendTask() function.
The task uses the FreeRTOS vTaskDelayUntil() and xQueueSend() API functions to
periodically send the number 100 on a queue.  The period is set to 200ms.  See
the comments in the function for more details.
http://www.freertos.org/vtaskdelayuntil.html
http://www.freertos.org/a00117.html

The Queue Receive Task:
The queue receive task is implemented by the prvQueueReceiveTask() function.
The task uses the FreeRTOS xQueueReceive() API function to receive values from
a queue.  The values received are those sent by the queue send task.  The queue
receive task increments the ulCountOfItemsReceivedOnQueue variable each time it
receives the value 100.  Therefore, as values are sent to the queue every 200ms,
the value of ulCountOfItemsReceivedOnQueue will increase by 5 every second.
http://www.freertos.org/a00118.html

An example software timer:
A software timer is created with an auto reloading period of 1000ms.  The
timer's callback function increments the ulCountOfTimerCallbackExecutions
variable each time it is called.  Therefore the value of
ulCountOfTimerCallbackExecutions will count seconds.
http://www.freertos.org/RTOS-software-timer.html

The FreeRTOS RTOS tick hook (or callback) function:
The tick hook function executes in the context of the FreeRTOS tick interrupt.
The function 'gives' a semaphore every 500th time it executes.  The semaphore
is used to synchronise with the event semaphore task, which is described next.

The event semaphore task:
The event semaphore task uses the FreeRTOS xSemaphoreTake() API function to
wait for the semaphore that is given by the RTOS tick hook function.  The task
increments the ulCountOfReceivedSemaphores variable each time the semaphore is
received.  As the semaphore is given every 500ms (assuming a tick frequency of
1KHz), the value of ulCountOfReceivedSemaphores will increase by 2 each second.

The idle hook (or callback) function:
The idle hook function queries the amount of free FreeRTOS heap space available.
See vApplicationIdleHook().

The malloc failed and stack overflow hook (or callback) functions:
These two hook functions are provided as examples, but do not contain any
functionality.
*/

/* Standard includes. */
#include <stdint.h>
#include <stdio.h>
#include "stm32f4_discovery.h"
/* Kernel includes. */
#include "stm32f4xx.h"
#include "../FreeRTOS_Source/include/FreeRTOS.h"
#include "../FreeRTOS_Source/include/queue.h"
#include "../FreeRTOS_Source/include/semphr.h"
#include "../FreeRTOS_Source/include/task.h"
#include "../FreeRTOS_Source/include/timers.h"
/* GPIO includes. */
//#include "stm32f4xx_gpio.c"
//#include "stm32f4xx_adc.c"
//#include "../Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c"
//#include "../Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_adc.c"



/*-----------------------------------------------------------*/
#define mainQUEUE_LENGTH 100

#define green 0
#define yellow 1
#define red 2

// The first iteration of ADC check will be 50
#define INITIAL_ADC 50

// Time for standard light length (in miliseconds):
#define GREEN_STD_TIME 10000UL
#define YELLOW_STD_TIME 3000UL
#define RED_STD_TIME 5000UL

/*
 * TODO: Implement this function for any hardware specific clock configuration
 * that was not already performed before main() was called.
 */
static void prvSetupHardware( void );
static void GPIO_Setup( void );
static void ADC_Setup ( void );

/*
 * The queue send and receive tasks as described in the comments at the top of
 * this file.
 */
//static void Manager_Task( void *pvParameters );
//static void Blue_LED_Controller_Task( void *pvParameters );
//static void Green_LED_Controller_Task( void *pvParameters );
//static void Red_LED_Controller_Task( void *pvParameters );
//static void Amber_LED_Controller_Task( void *pvParameters );

static void Traffic_Flow_Task( void *pvParameters );
static void Traffic_Generator_Task( void *pvParameters );
static void Traffic_Light_State_Task( void *pvParameters );
static void System_Display_Task( void *pvParameters );

// TODO: check if these should be 0 1 2 or all 0s
xQueueHandle xStreetQueue_handle = 0;
xQueueHandle xFlowQueue_handle = 0;
xQueueHandle xLightQueue_handle = 0;

/*-----------------------------------------------------------*/

int main( void )
{
	printf("Starting main func\n");
	fflush(stdout);
	/* Initialize LEDs */
//	STM_EVAL_LEDInit(amber_led);
//	STM_EVAL_LEDInit(green_led);
//	STM_EVAL_LEDInit(red_led);
//	STM_EVAL_LEDInit(blue_led);

	/* Configure the system ready to run the demo.  The clock configuration
	can be done here if it was not done before main() was called. */
	prvSetupHardware();


	/* Create the queue used by the queue send and queue receive tasks.
	http://www.freertos.org/a00116.html */
	xStreetQueue_handle = xQueueCreate(8, sizeof( uint16_t ) );
	// TODO: Check what happens when a full queue receives a push - will it overwrite?
	/* The number of items the queue can hold, and the size of each item the queue holds. */

	// Create a queue to hold active red/yellow/green status.
	xLightQueue_handle = xQueueCreate(1, sizeof( uint16_t ));

	// Create a queue to hold
	xFlowQueue_handle = xQueueCreate(1, sizeof( uint16_t ));

	/* Add to the registry, for the benefit of kernel aware debugging. */
	vQueueAddToRegistry( xStreetQueue_handle, "StreetQueue" );

//	xTaskCreate( Manager_Task, "Manager", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
//	xTaskCreate( Blue_LED_Controller_Task, "Blue_LED", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
//	xTaskCreate( Red_LED_Controller_Task, "Red_LED", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
//	xTaskCreate( Green_LED_Controller_Task, "Green_LED", configMINIMAL_STACK_SIZE, NULL, 1, NULL);
//	xTaskCreate( Amber_LED_Controller_Task, "Amber_LED", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	xTaskCreate( Traffic_Flow_Task, "Flow", configMINIMAL_STACK_SIZE, NULL, 3, NULL);
	xTaskCreate( Traffic_Generator_Task, "Generator", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate( Traffic_Light_State_Task, "Light_State", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate( System_Display_Task, "Sys_Display", configMINIMAL_STACK_SIZE, NULL, 1, NULL);

	/* Start the tasks and timer running. */
	vTaskStartScheduler();

	while(1) {
		printf("Hety!\n");
	}

	return 0;
}


/*-----------------------------------------------------------*/

static void Traffic_Flow_Task( void *pvParameters )
{
	uint16_t adc_val = INITIAL_ADC;

	while(1)
	{
//		if(xQueueSend(xFlowQueue_handle,&adc_val,1000))
		if(xQueueOverwrite(xFlowQueue_handle, &adc_val))
		{
			vTaskDelay(500);
			ADC_SoftwareStartConv(ADC1);
			while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)) {
				// Once EOC set read data
				;
			}
			// TODO: Normalize this after determining... how to do that.
			// adc_data = adc_convert() / 409;
			adc_val = ADC_GetConversionValue(ADC1);
			// Something to note here for report discussion: pytting the vTaskDelay before or after the process
		}
		else
		{
			printf("Flow Failed!\n");
		}
	}
}

/*-----------------------------------------------------------*/

static void Traffic_Generator_Task( void *pvParameters )
{
	uint16_t traffic_flow = INITIAL_ADC;

	while(1)
	{
		if(xQueuePeek(xFlowQueue_handle, &traffic_flow, 500))
		{
			// TODO: roll a dice to determine if should create new vehicle
			uint16_t should_gen = 0;

			if (should_gen)
			{
				// BE AWARE: should_gen should ONLY EVER BE a 1 here:
				if(xQueueSend(xStreetQueue_handle, &should_gen, 1000))
				{
					// Do... nothing? xQueueSend does everything we need to.
				}
				else
				{
					printf("Traffic Gen Failed!\n");
				}
			}
			vTaskDelay(500);
		}
	}
}

/*-----------------------------------------------------------*/

static void Traffic_Light_State_Task( void *pvParameters )
{
	while(1)
	{
		uint16_t traffic_flow = INITIAL_ADC;
		uint16_t which_light = red;

		if(xQueuePeek(xFlowQueue_handle, &traffic_flow, 500))
		{
			// First step: change the light.
			if (which_light == green) {
				which_light = yellow;
			}
			else if (which_light == yellow) {
				which_light = red;
			}
			else {
				which_light = green;
			}

			// Second step: update light state for others.
			if(xQueueOverwrite(xLightQueue_handle, &which_light))
			{
				// Third step: wait a time proportional to flow rate.
				if (which_light == green) {
					vTaskDelay(pdMS_TO_TICKS(GREEN_STD_TIME) * (traffic_flow / 100.0f));
				}
				else if (which_light == yellow) {
					vTaskDelay(pdMS_TO_TICKS(YELLOW_STD_TIME));
				}
				else {
					vTaskDelay(pdMS_TO_TICKS(RED_STD_TIME) / (traffic_flow / 100.0f));
				}
			}
			else
			{
				printf("Light State Failed!\n");
			}
		}
	}
}

/*-----------------------------------------------------------*/

static void System_Display_Task( void *pvParameters )
{
	uint16_t should_gen_new_car = 0;
	uint16_t pre_light = 0x01;
//	uint16_t post_light = 0x00;

	while(1)
	{
//		if(xQueueRecieve(xStreetQueue_handle, &should_gen_new_car, 500))
//		{
//			if (should_gen_new_car) {
//				pre_light = pre_light | 0x01;
//			}
//		}

		// TODO: printf is not working anymore! Beg Brent for help tomorrow...
		pre_light = pre_light << 1;

		// HOW TO WRITE TO DATA PORT?
		GPIO_ResetBits(GPIOC, GPIO_Pin_6);
		GPIO_Write(GPIOC, pre_light);
		GPIO_SetBits(GPIOC, GPIO_Pin_6);

		// unsigned char digits[10] = { 0x3f, 0x06, 0x5B, 0x4F, 0x66, 0x6D, 0x7D, 0x07, 0x7F, 0x6F };

		vTaskDelay(500);
	}
}

/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* The malloc failed hook is enabled by setting
	configUSE_MALLOC_FAILED_HOOK to 1 in FreeRTOSConfig.h.

	Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software 
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationStackOverflowHook( xTaskHandle pxTask, signed char *pcTaskName )
{
	( void ) pcTaskName;
	( void ) pxTask;

	/* Run time stack overflow checking is performed if
	configconfigCHECK_FOR_STACK_OVERFLOW is defined to 1 or 2.  This hook
	function is called if a stack overflow is detected.  pxCurrentTCB can be
	inspected in the debugger if the task name passed into this function is
	corrupt. */
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationIdleHook( void )
{
volatile size_t xFreeStackSpace;

	/* The idle task hook is enabled by setting configUSE_IDLE_HOOK to 1 in
	FreeRTOSConfig.h.

	This function is called on each cycle of the idle task.  In this case it
	does nothing useful, other than report the amount of FreeRTOS heap that
	remains unallocated. */
	xFreeStackSpace = xPortGetFreeHeapSize();

	if( xFreeStackSpace > 100 )
	{
		/* By now, the kernel has allocated everything it is going to, so
		if there is a lot of heap remaining unallocated then
		the value of configTOTAL_HEAP_SIZE in FreeRTOSConfig.h can be
		reduced accordingly. */
	}
}
/*-----------------------------------------------------------*/

static void prvSetupHardware( void )
{
	printf("Hardware being setup.\n");
	/* Ensure all priority bits are assigned as preemption priority bits.
	http://www.freertos.org/RTOS-Cortex-M3-M4.html */
	NVIC_SetPriorityGrouping( 0 );

	/* TODO: Setup the clocks, etc. here, if they were not configured before
	main() was called. */
	GPIO_Setup();
	ADC_Setup();

}

// gpio analog for pot
// one for each Big Boi LEDs might need different mode (GPIO to out)
// pino 0 1 2 3

static void GPIO_Setup( void ) {
	// Enable Clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
	// GPIO_Init
	// Must create an InitStruct to set characteristics of GPIOC, then pass
	// into GPIO_Init function
	GPIO_InitTypeDef GPIO_InitStruct;

    // Setup for PC0, PC1, PC2 for Traffic Lights (Red, Amber, Green)PC6, 7, 8 Shift Registers
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; // Alternate Function // out is also an option
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // Push-Pull
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // Pull Down? // no pull is also an option
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz; // Highest? // 50 and 25hz also work
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN;         // Analog mode for ADC
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL;       // No pull-up or pull-down for analog
	GPIO_Init(GPIOC, &GPIO_InitStruct);
	// Set or Reset Bits
	//GPIO_SetBits(GPIOC, 0x06); // Set GPIOC Pin 6 (Data)?
}

static void ADC_Setup ( void ) {
	printf("ADC Start\n");
	fflush(stdout);
	// Enable Clock
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_ADC1EN, ENABLE);
	// ADC Init Configuration
	ADC_InitTypeDef ADC_InitStruct;
	ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;
	// We should prefer a right-aligned endian-ness
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStruct.ADC_ExternalTrigConv = DISABLE;

	ADC_Init(ADC1, &ADC_InitStruct);
	// ADC Enable
	ADC_Cmd(ADC1, ENABLE);
	// ADC Channel Config - Slides say to try different sample times
	// Also confirm this is the correct channel?
	ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_3Cycles);
	// ADC Conversion
//	ADC_SoftwareStartConv(ADC1);
//
//	while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)) {
//		// Once EOC set read data
//		;
//	}
//	uint16_t adcval = ADC_GetConversionValue(ADC1);
//	printf("ADC Value: %d\n", adcval);
//	fflush(stdout);
}
