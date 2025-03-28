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

#include <stdlib.h>

/*-----------------------------------------------------------*/
#define mainQUEUE_LENGTH 100

#define green 0
#define yellow 1
#define red 2

// The first iteration of ADC check will be 50
#define INITIAL_ADC 50

// Time for standard light length (in miliseconds):
#define GREEN_STD_TIME 200UL // 0.2 seconds
#define YELLOW_STD_TIME 3000UL // 3 seconds
#define RED_STD_TIME 100UL // 0.1 seconds
// Somewhat unintuitively, despite std_time being less than yellow, green and red will be boosted by ADC's value, which will end up max 10 seconds

#define ADC_CEIL 80
#define ADC_FLOOR 10
#define DICE_ADJUSTMENT 15

// Hardware setup function definitions
static void prvSetupHardware( void );
static void GPIO_Setup( void );
static void ADC_Setup ( void );

static void manualSleep(int time);

// Task function definitions
static void Traffic_Flow_Task( void *pvParameters );
static void Traffic_Generator_Task( void *pvParameters );
static void Traffic_Light_State_Task( void *pvParameters );
static void System_Display_Task( void *pvParameters );

// Reserve a space in memory for the queue handles. Each handle is just something
// 		FreeRTOS uses to reference the queues.
xQueueHandle xStreetQueue_handle = 0;
xQueueHandle xFlowQueue_handle = 0;
xQueueHandle xLightQueue_handle = 0;

/*-----------------------------------------------------------*/

int main( void )
{
	// Call the hardware initilization functions.
	prvSetupHardware();

	// Create the queue used by our tasks.
	xStreetQueue_handle = xQueueCreate(8, sizeof( uint16_t ) );

	// Create a queue to hold active red/yellow/green status.
	xLightQueue_handle = xQueueCreate(1, sizeof( uint16_t ));

	// Create a queue to hold the current normalized ADC flow value.
	xFlowQueue_handle = xQueueCreate(1, sizeof( uint16_t ));

	// Add each queue to the registry, for kernel aware debugging.
	vQueueAddToRegistry( xStreetQueue_handle, "StreetQueue" );
	vQueueAddToRegistry( xLightQueue_handle, "LightQueue" );
	vQueueAddToRegistry( xFlowQueue_handle, "FlowQueue" );

	// Create each infividual task, providing a relative priority to each.
	xTaskCreate( Traffic_Flow_Task, "Flow", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate( Traffic_Generator_Task, "Generator", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate( Traffic_Light_State_Task, "Light_State", configMINIMAL_STACK_SIZE, NULL, 2, NULL);
	xTaskCreate( System_Display_Task, "Sys_Display", configMINIMAL_STACK_SIZE, NULL, 2, NULL);

	// Start the tasks and timer running.
	vTaskStartScheduler();

	return 0;
}


/*-----------------------------------------------------------*/

// This task is responsible for reading from the ADC to get the value of the potentiometer's resistence.
static void Traffic_Flow_Task( void *pvParameters )
{
	uint16_t adc_val = INITIAL_ADC;
	uint32_t unnorm_val;

	while(1)
	{
		if(xQueueOverwrite(xFlowQueue_handle, &adc_val))
		{
			// printf("$$ Overwrote ADC flow to %d\n", adc_val);
			vTaskDelay(500);

			// Begin reading a value from the ADC.
			ADC_SoftwareStartConv(ADC1);
			while(!ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC)) {
				// Once EOC set read data
				;
			}
			unnorm_val = ADC_GetConversionValue(ADC1);

			// Normalize the value so that it lands between 0 and 100.
			// printf("$$ What is our ADC reading? %d\n", unnorm_val);
			adc_val = 100 * (unnorm_val / 65520.0f); 
			// The normalization constant determined by observed maximum reading from potentiometer.

			// We flip the value so that it is representative of resistance.
			adc_val = 100 - adc_val;
			// printf("$$ Normed ADC Reading: %d\n", adc_val);
			// Something to note here for report discussion: putting the vTaskDelay before or after the process.

			// We just do this to avoid a divide by 0 elsewhere.
			// As it turns out, our initial minimum level as too powerful! Resulting in no vehicles appearing (as there was 1% chance).
			// So we are raising the floor so that we can guarantee at least SOME cars appear.
			if (adc_val <= ADC_FLOOR) {
				adc_val = ADC_FLOOR;
			}
			// printf("$$ Normed ADC Reading: %d\n", adc_val);
		}
		else
		{
			printf("Flow Failed!\n");
		}
	}
}

/*-----------------------------------------------------------*/

// This task is responsible for determining whether a vehicle should be generated based off of current traffic flow.
static void Traffic_Generator_Task( void *pvParameters )
{
	uint16_t traffic_flow = INITIAL_ADC;
	uint16_t should_gen_flag = 1; // This could potentially be uint8_t.

	while(1)
	{
		if(xQueuePeek(xFlowQueue_handle, &traffic_flow, 500))
		{
			// Roll a d100 dice; if the result is LESS then the traffic flow (which is 1-100), then we spawn a new vehicle.
			int result = rand() % 100;
			// printf("++ Dice Result: %d, Traffic Read: %d\n", result + 15, traffic_flow);

			if (result + DICE_ADJUSTMENT  < traffic_flow) {
				// BE AWARE: should_gen should ONLY EVER BE a 1 here:
				if(xQueueSend(xStreetQueue_handle, &should_gen_flag, 1000))
				{
					// printf("++ Traffic Sent.\n");
					// Do nothing. xQueueSend does everything we need to.
				}
				else
				{
					printf("++ Traffic Gen Failed!\n");
				}
			}
			vTaskDelay(500);
		}
	}
}

/*-----------------------------------------------------------*/

// This task is responsible for rotating through the current displayed traffic light (green, yellow, or red) and maintaining 
// 		that light for a period proportional to the traffic flow.
static void Traffic_Light_State_Task( void *pvParameters )
{
	uint16_t traffic_flow = INITIAL_ADC;
	uint16_t which_light = red;

	while(1)
	{
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
					// The traffic flow is a normalized value between 1-100. By multiplying it with teh standard
					//		green light time, we can wait an amount of miliseconds dependent on the traffic flow.
					vTaskDelay(pdMS_TO_TICKS(traffic_flow * GREEN_STD_TIME));
				}
				else if (which_light == yellow) {
					// Yellow light waits a constant time, so no need to interact with the traffic flow.
					vTaskDelay(pdMS_TO_TICKS(YELLOW_STD_TIME));
				}
				else {
					// Set up the inverse proportional relationship using traffic flow.
					vTaskDelay(pdMS_TO_TICKS((100 - traffic_flow) * RED_STD_TIME));
				}
			}
			else
			{
				printf("== Light State Failed!\n");
			}
		}
	}
}

/*-----------------------------------------------------------*/

// This task handles outputing bits to all hardware peripherals according to values read from the three queues.
static void System_Display_Task( void *pvParameters )
{
	uint16_t light = green;
	uint16_t should_gen_new_car = 0;
	uint32_t traffic_pattern = 0x00;
	uint8_t pre_light = 0x00;
	uint32_t post_light = 0x00;
	uint8_t preserving_mask = 0x00;
	uint8_t bitmask = 0x80;

	while(1)
	{
		// Light up the correct traffic control light.
		if (xQueuePeek(xLightQueue_handle, &light, 500)) {
			// TODO: could probably remove these if blocks by a modulo or some other math operation to use [light] directly.
			if (light == green) {
				// Green LED on Pin 2
				GPIO_ResetBits(GPIOC, GPIO_Pin_0);
				GPIO_SetBits(GPIOC, GPIO_Pin_2);
			}
			else if (light == yellow) {
				// Yellow LED on Pin 1
				GPIO_ResetBits(GPIOC, GPIO_Pin_2);
				GPIO_SetBits(GPIOC, GPIO_Pin_1);
			}
			else {
				// Red LED on Pin 0
				GPIO_ResetBits(GPIOC, GPIO_Pin_1);
				GPIO_SetBits(GPIOC, GPIO_Pin_0);
			}
		}

		// Consume 1 message from the StreetQueue, which will either be empty or a 1.
		if(xQueueReceive(xStreetQueue_handle, &should_gen_new_car, 500))
		{
			if (should_gen_new_car) {
				pre_light = pre_light | 0x1;
				should_gen_new_car = 0;
			}
		}

		// Assemble the traffic pattern
		traffic_pattern = (post_light << 8) | pre_light;

		// Reset for a new pattern to be input. Manual sleep is required to give the GPIO enough time to
		// 		fully reset and set the one pin.
		GPIO_ResetBits(GPIOC, GPIO_Pin_8);
		manualSleep(1000);
		GPIO_SetBits(GPIOC, GPIO_Pin_8);
		manualSleep(1000);

		// Loop through traffic pattern to display to LEDs by selecting individual bits with the help of a selector mask.
		// Some funky stuff like reading backwards here, as our system was right-endian yet 
		// 		vehicles needed to move left to right. This is further discussed in the report discussion.
		uint32_t selector_mask = 0x40000; // 0100 0000 0000 0000 0000
		for (int i = 19 - 1; i >= 0; i--) {
			// Determine whether the current bit is a 1 or 0, and output to the shift register data pin accordingly.
			if ((traffic_pattern & selector_mask) == 0) {
				GPIO_ResetBits(GPIOC, GPIO_Pin_6);
			}
			else {
				GPIO_SetBits(GPIOC, GPIO_Pin_6);
			}

			// After determining whether port 6 should output 0 or 1, we need to communicate to the shift registers
			// 		by pulsing the clock pin. This shifts the output by one down the daisy chain.
			GPIO_SetBits(GPIOC, GPIO_Pin_7);
			manualSleep(100);
			GPIO_ResetBits(GPIOC, GPIO_Pin_7);
			manualSleep(100);

			// Advance the selector mask by 1 to select the next bit in the traffic pattern.
			selector_mask = selector_mask >> 1;
		}

		// We always shift post_light, as the vehicles past the intersection never get stopped.
		post_light = post_light << 1;

		// This logic covers stopping traffic. It does so by identifying the most significant 1s which needed to be
		// 		preserved, as when you bitshift the pre_light, 1s may be lost.
		// The preserving mask begins equal to 0, as the vehicles which must be preserved have yet to be identified.
		preserving_mask = 0x00;
		if (light == red || light == yellow) {
			bitmask = 0x80; // 1000 0000, the 1 shifts to the right to select bits

			// Loop through and build a preserving mask out of the most significant 1s.
			for (int i = 7; i >= 0; i--) {
				// If the current bit is a 1...
				if ((pre_light & bitmask) != 0) {
					// Assemble the preserving mask by introducing a 1 into the same position.
					preserving_mask = preserving_mask | bitmask;
				}
				else {
					// Otherwise, a 0 has been encountered. This means we no longer need to update the preserving mask.
					break;
				}

				// Shift the bitmask to select the next bit in pre_light.
				bitmask = bitmask >> 1;
			}
		}
		else {
			// If it is green, we need to check if we will lose a 1.
			if ((pre_light & 0x80) != 0) {
				// If so, we introduce a new 1 into post_light.
				post_light = post_light | 0x1;
			}
		}

		// Shift the pre_light and use the preserving mask to restore any lost 1s.
		pre_light = (pre_light << 1) | preserving_mask;

		// Wait a contant time before updating the system display again.
		vTaskDelay(500);
	}
}

/*-----------------------------------------------------------*/

// A quick and dirty method for manually delaying our system so that the peripherals have time
//		to catch up to our way faster CPU.
static void manualSleep(int time) {
	for (int i = time; i > 0; i--) {
		// Wait!
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

// This is a helper function which just groups together our two peripheral setup functions.
static void prvSetupHardware( void )
{
	printf("Hardware being setup.\n");
	NVIC_SetPriorityGrouping( 0 );

	GPIO_Setup();
	ADC_Setup();

}

// gpio analog for pot
// one for each Big Boi LEDs might need different mode (GPIO to out)
// pino 0 1 2 3

// This function sets the correct config states for the GPIO and each pin we use.
static void GPIO_Setup( void ) {
	// Enable Clock
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);

	// GPIO_Init
	// Must create an InitStruct to set characteristics of GPIOC, then pass into GPIO_Init function.
	GPIO_InitTypeDef GPIO_InitStruct;

    // Setup for PC0, PC1, PC2 for Traffic Lights (Red, Amber, Green)PC6, 7, 8 Shift Registers.
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT; // Alternate Function, but out is also an option
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP; // Push-Pull
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // Pull Down? // no pull is also an option
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_2MHz; // Faster the clock is, drives the signal harder, draws more current, more spiking and ringing which poses problems.
	GPIO_Init(GPIOC, &GPIO_InitStruct);

	// Pin 3 needs special settings as it connects to the potentiometer via the ADC.
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AN; // Analog mode for ADC
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_NOPULL; // No pull-up or pull-down for analog
	GPIO_Init(GPIOC, &GPIO_InitStruct);
}

// This function similarly configures the ADC.
static void ADC_Setup ( void ) {
	// Enable Clock
	RCC_APB2PeriphClockCmd(RCC_APB2ENR_ADC1EN, ENABLE);

	// ADC Init Configuration
	ADC_InitTypeDef ADC_InitStruct;
	ADC_InitStruct.ADC_ContinuousConvMode = DISABLE;

	// We should prefer a right-aligned endian-ness
	ADC_InitStruct.ADC_DataAlign = ADC_DataAlign_Right;
	ADC_InitStruct.ADC_Resolution = ADC_Resolution_12b;
	ADC_InitStruct.ADC_ExternalTrigConv = DISABLE;

	// Initialize the ADC using the init structure.
	ADC_Init(ADC1, &ADC_InitStruct);

	// ADC Enable
	ADC_Cmd(ADC1, ENABLE);
	
	// ADC Channel Config - Slides say to try different sample times
	ADC_RegularChannelConfig(ADC1, ADC_Channel_13, 1, ADC_SampleTime_3Cycles);
}
