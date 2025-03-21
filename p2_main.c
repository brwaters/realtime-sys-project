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
#include <time.h>

/*-----------------------------------------------------------*/
#define mainQUEUE_LENGTH 100

#define DD_TASK_0_EX_TIME 95
#define DD_TASK_1_EX_TIME 150
#define DD_TASK_2_EX_TIME 250

#define DD_TASK_0_PERIOD 500
#define DD_TASK_1_PERIOD 500
#define DD_TASK_2_PERIOD 750

#define HYPER_PERIOD 1500

#define TICK_TO_MS_RATIO 1000/configTICK_RATE_HZ

typedef enum dd_task_type {
	PERIODIC,
	APERIODIC
} DD_Task_Type;

typedef struct dd_task_generator_parameters {
	uint32_t generator_id;
	DD_Task_Type task_type;
	TaskHandle_t f_task_handle;
	int execution_time;
	int period;
} DD_Task_Generator_Parameters;

typedef struct dd_task {
	TaskHandle_t t_handle;
	DD_Task_Type type;
	uint32_t task_id;
	uint8_t parent_id;
	uint32_t release_time;
	uint32_t absolute_deadline;
	uint32_t completion_time;
} DD_Task;

typedef struct dd_task_list_node {
	DD_Task task;
	struct dd_task_list_node* next_task;
} DD_Task_List_Node;

/*-----------------------------------------------------------*/

// Hardware setup function definitions
static void prvSetupHardware( void );
static void GPIO_Setup( void );
static void ADC_Setup ( void );

// Helper functions
static void manualSleep( int time );
void DD_Task_Generator_Setup( DD_Task_Generator_Parameters* param0, DD_Task_Generator_Parameters* param1, DD_Task_Generator_Parameters* param2 );

//void create_dd_task( TaskHandle_t t_handle,
//	task_type type,
//	uint32_t task_id,
//	uint32_t absolute_deadline,
//);

void delete_dd_task(uint32_t task_id);
DD_Task_List_Node** get_active_dd_task_list(void);
DD_Task_List_Node** get_complete_dd_task_list(void);
DD_Task_List_Node** get_overdue_dd_task_list(void);

// Task function definitions
static void DDT_Generator_Task( void *pvParameters );
static void User_Defined_Task( void *pvParameters );

// Reserve a space in memory for the queue handles. Each handle is just something
// 		FreeRTOS uses to reference the queues.
xQueueHandle event_queue_handle = 0;
xQueueHandle active_list_queue_handle = 0;
xQueueHandle complete_list_queue_handle = 0;
xQueueHandle overdue_list_queue_handle = 0;

/*-----------------------------------------------------------*/

int main( void )
{
	// Call the hardware initialization functions.
	prvSetupHardware();

	// Create the queue used by our tasks.
	event_queue_handle = xQueueCreate(8, sizeof( uint16_t ) );

	// Add each queue to the registry, for kernel aware debugging.
//	vQueueAddToRegistry( xStreetQueue_handle, "StreetQueue" );

	// REPORT: This is a workaround to preserve memory in scope
	DD_Task_Generator_Parameters params_0;
	DD_Task_Generator_Parameters params_1;
	DD_Task_Generator_Parameters params_2;

	// Call the helper function to create the dd task generators.
	DD_Task_Generator_Setup(&params_0, &params_1, &params_2);

	// Start the tasks and timer running.
	vTaskStartScheduler();

	while(1) {
		// This potentially will stop the parameters being garbage collected when scope ends.
	}

	return 0;
}

// REPORT: definitely somthine
void DD_Task_Generator_Setup( DD_Task_Generator_Parameters* param_pointer_0, DD_Task_Generator_Parameters* param_pointer_1, DD_Task_Generator_Parameters* param_pointer_2 ) {
	TaskHandle_t user_defined_handle_0 = 0;
	TaskHandle_t user_defined_handle_1 = 0;
	TaskHandle_t user_defined_handle_2 = 0;

	xTaskCreate( User_Defined_Task, "User_Defined_Task_0", configMINIMAL_STACK_SIZE, NULL, 2, &user_defined_handle_0 );
	xTaskCreate( User_Defined_Task, "User_Defined_Task_1", configMINIMAL_STACK_SIZE, NULL, 2, &user_defined_handle_1 );
	xTaskCreate( User_Defined_Task, "User_Defined_Task_2", configMINIMAL_STACK_SIZE, NULL, 2, &user_defined_handle_2 );

	DD_Task_Generator_Parameters params_0 = {22, PERIODIC, user_defined_handle_0, DD_TASK_0_EX_TIME, DD_TASK_0_PERIOD};
	DD_Task_Generator_Parameters params_1 = {1, PERIODIC, user_defined_handle_1, DD_TASK_1_EX_TIME, DD_TASK_1_PERIOD};
	DD_Task_Generator_Parameters params_2 = {2, PERIODIC, user_defined_handle_2, DD_TASK_2_EX_TIME, DD_TASK_2_PERIOD};

	// TODO: pvPortMalloc to solve memory problems!
	// or just regular malloc since we're in main rn.

	*param_pointer_0 = params_0;
	*param_pointer_1 = params_1; // (DD_Task_Generator_Parameters) {1, PERIODIC, user_defined_handle_1, DD_TASK_1_EX_TIME, DD_TASK_1_PERIOD};
	*param_pointer_2 = params_2; // (DD_Task_Generator_Parameters) {2, PERIODIC, user_defined_handle_2, DD_TASK_2_EX_TIME, DD_TASK_2_PERIOD};

	pvPortMalloc();

	printf("ddress in Setup(): %d\n", (int)param_pointer_0);

	printf("Check if it says 22 (gen id): %d\n", param_pointer_0->generator_id);
	printf("Check if it says 0 (type): %d\n", param_pointer_0->task_type);
	printf("Check if it says big number (handle): %d\n", param_pointer_0->f_task_handle);
	printf("Check if it says 95 (execut): %d\n", param_pointer_0->execution_time);
	printf("Check if it says 500 (period): %d\n", param_pointer_0->period);

	// Create each individual task, providing a relative priority to each.
	// The first null is a the task params; they must be any value w/ pointer type casted to null, later
	//		in the task the pointer must be type casted BACK to its original type for proper referencing.
	//xTaskCreate( DDT_Generator_Task, "Generator_0", configMINIMAL_STACK_SIZE, (void*) param_pointer_0, 2, NULL );
	xTaskCreate( DDT_Generator_Task, "Generator_0", configMINIMAL_STACK_SIZE, (void*) &params_0, 2, NULL );
//	xTaskCreate( DDT_Generator_Task, "Generator_1", configMINIMAL_STACK_SIZE, &params_1, 2, NULL );
//	xTaskCreate( DDT_Generator_Task, "Generator_2", configMINIMAL_STACK_SIZE, &params_2, 2, NULL );
}


/*-----------------------------------------------------------*/

static void DDT_Generator_Task( void *pvParameters )
{
	printf("\nVoid * Address in Task(): %d\n", (int)pvParameters);

	DD_Task_Generator_Parameters* params;
	params = (DD_Task_Generator_Parameters*) pvParameters;

	printf("Check if it says 22 (gen id): %d\n", params->generator_id);
	printf("Check if it says 0 (type): %d\n", params->task_type);
	printf("Check if it says big number (handle): %d\n", params->f_task_handle);
	printf("Check if it says 95 (execut): %d\n", params->execution_time);
	printf("Check if it says 500 (period): %d\n", params->period);

	printf("Params Address in Task(): %d\n", (int)params);

	uint8_t generator_id = params->generator_id; //TODO: mess with
	TaskHandle_t parent_f_task_handle = params->f_task_handle;
	DD_Task_Type task_type = params->task_type;
	int execution_time = params->execution_time;
	int period = params->period;

	int dd_task_id = 0;

	printf("Params of Generator %d: %d %d\n", generator_id, execution_time, period);

	while(1)
	{
		// ACTUALLY. This is the scheduler's responsibility; getting arrival time, calculating deadline + expected time.
		// TODO: do that
		// Get the time for the DD Task's creation.
//		uint32_t arrival_time = xTaskGetTickCount() * TICK_TO_MS_RATIO; // Ticks must be converted to MS // TODO: we could do this conversion in the monitor if we felt like it

		DD_Task new_dd_task = {
			parent_f_task_handle, // TaskHandle_t t_handle
			task_type, // DD_Task_Type type
			dd_task_id, // uint32_t task_id
			generator_id, // uint8_t parent_id
			(uint32_t) NULL, // uint32_t release_time,
			(uint32_t) NULL, // uint32_t absolute_deadline (release_time + period),
			(uint32_t) NULL, // uint32_t completion_time
		};

		// todo: fire off new dd_task into the event queue for the scheduler to consume
		//queuesend(new_dd_task);

		dd_task_id++;

		// Do we use vTaskdelay or our manual sleep function?
		vTaskDelay(pdMS_TO_TICKS(period));
	}
}

/*-----------------------------------------------------------*/

static void User_Defined_Task( void *pvParameters )
{
	while(1)
	{

	}
}

/*-----------------------------------------------------------*/

// This task is responsible for rotating through the current displayed traffic light (green, yellow, or red) and maintaining
// 		that light for a period proportional to the traffic flow.
//static void Traffic_Light_State_Task( void *pvParameters )
//{
//	while(1)
//	{
//
//	}
//}

/*-----------------------------------------------------------*/

//static void Monitor_Task( void *pvParameters )
//{
//	while(1)
//	{
//		TODO: combine DD_task's id w/ the generator task ID via it's handle
//	}
//}

/*-----------------------------------------------------------*/

void release_dd_task() {

}

void complete_dd_task() {

}

DD_Task_List_Node** get_active_dd_task_list() {

}

DD_Task_List_Node** get_completed_dd_task_list() {

}

DD_Task_List_Node** get_overdue_dd_task_list() {

}




/*-----------------------------------------------------------*/

// A quick and dirty method for manually delaying our system so that the peripherals have time
//		to catch up to our way faster CPU.
static void manualSleep(int time) {
	for (int i = time; i > 0; i--) {
		// Wait!
	}
}

// TODO: use the RTOS sleep thing
static void RTOS_Sleep(int time) {
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
//	ADC_Setup();

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
