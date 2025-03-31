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

// We are precalculating the maximum size any of the dd task list to limit the size of our task list queues.
#define NUM_DD_TASKS_0 HYPER_PERIOD / DD_TASK_0_PERIOD
#define NUM_DD_TASKS_1 HYPER_PERIOD / DD_TASK_1_PERIOD
#define NUM_DD_TASKS_2 HYPER_PERIOD / DD_TASK_2_PERIOD
#define MAX_LIST_SIZE NUM_DD_TASKS_0 + NUM_DD_TASKS_1 + NUM_DD_TASKS_2

#define TICK_TO_MS_RATIO 1000/configTICK_RATE_HZ

#define MONITOR_PERIOD_MS 50

#define GREEN_LED	LED4 // 0
#define AMBER_LED	LED3 // 1
#define RED_LED		LED5 // 2
#define BLUE_LED	LED6 // 3

typedef enum dd_task_type {
	PERIODIC,
	APERIODIC
} DD_Task_Type;

typedef struct dd_task_generator_parameters {
	uint32_t generator_id;
	DD_Task_Type task_type;
	int execution_time;
	int period;
} DD_Task_Generator_Parameters;

typedef struct dd_task {
	TaskHandle_t t_handle;
	DD_Task_Type type;
	uint32_t task_id;
	uint8_t parent_id;
	uint32_t time_to_execute;
	uint32_t release_time_ticks;
	uint32_t absolute_deadline_ticks;
	uint32_t completion_time_ticks;
} DD_Task;

typedef struct dd_task_list_node {
	DD_Task* task; // Since the generator allocates space for the DD_Task, to ensure we don't drop / make any copies, we pass around pointers to that first creation of the DD_Task.
	struct dd_task_list_node* next_node;
} DD_Task_List_Node;

typedef enum event_type {
	RELEASE,
	COMPLETE,
	OVERDUE,
	GET_ACTIVE,
	GET_COMPLETED,
	GET_OVERDUE
} Event_Type;

// Any getter event will have the dd_task = NULL, and the handling logic will not attempt to access the NULL dd_task.
typedef struct scheduler_event {
	Event_Type event_type;
	DD_Task* dd_task;
	xTimerHandle* overdue_timer;
} Scheduler_Event;

//typedef struct user_defined_task {
//	TaskHandle_t t_handle;
//	DD_Task_Type type;
//	uint32_t task_id;
//	uint8_t parent_id;
//	uint32_t release_time;
//	uint32_t absolute_deadline;
//	uint32_t completion_time;
//} User_Defined_Task;

/*-----------------------------------------------------------*/

// Hardware setup function definitions
static void prvSetupHardware( void );

/*-----------------------------------------------------------*/

// Helper functions
//static void manualSleep( int time );
void User_Def_Calc_For( int ms );
void DD_Task_Generator_Setup(
	DD_Task_Generator_Parameters* param_pointer_0,
	DD_Task_Generator_Parameters* param_pointer_1,
	DD_Task_Generator_Parameters* param_pointer_2
);
static void Overdue_Callback(TimerHandle_t timer);

// Event Interface Functions
//void release_dd_task(DD_Task dd_task); // we combined into one func handle_dd_task
//void complete_dd_task(uint32_t task_id);
void handle_dd_task( Event_Type event_type, DD_Task* dd_task );
DD_Task_List_Node** get_active_dd_task_list( void );
DD_Task_List_Node** get_complete_dd_task_list( void );
DD_Task_List_Node** get_overdue_dd_task_list( void );

// Task function definitions
static void DDT_Generator_Task( void *pvParameters );
static void User_Defined_Task( void *pvParameters );
static void Scheduler_Task( void *pvParameters );

/*-----------------------------------------------------------*/

// List Interaction Functions
DD_Task_List_Node* List_Init( void );
void Smart_Active_Insert(DD_Task_List_Node* list_head, DD_Task_List_Node* new_node);
DD_Task_List_Node* List_Pop(DD_Task_List_Node* list_head);
void List_Push(DD_Task_List_Node* list_head, DD_Task_List_Node* new_node);
void List_Priority_Update(DD_Task_List_Node* list_head);
void List_Sort(DD_Task_List_Node* list_head);
void Remove_Overdue(uint32_t taskId, DD_Task_List_Node* active_head, DD_Task_List_Node* overdue_head);
void Remove_Completed(uint32_t completed_dd_task_id, DD_Task_List_Node* active_head, DD_Task_List_Node* completed_head);
DD_Task_List_Node* getDDTaskById(uint32_t taskId, DD_Task_List_Node* list_head);

/*-----------------------------------------------------------*/

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

	// Create the queues used by our tasks.
	event_queue_handle = xQueueCreate( 2*(MAX_LIST_SIZE), sizeof(Scheduler_Event) ); // Decently large buffer queue for task communication
	active_list_queue_handle = xQueueCreate( 1, sizeof(DD_Task_List_Node**) ); // Length 1 buffer queue
	complete_list_queue_handle = xQueueCreate( 1, sizeof(DD_Task_List_Node**) ); // Length 1 buffer queue
	overdue_list_queue_handle = xQueueCreate( 1, sizeof(DD_Task_List_Node**) ); // Length 1 buffer queue

	// Add each queue to the registry, for kernel aware debugging.
	vQueueAddToRegistry( event_queue_handle, "EventQueue" );
	vQueueAddToRegistry( active_list_queue_handle, "ActiveListQueue" );
	vQueueAddToRegistry( complete_list_queue_handle, "CompleteListQueue" );
	vQueueAddToRegistry( overdue_list_queue_handle, "OverdueListQueue" );

	// REPORT: This is a workaround to preserve memory in scope, which didn't actually end up fixing the problem
	// Instead, malloc did. However, TODO - move this into the generator setup method.
	DD_Task_Generator_Parameters* params_0_buffer = pvPortMalloc(sizeof(DD_Task_Generator_Parameters));
	DD_Task_Generator_Parameters* params_1_buffer = pvPortMalloc(sizeof(DD_Task_Generator_Parameters));
	DD_Task_Generator_Parameters* params_2_buffer = pvPortMalloc(sizeof(DD_Task_Generator_Parameters));

	// Call the helper function to create the dd task generators.
	DD_Task_Generator_Setup(params_0_buffer, params_1_buffer, params_2_buffer);

	// We don't really need the handler, but passing it in just in case RTOS freaks out if we didn't.
//	TaskHandle_t dd_scheduler_handle;
	xTaskCreate( Scheduler_Task, "DD_Task_Scheduler", configMINIMAL_STACK_SIZE, NULL, 2, NULL ); // &dd_scheduler_handle );

	// Start the tasks and timer running (important for xTaskGetTickCount()).
	vTaskStartScheduler();

//	while(1) {
//		// This potentially will stop the parameters being garbage collected when scope ends.
//	}

	printf("===== End of Main Reached =====\n");

	return 0;
}

// REPORT: definitely something
void DD_Task_Generator_Setup( DD_Task_Generator_Parameters* param_pointer_0, DD_Task_Generator_Parameters* param_pointer_1, DD_Task_Generator_Parameters* param_pointer_2 ) {
	DD_Task_Generator_Parameters params_0 = {GREEN_LED, PERIODIC, DD_TASK_0_EX_TIME, DD_TASK_0_PERIOD};
	DD_Task_Generator_Parameters params_1 = {AMBER_LED, PERIODIC, DD_TASK_1_EX_TIME, DD_TASK_1_PERIOD};
	DD_Task_Generator_Parameters params_2 = {RED_LED, PERIODIC, DD_TASK_2_EX_TIME, DD_TASK_2_PERIOD};

	*param_pointer_0 = params_0;
	*param_pointer_1 = params_1; // (DD_Task_Generator_Parameters) {1, PERIODIC, user_defined_handle_1, DD_TASK_1_EX_TIME, DD_TASK_1_PERIOD};
	*param_pointer_2 = params_2; // (DD_Task_Generator_Parameters) {2, PERIODIC, user_defined_handle_2, DD_TASK_2_EX_TIME, DD_TASK_2_PERIOD};

	// Create each individual task, providing a relative priority to each.
	// The first null is a the task params; they must be any value w/ pointer type casted to null, later
	//		in the task the pointer must be type casted BACK to its original type for proper referencing.
	xTaskCreate( DDT_Generator_Task, "Generator_0", configMINIMAL_STACK_SIZE, (void*) param_pointer_0, 2, NULL );
	xTaskCreate( DDT_Generator_Task, "Generator_1", configMINIMAL_STACK_SIZE, (void*) param_pointer_1, 2, NULL );
	xTaskCreate( DDT_Generator_Task, "Generator_2", configMINIMAL_STACK_SIZE, (void*) param_pointer_2, 2, NULL );

	// TODO: why is attempting to free this pointer throwing a SIGINT? can't vPortFree with heap_1.c but we've changed it to 4 now
//	manualSleep(9999);
//	vPortFree(param_pointer_0);
}


/*-----------------------------------------------------------*/

static void DDT_Generator_Task( void *pvParameters )
{
	DD_Task_Generator_Parameters* params;
	params = (DD_Task_Generator_Parameters*) pvParameters;

//	printf("\nParams Address in Task(): %d\n", (int)params);
//	printf("Check if it says 22 (gen id): %d\n", params->generator_id);
//	printf("Check if it says 0 (type): %d\n", params->task_type);
//	printf("Check if it says big number (handle): %d\n", params->f_task_handle);
//	printf("Check if it says 95 (execut): %d\n", params->execution_time);
//	printf("Check if it says 500 (period): %d\n", params->period);

	uint8_t generator_id = params->generator_id; //TODO: mess with
	DD_Task_Type task_type = params->task_type;
	int execution_time = params->execution_time; // TODO: remove this if we end up reusing User Gen f_tasks
	int period_ms = params->period; // TODO: make all instances of "period" "period_ms" for clarity between ms and ticks

	int dd_task_id = 0;

	printf("Params of Generator %d: %d %d\n", generator_id, execution_time, period_ms);

	while(1)
	{
//		printf("Hi%d ", generator_id);

		// Get the time for the DD Task's creation.
		uint32_t release_time = xTaskGetTickCount() * TICK_TO_MS_RATIO; // Ticks must be converted to MS // TODO: we could do this conversion in the monitor if we felt like it
		uint32_t deadline_ticks = xTaskGetTickCount() + pdMS_TO_TICKS(period_ms);

		// Might malloc here if address space; we need to free these tasks at the end of their lifespan; maybe after moving them to their
		// various lists (active, complete, overdue) the info could be recorded and then the DD_Tasks vPortFree()'d
		DD_Task* new_dd_task = pvPortMalloc(sizeof(DD_Task));

		*new_dd_task = (DD_Task) {
			NULL, // TaskHandle_t t_handle
			task_type, // DD_Task_Type type
			dd_task_id, // uint32_t task_id
			generator_id, // uint8_t parent_id
			execution_time, // uint32_t time_to_execute
			release_time, // uint32_t release_time_ticks,
			deadline_ticks, // uint32_t absolute_deadline,
			(uint32_t) NULL, // uint32_t completion_time
		};

		// Create the f_task and hold onto it's handle.
		TaskHandle_t user_task_handle;
		xTaskCreate( User_Defined_Task, "User_Defined_Task", configMINIMAL_STACK_SIZE, (void *) new_dd_task, 2, &user_task_handle );

		// FUNKY - assign the handle back to the DD task we just passed into the f_task
		new_dd_task->t_handle = user_task_handle;

		// TODO: fire off new dd_task into the event queue for the scheduler to consume
//		handle_dd_task(RELEASE, new_dd_task);

		dd_task_id++;

		// Do we use vTaskdelay or our manual sleep function?
		vTaskDelay(pdMS_TO_TICKS(period_ms));
	}
}

/*-----------------------------------------------------------*/

// TODO: implement this
static void User_Defined_Task( void *pvParameters )
{
	// dereference pvParamaters, get execution time, enable LED, wait, disable LED, and then call handle_dd_event(COMPLETE) function
	// STM_EVAL_LEDInit(); Done in hardware setup func
	//	STM_EVAL_LEDOn(AMBER_LED);
	//	STM_EVAL_LEDOff(AMBER_LED);
	DD_Task* dd_task = (DD_Task*) pvParameters;
	uint32_t time_to_execute = dd_task->time_to_execute;
	STM_EVAL_LEDOn(dd_task->parent_id);
	User_Def_Calc_For(time_to_execute);
	STM_EVAL_LEDOff(dd_task->parent_id);
	handle_dd_task(COMPLETE, dd_task);

	// TODO: Place a breakpoint in this function so that every time one finishes we can pause and take a look at how the
	// 		system is performing in that particular snapshot of time.
}

/*-----------------------------------------------------------*/

static void Overdue_Callback(TimerHandle_t timer) {
	Scheduler_Event new_overdue_event = {OVERDUE, NULL, &timer};

	if (xQueueSend(event_queue_handle, &new_overdue_event, 500)) {
		// do nothing
	}
	else {
		printf("Overdue send to Event Queue failed!");
	}

}

static void Scheduler_Task( void *pvParameters )
{
	DD_Task_List_Node* active_dummy_head = List_Init();
	DD_Task_List_Node* overdue_dummy_head = List_Init();
	DD_Task_List_Node* complete_dummy_head = List_Init();

	// We may not have to use this? Not sure of FreeRTOS kills one-shot timers after they are completed.
//	xTimerHandle active_timers[100];

	Scheduler_Event incoming_event;

//	DD_Task_List_Node* list_head = List_Init();

	while(1)
	{
		if ( xQueueReceive(event_queue_handle, &incoming_event, 500)) {
			if (incoming_event.event_type == RELEASE) {
				printf("RECIEVED RELEASE FROM GENERATOR %d\n", incoming_event.dd_task->parent_id);
				printf("Task ID: %d\n", incoming_event.dd_task->task_id);

//				DD_Task* task_to_release = incoming_event.dd_task;
//				uint32_t overdue_deadline_ticks = task_to_release->absolute_deadline_ticks;
//				uint32_t timer_id = task_to_release->task_id;
//				// Sends an overdue event if timer expires via the Overdue_Callback() function.
//				xTimerHandle new_timer = xTimerCreate("DD_Task_Overdue_Timer", overdue_deadline_ticks, pdFALSE, (void* ) &timer_id, Overdue_Callback);
//
//				// TODO: actually add the task to the list
//				DD_Task_List_Node* new_active_node = pvPortMalloc(sizeof(DD_Task_List_Node));
//				*new_active_node = (DD_Task_List_Node) {task_to_release, NULL};
//				Smart_Active_Insert(active_dummy_head, new_active_node);
			}
			else if (incoming_event.event_type == COMPLETE) {
				printf("EVENT TRIGGER - COMPLETE! :)");
				// Does complete need access to the f_task?
				uint32_t completed_dd_task_id = incoming_event.

			}
			else if (incoming_event.event_type == OVERDUE) {
				// The callback function only has access to it's own timer; luckily, we assign the timer's ID to be the EXACT SAME as the DD_Task's id!
				xTimerHandle* overdue_timer = incoming_event.overdue_timer;
				// pvTimerGetTimerID returns a void pointer
				uint32_t timer_id = (uint32_t) pvTimerGetTimerID(overdue_timer); // TODO: could export to the callback function's responsibility, to save time in the Scheduler
				
				// WAIT WAIT WE COULD OPTIMIZE THIS
				// WE DON'T NEED TO FIND THE DD_TASK IN THE LIST if we just vTaskDelete INSIDE THE REMOVE() METHOD
				// DD_Task* overdue_dd_task = getDDTaskById(timer_id, active_dummy_head);
				// vTaskDelete(overdue_dd_task->t_handle);
				// overdue_dd_task->t_handle = NULL;

				Remove_Overdue(timer_id, active_dummy_head, overdue_dummy_head);
			}
			else if (incoming_event.event_type == GET_ACTIVE) {
				if( xQueueSend(active_list_queue_handle, &active_dummy_head, 500) ) // Lab 0/1 used 500; could tweak?
				{
					// do nothing
				}
				else {
					printf("-- Sending to active_list_queue Failed!");
				}
			}
			else if (incoming_event.event_type == GET_COMPLETED) {
				if( xQueueSend(completed_list_queue_handle, &completed_dummy_head, 500) ) // Lab 0/1 used 500; could tweak?
				{
					// do nothing
				}
				else {
					printf("-- Sending completed_list_queue Failed!");
				}
			}
			else if (incoming_event.event_type == GET_OVERDUE) {
				if( xQueueSend(completed_overdue_queue_handle, &overdue_dummy_head, 500) ) // Lab 0/1 used 500; could tweak?
				{
					// do nothing
				}
				else {
					printf("-- Sending to overdue_list_queue Failed!");
				}
			}
			else {
				printf("UNHANDLED INCOMING EVENT TYPE!!!\n");
			}
		}
		else {
			printf("Scheduler - No events to consume.\n");
		}
	}
}

/*-----------------------------------------------------------*/

static void Monitor_Task( void *pvParameters )
{
	uint8_t num_active;
	uint8_t num_overdue;
	uint8_t num_complete;

	// MAYBE: combine DD_task's id w/ the generator task ID via it's handle

	while(1)
	{
		num_active = get_active_dd_task_list();
		num_overdue = get_overdue_dd_task_list();
		num_complete = get_complete_dd_task_list();

		printf("### Time %d: %d Active Tasks, %d Overdue Tasks, and %d Complete Tasks. ###\n", xTaskGetTickCount(), num_active, num_overdue, num_complete);
	
		vTaskDelay(pdMS_TO_TICKS(MONITOR_PERIOD_MS));
	}
}

/*-----------------------------------------------------------*/

// Combined our release and complete into one function because they did the exact same thing. DRY PRINCIPLE!
void handle_dd_task( Event_Type event_type, DD_Task* dd_task ) {
	Scheduler_Event new_event = {event_type, dd_task, NULL};

	if( xQueueSend(event_queue_handle, &new_event, 500) ) // Lab 0/1 used 500; could tweak?
	{
		// do nothing
	}
	else {
		printf("-- DD_Event %d: Sending to Event Queue Failed!", event_type);
	}
}

DD_Task_List_Node** get_active_dd_task_list() {
	DD_Task_List_Node** p;
	Scheduler_Event new_event = {GET_ACTIVE, NULL, NULL};

	if( xQueueSend(event_queue_handle, &new_event, 500) )
	{
		if (xQueueReceive(active_list_queue_handle, &p, 1000))
		{
			return p;
		}
		else {
			printf("-- GET_ACTIVE Could not consume from active_list_queue !\n");
		}
	}
	else {
		printf("-- GET_ACTIVE Event Sending to Event Queue Failed!\n");
	}

	return NULL;
}

DD_Task_List_Node** get_completed_dd_task_list() {
	DD_Task_List_Node** p;
	Scheduler_Event new_event = {GET_COMPLETED, dd_task, NULL};

	if( xQueueSend(event_queue_handle, &new_event, 500) )
	{
		// TODO - maybe introduce a manual sleep timer here, just to give the DD Scheduler a chance?
		if (xQueueReceive(completed_list_queue_handle, &p, 1000))
		{
			return p;
		}
		else {
			printf("-- GET_COMPLETED Could not consume from completed_list_queue !\n");
		}
	}
	else {
		printf("-- GET_COMPLETED Event Sending to Event Queue Failed!\n");
	}

	return NULL;
}

DD_Task_List_Node** get_overdue_dd_task_list() {
	DD_Task_List_Node** p;
	Scheduler_Event new_event = {GET_OVERDUE, dd_task, NULL};

	if( xQueueSend(event_queue_handle, &new_event, 500) )
	{
		// TODO - maybe introduce a manual sleep timer here, just to give the DD Scheduler a chance?
		if (xQueueReceive(overdue_list_queue_handle, &p, 1000))
		{
			return p;
		}
		else {
			printf("-- GET_OVERDUE Could not consume from overdue_list_queue !\n");
		}
	}
	else {
		printf("-- GET_OVERDUE Event Sending to Event Queue Failed!\n");
	}

	return NULL;
}

/*-----------------------------------------------------------*/

// Update these to work with nodes that contain pointers to DD_Tasks
DD_Task_List_Node* List_Init( void ) {
	DD_Task_List_Node* head = pvPortMalloc(sizeof(DD_Task_List_Node));
	DD_Task_List_Node* tail = pvPortMalloc(sizeof(DD_Task_List_Node));
	head->task = NULL;
	head->next_node = tail;
	tail->task = NULL;
	tail->next_node = NULL;
	return head;
}

DD_Task_List_Node* getDDTaskById(uint32_t taskId, DD_Task_List_Node* list_head) {
	DD_Task_List_Node* curr = list_head->next_node;

	// First we check if the list is empty:
	if (curr == NULL) {
		printf("LIST ERROR - GETBYID ON EMPTY LIST");
		return NULL;
	}
	// We traverse the list while we haven't ran out of space and have yet to find the node with our desired ID
	// This is potentially risky if C does not behave as expected (checking 1st condition then 2nd linearly)
	while ((curr->next_node != NULL) && (curr->task->task_id != taskId)) {
		curr = curr->next_node;
	}
	// If we're at the end of the line (i.e. at dummy tail), we could not find the task by its id.
	if (curr->next_node == NULL) {
		printf("LIST ERROR - COULD NOT FIND NODE BY ID");
		return NULL;
	}

	return curr;
}

DD_Task_List_Node* List_Pop(DD_Task_List_Node* list_head) {
	// Since we have a dummy tail, there will always be at least one node after the head.
	// So we check if the dummy head does NOT point to the dummy tail
	if (list_head->next_node->next_node != NULL) {
		DD_Task_List_Node* node_to_pop = list_head->next_node; // Grab ref to first node (slot 1)
		list_head->next_node = list_head->next_node->next_node; // Head points to node at slot 2
		node_to_pop->next_node = NULL; // Isolate popped node from list
		return node_to_pop;
	}
	else {
		printf("LIST ERROR - Pop from Empty List");
		return NULL;
	}
}

// We assume that the given nodes have already been malloc'd
void List_Push(DD_Task_List_Node* list_head, DD_Task_List_Node* new_node) {
	new_node->next_node = list_head->next_node;
	list_head->next_node = new_node;
}

void Smart_Active_Insert(DD_Task_List_Node* list_head, DD_Task_List_Node* new_node) {
	DD_Task_List_Node* curr = list_head->next_node;
	DD_Task_List_Node* prev;

	// First we check if the list is empty:
	if (curr->next_node == NULL) {
		new_node->next_node = list_head->next_node; // Point new node to dummy tail
		list_head->next_node = new_node; // Point dummy head to new node
		return;
	}
	// We traverse the list while we haven't ran out of space and have yet to find a node with a larger deadline than ours
	while ((curr->next_node != NULL) && (new_node->task->absolute_deadline_ticks <= curr->task->absolute_deadline_ticks)) {
		prev = curr;
		curr = curr->next_node;
	}
	// If we're at the end of the line (i.e. we've arrived at the dummy tail), we are the largest deadline and add to end of list.
	if (curr->next_node == NULL) {
		prev->next_node = new_node; // Make the node before the dummy tail point to the new node
		new_node->next_node = curr; // Make the new node point to the dummy tail
		return;
	}

	// Otherwise, we have found the correct position to insert into.
	prev->next_node = new_node;
	new_node->next_node = curr;

	// IDEALLY because we are inserting into a position which invokes sorted priority, we do not have to further sort the list.
	// We expect all deadlines to approach at equal speed, so there is no need to sort the list.

	// If this doesn't actually work, we just implement a sort function and call it here.
	// sortList();
}

void List_Priority_Update(DD_Task_List_Node* list_head) {
	DD_Task_List_Node* curr = list_head->next_node;

	// If we start at the dummy tail...
	if (curr->next_node == NULL) {
		printf("LIST ERROR - REMOVE OVERDUE ON EMPTY LIST");
		return;
	}

	vTaskPrioritySet(curr->task->t_handle, 1);

	while (curr->next_node != NULL) {
		vTaskPrioritySet(curr->next_node->task->t_handle, 0);
		curr = curr->next_node;
	}
}

void List_Sort(DD_Task_List_Node* list_head) {
	// Potentially, hopefully, we don't have to do this.
}

void Remove_Overdue(uint32_t overdue_timer_id, DD_Task_List_Node* active_head, DD_Task_List_Node* overdue_head) {
	// while the given nodes next is not null, and until the id matches continue to traverse
	DD_Task_List_Node* curr = active_head->next_node;
	DD_Task_List_Node* prev;

	// First we check if the list is empty (i.e. dummy head points to dummy tail):
	if (curr->next_node == NULL) {
		printf("LIST ERROR - REMOVE OVERDUE ON EMPTY LIST");
		return;
	}
	// We traverse the list until either we find what we're looking for, or we run out of list.
	while ((curr->next_node != NULL) && (overdue_timer_id != curr->task->task_id)) {
		prev = curr;
		curr = curr->next_node;
	}
	// If we're at the end of the line...
	if (curr->next_node == NULL) {
		printf("LIST ERROR - COULD NOT FIND OVERDUE DD TASK");
		return;
	}

	// Otherwise we must've found it so we set prev to curr's next, and curr's next to NULL.
	prev->next_node = curr->next_node; // Point the previous to current's next node, effectively removing current from the list.
	curr->next_node = NULL; // Isolate current from the list.

	// Remove the corresponding f_task so that the overdue task does not get allocated any time/resources.
	vTaskDelete(curr->t_handle);
	curr->t_handle = NULL;

	// Add the overdue task to the front of the overdue list.
	List_Push(overdue_head, curr);

	// Should we also set the priority of this task to 0 (allegedly idle)? No let's do that in scheduler.
	// We actually delete the task in the scheduler before this function even gets called, so no need to change the priority of a dead task.

	// This might be dangerous
//	vPortFree(curr);
}

void Remove_Completed(uint32_t completed_dd_task_id, DD_Task_List_Node* active_head, DD_Task_List_Node* completed_head) {
	// while the given nodes next is not null, and until the id matches continue to traverse
	DD_Task_List_Node* curr = active_head->next_node;
	DD_Task_List_Node* prev;

	// First we check if the list is empty (i.e. dummy head points to dummy tail):
	if (curr->next_node == NULL) {
		printf("LIST ERROR - REMOVE COMPLETED ON EMPTY LIST");
		return;
	}
	// We traverse the list until either we find what we're looking for, or we run out of list (i.e. we're on the dummy tail).
	while ((curr->next_node != NULL) && (completed_dd_task_id != curr->task->task_id)) {
		prev = curr;
		curr = curr->next_node;
	}
	// If we're at the end of the line (where curr = dummy tail)...
	if (curr->next_node == NULL) {
		printf("LIST ERROR - COULD NOT FIND COMPLETED DD TASK");
		return;
	}

	// Otherwise we must've found it so we set prev to curr's next, and curr's next to NULL.
	prev->next_node = curr->next_node; // Point the previous to current's next node, effectively removing current from the list.
	curr->next_node = NULL; // Isolate current from the list.

	// Remove the corresponding f_task, mostly for posterity.
	vTaskDelete(curr->t_handle);
	curr->t_handle = NULL;

	// Add the overdue task to the front of the overdue list.
	List_Push(completed_head, curr);
}

uint8_t Get_List_Length(DD_Task_List_Node* list_head) {

	uint8_t length = 0;
	DD_Task_List_Node* curr = active_head->next_node;

	// We traverse the list until we run out of list (i.e. we're on the dummy tail).
	while (curr->next_node != NULL) {
		length++;
		curr = curr->next_node;
	}

	return length;
}

void DEBUG_Print_List(DD_Task_List_Node* list_head) {
	DD_Task_List_Node* curr = list_head->next_node;

	// While we're not at the dummy tail...
	while (curr->next_node != NULL) {
		printf("\nDEBUG - Printing Node in list:\n");
		printf("Generator ID: %d\n", curr->task->parent_id);
		printf("Task ID: %d\n", curr->task->task_id);
		printf("Absolute Deadline: %d\n", curr->task->absolute_deadline_ticks);
		printf("Release Time: %d\n", curr->task->release_time_ticks);
		printf("Time to Complete: %d\n", curr->task->completion_time_ticks);

		curr = curr->next_node;
	}
}

/*-----------------------------------------------------------*/

// A quick and dirty method for manually delaying our system so that the peripherals have time
//		to catch up to our way faster CPU.
//static void manualSleep(int time) {
//	for (int i = time; i > 0; i--) {
//		// Wait!
//	}
//}

// Dummy callback function, as nothing special needs to happen on timer end (these timers are one-shot)
//static void Timer_Callback(TimerHandle_t timer) {
//	return;
//}

void User_Def_Calc_For(int ms) {
	// TODO: WHY IS THIS AUTO BREAKPOINTING???1?!?!?!??!??!?!?!??!??!?!:
	TimerHandle_t timer = xTimerCreate("General Timer", pdMS_TO_TICKS(ms), pdFALSE, (void* ) 0, NULL); //Timer_Callback);
	printf("Timer created?");

	if ( xTimerStart(timer, 0) == pdFAIL) {
		printf("Timer Failed to execute.");
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

	printf("MALLOC FAILED HOOK");

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
	STM_EVAL_LEDInit(AMBER_LED);
	STM_EVAL_LEDInit(GREEN_LED);
	STM_EVAL_LEDInit(RED_LED);
	STM_EVAL_LEDInit(BLUE_LED);
	NVIC_SetPriorityGrouping( 0 );
}
