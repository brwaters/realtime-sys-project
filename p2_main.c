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
#include <unistd.h>

/*-----------------------------------------------------------*/
#define mainQUEUE_LENGTH 100 // I don't think we use this for anything?

// Benchmark 1
#define DD_TASK_0_EX_TIME 95
#define DD_TASK_1_EX_TIME 150
#define DD_TASK_2_EX_TIME 250
#define DD_TASK_0_PERIOD 500
#define DD_TASK_1_PERIOD 500
#define DD_TASK_2_PERIOD 750
#define HYPER_PERIOD 1500

// Benchmark 2
// #define DD_TASK_0_EX_TIME 95
// #define DD_TASK_1_EX_TIME 150
// #define DD_TASK_2_EX_TIME 250
// #define DD_TASK_0_PERIOD 250
// #define DD_TASK_1_PERIOD 500
// #define DD_TASK_2_PERIOD 750
// #define HYPER_PERIOD 1500

// Benchmark 3
//#define DD_TASK_0_EX_TIME 100
//#define DD_TASK_1_EX_TIME 200
//#define DD_TASK_2_EX_TIME 200
//#define DD_TASK_0_PERIOD 500
//#define DD_TASK_1_PERIOD 500
//#define DD_TASK_2_PERIOD 500
//#define HYPER_PERIOD 500

// We are precalculating the maximum size any of the dd task list to limit the size of our task list queues.
#define NUM_DD_TASKS_0 HYPER_PERIOD / DD_TASK_0_PERIOD
#define NUM_DD_TASKS_1 HYPER_PERIOD / DD_TASK_1_PERIOD
#define NUM_DD_TASKS_2 HYPER_PERIOD / DD_TASK_2_PERIOD
#define MAX_LIST_SIZE NUM_DD_TASKS_0 + NUM_DD_TASKS_1 + NUM_DD_TASKS_2

#define TICK_TO_MS_RATIO (1000/configTICK_RATE_HZ)

#define MONITOR_PERIOD_MS 50

#define GREEN_LED	LED4 // 0
#define AMBER_LED	LED3 // 1
#define RED_LED		LED5 // 2
#define BLUE_LED	LED6 // 3

#define GENERATOR_TASK_PRIORITY configMAX_PRIORITIES - 3
#define SCHEDULER_TASK_PRIORITY configMAX_PRIORITIES - 1
#define MONITOR_TASK_PRIORITY configMAX_PRIORITIES - 2
#define USER_TASK_PRIORITY 0

#define LIST_DEBUG 0


typedef enum dd_task_type {
	PERIODIC,
	APERIODIC,
	DUMMY
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
	uint8_t priority;
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
	uint32_t timer_id;
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
void handle_dd_task( Event_Type event_type, DD_Task* dd_task );
DD_Task_List_Node* get_active_dd_task_list( void );
DD_Task_List_Node* get_completed_dd_task_list( void );
DD_Task_List_Node* get_overdue_dd_task_list( void );

// Task function definitions
static void DDT_Generator_Task( void *pvParameters );
static void User_Defined_Task( void *pvParameters );
static void Scheduler_Task( void *pvParameters );
static void Monitor_Task( void *pvParameters );

void DEBUG_Print_List(DD_Task_List_Node* list_head);

/*-----------------------------------------------------------*/

// List Interaction Functions
DD_Task_List_Node* List_Init( void );
void Smart_Active_Insert(DD_Task_List_Node* list_head, DD_Task_List_Node* new_node);
DD_Task_List_Node* List_Pop(DD_Task_List_Node* list_head);
void List_Push(DD_Task_List_Node* list_head, DD_Task_List_Node* new_node);
void List_Priority_Update(DD_Task_List_Node* list_head);
void List_Sort(DD_Task_List_Node* list_head);
uint8_t Remove_Overdue(uint32_t taskId, DD_Task_List_Node* active_head, DD_Task_List_Node* overdue_head);
uint8_t Remove_Completed(uint32_t completed_dd_task_id, DD_Task_List_Node* active_head, DD_Task_List_Node* completed_head);
uint8_t Get_List_Length(DD_Task_List_Node* list_head);
DD_Task_List_Node* Get_Node_By_Id(uint32_t taskId, DD_Task_List_Node* list_head);

void DEBUG_Create_And_Smart_Insert_Wrapper(DD_Task_List_Node* list_head, int id, int parent_id, int exec_time, int release_time, int deadline);
void DEBUG_Print_List(DD_Task_List_Node* list_head);

/*-----------------------------------------------------------*/

// Reserve a space in memory for the queue handles. Each handle is just something
// 		FreeRTOS uses to reference the queues.
xQueueHandle event_queue_handle = 0;
xQueueHandle active_list_queue_handle = 0;
xQueueHandle completed_list_queue_handle = 0;
xQueueHandle overdue_list_queue_handle = 0;

TaskHandle_t scheduler_hotfix_handle;

/*-----------------------------------------------------------*/

int main( void )
{
	// Call the hardware initialization functions.
	prvSetupHardware();

	// Create the queues used by our tasks.
	event_queue_handle = xQueueCreate( 2*(MAX_LIST_SIZE), sizeof(Scheduler_Event) ); // Decently large buffer queue for task communication
	active_list_queue_handle = xQueueCreate( 1, sizeof(DD_Task_List_Node**) ); // Length 1 buffer queue
	completed_list_queue_handle = xQueueCreate( 1, sizeof(DD_Task_List_Node**) ); // Length 1 buffer queue
	overdue_list_queue_handle = xQueueCreate( 1, sizeof(DD_Task_List_Node**) ); // Length 1 buffer queue

	// Add each queue to the registry, for kernel aware debugging.
	vQueueAddToRegistry( event_queue_handle, "EventQueue" );
	vQueueAddToRegistry( active_list_queue_handle, "ActiveListQueue" );
	vQueueAddToRegistry( completed_list_queue_handle, "CompleteListQueue" );
	vQueueAddToRegistry( overdue_list_queue_handle, "OverdueListQueue" );

	DD_Task_Generator_Parameters* params_0_buffer = pvPortMalloc(sizeof(DD_Task_Generator_Parameters));
	DD_Task_Generator_Parameters* params_1_buffer = pvPortMalloc(sizeof(DD_Task_Generator_Parameters));
	DD_Task_Generator_Parameters* params_2_buffer = pvPortMalloc(sizeof(DD_Task_Generator_Parameters));

	// Call the helper function to create the dd task generators.
	DD_Task_Generator_Setup(params_0_buffer, params_1_buffer, params_2_buffer);


	xTaskCreate( Scheduler_Task, "DD_Task_Scheduler", configMINIMAL_STACK_SIZE, NULL, SCHEDULER_TASK_PRIORITY, &scheduler_hotfix_handle ); // &dd_scheduler_handle );
	xTaskCreate( Monitor_Task, "DD_Task_Monitor", configMINIMAL_STACK_SIZE, NULL, MONITOR_TASK_PRIORITY, NULL ); // &dd_scheduler_handle );

	// Start the tasks and timer running (important for xTaskGetTickCount()).
	vTaskStartScheduler();

	return 0;
}

void DD_Task_Generator_Setup( DD_Task_Generator_Parameters* param_pointer_0, DD_Task_Generator_Parameters* param_pointer_1, DD_Task_Generator_Parameters* param_pointer_2 ) {
	DD_Task_Generator_Parameters params_0 = {GREEN_LED, PERIODIC, DD_TASK_0_EX_TIME, DD_TASK_0_PERIOD};
	DD_Task_Generator_Parameters params_1 = {AMBER_LED, PERIODIC, DD_TASK_1_EX_TIME, DD_TASK_1_PERIOD};
	DD_Task_Generator_Parameters params_2 = {RED_LED, PERIODIC, DD_TASK_2_EX_TIME, DD_TASK_2_PERIOD};

	*param_pointer_0 = params_0;
	*param_pointer_1 = params_1;
	*param_pointer_2 = params_2;

	xTaskCreate( DDT_Generator_Task, "Generator_0", configMINIMAL_STACK_SIZE, (void*) param_pointer_0, 3, NULL );
	xTaskCreate( DDT_Generator_Task, "Generator_1", configMINIMAL_STACK_SIZE, (void*) param_pointer_1, 3, NULL );
	xTaskCreate( DDT_Generator_Task, "Generator_2", configMINIMAL_STACK_SIZE, (void*) param_pointer_2, 3, NULL );
}


/*-----------------------------------------------------------*/

static void DDT_Generator_Task( void *pvParameters )
{
	DD_Task_Generator_Parameters* params;
	params = (DD_Task_Generator_Parameters*) pvParameters;

	uint8_t generator_id = params->generator_id;
	DD_Task_Type task_type = params->task_type;
	int execution_time = params->execution_time;
	int period_ms = params->period;

	uint8_t num_tasks_per_hyperperiod = HYPER_PERIOD / period_ms;

	int num_dd_tasks_generated = 0;

	while(1)
	{
	// Get the time for the DD Task's creation.
	uint32_t release_time = xTaskGetTickCount() * TICK_TO_MS_RATIO;
	uint32_t deadline_ticks = xTaskGetTickCount() + pdMS_TO_TICKS(period_ms);

	DD_Task* new_dd_task = pvPortMalloc(sizeof(DD_Task));
	*new_dd_task = (DD_Task) {
		NULL, // TaskHandle_t t_handle
		task_type, // DD_Task_Type type
		num_dd_tasks_generated + 100 * generator_id, // uint32_t task_id
		generator_id, // uint8_t parent_id
		USER_TASK_PRIORITY,
		execution_time, // uint32_t time_to_execute
		release_time, // uint32_t release_time_ticks,
		deadline_ticks, // uint32_t absolute_deadline,
		(uint32_t) NULL, // uint32_t completion_time
	};

	handle_dd_task(RELEASE, new_dd_task);

		num_dd_tasks_generated++;
	}
		vTaskDelay(pdMS_TO_TICKS(period_ms));
	}
}

/*-----------------------------------------------------------*/

static void Complete_Callback(TimerHandle_t timer) {
	uint32_t complete_task_ID = *((uint32_t*) pvTimerGetTimerID(timer));

	if (complete_task_ID < 100) {
		STM_EVAL_LEDOff(GREEN_LED);
	}
	else if (complete_task_ID < 200) {
		STM_EVAL_LEDOff(AMBER_LED);
	}
	else {
		STM_EVAL_LEDOff(RED_LED);
	}

	Scheduler_Event new_complete_event = {COMPLETE, NULL, complete_task_ID};

	if (xQueueSend(event_queue_handle, &new_complete_event, 500)) {
		// do nothing
	}
	else {
		printf("Overdue send to Event Queue failed!");
	}
}

static void User_Defined_Task( void *pvParameters )
{
	DD_Task* dd_task = (DD_Task*) pvParameters;
	uint32_t time_to_execute = dd_task->time_to_execute;
	STM_EVAL_LEDOn(dd_task->parent_id);

	TimerHandle_t timer = xTimerCreate("General Timer", pdMS_TO_TICKS(time_to_execute), pdFALSE, &(dd_task->task_id), Complete_Callback); //Timer_Callback);
	if ( xTimerStart(timer, 0) == pdFAIL) {
		printf("Timer Failed to execute.");
	}

	while(1) {
		vTaskDelay(1);
	}
}

/*-----------------------------------------------------------*/

static void Overdue_Callback(TimerHandle_t timer) {
	uint32_t overdue_task_ID = *((uint32_t*) pvTimerGetTimerID(timer));
	Scheduler_Event new_overdue_event = {OVERDUE, NULL, overdue_task_ID};
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
	DD_Task_List_Node* completed_dummy_head = List_Init();

	// This keeps track of events for logging purposes, but also servces as an index for the overdue timers.
	uint16_t num_setter_events = 0;

	// We may not have to use this? Not sure of FreeRTOS kills one-shot timers after they are completed.
	xTimerHandle active_overdue_timers[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	uint8_t num_timers = 0;

	Scheduler_Event incoming_event;

	while(1)
	{
		if ( xQueueReceive(event_queue_handle, &incoming_event, 500)) {
			if (incoming_event.event_type == RELEASE) {
				num_setter_events++;
				printf("%d | Task %d Released | %d ms.\n", num_setter_events, incoming_event.dd_task->task_id, (incoming_event.dd_task->release_time_ticks) * TICK_TO_MS_RATIO);

				TaskHandle_t user_task_handle;
				xTaskCreate( User_Defined_Task, "User_Defined_Task", configMINIMAL_STACK_SIZE, (void *) incoming_event.dd_task, USER_TASK_PRIORITY, &user_task_handle);
				incoming_event.dd_task->t_handle = user_task_handle;

				// Create a timer to determine if this event ever becomes overdue.
				uint32_t timer_id = incoming_event.dd_task->task_id;

				uint32_t timer_period;
				if (incoming_event.dd_task->parent_id == 0) {
					timer_period = DD_TASK_0_PERIOD;
				}
				else if (incoming_event.dd_task->parent_id == 1) {
					timer_period = DD_TASK_1_PERIOD;
				}
				else {
					timer_period = DD_TASK_2_PERIOD;
				}

				TimerHandle_t timer = xTimerCreate("Overdue Timer", timer_period, pdFALSE, &timer_id, Complete_Callback);

				if ( xTimerStart(timer, 0) == pdFAIL) {
					printf("Overdue Timer Failed to execute.");
				}
				num_timers++;

				// Add the new node to the list.
				DD_Task_List_Node* new_active_node = pvPortMalloc(sizeof(DD_Task_List_Node));
				*new_active_node = (DD_Task_List_Node) {incoming_event.dd_task, NULL};
				Smart_Active_Insert(active_dummy_head, new_active_node);
			}
			else if (incoming_event.event_type == COMPLETE) {
				uint32_t complete_time = (uint32_t) xTaskGetTickCount();// * TICK_TO_MS_RATIO;
				if (Remove_Completed(incoming_event.timer_id, active_dummy_head, completed_dummy_head) == 0) {
					num_setter_events++;
					printf("%d | Task %d Completed | %d ms.\n", num_setter_events, incoming_event.timer_id, complete_time);
				}
			}
			else if (incoming_event.event_type == OVERDUE) {
				uint32_t overdue_time = (uint32_t) xTaskGetTickCount();
				if (Remove_Overdue(incoming_event.timer_id, active_dummy_head, overdue_dummy_head) == 0) {
					num_setter_events++;
					printf("%d | Task %d Overdue | %d ms.\n", num_setter_events, incoming_event.timer_id, overdue_time);
				}
			}
			else if (incoming_event.event_type == GET_ACTIVE) {
				if( xQueueSend(active_list_queue_handle, &active_dummy_head, 500) ) // Lab 0/1 used 500; could tweak?
				{
					// do nothing
				}
				else {
					printf("-- Sending to active_list_queue Failed!\n");
				}
			}
			else if (incoming_event.event_type == GET_COMPLETED) {
				if( xQueueSend(completed_list_queue_handle, &completed_dummy_head, 500) ) // Lab 0/1 used 500; could tweak?
				{
					// do nothing
				}
				else {
					printf("-- Sending completed_list_queue Failed!\n");
				}
			}
			else if (incoming_event.event_type == GET_OVERDUE) {
				if( xQueueSend(overdue_list_queue_handle, &overdue_dummy_head, 500) ) // Lab 0/1 used 500; could tweak?
				{
					// do nothing
				}
				else {
					printf("-- Sending to overdue_list_queue Failed!\n");
				}
			}
			else {
				printf("UNHANDLED INCOMING EVENT TYPE!!!\n");
			}
		}
		else {
			printf("Scheduler - No events to consume.\n");
		}

		taskYIELD();
	}
}

/*-----------------------------------------------------------*/

static void Monitor_Task( void *pvParameters )
{
	DD_Task_List_Node* active_dummy_head = get_active_dd_task_list();
	DD_Task_List_Node* overdue_dummy_head = get_overdue_dd_task_list();
	DD_Task_List_Node* completed_dummy_head = get_completed_dd_task_list();

	uint8_t num_active;
	uint8_t num_overdue;
	uint8_t num_complete;


	while(1)
	{
		num_active = Get_List_Length(active_dummy_head);
		num_overdue = Get_List_Length(overdue_dummy_head);
		num_complete = Get_List_Length(completed_dummy_head);

		printf("### Time %d: %d Active Tasks, %d Overdue Tasks, and %d Complete Tasks. ###\n", (uint32_t) xTaskGetTickCount(), num_active, num_overdue, num_complete);

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

DD_Task_List_Node* get_active_dd_task_list() {
	DD_Task_List_Node* p;
	Scheduler_Event new_event = {GET_ACTIVE, NULL, NULL};

	if( xQueueSend(event_queue_handle, &new_event, 500) )
	{
		if (xQueueReceive(active_list_queue_handle, &p, 5000))
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

DD_Task_List_Node* get_completed_dd_task_list( void ) {
	Scheduler_Event new_event = {GET_COMPLETED, NULL, NULL};

	if( xQueueSend(event_queue_handle, &new_event, 500) )
	{
		if (xQueueReceive(completed_list_queue_handle, &p, 5000))
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

DD_Task_List_Node* get_overdue_dd_task_list() {
	Scheduler_Event new_event = {GET_OVERDUE, NULL, NULL};

	if( xQueueSend(event_queue_handle, &new_event, 500) )
	{
		if (xQueueReceive(overdue_list_queue_handle, &p, 5000))
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

	head->task = pvPortMalloc(sizeof(DD_Task));
	*(head->task) = (DD_Task) {
		NULL,
		DUMMY,
		0,
		0,
		0,
		0,
		0,
		0
	};
	head->next_node = tail;

	tail->task = pvPortMalloc(sizeof(DD_Task));
	*(tail->task) = (DD_Task) {
		NULL,
		DUMMY,
		0,
		0,
		0,
		0,
		0,
		0
	};
	tail->next_node = NULL;

	return head;
}

DD_Task_List_Node* Get_Node_By_Id(uint32_t taskId, DD_Task_List_Node* list_head) {
	DD_Task_List_Node* curr = list_head->next_node;

	// First we check if the list is empty:
	if (curr == NULL) {
		printf("LIST ERROR - GETBYID ON EMPTY LIST\n");
		return NULL;
	}
	// We traverse the list while we haven't ran out of space and have yet to find the node with our desired ID
	// This is potentially risky if C does not behave as expected (checking 1st condition then 2nd linearly)
	while ((curr->next_node != NULL) && (curr->task->task_id != taskId)) {
		curr = curr->next_node;
	}
	// If we're at the end of the line (i.e. at dummy tail), we could not find the task by its id.
	if (curr->next_node == NULL) {
		printf("LIST ERROR - COULD NOT FIND NODE BY ID\n");
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
		printf("LIST ERROR - Pop from Empty List\n");
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
	DD_Task_List_Node* prev = list_head;

	// First we check if the list is empty:
	if (curr->next_node == NULL) {
		new_node->next_node = list_head->next_node; // Point new node to dummy tail
		list_head->next_node = new_node; // Point dummy head to new node
		return;
	}

	// We traverse the list while we haven't ran out of space and have yet to find a node with a larger deadline than ours
	while ((curr->next_node != NULL) && (new_node->task->absolute_deadline_ticks > curr->task->absolute_deadline_ticks)) {
		prev = curr;
		curr = curr->next_node;
	}

	// If we're at the end of the line (i.e. we've arrived at the dummy tail), we are the largest deadline and add to end of list.
	if (curr->next_node == NULL) {
		prev->next_node = new_node; // Make the node before the dummy tail point to the new node
		new_node->next_node = curr; // Make the new node point to the dummy tail
		return;
	}

	// If we have a tie, we resolve by...
	if (curr->task->absolute_deadline_ticks == new_node->task->absolute_deadline_ticks) {
		// First by release time
		if (curr->task->release_time_ticks < new_node->task->release_time_ticks) {
			new_node->next_node = curr->next_node;
			curr->next_node = new_node;
			return;
		}
		else if (curr->task->release_time_ticks == new_node->task->release_time_ticks) {
			if (curr->task->parent_id < new_node->task->parent_id) {
				new_node->next_node = curr->next_node;
				curr->next_node = new_node;
				return;
			}
		}
	}

	// Otherwise, we have found the correct position to insert into.
	prev->next_node = new_node;
	new_node->next_node = curr;

	if (prev->task->type == DUMMY) {
		List_Priority_Update(list_head);
	}
}

void List_Priority_Update(DD_Task_List_Node* list_head) {
	DD_Task_List_Node* curr = list_head->next_node;

	// If we start at the dummy tail...
	if (curr->next_node == NULL) {
		return;
	}

	vTaskPrioritySet(curr->task->t_handle, 1);
	curr->task->priority = 1;
	
	while (curr->next_node != NULL) {
		vTaskPrioritySet(curr->next_node->task->t_handle, 0);
		curr->task->priority = 0;
		curr = curr->next_node;
	}
}

uint8_t Remove_Overdue(uint32_t overdue_timer_id, DD_Task_List_Node* active_head, DD_Task_List_Node* overdue_head) {
	// while the given nodes next is not null, and until the id matches continue to traverse
	DD_Task_List_Node* curr = active_head->next_node;
	DD_Task_List_Node* prev = active_head;

	// First we check if the list is empty (i.e. dummy head points to dummy tail):
	if (curr->next_node == NULL) {
		return 1;
	}
	// We traverse the list until either we find what we're looking for, or we run out of list.
	while ((curr->next_node != NULL) && (overdue_timer_id != curr->task->task_id)) {
		prev = curr;
		curr = curr->next_node;
	}
	// If we're at the end of the line...
	if (curr->next_node == NULL) {
		return 1;
	}

	// Otherwise we must've found it so we set prev to curr's next, and curr's next to NULL.
	prev->next_node = curr->next_node; // Point the previous to current's next node, effectively removing current from the list.
	curr->next_node = NULL; // Isolate current from the list.

	// Remove the corresponding f_task so that the overdue task does not get allocated any time/resources.
	vTaskDelete(curr->task->t_handle);
	curr->task->t_handle = NULL;

	if (prev->task->type == DUMMY) {
		List_Priority_Update(active_head);
	}

	// Add the overdue task to the front of the overdue list.
	List_Push(overdue_head, curr);

	return 0;
}

uint8_t Remove_Completed(uint32_t completed_dd_task_id, DD_Task_List_Node* active_head, DD_Task_List_Node* completed_head) {
	// while the given nodes next is not null, and until the id matches continue to traverse
	DD_Task_List_Node* curr = active_head->next_node;
	DD_Task_List_Node* prev = active_head;

	// First we check if the list is empty (i.e. dummy head points to dummy tail):
	if (curr->next_node == NULL) {
		return 1;
	}
	// We traverse the list until either we find what we're looking for, or we run out of list (i.e. we're on the dummy tail).
	while ((curr->next_node != NULL) && (completed_dd_task_id != curr->task->task_id)) {
		prev = curr;
		curr = curr->next_node;
	}
	// If we're at the end of the line (where curr = dummy tail)...
	if (curr->next_node == NULL) {
		return 1;
	}

	// Otherwise we must've found it so we set prev to curr's next, and curr's next to NULL.
	prev->next_node = curr->next_node; // Point the previous to current's next node, effectively removing current from the list.
	curr->next_node = NULL; // Isolate current from the list.

	// Remove the corresponding f_task, mostly for posterity.
	vTaskDelete(curr->task->t_handle);
	curr->task->t_handle = NULL;

	if (prev->task->type == DUMMY) {
		List_Priority_Update(active_head);
	}

	// Add the overdue task to the front of the overdue list.
	List_Push(completed_head, curr);

	return 0;
}

uint8_t Get_List_Length(DD_Task_List_Node* list_head) {

	uint8_t length = 0;
	DD_Task_List_Node* curr = list_head->next_node;

	// We traverse the list until we run out of list (i.e. we're on the dummy tail).
	while (curr->next_node != NULL) {
		length++;
		curr = curr->next_node;
	}

	return length;
}

void DEBUG_Create_And_Smart_Insert_Wrapper(DD_Task_List_Node* list_head, int id, int parent_id, int exec_time, int release_time, int deadline)
{
	DD_Task* new_dd_task = pvPortMalloc(sizeof(DD_Task));
	*new_dd_task = (DD_Task) {
		0,
		PERIODIC,
		id,
		parent_id,
		exec_time,
		release_time,
		deadline,
		0
	};

	DD_Task_List_Node* new_node = pvPortMalloc(sizeof(DD_Task_List_Node));
	*new_node = (DD_Task_List_Node) {
		new_dd_task,
		NULL,
	};

	Smart_Active_Insert(list_head, new_node);
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

	printf("\n======\n");
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

	printf("\nMALLOC FAILED HOOK\n");

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
