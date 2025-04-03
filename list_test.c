#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

// ---------------------------------

typedef enum dd_task_type {
	PERIODIC,
	APERIODIC
} DD_Task_Type;

typedef struct dd_task_generator_parameters {
	int generator_id;
	DD_Task_Type task_type;
	int execution_time;
	int period;
} DD_Task_Generator_Parameters;

typedef struct dd_task {
	// TaskHandle_t t_handle;
	int t_handle;
	DD_Task_Type type;
	int task_id;
	int parent_id;
	int time_to_execute;
	int release_time_ticks;
	int absolute_deadline_ticks;
	int completion_time_ticks;
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
	// xTimerHandle* overdue_timer;
	int overdue_timer;
} Scheduler_Event;

// ---------------------------------

void DEBUG_Create_And_Smart_Insert_Wrapper(DD_Task_List_Node* list_head, int id, int parent_id, int exec_time, int release_time, int deadline);
void DEBUG_Print_List(DD_Task_List_Node* list_head);
DD_Task_List_Node* List_Init( void );
void Smart_Active_Insert(DD_Task_List_Node* list_head, DD_Task_List_Node* new_node);
DD_Task_List_Node* List_Pop(DD_Task_List_Node* list_head);
void List_Push(DD_Task_List_Node* list_head, DD_Task_List_Node* new_node);
void List_Priority_Update(DD_Task_List_Node* list_head);
void List_Sort(DD_Task_List_Node* list_head);
void Remove_Overdue(int taskId, DD_Task_List_Node* active_head, DD_Task_List_Node* overdue_head);
void Remove_Completed(int completed_dd_task_id, DD_Task_List_Node* active_head, DD_Task_List_Node* completed_head);
int Get_List_Length(DD_Task_List_Node* list_head);
DD_Task_List_Node* getDDTaskById(int taskId, DD_Task_List_Node* list_head);

// ---------------------------------

// #define DD_TASK_0_PERIOD 500
// #define DD_TASK_1_PERIOD 500
// #define DD_TASK_2_PERIOD 750

int main() {
	// printf("Hello World.\n");
	DD_Task_List_Node* active_head = List_Init();
	DD_Task_List_Node* overdue_head = List_Init();
	DD_Task_List_Node* completed_head = List_Init();

	// printf("Starting insertions.\n");
	DEBUG_Create_And_Smart_Insert_Wrapper(active_head, 0, 0,
		95, 0, 500 // exec time, release time, and deadline
	);

	DEBUG_Create_And_Smart_Insert_Wrapper(active_head, 100, 1,
		150, 0, 500 // exec time, release time, and deadline
	);

	// printf("Two inserted.\n");
	DEBUG_Create_And_Smart_Insert_Wrapper(active_head, 200, 2,
		250, 0, 750 // exec time, release time, and deadline
	);

	// printf("Three inserted.\n");
	DEBUG_Create_And_Smart_Insert_Wrapper(active_head, 1, 0,
		95, 500, 1000 // exec time, release time, and deadline
	);

	// printf("Four inserted.\n");
	DEBUG_Create_And_Smart_Insert_Wrapper(active_head, 101, 1,
		150, 500, 1000 // exec time, release time, and deadline
	);

	DEBUG_Create_And_Smart_Insert_Wrapper(active_head, 2, 0,
		95, 1000, 1500 // exec time, release time, and deadline
	);

	// printf("Six inserted.\n");
	DEBUG_Create_And_Smart_Insert_Wrapper(active_head, 102, 1,
		150, 1000, 1500 // exec time, release time, and deadline
	);

	DEBUG_Create_And_Smart_Insert_Wrapper(active_head, 201, 2,
		250, 750, 1500 // exec time, release time, and deadline
	);

	// printf("Eight inserted.\n");
	// printf("Printing list:");
	DEBUG_Print_List(active_head);
}

// ---------------------------------


void DEBUG_Create_And_Smart_Insert_Wrapper(DD_Task_List_Node* list_head, int id, int parent_id, int exec_time, int release_time, int deadline)
{
	DD_Task* new_dd_task = malloc(sizeof(DD_Task));
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

// 	DD_Task* task; // Since the generator allocates space for the DD_Task, to ensure we don't drop / make any copies, we pass around pointers to that first creation of the DD_Task.
// 	struct dd_task_list_node* next_node;
// } DD_Task_List_Node;

	DD_Task_List_Node* new_node = malloc(sizeof(DD_Task_List_Node));
	*new_node = (DD_Task_List_Node) {
		new_dd_task,
		NULL,
	};

	Smart_Active_Insert(list_head, new_node);
}

// Update these to work with nodes that contain pointers to DD_Tasks
DD_Task_List_Node* List_Init( void ) {
	DD_Task_List_Node* head = malloc(sizeof(DD_Task_List_Node));
	DD_Task_List_Node* tail = malloc(sizeof(DD_Task_List_Node));
	//	DD_Task empty_task = (DD_Task) {};

	head->task = NULL;
	head->next_node = tail;
	tail->task = NULL;
	tail->next_node = NULL;
	return head;
}

DD_Task_List_Node* getDDTaskById(int taskId, DD_Task_List_Node* list_head) {
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
	// printf("\n-- Inserting %d\n", new_node->task->task_id);
	DD_Task_List_Node* curr = list_head->next_node;
	DD_Task_List_Node* prev = list_head;

	// First we check if the list is empty:
	if (curr->next_node == NULL) {
		new_node->next_node = list_head->next_node; // Point new node to dummy tail
		list_head->next_node = new_node; // Point dummy head to new node
		return;
	}
	// We traverse the list while we haven't ran out of space and have yet to find a node with a larger deadline than ours
	while ((curr->next_node != NULL) && (new_node->task->absolute_deadline_ticks <= curr->task->absolute_deadline_ticks)) {
		// printf("-- Insert Traversal; Node %d\n", curr->task->task_id);
		prev = curr;
		curr = curr->next_node;
	}
	// If we're at the end of the line (i.e. we've arrived at the dummy tail), we are the largest deadline and add to end of list.
	if (curr->next_node == NULL) {
		// printf("-- Insert at end of line.\n");
		prev->next_node = new_node; // Make the node before the dummy tail point to the new node
		new_node->next_node = curr; // Make the new node point to the dummy tail
		return;
	}

	// Otherwise, we have found the correct position to insert into.
	
	// printf("-- Insert inserting.\n");
	prev->next_node = new_node;
	new_node->next_node = curr;

	// IDEALLY because we are inserting into a position which invokes sorted priority, we do not have to further sort the list.
	// We expect all deadlines to approach at equal speed, so there is no need to sort the list.

	// If this doesn't actually work, we just implement a sort function and call it here.
	// sortList();

	// printf("-- Insert finished.\n\n");
}

void List_Priority_Update(DD_Task_List_Node* list_head) {
	DD_Task_List_Node* curr = list_head->next_node;

	// If we start at the dummy tail...
	if (curr->next_node == NULL) {
		printf("LIST ERROR - REMOVE OVERDUE ON EMPTY LIST\n");
		return;
	}

	// vTaskPrioritySet(curr->task->t_handle, 1);
	curr->task->t_handle = 1;

	while (curr->next_node != NULL) {
		// vTaskPrioritySet(curr->next_node->task->t_handle, 0);
		curr->next_node->task->t_handle = 0;
		curr = curr->next_node;
	}
}

void List_Sort(DD_Task_List_Node* list_head) {
	// Potentially, hopefully, we don't have to do this.
}

void Remove_Overdue(int overdue_timer_id, DD_Task_List_Node* active_head, DD_Task_List_Node* overdue_head) {
	// while the given nodes next is not null, and until the id matches continue to traverse
	DD_Task_List_Node* curr = active_head->next_node;
	DD_Task_List_Node* prev = active_head;

	// First we check if the list is empty (i.e. dummy head points to dummy tail):
	if (curr->next_node == NULL) {
		printf("LIST ERROR - REMOVE OVERDUE ON EMPTY LIST\n");
		return;
	}
	// We traverse the list until either we find what we're looking for, or we run out of list.
	while ((curr->next_node != NULL) && (overdue_timer_id != curr->task->task_id)) {
		prev = curr;
		curr = curr->next_node;
	}
	// If we're at the end of the line...
	if (curr->next_node == NULL) {
		printf("LIST ERROR - COULD NOT FIND OVERDUE DD TASK\n");
		return;
	}

	// Otherwise we must've found it so we set prev to curr's next, and curr's next to NULL.
	prev->next_node = curr->next_node; // Point the previous to current's next node, effectively removing current from the list.
	curr->next_node = NULL; // Isolate current from the list.

	// Remove the corresponding f_task so that the overdue task does not get allocated any time/resources.
	printf("Deleting ftask.\n");
	// vTaskDelete(curr->task->t_handle)
	// curr->task->t_handle = NULL;
	curr->task->t_handle = 0;

	// Add the overdue task to the front of the overdue list.
	List_Push(overdue_head, curr);

	// Should we also set the priority of this task to 0 (allegedly idle)? No let's do that in scheduler.
	// We actually delete the task in the scheduler before this function even gets called, so no need to change the priority of a dead task.

	// This might be dangerous
//	vPortFree(curr);
}

void Remove_Completed(int completed_dd_task_id, DD_Task_List_Node* active_head, DD_Task_List_Node* completed_head) {
	// while the given nodes next is not null, and until the id matches continue to traverse
	DD_Task_List_Node* curr = active_head->next_node;
	DD_Task_List_Node* prev = active_head;

	// First we check if the list is empty (i.e. dummy head points to dummy tail):
	if (curr->next_node == NULL) {
		printf("LIST ERROR - REMOVE COMPLETED ON EMPTY LIST\n");
		return;
	}
	// We traverse the list until either we find what we're looking for, or we run out of list (i.e. we're on the dummy tail).
	while ((curr->next_node != NULL) && (completed_dd_task_id != curr->task->task_id)) {
		prev = curr;
		curr = curr->next_node;
	}
	// If we're at the end of the line (where curr = dummy tail)...
	if (curr->next_node == NULL) {
		printf("LIST ERROR - COULD NOT FIND COMPLETED DD TAS\n");
		return;
	}

	// Otherwise we must've found it so we set prev to curr's next, and curr's next to NULL.
	prev->next_node = curr->next_node; // Point the previous to current's next node, effectively removing current from the list.
	curr->next_node = NULL; // Isolate current from the list.

	// Remove the corresponding f_task, mostly for posterity.
	// vTaskDelete(curr->task->t_handle);
	// curr->task->t_handle = NULL;
	curr->task->t_handle = 0;

	// Add the overdue task to the front of the overdue list.
	List_Push(completed_head, curr);
}

int Get_List_Length(DD_Task_List_Node* list_head) {

	int length = 0;
	DD_Task_List_Node* curr = list_head->next_node;

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

