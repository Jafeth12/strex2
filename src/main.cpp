#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

//STR->MUST DO: Comment line 94, 95, 96 and 97 in nrf.h by clicking on Arduino.h and then in line 10 clicking on nrf.h. By doing this, the microbit v2 pinout is correct

/*
    - EDF_task struct
        - Deadline
        - Period
        - Task handler
        - Task priority

    - EDF_task queue

    - EDF scheduler task (highest priority)
*/

#define NUM_TASKS 4

typedef struct EDF_task
{
    TickType_t deadline;
    TickType_t relative_deadline;
    TaskHandle_t handle;
    unsigned long priority;
    void (*task)(void *pvParameters);
    char name[10];
} EDF_task;

EDF_task tasks[NUM_TASKS];

#define END_SCHEDULER 10000//Time to stop the kernel in milliseconds

#define EDF 10 //EDF scheduler period

//Task periods and deadline in milliseconds
#define T1_P 20 
#define T1_D 15

#define T2_P 50
#define T2_D 35

#define T3_P 100
#define T3_D 90

#define T4_P 150
#define T4_D 150

#define C1 5//computing times in milliseconds
#define C2 10
#define C3 25
#define C4 30

#define COL1 4
#define LED11 21 
#define LED12 22
#define LED13 23
#define LED14 24
#define LED15 25
#define LEDMICRO 28

bool led11_state=LOW;
bool led12_state=LOW;
bool led13_state=LOW;
bool led14_state=LOW;
bool led15_state=LOW;

//circular buffer for debugging
#define BUFF_SIZE 300//Can be increased as desired
float t[BUFF_SIZE] = {};
char circ_buffer1[BUFF_SIZE] = {};
char circ_buffer2[BUFF_SIZE] = {};
char circ_buffer3[BUFF_SIZE] = {};
char circ_buffer4[BUFF_SIZE] = {};
char circ_buffer5[BUFF_SIZE] = {};
char circ_buffer6[BUFF_SIZE] = {};
char circ_buffer7[BUFF_SIZE] = {};
char circ_buffer8[BUFF_SIZE] = {};
char circ_buffer9[BUFF_SIZE] = {};
float debug_data1[BUFF_SIZE] = {};
unsigned int circ_buffer_counter = 0;

//tasks handlers, required in last parameter of xTaskCreate when accessing task info
TaskHandle_t EDFHandle;

TaskHandle_t Task1Handle;
TaskHandle_t Task2Handle;
TaskHandle_t Task3Handle;
TaskHandle_t Task4Handle;

TickType_t xLastWakeTime1=0;
TickType_t xLastWakeTime2=0;
TickType_t xLastWakeTime3=0;
TickType_t xLastWakeTime4=0;

//timer handlers
TimerHandle_t xOneShotTimer;
BaseType_t xOneShotStarted;
TimerHandle_t xAperiodicJob1Timer;
BaseType_t xAperiodicJob1Started;

// define tasks and timers prototypes
void edf( void *pvParameters );
void Task1( void *pvParameters );
void Task2( void *pvParameters );
void Task3( void *pvParameters );
void Task4( void *pvParameters );
void OneShotTimerCallback( TimerHandle_t xTimer );

//Function prototypes
void str_trace(void);
void str_compute(unsigned long);
float str_getTime(void);

unsigned long calculateTaskPriority(TickType_t deadline, TickType_t maxDeadline) {
    // Calculate the priority as a proportion of the maximum deadline
    unsigned long priority = (unsigned long)((deadline * (configMAX_PRIORITIES - 1)) / maxDeadline);
    if (priority == configMAX_PRIORITIES-1) {
        priority--;
    }
    
    // Adjust the priority to fit within the valid range
    priority = configMAX_PRIORITIES - priority - 1;
    
    return priority;
}

TickType_t calculateMaxDeadline() {
    TickType_t maxDeadline = 0;
    for (int i = 0; i < NUM_TASKS; i++) {
        if (tasks[i].deadline > maxDeadline) {
            maxDeadline = tasks[i].deadline;
        }
    }

    return maxDeadline;
}

void setTasksPriorities() {
    // Calculate the maximum deadline
    TickType_t maxDeadline = calculateMaxDeadline();

    // Calculate the priority for each task
    for (int i = 0; i < NUM_TASKS; i++) {
        tasks[i].priority = calculateTaskPriority(tasks[i].deadline, maxDeadline);
    }
}

int compareTasks(const void *a, const void *b) {
    EDF_task *taskA = (EDF_task *)a;
    EDF_task *taskB = (EDF_task *)b;

    if (taskA->relative_deadline < taskB->relative_deadline) {
        return -1;
    } else if (taskA->relative_deadline > taskB->relative_deadline) {
        return 1;
    } else {
        return 0;
    }
}

void sortTasks() {
    qsort(tasks, NUM_TASKS, sizeof(EDF_task), compareTasks);
}

void initTasks() {
    tasks[0] = (EDF_task){T1_D, 0, Task1Handle, 0, Task1, "Task1"};
    tasks[1] = (EDF_task){T2_D, 0, Task2Handle, 0, Task2, "Task2"};
    tasks[2] = (EDF_task){T3_D, 0, Task3Handle, 0, Task3, "Task3"};
    tasks[3] = (EDF_task){T4_D, 0, Task4Handle, 0, Task4, "Task4"};

    setTasksPriorities();
    // no hace falta sorting porque ya estan en orden inicialmente.

    for (int i = 0; i < NUM_TASKS; i++) {
        xTaskCreate(tasks[i].task, tasks[i].name, configMINIMAL_STACK_SIZE, NULL, tasks[i].priority, &tasks[i].handle);
    }
}

void setup() 
{
	// because the LEDs are multiplexed, we must ground the opposite side of the LED
	pinMode(COL1, OUTPUT);
	digitalWrite(COL1, LOW); 
	pinMode(LED11, OUTPUT);
	digitalWrite(LED11, LOW);
	pinMode(LED12, OUTPUT);
	digitalWrite(LED12, LOW);
	pinMode(LED13, OUTPUT);
	digitalWrite(LED13, LOW);
	pinMode(LED14, OUTPUT);
	digitalWrite(LED14, LOW); 
	pinMode(LED15, OUTPUT);
	digitalWrite(LED15, LOW);  

	// Serial.begin(115200);
	Serial.begin(9600);

    str_compute(1000); // delay to feo para hacer que le dé tiempo al monitor a abrirse.

	Serial.println("Init...");

	xOneShotTimer = xTimerCreate("OneShotTimer",   pdMS_TO_TICKS(END_SCHEDULER) , pdFALSE, 0, OneShotTimerCallback );
	xOneShotStarted = xTimerStart( xOneShotTimer, 0 );

	xTaskCreate(edf,"EDF", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-1, &EDFHandle);

    initTasks();

	// Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started for this port.
	// Other ports require vTaskStartScheduler() at this point;  
	vTaskStartScheduler();  
}

void loop() 
{
	;
}

void updateRelativeDeadline(TaskHandle_t handle, TickType_t lastWakeTime, TickType_t deadline) {
    TickType_t tick = xTaskGetTickCount();
    for (int i = 0; i < NUM_TASKS; i++) {
        if (tasks[i].handle == handle) {
            tasks[i].relative_deadline = (lastWakeTime + deadline) - tick;
            break;
        }
    }
}

void vApplicationTaskSwitchHook( void ) {
    // Called when a task switch is performed.  Functionality here is down to the user.
    // vPrintString( "Task switch!\n" );
}

void edf(void *pvParameters)  // This is a task.
{
	(void) pvParameters;

	TickType_t xLastWakeTime;
	xLastWakeTime = 0;//xTaskGetTickCount();
    
    for (;;) 
    {
        sortTasks();

        // update their priorities based on their position in the sorted array
        for (int i = 0; i < NUM_TASKS; i++) {
            tasks[i].priority = configMAX_PRIORITIES - i - 2; // -2 because the EDF task is also a task
            vTaskPrioritySet(tasks[i].handle, tasks[i].priority);
        }
        
      vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(EDF) );
    }
}

void Task1(void *pvParameters)  // This is a task.
{
	(void) pvParameters;

	TickType_t xLastWakeTime;
	xLastWakeTime = 0;//xTaskGetTickCount();

	for (;;) // A Task shall never return or exit.
	{  
        updateRelativeDeadline(xTaskGetCurrentTaskHandle(), xLastWakeTime, T1_D);
      led11_state= led11_state^1;
      digitalWrite(LED11,led11_state);
      str_compute(C1);

      vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(T1_P) );
  	}
}


void Task2(void *pvParameters)  // This is a task.
{
	(void) pvParameters;

	TickType_t xLastWakeTime;
	xLastWakeTime = 0;//xTaskGetTickCount();

	for (;;) // A Task shall never return or exit.
	{  
        updateRelativeDeadline(xTaskGetCurrentTaskHandle(), xLastWakeTime, T2_D);
      led12_state= led12_state^1;
      digitalWrite(LED12,led12_state);
      str_compute(C2);

      vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(T2_P));
  	}
}

void Task3(void *pvParameters)  // This is a task.
{
	(void) pvParameters;

	TickType_t xLastWakeTime;
	xLastWakeTime = 0;//xTaskGetTickCount();

	for (;;) // A Task shall never return or exit.
	{  
        updateRelativeDeadline(xTaskGetCurrentTaskHandle(), xLastWakeTime, T3_D);
      led13_state= led13_state^1;
      digitalWrite(LED13,led13_state);
      str_compute(C3);

      vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(T3_P));
  	}
}

void Task4(void *pvParameters)  // This is a task.
{
	(void) pvParameters;

	TickType_t xLastWakeTime;
	xLastWakeTime = 0;//xTaskGetTickCount();

	for (;;) // A Task shall never return or exit.
	{  
        updateRelativeDeadline(xTaskGetCurrentTaskHandle(), xLastWakeTime, T4_D);
      led14_state= led14_state^1;
      digitalWrite(LED14,led14_state);
      str_compute(C4);
	  
      vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(T4_P));
	}
}

void str_trace(void)
{
	  circ_buffer_counter++;
	  if (circ_buffer_counter >= BUFF_SIZE)
	  {
	    circ_buffer_counter = 0;
	  }

	  t[circ_buffer_counter] = str_getTime();//sent time in milliseconds
	  circ_buffer1[circ_buffer_counter] = eTaskGetState(EDFHandle);
	  circ_buffer2[circ_buffer_counter] = eTaskGetState(Task2Handle);
	  circ_buffer3[circ_buffer_counter] = eTaskGetState(Task3Handle);
	  circ_buffer4[circ_buffer_counter] = eTaskGetState(Task4Handle);
	  circ_buffer5[circ_buffer_counter] = 0;
	  circ_buffer6[circ_buffer_counter] = 0;
	  circ_buffer7[circ_buffer_counter] = 0;
	  circ_buffer8[circ_buffer_counter] = 0;
	  debug_data1[circ_buffer_counter] = 0;
}

//str_compute(x) is only used to waste time without using delays
void str_compute(unsigned long milliseconds)
{
	/*unsigned int i = 0;
	unsigned int imax = 0;
	imax = milliseconds * 3500;//3725;//Double check again
	volatile float dummy = 1;
	for (i = 0; i < imax; i++)
	{
		dummy = dummy * dummy;
	}*/
	// nrf_delay_ms(milliseconds);
	unsigned int i = 0;
	for (i = 0; i < milliseconds; i++)
	{
		nrf_delay_us(1000);
	}
}

float str_getTime(void)
{
	//float t=(float)((micros()-t_init)/((float)1000));//(float)(0.5e-3*((float)OCR1A*xTaskGetTickCount()+TCNT1));//Sent time in milliseconds!!!
	float t=xTaskGetTickCount()*10.0+(10.0-0.000015625*( *( ( volatile uint32_t * ) 0xe000e018 )));
	//TODO: Check against micros()
	return t;
}

void OneShotTimerCallback( TimerHandle_t xTimer )
{
	//stop the kernel...
	//vTaskSuspend(Task1Handle);
	//vTaskSuspend(Task2Handle);
	//vTaskSuspend(Task3Handle);
	//vTaskSuspend(Task4Handle);
	str_trace();
	vTaskSuspendAll();
	str_trace();		
	vPortEndScheduler(); /* Stop and clear the SysTick. */

	//...and sent data to the host PC
	unsigned int i;
	for (i = 0; i < BUFF_SIZE; i++)
	{
		Serial.println("DAT ");
		Serial.print((float)t[i]);
		Serial.print(",");
		Serial.write((uint8_t)circ_buffer1[i]);
		Serial.print(",");
		Serial.write((uint8_t)circ_buffer2[i]);
		Serial.print(",");
		Serial.write((uint8_t)circ_buffer3[i]);
		Serial.print(",");
		Serial.write((uint8_t)circ_buffer4[i]);
		Serial.print(",");
		Serial.write((uint8_t)circ_buffer5[i]);
		Serial.print(",");
		Serial.write((uint8_t)circ_buffer6[i]);
		Serial.print(",");
		Serial.write((uint8_t)circ_buffer7[i]);
		Serial.print(",");
		Serial.write((uint8_t)circ_buffer8[i]);
		Serial.print(",");    
		Serial.print((float)debug_data1[i]);
		Serial.println();
	}
}
