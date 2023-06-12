#include <Arduino.h>
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"

//STR->MUST DO: Comment line 94, 95, 96 and 97 in nrf.h by clicking on Arduino.h and then in line 10 clicking on nrf.h. By doing this, the microbit v2 pinout is correct

#define END_SCHEDULER 10000//Time to stop the kernel in milliseconds

#define T1 50//Task periods in milliseconds
#define T2 100
#define T3 200
#define T4 500

#define C1 5//computing times in milliseconds
#define C2 10
#define C3 15
#define C4 20

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
void Task1( void *pvParameters );
void Task2( void *pvParameters );
void Task3( void *pvParameters );
void Task4( void *pvParameters );
void OneShotTimerCallback( TimerHandle_t xTimer );

//Function prototypes
void str_trace(void);
void str_compute(unsigned long);
float str_getTime(void);

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
	Serial.println("Init...");

	xOneShotTimer = xTimerCreate("OneShotTimer",   pdMS_TO_TICKS(END_SCHEDULER) , pdFALSE, 0, OneShotTimerCallback );
	xOneShotStarted = xTimerStart( xOneShotTimer, 0 );

	xTaskCreate(Task1,"Task1", configMINIMAL_STACK_SIZE, NULL, 5, &Task1Handle);
	xTaskCreate(Task2,"Task2", configMINIMAL_STACK_SIZE, NULL, 4, &Task2Handle);
	xTaskCreate(Task3,"Task3", configMINIMAL_STACK_SIZE, NULL, 3, &Task3Handle);
	xTaskCreate(Task4,"Task4", configMINIMAL_STACK_SIZE, NULL, 2, &Task4Handle);

	// Now the task scheduler, which takes over control of scheduling individual tasks, is automatically started for this port.
	// Other ports require vTaskStartScheduler() at this point;  
	vTaskStartScheduler();  
}

void loop() 
{
	;
}

void Task1(void *pvParameters)  // This is a task.
{
	(void) pvParameters;

	TickType_t xLastWakeTime;
	xLastWakeTime = 0;//xTaskGetTickCount();

	for (;;) // A Task shall never return or exit.
	{  
      led11_state= led11_state^1;
      digitalWrite(LED11,led11_state);
      str_compute(C1);

      vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(T1) );
  	}
}


void Task2(void *pvParameters)  // This is a task.
{
	(void) pvParameters;

	TickType_t xLastWakeTime;
	xLastWakeTime = 0;//xTaskGetTickCount();

	for (;;) // A Task shall never return or exit.
	{  
      led12_state= led12_state^1;
      digitalWrite(LED12,led12_state);
      str_compute(C2);

      vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(T2));
  	}
}

void Task3(void *pvParameters)  // This is a task.
{
	(void) pvParameters;

	TickType_t xLastWakeTime;
	xLastWakeTime = 0;//xTaskGetTickCount();

	for (;;) // A Task shall never return or exit.
	{  
      led13_state= led13_state^1;
      digitalWrite(LED13,led13_state);
      str_compute(C3);

      vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(T3));
  	}
}

void Task4(void *pvParameters)  // This is a task.
{
	(void) pvParameters;

	TickType_t xLastWakeTime;
	xLastWakeTime = 0;//xTaskGetTickCount();

	for (;;) // A Task shall never return or exit.
	{  
      led14_state= led14_state^1;
      digitalWrite(LED14,led14_state);
      str_compute(C4);
	  
      vTaskDelayUntil( &xLastWakeTime, pdMS_TO_TICKS(T4));
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
	  circ_buffer1[circ_buffer_counter] = eTaskGetState(Task1Handle);
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
