
/**
 ******************************************************************************
 * @file    main.c
 * @author  Ac6
 * @version V1.0
 * @date    01-December-2013
 * @brief   Default main function.
 ******************************************************************************
 */

#include "stdio.h"
#include "stdint.h"

#include "stm32f4xx.h"
#include "defines.h"

/* Board includes */
#include "stm32f4xx_gpio.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_adc.h"
#include "stm32f4xx_usart.h"

/* Custom Libs Include */
#include "tm_stm32f4_gpio.h"
#include "tm_stm32f4_disco.h"
#include "tm_stm32f4_i2c.h"
#include "tm_stm32f4_adc.h"
#include "tm_stm32f4_usart.h"

/* Kernel includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"

/*REGISTER ACC DEFINES*/
#define READ_ACC_1 	0x3B
#define WRITE_ACC_1 0x3A
#define READ_ACC_2 	0xA7
#define WRITE_ACC_2 0xA6

#define DEVICE_ID 	0x00
#define BW_RATE	 	0x2C
#define POWER_CTL	0x2D
#define DATA_FORMAT 0x31
#define DATAX0		0x32
#define DATAX1		0x33
#define DATAY0      0x34
#define DATAY1 		0x35
#define DATAZ0		0x36
#define DATAZ1		0x37
#define FIFO_CTL    0x38


/*REGISTER MUX DEFINES*/
#define MUX_1_ADDR 	0x4C
#define MUX_2_ADDR  0x4D

/*GLOBALS*/
int mux_LUTs[] = {0x00,0x01,0x02,0x04,0x08,0x10,0x20,0x40,0x80};
typedef struct DATA_PACKET {
	int16_t DATA_X_SX;
	int16_t DATA_Y_SX;
	int16_t DATA_Z_SX;
	int16_t DATA_X_DX;
	int16_t DATA_Y_DX;
	int16_t DATA_Z_DX;
	int16_t PIEZO_DX[8];
	int16_t PIEZO_SX[8];
} DATA_PACKET;

/*VARIABLES DEFINITIONS*/
#define MAX_LEN 30
#define MAX_BUF_count 2

typedef struct
{	int16_t * const buffer;
int head;
int tail;
int maxLen;
}circBuf_t;

#define CIRCBUF_DEF(x,y) int16_t x##_space[y]; circBuf_t x = { x##_space,0,0,y}

CIRCBUF_DEF(bufPiz1,MAX_LEN);
CIRCBUF_DEF(bufPiz2,MAX_LEN);

CIRCBUF_DEF(bufAcc1,MAX_LEN);
CIRCBUF_DEF(bufAcc2,MAX_LEN);


//Semaphores
xSemaphoreHandle Comm_Sem = 0;
xSemaphoreHandle Piezo_Sem = 0;
xSemaphoreHandle Acc_Sem = 0;

//Software Timer
TimerHandle_t Timer1KHz;
TimerHandle_t TimerPiz;
TimerHandle_t TimerACC;

// queue
QueueHandle_t xQueueACC;
QueueHandle_t xQueuePiz;


//Software Timer callback function
void Comm_Unblock(TimerHandle_t xTimer)
{
	xSemaphoreGive (Comm_Sem);
	//xSemaphoreGiveFromISR(Comm_Sem);
	TM_USART_Puts(USART6,"Timer triggered.\n");
}

void ACC_Unblock(TimerHandle_t xTimer)
{
	xSemaphoreGive (Acc_Sem);
	//xSemaphoreGiveFromISR(Comm_Sem);
	TM_USART_Puts(USART6,"ACC Task triggered.\n");
}

void Piz_Unblock(TimerHandle_t xTimer)
{
	xSemaphoreGive (Piezo_Sem);
	//xSemaphoreGiveFromISR(Comm_Sem);
	TM_USART_Puts(USART6,"ACC Task triggered.\n");
}

void Send(circBuf_t* buf)
{
	// send data
}

//Tasks
void Communication(void * pvParameters)
{
	circBuf_t* Abuf = 0;
	circBuf_t* Pbuf = 0;

	while(1)
	{
		if(xSemaphoreTake(Comm_Sem, portMAX_DELAY))
		{
			TM_USART_Puts(USART6,"Communication running.\n");
			TM_DISCO_LedToggle(LED_GREEN);

			//CODE
			if( xQueuePiz != 0 )
			{
				// Receive a message on the created queue.  Block for 10 ticks if a
				// message is not immediately available.
				if( xQueueReceive( xQueuePiz, ( Pbuf ), ( TickType_t ) 01 ) )
				{
					// pcRxedMessage now points to the struct AMessage variable posted
					// by vATask.
					Send(Pbuf);
				}
			}

			//CODE
			if( xQueueACC != 0 )
			{
				// Receive a message on the created queue.  Block for 10 ticks if a
				// message is not immediately available.
				if( xQueueReceive( xQueueACC, ( Abuf ), ( TickType_t ) 00 ) )
				{
					// pcRxedMessage now points to the struct AMessage variable posted
					// by vATask.
					Send(Pbuf);
				}
			}

			TM_DISCO_LedToggle(LED_GREEN);
			TM_USART_Puts(USART6,"	-> Communication Done.\n");

			xSemaphoreGive (Comm_Sem);

		}
	}
}
void fillABuf(circBuf_t* buf)
{
	// fill operation


}

void fillPBuf(circBuf_t* buf)
{
	// fill operation


}


void Piezo(void * pvParameters)
{
	uint8_t buf_cnt=0;


	xQueuePiz = xQueueCreate( 10, sizeof( circBuf_t ) );

	while(1)
	{

		//TM_USART_Puts(USART6,"Piezo running.\n");

		TM_DISCO_LedToggle(LED_ORANGE);

		//CODE
		switch(buf_cnt)
		{
		case 0:
			fillPBuf(&bufPiz1);
			buf_cnt = 1;
			/* Send an unsigned long.  Wait for 10 ticks for space to become
				       available if necessary. */
			if( xQueueSend( xQueuePiz,
					( void * ) &bufPiz1,
					( TickType_t ) 0 ) != pdPASS )
			{
				/* Failed to post the message, even after 10 ticks. */
			}
			break;
		case 1:
			fillPBuf(&bufPiz2);
			buf_cnt = 0;
			if( xQueueSend( xQueuePiz,
					( void * ) &bufPiz2,
					( TickType_t ) 0 ) != pdPASS )
			{
				/* Failed to post the message, even after 10 ticks. */
			}
			break;
		}


		TM_USART_Puts(USART6,"	-> Piezo done.\n");
		TM_DISCO_LedToggle(LED_ORANGE);
		xSemaphoreGive (Piezo_Sem);

	}

}

void Accelerometer(void * pvParameters)
{
	uint8_t buf_cnt=0;


	xQueueACC = xQueueCreate( 10, sizeof( circBuf_t ) );

	while(1)
	{
		if(xSemaphoreTake(Acc_Sem, portMAX_DELAY))
		{
			TM_USART_Puts(USART6,"Accelerometer running.\n");
			TM_DISCO_LedToggle(LED_BLUE);

			//CODE
			switch(buf_cnt)
			{
			case 0:
				fillABuf(&bufAcc1);
				buf_cnt = 1;
				/* Send an unsigned long.  Wait for 10 ticks for space to become
					       available if necessary. */
				if( xQueueSend( xQueueACC,
						( void * ) &bufAcc1,
						( TickType_t ) 0 ) != pdPASS )
				{
					/* Failed to post the message, even after 10 ticks. */
				}
				break;
			case 1:
				fillABuf(&bufAcc2);
				buf_cnt = 0;
				if( xQueueSend( xQueueACC,
						( void * ) &bufAcc2,
						( TickType_t ) 0 ) != pdPASS )
				{
					/* Failed to post the message, even after 10 ticks. */
				}
				break;
			}

			TM_USART_Puts(USART6,"	-> Accelerometer done.\n");
			TM_DISCO_LedToggle(LED_BLUE);
			xSemaphoreGive (Acc_Sem);

		}
	}

}

int main(void)
{
	//Initialization
	TM_USART_Init(USART6,TM_USART_PinsPack_1,115200);
	TM_DISCO_LedInit();
	TM_DISCO_LedOn(LED_ALL);

	//Create Semaphores
	vSemaphoreCreateBinary(Comm_Sem);
	vSemaphoreCreateBinary(Piezo_Sem);
	vSemaphoreCreateBinary(Acc_Sem);

	// create Queue

	//Create Timers
	Timer1KHz = xTimerCreate("Timer1KHz", pdMS_TO_TICKS(1000), pdTRUE, 0, Comm_Unblock);
	//Create Timers
	TimerACC = xTimerCreate("Timer1KHz", pdMS_TO_TICKS(1000), pdTRUE, 0, ACC_Unblock);
	//Create Timers
	TimerPiz = xTimerCreate("Timer1KHz", pdMS_TO_TICKS(1000), pdTRUE, 0, Piz_Unblock);

	if(Timer1KHz == NULL)
	{
		TM_USART_Puts(USART6,"Timer was note created.\n");
	}else
	{
		if(xTimerStart(Timer1KHz,0) != pdPASS)
		{
			TM_USART_Puts(USART6,"Timer1KHz did not get into an active state.\n");
		}else
		{
			TM_USART_Puts(USART6,"Timer1KHz created successfully.\nTimer1KHz started successfully.\n");
		}
	}

	//Task Creation
	xTaskCreate(Communication, (signed char*)"Communication",configMINIMAL_STACK_SIZE,NULL,1,NULL);
	xTaskCreate(Piezo, (signed char*)"Piezo",configMINIMAL_STACK_SIZE,NULL,1,NULL);
	xTaskCreate(Accelerometer, (signed char*)"Accelerometer",configMINIMAL_STACK_SIZE,NULL,1,NULL);

	//Start the scheduler
	TM_USART_Puts(USART6,"Started.\n");
	TM_DISCO_LedToggle(LED_RED);
	xSemaphoreGive (Piezo_Sem);
	xSemaphoreGive (Acc_Sem);
	vTaskStartScheduler();
	while(1){

	}

}






///* QUEUE MANAGEMENT */
//xQueueHandle PIEZO_DX,PIEZO_SX;		//Set queue handler for the piezo sensors
//xQueueHandle ACC_DX,ACC_SX;			//Set queue handler for the acceleromenters
//
///*SEMAPHORES*/
//xSemaphoreHandle PIEZO_SENSE_SEM 	= 0;	//Handler to fire semaphore for Piezo sensing task
//xSemaphoreHandle ACC_SAMPLER_SEM 	= 0;	//Handler to fire semaphore for Accelerometers sampling task
//xSemaphoreHandle COMMUNICATION_SEM	= 0;	//Handler to fire semaphore for Communication task
//
///*TIMERS*/
//xTimerHandle COMMUNICATION_TIMER;	//Software timer to trigger communication
//
///*TASK PROTOTYPES*/
//void COMMUNICATION(void *pvParameters);
//void PIEZO_SENSE(void *pvParameters);
//void ACC_SAMPLER(void *pvParameters);
//
///*FUNCTIONS*/
//uint8_t Setup_Accelerometer(int ADDR, int R_ADDR, TM_I2C_PinsPack_t PIN_PACK);
//
///*CALLBACKs*/
//void vCallbackFunction( TimerHandle_t xTimer );
//
///*MAIN*/
//int main(void)
//{
//	uint8_t data[3];
//	SystemInit();
//
//	/*DEBUGGING*/
//	 TM_USART_Init(USART6,TM_USART_PinsPack_1,9600);
//
//	/* Create IPC variables */
//	 PIEZO_SX = xQueueCreate(16, sizeof(int));
//	 PIEZO_DX = xQueueCreate(16, sizeof(int));
//	 ACC_DX = xQueueCreate(3*20, sizeof(int));
//	 ACC_SX = xQueueCreate(3*20, sizeof(int));
//
//	 /*SEMAPHORES*/
//	 PIEZO_SENSE_SEM   = xSemaphoreCreateMutex();		/*Create Semaphores -> default value is 0*/
//	 ACC_SAMPLER_SEM   = xSemaphoreCreateMutex();		/*Create Semaphores -> default value is 0*/
//	 COMMUNICATION_SEM = xSemaphoreCreateMutex();
//
//	 /*TIMER*/
//	 COMMUNICATION_TIMER = xTimerCreate("SAMPLING_TIMER_COMM",pdMS_TO_TICKS(900),pdTRUE,0,vCallbackFunction);
//	 BaseType_t timer_flag = xTimerStart(COMMUNICATION_TIMER,0);
//	 if(timer_flag) TM_USART_Puts(USART6,"timer started\n");
//
//	 TM_DISCO_LedInit();
//	 TM_DISCO_LedOn(LED_ALL);
//
//	 data[0] = Setup_Accelerometer(WRITE_ACC_2,READ_ACC_2,TM_I2C_PinsPack_1);
//	 data[1] = Setup_Accelerometer(WRITE_ACC_1,READ_ACC_1,TM_I2C_PinsPack_2);
//
//	 /* Create tasks */
////	 xTaskCreate(
////			 PIEZO_SENSE,                 	/* Function pointer */
////			 "Task_PIEZO_SENSE",         	/* Task name - for debugging only*/
////			 configMINIMAL_STACK_SIZE,         /* Stack depth in words */
////			 (void*) NULL,                     /* Pointer to tasks arguments (parameter) */
////			 tskIDLE_PRIORITY + 2UL,           /* Task priority*/
////			 NULL                              /* Task handle */
////	 );
//	 xTaskCreate(
//			 COMMUNICATION,                 	/* Function pointer */
//			 "Task_COMMUNICATION",         	/* Task name - for debugging only*/
//			 configMINIMAL_STACK_SIZE,         /* Stack depth in words */
//			 (void*) NULL,                     /* Pointer to tasks arguments (parameter) */
//			 tskIDLE_PRIORITY + 4UL,           /* Task priority*/
//			 NULL                              /* Task handle */
//	 );
//	 xTaskCreate(
//			 ACC_SAMPLER,                 	/* Function pointer */
//			 "Task_ACC_SAMPLER",         	/* Task name - for debugging only*/
//			 configMINIMAL_STACK_SIZE,         /* Stack depth in words */
//			 (void*) NULL,                     /* Pointer to tasks arguments (parameter) */
//			 tskIDLE_PRIORITY + 2UL,           /* Task priority*/
//			 NULL                              /* Task handle */
//	 );
//
//	 TM_DISCO_LedOff(LED_ALL);
//	 vTaskStartScheduler();
//	 while(1);
//}
//
///*TASK DEFINITIONS*/
///*PIEZO_SENSE*/
//void PIEZO_SENSE(void *pvParameters){
//	while(1){
//		TM_DISCO_LedToggle(LED_RED);
//		vTaskDelay(10/portTICK_RATE_MS);
//	}
//}
///*COMMUNICATION*/
//void COMMUNICATION(void *pvParameters){
//	DATA_PACKET packet;
//	while(1){
//		if (xSemaphoreTake(COMMUNICATION_SEM,0)) {
//		TM_USART_Puts(USART6,"Taken COMMUNICATION_SEM\n");
//		TM_DISCO_LedOn(LED_BLUE);
//		portBASE_TYPE statusX = xQueueReceive(ACC_DX, &packet.DATA_X_DX, 0); /* Receive Message */
//		portBASE_TYPE statusY = xQueueReceive(ACC_DX, &packet.DATA_Y_DX, 0); /* Receive Message */
//		portBASE_TYPE statusZ = xQueueReceive(ACC_DX, &packet.DATA_Z_DX, 0); /* Receive Message */
//		if ((statusX == pdTRUE) & (statusY == pdTRUE) & (statusZ == pdTRUE)) {
//			TM_DISCO_LedOn(LED_ORANGE);
//			TM_USART_Puts(USART6,"OK reading from all queues\n");
//		}else{
//			TM_USART_Puts(USART6,"NAK reading from all queues\n");
//		}
//		TM_DISCO_LedOff(LED_ORANGE);
//		TM_USART_Puts(USART6,"COMM TASK EXECUTED - Leaving for ACC_SAMPLER\n");
//		xSemaphoreGive(ACC_SAMPLER_SEM);
//		TM_DISCO_LedOff(LED_BLUE);
//		}
//		//vTaskDelay(1000/portTICK_RATE_MS);
//
//	}
//}
///*ACC_SAMPLER*/
//void ACC_SAMPLER(void *pvParameters){
//	uint16_t X = 0;
//	uint16_t Y = 0;
//	uint16_t Z = 0;
//	uint8_t data[] = {0,1,2,3,4,5};
//	portBASE_TYPE return1;
//	portBASE_TYPE return2;
//	portBASE_TYPE return3;
//	char char_buffer[128];
//	while(1){
//				if (xSemaphoreTake(ACC_SAMPLER_SEM,1000)) {
//				TM_DISCO_LedOn(LED_GREEN);
//		           /* We were able to obtain the semaphore and can now access the
//		           shared resource. */
//					TM_I2C_ReadMulti(I2C1,READ_ACC_1,DATAX0,data,6);
//					X = (data[1]<<8) | data[0];
//					Y = (data[3]<<8) | data[2];
//					Z = (data[5]<<8) | data[4];
//		   		return1 = xQueueSendToBack(ACC_DX,&X,0);
//		   		return2 = xQueueSendToBack(ACC_DX,&Y,0);
//		   		return3 = xQueueSendToBack(ACC_DX,&Z,0);
//		   		if ((return1 == pdTRUE) & (return2 == pdTRUE) & (return3 == pdTRUE)) {
//		   			TM_USART_Puts(USART6,"TASK_ACC_SAMPLER writing to queues OK\n");
//		   		}
//		   		sprintf(char_buffer,"ACC DX READING -> X: %d Y: %d Z: %d \n",X,Y,Z);
//		   		TM_USART_Puts(USART6,char_buffer);
//		   		TM_I2C_ReadMulti(I2C1,READ_ACC_2,DATAX0,data,6);
//					X = (data[1]<<8) | data[0];
//					Y = (data[3]<<8) | data[2];
//					Z = (data[5]<<8) | data[4];
//		   		return1 = xQueueSendToBack(ACC_SX,&X,0);
//		   		return2 = xQueueSendToBack(ACC_SX,&Y,0);
//		   		return3 = xQueueSendToBack(ACC_SX,&Z,0);
//		   		if ((return1 == pdTRUE) & (return2 == pdTRUE) & (return3 == pdTRUE)) {
//		   			TM_USART_Puts(USART6,"TASK_ACC_SAMPLER writing to queues OK\n");
//		   		}
//		   		sprintf(char_buffer,"ACC SX READING -> X: %d Y: %d Z: %d \n",X,Y,Z);
//		       	TM_USART_Puts(USART6,char_buffer);
//		           /* ... */
//		   		TM_DISCO_LedOff(LED_GREEN);
//				}
//	}
//}
//
///*FUNCTIONS DEFINITIONS*/
//uint8_t Setup_Accelerometer(int W_ADDR, int R_ADDR, TM_I2C_PinsPack_t PIN_PACK){
//	TM_I2C_Init(I2C1,PIN_PACK,400000);
//	TM_I2C_Write(I2C1,W_ADDR,BW_RATE,0x08);
//	TM_I2C_Write(I2C1,W_ADDR,DATA_FORMAT,0x0D);
//	TM_I2C_Write(I2C1,W_ADDR,POWER_CTL,0x28);
//	TM_I2C_Write(I2C1,W_ADDR,FIFO_CTL,0x00);
//	return TM_I2C_Read(I2C1,R_ADDR,DEVICE_ID);
//}
//
//void vCallbackFunction( TimerHandle_t xTimer ){
//	TM_USART_Puts(USART6,"TIMER_CALLBACK\n");
//	//xSemaphoreGive(COMMUNICATION_SEM);
//}

