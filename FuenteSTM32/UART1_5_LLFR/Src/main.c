/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  **
  ******************************************************************************
  * @file    main.c
  * @author  R.Oliva - RTOS (i) TP Final
  * @version V1.1
  * @brief   main del TPFinal RTOS i 2019
  ******************************************************************************
  */
  
  
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include <stdint.h>
#include "main.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#define TRUE 1
#define FALSE 0

#define PKTBUFSIZE  96
#define PACK_OK_LEN 35

#define DEBUG_USART6
#define DEBUG_VERBOSE

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct PACKET_OK
{ 
    uint8_t packet_content[PACK_OK_LEN+1];
} PACKET_OK_t;
// 
// v6 for use with METEO v20 uses packet "UUU$ttttt.bbbbb.dddd.sssss.vvv.CRCC*QQQ":  (WSpeed is 5 chars long) 
//      UUU$   start identifier
//      ttttt is 00000 08191 Raw Temperature ADC reading, can be 0-5V( Direct sensor with G=2) or 1-5V (4-20mA)
//      bbbbb is 00000 08191 Raw BaroPressure ADC reading, can be 0-5V( Direct sensor with G=1) or 1-5V (4-20mA)
//      dddd  is 0000 to 3600, WDIR*10 in UWORD
//      sssss is 00000 to 99999 from Anemometer / Thies.
//      vvv   was voltage, not used.
//      CRCC  is simple checksum
//      *QQQ  end identifier
//      Total length from $ is: 5+1+5+1+4+1+5+1+3+1+4= then '*' =15+7+4+5 =31
//      Total length form first U is 31+4 = 35
//      TestCStr[] = "UUU$29335.10156.2562.15100.125.1095*QQQ";
//  Parsed originally (after $ detected) with
// 	c = (char)sscanf(packet,"%5d.%5d.%4d.%5d.%3d.%4x",&out_t,
//  
//  Now Parsed with UUU$ at start with:
// 	c = (uint8_t)sscanf(packet,"%4s%5d.%5d.%4d.%5d.%3d.%4x",&out_id,&out_t,
//		&out_b, &out_w_dir,&out_w_speed,
//		&out_vbat,&checksum);
// char okTestCStr[] = "UUU$29335.10156.2562.15100.125.1095";	

//UART6 command structure Implem 21.4.19
typedef struct APP_CMD
{
	uint8_t COMMAND_NUM;
	uint8_t COMMAND_ARGS[10];
}APP_CMD_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Nuevo Menu 21.4.19
#define TOG_DEMO_U1_SINCOMM_COMMAND 	1
#define TOG_DEMO_U1_LOOPBK_COMMAND 		2
#define TOG_LEE_U1_METEO_COMMAND 		3
#define RTC_READ_DATE_TIME_COMMAND 	    4

// Flag defines Nuevo Menu 21.4.19
#define DEMO1_OFF   0
#define DEMO1_ON    1

char TestCStr[] = "UUU$29335.10156.2562.15100.125.1095*QQQ";
// y sin el tramo final:
char okTestCStr[] = "UUU$29335.10156.2562.15100.125.1095";	
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */




/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
//global space for some variable
char usr_msg[250]={0};

//task handles
TaskHandle_t xTaskHandleDisplay   = NULL;   // Task 1 Display Handle
TaskHandle_t xTaskHandleChkMeteo  = NULL;   // Task 2 CheckMeteo Handle
TaskHandle_t xTaskHandleProcOutd  = NULL;   // Task 3 ProcessOutdoorInfo Handle
TaskHandle_t xTaskHandleWrUart6   = NULL;   // Task 4 Write_Uart6 Handle
TaskHandle_t xTaskHandlePrintErr  = NULL;   // Task 5 PrintError
TaskHandle_t xTaskHandleHeartBeat = NULL;   // Task 6 OLED Heartbeat 21.4.19
TaskHandle_t xTaskHandleUart6CmdH = NULL;   // Task 7 Uart6 CmdHandling 21.4.19
TaskHandle_t xTaskHandleUart6CmdP = NULL;   // Task 8 Uart6 CmdProcess 21.4.19

//Queue handle
QueueHandle_t packet_queue       = NULL;   // ok packets queue handle
QueueHandle_t output_write_queue = NULL;   // output write queue handle
QueueHandle_t command_queue      = NULL;   // comandos via UART6 Terminal

//software timer handler
TimerHandle_t led_timer_handle = NULL;

// packet buffer
uint8_t packet_buffer[PKTBUFSIZE];
uint8_t packet_len = 0;
uint8_t command_buffer[20];
uint8_t command_len =0;
uint8_t flag_demo_U1_sinCom = DEMO1_OFF;

// Menu por USART 6
// Nuevo menu 21.4.2019
char menu[]={"\
\r\nToggle DEMO_UART1 (sinCom)----> 1 \
\r\nToggle DEMO_UART1 (loopbk)----> 2 \
\r\nToggle LEE UART1_METEO    ----> 3 \
\r\nToggle LEE RTC_FECHA+HORA ----> 4 \
\r\nEXIT_APP                  ----> 0 \
\r\nTipee su opcion : "};


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
//tasks prototypes - RTOS1
void vTask_Display(void *params);              // Task 1 = Display
void vTask_Check_Meteo_packet(void *params);   // Task 2 = Check Meteo Packet
void vTask_Process_OutdoorInfo(void *params);  // Task 3 = ProcessOutdoorInfo
void vTask_Write_Uart6(void *params);          // Task 4 = Write to UART6 -Terminal
void vTask_PrintError(void *params);           // Task 5 = Imprimeerror
void vTask_HeartBeat(void *params);            // Task 6 = OLED Heartbeat 21.4.19
void vTask_uart6_cmd_handling(void *params);   // Task 7 = Terminal UART6 Cmd_Handling
void vTask_uart6_cmd_processing(void *params); // Task 8 = Terminal UART6 proceso comandos



// Funciones auxiliares
// Enviar el Queue de salida a UART6
// void vTask_Write_Uart6(void *params);
// Prints message out on UART6..
void printmsg(char *msg);
// Enviar Items leidos via UART1
void print_items_message(char *task_msg, int16_t items);
// Salida 2 - viento en UART1
void print_Wind_Speed(char *task_msg, int16_t wspeed);
// Salida 3 - Error en Outdoor Info
void print_error_message(char *task_msg);
// Futuro - Terminal
uint8_t getCommandCode(uint8_t *buffer);
// Futuro: comandos UART6 - Terminal - Argumentos
void getArguments(uint8_t *buffer);
// Implementado 21.4.19 para comando 1
void toggle_demoU1_sinCom(char *task_msg);
// No implementado
void toggle_demoU1_Comloopbak(void);
// No implementado
void toggle_leeU1Meteo(void);
// Parcial
void read_rtc_info(char *task_msg);
// 22 4 2019  -Start Reception Function
// Rutinal LL requieren habilitación
// (STMCubeF4 v1_24 - LLNucleo 411 Examples)
void Start_Reception(void);


/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_USART6_UART_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  sprintf(usr_msg,"\r\n RTOS1 TP Final R.Oliva 2019 \r\n");
  printmsg(usr_msg); 
  
  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();

  // Start Reception - Habilita Flags USARTs 22.4.2019
  Start_Reception();

  // Create packet queue (up to 10 stripped packets)
  packet_queue = xQueueCreate(10,sizeof(PACKET_OK_t*));

  // Create the write queue (up to 30 chars)
  output_write_queue = xQueueCreate(30,sizeof(char*));

  // 21.4.2019 create command queue
  command_queue = xQueueCreate(10,sizeof(APP_CMD_t*));


	// tasks creation - RTOS1
	// void vTask_Display(void *params);              // Task 1 = Display
	// void vTask_Check_Meteo_packet(void *params);   // Task 2 = Check Meteo Packet
	// void vTask_Process_OutdoorInfo(void *params);  // Task 3 = ProcessOutdoorInfo
	// void vTask_Write_Uart6(void *params);          // Task 4 = Write to UART6 -Terminal
	// with task handles
	// TaskHandle_t xTaskHandleDisplay  = NULL;   // Task 1 Display Handle
	// TaskHandle_t xTaskHandleChkMeteo = NULL;   // Task 2 CheckMeteo Handle
	// TaskHandle_t xTaskHandleProcOutd = NULL;   // Task 3 ProcessOutdoorInfo Handle
	// TaskHandle_t xTaskHandleWrUart6  = NULL;   // Task 4 Write_Uart6 Handle

	
	if((packet_queue != NULL) && (output_write_queue != NULL) && (command_queue != NULL))
	{
		// Create task-1 vTask_Display
		// TaskHandle_t xTaskHandleDisplay -  Task 1 Display Handle
		xTaskCreate(vTask_Display,"TASK_DISPLAY-1",500,NULL,2,&xTaskHandleDisplay);

		// Create task-2 vTask_Check_Meteo_packet
		// TaskHandle_t xTaskHandleChkMeteo - Task 2 CheckMeteo Handle
		xTaskCreate(vTask_Check_Meteo_packet,"TASK_CHK_MET_2",500,NULL,2,&xTaskHandleChkMeteo);

		// Create task-3 vTask_Process_OutdoorInfo
		// TaskHandle_t xTaskHandleProcOutd - Task 3 ProcessOutdoorInfo Handle
		xTaskCreate(vTask_Process_OutdoorInfo,"TASK_PROCESS_OUTD_3",500,NULL,2,&xTaskHandleProcOutd);

		// Create task-4 vTask_Write_Uart6
		// TaskHandle_t xTaskHandleWrUart6  - Task 4 Write_Uart6 Handle
		xTaskCreate(vTask_Write_Uart6,"TASK4-UART-WRITE",500,NULL,2,&xTaskHandleWrUart6);

		// Create task-5 vTask_PrintError
		// TaskHandle_t xPrintError  - Task 5
		xTaskCreate(vTask_PrintError,"TASK5-PRINTERROR",500,NULL,2,&xTaskHandlePrintErr);

		// Create task-6 vTask_HeartBeat
		// TaskHandle_t HeartBeat  - Task 6
		xTaskCreate(vTask_HeartBeat,"TASK6-HrtBEAT",500,NULL,2,&xTaskHandleHeartBeat);

		//create task-7 U6 cmd handle
		xTaskCreate(vTask_uart6_cmd_handling,"TASK7-U6CMD-HANDLING",500,NULL,2,&xTaskHandleUart6CmdH);

		//create task-8 U6 cmd process
		xTaskCreate(vTask_uart6_cmd_processing,"TASK8-U6CMD-PROCESS",500,NULL,2,&xTaskHandleUart6CmdP);

	    // start the scheduler
	    vTaskStartScheduler();
	}else
	{
		sprintf(usr_msg,"Fallo la creacion de las colas!\r\n");
		printmsg(usr_msg);

   }
  
  /* Start scheduler */
  // osKernelStart();
  
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

// RTOSi - Implementacion de Task handlers
// TP Final R.Oliva 2019

// Task 1 - Display Menu por UART6
void vTask_Display(void *params)
{
char *pData = menu;
printmsg("\n\rTskDi1");
vTaskDelay(20);
	while(1)
	{
		xQueueSend(output_write_queue,&pData,portMAX_DELAY);

		//Esperar indefinidamente.
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);

	}
}

// Task2 vTask_Check_Meteo_packet - Verifica lo que llega
// de METEO por UART1, y lo pasa a la cola de paquetes ok
// En modo Demo, con uint8_t flag_demo_U1_sinCom = DEMO1_ON
// envía cada 1000 ms un paquete de muestra - sino en OFF
// espera notificación de un paquete via UART1
// 23.4.2019 Corregir acceso:new_packet-> packet_content
void vTask_Check_Meteo_packet(void *params)
{
// uint8_t command_code=0;
// typedef struct PACKET_OK
// {
//    uint8_t packet_content[PACK_OK_LEN+1];
// } PACKET_OK_t;
PACKET_OK_t *new_packet;
#ifdef DEBUG_VERBOSE
   printmsg("\n\rTskChM2");
#endif
vTaskDelay(20);
	while(1)
	{
		// Si no está en modo Demo, esperar indefinidamente a UART1
		// en modo "Demo1" generar un string de muestra cada 1000 ms
		if (flag_demo_U1_sinCom == DEMO1_OFF)
		{
		 xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
		}
		else
		{
		 vTaskDelay(pdMS_TO_TICKS(1000));
		}
		//1. allocate space..
		new_packet = (PACKET_OK_t*) pvPortMalloc(sizeof(PACKET_OK_t));

		taskENTER_CRITICAL();
		// From WG Book - Minimal printf facility
		// requires miniprintf.c / .h pair
		// make a copy of the buffer_packet to new_packet
		#ifdef PACKET_COPIAR
		mini_snprintf(new_packet->packet_content, PACK_OK_LEN,"%s",packet_buffer)
		#else
		// Copiamos uno de muestra
	    // char okTestCStr[] = "UUU$29335.10156.2562.15100.125.1095";	
		sprintf(new_packet->packet_content,"%s",okTestCStr);
		#endif
		taskEXIT_CRITICAL();

		//send the paquet to the packet queue
		xQueueSend(packet_queue,&new_packet,portMAX_DELAY);

	}

}

// Task3 vTask_Process_OutdoorInfo - el paquete
// de METEO por UART1 es procesado para sacar la info
// 23.4.2019 Idem Corregir acceso:new_packet-> packet_content
void vTask_Process_OutdoorInfo(void *params)
{
	PACKET_OK_t *new_packet;
	char task_msg[50];
	char out_id[6];
	uint16_t chk = 0;
    int16_t items = 0;
	int16_t out_t,out_b, out_w_speed,out_vbat;
    int16_t out_w_dir;
	int16_t checksum;  // temporary variables
	//uint32_t toggle_duration = pdMS_TO_TICKS(500);
    #ifdef DEBUG_VERBOSE
	 printmsg("\n\rTskPrOu3");
    #endif

	while(1)
	{
		xQueueReceive(packet_queue,(void*)&new_packet,portMAX_DELAY);

		items = sscanf(new_packet->packet_content,"%4s%5hd.%5hd.%4hd.%5hd.%3hd.%4hx",&out_id,&out_t,
		&out_b, &out_w_dir,&out_w_speed, &out_vbat,&checksum);
		
		print_items_message(task_msg, items);
		
		chk = out_t + out_b + out_w_dir;
	    chk += out_w_speed + out_vbat;

     
	    if((chk == checksum) && (items == 7))
		{
		print_Wind_Speed(task_msg, out_w_speed);
		}
		else
		{
			print_error_message(task_msg);
		}

		//lets free the allocated memory for the new packet
		vPortFree(new_packet);

	}
}

// Task4 vTask_PrintError solo en caso de recepcion erronea UART1
void vTask_PrintError(void *params)
{
	char pData[] = "Error en Recepcion";
	printmsg("\n\rTskPe5");
	vTaskDelay(20);
	while(1)
	{
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
		//1. allocate space..
		xQueueSend(output_write_queue,&pData,portMAX_DELAY);
	}

}



// Task5 vTask_Write_UART6 todo lo que llega a la cola de output va a la UART6 a 115200
void vTask_Write_Uart6(void *params)
{
	char *pData = NULL;
	printmsg("\n\rTskU4..");
	vTaskDelay(20);
	while(1)
	{

        xQueueReceive(output_write_queue,&pData,portMAX_DELAY);
		printmsg(pData);

	}
}

// Task 6 = OLED Heartbeat 21.4.19
void vTask_HeartBeat(void *params)
{

	while(1){
		vTaskDelay(pdMS_TO_TICKS(500));
		// OLED_GPIO_Port, OLED_Pin
		LL_GPIO_TogglePin(OLED_GPIO_Port,OLED_Pin);
	}
}

// void vTask_uart6_cmd_handling - decodifica el comando que le llega del usuario
// a traves de terminal UART6, según el menu, y lo manda a la cola de comandos
void vTask_uart6_cmd_handling(void *params)   // Task 7 = Terminal UART6 Cmd_Handling
{
	uint8_t command_code=0;
	APP_CMD_t *new_cmd;
	char dbg_msg[20];

	while(1)
	{
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
		//1. send command to queue
		#ifdef DEBUG_USART6
			sprintf(dbg_msg,"\r\nRxCmh");
			printmsg(dbg_msg);
		#endif
		new_cmd = (APP_CMD_t*) pvPortMalloc(sizeof(APP_CMD_t));

		taskENTER_CRITICAL();
		command_code = getCommandCode(command_buffer);
		new_cmd->COMMAND_NUM = command_code;
		getArguments(new_cmd->COMMAND_ARGS);
		taskEXIT_CRITICAL();
		#ifdef DEBUG_USART6
			sprintf(dbg_msg,"\r\n Cm%d",command_code);
			printmsg(dbg_msg);
		#endif
		// enviar al command queue 21.4.19
		xQueueSend(command_queue,&new_cmd,portMAX_DELAY);
	}
}


// void vTask6_cmd_processing - ejecuta los comandos de la cola
// de comandos alimentada por el task anterior
// Task 8 = Terminal UART6 proceso comandos
void vTask_uart6_cmd_processing(void *params)
{
	APP_CMD_t *new_cmd;
	char task_msg[50];

	// uint32_t toggle_duration = pdMS_TO_TICKS(500);

	while(1)
	{
		xQueueReceive(command_queue,(void*)&new_cmd,portMAX_DELAY);

		#ifdef DEBUG_USART6
			sprintf(task_msg,"\r\n C_N:%d", new_cmd->COMMAND_NUM);
			printmsg(task_msg);
		#endif

		if(new_cmd->COMMAND_NUM == TOG_DEMO_U1_SINCOMM_COMMAND)
		{
			toggle_demoU1_sinCom(task_msg);      // Opcion 1
		}
		else if(new_cmd->COMMAND_NUM == TOG_DEMO_U1_LOOPBK_COMMAND)
		{
			toggle_demoU1_Comloopbak();  // Opcion 2
		}
		else if(new_cmd->COMMAND_NUM == TOG_LEE_U1_METEO_COMMAND)
		{
			toggle_leeU1Meteo();
		}
		else if(new_cmd->COMMAND_NUM == RTC_READ_DATE_TIME_COMMAND )
		{
			read_rtc_info(task_msg);
		}else
		{
			print_error_message(task_msg);
		}

		// liberar memoria asignada a new_cmd
		vPortFree(new_cmd);

	}
}



// Prints message out on UART6..
// 18.4.2019 Use LL_USART functions as in STM32F4 LL_Examples
void printmsg(char *msg)
{
	for(uint32_t i=0; i < strlen(msg); i++)
	{
		while(!LL_USART_IsActiveFlag_TXE(USART6)){
			; // Wait forever
		}
		// while (USART_GetFlagStatus(USART6,USART_FLAG_TXE) != SET);
		LL_USART_TransmitData8(USART6,msg[i]);
	}

	while (!LL_USART_IsActiveFlag_TC(USART6)){
		;  // Wait again forever
	}

}

// Funciones de Salida al UART6 18.4.19

// Salida 1 - items recibidos de UART1
void print_items_message(char *task_msg, int16_t items)
{
	sprintf( task_msg,"\r\n Items leidos Outdoor %d\r\n", items);
	xQueueSend(output_write_queue,&task_msg,portMAX_DELAY);
}

// Salida 2 - viento en UART1
void print_Wind_Speed(char *task_msg, int16_t wspeed)
{
	sprintf( task_msg,"\r\n Velocidad de viento %d\r\n", wspeed);
	xQueueSend(output_write_queue,&task_msg,portMAX_DELAY);
}

// Salida 3 - Error en Outdoor Info
void print_error_message(char *task_msg)
{
	sprintf( task_msg,"\r\n Error en OutdoorInfo\r\n");
	xQueueSend(output_write_queue,&task_msg,portMAX_DELAY);
}

// Comandos UART6 - Terminal 21.4.19 Sacar el ASCII
uint8_t getCommandCode(uint8_t *buffer)
{

	return buffer[0]-48;
}

// Futuro: comandos UART6 - Terminal - Argumentos
void getArguments(uint8_t *buffer)
{


}

// Funcion ejecución Comando 1 - Pasar a modo Demo sin COM1, sin METEO
// Togglea el flag uint8_t flag_demo_U1_sinCom = DEMO1_OFF/ON
// que afecta a la tarea vTask_Check_Meteo_packet()
// En modo Demo, con uint8_t flag_demo_U1_sinCom = DEMO1_ON
// envía cada 1000 ms un "paqueteOK" de muestra - sino en OFF
// espera notificación de un paquete via UART1

void toggle_demoU1_sinCom(char *task_msg)
{
	uint8_t flag_copy = 0;
	// Hacer una copia del estado actual del Flag e invertirlo
	taskENTER_CRITICAL();
	flag_copy = flag_demo_U1_sinCom;
	if (flag_demo_U1_sinCom == DEMO1_OFF){
		flag_demo_U1_sinCom = DEMO1_ON;
	    }
		else {
		flag_demo_U1_sinCom = DEMO1_OFF;
		}
	taskEXIT_CRITICAL();
	// Si estaba en OFF, aviso que pasamos a ON, sinó al reves
	if (flag_copy == DEMO1_OFF){
		sprintf( task_msg,"\r\n Demo sin Com = ON\r\n");
		}
	else {
		sprintf( task_msg,"\r\n Demo sin Com = OFF\r\n");
	    }
	xQueueSend(output_write_queue,&task_msg,portMAX_DELAY);
}

// Funcion toggle_demoU1_Comloopbak() - demo con Loopback
// No implementada
void toggle_demoU1_Comloopbak(void)
{

}

// Funcion toggle_leeU1Meteo() - lectura METEO directa
// No implementada
void toggle_leeU1Meteo(void)
{

}

// Funcion read_rtc_info(task_msg) - RTC needs to be included 22.4.2019
void read_rtc_info(char *task_msg)
{
#ifdef LOW_LEVEL_RTC
	RTC_TimeTypeDef RTC_time;
	RTC_DateTypeDef RTC_date;
	//read time and date from RTC peripheral of the microcontroller
	RTC_GetTime(RTC_Format_BIN, &RTC_time);
	RTC_GetDate(RTC_Format_BIN, &RTC_date);

	sprintf(task_msg,"\r\nTime: %02d:%02d:%02d \r\n Date : %02d-%2d-%2d \r\n",RTC_time.RTC_Hours,RTC_time.RTC_Minutes,RTC_time.RTC_Seconds, \
									RTC_date.RTC_Date,RTC_date.RTC_Month,RTC_date.RTC_Year );
	xQueueSend(output_write_queue,&task_msg,portMAX_DELAY);
#endif
}

// Taken from
void Start_Reception(void){
	// Habilitar Flags limpieza y Recepcion
	sprintf(usr_msg,"\r\n Inicio Recepcion UART6 \r\n");
	printmsg(usr_msg);
	// Limpiar Overrun Flag UART6
	LL_USART_ClearFlag_ORE(USART6);
	// Habilitar Interrupcion pr RXNE (Rx Not Empty)
	LL_USART_EnableIT_RXNE(USART6);
	sprintf(usr_msg,"\r\n Inicio Recepcion UART1 \r\n");
	printmsg(usr_msg);
	// Limpiar Overrun Flag UART1
	LL_USART_ClearFlag_ORE(USART1);
	// Habilitar Interrupcion pr RXNE (Rx Not Empty)
	LL_USART_EnableIT_RXNE(USART1);
}


// Funciones de CallBack
// Callback de USART1 - Recepción (antes en stm32f4xx_it.c, ahora referenciado desde allí)
void USART1_Reception_Callback(void){
	uint16_t data_byte;
	//a data byte is received from the user
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	data_byte = LL_USART_ReceiveData8(USART1);

	packet_buffer[packet_len++] = (data_byte & 0xFF) ;

	if((data_byte == '*' ) && (packet_len == PACK_OK_LEN))
			{
				//then packet is ok..
				//reset the packet_len variable
				packet_len = 0;

				//notify the CheckMeteoHand task
				xTaskNotifyFromISR(xTaskHandleChkMeteo,0,eNoAction,&xHigherPriorityTaskWoken);
			}
			else if(packet_len > PACK_OK_LEN)
			{
				//then packet is corrupt..
				//reset the packet_len variable
				packet_len = 0;
				// notify printError Task
				xTaskNotifyFromISR(xTaskHandlePrintErr,0,eNoAction,&xHigherPriorityTaskWoken);
			}
	        else{
				// do nothing
			}
	// if the above freertos apis wake up any higher priority task, then yield the processor to the
	//higher priority task which is just woken up.

	if(xHigherPriorityTaskWoken)
	{
		taskYIELD();
	}
}

#define IMPLEMENT_USART6_ISR_IN_MAIN
#ifndef IMPLEMENT_USART6_ISR_IN_MAIN
// Callback de UART6 - Recepción como terminal (antes en stm32f4xx_it.c, ahora referenciado desde allí
// via main.h) -
// 22.4.19 Corregido para LL_USART_ReceiveData8, recibe tipo uint8_t, y agregamos DEBUG_USART6
// con un printout para verificar

void USART6_Reception_Callback(void){

	uint8_t data_byte;  // LL_ requirese 8 bit
	//a data byte is received from the user
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	char dbg_msg[10];

	data_byte = LL_USART_ReceiveData8(USART6);

	#ifdef DEBUG_USART6
      sprintf(dbg_msg,"\r\n %c",data_byte);
      printmsg(dbg_msg);
	#endif

	command_buffer[command_len++] = data_byte;

	if(data_byte == '\r')
	{
		//then user is finished entering the data

		//reset the command_len variable
		command_len = 0;

		// lets notify the Display Task
		// TaskHandle_t xTaskHandleDisplay -  Task 1 Display Handle
	    // xTaskCreate(vTask_Display,"TASK_DISPLAY-1",500,NULL,1,&xTaskHandleDisplay);
		// Avisar al command handling task
        #ifdef DEBUG_USART6B
          sprintf(dbg_msg,"\r\n NotCmh");
          printmsg(dbg_msg);
	    #endif
		xTaskNotifyFromISR(xTaskHandleUart6CmdH,0,eNoAction,&xHigherPriorityTaskWoken);
        // y despues al que imprime el menu..
		xTaskNotifyFromISR(xTaskHandleDisplay,0,eNoAction,&xHigherPriorityTaskWoken);

	}
	// if the above freertos apis wake up any higher priority task, then yield the processor to the
	//higher priority task which is just woken up.
	if(xHigherPriorityTaskWoken)
	{
		taskYIELD();
	}
}
#else
// Implementar localmente la ISR.. 22.4.19 14:00
void USART6_IRQHandler(void)
{
  /* USER CODE BEGIN USART6_IRQn 0 */
	uint8_t data_byte;  // LL_ requirese 8 bit
	//a data byte is received from the user
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	char dbg_msg[20];

	if( LL_USART_IsActiveFlag_RXNE(USART6) && LL_USART_IsEnabledIT_RXNE(USART6))
		{

		// Former USART6_Reception_Callback();
		data_byte = LL_USART_ReceiveData8(USART6);

		#ifdef DEBUG_USART6B
		      sprintf(dbg_msg,"\r\n %c",data_byte);
		      printmsg(dbg_msg);
		#endif

		command_buffer[command_len++] = data_byte;

		if(data_byte == '\r')
			{
				//then user is finished entering the data
				//reset the command_len variable
				command_len = 0;

		        #ifdef DEBUG_USART6B
		          sprintf(dbg_msg,"\r\n NotCmh");
		          printmsg(dbg_msg);
			    #endif
				// Avisar al command handling task
				xTaskNotifyFromISR(xTaskHandleUart6CmdH,0,eNoAction,&xHigherPriorityTaskWoken);
		        // y despues al que imprime el menu..
				xTaskNotifyFromISR(xTaskHandleDisplay,0,eNoAction,&xHigherPriorityTaskWoken);

			}
			#ifdef DEBUG_USART6B
				sprintf(dbg_msg,"\r\n NtCmHE");
				printmsg(dbg_msg);
			#endif
			// if the above freertos apis wake up any higher priority task, then yield the processor to the
			//higher priority task which is just woken up.
			if(xHigherPriorityTaskWoken)
			{
				taskYIELD();
			}


		}

}
#endif


/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{ 
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
