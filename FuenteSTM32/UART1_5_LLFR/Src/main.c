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

//UART6 command structure
typedef struct APP_CMD
{
	uint8_t COMMAND_NUM;
	uint8_t COMMAND_ARGS[10];
}APP_CMD_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
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
TaskHandle_t xTaskHandleDisplay  = NULL;   // Task 1 Display Handle
TaskHandle_t xTaskHandleChkMeteo = NULL;   // Task 2 CheckMeteo Handle
TaskHandle_t xTaskHandleProcOutd = NULL;   // Task 3 ProcessOutdoorInfo Handle
TaskHandle_t xTaskHandleWrUart6  = NULL;   // Task 4 Write_Uart6 Handle
TaskHandle_t xTaskHandlePrintErr  = NULL;   // Task 5 PrintError

//Queue handle
QueueHandle_t packet_queue       = NULL;   // ok packets queue handle
QueueHandle_t output_write_queue = NULL;   // output write queue handle

//software timer handler
TimerHandle_t led_timer_handle = NULL;

// packet buffer
uint8_t packet_buffer[PKTBUFSIZE];
uint8_t packet_len = 0;
uint8_t command_buffer[20];
uint8_t command_len =0;

//Menu por USART 6
char menu[]={"\
\r\nLectura USART METEO----> 1 \
\r\nEXIT_APP           ----> 0 \
\r\nType your option here : "};


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

// Funciones auxiliares
// Enviar el Queue de salida a UART6
void vTask_Write_Uart6(void *params);
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

  // Create packet queue (up to 10 stripped packets)
  packet_queue = xQueueCreate(10,sizeof(PACKET_OK_t*));

  // Create the write queue (up to 30 chars)
  output_write_queue = xQueueCreate(30,sizeof(char*));


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

	
	if((packet_queue != NULL) && (output_write_queue != NULL))
	{
		// Create task-1 vTask_Display
		// TaskHandle_t xTaskHandleDisplay -  Task 1 Display Handle
		xTaskCreate(vTask_Display,"TASK_DISPLAY-1",500,NULL,1,&xTaskHandleDisplay);

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

void vTask_Display(void *params)
{

char *pData = menu;

	while(1)
	{
		xQueueSend(output_write_queue,&pData,portMAX_DELAY);

		//Esperar indefinidamente.
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);

	}
}

void vTask_Check_Meteo_packet(void *params)
{
	uint8_t command_code=0;
	// typedef struct PACKET_OK
	// { 
	//    uint8_t packet_content[PACK_OK_LEN+1];
	// } PACKET_OK_t;
	PACKET_OK_t *new_packet;


	while(1)
	{
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
		//1. allocate space..
		new_packet = (PACKET_OK_t*) pvPortMalloc(sizeof(PACKET_OK_t));

		taskENTER_CRITICAL();
		// From WG Book - Minimal printf facility
		// requires miniprintf.c / .h pair
		// make a copy of the buffer_packet to new_packet
		#ifdef PACKET_COPIAR
		mini_snprintf(new_packet, PACK_OK_LEN,"%s",packet_buffer)
		#else
		// Copiamos uno de muestra
	    // char okTestCStr[] = "UUU$29335.10156.2562.15100.125.1095";	
		sprintf(new_packet,"%s",okTestCStr);
		#endif
		taskEXIT_CRITICAL();

		//send the paquet to the packet queue
		xQueueSend(packet_queue,&new_packet,portMAX_DELAY);

	}

}


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
	uint32_t toggle_duration = pdMS_TO_TICKS(500);

	while(1)
	{
		xQueueReceive(packet_queue,(void*)&new_packet,portMAX_DELAY);

		items = sscanf(new_packet,"%4s%5d.%5d.%4d.%5d.%3d.%4x",&out_id,&out_t,
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


void vTask_PrintError(void *params)
{
	char pData[] = "Error en Recepcion"
	while(1)
	{
		xTaskNotifyWait(0,0,NULL,portMAX_DELAY);
		//1. allocate space..
		xQueueSend(output_write_queue,&pData,portMAX_DELAY);
	}

}




void vTask_Write_Uart6(void *params)
{
	char *pData = NULL;
	while(1)
	{

        xQueueReceive(output_write_queue,&pData,portMAX_DELAY);
		printmsg(pData);

	}
}

// Prints message out on UART6..
void printmsg(char *msg)
{
	for(uint32_t i=0; i < strlen(msg); i++)
	{
		while(USART_G )
		// while (USART_GetFlagStatus(USART6,USART_FLAG_TXE) != SET);
		USART_SendData(USART6,msg[i]);
	}

	while ( USART_GetFlagStatus(USART6,USART_FLAG_TC) != SET);

}

// Funciones de Salida al UART6 

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

// Futuro: comandos UART6 - Terminal
uint8_t getCommandCode(uint8_t *buffer)
{

	return buffer[0]-48;
}

// Futuro: comandos UART6 - Terminal - Argumentos
void getArguments(uint8_t *buffer)
{


}


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
