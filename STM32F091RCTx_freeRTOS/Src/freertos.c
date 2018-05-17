/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V. 
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without 
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice, 
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other 
  *    contributors to this software may be used to endorse or promote products 
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this 
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under 
  *    this license is void and will automatically terminate your rights under 
  *    this license. 
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS" 
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT 
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT 
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, 
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF 
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */     
#include <stm32f0xx_hal.h>
#include <aewin_def.h>
#include <string.h>
#include <usart.h>
#include <iwdg.h>

/* USER CODE END Includes */

/* Variables -----------------------------------------------------------------*/
osThreadId defaultTaskHandle;
osThreadId USART1_TaskHandle;
osThreadId USART2_TaskHandle;
osThreadId USART3_TaskHandle;
osThreadId IGNITION_TaskHandle;
osThreadId I2C1_TaskHandle;
osThreadId ADC_TaskHandle;
osThreadId IWDG_TaskHandle;
osMessageQId GPIO_MISC_QHandle;
osMessageQId ADC_QHandle;
osMutexId MUTEX_GPIO_QHandle;
osMutexId MUTEX_ADC_QHandle;

/* USER CODE BEGIN Variables */

/* USER CODE END Variables */

/* Function prototypes -------------------------------------------------------*/
void StartDefaultTask(void const * argument);
void usart1_entry(void const * argument);
void usart2_entry(void const * argument);
void usart3_entry(void const * argument);
void ignition_entry(void const * argument);
void i2c1_entry(void const * argument);
void adc_entry(void const * argument);
void IWDG_entry(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
#define SVA_1000			"SVA-1000:\\>"
#define SVA_1000_NL     	"\n\rSVA-1000:\\>"
#define MAX_CML_CHAR		128



/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	HAL_UART_Transmit(&huart3, SVA_1000_NL, sizeof(SVA_1000_NL) - 1, 4);
	HAL_UART_Abort(&huart3);
  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* definition and creation of MUTEX_GPIO_Q */
  osMutexDef(MUTEX_GPIO_Q);
  MUTEX_GPIO_QHandle = osMutexCreate(osMutex(MUTEX_GPIO_Q));

  /* definition and creation of MUTEX_ADC_Q */
  osMutexDef(MUTEX_ADC_Q);
  MUTEX_ADC_QHandle = osMutexCreate(osMutex(MUTEX_ADC_Q));

  /* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of USART1_Task */
  osThreadDef(USART1_Task, usart1_entry, osPriorityNormal, 0, 128);
  USART1_TaskHandle = osThreadCreate(osThread(USART1_Task), NULL);

  /* definition and creation of USART2_Task */
  osThreadDef(USART2_Task, usart2_entry, osPriorityNormal, 0, 128);
  USART2_TaskHandle = osThreadCreate(osThread(USART2_Task), NULL);

  /* definition and creation of USART3_Task */
  osThreadDef(USART3_Task, usart3_entry, osPriorityBelowNormal, 0, 256);
  USART3_TaskHandle = osThreadCreate(osThread(USART3_Task), NULL);

  /* definition and creation of IGNITION_Task */
  osThreadDef(IGNITION_Task, ignition_entry, osPriorityNormal, 0, 128);
  IGNITION_TaskHandle = osThreadCreate(osThread(IGNITION_Task), NULL);

  /* definition and creation of I2C1_Task */
  osThreadDef(I2C1_Task, i2c1_entry, osPriorityNormal, 0, 128);
  I2C1_TaskHandle = osThreadCreate(osThread(I2C1_Task), NULL);

  /* definition and creation of ADC_Task */
  osThreadDef(ADC_Task, adc_entry, osPriorityNormal, 0, 128);
  ADC_TaskHandle = osThreadCreate(osThread(ADC_Task), NULL);

  /* definition and creation of IWDG_Task */
  osThreadDef(IWDG_Task, IWDG_entry, osPriorityAboveNormal, 0, 128);
  IWDG_TaskHandle = osThreadCreate(osThread(IWDG_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of GPIO_MISC_Q */
  osMessageQDef(GPIO_MISC_Q, 16, uint16_t);
  GPIO_MISC_QHandle = osMessageCreate(osMessageQ(GPIO_MISC_Q), NULL);

  /* definition and creation of ADC_Q */
  osMessageQDef(ADC_Q, 16, uint32_t);
  ADC_QHandle = osMessageCreate(osMessageQ(ADC_Q), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
  /* USER CODE END RTOS_QUEUES */
}

/* StartDefaultTask function */
void StartDefaultTask(void const * argument)
{

  /* USER CODE BEGIN StartDefaultTask */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END StartDefaultTask */
}

/* usart1_entry function */
void usart1_entry(void const * argument)
{
  /* USER CODE BEGIN usart1_entry */
	uint8_t recv1 = 0;
	/* Infinite loop */
	for(;;)
	{
		if(HAL_UART_Receive_DMA(&huart1, (uint8_t*)&recv1, 1) == HAL_OK){
		    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&recv1, 1);
		}
	osDelay(1);
	}
  /* USER CODE END usart1_entry */
}

/* usart2_entry function */
void usart2_entry(void const * argument)
{
  /* USER CODE BEGIN usart2_entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END usart2_entry */
}

/* usart3_entry function */
void usart3_entry(void const * argument)
{
  /* USER CODE BEGIN usart3_entry */
	int temp;
	uint8_t recv3[3] = {0};
	uint8_t cml_array[MAX_CML_CHAR] = {SVA_1000};
	uint8_t *cml_head, *cml_limit, *cml_ptr = cml_array;
	HAL_StatusTypeDef u3_states = HAL_OK;
	cml_ptr += sizeof(SVA_1000) - 1;
	*cml_ptr = ' ';
	cml_head = cml_limit = cml_ptr;
	//HAL_UART_Abort(&huart3);
	osDelay(10);
	// __HAL_UART_FLUSH_DRREGISTER(&huart3);

	/* Infinite loop */
	for(;;)
	{
		recv3[0] = 0;
		if( (u3_states = HAL_UART_Receive(&huart3, (uint8_t*)&recv3[0], 1, UART3_TIMEOUT)) == HAL_OK){
			recv3[1] = 0;			if (HAL_UART_Receive(&huart3, (uint8_t*)&recv3[1], 1, UART3_TIMEOUT) == HAL_TIMEOUT){
				switch(recv3[0]){
					//-----------------------------------------------------
					case '\r':
					case '\n':
						HAL_UART_Transmit(&huart3, "\r\n", 2, UART3_TIMEOUT);
						HAL_UART_Transmit(&huart3, SVA_1000, sizeof(SVA_1000) - 1, 10);
						*(++cml_ptr) = '\0';
						cml_ptr = cml_limit = cml_head;
						break;

					//-----------------------------------------------------
					case 0x7f:
						recv3[0] = '\b';

					case '\b':
						if ((cml_ptr > cml_head) && (cml_limit > cml_head)){							temp = cml_limit - cml_ptr;
							strncpy(cml_ptr, (cml_ptr + 1), temp);
							*(cml_limit--) = 0;
							HAL_UART_Transmit(&huart3, (uint8_t*)&recv3[0], 1, UART3_TIMEOUT);
							HAL_UART_Transmit(&huart3, cml_ptr, temp , temp);
							HAL_UART_Transmit(&huart3, " ", 1, UART3_TIMEOUT);
							HAL_UART_Transmit(&huart3, (uint8_t*)&recv3[0], 1, UART3_TIMEOUT);
							cml_ptr--;

							for(temp = 0; temp < cml_limit - cml_ptr; temp++){
								HAL_UART_Transmit(&huart3, (uint8_t*)&recv3[0], 1, UART3_TIMEOUT);
							}
						}

						//HAL_UART_Transmit(&huart3, " ", 1, 10);
						//HAL_UART_Transmit(&huart3, (uint8_t*)&recv3[0], 1, 1);
						break;

					//-----------------------------------------------------
					case '\e':
						break;

					//-----------------------------------------------------
					default:
						if (cml_ptr >= cml_head + (MAX_CML_CHAR - sizeof(SVA_1000) - 1)){
							break;
						}
						*(++cml_ptr) = recv3[0];
						cml_limit = cml_ptr;
						HAL_UART_Transmit(&huart3, (uint8_t*)&recv3[0], 1, UART3_TIMEOUT);
						break;
				}

			}
			else{
				recv3[2] = 0;
				if (HAL_UART_Receive(&huart3, (uint8_t*)&recv3[2], 1, UART3_TIMEOUT) == HAL_OK){
					if (recv3[0] =='\e'){
						//recv3[0] = 0xE0;
						switch(recv3[2]){
						//-----------------------------------------------------
						// Up
						case 'A':
							//recv3[1] = 0x48;
							break;

						//-----------------------------------------------------
						// Down
						case 'B':
							//recv3[1] = 0x50;
							break;

						//-----------------------------------------------------
						// Right
						case 'C':
							//recv3[1] = 0x4D;

							if (cml_ptr < cml_limit){
								cml_ptr++;
								HAL_UART_Transmit(&huart3, (uint8_t*)&recv3[0], 3, UART3_TIMEOUT);
							}

							break;

						//-----------------------------------------------------
						// Left
						case 'D':
							if (cml_ptr > cml_head){
								cml_ptr--;
								HAL_UART_Transmit(&huart3, (uint8_t*)&recv3[0], 3, UART3_TIMEOUT);
							}
							break;

						//-----------------------------------------------------
						default:
							break;

						}
					}
				}
			}
		}
		else if(u3_states == HAL_TIMEOUT){
			HAL_UART_Abort(&huart3);
		}

	osDelay(1);
	}

  /* USER CODE END usart3_entry */
}

/* ignition_entry function */
void ignition_entry(void const * argument)
{
  /* USER CODE BEGIN ignition_entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END ignition_entry */
}

/* i2c1_entry function */
void i2c1_entry(void const * argument)
{
  /* USER CODE BEGIN i2c1_entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END i2c1_entry */
}

/* adc_entry function */
void adc_entry(void const * argument)
{
  /* USER CODE BEGIN adc_entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END adc_entry */
}

/* IWDG_entry function */
void IWDG_entry(void const * argument)
{
  /* USER CODE BEGIN IWDG_entry */
  /* Infinite loop */
  for(;;)
  {
	  HAL_IWDG_Refresh(&hiwdg); //refresh watch dog
	  osDelay(10);
  }
  /* USER CODE END IWDG_entry */
}

/* USER CODE BEGIN Application */


/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
