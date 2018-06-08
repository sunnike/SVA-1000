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
#include <stdarg.h>
#include <aewin_def.h>
#include <string.h>
#include <usart.h>
#include <adc.h>
#include <iwdg.h>
#include <gpio.h>
#include <rtc.h>
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
osThreadId CMD_PROC_TaskHandle;
osThreadId GPIO_STATE_TaskHandle;
osMessageQId GPIO_MISC_QHandle;
osMessageQId ADC_VOLT_QHandle;
osMessageQId ADC_TEMP_QHandle;
osMutexId MUTEX_DebugHandle;
osMutexId MUTEX_ADC_QHandle;
osMutexId MUTEX_CMD_PROCHandle;

/* USER CODE BEGIN Variables */
#define SVA_1000			"SVA-1000:\\>"
#define SVA_1000_NL     	"\n\rSVA-1000:\\>"
#define MAX_CML_CHAR		64

#if (AEWIN_DBUG)
char dbg_buff[PRINT_BUFF];
#endif
sSVA_GPI_STATE 	sva_gpi={0};
sSVA_GPO_STATE 	sva_gpo={0};


uint8_t cml_array[MAX_CML_CHAR] = {0};
uint8_t cml_proc[MAX_CML_CHAR] = {0};

sIG_EVENT gIG_Event ={ IG_Recovery, 2, 2, 10, 2, 2, 10, 2, 100, 5, FALSE, 5, 20, 145, 0, 50, 100, 50, 0, 0};
volatile sIG_EVENT ig_event;

/** @defgroup STM32F0XX_RTC STM32F0XX RTC time, date and alarm.
  * @{
  */
RTC_TimeTypeDef  sTime = {0};
RTC_DateTypeDef  sDate = {0};
RTC_AlarmTypeDef sAlarm = {0};


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
void cmd_proc_entry(void const * argument);
void gpio_state_entry(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
#if (AEWIN_DBUG)
void aewin_dbg(char *fmt,...);
#endif


/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	HAL_UART_Transmit_DMA(&huart3, SVA_1000_NL, sizeof(SVA_1000_NL) - 1);

  /* USER CODE END Init */

  /* Create the mutex(es) */
  /* definition and creation of MUTEX_Debug */
  osMutexDef(MUTEX_Debug);
  MUTEX_DebugHandle = osMutexCreate(osMutex(MUTEX_Debug));

  /* definition and creation of MUTEX_ADC_Q */
  osMutexDef(MUTEX_ADC_Q);
  MUTEX_ADC_QHandle = osMutexCreate(osMutex(MUTEX_ADC_Q));

  /* definition and creation of MUTEX_CMD_PROC */
  osMutexDef(MUTEX_CMD_PROC);
  MUTEX_CMD_PROCHandle = osMutexCreate(osMutex(MUTEX_CMD_PROC));

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
  osThreadDef(defaultTask, StartDefaultTask, osPriorityIdle, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of USART1_Task */
  osThreadDef(USART1_Task, usart1_entry, osPriorityNormal, 0, 128);
  USART1_TaskHandle = osThreadCreate(osThread(USART1_Task), NULL);

  /* definition and creation of USART2_Task */
  osThreadDef(USART2_Task, usart2_entry, osPriorityNormal, 0, 128);
  USART2_TaskHandle = osThreadCreate(osThread(USART2_Task), NULL);

  /* definition and creation of USART3_Task */
  osThreadDef(USART3_Task, usart3_entry, osPriorityAboveNormal, 0, 128);
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

  /* definition and creation of CMD_PROC_Task */
  osThreadDef(CMD_PROC_Task, cmd_proc_entry, osPriorityNormal, 0, 128);
  CMD_PROC_TaskHandle = osThreadCreate(osThread(CMD_PROC_Task), NULL);

  /* definition and creation of GPIO_STATE_Task */
  osThreadDef(GPIO_STATE_Task, gpio_state_entry, osPriorityNormal, 0, 128);
  GPIO_STATE_TaskHandle = osThreadCreate(osThread(GPIO_STATE_Task), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Create the queue(s) */
  /* definition and creation of GPIO_MISC_Q */
  osMessageQDef(GPIO_MISC_Q, 16, uint16_t);
  GPIO_MISC_QHandle = osMessageCreate(osMessageQ(GPIO_MISC_Q), NULL);

  /* definition and creation of ADC_VOLT_Q */
  osMessageQDef(ADC_VOLT_Q, 1, uint32_t);
  ADC_VOLT_QHandle = osMessageCreate(osMessageQ(ADC_VOLT_Q), NULL);

  /* definition and creation of ADC_TEMP_Q */
  osMessageQDef(ADC_TEMP_Q, 1, uint32_t);
  ADC_TEMP_QHandle = osMessageCreate(osMessageQ(ADC_TEMP_Q), NULL);

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
    osDelay(100);
  }
  /* USER CODE END StartDefaultTask */
}

/* usart1_entry function */
void usart1_entry(void const * argument)
{
  /* USER CODE BEGIN usart1_entry */
	uint8_t recv1[CMD_MAX_LEN] = {0};
	uint8_t test[] = {0x16, 0x16 ,0x02 ,0x10 ,0x10 ,0x03 , 0x00, 0x04};
	uint8_t data_rec_index = 0, chk_index = 0;
	/* Infinite loop */

	for(;;)
    {
		#if 0
		// Get SYN.
		if(HAL_UART_Receive_DMA(&huart1, &recv1[chk_index++], UART1_TIMEOUT) == HAL_OK){
			if(recv1[chk_index - 1] == CMD_SYN_CODE){
				// GEt SYN.
				if(HAL_UART_Receive_DMA(&huart1, &recv1[chk_index++], UART1_TIMEOUT)  == HAL_OK){
					// GEt STX.
					if(HAL_UART_Receive_DMA(&huart1, &recv1[chk_index++], UART1_TIMEOUT)  == HAL_OK){
						// Get Main command.
						if(HAL_UART_Receive_DMA(&huart1, &recv1[chk_index++], UART1_TIMEOUT)  == HAL_OK){
							// Get sub-command.
							if(HAL_UART_Receive_DMA(&huart1, &recv1[chk_index++], UART1_TIMEOUT) == HAL_OK){
								if(*((uint32_t*)recv1) == CMD_MAIN_CHK(M_CMD_MCU_SETTING)){
									switch(recv1[4]){
										//-----------------------------------------------------
										case Subcmd_MCU_FW_Ver:
											data_rec_index = SUB_MCU_FW_VER_LEN + CMS_TAIL_SIZE;
											do{
												HAL_UART_Receive_DMA(&huart1, &recv1[chk_index++], 1);
											}while(--data_rec_index);

											if((recv1[chk_index - CMS_TAIL_SIZE] == CMD_ETX_CODE) && (recv1[chk_index - 1] == CMD_EOT_CODE) )
												HAL_UART_Transmit_DMA(&huart1, test, sizeof(test));
												//aewin_dbg("\r\nGet MCU version(0x10)");

											break;
									}
								}
							}
						}
					}
				}
			}
			//HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&recv1, 1);
			/*osMutexWait(MUTEX_DebugHandle, osWaitForever);
		    HAL_UART_Transmit_DMA(&huart3, (uint8_t*)&recv1, 1, 1);
		    osMutexRelease(MUTEX_DebugHandle);*/
		}
		else if (u1_states == HAL_TIMEOUT){
			//HAL_UART_Abort(&huart1);
		}
		HAL_UART_Abort(&huart1);
		chk_index = 0;
		data_rec_index = 0;
		memset(recv1, 0, CMD_MAX_LEN);
#endif
		//if(IG_Start_Up == ig_event.IG_States){

		//}
		/*data_rec_index = 1;
		//HAL_UART_DMAResume(&huart1);
		while(HAL_UART_Receive_DMA(&huart1, &recv1[0], 1) != HAL_OK){ //&& (CMD_SYN_CODE == recv1[0])
			while((HAL_UART_Receive_DMA(&huart1, &recv1[data_rec_index++], (CMD_MAX_LEN - 1)) == HAL_OK) && (data_rec_index < CMD_MAX_LEN));
		}

		chk_index = 0;
		if(data_rec_index > 1){
			//HAL_UART_DMAResume(&huart1);
			HAL_UART_Transmit_DMA(&huart1, recv1, (data_rec_index -1));
		}
		recv1[0] = 0;*/
		/*if(HAL_UART_Receive(&huart1, &recv1[0], 1, 1) == HAL_OK){
			HAL_UART_Transmit(&huart1, &recv1[0], 1, 1);
		}*/
		osDelay(UART1_TASK_ENTRY_TIME);
	}
  /* USER CODE END usart1_entry */
}

/* usart2_entry function */
void usart2_entry(void const * argument)
{
  /* USER CODE BEGIN usart2_entry */
	//uint8_t recv2 = 0;
	//HAL_StatusTypeDef u2_states = HAL_OK;
    /* Infinite loop */
    for(;;)
    {
		/*if(HAL_UART_Receive_DMA(&huart2, (uint8_t*)&recv2, 1) == HAL_OK){
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&recv2, 1);
		}*/
#if 0
		if((u2_states = HAL_UART_Receive(&huart2, (uint8_t*)&recv2, 1, UART3_TIMEOUT)) == HAL_OK){
			HAL_UART_Transmit(&huart2, (uint8_t*)&recv2, 1, UART3_TIMEOUT);
		}
		else if (u2_states == HAL_TIMEOUT){
			HAL_UART_Abort(&huart2);
		}
#endif
		osDelay(50);
    }
  /* USER CODE END usart2_entry */
}

/* usart3_entry function */
void usart3_entry(void const * argument)
{
  /* USER CODE BEGIN usart3_entry */
	int temp;
	uint8_t recv3[3] = {0};
	uint8_t *cml_head, *cml_limit, *cml_ptr = cml_array;
	HAL_StatusTypeDef u3_states = HAL_OK;
	cml_head = cml_limit = cml_ptr;

	/* Infinite loop */
	for(;;)
	{
		recv3[0] = 0;
		osMutexWait(MUTEX_DebugHandle, osWaitForever);
		if(HAL_UART_Receive_DMA(&huart3, (uint8_t*)&recv3[0], 1) == HAL_OK){
			HAL_Delay(1);
			recv3[1] = 0;
			if (HAL_UART_Receive_DMA(&huart3, (uint8_t*)&recv3[1], 1) != HAL_OK){
				switch(recv3[0]){
					//-----------------------------------------------------
					case '\r':
					case '\n':
						HAL_UART_Transmit_DMA(&huart3, SVA_1000_NL, sizeof(SVA_1000_NL) - 1);
						*cml_limit = '\0';
						//osMutexRelease(MUTEX_DebugHandle);
						//aewin_dbg("\n\rCommands Length = %d",(uint32_t)(cml_limit - cml_head));
						osMutexWait(MUTEX_CMD_PROCHandle, osWaitForever);
						memcpy(cml_proc, cml_array, (uint32_t)(cml_limit - cml_head + 1));
						osMutexRelease(MUTEX_CMD_PROCHandle);
						cml_ptr = cml_limit = cml_head;

						break;

					//-----------------------------------------------------
					/* Delete Key. */
					case 0x7E:

						break;
					//-----------------------------------------------------
					case 0x7F:
						recv3[0] = '\b';

					case '\b':
						if ((cml_ptr > cml_head) && (cml_limit > cml_head)){
							temp = cml_limit - cml_ptr;
							*(--cml_ptr) = 0;
							strncpy(cml_ptr, (cml_ptr + 1), temp);
							HAL_UART_Transmit_DMA(&huart3, (uint8_t*)&recv3[0], 1);
							HAL_UART_Transmit_DMA(&huart3, cml_ptr, temp);
							HAL_UART_Transmit_DMA(&huart3, " ", 1);
							//HAL_UART_Transmit_DMA(&huart3, (uint8_t*)&recv3[0], 1, UART3_TIMEOUT);


							for(temp = 0; temp < cml_limit - cml_ptr; temp++){
								HAL_UART_Transmit_DMA(&huart3, (uint8_t*)&recv3[0], 1);
							}
							*(--cml_limit) = 0;
						}

						//HAL_UART_Transmit_DMA(&huart3, " ", 1, 10);
						//HAL_UART_Transmit_DMA(&huart3, (uint8_t*)&recv3[0], 1, 1);
						break;

					//-----------------------------------------------------
					case '\e':
						break;

					//-----------------------------------------------------
					default:
						if (cml_ptr >= cml_head + MAX_CML_CHAR - 1){
							break;
						}
						*(cml_ptr++) = recv3[0];
						cml_limit = cml_ptr;
						HAL_UART_Transmit_DMA(&huart3, (uint8_t*)&recv3[0], 1);
						break;
				}

			}
			else{
				recv3[2] = 0;
				if (HAL_UART_Receive_DMA(&huart3, (uint8_t*)&recv3[2], 1) == HAL_OK){
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
								HAL_UART_Transmit_DMA(&huart3, (uint8_t*)&recv3[0], 3);
							}

							break;

						//-----------------------------------------------------
						// Left
						case 'D':
							if (cml_ptr > cml_head){
								cml_ptr--;
								HAL_UART_Transmit_DMA(&huart3, (uint8_t*)&recv3[0], 3);
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
		//else /*if(u3_states == HAL_TIMEOUT)*/{
			//HAL_UART_Abort(&huart3);
		//}

		//HAL_DMA_Abort(&hdma_usart3_tx);
		//HAL_DMA_Abort(&hdma_usart3_rx);
		osMutexRelease(MUTEX_DebugHandle);
		osDelay(UART3_TASK_ENTRY_TIME);
	}

  /* USER CODE END usart3_entry */
}

/* ignition_entry function */
void ignition_entry(void const * argument)
{
  /* USER CODE BEGIN ignition_entry */
	osEvent  	evt;
	uint32_t 	adc_24v, adc_temp;

	ig_event = gIG_Event;
    /* Infinite loop */
    for(;;)
    {

		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		aewin_dbg("\n\rMUC Time: %2d:%2d:%2d",sTime.Hours ,sTime.Minutes, sTime.Seconds);
		aewin_dbg("\n\rMUC Date: 20%2d_%2d_%2d  Weekday:%d",sDate.Year ,sDate.Month, sDate.Date ,sDate.WeekDay);

    	/** Get ADC 24V from Q. */
    	evt = osMessageGet(ADC_VOLT_QHandle, osWaitForever);
		if (evt.status == osEventMessage){
			adc_24v = evt.value.v;
			//aewin_dbg("\n\r");
			//aewin_dbg("\n\rGet ADC 24V = %.1d.%.2d V", (((adc_24v & 0xfffU) * 330) / 409600), \
							  	  	  	  	  	  	   ((((adc_24v & 0xfffU) * 330) / 4096) % 100));
		}

		/** Get the ADC value of MCU temperature. */
		evt = osMessageGet(ADC_TEMP_QHandle, osWaitForever);
		if (evt.status == osEventMessage){
			adc_temp = evt.value.v;
			//aewin_dbg("\n\r");
			//aewin_dbg("\n\rGet ADC TEMP = %.1d.%.2d V", (((adc_temp & 0xfffU) * 330) / 409600), \
													  ((((adc_temp & 0xfffU) * 330) / 4096) % 100));
		}


		switch(ig_event.IG_States){
			case IG_Recovery:
				// Recovery all of IG states and paremeters.
				ig_event = gIG_Event;
				ig_event.IG_States = IG_CloseUp;
				break;

			//-------------------------------------------------------------------------------------
			case IG_CloseUp:
				// Judge the IG switch states.
				if(sva_gpi.ig_sw == GPIO_PIN_SET){
					/* IG On process */
					// Ready to enter IG_PowerOn_Delay state.
					ig_event.IG_States = IG_PowerOn_Delay;
					// Reset the "Start up timeout"
					ig_event.startup_timeout = gIG_Event.startup_timeout;
					aewin_dbg("\n\rIgnition ON! IG_CloseUp --> IG_PowerOn_Delay");
				}

				// Judge the power button states.
				if(sva_gpi.pwr_btn == GPIO_PIN_RESET){
					ig_event.startup_timeout = gIG_Event.startup_timeout;
					ig_event.IG_States = IG_Wait_StartUp;
					aewin_dbg("\n\rPower button On! IG_CloseUp --> IG_Wait_StartUp");
				}
				break;

			//-------------------------------------------------------------------------------------
			case IG_PowerOn_Delay:
				// Judge the IG switch states.
				if(sva_gpi.ig_sw == GPIO_PIN_SET){
					if (0 == (ig_event.pwron_delay--)){
						ig_event.pwron_delay = gIG_Event.pwron_delay;
						ig_event.IG_States = IG_Wait_StartUp;
						aewin_dbg("\n\rPower on delay pass! IG_PowerOn_Delay --> IG_Wait_StartUp");
					}
				}
				else{
					ig_event.pwron_delay = gIG_Event.pwron_delay;
					ig_event.IG_States = IG_CloseUp;
					aewin_dbg("\n\rPower on delay failed! IG_PowerOn_Delay --> IG_CloseUp");
				}
				break;

			//-------------------------------------------------------------------------------------
			case IG_Wait_StartUp:
				/* Power button has been (long) pressed. */
				if(sva_gpi.pwr_btn == GPIO_PIN_RESET){
					ig_event.pwrbtn_pressed = TRUE;
					if(0 == (ig_event.pwroff_btn_cnt--)){
						ig_event.IG_States = IG_Recovery;
						HAL_GPIO_WritePin(GPIOC, D2D_EN_Pin, GPIO_PIN_RESET);
						aewin_dbg("\n\rIG_Wait_StartUp failed! IG_Wait_StartUp --> IG_Recovery");
					}
				}

				if(ig_event.pwrbtn_pressed && (sva_gpi.pwr_btn == GPIO_PIN_SET)){
					ig_event.pwroff_btn_cnt = gIG_Event.pwroff_btn_cnt;
					ig_event.pwrbtn_pressed = FALSE;
				}

				if(sva_gpi.ig_sw == GPIO_PIN_SET){
					if (0 == (ig_event.wait_startup_time--)){
						ig_event.wait_startup_time = gIG_Event.wait_startup_time;
						HAL_GPIO_WritePin(GPIOC, D2D_EN_Pin, GPIO_PIN_SET);
						// Confirm the the DC2DC power and system power are available.
						while((sva_gpi.dc2dc_pwrok != GPIO_PIN_SET) || (sva_gpi.sys_pwron == GPIO_PIN_SET)){
							if(0 == (ig_event.pwrgood_chk_time--)){
								break;
							}
						}
						if(0 != ig_event.pwrgood_chk_time){
							ig_event.IG_States = IG_Start_Up;
							aewin_dbg("\n\rPower on delay ok! IG_Wait_StartUp --> IG_Start_Up");
						}
						else{
							HAL_GPIO_WritePin(GPIOC, D2D_EN_Pin, GPIO_PIN_RESET);
							ig_event.IG_States = IG_CloseUp;
							aewin_dbg("\n\rDE2DC Power on failed! IG_Wait_StartUp --> IG_CloseUp");
						}
						ig_event.pwrgood_chk_time = gIG_Event.pwrgood_chk_time;
					}
				}
				else{
					ig_event.wait_startup_time = gIG_Event.wait_startup_time;
					ig_event.IG_States = IG_PowerOn_Delay;
					aewin_dbg("\n\rIG_Wait_StartUp failed! IG_Wait_StartUp --> IG_PowerOn_Delay");
				}
				break;

			//-------------------------------------------------------------------------------------
			case IG_Start_Up:
				if(ig_event.startup_timeout > 0){
					/* Lock ignition off to prevent that user shutdowns the OS. */
					ig_event.startup_timeout--;
					aewin_dbg("\n\rLock power on!");
				}
				else{
					if( sva_gpi.ig_sw ==  GPIO_PIN_RESET){
						ig_event.IG_States = IG_Shutdown_Delay;
						aewin_dbg("\n\rIngition switch off! IG_Start_Up --> IG_Shutdown_Delay");
					}
				}

				if(sva_gpi.pwr_btn == GPIO_PIN_RESET){
					ig_event.pwrbtn_pressed = TRUE;
					if(0 == (ig_event.pwroff_btn_cnt--)){
						HAL_GPIO_WritePin(GPIOC, D2D_EN_Pin, GPIO_PIN_RESET);
						ig_event.IG_States = IG_Recovery;
					}
				}

				if(ig_event.pwrbtn_pressed && (sva_gpi.pwr_btn == GPIO_PIN_SET)){
					ig_event.pwroff_btn_cnt = gIG_Event.pwroff_btn_cnt;
					ig_event.pwrbtn_pressed = FALSE;
					ig_event.IG_States = IG_shutting_Down;
					aewin_dbg("\n\rPower button off! IG_Start_Up --> IG_shutting_Down");
				}

				break;

			//-------------------------------------------------------------------------------------
			case IG_Shutdown_Delay:
				if(sva_gpi.ig_sw == GPIO_PIN_RESET){
					if (0 == (ig_event.pwroff_delay--)){
						ig_event.IG_States = IG_shutting_Down;
						ig_event.pwroff_delay = gIG_Event.pwroff_delay;
						aewin_dbg("\n\rPower off delay pass! IG_Shutdown_Delay --> IG_shutting_Down");
					}
				}
				else{
					ig_event.IG_States = IG_Start_Up;
					ig_event.pwroff_delay = gIG_Event.pwroff_delay;
					aewin_dbg("\n\rPower off delay failed! IG_Shutdown_Delay --> IG_Start_Up");
				}
				break;

			//-------------------------------------------------------------------------------------
			case IG_shutting_Down:
				if(sva_gpi.ig_sw == GPIO_PIN_RESET){
					if (0 == (ig_event.shutdown_delay--)){
						ig_event.IG_States = IG_CloseUp;
						ig_event.shutdown_delay = gIG_Event.shutdown_delay;
						HAL_GPIO_WritePin(GPIOC, D2D_EN_Pin, GPIO_PIN_RESET);
						aewin_dbg("\n\rShutdown delay pass! IG_shutting_Down --> IG_CloseUp");
					}
				}
				else{
					ig_event.IG_States = IG_Shutdown_Delay;
					ig_event.shutdown_delay = gIG_Event.shutdown_delay;
					aewin_dbg("\n\rShutdown delay failed! IG_shutting_Down --> IG_Shutdown_Delay");
				}
				break;

			//-------------------------------------------------------------------------------------
			case IG_LowPower_Delay:
				break;

		}
    	osDelay(INGITION_TASK_ENTRY_TIME);
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
		//aewin_dbg("\n\rI2C Task");
        osDelay(50);
	}
  /* USER CODE END i2c1_entry */
}

/* adc_entry function */
void adc_entry(void const * argument)
{
  /* USER CODE BEGIN adc_entry */
    /* Variable containing ADC conversions results */
	sADC_DATA adc_data;

	/* Infinite loop */
	for(;;)
    {

        /* Start ADC conversion on regular group with transfer by DMA */
		if (HAL_ADC_Start_DMA(&hadc, (uint32_t *)&adc_data, ADC_DEVICE_NUM) != HAL_OK)
		{
			// ADC start failed.
			aewin_dbg("\n\rADC start up failed!");
		}
		// ADC initial and capture
		osDelay(ADC_CAP_TIME);

		// Get ADC value of battery voltage.
		if( osMessagePut(ADC_VOLT_QHandle, adc_data.pwr_24v, osWaitForever) != osOK )
		{
			aewin_dbg("\n\rADC_24V put Q failed. \r\n");
		}

		// Get ADC value of MUC temperature
		if( osMessagePut(ADC_TEMP_QHandle, adc_data.mcu_temp, osWaitForever) != osOK )
		{
			aewin_dbg("\n\rMCU-Temperature put Q failed. \r\n");
		}

		/* Stop ADC converting. */
		HAL_ADC_Stop_DMA(&hadc);
		// Let ADC converter take a break.
		osDelay(ADC_RESET_TIME);

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
	  /* Refresh IWDG */
	  HAL_IWDG_Refresh(&hiwdg);
	  osDelay(IWDG_TASK_ENTRY_TIME);
  }
  /* USER CODE END IWDG_entry */
}

/* cmd_proc_entry function */
void cmd_proc_entry(void const * argument)
{
  /* USER CODE BEGIN cmd_proc_entry */
  /* Infinite loop */
  for(;;)
  {
    osDelay(2);
  }
  /* USER CODE END cmd_proc_entry */
}

/* gpio_state_entry function */
void gpio_state_entry(void const * argument)
{
  /* USER CODE BEGIN gpio_state_entry */
  /* Infinite loop */
    for(;;)
    {
		/* Get ignition switch states. */
		sva_gpi.ig_sw 		= HAL_GPIO_ReadPin(GPIOC, IGNITION_SW_Pin);
		/* Get system power states. */
		sva_gpi.sys_pwron	= HAL_GPIO_ReadPin(GPIOC, SYS_POWER_ON_Pin);
		/* Get DC2DC power states. */
		sva_gpi.dc2dc_pwrok	= HAL_GPIO_ReadPin(GPIOC, DC2DC_PWROK_Pin);
		/* Get power button states. */
		sva_gpi.pwr_btn 	= HAL_GPIO_ReadPin(GPIOC, PWR_BTN_IGN_R_Pin);

		osDelay(GPIO_GET_TASK_TIME);
    }
  /* USER CODE END gpio_state_entry */
}

/* USER CODE BEGIN Application */
#if (AEWIN_DBUG)
void aewin_dbg(char *fmt,...){
	osMutexWait(MUTEX_DebugHandle, osWaitForever);
	int i = 0;
	static va_list arg_ptr;
	va_start (arg_ptr, fmt);
	vsnprintf(dbg_buff, PRINT_BUFF, fmt, arg_ptr);
	while(i < (PRINT_BUFF - 1) && dbg_buff[i]){
		if (HAL_UART_Transmit_DMA(&huart3, (uint8_t*)&dbg_buff[i], 1) == HAL_OK){
			i++;
		}
	}
	va_end(arg_ptr);
	osMutexRelease(MUTEX_DebugHandle);
}
#endif

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
