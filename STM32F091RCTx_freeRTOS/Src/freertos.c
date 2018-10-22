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
#include <i2c.h>
#include <spi.h>
#include <can.h>

#include<math.h>
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
osThreadId SPI1_TaskHandle;
osThreadId CAN_TaskHandle;
osMessageQId GPIO_MISC_QHandle;
osMessageQId ADC_VOLT_QHandle;
osMessageQId ADC_TEMP_QHandle;
osMessageQId I2C1_GSENSOR_QHandle;
osMessageQId I2C1_XAXIS_QHandle;
osMessageQId I2C1_YAXIS_QHandle;
osMessageQId I2C1_ZAXIS_QHandle;
osMessageQId UART2_LAT_QHandle;
osMessageQId UART2_LONG_QHandle;
osMutexId MUTEX_DebugHandle;
osMutexId MUTEX_ADC_QHandle;
osMutexId MUTEX_CMD_PROCHandle;
osMutexId MUTEX_SPI1Handle;

/* USER CODE BEGIN Variables */
#define SVA_1000			"SVA-1000:\\>"
#define SVA_1000_NL     	"\n\rSVA-1000:\\>"
#define MAX_CML_CHAR		64

#if (AEWIN_DBUG)
char dbg_buff[PRINT_BUFF];
#endif

//debug timer counter
TickType_t xTimeNow;

uint8_t uart2_msg_print_switch = 1;
uint8_t wwan_command = 0;
// 0 : no command
// 1 : enable
// 2 : disable

sSVA_GPI_STATE 	sva_gpi={0};
sSVA_GPO_STATE 	sva_gpo={0};

s4G_MODULE r280_module_state = {0};

sCOUNTDOWN_TIMER countdown_timer = {0};
sIG_VAR ig_var = {IG_Recovery, FALSE, 0};


uint8_t cml_array[MAX_CML_CHAR] = {0};
uint8_t cml_proc[MAX_CML_CHAR] = {0};

uint8_t uart_Tx[][26] = {"AT", "AT+cind?", "at+cgdcont?", "AT+CGACT=1,1", "AT+COPS?", "AT+CSQ", "at+ugps=1,0", "at+ugps=0", "AT+UGRMC=1",
		"AT+UGRMC?", "AT+UGPRF=1", "AT+CFUN?", "AT+CFUN=4", "AT+CFUN=1",  "AT+UPING=\"www.google.com\"", "AT+UPSD=0,1,\"apn.name\"",
		"AT+UPSDA=0,0", "AT+UPSDA=0,3"};


//sIG_EVENT gIG_Event ={ 0x12345678, 0, 1, IG_Recovery, 1, 1, 1, 2, 2, 10, 2, 100, 5, FALSE, 5, 12, 20, 145, 0, 50, 100, 50, 0, 0, 0};
//sIG_EVENT gIG_Event ={ 0x12345678, 0, 1, IG_Recovery, 4, 1, 60, 10, 4, 60, 30, 300, 5, FALSE, 5, 12, 9, 54, 0, 50, 100, 50, 0, 0, 0}; //ignition mode
//sIG_EVENT gIG_Event ={ 0x12345678, 0, 1, IG_Recovery, 4, 1, 60, 10, 4, 90, 30, 300, 5, FALSE, 5, 12, 20, 145, 0, 50, 100, 50, 0, 0, 0}; //power adapter mode
//uint8_t flash_IgEvent[29] = {SPI_FLASH_PROGRAM_PAGE, SPI_FLASH_ADD_Byte0, SPI_FLASH_ADD_Byte1, SPI_FLASH_ADD_Byte2, SPI_FLASH_DATA_TAG, 0, 1, IG_Recovery, 1, 1, 1, 2, 2, 10, 2, 100, 5, FALSE, 5, 12, 20, 145, 0, 50, 100, 50, 0, 0, 0};

volatile uint8_t current_IgEvent[NUM_total];

// ignition (test, boot faster)
/*
//uint8_t flash_IgEvent[NUM_total] = {SPI_FLASH_PROGRAM_PAGE, SPI_FLASH_ADD_Byte0, SPI_FLASH_ADD_Byte1, SPI_FLASH_ADD_Byte2,
		SPI_FLASH_DATA_TAG, 0, 1, IG_Recovery, 4, 1,
		1, 10, 2, 10, 2, 30, 5, FALSE, 5, 12,
		20, 145, 0, 50,	100, 50, 0, 0, 0, 0,
		0, 0, 0, 0,	0, 0, 0, 0, 0, 0,
		0, 10, 10, 0, 9, 36, 0, 0, 0, 0,
		0, 0, 0x03, 0x03, 0x03, 0x03};
*/

// ignition (test, boot faster) less
uint8_t flash_IgEvent[NUM_total] = {SPI_FLASH_PROGRAM_PAGE, SPI_FLASH_ADD_Byte0, SPI_FLASH_ADD_Byte1, SPI_FLASH_ADD_Byte2,
		SPI_FLASH_DATA_TAG, 0, 1, 4, 1,	1,
		10, 2, 10, 2, 30, 5, 5, 12,	20, 145,
		0, 50, 100, 50, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 9, 36,
		0, 0, 0, 0, 0, 0, 0x03, 0x03, 0x03, 0x03};    // RTC_WakeT_min



/* ignition (default)
//less
uint8_t flash_IgEvent[NUM_total] = {SPI_FLASH_PROGRAM_PAGE, SPI_FLASH_ADD_Byte0, SPI_FLASH_ADD_Byte1, SPI_FLASH_ADD_Byte2,
		SPI_FLASH_DATA_TAG, 0, 1, 4, 1, 60,
		10, 4, 60, 2, 30, 5, 5, 12,	9, 54,
		0, 50,	100, 50, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 9, 36,
		0, 0, 0, 0, 0, 0, 0x03, 0x03, 0x03, 0x03};    // RTC_WakeT_min


//uint8_t flash_IgEvent[NUM_total] = {SPI_FLASH_PROGRAM_PAGE, SPI_FLASH_ADD_Byte0, SPI_FLASH_ADD_Byte1, SPI_FLASH_ADD_Byte2,
		SPI_FLASH_DATA_TAG, 0, 1, IG_Recovery, 4, 1,
		60, 10, 4, 60, 2, 30, 5, FALSE, 5, 12,
		9, 54, 0, 50,	100, 50, 0, 0, 0, 0,
		0, 0, 0, 0,	0, 0, 0, 0, 0, 0,
		0, 10, 10, 0, 9, 36, 0, 0, 0, 0,
		0, 0, 0x03, 0x03, 0x03, 0x03};

*/

/* power adapter (option)
//less
uint8_t flash_IgEvent[NUM_total] = {SPI_FLASH_PROGRAM_PAGE, SPI_FLASH_ADD_Byte0, SPI_FLASH_ADD_Byte1, SPI_FLASH_ADD_Byte2,
		SPI_FLASH_DATA_TAG, 0, 1, 4, 1, 60,
		10, 4, 90, 2, 30, 5, 5, 12,	20, 145,
		0, 50, 100, 50, 0, 0, 0, 0, 0, 0,
		0, 0, 0, 0, 0, 0, 0, 0, 9, 36,
		0, 0, 0, 0, 0, 0, 0x03, 0x03, 0x03, 0x03};    // RTC_WakeT_min


//uint8_t flash_IgEvent[NUM_total] = {SPI_FLASH_PROGRAM_PAGE, SPI_FLASH_ADD_Byte0, SPI_FLASH_ADD_Byte1, SPI_FLASH_ADD_Byte2,
		SPI_FLASH_DATA_TAG, 0, 1, IG_Recovery, 4, 1,
		60, 10, 4, 90, 2, 30, 5, FALSE, 5, 12,
		20, 145, 0, 50,	100, 50, 0, 0, 0, 0,
		0, 0, 0, 0,	0, 0, 0, 0, 0, 0,
		0, 10, 10, 0, 9, 36, 0, 0, 0, 0,
		0, 0, 0x03, 0x03, 0x03, 0x03};
*/

uint8_t flag_flashWrite = 0;

volatile sIG_EVENT ig_event;

/** @defgroup STM32F0XX_RTC STM32F0XX RTC time, date and alarm.
  * @{
  */
RTC_TimeTypeDef  sTime = {0};
RTC_DateTypeDef  sDate = {0};
RTC_AlarmTypeDef sAlarm = {0};

DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart1_tx;

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
void spi1_entry(void const * argument);
void can_entry(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* USER CODE BEGIN FunctionPrototypes */
#if (AEWIN_DBUG)
void aewin_dbg(char *fmt,...);
uint8_t adc_voltageConversion_int(uint32_t adc_value);
uint8_t adc_voltageConversion_float(uint32_t adc_value);
uint8_t adc_tempConversion_int(uint32_t adc_value);
uint8_t adc_tempConversion_float(uint32_t adc_value);
uint8_t uart2_findCommaIndexNext(uint8_t uart_msg[], uint8_t target_comma);
uint8_t uart2_isFindString(uint8_t uart_msg[], uint8_t target_string[], uint8_t length);
void enable_4GModule(void);
uint8_t uart_findDataIndex(uint8_t uart_msg[], uint8_t target_data);
void print_atCommand(uint8_t rx_msg[], uint8_t onOff);
uint8_t check_lowPower(uint32_t adc_value);
uint8_t check_highPower(uint32_t adc_value);
void config_recovery(void);
#endif


/* USER CODE END FunctionPrototypes */

/* Hook prototypes */

/* Init FreeRTOS */

void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	uint16_t data_index;

	//HAL_UART_Transmit(&huart3, SVA_1000_NL, sizeof(SVA_1000_NL) - 1, sizeof(SVA_1000_NL));
	HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	//read flash

	//current_IgEvent load data from flash_IgEvent
	config_recovery();


	//countdown_timer initialize
	countdown_timer.poweron_delay    = current_IgEvent[NUM_pwron_delay];
	countdown_timer.startup_timeout  = current_IgEvent[NUM_startup_timeout];
	countdown_timer.shutdown_delay   = current_IgEvent[NUM_shutdown_delay];
	countdown_timer.shutdown_timeout = current_IgEvent[NUM_shutdown_timeout];
	countdown_timer.poweroff_delay   = current_IgEvent[NUM_pwroff_delay];


	enable_4GModule();

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

  /* definition and creation of MUTEX_SPI1 */
  osMutexDef(MUTEX_SPI1);
  MUTEX_SPI1Handle = osMutexCreate(osMutex(MUTEX_SPI1));

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
  osThreadDef(USART1_Task, usart1_entry, osPriorityHigh, 0, 256);
  USART1_TaskHandle = osThreadCreate(osThread(USART1_Task), NULL);

  /* definition and creation of USART2_Task */
  osThreadDef(USART2_Task, usart2_entry, osPriorityNormal, 0, 256);
  USART2_TaskHandle = osThreadCreate(osThread(USART2_Task), NULL);

  /* definition and creation of USART3_Task */
  osThreadDef(USART3_Task, usart3_entry, osPriorityNormal, 0, 128);
  USART3_TaskHandle = osThreadCreate(osThread(USART3_Task), NULL);

  /* definition and creation of IGNITION_Task */
  osThreadDef(IGNITION_Task, ignition_entry, osPriorityNormal, 0, 256);
  IGNITION_TaskHandle = osThreadCreate(osThread(IGNITION_Task), NULL);

  /* definition and creation of I2C1_Task */
  osThreadDef(I2C1_Task, i2c1_entry, osPriorityNormal, 0, 128);
  I2C1_TaskHandle = osThreadCreate(osThread(I2C1_Task), NULL);

  /* definition and creation of ADC_Task */
  osThreadDef(ADC_Task, adc_entry, osPriorityNormal, 0, 128);
  ADC_TaskHandle = osThreadCreate(osThread(ADC_Task), NULL);

  /* definition and creation of IWDG_Task */
  osThreadDef(IWDG_Task, IWDG_entry, osPriorityHigh, 0, 128);
  IWDG_TaskHandle = osThreadCreate(osThread(IWDG_Task), NULL);

  /* definition and creation of CMD_PROC_Task */
  osThreadDef(CMD_PROC_Task, cmd_proc_entry, osPriorityNormal, 0, 128);
  CMD_PROC_TaskHandle = osThreadCreate(osThread(CMD_PROC_Task), NULL);

  /* definition and creation of GPIO_STATE_Task */
  osThreadDef(GPIO_STATE_Task, gpio_state_entry, osPriorityNormal, 0, 128);
  GPIO_STATE_TaskHandle = osThreadCreate(osThread(GPIO_STATE_Task), NULL);

  /* definition and creation of SPI1_Task */
  osThreadDef(SPI1_Task, spi1_entry, osPriorityHigh, 0, 128);
  SPI1_TaskHandle = osThreadCreate(osThread(SPI1_Task), NULL);

  /* definition and creation of CAN_Task */
  osThreadDef(CAN_Task, can_entry, osPriorityNormal, 0, 128);
  CAN_TaskHandle = osThreadCreate(osThread(CAN_Task), NULL);

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

  /* definition and creation of I2C1_GSENSOR_Q */
  osMessageQDef(I2C1_GSENSOR_Q, 16, uint16_t);
  I2C1_GSENSOR_QHandle = osMessageCreate(osMessageQ(I2C1_GSENSOR_Q), NULL);

  /* definition and creation of I2C1_XAXIS_Q */
  osMessageQDef(I2C1_XAXIS_Q, 2, uint16_t);
  I2C1_XAXIS_QHandle = osMessageCreate(osMessageQ(I2C1_XAXIS_Q), NULL);

  /* definition and creation of I2C1_YAXIS_Q */
  osMessageQDef(I2C1_YAXIS_Q, 2, uint16_t);
  I2C1_YAXIS_QHandle = osMessageCreate(osMessageQ(I2C1_YAXIS_Q), NULL);

  /* definition and creation of I2C1_ZAXIS_Q */
  osMessageQDef(I2C1_ZAXIS_Q, 2, uint16_t);
  I2C1_ZAXIS_QHandle = osMessageCreate(osMessageQ(I2C1_ZAXIS_Q), NULL);

  /* definition and creation of UART2_LAT_Q */
  osMessageQDef(UART2_LAT_Q, 1, uint32_t);
  UART2_LAT_QHandle = osMessageCreate(osMessageQ(UART2_LAT_Q), NULL);

  /* definition and creation of UART2_LONG_Q */
  osMessageQDef(UART2_LONG_Q, 1, uint32_t);
  UART2_LONG_QHandle = osMessageCreate(osMessageQ(UART2_LONG_Q), NULL);

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
	uint8_t xFer[CMD_MAX_LEN] = {0};
	uint8_t send2host = FALSE;
	uint8_t xFer_len = 0;

	uint8_t uart1_rx_buffer[CMD_MAX_LEN]= {0};
	uint8_t u16InsIdx,u16Delidx;
	uint8_t rx_byte_counter = 0;
	uint8_t flag_rx_finished = 0;

	osEvent  	evt_vol;
	osEvent     evt_temp;
	osEvent     evt_xaxis;
	osEvent     evt_yaxis;
	osEvent     evt_zaxis;
	osEvent     evt_lat;
	osEvent     evt_long;

	HAL_UART_Receive_DMA(&huart1, &uart1_rx_buffer[CMD_SYN_POS0] ,CMD_MAX_LEN);
	u16InsIdx = 0;
	u16Delidx = 0;

	/* Infinite loop */

	for(;;)
    {
		// check DMA new data
		u16InsIdx = CMD_MAX_LEN - (hdma_usart1_rx.Instance->CNDTR);

		// if data frame packet is not received completely and there are still some data in DMA buffer that uart1 haven't read
		// if haven't read the data in DMA buffer
		if((u16InsIdx != u16Delidx) && (flag_rx_finished == 0))
		{
			recv1[rx_byte_counter] = uart1_rx_buffer[u16Delidx];

			// end of data frame packet, received completely
			if((rx_byte_counter > 6) && (recv1[rx_byte_counter - 2] == 0x03) && (recv1[rx_byte_counter - 1] == 0x00) && (recv1[rx_byte_counter] == 0x04))
			{
				rx_byte_counter = 0;
				flag_rx_finished = 1;
			}
			else
			{
				rx_byte_counter = rx_byte_counter+1;
			}

			//get 1 byte from circular buffer
			u16Delidx = u16Delidx + 1;
			u16Delidx %= CMD_MAX_LEN;
		}
#if 1
		/*

		//xTimeNow = xTaskGetTickCount();
		//aewin_dbg("\n\rTask: UART1; TaskGetTickCount: %d", xTimeNow);

		// Wait until the first byte is coming
		//while((HAL_UART_Receive_IT(&huart1, &recv1[CMD_SYN_POS0], 100)) != HAL_OK );
		// Get the SYN code.
		//taskENTER_CRITICAL();
		while(HAL_UART_Receive_DMA(&huart1, &recv1[CMD_SYN_POS0] ,(CMD_MAX_LEN - 1)) != HAL_OK);
		//HAL_UART_Receive(&huart1, &recv1[CMD_SYN_POS1] ,(CMD_MAX_LEN - 1), 100);

		// Give some time for DMA receiving data. Let this thread take a break.
		osDelay(25);
		//taskENTER_CRITICAL();
		//HAL_Delay(20);

		// Abort UART processing.
		HAL_UART_Abort_IT(&huart1);

		*/
#else

		// Wait until the first byte is coming
		if ((HAL_UART_Receive(&huart1, &recv1[CMD_SYN_POS0], 1, 1)) == HAL_OK ){
			// Get the SYN code.
			HAL_UART_Receive_DMA(&huart1, &recv1[CMD_SYN_POS1] ,(CMD_MAX_LEN - 1));
			// Give some time for DMA receiving data. Let this thread take a break.
			osDelay(30);
			// Abort UART processing.
			HAL_UART_Abort_IT(&huart1);
		}
		else{
			continue;
		}
#endif
		//
		// Check command head.
		if ((CMD_SYN_CODE == recv1[CMD_SYN_POS0]) && (CMD_SYN_CODE == recv1[CMD_SYN_POS1]) && (CMD_STX_CODE == recv1[CMD_STX_POS]) && (flag_rx_finished == 1)){
			switch(recv1[CMD_MCMD_POS]){
				//-----------------------------------------------------
				case M_CMD_MCU_SETTING:
					switch(recv1[CMD_SCMD_POS]){
						//-----------------------------------------------------
						case Subcmd_MCU_FW_Ver:
							// Check tail codes.
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							xFer[2] = Subcmd_MCU_FW_Ver;
							xFer[3] = flash_IgEvent[NUM_major_ver];
							xFer[4] = flash_IgEvent[NUM_minor_ver];

							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE + MCU_SCMD10_LEN;

							aewin_dbg("\n\rHost get MCU version:%d.%d", xFer[3], xFer[4]);
							break;

						//-----------------------------------------------------
						case Subcmd_MCU_Get_Date:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
							HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
							xFer[2] = Subcmd_MCU_Get_Date;
							xFer[3] = sDate.Year; 		// Year low byte.
							xFer[4] = 20; 				// Year high byte.
							xFer[5] = sDate.Month;		// Month.
							xFer[6] = sDate.Date;		// Date.
							xFer[7] = sTime.Hours;		// Hours.
							xFer[8] = sTime.Minutes;	// Minutes.
							xFer[9] = sTime.Seconds;	// Seconds.

							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE + MCU_SCMD20_LEN;

							aewin_dbg("\n\rGet Time: %2d:%2d:%2d",sTime.Hours ,sTime.Minutes, sTime.Seconds);
							aewin_dbg("\n\rGet Date: 20%2d_%2d_%2d  Weekday:%d",sDate.Year ,sDate.Month, sDate.Date ,sDate.WeekDay);
							break;

						//-----------------------------------------------------
						case Subcmd_MCU_Set_Date:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE + MCU_SCMD21_LEN)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE + MCU_SCMD21_LEN)])) break;
							// recv1[6] = 20, no need to change
							sDate.Year  = recv1[5]; 		// Year low byte.
							sDate.Month	= recv1[7];			// Month.
							sDate.Date  = recv1[8];			// Date.
							sTime.Hours = recv1[9];			// Hours.
							sTime.Minutes = recv1[10];;		// Minutes.
							sTime.Seconds = recv1[11];;		// Seconds.

							HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
							HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

							aewin_dbg("\n\rSet Time: %2d:%2d:%2d",sTime.Hours ,sTime.Minutes, sTime.Seconds);
							aewin_dbg("\n\rSet Date: 20%2d_%2d_%2d  Weekday:%d",sDate.Year ,sDate.Month, sDate.Date ,sDate.WeekDay);
							break;

						//-----------------------------------------------------
						case Subcmd_Get_Sys_InVOLT:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							evt_vol = osMessageGet(ADC_VOLT_QHandle,osWaitForever);
							xFer[2] = Subcmd_Get_Sys_InVOLT;
							xFer[3] = current_IgEvent[NUM_in_sys_volt];         //Input voltage

							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE + MCU_SCMD22_LEN;

							aewin_dbg("\n\rGet system input voltage: %2d", xFer[3]);
							break;

						//-----------------------------------------------------
						case Subcmd_Set_Sys_InVOLT:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE + MCU_SCMD23_LEN)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE + MCU_SCMD23_LEN)])) break;
							current_IgEvent[NUM_in_sys_volt]=recv1[5];

							flag_flashWrite = 1;

							aewin_dbg("\n\rSet  system input voltage: %2d", current_IgEvent[NUM_in_sys_volt]);
							break;

						//-----------------------------------------------------
						case Subcmd_Get_RebootSrc:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							xFer[2] = Subcmd_Get_RebootSrc;
							xFer[3] = current_IgEvent[NUM_reboot_source];

							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE + MCU_SCMD26_LEN;

							aewin_dbg("\n\rGet reboot source: %d", xFer[3]);
							break;

						//-----------------------------------------------------
						case Subcmd_Set_RebootSrc:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE + MCU_SCMD27_LEN)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE + MCU_SCMD27_LEN)])) break;
							current_IgEvent[NUM_reboot_source]=recv1[5];

							flag_flashWrite = 1;

							aewin_dbg("\n\rSet reboot source: %d", current_IgEvent[NUM_reboot_source]);
							break;

						//-----------------------------------------------------
						case Subcmd_Get_BootMode:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							xFer[2] = Subcmd_Get_BootMode;
							xFer[3] = current_IgEvent[NUM_boot_mode];

							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE + MCU_SCMD28_LEN;

							aewin_dbg("\n\rGet boot mode: %d", xFer[3]);
							break;

						//-----------------------------------------------------
						case Subcmd_Set_BootMode:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE + MCU_SCMD29_LEN)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE + MCU_SCMD29_LEN)])) break;
							current_IgEvent[NUM_boot_mode]=recv1[5];

							flag_flashWrite = 1;

							aewin_dbg("\n\rSet boot mode: %d", current_IgEvent[NUM_boot_mode]);
							break;

						//-----------------------------------------------------
						case Subcmd_Get_WWAN_WKStat:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							xFer[2] = Subcmd_Get_WWAN_WKStat;
							xFer[3] = current_IgEvent[NUM_WWAN_wakeup];
							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE + MCU_SCMD30_LEN;

							aewin_dbg("\n\rGet WWAN wake up status: %d", xFer[3]);
							break;

						//-----------------------------------------------------
						case Subcmd_Set_WWAN_WKStat:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE + MCU_SCMD31_LEN)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE + MCU_SCMD31_LEN)])) break;
							current_IgEvent[NUM_WWAN_wakeup]=recv1[5];

							flag_flashWrite = 1;

							aewin_dbg("\n\rGet WWAN wake up status: %d", current_IgEvent[NUM_WWAN_wakeup]);
							break;

						//-----------------------------------------------------
						case Subcmd_Get_WWAN_Stat:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							xFer[2] = Subcmd_Get_WWAN_Stat;
							xFer[3] = current_IgEvent[NUM_WWAN_status];

							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE + MCU_SCMD32_LEN;

							aewin_dbg("\n\rGet WWAN status: %d", xFer[3]);
							break;

						//-----------------------------------------------------
						case Subcmd_Set_WWAN_Stat:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE + MCU_SCMD33_LEN)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE + MCU_SCMD33_LEN)])) break;
							current_IgEvent[NUM_WWAN_status]=recv1[5];

							if(current_IgEvent[NUM_WWAN_status] == 1)
							{
								wwan_command = 1;
							}
							else if(current_IgEvent[NUM_WWAN_status] == 0)
							{
								wwan_command = 2;
							}
							flag_flashWrite = 1;

							aewin_dbg("\n\rGet WWAN status: %d", current_IgEvent[NUM_WWAN_status]);
							break;

						//-----------------------------------------------------
						case Subcmd_Get_DigiIn:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							xFer[2] = Subcmd_Get_DigiIn;
							//xFer[3] = ((sva_gpi.ig_sw) || (sva_gpi.pwr_btn<<1) || (sva_gpi.sys_pwron<<2));
							xFer[3] = sva_gpi.ebtn_in;

							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE + MCU_SCMD32_LEN;

							aewin_dbg("\n\rGet GPIO digital input : %x", xFer[3]);
							break;

						//-----------------------------------------------------
						case Subcmd_Get_DigiOut:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							xFer[2] = Subcmd_Get_DigiOut;
							xFer[3] = sva_gpo.ebtn_out;

							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE + MCU_SCMD36_LEN;

							aewin_dbg("\n\rGet GPIO digital output : %x", xFer[3]);
							break;

						//-----------------------------------------------------
						case Subcmd_Set_DigiOut:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE + MCU_SCMD37_LEN)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE + MCU_SCMD37_LEN)])) break;
							sva_gpo.ebtn_out = (recv1[5] & 0x01);
							HAL_GPIO_WritePin(GPIOC, EBTN_OUT_Pin, (recv1[5] & 0x01));

							aewin_dbg("\n\rSet GPIO digital output : %x", sva_gpo.ebtn_out);
							break;

						//-----------------------------------------------------
						case Subcmd_Get_SIM_Mode:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							xFer[2] = Subcmd_Get_SIM_Mode;
							xFer[3] = current_IgEvent[NUM_sim_card_mode];

							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE + MCU_SCMD3A_LEN;

							aewin_dbg("\n\rGet SIM card select: %d", xFer[3]);
							break;

						//-----------------------------------------------------
						case Subcmd_Set_SIM_Mode:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE + MCU_SCMD3B_LEN)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE + MCU_SCMD3B_LEN)])) break;
							current_IgEvent[NUM_sim_card_mode]=recv1[5];

							flag_flashWrite = 1;

							aewin_dbg("\n\rSet SIM card select: %d", current_IgEvent[NUM_sim_card_mode]);
							//Write flash
							break;

						//-----------------------------------------------------
						case Subcmd_Get_WIFI_OnOff:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							xFer[2] = Subcmd_Get_WIFI_OnOff;
							xFer[3] = current_IgEvent[NUM_wifi_status];

							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE + MCU_SCMD40_LEN;

							aewin_dbg("\n\rGet WIFI status: %d", xFer[3]);
							break;

						//-----------------------------------------------------
						case Subcmd_Set_WIFI_OnOff:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE + MCU_SCMD41_LEN)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE + MCU_SCMD41_LEN)])) break;
							current_IgEvent[NUM_wifi_status]=recv1[5];

							flag_flashWrite = 1;

							aewin_dbg("\n\rSet WIFI status: %d", current_IgEvent[NUM_wifi_status]);
							break;

						//-----------------------------------------------------
						case Subcmd_Get_LAN_WKStat:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							xFer[2] = Subcmd_Get_LAN_WKStat;
							xFer[3] = current_IgEvent[NUM_LAN_wakeup];

							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE + MCU_SCMD44_LEN;

							aewin_dbg("\n\rGet LAN wake up enable/disable: %d", xFer[3]);
							break;

						//-----------------------------------------------------
						case Subcmd_Set_LAN_WKStat:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE + MCU_SCMD45_LEN)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE + MCU_SCMD45_LEN)])) break;
							current_IgEvent[NUM_LAN_wakeup]=recv1[5];

							flag_flashWrite = 1;

							aewin_dbg("\n\rSet LAN wake up enable/disable: %d", current_IgEvent[NUM_LAN_wakeup]);
							break;

						//-----------------------------------------------------
						case Subcmd_Get_DelayOffStat:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							xFer[2] = Subcmd_Get_DelayOffStat;
							xFer[3] = current_IgEvent[NUM_delay_off_setting];

							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE + MCU_SCMD50_LEN;

							aewin_dbg("\n\rGet delay off enable/disable: %d", xFer[3]);
							break;

						//-----------------------------------------------------
						case Subcmd_Set_DelayOffStat:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE + MCU_SCMD51_LEN)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE + MCU_SCMD51_LEN)])) break;
							current_IgEvent[NUM_delay_off_setting]=recv1[5];

							flag_flashWrite = 1;

							aewin_dbg("\n\rSet delay off enable/disable: %d", current_IgEvent[NUM_delay_off_setting]);
							break;

						//-----------------------------------------------------
						case Subcmd_Get_DelayOnStat:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							xFer[2] = Subcmd_Get_DelayOnStat;
							xFer[3] = current_IgEvent[NUM_delay_on_setting];

							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE + MCU_SCMD52_LEN;

							aewin_dbg("\n\rGet delay on enable/disable: %d", xFer[3]);
							break;

						//-----------------------------------------------------
						case Subcmd_Set_DelayOnStat:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE + MCU_SCMD53_LEN)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE + MCU_SCMD53_LEN)])) break;
							current_IgEvent[NUM_delay_on_setting]=recv1[5];

							flag_flashWrite = 1;

							aewin_dbg("\n\rSet delay on enable/disable: %d", current_IgEvent[NUM_delay_on_setting]);
							break;

						//-----------------------------------------------------
						case Subcmd_Get_DelayOffTime:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							xFer[2] = Subcmd_Get_DelayOffTime;
							//xFer[3] = current_IgEvent[NUM_power_off_time];
							xFer[3] = current_IgEvent[NUM_pwroff_delay];

							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE + MCU_SCMD54_LEN;

							aewin_dbg("\n\rGet delay time of power off: %d", xFer[3]);
							break;

						//-----------------------------------------------------
						case Subcmd_Set_DelayOffTime:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE + MCU_SCMD55_LEN)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE + MCU_SCMD55_LEN)])) break;
							//current_IgEvent[NUM_power_off_time]=recv1[5];
							current_IgEvent[NUM_pwroff_delay] = recv1[5];

							countdown_timer.poweroff_delay = current_IgEvent[NUM_pwroff_delay];
							flag_flashWrite = 1;

							aewin_dbg("\n\rSet delay time of power off: %d", current_IgEvent[NUM_pwroff_delay]);
							break;

						//-----------------------------------------------------
						case Subcmd_Get_DelayOnTime:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							xFer[2] = Subcmd_Get_DelayOnTime;
							//xFer[3] = current_IgEvent[NUM_power_on_time];
							xFer[3] = current_IgEvent[NUM_pwron_delay];

							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE + MCU_SCMD56_LEN;

							aewin_dbg("\n\rGet delay time of power on: %d", xFer[3]);
							break;

						//-----------------------------------------------------
						case Subcmd_Set_DelayOnTime:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE + MCU_SCMD57_LEN)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE + MCU_SCMD57_LEN)])) break;
							//current_IgEvent[NUM_power_on_time]=recv1[5];
							current_IgEvent[NUM_pwron_delay] = recv1[5];

							countdown_timer.poweron_delay = current_IgEvent[NUM_pwron_delay];
							flag_flashWrite = 1;

							aewin_dbg("\n\rSet delay time of power on: %d", current_IgEvent[NUM_pwron_delay]);
							break;

						//-----------------------------------------------------
						case Subcmd_Get_ADC:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							evt_vol = osMessageGet(ADC_VOLT_QHandle,osWaitForever);
							evt_temp = osMessageGet(ADC_TEMP_QHandle, osWaitForever);
							xFer[2] = Subcmd_Get_ADC;
							//xFer[3] = evt_vol.value.v;         //Input voltage
							xFer[3] = adc_voltageConversion_float(evt_vol.value.v);
							xFer[4] = adc_voltageConversion_int(evt_vol.value.v);
							xFer[5] = adc_tempConversion_float(evt_temp.value.v);      //MCU temperature
							xFer[6] = adc_tempConversion_int(evt_temp.value.v);        //MCU temperature

							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE + MCU_SCMD70_LEN;

							aewin_dbg("\n\rGet system input voltage: %2d.%2d", xFer[4], xFer[3]);
							aewin_dbg("\n\rGet system input temperature: %2d.%2d", xFer[6], xFer[5]);
							aewin_dbg("\n\rGet MCU temperature: %2d", xFer[4]);
							break;

						//-----------------------------------------------------
						case Subcmd_Get_GPS:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							evt_lat = osMessageGet(UART2_LAT_QHandle,osWaitForever);
							evt_long = osMessageGet(UART2_LONG_QHandle,osWaitForever);
							xFer[2] = Subcmd_Get_GPS;
							xFer[3] = (evt_lat.value.v)/10000000;
							xFer[4] = ((evt_lat.value.v)%10000000)/100000;
							xFer[5] = (((evt_lat.value.v)%10000000)%100000)*60/100000;
							xFer[6] = (evt_long.value.v)/10000000;
							xFer[7] = ((evt_long.value.v)%10000000)/100000;
							xFer[8] = (((evt_long.value.v)%10000000)%100000)*60/100000;

							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE + MCU_SCMD80_LEN;

							aewin_dbg("\n\rPrint GPS data: %u, %u", evt_lat.value.v, evt_long.value.v);
							aewin_dbg("\n\rPrint GPS data (compute result): %d %d %d, %d %d %d", (evt_lat.value.v)/10000000, ((evt_lat.value.v)%10000000)*60/10000000, ( (((evt_lat.value.v)%10000000)*60) % 10000000)*60/10000000,
									(evt_long.value.v)/10000000, ((evt_long.value.v)%10000000)*60/10000000, ( (((evt_long.value.v)%10000000)*60) % 10000000)*60/10000000);
							aewin_dbg("\n\rGet GPS data: %d %d %d, %d %d %d", xFer[3], xFer[4], xFer[5], xFer[6], xFer[7], xFer[8]);
							break;

						//-----------------------------------------------------
						case Subcmd_Get_Gsensor:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							evt_xaxis = osMessageGet(I2C1_XAXIS_QHandle,osWaitForever);
							evt_yaxis = osMessageGet(I2C1_YAXIS_QHandle,osWaitForever);
							evt_zaxis = osMessageGet(I2C1_ZAXIS_QHandle,osWaitForever);
							xFer[2] = Subcmd_Get_Gsensor;
							//if (evt_xaxis.status == osEventMessage)
							{
								xFer[3] = ((evt_xaxis.value.v)>>8);
								xFer[4] = ((evt_xaxis.value.v) & 0x00FF);

								xFer[5] = ((evt_yaxis.value.v)>>8);
								xFer[6] = ((evt_yaxis.value.v) & 0x00FF);

								xFer[7] = ((evt_zaxis.value.v)>>8);
								xFer[8] = ((evt_zaxis.value.v) & 0x00FF);
							}
							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE + MCU_SCMD90_LEN;

							aewin_dbg("\n\rGet G-sensor data: %x %x %x %x", xFer[3], xFer[4], xFer[5], xFer[6], xFer[7], xFer[8]);
							break;

						//-----------------------------------------------------
						case Subcmd_OS_Shutdown:
							break;

						//-----------------------------------------------------
						default:
							break;
					}
					break;

				//-----------------------------------------------------
				case M_CMD_IG_SETTING:
					switch(recv1[CMD_SCMD_POS]){
						case Subcmd_IG_Get_OnOff:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							xFer[2] = Subcmd_IG_Get_OnOff;
							//if(current_IgEvent[NUM_ig_states] == IG_Start_Up)
							if(ig_var.ig_states == IG_Start_Up)
							{
								xFer[3] = 0x01;
							}
							else
							{
								xFer[3] = 0x00;
							}
							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE + IGN_SCMD10_LEN;

							aewin_dbg("\n\rGet ignition on/off status: %x", xFer[3]);
							break;

						//-----------------------------------------------------
						case Subcmd_Get_LowBat_Data:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							xFer[2] = Subcmd_Get_LowBat_Data;
							xFer[3] = current_IgEvent[NUM_12V_startup];
							xFer[4] = current_IgEvent[NUM_12V_shutdown];
							xFer[5] = current_IgEvent[NUM_24V_startup];
							xFer[6] = current_IgEvent[NUM_24V_shutdown];

							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE + IGN_SCMD14_LEN;

							aewin_dbg("\n\rGet low battery data: %x %x %x %x", xFer[3], xFer[4], xFer[5], xFer[6]);
							break;

						//-----------------------------------------------------
						case Subcmd_Set_LowBat_Data:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE + IGN_SCMD15_LEN)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE + IGN_SCMD15_LEN)])) break;
							current_IgEvent[NUM_12V_startup] = recv1[5];
							current_IgEvent[NUM_12V_shutdown] = recv1[6];
							current_IgEvent[NUM_24V_startup] = recv1[7];
							current_IgEvent[NUM_24V_shutdown] = recv1[8];

							flag_flashWrite = 1;

							aewin_dbg("\n\rGet low battery data: %x %x %x %x", recv1[5], recv1[6], recv1[7], recv1[8]);
							break;

						//-----------------------------------------------------
						case Subcmd_Get_InVol_Limit:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							xFer[2] = Subcmd_Get_InVol_Limit;
							xFer[3] = current_IgEvent[NUM_InVol_limit_min];
							xFer[4] = current_IgEvent[NUM_InVol_limit_max];

							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE + IGN_SCMD16_LEN;

							aewin_dbg("\n\rGet input voltage limit: %d V ~ %d V", xFer[3], xFer[4]);
							break;

						//-----------------------------------------------------
						case Subcmd_Set_InVol_Limit:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE + IGN_SCMD17_LEN)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE + IGN_SCMD17_LEN)])) break;
							current_IgEvent[NUM_InVol_limit_min] = recv1[5];
							current_IgEvent[NUM_InVol_limit_max] = recv1[6];

							flag_flashWrite = 1;

							aewin_dbg("\n\rSet input voltage limit: %d V ~ %d V", current_IgEvent[NUM_InVol_limit_min], current_IgEvent[NUM_InVol_limit_max]);
							break;

						//-----------------------------------------------------
						case Subcmd_Get_PwrOn_DelayT:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							xFer[2] = Subcmd_Get_PwrOn_DelayT;
							//xFer[3] = current_IgEvent[NUM_power_on_time];
							//xFer[4] = current_IgEvent[NUM_power_off_time];
							xFer[3] = current_IgEvent[NUM_pwron_delay];
							xFer[4] = current_IgEvent[NUM_pwroff_delay];

							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE + IGN_SCMD18_LEN;

							aewin_dbg("\n\rGet power delay time. power on: %d, power off: %d", xFer[3], xFer[4]);
							break;

						//-----------------------------------------------------
						case Subcmd_Set_PwrOn_DelayT:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE + IGN_SCMD19_LEN)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE + IGN_SCMD19_LEN)])) break;
							//current_IgEvent[NUM_power_on_time] = recv1[5];
							//current_IgEvent[NUM_power_off_time] = recv1[6];
							current_IgEvent[NUM_pwron_delay] = recv1[5];
							current_IgEvent[NUM_pwroff_delay] = recv1[6];

							countdown_timer.poweron_delay = current_IgEvent[NUM_pwron_delay];
							countdown_timer.poweroff_delay = current_IgEvent[NUM_pwroff_delay];
							flag_flashWrite = 1;

							aewin_dbg("\n\rSet power delay time. power on: %d, power off: %d", current_IgEvent[NUM_pwron_delay], current_IgEvent[NUM_pwroff_delay]);
							break;

						//-----------------------------------------------------
						case Subcmd_Get_Alarm_Stat:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							xFer[2] = Subcmd_Get_Alarm_Stat;
							xFer[3] = current_IgEvent[NUM_alarm_status];

							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE + IGN_SCMD20_LEN;

							aewin_dbg("\n\rGet alarm status: %x", xFer[3]);
							break;

						//-----------------------------------------------------
						case Subcmd_Set_Alarm_Stat:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE + IGN_SCMD21_LEN)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE + IGN_SCMD21_LEN)])) break;
							current_IgEvent[NUM_alarm_status] = recv1[5];

							flag_flashWrite = 1;
							//HAL_RTC_DeactivateAlarm(&hrtc, RTC_ALARM_A);

							aewin_dbg("\n\rSet alarm status: %d", current_IgEvent[NUM_alarm_status]);
							break;

						//-----------------------------------------------------
						case Subcmd_Get_RTC_LocalT:
							//0x10 0x20?
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
							HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
							xFer[2] = Subcmd_Get_RTC_LocalT;
							//xFer[3] = sDate.Year; 		// Year low byte.
							//xFer[4] = 20; 				// Year high byte.
							//xFer[5] = sDate.Month;		// Month.
							//xFer[6] = sDate.Date;		// Date.
							xFer[3] = sTime.Hours;		// Hours.
							xFer[4] = sTime.Minutes;	// Minutes.
							xFer[5] = sTime.Seconds;	// Seconds.

							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE + IGN_SCMD22_LEN;

							aewin_dbg("\n\rGet RTC local time: %2d:%2d:%2d",sTime.Hours ,sTime.Minutes, sTime.Seconds);
							//aewin_dbg("\n\rGet Date: 20%2d_%2d_%2d  Weekday:%d",sDate.Year ,sDate.Month, sDate.Date ,sDate.WeekDay);
							break;

						//-----------------------------------------------------
						case Subcmd_Set_RTC_LocalT:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE + IGN_SCMD23_LEN)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE + IGN_SCMD23_LEN)])) break;
							sTime.Hours = recv1[5];		// Hours.
							sTime.Minutes = recv1[6];	// Minutes.
							sTime.Seconds = recv1[7];	// Seconds.
							HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BIN);

							aewin_dbg("\n\rSet RTC local time: %2d:%2d:%2d", sTime.Hours, sTime.Minutes, sTime.Seconds);
							break;

						//-----------------------------------------------------
						case Subcmd_Get_RTC_AlarmT:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							xFer[2] = Subcmd_Get_RTC_AlarmT;
							//xFer[3] = current_IgEvent[NUM_RTC_AlarmT_hour];
							//xFer[4] = current_IgEvent[NUM_RTC_AlarmT_min];
							//xFer[5] = current_IgEvent[NUM_RTC_AlarmT_sec];
							if(HAL_RTC_GetAlarm(&hrtc, &sAlarm, RTC_ALARM_A, RTC_FORMAT_BIN) != HAL_OK)
							{
								/* Initialization Error */
								//Error_Handler();
							}

							xFer[3] = sAlarm.AlarmTime.Hours;
							xFer[4] = sAlarm.AlarmTime.Minutes;
							xFer[5] = sAlarm.AlarmTime.Seconds;

							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE + IGN_SCMD24_LEN;

							aewin_dbg("\n\rGet RTC alarm time: %d : %d : %d", xFer[3],xFer[4], xFer[5]);
							break;

						//-----------------------------------------------------
						case Subcmd_Set_RTC_AlarmT:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE + IGN_SCMD25_LEN)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE + IGN_SCMD25_LEN)])) break;
							current_IgEvent[NUM_RTC_AlarmT_hour] = recv1[5];	// Hours
							current_IgEvent[NUM_RTC_AlarmT_min] = recv1[6];	    // Minutes
							current_IgEvent[NUM_RTC_AlarmT_sec] = recv1[7];	    // Seconds

							sAlarm.AlarmTime.Hours = recv1[5];
							sAlarm.AlarmTime.Minutes = recv1[6];
							sAlarm.AlarmTime.Seconds = recv1[7];
							sAlarm.Alarm = RTC_ALARM_A;

							if(HAL_RTC_SetAlarm_IT(&hrtc,&sAlarm,RTC_FORMAT_BIN) != HAL_OK)
							  {
								/* Initialization Error */
								Error_Handler();
							  }

							aewin_dbg("\n\rSet RTC alarm time: %d : %d : %d", current_IgEvent[NUM_RTC_AlarmT_hour], current_IgEvent[NUM_RTC_AlarmT_min], current_IgEvent[NUM_RTC_AlarmT_sec]);
							break;

						//-----------------------------------------------------
						case Subcmd_Get_RTC_WakeT:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							xFer[2] = Subcmd_Get_RTC_WakeT;
							xFer[3] = current_IgEvent[NUM_RTC_WakeT_hour];
							xFer[4] = current_IgEvent[NUM_RTC_WakeT_min];
							xFer[5] = current_IgEvent[NUM_RTC_WakeT_sec];

							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE + IGN_SCMD26_LEN;

							aewin_dbg("\n\rGet RTC wake time: %d : %d : %d", xFer[3],xFer[4], xFer[5]);
							break;

						//-----------------------------------------------------
						case Subcmd_Set_RTC_WakeT:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE + IGN_SCMD27_LEN)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE + IGN_SCMD27_LEN)])) break;
							current_IgEvent[NUM_RTC_WakeT_hour] = recv1[5];	 // Hours.
							current_IgEvent[NUM_RTC_WakeT_min] = recv1[6];	 // Minutes.
							current_IgEvent[NUM_RTC_WakeT_sec] = recv1[7];	 // Seconds.

							flag_flashWrite = 1;

							aewin_dbg("\n\rSet RTC wake time: %d : %d : %d", current_IgEvent[NUM_RTC_WakeT_hour], current_IgEvent[NUM_RTC_WakeT_min], current_IgEvent[NUM_RTC_WakeT_sec]);
							break;

						//-----------------------------------------------------
						case Subcmd_Get_Host_WTGT:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							xFer[2] = Subcmd_Get_Host_WTGT;
							xFer[3] = current_IgEvent[NUM_wtdog_default];

							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE + IGN_SCMD40_LEN;

							aewin_dbg("\n\rGet host WDT time: %d", xFer[3]);
							break;

						//-----------------------------------------------------
						case Subcmd_Set_Host_WTGT:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE + IGN_SCMD41_LEN)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE + IGN_SCMD41_LEN)])) break;
							current_IgEvent[NUM_wtdog_default] = recv1[5];

							flag_flashWrite = 1;

							aewin_dbg("\n\rSet host WDT time: %d", flash_IgEvent[NUM_wtdog_default]);
							break;

						//-----------------------------------------------------
						default:
							break;
					}
					break;

				//-----------------------------------------------------
				case M_CMD_4G_SETTING:
					switch(recv1[CMD_SCMD_POS]){
						case Subcmd_TeleComm_Event:
							if((CMD_ETX_CODE != recv1[CMD_ETX_POS(CMD_HEAD_MS_SIZE)]) || \
							   (CMD_EOT_CODE != recv1[CMD_EOT_POS(CMD_HEAD_MS_SIZE)])) break;
							HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
							HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
							xFer[2] = Subcmd_TeleComm_Event;
							xFer[3] = sDate.Year; 		// Year low byte.
							xFer[4] = 20; 				// Year high byte.
							xFer[5] = sDate.Month;		// Month.
							xFer[6] = sDate.Date;		// Date.
							xFer[7] = sTime.Hours;		// Hours.
							xFer[8] = sTime.Minutes;	// Minutes.
							xFer[9] = sTime.Seconds;	// Seconds.
							//xFer[10] = current_IgEvent[NUM_ig_states];	    // Ignition status.
							xFer[10] = ig_var.ig_states;
							xFer[11] = current_IgEvent[NUM_shutdown_delay];	// Ignition Shutdown Count.

							//if(current_IgEvent[NUM_ig_states] == IG_Start_Up)           //Ignition On/Off Status
							if(ig_var.ig_states == IG_Start_Up)
							{
								xFer[12] = 0x00;
							}
							else
							{
								xFer[12] = 0x01;
							}

							xFer[13] = 0x00;                                       // Restart Failed &  Retry Times
							xFer[14] = r280_module_state.normal_flight_states;     // 4G status
							xFer[15] = r280_module_state.register_3G_4G;           // 3G/4G register
							xFer[16] = r280_module_state.signal_strength;          // Signal strength indicator
							xFer[17] = r280_module_state.signal_channel;           // Signal strength channel#
							xFer[18] = r280_module_state.ip_add_0;                 // 4G IP address
							xFer[19] = r280_module_state.ip_add_1;                 // 4G IP address
							xFer[20] = r280_module_state.ip_add_2;                 // 4G IP address
							xFer[21] = r280_module_state.ip_add_3;                 // 4G IP address
							xFer[22] = r280_module_state.sim_card;                 // SIM Card Status
							xFer[23] = r280_module_state.ping_status;              // 4G Ping Status
							xFer[24] = r280_module_state.server_response_time_0;   // Customer Server Response Time byte0
							xFer[25] = r280_module_state.server_response_time_0;   // Customer Server Response Time byte1

							send2host = TRUE;
							xFer_len = MCU_REPO_HEAD_CMD_SIZE +  W4G_SCMD10_LEN;

							aewin_dbg("\n\rGet Telecommunication Event.");
							break;
						//-----------------------------------------------------
						default:
							break;
					}
					break;

				//-----------------------------------------------------
				default:
					break;

			}

			if(TRUE == send2host){
				xFer[MCU_ID_POS] = MCU_REPO_ID;
				switch(recv1[CMD_MCMD_POS]){
					//-----------------------------------------------------
					case M_CMD_MCU_SETTING:
						xFer[1] = M_CMD_MCU_SETTING;
						break;
					//-----------------------------------------------------
					case M_CMD_IG_SETTING:
						xFer[1] = M_CMD_IG_SETTING;
						break;

					//-----------------------------------------------------
					case M_CMD_4G_SETTING:
						xFer[1] = M_CMD_4G_SETTING;
						break;
				}

				xFer[xFer_len] = CMD_ETX_CODE; 		// ETX code.
				xFer[xFer_len + 1] = 0x00; 			// Check sum.
				xFer[xFer_len + 2] = CMD_EOT_CODE;	// EOT code.
				xFer_len += CMD_TAIL_SIZE;

				// Start to transmit to host.
				HAL_UART_Transmit_DMA(&huart1, xFer, xFer_len);
				osDelay(UART1_TX_DELAY);
				send2host = FALSE;
				xFer_len = 0;
			}

			flag_rx_finished = 0;
			memset(recv1, 0, CMD_MAX_LEN);

		}
		//taskEXIT_CRITICAL();
		//memset(recv1, 0, CMD_MAX_LEN);
		osDelay(UART1_TASK_ENTRY_TIME);
	}

  /* USER CODE END usart1_entry */
}

/* usart2_entry function */
void usart2_entry(void const * argument)
{
  /* USER CODE BEGIN usart2_entry */
	uint8_t recv2[UART2_RX_LENGTH] = {0};
	char split_count;
	uint32_t latitude = 0, longitude = 0;
	uint8_t latitude_index = 0, longitude_index = 0;
	uint8_t reg_index, ip_index, strength_index;
	uint8_t ip_counter;

	uint8_t string_OK[2] = {'O','K'};
	uint8_t string_Not[3] = {'N', 'o', 't'};
	uint8_t string_UUPING[7] = {'+', 'U', 'U', 'P', 'I', 'N', 'G'};
	uint8_t string_UUPINGER[9] = {'+', 'U', 'U', 'P', 'I', 'N', 'G', 'E', 'R'};

	uint8_t uart2_test;

	int i;


	osDelay(UART2_WAIT_READY);
	//----------------------------------------


	// Check register status AT+cind?
	HAL_UART_Transmit(&huart2, uart_Tx[ATCMD_Check_Status], sizeof(uart_Tx[ATCMD_Check_Status])-1, 30);
	HAL_UART_Transmit(&huart2, "\r", 1, 30);

	if(HAL_UART_Receive_DMA(&huart2, recv2, UART2_RX_LENGTH) != HAL_OK)
	{
		//Error_Handler();
	}
	osDelay(UART2_ATCMD_DELAY);
	HAL_UART_Abort_IT(&huart2);

	print_atCommand(recv2, uart2_msg_print_switch);
	memset(recv2, 0, UART2_RX_LENGTH);
	//----------------------------------------


	// Get IP AT+CGACT=1,1
	HAL_UART_Transmit(&huart2, uart_Tx[ATCMD_Get_IP], sizeof(uart_Tx[ATCMD_Get_IP])-1, 30);
	HAL_UART_Transmit(&huart2, "\r", 1, 30);

	if(HAL_UART_Receive_DMA(&huart2, recv2, UART2_RX_LENGTH) != HAL_OK)
	  {
		//Error_Handler();
	  }
	osDelay(UART2_ATCMD_DELAY);
	HAL_UART_Abort_IT(&huart2);

	print_atCommand(recv2, uart2_msg_print_switch);
	memset(recv2, 0, UART2_RX_LENGTH);
	//----------------------------------------


	// Check IP at+cgdcont?
	HAL_UART_Transmit(&huart2, uart_Tx[ATCMD_Check_APNIP], sizeof(uart_Tx[ATCMD_Check_APNIP])-1, 30);
	HAL_UART_Transmit(&huart2, "\r", 1, 30);

	if(HAL_UART_Receive_DMA(&huart2, recv2, UART2_RX_LENGTH) != HAL_OK)
    {
		//Error_Handler();
	}
	osDelay(UART2_ATCMD_DELAY);
	HAL_UART_Abort_IT(&huart2);

	print_atCommand(recv2, uart2_msg_print_switch);
	memset(recv2, 0, UART2_RX_LENGTH);
	//----------------------------------------


	// Check signal AT+CSQ
	HAL_UART_Transmit(&huart2, uart_Tx[ATCMD_Check_Signal], sizeof(uart_Tx[ATCMD_Check_Signal])-1, 30);
	HAL_UART_Transmit(&huart2, "\r", 1, 30);

	if(HAL_UART_Receive_DMA(&huart2, recv2, UART2_RX_LENGTH) != HAL_OK)
	{
		//Error_Handler();
	}
	osDelay(UART2_ATCMD_DELAY);
	HAL_UART_Abort_IT(&huart2);

	print_atCommand(recv2, uart2_msg_print_switch);
	memset(recv2, 0, UART2_RX_LENGTH);
	//----------------------------------------


	// set APN provider
	HAL_UART_Transmit(&huart2, uart_Tx[ATCMD_Set_APN], sizeof(uart_Tx[ATCMD_Set_APN])-1, 30);
	HAL_UART_Transmit(&huart2, "\r", 1, 30);

	if(HAL_UART_Receive_DMA(&huart2, recv2, UART2_RX_LENGTH) != HAL_OK)
	{
		//Error_Handler();
	}
	osDelay(UART2_ATCMD_DELAY);
	HAL_UART_Abort_IT(&huart2);

	print_atCommand(recv2, uart2_msg_print_switch);
	memset(recv2, 0, UART2_RX_LENGTH);
	//----------------------------------------


	// reset PSD
	HAL_UART_Transmit(&huart2, uart_Tx[ATCMD_Reset_PSD], sizeof(uart_Tx[ATCMD_Reset_PSD])-1, 30);
	HAL_UART_Transmit(&huart2, "\r", 1, 30);

	if(HAL_UART_Receive_DMA(&huart2, recv2, UART2_RX_LENGTH) != HAL_OK)
	{
		//Error_Handler();
	}
	osDelay(UART2_ATCMD_DELAY*2);
	HAL_UART_Abort_IT(&huart2);

	print_atCommand(recv2, uart2_msg_print_switch);
	memset(recv2, 0, UART2_RX_LENGTH);
	//----------------------------------------


	// activate PSD
	HAL_UART_Transmit(&huart2, uart_Tx[ATCMD_Enable_PSD], sizeof(uart_Tx[ATCMD_Enable_PSD])-1, 30);
	HAL_UART_Transmit(&huart2, "\r", 1, 30);

	if(HAL_UART_Receive_DMA(&huart2, recv2, UART2_RX_LENGTH) != HAL_OK)
	{
		//Error_Handler();
	}
	osDelay(UART2_ATCMD_DELAY*2);
	HAL_UART_Abort_IT(&huart2);

	print_atCommand(recv2, uart2_msg_print_switch);
	memset(recv2, 0, UART2_RX_LENGTH);
	//----------------------------------------


	// GPS
	//----------------------------------------
	// Configures the data flow to and from a u-blox GPS receiver connected to the Wireless Module AT+UGPRF=1
	HAL_UART_Transmit(&huart2, uart_Tx[ATCMD_Config_GPS], sizeof(uart_Tx[ATCMD_Config_GPS])-1, 30);
	HAL_UART_Transmit(&huart2, "\r", 1, 30);

	if(HAL_UART_Receive_DMA(&huart2, recv2, UART2_RX_LENGTH) != HAL_OK)
	  {
		//Error_Handler();
	  }
	osDelay(UART2_ATCMD_DELAY);
	HAL_UART_Abort_IT(&huart2);

	print_atCommand(recv2, uart2_msg_print_switch);
	memset(recv2, 0, UART2_RX_LENGTH);
	//----------------------------------------


	// Enable the NMEA $RMC messages AT+UGRMC=1
	HAL_UART_Transmit(&huart2, uart_Tx[ATCMD_Enable_RMC], sizeof(uart_Tx[ATCMD_Enable_RMC])-1, 30);
	HAL_UART_Transmit(&huart2, "\r", 1, 30);

	if(HAL_UART_Receive_DMA(&huart2, recv2, UART2_RX_LENGTH) != HAL_OK)
	  {
		//Error_Handler();
	  }
	osDelay(UART2_ATCMD_DELAY);
	HAL_UART_Abort_IT(&huart2);

	print_atCommand(recv2, uart2_msg_print_switch);
	memset(recv2, 0, UART2_RX_LENGTH);
	//----------------------------------------


	// Enable GPS AT+UGPS=1
	HAL_UART_Transmit(&huart2, uart_Tx[ATCMD_Enable_GPS], sizeof(uart_Tx[ATCMD_Enable_GPS])-1, 30);
	HAL_UART_Transmit(&huart2, "\r", 1, 30);

	if(HAL_UART_Receive_DMA(&huart2, recv2, UART2_RX_LENGTH) != HAL_OK)
	  {
		//Error_Handler();
	  }
	osDelay(UART2_ATCMD_DELAY);
	HAL_UART_Abort_IT(&huart2);

	print_atCommand(recv2, uart2_msg_print_switch);
	memset(recv2, 0, UART2_RX_LENGTH);
	//----------------------------------------


	//uint8_t recv2 = 0;
	//HAL_StatusTypeDef u2_states = HAL_OK;
    /* Infinite loop */
    for(;;)
    {
		/*if(HAL_UART_Receive_DMA(&huart2, (uint8_t*)&recv2, 1) == HAL_OK){
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)&recv2, 1);
		}*/


    	/*##-2- Start the transmission process #####################################*/
    	  /* While the UART in reception process, user can transmit data through
    	     "aTxBuffer" buffer */
    	#if 0
    	if(HAL_UART_Transmit(&huart2, "+CSQ: 2,5", 10, 5000)!= HAL_OK)
    	  {
    	    Error_Handler();
    	  }


    	  /*##-3- Put UART peripheral in reception process ###########################*/
    	  if(HAL_UART_Receive(&huart2, (uint8_t *)recv2, 10, 5000) != HAL_OK)
    	  {
    	    Error_Handler();
    	  }



    	  if(HAL_UART_Receive_IT(&huart2, (uint8_t*)&recv2, 10)!= HAL_OK)
    	{
    		Error_Handler();
    	}

    	while (UartReady != SET)
		  {
		  }
#endif


    	if(wwan_command == 1)
    	{
    		// enable WWAN, normal mode
    		wwan_command = 0;

    		// Normal mode
			HAL_UART_Transmit(&huart2, uart_Tx[ATCMD_Dis_airplane], sizeof(uart_Tx[ATCMD_Dis_airplane])-1, 30);
			HAL_UART_Transmit(&huart2, "\r", 1, 30);

			if(HAL_UART_Receive_DMA(&huart2, recv2, UART2_RX_LENGTH) != HAL_OK)
			{
				//Error_Handler();
			}
			osDelay(UART2_ATCMD_DELAY);
			HAL_UART_Abort_IT(&huart2);

			print_atCommand(recv2, uart2_msg_print_switch);
			memset(recv2, 0, UART2_RX_LENGTH);
    	}
    	else if(wwan_command == 2)
    	{
    		// disable WWAN, airplane mode
    		wwan_command = 0;

    		// Airplane mode
			HAL_UART_Transmit(&huart2, uart_Tx[ATCMD_En_airplane], sizeof(uart_Tx[ATCMD_En_airplane])-1, 30);
			HAL_UART_Transmit(&huart2, "\r", 1, 30);

			if(HAL_UART_Receive_DMA(&huart2, recv2, UART2_RX_LENGTH) != HAL_OK)
			{
				//Error_Handler();
			}
			osDelay(UART2_ATCMD_DELAY);
			HAL_UART_Abort_IT(&huart2);

			print_atCommand(recv2, uart2_msg_print_switch);
			memset(recv2, 0, UART2_RX_LENGTH);
    	}
    	//----------------------------------------


    	// Get the NMEA $RMC messages
		HAL_UART_Transmit(&huart2, uart_Tx[ATCMD_Get_GPS_DATA], sizeof(uart_Tx[ATCMD_Get_GPS_DATA])-1, 30);
		HAL_UART_Transmit(&huart2, "\r", 1, 30);

		if(HAL_UART_Receive_DMA(&huart2, recv2, UART2_RX_LENGTH) != HAL_OK)
		{
			//Error_Handler();
		}
		else
		{
			print_atCommand(recv2, uart2_msg_print_switch);

			if(uart2_isFindString(recv2, string_Not, 3) == 1)
			{
				// Enable GPS AT+UGPS=1
				HAL_UART_Transmit(&huart2, uart_Tx[ATCMD_Enable_GPS], sizeof(uart_Tx[ATCMD_Enable_GPS])-1, 30);
				HAL_UART_Transmit(&huart2, "\r", 1, 30);

				if(HAL_UART_Receive_DMA(&huart2, recv2, UART2_RX_LENGTH) != HAL_OK)
				{
					//Error_Handler();
				}
				osDelay(UART2_ATCMD_DELAY);
				HAL_UART_Abort_IT(&huart2);

				print_atCommand(recv2, uart2_msg_print_switch);
				memset(recv2, 0, UART2_RX_LENGTH);

				osDelay(UART2_ATCMD_DELAY*2);
			}
			else
			{
				// find data index
				// find index of latitude and longitude
				/*
				split_count = 0;
				for(i = 0; i<UART2_RX_LENGTH; i++)
				{
					if(recv2[i] == ',')
					{
						split_count++;
					}

					if(split_count == 4)
					{
						latitude_index = i+1;
					}

					if(split_count == 6)
					{
						longitude_index = i+1;
						break;
					}
				}
				*/
				latitude_index = uart2_findCommaIndexNext(recv2,4);
				longitude_index = uart2_findCommaIndexNext(recv2,6);

				// Get latitude and longitude data
				latitude = 0;   //24
				longitude = 0;  //121
				if((recv2[latitude_index] != ',') && (latitude_index != longitude_index))
				{
					// latitude
					for( i = 0; i < 10; i++)
					{
						if(recv2[latitude_index+i] != '.')
						{
							if(recv2[latitude_index+i] == ',')
							{
								break;
							}
							latitude = 10*latitude + (recv2[latitude_index+i]-'0');
						}
					}

					// longitude
					for( i = 0; i < 11; i++)
					{
						if(recv2[longitude_index+i] != '.')
						{
							if(recv2[longitude_index+i] == ',')
							{
								break;
							}
							longitude = 10*longitude + (recv2[longitude_index+i]-'0');
						}
					}
				}

				//aewin_dbg("\n\rlatitude  index is: %d, value is %u. \r\n", latitude_index, latitude);
				//aewin_dbg("\n\rlongitude index is: %d, value is %u. \r\n", longitude_index, longitude);

				// send data to queue
				if( osMessagePut(UART2_LAT_QHandle, latitude, UART2_TIMEOUT) != osOK )
				{
					//aewin_dbg("\n\rUART2 GPS latitude put Q failed. \r\n");
				}

				if( osMessagePut(UART2_LONG_QHandle, longitude, UART2_TIMEOUT) != osOK )
				{
					//aewin_dbg("\n\rUART2 GPS longitude put Q failed. \r\n");
				}
			}
		}
		osDelay(UART2_ATCMD_DELAY);
		HAL_UART_Abort_IT(&huart2);
		memset(recv2, 0, UART2_RX_LENGTH);
		//----------------------------------------


		// Airplane mode or normal mode AT+CFUN?
		HAL_UART_Transmit(&huart2, uart_Tx[ATCMD_Get_airplane], sizeof(uart_Tx[ATCMD_Get_airplane])-1, 30);
		HAL_UART_Transmit(&huart2, "\r", 1, 30);

		if(HAL_UART_Receive_DMA(&huart2, recv2, UART2_RX_LENGTH) != HAL_OK)
		{
			//Error_Handler();
		}
		osDelay(UART2_ATCMD_DELAY);
		HAL_UART_Abort_IT(&huart2);

		if(recv2[uart_findDataIndex(recv2,1)] == '4')
		{
			r280_module_state.normal_flight_states = 40;
		}
		else //0,1,19
		{
			r280_module_state.normal_flight_states = 10;
		}

		//aewin_dbg("SIM mode is %d (airplane mode is '4')\r\n",  (recv2[uart_findDataIndex(recv2,1)]-'0'));
		print_atCommand(recv2, uart2_msg_print_switch);
		memset(recv2, 0, UART2_RX_LENGTH);
		//----------------------------------------


		// Get 3G,4G status AT+COPS?
		HAL_UART_Transmit(&huart2, uart_Tx[ATCMD_Check_Reg], sizeof(uart_Tx[ATCMD_Check_Reg])-1, 30);
		HAL_UART_Transmit(&huart2, "\r", 1, 30);

		if(HAL_UART_Receive_DMA(&huart2, recv2, UART2_RX_LENGTH) != HAL_OK)
		  {
			//Error_Handler();
		  }
		osDelay(UART2_ATCMD_DELAY*3);
		HAL_UART_Abort_IT(&huart2);

		reg_index = uart2_findCommaIndexNext(recv2,3);

		if(recv2[reg_index] == '7')                  //4G
		{
			r280_module_state.register_3G_4G = 7;
		}
		else if(recv2[reg_index] == '2')             //3G
		{
			r280_module_state.register_3G_4G = 2;
		}
		else if(recv2[reg_index] == '0')             //2G
		{
			r280_module_state.register_3G_4G = 0;
		}

		print_atCommand(recv2, uart2_msg_print_switch);
		memset(recv2, 0, UART2_RX_LENGTH);
		//----------------------------------------


		// 4G signal strength AT+CSQ
		HAL_UART_Transmit(&huart2, uart_Tx[ATCMD_Check_Signal], sizeof(uart_Tx[ATCMD_Check_Signal])-1, 30);
		HAL_UART_Transmit(&huart2, "\r", 1, 30);

		if(HAL_UART_Receive_DMA(&huart2, recv2, UART2_RX_LENGTH) != HAL_OK)
		{
			//Error_Handler();
		}
		osDelay(UART2_ATCMD_DELAY);
		HAL_UART_Abort_IT(&huart2);

		r280_module_state.signal_strength = 0;
		strength_index = uart_findDataIndex(recv2, 1);

		if(strength_index != 0)
		{
			r280_module_state.signal_strength = (recv2[strength_index]-'0');
			if (recv2[strength_index+1] != ',')
			{
				r280_module_state.signal_strength = 10*r280_module_state.signal_strength + (recv2[strength_index+1]-'0');
			}
		}

		//aewin_dbg("signal_sterngth is %d\r\n", r280_module_state.signal_strength);
		print_atCommand(recv2, uart2_msg_print_switch);
		memset(recv2, 0, UART2_RX_LENGTH);
		//----------------------------------------


		// 4G IP Addresss at+cgdcont?
		HAL_UART_Transmit(&huart2, uart_Tx[ATCMD_Check_APNIP], sizeof(uart_Tx[ATCMD_Check_APNIP])-1, 30);
		HAL_UART_Transmit(&huart2, "\r", 1, 30);

		if(HAL_UART_Receive_DMA(&huart2, recv2, UART2_RX_LENGTH) != HAL_OK)
		{
			//Error_Handler();
		}
		osDelay(UART2_ATCMD_DELAY);
		HAL_UART_Abort_IT(&huart2);

		ip_index = uart2_findCommaIndexNext(recv2,3)+1;

		ip_counter = 0;
		r280_module_state.ip_add_0 = 0;
		r280_module_state.ip_add_1 = 0;
		r280_module_state.ip_add_2 = 0;
		r280_module_state.ip_add_3 = 0;

		for( i = ip_index ; i < uart2_findCommaIndexNext(recv2,4)-2 ; i++ )
		{
			if(recv2[i]!='.')
			{
				switch(ip_counter)
				{
					case 0:
						r280_module_state.ip_add_0 = r280_module_state.ip_add_0*10 + (recv2[i]-'0');
						break;

					case 1:
						r280_module_state.ip_add_1 = r280_module_state.ip_add_1*10 + (recv2[i]-'0');
						break;

					case 2:
						r280_module_state.ip_add_2 = r280_module_state.ip_add_2*10 + (recv2[i]-'0');
						break;

					case 3:
						r280_module_state.ip_add_3 = r280_module_state.ip_add_3*10 + (recv2[i]-'0');
						break;

					default:
						break;
				}

			}
			else
			{
				ip_counter++;
			}
		}

		//aewin_dbg("IP is %x.%x.%x.%x\r\n",  r280_module_state.ip_add_0, r280_module_state.ip_add_1, r280_module_state.ip_add_2, r280_module_state.ip_add_3);
		print_atCommand(recv2, uart2_msg_print_switch);
		memset(recv2, 0, UART2_RX_LENGTH);
		//----------------------------------------


		// ping status AT+UPING="www.google.com"
		HAL_UART_Transmit(&huart2, uart_Tx[ATCMD_Ping_web], sizeof(uart_Tx[ATCMD_Ping_web])-1, 30);
		HAL_UART_Transmit(&huart2, "\r", 1, 30);

		if(HAL_UART_Receive_DMA(&huart2, recv2, UART2_RX_LENGTH) != HAL_OK)
		  {
			//Error_Handler();
		  }
		osDelay(UART2_ATCMD_DELAY*5);
		HAL_UART_Abort_IT(&huart2);

		print_atCommand(recv2, uart2_msg_print_switch);


		// check ping status
		if((uart2_isFindString(recv2, string_UUPING, sizeof(string_UUPING)) == 1) && (uart2_isFindString(recv2, string_UUPINGER, sizeof(string_UUPINGER)) == 0))
		{
			r280_module_state.ping_status = 0; //OK
			memset(recv2, 0, UART2_RX_LENGTH);
		}
		else
		{
			r280_module_state.ping_status = 1; //ERROR
			memset(recv2, 0, UART2_RX_LENGTH);

			//reset APN and PSD

			// set APN provider
			HAL_UART_Transmit(&huart2, uart_Tx[ATCMD_Set_APN], sizeof(uart_Tx[ATCMD_Set_APN])-1, 30);
			HAL_UART_Transmit(&huart2, "\r", 1, 30);

			if(HAL_UART_Receive_DMA(&huart2, recv2, UART2_RX_LENGTH) != HAL_OK)
			{
				//Error_Handler();
			}
			osDelay(UART2_ATCMD_DELAY);
			HAL_UART_Abort_IT(&huart2);

			print_atCommand(recv2, uart2_msg_print_switch);
			memset(recv2, 0, UART2_RX_LENGTH);

			// reset PSD
			HAL_UART_Transmit(&huart2, uart_Tx[ATCMD_Reset_PSD], sizeof(uart_Tx[ATCMD_Reset_PSD])-1, 30);
			HAL_UART_Transmit(&huart2, "\r", 1, 30);

			if(HAL_UART_Receive_DMA(&huart2, recv2, UART2_RX_LENGTH) != HAL_OK)
			{
				//Error_Handler();
			}
			osDelay(UART2_ATCMD_DELAY*2);
			HAL_UART_Abort_IT(&huart2);

			print_atCommand(recv2, uart2_msg_print_switch);
			memset(recv2, 0, UART2_RX_LENGTH);

			// activate PSD
			HAL_UART_Transmit(&huart2, uart_Tx[ATCMD_Enable_PSD], sizeof(uart_Tx[ATCMD_Enable_PSD])-1, 30);
			HAL_UART_Transmit(&huart2, "\r", 1, 30);

			if(HAL_UART_Receive_DMA(&huart2, recv2, UART2_RX_LENGTH) != HAL_OK)
			{
				//Error_Handler();
			}
			osDelay(UART2_ATCMD_DELAY*2);
			HAL_UART_Abort_IT(&huart2);

			print_atCommand(recv2, uart2_msg_print_switch);
			memset(recv2, 0, UART2_RX_LENGTH);

			// ublox FAE ======================================================
			HAL_UART_Transmit(&huart2, "AT+COPS=2", sizeof("AT+COPS=2")-1, 30);
			HAL_UART_Transmit(&huart2, "\r", 1, 30);

			if(HAL_UART_Receive_DMA(&huart2, recv2, UART2_RX_LENGTH) != HAL_OK)
			{
				//Error_Handler();
			}
			osDelay(UART2_ATCMD_DELAY);
			HAL_UART_Abort_IT(&huart2);

			HAL_UART_Transmit(&huart2, "at+cgdcont=4,\"ip\",\"internet\"", sizeof("at+cgdcont=4,\"ip\",\"internet\"")-1, 30);
			HAL_UART_Transmit(&huart2, "\r", 1, 30);

			if(HAL_UART_Receive_DMA(&huart2, recv2, UART2_RX_LENGTH) != HAL_OK)
			{
				//Error_Handler();
			}
			osDelay(UART2_ATCMD_DELAY);
			HAL_UART_Abort_IT(&huart2);


			HAL_UART_Transmit(&huart2, "AT+COPS=0", sizeof("AT+COPS=0")-1, 30);
			HAL_UART_Transmit(&huart2, "\r", 1, 30);

			if(HAL_UART_Receive_DMA(&huart2, recv2, UART2_RX_LENGTH) != HAL_OK)
			{
				//Error_Handler();
			}
			osDelay(UART2_ATCMD_DELAY);
			HAL_UART_Abort_IT(&huart2);


			HAL_UART_Transmit(&huart2, "AT+CGACT=1,4", sizeof("AT+CGACT=1,4")-1, 30);
			HAL_UART_Transmit(&huart2, "\r", 1, 30);

			if(HAL_UART_Receive_DMA(&huart2, recv2, UART2_RX_LENGTH) != HAL_OK)
			{
				//Error_Handler();
			}
			osDelay(UART2_ATCMD_DELAY);
			HAL_UART_Abort_IT(&huart2);

			print_atCommand(recv2, uart2_msg_print_switch);
			memset(recv2, 0, UART2_RX_LENGTH);

			HAL_UART_Transmit(&huart2, "at+upsd=0,100,4", sizeof("at+upsd=0,100,4")-1, 30);
			HAL_UART_Transmit(&huart2, "\r", 1, 30);

			if(HAL_UART_Receive_DMA(&huart2, recv2, UART2_RX_LENGTH) != HAL_OK)
			{
				//Error_Handler();
			}
			osDelay(UART2_ATCMD_DELAY);
			HAL_UART_Abort_IT(&huart2);

			print_atCommand(recv2, uart2_msg_print_switch);
			memset(recv2, 0, UART2_RX_LENGTH);

			HAL_UART_Transmit(&huart2, "at+upsda=0,3", sizeof("at+upsda=0,3")-1, 30);
			HAL_UART_Transmit(&huart2, "\r", 1, 30);

			if(HAL_UART_Receive_DMA(&huart2, recv2, UART2_RX_LENGTH) != HAL_OK)
			{
				//Error_Handler();
			}
			osDelay(UART2_ATCMD_DELAY);
			HAL_UART_Abort_IT(&huart2);

			print_atCommand(recv2, uart2_msg_print_switch);
			memset(recv2, 0, UART2_RX_LENGTH);
		}



		//----------------------------------------

		//print_atCommand(recv2, uart2_msg_print_switch);


#if 0
		if((u2_states = HAL_UART_Receive(&huart2, (uint8_t*)&recv2, 1, UART3_TIMEOUT)) == HAL_OK){
			HAL_UART_Transmit(&huart2, (uint8_t*)&recv2, 1, UART3_TIMEOUT);
		}
		else if (u2_states == HAL_TIMEOUT){
			HAL_UART_Abort(&huart2);
		}
#endif
		osDelay(UART2_TASK_ENTRY_TIME);
    }
  /* USER CODE END usart2_entry */
}

/* usart3_entry function */
void usart3_entry(void const * argument)
{
  /* USER CODE BEGIN usart3_entry */
	uint8_t recv3[4] = {0};
	int temp;
	uint8_t *cml_head, *cml_limit, *cml_ptr = cml_array;
	HAL_StatusTypeDef u3_states = HAL_OK;
	cml_head = cml_limit = cml_ptr;
	uint8_t cnt = 0;
	/* Infinite loop */
	for(;;)
	{
#if (0)
		//recv3[0] = recv3[1] = recv3[2] = 0;
		osMutexWait(MUTEX_DebugHandle, osWaitForever);
		if(HAL_UART_Receive(&huart3, (uint8_t*)&recv3[0], 1, 1) == HAL_OK){
			if (HAL_UART_Receive(&huart3, (uint8_t*)&recv3[1], 1, 1) != HAL_OK){
				switch(recv3[0]){
					case '\0':
						break;
					//-----------------------------------------------------
					case '\r':
					case '\n':
						HAL_UART_Transmit(&huart3, SVA_1000_NL, sizeof(SVA_1000_NL) - 1, sizeof(SVA_1000_NL) - 1);
						*cml_limit = '\0';
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
							HAL_UART_Transmit(&huart3, (uint8_t*)&recv3[0], 1, 1);
							HAL_UART_Transmit(&huart3, cml_ptr, temp, temp);
							HAL_UART_Transmit(&huart3, " ", 1, 1);
							//HAL_UART_Transmit(&huart3, (uint8_t*)&recv3[0], 1, UART3_TIMEOUT);


							for(temp = 0; temp < cml_limit - cml_ptr; temp++){
								HAL_UART_Transmit(&huart3, (uint8_t*)&recv3[0], 1, 1);
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
						//cml_limit = cml_ptr;
						if(cml_limit < cml_ptr){
							cml_limit = cml_ptr;
						}
						HAL_UART_Transmit(&huart3, (uint8_t*)&recv3[0], 1, 1);
						break;
				}

			}
			else{
				if (HAL_UART_Receive(&huart3, (uint8_t*)&recv3[2], 1, 1) == HAL_OK){
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
								HAL_UART_Transmit(&huart3, (uint8_t*)&recv3[0], 3, 2);
							}

							break;

						//-----------------------------------------------------
						// Left
						case 'D':
							if (cml_ptr > cml_head){
								cml_ptr--;
								HAL_UART_Transmit(&huart3, (uint8_t*)&recv3[0], 3, 2);
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


		osMutexRelease(MUTEX_DebugHandle);
#else

		//HAL_DMA_Abort(&hdma_usart3_tx);
		//HAL_DMA_Abort(&hdma_usart3_rx);

		//HAL_Delay(1);
		osMutexWait(MUTEX_DebugHandle, osWaitForever);
		cnt = 0;
		if(HAL_UART_Receive_DMA(&huart3, &recv3[cnt], 1) == HAL_OK){
			cnt++;
			//HAL_Delay(1);
			while(HAL_UART_Receive_DMA(&huart3, &recv3[cnt], 1) == HAL_OK){
				cnt++;
				if(cnt > 3) break;
			}

			osDelay(1);
			//HAL_Delay(2);
			// Abort UART processing.

		}
		//HAL_UART_Abort_IT(&huart3);




		//}
		/*if (cnt > 0){
			HAL_UART_Transmit_DMA(&huart3, &recv3[0], cnt);
		}*/

		if(cnt == 3){
			if (recv3[0] == '\e'){
				//temp = recv3[0];
				//recv3[0] = '\e';
				//recv3[1] = '[';
				//recv3[2] = (uint8_t)temp;
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
						HAL_UART_Transmit_DMA(&huart3, recv3, 3);
					}

					break;

				//-----------------------------------------------------
				// Left
				case 'D':
					if (cml_ptr > cml_head){
						cml_ptr--;
						HAL_UART_Transmit_DMA(&huart3, recv3, 3);
					}
					break;

				//-----------------------------------------------------
				default:
					break;

				}
			}
		}
		else if(cnt == 1){
			switch(recv3[0]){
			case '\0':
				break;
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
				break;

			case '\b':
				if ((cml_ptr > cml_head) && (cml_limit > cml_head)){
					temp = cml_limit - cml_ptr;
					*(--cml_ptr) = 0;
					strncpy(cml_ptr, (cml_ptr + 1), temp);
					HAL_Delay(1);
					HAL_UART_Transmit_DMA(&huart3, recv3, 1);
					HAL_Delay(2);
					HAL_UART_Transmit_DMA(&huart3, cml_ptr, temp);
					HAL_Delay(2);
					HAL_UART_Transmit_DMA(&huart3, " ", 1);
					//HAL_UART_Transmit_DMA(&huart3, (uint8_t*)&recv3[0], 1, UART3_TIMEOUT);


					for(temp = 0; temp < cml_limit - cml_ptr; temp++){
						HAL_Delay(1);
						HAL_UART_Transmit_DMA(&huart3, recv3, 1);
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
				if (cml_ptr > cml_limit) cml_limit = cml_ptr;
				HAL_UART_Transmit_DMA(&huart3, recv3, 1);
				break;
			}
		}


		memset(recv3, 0, 4);
		osMutexRelease(MUTEX_DebugHandle);

#endif
		osDelay(UART3_TASK_ENTRY_TIME);
	}

  /* USER CODE END usart3_entry */
}

/* ignition_entry function */
void ignition_entry(void const * argument)
{
  /* USER CODE BEGIN ignition_entry */
	osEvent  	evt;
	uint32_t 	adc_24v = 0;
	uint32_t    adc_temp = 0;

	//ig_event = gIG_Event;

	aewin_dbg("\n\rInitialize finished.\n\r");
	aewin_dbg("\n\rReset status : 0x%x.\n\r", RCC->CSR);
	RCC->CSR |= 0x01000000;
	aewin_dbg("\n\rClear reset status : 0x%x.\n\r", RCC->CSR);
    /* Infinite loop */
    for(;;)
    {

		//HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
		//HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);
		//aewin_dbg("\n\rMUC Time: %2d:%2d:%2d",sTime.Hours ,sTime.Minutes, sTime.Seconds);
		//aewin_dbg("\n\rMUC Date: 20%2d_%2d_%2d  Weekday:%d",sDate.Year ,sDate.Month, sDate.Date ,sDate.WeekDay);

    	/** Get ADC 24V from Q. */
    	evt = osMessageGet(ADC_VOLT_QHandle, osWaitForever);
		if (evt.status == osEventMessage){
			adc_24v = evt.value.v;
			check_highPower(adc_24v);
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


		//switch(ig_event.IG_States){
		//switch(current_IgEvent[NUM_ig_states]){
		switch(ig_var.ig_states){
			case IG_Recovery:
				// Recovery all of IG states and parameters.
				//ig_event = gIG_Event;
				//config_recovery();

				//countdown_timer initialize
				countdown_timer.poweron_delay    = current_IgEvent[NUM_pwron_delay];
				countdown_timer.startup_timeout  = current_IgEvent[NUM_startup_timeout];
				countdown_timer.shutdown_delay   = current_IgEvent[NUM_shutdown_delay];
				countdown_timer.shutdown_timeout = current_IgEvent[NUM_shutdown_timeout];
				countdown_timer.poweroff_delay   = current_IgEvent[NUM_pwroff_delay];

				//ig_event.IG_States = IG_CloseUp;
				//current_IgEvent[NUM_ig_states] = IG_CloseUp;
				ig_var.ig_states = IG_CloseUp;
				break;

			//-------------------------------------------------------------------------------------
			case IG_CloseUp:
				// Judge the IG switch states.
				if(sva_gpi.ig_sw == GPIO_PIN_SET){
					/* IG On process */
					// Ready to enter IG_PowerOn_Delay state.
					//ig_event.IG_States = IG_PowerOn_Delay;
					//current_IgEvent[NUM_ig_states] = IG_PowerOn_Delay;
					ig_var.ig_states = IG_PowerOn_Delay;


					// Reset the "Start up timeout"
					//ig_event.startup_timeout = gIG_Event.startup_timeout;
					//current_IgEvent[NUM_startup_timeout] = flash_IgEvent[NUM_startup_timeout];
					countdown_timer.startup_timeout = current_IgEvent[NUM_startup_timeout];
					aewin_dbg("\n\rIgnition ON! IG_CloseUp --> IG_PowerOn_Delay");
				}

				// Judge the power button states.
				if(sva_gpi.pwr_btn == GPIO_PIN_RESET){
					//ig_event.startup_timeout = gIG_Event.startup_timeout;
					//current_IgEvent[NUM_startup_timeout] = flash_IgEvent[NUM_startup_timeout];
					countdown_timer.startup_timeout = current_IgEvent[NUM_startup_timeout];
					//ig_event.IG_States = IG_Wait_StartUp;
					//current_IgEvent[NUM_ig_states] = IG_Wait_StartUp;
					ig_var.ig_states = IG_Wait_StartUp;
					aewin_dbg("\n\rPower button On! IG_CloseUp --> IG_Wait_StartUp");
				}
				break;

			//-------------------------------------------------------------------------------------
			case IG_PowerOn_Delay:
				// Judge the IG switch states.
				if(sva_gpi.ig_sw == GPIO_PIN_SET){

					if(current_IgEvent[NUM_delay_on_setting] == 0)
					{
						countdown_timer.poweron_delay = 0;
					}

					//if (0 == (ig_event.pwron_delay--)){
					//if (0 == (current_IgEvent[NUM_pwron_delay]--)){
					if (0 == (countdown_timer.poweron_delay--)){
						//ig_event.pwron_delay = gIG_Event.pwron_delay;
						//current_IgEvent[NUM_pwron_delay] = flash_IgEvent[NUM_pwron_delay];
						countdown_timer.poweron_delay = current_IgEvent[NUM_pwron_delay];
						//ig_event.IG_States = IG_Wait_StartUp;
						//current_IgEvent[NUM_ig_states] = IG_Wait_StartUp;
						ig_var.ig_states = IG_Wait_StartUp;
						aewin_dbg("\n\rPower on delay pass! IG_PowerOn_Delay --> IG_Wait_StartUp");
					}
					else
					{
						aewin_dbg("\n\rpwron_delay_countdown:%d",countdown_timer.poweron_delay);
					}
				}
				else{
					//ig_event.pwron_delay = gIG_Event.pwron_delay;
					//current_IgEvent[NUM_pwron_delay] = flash_IgEvent[NUM_pwron_delay];
					countdown_timer.poweron_delay = current_IgEvent[NUM_pwron_delay];
					//ig_event.IG_States = IG_CloseUp;
					//current_IgEvent[NUM_ig_states] = IG_CloseUp;
					ig_var.ig_states = IG_CloseUp;
					aewin_dbg("\n\rPower on delay failed! IG_PowerOn_Delay --> IG_CloseUp");
				}
				break;

			//-------------------------------------------------------------------------------------
			case IG_Wait_StartUp:
				/* Power button has been (long) pressed. */
				if(sva_gpi.pwr_btn == GPIO_PIN_RESET){
					//ig_event.pwrbtn_pressed = TRUE;
					//current_IgEvent[NUM_pwrbtn_pressed] = TRUE;
					ig_var.pwrbtn_pressed = TRUE;
					//if(0 == (ig_event.pwroff_btn_cnt--)){
					//if(0 == (current_IgEvent[NUM_pwroff_btn_cnt]--)){
					if(0 == (countdown_timer.pwroff_btn_cnt--)){
						//ig_event.IG_States = IG_Recovery;
						//current_IgEvent[NUM_ig_states] = IG_Recovery;
						ig_var.ig_states = IG_Recovery;
						HAL_GPIO_WritePin(GPIOC, D2D_EN_Pin, GPIO_PIN_RESET);
						enable_4GModule();
						aewin_dbg("\n\rIG_Wait_StartUp failed! IG_Wait_StartUp --> IG_Recovery");
					}
				}

				//if(ig_event.pwrbtn_pressed && (sva_gpi.pwr_btn == GPIO_PIN_SET)){
				//if(current_IgEvent[NUM_pwrbtn_pressed] && (sva_gpi.pwr_btn == GPIO_PIN_SET)){
				if(ig_var.pwrbtn_pressed && (sva_gpi.pwr_btn == GPIO_PIN_SET)){
					//ig_event.pwroff_btn_cnt = gIG_Event.pwroff_btn_cnt;
					//current_IgEvent[NUM_pwroff_btn_cnt] = flash_IgEvent[NUM_pwroff_btn_cnt];
					countdown_timer.pwroff_btn_cnt = current_IgEvent[NUM_pwroff_btn_cnt];
					//ig_event.pwrbtn_pressed = FALSE;
					//current_IgEvent[NUM_pwrbtn_pressed] = FALSE;
					ig_var.pwrbtn_pressed = FALSE;
				}

				if(sva_gpi.ig_sw == GPIO_PIN_SET){
					//if (0 == (ig_event.wait_startup_time--)){
					//if (0 == (current_IgEvent[NUM_wait_startup_time]--)){
					if (0 == (countdown_timer.wait_startup_time--)){
						//ig_event.wait_startup_time = gIG_Event.wait_startup_time;
						//current_IgEvent[NUM_wait_startup_time] = flash_IgEvent[NUM_wait_startup_time];
						countdown_timer.wait_startup_time = current_IgEvent[NUM_wait_startup_time];
						HAL_GPIO_WritePin(GPIOC, D2D_EN_Pin, GPIO_PIN_SET);
						enable_4GModule();
						// Confirm the the DC2DC power and system power are available.
						while((sva_gpi.dc2dc_pwrok != GPIO_PIN_SET) || (sva_gpi.sys_pwron == GPIO_PIN_SET)){
							//if(0 == (ig_event.pwrgood_chk_time--)){
							//if(0 == (current_IgEvent[NUM_pwrgood_chk_time]--)){
							if(0 == (countdown_timer.pwrgood_chk_time--)){
								break;
							}
						}
						//if(0 != ig_event.pwrgood_chk_time){
						//if(0 != current_IgEvent[NUM_pwrgood_chk_time]){
						if(0 != countdown_timer.pwrgood_chk_time){
							//ig_event.IG_States = IG_Start_Up;
							//current_IgEvent[NUM_ig_states] = IG_Start_Up;
							ig_var.ig_states = IG_Start_Up;
							aewin_dbg("\n\rPower on delay ok! IG_Wait_StartUp --> IG_Start_Up");
						}
						else{
							HAL_GPIO_WritePin(GPIOC, D2D_EN_Pin, GPIO_PIN_RESET);
							enable_4GModule();
							//ig_event.IG_States = IG_CloseUp;
							//current_IgEvent[NUM_ig_states] = IG_CloseUp;
							ig_var.ig_states = IG_CloseUp;
							aewin_dbg("\n\rDE2DC Power on failed! IG_Wait_StartUp --> IG_CloseUp");
						}
						//ig_event.pwrgood_chk_time = gIG_Event.pwrgood_chk_time;
						//current_IgEvent[NUM_pwrgood_chk_time] = flash_IgEvent[NUM_pwrgood_chk_time];
						countdown_timer.pwrgood_chk_time = current_IgEvent[NUM_pwrgood_chk_time];
					}
				}
				else{
					//ig_event.wait_startup_time = gIG_Event.wait_startup_time;
					//current_IgEvent[NUM_wait_startup_time] = flash_IgEvent[NUM_wait_startup_time];
					countdown_timer.wait_startup_time = current_IgEvent[NUM_wait_startup_time];
					//ig_event.IG_States = IG_PowerOn_Delay;
					//current_IgEvent[NUM_ig_states] = IG_PowerOn_Delay;
					ig_var.ig_states = IG_PowerOn_Delay;
					aewin_dbg("\n\rIG_Wait_StartUp failed! IG_Wait_StartUp --> IG_PowerOn_Delay");
				}

				//check startup battery
				if(check_highPower(adc_24v) == 0)
				{
					//ig_var.fail_count++;
					//ig_var.ig_states = IG_Recovery;
				}
				break;

			//-------------------------------------------------------------------------------------
			case IG_Start_Up:
				//if(ig_event.startup_timeout > 0){
				//if(current_IgEvent[NUM_startup_timeout] > 0){
				if(countdown_timer.startup_timeout > 0){
					/* Lock ignition off to prevent that user shutdowns the OS. */
					//ig_event.startup_timeout--;
					//current_IgEvent[NUM_startup_timeout]--;
					countdown_timer.startup_timeout--;
					aewin_dbg("\n\rLock power on!");
				}
				else{
					if( sva_gpi.ig_sw ==  GPIO_PIN_RESET){
						//ig_event.IG_States = IG_Shutdown_Delay;
						//current_IgEvent[NUM_ig_states] = IG_Shutdown_Delay;
						ig_var.ig_states = IG_Shutdown_Delay;
						aewin_dbg("\n\rIngition switch off! IG_Start_Up --> IG_Shutdown_Delay");
					}
				}

				if(sva_gpi.pwr_btn == GPIO_PIN_RESET){
					//ig_event.pwrbtn_pressed = TRUE;
					//current_IgEvent[NUM_pwrbtn_pressed] = TRUE;
					ig_var.pwrbtn_pressed = TRUE;
					//if(0 == (ig_event.pwroff_btn_cnt--)){
					//if(0 == (current_IgEvent[NUM_pwroff_btn_cnt]--)){
					if(0 == (countdown_timer.pwroff_btn_cnt--)){
						HAL_GPIO_WritePin(GPIOC, D2D_EN_Pin, GPIO_PIN_RESET);
						enable_4GModule();
						//ig_event.IG_States = IG_Recovery;
						//current_IgEvent[NUM_ig_states] = IG_Recovery;
						ig_var.ig_states = IG_Recovery;
					}
				}

				//if(ig_event.pwrbtn_pressed && (sva_gpi.pwr_btn == GPIO_PIN_SET)){
				//if(current_IgEvent[NUM_pwrbtn_pressed] && (sva_gpi.pwr_btn == GPIO_PIN_SET)){
				if(ig_var.pwrbtn_pressed && (sva_gpi.pwr_btn == GPIO_PIN_SET)){
					//ig_event.pwroff_btn_cnt = gIG_Event.pwroff_btn_cnt;
					//current_IgEvent[NUM_pwroff_btn_cnt] = flash_IgEvent[NUM_pwroff_btn_cnt];
					countdown_timer.pwroff_btn_cnt = current_IgEvent[NUM_pwroff_btn_cnt];
					//ig_event.pwrbtn_pressed = FALSE;
					//current_IgEvent[NUM_pwrbtn_pressed] = FALSE;
					ig_var.pwrbtn_pressed = FALSE;
					//ig_event.IG_States = IG_shutting_Down;
					//current_IgEvent[NUM_ig_states] = IG_shutting_Down;
					ig_var.ig_states = IG_shutting_Down;
					aewin_dbg("\n\rPower button off! IG_Start_Up --> IG_shutting_Down");
				}

				if(check_lowPower(adc_24v) == 1)
				{
					//current_IgEvent[NUM_ig_states] = IG_LowPower_Delay;
					ig_var.ig_states = IG_LowPower_Delay;
					aewin_dbg("\n\rPower off delay failed! IG_Shutdown_Delay --> IG_LowPower_Delay");
				}

				break;

			//-------------------------------------------------------------------------------------
			case IG_Shutdown_Delay:
				if(sva_gpi.ig_sw == GPIO_PIN_RESET){

					if(current_IgEvent[NUM_delay_off_setting] == 0)
					{
						countdown_timer.poweroff_delay = 0;
					}

					//if (0 == (ig_event.pwroff_delay--)){
					//if (0 == (current_IgEvent[NUM_pwroff_delay]--)){
					if (0 == (countdown_timer.poweroff_delay--)){
						//ig_event.IG_States = IG_shutting_Down;
						//current_IgEvent[NUM_ig_states] = IG_shutting_Down;
						ig_var.ig_states = IG_shutting_Down;
						//ig_event.pwroff_delay = gIG_Event.pwroff_delay;
						//current_IgEvent[NUM_pwroff_delay] = flash_IgEvent[NUM_pwroff_delay];
						countdown_timer.poweroff_delay = current_IgEvent[NUM_pwroff_delay];
						aewin_dbg("\n\rPower off delay pass! IG_Shutdown_Delay --> IG_shutting_Down");
					}
					else
					{
						aewin_dbg("\n\rpwroff_delay_countdown:%d",countdown_timer.poweroff_delay);
					}
				}
				else{
					//ig_event.IG_States = IG_Start_Up;
					//current_IgEvent[NUM_ig_states] = IG_Start_Up;
					ig_var.ig_states = IG_Start_Up;
					//ig_event.pwroff_delay = gIG_Event.pwroff_delay;
					current_IgEvent[NUM_pwroff_delay] = flash_IgEvent[NUM_pwroff_delay];
					aewin_dbg("\n\rPower off delay failed! IG_Shutdown_Delay --> IG_Start_Up");
				}

				if(check_lowPower(adc_24v) == 1)
				{
					//current_IgEvent[NUM_ig_states] = IG_LowPower_Delay;
					ig_var.ig_states = IG_LowPower_Delay;
					//current_IgEvent[NUM_pwroff_delay] = flash_IgEvent[NUM_pwroff_delay];
					countdown_timer.poweroff_delay = current_IgEvent[NUM_pwroff_delay];
					aewin_dbg("\n\rPower off delay failed! IG_Shutdown_Delay --> IG_LowPower_Delay");
				}

				break;

			//-------------------------------------------------------------------------------------
			case IG_shutting_Down:
				if(sva_gpi.ig_sw == GPIO_PIN_RESET){
					//if (0 == (ig_event.shutdown_delay--)){
					//if (0 == (current_IgEvent[NUM_shutdown_delay]--)){
					if (0 == (countdown_timer.shutdown_delay--)){
						//ig_event.IG_States = IG_CloseUp;
						//current_IgEvent[NUM_ig_states] = IG_CloseUp;
						ig_var.ig_states = IG_CloseUp;
						//ig_event.shutdown_delay = gIG_Event.shutdown_delay;
						//current_IgEvent[NUM_shutdown_delay] = flash_IgEvent[NUM_shutdown_delay];
						countdown_timer.shutdown_delay = current_IgEvent[NUM_shutdown_delay];
						HAL_GPIO_WritePin(GPIOC, D2D_EN_Pin, GPIO_PIN_RESET);
						enable_4GModule();
						aewin_dbg("\n\rShutdown delay pass! IG_shutting_Down --> IG_CloseUp");
					}
				}
				else{
					//ig_event.IG_States = IG_Shutdown_Delay;
					//current_IgEvent[NUM_ig_states] = IG_Shutdown_Delay;
					ig_var.ig_states = IG_Shutdown_Delay;
					//ig_event.shutdown_delay = gIG_Event.shutdown_delay;
					//current_IgEvent[NUM_shutdown_delay] = flash_IgEvent[NUM_shutdown_delay];
					countdown_timer.shutdown_delay = current_IgEvent[NUM_shutdown_delay];
					aewin_dbg("\n\rShutdown delay failed! IG_shutting_Down --> IG_Shutdown_Delay");
				}
				break;

			//-------------------------------------------------------------------------------------
			case IG_LowPower_Delay:
				//if (0 == (current_IgEvent[NUM_lowpwr_dealy]--)){
				if (0 == (countdown_timer.lowpower_delay--)){
					//current_IgEvent[NUM_ig_states] = IG_CloseUp;
					ig_var.ig_states = IG_CloseUp;
					//current_IgEvent[NUM_lowpwr_dealy] = flash_IgEvent[NUM_lowpwr_dealy];
					countdown_timer.lowpower_delay = current_IgEvent[NUM_lowpwr_dealy];
					HAL_GPIO_WritePin(GPIOC, D2D_EN_Pin, GPIO_PIN_RESET);
					enable_4GModule();
					aewin_dbg("\n\rLowPower delay pass! IG_LowPower_Delay --> IG_CloseUp");
				}
				else
				{
					//Unknown
				}
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

	sI2C1_DATA i2c1_data;
	/* Buffer used for reception */
	uint8_t ADXL345_RxBuffer[6];
	uint8_t ADXL345_DEVID[1];
	//uint8_t ASXL345_sensorSetting[1];
	uint8_t regptr[] = {I2C_ADXL345_CMD_DEV_ID, I2C_ADXL345_CMD_PWR_CTL, I2C_ADXL345_DATA_MEASURE, I2C_ADXL345_CMD_GPS_DATA};

	//read ADXL345 device ID (0xE5)
	//HAL_I2C_Master_Transmit(&hi2c1, I2C_ADXL345_DEV_ADDRESS, &regptr[0], 1, 1000);
	//HAL_I2C_Master_Receive(&hi2c1, I2C_ADXL345_DEV_ADDRESS, (uint8_t *)ADXL345_DEVID, 1, 10000);

	//enable ADXL345 x,y,z-axis G-sensor
	//HAL_I2C_Master_Transmit(&hi2c1, I2C_ADXL345_DEV_ADDRESS, &regptr[1], 2, 1000);

	//Check G-sensor setting
	//HAL_I2C_Master_Transmit(&hi2c1, I2C_ADXL345_DEV_ADDRESS, &regptr[1], 1, 1000);
	//HAL_I2C_Master_Receive(&hi2c1, I2C_ADXL345_DEV_ADDRESS, (uint8_t *)ASXL345_sensorSetting, 1, 10000);

    /* Infinite loop */
	for(;;)
	{
		//HAL_SMBUS_Master_Transmit_IT(&hsmbus1, I2C_DEV_ADDRESS_ADXL345, &regptr, 1, SMBUS_AUTOEND_MODE);
		//HAL_Delay(10);
		//HAL_SMBUS_Master_Receive_IT(&hsmbus1, I2C_DEV_ADDRESS_ADXL345,  ADXL345_RxBuffer, 6, SMBUS_AUTOEND_MODE);

		//HAL_GPIO_WritePin(GPIOC, D2D_EN_Pin, GPIO_PIN_SET);
		if((sva_gpi.dc2dc_pwrok == GPIO_PIN_SET) && (sva_gpi.sys_pwron == GPIO_PIN_SET)){
			//HAL_I2C_Master_Transmit(&hi2c1, I2C_ADXL345_DEV_ADDRESS, (uint8_t*)regptr, sizeof(regptr), 1000);
			//HAL_I2C_Master_Receive(&hi2c1, I2C_ADXL345_DEV_ADDRESS, (uint8_t *)ADXL345_DEVID, 1, 10000);

			//read ADXL345 device ID (0xE5)
			HAL_I2C_Master_Transmit(&hi2c1, I2C_ADXL345_DEV_ADDRESS, &regptr[0], 1, 1000);
			HAL_I2C_Master_Receive(&hi2c1, I2C_ADXL345_DEV_ADDRESS, (uint8_t *)ADXL345_DEVID, 1, 10000);

			//enable ADXL345 measure
			HAL_I2C_Master_Transmit(&hi2c1, I2C_ADXL345_DEV_ADDRESS, &regptr[1], 2, 1000);

			HAL_I2C_Master_Transmit(&hi2c1, I2C_ADXL345_DEV_ADDRESS, &regptr[3], 1, 1000);
			HAL_I2C_Master_Receive(&hi2c1, I2C_ADXL345_DEV_ADDRESS, (uint8_t *)ADXL345_RxBuffer, 6, 10000);

			i2c1_data.x_axis = ((ADXL345_RxBuffer[1]<<8)|(ADXL345_RxBuffer[0]));
			i2c1_data.y_axis = ((ADXL345_RxBuffer[3]<<8)|(ADXL345_RxBuffer[2]));
			i2c1_data.z_axis = ((ADXL345_RxBuffer[5]<<8)|(ADXL345_RxBuffer[4]));

			if( osMessagePut(I2C1_XAXIS_QHandle, i2c1_data.x_axis, osWaitForever) != osOK )
			{
				aewin_dbg("\n\rI2C1 G-sensor x-axis put Q failed. \r\n");
			}

			if( osMessagePut(I2C1_YAXIS_QHandle, i2c1_data.y_axis, osWaitForever) != osOK )
			{
				aewin_dbg("\n\rI2C1 G-sensor y-axis put Q failed. \r\n");
			}

			if( osMessagePut(I2C1_ZAXIS_QHandle, i2c1_data.z_axis, osWaitForever) != osOK )
			{
				aewin_dbg("\n\rI2C1 G-sensor put z-axis Q failed. \r\n");
			}
		}



#if 0
		/* While the I2C in reception process, user can transmit data through
			 "aTxBuffer" buffer */
		  /* Timeout is set to 10S */
		  while(HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)0xE5, &regptr, 1, 10)!= HAL_OK)
		  {
			/* Error_Handler() function is called when Timeout error occurs.
			   When Acknowledge failure occurs (Slave don't acknowledge its address)
			   Master restarts communication */
			if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
			{
			  Error_Handler();
			}
		  }

		  while(HAL_I2C_Master_Receive(&hi2c1, (uint16_t)0xE5, (uint8_t *)ADXL345_RxBuffer, 6, 10000) != HAL_OK)
		  {
			  /* Error_Handler() function is called when Timeout error occurs.
				 When Acknowledge failure occurs (Slave don't acknowledge it's address)
				 Master restarts communication */
			  if (HAL_I2C_GetError(&hi2c1) != HAL_I2C_ERROR_AF)
			  {
				Error_Handler();
			  }
		   }


		  //if(HAL_I2C_Master_Receive_DMA(&I2cHandle, (uint16_t)I2C_DEV_ADDRESS_ADXL345, (uint8_t *)ADXL345_RxBuffer, BUFFER_SIZE_ADXL345) != HAL_OK)
		if(HAL_I2C_Mem_Read_DMA( &I2cHandle,  0xE5,  0x32, I2C_MEMADD_SIZE_8BIT,  (uint8_t *)ADXL345_RxBuffer,  6) != HAL_OK)
		{
		  /* Error_Handler() function is called when error occurs. */
		  Error_Handler();
		}

		/* Wait for the end of the transfer */
		while (HAL_I2C_GetState(&I2cHandle) != HAL_I2C_STATE_READY)
		{
		}

	#endif


		//aewin_dbg("\n\rI2C Task");
        osDelay(I2C1_TASK_ENTRY_TIME);
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
		osDelay(ADC_TASK_ENTRY_TIME);

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

	  HAL_GPIO_WritePin(DEBUG_GPO_GPIO_Port, DEBUG_GPO_Pin, GPIO_PIN_SET);
	  HAL_Delay(1);
	  HAL_GPIO_WritePin(DEBUG_GPO_GPIO_Port, DEBUG_GPO_Pin, GPIO_PIN_RESET);

	  HAL_IWDG_Refresh(&hiwdg);
	  osDelay(IWDG_TASK_ENTRY_TIME);
	  //osDelay(current_IgEvent[NUM_wtdog_default]*1000);
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
	  HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BIN);
	  HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BIN);

	  HAL_RTC_GetAlarm(&hrtc, &sAlarm, RTC_ALARM_A, RTC_FORMAT_BIN);

	  //aewin_dbg("\n\rGet RTC alarm time: %d : %d : %d", sAlarm.AlarmTime.Hours,sAlarm.AlarmTime.Minutes, sAlarm.AlarmTime.Seconds);
	  //aewin_dbg("\n\rGet Time: %2d:%2d:%2d",sTime.Hours ,sTime.Minutes, sTime.Seconds);
	  //aewin_dbg("\n\r=============================",sTime.Hours ,sTime.Minutes, sTime.Seconds);
	  osDelay(1000);


	  //osDelay(1000);
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

		/* Get ebtn_in states. */
		sva_gpi.ebtn_in 	= HAL_GPIO_ReadPin(GPIOC, EBTN_IN_Pin);

		osDelay(GPIO_GET_TASK_TIME);
    }
  /* USER CODE END gpio_state_entry */
}

/* spi1_entry function */
void spi1_entry(void const * argument)
{
  /* USER CODE BEGIN spi1_entry */
	uint8_t loop_index = 0;
	uint8_t cmd_rdsr[] = {0x05};
	uint8_t cmd_devID[] = {0x90, 0x00, 0x00, 0X01};
	uint8_t aTxBuffer0[] = {0x06};
	uint8_t aTxBuffer1[] = {0x20, 0x00, 0x00, 0x00};                           //erase
	uint8_t aTxBuffer2[] = {0x02, 0x00, 0x00, 0x00, 0x55, 0x5A, 0xA5, 0xAA};   //program
	uint8_t aTxBuffer3[] = {0x03, 0x00, 0x00, 0x00};                           //read
	uint8_t cmd_erase[] = {0xC7};

	/* Buffer used for reception */
	uint8_t aRxBuffer[6];
	uint8_t data_rdsr[1];
	uint8_t data_devID[2];
	uint8_t flash_RxBuffer[NUM_total];

	//read deviceID
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)cmd_devID, sizeof(cmd_devID), 5000);

	HAL_SPI_Receive(&hspi1, (uint8_t*)data_devID, 2, 5000);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	//if flash tag not correct, program default data into flash
	//Read flash data
	//RDSR
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)cmd_rdsr, sizeof(cmd_rdsr), 5000);

	HAL_SPI_Receive(&hspi1, (uint8_t*)data_rdsr, 1, 5000);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	//Check WEL
	/*
	while((data_rdsr[0] & (1<<1)) != 0)
	{
		//RDSR
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, (uint8_t*)cmd_rdsr, sizeof(cmd_rdsr), 5000);

		HAL_SPI_Receive(&hspi1, (uint8_t*)data_rdsr, 1, 5000);
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	}
	*/

	//Read
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, (uint8_t*)aTxBuffer3, sizeof(aTxBuffer3), 5000);

	HAL_SPI_Receive(&hspi1, (uint8_t*)flash_RxBuffer, sizeof(flash_RxBuffer), 10000);
	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	if(flash_RxBuffer[NUM_eeprom_tag_init - SPI_FLASH_LEN_CMDADD] != SPI_FLASH_DATA_TAG)
	{
		//WREN
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, (uint8_t*)aTxBuffer0, sizeof(aTxBuffer0), 5000);
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

		//RDSR
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, (uint8_t*)cmd_rdsr, sizeof(cmd_rdsr), 5000);

		HAL_SPI_Receive(&hspi1, (uint8_t*)data_rdsr, 1, 5000);
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

		//Check WEL
		while((data_rdsr[0] & (1<<1)) == 0)
		{
			//WREN
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1, (uint8_t*)aTxBuffer0, sizeof(aTxBuffer0), 5000);
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

			//RDSR
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1, (uint8_t*)cmd_rdsr, sizeof(cmd_rdsr), 5000);

			HAL_SPI_Receive(&hspi1, (uint8_t*)data_rdsr, 1, 5000);
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
		}

		//Erase
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, (uint8_t*)aTxBuffer1, sizeof(aTxBuffer1), 5000);
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);


		//Erase 0xC7
		//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		//HAL_SPI_Transmit(&hspi1, (uint8_t*)cmd_erase, sizeof(cmd_erase), 5000);
		//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

		//Check WIP
		while((data_rdsr[0] & (1<<0)) != 0)
		{
			//RDSR
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1, (uint8_t*)cmd_rdsr, sizeof(cmd_rdsr), 5000);

			HAL_SPI_Receive(&hspi1, (uint8_t*)data_rdsr, 1, 5000);
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
		}

		//WREN
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, (uint8_t*)aTxBuffer0, sizeof(aTxBuffer0), 5000);
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);


		//RDSR
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, (uint8_t*)cmd_rdsr, sizeof(cmd_rdsr), 5000);

		HAL_SPI_Receive(&hspi1, (uint8_t*)data_rdsr, 1, 5000);
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

		//Check WEL
		while((data_rdsr[0] & (1<<1)) == 0)
		{
			//WREN
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1, (uint8_t*)aTxBuffer0, sizeof(aTxBuffer0), 5000);
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

			//RDSR
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1, (uint8_t*)cmd_rdsr, sizeof(cmd_rdsr), 5000);

			HAL_SPI_Receive(&hspi1, (uint8_t*)data_rdsr, 1, 5000);
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
		}

		//Program
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, (uint8_t*)flash_IgEvent, sizeof(flash_IgEvent), 5000);
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

		//RDSR
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, (uint8_t*)cmd_rdsr, sizeof(cmd_rdsr), 5000);

		HAL_SPI_Receive(&hspi1, (uint8_t*)data_rdsr, 1, 5000);
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

		//Check WIP
		while((data_rdsr[0] & (1<<0)) != 0)
		{
			//RDSR
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1, (uint8_t*)cmd_rdsr, sizeof(cmd_rdsr), 5000);

			HAL_SPI_Receive(&hspi1, (uint8_t*)data_rdsr, 1, 5000);
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
		}

		//Read flash data
		//RDSR
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, (uint8_t*)cmd_rdsr, sizeof(cmd_rdsr), 5000);

		HAL_SPI_Receive(&hspi1, (uint8_t*)data_rdsr, 1, 5000);
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

		//Check WEL
		/*
		while((data_rdsr[0] & (1<<1)) != 0)
		{
			//RDSR
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
			HAL_SPI_Transmit(&hspi1, (uint8_t*)cmd_rdsr, sizeof(cmd_rdsr), 5000);

			HAL_SPI_Receive(&hspi1, (uint8_t*)data_rdsr, 1, 5000);
			HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
		}
		*/

		//Read
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		HAL_SPI_Transmit(&hspi1, (uint8_t*)aTxBuffer3, sizeof(aTxBuffer3), 5000);

		HAL_SPI_Receive(&hspi1, (uint8_t*)flash_RxBuffer, sizeof(flash_RxBuffer), 10000);
		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

	}
	else
	{
		for(loop_index = 0; loop_index < (NUM_total-SPI_FLASH_LEN_CMDADD); loop_index++)
		{
			flash_IgEvent[loop_index + SPI_FLASH_LEN_CMDADD] = flash_RxBuffer[loop_index];
		}
		flash_IgEvent[0] = SPI_FLASH_PROGRAM_PAGE;
		flash_IgEvent[1] = SPI_FLASH_ADD_Byte0;
		flash_IgEvent[2] = SPI_FLASH_ADD_Byte1;
		flash_IgEvent[3] = SPI_FLASH_ADD_Byte2;
	}


	//=========================================================

  /* Infinite loop */
  for(;;)
  {
	  if(flag_flashWrite == 1)
	  {
		  //write flash
		  //WREN
		  	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		  	HAL_SPI_Transmit(&hspi1, (uint8_t*)aTxBuffer0, sizeof(aTxBuffer0), 5000);
		  	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

		  	//RDSR
		  	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		  	HAL_SPI_Transmit(&hspi1, (uint8_t*)cmd_rdsr, sizeof(cmd_rdsr), 5000);

		  	HAL_SPI_Receive(&hspi1, (uint8_t*)data_rdsr, 1, 5000);
		  	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

		  	//Check WEL
		  	while((data_rdsr[0] & (1<<1)) == 0)
		  	{
		  		//WREN
		  		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		  		HAL_SPI_Transmit(&hspi1, (uint8_t*)aTxBuffer0, sizeof(aTxBuffer0), 5000);
		  		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

		  		//RDSR
		  		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		  		HAL_SPI_Transmit(&hspi1, (uint8_t*)cmd_rdsr, sizeof(cmd_rdsr), 5000);

		  		HAL_SPI_Receive(&hspi1, (uint8_t*)data_rdsr, 1, 5000);
		  		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
		  	}

		  	//Erase
		  	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		  	HAL_SPI_Transmit(&hspi1, (uint8_t*)aTxBuffer1, sizeof(aTxBuffer1), 5000);
		  	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);


		  	//Erase 0xC7
		  	//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		  	//HAL_SPI_Transmit(&hspi1, (uint8_t*)cmd_erase, sizeof(cmd_erase), 5000);
		  	//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

		  	//Check WIP
		  	while((data_rdsr[0] & (1<<0)) != 0)
		  	{
		  		//RDSR
		  		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		  		HAL_SPI_Transmit(&hspi1, (uint8_t*)cmd_rdsr, sizeof(cmd_rdsr), 5000);

		  		HAL_SPI_Receive(&hspi1, (uint8_t*)data_rdsr, 1, 5000);
		  		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
		  	}

		  	//WREN
		  	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		  	HAL_SPI_Transmit(&hspi1, (uint8_t*)aTxBuffer0, sizeof(aTxBuffer0), 5000);
		  	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);


		  	//RDSR
		  	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		  	HAL_SPI_Transmit(&hspi1, (uint8_t*)cmd_rdsr, sizeof(cmd_rdsr), 5000);

		  	HAL_SPI_Receive(&hspi1, (uint8_t*)data_rdsr, 1, 5000);
		  	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

		  	//Check WEL
		  	while((data_rdsr[0] & (1<<1)) == 0)
		  	{
		  		//WREN
		  		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		  		HAL_SPI_Transmit(&hspi1, (uint8_t*)aTxBuffer0, sizeof(aTxBuffer0), 5000);
		  		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

		  		//RDSR
		  		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		  		HAL_SPI_Transmit(&hspi1, (uint8_t*)cmd_rdsr, sizeof(cmd_rdsr), 5000);

		  		HAL_SPI_Receive(&hspi1, (uint8_t*)data_rdsr, 1, 5000);
		  		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
		  	}

		  	//Program
		  	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		  	HAL_SPI_Transmit(&hspi1, (uint8_t*)flash_IgEvent, sizeof(flash_IgEvent), 5000);
		  	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

		  	//RDSR
		  	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		  	HAL_SPI_Transmit(&hspi1, (uint8_t*)cmd_rdsr, sizeof(cmd_rdsr), 5000);

		  	HAL_SPI_Receive(&hspi1, (uint8_t*)data_rdsr, 1, 5000);
		  	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

		  	//Check WIP
		  	while((data_rdsr[0] & (1<<0)) != 0)
		  	{
		  		//RDSR
		  		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		  		HAL_SPI_Transmit(&hspi1, (uint8_t*)cmd_rdsr, sizeof(cmd_rdsr), 5000);

		  		HAL_SPI_Receive(&hspi1, (uint8_t*)data_rdsr, 1, 5000);
		  		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
		  	}

		  	//RDSR
		  	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		  	HAL_SPI_Transmit(&hspi1, (uint8_t*)cmd_rdsr, sizeof(cmd_rdsr), 5000);

		  	HAL_SPI_Receive(&hspi1, (uint8_t*)data_rdsr, 1, 5000);
		  	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);

		  	//Check WEL
		  	/*
		  	while((data_rdsr[0] & (1<<1)) != 0)
		  	{
		  		//RDSR
		  		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		  		HAL_SPI_Transmit(&hspi1, (uint8_t*)cmd_rdsr, sizeof(cmd_rdsr), 5000);

		  		HAL_SPI_Receive(&hspi1, (uint8_t*)data_rdsr, 1, 5000);
		  		HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
		  	}
		  	*/

		  	//Read
		  	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
		  	HAL_SPI_Transmit(&hspi1, (uint8_t*)aTxBuffer3, sizeof(aTxBuffer3), 5000);

		  	HAL_SPI_Receive(&hspi1, (uint8_t*)flash_RxBuffer, sizeof(flash_RxBuffer), 10000);
		  	HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);


		    flag_flashWrite = 0;
	  }

	  osDelay(SPI1_TASK_ENTRY_TIME);
  }
  /* USER CODE END spi1_entry */
}

/* can_entry function */
void can_entry(void const * argument)
{
  /* USER CODE BEGIN can_entry */
	//uint8_t canTx[8] = {0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07};
	uint8_t ubKeyNumber = 0x0;
	CAN_FilterConfTypeDef  sFilterConfig;
	CanTxMsgTypeDef  TxMessage;
 	CanRxMsgTypeDef  RxMessage;

 	hcan.pTxMsg = &TxMessage;
 	hcan.pRxMsg = &RxMessage;


 	/*##-2- Configure the CAN Filter ###########################################*/
	  sFilterConfig.FilterNumber = 0;
	  sFilterConfig.FilterMode = CAN_FILTERMODE_IDMASK;
	  sFilterConfig.FilterScale = CAN_FILTERSCALE_32BIT;
	  sFilterConfig.FilterIdHigh = 0x0000;
	  sFilterConfig.FilterIdLow = 0x0000;
	  sFilterConfig.FilterMaskIdHigh = 0x0000;
	  sFilterConfig.FilterMaskIdLow = 0x0000;
	  sFilterConfig.FilterFIFOAssignment = 0;
	  sFilterConfig.FilterActivation = ENABLE;
	  sFilterConfig.BankNumber = 14;

	  if (HAL_CAN_ConfigFilter(&hcan, &sFilterConfig) != HAL_OK)
	  {
		/* Filter configuration Error */
		Error_Handler();
	  }

	  /*##-3- Configure Transmission process #####################################*/
	  hcan.pTxMsg->StdId = 0x321;
	  hcan.pTxMsg->ExtId = 0x01;
	  hcan.pTxMsg->RTR = CAN_RTR_DATA;
	  hcan.pTxMsg->IDE = CAN_ID_STD;
	  hcan.pTxMsg->DLC = 4;

	  //hcan.pTxMsg->Data[0] = 0xA5;
	  //hcan.pTxMsg->Data[1] = 0x63;


  /* Infinite loop */
  for(;;)
  {
	  /*##-2- Start the Reception process and enable reception interrupt #########*/
		if (HAL_CAN_Receive_IT(&hcan, CAN_FIFO0) != HAL_OK)
		{
			/* Reception Error */
			Error_Handler();
		}

	  /* Set the data to be transmitted */
			ubKeyNumber++;
			hcan.pTxMsg->Data[0] = ubKeyNumber;
			hcan.pTxMsg->Data[1] = 0x55;
			hcan.pTxMsg->Data[2] = 0xaa;
			hcan.pTxMsg->Data[3] = 0xaa;

	  /*##-3- Start the Transmission process ###############################*/
	  if (HAL_CAN_Transmit(&hcan, 10) != HAL_OK)
	  {
		/* Transmission Error */
		Error_Handler();
	  }

	  //HAL_CAN_Transmit_IT(&hcan);
	  //HAL_CAN_Receive(&hcan,  CAN_FIFO0, 1000);
	  osDelay(CAN_TASK_ENTRY_TIME);
  }
  /* USER CODE END can_entry */
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

void HAL_RTC_AlarmAEventCallback(RTC_HandleTypeDef *handleRTC)
{
    // RTC wakeup
	if(current_IgEvent[NUM_alarm_status] == 1)
	{
		HAL_GPIO_WritePin(GPIOC, TEST_4G_Pin, GPIO_PIN_RESET);
		HAL_Delay(1);
		HAL_GPIO_WritePin(GPIOC, TEST_4G_Pin, GPIO_PIN_SET);
	}
}



uint8_t adc_voltageConversion_int(uint32_t adc_value)
{
	// (((adc_24v & 0xfffU) * 330) / 409600) * 117 / 10
	//return ((((adc_value & 0xfffU) * 330) ) * 11700) / 1000 / 409600;
	return ((((adc_value & 0xfffU) * 330) ) * 117) / 1000 / 4096;
}

uint8_t adc_voltageConversion_float(uint32_t adc_value)
{
	// (((((adc_24v & 0xfffU) * 330) / 409600) * 117 / 10  * 100)% 100)
	return (((((adc_value & 0xfffU) * 330) ) * 11700) / 10 / 409600) % 100;
}

uint8_t adc_tempConversion_int(uint32_t adc_value)
{
	uint16_t mcu_adc, r_adc;
	double temp;
	mcu_adc = (((adc_value & 0xfffU) * 330) / 409600);

	// R = (10*MCU_ADC) / (3.3-MCU_ADC)
	r_adc = (100 * mcu_adc)/(33 - 10*mcu_adc);

	//r_adc = 10*(adc_value & 0xfffU)/(4096-(adc_value & 0xfffU))


	// 1/( log(r_adc/THERMISTOR_R25)/THERMISTOR_B_CONSTANT+1/(298.13) ) -273.13??;
	//return (100/( log(r_adc/THERMISTOR_R25)/THERMISTOR_B_CONSTANT + 100/(29813) ) -27313)/100;
	//return ((-1379)*(1*r_adc-10)+25000)/1000;
	return (158883840-26379*(adc_value & 0xfffU))/4096000;
}

uint8_t adc_tempConversion_float(uint32_t adc_value)
{
	uint16_t mcu_adc, r_adc, temp;
	mcu_adc = (((adc_value & 0xfffU) * 330) / 409600);

	// R = (10*MCU_ADC) / (3.3-MCU_ADC)
	r_adc = (100 * mcu_adc)/(33 - 10*mcu_adc);

	// 1/( log(r_adc/THERMISTOR_R25)/THERMISTOR_B_CONSTANT+1/(298.13) ) -273.13??;
	//return (100/( log(r_adc/THERMISTOR_R25)/THERMISTOR_B_CONSTANT+100/(29813) ) -27313) %100;
	//return (((-1379)*(1*r_adc-10)+25000)/10) % 100;
	return ((158883840-26379*(adc_value & 0xfffU))/40960)%100;
}

uint8_t uart2_findCommaIndexNext(uint8_t uart_msg[], uint8_t target_comma)
{
	uint16_t index, split_count;
	// find index
	split_count = 0;
	for(index = 0; index<UART2_RX_LENGTH; index++)
	{
		if(uart_msg[index] == ',')
		{
			split_count++;
		}

		if(split_count == target_comma)
		{
			return (index+1);
		}
	}
	return 0;
}


uint8_t uart_findDataIndex(uint8_t uart_msg[], uint8_t target_data)
{
	// target data is the target_comma
	// first data is regard as 0th comma

	uint16_t index, split_count;
	// find index
	split_count = 0;
	for(index = 0; index<UART2_RX_LENGTH; index++)
	{
		if(target_data == 1)
		{
			if(uart_msg[index] == ':')
			{
				return (index+2);
			}
		}
		else
		{
			if(uart_msg[index] == ',')
			{
				split_count++;
			}

			if(split_count == target_data)
			{
				return (index+1);
			}
		}
	}
	return 0; //not found
}

uint8_t uart2_isFindString(uint8_t uart_msg[], uint8_t target_string[], uint8_t length)
{
	uint16_t index, string_index, flag_find = 0;
	for(index = 0; index<UART2_RX_LENGTH; index++)
	{
		if(uart_msg[index] == target_string[0])
		{
			for(string_index = 1; string_index < length; string_index++)
			{
				if(uart_msg[index + string_index] != target_string[string_index])
				{
					flag_find = 0;
					break;
				}
				else
				{
					flag_find = 1;
				}
			}
			//break;
		}
	}
	return flag_find;
}

void enable_4GModule(void)
{
	//A1
	HAL_GPIO_WritePin(ENABLE_4G_GPIO_Port, ENABLE_4G_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(ENABLE_4G_GPIO_Port, ENABLE_4G_Pin, GPIO_PIN_SET);

	//A0
	HAL_GPIO_WritePin(GPIOC, TEST_4G_Pin, GPIO_PIN_RESET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(GPIOC, TEST_4G_Pin, GPIO_PIN_SET);
}

void print_atCommand(uint8_t rx_msg[], uint8_t onOff)
{
	int i;
	if(onOff == 1)
	{
		aewin_dbg("=====AT command=====\r\n");
		for(i=0;i<UART2_RX_LENGTH;i++)
		{
			aewin_dbg("%c",rx_msg[i]);
		}
		aewin_dbg("---------------------\r\n");
	}
}

uint8_t check_lowPower(uint32_t adc_value)
{
	uint8_t low_std = 9*10 + current_IgEvent[NUM_12V_shutdown]*5;
	//low_std = current_IgEvent[NUM_in_volt_min]*10;
	uint8_t current_voltage = (adc_voltageConversion_int(adc_value)*10 + (adc_voltageConversion_float(adc_value)/10) );

	if( current_voltage < low_std)
	{
		//aewin_dbg("Current Voltage %d V is lower than shutdown power setting %d V.\r\n", current_voltage, low_std);
		return 1;
	}
	else
	{
		//aewin_dbg("Current Voltage %d V is higher than shutdown power setting %d V.\r\n", current_voltage, low_std);
		return 0;
	}
}

uint8_t check_highPower(uint32_t adc_value)
{
	uint8_t high_std = 10*10 + current_IgEvent[NUM_12V_startup]*5;
	uint8_t current_voltage = (adc_voltageConversion_int(adc_value)*10 + (adc_voltageConversion_float(adc_value)/10) );

	if( current_voltage > high_std)
	{
		//aewin_dbg("\r\nCurrent Voltage %d V is higher than startup power setting %d V.\r\n", current_voltage, high_std);
		return 1;
	}
	else
	{
		//aewin_dbg("\r\nCurrent Voltage %d V is lower than startup power setting %d V.\r\n", current_voltage, high_std);
		return 0;
	}
}

void config_recovery(void)
{
	uint8_t data_index;

	for(data_index = 0; data_index<NUM_total;data_index++)
	{
		current_IgEvent[data_index] = flash_IgEvent[data_index];
	}

}

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
