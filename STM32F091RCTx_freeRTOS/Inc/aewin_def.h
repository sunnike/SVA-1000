/*
 * aewin_def.h
 *
 *  Created on: 2018年4月15日
 *      Author: John Chen
 */

#ifndef AEWIN_DEF_H_
#define AEWIN_DEF_H_

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_hal.h"
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */


/* USER CODE BEGIN Definition */
#ifndef TRUE
	#define TRUE 	(1)
#endif

#ifndef FALSE
	#define FALSE	(0)
#endif


 /** @defgroup AEWIN_DEBUG Aewin Debug Configuration
   * @{
   */
#define AEWIN_DBUG			(1)
#define PRINT_BUFF			(128)



#define UART1_TIMEOUT		1
#define UART2_TIMEOUT		1
#define UART3_TIMEOUT		1

#define I2C_DEV_ID_ADXL345       0xE5
#define I2C_DEV_ADDRESS_ADXL345  0x3A
#define I2C_MEM_ADDRESS_ADXL345  0x32
#define BUFFER_SIZE_ADXL345      6



 /** @defgroup Aewin_Task_Entry_Time Tasks entry time configuration
   * @{
   */
#define UART1_TASK_ENTRY_TIME		(1U)
#define UART3_TASK_ENTRY_TIME		(1U)

#define INGITION_TASK_ENTRY_TIME	(980U)
#define GPIO_GET_TASK_TIME			(50)
#define IWDG_TASK_ENTRY_TIME		(10U)


 /** @defgroup STM32F0XX_ADC STM32F0XX ADC Configuration
   * @{
   */
#define ADC_DEVICE_NUM				(2U)
#define ADC_CAP_TIME				(450U)
#define ADC_RESET_TIME				(450U)




 /** @defgroup Aewin_SVA-1000_Command_Protocol Aewin SVA-1000 Command Protocol
   * @{
   */
#define CMD_MAX_LEN					(32U)
#define CMD_HEAD_SIZE				(3U)
#define CMD_MS_SIZE					(2U)
#define CMD_HEAD_MS_SIZE			(CMD_HEAD_SIZE + CMD_MS_SIZE)
#define CMD_TAIL_SIZE				(3U)

#define CMD_SYN_CODE				(0x16U)
#define CMD_STX_CODE				(0x02U)
#define CMD_ETX_CODE				(0x03U)
#define CMD_EOT_CODE				(0x04U)


#define CMD_SYN_POS0				(0U)
#define CMD_SYN_POS1				(1U)
#define CMD_STX_POS					(2U)
#define CMD_MCMD_POS				(3U)
#define CMD_SCMD_POS 				(4U)

#define MCU_REPO_HEAD_CMD_SIZE		(3)
#define MCU_REPO_ID					(0x91U)
#define MCU_ID_POS					(0U)
#define MCU_REPO_MCMD_POS			(1U)

#define CMD_ETX_POS(x) 				(x + 0U)
#define CMD_CHKSUM_POS(x)			(x + 1U)
#define CMD_EOT_POS(x)				(x + 2U)

 /** MCU sub-commands length (Unit: byte). */
#define MCU_SCMD10_LEN				(2U)
#define MCU_SCMD20_LEN				(7U)
#define MCU_SCMD21_LEN				(7U)
#define MCU_SCMD22_LEN				(1U)
#define MCU_SCMD23_LEN				(1U)
#define MCU_SCMD26_LEN				(1U)
#define MCU_SCMD27_LEN				(1U)
#define MCU_SCMD28_LEN				(1U)
#define MCU_SCMD29_LEN				(1U)
#define MCU_SCMD30_LEN				(1U)
#define MCU_SCMD31_LEN				(1U)
#define MCU_SCMD32_LEN				(1U)
#define MCU_SCMD33_LEN				(1U)
#define MCU_SCMD34_LEN				(1U)
#define MCU_SCMD36_LEN				(1U)
#define MCU_SCMD37_LEN				(1U)
#define MCU_SCMD3A_LEN				(1U)
#define MCU_SCMD3B_LEN				(1U)
#define MCU_SCMD40_LEN				(1U)
#define MCU_SCMD41_LEN				(1U)
#define MCU_SCMD44_LEN				(1U)
#define MCU_SCMD45_LEN				(1U)
#define MCU_SCMD50_LEN				(1U)
#define MCU_SCMD51_LEN				(1U)
#define MCU_SCMD52_LEN				(1U)
#define MCU_SCMD53_LEN				(1U)
#define MCU_SCMD54_LEN				(1U)
#define MCU_SCMD55_LEN				(1U)
#define MCU_SCMD56_LEN				(1U)
#define MCU_SCMD57_LEN				(1U)
#define MCU_SCMD70_LEN				(4U)
#define MCU_SCMD80_LEN				(6U)
#define MCU_SCMD90_LEN				(6U)
#define MCU_SCMDA0_LEN				(2U)


/** Ignition sub-commands length (Unit: byte). */
#define IGN_SCMD10_LEN				(1U)
#define IGN_SCMD14_LEN				(4U)
#define IGN_SCMD15_LEN				(4U)
#define IGN_SCMD16_LEN				(2U)
#define IGN_SCMD17_LEN				(2U)
#define IGN_SCMD18_LEN				(2U)
#define IGN_SCMD19_LEN				(2U)
#define IGN_SCMD20_LEN				(1U)
#define IGN_SCMD21_LEN				(1U)
#define IGN_SCMD22_LEN				(2U)
#define IGN_SCMD23_LEN				(2U)
#define IGN_SCMD24_LEN				(2U)
#define IGN_SCMD25_LEN				(2U)
#define IGN_SCMD26_LEN				(3U)
#define IGN_SCMD27_LEN				(3U)
#define IGN_SCMD40_LEN				(1U)
#define IGN_SCMD41_LEN				(1U)

 /** Ignition sub-commands length (Unit: byte). */
 #define W4G_SCMD10_LEN				(22U)


/** Main Commands */
#define M_CMD_MCU_SETTING 			(0x10U)
#define M_CMD_IG_SETTING			(0x20U)
#define M_CMD_4G_SETTING			(0x30U)

#define CMD_HEAD_CHK				((CMD_SYN_CODE << 24) | (CMD_SYN_CODE << 16) | (CMD_STX_CODE << 8))
#define CMD_MAIN_CHK(mcmd)			(CMD_HEAD_CHK | mcmd)


#define SUB_MCU_FW_VER_LEN			(2U)


/** EEPROM accessing */
#define ADDR_FLASH_PAGE_126      ((uint32_t)0x0803F000) /* Base @ of Page 126, 2 Kbytes */
#define ADDR_FLASH_PAGE_127      ((uint32_t)0x0803F800) /* Base @ of Page 127, 2 Kbytes */

#define FLASH_USER_START_ADDR    ADDR_FLASH_PAGE_127   /* Start @ of user Flash area */
#define FLASH_USER_END_ADDR      ADDR_FLASH_PAGE_127   /* End @ of user Flash area */

#define EEPROM_TAG               ((uint32_t)0x12345678)

 /* USER CODE END Definition */


/* USER CODE BEGIN ENUM definitions */

 /******  SVA-1000 specific Ignition States ******************************************************************/
typedef enum{
	IG_Reserved = 0,			 	/*!< Reserved */
	IG_Recovery = 1,				/*!< Recovery all of states and parameters */
	IG_PowerOn_Delay,				/*!< Indicate that the system is ready to "power on" */
	IG_Wait_StartUp,				/*!< Indicate that the system is ready to "start up" */
	IG_Start_Up,					/*!< Indicate that the system is in "start up" state */
	IG_Shutdown_Delay,
	IG_shutting_Down,
	IG_LowPower_Delay,
	IG_CloseUp,
	IG_End = 100
}eIgnition_States;


/******  SVA-1000 MCU setting/Information sub-commands ******************************************************************/
typedef enum{
	Subcmd_MCU_FW_Ver 		= 0x10,
	Subcmd_MCU_Get_Date		= 0x20,
	Subcmd_MCU_Set_Date 	= 0x21,
	Subcmd_Get_Sys_InVOLT	= 0x22,
	Subcmd_Set_Sys_InVOLT	= 0x23,
	Subcmd_Get_RebootSrc	= 0x26,
	Subcmd_Set_RebootSrc	= 0x27,
	Subcmd_Get_BootMode		= 0x28,
	Subcmd_Set_BootMode		= 0x29,
	Subcmd_Get_WWAN_WKStat	= 0x30,
	Subcmd_Set_WWAN_WKStat	= 0x31,
	Subcmd_Get_WWAN_Stat	= 0x32,
	Subcmd_Set_WWAN_Stat	= 0x33,
	Subcmd_Get_DigiIn		= 0x34,
	Subcmd_Get_DigiOut		= 0x36,
	Subcmd_Set_DigiOut		= 0x37,
	Subcmd_Get_SIM_Mode		= 0x3A,
	Subcmd_Set_SIM_Mode		= 0x3B,
	Subcmd_Get_WIFI_OnOff	= 0x40,
	Subcmd_Set_WIFI_OnOff	= 0x41,
	Subcmd_Get_LAN_WKStat	= 0x44,
	Subcmd_Set_LAN_WKStat	= 0x45,
	Subcmd_Get_DelayOffStat	= 0x50,
	Subcmd_Set_DelayOffStat = 0x51,
	Subcmd_Get_DelayOnStat	= 0x52,
	Subcmd_Set_DelayOnStat  = 0x53,
	Subcmd_Get_DelayOffTime	= 0x54,
	Subcmd_Set_DelayOffTime = 0x55,
	Subcmd_Get_DelayOnTime  = 0x56,
	Subcmd_Set_DelayOnTime  = 0x57,
	Subcmd_Get_ADC			= 0x70,
	Subcmd_Get_GPS			= 0x80,
	Subcmd_Get_Gsensor		= 0x90,
	Subcmd_OS_Shutdown		= 0xA0
}eMCU_Setting_SubCMDs;




/******  SVA-1000 Ignition setting/Information sub-commands ******************************************************************/
typedef enum{
	Subcmd_IG_Get_OnOff 	= 0x10,
	Subcmd_Get_LowBat_Data	= 0x14,
	Subcmd_Set_LowBat_Data 	= 0x15,
	Subcmd_Get_InVol_Limit	= 0x16,
	Subcmd_Set_InVol_Limit	= 0x17,
	Subcmd_Get_PwrOn_DelayT	= 0x18,
	Subcmd_Set_PwrOn_DelayT = 0x19,
	Subcmd_Get_Alarm_Stat	= 0x20,
	Subcmd_Set_Alarm_Stat	= 0x21,
	Subcmd_Get_RTC_LocalT	= 0x22,
	Subcmd_Set_RTC_LocalT	= 0x23,
	Subcmd_Get_RTC_AlarmT	= 0x24,
	Subcmd_Set_RTC_AlarmT	= 0x25,
	Subcmd_Get_RTC_WakeT	= 0x26,
	Subcmd_Set_RTC_WakeT	= 0x27,
	Subcmd_Get_Host_WTGT	= 0x40,
	Subcmd_Set_Host_WTGT	= 0x41,
}eIG_Setting_SubCMDs;


/******  SVA-1000  Event/Log information sub-commands ******************************************************************/
typedef enum{
	Subcmd_TeleComm_Event	= 0x10
}eEvent_Log_SubCMDs;


/* USER CODE END ENUM definitions  */



/* USER CODE BEGIN structure definitions */

typedef struct{
	uint32_t pwr_24v;
	uint32_t mcu_temp;
}sADC_DATA;


/* The structure that contains the ignition related parameters, along with an IG_States
that is used to identify which power state is. */
typedef struct{
	uint32_t eeprom_tag_init;       /*!< EEPROM TAG for mark if this data saved to flash or not */
	uint8_t  major_ver;				/*!< MCU major version. */
	uint8_t  minor_ver;				/*!< MCU minor version. */
	eIgnition_States IG_States;		/*!< Ignition states. */
	uint8_t	 pwron_delay;			/*!< Delay time after ignition switch-on from "Close-Up" */
	uint8_t  wait_startup_time;		/*!< /*!< The delay time to prepare entering "Start Up" state. */
	uint8_t	 startup_timeout;		/*!< Timeout value to accept the ignition signal. */
	uint8_t  pwroff_delay;			/*!< Delay time after ignition switch-off from "Start-Up" */
	uint8_t  shutdown_delay;		/*!< The delay time to prepare entering "Power Off" state. */
	uint8_t  shutdown_timeout;		/*!< Timeout value to accept the ignition signal. */
	uint8_t  lowpwr_dealy;			/*!< The delay time of low-power shutdown */
	uint8_t  wtdog_default;
	uint8_t  pwroff_btn_cnt;
	uint8_t	 pwrbtn_pressed;
	uint8_t  pwrgood_chk_time;
	uint8_t  in_sys_volt;			/*!< System voltage */
	uint8_t  in_volt_min;			/*!< Minimum input voltage */
	uint8_t  in_volt_max;			/*!< Maximum input voltage */
	uint8_t  startup_volt;			/*!< The lowest voltage that can be accepted to start up system */
	uint8_t  in_temp_min;			/*!< The acceptable minimum temperature */
	uint8_t  in_temp_max;			/*!< The acceptable maximum temperature */
	uint8_t  startup_temp;			/*!< The highest temperature that can be accepted to start up system */
	uint8_t  wireless_state;		/*!< Wireless states, like 3G/4G, WiFi, LAN, WLAN... */
	uint16_t fail_retry;			/*!< Record the re-power times in case of system failure */
	uint16_t fail_count;
}sIG_EVENT;






typedef struct{
	uint8_t	 ig_sw			: 1;
	uint8_t  pwr_btn		: 1;
	uint8_t  sys_pwron		: 1;
	uint8_t  dc2dc_pwrok	: 1;
	uint8_t  reserved		: 4;
}sSVA_GPI_STATE;


typedef struct{
	uint8_t	 spi1_cs 		: 1;
	uint8_t	 dc2dc_en 		: 1;
	uint8_t	 reseved 		: 6;
}sSVA_GPO_STATE;

/* USER CODE END structure definitions  */




/* USER CODE BEGIN type definitions */
typedef void (*cmd_fun)(int argc, char **argv);



/* USER CODE END type defines */


/* USER CODE BEGIN Private defines */
extern DMA_HandleTypeDef hdma_usart3_tx;
extern DMA_HandleTypeDef hdma_usart3_rx;
/* USER CODE END Private defines */



/* USER CODE BEGIN Prototypes */
/*
int fputc(int ch, FILE *f);
int getc(FILE *f);
*/


#if (AEWIN_DBUG)
	void aewin_dbg(char *fmt,...);
	#define printf		aewin_dbg
#else
    #define aewin_dbg(var,...)
#endif

/* USER CODE END Prototypes */



#ifdef __cplusplus
}
#endif




#endif /* AEWIN_DEF_H_ */
