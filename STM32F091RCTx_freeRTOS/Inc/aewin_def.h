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
#define AEWIN_DBUG			1
#define PRINT_BUFF			64


#define UART3_TIMEOUT		2


#define ADC_DEVICE_NUM		2
/* USER CODE END Definition */


/* USER CODE BEGIN ENUM definitions */

/* USER CODE END ENUM definitions  */



/* USER CODE BEGIN structure definitions */
typedef struct{
	uint32_t pwr_24v;
	uint32_t mcu_temp;
}sADC_Data;


/* USER CODE END structure definitions  */




/* USER CODE BEGIN Private defines */

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
