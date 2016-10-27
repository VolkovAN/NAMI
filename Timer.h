/**
  ******************************************************************************
  * @file    Табло.h 
  * @author  Волков А.Н.
  * @version V1.0.0
  * @date    13-Июня-2016
  *
  */

#ifndef  TIMER_SECTION /*-----------------------------------*/

	#define	TIMER_SECTION
	#include "stm32f0xx.h"
	/* Global define ------------------------------------------------------------*/
		#define		NUM_TIMER			2 /* Количество глобальных таймеров в системе */
		#if   !defined	NULL
			#define		NULL			(void *) 0 /*  */
		#endif
	/* Private define ------------------------------------------------------------*/

	/* Global typedef -----------------------------------------------------------*/
		typedef	void (*p_FuncCallBack)	();

		typedef struct	Timer_Struct
		{	uint8_t	    				Start;
			uint8_t	    				Flag;				//
			uint32_t						Tick;
			uint32_t						EndTick;
			p_FuncCallBack			pF;
		}TypeTimer;

	/* Private typedef -----------------------------------------------------------*/
	/* Global macro -------------------------------------------------------------*/
	/* Private macro -------------------------------------------------------------*/
	/* Global variables ---------------------------------------------------------*/
//		volatile	uint32_t	msTicks;                      /* counts 1ms timeTicks       */
//		volatile	uint32_t	flag_msTicks;                 /* flag counts 1ms timeTicks       */
//		volatile	uint32_t	flag_short_timer;
//		volatile	uint32_t	flag_long_timer;
//		volatile	uint32_t	Start_long_timer  = 0; 
//		volatile	uint32_t	Start_short_timer = 0;
//		volatile 	uint16_t	error = 0;  //initialized at 0 and modified by the functions 
	/* Private variables ---------------------------------------------------------*/
	/* Global function prototypes -----------------------------------------------*/
		void Timer_Handler(void);
	/* Private function prototypes -----------------------------------------------*/

#endif	


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
