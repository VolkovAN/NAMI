/**
  ******************************************************************************
  * @file    03_ContinuousConversionSW_Trig/main.c 
  * @author  Волков А.Н.
  * @version V2.0.0
  * @date    26-октября-2016
  *
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - SYSTICK ()

  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "Timer.h"

TypeTimer							Timer[NUM_TIMER];

/** @addtogroup STM32F0_Snippets
  * @{
  */



/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Delay value : short one is used for the error coding, long one (~1s) in case 
   of no error or between two bursts */


//#define 100_mS			100

/* Private functions ---------------------------------------------------------*/
/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
  delays number of tick Systicks (happens every 1 ms)
 *----------------------------------------------------------------------------*/

/**
* @brief  This function handles Timer Handler.
  *         It toggles the green led if the action has been performed correctly
  *         and toggles the orange led coding the error number
  * @param  None
  * @retval None
  */
void Timer_Handler(void)
{ uint32_t	i;
//*** работа таймеров *************************
	for(i=0;i < NUM_TIMER;i++)
	if( Timer[i].Start )
		{	Timer[i].Tick++; 
			if( Timer[i].Tick == Timer[i].EndTick )
			{	Timer[i].Start = 0;
				Timer[i].Tick = 0;
				Timer[i].Flag = 1;
				if((Timer[i].pF) != NULL ) Timer[i].pF(NULL);
			}
		}
}


/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                               */
/******************************************************************************/


/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
