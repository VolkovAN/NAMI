/********************************************************************************
 *
 *      http://we.easyelectronics.ru/blog/STM32/3431.html
 *      PA9  - USART1_TX	7(rx)-m590e - 8 нога разъёма
 *      PA10 - USART1_RX  
 *
 ********************************************************************************
  * @author  Волков А.Н.
  * @version V2.0.0
  * @date    26-октября-2016
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - RCC
   - GPIO PA9(USART1_TX),PA0,PC8,PC9
   - USART1
  
  *    
  ******************************************************************************

  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include "USART.h"

/** @addtogroup STM32F0_Snippets
  * @{
  */

/* Private typedef -----------------------------------------------------------*/

/* Private define ------------------------------------------------------------*/
// USART_BRR = (fck + baudrate /2 ) / baudrate
// http://easystm32.ru/interfaces/15-uart-in-stm32-part-1
// http://frela-mk.narod.ru/olderfiles/1/metodichka_lr3.pdf
//  http://catethysis.ru/stm32-uart-usart/

	/* Private macro -------------------------------------------------------------*/
	/* Variables ---------------------------------------------------------*/
		/*static TBUF_SPACE*/volatile	 uint8_t tbuf [TBUF_SIZE];
		/*static RBUF_SPACE*/volatile	 uint8_t rbuf [RBUF_SIZE];
		InterfaceUSART	UART1 =	{		USART1,
																rbuf,
																tbuf,
																0x00,//r_in
																0x00,//r_out
																0x00,//t_in
																0x00,//t_out
																0x00,//ReciveBufferOverFlow;
																0x00,//TransmitBufferOverFlow;
																0x00,	/* LengthPackageRecive*/
																0x00,	/* LengthPackageTransmit */
																0x00,	/* CountAttempt */
																BAUDRATE,// Скорость обмена
													}; //_at_	0x80
  
//#define APBCLK   8000000UL  // частота тактирования usart
	/* Private functions ---------------------------------------------------------*/
void	Clear_USART()
	{	for(UART1.r_in=0; UART1.r_in < RBUF_SIZE; )
		{	UART1.pBR[UART1.r_in] = 0x00; UART1.r_in++; }
		UART1.r_in = 0x00;
		for(UART1.t_in=0; UART1.t_in < TBUF_SIZE; )
		{	UART1.pBT[UART1.t_in] = 0x00; UART1.t_in++; }
		UART1.t_in = 0x00;
	}

/*__INLINE*/ void Configure_GPIO_USART1(void)
{
  /* Enable the peripheral clock of GPIOA */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
  /* GPIO configuration for USART1 signals */

	GPIOA->MODER |= GPIO_MODER_MODER9_1;  //Alternate function mode
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_9;  //Output push-pull
	GPIOA->OSPEEDR |=GPIO_OSPEEDER_OSPEEDR9;  //40 MHz
	GPIOA->PUPDR &=~GPIO_PUPDR_PUPDR9;  //No pull-up, pull-down

	GPIOA->MODER |= GPIO_MODER_MODER10_1;  //Alternate function mode
	GPIOA->OTYPER |= GPIO_OTYPER_OT_10;  //Output open-drain
	GPIOA->OSPEEDR |=GPIO_OSPEEDER_OSPEEDR10;  //40 MHz
	GPIOA->PUPDR &=~GPIO_PUPDR_PUPDR10;  //No pull-up, pull-down
  /* (5) AF1 for PA9 */
  /* (6) AF1 for PA10 */
  GPIOA->AFR[1] = (GPIOA->AFR[1] &~ (GPIO_AFRH_AFRH1)) | (1<<(1*4)); /* (5) */
  GPIOA->AFR[1] = (GPIOA->AFR[1] &~ (GPIO_AFRH_AFRH2)) | (1<<(2*4)); /* (6) */
}

/**
  * @brief  This function configures USART1.
  * @param  None
  * @retval None
  */
/*__INLINE */void Configure_USART1(void)
{
  /* Enable the peripheral clock USART1 */
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;

  /* Configure USART1 */
  /* (1) oversampling by 16, 9600 baud */
  USART1->BRR = 32000000UL / BAUDRATE; /* (1) */
//  /* (2) Single-wire half-duplex mode */
//  USART1->CR3 = USART_CR3_HDSEL; /* (2) */
	
  /* (3) 8 data bit, 1 start bit, 1 stop bit, no parity, reception and transmission enabled */
  USART1->CR1 = USART_CR1_TE | USART_CR1_RXNEIE | USART_CR1_RE | USART_CR1_UE; /* (3) */
  
  while((USART1->ISR & USART_ISR_TC) != USART_ISR_TC)/* polling idle frame Transmission */
  { 
    /* add time out here for a robust application */
  }
  USART1->ICR |= USART_ICR_TCCF;/* Clear TC flag */
//#ifdef  TRANSMIT_ITERRUPT /*-----------------------------------*/
//		USART1->CR1 |= USART_CR1_TCIE;/* Enable TC interrupt */
//#endif 
  /* Configure IT */
  /* (4) Set priority for USART1_IRQn */
  /* (5) Enable USART1_IRQn */
  NVIC_SetPriority(USART1_IRQn, 0); /* (3) */
  NVIC_EnableIRQ(USART1_IRQn); /* (4) */
}
/********************************************************************************/
void Init_USART(void)
{
  /* Enable the peripheral clock of GPIOA */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
//  GPIOA_CLK_ENABLE;  //Включаем тактирование GPIOA
  /* Enable the peripheral clock USART1 */
  RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
//  USART1_CLK_ENABLE; //Включаем тактирование USART1
  
  GPIOA->MODER   |= GPIO_MODER_MODER9_1;         // PA9  (TX) - Alternate function mode
  GPIOA->MODER   |= GPIO_MODER_MODER10_1;        // PA10 (RX) - Alternate function mode
  GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR9;      // PA9  (TX) - High speed
  
  
  
  GPIOA->AFR[1]    |= 0x0110;  // альтернативные функции
  
  RCC->CFGR3     &= ~RCC_CFGR3_USART1SW;
  RCC->CFGR3     |=  RCC_CFGR3_USART1SW;   //HSI selected as USART1 clock
  
  USART1->CR1 |= USART_CR1_UE;               //Включаем USART1
  USART1->CR1 &= ~USART_CR1_M;               //Данные - 8 бит
  USART1->CR2 &= ~USART_CR2_STOP;            //1 стоп-бит

USART1->BRR = 480000 / 96; /* (1) */
	//USART1->BRR =(APBCLK+BAUDRATE/2)/BAUDRATE; //скорость usart
  USART1->CR1 |= USART_CR1_TE;               //Включаем передатчик USART1
  USART1->CR1 |= USART_CR1_RE;               //Включаем приемник USART1
  
  USART1->CR1  |= USART_CR1_RXNEIE;       //прерывание по приему данных
  NVIC_EnableIRQ (USART1_IRQn);           //разрешить прерывания от USART1
  
}
/*********************************************************************************/
/**
  * @brief  This function handles USART1 interrupt request.
  * @param  None
  * @retval None
  */
void USART1_IRQHandler(void)
{
#ifdef		TRANSMIT_ITERRUPT
  if((USART1->ISR & USART_ISR_TC) == USART_ISR_TC)
  {	// Transmission Complete
    if (UART1.t_in != UART1.t_out) 		// буфер передачи не пуст /* TBUF_SIZE*/)
    { /* clear transfer complete flag and fill TDR with a new char to send */
			//USART1->ICR |= USART_ICR_TCCF;/* Clear TC flag */
			USART1->TDR = (UART1.pBT[UART1.t_out]);
			UART1.t_out++; // t_out прибавляется только здесь!
    }
    else
    {	USART1->ICR |= USART_ICR_TCCF;/* Clear transfer complete flag */
			UART1.LengthPackageTransmit = 0;
			UART1.t_in = 0;UART1.t_out = 0;
    }
  }
   else
#endif		 
	{	if((USART1->ISR & USART_ISR_RXNE) == USART_ISR_RXNE)
		{
			(UART1.pBR[UART1.r_in]) = USART1->RDR;/* Receive data, clear flag */
			UART1.r_in++; // r_in прибавляется только здесь.
			if(UART1.r_in == RBUF_SIZE)
			{	UART1.r_in = 0x00; UART1.LengthPackageRecive = RBUF_SIZE;}
	//    if((USART_Data == 'g')||(USART_Data == 'G'))
	//    {
	//      GPIOC->ODR ^= GPIO_ODR_9; /* Toggle Green LED */
	//    }
		}
		else
		{		UART1.CountAttempt++;
				if( USART1->ISR & USART_ISR_ORE )
				{	UART1.ReciveBufferOverFlow++;
					USART1->ICR |= USART_ICR_ORECF;/* Clear ORE flag */
				}
			//NVIC_DisableIRQ(USART1_IRQn); /* Disable USART1_IRQn */
		}
	}
}

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
