/**
  ******************************************************************************
  * @file    /main.V02.c 
  * @author  Волков А.Н.
  * @version V2.0.0
  * @date    26-октября-2016
 ===============================================================================
                    #####       MCU Resources     #####
 ===============================================================================
   - ADC on PA1
   - HSI14 MHz for ADC
   - SYSTICK 

 
 ===============================================================================
                    ##### How to test this example #####
 ===============================================================================
    - This example configures the ADC to convert 1 channels (CH1)
    - The  end of conversion (EOC) and end of sequence (EOSEQ) are managed by interrupt.

******************************************************************************
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx.h"
#include <string.h>
#include <ctype.h>
//#include "Tablo.h"
#include "Timer.h"
#include "USART.h"

//#include "gpio_emcu.h"

/** @addtogroup STM32F0_Snippets
  * @{
  */



/* Private typedef -----------------------------------------------------------*/
struct byte {
    unsigned bSlave_1: 1;
    unsigned bSlave_2: 1;
};
 
union Byte {
    unsigned char uSlave;
    struct byte x;
};

/* Private define ------------------------------------------------------------*/

/* define for ADC */
#define NUMBER_OF_ADC_CHANNEL			1 					// Количество каналов АЦП
#define NUMBER_OF_ADC_SAMPLE			16 					// Длинна скользящего среднего
#define ERROR_UNEXPECTED_ADC_IT 	0x01
#define N_S										 		8
/* define for SPI */
//#define SPI2_DATA (0xDE)
//#define SPI1_DATA (0xCA)

/* Internal voltage reference calibration value address */
#define VREFINT_CAL_ADDR ((uint16_t*) ((uint32_t) 0x1FFFF7BA))

/* Error codes used to make the orange led blinking */

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void ConfigureGPIO(void);
void ConfigureTIMxAsPWM(void);
void SetClockForADC(void);
void CalibrateADC(void);
void ConfigureADC(void);
void ConfigureGPIOforADC(void);
void EnableADC(void);
void DisableADC(void);
/* SPI */
void Configure_GPIO_SPI1(void);
void Configure_SPI1(void);
void SPI_10Hz(void);


//void Configure_GPIO_USART1(void);
//void Configure_USART1(void);

/* Private functions ---------------------------------------------------------*/
volatile	uint32_t	msTicks;                      /* counts 1ms timeTicks       */
volatile	uint32_t	flag_msTicks;                 /* flag counts 1ms timeTicks       */
volatile 	uint16_t	error = 0;  //initialized at 0 and modified by the functions 

volatile	uint32_t			count_sec = 0;
volatile	uint32_t			count_msec = 0;
volatile 	uint32_t			Count = 0; 				// счётчик циклов -> 0
volatile 	uint16_t			ADC_array[NUMBER_OF_ADC_SAMPLE]; //Array to store the values coming from the ADC 
volatile 	uint16_t			ADC_Value; //скользящее среднее АЦП 
volatile 	uint32_t			CurrentSample; //index on the ADC_array

union			Byte					SSelect; /* переменная для хранения состояний линий выбора ведомого для SPI */ 

extern		TypeTimer							Timer[NUM_TIMER];
extern		InterfaceUSART				UART1;

/*----------------------------------------------------------------------------
  SysTick_Handler
 *----------------------------------------------------------------------------*/

/*----------------------------------------------------------------------------
  delays number of tick Systicks (happens every 1 ms)
 *----------------------------------------------------------------------------*/
void Delay (uint32_t dlyTicks) {
  uint32_t curTicks;

  curTicks = msTicks;
  while ((msTicks - curTicks) < dlyTicks);
}
/**
  * @brief  Main program.
  * @param  None
  * @retval None
  */


int main(void)
{		
//	uint8_t 		*p_data = &data[0]; // указатель на данные индикатора
//	unsigned char SW_D_Pressure;
	
  /*!< At this stage the microcontroller clock setting is already configured, 
       this is done through SystemInit() function which is called from startup
       file (startup_stm32f0xx.s) before to branch to application main.
       To reconfigure the default setting of SystemInit() function, refer to
       system_stm32f0xx.c file
  
     PLL is set to x4  the system clock SYSCLK = 32MHz
     */
  ConfigureGPIO(); 
//    __WFI();
  SysTick_Config(32000); /* 1ms config */
	/* Настройка таймеров на генерацию 100 Гц, 300 Гц и 500 кГц */
	ConfigureTIMxAsPWM();
#ifdef	ADC_MODE
/*------------------------- Set ADC ------------------*/
/*******************************************************/
  SetClockForADC();
  CalibrateADC();
  ConfigureGPIOforADC();/* ADC - PС4 */
  EnableADC();
  ConfigureADC();
   /*  */
	for(CurrentSample = 0; CurrentSample < NUMBER_OF_ADC_SAMPLE; CurrentSample++)
	{	ADC_array[CurrentSample] = 0x0000;
	}
	ADC_Value = 0x0000;
  ADC1->CR |= ADC_CR_ADSTART; /*  */
/*******************************************************/
#endif

//**********************************************************************************
//* инициализация RS232 ************************************************************
//**********************************************************************************

 	Configure_GPIO_USART1(); // Инициализация ножек PA9,PA10
	Configure_USART1();
	Clear_USART();
	SSelect.uSlave = 0; /* нет выбраных слэйвов */
	Timer[0].EndTick = 100;	/* частота 10 Гц */
	Timer[0].pF = SPI_10Hz;	/* задание callback на десятигерцовый таймер */
	msTicks = 0;
	while (1) /* Infinite loop */
	{
  } /* Infinite loop */ 
} /* End Main */


/**
  * @brief  This function enables the peripheral clocks on GPIO port A,B, F
  * @param  None
  * @retval None
  */
__INLINE void  ConfigureGPIO(void)
{  
  /* (1) Enable the peripheral clock of GPIOA,GPIOB */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN; /* (1a) */  
  RCC->AHBENR |= RCC_AHBENR_GPIOBEN; /* (1b) */  
/******************************* INI Port A  ***************************/
/* [PA8] -> Выход таймера 100 Гц Инициализируются там*/
/* [PA1] -> Выход таймера 300 Гц Инициализируются там*/


	/* PA9,PA10 -> USART. Инициализируются там.*/

	/* PA13 ->SWDIO(SWD)    */
	/* PA14 ->SWCLK(SWD)    */

/******************************* INI Port B  ***************************/
	/* [PB0] -> Выход таймера 500 кГц Инициализируются там*/

/******************************* INI Port C  ***************************/
	/* [PC0] -> Выход SSelect_1 */
	GPIOC->MODER |= GPIO_MODER_MODER0_0;  //output
	GPIOC->OTYPER &= ~GPIO_OTYPER_OT_0;  //Output push-pull
	GPIOC->OSPEEDR |=GPIO_OSPEEDER_OSPEEDR0_0;  //2 MHz
	GPIOC->PUPDR |=GPIO_PUPDR_PUPDR0_0;  //Pull-up
	/* [PC1] -> Выход SSelect_2 */
	GPIOC->MODER |= GPIO_MODER_MODER1_0;  //output
	GPIOC->OTYPER &= ~GPIO_OTYPER_OT_1;  //Output push-pull
	GPIOC->OSPEEDR |=GPIO_OSPEEDER_OSPEEDR1_0;  //2 MHz
	GPIOC->PUPDR |=GPIO_PUPDR_PUPDR1_0;  //Pull-up
/******************************* INI Port F  ***************************/

}

/**
  * @brief  This function configures the TIMx as PWM mode 1
  *         and enables the peripheral clock on TIMx.
  *         It configures GPIO PA8 as Alternate function for TIM1_CH1
  *         It configures GPIO PA1 as Alternate function for TIM2_CH2
  *         It configures GPIO PB0 as Alternate function for TIM3_CH3
  */
__INLINE void ConfigureTIMxAsPWM(void)
{
  RCC->APB2ENR |= RCC_APB2ENR_TIM1EN; /*  Enable the peripheral clock of Timer 1 */
  RCC->APB1ENR |= RCC_APB1ENR_TIM2EN; /*  Enable the peripheral clock of Timer 2 */
  RCC->APB1ENR |= RCC_APB1ENR_TIM3EN; /*  Enable the peripheral clock of Timer 3 */

	/* PA8 как выход TIM1_CH1*/
	GPIOA->MODER |= GPIO_MODER_MODER8_1;  //Alternate function mode
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_8;  //Output push-pull
	GPIOA->OSPEEDR |=GPIO_OSPEEDER_OSPEEDR8;  //40 MHz
	GPIOA->PUPDR &=~GPIO_PUPDR_PUPDR8;  //No pull-up, pull-down
  GPIOA->AFR[1] |= 0x02<<(0*4); /* (4) */
	/* PA1 как выход TIM2_CH2*/
	GPIOA->MODER |= GPIO_MODER_MODER1_1;  //Alternate function mode
	GPIOA->OTYPER &= ~GPIO_OTYPER_OT_1;  //Output push-pull
	GPIOA->OSPEEDR |=GPIO_OSPEEDER_OSPEEDR1;  //40 MHz
	GPIOA->PUPDR &=~GPIO_PUPDR_PUPDR1;  //No pull-up, pull-down
  GPIOA->AFR[0] |= 0x02<<(1*4); /* (5) */
	/* PB0 как выход TIM3_CH3*/
	GPIOB->MODER |= GPIO_MODER_MODER0_1;  //Alternate function mode
	GPIOB->OTYPER &= ~GPIO_OTYPER_OT_0;  //Output push-pull
	GPIOB->OSPEEDR |=GPIO_OSPEEDER_OSPEEDR0;  //40 MHz
	GPIOB->PUPDR &=~GPIO_PUPDR_PUPDR0;  //No pull-up, pull-down
  GPIOB->AFR[0] |= 0x01<<(0*4); /* (6) */
  
/*********************************************************/
// Настраиваем таймер "100 Гц" на использование 1 канала (т.е. контакта PA8)
	/* The TIM1 timer channel 1 after reset is configured as output */
	TIM1->CCER |= TIM_CCER_CC1E;
 // ------- TIMx_CCMRx --------------
	TIM1->CR1 	|= TIM_CR1_ARPE;				//Включен режим предварительной записи регистра автоперезагрузки
	TIM1->CCMR1 |= TIM_CCMR1_OC1PE;			//Включен режим предварительной загрузки регистра сравнения
	TIM1->CCMR1 |= (TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC1M_1);// PWM mode 1

	TIM1->PSC = 20;	/* (1) Set prescaler to 20, so APBCLK/20 i.e 100 kHz */
	TIM1->ARR = 1000; 	/* (2) Set ARR = 1000, as timer clock is 100 kHz the period is 1 ms */

	TIM1->CCR1 = 500; /* Set the duty cycle to 50% */
	/* Enable the TIM1 channel 1 and keep the default configuration [state after reset] for channel polarity */
	TIM1->CCER |= TIM_CCER_CC1E;
	/* Start the timer counter */
  TIM1->CR1 |= TIM_CR1_CEN;		//Старт счета таймера

// Настраиваем таймер "300 Гц" на использование 2 канала (т.е. контакта PA1)
	/* The TIM2 timer channel 2 after reset is configured as output */
	TIM2->CCER |= TIM_CCER_CC2E;
 // ------- TIMx_CCMRx --------------
	TIM2->CR1 	|= TIM_CR1_ARPE;				//Включен режим предварительной записи регистра автоперезагрузки
	TIM2->CCMR1 |= TIM_CCMR1_OC2PE;			//Включен режим предварительной загрузки регистра сравнения
	TIM2->CCMR1 |= (TIM_CCMR1_OC2M_2 | TIM_CCMR1_OC2M_1);// PWM mode 1

	TIM2->PSC = 20;	/* (1) Set prescaler to 20, so APBCLK/240 i.e 100 kHz */
	TIM2->ARR = 333; 	/* (2) Set ARR = 333, as timer clock is 100 kHz the period is 3.33 ms */

	TIM2->CCR2 = 167; /* Set the duty cycle to 50% */
	/* Enable the TIM2 channel 2 and keep the default configuration [state after reset] for channel polarity */
	TIM2->CCER |= TIM_CCER_CC2E;
	/* Start the timer counter */
  TIM2->CR1 |= TIM_CR1_CEN;		//Старт счета таймера

// Настраиваем таймер "500 кГц" на использование 3 канала (т.е. контакта PB0)
	/* The TIM3 timer channel 3 after reset is configured as output */
	/* TIM3->CC3S reset value is 0 */
	TIM3->CCER |= TIM_CCER_CC3E;
 // ------- TIMx_CCMRx --------------
	TIM3->CR1 	|= TIM_CR1_ARPE;				//Включен режим предварительной записи регистра автоперезагрузки
	TIM3->CCMR2 |= TIM_CCMR2_OC3PE;			//Включен режим предварительной загрузки регистра сравнения
	TIM3->CCMR2 |= (TIM_CCMR2_OC3M_2 | TIM_CCMR2_OC3M_1);// PWM mode 1

	TIM3->PSC = 1; 		/* (1) Set prescaler to 1, so APBCLK/1 i.e 2 MHz */
	TIM3->ARR = 4; 	/* (2) Set ARR = 4, as timer clock is 2 MHz the period is 2 us */

	TIM3->CCR3 = 2; /* Set the duty cycle to 50% */
	/* Enable the TIM3 channel 3 and keep the default configuration [state after reset] for channel polarity */
	TIM3->CCER |= TIM_CCER_CC3E;
	/* Start the timer counter */
  TIM3->CR1 |= TIM_CR1_CEN;		//Старт счета таймера
/**********************************************************/

}


/**
  * @brief  This function enables the peripheral clocks on GPIO ports A,B,C
  *         configures PA1, PB1 and PC0 in Analog mode.
  *         For portability, some GPIO are again enabled.
  * @param  None
  * @retval None
  */
__INLINE void  ConfigureGPIOforADC(void)
{ /* (1) Enable the peripheral clock of GPIOC */
  /* (2) Select analog mode for PC4 */
  RCC->AHBENR |= RCC_AHBENR_GPIOCEN; /* (1) */
 	GPIOC->MODER |= GPIO_MODER_MODER4;  /* Analog mode (2) */
}


/**
  * @brief  This function enables the clock in the RCC for the ADC
  *         and start HSI 14MHz dedicated RC oscillator
  * @param  None
  * @retval None
  */
__INLINE void SetClockForADC(void)
{
  /* (1) Enable the peripheral clock of the ADC */
  /* (2) Start HSI14 RC oscillator */ 
  /* (3) Wait HSI14 is ready */
  RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; /* (1) */
  RCC->CR2 |= RCC_CR2_HSI14ON; /* (2) */
  while ((RCC->CR2 & RCC_CR2_HSI14RDY) == 0) /* (3) */
  {
    /* For robust implementation, add here time-out management */
  }  
}


/**
  * @brief  This function performs a self-calibration of the ADC
  * @param  None
  * @retval None
  */
__INLINE void  CalibrateADC(void)
{
  /* (1) Ensure that ADEN = 0 */
  /* (2) Clear ADEN */ 
  /* (3) Launch the calibration by setting ADCAL */
  /* (4) Wait until ADCAL=0 */
  if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
  {
    ADC1->CR &= (uint32_t)(~ADC_CR_ADEN);  /* (2) */  
  }
  ADC1->CR |= ADC_CR_ADCAL; /* (3) */
  while ((ADC1->CR & ADC_CR_ADCAL) != 0) /* (4) */
  {
    /* For robust implementation, add here time-out management */
  }
  __NOP();__NOP();   /* This 2 NOPs are to ensure 2 ADC Cycles 
                        before setting ADEN bit  */
  
}


/**
  * @brief  This function configures the ADC to convert in continuous mode.
  *          
  *         The interrupt on overrun is enabled and the NVIC is configured
  * @param  None
  * @retval None
  */
__INLINE void ConfigureADC(void) /*  */
{
  /* (1) Select HSI14 by writing 00 in CKMODE (reset value) */ 
  /* (2) Select the continuous mode by  */
  /* (3) Select CHSEL14 */ 
  /* (4) Select a sampling mode of 111 i.e. 239.5 ADC clk to be greater than 17.1us */
  /* (5) Enable interrupts on EOC, EOSEQ  */
  /* (6) Wake-up the VREFINT (only for VBAT, Temp sensor and VRefInt) */
  ADC1->CFGR2 &= ~ADC_CFGR2_CKMODE; /* (1) */   
  ADC1->CFGR1 |= ADC_CFGR1_CONT; /* (2)  */
  ADC1->CHSELR = ADC_CHSELR_CHSEL14 ; /* (3) */
  ADC1->SMPR |= ADC_SMPR_SMP; /* (uint32_t)0x00000007) (4) */
  ADC1->IER = ADC_IER_EOCIE | ADC_IER_EOSEQIE /*| ADC_IER_OVRIE*/; /* (5) */
  ADC->CCR |= ADC_CCR_VREFEN; /* (6) */
  
  /* Configure NVIC for ADC */
  /* (7) Enable Interrupt on ADC */
  /* (8) Set priority for ADC */
  NVIC_EnableIRQ(ADC1_COMP_IRQn); /* (7) */
  NVIC_SetPriority(ADC1_COMP_IRQn,0); /* (8) */

}



/**
  * @brief  This function enables the ADC
  * @param  None
  * @retval None
  */
__INLINE void EnableADC(void)
{
  /* (1) Enable the ADC */
  /* (2) Wait until ADC ready */
  ADC1->CR |= ADC_CR_ADEN; /* (1) */
  while ((ADC1->ISR & ADC_ISR_ADRDY) == 0) /* (2) */
  {
    /* For robust implementation, add here time-out management */
  }  
}


/**
  * @brief  This function disables the ADC
  * @param  None
  * @retval None
  */
__INLINE void DisableADC(void)
{
  /* (1) Ensure that no conversion on going */
  /* (2) Stop any ongoing conversion */
  /* (3) Wait until ADSTP is reset by hardware i.e. conversion is stopped */
  /* (4) Disable the ADC */
  /* (5) Wait until the ADC is fully disabled */
  if ((ADC1->CR & ADC_CR_ADSTART) != 0) /* (1) */
  {
    ADC1->CR |= ADC_CR_ADSTP; /* (2) */
  }
  while ((ADC1->CR & ADC_CR_ADSTP) != 0) /* (3) */
  {
     /* For robust implementation, add here time-out management */
  }
  ADC1->CR |= ADC_CR_ADDIS; /* (4) */
  while ((ADC1->CR & ADC_CR_ADEN) != 0) /* (5) */
  {
    /* For robust implementation, add here time-out management */
  }  
}

__INLINE void Configure_GPIO_SPI1(void)
{
  /* Enable the peripheral clock of GPIOA */
  RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
	
  /* (1) Select AF mode (10) on PA4, PA5, PA6, PA7 */
  /* (2) AF0 for SPI1 signals */
  GPIOA->MODER = (GPIOA->MODER 
                  & ~(GPIO_MODER_MODER4 | GPIO_MODER_MODER5 | \
                      GPIO_MODER_MODER6 | GPIO_MODER_MODER7))\
                  | (GPIO_MODER_MODER4_1 | GPIO_MODER_MODER5_1 |\
                     GPIO_MODER_MODER6_1 | GPIO_MODER_MODER7_1); /* (1) */
  GPIOA->AFR[0] = (GPIOA->AFR[0] & \
                   ~(GPIO_AFRL_AFRL4 | GPIO_AFRL_AFRL5 |\
                     GPIO_AFRL_AFRL6 | GPIO_AFRL_AFRL7)); /* (2) */
}

/**
  * @brief  This function configures SPI1.
  * @param  None
  * @retval None
  */
__INLINE void Configure_SPI1(void)
{
  /* Enable the peripheral clock SPI1 */
  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;

  /* Configure SPI1 in master */
  /* (1) Master selection, BR: Fpclk/32 (due to C27 on the board, SPI_CLK is set to 1.5 MHz )
         CPOL and CPHA at zero (rising first edge) */
  /* (2) Slave select output enabled, RXNE IT, 16-bit Rx fifo */
  /* (3) Enable SPI1 */
  SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR_2; /* (100b) */
  SPI2->CR2 = SPI_CR2_SSOE | SPI_CR2_RXNEIE | SPI_CR2_FRXTH | SPI_CR2_DS; /* (2) */
  SPI1->CR1 |= SPI_CR1_SPE; /* (3) */
 
  /* Configure IT */
  /* (4) Set priority for SPI1_IRQn */
  /* (5) Enable SPI1_IRQn */
//  NVIC_SetPriority(SPI1_IRQn, 0); /* (4) */
//  NVIC_EnableIRQ(SPI1_IRQn); /* (5) */ 
}

void SPI_10Hz(void)
{ //ждём пока опустошится Tx буфер
	while(!(SPI1->SR & SPI_SR_TXE));
	if(SSelect.x.bSlave_1)
		GPIOC->BSRR |= GPIO_BSRR_BR_0; /* PC0 = 0 (Set SSelect_1 ) */
	if(SSelect.x.bSlave_2)
		GPIOC->BSRR |= GPIO_BSRR_BR_1; /* PC1 = 0 (Set SSelect_2 ) */
	SPI1->DR = 0xaa55;  //отправляем данные 
	while(!(SPI1->SR & SPI_SR_RXNE)); /* ожидание окончания передачи*/
	GPIOC->BSRR |= GPIO_BSRR_BS_0; /* PC0 = 1 (Res SSelect_1 ) */
	GPIOC->BSRR |= GPIO_BSRR_BS_1; /* PC1 = 1 (Res SSelect_2 ) */
}
/******************************************************************************/
/*            Cortex-M0 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
void SVC_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
void PendSV_Handler(void)
{
}

/**
  * @brief  This function handles SysTick Handler.
  *         It toggles the green led if the action has been performed correctly
  *         and toggles the orange led coding the error number
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{ msTicks++;
//*** работа таймеров *************************
	if( msTicks%5 == 0 )
	{	flag_msTicks = 1;
		Timer_Handler();
		if( (count_msec++)%1000 == 0)
		{	count_sec++; count_msec = 0;
			// Запуск 16-ти преобразований раз в секунду.
			EnableADC(); 
			for(CurrentSample = 0; CurrentSample < NUMBER_OF_ADC_SAMPLE; CurrentSample++)
			{	ADC_array[CurrentSample] = 0x0000;
			}
			ADC_Value = 0x0000;
			ADC1->CR |= ADC_CR_ADSTART; /*  */
		}
	} /* if( msTicks%5 == 0 )*/
}


/******************************************************************************/
/*                 STM32F0xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f0xx.s).                                               */
/******************************************************************************/

void ADC1_COMP_IRQHandler(void)
{
  if ((ADC1->ISR & (ADC_ISR_EOC | ADC_ISR_OVR)) == 0) /* checks if one the expected flag is set */
  {
    error |= ERROR_UNEXPECTED_ADC_IT; /* Report an error */
  }
  else
  {
    if ((ADC1->ISR & ADC_ISR_OVR) != 0)  /* checks OVR has triggered the IT */
    {
      ADC1->ISR |= ADC_ISR_EOC | ADC_ISR_EOSEQ | ADC_ISR_OVR; /* clears all pending flags */
      //ADC1->CR |= ADC_CR_ADSTP; /* stop the sequence conversion */
      /* the data in the DR is considered as not valid */
    }
    else
    {
      if ((ADC1->ISR & ADC_ISR_EOSEQ) != 0)  /* checks EOSEQ has triggered the IT */
      {
        ADC1->ISR |= ADC_ISR_EOSEQ; /* clears the pending bit */
        //CurrentChannel = 0; /* reinitialize the CurrentChannel */
      }
      if ((ADC1->ISR & ADC_ISR_EOC) != 0)  /* checks EOC has triggered the IT */
      {
        ADC_array[CurrentSample] = ADC1->DR; /* reads data and clears EOC flag */
        ADC_Value += ADC_array[CurrentSample];
				ADC_Value -= ADC_array[(CurrentSample+1) % NUMBER_OF_ADC_SAMPLE];
				CurrentSample++;  										/* increments the index on ADC_array */        

				if(CurrentSample == NUMBER_OF_ADC_SAMPLE)
				{	DisableADC();		}

				CurrentSample = CurrentSample % NUMBER_OF_ADC_SAMPLE; 
				//ADC1->CR |= ADC_CR_ADSTART; 					/* ????????*/
      }
    }
  }
}

/**
  * @brief  This function handles SPI1 interrupt request.
  * @param  None
  * @retval None
  */
void SPI1_IRQHandler(void)
{ NVIC_DisableIRQ(SPI1_IRQn); /* Disable SPI1_IRQn */
}


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
