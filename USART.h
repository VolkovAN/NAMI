/**
  ******************************************************************************
  * @file    Табло.h 
  * @author  Волков А.Н.
  * @version V1.0.0
  * @date    13-Июня-2016
  *
  */

#ifndef USART_SECTION /*-----------------------------------*/

	#define	USART_SECTION
	#include "stm32f0xx.h"
	/* Global define ------------------------------------------------------------*/
		#define TBUF_SIZE   250          /*** Must be one of these powers of 2 (2,4,8,16,32,64,128) ***/
		#define RBUF_SIZE   500          /*** Must be one of these powers of 2 (2,4,8,16,32,64,128) ***/
		#define BAUDRATE 		38400UL   		// скорость usart
		//		#define TRANSMIT_ITERRUPT /* Работа секции передатчика по прерыванию*/

//		#define TBUF_SPACE  xdata       /*** Memory space where the transmit buffer resides ***/
//		#define RBUF_SPACE  xdata       /*** Memory space where the receive buffer resides ***/

		//sbit	B_UART_ERR0R_CRC				=STATUS_UART^0;	/* bit 0 of STATUS_UART */
		//sbit	B_UART_R_TIMEOUT				=STATUS_UART^1;	/* bit 1 of STATUS_UART */
		//sbit	B_UART_T_COMPLEAT	 			=STATUS_UART^2;	/* bit 2 of STATUS_UART */
		//sbit	B_UART_R_COMPLEAT				=STATUS_UART^3;	/* bit 3 of STATUS_UART */
		//sbit	B_UART_T_BUFF_NOT_EMPTY			=STATUS_UART^4;	/* bit 4 of STATUS_UART */
		//sbit	B_UART_R_BUFF_NOT_EMPTY			=STATUS_UART^5;	/* bit 5 of STATUS_UART */
		//sbit	B_UART_R_BUFF_OVERFLOW			=STATUS_UART^6;	/* bit 6 of STATUS_UART */
		//sbit	B_UART_CONNECT					=STATUS_UART^7;	/* bit 7 of STATUS_UART */

		//unsigned	char 	bdata		STATUS_TRAN;
		//sbit	B_TRANSACTION_WAIT_REPLAY		=STATUS_TRAN^0;
		//sbit	B_TRANSACTION_TIMEOUT			=STATUS_TRAN^1;
		//sbit	B_TRANSACTION_NOT_REPLAY		=STATUS_TRAN^2;
		//sbit	B_TRANSACTION_BAD_REPLAY		=STATUS_TRAN^3;
		//sbit	B_TRANSACTION_END				=STATUS_TRAN^4;
		//sbit	B_TRANSACTION_WAIT_REPLAY_P		=STATUS_TRAN^5;
		//sbit	B_TRANSACTION_6		=STATUS_TRAN^6;
		//sbit	B_TRANSACTION_OK				=STATUS_TRAN^7;

	/* Private define ------------------------------------------------------------*/

	/* Global typedef -----------------------------------------------------------*/
		typedef struct	Interface_Serial
		{	USART_TypeDef		  	*pCR;
			volatile	 unsigned	char		*pBR;
			volatile	 unsigned	char		*pBT;
			unsigned 	int					r_in;
			unsigned 	int        r_out;
			unsigned 	int        t_in;
			unsigned 	int        t_out;
			unsigned	char        ReciveBufferOverFlow;
			unsigned	char        TransmitBufferOverFlow;
			unsigned 	int        LengthPackageRecive;
			unsigned 	int        LengthPackageTransmit;
			unsigned	char        CountAttempt;
			unsigned	long        Baudrate;
//			void (*p_f_INTER)(struct	Interface_Serial*);	//
		}InterfaceUSART;
	/* Private typedef -----------------------------------------------------------*/
	/* Global macro -------------------------------------------------------------*/
	/* Private macro -------------------------------------------------------------*/
	/* Global variables ---------------------------------------------------------*/
	/* Private variables ---------------------------------------------------------*/
	/* Global function prototypes -----------------------------------------------*/
		/*__INLINE*/ void Configure_GPIO_USART1(void);
		/*__INLINE */void Configure_USART1(void);
		void Init_USART(void);
		void Clear_USART(void);
	/* Private function prototypes -----------------------------------------------*/

#endif	


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
