#define __RW	volatile
#define __R		volatile

#ifndef REGS_H_
#define REGS_H_


	//!< GPIO - PORT0
	/*	*						*
		*************************
		*		FIODIR			*	0x2009C000
		*************************
		*		RESERVED		*	0x2009C004
		*************************
		*		RESERVED		*	0x2009C008
		*************************
		*		RESERVED		*	0x2009C00C
		*************************
		*		FIOMASK			*	0x2009C010
		*************************
		*		FIOPIN			*	0x2009C014
		*************************
		*		FIOSET			*	0x2009C018
		*************************
		*		FIOCLR			*	0x2009C01C
		*************************
		*						*
		*						*
	*/
	#define		FIO0DIR		( * ( (__RW uint32_t * ) 0x2009C000UL ) )
	#define		FIO0MASK	( * ( (__RW uint32_t * ) 0x2009C010UL ) )
	#define		FIO0PIN		( * ( (__RW uint32_t * ) 0x2009C014UL ) )
	#define		FIO0SET		( * ( (__RW uint32_t * ) 0x2009C018UL ) )
	#define		FIO0CLR		( * ( (__W uint32_t * ) 0x2009C01CUL ) )

    #define    GPIO   ( (uint32_t  * ) 0x2009C000UL ) //0x2009C000UL : Direccion de inicio del GPIO0


	//!< ////////////////Registros PINSEL//////////////////////////////
	//!< 00	GPIO (reset value)		01	funcion 1
	//!< 11	funcion 3				10	funcion 2

	//Registros generales:
	#define		PINSEL		( ( uint32_t  * ) 0x4002C000UL )
	#define		PINSEL0		( * ( (__RW uint32_t * ) 0x4002C000UL ) ) //!< P0[15:0]
	#define		PINSEL1		( * ( (__RW uint32_t * ) 0x4002C004UL ) ) //!< P0[31:16]

	//!< ----------- Estados de PINSEL:
	#define		PINSEL_GPIO			0
	#define		PINSEL_FUNC1		1
	#define		PINSEL_FUNC2		2
	#define		PINSEL_FUNC3		3

	#define		FUNCION_GPIO	0
	#define		FUNCION_1		1
	#define		FUNCION_2		2
	#define		FUNCION_3		3

	//!< //////////////////Registros PINMODE ///////////////////////////
    #define		PINMODE		( ( uint32_t  * ) 0x4002C040UL )
    #define		PINMODE0	( * ( (__RW uint32_t * ) 0x4002C040UL ) )	//!< P0[15:0]
	#define		PINMODE1	( * ( (__RW uint32_t * ) 0x4002C044UL ) )	//!< P0[31:16]

	//!< ----------- Estados de PINMODE
	//!< 00	Pull Up resistor enable (reset value)		01	repeated mode enable
	//!< 11	Pull Down resistor enable					10	ni Pull Up ni Pull DOwn
	#define		PINMODE_PULLUP 		0
	#define		PINMODE_REPEAT 		1
	#define		PINMODE_NONE 		2
	#define		PINMODE_PULLDOWN 	3

	//!< ///////////////////   PCONP   //////////////////////////
	//!<  Power Control for Peripherals register (PCONP - 0x400F C0C4) [pag. 62 user manual LPC1769]
//	#define 	PCONP	(* ( ( __RW uint32_t  * ) 0x400FC0C4UL ))


	//!< ///////////////////   PCLKSEL   //////////////////////////
	//!< Peripheral Clock Selection registers 0 and 1 (PCLKSEL0 -0x400FC1A8 and PCLKSEL1 - 0x400F C1AC) [pag. 56 user manual]
	#define		PCLKSEL_		( ( __RW uint32_t  * ) 0x400FC1A8UL )



	//!< Registros PCLKSEL
	#define		PCLKSEL0_	PCLKSEL_[0]
	#define		PCLKSEL1_	PCLKSEL_[1]

	#define PCONP_		((__RW uint32_t * )	0x400FC0C4UL)
	#define PCONP0_		PCONP_[0]
	//Registros del SysTick:

	//0xE000E010UL: Registro de control del SysTick:
	#define DIR_STCTRL		( (uint32_t *) 0xE000E010UL)
	//0xE000E014UL: Registro de recarga del SysTick:
	#define DIR_STRELOAD	( (uint32_t *) 0xE000E014UL)
	//0xE000E018UL: Registro de cuenta del SysTick:
	#define DIR_STCURR		( (uint32_t *) 0xE000E018UL)
	//0xE000E01CUL: Registro de calibracion del SysTick:
	#define DIR_STCALIB		( (uint32_t *) 0xE000E01CUL)

	//0xE000E100UL : Direccion de inicio de los registros de habilitación (set) de interrupciones en el NVIC:
	#define		ISER		( ( uint32_t * ) 0xE000E100UL )
	//0xE000E180UL : Direccion de inicio de los registros de deshabilitacion (clear) de interrupciones en el NVIC:
	#define		ICER		( (  uint32_t * ) 0xE000E180UL )


	//Registros ISER: Para habilitar las Interupciones Se activan con 1 Escribiendo un 0 no hace nada
	#define		ISER0		ISER[0]
	#define		ISER1		ISER[1]
	#define		ISE_UART0	ISER[0]	|= (0x00000001 <<  5)  //ISER0->bit5 pongo un 1 en el bit  para habilitar la INT UART0
//	#define		ISE_UART1	ISER[1]	|= (0x40)  //ISER0->bit5 pongo un 1 en el bit  para habilitar la INT UART0
	#define		ISE_UART1	ISER[0]	|= (0x00000001 <<  6)  //ISER0->bit5 pongo un 1 en el bit  para habilitar la INT UART1
	#define		ISE_EINT3	ISER[0] |= (0x00000001 << 21)  //ISER0->bit21 pongo un 1 en el bit 21 para habilitar la INT EINT3
	#define     ISE_EINT2	ISER[0] |= (0x00000001 << 20)  //ISER0->bit20 pongo un 1 en el bit 20 para habilitar la INT EINT2
	#define     ISE_EINT1	ISER[0] |= (0x00000001 << 19)  //ISER0->bit19 pongo un 1 en el bit 19 para habilitar la INT EINT1
	#define     ISE_EINT0	ISER[0] |= (0x00000001 << 18)  //ISER0->bit18 pongo un 1 en el bit 18 para habilitar la INT EINT1
	#define     ISE_TIMER0  ISER[0] |= (0x00000001 <<  1)  //ISER0->bit1 pongo un 1 en el bit 1 para habilitar la INT TIMER0
	#define     ISE_TIMER1  ISER[0] |= (0x00000001 <<  2)  //ISER0->bit2 pongo un 1 en el bit 2 para habilitar la INT TIMER1


	//Systick:
		typedef struct
		{
			union
			{
				__RW uint32_t  _STCTRL ;                         /*!< Offset: 0x00  SysTick Control and Status Register */
				struct
				{
					__RW uint32_t	_ENABLE:1;
					__RW uint32_t	_TICKINT:1;
					__RW uint32_t	_CLKSOURCE:1;
					__RW uint32_t	_RESERVED0:14;
					__RW uint32_t	_COUNTFLAG:1;
					__RW uint32_t	_RESERVED1:14;
				};
			};
			__RW uint32_t _STRELOAD;                   /*!< Offset: 0x04  SysTick Reload Value Register       */
			__RW uint32_t _STCURR ;                    /*!< Offset: 0x08  SysTick Current Value Register      */

			union
			{
				__R uint32_t _STCALIB;
				struct
				{
					__R uint32_t _TENMS: 24;
					__R uint32_t _OTHERS: 8;
				};

			};
		} systick_t;


		#define systick             ((systick_t *)       (0xE000E010))

		#define	MAX_TICKS	0x00fffffful

		#define STCTRL			systick->_STCTRL
			#define ENABLE		systick->_ENABLE
			#define TICKINT		systick->_TICKINT
			#define CLKSOURCE	systick->_CLKSOURCE
			#define COUNTFLAG	systick->_COUNTFLAG

		#define STRELOAD		systick->_STRELOAD
		#define STCURR 	    	systick->_STCURR

		#define STCALIB 		systick->_STCALIB
			#define TENMS		systick->_TENMS
			#define OTHERS		systick->_OTHERS


	// para los funciones de los Timers por soft
	#define TICKINT_ON    STCTRL |= 0x00000002 // pongo en 1  el bit 1 del STCTRL Hablito   interr del Systick
	#define TICKINT_OFF   STCTRL &= 0xFFFFFFFD // pongo en 0  el bit 1 del STCTRL Deshablito interr del Systick


	//Registros de la UART0:

	//0x4001000CUL : Registro de control de la UART0:
	#define		DIR_U0LCR		( ( uint32_t  * ) 0x4000C00CUL )
	//0x40010014UL : Registro de recepcion de la UART0:
	#define		DIR_U0LSR		( ( uint32_t  * ) 0x4000C014UL )
	//0x40010000UL : Parte baja del divisor de la UART0:
	#define		DIR_U0DLL		( ( uint32_t  * ) 0x4000C000UL )
	//0x40010004UL : Parte alta del divisor de la UART0:
	#define		DIR_U0DLM		( ( uint32_t  * ) 0x4000C004UL )
	#define		DIR_U0IER		( ( uint32_t  * ) 0x4000C004UL )
		#define		IER_RBR		0
		#define		IER_THRE	1
	//0x40010000UL : Registro de recepcion de la UART0:
	#define		DIR_U0RBR		( ( uint32_t  * ) 0x4000C000UL )
	//0x40010000UL : Registro de transmision de la UART0:
	#define		DIR_U0THR		( ( uint32_t  * ) 0x4000C000UL )
	#define		DIR_U0IIR 		( ( uint32_t  * ) 0x4000C008UL )

	#define		U0THR		DIR_U0THR[0]
	#define		U0RBR		DIR_U0RBR[0]
	#define		U0LCR		DIR_U0LCR[0]
	#define		U0LSR		DIR_U0LSR[0]
	#define		U0DLL		DIR_U0DLL[0]
	#define		U0DLM		DIR_U0DLM[0]
	#define		U0IER		DIR_U0IER[0]
	#define		U0IIR		*DIR_U0IIR

	#define		PCLK_UART0	6

	#define		U0RDR		(U0LSR&0x01)
	#define		U0THRE		((U0LSR&0x20)>>5)

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//0x4000C008UL : Registro de la FIFO
	#define		DIR_U0FCR		( ( uint32_t  * ) 0x4000C008UL )
	#define		U0FCR		DIR_U0FCR[0]

	//------------------------------------------------------------------------------------------
	//Registros de la UART1:

	//0x4001000CUL : Registro de control de la UART0:
	#define		DIR_U1LCR		( ( uint32_t  * ) 0x4001000CUL )
	//0x40010014UL : Registro de recepcion de la UART0:
	#define		DIR_U1LSR		( ( uint32_t  * ) 0x40010014UL )
	//0x40010000UL : Parte baja del divisor de la UART0:
	#define		DIR_U1DLL		( ( uint32_t  * ) 0x40010000UL )
	//0x40010004UL : Parte alta del divisor de la UART0:
	#define		DIR_U1DLM		( ( uint32_t  * ) 0x40010004UL )
	#define		DIR_U1IER		( ( uint32_t  * ) 0x40010004UL )
		#define		IER_RBR		0
		#define		IER_THRE	1
	//0x40010000UL : Registro de recepcion de la UART0:
	#define		DIR_U1RBR		( ( uint32_t  * ) 0x40010000UL )
	//0x40010000UL : Registro de transmision de la UART0:
	#define		DIR_U1THR		( ( uint32_t  * ) 0x40010000UL )
	#define		DIR_U1IIR 		( ( uint32_t  * ) 0x40010008UL )

	#define		U1THR		DIR_U1THR[0]
	#define		U1RBR		DIR_U1RBR[0]
	#define		U1LCR		DIR_U1LCR[0]
	#define		U1LSR		DIR_U1LSR[0]
	#define		U1DLL		DIR_U1DLL[0]
	#define		U1DLM		DIR_U1DLM[0]
	#define		U1IER		DIR_U1IER[0]
	#define		U1IIR		*DIR_U1IIR

	#define		PCLK_UART1	8

	#define		U1RDR		(U1LSR&0x01)
	#define		U1THRE		((U1LSR&0x20)>>5)

	//++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
	//0x4000C008UL : Registro de la FIFO
	#define		DIR_U1FCR		( ( uint32_t  * ) 0x40010008UL )
	#define		U1FCR		DIR_U1FCR[0]


















	//Estructura para manejar los Timers:

	//Timers:
	//0x40004000UL : Direccion de inicio de los registros del Timer0
	#define		DIR_TIMER0		( (__RW timer_t  * ) 0x40004000UL )
	//0x40008000UL : Direccion de inicio de los registros del Timer1
	#define		DIR_TIMER1		( (__RW timer_t  * ) 0x40008000UL )
	//0x40090000UL : Direccion de inicio de los registros del Timer2
	#define		DIR_TIMER2		( ( __RW timer_t  * ) 0x40090000UL )
	//0x40094000UL : Direccion de inicio de los registros del Timer3
	#define		DIR_TIMER3		( (__RW timer_t  * ) 0x40094000UL )

	//Registros de timers:
	#define		TIMER0		DIR_TIMER0[0]
	#define		TIMER1		DIR_TIMER1[0]
	#define		TIMER2		DIR_TIMER2[0]
	#define		TIMER3		DIR_TIMER3[0]

	 // Timer 0
	#define		T0IR		TIMER0.IR
	#define     T0_IR_MR0   T0IR & 0X01       // Bit 0 del registro IR corresponde al flag de la Int por Match 0
	#define     T0_IR_MR1   (T0IR & (0X01 <<1))>>1 // Bit 1 del registro IR corresponde al flag de la Int por Match 1
	#define     T0_IR_MR2   (T0IR & (0X01 <<2))>>2 // Bit 2 del registro IR corresponde al flag de la Int por Match 2
	#define     T0_IR_MR3   (T0IR & (0X01 <<3))>>3 // Bit 3 del registro IR corresponde al flag de la Int por Match 3
	#define     T0_IR_CR0   (T0IR & (0X01 <<1))>>4 // Bit 4 del registro IR corresponde al flag de la Int por CAPTURE 0
	#define     T0_IR_CR1   (T0IR & (0X01 <<1))>>5 // Bit 5 del registro IR corresponde al flag de la Int por CAPTURE 1

	//Timer 1
	#define		T1IR		TIMER1.IR
	#define     T1_IR_MR0   T1IR & 0X01       // Bit 0 del registro IR corresponde al flag de la Int por Match 0
	#define     T1_IR_MR1   (T1IR & (0X01 <<1))>>1 // Bit 1 del registro IR corresponde al flag de la Int por Match 1
	#define     T1_IR_MR2   (T1IR & (0X01 <<2))>>2 // Bit 2 del registro IR corresponde al flag de la Int por Match 2
	#define     T1_IR_MR3   (T1IR & (0X01 <<3))>>3 // Bit 3 del registro IR corresponde al flag de la Int por Match 3
	#define     T1_IR_CR0   (T1IR & (0X01 <<1))>>4 // Bit 4 del registro IR corresponde al flag de la Int por CAPTURE 0
	#define     T1_IR_CR1   (T1IR & (0X01 <<1))>>5 // Bit 5 del registro IR corresponde al flag de la Int por CAPTURE 1

	#define		T2IR		TIMER2.IR
	#define		T3IR		TIMER3.IR

	#define		T0TCR		TIMER0.TCR
	#define		T1TCR		TIMER1.TCR
	#define		T2TCR		TIMER2.TCR
	#define		T3TCR		TIMER3.TCR

	#define		T0PC		TIMER0.PC
	#define		T1PC		TIMER1.PC
	#define		T2PC		TIMER2.PC
	#define		T3PC		TIMER3.PC

	#define		T0MCR		TIMER0.MCR
	#define		T1MCR		TIMER1.MCR
	#define		T2MCR		TIMER2.MCR
	#define		T3MCR		TIMER3.MCR

	#define		T0MR0		TIMER0.MR0
	#define		T1MR0		TIMER1.MR0
	#define		T2MR0		TIMER2.MR0
	#define		T3MR0		TIMER3.MR0

	#define		T0MR1		TIMER0.MR1
	#define		T1MR1		TIMER1.MR1
	#define		T2MR1		TIMER2.MR1
	#define		T3MR1		TIMER3.MR1

	#define		T0MR2		TIMER0.MR2
	#define		T1MR2		TIMER1.MR2
	#define		T2MR2		TIMER2.MR2
	#define		T3MR2		TIMER3.MR2

	#define		T0MR3		TIMER0.MR3
	#define		T1MR3		TIMER1.MR3
	#define		T2MR3		TIMER2.MR3
	#define		T3MR3		TIMER3.MR3

	#define		T0CCR		TIMER0.CCR
	#define		T1CCR		TIMER1.CCR
	#define		T2CCR		TIMER2.CCR
	#define		T3CCR		TIMER3.CCR

	#define		T0CR0		TIMER0.CR0
	#define		T1CR0		TIMER1.CR0
	#define		T2CR0		TIMER2.CR0
	#define		T3CR0		TIMER3.CR0

	#define		T0CR1		TIMER0.CR1
	#define		T1CR1		TIMER1.CR1
	#define		T2CR1		TIMER2.CR1
	#define		T3CR1		TIMER3.CR1

	#define		T0EMR		TIMER0.EMR
	#define		T1EMR		TIMER1.EMR
	#define		T2EMR		TIMER2.EMR
	#define		T3EMR		TIMER3.EMR
	#define     EMCO_TOGGLE  T1EMR|=(0x3 << 4) // Configurado como TOGGLE el pin MAT1.0
	#define     EMC1_TOGGLE  T1EMR|=(0x3 << 6) //  Configurado como TOGGLE el pin MAT1.1
	#define     T1PR         TIMER1.PR
	#define     T0PR         TIMER0.PR

#endif /* REGS_H_ */
