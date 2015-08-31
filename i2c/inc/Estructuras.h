#ifndef _registros
	#include "registros.h"
	#define		_registros
#endif

//-----------------------------------PCONP----------------------------------------//

typedef struct
{
	__RW uint32_t	_RESERVED1:1;
	__RW uint32_t	_PCTIM0:1;
	__RW uint32_t	_PCTIM1:1;
	__RW uint32_t	_PCUART0:1;
	__RW uint32_t  	_PCUART1:1;
	__RW uint32_t	_RESERVED2:1;
	__RW uint32_t 	_PCPWM1:1;
	__RW uint32_t 	_PCI2C0:1;
	__RW uint32_t 	_PCSPI:1;
	__RW uint32_t 	_PCRTC:1;
	__RW uint32_t  	_PCSSP1:1;
	__RW uint32_t 	_RESERVED3:1;
	__RW uint32_t 	_PCADC:1;
	__RW uint32_t  	_PCCAN1:1;
	__RW uint32_t 	_PCCAN2:1;
	__RW uint32_t 	_PCGPIO:1;
	__RW uint32_t  	_PCRIT:1;
	__RW uint32_t 	_PCMCPWM:1;
	__RW uint32_t 	_PCQEI:1;
	__RW uint32_t  	_PCI2C1:1;
	__RW uint32_t 	_RESERVED4:1;
	__RW uint32_t 	_PCSSP0:1;
	__RW uint32_t 	_PCTIM2:1;
	__RW uint32_t 	_PCTIM3:1;
	__RW uint32_t  	_PCUART2:1;
	__RW uint32_t 	_PCUART3:1;
	__RW uint32_t  	_PCI2C2:1;
	__RW uint32_t 	_PCI2S:1;
	__RW uint32_t 	_RESERVED5:1;
	__RW uint32_t 	_PCGPDMA:1;
	__RW uint32_t 	_PCENET:1;
	__RW uint32_t  	_PCUSB:1;
} pconp_t;

#define PCONP             ((pconp_t *)       (0x400FC0C4UL))

#define		PCTIM0			PCONP->_PCTIM0;
#define		PCTIM1			PCONP->_PCTIM1;
#define		PCUART0			PCONP->_PCUART0;
#define 	PCUART1			PCONP->_PCUART1;
#define 	PCPWM1			PCONP->_PCPWM1;
#define 	PCI2C0			PCONP->_PCI2C0;
#define 	PCSPI			PCONP->_PCSPI;
#define 	PCRTC			PCONP->_PCRTC;
#define 	PCSSP1			PCONP->_PCSSP1;
#define 	PCADC			PCONP->_PCADC;
#define 	PCCAN1			PCONP->_PCCAN1;
#define 	PCCAN2			PCONP->_PCCAN2;
#define 	PCGPIO			PCONP->_PCGPIO;
#define 	PCRIT			PCONP->_PCRIT;
#define 	PCMCPWM			PCONP->_PCMCPWM;
#define 	PCQEI			PCONP->_PCQEI;
#define 	PCI2C1			PCONP->_PCI2C1;
#define 	PCSSP0			PCONP->_PCSSP0;
#define 	PCTIM2			PCONP->_PCTIM2;
#define 	PCTIM3			PCONP->_PCTIM3;
#define 	PCUART2			PCONP->_PCUART2;
#define 	PCUART3			PCONP->_PCUART3;
#define 	PCI2C2			PCONP->_PCI2C2;
#define 	PCI2S			PCONP->_PCI2S;
#define 	PCGPDMA			PCONP->_PCGPDMA;
#define 	PCENET			PCONP->_PCENET;
#define 	PCUSB			PCONP->_PCUSB;


//--------------------------------PCLKSEL0----------------------------------------//

typedef struct
{
	union
	{
		__RW uint32_t  _PCLKSEL0;
		struct
		{
			__RW uint32_t _PCLK_WDT:2;
			__RW uint32_t _PCLK_TIMER0:2;
			__RW uint32_t _PCLK_TIMER1:2;
			__RW uint32_t _PCLK_UART0:2;
			__RW uint32_t _PCLK_UART1:2;
			__RW uint32_t _RESERVED1:2;
			__RW uint32_t _PCLK_PWM1:2;
			__RW uint32_t _PCLK_I2C0:2;
			__RW uint32_t _PCLK_SPI:2;
			__RW uint32_t _RESERVED2:2;
			__RW uint32_t _PCLK_SSP1:2;
			__RW uint32_t _PCLK_DAC:2;
			__RW uint32_t _PCLK_ADC:2;
			__RW uint32_t _PCLK_CAN1:2;
			__RW uint32_t _PCLK_CAN2:2;
			__RW uint32_t _PCLK_ACF:2;
		};
	};
	union
	{
		__RW uint32_t  _PCLKSEL1 ;
		struct
		{
			__RW uint32_t _PCLK_QEI:2;
			__RW uint32_t _PCLK_GPIOINT:2;
			__RW uint32_t _PCLK_PCB:2;
			__RW uint32_t _PCLK_I2C1:2;
			__RW uint32_t _RESERVED1:2;
			__RW uint32_t _PCLK_SSP0:2;
			__RW uint32_t _PCLK_TIMER2:2;
			__RW uint32_t _PCLK_TIMER3:2;
			__RW uint32_t _PCLK_UART2:2;
			__RW uint32_t _PCLK_UART3:2;
			__RW uint32_t _PCLK_I2C2:2;
			__RW uint32_t _PCLK_I2S:2;
			__RW uint32_t _RESERVED2:2;
			__RW uint32_t _PCLK_RIT:2;
			__RW uint32_t _PCLK_SYSCON:2;
			__RW uint32_t _PCLK_MC:2;
		};
	};


} pclksel_t;

#define		PCLKSEL		((pclksel_t*) 0x400FC1A8UL)

//PCLKSEL0
#define 	PCLK_WDT		PCLKSEL->_PCLK_WDT;
#define 	PCLK_TIMER0		PCLKSEL->_PCLK_TIMER0;
#define 	PCLK_TIMER1		PCLKSEL->_PCLK_TIMER1;
#define 	PCLK_UART0		PCLKSEL->_PCLK_UART0;
#define 	PCLK_UART1		PCLKSEL->_PCLK_UART1;
#define 	PCLK_PWM1		PCLKSEL->_PCLK_PWM1;
#define 	PCLK_I2C0		PCLKSEL->_PCLK_I2C0;
#define 	PCLK_SPI		PCLKSEL->_PCLK_SPI;
#define 	PCLK_SSP1		PCLKSEL->_PCLK_SSP1;
#define 	PCLK_DAC		PCLKSEL->_PCLK_DAC;
#define 	PCLK_ADC		PCLKSEL->_PCLK_ADC;
#define 	PCLK_CAN1		PCLKSEL->_PCLK_CAN1;
#define 	PCLK_CAN2		PCLKSEL->_PCLK_CAN2;
#define 	PCLK_ACF		PCLKSEL->_PCLK_ACF;

//PCLKSEL1
#define 	PCLK_QEI		PCLKSEL->_PCLK_QEI;
#define 	PCLK_GPIOINT	PCLKSEL->_PCLK_GPIOINT;
#define 	PCLK_PCB		PCLKSEL->_PCLK_PCB;
#define 	PCLK_I2C1		PCLKSEL->_PCLK_I2C1;
#define 	PCLK_SSP0		PCLKSEL->_PCLK_SSP0;
#define 	PCLK_TIMER2		PCLKSEL->_PCLK_TIMER2;
#define 	PCLK_TIMER3		PCLKSEL->_PCLK_TIMER3;
#define 	PCLK_UART2		PCLKSEL->_PCLK_UART2;
#define 	PCLK_UART3		PCLKSEL->_PCLK_UART3;
#define 	PCLK_I2C2		PCLKSEL->_PCLK_I2C2;
#define 	PCLK_I2S		PCLKSEL->_PCLK_I2S;
#define 	PCLK_RIT		PCLKSEL->_PCLK_RIT;
#define 	PCLK_SYSCON		PCLKSEL->_PCLK_SYSCON;
#define 	PCLK_MC			PCLKSEL->_PCLK_MC;


//--------------------------------ULCR----------------------------------------//

typedef struct
{
	__RW uint32_t	_WLS:2;
	__RW uint32_t	_SBS:1;
	__RW uint32_t	_PE:1;
	__RW uint32_t	_PS:2;
	__RW uint32_t  	_BC:1;
	__RW uint32_t	_DLAB:1;
	__RW uint32_t 	_RESERVED:24;
} ulcr_t;

//U0LCR
#define U0LCR             ((ulcr *)       (0x4000C00CUL))

#define		WLS_0		U0LCR->_WLS:2;
#define		SBS_0		U0LCR->_SBS:1;
#define		PE_0		U0LCR->_PE:1;
#define		PS_0		U0LCR->_PS:2;
#define 	BC_0		U0LCR->_BC:1;
#define		DLAB_0		U0LCR->_DLAB:1;


//U2LCR
#define U2LCR             ((ulcr *)       (0x4009800CUL))

#define		WLS_2		U2LCR->_WLS:2;
#define		SBS_2		U2LCR->_SBS:1;
#define		PE_2		U2LCR->_PE:1;
#define		PS_2		U2LCR->_PS:2;
#define 	BC_2		U2LCR->_BC:1;
#define		DLAB_2		U2LCR->_DLAB:1;

//U3LCR
#define U3LCR             ((ulcr *)       (0x4009C00CUL))

#define		WLS_3		U3LCR->_WLS:2;
#define		SBS_3		U3LCR->_SBS:1;
#define		PE_3		U3LCR->_PE:1;
#define		PS_3		U3LCR->_PS:2;
#define 	BC_3		U3LCR->_BC:1;
#define		DLAB_3		U3LCR->_DLAB:1;

//----------------------------UART DIVISION LATCH (UDLL Y UDLM)----------------------------------------//

typedef struct
{
	__RW uint32_t	_DLLSB:8;
	__RW uint32_t 	_RESERVED:24;
} udll_t;

typedef struct
{
	__RW uint32_t	_DLMSB:8;
	__RW uint32_t 	_RESERVED:24;
} udlm_t;

//U0DLL
#define U0DLL             ((udll *)       (0x4000C000UL))

#define		DLLSB_0		U0DLL->_DLLSB;

//U2DLL
#define U2DLL             ((udll *)       (0x40098000UL))

#define		DLLSB_2		U2DLL->_DLLSB;

//U3DLL
#define U3DLL             ((udll *)       (0x4009C000UL))

#define		DLLSB_3		U3DLL->_DLLSB;


//U0DLM
#define U0DLM             ((udll *)       (0x4000C004UL))

#define		DLMSB_0		U0DLM->_DLMSB;

//U2DLM
#define U2DLM             ((udll *)       (0x40098004UL))

#define		DLMSB_2		U2DLM->_DLMSB;

//U3DLM
#define U3DLM             ((udll *)       (0x4009C004UL))

#define		DLMSB_3		U3DLM->_DLMSB;


//----------------------------ISER----------------------------------------//

typedef struct
{
	union
	{
		__RW uint32_t  _ISER0 ;
		struct
		{
			__RW uint32_t	_ISE_WDT:1;
			__RW uint32_t	_ISE_TIMER0:1;
			__RW uint32_t	_ISE_TIMER1:1;
			__RW uint32_t	_ISE_TIMER2:1;
			__RW uint32_t  	_ISE_TIMER3:1;
			__RW uint32_t	_ISE_UART0:1;
			__RW uint32_t 	_ISE_UART1:1;
			__RW uint32_t 	_ISE_UART2:1;
			__RW uint32_t 	_ISE_UART3:1;
			__RW uint32_t 	_ISE_PWM:1;
			__RW uint32_t  	_ISE_I2C0:1;
			__RW uint32_t 	_ISE_I2C1:1;
			__RW uint32_t 	_ISE_I2C2:1;
			__RW uint32_t  	_ISE_SPI:1;
			__RW uint32_t 	_ISE_SSP0:1;
			__RW uint32_t 	_ISE_SSP1:1;
			__RW uint32_t  	_ISE_PLL0:1;
			__RW uint32_t 	_ISE_RTC:1;
			__RW uint32_t 	_ISE_EINT0:1;
			__RW uint32_t  	_ISE_EINT1:1;
			__RW uint32_t 	_ISE_EINT2:1;
			__RW uint32_t 	_ISE_EINT3:1;
			__RW uint32_t 	_ISE_ADC:1;
			__RW uint32_t 	_ISE_BOD:1;
			__RW uint32_t  	_ISE_USB:1;
			__RW uint32_t 	_ISE_CAN:1;
			__RW uint32_t  	_ISE_DMA:1;
			__RW uint32_t 	_ISE_I2S:1;
			__RW uint32_t 	_ISE_ENET:1;
			__RW uint32_t 	_ISE_RIT:1;
			__RW uint32_t 	_ISE_MCPWM:1;
			__RW uint32_t  	_ISE_QEI:1;
		};
	};
	union
	{
		__RW uint32_t  _ISER1 ;
		struct
		{
			__RW uint32_t	_ISE_PLL1:1;
			__RW uint32_t	_ISE_USBACT:1;
			__RW uint32_t	_ISE_CANACT:1;
			__RW uint32_t	_RESERVED:29;
		};
	};
} iser_t;

#define ISER             ((iser_t *)       (0xE000E100UL))

//ISER0
#define 	ISE_WDT		ISER->_ISE_WDT;
#define 	ISE_TIMER0	ISER->_ISE_TIMER0;
#define 	ISE_TIMER1	ISER->_ISE_TIMER1;
#define 	ISE_TIMER2	ISER->_ISE_TIMER2;
#define 	ISE_TIMER3	ISER->_ISE_TIMER3;
#define 	ISE_UART0	ISER->_ISE_UART0;
#define 	ISE_UART1	ISER->_ISE_UART1;
#define 	ISE_UART2	ISER->_ISE_UART2;
#define 	ISE_UART3	ISER->_ISE_UART3;
#define 	ISE_PWM		ISER->_ISE_PWM;
#define 	ISE_I2C0	ISER->_ISE_I2C0;
#define 	ISE_I2C1	ISER->_ISE_I2C1;
#define 	ISE_I2C2	ISER->_ISE_I2C2;
#define 	ISE_SPI		ISER->_ISE_SPI;
#define 	ISE_SSP0	ISER->_ISE_SSP0;
#define 	ISE_SSP1	ISER->_ISE_SSP1;
#define 	ISE_PLL0	ISER->_ISE_PLL0;
#define 	ISE_RTC		ISER->_ISE_RTC;
#define 	ISE_EINT0	ISER->_ISE_EINT0;
#define 	ISE_EINT1	ISER->_ISE_EINT1;
#define 	ISE_EINT2	ISER->_ISE_EINT2;
#define 	ISE_EINT3	ISER->_ISE_EINT3;
#define 	ISE_ADC		ISER->_ISE_ADC;
#define 	ISE_BOD		ISER->_ISE_BOD;
#define 	ISE_USB		ISER->_ISE_USB;
#define 	ISE_CAN		ISER->_ISE_CAN;
#define 	ISE_DMA		ISER->_ISE_DMA;
#define 	ISE_I2S		ISER->_ISE_I2S;
#define 	ISE_ENET	ISER->_ISE_ENET;
#define 	ISE_RIT		ISER->_ISE_RIT;
#define 	ISE_MCPWM	ISER->_ISE_MCPWM;
#define 	ISE_QEI		ISER->_ISE_QEI;

//ISER1
#define 	ISE_PLL1	ISER->_ISE_PLL1;
#define 	ISE_USBACT	ISER->_ISE_USBACT;
#define 	ISE_CANACT	ISER->_ISE_CANACT;

