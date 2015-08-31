#ifndef KIT_INFO2_H_
#define KIT_INFO2_H_

	#include <stdio.h>										//para display LCD
	#include <stdlib.h>
	#include <string.h>
	#include "chip.h"
	#include "board.h"
	#include "RegsLPC1769.h"

	#include "main.h"

    #define PORT0     0
    #define PORT1     1
    #define PORT2     2
    #define PORT3     3
    #define PORT4     4


    // prototipos
    void InicializarKit (void);
    void InitGPIOs (void);
    void UART0_Init(void);
    void UART1_Init(void);


    void SetPINSEL (uint8_t , uint8_t , uint8_t );
    void SetPINMODE (uint8_t , uint8_t , uint8_t );
    void SetDIR (uint8_t , uint8_t , uint8_t );
    void SetPIN (uint8_t , uint8_t , uint8_t );
    uint8_t GetPIN (uint8_t , uint8_t , uint8_t );

    // Teclado --------------------------------------------------------------------------------------------------
    void Teclado_SW(uint8_t);			//antirrebote llamado desde SysTickTimer
    uint8_t Teclado_HW(void);		//driver
    uint8_t Teclado (void);			//primitiva
    void DriverTecladoSWfijo(uint8_t);
    void DriverTecladoFijo(void);
    uint8_t TecladoFijo(void);

    void getVelocidadViento ( void );
    // LCD ------------------------------------------------------------------------------------------------------
	void Display_lcd ( char * , char  , char  );
	void Borro_LCD(void);
	void Dato_LCD ( void );
	void Inic_LCD ( void );
	char PushLCD ( char , char );
	uint32_t PopLCD ( void );
	void Config_LCD( void );

    // Comunicacion Serie ------------------------------------------------------------------------------------------------------
    void Transmitir (char*);
    void PushRX (uint8_t);
    int PopTX (void);
    int PopRX (void);
    void PushTX (uint8_t);

    void Mensajes(void);


    void ADC_Inicializacion(void);
    void Inicializar_ADC(uint8_t);
    uint16_t readADC0(void);
    //      ----------------------------------------------------------------------------------------------------
    void SysTickInic(void);

    // Veleta ------------------------------------------------------------------------------------------------------
    uint8_t GetVeleta (void);

    //Identificación de los puertos de expansion:
    #define EXPANSION0		PORT2,7
    #define EXPANSION1		PORT1,29
    #define EXPANSION2		PORT4,28
    #define EXPANSION3		PORT1,23
    #define EXPANSION4		PORT1,20
    #define EXPANSION5		PORT0,19
    #define EXPANSION6		PORT3,26
    #define EXPANSION7		PORT1,25
    #define EXPANSION8		PORT1,22
    #define EXPANSION9		PORT1,19
    #define EXPANSION10		PORT0,20
    #define EXPANSION11		PORT3,25
    #define EXPANSION12		PORT1,27
    #define EXPANSION13		PORT1,24
    #define EXPANSION14		PORT1,21
    #define EXPANSION15		PORT1,18
    #define EXPANSION16		PORT1,31
    #define EXPANSION17		PORT0,24
    #define EXPANSION18		PORT0,25
    #define EXPANSION19		PORT0,17
    #define EXPANSION20		PORT1,31
    #define EXPANSION21		PORT0,22
    #define EXPANSION22		PORT0,15
    #define EXPANSION23		PORT0,16
    #define EXPANSION24		PORT2,8
    #define EXPANSION25		PORT2,12
    #define EXPANSION26		PORT1,31
    #define EXPANSION27		PORT1,31


	#define ENTRADA 0
	#define SALIDA  1

	//Leds (Reles)
	#define		LED1				PORT2,0 		//GPIO2
	#define		LED2				PORT0,23		//GPIO0
	#define		LED3				PORT0,21		//GPIO0
	#define		LED4				PORT0,27		//GPIO0

	//Led RGB:
	#define		RGBR				PORT2,2			//GPIO2
	#define		RGBG				PORT2,3			//GPIO2
	#define		RGBB				PORT2,1			//GPIO2

	#define		RED_ON			SetPIN(RGBR, ON)
	#define		GREEN_ON		SetPIN(RGBG, ON)
	#define		BLUE_ON			SetPIN(RGBB, ON)

	#define		RED_OFF				SetPIN(RGBR, OFF)
	#define		GREEN_OFF			SetPIN(RGBG, OFF)
	#define		BLUE_OFF			SetPIN(RGBB, OFF)

	#define		RGB_OFF				RED_OFF;GREEN_OFF;BLUE_OFF
	#define		RGB_ON				RED_ON;GREEN_ON;BLUE_ON

    #define		BUZZ				PORT0,28		//GPIO0

	#define		STICK_LED			PORT0,22		//LED STICK

	//Teclas (teclado 4x1)
	#define		KEY0				PORT2,10		//GPIO2
    #define     SW1         		PORT2,10
	#define		KEY1				PORT0,18		//GPIO0
    #define     SW4         		PORT0,18
	#define		KEY2				PORT0,11		//GPIO0
    #define     SW7         		PORT0,11
	#define		KEY3				PORT2,13		//GPIO2
    #define     SW10        		PORT2,13

	#define		KEY_RC				PORT1,26		//GPIO1
    #define     SW13        		PORT1,26

	#define 	NO_KEY				0xFF
	#define		CANTIDADdeREBOTES	10

	//Entradas digitales:
	#define		IN0					PORT4,29	//GPIO4
	#define		IN1					PORT2,11	//GPIO2

	//GPIOS LCD:
	#define		LCD_D4				PORT0,5			//GPIO0
	#define		LCD_D5				PORT0,10		//GPIO0
	#define		LCD_D6				PORT2,4			//GPIO2
	#define		LCD_D7				PORT2,5			//GPIO2

	#define		LCD_RS				PORT2,6			//GPIO2
	#define		LCD_BF				PORT1,28		//GPIO1
	#define		LCD_E				PORT0,4			//GPIO0


    //GPIOS Comunicación serie
	#define 	TX0					PORT0, 2	//Tx de la UART0
	#define 	RX0					PORT0, 3	//Rx de la UART0
	#define 	TX1					PORT0, 15	//Tx de la UART1
	#define 	RX1					PORT0, 16	//Rx de la UART1
	#define		FUNC_TXD0			1
	#define		FUNC_RXD0			1
	#define		FUNC_TXD1			1
	#define		FUNC_RXD1			1

    //GPIOS Veleta
	#define		BIT0_VELETA			PORT2, 7		//Expansion 0
	#define		BIT1_VELETA			PORT1, 29		//Expansion 1
	#define		BIT2_VELETA			PORT4, 28		//Expansion 2

	#define		PCUART0_POWERON		( 0x01 << 3 )
	#define		PCUART1_POWERON		( 0x01 << 4 )
	#define		SYSTEMCORECLOCK		100000000			//100MHz

  	//Defines generales:
	#define		OFF					0
	#define		ON					1

    #define     ACTIVO_ALTO    		1
    #define     ACTIVO_BAJO    		0

	#define		BUZZ_ON				0
	#define		BUZZ_OFF			1

	//Defines LCD:
	#define 	LARGO_RENGLON		25
	#define 	RENGLON_1			0
	#define		RENGLON_2			1
	#define		LCD_CONTROL			1
	#define		LCD_DATA			0
	#define 	TOPE_BUFFER_LCD		160

    //Defines Comunicacion Serie:
	#define MAX_RX	30
	#define MAX_TX	30



    //Nuevo
	#define		TICKS			40
	#define		DECIMAS			10
	#define		SEGUNDOS		10
	#define		MINUTOS			60


#endif /* KIT_INFO2_H_ */
