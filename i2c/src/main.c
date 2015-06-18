/* Copyright 2015, Pablo Ridolfi
 *
 * This file is part of TD2-Template.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/** @brief This is a simple C example file.
 **
 **/

/** \addtogroup TD2 Técnicas Digitales II
 ** @{ */

/** @addtogroup App Aplicación de usuario
 * 	@{
 */

/*
 * Initials     Name
 * ---------------------------
 * PR           Pablo Ridolfi
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20150421 v0.0.1   PR first version
 */

/*==================[inclusions]=============================================*/

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

#include <cr_section_macros.h>

#include "main.h"
/*==================[macros and definitions]=================================*/
#define MPU6050_DEVICE_ADDRESS   0x68
#define MPU6050_RA_ACCEL_XOUT_H  0x3B
#define MPU6050_RA_PWR_MGMT_1    0x6B
#define MPU6050_PWR1_SLEEP_BIT   6
/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/** @brief hardware initialization function
 *	@return none
 */

/*==================[internal data definition]===============================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

static void initHardware(void)
{
#if defined (__USE_LPCOPEN)
#if !defined(NO_BOARD_LIB)
    // Read clock settings and update SystemCoreClock variable
    SystemCoreClockUpdate();
    // Set up and initialize all required blocks and
    // functions related to the board hardware
    Board_Init();
    // Set the LED to the state of "Off"
    Board_LED_Set(0, false);
#endif
#endif
    Board_I2C_Init(I2C1);

    /* pines del stick */
	Chip_IOCON_PinMux(LPC_IOCON, 0, 0, IOCON_MODE_INACT, IOCON_FUNC3);
	Chip_IOCON_PinMux(LPC_IOCON, 0, 1, IOCON_MODE_INACT, IOCON_FUNC3);
	Chip_IOCON_EnableOD(LPC_IOCON, 0, 0);
	Chip_IOCON_EnableOD(LPC_IOCON, 0, 1);

    Chip_I2C_SetClockRate(I2C1, 100000);
	Chip_I2C_SetMasterEventHandler(I2C1, Chip_I2C_EventHandlerPolling);
}

/*==================[external functions definition]==========================*/

//TODO: Leer 10 veces el registro de Acelerometro en X, esto nos dara 5 datos
//	  	compuestos de 2 bytes cada uno (parte high y low)


int main(void)
{
	/*==================[Inicializacion]==========================*/
	uint8_t wbuf[2] = {0,0};
	uint8_t rbuf[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint16_t samples[10] = {0,0,0,0,0,0,0,0,0,0}; //cada posicion es de 16 bits, necesario para guardar
												  //la parte low y high de las muestras de accel
	//uint32_t i;


	I2C_XFER_T xfer;

	int i =0;


	initHardware();


	/*==================[Configuracion del I2C_XFER_T]==========================*/

	//Escritura

	//Define el registro que se va a leer
	//wbuf[0] = MPU6050_RA_ACCEL_XOUT_H; //Parte high de la lectura en x del acelerometro
							//Como la lectura se realiza de forma secuencial, en la posicion 0
							//del rbuf ira este dato y en la posicion 1 ira la correspondiente a
							//la posicion siguiente, osea ACCEL_XOUT_L (0x3C)


	MPU6050_wakeup(&xfer);

	wbuf[0] = MPU6050_RA_ACCEL_XOUT_H;
	while(1)
	{
		I2C_XFER_T_config(&xfer, rbuf, 10, MPU6050_I2C_SLAVE_ADDRESS, 0, wbuf, 1);

			//De momento leer rbuf es lo mismo que leer xfer.rxBuff porque apuntan a la misma direccion
			samples[0]=(rbuf[0] << 8) | rbuf[1]; //Desplazamos la parte alta a los 8 ultimos bits y le hacemos un OR
												//para tener en los 8 primeros la parte baja
			//printf("%d", rbuf[0]);
	}

	return 0;
}


void I2C_XFER_T_config (I2C_XFER_T * xfer,uint8_t *rbuf, int rxSz, uint8_t slaveAddr, I2C_STATUS_T status, uint8_t * wbuf, int txSz)
{
	//I2C_XFER_T  xfer_aux = *xfer;

	xfer->rxBuff = rbuf; //Buffer de lectura
	xfer->rxSz = rxSz;	//cantidad de bytes que se desean leer, arbitrariamente seteamos 10
	xfer->slaveAddr = slaveAddr; //Adress estatica del dispositivo i2c a leer (MPU6050)
	xfer->status = status;
	xfer->txBuff = wbuf; //Buffer de escritura
	xfer->txSz = txSz; //cantidad de bytes que se desean escribir, solo escribimos el registro desde
					//el que comenzamos a leer

	Chip_I2C_MasterTransfer(I2C1, xfer);

}

void MPU6050_wakeup(I2C_XFER_T * xfer)
{
		//writeBit(MPU6050_RA_PWR_MGMT_1, MPU6050_PWR1_SLEEP_BIT, 0);
		uint8_t wbuf[2] = {MPU6050_RA_PWR_MGMT_1, 0};
		xfer->slaveAddr = MPU6050_DEVICE_ADDRESS;
		xfer->txBuff = wbuf;
		xfer->txSz = 2;
		xfer->rxSz = 0;

		//Chip_I2C_MasterSend(I2C1, xfer->slaveAddr, xfer->txBuff, xfer->txSz);
		do{
			Chip_I2C_MasterTransfer(I2C1, xfer);
		}while(xfer->status != I2C_STATUS_DONE);
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */

/*==================[end of file]============================================*/


