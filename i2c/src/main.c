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

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

/** @brief hardware initialization function
 *	@return none
 */
static void initHardware(void);
void I2C_XFER_T_config (I2C_XFER_T * xfer,uint8_t *rbuf, int rxSz, uint8_t slaveAddr, I2C_STATUS_T status, uint8_t * wbuf, int txSz);

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

//TODO: Leer 10 veces el registro estatico


int main(void)
{
	/*==================[Inicializacion]==========================*/
	uint8_t wbuf[3] = {0,0,0};
	uint8_t rbuf[20];
	uint8_t samples[10] = {0,0,0,0,0,0,0,0,0,0};
	//uint32_t i;
	I2C_XFER_T xfer;

	int i =0;


	initHardware();


	/*==================[Configuracion del I2C_XFER_T]==========================*/

	//Escritura

	//Define el registro que se va a leer
	wbuf[0] = STATIC_0x40_REFERENCE_REGISTER; //En el registro 0x6B (107) se supone que deberia leerse siempre un 0x40

	//Lectura

	I2C_XFER_T_config(&xfer, rbuf, 10, MPU6050_I2C_SLAVE_ADDRESS, 0, wbuf, 1);

	//Resuelve el protocolo i2c, solo nos comunicamos como master con el slave (MPU6050)
	Chip_I2C_MasterTransfer(I2C1, &xfer);

	while(1)
	{
		Chip_I2C_MasterTransfer(I2C1, &xfer);

		if(i<20)
		{
			//TODO: Por que no funciona???
			//samples[i]=xfer.rxBuff[0];
			//De momento leer rbuf es lo mismo que ñeer xfer.rxBuff porque apuntan a la misma direccion
			samples[i]=rbuf[0];
			i++;
		}
		else
		{
			i=0;
			//TODO: Por que no funciona???
			//samples[i]=xfer.rxBuff[0];
			samples[i]=rbuf[0];
			i++;
		}
	}
}


void I2C_XFER_T_config (I2C_XFER_T * xfer,uint8_t *rbuf, int rxSz, uint8_t slaveAddr, I2C_STATUS_T status, uint8_t * wbuf, int txSz)
{
	xfer->rxBuff = rbuf; //Buffer de lectura
	xfer->rxSz = rxSz;	//cantidad de bytes que se desean leer, arbitrariamente seteamos 10
	xfer->slaveAddr = slaveAddr; //Adress estatica del dispositivo i2c a leer (MPU6050)
	xfer->status = status;
	xfer->txBuff = wbuf; //Buffer de escritura
	xfer->txSz = txSz; //cantidad de bytes que se desean escribir, solo escribimos el registro desde
					//el que comenzamos a leer
}





/** @} doxygen end group definition */
/** @} doxygen end group definition */

/*==================[end of file]============================================*/


