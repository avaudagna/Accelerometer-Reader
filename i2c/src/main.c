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

int main(void)
{
	/*==================[Inicializacion]==========================*/
	uint8_t wbuf[3] = {0,0,0};
	uint8_t rbuf[5] = {0,0,0,0,0};
	//uint32_t i;
	I2C_XFER_T xfer;

	initHardware();


	/*==================[Configuracion del MPU]==========================*/

	MPU6050_wakeup(&xfer); //Despierto el MPU seteando PWR_MGMT_1 y 2 en 0x00


	wbuf[1]=0x19;
	I2C_XFER_config(&xfer, rbuf, 5, MPU6050_I2C_DEVICE_ADDRESS, 0, wbuf, 1);
	/*
	 * SMPLRT_DIV
	 * CONFIG R/W
	 * GYRO_CONFIG
	 * ACCEL_CONFIG
	 * MOT_THR
	 */
	/*==================[LOOP]==========================*/

	while(1)
	{

	}

}

/*
 * Configura la estructura XFER para realizar la comunicacion i2c.
 * * xfer	  : Puntero a la estructura del tipo I2C_XFER_T necesaria para utilizar la funcion Chip_I2C_MasterTransfer.
 * 				Chip_I2C_MasterTransfer : Funcion que resuelve la interaccion i2c en funcion de lo especificado en la estructura I2C_XFER_T
 *	 rbuf 	  : Puntero al buffer de lectura donde se volcaran los bytes leidos
 *	 rxSz 	  : Cantidad de bytes que se leeran y volcaran en rbuf
 *	 slaveAddr: Direccion estatica del slave con el que se desea comunicar
 *	 status   : Estado de la comunicacion, (estado inicial 0)
 *	 wbuf	  : Buffer de escritura donde se colocara tanto el registro que se desea escribir como el dato que desea ser escrito
 *	 			Ej de uso: wbuf[] = {reg_inicial, dato} solo escribe el byte dato en reg_inicial
 *	 					   wbuf[] = {reg_inicial, dato1, dato2} escribe el byte dato1 en reg_incial y dato2 en reg_inicial+1 (el registro siguiente)
 *	 txSz	  : La cantidad de bytes que se desean enviar, osea empezando a leer wbuf desde 0 inclusive, cuantos bytes manda de ese buffer
 *	 			Ej : wbuf[] = {reg_inicial, dato1, dato2}, entonces txSz deberia ser = 3
 *	 				 wbuf[] = {reg_inicial}, (caso tipico de solo lectura de ese registro), entonces txSz deberia ser = 1
 */

void I2C_XFER_config (I2C_XFER_T * xfer,uint8_t *rbuf, int rxSz, uint8_t slaveAddr, I2C_STATUS_T status, uint8_t * wbuf, int txSz)
{
	xfer->rxBuff = rbuf; //Buffer de lectura
	xfer->rxSz = rxSz;	//cantidad de bytes que se desean leer, arbitrariamente seteamos 10
	xfer->slaveAddr = slaveAddr; //Adress estatica del dispositivo i2c a leer (MPU6050)
	xfer->status = status;
	xfer->txBuff = wbuf; //Buffer de escritura
	xfer->txSz = txSz; //cantidad de bytes que se desean escribir, solo escribimos el registro desde
					//el que comenzamos a leer
	Chip_I2C_MasterTransfer(I2C1, xfer);
}


/*
 * Se encarga de inicializar los registros de PWR_MGMT necesarios para habilitar los
 *  sensores (acelerometro y giroscopo) en cada eje, para sus lecturas.
 *  La configuracion en la que quedan seteados es la por defecto:
 *  accelerometer (±2g) , gyroscope (±250°/sec).
 *
 *  * xfer : Puntero a la estructura del tipo I2C_XFER_T necesaria para la utilizacion de Chip_I2C_MasterTransfer.
 *  		 Chip_I2C_MasterTransfer : Funcion que resuelve la interaccion i2c en funcion de lo especificado en la estructura I2C_XFER_T
 */
void MPU6050_wakeup(I2C_XFER_T * xfer)
{
		//Setea PWR_MGMT_1 y 2 en 0, el byte de cada uno


		uint8_t wbuf[3] = {MPU6050_RA_PWR_MGMT_1, 0, 0};
		/*xfer->slaveAddr = MPU6050_DEVICE_ADDRESS;
		xfer->txBuff = wbuf;
		xfer->txSz = 3;
		xfer->rxSz = 0;*/

		I2C_XFER_config(xfer, xfer->rxBuff, 0, MPU6050_I2C_DEVICE_ADDRESS, 0, wbuf, 3);
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */

/*==================[end of file]============================================*/
