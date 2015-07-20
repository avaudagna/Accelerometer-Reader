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
#include <stdlib.h> //por malloc
#include "main.h"

/*==================[macros and definitions]=================================*/

#define MS_CLOCK_DIVIDER 1000 //usado para obtener la interrupcion cada 1 ms
//Como se configura que la frecuencia de clock sea paralela al eje x del giroscopo
//y el mismo se actualiza cada 1ms, entonces coincidira que cada 1 ms

#define FIFO_R_W 0x74
/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/

volatile uint32_t msTicks = 0; // counter for 1ms SysTicks

/** @brief SysTick_Handler: Funcion de atencion de interrupcion del Systick
 *	@return none
 */
void SysTick_Handler(void)
{
	//Aumento msTicks, que es quien tiene la cantidad de ms que pasaron desde el inicio del programa
	msTicks++;
}


/** @brief delay: Delay en milisegundos
 *	@return none
 */
void delay (uint32_t delay_ms) {
  uint32_t current_msTicks;

  current_msTicks = msTicks;	// read current mstick counter
  // Now loop until required number of ms passes.
  //msTicks se ira actualizando por la interrupcion de systick
  while ((msTicks - current_msTicks) < delay_ms);
}

//TODO:ESTO NO PUEDE GENERAR OVERFLOW UNA VEZ LLEGADO AL MAX DE uint32 y EXPLOTA TODO???
/** @brief Devuelve la cantidad de tiempo en ms que paso desde la ejecucion del programa
 *  Necasario para el calculo de gyro angle
 *	@return none
 */
uint32_t millis(void)
{
	 return msTicks;
}


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

	//Configuro la interrupcion de Systic para poder contar la cantidad de milisegundos desde
	//el inicio del programa (funcion millis en arduino)
	SysTick_Config(SystemCoreClock / MS_CLOCK_DIVIDER);

}

/*==================[external functions definition]==========================*/

int main(void)
{
	/*==================[Inicializacion]==========================*/
	uint8_t wbuf[3] = {0,0,0};
	uint8_t rbuf[7] = {0,0,0,0,0,0,0};
	//uint32_t i;
	I2C_XFER_T xfer;

	//Del LPC
	initHardware();

	/*==================[Configuracion del MPU]==========================*/

	//Inicializacion de los registros del MPU
	registers_setup_MPU6050();

	MPU6050_wakeup(&xfer); //Despierto el MPU seteando PWR_MGMT_1 y 2 en 0x00
	//Delay que se usa despues de setear los pwr_mgmt
    delay(200);

	wbuf[1]=0x19;
	I2C_XFER_config(&xfer, rbuf, 7, MPU6050_I2C_DEVICE_ADDRESS, 0, wbuf, 1);
	/*
	 * SMPLRT_DIV
	 * CONFIG R/W
	 * GYRO_CONFIG
	 * ACCEL_CONFIG
	 * MOT_THR
	 */
	/*==================[LOOP]==========================*/

	//Porque quiero levantar 10 muestras, hago esto para probar
	//solo me va a servir para encajar 10 muestras de una pero no es
	//util para ir polleando, de hecho es recontra bloqueante
	delay(10);
	mpu_data * mdata  =  BurstRead(10,12);//10 muestras de 16 bits por leer y 12 bytes por la configuracion del registro FIFO_EN

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

/*
 * Nosotros seguramente implementemos la lectura de la Fifo en funcion de 1 de dos cosas:
 * 	-Interrupcion externa por el PIN INT de la plaqueta del MPU
 * 	-Interrupcion de timer sabiendo que cada X cant de tiempo, se toma una muestra de accel
 * 		y gyro y se lo manda a la FIFO. El tiempo que tiene que pasar entre cada interrupcion
 * 		de timer tiene que estar entre cada muestra nueva ingresada (ej si el SMPL_RT es 1k->1ms
 * 		-> tendriamos que tener un tmr_time mayor a 1ms y menor a 2 ms (1ms < tmr_time < 2ms) ej 1,5)
 *
 * 	En esta funcion de ejemplo se llama a la funcion delay(n) siendo n la cantidad de ms que queremos
 * 	que pase, que coinciden con el T_RT y por lo tanto con la cantidad de muestras que se almacenaran
 * 	en la fifo.
 */
//cant es la cantidad de samples que se desean leer
//TODO: hacer esta funcion generica y poner el resto en una que se llame MPU_DATA_OBTENTION
mpu_data * BurstRead (int cant, int fifo_size)
{
	int i =0;
	I2C_XFER_T xfer;
	uint8_t wbuf [] = {FIFO_R_W};
	uint8_t rbuf[] = {0,0,0,0,0,0,0,0,0,0,0,0};

	//Voy a guardar en un vector de estructura de datos de gyro y accel todos los samples
	//obtenidos (cant samples)
	mpu_data * mdata = (mpu_data*) malloc (cant*sizeof(mpu_data));
    if(mdata==NULL)
    	return NULL;

	for (i=0; i<cant; i++)
	{
		//Pongo que se van a leer 12 por los 3 LH del gyro y los 3 LH del accel
		//configurados previamente para guardarse en la FIFO a la SMPL_RT
		I2C_XFER_config(&xfer, rbuf, fifo_size, MPU6050_I2C_DEVICE_ADDRESS, 0, wbuf, 1);
		Fill_Mpu_Data(mdata[i],rbuf);
	}

	return mdata;

}


void Fill_Mpu_Data(mpu_data * mdata, uint8_t * rbuf)
{
	//De momento leer rbuf es lo mismo que leer xfer.rxBuff porque apuntan a la misma direccion
	//Desplazamos la parte alta a los 8 ultimos bits y le hacemos un OR para tener en los 8 primeros la parte baja

	mdata.ax=(rbuf[0] << 8) | rbuf[1];
	mdata.ay=(rbuf[2] << 8) | rbuf[3];
	mdata.az=(rbuf[4] << 8) | rbuf[5];
	mdata.gx=(rbuf[6] << 8) | rbuf[7];
	mdata.gy=(rbuf[8] << 8) | rbuf[9];
	mdata.gz=(rbuf[10] << 8) | rbuf[11];
}

void registers_setup_MPU6050(void)
{

	I2C_XFER_T xfer;
	int rbuf[1]=0;
	int wbuf1[] = {0x0D,//first register (self_text_x)
				  0,0,0,0 //self test registers
				  ,0 // 00000000   /sampling rate setting, 1kHz/(0+1)=1kHz
				  ,1 // 00 000 001 / xx [extsync] [dlfp] / digital LPF setting, 1kHz-184Hz band Accel & 1kHz-188Hz band Gyro
				  ,0 // 000 00 000 / [STx][y][z] [FS] 000 / gyro scale setting, 250deg/s
				  ,0 // 000 00 000/ [STx][y][z] [FS] 000 / acc scale setting, 2g
				  //MOT_THR queda igual como viene default, no lo uso
				  // FIFO setting, hago que guarde en la fifo todos los bytes de los registros de gyro y accel
				  ,78//0 111 1 000 [temp][gyro x][y][z] [accxyz] [slv2][1][0] / FIFO mask
				  ,2 // 0 0 000 010 / - [FIFOen] [I2CMST][I2CDIS]- [FIFOreset][I2CMSTrst][SIGreset] / I2C Master Reset
	};

	I2C_XFER_config(&xfer, rbuf, 0, MPU6050_I2C_DEVICE_ADDRESS, 0, wbuf, (int) sizeof(wbuf1)/sizeof(int));

	int wbuf2[] = {40}; ///0 1 000 000 / - [FIFOen] [I2CMST][I2CDIS]- [FIFOreset][I2CMSTrst][SIGreset] / Enable FIFO

	I2C_XFER_config(&xfer, rbuf, 0, MPU6050_I2C_DEVICE_ADDRESS, 0, wbuf, (int) sizeof(wbuf2)/sizeof(int));
}

/** @} doxygen end group definition */
/** @} doxygen end group definition */

/*==================[end of file]============================================*/
