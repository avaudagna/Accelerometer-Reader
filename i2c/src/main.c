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
#define MPU6050_RA_PWR_MGMT_2    0x6C
#define MPU6050_PWR1_SLEEP_BIT   6

#define MS_CLOCK_DIVIDER 1000 //usado para obtener la interrupcion cada 1 ms
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

	//Configuro la interrupcion de Systic para poder contar la cantidad de milisegundos desde
	//el inicio del programa (funcion millis en arduino)
	SysTick_Config(SystemCoreClock / MS_CLOCK_DIVIDER);
}


volatile uint32_t msTicks = 0; // counter for 1ms SysTicks

void SysTick_Handler(void)
{
	//Aumento msTicks, que es quien tiene la cantidad de ms que pasaron desde el inicio del programa
	msTicks++;
}

//delayTicks: Delay en milisegundos
void delay (uint32_t delay_ms) {
  uint32_t current_msTicks;

  current_msTicks = msTicks;	// read current mstick counter
  // Now loop until required number of ms passes.
  //msTicks se ira actualizando por la interrupcion de systick
  while ((msTicks - current_msTicks) < delay_ms);
}

//Devuelve la cantidad de tiempo en ms que paso desde la ejecucion del programa
//Necasario para el calculo de gyro angle
//TODO:ESTO NO PUEDE GENERAR OVERFLOW UNA VEZ LLEGADO AL MAX DE uint32 y EXPLOTA TODO???
uint32_t millis(void)
{
	 return msTicks;
}

/*==================[external functions definition]==========================*/

//TODO: Leer 10 veces el registro de Acelerometro en X, esto nos dara 5 datos
//	  	compuestos de 2 bytes cada uno (parte high y low)


//  Use the following global variables and access functions
//  to calibrate the acceleration sensor
float    base_x_accel;
float    base_y_accel;
float    base_z_accel;

float    base_x_gyro;
float    base_y_gyro;
float    base_z_gyro;

int main(void)
{
	/*==================[Inicializacion]==========================*/
	uint8_t wbuf[2] = {0,0};
	uint8_t rbuf[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	//Se lo crea con 6 posiciones porque los registros que se procesan de los sensores son :
	/*
	ACCEL_XOUT_H
	ACCEL_XOUT_L
	ACCEL_YOUT_H
	ACCEL_YOUT_L
	ACCEL_ZOUT_H
	ACCEL_ZOUT_L
	TEMP_OUT_H
	TEMP_OUT_L
	GYRO_XOUT_H
	GYRO_XOUT_L
	GYRO_YOUT_H
	GYRO_YOUT_L
	GYRO_ZOUT_H
	GYRO_ZOUT_L
	 */
	uint16_t samples[7] = {0,0,0,0,0,0,0}; //cada posicion es de 16 bits, necesario para guardar
											//la parte low y high de las muestras de accel
	I2C_XFER_T xfer;

	initHardware();


	/*==================[Configuracion del I2C_XFER_T]==========================*/

	// Metodo de Escritura
	//Define el registro que se va a leer en la primera posicion de wbuf
	//wbuf[0] = MPU6050_RA_ACCEL_XOUT_H; //Parte high de la lectura en x del acelerometro
	//Como la lectura se realiza de forma secuencial, en la posicion 0
	//del rbuf ira este dato y en la posicion 1 ira la correspondiente a
	//la posicion siguiente, osea ACCEL_XOUT_L (0x3C)


	MPU6050_wakeup(&xfer);

	//Lectura de PWR_MGMMT_1 2 (para verificar si se lo saco del sleep y de standby a los ejes)
	wbuf[0] = MPU6050_RA_PWR_MGMT_1;
	I2C_XFER_config(&xfer, rbuf, 2, MPU6050_I2C_SLAVE_ADDRESS, 0, wbuf, 1);

	//Initialize the angles
	calibrate_sensors();
	set_last_read_angle_data(millis(), 0, 0, 0, 0, 0, 0);


	//Configuracion de la 1era direccion desde la que se leeran los valores de los registros de los sensores
	wbuf[0]=MPU6050_RA_ACCEL_XOUT_H;
	wbuf[1]=0;


	double ax ;
	double ay ;
	double az ;
	double gx ;
	double gy ;
	double gz ;
	double arx ;
	double ary ;
	double arz ;
	unsigned long t_now;
	double dT;

	while(1)
	{
		//Leo los valores de los registros de los sensores
		I2C_XFER_config(&xfer, rbuf, 14, MPU6050_I2C_SLAVE_ADDRESS, 0, wbuf, 1);

		// Obtengo el tiempo en el que se efectuo la lectura para realizar el calculo rotacional
	    t_now = millis();

	    //Parseo a datos de 16 bits
		Fill_Samples(&samples, &rbuf);


	      // The temperature sensor is -40 to +85 degrees Celsius.
	      // It is a signed integer.
	      // According to the datasheet:
	      //   340 per degrees Celsius, -512 at 35 degrees.
	      // At 0 degrees: -512 - (340 * 35) = -12412
	      // samples[3] contiene el valor de la temperatura en unidades de medicion
	      //Aca se lo pasa a grados celsius implementando la logica de arriba
	      dT = ( (double) samples[3] + 12412.0) / 340.0;


		 ax = samples[0];
		 ay = samples[1];
		 az = samples[2];
		 gx = samples[4];
		 gy = samples[5];
		 gz = samples[6];



		  // Convert gyro values to degrees/sec
		  float FS_SEL = 131;

		  float gyro_x = (gx - base_x_gyro)/FS_SEL;
		  float gyro_y = (gy - base_y_gyro)/FS_SEL;
		  float gyro_z = (gz - base_z_gyro)/FS_SEL;

		  // Get raw acceleration values
		   //float G_CONVERT = 16384;

		  // Get angle values from accelerometer
		    float RADIANS_TO_DEGREES = 180/3.14159;
		  //  float accel_vector_length = sqrt(pow(accel_x,2) + pow(accel_y,2) + pow(accel_z,2));
		    float accel_angle_y = atan(-1*ax/sqrt(ay*ay) + (az*az))*RADIANS_TO_DEGREES;
		    float accel_angle_x = atan(ay/sqrt((ax*ax) + (az*az)))*RADIANS_TO_DEGREES;
		    float accel_angle_z = 0; //No puede obtenerse con el accelerometer


		    // Compute the (filtered) gyro angles
		  //dt esta medido en segundos, por eso se multiplica por 1000 al last_read_time obtenido, que esta en ms
		  //TODO: El dt es el correcto sobre el que se tiene que hacer las cuentas???
		  float dt =(t_now - get_last_time())/1000.0;
		  float gyro_angle_x = gyro_x*dt + get_last_x_angle();
		  float gyro_angle_y = gyro_y*dt + get_last_y_angle();
		  float gyro_angle_z = gyro_z*dt + get_last_z_angle();


		  // Compute the drifting gyro angles, estos son los que tienen error de drift
		  float unfiltered_gyro_angle_x = gyro_x*dt + get_last_gyro_x_angle();
		  float unfiltered_gyro_angle_y = gyro_y*dt + get_last_gyro_y_angle();
		  float unfiltered_gyro_angle_z = gyro_z*dt + get_last_gyro_z_angle();




		  // Apply the complementary filter to figure out the change in angle - choice of alpha is
		  // estimated now.  Alpha depends on the sampling rate...
		  float alpha = 0.96;
		  float angle_x = alpha*gyro_angle_x + (1.0 - alpha)*accel_angle_x;
		  float angle_y = alpha*gyro_angle_y + (1.0 - alpha)*accel_angle_y;
		  float angle_z = gyro_angle_z;  //Accelerometer doesn't give z-angle

		  // Update the saved data with the latest values
		   set_last_read_angle_data(t_now, angle_x, angle_y, angle_z, unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);

		//Implementacion vieja

		/* arx = (180/3.141592) * atan(ax / sqrt((ay*ay) + (az*az)));
		 ary = (180/3.141592) * atan(ay / sqrt((ax*ax) + (az*az)));
		 arz = (180/3.141592) * atan(sqrt((ay*ay) + (ax*ax)) / az);

		 arx =  3.141592 * atan(ax / sqrt((ay*ay) + (az*az)));
		 ary =  3.141592 * atan(ay / sqrt((ax*ax) + (az*az)));
		 arz =  3.141592 * atan(sqrt((ay*ay) + (ax*ax)) / az);*/


		 //PARTE UART
	     //Proseguimos a enviar por UART los angulos obtenidos
		 //float angles [] = {angle_x, angle_y, angle_z};
		 //UART_Enviar_Angulos(angles);

		 //Impresion de los angulos obtenidos a traves de la consola
		 //printf("(%f,%f,%f)\n", angle_x, angle_y, angle_z);
		 //printf("(%f,%f,%f)\n", unfiltered_gyro_angle_x, unfiltered_gyro_angle_y, unfiltered_gyro_angle_z);


		 //DEBUG
	     printf("(%f,%f,%f)\n", accel_angle_x, accel_angle_y, accel_angle_z);

	}

	//Desabilitar_UART();

	return 0;
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

		I2C_XFER_config(xfer, xfer->rxBuff, 0, MPU6050_I2C_SLAVE_ADDRESS, 0, wbuf, 3);
}

/* Llena el vector de muestras samples con la data de los registros de ACCEL, GYRO y TEMP del MPU
 * Al estar la informacion en 16 bits y ser levantada por registros de 8, en rbuf esta la parte low
 * 	y high de cada componente, por lo que se debe recomponer desplazando la parte high y concatenando la low]
 * 	para obtener el valor que se midio. Proceso que se realiza en esta funcion para pasar a samples.
 *
 * 	 rbuf : Tiene la data leida por el MPU con la data dividida en high y low
 * 	 samples : Tendra la data agrupada que representa al valor medido por los sensores en cada eje
 */
void Fill_Samples(uint16_t * samples, uint8_t * rbuf)
{
	//De momento leer rbuf es lo mismo que leer xfer.rxBuff porque apuntan a la misma direccion
	//Desplazamos la parte alta a los 8 ultimos bits y le hacemos un OR para tener en los 8 primeros la parte baja


	samples[0]=(rbuf[0] << 8) | rbuf[1];
	samples[1]=(rbuf[2] << 8) | rbuf[3];
	samples[2]=(rbuf[4] << 8) | rbuf[5];
	samples[3]=(rbuf[6] << 8) | rbuf[7];
	samples[4]=(rbuf[8] << 8) | rbuf[9];
	samples[5]=(rbuf[10] << 8) | rbuf[11];
	samples[6]=(rbuf[12] << 8) | rbuf[13];
}


void calibrate_sensors() {
  int                   num_readings = 10;
  float                 x_accel = 0;
  float                 y_accel = 0;
  float                 z_accel = 0;
  float                 x_gyro = 0;
  float                 y_gyro = 0;
  float                 z_gyro = 0;
  //Serial.println("Starting Calibration");

  // Discard the first 10 set of values read from the IMU
  //TODO:por que 10? regla empirica?
  uint8_t wbuf[2] = {MPU6050_RA_ACCEL_XOUT_H};
  uint8_t rbuf[10] = {0,0,0,0,0,0,0,0,0,0};
  I2C_XFER_T xfer;
  uint16_t samples[7] = {0,0,0,0,0,0,0};
  int i =0;
  for ( i=0; i< num_readings; i++)
  {
	  // Read and average the raw values from the IMU

	  I2C_XFER_config(&xfer, rbuf, 14, MPU6050_I2C_SLAVE_ADDRESS, 0, wbuf, 1);
	  Fill_Samples(&samples, &rbuf);
	  x_accel += samples[0];
	  y_accel += samples[1];
	  z_accel += samples[2];
	  x_gyro += samples[4];
	  y_gyro += samples[5];
	  z_gyro += samples[6];
	  delay(100);
  }
  x_accel /= num_readings;
  y_accel /= num_readings;
  z_accel /= num_readings;
  x_gyro /= num_readings;
  y_gyro /= num_readings;
  z_gyro /= num_readings;

  // Store the raw calibration values globally
  base_x_accel = x_accel;
  base_y_accel = y_accel;
  base_z_accel = z_accel;
  base_x_gyro = x_gyro;
  base_y_gyro = y_gyro;
  base_z_gyro = z_gyro;
}

// Use the following global variables and access functions to help store the overall
// rotation angle of the sensor
unsigned long last_read_time;
float         last_x_angle;  // These are the filtered angles
float         last_y_angle;
float         last_z_angle;
float         last_gyro_x_angle;  // Store the gyro angles to compare drift
float         last_gyro_y_angle;
float         last_gyro_z_angle;

//Devuelve el momento en el que se hizo la ultima lectura, en milisegundos desde la ejecucion del programa
unsigned long get_last_time() {return last_read_time;}
float get_last_x_angle() {return last_x_angle;}
float get_last_y_angle() {return last_y_angle;}
float get_last_z_angle() {return last_z_angle;}
float get_last_gyro_x_angle() {return last_gyro_x_angle;}
float get_last_gyro_y_angle() {return last_gyro_y_angle;}
float get_last_gyro_z_angle() {return last_gyro_z_angle;}


void set_last_read_angle_data(unsigned long time, float x, float y, float z, float x_gyro, float y_gyro, float z_gyro) {
  last_read_time = time;
  last_x_angle = x;
  last_y_angle = y;
  last_z_angle = z;
  last_gyro_x_angle = x_gyro;
  last_gyro_y_angle = y_gyro;
  last_gyro_z_angle = z_gyro;
}


/** @} doxygen end group definition */
/** @} doxygen end group definition */

/*==================[end of file]============================================*/


