
#include "mpu6050.h"
#include "i2c.h"
#include <stdlib.h>

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

void Fill_Mpu_Data(mpu_data * mdata, uint8_t * rbuf)
{
	//De momento leer rbuf es lo mismo que leer xfer.rxBuff porque apuntan a la misma direccion
	//Desplazamos la parte alta a los 8 ultimos bits y le hacemos un OR para tener en los 8 primeros la parte baja

	mdata->ax=(rbuf[0] << 8) | rbuf[1];
	mdata->ay=(rbuf[2] << 8) | rbuf[3];
	mdata->az=(rbuf[4] << 8) | rbuf[5];
	mdata->gx=(rbuf[6] << 8) | rbuf[7];
	mdata->gy=(rbuf[8] << 8) | rbuf[9];
	mdata->gz=(rbuf[10] << 8) | rbuf[11];
}


void registers_setup_MPU6050(void)
{

	I2C_XFER_T xfer;
	uint8_t rbuf[1]={0};
	uint8_t wbuf1[] = {
			0x0D,//first register (self_text_x)
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

	printf("%d\n",(int) sizeof(wbuf1)/sizeof(uint8_t));
	I2C_XFER_config(&xfer, rbuf, 0, MPU6050_I2C_DEVICE_ADDRESS, 0, wbuf1, (int) sizeof(wbuf1)/sizeof(uint8_t));

	uint8_t wbuf2[] = {40}; ///0 1 000 000 / - [FIFOen] [I2CMST][I2CDIS]- [FIFOreset][I2CMSTrst][SIGreset] / Enable FIFO

	I2C_XFER_config(&xfer, rbuf, 0, MPU6050_I2C_DEVICE_ADDRESS, 0, wbuf2, (int) sizeof(wbuf2)/sizeof(uint8_t));
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
	uint8_t wbuf [] = {MPU6050_RA_FIFO_R_W};
	uint8_t rbuf[] = {0,0,0,0,0,0,0,0,0,0,0,0};

	//Voy a guardar en un vector de estructura de datos de gyro y accel todos los samples
	//obtenidos (cant samples)
	mpu_data * mdata = (mpu_data*) calloc (cant,(sizeof(mpu_data)));
    if(mdata==NULL)
    	return NULL;

	for (i=0; i<cant; i++)
	{
		//Pongo que se van a leer 12 por los 3 LH del gyro y los 3 LH del accel
		//configurados previamente para guardarse en la FIFO a la SMPL_RT
		I2C_XFER_config(&xfer, rbuf, fifo_size, MPU6050_I2C_DEVICE_ADDRESS, 0, wbuf, 1);
		Fill_Mpu_Data(&mdata[i],rbuf);
	}
	return mdata;

}


/*
 * Devuelve la cantidad de bytes que se
 * almacenaron en la fifo
 */
uint16_t FIFO_Count(void)
{

	uint8_t wbuf[1] = {MPU6050_RA_FIFO_COUNTH};
	uint8_t rbuf[2]={0,0};
	uint16_t count =0;
	I2C_XFER_T xfer;

	I2C_XFER_config(&xfer, rbuf, 2, MPU6050_I2C_DEVICE_ADDRESS, 0, wbuf, 1);

	count = (rbuf[0] << 8) | rbuf[1];

	return count;
}

