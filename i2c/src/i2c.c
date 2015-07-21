

#include "i2c.h"
#include <stdlib.h>

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


uint8_t * I2C_ReadByte(uint8_t slaveAddr, uint8_t regAddr)
{
	I2C_XFER_T xfer;
	uint8_t rbuf[]={0};
	uint8_t wbuf[]={regAddr};
	xfer.rxBuff = rbuf; //Buffer de lectura
	xfer.rxSz = 1;	//cantidad de bytes que se desean leer, en este caso 1
	xfer.slaveAddr = slaveAddr; //Adress estatica del dispositivo i2c a leer (MPU6050)
	xfer.status = 0; //Siempre lo seteamos en 0
	xfer.txBuff = wbuf; //Buffer de escritura
	xfer.txSz = 1; //cantidad de bytes que se desean escribir, en este caso solo la
					//regAddr del registro que queremos leer
	Chip_I2C_MasterTransfer(I2C1, &xfer);

	return rbuf;
}

uint8_t * I2C_ReadBytes(uint8_t slaveAddr, uint8_t regAddr, int cant)
{
	I2C_XFER_T xfer;
	uint8_t * rbuf = (uint8_t*) calloc(cant,sizeof(uint8_t));
	if (rbuf==NULL)
		return NULL;
	uint8_t wbuf[]={regAddr};
	xfer.rxBuff = rbuf; //Buffer de lectura
	xfer.rxSz = cant;	//cantidad de bytes que se desean leer, en este caso cant
	xfer.slaveAddr = slaveAddr; //Adress estatica del dispositivo i2c a leer (MPU6050)
	xfer.status = 0; //Siempre lo seteamos en 0
	xfer.txBuff = wbuf; //Buffer de escritura
	xfer.txSz = 1; //cantidad de bytes que se desean escribir, en este caso solo la
					//regAddr del registro que queremos leer
	Chip_I2C_MasterTransfer(I2C1, &xfer);

	return rbuf;
}




void I2C_WriteByte(uint8_t slaveAddr, uint8_t regAddr, uint8_t write_value)
{
	I2C_XFER_T xfer;
	uint8_t rbuf[]={0};
	uint8_t wbuf[]={regAddr,write_value};
	xfer.rxBuff = rbuf; //Buffer de lectura
	xfer.rxSz = 0;	//cantidad de bytes que se desean leer, en este caso 0
	xfer.slaveAddr = slaveAddr; //Adress estatica del dispositivo i2c a leer (MPU6050)
	xfer.status = 0; //Siempre lo seteamos en 0
	xfer.txBuff = wbuf; //Buffer de escritura
	xfer.txSz = 2; //cantidad de bytes que se desean escribir, en este caso solo la
					//regAddr del registro que queremos leer
	Chip_I2C_MasterTransfer(I2C1, &xfer);
}


