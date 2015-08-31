/**
 * 	file\			PR_Serie
 * 	brief\
 * 	details\
 * */
#include "Infotronic.h"

extern uint8_t BuffRX[MAX_RX];
extern uint8_t BuffTX[MAX_TX];

extern uint8_t IndiceBuff_InRX;
extern uint8_t IndiceBuff_OutRX;
extern uint8_t IndiceBuff_InTX;
extern uint8_t IndiceBuff_OutTX;

extern volatile uint8_t flagTX;
extern volatile uint8_t flag_b;

//--------------------------------------------------------------------------------//
/**
 * 	fn\			void Transmitir (char* m)
 * 	brief\		le pasa a PushTX los chars del array para que los transmita por serie
 * */
void Transmitir (char* m)
{
	while(*m)
	{
		PushTX(*m);
		m++;

	}
}

//--------------------------------------------------------------------------------//
// pone en el buffer lo que recibio de la pc (registro -> buffer)
//ANDA
/**
 * 	fn\			void PushRX (uint8_t dato)
 * 	brief\		agarra el registro de llegada y lo pone en el vector
 * */
void PushRX (uint8_t dato)
{
	BuffRX [ IndiceBuff_InRX ] = dato;
	IndiceBuff_InRX++;
	IndiceBuff_InRX %= MAX_RX;
}

//--------------------------------------------------------------------------------//
// toma del buffer de recepcion para trabajar en el micro. Entrega a Mensajes()
// ANDA
/**
 * 	fn\			int PopRX (void)
 * 	brief\		devuelve un valor del buffer
 * */
int PopRX (void)
{
	int dato = -1;									//retorna -1 por falso (buffer vacío)
	if(IndiceBuff_OutRX != IndiceBuff_InRX)
	{
		dato = (int)BuffRX [ IndiceBuff_OutRX ];
		IndiceBuff_OutRX++;
		IndiceBuff_OutRX %= MAX_RX;
	}
	return dato;
}

//--------------------------------------------------------------------------------//
// pone en el buffer de salida
//
/**
 * 	fn\			void PushTX (uint8_t dato)
 * 	brief\		envia a la uart cuando se utiliza transmitir
 * */
void PushTX (uint8_t dato)
{
	if (flagTX)
	{
		BuffTX [ IndiceBuff_InTX ] = dato;
		IndiceBuff_InTX++;
		IndiceBuff_InTX %= MAX_TX;
	}
	else
	{
		flagTX = 1;			//cuando flagTX es 0 es que el buffer esta vacío y quiero forzar la primera transmision
		//U1THR = dato;
		U0THR = dato;
	}
}
//--------------------------------------------------------------------------------//
// toma del buffer de salida para serializar
//
/**
 * 	fn\			int PopTX (void)
 * 	brief\		saca del buffer y se lo entrega a Mensajes();
 * */
int PopTX (void)
{
	int dato = -1;									//retorna -1 por falso (buffer vacío
	if(IndiceBuff_InTX != IndiceBuff_OutTX)
	{
		dato = (int)BuffTX [ IndiceBuff_OutTX ];
		IndiceBuff_OutTX++;
		IndiceBuff_OutTX %= MAX_TX;
	}
	return dato;
}

