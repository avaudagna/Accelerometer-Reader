#include "Infotronic.h"
/**
 * 	file\			PR_GPIO
 * 	brief\
 * 	details\
 * */
//******************************************************************************************************/
//SetPINSEL configura la funcion del puerto

void SetPINSEL (uint8_t port, uint8_t pin, uint8_t sel)
{
port = port * 2 + ( pin / 16 );							//analisis de la posicion de puertos
														//surgido de la hoja de datos.
pin = ( pin % 16 ) * 2;									//pin varía de 2 en 2 hasta un máximo de 15,

PINSEL[port] = PINSEL[port] & (~(3 << pin));
PINSEL[port] = PINSEL[port] | (sel << pin);
}

//******************************************************************************************************/
//SetPINMODE comfigura PullUp, PullDown, etc

void SetPINMODE (uint8_t port, uint8_t pin, uint8_t modo)
{
port = port * 2 + ( pin / 16 );							//analisis de la posicion de puertos
														//surgido de la hoja de datos.
pin = ( pin % 16 ) * 2;									//pin varía de 2 en 2 hasta un máximo de 15,

PINMODE[port] = PINMODE[port] & (~(3 << pin));
PINMODE[port] = PINMODE[port] | (modo << pin);
}

//******************************************************************************************************/
// Configura GPIO entrada o salida

void SetDIR (uint8_t port, uint8_t pin, uint8_t dir)
{
port = port*8;

GPIO[port] = GPIO[port] & (~(1 << pin));
GPIO[port] = GPIO[port] | (dir << pin);
}

//******************************************************************************************************/
//Pone un 1 o un 0 en el pin indicado

void SetPIN (uint8_t port, uint8_t pin, uint8_t estado)
{
port = port*8 + 5;

GPIO[port] = GPIO[port] & (~(1 << pin));
GPIO[port] = GPIO[port] | (estado << pin);
}

//******************************************************************************************************/
//Lee el pin indicado

uint8_t GetPIN (uint8_t port, uint8_t pin, uint8_t actividad)
{
port = port*8 + 5;

return ( ( ( GPIO[port] >> pin ) & 1 ) == actividad ) ? 1 : 0;
}

