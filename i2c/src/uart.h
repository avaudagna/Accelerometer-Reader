/*
 * uart.h
 *
 *  Created on: Jun 25, 2015
 *      Author: alan
 */

#ifndef UART_H_
#define UART_H_

void HANDLER_NAME(void);
int UART_Init (void);
void UART_Enviar_Angulos (float * data);
void Desabilitar_UART();

#endif /* UART_H_ */
