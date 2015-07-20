#ifndef _TIME_MGMT_H_
#define _TIME_MGMT_H_

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

/*==================[macros]=================================================*/

#define MS_CLOCK_DIVIDER 1000 //usado para obtener la interrupcion cada 1 ms
//Como se configura que la frecuencia de clock sea paralela al eje x del giroscopo
//y el mismo se actualiza cada 1ms, entonces coincidira que cada 1 ms

/*==================[external data declaration]==============================*/
extern volatile uint32_t msTicks;

/*==================[internal functions declaration]=========================*/

void setup_SysTick_INT(void);
void SysTick_Handler(void);
void delay (uint32_t delay_ms);
uint32_t millis(void);

#endif


