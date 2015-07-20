#include "time_mgmt.h"

volatile uint32_t msTicks = 0; // counter for 1ms SysTicks


/** @brief setup_SysTick_INT: Configuracion de la interrupcion por SysTick
 *	@return none
 */
void setup_SysTick_INT(void)
{
	//Configuro la interrupcion de Systic para poder contar la cantidad de milisegundos desde
	//el inicio del programa (funcion millis en arduino)
	SysTick_Config(SystemCoreClock / MS_CLOCK_DIVIDER);
}


/** @brief SysTick_Handler: ISR del Systick
 *	@return none
 */
void SysTick_Handler(void)
{
	//Aumento msTicks, que es quien tiene la cantidad de ms que pasaron desde el inicio del programa
	msTicks++;
}


/** @brief delay: Delay en milisegundos, BLOQUEANTE
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

