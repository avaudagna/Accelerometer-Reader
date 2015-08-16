#ifndef _MAIN_H_
#define _MAIN_H_


/*==================[inclusions]=============================================*/
#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif

//Externos
#include <cr_section_macros.h>
#include <stdlib.h> //por malloc
//Internos
#include "i2c.h"
#include "mpu6050.h"
#include "time_mgmt.h"
/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif


/** @brief main function
 * @return main function should never return
 */
int main(void);
/** @brief I2C config function. Resolves the i2c com.
 * @return main function should never return
 */

//void InitMPU(void);

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* #ifndef _MAIN_H_ */

