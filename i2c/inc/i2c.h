#ifndef _I2C_H_
#define _I2C_H_

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif


/*==================[internal functions declaration]=========================*/
/** @brief I2C setup function. Resolves the i2c com
 * @return main function should never return
 */
void I2C_XFER_config (I2C_XFER_T * xfer,uint8_t *rbuf, int rxSz, uint8_t slaveAddr, I2C_STATUS_T status, uint8_t * wbuf, int txSz);
uint8_t * I2C_ReadByte(uint8_t slaveAddr, uint8_t regAddr);
void I2C_WriteByte(uint8_t slaveAddr, uint8_t regAddr, uint8_t write_value);
uint8_t * I2C_ReadBytes(uint8_t slaveAddr, uint8_t regAddr, int cant);

#endif
