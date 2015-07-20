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

#ifndef _MAIN_H_
#define _MAIN_H_
/** @brief Brief for this header file.
 **
 ** Full description for this header file.
 **
 **/

/** \addtogroup TD2 Técnicas Digitales II
 ** @{ */

/*
 * Initials    Name
 * ---------------------------
 * PR          Pablo Ridolfi
 */

/*
 * modification history (new versions first)
 * -----------------------------------------------------------
 * 20150421 v0.0.1 PR Initial release.
 */

/*==================[inclusions]=============================================*/

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
extern "C" {
#endif

/*==================[macros]=================================================*/
#define MPU6050_I2C_DEVICE_ADDRESS   0x68
#define MPU6050_RA_PWR_MGMT_1    0x6B
#define MPU6050_RA_PWR_MGMT_2    0x6C
/*==================[typedef]================================================*/

typedef struct mpu_data
{
   uint16_t   ax;
   uint16_t   ay;
   uint16_t   az;
   uint16_t   gx;
   uint16_t   gy;
   uint16_t   gz;
} mpu_data;

/*==================[external data declaration]==============================*/

/*==================[external functions declaration]=========================*/

/** @brief main function
 * @return main function should never return
 */
int main(void);
/** @brief I2C config function. Resolves the i2c com.
 * @return main function should never return
 */
void I2C_XFER_config (I2C_XFER_T * xfer,uint8_t *rbuf, int rxSz, uint8_t slaveAddr, I2C_STATUS_T status, uint8_t * wbuf, int txSz);
/** @brief I2C config function. Resolves the i2c com.
 * @return main function should never return
 */
void MPU6050_wakeup(I2C_XFER_T * xfer);


void registers_setup_MPU6050(void);

/*==================[cplusplus]==============================================*/

#ifdef __cplusplus
}
#endif

/** @} doxygen end group definition */
/*==================[end of file]============================================*/
#endif /* #ifndef _MAIN_H_ */

