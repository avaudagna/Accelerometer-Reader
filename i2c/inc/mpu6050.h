#ifndef _TIME_MGMT_H_
#define _TIME_MGMT_H_

#if defined (__USE_LPCOPEN)
#if defined(NO_BOARD_LIB)
#include "chip.h"
#else
#include "board.h"
#endif
#endif


#include "i2c.h"

/*==================[macros]=================================================*/

#define MPU6050_I2C_DEVICE_ADDRESS   0x68
#define MPU6050_I2C_DEVICE_ADDRESS   0x68
#define MPU6050_RA_PWR_MGMT_1   	 0x6B
#define MPU6050_RA_PWR_MGMT_2    	 0x6C
#define MPU6050_RA_FIFO_R_W 0x74
#define MPU6050_RA_FIFO_COUNTH		 0x72

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

/*==================[internal functions declaration]=========================*/
void MPU6050_wakeup(I2C_XFER_T * xfer);
void Fill_Mpu_Data(mpu_data * mdata, uint8_t * rbuf);
void registers_setup_MPU6050(void);
mpu_data * BurstRead (int cant, int fifo_size);
uint16_t FIFO_Count(void);

#endif
