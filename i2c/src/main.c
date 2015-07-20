
/*==================[inclusions]=============================================*/

#include "main.h"

#include <stdio.h>
/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/


/** @brief hardware initialization function
 *	@return none
 */
static void initHardware(void);

/*==================[internal data definition]===============================*/


/*==================[internal functions definition]==========================*/

static void initHardware(void)
{
#if defined (__USE_LPCOPEN)
#if !defined(NO_BOARD_LIB)
    // Read clock settings and update SystemCoreClock variable
    SystemCoreClockUpdate();
    // Set up and initialize all required blocks and
    // functions related to the board hardware
    Board_Init();
    // Set the LED to the state of "Off"
    Board_LED_Set(0, false);
#endif
#endif
    Board_I2C_Init(I2C1);

    /* pines del stick */
	Chip_IOCON_PinMux(LPC_IOCON, 0, 0, IOCON_MODE_INACT, IOCON_FUNC3);
	Chip_IOCON_PinMux(LPC_IOCON, 0, 1, IOCON_MODE_INACT, IOCON_FUNC3);
	Chip_IOCON_EnableOD(LPC_IOCON, 0, 0);
	Chip_IOCON_EnableOD(LPC_IOCON, 0, 1);

    Chip_I2C_SetClockRate(I2C1, 100000);
	Chip_I2C_SetMasterEventHandler(I2C1, Chip_I2C_EventHandlerPolling);

	setup_SysTick_INT();
}


/*==================[external functions definition]==========================*/

int main(void)
{
	/*==================[Inicializacion]==========================*/
	uint8_t wbuf[3] = {0,0,0};
	uint8_t rbuf[10] = {0,0,0,0,0,0,0,0,0,0};
	int i;
	I2C_XFER_T xfer;
	uint16_t fifo_count=0;

	//Del LPC
	initHardware();

	/*==================[Configuracion del MPU]==========================*/
	MPU6050_wakeup(&xfer); //Despierto el MPU seteando PWR_MGMT_1 y 2 en 0x00
	//Inicializacion de los registros del MPU
	//Chequeo el valor de los registros configurados
	wbuf[1]=MPU6050_RA_PWR_MGMT_1;
	for(i=0;i<10;i++)
			printf("INITIAL_VALUES:rbuf[%d]=%d\n",i,rbuf[i]);
	delay(200);
	I2C_XFER_config(&xfer, rbuf, 2, MPU6050_I2C_DEVICE_ADDRESS, 0, wbuf, 1);
	for(i=0;i<2;i++)
		printf("PWR_MGMNT_CHK:rbuf[%d]=%d\n",i,rbuf[i]);

	for(i=0;i<10;i++)
		rbuf[i]= 0;
	registers_setup_MPU6050();
	//Chequeo el valor de los registros configurados
	wbuf[1]=0x0D;
	I2C_XFER_config(&xfer, rbuf, 10, MPU6050_I2C_DEVICE_ADDRESS, 0, wbuf, 1);
	for(i=0;i<10;i++)
		printf("CONF_REGS_CHK:\nrbuf[%d]=%d\n",i,rbuf[i]);
	//Delay que se usa despues de setear los pwr_mgmt
	printf("time since startup:%d\n",millis());
    delay(200);

	wbuf[1]=0x19;
	I2C_XFER_config(&xfer, rbuf, 7, MPU6050_I2C_DEVICE_ADDRESS, 0, wbuf, 1);
	/*
	 * SMPLRT_DIV
	 * CONFIG R/W
	 * GYRO_CONFIG
	 * ACCEL_CONFIG
	 * MOT_THR
	 */
	/*==================[LOOP]==========================*/

	//Porque quiero levantar 10 muestras, hago esto para probar
	//solo me va a servir para encajar 10 muestras de una pero no es
	//util para ir polleando, de hecho es recontra bloqueante
	delay(10);
	printf("time since startup:%d\n",millis());
	//Leo el fifo count para ver cuantos bytes puso en la FIFO ya
	fifo_count =FIFO_Count();
	printf("Se almacenaron en la FIFO %d bytes\n",fifo_count);
	mpu_data * mdata  =  BurstRead(10,12);//10 muestras de 16 bits por leer y 12 bytes por la configuracion del registro FIFO_EN


	printf("time since startup:%d\n",millis());
	printf("ax:%d,ay:%d,az:%d|gx:%d,gy:%d,gz:%d\n",mdata->ax,mdata->ay,mdata->az,mdata->gx,mdata->gy,mdata->gz);

	while(1)
	{
	}

}


/*==================[end of file]============================================*/
