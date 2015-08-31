
/*==================[inclusions]=============================================*/

#include "main.h"


#include "Infotronic.h"


#include <stdio.h>
/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/

/*==================[internal functions declaration]=========================*/


/** @brief hardware initialization function
 *	@return none
 */
static void initHardware(void);
//void InitMPU(void);
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



volatile uint8_t BufferSalidas;				//!< Buffer de Salidas Relay
volatile uint8_t BufferEntradas;				//!< Buffer de Entradas Digitales

uint8_t BuffRX[MAX_RX];
uint8_t BuffTX[MAX_TX];

uint8_t IndiceBuff_InRX = 0;
uint8_t IndiceBuff_OutRX = 0;
uint8_t IndiceBuff_InTX = 0;
uint8_t IndiceBuff_OutTX = 0;

volatile uint8_t flagTX = 0;
volatile uint8_t flagRX = 0;

char stringTX[7] = {'#','a','e','i','o','u','$'};


int main(void)
{
	/*==================[Inicializacion]==========================*/
	uint8_t wbuf[3] = {0,0,0};
	uint8_t rbuf[10] = {0,0,0,0,0,0,0,0,0,0};
	uint8_t * prbuf;
	int i=0;
	I2C_XFER_T xfer;
	uint16_t fifo_count=0;
	int samples_cant=0;
	char msg[100];

	//Del LPC
	initHardware();

	UART0_Init();

	/*==================[INET MPU CONF CHECK]==========================*/
	reg_set_regs_inet();//Setea todos los registros necesarios para utilizar la FIFO

	samples_cant=1; //Porque pinta, esto puede tomar cualquier valor

	mpu_data * mdata=(mpu_data *) calloc(samples_cant,sizeof(mpu_data));
	if (mdata==NULL)
	{
		return NULL;
	}

	prbuf=Read_FIFO(samples_cant, &mdata); //seteo para leer 10 samples porque si

	for(i=0;i<12;i++)
	{
		printf("PRBUF_FIFO_R_W_CHK:prbuf[%d]=%d\n",i,*(prbuf+i));
		sprintf(msg,"%d\n",*(prbuf+i));
		Transmitir(msg);
	}/*
	sprintf(msg,"a%d\n b%d\n c%d\n d%d\n e%d\n f%d\n g%d\n h%d\n i%d\n j%d\n k%d\n l%d\n",
			*(prbuf),*(prbuf+1),*(prbuf+2),*(prbuf+3),*(prbuf+4),*(prbuf+5),*(prbuf+6),*(prbuf+7)
			,*(prbuf+8),*(prbuf+9),*(prbuf+10),*(prbuf+11));
	//a: ACC_X_H
	//b: ACC_X_L
	//c: ACC_Y_H
	//d: ACC_Y_L
	//e: ACC_Z_H
	//f: ACC_Z_L
	//g: GYR_X_H
	//h: GYR_X_L
	//i: GYR_Y_H
	//j: GYR_Y_L
	//k: GYR_Z_H
	//l: GYR_Z_L

	Transmitir(msg);*/
	while(1)
	{
		//Lo que se podria hacer es una interrupcion que cada 1,5 ms
		//Haga un Read_FIFO de samples_cant=1, ya que cada 1ms habra
		//data nueva (12 bytes nuevos) y 1,5 ms esta en el medio del
		//intervalo de refresco
	}

}






/*
void InitMPU(void)
{
	/*==================[Configuracion del MPU]==========================*/
	/*MPU6050_wakeup(&xfer); //Despierto el MPU seteando PWR_MGMT_1 y 2 en 0x00
	//Inicializacion de los registros del MPU
	//Chequeo el valor de los registros configurados
	delay(100);
	wbuf[0]=MPU6050_RA_PWR_MGMT_1;
	for(i=0;i<10;i++)
		printf("INITIAL_VALUES:rbuf[%d]=%d\n",i,rbuf[i]);

	I2C_XFER_config(&xfer, rbuf, 2, MPU6050_I2C_DEVICE_ADDRESS, 0, wbuf, 1);
	for(i=0;i<2;i++)
		printf("PWR_MGMNT_CHK:rbuf[%d]=%d\n",i,rbuf[i]);

	for(i=0;i<10;i++)
		rbuf[i]= 0;

	/* get stable time source
	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	/*wbuf[0]=MPU6050_RA_PWR_MGMT_1;
	wbuf[1]=0x01;
	I2C_XFER_config(&xfer, rbuf, 0, MPU6050_I2C_DEVICE_ADDRESS, 0, wbuf, 2);
	delay(200);*/

	/*registers_setup_MPU6050();

	//Chequeo el valor de los registros configurados
	wbuf[0]=0x0D;
	I2C_XFER_config(&xfer, rbuf, 10, MPU6050_I2C_DEVICE_ADDRESS, 0, wbuf, 1);
	for(i=0;i<10;i++)
		printf("CONF_REGS_CHK:rbuf[%d]=%d\n",i,rbuf[i]);

	//Delay que se usa despues de setear los pwr_mgmt
	printf("time since startup:%d\n",millis());


	/*wbuf[0]=0x19;
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
	/*delay(10);
	printf("time since startup:%d\n",millis());
	//Leo el fifo count para ver cuantos bytes puso en la FIFO ya
	fifo_count =FIFO_Count();
	printf("Se almacenaron en la FIFO %d bytes\n",fifo_count);
	mpu_data * mdata  =  BurstRead(10,12);//10 muestras de 16 bits por leer y 12 bytes por la configuracion del registro FIFO_EN


	printf("time since startup:%d\n",millis());
	printf("ax:%d,ay:%d,az:%d|gx:%d,gy:%d,gz:%d\n",mdata->ax,mdata->ay,mdata->az,mdata->gx,mdata->gy,mdata->gz);
}*/


/*==================[end of file]============================================*/
