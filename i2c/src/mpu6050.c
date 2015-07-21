
#include "mpu6050.h"
#include "i2c.h"
#include <stdlib.h>
#include "inet_defines.h"
#include "time_mgmt.h"

void MPU6050_wakeup(I2C_XFER_T * xfer)
{
		//Setea PWR_MGMT_1 y 2 en 0, el byte de cada uno


		uint8_t wbuf[3] = {MPU6050_RA_PWR_MGMT_1, 0, 0};
		/*xfer->slaveAddr = MPU6050_DEVICE_ADDRESS;
		xfer->txBuff = wbuf;
		xfer->txSz = 3;
		xfer->rxSz = 0;*/

		I2C_XFER_config(xfer, xfer->rxBuff, 0, MPU6050_I2C_DEVICE_ADDRESS, 0, wbuf, 3);
}

void Fill_Mpu_Data(mpu_data * mdata, uint8_t * rbuf)
{
	//De momento leer rbuf es lo mismo que leer xfer.rxBuff porque apuntan a la misma direccion
	//Desplazamos la parte alta a los 8 ultimos bits y le hacemos un OR para tener en los 8 primeros la parte baja

	mdata->ax=(rbuf[0] << 8) | rbuf[1];
	mdata->ay=(rbuf[2] << 8) | rbuf[3];
	mdata->az=(rbuf[4] << 8) | rbuf[5];
	mdata->gx=(rbuf[6] << 8) | rbuf[7];
	mdata->gy=(rbuf[8] << 8) | rbuf[9];
	mdata->gz=(rbuf[10] << 8) | rbuf[11];
}


void registers_setup_MPU6050(void)
{

	I2C_XFER_T xfer;
	uint8_t rbuf[1]={0};
	uint8_t wbuf1[] = {
			MPU6050_RA_SELF_TEST_X,//first register (self_text_x)
		    0,0,0,0 //self test registers
		    ,0 // 00000000   /sampling rate setting, 1kHz/(0+1)=1kHz
		    ,1 // 00 000 001 / xx [extsync] [dlfp] / digital LPF setting, 1kHz-184Hz band Accel & 1kHz-188Hz band Gyro
		    ,0 // 000 00 000 / [STx][y][z] [FS] 000 / gyro scale setting, 250deg/s
		    ,0 // 000 00 000/ [STx][y][z] [FS] 000 / acc scale setting, 2g
		    //MOT_THR queda igual como viene default, no lo uso
		    // FIFO setting, hago que guarde en la fifo todos los bytes de los registros de gyro y accel
			,78//0 111 1 000 [temp][gyro x][y][z] [accxyz] [slv2][1][0] / FIFO mask
			,2 // 0 0 000 010 / - [FIFOen] [I2CMST][I2CDIS]- [FIFOreset][I2CMSTrst][SIGreset] / I2C Master Reset
	};

	printf("REG_SETUP_W1_SIZE:%d\n",(int) sizeof(wbuf1)/sizeof(uint8_t));
	I2C_XFER_config(&xfer, rbuf, 0, MPU6050_I2C_DEVICE_ADDRESS, 0, wbuf1, (int) sizeof(wbuf1)/sizeof(uint8_t));

	uint8_t wbuf2[] = {MPU6050_RA_USER_CTRL,0x40}; ///0 1 000 000 / - [FIFOen] [I2CMST][I2CDIS]- [FIFOreset][I2CMSTrst][SIGreset] / Enable FIFO

	printf("REG_SETUP_W2_SIZE:%d\n",(int) sizeof(wbuf2)/sizeof(uint8_t));
	I2C_XFER_config(&xfer, rbuf, 0, MPU6050_I2C_DEVICE_ADDRESS, 0, wbuf2, (int) sizeof(wbuf2)/sizeof(uint8_t));
}


/*
 * Esta funcion setea todos los registros que necesito para leer datos de la FIFO
 * Y FUNCIONA
 */
void reg_set_regs_inet(void)
{
	I2C_XFER_T xfer;//CHECK
	uint8_t rbuf[]={0,0};//CHECK
	uint8_t * fifo_count_data = NULL;
	uint8_t wbuf[]={0};//CHECK
	int i=0;//CHECK


    uint16_t packet_count, fifo_count;

	//CHECK
	wbuf[0]=WHO_AM_I_MPU6050;
	I2C_XFER_config(&xfer, rbuf, 1, MPU6050_I2C_DEVICE_ADDRESS, 0, wbuf, 1);
	printf("WHO_AM_I_CHK:rbuf[0]=%d\n",rbuf[i]);


	// reset device, reset all registers, clear gyro and accelerometer bias registers
	I2C_WriteByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	delay(100);							//El valor por defecto del registro luego del reset es el 0x40

	//CHECK
	wbuf[0]=MPU6050_RA_PWR_MGMT_1;
	I2C_XFER_config(&xfer, rbuf, 2, MPU6050_I2C_DEVICE_ADDRESS, 0, wbuf, 1);
	for(i=0;i<2;i++)
		printf("PWR_MGMNT_CHK:rbuf[%d]=%d\n",i,rbuf[i]);

	// get stable time source
	// Set clock source to be PLL with x-axis gyroscope reference, bits 2:0 = 001
	I2C_WriteByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x01);
	I2C_WriteByte(MPU6050_ADDRESS, PWR_MGMT_2, 0x00);
	delay(200);

	// Configure device for bias calculation
	I2C_WriteByte(MPU6050_ADDRESS, INT_ENABLE, 0x00);   // Disable all interrupts
	I2C_WriteByte(MPU6050_ADDRESS, FIFO_EN, 0x00);      // Disable FIFO
	I2C_WriteByte(MPU6050_ADDRESS, PWR_MGMT_1, 0x00);   // Turn on internal clock source
	I2C_WriteByte(MPU6050_ADDRESS, I2C_MST_CTRL, 0x00); // Disable I2C master
	I2C_WriteByte(MPU6050_ADDRESS, USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	I2C_WriteByte(MPU6050_ADDRESS, USER_CTRL, 0x0C);    // Reset FIFO and DMP
	delay(15);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	I2C_WriteByte(MPU6050_ADDRESS, CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	I2C_WriteByte(MPU6050_ADDRESS, SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	I2C_WriteByte(MPU6050_ADDRESS, GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	I2C_WriteByte(MPU6050_ADDRESS, ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity


	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	I2C_WriteByte(MPU6050_ADDRESS, USER_CTRL, 0x40);   // Enable FIFO
	I2C_WriteByte(MPU6050_ADDRESS, FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 1024 bytes in MPU-6050)


	//Ahora puedo leer los datos que estan en la FIFO
}

/*
 * La idea de que mdata sea ** es que se guarden directamente
 * en la estructura que se creo en el main los datos de la FIFO que se lean
 */
void Read_FIFO(int samples_cant, mpu_data ** mdata)
{
	//uint16_t fifo_count_check=0,packet_count_check=0; //CHECK
	I2C_XFER_T xfer;//CHECK
	uint8_t rbuf[130], *prbuf ;
	uint8_t wbuf[]={FIFO_R_W};


	int i =0, j=0, ii=0;
	uint8_t * fifo_count_data =0;
	uint16_t fifo_count =0,packet_count=0;

	for (i=0;i<130;i++)
	{
		rbuf[i]=0;
	}
	//uint8_t * rbuf;
	//TODO: Por que sucede esto a partir del 39????
	//delay(80); // accumulate 80 samples in 80 milliseconds = 960 bytes (FUNCIONA Y DEVUELVE 80 SAMPLES)
	//delay(39); // accumulate 80 samples in 80 milliseconds = 960 bytes (A PARTIR DE ESTE PUNTO PARA ABAJO TOMA 1 SAMPLE MENOS)
	if(samples_cant<39) //HOTFIX VILLERO
	{
		delay(samples_cant+1);
	}
	else
	{
		delay(samples_cant);
	}

	// At end of sample accumulation, turn off FIFO sensor read
	I2C_WriteByte(MPU6050_ADDRESS, FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	fifo_count_data=I2C_ReadBytes(MPU6050_ADDRESS, FIFO_COUNTH, 2); // read FIFO sample count
	fifo_count = ((uint16_t)(*fifo_count_data) << 8) | (*(fifo_count_data+1));
	//packet_count tendria que tener el mismo valor que samples_cant
	packet_count = fifo_count/12;// How many sets of full gyro and accelerometer data for averaging
	printf("fifo_count:%d\npacket_count:%d\n",fifo_count,packet_count);
	//CHECK
	//fifo_count_check=FIFO_Count();
	//packet_count_check=fifo_count_check/12;
	//printf("fifo_count_check:%d\npacket_count_check:%d\n",fifo_count_check,packet_count_check);



	//CHECK
	wbuf[0]=WHO_AM_I_MPU6050;
	I2C_XFER_config(&xfer, rbuf, 1, MPU6050_I2C_DEVICE_ADDRESS, 0, wbuf, 1);
	printf("WHO_AM_I_CHK:rbuf[0]=%d\n",rbuf[0]);

	/*wbuf[0]=FIFO_R_W;
	I2C_XFER_config(&xfer, rbuf, 120, MPU6050_I2C_DEVICE_ADDRESS, 0, wbuf, 1);
	for(i=0;i<130;i++)
	{
		printf("RBUF_FIFO_R_W_CHK:rbuf[%d]=%d\n",i,rbuf[i]);
	}*/
	//LECTURA FIFO

	prbuf = I2C_ReadBytes(MPU6050_ADDRESS, FIFO_R_W, 120); // 120 porque son 10 samples de 12 bytes de accel y gyro leidos (2*(2*3) de cada uno)
	/*for(j=0;j<120;j++)
	{
		printf("PRBUF_FIFO_R_W_CHK:prbuf[%d]=%d\n",j,*(prbuf+j));
	}*/

	return prbuf;

    	/*
    	 * IMPLEMENTACION CON ESTRUCTURA mpu_data
    	prbuf = I2C_ReadBytes(MPU6050_ADDRESS, FIFO_R_W, 120); // 12 porque son 12 los bytes de accel y gyro leidos (2*(2*3) de cada uno)
    	ii=0;
    	for (i = 0; i < packet_count; i++) {
		(*(mdata+i))->ax = (uint16_t) (((uint16_t)*(prbuf+ii+0) << 8) | *(prbuf+ii+1)  ) ;  // Form signed 16-bit integer for each sample in FIFO
		(*(mdata+i))->ay = (uint16_t) (((uint16_t)*(prbuf+ii+2) << 8) | *(prbuf+ii+3)  ) ;
		(*(mdata+i))->az = (uint16_t) (((uint16_t)*(prbuf+ii+4) << 8) | *(prbuf+ii+5)  ) ;
		(*(mdata+i))->gx = (uint16_t) (((uint16_t)*(prbuf+ii+6) << 8) | *(prbuf+ii+7)  ) ;
		(*(mdata+i))->gy = (uint16_t) (((uint16_t)*(prbuf+ii+8) << 8) | *(prbuf+ii+9)  ) ;
		(*(mdata+i))->gz = (uint16_t) (((uint16_t)*(prbuf+ii+10) << 8)| *(prbuf+ii+11) ) ;
		ii+=12;	}
    //Se supone que en este punto estan todas las samples guardadas en el vector
    //de estructuras de mdata
    for(i=0;i<packet_count;i++)
    {
    	printf("(ax:%d,ay:%d,az:%d|gx:%d,gy:%d,gz:%d",(*(mdata+i))->ax,(*(mdata+i))->ay,(*(mdata+i))->az,(*(mdata+i))->gx,(*(mdata+i))->gy,(*(mdata+i))->gz);
    }*/
}

























/*
 * Nosotros seguramente implementemos la lectura de la Fifo en funcion de 1 de dos cosas:
 * 	-Interrupcion externa por el PIN INT de la plaqueta del MPU
 * 	-Interrupcion de timer sabiendo que cada X cant de tiempo, se toma una muestra de accel
 * 		y gyro y se lo manda a la FIFO. El tiempo que tiene que pasar entre cada interrupcion
 * 		de timer tiene que estar entre cada muestra nueva ingresada (ej si el SMPL_RT es 1k->1ms
 * 		-> tendriamos que tener un tmr_time mayor a 1ms y menor a 2 ms (1ms < tmr_time < 2ms) ej 1,5)
 *
 * 	En esta funcion de ejemplo se llama a la funcion delay(n) siendo n la cantidad de ms que queremos
 * 	que pase, que coinciden con el T_RT y por lo tanto con la cantidad de muestras que se almacenaran
 * 	en la fifo.
 */
//cant es la cantidad de samples que se desean leer
//TODO: hacer esta funcion generica y poner el resto en una que se llame MPU_DATA_OBTENTION
mpu_data * BurstRead (int cant, int fifo_size)
{
	int i =0;
	I2C_XFER_T xfer;
	uint8_t wbuf [] = {MPU6050_RA_FIFO_R_W};
	uint8_t rbuf[] = {0,0,0,0,0,0,0,0,0,0,0,0};

	//Voy a guardar en un vector de estructura de datos de gyro y accel todos los samples
	//obtenidos (cant samples)
	mpu_data * mdata = (mpu_data*) calloc (cant,(sizeof(mpu_data)));
    if(mdata==NULL)
    	return NULL;

	for (i=0; i<cant; i++)
	{
		//Pongo que se van a leer 12 por los 3 LH del gyro y los 3 LH del accel
		//configurados previamente para guardarse en la FIFO a la SMPL_RT
		I2C_XFER_config(&xfer, rbuf, fifo_size, MPU6050_I2C_DEVICE_ADDRESS, 0, wbuf, 1);
		Fill_Mpu_Data(&mdata[i],rbuf);
	}
	return mdata;

}


/*
 * Devuelve la cantidad de bytes que se
 * almacenaron en la fifo
 */
uint16_t FIFO_Count(void)
{

	uint8_t wbuf[1] = {MPU6050_RA_FIFO_COUNTH};
	uint8_t rbuf[2]={0,0};
	uint16_t count =0;
	I2C_XFER_T xfer;

	I2C_XFER_config(&xfer, rbuf, 2, MPU6050_I2C_DEVICE_ADDRESS, 0, wbuf, 1);

	count = (rbuf[0] << 8) | rbuf[1];

	return count;
}

