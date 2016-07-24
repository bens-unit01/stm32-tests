

#include "IMU.h"
//#include "tim.h"
//#include "DebugSerial.h"
//#include "LED.h"
#include "math.h"
#include "stdbool.h"

//This function will initialize the IMU registers (over SPI) with the correct configuration
//It will then perform calibration (takes 512 samples and averages them)
//It will then output the gyroZero and accelZero calibration value that is required for the read function

SPI_HandleTypeDef hspi1;

static uint8_t spi_write(uint8_t out){
uint8_t in; 
   HAL_StatusTypeDef error = HAL_SPI_TransmitReceive(&hspi1, &out ,&in, 1,1000); 
return in;
}


static void select(void){
		HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET); 
}

static void deselect(void){
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET);
}


/* SPI1 init function */
void MX_SPI1_Init(void)
{

  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
 // hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
	hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  HAL_SPI_Init(&hspi1);

}


uint8_t IMU_Init_SPI2(){

		__disable_irq();
	uint32_t delaycounter = 0	;
	  uint8_t response;
	   //FIRST OF ALL DISABLE I2C
    select();
    response=spi_write(MPUREG_USER_CTRL);
    response=spi_write(BIT_I2C_IF_DIS);
    deselect();
    //RESET CHIP
    select();
    response=spi_write(MPUREG_PWR_MGMT_1);
    response=spi_write(BIT_H_RESET); 
    deselect();

//	wait(0.15);
	//HAL_Delay(100); //wait 100 miliseconds for reboot // removed because it needs irq
	while(delaycounter < 100000)
	{
		delaycounter++;
	}
    //WAKE UP AND SET GYROZ CLOCK
    select();
    response=spi_write(MPUREG_PWR_MGMT_1);
    response=spi_write(MPU_CLK_SEL_PLLGYROZ); 
    deselect();
    //DISABLE I2C
    select();
    response=spi_write(MPUREG_USER_CTRL);
    response=spi_write(BIT_I2C_IF_DIS);
    deselect();
    //WHO AM I?
    select();
    response=spi_write(MPUREG_WHOAMI|READ_FLAG);
    response=spi_write(0x00);
    deselect();
 //   if(response<100){return 0;}//COULDN'T RECEIVE WHOAMI
    //SET SAMPLE RATE
    select();
    response=spi_write(MPUREG_SMPLRT_DIV);
    response=spi_write(1); 
    deselect();
    // FS & DLPF
    select();
    response=spi_write(MPUREG_CONFIG);
    response=spi_write(BITS_DLPF_CFG_5HZ);
    deselect();
    //DISABLE INTERRUPTS
    select();
    response=spi_write(MPUREG_INT_ENABLE);
    response=spi_write(0x00);
    deselect();
		
		__enable_irq();
    return 0;

}
void IMU_Init_SPI()
{
	__disable_irq();
	uint32_t delaycounter = 0	;
	uint8_t x[2];
	uint8_t y = 99;
	HAL_StatusTypeDef error = HAL_OK;
	//x[0] = 0x6A;// | WRITE_SPI; // disable I2C
	x[0] = 0x6A ; //| WRITE_SPI; // disable I2C
	x[1] = 0x10; //the data
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET); //chip select low
	error = HAL_SPI_TransmitReceive(&hspi1,&x[0],&y,1,1000); //disable I2C
	error = HAL_SPI_TransmitReceive(&hspi1,&x[1],&y,1,1000); //disable I2C
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); //chip select high
	
	x[0] = 0x6B | WRITE_SPI;
	x[1] = 0x80;  //write 0x80 to IMU register at address 0x6B
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET); //chip select low
	error = HAL_SPI_TransmitReceive(&hspi1,&x[0],&y,1,1000); //PWR_MGMT_1    -- DEVICE_RESET 1v
	error = HAL_SPI_TransmitReceive(&hspi1,&x[1],&y,1,1000); //PWR_MGMT_1    -- DEVICE_RESET 1v
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); //chip select high
	
	//HAL_Delay(100); //wait 100 miliseconds for reboot // removed because it needs irq
	while(delaycounter < 100000)
	{
		delaycounter++;
	}

	x[0] = 0x6B | WRITE_SPI;
	x[1] = 0x03;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET); //chip select low
	error = HAL_SPI_TransmitReceive(&hspi1,&x[0],&y,1,1000); //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
	error = HAL_SPI_TransmitReceive(&hspi1,&x[1],&y,1,1000); //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); //chip select high

	// disable I2C again
	x[0] = 0x6A | WRITE_SPI; 
	x[1] = 0x10; //the data
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET); //chip select low
	error = HAL_SPI_TransmitReceive(&hspi1,&x[0],&y,1,1000); 
	error = HAL_SPI_TransmitReceive(&hspi1,&x[1],&y,1,1000);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); //chip select high

	//CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)	
	x[0] = 0x1A | WRITE_SPI;
	//x[1] = 0x03; //mpu6050
	x[1] = 0x04;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4,GPIO_PIN_RESET); //chip select low
	error = HAL_SPI_TransmitReceive(&hspi1,&x[0],&y,1,1000); 
        error = HAL_SPI_TransmitReceive(&hspi1,&x[1],&y,1,1000);
        HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); //chip select high
	
	//Gyro Init
	//GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
	x[0] = 0x1B | WRITE_SPI;
	x[1] = 0x18;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET); //chip select low
        error = HAL_SPI_TransmitReceive(&hspi1,&x[0],&y,1,1000); 
	error = HAL_SPI_TransmitReceive(&hspi1,&x[1],&y,1,1000);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); //chip select high

	//Accelermoter init
	//ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
	x[0] = 0x1C | WRITE_SPI;
	x[1] = 0x10;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET); //chip select low
	error = HAL_SPI_TransmitReceive(&hspi1,&x[0],&y,1,1000); 
	error = HAL_SPI_TransmitReceive(&hspi1,&x[1],&y,1,1000);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); //chip select high
  //note: something seems to be wrong in the spec here. With AFS=2 1G = 4096 but according to my measurement: 1G=2048 (and 2048/8 = 256)
  //confirmed here: http://www.multiwii.com/forum/viewtopic.php?f=8&t=1080&start=10#p7480
	//4096 on the MPU6500
	
	//disable interrupts
	x[0] = 0x38 | WRITE_SPI;
	x[1] = 0x00;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET); //chip select low
	error = HAL_SPI_TransmitReceive(&hspi1,&x[0],&y,1,1000); 
	error = HAL_SPI_TransmitReceive(&hspi1,&x[1],&y,1,1000);
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); //chip select high
	
	__enable_irq();
	
}


//This will correctly read the Accel and Gyro Values from the MPU6050 over SPI
//int16_t* accelZero: input calibration result for accelerometer
//int16_t* gyroZero: input calibration result for gyroscope
//uint16_t* GyroData: will output array of 3 x 16 bit gyroscope values (x,y,z)
//int16_t* AccelData: will output array of 3 x 16 bit accelerometer values (x,y,z)
void IMU_Read_SPI(int16_t* accelZero, int16_t* gyroZero, int16_t* GyroData, int16_t* AccelData)
{
	uint8_t x[2];
	uint8_t RawGyroData[6];
	uint8_t RawAccelData[6];
	uint8_t discard;

	x[0] = (0x3B) | READ_SPI; //HIGH
	x[1] = 0x00;
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_RESET); //chip select low
	HAL_SPI_TransmitReceive(&hspi1,&x[0],&discard,1,1000);
  HAL_SPI_TransmitReceive(&hspi1,&x[1],&RawAccelData[0],1,1000);
/*	HAL_SPI_TransmitReceive(&hspi1,&x[1],&RawAccelData[1],1,1000);
	HAL_SPI_TransmitReceive(&hspi1,&x[1],&RawAccelData[2],1,1000);
	HAL_SPI_TransmitReceive(&hspi1,&x[1],&RawAccelData[3],1,1000);
	HAL_SPI_TransmitReceive(&hspi1,&x[1],&RawAccelData[4],1,1000);
	HAL_SPI_TransmitReceive(&hspi1,&x[1],&RawAccelData[5],1,1000);
	HAL_SPI_TransmitReceive(&hspi1,&x[1],&discard,1,1000); //temperature
	HAL_SPI_TransmitReceive(&hspi1,&x[1],&discard,1,1000); //temperature
	HAL_SPI_TransmitReceive(&hspi1,&x[1],&RawGyroData[0],1,1000);
	HAL_SPI_TransmitReceive(&hspi1,&x[1],&RawGyroData[1],1,1000);
	HAL_SPI_TransmitReceive(&hspi1,&x[1],&RawGyroData[2],1,1000);
	HAL_SPI_TransmitReceive(&hspi1,&x[1],&RawGyroData[3],1,1000);
	HAL_SPI_TransmitReceive(&hspi1,&x[1],&RawGyroData[4],1,1000);
	HAL_SPI_TransmitReceive(&hspi1,&x[1],&RawGyroData[5],1,1000);
	*/
	HAL_GPIO_WritePin(GPIOA,GPIO_PIN_4,GPIO_PIN_SET); //chip select high

	//converting data into correct 16 bit format
	*(uint16_t*)(&GyroData[0]) = RawGyroData[0] << 8 | RawGyroData[1]; //GyroX (ROLL)
	*(uint16_t*)(&GyroData[1]) = RawGyroData[2] << 8 | RawGyroData[3]; //GyroY (PITCH)
	*(uint16_t*)(&GyroData[2]) = RawGyroData[4] << 8 | RawGyroData[5]; //GyroZ (YAW)

	*(uint16_t*)(&AccelData[0]) = RawAccelData[0] << 8 | RawAccelData[1]; //AccelX
	*(uint16_t*)(&AccelData[1]) = RawAccelData[2] << 8 | RawAccelData[3]; //AccelY
	*(uint16_t*)(&AccelData[2]) = RawAccelData[4] << 8 | RawAccelData[5]; //AccelZ
	//remove the calibration results from the Gyrodata
	GyroData[0] -= gyroZero[0]; 
	GyroData[1] -= gyroZero[1];
	GyroData[2] -= gyroZero[2];
	//remove the calibration results from the AccelData
	AccelData[0] -= accelZero[0]; 
	AccelData[1] -= accelZero[1];
	AccelData[2] -= accelZero[2];
}


//dumps all registers over SPI
void IMU_Read_SPI_Dump(uint8_t * dump)
{
	uint8_t x[2];
	uint8_t discard;

	x[0] = (0x00) | READ_SPI; //read at this address
	x[1] = 0x00;
	
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET); //chip select low
//	HAL_SPI_TransmitReceive(&hspi1,&x[0],&discard,1,1000);
	for(int i = 0; i < 119 ; i++)
	{
//		HAL_SPI_TransmitReceive(&hspi1,&x[1],&dump[i],1,1000);	
	}
	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_SET); //chip select high
	
//	x[0] = (0x45) | READ_SPI; //read at this address
//	x[1] = 0x00;
//	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET); //chip select low
//	HAL_SPI_TransmitReceive(&hspi1,&x[0],&discard,1,1000);
//	HAL_SPI_TransmitReceive(&hspi1,&x[1],&dump[0],1,1000);
//	HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1,GPIO_PIN_RESET); //chip select low	
	
	
	
	
	

}


//This function will initialize the IMU registers (over I2C) with the correct configuration
//It will then perform calibration (takes 512 samples and averages them)
//It will then output the gyroZero and accelZero calibration value that is required for the read function
//void IMU_Init_I2C(int16_t* accelZero, int16_t* gyroZero)
//{
//	uint8_t x;
//	x = 0x80;
//	HAL_I2C_Mem_Write(&hi2c1,0xD0,0x6B,I2C_MEMADD_SIZE_8BIT,&x,1,1000); //PWR_MGMT_1    -- DEVICE_RESET 1
//	HAL_Delay (5); //wait 5 miliseconds for reboot
//	x = 0x03;
//	HAL_I2C_Mem_Write(&hi2c1,0xD0,0x6B,I2C_MEMADD_SIZE_8BIT,&x,1,1000); //PWR_MGMT_1    -- SLEEP 0; CYCLE 0; TEMP_DIS 0; CLKSEL 3 (PLL with Z Gyro reference)
//	x = 0x03;
//	HAL_I2C_Mem_Write(&hi2c1,0xD0,0x1A,I2C_MEMADD_SIZE_8BIT,&x,1,1000); //CONFIG        -- EXT_SYNC_SET 0 (disable input pin for data sync) ; default DLPF_CFG = 0 => ACC bandwidth = 260Hz  GYRO bandwidth = 256Hz)
//	x = 0x18;
//	//Gyro Init
//	HAL_I2C_Mem_Write(&hi2c1,0xD0,0x1B,I2C_MEMADD_SIZE_8BIT,&x,1,1000); //GYRO_CONFIG   -- FS_SEL = 3: Full scale set to 2000 deg/sec
//	//Accelermoter init
//	x = 0x10;
//	HAL_I2C_Mem_Write(&hi2c1,0xD0,0x1C,I2C_MEMADD_SIZE_8BIT,&x,1,1000);//ACCEL_CONFIG  -- AFS_SEL=2 (Full Scale = +/-8G)  ; ACCELL_HPF=0   //note something is wrong in the spec.
//  //note: something seems to be wrong in the spec here. With AFS=2 1G = 4096 but according to my measurement: 1G=2048 (and 2048/8 = 256)
//  //confirmed here: http://www.multiwii.com/forum/viewtopic.php?f=8&t=1080&start=10#p7480

//	IMU_Calibrate(accelZero, gyroZero); //in present configuration (with digital low pass filter) data will at most change 256 times every second (can sample every 3.9 ms)

//	
//}

//This will correctly read the Accel and Gyro Values from the MPU6050 over I2C
//int16_t* accelZero: input calibration result for accelerometer
//int16_t* gyroZero: input calibration result for gyroscope
//uint16_t* GyroData: will output array of 3 x 16 bit gyroscope values (x,y,z)
//int16_t* AccelData: will output array of 3 x 16 bit accelerometer values (x,y,z)
//void IMU_Read_I2C(int16_t* accelZero, int16_t* gyroZero, int16_t* GyroData, int16_t* AccelData)
//{
//	uint8_t x;
//	uint8_t i;
//	uint8_t RawGyroData[6];
//	uint8_t RawAccelData[6];
//	
//	for(i = 0 ; i < 6 ; i++) //read raw data from IMU
//	{
//		x = 0x43 + i;
//		HAL_I2C_Mem_Read(&hi2c1, 0xD0, x ,I2C_MEMADD_SIZE_8BIT,&RawGyroData[i],1,1000);
//		x = 0x3B + i;
//		HAL_I2C_Mem_Read(&hi2c1, 0xD0, x ,I2C_MEMADD_SIZE_8BIT,&RawAccelData[i],1,1000);
//	}

//	//converting data into correct 16 bit format
//	GyroData[0] = RawGyroData[0] << 8 | RawGyroData[1]; //GyroX (ROLL)
//	GyroData[1] = RawGyroData[2] << 8 | RawGyroData[3]; //GyroY (PITCH)
//	GyroData[2] = RawGyroData[4] << 8 | RawGyroData[5]; //GyroZ (YAW)
//	AccelData[0] = RawAccelData[0] << 8 | RawAccelData[1]; //AccelX
//	AccelData[1] = RawAccelData[2] << 8 | RawAccelData[3]; //AccelY
//	AccelData[2] = RawAccelData[4] << 8 | RawAccelData[5]; //AccelZ
//	//remove the calibration results from the Gyrodata
//	GyroData[0] -= gyroZero[0]; 
//	GyroData[1] -= gyroZero[1];
//	GyroData[2] -= gyroZero[2];
//	//remove the calibration results from the AccelData
//	AccelData[0] -= accelZero[0]; 
//	AccelData[1] -= accelZero[1];
//	AccelData[2] -= accelZero[2];
//	
//	//used to be stored in imu object in multiwii

//}


// ****************
// will output array of gyroZero[3] which must be substracted from each gyro reading
// will output array of accelZero[3] which must be substracted from each accel reading
// ****************
void IMU_Calibrate(int16_t *accelZero, int16_t *gyroZero) 
{
  int32_t g[3] = {0,0,0};
	int32_t a[3] = {0,0,0};
  uint8_t axis;
	uint16_t counter;
	int16_t GyroData[3], AccelData[3];
	
	//TurnOnLRGB(255,0,0,35000);
	
			for(axis = 0 ; axis < 3; axis++) 
			{
				//initialize offsets to zero;
				gyroZero[axis] = 0; 
				accelZero[axis] = 0; 
			}

			//sum up 512 readings for each gyro and accel axis
			for(counter = 0; counter <512; counter++)
			{
				HAL_Delay (5); //wait 5 miliseconds to avoid oversampling IMU (min delay between reads is 3.9ms, when no delay I measured 2ms. Could theoretically push this down to 2ms delay)
				//IMU_Read_I2C(accelZero, gyroZero, GyroData, AccelData); //take IMU reading (gyroZero and accelZero values are equal to zero at this point)
				IMU_Read_SPI(accelZero, gyroZero, GyroData, AccelData);
				for (axis = 0; axis < 3; axis++) 
				{
				g[axis] += GyroData[axis]; 
				a[axis] += AccelData[axis];
				}
			}
			
			for (axis = 0; axis < 3; axis++) 
			{
				gyroZero[axis]=(g[axis])>>9; //divide by 512 (take average)
				accelZero[axis] = (a[axis])>>9; //divide by 512 (take average)
			}
				//accelZero[2] -= 2048; //Take gravity into account for the Z-axis...MPU6050
				accelZero[2] -= 4096; //Take gravity into account for the Z-axis...MPU6500
			
	//TurnOnLRGB(0,0,0,0);
			
}


bool CrashDetect(int16_t* AccelData)
{
	float accelMag = 0.0;
	static float AccelDataLPF[3] = {0.0, 0.0, 0.0};
	
	for (int axis = 0; axis < 3; axis++) 
  {
		AccelDataLPF[axis] = (float)(AccelData[axis])*(0.09) + AccelDataLPF[axis] * ( 1.0 - 0.09); //filter accelerometer
		accelMag += AccelDataLPF[axis] * AccelDataLPF[axis]; 
  }
	accelMag = sqrtf(accelMag);

//	FloatPacket('f', accelMag / 4096.0, 10);
	
	if(accelMag > 14000) //14336) //crash detected (3.5g of force after filter) ... MPU6500		
	{
		return true;
	}
	else
	{	
		return false;
	}
	
	
}



