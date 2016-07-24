#ifndef IMU_H_
#define IMU_H_

#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
//#include "i2c.h"
#include "spi.h"
//#include "main.h"

#define READ_SPI 0x80
#define WRITE_SPI 0x00

//floating point vector
typedef struct fp_vector {		
  float X,Y,Z;
} t_fp_vector_def;

//this union allows the floating point vector to be accessed using myvector.x or myvector.A[0]
typedef union {		
  float A[3];		
  t_fp_vector_def V;		
} t_fp_vector;

//int32 vector
typedef struct int32_t_vector {
  int32_t X,Y,Z;
} t_int32_t_vector_def;

//this union allows the floating point vector to be accessed using myvector.x or myvector.A[0]
typedef union {
  int32_t A[3];
  t_int32_t_vector_def V;
} t_int32_t_vector;



//function prototypes
void IMU_Init_SPI(void);
void IMU_Init_I2C(int16_t* accelZero, int16_t* gyroZero);
void IMU_Read_SPI(int16_t* accelZero, int16_t* gyroZero, int16_t* GyroData, int16_t* AccelData);
void IMU_Read_SPI_Dump(uint8_t * dump); 
void IMU_Read_I2C(int16_t* accelZero, int16_t* gyroZero, int16_t* GyroData, int16_t* AccelData);
void IMU_Calibrate(int16_t *accelZero, int16_t *gyroZero);
void ACCEL_Calibrate(void);
//att_t getEstimatedAttitude(int16_t* GyroData, int16_t* AccelData);
//bool CrashDetect(int16_t* AccelData);

#endif
