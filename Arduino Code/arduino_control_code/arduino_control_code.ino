/******************************************************************************  
*   ©copyright Baixiao Huang
*   
*   This file is part of Quadcopter.
*   
*   Quadcopter is free software: you can redistribute it and/or modify
*   it under the terms of the GNU General Public License as published by
*   the Free Software Foundation, either version 3 of the License, or
*   (at your option) any later version.
*   
*   Quadcopter is distributed in the hope that it will be useful,
*   but WITHOUT ANY WARRANTY; without even the implied warranty of
*   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
*   GNU General Public License for more details.
*   
*   You should have received a copy of the GNU General Public License
*   along with Foobar.  If not, see <http://www.gnu.org/licenses/>. 
******************************************************************************/
//select orientation representation mode here
//#define OUTPUT_READABLE_QUATERNION
//#define OUTPUT_READABLE_EULER
#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_READABLE_REALACCEL
//#define OUTPUT_TEAPOT

/*********************************INCLUDE LIBRARY HERE**************************/

//You need to install I2Cdev library and MPU6050 library, and place it
//under library folder of your Arduino path
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>
/*******************************************************************************/

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;
// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

/******************************VARIABLES DECLARE HERE****************************/

//MPU variables
bool dmpReady = false;  //state of DMP initializtion
uint8_t mpuIntStatus;  //holds 1 byte of interrupt status from MPU6050 int_status register
//it should be updated upon an interrupt
uint8_t devStatus; //device status after each opertaion on device 0 for success
uint16_t packetSize; //DMP packet size default to 42 bytes
uint16_t fifoCount; //courrently available data size in fifo buffer of MPU6050
uint8_t fifoBuffer[64]; //storage for FIFO data

//orientation variable
Quaternion q;         //[w, x, y, z]
VectorInt16 aa;       //[x, y, z]           acceleration sensor measurements
VectorInt16 aaReal;    //[x, y, z]           gravity-free acceleration in local reference frame
VectorInt16 aaWorld;  //[x, y, z]           gravity-free acceleration in world reference frame
VectorFloat gravity;  //[x, y, z]           gravity vector
float euler[3];       //[psi, theta, phi]   Euler angle
float ypr[3];         //[yaw, pitch, roll]  yaw/pitch/roll container and gravity vector

//teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

//setup for ESC
int MAX_ESC_RATE = 2000;
int MIN_ESC_RATE = 800;

Servo ESC1, ESC2, ESC3, ESC4;
int ESC1_CTR_PIN = 6;
int ESC2_CTR_PIN = 9;
int ESC3_CTR_PIN = 10;
int ESC4_CTR_PIN = 11;

/***********************************************************
****                 interrupt routine                  ****
***********************************************************/
volatile bool mpuInterrupt = false;
void interruptRoutine()
{
  mpuInterrupt = true;
}

/***********************************************************
****                       SET UP                       ****
***********************************************************/

void setup()
{
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  Serial.begin(115200);
  //initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  //initialize DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  
  //gyro and accelerometer offset
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788);
  
  if(devStatus == 0)
  {
    //turen on DMP
    mpu.setDMPEnabled(true);
    //enable interrupt on Arduino
    attachInterrupt(0, interruptRoutine, RISING);
    mpuIntStatus = mpu.getIntStatus();
    dmpReady = true;
    //get packet size of DMP packet
    packetSize = mpu.dmpGetFIFOPacketSize();
    //attach free fall interrupt
    mpu.setIntEnabled(0x82);
    //mpu.setIntFreefallEnabled(true);
    //mpu.setFreefallDetectionCounterDecrement(uint8_t(1));
    mpu.setFreefallDetectionThreshold(500);
    mpu.setFreefallDetectionDuration(150);
  }else{
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
  
}


void loop(){}
