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
*   along with Quadcopter.  If not, see <http://www.gnu.org/licenses/>. 
******************************************************************************/
//select orientation representation mode here
#define OUTPUT_READABLE_YAWPITCHROLL
//#define OUTPUT_TEAPOT

/*********************************INCLUDE LIBRARY HERE**************************/

//You need to install I2Cdev library and MPU6050 library, and place it
//under library folder of your Arduino path
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <Servo.h>
#include <PID_v1.h>
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
VectorInt16 aaReal;    //[x, y, z]          gravity-free acceleration in local reference frame
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

//flight status
enum flightMode{GROUND, TAKEOFF, LANDING, ALTITUDE_HOLD, FREE};
int flightStatus = GROUND;


/***********************************************************
****                     PID Class                      ****
***********************************************************/

class PID_Pose
{
  private:
    VectorInt16 accWorld; //acceleration in world frame
    PID yaw_pid;
    PID pitch_pid;
    PID roll_pid;
  public:
    PID_Pose(double *current_ypr, double *output, double *destined_ypr, 
             double *k_yaw, double *k_pitch, double *k_roll,int sample_rate);
    void compute();
};

PID_Pose::PID_Pose(double *current_ypr, double *output, double *destined_ypr, 
                   double *k_yaw, double *k_pitch, double *k_roll, int sample_rate=50)
: yaw_pid(&current_ypr[0], &output[0], &destined_ypr[0], k_yaw[0], k_yaw[1], k_yaw[2], DIRECT),
  pitch_pid(&current_ypr[1], &output[1], &destined_ypr[1], k_pitch[0], k_pitch[1], k_yaw[2], DIRECT),
  roll_pid(&current_ypr[2], &output[2], &destined_ypr[2], k_roll[0], k_roll[1], k_roll[2], DIRECT)
{
  yaw_pid.SetSampleTime(sample_rate);
  pitch_pid.SetSampleTime(sample_rate);
  roll_pid.SetSampleTime(sample_rate);
  yaw_pid.SetMode(AUTOMATIC);
  pitch_pid.SetMode(AUTOMATIC);
  roll_pid.SetMode(AUTOMATIC);
  yaw_pid.SetOutputLimits(-500,500);
  pitch_pid.SetOutputLimits(-500,500);
  roll_pid.SetOutputLimits(-500,500);
}

void PID_Pose::compute()
{
  yaw_pid.Compute();
  pitch_pid.Compute();
  roll_pid.Compute();
}

/************************************PID CONTROLLER******************************/
double Kp_yaw = 1;
double Ki_yaw = 0;
double Kd_yaw = 0;
double Kp_pitch = 1;
double Ki_pitch = 0;
double Kd_pitch = 0;
double Kp_roll = 1;
double Ki_roll = 0;
double Kd_roll = 0;
double K_yaw_pid[3] = {Kp_yaw, Ki_yaw, Kd_yaw};
double K_pitch_pid[3] = {Kp_pitch, Ki_pitch, Kd_pitch};
double K_roll_pid[3] = {Kp_roll, Ki_roll, Kd_roll};
double current_ypr[3]={0,0,0};                //a copy of ypr, making it compatible with pid library
double output[3];                             //raw output of pid
double destined_ypr[3] = {0,0,0};
PID_Pose pose(current_ypr, output, destined_ypr, K_yaw_pid, K_pitch_pid, K_roll_pid,10);


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

void ESC_setup()
{
  ESC1.attach(ESC1_CTR_PIN);
  ESC2.attach(ESC2_CTR_PIN);
  ESC3.attach(ESC3_CTR_PIN);
  ESC4.attach(ESC4_CTR_PIN);
  ESC1.writeMicroseconds(MIN_ESC_RATE);
  ESC2.writeMicroseconds(MIN_ESC_RATE);
  ESC3.writeMicroseconds(MIN_ESC_RATE);
  ESC4.writeMicroseconds(MIN_ESC_RATE);
}

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
  mpu.setZAccelOffset(1350);
  
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
    mpu.setFreefallDetectionThreshold(50);
    mpu.setFreefallDetectionDuration(400);
    
    //setup ESC
    Serial.println(F("Setting up ESCs..."));
    ESC_setup();
    
    Serial.println(F("Setup complete, Ready to fly"));
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

/***********************************************************
****                    MAIN LOOP                       ****
***********************************************************/

void loop()
{
  if(!dmpReady) return;
  if(mpuInterrupt || fifoCount >= packetSize)
    interruptResponse();
  pose.compute();
  Serial.print("ypr pid output: ");
  Serial.print(output[0]);
  Serial.print("\t");
  Serial.print(output[1]);
  Serial.print("\t");
  Serial.print(output[2]);
  Serial.print("ypr: ");
  Serial.print(current_ypr[0]);
  Serial.print("\t");
  Serial.print(current_ypr[1]);
  Serial.print("\t");
  Serial.println(current_ypr[2]);
}

/***********************************************************
****                INTERRUPT RESPONSE                  ****
***********************************************************/

void interruptResponse()
{
  mpuInterrupt = false;
  //get interrupt status to know what kind of interrupt is issued
  mpuIntStatus = mpu.getIntStatus();
  //get FIFO count
  fifoCount = mpu.getFIFOCount();
  if((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    mpu.resetFIFO();
    Serial.println(F("FIFO overflows"));
    return;
   //Data is ready
  }
  else if(mpuIntStatus & 0x02){
    //wait for correct available data length
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    //get FIFO data
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    
    //get quaternion representation
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    //get gravity vector in local reference frame
    //gravity is represented in unit of g (9.8m/s^2)
    mpu.dmpGetGravity(&gravity, &q);
    
    //get YawPitchRoll in radian
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    current_ypr[0] = (double) ypr[0]*180.0/M_PI;
    current_ypr[1] = (double) ypr[1]*180.0/M_PI;
    current_ypr[2] = (double) ypr[2]*180.0/M_PI;
    
    //get gravity-free acceleration in local reference frame
    //aaReal and aa are represented in raw data, +1g = 8192 in DMP FIFO packet
    mpu.dmpGetAccel(&aa, fifoBuffer);
    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
    
    //get gravity-free acceleration in global reference frame
    mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
  }
  if(mpuIntStatus & 0x80)
  {
    if(flightStatus == GROUND){
      Serial.println("Initia free fall protection precedure");
    }
  }
}


/*********************************FREEFALL TAKEOFF*******************************/
void freeFallTakeOff()
{
  
}

