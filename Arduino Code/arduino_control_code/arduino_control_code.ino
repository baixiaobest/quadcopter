/******************************************************************************  
*   ©copyright 2015 Baixiao Huang
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

/************************DESCRIPTION OF QUADCOPTER ROTORS ARRANGEMENT***********
                  motor 0 CCW    motor 3 CW               X
                       (O)        (O)                     ^
                        \         /                       |
                          \  $  /                         |
                            $$$                           |
                            $$$               Y<----------+
                          /     \
                        /         \
                      (O)          (O)
                  motor 1 CW    motor 2 CCW
/********************************************************************************/
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
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
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

/*********************************SETUP FOR ESC************************************/
int MAX_ESC_RATE = 2000;
int MIN_ESC_RATE = 800;

Servo ESC1, ESC2, ESC3, ESC4;
 int ESC1_CTR_PIN = 9;
 int ESC2_CTR_PIN = 10;
 int ESC3_CTR_PIN = 5;
 int ESC4_CTR_PIN = 6;

int ESCs_pid_offset[4] = {0,0,0,0};

int throttle = MIN_ESC_RATE;

//flight status
enum flightMode{GROUND, TAKEOFF, LANDING, ALTITUDE_HOLD, FREE};
int flightStatus = GROUND;

/*********************************SETUP FOR RADIO************************************/
const uint64_t pipe = 0xE8E8F0F0E1LL;
#define CE_PIN   4
#define CSN_PIN 7
RF24 radio(CE_PIN, CSN_PIN); 
char data;


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
    double* m_current_ypr;
    double* m_destined_ypr;
    double error_tolerance; //error allowed in degrees
  public:
    PID_Pose(double *current_ypr, double *output, double *destined_ypr, 
             double *k_yaw, double *k_pitch, double *k_roll,int sample_rate);
    void compute();
    void interpret_output(double* raw_output, int* ESC_output);
    void setYawPid(double Kp, double Ki, double Kd){yaw_pid.SetTunings(Kp, Ki, Kd);}
    void setPitchPid(double Kp, double Ki, double Kd){pitch_pid.SetTunings(Kp, Ki, Kd);}
    void setRollPid(double Kp, double Ki, double Kd){roll_pid.SetTunings(Kp, Ki, Kd);}
};


/*Initialize the PID control for yaw/pitch/roll
orientation is represented in yaw/pitch/roll format
you need to supply current ypr, container for raw PID output, destined ypr, PID constants for
yaw pitch and roll, and sample rate. the default sample rate is 50ms*/

PID_Pose::PID_Pose(double *current_ypr, double *output, double *destined_ypr, 
                   double *k_yaw, double *k_pitch, double *k_roll, int sample_rate=50)
: yaw_pid(&current_ypr[0], &output[0], &destined_ypr[0], k_yaw[0], k_yaw[1], k_yaw[2], DIRECT),
  pitch_pid(&current_ypr[1], &output[1], &destined_ypr[1], k_pitch[0], k_pitch[1], k_yaw[2], DIRECT),
  roll_pid(&current_ypr[2], &output[2], &destined_ypr[2], k_roll[0], k_roll[1], k_roll[2], DIRECT)
{
  m_current_ypr = current_ypr;
  m_destined_ypr = destined_ypr;
  yaw_pid.SetSampleTime(sample_rate);
  pitch_pid.SetSampleTime(sample_rate);
  roll_pid.SetSampleTime(sample_rate);
  yaw_pid.SetMode(AUTOMATIC);
  pitch_pid.SetMode(AUTOMATIC);
  roll_pid.SetMode(AUTOMATIC);
  yaw_pid.SetOutputLimits(-500,500);
  pitch_pid.SetOutputLimits(-500,500);
  roll_pid.SetOutputLimits(-500,500);
  error_tolerance = 3;
}

/*This function should be call every loop, if elapsed time between last time
pid is evaluated and current time is larger than sample_rate, this function will 
compute the pid raw_output for three aixs of orientation (yaw/pitch/roll)*/
void PID_Pose::compute()
{
  yaw_pid.Compute();
  pitch_pid.Compute();
  roll_pid.Compute();
}

/*a mapping function from raw PID output to actual offset in four ESCs*/
void PID_Pose::interpret_output(double* raw_output, int* ESC_output)
{
  ESC_output[0]=0; ESC_output[1]=0; ESC_output[2]=0;ESC_output[3]=0;
  if(abs(m_destined_ypr[0]-m_current_ypr[0]) >= error_tolerance)
  {
      ESC_output[0] += raw_output[0];
      ESC_output[2] += raw_output[0];
      ESC_output[1] -= raw_output[0];
      ESC_output[3] -= raw_output[0];
  }
  if(abs(m_destined_ypr[1]-m_current_ypr[1]) >= error_tolerance)
  {

      ESC_output[0] += raw_output[1];
      ESC_output[1] -= raw_output[1];
      ESC_output[2] -= raw_output[1];
      ESC_output[3] += raw_output[1];
  }
  if(abs(m_destined_ypr[2]-m_current_ypr[2]) >= error_tolerance)
  {

      ESC_output[0] += raw_output[2];
      ESC_output[1] += raw_output[2];
      ESC_output[2] -= raw_output[2];
      ESC_output[3] -= raw_output[2];
  }
}

/************************************PID CONTROLLER******************************/
double Kp_yaw = 0.7;
double Ki_yaw = 0;
double Kd_yaw = 0;
double Kp_pitch_roll = 0;
double Ki_pitch_roll = 0;
double Kd_pitch_roll = 1;
double K_yaw_pid[3] = {Kp_yaw, Ki_yaw, Kd_yaw};
double K_pitch_pid[3] = {Kp_pitch_roll, Ki_pitch_roll, Kd_pitch_roll};
double K_roll_pid[3] = {Kp_pitch_roll, Ki_pitch_roll, Kd_pitch_roll};
double current_ypr[3]={0,0,0};                //a copy of ypr, making it compatible with pid library
double output[3];                             //raw output of pid
double destined_ypr[3] = {0,0,0};
PID_Pose pose(current_ypr, output, destined_ypr, K_yaw_pid, K_pitch_pid, K_roll_pid,10);

int last_time = millis();
/***********************************************************
****                 INTERRUPT ROUTINE                  ****
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
  /*ESC1.writeMicroseconds(MAX_ESC_RATE);
  ESC2.writeMicroseconds(MAX_ESC_RATE);
  ESC3.writeMicroseconds(MAX_ESC_RATE);
  ESC4.writeMicroseconds(MAX_ESC_RATE);
  delay(3000);*/
  ESC1.writeMicroseconds(MIN_ESC_RATE);
  ESC2.writeMicroseconds(MIN_ESC_RATE);
  ESC3.writeMicroseconds(MIN_ESC_RATE);
  ESC4.writeMicroseconds(MIN_ESC_RATE);
  //while(!radio.available());
  /*while(!radio.available());
  char junk;
  radio.read(&junk, sizeof(char));*/
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
  Serial.println("Initializing I2C");
  mpu.initialize();
  Serial.println(mpu.testConnection() ? "MPU6050 successful" : "MPU6050 failed");
  //initialize DMP
  Serial.println("Initializing DMP");
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
    
    //setup radio
    Serial.println("Radio");
    radio.begin();
    radio.openReadingPipe(1,pipe);
    radio.startListening();
    
    //setup ESC
    Serial.println("ESCs");
    ESC_setup();
    Serial.println("Setup complete, Ready to fly");
  }else{
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print("DMP failed code ");
    Serial.print(devStatus);
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
  pose.interpret_output(output, ESCs_pid_offset);
  checkCommunication();
  controlESCs();
  /*Serial.print(F("ypr pid output: "));
  Serial.print(ESCs_pid_offset[0]);
  Serial.print(F("\t"));
  Serial.print(ESCs_pid_offset[1]);
  Serial.print(F("\t"));
  Serial.print(ESCs_pid_offset[2]);
  Serial.print(F("\t"));
  Serial.print(ESCs_pid_offset[3]);*/
  /*Serial.print("ypr: ");
  Serial.print(current_ypr[0]);
  Serial.print("\t");
  Serial.print(current_ypr[1]);
  Serial.print("\t");
  Serial.println(current_ypr[2]);*/
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
    Serial.println("FIFO overflows");
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
      Serial.println("fall");
      freeFallTakeOff();
    }
  }
}


/*********************************FREEFALL TAKEOFF*******************************/
void freeFallTakeOff()
{
  
}

/*************************************CONTROL************************************/
void controlESCs()
{
  if(throttle<MIN_ESC_RATE) throttle = MIN_ESC_RATE;
  int ESC1_out = throttle+ESCs_pid_offset[0] < MIN_ESC_RATE ? MIN_ESC_RATE : throttle+ESCs_pid_offset[0];
  int ESC2_out = throttle+ESCs_pid_offset[1] < MIN_ESC_RATE ? MIN_ESC_RATE : throttle+ESCs_pid_offset[1];
  int ESC3_out = throttle+ESCs_pid_offset[2] < MIN_ESC_RATE ? MIN_ESC_RATE : throttle+ESCs_pid_offset[2];
  int ESC4_out = throttle+ESCs_pid_offset[3] < MIN_ESC_RATE ? MIN_ESC_RATE : throttle+ESCs_pid_offset[3];
  ESC1.writeMicroseconds(ESC1_out);
  ESC2.writeMicroseconds(ESC2_out);
  ESC3.writeMicroseconds(ESC3_out);
  ESC4.writeMicroseconds(ESC4_out);
  Serial.print(ESC1_out);
  Serial.print(" ");
  Serial.print(ESC2_out);
  Serial.print(" ");
  Serial.print(ESC3_out);
  Serial.print(" ");
  Serial.println(ESC4_out);

  
}

/**********************************COMMUNICATION*********************************/
void checkCommunication()
{
  if(Serial.available())
    throttle = Serial.parseInt();//get throttle from serial
    if(throttle == 101)
    {
      reset_yaw();
      reset_pitch();
      reset_roll();
    }
  if(radio.available())
  {
    radio.read(&data, sizeof(char)); //get throttle from radio
    int value = (int)data;
    if(value == 101) reset_yaw();
    throttle = 12*value+MIN_ESC_RATE;
    Serial.println(value);
  }
}

void reset_yaw()
{
  destined_ypr[0] = current_ypr[0]; //calibrate
  Serial.print("resetting yaw to ");
  Serial.println(current_ypr[0]);
}
void reset_pitch()
{
  destined_ypr[1] = current_ypr[1]; //calibrate
  Serial.print("resetting pitch to ");
  Serial.println(current_ypr[1]);
}
void reset_roll()
{
  destined_ypr[2] = current_ypr[2]; //calibrate
  Serial.print("resetting roll to ");
  Serial.println(current_ypr[2]);
}
