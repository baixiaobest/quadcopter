#include <Servo.h>


int MAX_ESC_RATE = 2000;
int MIN_ESC_RATE = 800;
int value = MIN_ESC_RATE; // set values you need to zero

Servo ESC1; //Create as much as Servoobject you want. You can controll 2 or more Servos at the same time

void setup() {

  ESC1.attach(10);    // attached to pin 10 I just do this with 1 Servo
  Serial.begin(9600);    // start serial at 9600 baud
  
  Serial.println("Setting up max throttle");
  ESC1.writeMicroseconds(MAX_ESC_RATE);
  while(!Serial.available());
  Serial.read();
  
  Serial.println("Setting up min throttle");
  ESC1.writeMicroseconds(MIN_ESC_RATE);
  while(!Serial.available());
  Serial.read();
  
}

void loop() {

//First connect your ESC WITHOUT Arming. Then Open Serial and follo Instructions
 
  ESC1.writeMicroseconds(value);
 
  if(Serial.available()){ 
    value = Serial.parseInt();    // Parse an Integer from Serial
    Serial.println(value);
  }

}

