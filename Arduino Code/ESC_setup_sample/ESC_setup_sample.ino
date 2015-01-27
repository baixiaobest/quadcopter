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
#include <Servo.h>


int MAX_ESC_RATE = 2000;
int MIN_ESC_RATE = 800;
int value = MIN_ESC_RATE; // set values you need to zero

Servo ESC1, ESC2, ESC3, ESC4; //Create as much as Servoobject you want. You can controll 2 or more Servos at the same time

void setup() {

  ESC1.attach(5); 
  //ESC2.attach(9);
  //ESC3.attach(10);
  //ESC4.attach(11);
  Serial.begin(9600);    // start serial at 9600 baud
  
  //Serial.println("Setting up max throttle");
  //ESC1.writeMicroseconds(MAX_ESC_RATE);
  //while(!Serial.available());
  //Serial.read();
  
  Serial.println("Setting up min throttle");
  ESC1.writeMicroseconds(MIN_ESC_RATE);
  //ESC2.writeMicroseconds(MIN_ESC_RATE);
  //ESC3.writeMicroseconds(MIN_ESC_RATE);
  //ESC4.writeMicroseconds(MIN_ESC_RATE);
  while(!Serial.available());
  Serial.read();
  
}

void loop() {

//First connect your ESC WITHOUT Arming. Then Open Serial and follo Instructions
 
  ESC1.writeMicroseconds(value);
  //ESC2.writeMicroseconds(value);
  //ESC3.writeMicroseconds(value);
  //ESC4.writeMicroseconds(value);
 
  if(Serial.available()){ 
    value = Serial.parseInt();    // Parse an Integer from Serial
    Serial.println(value);
  }

}

