# Quadcopter
software for quadcopter control
This software is used to control quadcopter on Raspbian operating system and microcontroller.

##Arduino
This is the main flight controller, control code is in [**Arduino Code/arduino_control_code**](https://github.com/baixiaobest/quadcopter/tree/master/Arduino%20Code/arduino_control_code)
To determine the orientation of quadcopter, this project uses MPU6050 as IMU.
So four ESCs and one IMU is connected to Arduino. Arduino then takes the pose calculated by IMU to adjust
quadcopter pose by issuing command to ESC.
Control algorithm is implemented in three PID controllers of yaw, pitch and roll independently.

##Raspberry Pi
It is responsible for Video Recording and Streaming through LAN using TCP protocol,flight control
and radio communication.

#Reference
Reference contains some recent research results on quadcopter.
A detailed research of implementing VisualSLAM algorithm on quadcopter is included.
This algoritm allow quadcopter to learn new spatial envirnment during flight. But this feature is not yet implemented.

Baixiao Huang
huangbaixiao@g.ucla.edu
