#Flight Controller Code

arduino_control_code is the main controller program that will run on **Arduino** as a flight controller.
In order to run it, you need to download and install 
* [i2cdevlib](https://github.com/jrowberg/i2cdevlib)
* [MPU6050 library](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)I am using this chip for IMU,
  you can use other chips available
* [pid library](http://playground.arduino.cc/Code/PIDLibrary)
* [nRF24 library](https://github.com/maniacbug/RF24) You can use other radio module like xBee, which is easier to deal with

There are some other sample code to setup ESCs and communication with MPU6050
