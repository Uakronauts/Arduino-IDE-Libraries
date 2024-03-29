/*
* Nanhe Chen
* nanhe_chen@qq.com
* 
* Copyright (c) 2022 High Performance IMU BMI085
* 
* Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
* and associated documentation files (the "Software"), to deal in the Software without restriction, 
* including without limitation the rights to use, copy, modify, merge, publish, distribute, 
* sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is 
* furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be included in all copies or 
* substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING 
* BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND 
* NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, 
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, 
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "BMI085.h"

/* accel object */
BMI085Accel accel(Wire,0x18);
/* gyro object */
BMI085Gyro gyro(Wire,0x68);

volatile bool accel_flag, gyro_flag;

void accel_drdy()
{
  accel_flag = true;
}

void gyro_drdy()
{
  gyro_flag = true;
}

void setup() 
{
  int status;
  /* USB Serial to print data */
  Serial.begin(115200);
  while(!Serial) {}
  /* start the sensors */
  status = accel.begin();
  if (status < 0) {
    Serial.println("Accel Initialization Error");
    Serial.println(status);
    while (1) {}
  }
  status = accel.setOdr(BMI085Accel::ODR_100HZ_BW_19HZ);
  status = accel.pinModeInt1(BMI085Accel::PUSH_PULL,BMI085Accel::ACTIVE_HIGH);
  status = accel.mapDrdyInt1(true);


  status = gyro.begin();
  if (status < 0) {
    Serial.println("Gyro Initialization Error");
    Serial.println(status);
    while (1) {}
  }
  status = gyro.setOdr(BMI085Gyro::ODR_100HZ_BW_12HZ);
  status = gyro.pinModeInt3(BMI085Gyro::PUSH_PULL,BMI085Gyro::ACTIVE_HIGH);
  status = gyro.mapDrdyInt3(true);

  pinMode(3,INPUT);
  attachInterrupt(3,accel_drdy,RISING);
  pinMode(4,INPUT);
  attachInterrupt(4,gyro_drdy,RISING);  
}

void loop() 
{
  if (accel_flag && gyro_flag) {
    accel_flag = false;
    gyro_flag = false;
    /* read the accel */
    accel.readSensor();
    /* read the gyro */
    gyro.readSensor();
    /* print the data */
    Serial.print(accel.getAccelX_mss());
    Serial.print("\t");
    Serial.print(accel.getAccelY_mss());
    Serial.print("\t");
    Serial.print(accel.getAccelZ_mss());
    Serial.print("\t");
    Serial.print(gyro.getGyroX_rads());
    Serial.print("\t");
    Serial.print(gyro.getGyroY_rads());
    Serial.print("\t");
    Serial.print(gyro.getGyroZ_rads());
    Serial.print("\t");
    Serial.print(accel.getTemperature_C());
    Serial.print("\n");
  }
}