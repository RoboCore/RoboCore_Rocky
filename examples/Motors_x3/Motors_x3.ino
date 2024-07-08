/*******************************************************************************
* RoboCore - Motors x3 (v1.0)
* 
* Demo of how to use the motors with the Rocky.
* This program uses the three motors (A, B and C) separately.
* 
* Copyright 2024 RoboCore.
* Written by Francois (17/01/2024).
* 
* 
* This file is part of the Rocky library by RoboCore ("RoboCore-Rocky-lib").
* 
* "RoboCore-Rocky-lib" is free software: you can redistribute it and/or modify
* it under the terms of the GNU Lesser General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
* 
* "RoboCore-Rocky-lib" is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU Lesser General Public License for more details.
* 
* You should have received a copy of the GNU Lesser General Public License
* along with "RoboCore-Rocky-lib". If not, see <https://www.gnu.org/licenses/>
*******************************************************************************/

// --------------------------------------------------
// Libraries

#include <RoboCore_Rocky.h>

// --------------------------------------------------
// Variables

RockyMotors motors;
const int PAUSE_TIME = 2000;

// --------------------------------------------------

void setup(){
  Serial.begin(115200);
}

// --------------------------------------------------

void loop(){
  Serial.println("All on 1");
  motors.setSpeedA(100);
  motors.setSpeedB(100);
  motors.setSpeedC(100);
  delay(PAUSE_TIME);

  Serial.println("Sleep");
  motors.sleep();
  delay(PAUSE_TIME);

  Serial.println("Wake up");
  motors.wakeup();
  delay(PAUSE_TIME);

  Serial.println("All off");
  motors.setSpeedA(0);
  motors.setSpeedB(0);
  motors.setSpeedC(0);
  delay(PAUSE_TIME);
  
  Serial.println("All on 2");
  motors.setSpeedA(-100);
  motors.setSpeedB(-100);
  motors.setSpeedC(-100);
  delay(PAUSE_TIME);

  Serial.println("All off");
  motors.setSpeedA(0);
  motors.setSpeedB(0);
  motors.setSpeedC(0);
  delay(PAUSE_TIME);
}

// --------------------------------------------------
