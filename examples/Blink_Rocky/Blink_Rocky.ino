/*******************************************************************************
* RoboCore - Blink Rocky (v1.0)
* 
* Blink the LED of the Rocky Board.
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

RockyLED led;

// --------------------------------------------------

void setup() {
  led.blink(1000);
}

// --------------------------------------------------

void loop() {
  led.update();
}

// --------------------------------------------------