/*******************************************************************************
* RoboCore - Blink (v1.0)
* 
* Blink the LED of the Rocky Board.
* 
* Based on the example "Blink" by the Arduino team.
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
// Variables

const int PIN_LED = 15;
const int PAUSE_TIME = 1000; // [ms]

// --------------------------------------------------

void setup() {
  // set the pin as an output
  pinMode(PIN_LED, OUTPUT);
}

// --------------------------------------------------

void loop() {
  digitalWrite(PIN_LED, HIGH);
  delay(PAUSE_TIME);
  digitalWrite(PIN_LED, LOW);
  delay(PAUSE_TIME);
}

// --------------------------------------------------