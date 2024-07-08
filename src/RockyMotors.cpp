/*******************************************************************************
* RoboCore Rocky Motors Library (v1.0)
* 
* Library to use the motors of the Rocky board.
* 
* Copyright 2024 RoboCore.
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

// Reference: https://docs.espressif.com/projects/arduino-esp32/en/latest/api/ledc.html

// --------------------------------------------------
// Libraries

#include "RoboCore_Rocky.h"

// --------------------------------------------------
// --------------------------------------------------

// Constructor
RockyMotors::RockyMotors(void) :
  _pinMA1(25),
  _pinMA2(26),
  _pinMB1(14),
  _pinMB2(13),
  _pinMC1(4),
  _pinMC2(27),
  _pinSleep(33),
  _pwm_channel_A(ROCKY_MOTORS_CHANNEL_A),
  _pwm_channel_B(ROCKY_MOTORS_CHANNEL_B),
  _pwm_channel_C(ROCKY_MOTORS_CHANNEL_C),
  _pwm_frequency(5000), // 5 kHz
  _pwm_resolution(10)  // 10 bits
{
  // configure the pins
  pinMode(this->_pinMA1, OUTPUT);
  pinMode(this->_pinMA2, OUTPUT);
  pinMode(this->_pinMB1, OUTPUT);
  pinMode(this->_pinMB2, OUTPUT);
  pinMode(this->_pinMC1, OUTPUT);
  pinMode(this->_pinMC2, OUTPUT);
  pinMode(this->_pinSleep, OUTPUT);

  // turn all channels off
  digitalWrite(this->_pinMA1, LOW);
  digitalWrite(this->_pinMA2, LOW);
  digitalWrite(this->_pinMB1, LOW);
  digitalWrite(this->_pinMB2, LOW);
  digitalWrite(this->_pinMC1, LOW);
  digitalWrite(this->_pinMC2, LOW);
  digitalWrite(this->_pinSleep, LOW);
  
  // configure the PWM
  this->_configurePWM();

  // default to stopped
  //this->stop();
}

// --------------------------------------------------

// Destructor
RockyMotors::~RockyMotors(void){
  // detach the pins from the PWM
  ledcDetach(*this->_active_pin_A);
  ledcDetach(*this->_active_pin_B);
  ledcDetach(*this->_active_pin_C);

  // set all pins as inputs
  pinMode(this->_pinMA1, INPUT);
  pinMode(this->_pinMA2, INPUT);
  pinMode(this->_pinMB1, INPUT);
  pinMode(this->_pinMB2, INPUT);
  pinMode(this->_pinMC1, INPUT);
  pinMode(this->_pinMC2, INPUT);
  pinMode(this->_pinSleep, INPUT);
}

// --------------------------------------------------
// --------------------------------------------------

// Set the motors to move backwards
//  @param (speed) : the speed of the motor (0-100) [uint8_t]
void RockyMotors::backward(uint8_t speed){
  // constrain the value
  if(speed > 100){
    speed = 100;
  }

  // update the directions
  this->_attachPin(&this->_pinMA2);
  this->_attachPin(&this->_pinMB2);

  this->_pwmA = map(speed, 0, 100, 0, this->_max_duty_cyle); // transform to the current configuration
  this->_pwmB = this->_pwmA;
  ledcWrite(*this->_active_pin_A, this->_pwmA); // update
  ledcWrite(*this->_active_pin_B, this->_pwmB); // update
  
  this->wakeup();
}

// --------------------------------------------------

// Set the motors to move forwards
//  @param (speed) : the speed of the motor (0-100%) [uint8_t]
void RockyMotors::forward(uint8_t speed){
  // constrain the value
  if(speed > 100){
    speed = 100;
  }

  // update the directions
  this->_attachPin(&this->_pinMA1);
  this->_attachPin(&this->_pinMB1);

  this->_pwmA = map(speed, 0, 100, 0, this->_max_duty_cyle); // transform to the current configuration
  this->_pwmB = this->_pwmA;
  ledcWrite(*this->_active_pin_A, this->_pwmA); // update
  ledcWrite(*this->_active_pin_B, this->_pwmB); // update
  
  this->wakeup();
}

// --------------------------------------------------

// Set the speed for motor A
//  @param (speed) : the speed of the motor (-100-100%) [int8_t]
void RockyMotors::setSpeedA(int8_t speed){
  this->_setMotorSpeed('A', speed);
}

// Set the speed for motor B
//  @param (speed) : the speed of the motor (-100-100%) [int8_t]
void RockyMotors::setSpeedB(int8_t speed){
  this->_setMotorSpeed('B', speed);
}

// Set the speed for motor C
//  @param (speed) : the speed of the motor (-100-100%) [int8_t]
void RockyMotors::setSpeedC(int8_t speed){
  this->_setMotorSpeed('C', speed);
}

// --------------------------------------------------

// Set the left motor speed
//  @param (speed) : the speed of the motor (-100-100%) [int8_t]
void RockyMotors::setSpeedLeft(int8_t speed){
  this->_setMotorSpeed('A', speed);
}

// --------------------------------------------------

// Set the right motor speed
//  @param (speed) : the speed of the motor (-100-100%) [int8_t]
void RockyMotors::setSpeedRight(int8_t speed){
  this->_setMotorSpeed('B', speed);
}

// --------------------------------------------------

// Disable the drivers
void RockyMotors::sleep(void){
  digitalWrite(this->_pinSleep, LOW);
}

// --------------------------------------------------

// Stop both motors
void RockyMotors::stop(void){
  this->_pwmA = 0; // reset
  this->_pwmB = 0; // reset

  ledcWrite(*this->_active_pin_A, this->_pwmA); // update
  ledcWrite(*this->_active_pin_B, this->_pwmB); // update
}

// --------------------------------------------------

// Set the motors to turn
//  @param (speedA) : the speed of the left motor (-100-100%) [int8_t]
//         (speedB) : the speed of the right motor (-100-100%) [int8_t]
//  Note: a negative value sets the motor to move backwards
void RockyMotors::turn(int8_t speedA, int8_t speedB){
  // update both speeds (the values and the directions are automatically constrained)
  this->setSpeedLeft(speedA);
  this->setSpeedRight(speedB);
}

// --------------------------------------------------

// Enable the drivers
void RockyMotors::wakeup(void){
  digitalWrite(this->_pinSleep, HIGH);
}

// --------------------------------------------------
// --------------------------------------------------

// Attach a pin to the active PWM channel
//  @param (pin) : the new pin to attach [uint8_t *]
//  @returns true if successful [bool]
//  Note: the current active pin is then set to LOW.
bool RockyMotors::_attachPin(uint8_t * pin){
  // check if is already the active pin
  if((pin == this->_active_pin_A) || (pin == this->_active_pin_B) || (pin == this->_active_pin_C)){
    return false;
  }

  bool res = false;

  // Note: in Arduino ESP v3.0, the LEDC API has the <ledcAttach()> function
  //       which selects the channel automatically. This functions doesn't
  //       work properly with the motors because the pins are frequently
  //       attached and detached from the channels. The solution is to
  //       use fixed channels, but it might interfere with other devices
  //       that use the API.

  // motor A
  if((pin == &this->_pinMA1) || (pin == &this->_pinMA2)){
    if (this->_active_pin_A != nullptr){
      ledcDetach(*this->_active_pin_A);
      digitalWrite(*this->_active_pin_A, LOW);
    }
    this->_active_pin_A = pin;
    res = ledcAttachChannel(*this->_active_pin_A, this->_pwm_frequency, this->_pwm_resolution, this->_pwm_channel_A);
  }
  // motor B
  if((pin == &this->_pinMB1) || (pin == &this->_pinMB2)){
    if (this->_active_pin_B != nullptr){
      ledcDetach(*this->_active_pin_B);
      digitalWrite(*this->_active_pin_B, LOW);
    }
    this->_active_pin_B = pin;
    res = ledcAttachChannel(*this->_active_pin_B, this->_pwm_frequency, this->_pwm_resolution, this->_pwm_channel_B);
  }
  // motor C
  if((pin == &this->_pinMC1) || (pin == &this->_pinMC2)){
    if (this->_active_pin_C != nullptr){
      ledcDetach(*this->_active_pin_C);
      digitalWrite(*this->_active_pin_C, LOW);
    }
    this->_active_pin_C = pin;
    res = ledcAttachChannel(*this->_active_pin_C, this->_pwm_frequency, this->_pwm_resolution, this->_pwm_channel_C);
  }

  return res;
}

// --------------------------------------------------

// Configure the PWM channels
//  @returns true if successful [bool]
//  Note: the pins are detached if unsuccessful.
bool RockyMotors::_configurePWM(void){
  // calculate the maximum duty cycle
  this->_max_duty_cyle = (uint16_t)(pow(2, this->_pwm_resolution) - 1);

  // reset the default attached pins
  this->_active_pin_A = nullptr;
  this->_active_pin_B = nullptr;
  this->_active_pin_C = nullptr;

  // attach the pins
  uint8_t attached = 0x00;
  attached |= (this->_attachPin(&this->_pinMA1)) ? 0x01 : 0x00;
  attached |= (this->_attachPin(&this->_pinMB1)) ? 0x02 : 0x00;
  attached |= (this->_attachPin(&this->_pinMC1)) ? 0x04 : 0x00;

  if (attached == 0x07){
    return true;
  } else {
    if (attached & 0x01){
      ledcDetach(*this->_active_pin_A);
    }
    if (attached & 0x02){
      ledcDetach(*this->_active_pin_B);
    }
    if (attached & 0x04){
      ledcDetach(*this->_active_pin_C);
    }

    return false;
  }
}

// --------------------------------------------------

// Set the speed of a motor
//  @param (motor) : the motor to change ({1,2,3} or {'A','B','C'}) [uint8_t]
//         (speed) : the speed of the motor (-100-100%) [int8_t]
void RockyMotors::_setMotorSpeed(uint8_t motor, int8_t speed){
  uint16_t *pwm = nullptr;
  uint8_t *channel = nullptr;
  uint8_t pin = 0;

  switch(motor){
    case 1:
    case 'A': {
      pwm = &this->_pwmA;
      channel = &this->_pwm_channel_A;
      break;
    }
    
    case 2:
    case 'B': {
      pwm = &this->_pwmB;
      channel = &this->_pwm_channel_B;
      break;
    }
    
    case 3:
    case 'C': {
      pwm = &this->_pwmC;
      channel = &this->_pwm_channel_C;
      break;
    }
  }

  // check for correct assignment
  if (channel == nullptr){
    return;
  }


  // update the directions
  if(speed >= 0){
    if (channel == &this->_pwm_channel_A){
      this->_attachPin(&this->_pinMA1);
    } else if (channel == &this->_pwm_channel_B){
      this->_attachPin(&this->_pinMB1);
    } else if(channel == &this->_pwm_channel_C){
      this->_attachPin(&this->_pinMC1);
    }
  } else {
    speed *= -1; // update

    if (channel == &this->_pwm_channel_A){
      this->_attachPin(&this->_pinMA2);
    } else if (channel == &this->_pwm_channel_B){
      this->_attachPin(&this->_pinMB2);
    } else if(channel == &this->_pwm_channel_C){
      this->_attachPin(&this->_pinMC2);
    }
  }
  
  // constrain the value
  if(speed > 100){
    speed = 100;
  }

  // select the pin
  if (channel == &this->_pwm_channel_A){
    pin = *this->_active_pin_A;
  } else if (channel == &this->_pwm_channel_B){
    pin = *this->_active_pin_B;
  } else if(channel == &this->_pwm_channel_C){
    pin = *this->_active_pin_C;
  }

  *pwm = map(speed, 0, 100, 0, this->_max_duty_cyle); // transform to the current configuration
  ledcWrite(pin, *pwm); // update

  this->wakeup();
}

// --------------------------------------------------
// --------------------------------------------------
