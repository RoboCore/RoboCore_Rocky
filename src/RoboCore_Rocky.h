#ifndef ROCKY_H
#define ROCKY_H

/*******************************************************************************
* RoboCore - Rocky Library (v1.0)
* 
* Library to use the functions of the Rocky board.
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

#if !defined(ARDUINO_ESP32_DEV) // ESP32
#error Use this library with the ESP32
#endif

// --------------------------------------------------
// Libraries

#include <Arduino.h>

extern "C" {
  #include <stdarg.h>
  #include <stdint.h>
  #include <stdlib.h>

  #include <esp_arduino_version.h>

  #include <esp32-hal-adc.h>
  #include <esp32-hal-ledc.h>
}

#ifdef ESP_ARDUINO_VERSION_MAJOR
#if ESP_ARDUINO_VERSION_MAJOR < 3
#warning RoboCore Vespa v1.3 is meant to use the Arduino ESP package v3.0+
#endif
#endif

// --------------------------------------------------
// Macros

#define ROCKY_VERSION_MAJOR 1 // (X.x.x)
#define ROCKY_VERSION_MINOR 3 // (x.X.x)
#define ROCKY_VERSION_PATCH 0 // (x.x.X)

#define ROCKY_BATTERY_ADC_ATTENUATION (ADC_11db)
#define ROCKY_BATTERY_PIN (34)
#define ROCKY_BATTERY_VOLTAGE_CONVERSION (5702) // Vin = Vout * (R1+R2)/R2

#define ROCKY_LED_PIN (15)

#define ROCKY_MOTORS_CHANNEL_A (13)
#define ROCKY_MOTORS_CHANNEL_B (14)
#define ROCKY_MOTORS_CHANNEL_C (15)

// --------------------------------------------------
// Enumerators

enum BatteryType : uint8_t {
  BATTERY_UNDEFINED = 0, 
  BATTERY_LIPO
};

// --------------------------------------------------
// Class - Rocky Battery

class RockyBattery {
  public:
    RockyBattery(void);
    ~RockyBattery(void);
    uint8_t readCapacity(void);
    uint32_t readVoltage(void);
    bool setBatteryType(uint8_t);

    void (*handler_critical)(uint8_t); // critical voltage (capacity)

  private:
    uint8_t _pin;
    uint8_t _battery_type;
};

// --------------------------------------------------
// Class - Rocky LED

class RockyLED {
  public:
    RockyLED(void);
    RockyLED(uint8_t);
    ~RockyLED(void);
    void blink(uint32_t);
    void on(void);
    void off(void);
    void toggle(void);
    void update(void);

  private:
    uint8_t _pin, _state;
    uint32_t _stop_time, _delay;
};

// --------------------------------------------------
// Class - Rocky Motors

class RockyMotors {
  public:
    RockyMotors(void);
    ~RockyMotors(void);
    void backward(uint8_t);
    void forward(uint8_t);
    void setSpeedA(int8_t);
    void setSpeedB(int8_t);
    void setSpeedC(int8_t);
    void setSpeedLeft(int8_t);
    void setSpeedRight(int8_t);
    void sleep(void);
    void stop(void);
    void turn(int8_t, int8_t);
    void wakeup(void);

  private:
    uint8_t _pinMA1, _pinMA2, _pinMB1, _pinMB2, _pinMC1, _pinMC2;
    uint8_t _pinSleep;
    uint8_t *_active_pin_A, *_active_pin_B, *_active_pin_C;
    uint16_t _pwmA, _pwmB, _pwmC;
    uint8_t _pwm_channel_A, _pwm_channel_B, _pwm_channel_C;
    double _pwm_frequency; // [Hz]
    uint8_t _pwm_resolution;
    uint16_t _max_duty_cyle;

    bool _attachPin(uint8_t *);
    bool _configurePWM(void);
    void _setMotorSpeed(uint8_t, int8_t);
};

// --------------------------------------------------

#endif // ROCKY_H
