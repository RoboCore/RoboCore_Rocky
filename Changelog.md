RoboCore Rocky Arduino Library
==============================

Arduino library for the [*RoboCore BlackBoard Rocky v1.0*](https://www.robocore.net/loja/produtos/3128).

This file is part of the Rocky library by RoboCore ("RoboCore-Rocky-lib").

Change log
----------

**v1.0**
* Contributors: @Francois.
* Based on the Arduino implementation of the ESP32 (v3.0.1).
	* ( https://docs.espressif.com/projects/arduino-esp32/en/latest/libraries.html#apis )
* `RockyMotors`
	* There is no option to change the PWM channels, but it might be useful to update them without needing to recompile.
	* By default, channels 13, 14 and 15 are used so that there is a lesser chance of collision with other libraries (e.g. Servo).

