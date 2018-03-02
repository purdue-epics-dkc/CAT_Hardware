# README #
### What is this repository for? ###

* Quick summary
Hardware code for the sensor/Bluetooth assembly for EPICS DKC CAT.

### How do I get set up? ###

* Summary of set up
	* Meant to be run through the Arduino IDE 1.6.4 or higher.
	* Current development is for the Adafruit Feather nRF52 Bluefruit and is located in feather/.
	* Deprecated development for the old Adafruit Flora is located in flora/.
	* For the Flora, glove_sketch.ino is the main file. It executes I2C reads from the finger sensors and outputs data packets on request over the UART bus to the Bluetooth modem.
	* The other source files are proof of concept tests for other submodules.
* Configuration
	* Adafruit drivers required.
	* [Installation instructions](https://learn.adafruit.com/adafruit-arduino-ide-setup/arduino-1-dot-6-x-ide)
* Dependencies
	* Adafruit Feather/Flora drivers.
	* Feather code depends on [SiLabs CP201X Driver](https://learn.adafruit.com/bluefruit-nrf52-feather-learning-guide/arduino-board-setup).

