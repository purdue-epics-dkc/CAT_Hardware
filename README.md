# README #
### What is this repository for? ###

* Quick summary
Hardware code for the sensor/Bluetooth assembly for EPICS DKC CAT.

### How do I get set up? ###

* Summary of set up
	* Meant to be run through the Arduino IDE 1.6.4 or higher.
	* glove_sketch.ino is the main file. It executes I2C reads from the finger sensors and outputs data packets on request over the UART bus to the Bluetooth modem.
	* glove_ble_test.ino is a debugging file that does not implement the I2C functionality, but instead outputs arbitrary data over the Bluetooth connection.
	* The other source files are proof of concept tests for other submodules.
* Configuration
	* Adafruit drivers required.
	* [Installation instructions](https://learn.adafruit.com/adafruit-arduino-ide-setup/arduino-1-dot-6-x-ide)
* Dependencies
	* Adafruit Flora driver.

