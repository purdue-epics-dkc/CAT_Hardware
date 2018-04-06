# README #
### Summary ###


* Hardware code for the sensor/Bluetooth assembly for EPICS DKC CAT.
	* Current development is for the Adafruit Feather nRF52 Bluefruit and is located in feather/.
	* feather/glove_ble/glove_ble.ino is the main controller file.
	* Deprecated development for the old Adafruit Flora is located in flora/.

### Setup ###

* Dependencies
	* Meant to be run through the Arduino IDE 1.6.4 or higher.
		* [Installation instructions](https://learn.adafruit.com/adafruit-arduino-ide-setup/arduino-1-dot-6-x-ide)
	* Adafruit drivers required.
	* Adafruit [Feather](https://learn.adafruit.com/bluefruit-nrf52-feather-learning-guide/arduino-bsp-setup)/[Flora](https://learn.adafruit.com/getting-started-with-flora/windows-setup) drivers.
	* Feather code depends on [SiLabs CP201X Driver](https://learn.adafruit.com/bluefruit-nrf52-feather-learning-guide/arduino-board-setup).
	* Sensor fusion depends on the [MadgwickAHRS](https://github.com/PaulStoffregen/MadgwickAHRS) library.
		* To install in the Arduino IDE, navigate to your Arduino/libraries directory, clone the repository linked above, and restart the IDE.

### BLE Service Profile ###
* BLE Serivce:
	* Name: DeafSigner
	* UUID128: 09200cd2-e2cd-4210-b647-f022ec29fd47

	* Characteristic: dataEnable
		* Phone sets to start recording data, unsets to end.
		* UUID128: 09200cd3-e2cd-4210-b647-f022ec29fd47
		* fixed len: 1
		* data: if 0, don't send data. Else send data. Only set by phone except for initialization.

	* Characteristic: RightHand
		* Has the flex data for 5 fingers of the right hand. Sends notifications to phone. Phone should read all data services upon recieving the RightHand notfiication.
		* UUID128: 09200cd4-e2cd-4210-b647-f022ec29fd47
		* fixed len: 10
		* data: 5 2-byte words in finger ID order (see bellow) written MSB first.

	* Characteristic: Orientation
		* Compensated Euler angles from the IMU.
		* UUID128: 09200cd5-e2cd-4210-b647-f022ec29fd47
		* fixed len: 12
		* data: 3 4-byte floating point numbers.


### Finger Flex Data Format ###
* Each data frame for one hand is 10 bytes.
	* Each Finger sends 2 bytes, MSB first. This contains a 12 bit binary fraction.
	* The two byte words are organized in this order: thumb, pointer, middle, ring, pinky.
