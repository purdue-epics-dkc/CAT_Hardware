/***********************************************************
 * Joe Mynhier
 * 
 * Written for EPICS DKC CAT Spring 2018
 * This program establishes a BLE 5.0 service that delivers
 * glove data to the phone. Currently implements reading of
 * FDC2114 chips and IMU data.
 * 
 * BLE Serivce:
 *  Name: DeafSigner
 *  UUID128: 09200cd2-e2cd-4210-b647-f022ec29fd47
 *  
 *  Characteristic: dataEnable
 *    - Phone sets to start recording data, unsets to end.
 *    UUID128: 09200cd2-e2cd-4210-b647-f022ec29fd48
 *    fixed len: 1
 *    data: if 0, don't send data. Else send data. Only set by
 *          phone except for initialization.
 *  
 *  Characteristic: RightHand
 *    - Has the flex data for 5 fingers fo the right hand.
 *    UUID128: 09200cd2-e2cd-4210-b647-f022ec29fd49
 *    fixed len: 10
 *    data: 5 2-byte words in finger ID order (see bellow)
 *          written MSB first.
 *          
 *  Characteristic: Orientation
 *    - Four Point Quaterion
 *    UUID128: 09200cd2-e2cd-4210-b647-f022ec29fd4a
 *    fixed len: 
 *    data: 
 *  
 *  Characteristic: AngularVelocity
 *    - Three axis rotation speed (rad/s)
 *    UUID128: 09200cd2-e2cd-4210-b647-f022ec29fd4b
 *    fixed len: 
 *    data: 
 *  
 *  Characteristic: LinearAcceleration
 *    - Compensated for gravity. (m/s^2)
 *    UUID128: 09200cd2-e2cd-4210-b647-f022ec29fd4c
 *    fixed len: 
 *    data: 
 * 
 * Flex sensors:
 * Finger ID numbers should correspond to the input port 
 * numbers on the FDC2114 chips this way:
 * 
 * FDC2114 #1 (0x2A)
 *  port | ID|addr| finger
 *  IN0X:   0 (0x0) Thumb
 *  IN1X:   1 (0x2) Pointer
 *  IN2X:   2 (0x4) Middle
 *  IN3X:  
 *  
 * FDC2114 #2 (0x2B)
 *  port | ID|addr| finger
 *  IN0X1:  3 (0x0) Ring
 *  IN1X1:  4 (0x2) Pinky
 *  IN2X1: 
 *  IN3x1: 
 *  
 *  Packets are read as two consecutive one byte reads on the 
 *  I2C bus, big endian.
 **********************************************************/

#include <Wire.h>
#include <bluefruit.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// Use these define statements to easily reconfigure the code
// to test individual hardware components.
#define USE_BLE       // Output data using the Feather's BLE modem.
#define USE_CR        // Read flex sensors from the capacitive reader board.
//#define USE_IMU       // Read position data from the BNO055.
//#define USE_TOUCH     // Read finger contact data from the touch sensor board.

// Number of words in the main data array.
#define NUM_SENSORS 5

// Number of bytes in the main data array.
// Used for writing to the Bluetooth characteristic.
#define DATA_LEN 10

// Period of data refresh in ms.
#define PERIOD 10

#ifdef USE_BLE
  // Custom UUID: byte order is little endian.
  uint8_t const DSUUID[] = {0x47, 0xfd, 0x29, 0xec, 
                            0x22, 0xf0, 0x47, 0xb6,
                            0x10, 0x42, 0xcd, 0xe2,
                            0xd2, 0x0c, 0x20, 0x09};
  
  uint8_t const RHUUID[] = {0x48, 0xfd, 0x29, 0xec, 
                            0x22, 0xf0, 0x47, 0xb6,
                            0x10, 0x42, 0xcd, 0xe2,
                            0xd2, 0x0c, 0x20, 0x09};
  
  uint8_t const ENUUID[] = {0x49, 0xfd, 0x29, 0xec, 
                            0x22, 0xf0, 0x47, 0xb6,
                            0x10, 0x42, 0xcd, 0xe2,
                            0xd2, 0x0c, 0x20, 0x09};
#endif // USE_BLE

#ifdef USE_CR
  // Map finger_id to device address.
  const byte device_addr[NUM_SENSORS] = {0x2A, 0x2A, 0x2A, 0x2B, 0x2B};
  
  // Map finger_id to register address on the finger's device.
  const byte reg[NUM_SENSORS] = {0x00, 0x02, 0x04, 0x00, 0x02};
#endif // USE_CR

// Store data.
uint8_t data[DATA_LEN] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#ifdef USE_BLE
  // Make the glove service.
  BLEService moCapGlove(DSUUID);
  #ifdef USE_CR
    BLECharacteristic rightHand(RHUUID);
  #endif
  BLECharacteristic dataEnable(ENUUID);

  // Enable state set by the phone.
  uint8_t enableState;
#endif // USE_BLE

// Handle delay between data frames.
unsigned long old_time, current_time;

void setup() {

  // Set up the I2C bus
  Wire.begin(); // Use default clock speed.
  Wire.setClock(100000);

  // Turn on serial monitor.
  Serial.begin(115200);

  #ifdef USE_CR
    // Configure capacitive reader board with FDC2114 chips.
    setupCR();
  #endif // USE_CR

  #ifdef USE_BLE
    // Set up the BLE service.
    Bluefruit.begin();
    //DEBUG Set to a slightly lower power setting. Need to check battery
    // usage and compatibility with phones.
    Bluefruit.setTxPower(-4);
    
    Bluefruit.setName("DSRight");
    setupDS();
  
    // Set up the advertising profile.
    setupAd();
  #endif // USE_BLE

  // Initialize the timer.
  old_time = millis();
}

#ifdef USE_CR
  void setupCR() {
    // Reset the FDC2114 chips
    // See documentation here: http://www.ti.com/product/FDC2114/technicaldocuments
    // Toggle shutdown mode to reset errors.
    digitalWrite(16, HIGH);
    delay(1);
    digitalWrite(16, LOW);
    delay(1);
  
    // Set the DRDY_2INT bit in the STATUS_CONFIG register 0x19.
    // 0x1 should preserve other bits. May not be needed.
    configure(0x2B, 0x19, 0, 1);
  
    // Set the MUX_CONFIG register to ready all channels.
    // Write 0xc20f to 0x1B. Turns on autoscan, selects correct channels.
    configure(0x2B, 0x1B, 0xc2, 0xf);
    
    // Write 0b0 to the sleep mode enable register to exit power on sleep mode.
    // Configure any status registers before exiting sleep mode.
    // Writing 0x1c01 should preserve the other default values in that register (?).
    configure(0x2B, 0x1A, 0x1C, 0x1);
  }
#endif // USE_CR

#ifdef USE_BLE  
  void setupAd() {  
    // Start advertising.
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addName();
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);
    Bluefruit.Advertising.setFastTimeout(30);
  
    Bluefruit.Advertising.addService(moCapGlove);
    
    Bluefruit.Advertising.start(0); // keep advertising indefinitely.
  }

  void setupDS() {
    moCapGlove.begin();

    #ifdef USE_CR
      // Finger flex data characteristic.
      // Set to read only status from the perspective of the phone.
      // Notify the phone to read when updated.
      rightHand.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
      rightHand.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
      rightHand.setFixedLen(DATA_LEN);
      rightHand.setUserDescriptor("right hand");
      rightHand.begin();
    #endif // USE_CR
  
    // Data recording enable flag characteristic.
    // Starts unset, phone can set it to begin data flow
    // and unset it to end.
    enableState = 0;
    dataEnable.setProperties(CHR_PROPS_WRITE | CHR_PROPS_NOTIFY);
    dataEnable.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    dataEnable.setFixedLen(1);
    dataEnable.setUserDescriptor("enable");
    dataEnable.begin();
  
    dataEnable.write(0);
  }
#endif // USE_BLE

void loop() {

  #ifdef USE_BLE
    // Check for enable/disable toggle from the phone.
    if (dataEnable.notifyEnabled()) {
      enableState = dataEnable.read8();
    }
  #endif // USE_BLE
  
  // Target a refresh rate of 100 Hz.
  current_time = millis();
  #ifdef USE_BLE
  if (enableState != 0 && current_time - old_time >= PERIOD) {
  #else
  if (current_time - old_time >= PERIOD) {
  #endif // USE_BLE
    old_time = current_time;
    
    #ifdef USE_BLE
    // Check for a connection and pairing 
    if (Bluefruit.connected() && Bluefruit.connPaired()) {
    #endif // USE_BLE

      // Read flex values.
      #ifdef USE_CR

        // If not using BLE, use Serial output for debugging.
        #ifndef USE_BLE
          Serial.print("Flex: ");
        #endif //USE_BLE
        
        // Read each flex sensor on each chip.
        byte finger_id, i;
        for (finger_id=0, i=0; finger_id<NUM_SENSORS; finger_id++, i+=2) {
          read_request(device_addr[finger_id], reg[finger_id], i);
          #ifndef USE_BLE
            Serial.print(data[i], HEX);
            Serial.print(data[i+1], HEX);
            Serial.print(" ");
          #endif // USE_BLE
        }
        #ifndef USE_BLE
          Serial.println("");
        #endif //USE_BLE
      #endif // USE_CR
      
      #ifdef USE_BLE
      #ifdef USE_CR
        // Write the new flex frame to the characteristic and notify phone.
        rightHand.notify((uint8_t *)data, DATA_LEN);
      #endif // USE_CR
    } 
    #endif // USE_BLE
  }
}

// Read sensor data from a specific device on the i2c bus.
void read_request(uint8_t rr_device_addr, byte rr_reg, byte index) {  
  // Request a 2 byte read at the specified register.
  // FDC2114 sends MSB then LSB

  // Set register to read from.
  Wire.beginTransmission(rr_device_addr);
  Wire.write(rr_reg);
  Wire.endTransmission();
  
  // Read two bytes.
  Wire.requestFrom(rr_device_addr, 2);
  while (Wire.available()) {
    data[index] = Wire.read();
    index++;
  }
}

// Set a configuration register in one of the FDC2114 chips.
void configure(byte device, byte reg, byte upper, byte lower) {
  Wire.beginTransmission(device);
  Wire.write(reg);
  Wire.write(upper);
  Wire.write(lower);
  delay(1);
  Wire.endTransmission();
}

