/***********************************************************
 * Joe Mynhier
 * 
 * Written for EPICS DKC CAT Spring 2018
 * This program establishes a BLE 5.0 service that delivers
 * glove data to the phone. Currently implements reading of
 * FDC2114 chips.
 * 
 * Serivce:
 *  Name: DeafSigner
 *  UUID128: 09200cd2-e2cd-4210-b647-f022ec29fd47
 *  
 *  Characteristic: RightHand
 *    - Has the flex data for 5 fingers fo the right hand.
 *    UUID128: 0bed7cea-5a41-4039-a1cc-70b942731b0f
 *    fixed len: 10
 *    data: 5 2-byte words in finger ID order (see bellow)
 *          written MSB first.
 * 
 * Finger ID numbers should correspond to the input port 
 * numbers on the FDC2114 chips this way:
 * 
 * FDC2114 #1 (0x2A)
 *  port |id |addr| finger
 *  IN0X:  0 (0x0)  Thumb
 *  IN1X:  1 (0x2)  Pointer
 *  IN2X:  2 (0x4)  Middle
 *  IN3X:  
 *  
 * FDC2114 #2 (0x2B)
 *  port |id |addr| finger
 *  IN0X1: 3 (0x0)  Ring
 *  IN1X1: 4 (0x2)  Pinky
 *  IN2X1: 
 *  IN3x1: 
 *  
 *  Packets are read as two consecutive one byte reads on the 
 *  I2C bus, big endian.
 **********************************************************/

#include <Wire.h>
#include <bluefruit.h>

// Number of words in the main data array.
#define NUM_SENSORS 5

// Number of bytes in the main data array.
// Used for writing to the Bluetooth characteristic.
#define DATA_LEN 10

// Period of data refresh in ms.
#define PERIOD 10

// Custom UUID: byte order is little endian.
const uint8_t DSUUID[] = {0x47, 0xfd, 0x29, 0xec, 
                          0x22, 0xf0, 0x47, 0xb6,
                          0x10, 0x42, 0xcd, 0xe2,
                          0xd2, 0x0c, 0x20, 0x09};

const uint8_t RHUUID[] = {0x0f, 0x1b, 0x73, 0x42,
                          0xb9, 0x70, 0xcc, 0xa1,
                          0x39, 0x40, 0x41, 0x5a,
                          0xea, 0x7c, 0xed, 0x0b};

// Map finger_id to device address.
const byte device_addr[NUM_SENSORS] = {0x2A, 0x2A, 0x2A, 0x2B, 0x2B};

// Map finger_id to register address on the finger's device.
const byte reg[NUM_SENSORS] = {0x00, 0x02, 0x04, 0x00, 0x02};

// Store data.
word data[NUM_SENSORS] = {0, 0, 0, 0, 0};

// Make the glove service.
BLEService moCapGlove(DSUUID);
BLECharacteristic rightHand(RHUUID);

// Handle delay between data frames.
unsigned long old_time, current_time;

void setupAd();
void setupDS();

void setup() {

  // Set up the I2C bus
  Wire.begin(); // Use default clock speed.
  Wire.setClock(100000);

  // Turn on serial monitor.
  Serial.begin(115200);
  
  // Reset the FDC2114
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
  
  // Set up the BLE service.
  setupDS();

  // Set up the advertising profile.
  setupAd();

  // Initialize the timer.
  old_time = millis();
}

void setupAd() {
  Bluefruit.begin();
  //DEBUG Set to a slightly lower power setting. Need to check battery
  // usage and compatibility with phones.
  Bluefruit.setTxPower(-4);
  
  Bluefruit.setName("DSRight");
  
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

  // Set to ready only status from the perspective of the phone.
  rightHand.setProperties(CHR_PROPS_READ);
  rightHand.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
  rightHand.setFixedLen(DATA_LEN);
  rightHand.begin();
}

void loop() {
  // Target a refresh rate of 100 Hz.
  current_time = millis();
  if (current_time - old_time >= PERIOD) {
    old_time = current_time;
    
    // Check for a connection and pairing.
    if (Bluefruit.connected() && Bluefruit.connPaired()) {
      // Read each sensor on each chip.
      byte finger_id;
      for (finger_id=0; finger_id<NUM_SENSORS; finger_id++) {
        data[finger_id] = read_request(device_addr[finger_id], reg[finger_id]);
      }
  
      // Write the new frame to the characteristic.
      rightHand.write((uint8_t *)data, DATA_LEN);
    } 
  }
}

// Read sensor data from a specific device on the i2c bus.
word read_request(uint8_t rr_device_addr, byte rr_reg) {  
  byte rr_temp[2] = {0xBA, 0xD1};
  byte rr_cnt = 0;

  // Request a 2 byte read at the specified register.
  // FDC2114 sends MSB then LSB

  // Set register to read from.
  Wire.beginTransmission(rr_device_addr);
  Wire.write(rr_reg);
  Wire.endTransmission();
  
  // Read two bytes.
  Wire.requestFrom(rr_device_addr, 2);
  while (Wire.available()) {
    rr_temp[rr_cnt] = Wire.read();
    rr_cnt++;
  }

  // Concatenate into raw data word.
  byte rr_res = (rr_temp[0] << 8) | (rr_temp[1]);
  return rr_res;
}

void configure(byte device, byte reg, byte upper, byte lower) {
  Wire.beginTransmission(device);
  Wire.write(reg);
  Wire.write(upper);
  Wire.write(lower);
  delay(1);
  Wire.endTransmission();
}

