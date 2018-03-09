/***********************************************************
 * Joe Mynhier
 * 
 * Written for EPICS DKC CAT Spring 2018
 * This program establishes a BLE 5.0 service that delivers
 * glove data to the phone. Currently implements reading of
 * FDC2114 chips.
 * 
 * Serivce:
 *  Name: DeafSignerRight
 * 
 * Finger ID numbers should correspond to the input port 
 * numbers on the FDC2114 chips this way:
 * 
 * FDC2114 #1 (0x2A)
 *  IN0X:  0 (0x0)
 *  IN1X:  1 (0x2)
 *  IN2X:  2 (0x4)
 *  IN3X:  
 *  
 * FDC2114 #2 (0x2B)
 *  IN0X1: 3 (0x0)
 *  IN1X1: 4 (0x2)
 *  IN2X1: 
 *  IN3x1: 
 *  
 *  Packets are read as two consecutive one byte reads on the 
 *  I2C bus.
 **********************************************************/

#include <Wire.h>
#include <bluefruit.h>

#define NUM_SENSORS 5

// Map finger_id to device address.
const byte device_addr[NUM_SENSORS] = {0x2A, 0x2A, 0x2A, 0x2B, 0x2B};

// Map finger_id to register address on the finger's device.
const byte reg[NUM_SENSORS] = {0x00, 0x02, 0x04, 0x00, 0x02};

// Store data.
word data[NUM_SENSORS] = {0, 0, 0, 0, 0};

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

  // Set up Bluefruit service (advertising for now).
  Bluefruit.begin();
  //DEBUG Set to a slightly lower power setting. Need to check battery
  // usage and compatibility with phones??
  Bluefruit.setTxPower(-4);
  Bluefruit.setName("DeafSignerRight");
  // Start advertising.
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();
  Bluefruit.Advertising.addName();
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);
  Bluefruit.Advertising.setFastTimeout(30);
  Bluefruit.Advertising.start(0); // keep advertising indefinitely??
  //TODO: Add the custom GAP service for finger flex.
  
}

void loop() {
  // Read a sensor and print it.
  word result;

  // Alternate between INT0 and INT1;
  if (reg == 0) {
    reg = 2;
    Serial.print("\t\t");
  } else {
    reg = 0;
    Serial.println("");
    delay(100);
  }

   result = read_request(0x2B, reg);

   Serial.print(reg);
   Serial.print(": ");
   Serial.print(result, HEX);

  // Read the STATUS register.
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

