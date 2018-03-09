/***********************************************************
 * Joe Mynhier
 * 
 * Written for EPICS DKC CAT Spring 2018
 * This program is meant to test the integration between the
 * Adafruit Feather nRF52 and the FDC2114 by connecting to
 * an FDC2114EVM board.
 * 
 * It reads from the two sensors on the board and prints
 * the values to the serial monitor.
 **********************************************************/

#include <Wire.h>

byte reg;

void setup() {

  // Set up the I2C bus
  Wire.begin(); // Use default clock speed.
  Wire.setClock(10000);

  // Turn on serial monitor.
  Serial.begin(115200);
  
  // Reset the FDC2114
  // Toggle shutdown mode to reset errors.
  digitalWrite(16, HIGH);
  delay(1);
  digitalWrite(16, LOW);
  delay(1);

  // Set the DRDY_2INT bit in the STATUS_CONFIG register 0x19.
  // 0x1 should preserve other bits. May not be needed.
  configure(0x2B, 0x19, 0, 1);

  // Set the MUX_CONFIG register to ready only Ch0, Ch1 (not for final design).
  // Write 0xe20f to 0x1B. Turns on autoscan, selects correct channels.
  configure(0x2B, 0x1B, 0xe2, 0xf);
  
  // Write 0b0 to the sleep mode enable register to exit power on sleep mode.
  // Configure any status registers before exiting sleep mode.
  // Writing 0x1c01 should preserve the other default values in that register (?).
  configure(0x2B, 0x1A, 0x1C, 0x1);
  

  // Select which sensor to read first.
  reg = 0;

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
  
  Serial.print("\nStatus val at ");
  Serial.print(digitalRead(15));
  Serial.print(": ");
  Wire.beginTransmission(0x2B);
  Wire.write(0x18);
  Wire.endTransmission();
  delay(100);
  Wire.requestFrom(0x2B, 2);
  while (Wire.available()) {
    Serial.print(Wire.read(), HEX);
    Serial.print(" ");
  }  
  Serial.println("");  
 
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

  // DEBUG
  //delay(100);
  
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
  //delay(1);
  Wire.endTransmission();
}

