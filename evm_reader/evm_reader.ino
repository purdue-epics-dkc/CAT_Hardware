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

  // Turn on serial monitor.
  Serial.begin(115200);

  // Select which sensor to read first.
  reg = 0;
 
  // Keep the reader awake and don't interrupt.
  digitalWrite(15, LOW);
  digitalWrite(16, LOW);
}

void loop() {
  // Read a sensor and print it.
  word result;

  // Alternate between INT0 and INT1;
  if (reg == 0) {
    reg = 2;
  } else {
    reg = 0;
  }

  result = read_request(0x2B, reg);
  Serial.print("Read ");
  Serial.print(result);
  Serial.print(" from ");
  Serial.println(reg);

}

// Read sensor data from a specific device on the i2c bus.
word read_request(uint8_t rr_device_addr, byte rr_reg) {  
  byte rr_temp[2] = {0, 0};
  byte rr_cnt = 0;

  //Serial.print(" dev: ");
  //Serial.println(rr_device_addr, HEX);
  //Serial.print(" reg: ");
  //Serial.println(rr_reg, HEX);

  // Request a 2 byte read at the specified register.
  // FDC2114 sends MSB then LSB
  Wire.beginTransmission(rr_device_addr);
  Wire.write(rr_reg);
  delay(100);
  Wire.endTransmission();
  
  Wire.requestFrom(rr_device_addr, (uint8_t)2);

  // Read two bytes.
  while (Wire.available()) {
    rr_temp[rr_cnt] = Wire.read();
    rr_cnt++;
  }

  // Concatonate into raw data word.
  byte rr_res = (rr_temp[0] << 8) | (rr_temp[1]);
  return rr_res;
}
