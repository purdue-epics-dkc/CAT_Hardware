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
  // Write 0b0 to the sleep mode enable register to exit power on sleep mode.
  // Configure any status registers before exiting sleep mode.
  // Writing 0x801 should preserve the other default values in that register.
  Wire.beginTransmission(0x2B);
  Wire.write(0x1A);
  Wire.write(0x8);
  Wire.write(0x1);
  delay(1);
  Wire.endTransmission();
  

  // Select which sensor to read first.
  reg = 0;

}

void loop() {
  // Read a sensor and print it.
  word result;

  if (digitalRead(15) == HIGH) {
    Serial.println("FDC2114 in error state!!");
  }

  // Alternate between INT0 and INT1;
  if (reg == 0) {
    reg = 2;
  } else {
    reg = 0;
  }

  result = read_request(0x2B, reg);
  Serial.print("Read ");
  Serial.print(result, HEX);
  Serial.print(" from ");
  Serial.println(reg);

}

// Read sensor data from a specific device on the i2c bus.
word read_request(uint8_t rr_device_addr, byte rr_reg) {  
  byte rr_temp[2] = {0xBA, 0xD1};
  byte rr_cnt = 0;

  // Request a 2 byte read at the specified register.
  // FDC2114 sends MSB then LSB
  Serial.print("Read request for ");
  Serial.print(rr_device_addr, HEX);
  Serial.print(":");
  Serial.print(rr_reg, HEX);

  // Set register to read from.
  Wire.beginTransmission(rr_device_addr);
  Wire.write(rr_reg);
  delay(100);
  Wire.endTransmission();
  
  // Read two bytes.
  Serial.print(". Request returned ");
  Serial.print(Wire.requestFrom(rr_device_addr, 2));

  Serial.print(". Got ");
  
  while (Wire.available()) {
    rr_temp[rr_cnt] = Wire.read();
    Serial.print(rr_temp[rr_cnt]);
    Serial.print(" ");
    rr_cnt++;
  }
  Serial.println("");

  // Concatenate into raw data word.
  byte rr_res = (rr_temp[0] << 8) | (rr_temp[1]);
  return rr_res;
}
