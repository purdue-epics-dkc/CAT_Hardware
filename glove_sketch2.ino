/*************************************************************
 * Joe Mynhier
 * 
 * Written for the CAT project on the DKC team in Purdue's
 * EPICS program.
 * 
 * This program is targeted for the Adafruit Flora. It's 
 * purpose is to read data from the FDC2114 chips on our glove
 * via I2C, tag each info packet with a finger ID number, then
 * send it out to a Bluetooth modem on a serial connection.
 * 
 * Finger ID numbers should correspond to the input port 
 * numbers on the FDC2114 chips this way:
 * 
 * FDC2114 #1
 *  IN0X:  0
 *  IN1X:  1
 *  IN2X:  2
 *  IN3X:  3
 *  
 * FDC2114 #2
 *  IN0X1: 4
 *  IN1X1: 5
 *  IN2X1: 6
 *  IN3x1: 7
 *  
 *  Packets are read as two concecutive one byte reads on the 
 *  I2C bus. Then they are composited and immediately sent out
 *  to the Modem. This is done so that there will be time for 
 *  the data to be processed before the next packet is sent.
 *  
 ************************************************************/

#include <Wire.h>

#define NUM_CHIPS 2
#define NUM_REG_PER_CHIP 4
#define NUM_SENSORS 8

const byte device_addr[NUM_CHIPS] = {0x2A, 0x2B};
const byte reg[NUM_REG_PER_CHIP] = {0x00, 0x02, 0x04, 0x06};

// System initialization
void setup() {
  
  // Set up i2c bus and UART connection.
  Wire.begin();  // I2C
  Serial.begin(9600);  // UART
  // Wait to establish connection.
  while (!Serial) { } 
}

// Main execution logic
// Read each sensor on each chip.
// Send out 8 words total on the UART to the bluetooth.
void loop() {
 
  byte cnt = 0;  // keep track of finger id number
  byte i, j;
  word temp_loop; // hold data
  
  for (i=0; i<NUM_CHIPS; i++) {
    for (j=0; j<NUM_REG_PER_CHIP; j++) {
      // Read the sensor data
      temp_loop = read_request(device_addr[i], reg[j]);
      // Send out the packet
      send_word(temp_loop | (cnt << 12));
      cnt++;
    }
  }  
}

// Read sensor data from a specific device on the i2c bus.
word read_request(byte rr_device_addr, byte rr_reg) {
  
  // Hold MSB and LSB of the 12 bit data
  byte rr_temp[2] = {0, 0};
  byte rr_cnt = 0;

  // Request a 2 byte read at the specified register.
  // FDC2114 sends MSB then LSB
  Wire.beginTransmission(rr_device_addr);
  Wire.write(rr_reg);
  Wire.requestFrom(rr_device_addr, 2);

  // Read two bytes.
  while (Wire.available()) {
    rr_temp[rr_cnt] = Wire.read();
    rr_cnt++;
  }
  Wire.endTransmission();

  // Concatonate into raw data word.
  word rr_res = (word(rr_temp[0]) << 8) | word(rr_temp[1]);
  return rr_res;
}

// Take a word and send out MSB then LSB on UART
void send_word(word data) {
  Serial.write(byte(data >> 8)); // MSB
  Serial.write(byte(data & 0xFF)); // LSB
}

