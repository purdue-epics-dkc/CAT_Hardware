/*************************************************************
 * Joe Mynhier
 * 
 * Written for the CAT project on the DKC team in Purdue's
 * EPICS program.
 * 
 * This program is meant for debugging the data to Bluetooth
 * system. Data sent out via UART should be retransmitted by
 * the Bluetooth modem and be read by external software. 
 * 
 * It outputs a fixed pattern where fingers should appear to
 * move from a fist to an open hand and back. It will pause 
 * for 2 seconds whenever it reaches the fist or open hand
 * shape.
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
 *  Packets are read as two consecutive one byte reads on the 
 *  I2C bus.
 *  
 ************************************************************/

#include <Wire.h>

// System initialization
void setup() {
  
  // Set up UART connection.
  Serial.begin(9600);  // UART
  // Wait to establish connection.
  while (!Serial) { } 
}

// Main execution logic
// Send out 8 words total on the UART to the bluetooth.
void loop() {
 
  byte finger_id = 0;  
  word data = 0;
  
  while (data < 0xfff) {
    // Transmit the data for each finger.
    for (finger_id=0; finger_id < 5; finger_id++) {
      // Send out the packet
      send_word(data | (finger_id << 12));

      // Wait 50 ms for transmission. Arbitrary length.
      delay(50); 
    }
    data += 5;
  }
  // Wait 2 seconds before changing direction.
  delay(2000);

  // Send it back the other way.
  while (data > 0) {
    for (finger_id=0; finger_id < 5; finger_id++) {
      send_word(data | (finger_id << 12));
      delay(50);
    }
    data -= 5;
  }
  delay(2000);
}

// Take a word and send out MSB then LSB on UART
void send_word(word data) {
  Serial.write(byte(data >> 8)); // MSB
  Serial.write(byte(data & 0xFF)); // LSB
}

