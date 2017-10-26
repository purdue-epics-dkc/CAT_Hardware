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

#define OFFSET 201

word data;
byte add;

// System initialization
void setup() {

  data = 0;
  add = 1;
  
  // Set up UART connection.
  Serial.begin(9600);  // The Serial monitor
  Serial1.begin(115200); // The Bluesmirf 
  
  Serial1.print("$$$");  // Enter command mode
  delay(100);  
  Serial1.println("U,9600,N");  // Temporarily Change the baud rate to 9600, no parity 
  delay(100);
  Serial1.begin(9600);
  Serial1.println("r,1");  // Exit command mode and reboot.
}

// Main execution logic
// Send out 8 words total on the UART to the bluetooth.
void loop() {

  char input;
  byte finger_id = 0;  
  if (Serial1.available() > 0) {
    input = (char)Serial1.read();
    Serial.println(input);
    if (input == 'S') {
      Serial.print("data: ");
      Serial.println(data, HEX);
      for (finger_id=0; finger_id < 5; finger_id++) {
        // Send out the packet
        send_word(data | (finger_id << 12));
        // Wait ? ms for transmission. Arbitrary length.
        delay(50); 
      }

      if (data >= (0xFFF - OFFSET)) {
        add = 0;
      } else if (data <= OFFSET) {
        add = 1;
      }
      data = add ? data + OFFSET : data - OFFSET;
    }
  }
}

// Take a word and send out MSB then LSB on UART
void send_word(word data) {
  //Serial1.print((char)data >> 8);
  if ((char)data == '$') {
    data += 1;
  }
  Serial1.write(data & 0xff);
  Serial1.write(data >> 8);
  
  //Serial1.println();
  //Serial1.println(byte(data & 0xFF)); // LSB
}

