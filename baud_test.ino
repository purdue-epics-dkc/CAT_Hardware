/*************************************************************
 * Joe Mynhier
 * 
 * Written for the CAT project on the DKC team in Purdue's
 * EPICS program.
 * 
 * This program is meant for debugging the data to Bluetooth
 * system. Specifically, it repeatedly sends the sequence "E0"
 * so that we can confirm that the UART is running at the 
 * correct baud rate using the Serial Monitor.
 * 
 * It also tries to set the name of the modem to "DKCCATBLuEs"
 *  
 ************************************************************/

#include <Wire.h>

// controll sending start state.
byte sending;

// System initialization
void setup() {
  
  // Set up UART connection.
  Serial1.begin(115200);  // UART
  //Serial.begin(115200); // Programming connection
  // Wait to establish connection.
  while (!Serial1) { } 

  sending = 0;
  /*
  // Enter command mode.
  byte i;
  for (i=0; i<3; i++) {
    Serial.print("$");
  }

  // Change the name.
  Serial.println("S-,DKCCATBLuEs");

  // Leav command mode.
  for (i=0; i<3; i++) {
    Serial.print("-");
  }
  */
}

// Main execution logic
// Send out 8 words total on the UART to the bluetooth.
void loop() {
  //int data = 0;
  
  Serial1.print(char(0x45));
  Serial1.println(char(0x30));
  delay(1000);
    

  /*
  byte finger_id = 0;  
  //word data = 0;
  // Wait to send until you recieve the start byte 0xaa
  if (sending == 0) {
    byte temp = Serial.read();
    if (temp == 0xaa) {
      sending = 1;
    } else {
      Serial.println(temp);
    }
  } else {  
    while (1) { //data < 0xfff) {
      send_word(0x4530);
      // Wait 50 ms for transmission. Arbitrary length.
      delay(50); 
    }
    //Serial.print(EOF);
  }
  */
}

// Take a word and send out MSB then LSB on UART
void send_word(word data) {
  //Serial.println(data);
  Serial.print(char(data >> 8)); // MSB
  Serial.println(char(data & 0xFF)); // LSB
}

