/*************************************************************
 * Joe Mynhier
 * 
 * Written for the CAT project on the DKC team in Purdue's
 * EPICS program.
 * 
 * This program is the production version of the controller
 * code for the glove.
 * 
 * It reads data from the FDC2114 chips on the PCB via an I2C
 * bus and buffers the state. It receives send requests (the 
 * string "S") on the Bluetooth connection and sends a data packet
 * out to the Bluetooth modem.
 * 
 * The pack conists of five two-byte words composed like this:
 *    YYYY YYYY    XXXX YYYY
 * in little-endian order. Y represents the 12 data bits and
 * X represents the finger ID number.
 * 
 * Finger ID numbers should correspond to the input port 
 * numbers on the FDC2114 chips this way:
 * 
 * FDC2114 #1
 *  IN0X:  0
 *  IN1X:  1
 *  IN2X:  2
 *  IN3X:  
 *  
 * FDC2114 #2
 *  IN0X1: 3
 *  IN1X1: 4
 *  IN2X1: 
 *  IN3x1: 
 *  
 *  Packets are read as two consecutive one byte reads on the 
 *  I2C bus.
 *  
 ************************************************************/


#include <Wire.h>

#define NUM_CHIPS 2
#define NUM_REG_PER_CHIP 4
#define NUM_SENSORS 5
#define NUM_OUTPUTS_CHIP_1 3
#define NUM_OUTPUTS_CHIP_2 2

const byte device_addr[NUM_CHIPS] = {0x2A, 0x2B};
const byte reg[NUM_REG_PER_CHIP] = {0x00, 0x02, 0x04, 0x06};

word data[NUM_SENSORS] = {0, 0, 0, 0, 0};

void setup() {
  // Set up i2c bus
  Wire.begin();

  // Set up the serial monitor
  Serial.begin(9600);

  // Set up the UART connection
  Serial1.begin(115200); // The Bluesmirf 
  Serial1.print("$$$");  // Enter command mode
  delay(100);  
  Serial1.println("U,9600,N");  // Temporarily change the baud rate to 9600, no parity 
  delay(100);
  Serial1.begin(9600);
  Serial1.println("r,1");  // Exit command mode and reboot.
}

void loop() {
  // Read each sensor on each chip.
  byte i, j, limit, input, finger_id = 0;
  for (i=0; i<NUM_CHIPS; i++) {
    if (i == 0) {
      limit = NUM_OUTPUTS_CHIP_1;
    } else {
      limit = NUM_OUTPUTS_CHIP_2;
    }
    for (j=0; j<limit; j++) {
      data[finger_id] = read_request(device_addr[i], reg[j]);
      finger_id += 1;
    }
  }

  if (Serial1.available() > 0) {
    input = (char)Serial1.read();
    Serial.println(input);
    if (input == 'S') {
      Serial.print("data: ");
      Serial.println(data[0], HEX);
      // Send out the packet
      for (finger_id=0; finger_id<NUM_SENSORS; finger_id++) {
        send_data(data[finger_id] | (finger_id << 12));
      }
      Serial.println("");
    }
  } else {
    // delay so there is more time without I2C reads to get a UART read.
    delay(500);
  }
  
}

// Read sensor data from a specific device on the i2c bus.
word read_request(uint8_t rr_device_addr, byte rr_reg) {
  byte rr_temp[2] = {0, 0};
  byte rr_cnt = 0;

  // Request a 2 byte read at the specified register.
  // FDC2114 sends MSB then LSB
  Wire.beginTransmission(rr_device_addr);
  Wire.write(rr_reg);
  Wire.requestFrom(rr_device_addr, (uint8_t)2);

  // Read two bytes.
  while (Wire.available()) {
    rr_temp[rr_cnt] = Wire.read();
    rr_cnt++;
  }
  Wire.endTransmission();

  // Concatonate into raw data word.
  byte rr_res = (rr_temp[0] << 8) | (rr_temp[1]);
  return rr_res;
}

// Take a word and send out MSB then LSB on UART
void send_data(word data_word) {
  // Sending "$$$" puts the Bluesmirf into command mode. Don't do that.
  if ((char)data == '$') {
    data_word += 1;
  }
  Serial1.write(data_word & 0xff);
  Serial1.write(data_word >> 8);
}

