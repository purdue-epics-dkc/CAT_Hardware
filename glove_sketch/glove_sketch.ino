#include <Wire.h>

#define NUM_CHIPS 2
#define NUM_REG_PER_CHIP 4
#define NUM_SENSORS 5

const byte device_addr[NUM_CHIPS] = {0x2A, 0x2B};
const byte reg[NUM_REG_PER_CHIP] = {0x00, 0x02, 0x04, 0x06};

word data[NUM_SENSORS] = {0, 0, 0, 0, 0};

void setup() {
  // Set up i2c bus and UART connection.
  Wire.begin();
  Serial.begin(9600);  // The Serial monitor
  Serial1.begin(115200); // The Bluesmirf 
  
  Serial1.print("$$$");  // Enter command mode
  delay(100);  
  Serial1.println("U,9600,N");  // Temporarily Change the baud rate to 9600, no parity 
  delay(100);
  Serial1.begin(9600);
  Serial1.println("r,1");  // Exit command mode and reboot.
}

void loop() {
  // Read each sensor on each chip.
  byte i, j, limit, input, finger_id = 0;
  for (i=0; i<NUM_CHIPS; i++) {
    if (i == 0) {
      limit = 3;
    } else {
      limit = 2;
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
      for (finger_id=0; finger_id < NUM_SENSORS; finger_id++) {
        // Send out the packet
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

