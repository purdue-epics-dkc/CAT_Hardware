#include <Wire.h>

#define NUM_CHIPS 2
#define NUM_REG_PER_CHIP 4
#define NUM_SENSORS 8

const byte device_addr[NUM_CHIPS] = {0x2A, 0x2B};
const byte reg[NUM_REG_PER_CHIP] = {0x00, 0x02, 0x04, 0x06};

word data[NUM_SENSORS] = {0, 0, 0, 0, 0, 0, 0, 0};

void setup() {
  // Set up i2c bus and UART connection.
  Wire.begin();
  Serial.begin(9600);
  while (!Serial) {
    // establish connection.
  }
}

void loop() {
  // Read each sensor on each chip.
  byte i, j, cnt = 0;
  for (i=0; i<NUM_CHIPS; i++) {
    for (j=0; j<NUM_REG_PER_CHIP; j++) {
      data[cnt] = read_request(device_addr[i], reg[j]);
    }
  }

  // Send out 8 words on the UART to the bluetooth.
  for (i=0; i<NUM_SENSORS; i++) {
    send_data(data[i]);
  }
  
}

// Read sensor data from a specific device on the i2c bus.
word read_request(byte rr_device_addr, byte rr_reg) {
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
  byte rr_res = (rr_temp[0] << 8) | (rr_temp[1]);
  return rr_res;
}

// Take a word and send out MSB then LSB on UART
void send_data(word data_word) {
  Serial.write(data_word >> 8); // MSB
  Serial.write(data_word & 0xFF); // LSB
}

