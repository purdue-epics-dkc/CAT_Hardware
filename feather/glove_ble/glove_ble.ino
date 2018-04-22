/***********************************************************
 * Joe Mynhier
 * 
 * Written for EPICS DKC CAT Spring 2018
 * This program establishes a BLE 5.0 service that delivers
 * glove data to the phone. Currently implements reading of
 * FDC2114 chips and IMU data.
 * 
 * Device: e0:18:16:ac:bc:02
 * BLE Serivce:
 *  Name: DeafSigner
 *  UUID128: 09200cd2-e2cd-4210-b647-f022ec29fd47
 *  
 *  Characteristic: dataEnable
 *    - Phone sets to start recording data, unsets to end.
 *    UUID128: 09200cd3-e2cd-4210-b647-f022ec29fd47
 *    fixed len: 1
 *    data: if 0, don't send data. Else send data. Only set by
 *          phone except for initialization.
 *  
 *  Characteristic: RightHand
 *    - Has the flex data for 5 fingers fo the right hand.
 *    - Sends notifications to phone. Phone should read all
 *      data services upon recieving the RightHand notfiication.
 *    UUID128: 09200cd4-e2cd-4210-b647-f022ec29fd47
 *    fixed len: 10
 *    data: 5 2-byte words in finger ID order (see bellow)
 *          written MSB first.
 *          
 *  Characteristic: Orientation
 *    - Compensated Euler angles (degrees / sec) from the IMU
 *    UUID128: 09200cd5-e2cd-4210-b647-f022ec29fd47
 *    fixed len: 12
 *    data: 3 floating point values [pitch, roll, yaw]
 *    
 * 
 * Flex sensors (Custom PCB DeafSigner CRv0.2):
 *    Finger ID numbers should correspond to the input port 
 *    numbers on the FDC2114 chips this way:
 * 
 *    FDC2114 #1 (0x2A)
 *      port | ID|addr| finger
 *      IN0X:   0 (0x0) Thumb
 *      IN1X:   1 (0x2) Pointer
 *      IN2X:   2 (0x4) Middle
 *      IN3X:  
 *  
 *    FDC2114 #2 (0x2B)
 *      port | ID|addr| finger
 *      IN0X1:  3 (0x0) Ring
 *      IN1X1:  4 (0x2) Pinky
 *      IN2X1: 
 *      IN3x1: 
 *  
 *      Packets are read as two consecutive one byte reads on the 
 *      I2C bus, big endian.
 *  
 *      Pinouts
 *        16  SD1   Shutdown signal to FDC2114 1
 *        15  SD2   Shutdown signal to FDC2114 2
 *        7   INTB1 Interrupt signal from FDC2114 1
 *        11  INTB2 Interrupt signal from FDC2114 2
 *      
 **********************************************************/

#include <Wire.h>
#include <bluefruit.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS0.h>
#include <MadgwickAHRS.h>

// Use these define statements to easily reconfigure the code
// to test individual hardware components.
#define USE_BLE       // Output data using the Feather's BLE modem.
#define USE_CR        // Read flex sensors from the capacitive reader board.
#define USE_IMU       // Read position data from the LSM9DS0
//#define USE_TOUCH     // Read finger contact data from the touch sensor board.

// Number of words in the main data array.
#define NUM_SENSORS 5

// Number of bytes in the main data array.
// Used for writing to the Bluetooth characteristic.
#define DATA_LEN 10

// Period of data refresh in ms.
#define PERIOD 5

// Data lengths for IMU services.
#define ORIENT_LEN 3 * sizeof(float)

#ifdef USE_BLE
  // Custom UUID: byte order is little endian.
  uint8_t const DSUUID[] = {0x47, 0xfd, 0x29, 0xec, 
                            0x22, 0xf0, 0x47, 0xb6,
                            0x10, 0x42, 0xcd, 0xe2,
                            0xd2, 0x0c, 0x20, 0x09};

  uint8_t const RHUUID[] = {0x47, 0xfd, 0x29, 0xec, 
                            0x22, 0xf0, 0x47, 0xb6,
                            0x10, 0x42, 0xcd, 0xe2,
                            0xd4, 0x0c, 0x20, 0x09};

  uint8_t const ENUUID[] = {0x47, 0xfd, 0x29, 0xec, 
                            0x22, 0xf0, 0x47, 0xb6,
                            0x10, 0x42, 0xcd, 0xe2,
                            0xd3, 0x0c, 0x20, 0x09};
  

  uint8_t const ORUUID[] = {0x47, 0xfd, 0x29, 0xec, 
                            0x22, 0xf0, 0x47, 0xb6,
                            0x10, 0x42, 0xcd, 0xe2,
                            0xd5, 0x0c, 0x20, 0x09};

  uint8_t const AVUUID[] = {0x47, 0xfd, 0x29, 0xec, 
                            0x22, 0xf0, 0x47, 0xb6,
                            0x10, 0x42, 0xcd, 0xe2,
                            0xd6, 0x0c, 0x20, 0x09};

  uint8_t const LAUUID[] = {0x47, 0xfd, 0x29, 0xec, 
                            0x22, 0xf0, 0x47, 0xb6,
                            0x10, 0x42, 0xcd, 0xe2,
                            0xd7, 0x0c, 0x20, 0x09};
#endif // USE_BLE

#ifdef USE_CR
  // Map fingerId to device address.
  const byte device_addr[NUM_SENSORS] = {0x2A, 0x2A, 0x2A, 0x2B, 0x2B};
  
  // Map fingerId to register address on the finger's device.
  const byte reg[NUM_SENSORS] = {0x00, 0x02, 0x04, 0x00, 0x02};
#endif // USE_CR

// Store data.
uint8_t data[DATA_LEN] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

#ifdef USE_IMU
  // Instantiate the sensor object.
  Adafruit_LSM9DS0 lsm = Adafruit_LSM9DS0(1000);

  // Declare the Madgwick sensor fusion object.
  Madgwick fusionFilter;

  // Data structures needed to calibarate the Madgwick algorithm
  // TODO Calculate appropriate values.
  // raw x/y/z offsets.
  float mag_offsets[3] = { 0.0, 0.0, 0.0 };

  // soft iron compensation
  float mag_softiron_matrix[3][3] = {{ 0.0, 0.0, 0.0 },
                                     { 0.0, 0.0, 0.0 },
                                     { 0.0, 0.0, 0.0 }};

  float mag_field_strength = 0.0;
#endif // USE_IMU

#ifdef USE_BLE
  // Make the glove service.
  BLEService moCapGlove(DSUUID);
  #ifdef USE_CR
    BLECharacteristic rightHand(RHUUID);
  #endif
  BLECharacteristic dataEnable(ENUUID);
  #ifdef USE_IMU
    BLECharacteristic orientation(ORUUID);
  #endif // USE_IMU

  // Enable state set by the phone.
  uint8_t enableState;
#endif // USE_BLE

// Handle delay between data frames.
unsigned long oldTime, currentTime;

void setupCR();
void setupDS();
void setupAd();
void setupIMU();

void setup() {

  // Set up the I2C bus
  Wire.begin(); // Use default clock speed.
  Wire.setClock(100000);

  // Turn on serial monitor.
  Serial.begin(115200);

  #ifdef USE_CR
    // Configure capacitive reader board with FDC2114 chips.
    setupCR();
  #endif // USE_CR

  #ifdef USE_BLE
    // Set up the BLE service.
    Bluefruit.begin();
    //DEBUG Set to a slightly lower power setting. Need to check battery
    // usage and compatibility with phones.
    Bluefruit.setTxPower(-4);
    
    Bluefruit.setName("DSRight");
    setupDS();
  
    // Set up the advertising profile.
    setupAd();
  #endif // USE_BLE

  #ifdef USE_IMU
    // Set up the LSM9DS0
    setupIMU();
  #endif // USE_IMU

  // Initialize the timer.
  oldTime = millis();
}

#ifdef USE_CR
  void setupCR() {
    // Reset the FDC2114 chips
    // See documentation here: http://www.ti.com/product/FDC2114/technicaldocuments
    // Toggle shutdown mode to reset errors.
    digitalWrite(16, HIGH);
    digitalWrite(15, HIGH);
    delay(1);
    digitalWrite(16, LOW);
    digitalWrite(15, LOW);
    delay(1);
  
    // Set the DRDY_2INT bit in the STATUS_CONFIG register 0x19.
    // 0x1 should preserve other bits. May not be needed.
    configure(0x2B, 0x19, 0, 1);
  
    // Set the MUX_CONFIG register to ready all channels.
    // Write 0xc20f to 0x1B. Turns on autoscan, selects correct channels.
    configure(0x2B, 0x1B, 0xc2, 0xf);
    
    // Write 0b0 to the sleep mode enable register to exit power on sleep mode.
    // Configure any status registers before exiting sleep mode.
    // Writing 0x1c01 should preserve the other default values in that register (?).
    configure(0x2B, 0x1A, 0x1C, 0x1);
  }
#endif // USE_CR

#ifdef USE_BLE  
  void setupAd() {  
    // Start advertising.
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addName();
    Bluefruit.Advertising.restartOnDisconnect(true);
    Bluefruit.Advertising.setInterval(32, 244);
    Bluefruit.Advertising.setFastTimeout(30);
  
    Bluefruit.Advertising.addService(moCapGlove);
    
    Bluefruit.Advertising.start(0); // keep advertising indefinitely.
  }

  void setupDS() {
    moCapGlove.begin();

    #ifdef USE_CR
      // Finger flex data characteristic.
      // Set to read only status from the perspective of the phone.
      // Notify the phone to read when updated.
      rightHand.setProperties(CHR_PROPS_READ | CHR_PROPS_NOTIFY);
      rightHand.setPermission(SECMODE_OPEN, SECMODE_OPEN); //SECMODE_NO_ACCESS);
      rightHand.setFixedLen(DATA_LEN);
      rightHand.setUserDescriptor("right hand");
      rightHand.begin();
    #endif // USE_CR

    #ifdef USE_IMU
      orientation.setProperties(CHR_PROPS_READ);
      orientation.setPermission(SECMODE_OPEN, SECMODE_NO_ACCESS);
      orientation.setFixedLen(ORIENT_LEN);
      orientation.setUserDescriptor("orientation");
      orientation.begin();
    #endif // USE_IMU
  
    // Data recording enable flag characteristic.
    // Starts unset, phone can set it to begin data flow
    // and unset it to end.
    enableState = 0;
    dataEnable.setProperties(CHR_PROPS_WRITE | CHR_PROPS_NOTIFY);
    dataEnable.setPermission(SECMODE_OPEN, SECMODE_OPEN);
    dataEnable.setFixedLen(1);
    dataEnable.setUserDescriptor("enable");
    dataEnable.begin();
  
    dataEnable.write(0);
  }
#endif // USE_BLE

#ifdef USE_IMU
  void setupIMU() {
    // Start up IMU.
    lsm.begin();
    delay(1000);

    //TODO verify that these are the correct ranges.
    lsm.setupAccel(lsm.LSM9DS0_ACCELRANGE_2G);
    lsm.setupGyro(lsm.LSM9DS0_GYROSCALE_245DPS);
    lsm.setupMag(lsm.LSM9DS0_MAGGAIN_2GAUSS);
    
    //TODO callibrate sensors.

    // Sensor fusion expects 100 frames per second
    fusionFilter.begin(100);
  }
#endif // USE_BLE

void loop() {

  #ifdef USE_BLE
    // Check for enable/disable toggle from the phone.
    if (dataEnable.notifyEnabled()) {
      enableState = dataEnable.read8();
    }
  #endif // USE_BLE
  
  // Target a refresh rate of 200 Hz.
  currentTime = millis();
  #ifdef USE_BLE
  if (enableState != 0 && currentTime - oldTime >= PERIOD) {
  #else
  if (currentTime - oldTime >= PERIOD) {
  #endif // USE_BLE
    oldTime = currentTime;
    
    #ifdef USE_BLE
    // Check for a connection and pairing 
    if (Bluefruit.connected() && Bluefruit.connPaired()) {
    #endif // USE_BLE

      // Read flex values.
      #ifdef USE_CR

        // If not using BLE, use Serial output for debugging.
        #ifndef USE_BLE
          Serial.print("Flex: ");
        #endif //USE_BLE
        
        // Read each flex sensor on each chip.
        byte fingerId, i;
        for (fingerId=0, i=0; fingerId<NUM_SENSORS; fingerId++, i+=2) {
          //read_request(device_addr[fingerId], reg[fingerId], i);
          #ifndef USE_BLE
            Serial.print(data[i], HEX);
            Serial.print(data[i+1], HEX);
            Serial.print(" ");
          #endif // USE_BLE
        }
        #ifndef USE_BLE
          Serial.println("");
        #endif //USE_BLE
      #endif // USE_CR

      #ifdef USE_IMU
        // Read data from the IMU.
        sensors_event_t accel, mag, gyro, temp;
        lsm.getEvent(&accel, &mag, &gyro, &temp);

        // Run the Madgwick algorithm.
        // Based on https://github.com/adafruit/Adafruit_AHRS/blob/master/examples/ahrs_mahony/ahrs_mahony.ino
        float x = mag.magnetic.x - mag_offsets[0];
        float y = mag.magnetic.y - mag_offsets[1];
        float z = mag.magnetic.z - mag_offsets[2];
      
        // Apply mag soft iron error compensation
        float mx = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
        float my = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
        float mz = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];
      
        // The filter library expects gyro data in degrees/s, but adafruit sensor
        // uses rad/s so we need to convert them first (or adapt the filter lib
        // where they are being converted)
        float gx = gyro.gyro.x * 57.2958F;
        float gy = gyro.gyro.y * 57.2958F;
        float gz = gyro.gyro.z * 57.2958F;
      
        // Update the filter
        fusionFilter.update(gx, gy, gz, accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, mx, my, mz);
      #endif // USE_IMU
      
      #ifdef USE_BLE
        #ifdef USE_IMU
          // Write IMU data to characteristic. 
          //TODO Use htonl to convert endianness??
          float eulerAngles[3];
          eulerAngles[0] = fusionFilter.getPitch();
          eulerAngles[1] = fusionFilter.getRoll(); 
          eulerAngles[2] = fusionFilter.getYaw();
          orientation.write((uint8_t *)&eulerAngles, ORIENT_LEN);
        #endif // USE_IMU
      #ifdef USE_CR      
        // Write the new flex frame to the characteristic and notify phone.
        rightHand.notify((uint8_t *)data, DATA_LEN);
      #endif // USE_CR
    } 
    #endif // USE_BLE
  }
}

// Read sensor data from a specific device on the i2c bus.
void read_request(uint8_t rr_device_addr, byte rr_reg, byte index) {  
  // Request a 2 byte read at the specified register.
  // FDC2114 sends MSB then LSB

  // Set register to read from.
  Wire.beginTransmission(rr_device_addr);
  Wire.write(rr_reg);
  Wire.endTransmission();
  
  // Read two bytes.
  Wire.requestFrom(rr_device_addr, 2);
  while (Wire.available()) {
    data[index] = Wire.read();
    index++;
  }
}

// Set a configuration register in one of the FDC2114 chips.
void configure(byte device, byte reg, byte upper, byte lower) {
  Wire.beginTransmission(device);
  Wire.write(reg);
  Wire.write(upper);
  Wire.write(lower);
  delay(1);
  Wire.endTransmission();
}

