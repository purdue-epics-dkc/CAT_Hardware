/*
 * Copied from https://learn.sparkfun.com/tutorials/using-the-bluesmirf
 * Modified to run with Bluesmirf on the master UART for debugging.
 * 
 */

void setup()
{
  Serial.begin(9600);  // The Serial monitor
  Serial1.begin(115200); // The Bluesmirf 
  
  Serial1.print("$");  // Print three times individually
  Serial1.print("$");
  Serial1.print("$");  // Enter command mode
  delay(100);  // Short delay, wait for the Mate to send back CMD
  Serial1.println("U,9600,N");  // Temporarily Change the baudrate to 9600, no parity
  // 115200 can be too fast at times for NewSoftSerial to relay the data reliably
  
  delay(100);
  Serial1.begin(9600);  // Start Serial serial at 9600
  Serial1.println("r,1");
  //Serial1.print("-");
  //Serial1.print("-");

  //Serial1.begin(9600);  // Start Serial serial at 9600
}

void loop()
{

  word data;
  while (Serial.available() > 0)  // If the Serial sent any characters
  {
    // Send any characters the Serial prints to the serial monitor
    Serial1.print((char)Serial.read());  
  }
  while (Serial1.available() > 0)  // If stuff was typed in the serial monitor
  {
    // Send any characters the Serial monitor prints to the Serial
    Serial.print("Byte: ");
    data = Serial1.read();
    Serial.println(data);
    Serial.print("Char: ");
    Serial.println((char)data);
  }
  // and loop forever and ever!
}
