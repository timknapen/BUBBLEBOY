
/***********************************************************

 Reading a TFMini LIDAR sensor
 
 Hardware serial on Teensy 3.6
 SERIAL1 :
 RX 0, 21, 27
 TX 1, 5, 26
 CTS 18, 20
***********************************************************/

/*
Example code for Benewake TFMini time-of-flight distance sensor. 
by Peter Jansen (December 11/2017)
This example code is in the public domain.

This example communicates to the TFMini using a SoftwareSerial port at 115200, 
while communicating the distance results through the default Arduino hardware
Serial debug port. 

SoftwareSerial for some boards can be unreliable at high speeds (such as 115200). 
The driver includes some limited error detection and automatic retries, that
means it can generally work with SoftwareSerial on (for example) an UNO without
the end-user noticing many communications glitches, as long as a constant refresh
rate is not required. 

The (UNO) circuit:
 * Uno RX is digital pin 10 (connect to TX of TF Mini)
 * Uno TX is digital pin 11 (connect to RX of TF Mini)
 
*/

#include "TFMini.h"

// Setup software serial port
//SoftwareSerial mySerial(10, 11);      // Uno RX (TFMINI TX), Uno TX (TFMINI RX)
TFMini tfmini;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  // wait for serial port to connect. Needed for native USB port only
  while (!Serial)
  {
    for (int i = 0; i < 3; i++)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(100);
      digitalWrite(LED_BUILTIN, LOW);
      delay(100);
    }
    delay(200);
  }
  // Step 1: Initialize hardware serial port (serial debug port)
  Serial.begin(115200);
  Serial.println("Initializing...");

  // Step 2: Initialize the data rate for the SoftwareSerial port
  Serial1.begin(115200);

  Serial.println("setup Serial1");
  // Step 3: Initialize the TF Mini sensor
  tfmini.begin(&Serial1);

  Serial.println("Setup tfmini");
}

void loop()
{
  Serial.print("loop ");
  Serial.println(millis());
  // Take one TF Mini distance measurement
  uint16_t dist = tfmini.getDistance();
  Serial.println("Received distance");

  uint16_t strength = tfmini.getRecentSignalStrength();
  Serial.println("Received Signal strength");

  // Display the measurement
  Serial.print(dist);
  Serial.print(" cm      sigstr: ");
  Serial.println(strength);

  // Wait some short time before taking the next measurement
  delay(25);
}
