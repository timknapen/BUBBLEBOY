
/***********************************************************

 Reading a TFMini LIDAR sensor
 
 Hardware serial on Teensy 3.6
 SERIAL1 :
 RX 0, 21, 27
 TX 1, 5, 26
 CTS 18, 20
***********************************************************/

/*
Based on example code for Benewake TFMini time-of-flight distance 
sensor by Peter Jansen (December 11/2017)
This example code is in the public domain.
*/

#define TXPIN 26 // white wire on TFMini RX
#define RXPIN 27 // Green wire on TFMini TX

#include "TFMini.h"

TFMini tfmini;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);

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

  // Step 2: Initialize the data rate for the LIDAR serial port
  Serial1.setTX(TXPIN);
  Serial1.setRX(RXPIN);
  Serial1.begin(TFMINI_BAUDRATE);

  // Step 3: Initialize the TF Mini sensor
  tfmini.begin(&Serial1);

  Serial.println("Done settting up TFmini LIDAR");
}

void loop()
{
  

  uint16_t dist = tfmini.getDistance();
  uint16_t strength = tfmini.getRecentSignalStrength();

  // Display the measurement
  Serial.print(dist);
  //Serial.print(" ");
  Serial.print(strength);
  Serial.println();

  // Wait some short time before taking the next measurement
  delay(25);
}
