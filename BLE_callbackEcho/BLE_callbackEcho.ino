/*********************************************************************
This is an example for our nRF8001 Bluetooth Low Energy Breakout

  Pick one up today in the adafruit shop!
  ------> http://www.adafruit.com/products/1697

Adafruit invests time and resources providing this open source code, 
please support Adafruit and open-source hardware by purchasing 
products from Adafruit!

Written by Kevin Townsend/KTOWN  for Adafruit Industries.
MIT license, check LICENSE for more information
All text above, and the splash screen below must be included in any redistribution
*********************************************************************/

// This version uses call-backs on the event and RX so there's no data handling in the main loop!

#include <SPI.h>
#include "Adafruit_BLE_UART.h"

#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 32
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART uart = Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

#define CMD_BUF_LEN 64
uint8_t cmdBufPos = 0;
char cmdBuffer[CMD_BUF_LEN] = {0};

void setup(void)
{
  Serial.begin(9600);
  while (!Serial)
    ; // Leonardo/Micro should wait for serial init
  Serial.println(F("Adafruit Bluefruit Low Energy nRF8001 Callback Echo demo"));

  uart.setRXcallback(rxCallback);
  uart.setACIcallback(aciCallback);
  uart.setDeviceName("DUMB BBLE BOY"); /* 7 characters max! */
  uart.begin();
}

unsigned long lastMessage = 0;

void loop()
{
  uart.pollACI();
  unsigned long now = millis();
  if( now > lastMessage + 1000){
    lastMessage = now;
    Serial.print(now/1000);
    Serial.println(" waiting... ");
  }
}

// ACI Event callback
void aciCallback(aci_evt_opcode_t event)
{
  switch (event)
  {
  case ACI_EVT_DEVICE_STARTED:
    Serial.println(F("Advertising started"));
    break;
  case ACI_EVT_CONNECTED:
    Serial.println(F("Connected!"));
    break;
  case ACI_EVT_DISCONNECTED:
    Serial.println(F("Disconnected or advertising timed out"));
    break;
  default:
    break;
  }
}



void buttonPress(int button)
{
  Serial.print("Button ");
  Serial.print(button);
  Serial.println(" pressed.");
  // reset our buffer!
  cmdBufPos = 0;
}

// data received from bluetooth UART
void rxCallback(uint8_t *buffer, uint8_t len)
{

 for (int i = 0; i < len; i++)
  {
    addToCMDBuffer((char)buffer[i]);
  }

 // uart.write(buffer, len);
}

void addToCMDBuffer(char c)
{
  if (cmdBufPos < CMD_BUF_LEN - 1)
  {
    if(c == '!'){
      cmdBufPos = 0;
    }
    cmdBuffer[cmdBufPos] = c;
    cmdBufPos++;
    cmdBuffer[cmdBufPos] = '\0';
    if (cmdBufPos >= 4)
    {
      // check command here!
      if (strcmp(cmdBuffer, "!B11:") == 0)
      {
        buttonPress(1); // 1 DOWN
      }
      else if (strcmp(cmdBuffer, "!B219") == 0)
      {
        buttonPress(2); // 2 DOWN
      }
      else if (strcmp(cmdBuffer, "!B318") == 0)
      {
        buttonPress(3); // 3 DOWN
      }
      else if (strcmp(cmdBuffer, "!B417") == 0)
      {
        buttonPress(4); // 4 DOWN
      }else{
        // DEBUG
        /* 
        Serial.print(" BUFFER: [");
        Serial.print(cmdBuffer);
        Serial.println("]");
        */
      }
    }
  }
  else
  {
    // buffer overflow!
    cmdBufPos = 0;
  }
}