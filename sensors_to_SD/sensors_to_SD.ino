#include <Bounce.h>
#include <SD.h>
#include <SPI.h>
// #include "DHT.h"
#include "Adafruit_BLE_UART.h"
#include <TinyGPS.h>





#define LED_PIN 33
// not used for now, but could be used to attach a bright LED (with a
// driver!) as a signal

// MICRO SD
const int chipSelect = BUILTIN_SDCARD;

// WEATHER SENSOR
/*
#define DHTPIN 12
#define DHTTYPE DHT11 // DHT 11
DHT dht(DHTPIN, DHTTYPE);
*/
float humidity = 0;
float temperature = 0;

// WIND SENSOR
#define ANEMOPIN 34
Bounce bouncer = Bounce(ANEMOPIN, 5);
unsigned long lastPulseTime = 0;
float pulseTime = -1;
float windSpeed = 0; // windspeed = rotations / minute ( = pulses / minute)
int lastPinState = LOW;

// SOUND VOLUME DETECTION
// the microphone analog out connects to A0
#define MICROPHONE_PIN A0
#define LEVEL0 520
int soundVolume = 0;
#define BUFSIZE 256 // 2^12 (4096)// 8 seconds of recording!
int volBuffer[BUFSIZE];
int bufPos = 0;

// MEASURING FEEDBACK
unsigned long lastBlink = 0;
bool ledOn = false;

unsigned long lastMessageTime = 0;

// BLE STUFF
#define ADAFRUITBLE_REQ 10
#define ADAFRUITBLE_RDY 32
#define ADAFRUITBLE_RST 9

Adafruit_BLE_UART uart =
    Adafruit_BLE_UART(ADAFRUITBLE_REQ, ADAFRUITBLE_RDY, ADAFRUITBLE_RST);

#define CMD_BUF_LEN 64
uint8_t cmdBufPos = 0;
char cmdBuffer[CMD_BUF_LEN] = {0};

float altitude = 1.6;
int measureState = -1;

// GPS
TinyGPS gps;
#define gpsPort Serial1
char gpsBuf[32];  // for gps text printing
char timeMsg[32]; // holds the time of this measurement, from GPS
float latitude, longitude, elevation;

//-------------------------------------------------------------------------------------------------
void setup() {

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  // WEATHER
  // dht.begin();

  // WIND
  pinMode(ANEMOPIN, INPUT_PULLUP);

  // SD
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    errorBlink();
    return;
  }
  Serial.println("card initialized.");

  // MICROPHONE
  pinMode(MICROPHONE_PIN, INPUT);
  for (int i = 0; i < BUFSIZE; i++) {
    volBuffer[i] = 0;
  }

  // BLE Initializing
  uart.setDeviceName("BBLEBOY"); /* 7 characters max! */
  uart.begin();

  // GPS Initializing
  gpsPort.begin(9600);

  // while(!Serial);
  Serial.begin(57600);
  Serial.println("Hello, I am the sensor clubhouse. I live in a balloon.");
}

//-------------------------------------------------------------------------------------------------
void loop() {
  uart.pollACI();

  // read from BLE buffer

  // OK while we still have something to read, get a character and print it out
  while (uart.available()) {
    char c = uart.read();
    Serial.print(c);
    addToCMDBuffer(c);
  }

  // measuringBlink();
  sensorLoop();

  // keep alive messages
  unsigned long now = millis();
  if (now > lastMessageTime + 1000) {
    lastMessageTime = now;
    Serial.print(now / 1000);
    Serial.println(" waiting... ");
    uart.write((unsigned char *)"waiting...", strlen("waiting..."));
  }
}

// ACI Event callback
//-------------------------------------------------------------------------------------------------
void aciCallback(aci_evt_opcode_t event) {
  switch (event) {
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

//-------------------------------------------------------------------------------------------------
void addToCMDBuffer(char c) {
  if (cmdBufPos < CMD_BUF_LEN - 1) {
    if (c == '!') {
      cmdBufPos = 0;
    }
    cmdBuffer[cmdBufPos] = c;
    cmdBufPos++;
    cmdBuffer[cmdBufPos] = '\0';
    if (cmdBufPos >= 4) {
      // check command here!
      if (strcmp(cmdBuffer, "!B11:") == 0) {
        buttonPress(1); // 1 DOWN
      } else if (strcmp(cmdBuffer, "!B219") == 0) {
        buttonPress(2); // 2 DOWN
      } else if (strcmp(cmdBuffer, "!B318") == 0) {
        buttonPress(3); // 3 DOWN
      } else if (strcmp(cmdBuffer, "!B417") == 0) {
        buttonPress(4); // 4 DOWN
      } else {
        // DEBUG
        /*
        Serial.print(" BUFFER: [");
        Serial.print(cmdBuffer);
        Serial.println("]");
        */
      }
    }
  } else {
    // buffer overflow!
    cmdBufPos = 0;
  }
}

//-------------------------------------------------------------------------------------------------
void buttonPress(int button) {
  Serial.print("Button ");
  Serial.print(button);
  Serial.println(" pressed.");
  lastMessageTime = millis();
  // reset our buffer!
  cmdBufPos = 0;
  measureState = button;

  switch (button) {
  case 1:
    altitude = 1.6;
    measureState = button;
    break;
  case 2:
    altitude = 5;
    break;
  case 3:
    altitude = 10;
    break;
  case 4:
    altitude = 15;
    break;
  }
}

// data received from bluetooth UART
//-------------------------------------------------------------------------------------------------
void rxCallback(uint8_t *buffer, uint8_t len) {
  /*
  Serial.print(F("         Received "));
  //Serial.print(len);
  //Serial.print(F(" bytes: "));
  for (int i = 0; i < len; i++)
  {
    Serial.print((char)buffer[i]);
  }
  Serial.println("");
 */
  for (int i = 0; i < len; i++) {
    addToCMDBuffer((char)buffer[i]);
  }

  uart.write(buffer, len);
}

//-------------------------------------------------------------------------------------------------
void sensorLoop() {
  if (measureState >= 0) {
    // good, we should measure!
    measureState = -1;
    Serial.println("SENSOR LOOP WITHOUT DHT. :-(");
  } else {
    return;
  }
  // WEATHER
  humidity = -1;
  temperature = -1;
  // humidity = dht.readHumidity();
  // temperature = dht.readTemperature(); // Read temperature as Celsius (the
  // default)
  /*
  if (isnan(humidity) || isnan(temperature))
  {
    Serial.println("Failed to read from DHT sensor!");
    humidity = -1;
    temperature = -1;
  }
  */

  // ANEMOMETER
  readWindSensor();

  // GPS
  readGPS();

  // SOUND recording
  recordMicrophoneVolume();

  logToSDCard();
}

//-------------------------------------------------------------------------------------------------
void readGPS() {

  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;) {
    while (gpsPort.available()) {
      char c = gpsPort.read();
      Serial.write(
          c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  unsigned long age;
  if (newData) {
    gps.f_get_position(&latitude, &longitude, &age);
    elevation = gps.f_altitude();
    Serial.println("");
    Serial.print("LAT=");
    Serial.print(latitude == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : latitude, 6);
    Serial.print(" LON=");
    Serial.print(longitude == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : longitude,
                 6);
    Serial.print(" ALT=");
    Serial.println(
        elevation == TinyGPS::GPS_INVALID_F_ALTITUDE ? 0.0 : elevation, 6);
    Serial.println("");
  } else {

    // satellites in view
    uint32_t *satz = gps.trackedSatellites();
    uint8_t sat_count = 0;
    for (int i = 0; i < 24; i++) {
      if (satz[i] != 0) { // exclude zero SNR sats
        sat_count++;
        byte strength = (satz[i] & 0xFF) >> 1;
        byte prn = satz[i] >> 8;
        sprintf(gpsBuf, "PRN %d: ", prn);
        Serial.print(gpsBuf);
        Serial.print(strength);
        Serial.println("dB");
      }
    }
  }

  // date time
  int year;
  uint8_t month, day, hour, minutes, second, hundredths;
  gps.crack_datetime(&year, &month, &day, &hour, &minutes, &second, &hundredths,
                     &age);
  sprintf(timeMsg, "GPS time: %d/%02d/%02d %02d:%02d:%02d", year, month, day,
          hour, minutes, second);
  Serial.println("");
  Serial.println(timeMsg);

  gps.stats(&chars, &sentences, &failed);

  /*  // DEBUG STUFF
    Serial.print(" CHARS=");
    Serial.print(chars);
    Serial.print(" SENTENCES=");
    Serial.print(sentences);
    Serial.print(" CSUM ERR=");
    Serial.println(failed);
    */
  if (chars == 0) {
    Serial.println("** No characters received from GPS: check wiring **");
  }
}

//-------------------------------------------------------------------------------------------------
void readWindSensor() {
  lastPulseTime = 0;
  pulseTime = -1;
  unsigned long recordingStart = millis();
  unsigned long now = millis();

  while (now < recordingStart + 2000) // 2 seconds of reading
  {
    measuringBlink();
    if (lastPulseTime != 0) {
      // WIND SENSOR
      if (now - lastPulseTime > pulseTime) {
        if (pulseTime == -1) {
          pulseTime = (now - lastPulseTime);
        } else {
          pulseTime += ((float)(now - lastPulseTime) - pulseTime) / 50;
        }
      }
    }

    if (bouncer.update()) {
      if (bouncer.read() == HIGH) {
        if (lastPulseTime != 0) {
          if (pulseTime == -1) {
            pulseTime = (now - lastPulseTime);
          } else {
            pulseTime += ((float)(now - lastPulseTime) - pulseTime) / 50;
          }
        }
        lastPulseTime = now;
      }
    }
    now = millis();
  }

  if (pulseTime <= 0) {
    windSpeed = 0;
  } else {
    // pulsetime = milliseconds / 1 rotation
    // rotations / minute =  1 / (pulsetime/60000)
    windSpeed = 60000 / pulseTime;
    // TODO: weird peaks when going from 0 speed to something higher and also at
    // the low end close to measurement threshold :-/
  }
}

//-------------------------------------------------------------------------------------------------
void recordMicrophoneVolume() {

  bufPos = 0;
  while (bufPos < BUFSIZE) {
    measuringBlink();

    unsigned long beginSampling = millis(); // start time of our sampling
    unsigned long numSamples = 0;
    int maxSample = LEVEL0;
    int minSample = LEVEL0;

    while (millis() < beginSampling + 2) {
      int sample =
          analogRead(MICROPHONE_PIN); // audio is going up and down around 2.5V
                                      // == 512 in analog read values
      maxSample = max(maxSample, sample);
      minSample = min(minSample, sample);
      numSamples++; // keep track of how many samples we've collected
    }

    float volume = 2 * (maxSample - minSample);
    volBuffer[bufPos] = volume;
    bufPos++;

    /*
  Serial.print(numSamples);
  Serial.print(" ");
  Serial.print(zenoVolume);
  Serial.print(" ");
  Serial.print(avgVolume);
  Serial.print(" ");
  Serial.println(volume/2);
   */
  }

  soundVolume = 0;
  for (int i = 0; i < BUFSIZE; i++) {
    soundVolume += volBuffer[i];
  }
  soundVolume /= BUFSIZE;
}

//-------------------------------------------------------------------------------------------------
void logToSDCard() {
  // timeMsg, altitude, windSpeed, humidity, temperature, soundVolume,
  // longitude, latitude, elevation
  String dataString = "";
  dataString += String(timeMsg);
  dataString += ", ";
  dataString += String(altitude);
  dataString += ", ";
  dataString += String(windSpeed);
  dataString += ", ";
  dataString += String(humidity);
  dataString += ", ";
  dataString += String(temperature);
  dataString += ", ";
  dataString += String(soundVolume);
  dataString += ", ";
  dataString += String(longitude);
  dataString += ", ";
  dataString += String(latitude);
  dataString += ", ";
  dataString += String(elevation);

  File dataFile = SD.open("LOG.CSV", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.print("Logged data: ");
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.CSV");
  }
  lastMessageTime = millis();
}

//-------------------------------------------------------------------------------------------------
void measuringBlink() {
  unsigned long now = millis();
  if (((now > lastBlink + 50) && ledOn) ||
      ((now > lastBlink + 1000) && !ledOn)) {
    ledOn = !ledOn;
    if (ledOn) {
      digitalWrite(33, HIGH);
    } else {
      digitalWrite(33, LOW);
    }
    lastBlink = now;
  }
}

//-------------------------------------------------------------------------------------------------
void errorBlink() {
  while (true) {
    digitalWrite(33, HIGH);
    delay(100);
    digitalWrite(33, LOW);
    delay(100);
  }
}