// uncomment to use magnetic sensor (compass)
//#define USE_MAG_SENSOR

// uncomment to use DHT11
#define USE_DHT11
//#define FUCK_INTERRUPTS

#ifdef USE_MAG_SENSOR
#include <Adafruit_HMC5883_U.h>
#endif

#ifdef USE_DHT11
#include "DHT.h"
#endif

#include <Adafruit_Sensor.h>
#include <Bounce.h>
#include <SD.h>
#include <SPI.h>
#include <Wire.h>
#include "Adafruit_BLE_UART.h"
#include <TinyGPS.h>

#define LED_PIN 33
// not used for now, but could be used to attach a bright LED (with a
// driver!) as a signal

// MICRO SD
const int chipSelect = BUILTIN_SDCARD;
bool bFoundSD = false;

// WEATHER SENSOR
#ifdef USE_DHT11
#define DHTPIN 24
#define DHTTYPE DHT11 // DHT 11
DHT dht(DHTPIN, DHTTYPE);
#endif

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

Adafruit_BLE_UART ble_uart =
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
long long_latitude, long_longitude;
// date time for GPS
int year;
uint8_t month, day, hour, minutes, second, hundredths;

// COMPASS / MAG SENSOR
/* Assign a unique ID to this sensor at the same time */
#ifdef USE_MAG_SENSOR
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
#endif
bool bFoundMagSensor = false;

// we didn't figure out how to calculate the compass heading, so let's just
// record raw data and worry about it later!
float heading_x = 0;
float heading_y = 0;
float heading_z = 0;

//-------------------------------------------------------------------------------------------------
void setup()
{

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  for (int i = 0; i < 5; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(50);
  }
  // while (!Serial) ;
  Serial.begin(57600);
  delay(100);
  Serial.println("Hello, I am the sensor clubhouse. I live in a balloon.");

#ifdef USE_DHT11
  // WEATHER
  dht.begin();
#endif

  // WIND
  pinMode(ANEMOPIN, INPUT_PULLUP);

  // SD
  Serial.println("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect))
  {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    errorBlink();
  }
  else
  {
    Serial.println("card initialized.");
    bFoundSD = true;
  }

  // MICROPHONE
  pinMode(MICROPHONE_PIN, INPUT);
  for (int i = 0; i < BUFSIZE; i++)
  {
    volBuffer[i] = 0;
  }

  // BLE Initializing
  ble_uart.setDeviceName("BBLEBOY"); /* 7 characters max! */
  ble_uart.begin();

  // GPS Initializing
  gpsPort.begin(9600);

// MAG SENSOR
#ifdef USE_MAG_SENSOR
  if (!mag.begin())
  {
    Serial.println("Ooops, no HMC5883 detected ... Check your wiring!");
  }
  else
  {
    bFoundMagSensor = true;
  }

  /* Display some basic information on this sensor */
  displayMagSensorDetails();
#endif
}

//-------------------------------------------------------------------------------------------------
void loop()
{
  ble_uart.pollACI();

  // read from BLE buffer

  // OK while we still have something to read, get a character and print it out
  while (ble_uart.available())
  {
    char c = ble_uart.read();
    // Serial.print(c);
    addToCMDBuffer(c);
  }

  // measuringBlink();
  sensorLoop();

  // keep alive messages
  unsigned long now = millis();
  if (now > lastMessageTime + 5000)
  {
    int waitCycle = now / 5000;
    lastMessageTime = now;
    Serial.print(waitCycle);
    Serial.println(" waiting... ");
    // DEBUG
    /*
    if (waitCycle == 2) {
      buttonPress(5);
    }
    */
    // uncomment to send waiting messages of BLE
    //ble_uart.write((unsigned char *)"waiting...", strlen("waiting..."));
  }
}

#ifdef USE_MAG_SENSOR
//-------------------------------------------------------------------------------------------------
void displayMagSensorDetails(void)
{
  if (!bFoundMagSensor)
  {
    return;
  }
  sensor_t sensor;
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print("Sensor:       ");
  Serial.println(sensor.name);
  Serial.print("Driver Ver:   ");
  Serial.println(sensor.version);
  Serial.print("Unique ID:    ");
  Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    ");
  Serial.print(sensor.max_value);
  Serial.println(" uT");
  Serial.print("Min Value:    ");
  Serial.print(sensor.min_value);
  Serial.println(" uT");
  Serial.print("Resolution:   ");
  Serial.print(sensor.resolution);
  Serial.println(" uT");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(100);
}

//-------------------------------------------------------------------------------------------------
void readCompass()
{
  if (!bFoundMagSensor)
  {
    return;
  }
  /* Get a new sensor event */
  sensors_event_t event;
  mag.getEvent(&event);

  heading_x = event.magnetic.x;
  heading_y = event.magnetic.y;
  heading_z = event.magnetic.z;
}
#endif

// BLE ACI Event callback
//-------------------------------------------------------------------------------------------------
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

//-------------------------------------------------------------------------------------------------
void addToCMDBuffer(char c)
{
  if (cmdBufPos < CMD_BUF_LEN - 1)
  {
    if (c == '!')
    {
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
      }
      else
      {
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

//-------------------------------------------------------------------------------------------------
void buttonPress(int button)
{
  Serial.print("Button ");
  Serial.print(button);
  Serial.println(" pressed.");
  lastMessageTime = millis();
  // reset our buffer!
  cmdBufPos = 0;
  measureState = button;

  switch (button)
  {
  case 5: // test mode!
    altitude = -1;
    Serial.println("THIS IS A FAKE BUTTON PRESS");
    break;
  case 1:
    altitude = 1.6;
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
void rxCallback(uint8_t *buffer, uint8_t len)
{
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
  for (int i = 0; i < len; i++)
  {
    addToCMDBuffer((char)buffer[i]);
  }

  ble_uart.write(buffer, len);
}

//-------------------------------------------------------------------------------------------------
void sensorLoop()
{
  if (measureState >= 0)
  {
    // good, we should measure!
    measureState = -1;

    Serial.println("");
#ifndef USE_DHT11
    Serial.println("SENSOR LOOP WITHOUT DHT. :-(");
#else
    Serial.println("SENSOR LOOP (with DHT ^_^)");
#endif
    ble_uart.write((uint8_t *)"Start log ", strlen("Start log "));
  }
  else
  {
    return;
  }
  // WEATHER
  humidity = -1;
  temperature = -1;

#ifdef USE_DHT11
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();
  // Read temperature as Celsius (the default)

  if (isnan(humidity) || isnan(temperature))
  {
    Serial.println("Failed to read from DHT sensor!");
    humidity = -1;
    temperature = -1;
  }else{
    Serial.print("temp: ");
    Serial.print(temperature);
    Serial.print("°C humidity: ");
    Serial.print(humidity);
    Serial.println("%");
  }
#endif

  // GPS
  readGPS();
  char bleBuf[64];
  sprintf(bleBuf, "at %02d:%02d:%02d, alt %.2fm.\n", hour, minutes, second, altitude);
  ble_uart.write((uint8_t *)bleBuf, strlen(bleBuf));

  // ANEMOMETER
  readWindSensor();

  // SOUND recording
  recordMicrophoneVolume();

#ifdef USE_MAG_SENSOR
  // read Compass/bFoundMagSensor
  readCompass();
#endif

  logToSDCard();

  Serial.println("");
}

//-------------------------------------------------------------------------------------------------
void readGPS()
{

  bool newData = false;
  unsigned long chars;
  unsigned short sentences, failed;

  // For one second we parse GPS data and report some key values
  for (unsigned long start = millis(); millis() - start < 1000;)
  {
    measuringBlink();
    while (gpsPort.available())
    {
      char c = gpsPort.read();
      //Serial.write( c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
    }
  }

  unsigned long age;
  if (newData)
  {
    Serial.println("New GPS data OK");
    gps.f_get_position(&latitude, &longitude, &age);
    unsigned long fix_age = 0;
    gps.get_position(&long_latitude, &long_longitude, &fix_age);
    elevation = gps.f_altitude();
    //*
    Serial.println("");
    Serial.print("LAT=");
    Serial.print(latitude == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : latitude, 6);
    Serial.print(" LON=");
    Serial.print(longitude == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : longitude,
                 6);

    Serial.print(" ALT=");
    Serial.print(
        elevation == TinyGPS::GPS_INVALID_F_ALTITUDE ? 0.0 : elevation, 6);
    Serial.print(" (LONG version: ");
    Serial.print(long_latitude);
    Serial.print(", ");
    Serial.print(long_longitude);
    Serial.println(")");
    Serial.println("");
    //*/
  }
  else
  {
    Serial.println("No new GPS data :(");
    ble_uart.write((uint8_t *)"(°_°) No GPS :(\n", strlen("(°_°) No GPS :(\n"));
    // satellites in view
    uint32_t *satz = gps.trackedSatellites();
    uint8_t sat_count = 0;
    for (int i = 0; i < 24; i++)
    {
      if (satz[i] != 0)
      { // exclude zero SNR sats
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

  gps.crack_datetime(&year, &month, &day, &hour, &minutes, &second, &hundredths, &age);
  hour += 2; // timezon Paris is +2 now (summer)
  sprintf(timeMsg, "%d/%02d/%02d %02d:%02d:%02d", year, month, day, hour, minutes, second);
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
  if (chars == 0)
  {
    Serial.println("** No characters received from GPS: check wiring **");
  }
}

//-------------------------------------------------------------------------------------------------
void readWindSensor()
{
  lastPulseTime = 0;
  pulseTime = -1;
  unsigned long recordingStart = millis();
  unsigned long now = millis();

  Serial.print("Reading wind sensor... ");
  while (now < recordingStart + 2000) // 2 seconds of reading
  {
    measuringBlink();
    if (lastPulseTime != 0)
    {
      // WIND SENSOR
      if (now - lastPulseTime > pulseTime)
      {
        if (pulseTime == -1)
        {
          pulseTime = (now - lastPulseTime);
        }
        else
        {
          pulseTime += ((float)(now - lastPulseTime) - pulseTime) / 50;
        }
      }
    }

    if (bouncer.update())
    {
      if (bouncer.read() == HIGH)
      {
        if (lastPulseTime != 0)
        {
          if (pulseTime == -1)
          {
            pulseTime = (now - lastPulseTime);
          }
          else
          {
            pulseTime += ((float)(now - lastPulseTime) - pulseTime) / 50;
          }
        }
        lastPulseTime = now;
      }
    }
    now = millis();
  }

  if (pulseTime <= 0)
  {
    windSpeed = 0;
  }
  else
  {
    // pulsetime = milliseconds / 1 rotation
    // rotations / minute =  1 / (pulsetime/60000)
    windSpeed = 60000 / pulseTime;
    // TODO: weird peaks when going from 0 speed to something higher and also at
    // the low end close to measurement threshold :-/
  }
  Serial.print(windSpeed);
  Serial.println("rpm");
}

//-------------------------------------------------------------------------------------------------
void recordMicrophoneVolume()
{
  bufPos = 0;
  while (bufPos < BUFSIZE)
  {
    measuringBlink();

    unsigned long beginSampling = millis(); // start time of our sampling
    unsigned long numSamples = 0;
    int maxSample = LEVEL0;
    int minSample = LEVEL0;

    while (millis() < beginSampling + 2)
    {
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
  for (int i = 0; i < BUFSIZE; i++)
  {
    soundVolume += volBuffer[i];
  }
  soundVolume /= BUFSIZE;
}

//-------------------------------------------------------------------------------------------------
void logToSDCard()
{
  // timeMsg, altitude, windSpeed, humidity, temperature, soundVolume,
  // latitude, longitude, elevation,
  // compass X, Y, Z
  char gpsNumber[16];
  String dataString = "";
  dataString += String(timeMsg); // text version of the time
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
  sprintf(gpsNumber, "%.6f", latitude);
  dataString += gpsNumber;
  //dataString += String(latitude);
  dataString += ", ";
  sprintf(gpsNumber, "%.6f", longitude);
  dataString += gpsNumber;
  //dataString += String(longitude);

  dataString += ", ";
  dataString += String(elevation);
  dataString += ", ";
  dataString += String(heading_x);
  dataString += ", ";
  dataString += String(heading_y);
  dataString += ", ";
  dataString += String(heading_z);

  File dataFile;
  if (bFoundSD)
  {
    dataFile = SD.open("LOG.CSV", FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile)
    {
      dataFile.println(dataString);
      dataFile.close();
      // print to the serial port too:
      Serial.print("Logged data: ");
      Serial.println(dataString);
    }
    // if the file isn't open, pop up an error:
    else
    {
      Serial.println("error opening datalog.CSV");
      Serial.print("Data: ");
      Serial.println(dataString);
    }
  }

  char bleBuf[64];
  sprintf(bleBuf, "%02d:%02d:%02d logged at %.2fm.\n", hour, minutes, second, altitude);
  ble_uart.write((uint8_t *)bleBuf, strlen(bleBuf));

  lastMessageTime = millis();
}

//-------------------------------------------------------------------------------------------------
void measuringBlink()
{
  unsigned long now = millis();
  if (((now > lastBlink + 50) && ledOn) ||
      ((now > lastBlink + 200) && !ledOn))
  {
    ledOn = !ledOn;
    if (ledOn)
    {
      digitalWrite(LED_PIN, HIGH);
    }
    else
    {
      digitalWrite(LED_PIN, LOW);
    }
    lastBlink = now;
  }
}

//-------------------------------------------------------------------------------------------------
void errorBlink()
{
  for (int i = 0; i < 10; i++)
  {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}