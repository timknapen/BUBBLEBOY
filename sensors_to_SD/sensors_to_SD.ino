#include <Bounce.h>
#include <SD.h>
#include <SPI.h>
#include "DHT.h"

// MICRO SD
const int chipSelect = BUILTIN_SDCARD;

// WEATHER SENSOR
#define DHTPIN 12
#define DHTTYPE DHT11 // DHT 11
DHT dht(DHTPIN, DHTTYPE);

// WIND SENSOR
#define ANEMOPIN 34
Bounce bouncer = Bounce(ANEMOPIN, 5);
unsigned long lastPulseTime = 0;
float pulseTime = -1;
float windSpeed = 0; // windspeed = rotations / minute ( = pulses / minute)
int lastPinState = LOW;

// SOUND VOLUME DETECTION
//the microphone analog out connects to A0
#define MICROPHONE_PIN A0
#define LEVEL0 520
int soundVolume = 0;
#define BUFSIZE 256 // 2^12 (4096)// 8 seconds of recording!
int volBuffer[BUFSIZE];
int bufPos = 0;

// MEASURING FEEDBACK
unsigned long lastBlink = 0;
bool ledOn = false;

unsigned long lastLogTime = 0;

void setup()
{

  Serial.begin(57600);
  Serial.println("Hello, I am the sensor clubhouse. I live in a balloon.");

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // WEATHER
  dht.begin();

  // WIND
  pinMode(ANEMOPIN, INPUT_PULLUP);

  // SD
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect))
  {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    errorBlink();
    return;
  }
  Serial.println("card initialized.");

  // MICROPHONE
  pinMode(MICROPHONE_PIN, INPUT);
  for (int i = 0; i < BUFSIZE; i++)
  {
    volBuffer[i] = 0;
  }
}

void loop()
{

  // WEATHER
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature(); // Read temperature as Celsius (the default)
  if (isnan(humidity) || isnan(temperature))
  {
    Serial.println("Failed to read from DHT sensor!");
    humidity = -1;
    temperature = -1;
  }

  /********************************************/
  readWindSensor();

  // SOUND recording
  recordMicrophoneVolume();

  // LOGGING
  unsigned long now = millis();
  if (now - lastLogTime > 500)
  {
    lastLogTime = now;
    logToSDCard(windSpeed,
                humidity,
                temperature,
                soundVolume);
  }
}

void readWindSensor()
{
  lastPulseTime = 0;
  pulseTime = -1;
  unsigned long recordingStart = millis();
  unsigned long now = millis();

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

  if (pulseTime <= 0 )
  {
    windSpeed = 0;
  }
  else
  {
    // pulsetime = milliseconds / 1 rotation
    // rotations / minute =  1 / (pulsetime/60000)
    windSpeed = 60000 / pulseTime;
    // TODO: weird peaks when going from 0 speed to something higher and also at the low end close to measurement threshold :-/
  }
}

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
      int sample = analogRead(MICROPHONE_PIN); // audio is going up and down around 2.5V == 512 in analog read values
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

void logToSDCard(float _windspeed,
                 float _humidity,
                 float _temperature,
                 int _soundVolume)
{
  String dataString = "";
  dataString += millis();
  dataString += ", ";
  dataString += String(_windspeed);
  dataString += ", ";
  dataString += String(_humidity);
  dataString += ", ";
  dataString += String(_temperature);
  dataString += ", ";
  dataString += String(_soundVolume);

  File dataFile = SD.open("LOG.CSV", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile)
  {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else
  {
    Serial.println("error opening datalog.CSV");
  }
}

void measuringBlink()
{
  unsigned long now = millis();
  if (((now > lastBlink + 50) && ledOn) ||
      ((now > lastBlink + 1000) && !ledOn))
  {
    ledOn = !ledOn;
    if (ledOn)
    {
      digitalWrite(LED_BUILTIN, HIGH);
    }
    else
    {
      digitalWrite(LED_BUILTIN, LOW);
    }
    lastBlink = now;
  }
}

void errorBlink()
{
  while (true)
  {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN, LOW);
    delay(100);
  }
}