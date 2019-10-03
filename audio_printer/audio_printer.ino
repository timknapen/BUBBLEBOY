/****
   Audio output to serial
*/
#define MICROPHONE_PIN A0

#define NUMSAMPLES 4096 // 2^11
//#define NUMSAMPLES 4096 // 2^12

int samples[NUMSAMPLES];
int lastSample = 0;
int maxVal = 0;
int minVal = 0;

int sampleDelayUs = 50; // delay between samples in microseconds

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  while (!Serial)
  {
    // wait for serial..
    for (int i = 0; i < 3; i++)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
    delay(500);
  }
  Serial.begin(57600);
  Serial.println("Audio Sample Recorder");
  Serial.print("Recorder sample size: ");
  Serial.println(NUMSAMPLES);
  Serial.println(NUMSAMPLES - 1);
  delay(200);

  pinMode(MICROPHONE_PIN, INPUT);
}

void loop()
{

  unsigned long beginTime = millis(); // the time when we begin sampling

  // Gather samples for 10 milliseconds
  int pos = 0; // position in sample buffer
  maxVal = 0;
  minVal = 1024;
  while (pos < 100)
  { // half a second
    int val = analogRead(MICROPHONE_PIN);
    samples[pos] = val;
    minVal = min(val, minVal);
    maxVal = max(val, maxVal);
    pos++;
    delayMicroseconds(sampleDelayUs);
  }

  if (maxVal - minVal > 200) // we detected some kind of peak!
  {

    // continue sampling
    while (pos < NUMSAMPLES)
    { // half a second
      int val = analogRead(MICROPHONE_PIN);
      samples[pos] = val;
      minVal = min(val, minVal);
      maxVal = max(val, maxVal);
      pos++;
      delayMicroseconds(sampleDelayUs);
    }
    //*
    unsigned long endTime = millis();
    unsigned long runTime = endTime - beginTime;
    float sampleFreq = (float)NUMSAMPLES / runTime * 1000;
    //*/

    printSamples();

    Serial.print("#");
    Serial.print(NUMSAMPLES);
    Serial.print(" samples in ");
    Serial.print(runTime);
    Serial.print("ms. That's ");
    Serial.print((int)sampleFreq);
    Serial.println("Hz");
    delay(100);

    lastSample = samples[NUMSAMPLES - 1];
    delay(200);
  }
}

void printSamples()
{
  // prints our sound samples to the serial port
  for (int i = 0; i < NUMSAMPLES-1; i++)
  {
    Serial.println(samples[i]); // raw version
    //Serial.println(toDb(abs(samples[i] - samples[i+1]))); // dB version
    //Serial.println(abs(samples[i] - samples[i+1])); // dB version

    delayMicroseconds(10);
  }
}

float toDb(float intensity)
{
  return 20 * log10(intensity);
}