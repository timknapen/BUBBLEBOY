
/***********************************************************

  Detecting volume levels from a small amplified microphone module
  and making a servo move with the volume

  First bias the microphone output so it's around Vcc/2 (2.5V in Arduino 5V land)

  26-7-2019

***********************************************************/

//the microphone analog out connects to A0
#define MICROPHONE_PIN A0
#define LEVEL0 520
float zenoVolume = 0;

#define BUFSIZE 4096 // 2^12 // 8 seconds of recording!
int volBuffer[BUFSIZE];
int bufPos = 0;

void setup() {

  Serial.begin(57600);
  Serial.println("Microphone volume detection test");
  delay(10);

  pinMode(MICROPHONE_PIN, INPUT);
  for(int i = 0; i < BUFSIZE; i ++){
    volBuffer[i] = 0;
  }

}

void loop() {

  unsigned long beginSampling = millis();              // start time of our sampling 
  unsigned long numSamples = 0;
  int maxSample = LEVEL0;
  int minSample = LEVEL0;

  while (millis() < beginSampling + 2 ) {
    int sample = analogRead(MICROPHONE_PIN); // audio is going up and down around 2.5V == 512 in analog read values
    maxSample = max( maxSample, sample);
    minSample = min( minSample, sample);
    numSamples ++;                                    // keep track of how many samples we've collected
  }

  float volume = 2 * (maxSample - minSample);
  volBuffer[bufPos] = volume;
  bufPos ++;
  if(bufPos >= BUFSIZE){
    bufPos = 0;
  }
  unsigned long avgVolume = 0;
  for(int i = 0; i < BUFSIZE; i++){
    avgVolume += volBuffer[i];
  }
  avgVolume /= BUFSIZE;


  zenoVolume += (volume - zenoVolume) / 1000.0f;

  Serial.print(numSamples);
  Serial.print(" ");
  Serial.print(zenoVolume);
  Serial.print(" ");
  Serial.print(avgVolume);
  Serial.print(" ");
  Serial.println(volume/2);

}
