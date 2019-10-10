/****
   Simple analog A0 output to serial
*/
#define SENSE_PIN A0
#define SAMPLE_DELAY 5 // delay between samples in milliseconds

#define SAMPLEBUFFERSIZE 256
int samplebuffer[SAMPLEBUFFERSIZE];
unsigned long lastMessage = 0;

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
  Serial.println("# Simple Sampler");
  Serial.print("# Sample delay is: ");
  Serial.println(SAMPLE_DELAY);
  Serial.print("# so frequency is ");
  Serial.print((int) 1000 / SAMPLE_DELAY);
  Serial.println();
  delay(1000);

  pinMode(SENSE_PIN, INPUT);
}

void loop()
{

  //* // TODO: work with a sample buffer instead of one sample
  // Gather samples until buffer is full
  int pos = 0; // position in sample buffer
  float avgVal = 0;
  int val;
  while (pos < SAMPLEBUFFERSIZE)
  { // half a second
    val = analogRead(SENSE_PIN);
    samplebuffer[pos] = val;
    avgVal += val;
    pos++;
  }
  avgVal /= (float)SAMPLEBUFFERSIZE;
  Serial.print(val);
  Serial.print(" ");
  Serial.print((int)avgVal);
  Serial.println();

  while (millis() < lastMessage + SAMPLE_DELAY)
  {
    // wait before sending out the data
    lastMessage = millis();
  }
}
