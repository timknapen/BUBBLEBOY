
#define AIRQPIN A17
#define AIRQPOWERPIN 39

unsigned long lastEvent = 0;
bool powerIsOn = false;

void setup()
{
  pinMode(LED_BUILTIN, OUTPUT);
  while (!Serial)
  {
    for (int i = 0; i < 3; i++)
    {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(200);
      digitalWrite(LED_BUILTIN, LOW);
      delay(200);
    }
    delay(500);
  }
  pinMode(AIRQPOWERPIN, OUTPUT);
  digitalWrite(AIRQPOWERPIN, LOW);
  pinMode(AIRQPIN, INPUT);

  Serial.begin(57600);
  Serial.println("Air Quality measurement");
}

void loop()
{
  int analogSensor = analogRead(AIRQPIN);

  // Serial.print("Pin A0: ");
  Serial.println(analogSensor);
  delay(50);

  unsigned long now = millis();
  if (powerIsOn)
  {
    if (now > lastEvent + 60000)
    {
      lastEvent = now;
      digitalWrite(AIRQPOWERPIN, LOW);
      powerIsOn = false;
      Serial.println("# power off");
    }
  }
  else
  {
    if (now > lastEvent + 10000)
    {
      lastEvent = now;

      digitalWrite(AIRQPOWERPIN, HIGH);
      powerIsOn = true;
      Serial.println("# power back on");
    }
  }
}
