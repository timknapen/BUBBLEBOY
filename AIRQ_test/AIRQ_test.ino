
#define AIRQPIN A0


void setup() {
  pinMode(AIRQPIN, INPUT);
  Serial.begin(9600);
  
}

void loop() {
  int analogSensor = analogRead(AIRQPIN);

  Serial.print("Pin A0: ");
  Serial.println(analogSensor);
  delay(1000);
}
