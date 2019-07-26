#include <Bounce.h>

#define ANEMOPIN 34
Bounce bouncer = Bounce( ANEMOPIN, 5 );


unsigned long lastPulseTime = 0;
float pulseTime = 0;
int lastPinState = LOW;

void setup() {

  while (!Serial);

  Serial.begin(57600);
  Serial.println("Hello, I am an anemometer, look me up on wikipedia");
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  pinMode(ANEMOPIN, INPUT_PULLUP);

}

void loop() {

  if ( bouncer.update() ) {
    if ( bouncer.read() == HIGH) {
      digitalWrite(LED_BUILTIN, HIGH);

      unsigned long now = millis();
      pulseTime = pulseTime + ((float)( now - lastPulseTime) - pulseTime) / 5;
      lastPulseTime = now;
      Serial.println(pulseTime);
    } else {
      digitalWrite(LED_BUILTIN, LOW);
    }

  }

}
