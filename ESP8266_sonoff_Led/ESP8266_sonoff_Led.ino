#include <ESP8266WiFi.h>

const byte pinLed = 13;
const byte pinPulsador = 0;

// Estado LED
boolean estadoLed = true;

void setup() {
  Serial.begin(115200);
  pinMode(pinLed, OUTPUT);
  pinMode(pinPulsador, INPUT);
}

void loop() {
  estadoLed = !estadoLed;
  if(estadoLed)
    Serial.println("ON");
  else
    Serial.println("OFF");
  
  delay(500);


  digitalWrite(pinLed, estadoLed);
}
