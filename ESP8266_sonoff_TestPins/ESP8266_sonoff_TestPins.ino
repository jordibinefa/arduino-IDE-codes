// Testing Sonoff Basic
//
// by Jordi Binefa - twitter.com/jordibinefa
// 20190405 - wiki.binefa.cat
#include <ESP8266WiFi.h>

#define BUTTON 0 /* Button */
#define STATUS_LED 13 /* Status LED */
#define RELAY 12 /* Relay */
#define XTRA_PIN 14 /* Extra pin*/

String szMsg;

bool bRelay, bLed, bXtra;

void delayESP8266(unsigned long ulMilliseconds) {
  unsigned long ulPreviousMillis = millis();

  do {
    yield();
  } while (millis() - ulPreviousMillis <= ulMilliseconds);
}

void setup() {
  Serial.begin(115200);

  pinMode(STATUS_LED, OUTPUT);
  pinMode(BUTTON, INPUT );
  pinMode(RELAY, OUTPUT);
  pinMode(XTRA_PIN, OUTPUT);
  bRelay = bLed = bXtra = LOW;
  Serial.println("Testing Sonoff pins");
}

void vManageMsg() {
  if (szMsg == "i" || szMsg == "I") {
    if (digitalRead(BUTTON))
      Serial.println("Button is not pressed: GPIO0 HIGH");
    else
      Serial.println("Button is pressed: GPIO0 LOW");
  }
  if (szMsg == "l" || szMsg == "L") {
    bLed = !bLed;
    digitalWrite(STATUS_LED, bLed);
    Serial.print("LED GPIO13");
    (bLed)?Serial.println(" HIGH"):Serial.println(" LOW");
  }
  if (szMsg == "r" || szMsg == "R") {
    bRelay = !bRelay;
    digitalWrite(RELAY, bRelay);
    Serial.print("Relay GPIO12");
    (bRelay)?Serial.println(" HIGH"):Serial.println(" LOW");
  }
  if (szMsg == "x" || szMsg == "X") {
    bXtra = !bXtra;
    digitalWrite(XTRA_PIN, bXtra);
    Serial.print("Xtra GPIO14");
    (bXtra)?Serial.println(" HIGH"):Serial.println(" LOW");
  }
}

void loop() {
  while (Serial.available()) {
    delayESP8266(3);
    char c = Serial.read();
    szMsg += c;
  }
  vManageMsg();
  szMsg = "";
  digitalWrite(STATUS_LED, bLed);
  digitalWrite(RELAY, bRelay);
  digitalWrite(XTRA_PIN, bXtra);
}
