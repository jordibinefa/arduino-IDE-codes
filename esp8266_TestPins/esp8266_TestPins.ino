// Testing ESP8266 board using LDR, RGB led & button
//
// by Jordi Binefa - twitter.com/jordibinefa
//
// 20160328 - wiki.binefa.cat
#include <ESP8266WiFi.h>

#define GPIO5 5
#define GPIO4 4
#define GPIO0 0
#define GPIO2 2
#define GPIO15 15
#define GPIO13 13
#define GPIO12 12
#define GPIO14 14
#define GPIO16 16

String szMsg;

void delayESP8266(unsigned long ulMilliseconds){
  unsigned long ulPreviousMillis = millis();

  do{
    yield();
  }while(millis() - ulPreviousMillis <= ulMilliseconds);
}

void setup() {
  Serial.begin(9600);

  pinMode(GPIO5, OUTPUT);
  pinMode(GPIO4, INPUT ); // For button. It can be used as output as well.
  pinMode(GPIO0, OUTPUT); // Flash button
  pinMode(GPIO2, OUTPUT); // Small blue led
  pinMode(GPIO15,OUTPUT); // Red of RGB led
  pinMode(GPIO13,OUTPUT); // Blue of RGB led
  pinMode(GPIO12,OUTPUT); // Green of RGB led
  pinMode(GPIO14,OUTPUT);
  pinMode(GPIO16,OUTPUT);
  pinMode(A0    ,INPUT ); // LDR
}

void vManageMsg(){
  //Serial.println(szMsg);

  if(szMsg == "a" || szMsg == "A"){
    Serial.print("LDR level (0..1024): ");
    Serial.println(analogRead(A0));
  }
  if(szMsg == "i" || szMsg == "I"){
    if(digitalRead(GPIO4))
      Serial.println("Button is not pressed: GPIO4 HIGH");
    else
      Serial.println("Button is pressed: GPIO4 LOW");
  }
  if(szMsg == "5h" || szMsg == "5H"){
    digitalWrite(GPIO5, HIGH);
    Serial.println("GPIO5 HIGH");
  }
  if(szMsg == "5l" || szMsg == "5L"){
    digitalWrite(GPIO5, LOW);
    Serial.println("GPIO5 LOW");
  }
  if(szMsg == "4h" || szMsg == "4H"){
    digitalWrite(GPIO4, HIGH);
    Serial.println("GPIO4 HIGH");
  }
  if(szMsg == "4l" || szMsg == "4L"){
    digitalWrite(GPIO4, LOW);
    Serial.println("GPIO4 LOW");
  }
  if(szMsg == "0h" || szMsg == "0H"){
    digitalWrite(GPIO0, HIGH);
    Serial.println("GPIO0 HIGH");
  }
  if(szMsg == "0l" || szMsg == "0L"){
    digitalWrite(GPIO0, LOW);
    Serial.println("GPIO0 LOW");
  }
  if(szMsg == "2h" || szMsg == "2H"){
    digitalWrite(GPIO2, HIGH);
    Serial.println("GPIO2 HIGH");
  }
  if(szMsg == "2l" || szMsg == "2L"){
    digitalWrite(GPIO2, LOW);
    Serial.println("GPIO2 LOW");
  }
  if(szMsg == "15h" || szMsg == "15H"){
    digitalWrite(GPIO15, HIGH);
    Serial.println("GPIO15 HIGH -> Red ON");
  }
  if(szMsg == "15l" || szMsg == "15L"){
    digitalWrite(GPIO15, LOW);
    Serial.println("GPIO15 LOW -> Red OFF");
  }
  if(szMsg == "13h" || szMsg == "13H"){
    digitalWrite(GPIO13, HIGH);
    Serial.println("GPIO13 HIGH -> Blue ON");
  }
  if(szMsg == "13l" || szMsg == "13L"){
    digitalWrite(GPIO13, LOW);
    Serial.println("GPIO13 LOW -> Blue OFF");
  }
  if(szMsg == "12h" || szMsg == "12H"){
    digitalWrite(GPIO12, HIGH);
    Serial.println("GPIO12 HIGH -> Green ON");
  }
  if(szMsg == "12l" || szMsg == "12L"){
    digitalWrite(GPIO12, LOW);
    Serial.println("GPIO12 LOW -> Green OFF");
  }
  if(szMsg == "14h" || szMsg == "14H"){
    digitalWrite(GPIO14, HIGH);
    Serial.println("GPIO14 HIGH");
  }
  if(szMsg == "14l" || szMsg == "14L"){
    digitalWrite(GPIO14, LOW);
    Serial.println("GPIO14 LOW");
  }
  if(szMsg == "16h" || szMsg == "16H"){
    digitalWrite(GPIO16, HIGH);
    Serial.println("GPIO16 HIGH");
  }
  if(szMsg == "16l" || szMsg == "16L"){
    digitalWrite(GPIO16, LOW);
    Serial.println("GPIO16 LOW");
  }
}

void loop() {
  while(Serial.available()){
    delayESP8266(3);
    char c = Serial.read();
    szMsg += c;
  }
  if(szMsg != "")
    vManageMsg();
  szMsg = "";
}
