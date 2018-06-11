#include <ESP8266WiFi.h>

const char* ssid     = "IoT-eCat";
const char* password = "clotClot";

/*
 * This is the IP address of your PC
 * [Wins: use ipconfig command, Linux: use ifconfig command]
*/
const char* host = "192.168.1.17";
const int port = 8088;

void vDelayESP8266(unsigned long ulMilliseconds) {
  unsigned long ulPreviousMillis = millis();

  do {
    yield();
  } while (millis() - ulPreviousMillis <= ulMilliseconds);
}

void setup() {
  Serial.begin(115200);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  /* connecting to WiFi */
  WiFi.begin(ssid, password);
  /*wait until ESP32 connect to WiFi*/
  while (WiFi.status() != WL_CONNECTED) {
    vDelayESP8266(500);
    Serial.print(".");
  }
  Serial.println("");
  Serial.println("WiFi connected with IP address: ");
  Serial.println(WiFi.localIP());
}

void loop() {
  static long int lnCmpt = 0;
  String stMsg;
  char szMsg[22];
  
  Serial.print("connecting to ");
  Serial.println(host);
  /* Use WiFiClient class to create TCP connections */
  WiFiClient client;

  if (!client.connect(host, port)) {
    Serial.println("connection failed");
    return;
  }
  stMsg = "Cmpt: " + String(itoa(lnCmpt++,szMsg,10));
  stMsg.toCharArray(szMsg,stMsg.length());
  /* This will send the data to the server */
  client.print(stMsg);
  client.stop();
  vDelayESP8266(5000);
}
