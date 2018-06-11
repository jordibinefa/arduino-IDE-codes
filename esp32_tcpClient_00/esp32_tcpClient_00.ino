#include <WiFi.h>

const char* ssid     = "IoT-eCat";
const char* password = "clotClot";

/*
 * This is the IP address of your PC
 * [Wins: use ipconfig command, Linux: use ifconfig command]
*/
const char* host = "192.168.1.17";
const int port = 8088;

void setup() {
  Serial.begin(115200);
  Serial.print("Connecting to ");
  Serial.println(ssid);
  /* connecting to WiFi */
  WiFi.begin(ssid, password);
  /*wait until ESP32 connect to WiFi*/
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
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

  delay(5000);

  Serial.print("Connectant a ");
  Serial.print(host);
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
  Serial.print(" - Tramès: ");
  Serial.println(stMsg);
}
