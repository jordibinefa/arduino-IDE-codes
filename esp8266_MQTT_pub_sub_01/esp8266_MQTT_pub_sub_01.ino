// Based on https://www.baldengineer.com/mqtt-tutorial.html
//
// https://wiki.binefa.cat

#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

#define EXTERNAL_BUTTON 4

// Connect to the WiFi
const char* ssid = "IoT-eCat";
const char* password = "clotClot";
const char* mqtt_server = "popotamo.binefa.cat";
const int mqtt_port = 1888; // normally 1883
//const char* mqtt_server = "test.mosquitto.org";
//const int mqtt_port = 1883; // normally 1883

#define TEMA_PUBLICA_ESTAT_BOTO "/jordi/witty/boto"
#define TEMA_SUBSCRIPCIO_ESTAT_LEDS "/jordi/witty/leds"

WiFiClient espClient;
PubSubClient client(espClient);

const byte ledPin = 2, ledRed = 15, ledGreen = 12, ledBlue = 13;
const byte button = EXTERNAL_BUTTON;

void callback(char* topic, byte* payload, unsigned int length) {
  String szRx = "", szTema(topic);

  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    char receivedChar = (char)payload[i];
    szRx += receivedChar;
  }
  Serial.println(szRx);
  if (szTema == TEMA_SUBSCRIPCIO_ESTAT_LEDS) {
    if (szRx == "2L") digitalWrite(ledPin, LOW);
    if (szRx == "2H") digitalWrite(ledPin, HIGH);
    if (szRx == "12L") digitalWrite(ledBlue, LOW);
    if (szRx == "12H") digitalWrite(ledBlue, HIGH);
    if (szRx == "13L") digitalWrite(ledGreen, LOW);
    if (szRx == "13H") digitalWrite(ledGreen, HIGH);
    if (szRx == "15L") digitalWrite(ledRed, LOW);
    if (szRx == "15H") digitalWrite(ledRed, HIGH);
  }
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266 Client <- Canvieu aquest nom")) {
      Serial.println("connected");
      // ... and subscribe to topic
      client.subscribe(TEMA_SUBSCRIPCIO_ESTAT_LEDS);
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup_wifi() {
  delay(10);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  digitalWrite(ledRed, HIGH);
  digitalWrite(ledBlue, LOW);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  digitalWrite(ledRed, LOW);
  digitalWrite(ledBlue, HIGH);

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledBlue, OUTPUT);
  pinMode(button, INPUT);

  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop() {
  boolean bButtonState = !digitalRead(EXTERNAL_BUTTON);
  static boolean bLastButtonState = bButtonState;

  if (!client.connected()) {
    reconnect();
  }

  if (bButtonState != bLastButtonState) {
    bLastButtonState = bButtonState;
    delay(50);
    if (bButtonState)
      client.publish(TEMA_PUBLICA_ESTAT_BOTO, "Botó GPIO4 NO premut");
    else
      client.publish(TEMA_PUBLICA_ESTAT_BOTO, "Botó GPIO4 ÉS premut");
  }
  client.loop();
}
