// Based on https://www.baldengineer.com/mqtt-tutorial.html
//
// https://wiki.binefa.cat

#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>

// Connect to the WiFi
const char* ssid = "IoT-eCat";
const char* password = "clotClot";
const char* mqtt_server = "popotamo.binefa.cat";
const int mqtt_port = 1888; // normally 1883

WiFiClient espClient;
PubSubClient client(espClient);

const byte ledPin = 2,ledRed = 15,ledGreen = 12,ledBlue = 13;

void callback(char* topic, byte* payload, unsigned int length) {
  String szRx="";
  
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i = 0; i < length; i++) {
    char receivedChar = (char)payload[i];
    szRx += receivedChar;
    /*Serial.print(receivedChar);
    if (receivedChar == '0'){
      // ESP8266 Huzzah outputs are "reversed"
      digitalWrite(ledPin, HIGH);
      client.publish("outTopic", "I've got a 0");
    }
    if (receivedChar == '1'){
      digitalWrite(ledPin, LOW);
      client.publish("outTopic", "I've got a 1");
    }*/
  }
  Serial.println(szRx);
  if(szRx == "2L") digitalWrite(ledPin,LOW);
  if(szRx == "2H") digitalWrite(ledPin,HIGH);
  if(szRx == "13L") digitalWrite(ledGreen,LOW);
  if(szRx == "13H") digitalWrite(ledGreen,HIGH);
}


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect("ESP8266 Client")) {
      Serial.println("connected");
      // ... and subscribe to topic
      client.subscribe("/witty/led");
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

  digitalWrite(ledRed,HIGH);
  digitalWrite(ledBlue,LOW);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  digitalWrite(ledRed,LOW);
  digitalWrite(ledBlue,HIGH);

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void setup()
{
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
  pinMode(ledRed, OUTPUT);
  pinMode(ledGreen, OUTPUT);
   pinMode(ledBlue, OUTPUT);
 
  setup_wifi();
  client.setServer(mqtt_server, mqtt_port);
  client.setCallback(callback);
}

void loop()
{
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
}
