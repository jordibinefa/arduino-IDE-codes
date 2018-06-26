// https://wiki.binefa.cat

#include "PCF8574.h" // https://github.com/RobTillaart/Arduino/tree/master/libraries/PCF8574
#include <PubSubClient.h>
#include <WiFi.h>
#include <Wire.h>
#include "credentials.h"

#define I2C_SDA 21
#define I2C_SCL 22

#define I2C_SDA_OLED 4
#define I2C_SCL_OLED 15

#define R1 0x01
#define R2 0x02
#define R3 0x04
#define R4 0x08

#define EXTERNAL_BUTTON 23
#define ADC_POT 35

PCF8574 PCF_38(0x38);
//PCF8574 PCF_38(0x20);

#define TEMA_PUBLICA_ESTAT_BOTO "/jordi/witty/boto"
#define TEMA_SUBSCRIPCIO_ESTAT_LEDS "/jordi/witty/leds"

WiFiClient espClient;
PubSubClient client(espClient);

static byte byState;
const byte button = EXTERNAL_BUTTON;

void vDelayESP(unsigned long ulMilliseconds) {
  unsigned long ulPreviousMillis = millis();

  do {
    yield();
  } while (millis() - ulPreviousMillis <= ulMilliseconds);
}

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
    if (szRx == "2L" || szRx == "R1L" ) byState &= ~R1;
    if (szRx == "2H" || szRx == "R1H") byState |= R1;
    if (szRx == "12L" || szRx == "R2L") byState &= ~R2;
    if (szRx == "12H" || szRx == "R2H") byState |= R2;
    if (szRx == "13L" || szRx == "R3L") byState &= ~R3;
    if (szRx == "13H" || szRx == "R3H") byState |= R3;;
    if (szRx == "15L" || szRx == "R4L" ) byState &= ~R4;
    if (szRx == "15H" || szRx == "R4H") byState |= R4;
    PCF_38.write8(~byState);
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
      vDelayESP(5000);
    }
  }
}

void setup_wifi() {
  WiFi.enableSTA(true);
  vDelayESP(2000);
  WiFi.begin(ssid, password);

  Serial.print("[Connecting]");
  Serial.print(ssid);
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED) {
    vDelayESP(500);
    Serial.print(".");
    //tries++;
    //if (tries > 30) {
    //  break;
    //}
  }
  Serial.println();

  printWifiStatus();

  Serial.println("Connected to wifi");
}

void setup() {
  pinMode(EXTERNAL_BUTTON, INPUT_PULLUP);
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  //Wire.begin(I2C_SDA_OLED, I2C_SCL_OLED);

  byState = 0x00;
  PCF_38.write8(~byState);
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
    vDelayESP(50);
    if (!bButtonState)
      client.publish(TEMA_PUBLICA_ESTAT_BOTO, "Botó GPIO4 NO premut");
    else
      client.publish(TEMA_PUBLICA_ESTAT_BOTO, "Botó GPIO4 ÉS premut");
  }
  client.loop();
}

void printWifiStatus() {
  // print the SSID of the network you're attached to:
  Serial.print("SSID: ");
  Serial.println(WiFi.SSID());

  // print your WiFi ESP8266's IP address:
  IPAddress ip = WiFi.localIP();
  Serial.print("IP Address: ");
  Serial.println(ip);
}
