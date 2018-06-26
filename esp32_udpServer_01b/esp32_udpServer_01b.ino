#include "PCF8574.h" // https://github.com/RobTillaart/Arduino/tree/master/libraries/PCF8574
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
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

unsigned int udpPortRx = 3334;
unsigned int udpPortTx = 3333;

static byte byState;

byte packetBuffer[512]; //buffer to hold incoming and outgoing packets

WiFiUDP Udp;

void vDelayESP(unsigned long ulMilliseconds) {
  unsigned long ulPreviousMillis = millis();

  do {
    yield();
  } while (millis() - ulPreviousMillis <= ulMilliseconds);
}

void vSendUdp(String sz, unsigned int uiLocalPort) {
  Udp.beginPacket(Udp.remoteIP(), uiLocalPort);
  Udp.print(sz);
  Udp.endPacket();
}

bool bManageMsg(String szMsg) {
  unsigned int nLdr = analogRead(ADC_POT)/4;
  bool bUnderstood = false;

  if (szMsg == "a" || szMsg == "A") {
    Serial.print("Pot level (0..4095/4): ");
    Serial.println(nLdr);
    vSendUdp(String(nLdr), udpPortTx);
    bUnderstood = true;
  }
  if (szMsg == "i" || szMsg == "I") {
    if (digitalRead(EXTERNAL_BUTTON)) {
      Serial.println("Button is not pressed: GPIO4 HIGH");
      vSendUdp("H", udpPortTx);
    } else {
      Serial.println("Button is pressed: GPIO4 LOW");
      vSendUdp("L", udpPortTx);
    }
    bUnderstood = true;
  }
  if (szMsg == "2h" || szMsg == "2H" || szMsg == "R1H" || szMsg == "r1h") {
    byState |= R1;
    Serial.println("R1 HIGH");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "2l" || szMsg == "2L" || szMsg == "R1L" || szMsg == "r1l") {
    byState &= ~R1;
    Serial.println("R1 LOW");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "13h" || szMsg == "13H" || szMsg == "R2H" || szMsg == "r2h") {
    byState |= R2;
    Serial.println("R2 HIGH");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "13l" || szMsg == "13L" || szMsg == "R2L" || szMsg == "r2l") {
    byState &= ~R2;
    Serial.println("R2 LOW");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "12h" || szMsg == "12H" || szMsg == "R3H" || szMsg == "r3h") {
    byState |= R3;
    Serial.println("R3 HIGH");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "12l" || szMsg == "13L" || szMsg == "R3L" || szMsg == "r3l") {
    byState &= ~R3;
    Serial.println("R3 LOW");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "15h" || szMsg == "15H" || szMsg == "R4H" || szMsg == "r4h") {
    byState |= R4;
    Serial.println("R4 HIGH");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "15l" || szMsg == "15L" || szMsg == "R4L" || szMsg == "r4l") {
    byState &= ~R4;
    Serial.println("R4 LOW");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }

  return bUnderstood;
}

void setup() {
  pinMode(EXTERNAL_BUTTON, INPUT_PULLUP);

  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);
  //Wire.begin(I2C_SDA_OLED, I2C_SCL_OLED);

  byState = 0x00;
  
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
  Serial.print("Udp server started at port ");
  Serial.println(udpPortRx);
  Udp.begin(udpPortRx);
}

void loop() {
  int noBytes = Udp.parsePacket();
  String received_command = "";

  if ( noBytes ) {
    Serial.print(millis() / 1000);
    Serial.print(":Packet of ");
    Serial.print(noBytes);
    Serial.print(" received from ");
    Serial.print(Udp.remoteIP());
    Serial.print(":");
    Serial.println(Udp.remotePort());
    // We've received a packet, read the data from it
    Udp.read(packetBuffer, noBytes); // read the packet into the buffer

    // display the packet contents in HEX
    for (int i = 1; i <= noBytes; i++) {
      Serial.print(packetBuffer[i - 1], HEX);
      received_command = received_command + char(packetBuffer[i - 1]);
      if (i % 32 == 0) {
        Serial.println();
      }
      else Serial.print(' ');
    } // end for
    Serial.println();

    if (! bManageMsg(received_command)) {
      vSendUdp("I don't understand this command: ", udpPortTx);
      vSendUdp(received_command, udpPortTx);
    }
    Serial.println(received_command);
    Serial.println();
    PCF_38.write8(~byState);
  }
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
