#include <ESP8266WiFi.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include "esp8266_hw.h"
#include "wifiCredentials.h"
#include "networkSettings.h"

extern "C" {
#include "user_interface.h"
}

int status = WL_IDLE_STATUS;

byte packetBuffer[512]; //buffer to hold incoming and outgoing packets

WiFiUDP Udp;

void vSendUdp(String sz, unsigned int uiLocalPort) {
  Udp.beginPacket(Udp.remoteIP(), uiLocalPort);
  Udp.print(sz);
  Udp.endPacket();
}

bool bManageMsg(String szMsg) {
  unsigned int nLdr = analogRead(A0);
  bool bUnderstood = false;

  if (szMsg == "a" || szMsg == "A") {
    Serial.print("LDR level (0..1024): ");
    Serial.println(nLdr);
    vSendUdp(String(nLdr), udpPortTx);
    bUnderstood = true;
  }
  if (szMsg == "i" || szMsg == "I") {
    if (digitalRead(GPIO4)) {
      Serial.println("Button is not pressed: GPIO4 HIGH");
      vSendUdp("H", udpPortTx);
    } else {
      Serial.println("Button is pressed: GPIO4 LOW");
      vSendUdp("L", udpPortTx);
    }
    bUnderstood = true;
  }
  if (szMsg == "5h" || szMsg == "5H") {
    digitalWrite(GPIO5, HIGH);
    Serial.println("GPIO5 HIGH");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "5l" || szMsg == "5L") {
    digitalWrite(GPIO5, LOW);
    Serial.println("GPIO5 LOW");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "4h" || szMsg == "4H") {
    digitalWrite(GPIO4, HIGH);
    Serial.println("GPIO4 HIGH");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "4l" || szMsg == "4L") {
    digitalWrite(GPIO4, LOW);
    Serial.println("GPIO4 LOW");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "0h" || szMsg == "0H") {
    digitalWrite(GPIO0, HIGH);
    Serial.println("GPIO0 HIGH");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "0l" || szMsg == "0L") {
    digitalWrite(GPIO0, LOW);
    Serial.println("GPIO0 LOW");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "2h" || szMsg == "2H") {
    digitalWrite(GPIO2, HIGH);
    Serial.println("GPIO2 HIGH");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "2l" || szMsg == "2L") {
    digitalWrite(GPIO2, LOW);
    Serial.println("GPIO2 LOW");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "13h" || szMsg == "12H") {
    digitalWrite(GPIO13, HIGH);
    Serial.println("GPIO13 HIGH -> Blue ON");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "13l" || szMsg == "12L") {
    digitalWrite(GPIO13, LOW);
    Serial.println("GPIO13 LOW -> Blue OFF");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "12h" || szMsg == "13H") {
    digitalWrite(GPIO12, HIGH);
    Serial.println("GPIO12 HIGH -> Green ON");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "12l" || szMsg == "13L") {
    digitalWrite(GPIO12, LOW);
    Serial.println("GPIO12 LOW -> Green OFF");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "15h" || szMsg == "15H") {
    digitalWrite(GPIO15, HIGH);
    Serial.println("GPIO15 HIGH -> Red ON");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "15l" || szMsg == "15L") {
    digitalWrite(GPIO15, LOW);
    Serial.println("GPIO15 LOW -> Red OFF");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "14h" || szMsg == "14H") {
    digitalWrite(GPIO14, HIGH);
    Serial.println("GPIO14 HIGH");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "14l" || szMsg == "14L") {
    digitalWrite(GPIO14, LOW);
    Serial.println("GPIO14 LOW");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "16h" || szMsg == "16H") {
    digitalWrite(GPIO16, HIGH);
    Serial.println("GPIO16 HIGH");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "16l" || szMsg == "16L") {
    digitalWrite(GPIO16, LOW);
    Serial.println("GPIO16 LOW");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }

  return bUnderstood;
}


//  WiFi{

void vDelayESP(unsigned long ulMilliseconds) {
  unsigned long ulPreviousMillis = millis();

  do {
    yield();
  } while (millis() - ulPreviousMillis <= ulMilliseconds);
}

void vConnectToWiFi(const char* szSsid, const char* szPwd) {
  char ssid[MAX_STRING_SIZE],  pwd[MAX_STRING_SIZE];

  Serial.println("Connecting to WiFi network: " + String(szSsid) + ", pwd: " + String(szPwd));

  WiFi.begin(szSsid, szPwd);

  Serial.println("Waiting for WIFI connection...");
  while (WiFi.status() != WL_CONNECTED /*&& i++ < 20*/) {
    Serial.print(".");
    vDelayESP(500);
  }
  Serial.println();
}

boolean bIsListed(String szSSID, int *pNwO) {
  for (int i = 0; i < N_WIFIS ; i++) {
    if (String(stWiFi[i].szSSID) == szSSID) {
      *pNwO = i;
      return true;
    }
  }
  return false;
}

boolean bTryWifiConnection() {
  int n = WiFi.scanNetworks(), nWhichOne;

  //Serial.print("*");
  if (n == 0) {
    Serial.println("\nNo networks found");
    vDelayESP(1000);
  } else {
    for (int i = 0; i < n; ++i) {
      if (bIsListed(WiFi.SSID(i), &nWhichOne)) {
        vConnectToWiFi(stWiFi[nWhichOne].szSSID, stWiFi[nWhichOne].szPWD);
        return true;
      }
    }
  }
  return false;
}

void setup_wifi() {
  do {
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();
    vDelayESP(100);
  } while (!bTryWifiConnection());
  IPAddress ip = WiFi.localIP();
  Serial.print("Connected to wifi at "); Serial.println(ip);
}

// }WiFi

void setup_mdns(){
  if (!MDNS.begin(NODE_UNIQUE_NAME)) {
    Serial.println("Error setting up MDNS responder!");
    while (1) {
      delay(1000);
    }
  }
  Serial.println("mDNS responder started");  

  MDNS.addService("_prova","_udp",udpPortRx);
  MDNS.addServiceTxt("_prova","_udp","Ho veig","v0.1"); // Descripció del servei
}

void setup() {
  pinMode(GPIO2, OUTPUT); // Internal blue led
  pinMode(GPIO15, OUTPUT); // Red of RGB led
  pinMode(GPIO13, OUTPUT); // Blue of RGB led
  pinMode(GPIO12, OUTPUT); // Green of RGB led

  Serial.begin(115200);

  setup_wifi();

  Serial.print("Udp server started at port ");
  Serial.println(udpPortRx);
  Udp.begin(udpPortRx);

  setup_mdns();
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
  }
}


