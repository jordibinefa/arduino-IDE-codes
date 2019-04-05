// Testing Sonoff Basic
//
// by Jordi Binefa - twitter.com/jordibinefa
// 20190405 - wiki.binefa.cat
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "esp8266_hw.h"
#include "wifiCredentials.h"
#include "networkSettings.h"

String szMsg;
bool bRelay, bLed, bXtra;

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

// WiFi{
void delayESP8266(unsigned long ulMilliseconds) {
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
    delayESP8266(500);
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
    delayESP8266(1000);
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
    delayESP8266(100);
  } while (!bTryWifiConnection());
  IPAddress ip = WiFi.localIP();
  Serial.print("Connected to wifi at "); Serial.println(ip);
}

// }WiFi


void setup() {
  Serial.begin(115200);

  pinMode(STATUS_LED, OUTPUT);
  pinMode(BUTTON, INPUT );
  pinMode(RELAY, OUTPUT);
  pinMode(XTRA_PIN, OUTPUT);
  bRelay = bLed = bXtra = LOW;
  Serial.println("Testing Sonoff pins");
  setup_wifi();
  Serial.print("Udp server started at port ");
  Serial.println(udpPortRx);
  Udp.begin(udpPortRx);
}

bool bManageMsg(String szMsg) {
  bool bUnderstood = false;
  szMsg = String(szMsg.charAt(0)); // Only first character

  if (szMsg == "i" || szMsg == "I") {
    if (digitalRead(BUTTON)) {
      Serial.println("Button is not pressed: GPIO0 HIGH");
      vSendUdp("H", udpPortTx);
    } else {
      Serial.println("Button is pressed: GPIO0 LOW");
      vSendUdp("L", udpPortTx);
    }
    bUnderstood = true;
  }
  if (szMsg == "l" || szMsg == "L") {
    bLed = !bLed;
    digitalWrite(STATUS_LED, bLed);
    Serial.print("LED GPIO13");
    (bLed) ? Serial.println(" HIGH") : Serial.println(" LOW");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "r" || szMsg == "R") {
    bRelay = !bRelay;
    digitalWrite(RELAY, bRelay);
    Serial.print("Relay GPIO12");
    (bRelay) ? Serial.println(" HIGH") : Serial.println(" LOW");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  if (szMsg == "x" || szMsg == "X") {
    bXtra = !bXtra;
    digitalWrite(XTRA_PIN, bXtra);
    Serial.print("Xtra GPIO14");
    (bXtra) ? Serial.println(" HIGH") : Serial.println(" LOW");
    bUnderstood = true;
    vSendUdp("OK", udpPortTx);
  }
  return bUnderstood;
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
  digitalWrite(STATUS_LED, bLed);
  digitalWrite(RELAY, bRelay);
  digitalWrite(XTRA_PIN, bXtra);
}
